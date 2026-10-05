// Hyper-V synthetic SCSI controller (storvsc) boot support.
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "block.h" // struct drive_s
#include "blockcmd.h" // scsi_drive_setup
#include "config.h" // CONFIG_HYPERV_STORVSC
#include "malloc.h" // malloc_fseg
#include "memmap.h" // PAGE_SIZE
#include "output.h" // dprintf
#include "stacks.h" // yield
#include "std/disk.h" // DISK_RET_SUCCESS
#include "storvsc.h" // storvsc_setup
#include "string.h" // memset
#include "util.h" // timer_calc
#include "vmbus.h" // vmbus_send

#define VSTOR_OP_COMPLETE_IO            1
#define VSTOR_OP_EXECUTE_SRB            3
#define VSTOR_OP_BEGIN_INITIALIZATION   7
#define VSTOR_OP_END_INITIALIZATION     8
#define VSTOR_OP_QUERY_PROTOCOL_VERSION 9
#define VSTOR_OP_QUERY_PROPERTIES       10

#define VSTOR_REQUEST_COMPLETION_FLAG   1

#define VSTOR_PROTO_WIN8                0x0501

#define SRB_STATUS_SUCCESS              0x01
#define SRB_STATUS_DATA_OVERRUN         0x12
#define SRB_FLAGS_DISABLE_SYNCH_TRANSFER 0x08
#define SRB_FLAGS_DATA_IN               0x40
#define SRB_FLAGS_DATA_OUT              0x80

#define STORVSC_WRITE_TYPE              0
#define STORVSC_READ_TYPE               1
#define STORVSC_UNKNOWN_TYPE            2

#define STORVSC_RING_PAGES              4
#define STORVSC_MAX_PAGES               32
#define STORVSC_MAX_CONTROLLERS         4

// Interface type of the synthetic SCSI controller,
// {ba6163d9-04a1-4d29-b605-72e2ffb1dc7f}
static const u8 storvsc_scsi_guid[16] = {
    0xd9, 0x63, 0x61, 0xba, 0xa1, 0x04, 0x29, 0x4d,
    0xb6, 0x05, 0x72, 0xe2, 0xff, 0xb1, 0xdc, 0x7f,
};

struct vmscsi_request {
    u16 length;
    u8 srb_status;
    u8 scsi_status;
    u8 port_number;
    u8 path_id;
    u8 target_id;
    u8 lun;
    u8 cdb_length;
    u8 sense_info_length;
    u8 data_in;
    u8 reserved;
    u32 data_transfer_length;
    u8 cdb[20];                 // shared with the sense data
    // Windows 8 extension
    u16 reserve;
    u8 queue_tag;
    u8 queue_action;
    u32 srb_flags;
    u32 time_out_value;
    u32 queue_sort_key;
} PACKED;

#define VMSCSI_WIN8_EXTENSION_SIZE      16

struct vstor_packet {
    u32 operation;
    u32 flags;
    u32 status;
    union {
        struct vmscsi_request srb;
        struct {
            u16 major_minor;
            u16 revision;
        } version;
        u8 buffer[0x34];
    };
} PACKED;

struct storvsc_ctl {
    struct vmbus_channel chan;
    u32 pkt_size;
    u16 srb_size;
    u8 sense_size;
    u64 trans_id;
};

struct storvsc_lun_s {
    struct drive_s drive;
    struct storvsc_ctl *ctl;
    u8 target;
    u8 lun;
};

struct storvsc_range {
    struct vmbus_gpa_range range;
    u64 pfn[STORVSC_MAX_PAGES];
} PACKED;

// Send a request and poll for its completion, which replaces *pkt.
static int
storvsc_request(struct storvsc_ctl *ctl, struct vstor_packet *pkt
                , struct vmbus_gpa_range *range, u32 range_len)
{
    u64 id = ++ctl->trans_id;
    pkt->flags = VSTOR_REQUEST_COMPLETION_FLAG;
    if (vmbus_send(&ctl->chan, range ? VM_PKT_DATA_USING_GPA_DIRECT
                   : VM_PKT_DATA_INBAND, range, range_len
                   , pkt, ctl->pkt_size, id))
        return -1;
    u32 end = timer_calc(30000);
    for (;;) {
        struct vstor_packet resp;
        u16 type;
        u64 rid;
        int len = vmbus_recv(&ctl->chan, &resp, sizeof(resp), &type, &rid);
        if (len >= 0) {
            // Skip unsolicited packets such as bus rescan requests.
            if (type != VM_PKT_COMP || rid != id)
                continue;
            if (len > sizeof(resp))
                len = sizeof(resp);
            memcpy(pkt, &resp, len);
            return 0;
        }
        if (timer_check(end)) {
            warn_timeout();
            return -1;
        }
        yield();
    }
}

static int
storvsc_simple_request(struct storvsc_ctl *ctl, u32 operation
                       , struct vstor_packet *pkt)
{
    memset(pkt, 0, sizeof(*pkt));
    pkt->operation = operation;
    if (storvsc_request(ctl, pkt, NULL, 0))
        return -1;
    if (pkt->operation != VSTOR_OP_COMPLETE_IO || pkt->status) {
        dprintf(3, "storvsc: operation %d failed: op %d status %x\n"
                , operation, pkt->operation, pkt->status);
        return -1;
    }
    return 0;
}

static u8
scsi_cdb_length(u8 opcode)
{
    switch (opcode >> 5) {
    case 0:
        return 6;
    case 1:
    case 2:
        return 10;
    case 5:
        return 12;
    default:
        return 16;
    }
}

int
storvsc_process_op(struct disk_op_s *op)
{
    if (!CONFIG_HYPERV_STORVSC)
        return DISK_RET_EBADTRACK;
    struct storvsc_lun_s *slun =
        container_of(op->drive_fl, struct storvsc_lun_s, drive);
    struct storvsc_ctl *ctl = slun->ctl;
    struct vstor_packet pkt;
    memset(&pkt, 0, sizeof(pkt));
    int blocksize = scsi_fill_cmd(op, pkt.srb.cdb, 16);
    if (blocksize < 0)
        return default_process_op(op);

    u32 len = op->count * blocksize;
    int read = scsi_is_read(op);
    pkt.operation = VSTOR_OP_EXECUTE_SRB;
    pkt.srb.length = ctl->srb_size;
    pkt.srb.target_id = slun->target;
    pkt.srb.lun = slun->lun;
    pkt.srb.cdb_length = scsi_cdb_length(pkt.srb.cdb[0]);
    pkt.srb.sense_info_length = ctl->sense_size;
    pkt.srb.data_in = !len ? STORVSC_UNKNOWN_TYPE
                      : read ? STORVSC_READ_TYPE : STORVSC_WRITE_TYPE;
    pkt.srb.data_transfer_length = len;
    pkt.srb.srb_flags = SRB_FLAGS_DISABLE_SYNCH_TRANSFER
        | (!len ? 0 : read ? SRB_FLAGS_DATA_IN : SRB_FLAGS_DATA_OUT);

    struct storvsc_range range;
    u32 range_len = 0;
    if (len) {
        u32 buf = (u32)op->buf_fl;
        u32 first = buf >> PAGE_SHIFT, pages = ((buf + len - 1) >> PAGE_SHIFT)
                                                - first + 1;
        if (pages > STORVSC_MAX_PAGES)
            return DISK_RET_EBOUNDARY;
        range.range.byte_count = len;
        range.range.byte_offset = buf & (PAGE_SIZE - 1);
        u32 i;
        for (i = 0; i < pages; i++)
            range.pfn[i] = first + i;
        range_len = sizeof(range.range) + pages * sizeof(range.pfn[0]);
    }
    if (storvsc_request(ctl, &pkt, len ? &range.range : NULL, range_len))
        return DISK_RET_EBADTRACK;

    u8 srb_status = pkt.srb.srb_status & 0x3f;
    if ((srb_status == SRB_STATUS_SUCCESS
         || srb_status == SRB_STATUS_DATA_OVERRUN) && !pkt.srb.scsi_status)
        return DISK_RET_SUCCESS;
    dprintf(3, "storvsc: cdb %02x failed: srb %02x scsi %02x\n"
            , pkt.srb.cdb[0], pkt.srb.srb_status, pkt.srb.scsi_status);
    return DISK_RET_EBADTRACK;
}

static int
storvsc_add_lun(u32 lun, struct drive_s *tmpl_drv)
{
    struct storvsc_lun_s *tmpl =
        container_of(tmpl_drv, struct storvsc_lun_s, drive);
    struct storvsc_lun_s *slun = malloc_fseg(sizeof(*slun));
    if (!slun) {
        warn_noalloc();
        return -1;
    }
    memset(slun, 0, sizeof(*slun));
    slun->drive.type = DTYPE_STORVSC;
    slun->drive.cntl_id = tmpl->ctl->chan.child_relid;
    slun->ctl = tmpl->ctl;
    slun->target = tmpl->target;
    slun->lun = lun;

    char *name = znprintf(MAXDESCSIZE, "Hyper-V SCSI %d:%d:%d"
                          , slun->drive.cntl_id, slun->target, lun);
    int ret = scsi_drive_setup(&slun->drive, name, -1);
    free(name);
    if (ret) {
        free(slun);
        return -1;
    }
    return 0;
}

static int
storvsc_negotiate(struct storvsc_ctl *ctl)
{
    static const u16 versions[] = { 0x0602, 0x0600, VSTOR_PROTO_WIN8
                                    , 0x0402, 0x0200 };
    struct vstor_packet pkt;

    // Until a Windows 8 or later protocol is agreed on, requests omit the
    // Windows 8 SRB extension.
    ctl->pkt_size = sizeof(pkt) - VMSCSI_WIN8_EXTENSION_SIZE;
    ctl->srb_size = sizeof(pkt.srb) - VMSCSI_WIN8_EXTENSION_SIZE;
    ctl->sense_size = 0x12;
    if (storvsc_simple_request(ctl, VSTOR_OP_BEGIN_INITIALIZATION, &pkt))
        return -1;

    int i;
    for (i = 0; i < ARRAY_SIZE(versions); i++) {
        memset(&pkt, 0, sizeof(pkt));
        pkt.operation = VSTOR_OP_QUERY_PROTOCOL_VERSION;
        pkt.version.major_minor = versions[i];
        if (storvsc_request(ctl, &pkt, NULL, 0))
            return -1;
        if (pkt.operation == VSTOR_OP_COMPLETE_IO && !pkt.status)
            break;
    }
    if (i == ARRAY_SIZE(versions)) {
        dprintf(1, "storvsc: no supported protocol version\n");
        return -1;
    }
    dprintf(3, "storvsc: protocol %04x\n", versions[i]);
    if (versions[i] >= VSTOR_PROTO_WIN8) {
        ctl->pkt_size = sizeof(pkt);
        ctl->srb_size = sizeof(pkt.srb);
        ctl->sense_size = 0x14;
    }

    if (storvsc_simple_request(ctl, VSTOR_OP_QUERY_PROPERTIES, &pkt)
        || storvsc_simple_request(ctl, VSTOR_OP_END_INITIALIZATION, &pkt))
        return -1;
    return 0;
}

static void
storvsc_init_ctl(u32 relid, u32 connid)
{
    struct storvsc_ctl *ctl = malloc_high(sizeof(*ctl));
    if (!ctl) {
        warn_noalloc();
        return;
    }
    memset(ctl, 0, sizeof(*ctl));
    if (vmbus_open(&ctl->chan, relid, connid, STORVSC_RING_PAGES)
        || storvsc_negotiate(ctl)) {
        dprintf(1, "storvsc: controller relid %d init failed\n", relid);
        return;
    }
    dprintf(1, "found Hyper-V SCSI controller (relid %d)\n", relid);

    struct storvsc_lun_s tmpl;
    memset(&tmpl, 0, sizeof(tmpl));
    tmpl.drive.type = DTYPE_STORVSC;
    tmpl.drive.cntl_id = relid;
    tmpl.ctl = ctl;
    for (tmpl.target = 0; tmpl.target < 2; tmpl.target++)
        if (scsi_rep_luns_scan(&tmpl.drive, storvsc_add_lun) < 0)
            scsi_sequential_scan(&tmpl.drive, 8, storvsc_add_lun);
}

void
storvsc_setup(void)
{
    ASSERT32FLAT();
    if (!CONFIG_HYPERV_STORVSC || vmbus_setup())
        return;
    dprintf(3, "init storvsc\n");
    u32 relids[STORVSC_MAX_CONTROLLERS], connids[STORVSC_MAX_CONTROLLERS];
    int count = vmbus_find_offers(storvsc_scsi_guid, relids, connids
                                  , STORVSC_MAX_CONTROLLERS);
    int i;
    for (i = 0; i < count; i++)
        storvsc_init_ctl(relids[i], connids[i]);
}
