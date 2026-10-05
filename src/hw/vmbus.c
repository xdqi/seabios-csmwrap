// Minimal Hyper-V VMBus client (polling only).
//
// Sets up the hypercall page and the SynIC message page on the BSP,
// negotiates a VMBus version, collects channel offers and opens channels
// with a single GPADL holding both ring buffers. At runtime only the ring
// buffers and the HvSignalEvent fast hypercall are used, so I/O works from
// any CPU and needs no interrupts.
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "biosvar.h" // SET_IVT
#include "config.h" // CONFIG_HYPERV_STORVSC
#include "fw/paravirt.h" // runningOnHyperV
#include "malloc.h" // memalign_high
#include "memmap.h" // PAGE_SIZE
#include "output.h" // dprintf
#include "stacks.h" // yield
#include "string.h" // memcpy
#include "util.h" // timer_calc
#include "vmbus.h" // vmbus_setup
#include "x86.h" // rdmsr

#define READ_ONCE(x)        (*(volatile typeof(x) *)&(x))
#define WRITE_ONCE(x, v)    (*(volatile typeof(x) *)&(x) = (v))

static inline void
vmbus_mb(void)
{
    asm volatile("lock; addl $0, (%%esp)" ::: "memory", "cc");
}

#define HV_CPUID_FEATURES               0x40000003
#define HV_MSR_SYNIC_AVAILABLE          (1 << 2)
#define HV_MSR_HYPERCALL_AVAILABLE      (1 << 5)
#define HV_MSR_VP_INDEX_AVAILABLE       (1 << 6)

#define HV_X64_MSR_GUEST_OS_ID          0x40000000
#define HV_X64_MSR_HYPERCALL            0x40000001
#define HV_X64_MSR_VP_INDEX             0x40000002
#define HV_X64_MSR_SCONTROL             0x40000080
#define HV_X64_MSR_SIMP                 0x40000083
#define HV_X64_MSR_EOM                  0x40000084
#define HV_X64_MSR_SINT0                0x40000090

#define HV_SINT_MASKED                  (1 << 16)
#define HV_SINT_AUTO_EOI                (1 << 17)

// Open source OS, vendor/OS id chosen for SeaBIOS
#define HV_GUEST_OS_ID_SEABIOS          ((1ULL << 63) | (0x5eULL << 48))

#define HVCALL_POST_MESSAGE             0x005c
#define HVCALL_SIGNAL_EVENT             0x005d
#define HV_HYPERCALL_FAST_BIT           (1 << 16)
#define HV_STATUS_INSUFFICIENT_BUFFERS  0x0013

#define VMBUS_MESSAGE_SINT              2
#define VMBUS_MESSAGE_CONNECTION_ID     1
#define VMBUS_SINT_VECTOR               0xf9

// VMBus protocol versions (major << 16 | minor)
#define VERSION_WIN8                    0x00020004
#define VERSION_WIN8_1                  0x00030000
#define VERSION_WIN10                   0x00040000

// Channel message types
#define CHANNELMSG_OFFERCHANNEL         1
#define CHANNELMSG_REQUESTOFFERS        3
#define CHANNELMSG_ALLOFFERS_DELIVERED  4
#define CHANNELMSG_OPENCHANNEL          5
#define CHANNELMSG_OPENCHANNEL_RESULT   6
#define CHANNELMSG_GPADL_HEADER         8
#define CHANNELMSG_GPADL_CREATED        10
#define CHANNELMSG_INITIATE_CONTACT     14
#define CHANNELMSG_VERSION_RESPONSE     15

#define HV_MESSAGE_PAYLOAD_SIZE         240

struct hv_message {
    u32 type;
    u8 payload_size;
    u8 flags;
    u16 reserved;
    u64 origin;
    u8 payload[HV_MESSAGE_PAYLOAD_SIZE];
} PACKED;

#define HV_MESSAGE_FLAG_PENDING         (1 << 0)

struct hv_post_message_input {
    u32 connection_id;
    u32 reserved;
    u32 message_type;
    u32 payload_size;
    u8 payload[HV_MESSAGE_PAYLOAD_SIZE];
} PACKED;

struct vmbus_msg_header {
    u32 msgtype;
    u32 padding;
} PACKED;

struct vmbus_initiate_contact {
    struct vmbus_msg_header header;
    u32 version_requested;
    u32 target_vcpu;
    u64 interrupt_page;
    u64 monitor_page1;
    u64 monitor_page2;
} PACKED;

struct vmbus_version_response {
    struct vmbus_msg_header header;
    u8 version_supported;
    u8 connection_state;
    u16 padding;
    u32 msg_conn_id;
} PACKED;

struct vmbus_offer_channel {
    struct vmbus_msg_header header;
    u8 if_type[16];
    u8 if_instance[16];
    u64 reserved1;
    u64 reserved2;
    u16 chn_flags;
    u16 mmio_megabytes;
    u8 user_def[120];
    u16 sub_channel_index;
    u16 reserved3;
    u32 child_relid;
    u8 monitorid;
    u8 monitor_allocated;
    u16 is_dedicated_interrupt;
    u32 connection_id;
} PACKED;

struct vmbus_gpadl_header {
    struct vmbus_msg_header header;
    u32 child_relid;
    u32 gpadl;
    u16 range_buflen;
    u16 rangecount;
    struct vmbus_gpa_range range;
} PACKED;

#define VMBUS_GPADL_HEADER_MAX_PFNS \
    ((HV_MESSAGE_PAYLOAD_SIZE - sizeof(struct vmbus_gpadl_header)) / 8)

struct vmbus_gpadl_created {
    struct vmbus_msg_header header;
    u32 child_relid;
    u32 gpadl;
    u32 creation_status;
} PACKED;

struct vmbus_open_channel {
    struct vmbus_msg_header header;
    u32 child_relid;
    u32 openid;
    u32 ringbuffer_gpadl;
    u32 target_vp;
    u32 downstream_ring_pageoffset;
    u8 user_data[120];
} PACKED;

struct vmbus_open_result {
    struct vmbus_msg_header header;
    u32 child_relid;
    u32 openid;
    u32 status;
} PACKED;

struct vmbus_ring {
    u32 write_index;
    u32 read_index;
    u32 interrupt_mask;
    u32 pending_send_sz;
    u32 reserved1[12];
    u32 feature_bits;
    u8 reserved2[PAGE_SIZE - 68];
    u8 buffer[];
} PACKED;

struct vmpacket_descriptor {
    u16 type;
    u16 offset8;
    u16 len8;
    u16 flags;
    u64 trans_id;
} PACKED;

#define VMBUS_DATA_PACKET_FLAG_COMPLETION_REQUESTED 1

// Set once by vmbus_setup(); only read afterwards, so may live in the f-seg.
u32 VmbusHypercallPage VARFSEG;
u8 *VmbusSendIntPage VARFSEG;

// Init-only state
static struct hv_message *VmbusMsgSlot;
static struct hv_post_message_input *VmbusPostInput;
static u32 VmbusVpIndex, VmbusNextGpadl = 0xe1e10;


/****************************************************************
 * Hypercalls and SynIC messages
 ****************************************************************/

// 32-bit hypercall convention: control in edx:eax, input GPA in ebx:ecx,
// output GPA in edi:esi, result in edx:eax.
static u16
hv_post_message(void *msg, u32 len)
{
    struct hv_post_message_input *in = VmbusPostInput;
    in->connection_id = VMBUS_MESSAGE_CONNECTION_ID;
    in->reserved = 0;
    in->message_type = 1;
    in->payload_size = len;
    memcpy(in->payload, msg, len);
    u64 status;
    u32 in_gpa = virt_to_phys(in);
    asm volatile("call *%[page]"
                 : "=A"(status), "+c"(in_gpa)
                 : "0"((u64)HVCALL_POST_MESSAGE), "b"(0), "D"(0), "S"(0)
                   , [page] "m"(VmbusHypercallPage)
                 : "cc", "memory");
    return status;
}

static void
hv_signal_event(u32 connection_id)
{
    u64 status;
    asm volatile("call *%[page]"
                 : "=A"(status), "+c"(connection_id)
                 : "0"((u64)(HVCALL_SIGNAL_EVENT | HV_HYPERCALL_FAST_BIT))
                   , "b"(0), [page] "m"(VmbusHypercallPage)
                 : "cc", "memory", "edi", "esi");
}

static int
vmbus_post(void *msg, u32 len)
{
    int tries;
    for (tries = 0; tries < 100; tries++) {
        u16 status = hv_post_message(msg, len);
        if (status != HV_STATUS_INSUFFICIENT_BUFFERS) {
            if (status)
                dprintf(1, "vmbus: post message failed %x\n", status);
            return status ? -1 : 0;
        }
        yield();
    }
    return -1;
}

// Wait for the next VMBus message; returns its type or -1 on timeout.
static int
vmbus_wait_msg(void *buf, u32 timeout_ms)
{
    u32 end = timer_calc(timeout_ms);
    for (;;) {
        struct hv_message *m = VmbusMsgSlot;
        if (READ_ONCE(m->type)) {
            barrier();
            memcpy(buf, m->payload, HV_MESSAGE_PAYLOAD_SIZE);
            u32 msgtype = ((struct vmbus_msg_header *)buf)->msgtype;
            // Free the slot before looking at MessagePending, so a message
            // queued meanwhile gets delivered after the EOM.
            WRITE_ONCE(m->type, 0);
            vmbus_mb();
            if (READ_ONCE(m->flags) & HV_MESSAGE_FLAG_PENDING)
                wrmsr(HV_X64_MSR_EOM, 0);
            return msgtype;
        }
        if (timer_check(end))
            return -1;
        yield();
    }
}

// Wait for a specific message type, dropping anything else.
static int
vmbus_wait_for(u32 msgtype, void *buf)
{
    for (;;) {
        int t = vmbus_wait_msg(buf, 5000);
        if (t < 0) {
            dprintf(1, "vmbus: timeout waiting for message %d\n", msgtype);
            return -1;
        }
        if (t == msgtype)
            return 0;
        dprintf(3, "vmbus: dropping message %d\n", t);
    }
}


/****************************************************************
 * Connection setup
 ****************************************************************/

static int
vmbus_connect(void)
{
    static const u32 versions[] = { VERSION_WIN10, VERSION_WIN8_1, VERSION_WIN8 };
    u8 *int_page = memalign_high(PAGE_SIZE, PAGE_SIZE);
    u8 *monitor = memalign_high(PAGE_SIZE, 2 * PAGE_SIZE);
    if (!int_page || !monitor) {
        warn_noalloc();
        return -1;
    }
    memset(int_page, 0, PAGE_SIZE);
    memset(monitor, 0, 2 * PAGE_SIZE);
    VmbusSendIntPage = int_page + PAGE_SIZE / 2;

    int i;
    for (i = 0; i < ARRAY_SIZE(versions); i++) {
        struct vmbus_initiate_contact msg;
        memset(&msg, 0, sizeof(msg));
        msg.header.msgtype = CHANNELMSG_INITIATE_CONTACT;
        msg.version_requested = versions[i];
        msg.target_vcpu = VmbusVpIndex;
        msg.interrupt_page = virt_to_phys(int_page);
        msg.monitor_page1 = virt_to_phys(monitor);
        msg.monitor_page2 = virt_to_phys(monitor + PAGE_SIZE);
        if (vmbus_post(&msg, sizeof(msg)))
            return -1;
        u8 buf[HV_MESSAGE_PAYLOAD_SIZE];
        if (vmbus_wait_for(CHANNELMSG_VERSION_RESPONSE, buf))
            return -1;
        struct vmbus_version_response *resp = (void *)buf;
        dprintf(3, "vmbus: version %x supported=%d state=%d\n"
                , versions[i], resp->version_supported, resp->connection_state);
        if (resp->version_supported) {
            dprintf(1, "vmbus: connected, version %x\n", versions[i]);
            return 0;
        }
    }
    dprintf(1, "vmbus: no supported version\n");
    return -1;
}

int
vmbus_setup(void)
{
    ASSERT32FLAT();
    if (!CONFIG_HYPERV_STORVSC || !runningOnHyperV())
        return -1;
    if (VmbusHypercallPage)
        return 0;

    u32 eax, ebx, ecx, edx;
    cpuid(HV_CPUID_FEATURES, &eax, &ebx, &ecx, &edx);
    u32 need = HV_MSR_SYNIC_AVAILABLE | HV_MSR_HYPERCALL_AVAILABLE
               | HV_MSR_VP_INDEX_AVAILABLE;
    if ((eax & need) != need) {
        dprintf(1, "vmbus: missing hypervisor features %x\n", eax);
        return -1;
    }

    void *hypercall = memalign_high(PAGE_SIZE, PAGE_SIZE);
    VmbusMsgSlot = memalign_high(PAGE_SIZE, PAGE_SIZE);
    VmbusPostInput = memalign_high(PAGE_SIZE, PAGE_SIZE);
    if (!hypercall || !VmbusMsgSlot || !VmbusPostInput) {
        warn_noalloc();
        return -1;
    }
    memset(VmbusMsgSlot, 0, PAGE_SIZE);

    wrmsr(HV_X64_MSR_GUEST_OS_ID, HV_GUEST_OS_ID_SEABIOS);
    wrmsr(HV_X64_MSR_HYPERCALL, virt_to_phys(hypercall) | 1);
    VmbusHypercallPage = virt_to_phys(hypercall);
    VmbusVpIndex = rdmsr(HV_X64_MSR_VP_INDEX);

    // Poll the message page; the AutoEOI vector only has to exist.
    SET_IVT(VMBUS_SINT_VECTOR, FUNC16(entry_iret_official));
    wrmsr(HV_X64_MSR_SIMP, virt_to_phys(VmbusMsgSlot) | 1);
    wrmsr(HV_X64_MSR_SINT0 + VMBUS_MESSAGE_SINT
          , VMBUS_SINT_VECTOR | HV_SINT_AUTO_EOI);
    wrmsr(HV_X64_MSR_SCONTROL, rdmsr(HV_X64_MSR_SCONTROL) | 1);
    VmbusMsgSlot += VMBUS_MESSAGE_SINT;

    if (vmbus_connect()) {
        VmbusHypercallPage = 0;
        return -1;
    }
    return 0;
}

// Messages are only needed during setup. Mask the message SINT and drop
// the message page so that an OS VMBus driver can take over (the minimum
// Windows needs); the rings and the hypercall page stay in use.
void
vmbus_quiesce(void)
{
    if (!VmbusMsgSlot)
        return;
    wrmsr(HV_X64_MSR_SINT0 + VMBUS_MESSAGE_SINT, HV_SINT_MASKED);
    wrmsr(HV_X64_MSR_SIMP, 0);
    VmbusMsgSlot = NULL;
}

// Collect up to max offers of the given interface type.
int
vmbus_find_offers(const u8 *if_type, u32 *relids, u32 *connids, int max)
{
    struct vmbus_msg_header req = { .msgtype = CHANNELMSG_REQUESTOFFERS };
    if (vmbus_post(&req, sizeof(req)))
        return -1;
    int count = 0;
    for (;;) {
        u8 buf[HV_MESSAGE_PAYLOAD_SIZE];
        int t = vmbus_wait_msg(buf, 5000);
        if (t < 0) {
            dprintf(1, "vmbus: timeout waiting for offers\n");
            return count;
        }
        if (t == CHANNELMSG_ALLOFFERS_DELIVERED)
            return count;
        if (t != CHANNELMSG_OFFERCHANNEL)
            continue;
        struct vmbus_offer_channel *offer = (void *)buf;
        dprintf(3, "vmbus: offer relid %d conn %x type %08x\n"
                , offer->child_relid, offer->connection_id
                , *(u32 *)offer->if_type);
        if (count < max && memcmp(offer->if_type, if_type, 16) == 0) {
            relids[count] = offer->child_relid;
            connids[count] = offer->connection_id;
            count++;
        }
    }
}

// Open a channel whose two rings (ring_pages each, header page included)
// share one GPADL.
int
vmbus_open(struct vmbus_channel *chan, u32 relid, u32 connid, u32 ring_pages)
{
    u32 pages = 2 * ring_pages;
    if (pages > VMBUS_GPADL_HEADER_MAX_PFNS)
        return -1;
    u8 *ring = memalign_high(PAGE_SIZE, pages * PAGE_SIZE);
    if (!ring) {
        warn_noalloc();
        return -1;
    }
    memset(ring, 0, pages * PAGE_SIZE);

    u8 buf[HV_MESSAGE_PAYLOAD_SIZE];
    struct vmbus_gpadl_header *hdr = (void *)buf;
    memset(buf, 0, sizeof(buf));
    u32 gpadl = VmbusNextGpadl++;
    hdr->header.msgtype = CHANNELMSG_GPADL_HEADER;
    hdr->child_relid = relid;
    hdr->gpadl = gpadl;
    hdr->range_buflen = sizeof(struct vmbus_gpa_range) + pages * 8;
    hdr->rangecount = 1;
    hdr->range.byte_count = pages * PAGE_SIZE;
    hdr->range.byte_offset = 0;
    u32 i;
    for (i = 0; i < pages; i++)
        hdr->range.pfn_array[i] = virt_to_phys(ring + i * PAGE_SIZE) >> PAGE_SHIFT;
    if (vmbus_post(hdr, sizeof(*hdr) + pages * 8))
        return -1;
    if (vmbus_wait_for(CHANNELMSG_GPADL_CREATED, buf))
        return -1;
    struct vmbus_gpadl_created *created = (void *)buf;
    if (created->creation_status) {
        dprintf(1, "vmbus: gpadl failed %x\n", created->creation_status);
        return -1;
    }

    struct vmbus_open_channel *open = (void *)buf;
    memset(buf, 0, sizeof(buf));
    open->header.msgtype = CHANNELMSG_OPENCHANNEL;
    open->child_relid = relid;
    open->openid = relid;
    open->ringbuffer_gpadl = gpadl;
    open->target_vp = VmbusVpIndex;
    open->downstream_ring_pageoffset = ring_pages;
    if (vmbus_post(open, sizeof(*open)))
        return -1;
    if (vmbus_wait_for(CHANNELMSG_OPENCHANNEL_RESULT, buf))
        return -1;
    struct vmbus_open_result *result = (void *)buf;
    if (result->status) {
        dprintf(1, "vmbus: open relid %d failed %x\n", relid, result->status);
        return -1;
    }

    chan->child_relid = relid;
    chan->connection_id = connid;
    chan->out = (void *)ring;
    chan->in = (void *)(ring + ring_pages * PAGE_SIZE);
    chan->out_size = chan->in_size = (ring_pages - 1) * PAGE_SIZE;
    dprintf(3, "vmbus: opened relid %d\n", relid);
    return 0;
}


/****************************************************************
 * Ring buffers
 ****************************************************************/

static u32
ring_copy_to(struct vmbus_ring *r, u32 size, u32 pos, const void *data, u32 len)
{
    const u8 *p = data;
    while (len) {
        u32 n = size - pos;
        if (n > len)
            n = len;
        if (p)
            memcpy(r->buffer + pos, p, n);
        else
            memset(r->buffer + pos, 0, n);
        pos = pos + n == size ? 0 : pos + n;
        if (p)
            p += n;
        len -= n;
    }
    return pos;
}

static u32
ring_copy_from(struct vmbus_ring *r, u32 size, u32 pos, void *data, u32 len)
{
    u8 *p = data;
    while (len) {
        u32 n = size - pos;
        if (n > len)
            n = len;
        if (p)
            memcpy(p, r->buffer + pos, n);
        pos = pos + n == size ? 0 : pos + n;
        if (p)
            p += n;
        len -= n;
    }
    return pos;
}

// Queue one packet (optionally with a GPA direct range) and kick the host.
int
vmbus_send(struct vmbus_channel *chan, u16 type
           , struct vmbus_gpa_range *range, u32 range_len
           , void *data, u32 len, u64 trans_id)
{
    struct vmbus_ring *r = chan->out;
    u32 size = chan->out_size;
    u32 hdrlen = sizeof(struct vmpacket_descriptor) + (range ? 8 + range_len : 0);
    u32 pktlen = ALIGN(hdrlen + len, 8);
    u32 write = r->write_index, read = READ_ONCE(r->read_index);
    u32 avail = write >= read ? size - (write - read) : read - write;
    if (pktlen + 8 >= avail)
        return -1;

    struct vmpacket_descriptor desc = {
        .type = type, .offset8 = hdrlen / 8, .len8 = pktlen / 8,
        .flags = VMBUS_DATA_PACKET_FLAG_COMPLETION_REQUESTED,
        .trans_id = trans_id,
    };
    u32 pos = ring_copy_to(r, size, write, &desc, sizeof(desc));
    if (range) {
        u32 gpa_hdr[2] = { 0, 1 }; // reserved, range count
        pos = ring_copy_to(r, size, pos, gpa_hdr, sizeof(gpa_hdr));
        pos = ring_copy_to(r, size, pos, range, range_len);
    }
    pos = ring_copy_to(r, size, pos, data, len);
    pos = ring_copy_to(r, size, pos, NULL, pktlen - hdrlen - len);
    u64 prev = (u64)write << 32;
    pos = ring_copy_to(r, size, pos, &prev, sizeof(prev));
    smp_wmb();
    WRITE_ONCE(r->write_index, pos);
    vmbus_mb();

    // The host only expects a signal when the ring goes from empty to
    // non-empty, and not at all while it has masked interrupts.
    if (READ_ONCE(r->interrupt_mask) || READ_ONCE(r->read_index) != write)
        return 0;
    VmbusSendIntPage[chan->child_relid / 8] |= 1 << (chan->child_relid % 8);
    hv_signal_event(chan->connection_id);
    return 0;
}

// Dequeue one packet if available. Returns the payload length (copied up
// to maxlen) or -1 if the ring is empty.
int
vmbus_recv(struct vmbus_channel *chan, void *data, u32 maxlen
           , u16 *type, u64 *trans_id)
{
    struct vmbus_ring *r = chan->in;
    u32 size = chan->in_size;
    u32 read = r->read_index;
    if (read == READ_ONCE(r->write_index))
        return -1;
    smp_rmb();

    struct vmpacket_descriptor desc;
    ring_copy_from(r, size, read, &desc, sizeof(desc));
    u32 offset = desc.offset8 * 8, total = desc.len8 * 8;
    u32 len = total - offset;
    u32 pos = ring_copy_from(r, size, read, NULL, offset);
    pos = ring_copy_from(r, size, pos, data, len < maxlen ? len : maxlen);
    if (len > maxlen)
        pos = ring_copy_from(r, size, pos, NULL, len - maxlen);
    pos = ring_copy_from(r, size, pos, NULL, sizeof(u64));
    vmbus_mb();
    WRITE_ONCE(r->read_index, pos);

    *type = desc.type;
    *trans_id = desc.trans_id;
    return len;
}
