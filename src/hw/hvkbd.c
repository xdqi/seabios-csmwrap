// Hyper-V synthetic keyboard support.
//
// The synthetic keyboard sends set 1 make/break codes over VMBus. The
// channel is polled from the timer tick and the codes are fed to the
// regular keyboard code, as for USB keyboards.
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "biosvar.h" // GET_GLOBAL
#include "config.h" // CONFIG_HYPERV_KEYBOARD
#include "hvkbd.h" // hvkbd_setup
#include "malloc.h" // malloc_high
#include "output.h" // dprintf
#include "stacks.h" // call32_params
#include "util.h" // process_key
#include "vmbus.h" // vmbus_open

#define HK_VERSION_WIN8                 0x00010000

#define HK_MSG_PROTOCOL_REQUEST         1
#define HK_MSG_PROTOCOL_RESPONSE        2
#define HK_MSG_EVENT                    3

#define HK_RESPONSE_ACCEPTED            (1 << 0)
#define HK_KEY_UNICODE                  (1 << 0)
#define HK_KEY_BREAK                    (1 << 1)
#define HK_KEY_E0                       (1 << 2)
#define HK_KEY_E1                       (1 << 3)

#define HVKBD_RING_PAGES                2
#define HVKBD_MAX_CODES                 24

// Interface type of the synthetic keyboard,
// {f912ad6d-2b17-48ea-bd65-f927a61c7684}
static const u8 hvkbd_guid[16] = {
    0x6d, 0xad, 0x12, 0xf9, 0x17, 0x2b, 0xea, 0x48,
    0xbd, 0x65, 0xf9, 0x27, 0xa6, 0x1c, 0x76, 0x84,
};

struct hk_message {
    u32 type;
    union {
        u32 version;            // protocol request
        u32 status;             // protocol response
        struct {
            u16 make_code;
            u16 reserved;
            u32 info;
        } key;                  // event
    };
} PACKED;

// Set once the keyboard channel is open
struct vmbus_channel *HvKbdChannel VARFSEG;

void
hvkbd_setup(void)
{
    ASSERT32FLAT();
    if (!CONFIG_HYPERV_KEYBOARD || vmbus_setup())
        return;
    u32 relid, connid;
    if (vmbus_find_offers(hvkbd_guid, &relid, &connid, 1) != 1)
        return;
    struct vmbus_channel *chan = malloc_high(sizeof(*chan));
    if (!chan) {
        warn_noalloc();
        return;
    }
    if (vmbus_open(chan, relid, connid, HVKBD_RING_PAGES))
        goto fail;

    struct hk_message msg = { .type = HK_MSG_PROTOCOL_REQUEST,
                              .version = HK_VERSION_WIN8 };
    if (vmbus_send(chan, VM_PKT_DATA_INBAND, NULL, 0, &msg, 8, 0))
        goto fail;
    u32 end = timer_calc(5000);
    for (;;) {
        u16 type;
        u64 id;
        int len = vmbus_recv(chan, &msg, sizeof(msg), &type, &id);
        if (len >= 8 && type == VM_PKT_DATA_INBAND
            && msg.type == HK_MSG_PROTOCOL_RESPONSE) {
            if (!(msg.status & HK_RESPONSE_ACCEPTED))
                goto fail;
            break;
        }
        if (len < 0) {
            if (timer_check(end))
                goto fail;
            yield();
        }
    }
    HvKbdChannel = chan;
    dprintf(1, "found Hyper-V synthetic keyboard (relid %d)\n", relid);
    return;
fail:
    dprintf(1, "hvkbd: init failed\n");
    free(chan);
}

// Move pending keystrokes into codes[] as set 1 scancode bytes.
u32 VISIBLE32FLAT
hvkbd_poll(u8 *codes)
{
    struct vmbus_channel *chan = HvKbdChannel;
    u32 n = 0;
    while (n + 3 <= HVKBD_MAX_CODES) {
        struct hk_message msg;
        u16 type;
        u64 id;
        int len = vmbus_recv(chan, &msg, sizeof(msg), &type, &id);
        if (len < 0)
            break;
        if (len < sizeof(msg) || type != VM_PKT_DATA_INBAND
            || msg.type != HK_MSG_EVENT || msg.key.info & HK_KEY_UNICODE)
            continue;
        if (msg.key.info & HK_KEY_E0)
            codes[n++] = 0xe0;
        if (msg.key.info & HK_KEY_E1)
            codes[n++] = 0xe1;
        codes[n++] = msg.key.make_code
                     | (msg.key.info & HK_KEY_BREAK ? 0x80 : 0);
    }
    return n;
}

// Feed pending keystrokes to the keyboard buffer; called from the timer tick.
void
hvkbd_check_event(void)
{
    if (!CONFIG_HYPERV_KEYBOARD || !GET_GLOBAL(HvKbdChannel))
        return;
    u8 codes[HVKBD_MAX_CODES];
    u32 i, n = call32_params(hvkbd_poll, MAKE_FLATPTR(GET_SEG(SS), codes)
                             , 0, 0, 0);
    for (i = 0; i < n; i++)
        process_key(codes[i]);
}
