// Hyper-V synthetic keyboard support.
//
// The synthetic keyboard sends set 1 make/break codes over VMBus. The
// channel is polled from the timer tick and the codes are fed to the
// regular keyboard code, as for USB keyboards.
//
// Without an i8042 (Hyper-V Generation 2 VMs) nothing raises IRQ 1, so
// programs that hook int 09h, like DOS keyboard drivers or the Windows
// keyboard driver, never see a key. There the codes are queued instead
// and, after each synthetic timer tick, raised one by one as int 09h with
// the scancode in the BDA at 40:16h, where a driver that can't read port
// 60h finds it. The BIOS int 09h handler reads it from there as well.
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "biosvar.h" // GET_GLOBAL
#include "bregs.h" // struct bregs
#include "config.h" // CONFIG_HYPERV_KEYBOARD
#include "fw/paravirt.h" // HyperVTick
#include "hw/ps2port.h" // PORT_PS2_STATUS
#include "hvkbd.h" // hvkbd_setup
#include "malloc.h" // malloc_high
#include "output.h" // dprintf
#include "stacks.h" // call32_params
#include "string.h" // memset
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

// Set when keys are raised as int 09h (no i8042); codes waiting for it
u8 HvKbdIrq VARFSEG;
u8 HvKbdQueue[32] VARLOW;
u8 HvKbdHead VARLOW, HvKbdTail VARLOW;

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

    // Without an i8042 (its status port floats to all ones), raise int 09h
    // after each synthetic tick.
    if (HyperVTick && inb(PORT_PS2_STATUS) == 0xff) {
        HvKbdIrq = 1;
        SET_IVT(0x09, FUNC16(entry_hvkbd_09));
        dprintf(1, "hvkbd: keys raise int 09h, scancode at 40:16h\n");
    }
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
    if (!GET_GLOBAL(HvKbdIrq)) {
        for (i = 0; i < n; i++)
            process_key(codes[i]);
        return;
    }
    u8 tail = GET_LOW(HvKbdTail);
    for (i = 0; i < n; i++) {
        u8 next = (tail + 1) % ARRAY_SIZE(HvKbdQueue);
        if (next == GET_LOW(HvKbdHead))
            break;
        SET_LOW(HvKbdQueue[tail], codes[i]);
        tail = next;
    }
    SET_LOW(HvKbdTail, tail);
}

// Raise int 09h for each queued code, as the i8042 would raise IRQ 1;
// called after each synthetic timer tick, once its interrupt is over.
void VISIBLE16
handle_hvkbd_irq(void)
{
    if (!CONFIG_HYPERV_KEYBOARD || !GET_GLOBAL(HvKbdIrq))
        return;
    u8 head = GET_LOW(HvKbdHead);
    while (head != GET_LOW(HvKbdTail)) {
        SET_BDA(kbd_scancode, GET_LOW(HvKbdQueue[head]));
        head = (head + 1) % ARRAY_SIZE(HvKbdQueue);
        SET_LOW(HvKbdHead, head);
        struct bregs br;
        memset(&br, 0, sizeof(br));
        call16_int(0x09, &br);
    }
}

// INT09h handler without an i8042: the scancode is in the BDA
void VISIBLE16
handle_hvkbd_09(void)
{
    debug_isr(DEBUG_ISR_09);
    process_key(GET_BDA(kbd_scancode));
}
