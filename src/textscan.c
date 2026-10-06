// Show the legacy text buffer on the framebuffer ("software CRTC").
//
// Without VGA hardware the legacy text buffer at 0xb8000 can be plain RAM
// (e.g. on Hyper-V Generation 2 VMs). SeaVGABIOS then keeps the standard
// text modes there (see vgasrc/cbvga.c), and the BIOS helper core of
// CSMWrap, which stays idle in the BIOS whenever it is not serving a
// call32 request, draws the buffer on the framebuffer periodically, much
// like a VGA CRTC scans out its text memory. Software that writes the
// buffer directly thus becomes visible, and nothing is drawn as soon as
// SeaVGABIOS sets a graphics mode, e.g. for an operating system. It tells
// which mode is set through video memory (see textscan.h), as the BDA is
// not updated when Windows XP's VideoPortInt10 sets a mode.
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "biosvar.h" // GET_BDA
#include "bregs.h" // struct bregs
#include "config.h" // CONFIG_VGA_TEXT_SCANOUT
#include "malloc.h" // malloc_high
#include "output.h" // dprintf
#include "stacks.h" // call16_int
#include "string.h" // memset
#include "textscan.h" // struct scanout_s
#include "util.h" // find_cb_table
#include "x86.h" // rdtscll

#define CB_TAG_FRAMEBUFFER      0x0012
struct cb_framebuffer {
    u32 tag;
    u32 size;
    u64 physical_address;
    u32 x_resolution;
    u32 y_resolution;
    u32 bytes_per_line;
    u8 bits_per_pixel;
    u8 red_mask_pos;
    u8 red_mask_size;
    u8 green_mask_pos;
    u8 green_mask_size;
    u8 blue_mask_pos;
    u8 blue_mask_size;
    u8 reserved_mask_pos;
    u8 reserved_mask_size;
};

#define TEXTSCAN_MAX_CELLS      (80*50)
#define TEXTSCAN_INTERVAL_MS    20

struct textscan_s {
    u8 *fb;
    u32 pitch, xres, yres;
    u32 interval, next;         // TSC ticks
    u8 *font[4];                // 8x8 (chars 0-127, 128-255), 8x14, 8x16
    u32 palette[16];
    u32 layout;                 // geometry on screen, 0 if not drawn
    u32 state;                  // scan-out state drawn (see textscan.h)
    int cursor;                 // cell with the cursor drawn, -1 if none
    u16 cursor_lines;
    u16 shadow[TEXTSCAN_MAX_CELLS]; // cells on screen
};

struct textscan_s *TextScan VARFSEG;
// Set while no text mode is shown, when the helper core may wait for
// requests with mwait.
u8 TextScanIdle VARFSEG;

// Get the scan-out state published by SeaVGABIOS, or 0 if there is none.
static u32
textscan_published(void)
{
    struct scanout_s *so = (void*)SCANOUT_OFFSET;
    u32 state = GET_FARVAR(SCANOUT_SEG, so->state);
    if (GET_FARVAR(SCANOUT_SEG, so->signature) != SCANOUT_SIGNATURE
        || !SCANOUT_STATE_VALID(state))
        return 0;
    return state;
}

// Get the scan-out state. Should something have overwritten the published
// one (e.g. a DOS memory manager mapping memory there), fall back to the
// BDA, which then is in use.
static u32
textscan_state(void)
{
    u32 state = textscan_published();
    if (!state)
        state = SCANOUT_STATE(GET_BDA(video_mode), 0xffff);
    return state;
}

// Check whether a scan-out state is a mode drawn by textscan_poll.
static int
textscan_mode(u32 state)
{
    u8 mode = state;
    return SCANOUT_STATE_VALID(state) && (mode <= 0x03 || mode == 0x07);
}

// Get a font of SeaVGABIOS.
static u8 *
textscan_font(u8 bh)
{
    struct bregs br;
    memset(&br, 0, sizeof(br));
    br.ax = 0x1130;
    br.bh = bh;
    br.flags = F_IF;
    call16_int(0x10, &br);
    return MAKE_FLATPTR(br.es, br.bp);
}

// Set up drawing if SeaVGABIOS has put a text mode in RAM.
void
textscan_setup(void)
{
    ASSERT32FLAT();
    if (!CONFIG_VGA_TEXT_SCANOUT || !textscan_mode(textscan_published()))
        return;
    struct cb_header *cbh = find_cb_table();
    if (!cbh)
        return;
    struct cb_framebuffer *cbfb = find_cb_subtable(cbh, CB_TAG_FRAMEBUFFER);
    if (!cbfb || cbfb->physical_address >> 32 || cbfb->bits_per_pixel != 32)
        return;
    struct textscan_s *ts = malloc_high(sizeof(*ts));
    if (!ts) {
        warn_noalloc();
        return;
    }
    memset(ts, 0, sizeof(*ts));
    ts->fb = (void*)(u32)cbfb->physical_address;
    ts->pitch = cbfb->bytes_per_line;
    ts->xres = cbfb->x_resolution;
    ts->yres = cbfb->y_resolution;
    ts->font[0] = textscan_font(3);
    ts->font[1] = textscan_font(4);
    ts->font[2] = textscan_font(2);
    ts->font[3] = textscan_font(6);
    // The 16 CGA colors
    int i;
    for (i = 0; i < 16; i++) {
        u8 h = i & 8 ? 0x55 : 0;
        u8 r = (i & 4 ? 0xaa : 0) + h, g = (i & 2 ? 0xaa : 0) + h;
        u8 b = (i & 1 ? 0xaa : 0) + h;
        if (i == 6)
            g = 0x55;
        ts->palette[i] = (r >> (8 - cbfb->red_mask_size)) << cbfb->red_mask_pos
            | (g >> (8 - cbfb->green_mask_size)) << cbfb->green_mask_pos
            | (b >> (8 - cbfb->blue_mask_size)) << cbfb->blue_mask_pos;
    }
    // The helper core can't use the BIOS timer, which may be shared
    // hardware, so measure the TSC.
    u32 end = timer_calc(1);
    u32 start = rdtscll();
    while (!timer_check(end))
        cpu_relax();
    ts->interval = ((u32)rdtscll() - start) * TEXTSCAN_INTERVAL_MS;
    ts->cursor = -1;
    TextScanIdle = 1;
    barrier();
    TextScan = ts;
    dprintf(1, "Text scan-out to framebuffer at %p (%d TSC ticks)\n"
            , ts->fb, ts->interval);
}

static void
textscan_fill(struct textscan_s *ts, u32 x, u32 y, u32 w, u32 h)
{
    u8 *dest = ts->fb + y * ts->pitch + x * 4;
    for (; h; h--, dest += ts->pitch) {
        u32 *d = (void*)dest, n = w;
        asm volatile("rep stosl" : "+D"(d), "+c"(n) : "a"(0) : "memory");
    }
}

// Draw the cells of the text buffer that changed since the last call.
static void
textscan_draw(struct textscan_s *ts, u32 state)
{
    u8 mode = state;
    u32 cols = GET_BDA(video_cols), rows = GET_BDA(video_rows) + 1;
    u32 cheight = GET_BDA(char_height), cwidth = cols <= 40 ? 16 : 8;
    u32 width = cols * cwidth, height = rows * cheight;
    u16 *text = (void*)(mode == 7 ? 0xb0000 : 0xb8000)
        + GET_BDA(video_pagestart);
    u8 *font = ts->font[cheight == 8 ? 0 : cheight == 14 ? 2 : 3];
    if ((cheight != 8 && cheight != 14 && cheight != 16)
        || cols * rows > TEXTSCAN_MAX_CELLS || width > ts->xres
        || height > ts->yres || (void*)&text[cols * rows] > (void*)0xc0000) {
        ts->layout = 0;
        return;
    }
    u32 xoff = (ts->xres - width) / 2, yoff = (ts->yres - height) / 2;

    // Redraw everything after a mode set.
    u32 layout = cols << 16 | rows << 8 | cheight;
    int full = layout != ts->layout || state != ts->state;
    if (full) {
        ts->layout = layout;
        ts->state = state;
        textscan_fill(ts, 0, 0, ts->xres, yoff);
        textscan_fill(ts, 0, yoff + height, ts->xres
                      , ts->yres - yoff - height);
        textscan_fill(ts, 0, yoff, xoff, height);
        textscan_fill(ts, xoff + width, yoff, ts->xres - xoff - width, height);
    }

    // Find the cursor and its scan lines (emulating CGA cursor shapes
    // like SeaVGABIOS does for the VGA CRTC).
    u16 pos = GET_BDA(cursor_pos[GET_BDA(video_page) & 7]);
    u16 type = GET_BDA(cursor_type);
    u32 start = (type >> 8) & 0x3f, end = type & 0x1f;
    if (!(GET_BDA(video_ctl) & 1) && cheight > 8 && end < 8 && start < 0x20) {
        if (end != start + 1)
            start = (start + 1) * cheight / 8 - 1;
        else
            start = (end + 1) * cheight / 8 - 2;
        end = (end + 1) * cheight / 8 - 1;
    }
    int cursor = (pos >> 8) * cols + (pos & 0xff);
    if ((pos & 0xff) >= cols || (pos >> 8) >= rows || start > end)
        cursor = -1;
    u16 lines = start << 8 | end;
    int oldcursor = ts->cursor;
    if (cursor != oldcursor || lines != ts->cursor_lines) {
        ts->cursor = cursor;
        ts->cursor_lines = lines;
    } else {
        oldcursor = cursor = -1;
    }

    u8 *dest = ts->fb + yoff * ts->pitch + xoff * 4;
    u32 x, y, i = 0;
    for (y = 0; y < rows; y++, dest += cheight * ts->pitch) {
        for (x = 0; x < cols; x++, i++) {
            u16 cell = text[i];
            if (cell == ts->shadow[i] && !full && i != cursor && i != oldcursor)
                continue;
            // Stop as soon as the text mode is left.
            if (textscan_state() != state) {
                ts->layout = 0;
                return;
            }
            ts->shadow[i] = cell;
            u8 c = cell, *glyph = font + c * cheight;
            if (cheight == 8 && c >= 128)
                glyph = ts->font[1] + (c - 128) * 8;
            u32 fg = ts->palette[(cell >> 8) & 0x0f];
            u32 bg = ts->palette[cell >> 12];
            u8 *line = dest + x * cwidth * 4;
            u32 j;
            for (j = 0; j < cheight; j++, line += ts->pitch) {
                u8 bits = glyph[j];
                if (i == ts->cursor && j >= start && j <= end)
                    bits = 0xff;
                u32 *pix = (void*)line;
                int k;
                for (k = 0; k < 8; k++, bits <<= 1) {
                    u32 color = bits & 0x80 ? fg : bg;
                    *pix++ = color;
                    if (cwidth > 8)
                        *pix++ = color;
                }
            }
        }
    }
}

// Called by the BIOS helper core whenever it is idle. Returns non-zero
// while a text mode is shown, so that the caller keeps polling.
u32 VISIBLE32FLAT
textscan_poll(void)
{
    struct textscan_s *ts = TextScan;
    if (!CONFIG_VGA_TEXT_SCANOUT || !ts)
        return 0;
    u32 state = textscan_state();
    u8 idle = !textscan_mode(state);
    if (TextScanIdle != idle)
        TextScanIdle = idle;
    if (idle) {
        ts->layout = 0;
        return 0;
    }
    u32 now = rdtscll();
    if ((s32)(now - ts->next) < 0)
        return 1;
    ts->next = now + ts->interval;
    textscan_draw(ts, state);
    return 1;
}

// Called from the BIOS timer tick. A text mode set through int 10h doesn't
// involve the helper core, so wake it up in case it waits with mwait.
void
textscan_check_event(void)
{
    if (CONFIG_VGA_TEXT_SCANOUT && GET_GLOBAL(TextScanIdle)
        && textscan_mode(textscan_state()))
        call32(textscan_poll, 0, 0);
}
