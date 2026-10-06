// Simple framebuffer vgabios for use with coreboot native vga init.
//
// Copyright (C) 2014  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2017  Patrick Rudolph <siro@das-labor.org>
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "biosvar.h" // GET_BDA
#include "output.h" // dprintf
#include "stdvga.h" // SEG_CTEXT
#include "string.h" // memset16_far
#include "util.h" // find_cb_table
#include "vgabios.h" // SET_VGA
#include "vgafb.h" // handle_gfx_op
#include "vgautil.h" // VBE_total_memory
#include "x86.h" // inb
#include "svgamodes.h" // svga_modes
#include "textscan.h" // struct scanout_s

static int CBmode VAR16;
static struct vgamode_s CBmodeinfo VAR16;
static struct vgamode_s CBemulinfo VAR16;
static u32 CBlinelength VAR16;

// Without VGA hardware the standard text modes are kept in the legacy text
// buffer, which the BIOS draws on the framebuffer (see src/textscan.c).
// Optionally mode 12h is approximated the same way with a single bit plane
// at 0xa0000, losing its colors.
static u8 CBtextscan VAR16, CBplanar VAR16;
static struct vgamode_s CBscanmodes[] VAR16 = {
    { MM_TEXT, 40, 25, 4, 8, 16, SEG_CTEXT }, // modes 0 and 1
    { MM_TEXT, 80, 25, 4, 8, 16, SEG_CTEXT }, // modes 2 and 3
    { MM_TEXT, 80, 25, 4, 8, 16, SEG_MTEXT }, // mode 7
    { MM_PLANAR, 640, 480, 4, 8, 16, SEG_GRAPH }, // mode 12h
};
static u8 CBscanmodenum[] VAR16 = { 0x01, 0x03, 0x07, 0x12 };

static int
cbvga_is_scanmode(struct vgamode_s *vmode_g)
{
    return CONFIG_VGA_TEXT_SCANOUT && vmode_g >= CBscanmodes
        && vmode_g < &CBscanmodes[ARRAY_SIZE(CBscanmodes)];
}

struct vgamode_s *cbvga_find_mode(int mode)
{
    if (mode == GET_GLOBAL(CBmode))
        return &CBmodeinfo;
    if (CONFIG_VGA_TEXT_SCANOUT && GET_GLOBAL(CBtextscan)
        && (mode <= 0x03 || mode == 0x07))
        return &CBscanmodes[mode == 0x07 ? 2 : mode / 2];
    if (CONFIG_VGA_TEXT_SCANOUT && GET_GLOBAL(CBplanar) && mode == 0x12)
        return &CBscanmodes[3];
    if (mode == 0x03)
        return &CBemulinfo;

    int i;
    for (i = 0; i < GET_GLOBAL(svga_mcount); i++) {
        struct generic_svga_mode *cbmode_g = &svga_modes[i];
        if (GET_GLOBAL(cbmode_g->mode) == 0xffff)
            continue;
        if (GET_GLOBAL(cbmode_g->mode) == mode)
            return &cbmode_g->info;
    }
    return NULL;
}

void
cbvga_list_modes(u16 seg, u16 *dest, u16 *last)
{
    int seen = 0;

    if (GET_GLOBAL(CBmode) != 0x3) {
        /* Advertise additional SVGA modes for Microsoft NTLDR graphical mode.
         * Microsoft NTLDR:
         * + Graphical mode uses a maximum resolution of 1600x1200.
         * + Expects to find VESA mode with 800x600 or 1024x768.
         * + 24 Bpp and 32 Bpp are supported
         */
        int i;
        for (i = 0; i < GET_GLOBAL(svga_mcount) && dest < last; i++) {
            struct generic_svga_mode *cbmode_g = &svga_modes[i];
            u16 mode = GET_GLOBAL(cbmode_g->mode);
            if (mode == 0xffff)
                continue;
            SET_FARVAR(seg, *dest, mode);
            dest++;
            if (GET_GLOBAL(CBmode) == mode)
                seen = 1;
        }
    }
    if (dest < last && !seen) {
        SET_FARVAR(seg, *dest, GET_GLOBAL(CBmode));
        dest++;
    }
    SET_FARVAR(seg, *dest, 0xffff);
}

int
cbvga_get_window(struct vgamode_s *curmode_g, int window)
{
    return -1;
}

int
cbvga_set_window(struct vgamode_s *curmode_g, int window, int val)
{
    return -1;
}

int
cbvga_minimum_linelength(struct vgamode_s *vmode_g)
{
    /* Can't change mode, always report active pitch. */
    return GET_GLOBAL(CBlinelength);
}

int
cbvga_get_linelength(struct vgamode_s *curmode_g)
{
    if (CONFIG_VGA_TEXT_SCANOUT && curmode_g == &CBscanmodes[3])
        return 640 / 8;
    return GET_GLOBAL(CBlinelength);
}

int
cbvga_set_linelength(struct vgamode_s *curmode_g, int val)
{
    return -1;
}

int
cbvga_get_displaystart(struct vgamode_s *curmode_g)
{
    return 0;
}

int
cbvga_set_displaystart(struct vgamode_s *curmode_g, int val)
{
    return -1;
}

int
cbvga_get_dacformat(struct vgamode_s *curmode_g)
{
    return -1;
}

int
cbvga_set_dacformat(struct vgamode_s *curmode_g, int val)
{
    return -1;
}

// Publish the current mode for the BIOS scan-out (see src/textscan.h).
static void
cbvga_scanout_set(u8 mode)
{
    struct scanout_s *so = (void*)SCANOUT_OFFSET;
    u32 state = GET_FARVAR(SCANOUT_SEG, so->state);
    u16 gen = 0;
    if (GET_FARVAR(SCANOUT_SEG, so->signature) == SCANOUT_SIGNATURE
        && SCANOUT_STATE_VALID(state))
        gen = (state >> 16) + 1;
    else
        SET_FARVAR(SCANOUT_SEG, so->signature, SCANOUT_SIGNATURE);
    SET_FARVAR(SCANOUT_SEG, so->state, SCANOUT_STATE(mode, gen));
}

// Return the memory model of the current mode if it is kept in RAM and
// drawn by the BIOS (see src/textscan.c), else -1.
int
cbvga_scanout_memmodel(void)
{
    struct vgamode_s *curmode_g = get_current_mode();
    if (!cbvga_is_scanmode(curmode_g))
        return -1;
    return GET_GLOBAL(curmode_g->memmodel);
}

int
cbvga_save_restore(int cmd, u16 seg, void *data)
{
    if (cmd & (SR_HARDWARE|SR_DAC|SR_REGISTERS))
        return -1;
    int ret = bda_save_restore(cmd, seg, data);
    if (CONFIG_VGA_TEXT_SCANOUT && GET_GLOBAL(CBtextscan)
        && cmd & SR_RESTORE && cmd & SR_BDA)
        cbvga_scanout_set(GET_BDA(video_mode));
    return ret;
}

int
cbvga_set_mode(struct vgamode_s *vmode_g, int flags)
{
    u8 emul = vmode_g == &CBemulinfo || GET_GLOBAL(CBmode) == 0x03;
    /*
     * The extra_stack flag is false when running in windows x86
     * emulator, to avoid stack switching triggering bugs.  Using the
     * same flag here to skip screen clearing, because the windows
     * emulator seems to have problems to handle the int 1587 call
     * too, and GO_MEMSET uses that.
     */
    u8 extra_stack = GET_BDA_EXT(flags) & BF_EXTRA_STACK;
    MASK_BDA_EXT(flags, BF_EMULATE_TEXT, emul ? BF_EMULATE_TEXT : 0);
    if (cbvga_is_scanmode(vmode_g)) {
        if (!(flags & MF_NOCLEARMEM)) {
            if (GET_GLOBAL(vmode_g->memmodel) == MM_TEXT)
                memset16_far(GET_GLOBAL(vmode_g->sstart), (void*)0, 0x0720
                             , 32*1024);
            else
                memset_far(SEG_GRAPH, (void*)0, 0, 640 * 480 / 8);
        }
        // Have the BIOS draw the mode (and the whole screen again).
        cbvga_scanout_set(GET_GLOBAL(CBscanmodenum[vmode_g - CBscanmodes]));
        return 0;
    }
    if (CONFIG_VGA_TEXT_SCANOUT && GET_GLOBAL(CBtextscan))
        // Stop the BIOS drawing before the framebuffer gets cleared.
        cbvga_scanout_set(SCANOUT_NONE);
    if (!(flags & MF_NOCLEARMEM)) {
        if (GET_GLOBAL(CBmodeinfo.memmodel) == MM_TEXT) {
            memset16_far(SEG_CTEXT, (void*)0, 0x0720, 80*25*2);
            return 0;
        }
        if (extra_stack || flags & MF_LEGACY) {
            struct gfx_op op;
            init_gfx_op(&op, &CBmodeinfo);
            op.x = op.y = 0;
            op.xlen = GET_GLOBAL(CBmodeinfo.width);
            op.ylen = GET_GLOBAL(CBmodeinfo.height);
            op.op = GO_MEMSET;
            handle_gfx_op(&op);
        }
    }
    return 0;
}

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

// CSMWrap private record with video options
#define CB_TAG_CSMWRAP_VIDEO    0x43534d57
#define CB_CSMWRAP_VIDEO_PLANAR_APPROX 0x01
struct cb_csmwrap_video {
    u32 tag;
    u32 size;
    u32 flags;
};

void
cbvga_setup_modes(u64 addr, u8 bpp, u32 xlines, u32 ylines, u32 linelength)
{
    int i;

    SET_VGA(CBmode, 0x140);
    SET_VGA(VBE_framebuffer, addr);
    SET_VGA(VBE_total_memory, linelength * ylines);
    SET_VGA(CBlinelength, linelength);
    SET_VGA(CBmodeinfo.memmodel, MM_DIRECT);
    SET_VGA(CBmodeinfo.width, xlines);
    SET_VGA(CBmodeinfo.height, ylines);
    SET_VGA(CBmodeinfo.depth, bpp);
    SET_VGA(CBmodeinfo.cwidth, 8);
    SET_VGA(CBmodeinfo.cheight, 16);
    memcpy_far(get_global_seg(), &CBemulinfo
               , get_global_seg(), &CBmodeinfo, sizeof(CBemulinfo));

    // Validate modes
    for (i = 0; i < GET_GLOBAL(svga_mcount); i++) {
        struct generic_svga_mode *cbmode_g = &svga_modes[i];
        /* Skip VBE modes that doesn't fit into coreboot's framebuffer */
        if ((GET_GLOBAL(cbmode_g->info.memmodel) != MM_DIRECT)
            || (GET_GLOBAL(cbmode_g->info.height) > ylines)
            || (GET_GLOBAL(cbmode_g->info.width) > xlines)
            || (GET_GLOBAL(cbmode_g->info.depth) != bpp)) {
            dprintf(3, "Removing mode %x\n", GET_GLOBAL(cbmode_g->mode));
            SET_VGA(cbmode_g->mode, 0xffff);
        }
        if ((GET_GLOBAL(cbmode_g->info.height) == ylines)
            && (GET_GLOBAL(cbmode_g->info.width) == xlines)
            && (GET_GLOBAL(cbmode_g->info.depth) == bpp)) {
            SET_VGA(CBmode, GET_GLOBAL(cbmode_g->mode));
        }
    }
}

// Check whether the standard VGA registers respond.
static int
cbvga_vga_present(void)
{
    u8 index = inb(VGAREG_GRDC_ADDRESS);
    outb(0x05, VGAREG_GRDC_ADDRESS);
    u8 val = inb(VGAREG_GRDC_ADDRESS);
    outb(index, VGAREG_GRDC_ADDRESS);
    return val != 0xff || inb(VGAREG_READ_MISC_OUTPUT) != 0xff;
}

// Check whether memory at the given segment keeps what is written to it.
static int
cbvga_is_ram(u16 seg)
{
    u16 old = GET_FARVAR(seg, *(u16*)0);
    SET_FARVAR(seg, *(u16*)0, 0x55aa);
    int ret = GET_FARVAR(seg, *(u16*)0) == 0x55aa;
    SET_FARVAR(seg, *(u16*)0, old);
    return ret;
}

int
cbvga_setup(void)
{
    dprintf(1, "coreboot vga init\n");

    if (GET_GLOBAL(HaveRunInit))
        return 0;

    struct cb_header *cbh = find_cb_table();
    if (!cbh) {
        dprintf(1, "Unable to find coreboot table\n");
        return -1;
    }
    struct cb_framebuffer *cbfb = find_cb_subtable(cbh, CB_TAG_FRAMEBUFFER);
    if (!cbfb) {
        // Assume there is an EGA text framebuffer.
        dprintf(1, "Did not find coreboot framebuffer - assuming EGA text\n");
        SET_VGA(CBmode, 0x03);
        SET_VGA(CBlinelength, 80*2);
        SET_VGA(CBmodeinfo.memmodel, MM_TEXT);
        SET_VGA(CBmodeinfo.width, 80);
        SET_VGA(CBmodeinfo.height, 25);
        SET_VGA(CBmodeinfo.depth, 4);
        SET_VGA(CBmodeinfo.cwidth, 9);
        SET_VGA(CBmodeinfo.cheight, 16);
        SET_VGA(CBmodeinfo.sstart, SEG_CTEXT);
        return 0;
    }

    u64 addr = GET_FARVAR(0, cbfb->physical_address);
    u8 bpp = GET_FARVAR(0, cbfb->blue_mask_size)
             + GET_FARVAR(0, cbfb->green_mask_size)
             + GET_FARVAR(0, cbfb->red_mask_size)
             + GET_FARVAR(0, cbfb->reserved_mask_size);
    u32 xlines = GET_FARVAR(0, cbfb->x_resolution);
    u32 ylines = GET_FARVAR(0, cbfb->y_resolution);
    u32 linelength = GET_FARVAR(0, cbfb->bytes_per_line);
    //fall back to coreboot reported bpp if calculated value invalid
    if (bpp != 15 && bpp != 16 && bpp != 24 && bpp != 32)
        bpp = GET_FARVAR(0, cbfb->bits_per_pixel);

    dprintf(1, "Found FB @ %llx %dx%d with %d bpp (%d stride)\n"
            , addr, xlines, ylines, bpp, linelength);

    if (!addr || addr > 0xffffffff
        || (bpp != 15 && bpp != 16 && bpp != 24 && bpp != 32)) {
        dprintf(1, "Unable to use FB\n");
        return -1;
    }

    cbvga_setup_modes(addr, bpp, xlines, ylines, linelength);

    if (!CONFIG_VGA_TEXT_SCANOUT || bpp != 32 || xlines < 80*8
        || ylines < 25*16 || cbvga_vga_present()
        || !cbvga_is_ram(SEG_CTEXT) || !cbvga_is_ram(SEG_MTEXT))
        return 0;
    dprintf(1, "No VGA hardware - keeping text modes in RAM\n");
    SET_VGA(CBtextscan, 1);

    struct cb_csmwrap_video *cbvid = find_cb_subtable(cbh
                                                      , CB_TAG_CSMWRAP_VIDEO);
    if (cbvid && GET_FARVAR(0, cbvid->flags) & CB_CSMWRAP_VIDEO_PLANAR_APPROX
        && ylines >= 480 && cbvga_is_ram(SEG_GRAPH)) {
        dprintf(1, "Approximating mode 12h with one plane in RAM\n");
        SET_VGA(CBplanar, 1);
    }
    return 0;
}
