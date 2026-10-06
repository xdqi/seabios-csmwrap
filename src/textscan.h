#ifndef __TEXTSCAN_H
#define __TEXTSCAN_H

#include "types.h" // u32

// SeaVGABIOS publishes the video mode for the BIOS scan-out (textscan.c)
// in video memory, at the end of the monochrome text buffer, which color
// text and graphics modes leave unused. Unlike the BDA, video memory is
// also shared with the private copy of low memory in which Windows XP's
// VideoPortInt10 runs the video BIOS.
#define SCANOUT_SEG             0xb000
#define SCANOUT_OFFSET          0x7ff0
#define SCANOUT_SIGNATURE       0x65646f4d6e616353ULL // "ScanMode"
#define SCANOUT_NONE            0xff // mode the scan-out doesn't draw

struct scanout_s {
    u64 signature;
    u32 state;
    u32 reserved;
};

// The mode, its complement for validation, and a count of mode sets
#define SCANOUT_STATE(mode, gen) \
    ((mode) | ((~(mode) & 0xff) << 8) | ((u32)(gen) << 16))
#define SCANOUT_STATE_VALID(state) \
    (((state) & 0xff) == (~((state) >> 8) & 0xff))

#endif // textscan.h
