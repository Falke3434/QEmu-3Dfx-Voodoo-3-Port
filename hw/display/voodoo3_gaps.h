/*
 * QEMU 3Dfx Voodoo 3 -- gap tracker
 *
 * "The guest asked for something and the model did not do it."
 *
 * Every place where the device meets a value it does not implement (an
 * unknown texture format, a blend factor outside the table, a CMDFIFO packet
 * type it skips, a blitter command it ignores ...) reports it here with
 *
 *     voodoo3_note_gap(s, V3_GAP_<kind>, <the offending value>);
 *
 * The counters are indexed by the VALUE itself, so the tally says exactly
 * what the guest requested, not just that something was missing.
 *
 * - The first occurrence of each (kind, value) prints one warning
 *   ("voodoo3: unimplemented <what> 0x<value> -- the guest asked for it and
 *   did not get it"), later ones only count.
 * - Read the running tally at any time from the monitor:
 *       qom-get /machine/peripheral-anon/device[N] gaps
 *   (N = the device's index; `info qtree` shows it).  A clean run prints
 *   "none".
 * - With debug=on the tally is also appended to the periodic v3dbg
 *   summary, so it ends up in the same log file as the rest of the trace.
 *
 * WHAT A COUNT MEANS (it differs per kind, on purpose):
 *   - 3D pipeline state (combine selectors, blend factors): counted ONCE PER
 *     TRIANGLE, at queue time, never per pixel.  The render threads are
 *     lock-free and a per-pixel atomic would cost more than the rasterizer.
 *   - texture format: once per texture DECODE (the decoded texture is cached).
 *   - LFB writes, 2D blitter commands, CMDFIFO packets, register writes:
 *     once per event.
 *
 * Thread safety: the counters are updated with qatomic_*, so the vCPU thread,
 * the CMDFIFO thread and the render/queue paths may all report concurrently.
 *
 * Plain C, no host dependencies.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_VOODOO3_GAPS_H
#define HW_DISPLAY_VOODOO3_GAPS_H

#include <stdint.h>

typedef enum Voodoo3GapKind {
    /* 3D pipeline: per triangle */
    V3_GAP_CC_RGBSEL,        /* fbzColorPath cc_rgbselect (cother) */
    V3_GAP_CC_ASEL,          /* fbzColorPath cc_aselect (aother) */
    V3_GAP_CCA_LOCALSEL,     /* fbzColorPath cca_localselect */
    V3_GAP_CC_MSELECT,       /* fbzColorPath cc_mselect */
    V3_GAP_CCA_MSELECT,      /* fbzColorPath cca_mselect */
    V3_GAP_TMU_TC_MSELECT,   /* textureMode tc_mselect (colour), idx 0x10*tmu+v */
    V3_GAP_TMU_TCA_MSELECT,  /* textureMode tca_mselect (alpha), idx 0x10*tmu+v */
    V3_GAP_BLEND_SRC,        /* alphaMode source RGB blend factor */
    V3_GAP_BLEND_DST,        /* alphaMode destination RGB blend factor */
    /* textures: per decode */
    V3_GAP_TEX_FORMAT,       /* texture format code (decoded as grey) */
    /* linear frame buffer writes through the 3D pipe: per write */
    V3_GAP_LFB_FORMAT,       /* lfbMode pixel format, 32-bit write */
    V3_GAP_LFB_FORMAT16,     /* lfbMode pixel format, 16-bit write */
    /* 2D blitter: per command */
    V3_GAP_BLT_CMD,          /* blitter command opcode ignored */
    V3_GAP_BLT_DSTFMT,       /* dstFormat colour code (guessed 16 bpp) */
    /* command FIFO: per packet */
    V3_GAP_CMDFIFO_PKT,      /* packet type not implemented (header & 7) */
    V3_GAP_CMDFIFO_PKT0,     /* packet 0 sub-type not implemented */
    V3_GAP_CMDFIFO_AGPDMA,   /* packet 6 AGP DMA, ignored on PCI */
    /* registers: per write, idx = register offset / 4 */
    V3_GAP_REG_3D_WRITE,     /* 3D register 0x000..0x2fc, unhandled */
    V3_GAP_REG_EXT_WRITE,    /* extended register 0x00..0xfc, unhandled */
    V3_GAP_MAX
} Voodoo3GapKind;

/* idx values >= this are dropped (the reporting site's value range is
 * narrower than this for every kind above). */
#define V3_GAP_SLOTS 256

struct Voodoo3State;

/* Report one gap.  Safe from any thread. */
void voodoo3_note_gap(struct Voodoo3State *s, Voodoo3GapKind kind,
                      unsigned idx);

/* Per-triangle scan of the 3D state (defined in voodoo3_render.c, where the
 * register field macros live); called once per triangle at queue time. */
struct voodoo3_params_t;
void voodoo3_gap_scan_triangle(struct Voodoo3State *s,
                               const struct voodoo3_params_t *p);

#endif /* HW_DISPLAY_VOODOO3_GAPS_H */
