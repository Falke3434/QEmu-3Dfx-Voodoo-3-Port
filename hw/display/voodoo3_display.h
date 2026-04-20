/*
 * QEMU 3Dfx Voodoo 3 — Display subsystem header
 *
 * Copyright (C) 2026 <your name here>
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_VOODOO3_DISPLAY_H
#define HW_DISPLAY_VOODOO3_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>

typedef struct Voodoo3State Voodoo3State;

/* Number of dirty-line tracking slots (max vertical resolution tracked) */
#define V3_DIRTY_LINES  2048

/* Dither table access (built by voodoo3_init_dither_tables()) */
void voodoo3_init_dither_tables(void);
extern const uint8_t (*voodoo3_dither_rb)[4][4];  /* [256][4][4] */
extern const uint8_t (*voodoo3_dither_g )[4][4];  /* [256][4][4] */

/* NCC (YIQ) lookup table update — call when nccTable regs change */
void voodoo3_update_ncc(Voodoo3State *s, int tmu);

/* FastFill — clear draw + depth buffers within clip rectangle */
void voodoo3_fastfill(Voodoo3State *s);

/* SwapBuffer — request a front/back flip at next vblank */
void voodoo3_swap_buffer(Voodoo3State *s, uint32_t cmd_val);

/* Called from vblank callback — perform flip if swap_interval elapsed */
void voodoo3_do_swap_if_pending(Voodoo3State *s);

/* Dirty-line-aware display blit — only redraws changed scanlines */
void voodoo3_update_display_dirty(Voodoo3State *s);

/* Hardware cursor compositing — call after voodoo3_update_display_dirty() */
void voodoo3_draw_cursor(Voodoo3State *s,
                         uint8_t *dst_base, int dst_bpp, int dst_pitch,
                         int w, int dirty_lo, int dirty_hi);

/* Video overlay compositing — call after desktop blit, before cursor */
void voodoo3_overlay_draw(Voodoo3State *s,
                          uint8_t *dst_base, int dst_bpp, int dst_pitch,
                          int scr_w, int dirty_lo, int dirty_hi);

#endif /* HW_DISPLAY_VOODOO3_DISPLAY_H */
