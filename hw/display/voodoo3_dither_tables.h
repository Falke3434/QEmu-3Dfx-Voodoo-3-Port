/*
 * QEMU 3Dfx Voodoo 3 — Hardware-accurate dither tables
 *
 * These tables are taken verbatim from 86Box's vid_voodoo_dither.h
 * (originally authored by Sarah Walker and leilei, Copyright 2008-2020).
 *
 * They encode the exact per-entry thresholds measured from real Voodoo
 * hardware and must NOT be replaced by runtime Bayer-matrix approximations.
 *
 * Table overview:
 *   dither_rb[256][4][4]      — 4×4 dither for R and B channels (→ 5-bit)
 *   dither_g[256][4][4]       — 4×4 dither for G channel        (→ 6-bit)
 *   dither_rb2x2[256][2][2]   — 2×2 dither for R and B channels (→ 5-bit)
 *   dither_g2x2[256][2][2]    — 2×2 dither for G channel        (→ 6-bit)
 *   dithersub_rb[256][4][4]   — 4×4 subtraction dither for R/B
 *   dithersub_g[256][4][4]    — 4×4 subtraction dither for G
 *   dithersub_rb2x2[256][2][2]— 2×2 subtraction dither for R/B
 *   dithersub_g2x2[256][2][2] — 2×2 subtraction dither for G
 *
 * Usage:
 *   4×4:  table[value][screen_y & 3][x & 3]
 *   2×2:  table[value][screen_y & 1][x & 1]
 *
 * Original 86Box source: src/include/86box/vid_voodoo_dither.h
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_VOODOO3_DITHER_TABLES_H
#define HW_DISPLAY_VOODOO3_DITHER_TABLES_H

#include <stdint.h>

/* 4×4 ordered dither — primary forward dither */
extern const uint8_t voodoo3_dither_rb[256][4][4];
extern const uint8_t voodoo3_dither_g[256][4][4];

/* 2×2 ordered dither — primary forward dither */
extern const uint8_t voodoo3_dither_rb2x2[256][2][2];
extern const uint8_t voodoo3_dither_g2x2[256][2][2];

/* Subtraction dither (FBZ_DITHER_SUB / fbzMode bit 19) */
extern const uint8_t voodoo3_dithersub_rb[256][4][4];
extern const uint8_t voodoo3_dithersub_g[256][4][4];
extern const uint8_t voodoo3_dithersub_rb2x2[256][2][2];
extern const uint8_t voodoo3_dithersub_g2x2[256][2][2];

#endif /* HW_DISPLAY_VOODOO3_DITHER_TABLES_H */
