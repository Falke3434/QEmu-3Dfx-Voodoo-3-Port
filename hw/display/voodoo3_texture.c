/*
 * QEMU 3Dfx Voodoo 3 — Texture Subsystem
 *
 * Ported from 86Box vid_voodoo_texture.c
 * Original author: Sarah Walker <https://pcem-emulator.co.uk/>
 * Copyright (C) 2008-2024 Sarah Walker and 86Box contributors
 * Copyright (C) 2026 <your name here>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * -------------------------------------------------------------------------
 * What this file provides
 * -------------------------------------------------------------------------
 *  voodoo3_recalc_tex()    — compute tex_base/mask/shift/lod arrays for one
 *                            TMU from the tLOD / textureMode registers.
 *                            Ported from voodoo_recalc_tex3() which is used
 *                            for Voodoo 3 (type >= VOODOO_BANSHEE).
 *
 *  voodoo3_tex_download()  — decode raw FIFO_WRITEL_TEX writes into the
 *                            texture RAM and invalidate affected cache slots.
 *                            Ported from voodoo_tex_writel().
 *
 *  voodoo3_use_texture()   — look up or decode a texture into the cache and
 *                            return a pointer array for the rasterizer.
 *                            Ported from voodoo_use_texture().
 *
 * Texture formats decoded (all 86Box TEX_* values):
 *   TEX_RGB332, TEX_Y4I2Q2, TEX_A8, TEX_I8, TEX_AI8,
 *   TEX_PAL8, TEX_APAL8, TEX_ARGB8332, TEX_A8Y4I2Q2,
 *   TEX_R5G6B5, TEX_ARGB1555, TEX_ARGB4444, TEX_A8I8, TEX_APAL88
 * -------------------------------------------------------------------------
 */

#include "qemu/osdep.h"
#include "qemu/atomic.h"
#include "qemu/bitmap.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "hw/display/voodoo3_int.h"
#include "hw/display/voodoo3_texture.h"

/* =========================================================================
 * Texture format codes — from 86Box vid_voodoo_regs.h
 * ========================================================================= */
#define TEX_RGB332    0
#define TEX_Y4I2Q2    1
#define TEX_A8        2
#define TEX_I8        3
#define TEX_AI8       4
#define TEX_PAL8      5
#define TEX_APAL8     6
#define TEX_ARGB8332  8
#define TEX_A8Y4I2Q2  9
#define TEX_R5G6B5    10
#define TEX_ARGB1555  11
#define TEX_ARGB4444  12
#define TEX_A8I8      13
#define TEX_APAL88    14
#define TEX_ARGB_8888 15  /* GR_TEXFMT_ARGB_8888 — 32-bit, 4 bytes/texel (Voodoo3/Banshee) */

/* tLOD bit fields */
/* tLOD / textureMode bits — 86Box vid_voodoo_regs.h.  The previous values
 * (LOD_ODD 1<<24, LOD_SPLIT 1<<23, TMULTIBASEADDR 1<<25, TRILINEAR 1<<2)
 * were wrong: bit 2 of textureMode is the magnification filter, so every
 * bilinear texture was treated as trilinear and its mip levels were taken
 * from the wrong addresses. */
#define LOD_ODD             (1 << 18)
#define LOD_SPLIT           (1 << 19)
#define LOD_S_IS_WIDER      (1 << 20)
#define LOD_TMULTIBASEADDR  (1 << 24)
#define LOD_TMIRROR_S       (1 << 28)
#define LOD_TMIRROR_T       (1 << 29)
#define TEXTUREMODE_TRILINEAR (1u << 30)
#define TEXTUREMODE_NCC_SEL   (1 << 5)

/* Pack RGBA bytes into a 32-bit ABGR word (86Box internal format) */
#define MAKERGBA(r, g, b, a) \
    ((uint32_t)(b) | ((uint32_t)(g) << 8) | ((uint32_t)(r) << 16) | ((uint32_t)(a) << 24))

/* =========================================================================
 * Colour-conversion look-up tables
 * These are generated from the raw bit patterns of each format.
 * ========================================================================= */

/* RGB332 → R8G8B8 */
static uint8_t rgb332_r[256], rgb332_g[256], rgb332_b[256];

/* RGB565 → R8G8B8 */
static uint8_t rgb565_r[65536], rgb565_g[65536], rgb565_b[65536];

/* ARGB1555 → A8R8G8B8 */
static uint8_t a1555_r[65536], a1555_g[65536], a1555_b[65536], a1555_a[65536];

/* ARGB4444 → A8R8G8B8 */
static uint8_t a4444_r[65536], a4444_g[65536], a4444_b[65536], a4444_a[65536];

static bool lut_init_done;

static void voodoo3_init_luts(void)
{
    if (lut_init_done) return;
    lut_init_done = true;

    for (int i = 0; i < 256; i++) {
        rgb332_r[i] = (uint8_t)(((i >> 5) & 7) * 255 / 7);
        rgb332_g[i] = (uint8_t)(((i >> 2) & 7) * 255 / 7);
        rgb332_b[i] = (uint8_t)((i & 3) * 255 / 3);
    }

    for (int i = 0; i < 65536; i++) {
        rgb565_r[i] = (uint8_t)(((i >> 11) & 0x1f) * 255 / 31);
        rgb565_g[i] = (uint8_t)(((i >>  5) & 0x3f) * 255 / 63);
        rgb565_b[i] = (uint8_t)((i & 0x1f) * 255 / 31);

        a1555_r[i] = (uint8_t)(((i >> 10) & 0x1f) * 255 / 31);
        a1555_g[i] = (uint8_t)(((i >>  5) & 0x1f) * 255 / 31);
        a1555_b[i] = (uint8_t)((i & 0x1f) * 255 / 31);
        a1555_a[i] = (i & 0x8000) ? 0xff : 0x00;

        uint8_t a4 = (i >> 12) & 0xf;
        uint8_t r4 = (i >>  8) & 0xf;
        uint8_t g4 = (i >>  4) & 0xf;
        uint8_t b4 =  i        & 0xf;
        a4444_r[i] = (uint8_t)((r4 << 4) | r4);
        a4444_g[i] = (uint8_t)((g4 << 4) | g4);
        a4444_b[i] = (uint8_t)((b4 << 4) | b4);
        a4444_a[i] = (uint8_t)((a4 << 4) | a4);
    }
}

/* =========================================================================
 * Texture parameter calculation
 *
 * Ported from 86Box voodoo_recalc_tex3() which handles Voodoo 3 / Banshee.
 * Populates the per-LOD geometry arrays in voodoo3_tex_params_t.
 * ========================================================================= */
void voodoo3_recalc_tex(voodoo3_tex_params_t *tp, uint32_t tLOD,
                        uint32_t textureMode, uint32_t texBaseAddr,
                        uint32_t texBaseAddr1, uint32_t texBaseAddr2,
                        uint32_t texBaseAddr38, int tformat)
{
    int      aspect = (int)((tLOD >> 21) & 3);
    int      width  = 256, height = 256, shift = 8;
    int      lod;
    uint32_t offset = 0;
    int      tex_lod = 0;

    uint32_t offsets[V3_LOD_MAX + 3];
    int      widths [V3_LOD_MAX + 3];
    int      heights[V3_LOD_MAX + 3];
    int      shifts [V3_LOD_MAX + 3];

    if (tLOD & LOD_S_IS_WIDER)
        height >>= aspect;
    else { width >>= aspect; shift -= aspect; }

    /* Pre-compute per-mip geometry */
    for (lod = 0; lod <= V3_LOD_MAX + 2; lod++) {
        int w = width  >> lod;
        int h = height >> lod;
        int s = shift  -  lod;
        if (!w) w = 1;
        if (!h) h = 1;
        if (s < 0) s = 0;
        offsets[lod] = offset;
        widths [lod] = w;
        heights[lod] = h;
        shifts [lod] = s;

        bool store_this = !(tLOD & LOD_SPLIT) ||
            ((lod & 1) && (tLOD & LOD_ODD)) ||
            (!(lod & 1) && !(tLOD & LOD_ODD));
        if (store_this) {
            /* 86Box voodoo_recalc_tex3(): unclamped (width>>lod)*(height>>lod),
             * i.e. mip levels below 1x1 add nothing to the offset. */
            uint32_t uw = (uint32_t)(width >> lod), uh = (uint32_t)(height >> lod);
            if (tformat == TEX_ARGB_8888)
                offset += uw * uh * 4;
            else if (tformat & 8)
                offset += uw * uh * 2;
            else
                offset += uw * uh;
        }
    }

    if ((textureMode & TEXTUREMODE_TRILINEAR) && (tLOD & LOD_ODD))
        tex_lod++; /* Skip LOD 0 for trilinear odd */

    for (lod = 0; lod <= V3_LOD_MAX + 1; lod++) {
        uint32_t base = texBaseAddr;
        if (tLOD & LOD_TMULTIBASEADDR) {
            switch (tex_lod) {
            case 0:  base = texBaseAddr;   break;
            case 1:  base = texBaseAddr1;  break;
            case 2:  base = texBaseAddr2;  break;
            default: base = texBaseAddr38; break;
            }
        }

        int tl = (tex_lod < V3_LOD_MAX + 2) ? tex_lod : V3_LOD_MAX + 1;
        tp->tex_base  [lod] = base + offsets[tl];
        tp->tex_w_mask[lod] = widths [tl] - 1;
        tp->tex_h_mask[lod] = heights[tl] - 1;
        tp->tex_shift [lod] = shifts [tl];
        tp->tex_lod   [lod] = tex_lod;
        if (tformat == TEX_ARGB_8888)
            tp->tex_end[lod] = base + offsets[tl]
                              + (uint32_t)(widths[tl] * heights[tl] * 4);
        else if (tformat & 8)
            tp->tex_end[lod] = base + offsets[tl]
                              + (uint32_t)(widths[tl] * heights[tl] * 2);
        else
            tp->tex_end[lod] = base + offsets[tl]
                              + (uint32_t)(widths[tl] * heights[tl]);

        bool advance = !(textureMode & TEXTUREMODE_TRILINEAR) ||
            ((lod & 1) && (tLOD & LOD_ODD)) ||
            (!(lod & 1) && !(tLOD & LOD_ODD));
        if (advance && !((tLOD & LOD_ODD) && lod == 0)) {
            tex_lod += (textureMode & TEXTUREMODE_TRILINEAR) ? 2 : 1;
        }
    }

    tp->tformat      = tformat;
    tp->tLOD         = tLOD;
    tp->textureMode  = textureMode;
    tp->base         = texBaseAddr;
    tp->width        = widths[0];
}

/* =========================================================================
 * texture_offset[] — mip-chain word offsets in the decoded cache.
 * Matches 86Box vid_voodoo_texture.h texture_offset[LOD_MAX + 3].
 * data[texture_offset[lod]] is the first word for LOD level lod.
 * ========================================================================= */
static const uint32_t texture_offset[V3_LOD_MAX + 3] = {
    0,
    256 * 256,
    256 * 256 + 128 * 128,
    256 * 256 + 128 * 128 + 64 * 64,
    256 * 256 + 128 * 128 + 64 * 64 + 32 * 32,
    256 * 256 + 128 * 128 + 64 * 64 + 32 * 32 + 16 * 16,
    256 * 256 + 128 * 128 + 64 * 64 + 32 * 32 + 16 * 16 + 8 * 8,
    256 * 256 + 128 * 128 + 64 * 64 + 32 * 32 + 16 * 16 + 8 * 8 + 4 * 4,
    256 * 256 + 128 * 128 + 64 * 64 + 32 * 32 + 16 * 16 + 8 * 8 + 4 * 4 + 2 * 2,
    256 * 256 + 128 * 128 + 64 * 64 + 32 * 32 + 16 * 16 + 8 * 8 + 4 * 4 + 2 * 2 + 1,
    256 * 256 + 128 * 128 + 64 * 64 + 32 * 32 + 16 * 16 + 8 * 8 + 4 * 4 + 2 * 2 + 1 + 1
};

/* =========================================================================
 * Decode one texture from SGRAM into the cache
 *
 * Ported from 86Box voodoo_use_texture() — the inner decode loop.
 * Reads raw bytes from tex_mem[] and converts to ABGR32 words in cache[].
 * Uses texture_offset[lod] for correct mip-chain placement.
 * ========================================================================= */
static void decode_texture(Voodoo3State *s, v3_tex_cache_entry_t *entry,
                           const voodoo3_tex_params_t *tp, int tmu,
                           int lod_min, int lod_max)
{
    voodoo3_init_luts();

    uint8_t *tex_mem  = s->tex_mem[tmu];
    /*
     * SGRAM holds CPU-written data in CPU byte order; the chip saw it
     * rearranged by the LFB swizzle (byte k at k ^ x).  Texels uploaded by
     * the CPU through the frame buffer (MiniGL/Warp3D do this) must be read
     * the way the TMU saw them.  Engine writes (texture aperture, CMDFIFO
     * packet 5) are stored with the same rearrangement, so one rule fits.
     */
    const uint32_t sx = v3_lfb_x(s);
#define TB(a)   (tex_mem[((uint32_t)(a) & tex_mask) ^ sx])
#define T16(a)  ((uint16_t)(TB(a) | (TB((a) + 1) << 8)))
#define T32(a)  ((uint32_t)TB(a) | ((uint32_t)TB((a) + 1) << 8) | \
                 ((uint32_t)TB((a) + 2) << 16) | ((uint32_t)TB((a) + 3) << 24))
    uint32_t tex_mask = s->tex_mask;
    int      tformat  = tp->tformat;

    lod_min = MIN(lod_min, V3_LOD_MAX);
    lod_max = MIN(lod_max, V3_LOD_MAX);

    for (int lod = lod_min; lod <= lod_max; lod++) {
        uint32_t *base     = &entry->data[texture_offset[lod]];
        uint32_t  tex_addr = tp->tex_base[lod] & tex_mask;
        int       w        = tp->tex_w_mask[lod] + 1;
        int       h        = tp->tex_h_mask[lod] + 1;
        int       src_shift= tp->tex_shift[lod];
        /*
         * Row byte-stride depends on bytes-per-texel:
         *   8-bit  formats (tformat 0–6):  stride = width     = 1 << src_shift
         *   16-bit formats (tformat 8–14): stride = width * 2 = 1 << (src_shift+1)
         *   32-bit format  (tformat 15):   stride = width * 4 = 1 << (src_shift+2)
         * Ported from 86Box vid_voodoo_texture.c where 16-bit uses
         * `tex_addr += (1 << (tex_shift+1))` per row.
         */
        int row_shift = (tformat == TEX_ARGB_8888) ? src_shift + 2
                      : (tformat & 8)              ? src_shift + 1
                                                   : src_shift;

        for (int y = 0; y < h; y++) {
            uint32_t  row_addr = tex_addr + (uint32_t)(y << row_shift);
            uint32_t *dst_row  = base + (uint32_t)(y * w);
            for (int x = 0; x < w; x++) {
                uint32_t out;
                switch (tformat) {
                case TEX_RGB332: {
                    uint8_t d = TB(row_addr + (uint32_t)x);
                    out = MAKERGBA(rgb332_r[d], rgb332_g[d], rgb332_b[d], 0xff);
                    break; }
                case TEX_Y4I2Q2: {
                    uint8_t  d   = TB(row_addr + (uint32_t)x);
                    int      sel = (tp->textureMode & TEXTUREMODE_NCC_SEL) ? 1 : 0;
                    uint32_t c   = s->ncc_lookup[tmu][sel][d];
                    out = c;
                    break; }
                case TEX_A8: {
                    uint8_t d = TB(row_addr + (uint32_t)x);
                    out = MAKERGBA(d, d, d, d);
                    break; }
                case TEX_I8: {
                    uint8_t d = TB(row_addr + (uint32_t)x);
                    out = MAKERGBA(d, d, d, 0xff);
                    break; }
                case TEX_AI8: {
                    uint8_t d  = TB(row_addr + (uint32_t)x);
                    uint8_t lo = (uint8_t)((d & 0x0f) | ((d & 0x0f) << 4));
                    uint8_t hi = (uint8_t)((d & 0xf0) | ((d & 0xf0) >> 4));
                    out = MAKERGBA(lo, lo, lo, hi);
                    break; }
                case TEX_PAL8: {
                    uint8_t  idx = TB(row_addr + (uint32_t)x);
                    uint32_t c   = s->tex_palette[tmu][idx];
                    out = MAKERGBA((c >> 16) & 0xff, (c >> 8) & 0xff,
                                    c & 0xff, 0xff);
                    break; }
                case TEX_APAL8: {
                    uint8_t  idx = TB(row_addr + (uint32_t)x);
                    uint32_t c   = s->tex_palette[tmu][idx];
                    uint8_t  pr  = (c >> 16) & 0xff;
                    uint8_t  pg  = (c >>  8) & 0xff;
                    uint8_t  pb  =  c        & 0xff;
                    uint8_t  r2  = (uint8_t)(((pr & 3) << 6) | ((pg & 0xf0) >> 2) | (pr & 3));
                    uint8_t  g2  = (uint8_t)(((pg & 0xf) << 4) | ((pb & 0xc0) >> 4) | ((pg & 0xf) >> 2));
                    uint8_t  b2  = (uint8_t)(((pb & 0x3f) << 2) | ((pb & 0x30) >> 4));
                    uint8_t  a2  = (uint8_t)((pr & 0xfc) | ((pr & 0xc0) >> 6));
                    out = MAKERGBA(r2, g2, b2, a2);
                    break; }
                case TEX_ARGB8332: {
                    uint16_t d;
                    d = T16(row_addr + (uint32_t)(x * 2));
                    out = MAKERGBA(rgb332_r[d & 0xff], rgb332_g[d & 0xff],
                                   rgb332_b[d & 0xff], d >> 8);
                    break; }
                case TEX_A8Y4I2Q2: {
                    uint16_t d;
                    d = T16(row_addr + (uint32_t)(x * 2));
                    int      sel = (tp->textureMode & TEXTUREMODE_NCC_SEL) ? 1 : 0;
                    uint32_t c   = s->ncc_lookup[tmu][sel][d & 0xff];
                    out = (c & 0x00ffffffu) | ((uint32_t)(d >> 8) << 24);
                    break; }
                case TEX_R5G6B5: {
                    uint16_t d;
                    d = T16(row_addr + (uint32_t)(x * 2));
                    out = MAKERGBA(rgb565_r[d], rgb565_g[d], rgb565_b[d], 0xff);
                    break; }
                case TEX_ARGB1555: {
                    uint16_t d;
                    d = T16(row_addr + (uint32_t)(x * 2));
                    out = MAKERGBA(a1555_r[d], a1555_g[d], a1555_b[d], a1555_a[d]);
                    break; }
                case TEX_ARGB4444: {
                    uint16_t d;
                    d = T16(row_addr + (uint32_t)(x * 2));
                    out = MAKERGBA(a4444_r[d], a4444_g[d], a4444_b[d], a4444_a[d]);
                    break; }
                case TEX_A8I8: {
                    uint16_t d;
                    d = T16(row_addr + (uint32_t)(x * 2));
                    out = MAKERGBA(d & 0xff, d & 0xff, d & 0xff, d >> 8);
                    break; }
                case TEX_APAL88: {
                    uint16_t d;
                    d = T16(row_addr + (uint32_t)(x * 2));
                    uint32_t c   = s->tex_palette[tmu][d & 0xff];
                    out = MAKERGBA((c >> 16) & 0xff, (c >> 8) & 0xff,
                                    c & 0xff, d >> 8);
                    break; }
                /*
                 * TEX_ARGB_8888 (tformat=15) — 32-bit ARGB, 4 bytes per texel.
                 * GR_TEXFMT_ARGB_8888 in the Glide3 / Voodoo3 register spec.
                 * Memory layout: [A][R][G][B] little-endian (BGRA in byte order).
                 * 86Box does not implement this format (fatal() on unknown format);
                 * it is used by Warp3D/AmigaOS4 drivers for high-quality textures.
                 * Ported from the SST-1 hardware spec and Glide3 source.
                 */
                case TEX_ARGB_8888: {
                    uint32_t d;
                    d = T32(row_addr + (uint32_t)(x * 4));
                    /* Wire layout: bits[31:24]=A, [23:16]=R, [15:8]=G, [7:0]=B */
                    out = MAKERGBA((d >> 16) & 0xff,   /* R */
                                   (d >>  8) & 0xff,   /* G */
                                    d        & 0xff,   /* B */
                                   (d >> 24) & 0xff);  /* A */
                    break; }
                default:
                    out = 0xff808080u;
                    break;
                }
                dst_row[x] = out;
            }
        }
    }
#undef TB
#undef T16
#undef T32
}




/* =========================================================================
 * Source-content hash of the SGRAM bytes a cache entry was decoded from.
 * Cheap enough to run once per cache entry and epoch (LOD range only).
 * ========================================================================= */
static uint64_t tex_hash_bytes(const uint8_t *mem, uint32_t mem_mask,
                               uint32_t start, uint32_t len, uint64_t h)
{
    const uint64_t K = 0x9E3779B97F4A7C15ull;
    uint32_t size = mem_mask + 1;

    start &= mem_mask;
    while (len) {
        uint32_t n = size - start;      /* bytes until the end of SGRAM */
        if (n > len) {
            n = len;
        }
        const uint8_t *p = mem + start;
        uint32_t i = 0;
        for (; i + 8 <= n; i += 8) {
            h = (h ^ ldq_le_p(p + i)) * K;
            h ^= h >> 29;
        }
        for (; i < n; i++) {
            h = (h ^ p[i]) * K;
            h ^= h >> 29;
        }
        len  -= n;
        start = (start + n) & mem_mask;
    }
    return h;
}

static uint64_t tex_source_hash(Voodoo3State *s, const voodoo3_tex_params_t *tp,
                                int tmu, int lod_min, int lod_max)
{
    uint64_t h = 0xcbf29ce484222325ull;

    for (int lod = lod_min; lod <= lod_max; lod++) {
        uint32_t b   = tp->tex_base[lod];
        uint32_t len = tp->tex_end[lod] - tp->tex_base[lod];
        h = tex_hash_bytes(s->tex_mem[tmu], s->tex_mask, b, len, h);
        h ^= (uint64_t)lod << 56;
    }
    return h;
}

/* =========================================================================
 * Dirty-page tracking for the texture cache
 *
 * Banshee/V3 textures live in the shared SGRAM, so any write to SGRAM can
 * change a cached texture.  Writers only set a bit per 4 KiB page (cheap,
 * lock-free); the producer that looks up textures consumes the bitmap and
 * invalidates overlapping cache entries (86Box: texture_present[] +
 * flush_texture_cache()).
 * ========================================================================= */
void voodoo3_vram_mark_dirty(Voodoo3State *s, uint32_t addr, uint32_t len)
{
    if (!len || !s->tex_dirty_pages) {
        return;
    }
    uint32_t first = (addr & s->tex_mask) >> V3_VRAM_PAGE_SHIFT;
    uint64_t last64 = ((uint64_t)(addr & s->tex_mask) + len - 1)
                      >> V3_VRAM_PAGE_SHIFT;
    uint32_t npages = s->tex_mem_size >> V3_VRAM_PAGE_SHIFT;
    uint32_t last = last64 >= npages ? npages - 1 : (uint32_t)last64;

    for (uint32_t pg = first; pg <= last; pg++) {
        if (!test_bit(pg, s->tex_dirty_pages)) {
            set_bit_atomic(pg, s->tex_dirty_pages);
        }
        if (!test_bit(pg, s->disp_dirty_pages)) {
            set_bit_atomic(pg, s->disp_dirty_pages);
        }
    }
    if (!qatomic_read(&s->tex_dirty_any)) {
        qatomic_set(&s->tex_dirty_any, true);
    }
    if (!qatomic_read(&s->disp_dirty_any)) {
        qatomic_set(&s->disp_dirty_any, true);
    }
}

void voodoo3_flush_tex_if_dirty(Voodoo3State *s, uint32_t addr_fb)
{
    voodoo3_vram_mark_dirty(s, addr_fb, 4);
}

void voodoo3_tex_cache_flush_all(Voodoo3State *s)
{
    for (int tmu = 0; tmu < 2; tmu++) {
        for (int c = 0; c < V3_TEX_CACHE_SIZE; c++) {
            qatomic_set(&s->tex_cache[tmu][c].valid, false);
        }
    }
}

static bool tex_entry_overlaps(const v3_tex_cache_entry_t *e,
                               const unsigned long *pages, uint32_t npages)
{
    uint32_t first = e->addr_start >> V3_VRAM_PAGE_SHIFT;
    uint32_t last  = (e->addr_end ? e->addr_end - 1 : 0) >> V3_VRAM_PAGE_SHIFT;

    if (last < first) {            /* wrapped around the end of SGRAM */
        return find_next_bit(pages, npages, first) < npages ||
               find_next_bit(pages, last + 1, 0) <= last;
    }
    if (last >= npages) {
        last = npages - 1;
    }
    return find_next_bit(pages, last + 1, first) <= last;
}

static void tex_process_dirty(Voodoo3State *s)
{
    uint32_t npages = s->tex_mem_size >> V3_VRAM_PAGE_SHIFT;
    unsigned long snap[BITS_TO_LONGS(V3_VRAM_PAGES)];

    if (!qatomic_read(&s->tex_dirty_any)) {
        return;
    }
    qatomic_set(&s->tex_dirty_any, false);
    smp_mb();
    bitmap_copy_and_clear_atomic(snap, s->tex_dirty_pages, npages);

    for (int tmu = 0; tmu < 2; tmu++) {
        for (int c = 0; c < V3_TEX_CACHE_SIZE; c++) {
            v3_tex_cache_entry_t *e = &s->tex_cache[tmu][c];
            if (e->valid && tex_entry_overlaps(e, snap, npages)) {
                qatomic_set(&e->valid, false);
            }
        }
    }
}

/* True when no queued/in-flight triangle still samples from this entry. */
static bool tex_entry_idle(Voodoo3State *s, const v3_tex_cache_entry_t *e)
{
    for (uint32_t t = 0; t < s->render_threads_count; t++) {
        if (qatomic_load_acquire(&e->refcount_r[t]) != e->refcount) {
            return false;
        }
    }
    return true;
}

static void tex_wire(voodoo3_params_t *p, int tmu, int slot,
                     v3_tex_cache_entry_t *e, int lod_min, int lod_max)
{
    for (int lod = 0; lod <= V3_LOD_MAX; lod++) {
        int ul = lod < lod_min ? lod_min : (lod > lod_max ? lod_max : lod);
        p->tex_ptr[tmu][lod] = &e->data[texture_offset[ul]];
    }
    /* One reference per queued triangle; released by every render thread */
    e->refcount++;
    p->tex_slot[tmu] = (int8_t)slot;
}

/* =========================================================================
 * voodoo3_use_texture — look up or decode a texture; wire pointers
 *
 * Called from voodoo3_queue_triangle() (under queue_lock) before the
 * triangle is pushed to the param ring.
 * ========================================================================= */
void voodoo3_use_texture(Voodoo3State *s, voodoo3_params_t *p, int tmu)
{
    voodoo3_tex_params_t *tp = &p->tex_params[tmu];

    tex_process_dirty(s);

    int lod_min = (int)((tp->tLOD >> 2)  & 0xf);
    int lod_max = (int)((tp->tLOD >> 8)  & 0xf);
    if (lod_min > V3_LOD_MAX) lod_min = V3_LOD_MAX;
    if (lod_max > V3_LOD_MAX) lod_max = V3_LOD_MAX;
    if (lod_max < lod_min) lod_max = lod_min;

    uint32_t cache_addr = tp->base;
    /* LOD range/aspect plus odd/split/multibase layout bits (18..24) */
    uint32_t cache_lod  = tp->tLOD & 0x01fc0fffu;
    /* Texel format (bits 11:8) and NCC table select (bit 5) change decode */
    uint32_t cache_mode = tp->textureMode & ((0xfu << 8) | TEXTUREMODE_NCC_SEL);

    bool is_ncc = (tp->tformat == TEX_Y4I2Q2 || tp->tformat == TEX_A8Y4I2Q2);
    bool is_pal = (tp->tformat == TEX_PAL8 || tp->tformat == TEX_APAL8 ||
                   tp->tformat == TEX_APAL88);
    uint32_t cur_ncc_gen = is_ncc ? s->ncc_gen[tmu] : 0u;
    uint32_t cur_pal_gen = is_pal ? s->pal_gen[tmu] : 0u;

    /*
     * Epoch is read BEFORE hashing the source so a guest write that lands
     * while we verify forces another verification next time.
     */
    const uint32_t cur_epoch = qatomic_read(&s->tex_epoch);
    const uint32_t cur_sx    = v3_lfb_x(s);

    /* Search cache for a valid matching entry */
    for (int c = 0; c < V3_TEX_CACHE_SIZE; c++) {
        v3_tex_cache_entry_t *e = &s->tex_cache[tmu][c];
        if (e->valid && e->base == cache_addr && e->tLOD == cache_lod
                && e->textureMode == cache_mode && e->sx == cur_sx
                && e->ncc_gen == cur_ncc_gen && e->pal_gen == cur_pal_gen) {
            if (e->verified_epoch != cur_epoch) {
                uint64_t hnow = tex_source_hash(s, tp, tmu, lod_min, lod_max);
                if (hnow != e->src_hash) {
                    /* SGRAM changed under the cached texture without the
                     * dirty log having reported it yet: drop and re-decode */
                    static int stale_log_left = 200;
                    if (stale_log_left > 0 && qemu_loglevel_mask(LOG_UNIMP)) {
                        stale_log_left--;
                        qemu_log_mask(LOG_UNIMP,
                            "v3dbg: TEXSTALE tmu=%d base=0x%x fmt=%u "
                            "tLOD=0x%08x hash %016" PRIx64 " -> %016" PRIx64 "\n",
                            tmu, (unsigned)cache_addr, (unsigned)tp->tformat,
                            (unsigned)tp->tLOD, e->src_hash, hnow);
                    }
                    qatomic_set(&e->valid, false);
                    continue;
                }
                e->verified_epoch = cur_epoch;
            }
            tex_wire(p, tmu, c, e, lod_min, lod_max);
            return;
        }
    }

    /*
     * Cache miss — recycle the next slot (round-robin like 86Box) that no
     * render thread is still sampling from.  If every slot is busy, wait
     * for the render threads to drain (86Box
     * voodoo_wait_for_render_thread_idle()).
     */
    int slot = -1;
    for (int tries = 0; tries < V3_TEX_CACHE_SIZE; tries++) {
        int cand = (int)(s->tex_lru[tmu]++ & (V3_TEX_CACHE_SIZE - 1));
        if (tex_entry_idle(s, &s->tex_cache[tmu][cand])) {
            slot = cand;
            break;
        }
    }
    if (slot < 0) {
        voodoo3_wait_render_idle(s);
        slot = (int)(s->tex_lru[tmu]++ & (V3_TEX_CACHE_SIZE - 1));
    }
    v3_tex_cache_entry_t *e = &s->tex_cache[tmu][slot];

    if (cache_addr == 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
            "voodoo3: use_texture tmu=%d: base=0 — texture not uploaded yet "
            "(tLOD=0x%08x tformat=%u); rendering may produce garbage\n",
            tmu, tp->tLOD, tp->tformat);
    }

    e->valid       = false;
    e->sx          = cur_sx;
    e->verified_epoch = cur_epoch;
    e->src_hash    = tex_source_hash(s, tp, tmu, lod_min, lod_max);
    e->base        = cache_addr;
    e->tLOD        = cache_lod;
    e->textureMode = cache_mode;
    e->ncc_gen     = cur_ncc_gen;
    e->pal_gen     = cur_pal_gen;

    /* SGRAM range covered by the decoded LODs (for dirty-page invalidation) */
    uint32_t start = tp->tex_base[lod_min] & s->tex_mask;
    uint32_t end   = tp->tex_end[lod_min] & s->tex_mask;
    for (int lod = lod_min + 1; lod <= lod_max; lod++) {
        uint32_t b = tp->tex_base[lod] & s->tex_mask;
        uint32_t en = tp->tex_end[lod] & s->tex_mask;
        if (b < start) start = b;
        if (en > end) end = en;
    }
    e->addr_start = start;
    e->addr_end   = end;

    decode_texture(s, e, tp, tmu, lod_min, lod_max);
    e->valid = true;

    {
        /* Diagnostic (-d unimp): what did the decoder produce? */
        static int tex_log_left = 400;
        static uint64_t last_logged_hash;
        static uint32_t last_logged_base = ~0u;
        /* identical re-decodes (same base + same source) used up the old
         * 120-line budget within the first frames; log changes only */
        bool log_it = !(e->src_hash == last_logged_hash &&
                        tp->base == last_logged_base);
        if (tex_log_left > 0 && log_it && qemu_loglevel_mask(LOG_UNIMP)) {
            tex_log_left--;
            last_logged_hash = e->src_hash;
            last_logged_base = tp->base;
            const uint32_t *d = &e->data[texture_offset[lod_min]];
            int w = tp->tex_w_mask[lod_min] + 1;
            int h = tp->tex_h_mask[lod_min] + 1;
            int nz = 0, nz_rows = 0;
            for (int y = 0; y < h; y++) {
                int row_nz = 0;
                for (int x = 0; x < w; x++) {
                    if (d[y * w + x] & 0x00ffffffu) { nz++; row_nz++; }
                }
                if (row_nz) { nz_rows++; }
            }
            qemu_log_mask(LOG_UNIMP,
                "v3dbg: TEXDEC tmu=%d fmt=%u base=0x%x %dx%d lod=%d..%d "
                "tLOD=0x%08x sx=%u nonzero=%d/%d rows_nz=%d/%d "
                "t0=%08x t1=%08x t2=%08x t3=%08x src=%016" PRIx64 "\n",
                tmu, (unsigned)tp->tformat, (unsigned)tp->base, w, h, lod_min, lod_max,
                (unsigned)tp->tLOD, (unsigned)v3_lfb_x(s), nz, w * h, nz_rows, h,
                d[0], d[1], d[2], d[3], e->src_hash);
        }
    }

    tex_wire(p, tmu, slot, e, lod_min, lod_max);
}

/* =========================================================================
 * voodoo3_tex_download — handle one texture-aperture write (BAR0 0x600000+)
 *
 * Ported from 86Box voodoo_tex_writel() — Banshee/V3 linear path.
 * For Voodoo 3 the address is: (addr & 0x1ffffc) + tex_base[tmu][0]
 * Executed synchronously in the vCPU thread.  Because Banshee/V3 texture
 * memory is the shared SGRAM, the store goes straight into fb_mem and the
 * affected page is marked dirty; the cache is invalidated lazily.
 * ========================================================================= */
void voodoo3_tex_download(Voodoo3State *s, uint32_t fifo_addr, uint32_t val,
                          int tmu)
{
    if (tmu < 0 || tmu > 1) return;

    uint32_t tex_base0 = s->params.tex_params[tmu].tex_base[0];
    uint32_t addr      = ((fifo_addr & 0x1ffffc) + tex_base0) & s->tex_mask;
    uint32_t waddr     = addr & ~3u;

    if (waddr + 4 <= s->tex_mem_size) {
        uint32_t sx = v3_lfb_x(s);
        for (int k = 0; k < 4; k++) {
            s->tex_mem[tmu][(waddr + k) ^ sx] = (uint8_t)(val >> (8 * k));
        }
        voodoo3_vram_mark_dirty(s, waddr, 4);
    }
}
