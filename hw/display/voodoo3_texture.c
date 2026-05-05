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

/* tLOD bit fields */
#define LOD_S_IS_WIDER      (1 << 20)
#define LOD_SPLIT           (1 << 23)
#define LOD_ODD             (1 << 24)
#define LOD_TMULTIBASEADDR  (1 << 25)
#define LOD_TRILINEAR       (1 << 26)   /* = TEXTUREMODE_TRILINEAR */
#define LOD_TMIRROR_S       (1 << 17)
#define LOD_TMIRROR_T       (1 << 18)

#define TEXTUREMODE_TRILINEAR (1 << 2)
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
            if (tformat & 8)
                offset += (uint32_t)(w * h * 2);
            else
                offset += (uint32_t)(w * h);
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
        if (tformat & 8)
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

        for (int y = 0; y < h; y++) {
            uint32_t  row_addr = tex_addr + (uint32_t)(y << src_shift);
            uint32_t *dst_row  = base + (uint32_t)(y * w);
            for (int x = 0; x < w; x++) {
                uint32_t out;
                switch (tformat) {
                case TEX_RGB332: {
                    uint8_t d = tex_mem[(row_addr + (uint32_t)x) & tex_mask];
                    out = MAKERGBA(rgb332_r[d], rgb332_g[d], rgb332_b[d], 0xff);
                    break; }
                case TEX_Y4I2Q2: {
                    uint8_t  d   = tex_mem[(row_addr + (uint32_t)x) & tex_mask];
                    int      sel = (tp->textureMode & TEXTUREMODE_NCC_SEL) ? 1 : 0;
                    uint32_t c   = s->ncc_lookup[tmu][sel][d];
                    out = c;
                    break; }
                case TEX_A8: {
                    uint8_t d = tex_mem[(row_addr + (uint32_t)x) & tex_mask];
                    out = MAKERGBA(d, d, d, d);
                    break; }
                case TEX_I8: {
                    uint8_t d = tex_mem[(row_addr + (uint32_t)x) & tex_mask];
                    out = MAKERGBA(d, d, d, 0xff);
                    break; }
                case TEX_AI8: {
                    uint8_t d  = tex_mem[(row_addr + (uint32_t)x) & tex_mask];
                    uint8_t lo = (uint8_t)((d & 0x0f) | ((d & 0x0f) << 4));
                    uint8_t hi = (uint8_t)((d & 0xf0) | ((d & 0xf0) >> 4));
                    out = MAKERGBA(lo, lo, lo, hi);
                    break; }
                case TEX_PAL8: {
                    uint8_t  idx = tex_mem[(row_addr + (uint32_t)x) & tex_mask];
                    uint32_t c   = s->tex_palette[tmu][idx];
                    out = MAKERGBA((c >> 16) & 0xff, (c >> 8) & 0xff,
                                    c & 0xff, 0xff);
                    break; }
                case TEX_APAL8: {
                    uint8_t  idx = tex_mem[(row_addr + (uint32_t)x) & tex_mask];
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
                    memcpy(&d, &tex_mem[(row_addr + (uint32_t)(x * 2)) & tex_mask], 2);
                    out = MAKERGBA(rgb332_r[d & 0xff], rgb332_g[d & 0xff],
                                   rgb332_b[d & 0xff], d >> 8);
                    break; }
                case TEX_A8Y4I2Q2: {
                    uint16_t d;
                    memcpy(&d, &tex_mem[(row_addr + (uint32_t)(x * 2)) & tex_mask], 2);
                    int      sel = (tp->textureMode & TEXTUREMODE_NCC_SEL) ? 1 : 0;
                    uint32_t c   = s->ncc_lookup[tmu][sel][d & 0xff];
                    out = (c & 0x00ffffffu) | ((uint32_t)(d >> 8) << 24);
                    break; }
                case TEX_R5G6B5: {
                    uint16_t d;
                    memcpy(&d, &tex_mem[(row_addr + (uint32_t)(x * 2)) & tex_mask], 2);
                    out = MAKERGBA(rgb565_r[d], rgb565_g[d], rgb565_b[d], 0xff);
                    break; }
                case TEX_ARGB1555: {
                    uint16_t d;
                    memcpy(&d, &tex_mem[(row_addr + (uint32_t)(x * 2)) & tex_mask], 2);
                    out = MAKERGBA(a1555_r[d], a1555_g[d], a1555_b[d], a1555_a[d]);
                    break; }
                case TEX_ARGB4444: {
                    uint16_t d;
                    memcpy(&d, &tex_mem[(row_addr + (uint32_t)(x * 2)) & tex_mask], 2);
                    out = MAKERGBA(a4444_r[d], a4444_g[d], a4444_b[d], a4444_a[d]);
                    break; }
                case TEX_A8I8: {
                    uint16_t d;
                    memcpy(&d, &tex_mem[(row_addr + (uint32_t)(x * 2)) & tex_mask], 2);
                    out = MAKERGBA(d & 0xff, d & 0xff, d & 0xff, d >> 8);
                    break; }
                case TEX_APAL88: {
                    uint16_t d;
                    memcpy(&d, &tex_mem[(row_addr + (uint32_t)(x * 2)) & tex_mask], 2);
                    uint32_t c   = s->tex_palette[tmu][d & 0xff];
                    out = MAKERGBA((c >> 16) & 0xff, (c >> 8) & 0xff,
                                    c & 0xff, d >> 8);
                    break; }
                default:
                    out = 0xff808080u;
                    break;
                }
                dst_row[x] = out;
            }
        }
    }
}

/* =========================================================================
 * voodoo3_use_texture — look up or decode a texture; wire pointers
 *
 * Called from voodoo3_queue_triangle() before pushing to param ring.
 * Sets tp->decoded_data[tmu][lod] so the rasterizer can read texels.
 * ========================================================================= */
void voodoo3_use_texture(Voodoo3State *s, voodoo3_params_t *p, int tmu)
{
    voodoo3_tex_params_t *tp = &p->tex_params[tmu];

    int lod_min = (int)((tp->tLOD >> 2)  & 0xf);
    int lod_max = (int)((tp->tLOD >> 8)  & 0xf);
    if (lod_min > V3_LOD_MAX) lod_min = V3_LOD_MAX;
    if (lod_max > V3_LOD_MAX) lod_max = V3_LOD_MAX;

    uint32_t cache_addr = tp->base;
    uint32_t cache_lod  = tp->tLOD & 0xf00fffu;

    /*
     * For NCC-format textures (TEX_Y4I2Q2, TEX_A8Y4I2Q2) the decoded pixel
     * data depends on the nccTable registers, not just the raw SGRAM bytes.
     * We track the current generation counter so that any nccTable write
     * (which bumps ncc_gen[tmu]) forces a cache miss and re-decode.
     * For non-NCC formats ncc_gen is always 0 in the cache entry (stored as
     * 0) and s->ncc_gen is ignored, so there is no overhead.
     */
    bool is_ncc = (tp->tformat == TEX_Y4I2Q2 || tp->tformat == TEX_A8Y4I2Q2);
    uint32_t cur_ncc_gen = is_ncc ? s->ncc_gen[tmu] : 0u;

    /* Search cache for a valid matching entry */
    for (int c = 0; c < V3_TEX_CACHE_SIZE; c++) {
        v3_tex_cache_entry_t *e = &s->tex_cache[tmu][c];
        if (e->valid && e->base == cache_addr && e->tLOD == cache_lod
                && e->ncc_gen == cur_ncc_gen) {
            /* Hit — wire per-LOD pointers via texture_offset[], clamped */
            for (int lod = 0; lod <= V3_LOD_MAX; lod++) {
                int ul = lod < lod_min ? lod_min
                       : (lod > lod_max ? lod_max : lod);
                p->tex_ptr[tmu][lod] = &e->data[texture_offset[ul]];
            }
            return;
        }
    }

    /* Cache miss — evict LRU slot (round-robin, same as 86Box) */
    int slot = (int)(s->tex_lru[tmu] & (V3_TEX_CACHE_SIZE - 1));
    s->tex_lru[tmu]++;
    v3_tex_cache_entry_t *e = &s->tex_cache[tmu][slot];

    /* Warn if the base address looks uninitialized (driver hasn't uploaded texture yet) */
    if (cache_addr == 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
            "voodoo3: use_texture tmu=%d: base=0 — texture not uploaded yet "
            "(tLOD=0x%08x tformat=%u); rendering may produce garbage\n",
            tmu, tp->tLOD, tp->tformat);
    } else {
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: tex cache miss tmu=%d slot=%d base=0x%08x "
            "tLOD=0x%06x tformat=%u lod_min=%d lod_max=%d ncc=%d\n",
            tmu, slot, cache_addr, cache_lod, tp->tformat,
            lod_min, lod_max, is_ncc);
    }

    e->valid   = true;
    e->base    = cache_addr;
    e->tLOD    = cache_lod;
    e->ncc_gen = cur_ncc_gen;

    decode_texture(s, e, tp, tmu, lod_min, lod_max);

    /* Wire rasterizer pointers via texture_offset[] */
    for (int lod = 0; lod <= V3_LOD_MAX; lod++) {
        int ul = lod < lod_min ? lod_min : (lod > lod_max ? lod_max : lod);
        p->tex_ptr[tmu][lod] = &e->data[texture_offset[ul]];
    }
}

/* =========================================================================
 * voodoo3_tex_download — handle one FIFO_WRITEL_TEX write
 *
 * Ported from 86Box voodoo_tex_writel() — Banshee/V3 linear path.
 * For Voodoo 3 the address is: (fifo_addr & 0x1ffffc) + tex_base[tmu][0]
 * ========================================================================= */
void voodoo3_tex_download(Voodoo3State *s, uint32_t fifo_addr, uint32_t val,
                          int tmu)
{
    if (tmu < 0 || tmu > 1) return;

    /* Banshee/V3 linear addressing (from voodoo_tex_writel Banshee branch) */
    uint32_t tex_base0 = s->params.tex_params[tmu].tex_base[0];
    uint32_t addr      = (fifo_addr & 0x1ffffc) + tex_base0;
    addr              &= s->tex_mask;

    /* Invalidate any cached texture that overlaps this address */
    for (int c = 0; c < V3_TEX_CACHE_SIZE; c++) {
        v3_tex_cache_entry_t *e = &s->tex_cache[tmu][c];
        if (e->valid) {
            /* Simple range check against LOD 0 */
            uint32_t start = s->params.tex_params[tmu].tex_base[0] & s->tex_mask;
            uint32_t end   = s->params.tex_params[tmu].tex_end [0] & s->tex_mask;
            uint32_t masked = addr & ~0x3ffu;
            if (masked >= (start & ~0x3ffu) && masked <= ((end + 0x3ff) & ~0x3ffu)) {
                e->valid = false;
            }
        }
        /* Also check the other TMU for Voodoo 3 shared SGRAM */
        v3_tex_cache_entry_t *e2 = &s->tex_cache[tmu ^ 1][c];
        if (e2->valid) {
            uint32_t start2 = s->params.tex_params[tmu ^ 1].tex_base[0] & s->tex_mask;
            uint32_t end2   = s->params.tex_params[tmu ^ 1].tex_end [0] & s->tex_mask;
            uint32_t masked = addr & ~0x3ffu;
            if (masked >= (start2 & ~0x3ffu) && masked <= ((end2 + 0x3ff) & ~0x3ffu)) {
                e2->valid = false;
            }
        }
    }

    /* Write 4 bytes to texture RAM */
    uint32_t waddr = addr & ~3u;
    if (waddr + 4 <= s->tex_mem_size) {
        memcpy(s->tex_mem[tmu] + waddr, &val, 4);
    }
}
