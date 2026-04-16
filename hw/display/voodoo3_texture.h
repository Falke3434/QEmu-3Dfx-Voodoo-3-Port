/*
 * QEMU 3Dfx Voodoo 3 — Texture Subsystem Header
 *
 * Copyright (C) 2026 <your name here>
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_VOODOO3_TEXTURE_H
#define HW_DISPLAY_VOODOO3_TEXTURE_H

#include <stdint.h>
#include <stdbool.h>
#ifndef V3_LOD_MAX
#define V3_LOD_MAX  8
#endif

/* Forward declarations (full definitions in voodoo3_int.h) */
typedef struct voodoo3_params_t voodoo3_params_t;
typedef struct Voodoo3State Voodoo3State;

/* -------------------------------------------------------------------------
 * Texture cache sizing
 * 86Box uses TEX_CACHE_MAX=32 per TMU.  We start with 16.
 * Each entry holds decoded ABGR32 texels for all LODs.
 * Per-LOD words: 256×256 = 65536 words at LOD 0, total mip chain ≈ 87K.
 * We allocate V3_TEX_LEVEL_WORDS per LOD level.
 * ------------------------------------------------------------------------- */
#define V3_TEX_CACHE_SIZE   16
#define V3_TEX_LEVEL_WORDS  (256 * 256)   /* max texels per mip level (LOD 0) */
#define V3_TEX_DATA_WORDS   (V3_TEX_LEVEL_WORDS * (V3_LOD_MAX + 1))

/* Texture RAM per TMU: 4 MB (Voodoo 3 has 8 MB shared, split 4+4) */
#define V3_TEX_MEM_SIZE     (4 * 1024 * 1024)
#define V3_TEX_MASK         (V3_TEX_MEM_SIZE - 1)

/* -------------------------------------------------------------------------
 * Per-LOD texture geometry (mirrors 86Box voodoo_params_t tex_* arrays)
 * ------------------------------------------------------------------------- */
typedef struct {
    uint32_t tex_base  [V3_LOD_MAX + 2];   /* byte offset of each LOD in SGRAM  */
    uint32_t tex_end   [V3_LOD_MAX + 2];   /* last byte (exclusive)              */
    int      tex_w_mask[V3_LOD_MAX + 2];   /* width  - 1                         */
    int      tex_h_mask[V3_LOD_MAX + 2];   /* height - 1                         */
    int      tex_shift [V3_LOD_MAX + 2];   /* log2(width)                        */
    int      tex_lod   [V3_LOD_MAX + 2];   /* mip index                          */
    uint32_t base;                          /* texBaseAddr[tmu]                   */
    uint32_t tLOD;                          /* tLOD register snapshot             */
    int      tformat;                       /* TEX_* constant                     */
    int      width;                         /* LOD-0 width                        */
} voodoo3_tex_params_t;

/* -------------------------------------------------------------------------
 * Decoded texture cache entry
 * data[] holds ABGR32 words, laid out as V3_LOD_MAX+1 slabs of
 * V3_TEX_LEVEL_WORDS each:  data[lod * V3_TEX_LEVEL_WORDS + y*w + x]
 * ------------------------------------------------------------------------- */
#define V3_DATA_WORDS_TOTAL  (V3_TEX_LEVEL_WORDS * (V3_LOD_MAX + 1))

typedef struct {
    bool     valid;
    uint32_t base;     /* texBaseAddr used when decoded */
    uint32_t tLOD;     /* tLOD & 0xf00fff              */
    uint32_t data[V3_DATA_WORDS_TOTAL]; /* flat decoded texel array */
} v3_tex_cache_entry_t;

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

/* Recompute per-LOD geometry arrays from register values.
 * Called whenever tLOD / texBaseAddr / textureMode changes. */
void voodoo3_recalc_tex(voodoo3_tex_params_t *tp,
                        uint32_t tLOD, uint32_t textureMode,
                        uint32_t texBaseAddr,  uint32_t texBaseAddr1,
                        uint32_t texBaseAddr2, uint32_t texBaseAddr38,
                        int tformat);

/* Look up or decode a texture into the cache and wire p->tex_ptr[][]. */
void voodoo3_use_texture(Voodoo3State *s, voodoo3_params_t *p, int tmu);

/* Handle a FIFO_WRITEL_TEX write into texture RAM. */
void voodoo3_tex_download(Voodoo3State *s, uint32_t fifo_addr,
                          uint32_t val, int tmu);

#endif /* HW_DISPLAY_VOODOO3_TEXTURE_H */
