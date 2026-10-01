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
 *
 * 86Box uses TEX_CACHE_MAX=64 per TMU.  We use 32 entries per TMU.
 * Each entry holds the decoded ABGR32 texels of the complete mip chain
 * (256x256 + 128x128 + ... + 1x1 = 87381 words, ~342 KiB).  The previous
 * layout reserved a full 256x256 slab for every LOD (~2.3 MiB per entry,
 * ~75 MiB for the whole cache) although LOD n only needs 1/4^n of it.
 * ------------------------------------------------------------------------- */
#define V3_TEX_CACHE_SIZE   32
#define V3_TEX_LEVEL_WORDS  (256 * 256)   /* max texels at LOD 0 */
#define V3_TEX_CACHE_WORDS  (256 * 256 + 128 * 128 + 64 * 64 + 32 * 32 + \
                             16 * 16 + 8 * 8 + 4 * 4 + 2 * 2 + 1 + 1)

/*
 * Banshee / Voodoo3 have no separate texture RAM: the TMUs fetch texels
 * from the same SGRAM that holds the frame buffers (86Box points
 * voodoo->tex_mem[] at the Banshee VRAM for the same reason).  The TMU
 * address space therefore equals the frame-buffer size.
 */
#define V3_TEX_MEM_SIZE     (16 * 1024 * 1024)
#define V3_TEX_MASK         (V3_TEX_MEM_SIZE - 1)

/* Dirty-page granularity used for texture-cache invalidation */
#define V3_VRAM_PAGE_SHIFT  12
#define V3_VRAM_PAGES       (V3_TEX_MEM_SIZE >> V3_VRAM_PAGE_SHIFT)

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
    uint32_t textureMode;                   /* textureMode register snapshot      */
    int      tformat;                       /* TEX_* constant                     */
    int      width;                         /* LOD-0 width                        */
} voodoo3_tex_params_t;

/* -------------------------------------------------------------------------
 * Decoded texture cache entry
 * data[] holds ABGR32 words, laid out as V3_LOD_MAX+1 slabs of
 * V3_TEX_LEVEL_WORDS each:  data[lod * V3_TEX_LEVEL_WORDS + y*w + x]
 * ------------------------------------------------------------------------- */
typedef struct {
    bool     valid;
    uint32_t base;     /* texBaseAddr used when decoded */
    uint32_t tLOD;     /* tLOD & 0xf00fff              */
    uint32_t textureMode; /* textureMode & 0xfff (format + NCC select) */
    uint32_t ncc_gen;  /* s->ncc_gen[tmu] at decode time — NCC-table change detection */
    uint32_t pal_gen;  /* s->pal_gen[tmu] at decode time — palette change detection */
    uint32_t addr_start, addr_end; /* SGRAM byte range covered by all LODs */
    /*
     * Reference counting (86Box refcount / refcount_r[]):
     * refcount is bumped by the producer each time a queued triangle uses
     * this entry; each render thread bumps its own refcount_r[] when it
     * has finished that triangle.  A slot may only be recycled when every
     * refcount_r[t] has caught up with refcount.
     */
    uint32_t refcount;
    uint32_t refcount_r[4];
    uint32_t data[V3_TEX_CACHE_WORDS]; /* flat decoded texel array (mip chain) */
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

/*
 * Invalidate texture-cache entries that overlap a direct VRAM write address.
 * Kept for the CMDFIFO packet-5 callers; equivalent to marking the page dirty.
 */
void voodoo3_flush_tex_if_dirty(Voodoo3State *s, uint32_t addr_fb);

/*
 * Record that [addr, addr+len) of SGRAM was modified by something other than
 * the texture aperture (CPU LFB writes, 2D blits, CMDFIFO packet 5, ...).
 * Thread-safe; the texture cache is invalidated lazily on next use.
 */
void voodoo3_vram_mark_dirty(Voodoo3State *s, uint32_t addr, uint32_t len);

/* Invalidate every cache entry of both TMUs (reset / snapshot load). */
void voodoo3_tex_cache_flush_all(Voodoo3State *s);

#endif /* HW_DISPLAY_VOODOO3_TEXTURE_H */
