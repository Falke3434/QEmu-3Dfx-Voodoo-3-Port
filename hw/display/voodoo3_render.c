/*
 * QEMU 3Dfx Voodoo 3 — Pixel Rasterizer
 *
 * Ported from 86Box vid_voodoo_render.c
 * Original author: Sarah Walker <https://pcem-emulator.co.uk/>
 * Copyright (C) 2008-2024 Sarah Walker and 86Box contributors
 * Copyright (C) 2026 <your name here>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * -------------------------------------------------------------------------
 * What this file contains
 * -------------------------------------------------------------------------
 * voodoo3_triangle()       — full triangle rasterizer entry point
 *                             (ported from voodoo_triangle + voodoo_half_triangle)
 *
 * The following sub-systems are included:
 *  - Scanline edge-walk (dxAB / dxAC / dxBC sub-pixel exact)
 *  - Clipping (left/right/top/bottom, both clip rectangles)
 *  - Sub-pixel parameter correction (FBZ_PARAM_ADJUST)
 *  - Depth/W-buffer (Z-compare: LT / GT / LE / GE / EQ / NE / ALWAYS / NEVER)
 *  - Stipple patterns (both rotating and pattern modes)
 *  - fbzColorPath colour combine (all CC_MSELECT / CC_ADD modes)
 *  - Alpha test
 *  - Fog (linear, per-table, z-based)
 *  - Alpha blend (all src/dst factors)
 *  - 4×4 and 2×2 ordered dither (RGB565 output)
 *  - Pixel write-back to framebuffer (tiled and linear)
 *  - Depth/alpha write-back to aux buffer
 *  - Flat-shaded (no texture) path fully implemented
 *  - Perspective-correct texture fetch (voodoo_tmu_fetch)
 *  - Bilinear filtering (tex_read_4 / tex_read)
 *  - Dual-TMU colour blend (voodoo_tmu_fetch_and_blend)
 *
 * NOT yet ported (stubs only):
 *  - Texture cache (tex[] pointers filled by voodoo3_use_texture)
 *  - x86-64 / ARM64 JIT recompiler path (NO_CODEGEN forced)
 *  - SLI (single-GPU only)
 * -------------------------------------------------------------------------
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/display/voodoo3_int.h"
#include "hw/display/voodoo3_render.h"
#include "hw/display/voodoo3_texture.h"
#include "hw/display/voodoo3_display.h"

#include <math.h>   /* log2() for LOD calc */

/* =========================================================================
 * fbzColorPath / fbzMode / alphaMode / textureMode bit definitions
 * Kept local to this file — mirror the 86Box vid_voodoo_regs.h values.
 * ========================================================================= */

/* fbzColorPath */
#define FBZCP_CC_RGBSELECT(r)       ((r) & 3)
#define FBZCP_CC_ASELECT(r)         (((r) >> 2) & 3)
#define FBZCP_CC_LOCALSELECT(r)     (!!((r) & (1 << 4)))
#define FBZCP_CCA_LOCALSELECT(r)    (((r) >> 5) & 3)
#define FBZCP_CC_LOCALSELECT_OVR(r) (!!((r) & (1 << 7)))
#define FBZCP_CC_ZERO_OTHER(r)      (!!((r) & (1 << 8)))
#define FBZCP_CC_SUB_CLOCAL(r)      (!!((r) & (1 << 9)))
#define FBZCP_CC_MSELECT(r)         (((r) >> 10) & 7)
#define FBZCP_CC_REVERSE_BLEND(r)   (!!((r) & (1 << 13)))
#define FBZCP_CC_ADD(r)             (((r) >> 14) & 3)
#define FBZCP_CC_ADD_ALOCAL(r)      (!!((r) & (1 << 15)))
#define FBZCP_CC_INVERT_OUT(r)      (!!((r) & (1 << 16)))
#define FBZCP_CCA_ZERO_OTHER(r)     (!!((r) & (1 << 17)))
#define FBZCP_CCA_SUB_CLOCAL(r)     (!!((r) & (1 << 18)))
#define FBZCP_CCA_MSELECT(r)        (((r) >> 19) & 7)
#define FBZCP_CCA_REVERSE_BLEND(r)  (!!((r) & (1 << 22)))
#define FBZCP_CCA_ADD(r)            (((r) >> 23) & 3)
#define FBZCP_CCA_INVERT_OUT(r)     (!!((r) & (1 << 25)))
#define FBZCP_TEXTURE_ENABLED(r)    (!!((r) & (1 << 27)))
#define FBZCP_PARAM_ADJUST(r)       (!!((r) & (1 << 30)))

/* fbzMode */
#define FBZ_ENABLE_CLIPPING     (1 << 0)
#define FBZ_STIPPLE             (1 << 2)
#define FBZ_STIPPLE_PATT        (1 << 12)
#define FBZ_W_BUFFER            (1 << 3)
#define FBZ_DEPTH_ENABLE        (1 << 4)
#define FBZ_DEPTH_OP_SHIFT      5
#define FBZ_DEPTH_OP_MASK       (7 << FBZ_DEPTH_OP_SHIFT)
#define FBZ_DEPTH_BIAS          (1 << 16)
#define FBZ_DEPTH_SOURCE        (1 << 20)
#define FBZ_RGB_WMASK           (1 << 9)
#define FBZ_ALPHA_MASK          (1 << 13)
#define FBZ_DEPTH_WMASK         (1 << 10)
#define FBZ_DITHER              (1 << 8)
#define FBZ_DITHER_2X2          (1 << 11)
#define FBZ_ALPHA_ENABLE        (1 << 18)
#define FBZ_Y_ORIGIN            (1 << 17)
#define FBZ_CHROMAKEY           (1 << 1)    /* hardware fbzMode bit 1 */
#define FBZ_DITHER_SUB          (1 << 19)   /* hardware fbzMode bit 19: subtraction dither */
#define FBZ_PARAM_ADJUST        (1 << 30)   /* reuse bit from fbzColorPath */

/* alphaMode */
#define ALPHA_FUNC(r)   (((r) >> 1) & 7)
#define ALPHA_REF(r)    (((r) >> 24) & 0xff)
#define ALPHA_ENABLE    (1 << 0)
#define ALPHA_BLEND_EN  (1 << 4)
#define ALPHA_SRC_FUNC(r) (((r) >> 8)  & 0xf)
#define ALPHA_DST_FUNC(r) (((r) >> 12) & 0xf)

/* fogMode */
#define FOG_ENABLE      (1 << 0)
#define FOG_ADD         (1 << 1)
#define FOG_MULT        (1 << 2)
#define FOG_Z           (1 << 3)
#define FOG_ALPHA       (1 << 4)
#define FOG_CONSTANT    (1 << 5)
/* FOG_W: both Z and ALPHA bits set — use w>>32 directly (86Box: AFUNC_AOM_COLOR path) */
#define FOG_W           (FOG_Z | FOG_ALPHA)

/* textureMode */
#define TEXMODE_PERSP_CORR  (1 << 0)
#define TEXMODE_BILINEAR    (1 << 1)
/*
 * TEXMODE_TRILINEAR — enables trilinear mipmap blending.
 * 86Box: TEXTUREMODE_TRILINEAR = (1 << 30) in vid_voodoo_regs.h.
 * The original port had this wrong as (1 << 2); corrected here.
 * When set and lod is odd, the blend-direction flag (cc_rev_blend /
 * cca_rev_blend) is inverted for the multiplier step — see 86Box
 * voodoo_render.c lines 445 and 544.
 */
#define TEXMODE_TRILINEAR   (1u << 30)
#define TEXMODE_LOCAL_MASK  0x00643000
#define TEXMODE_LOCAL       0x00241000
#define TEXMODE_PASSTHROUGH 0x00
#define TEXMODE_TCLAMPS     (1 << 6)
#define TEXMODE_TCLAMPT     (1 << 7)
#define TEXMODE_TMIRROR_S   (1 << 17)
#define TEXMODE_TMIRROR_T   (1 << 18)

/* CC selectors (fbzColorPath bits [12:10]) — from 86Box TC_MSELECT_* enum */
#define CC_MSELECT_ZERO    0
#define CC_MSELECT_CLOCAL  1
#define CC_MSELECT_AOTHER  2
#define CC_MSELECT_ALOCAL  3
/*
 * CC_MSELECT_DETAIL (4) and CC_MSELECT_LOD_FRAC (5):
 * Added from 86Box TC_MSELECT_DETAIL / TC_MSELECT_LOD_FRAC.
 * The original port had CC_MSELECT_TEX=4 and CC_MSELECT_TEXRGB=5 at these
 * positions, which is wrong — Glide/hardware has no "tex-alpha" mselect in
 * the CC path at value 4; that slot is detail texture.
 * CC_MSELECT_TEX / CC_MSELECT_TEXRGB are kept as aliases for code paths
 * that were already using the wrong names so they still compile, but the
 * active switch cases use the correct names.
 */
#define CC_MSELECT_DETAIL    4
#define CC_MSELECT_LOD_FRAC  5
/* Legacy aliases (wrong encoding but kept so existing callers compile) */
#define CC_MSELECT_TEX     CC_MSELECT_DETAIL
#define CC_MSELECT_TEXRGB  CC_MSELECT_LOD_FRAC

/* CCA selectors (fbzColorPath bits [21:19]) — from 86Box TCA_MSELECT_* enum */
#define CCA_MSELECT_ZERO     0
#define CCA_MSELECT_ALOCAL   1
#define CCA_MSELECT_AOTHER   2
#define CCA_MSELECT_ALOCAL2  3
/*
 * CCA_MSELECT_DETAIL (4) and CCA_MSELECT_LOD_FRAC (5):
 * Added from 86Box TCA_MSELECT_DETAIL / TCA_MSELECT_LOD_FRAC.
 * The original port had CCA_MSELECT_TEX=4, which is wrong.
 */
#define CCA_MSELECT_DETAIL    4
#define CCA_MSELECT_LOD_FRAC  5
/* Legacy alias */
#define CCA_MSELECT_TEX  CCA_MSELECT_DETAIL

/* CC_ADD */
#define CC_ADD_ZERO   0
#define CC_ADD_CLOCAL 1
#define CC_ADD_ALOCAL 2

/* A_SEL */
#define A_SEL_ITER_A  0
#define A_SEL_TEX     1
#define A_SEL_COLOR1  2

/* CCA_LOCALSELECT */
#define CCA_LOCALSEL_ITER_A  0
#define CCA_LOCALSEL_COLOR0  1
#define CCA_LOCALSEL_ITER_Z  2

/* Depth ops */
#define DEPTH_OP_NEVER  0
#define DEPTH_OP_LT     1
#define DEPTH_OP_EQ     2
#define DEPTH_OP_LE     3
#define DEPTH_OP_GT     4
#define DEPTH_OP_NE     5
#define DEPTH_OP_GE     6
#define DEPTH_OP_ALWAYS 7

/* =========================================================================
 * Helper macros — identical semantics to 86Box
 * ========================================================================= */
/* glib defines CLAMP(x,low,high) with 3 args - override with our 1-arg version */
#ifdef CLAMP
#undef CLAMP
#endif
#define CLAMP(x)    ((x) < 0 ? 0 : ((x) > 255 ? 255 : (x)))
#define CLAMP16(x)  ((x) < 0 ? 0 : ((x) > 65535 ? 65535 : (x)))
/* Log2 LOD lookup table (identical to 86Box logtable[]) */
static const uint8_t logtable[256] = {
    0x00,0x01,0x02,0x04,0x05,0x07,0x08,0x09,0x0b,0x0c,0x0e,0x0f,0x10,0x12,0x13,0x15,
    0x16,0x17,0x19,0x1a,0x1b,0x1d,0x1e,0x1f,0x21,0x22,0x23,0x25,0x26,0x27,0x28,0x2a,
    0x2b,0x2c,0x2e,0x2f,0x30,0x31,0x33,0x34,0x35,0x36,0x38,0x39,0x3a,0x3b,0x3d,0x3e,
    0x3f,0x40,0x41,0x43,0x44,0x45,0x46,0x47,0x49,0x4a,0x4b,0x4c,0x4d,0x4e,0x50,0x51,
    0x52,0x53,0x54,0x55,0x57,0x58,0x59,0x5a,0x5b,0x5c,0x5d,0x5e,0x60,0x61,0x62,0x63,
    0x64,0x65,0x66,0x67,0x68,0x69,0x6a,0x6c,0x6d,0x6e,0x6f,0x70,0x71,0x72,0x73,0x74,
    0x75,0x76,0x77,0x78,0x79,0x7a,0x7b,0x7c,0x7d,0x7e,0x7f,0x80,0x81,0x83,0x84,0x85,
    0x86,0x87,0x88,0x89,0x8a,0x8b,0x8c,0x8c,0x8d,0x8e,0x8f,0x90,0x91,0x92,0x93,0x94,
    0x95,0x96,0x97,0x98,0x99,0x9a,0x9b,0x9c,0x9d,0x9e,0x9f,0xa0,0xa1,0xa2,0xa2,0xa3,
    0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xab,0xac,0xad,0xad,0xae,0xaf,0xb0,0xb1,0xb2,
    0xb3,0xb4,0xb5,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xbb,0xbc,0xbc,0xbd,0xbe,0xbf,0xc0,
    0xc1,0xc2,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,0xc8,0xc9,0xca,0xcb,0xcc,0xcd,0xcd,
    0xce,0xcf,0xd0,0xd1,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd6,0xd7,0xd8,0xd9,0xda,0xda,
    0xdb,0xdc,0xdd,0xde,0xde,0xdf,0xe0,0xe1,0xe1,0xe2,0xe3,0xe4,0xe5,0xe5,0xe6,0xe7,
    0xe8,0xe8,0xe9,0xea,0xeb,0xeb,0xec,0xed,0xee,0xef,0xef,0xf0,0xf1,0xf2,0xf2,0xf3,
    0xf4,0xf5,0xf5,0xf6,0xf7,0xf7,0xf8,0xf9,0xfa,0xfa,0xfb,0xfc,0xfd,0xfd,0xfe,0xff
};

/*
 * Hardware-accurate dither tables (voodoo3_dither_tables.c / vid_voodoo_dither.h).
 * Convenience aliases matching 86Box naming used in the render loop below.
 */
#include "hw/display/voodoo3_dither_tables.h"
#define dither_rb      voodoo3_dither_rb
#define dither_g       voodoo3_dither_g
#define dither_rb2x2   voodoo3_dither_rb2x2
#define dither_g2x2    voodoo3_dither_g2x2
#define dithersub_rb   voodoo3_dithersub_rb
#define dithersub_g    voodoo3_dithersub_g
#define dithersub_rb2x2 voodoo3_dithersub_rb2x2
#define dithersub_g2x2  voodoo3_dithersub_g2x2

/* fastlog — same algorithm as 86Box */
static inline int fastlog(uint64_t val)
{
    uint64_t oldval = val;
    int exp = 63, frac;
    if (!val || (val & (1ULL << 63))) return (int)0x80000000;
    if (!(val & 0xffffffff00000000ULL)) { exp -= 32; val <<= 32; }
    if (!(val & 0xffff000000000000ULL)) { exp -= 16; val <<= 16; }
    if (!(val & 0xff00000000000000ULL)) { exp -= 8;  val <<= 8;  }
    if (!(val & 0xf000000000000000ULL)) { exp -= 4;  val <<= 4;  }
    if (!(val & 0xc000000000000000ULL)) { exp -= 2;  val <<= 2;  }
    if (!(val & 0x8000000000000000ULL)) { exp -= 1;  val <<= 1;  }
    frac = (exp >= 8) ? (int)((oldval >> (exp - 8)) & 0xff)
                      : (int)((oldval << (8 - exp)) & 0xff);
    return (exp << 8) | logtable[frac];
}

/* Count leading zeros (16-bit) for W-depth calculation */
static inline int voodoo_fls(uint16_t val)
{
    int n = 0;
    if (!(val & 0xff00)) { n += 8; val <<= 8; }
    if (!(val & 0xf000)) { n += 4; val <<= 4; }
    if (!(val & 0xc000)) { n += 2; val <<= 2; }
    if (!(val & 0x8000)) { n += 1; }
    return n;
}

/* =========================================================================
 * Per-scanline render state — equivalent to 86Box voodoo_state_t
 * ========================================================================= */
typedef struct {
    /* Scanline bounds */
    int      xstart, xend, xdir;
    int      y, yend;

    /* Edge deltas (fixed-point 12.4) */
    int32_t  dxAB, dxAC, dxBC;
    int      dx1, dx2;

    /* Vertex screen coordinates (fixed-point) */
    int32_t  vertexAx, vertexAy;
    int32_t  vertexBx, vertexBy;
    int32_t  vertexCx, vertexCy;

    /* Interpolated colour / depth / alpha */
    int32_t  base_r, base_g, base_b, base_a, base_z;
    int32_t  ir, ig, ib, ia, z;

    /* Homogeneous W */
    int64_t  base_w, w;

    /* Per-TMU interpolants */
    struct {
        int64_t base_s, base_t, base_w;
        int     lod;
    } tmu[2];
    int64_t tmu0_s, tmu0_t, tmu0_w;
    int64_t tmu1_s, tmu1_t, tmu1_w;

    /* LOD */
    int     lod, lod_min[2], lod_max[2], lod_frac[2];

    /* Texture samples (from tex_read) */
    int     tex_r[2], tex_g[2], tex_b[2], tex_a[2];
    int     tex_s, tex_t;
    int     clamp_s[2], clamp_t[2];

    /* Texture data pointers (set from texture cache — currently NULL) */
    uint32_t *tex[2][V3_LOD_MAX + 1];
    int      *tex_w_mask[2], *tex_h_mask[2], *tex_shift[2], *tex_lod[2];

    /* FB / aux row pointers for current scanline */
    uint16_t *fb_mem;
    uint16_t *aux_mem;

    /* Stipple state */
    uint32_t  stipple;
} v3_state_t;

/* =========================================================================
 * Texture fetch helpers
 * Ported from 86Box tex_read() and tex_read_4()
 * ========================================================================= */
static inline void tex_read(v3_state_t *st, int s, int t,
                             int w_mask, int h_mask, int shift, int tmu)
{
    uint32_t dat;
    if (s & ~w_mask) {
        if (st->clamp_s[tmu]) { s = s < 0 ? 0 : (s > w_mask ? w_mask : s); }
        else s &= w_mask;
    }
    if (t & ~h_mask) {
        if (st->clamp_t[tmu]) { t = t < 0 ? 0 : (t > h_mask ? h_mask : t); }
        else t &= h_mask;
    }
    /* tex_ptr is set by voodoo3_use_texture() before triangle is queued */
    const uint32_t *texdata = st->tex[tmu][st->lod];
    if (!texdata) {
        /* NULL texture pointer — voodoo3_use_texture() was not called or the
         * texture cache entry was evicted.  Return a mid-grey sentinel so
         * missing textures are visible without a crash. */
        qemu_log_mask(LOG_GUEST_ERROR,
            "voodoo3: tex_read: NULL tex ptr tmu=%d lod=%d — "
            "texture not loaded, returning grey sentinel\n",
            tmu, st->lod);
        st->tex_r[tmu] = st->tex_g[tmu] = st->tex_b[tmu] = 0x80;
        st->tex_a[tmu] = 0xff;
        return;
    }
    dat = texdata[s + (t << shift)];
    st->tex_b[tmu] = dat & 0xff;
    st->tex_g[tmu] = (dat >> 8)  & 0xff;
    st->tex_r[tmu] = (dat >> 16) & 0xff;
    st->tex_a[tmu] = (dat >> 24) & 0xff;
}

static inline void tex_read_bilinear(v3_state_t *st,
                                     int s, int t, int tex_lod,
                                     int w_mask, int h_mask, int shift,
                                     int tmu)
{
    /* s/t already adjusted: sub-pixel fractions in low 4 bits */
    int ds = s & 0xf, dt = t & 0xf;
    s >>= 4; t >>= 4;
    int d[4] = { (16 - ds) * (16 - dt), ds * (16 - dt),
                 (16 - ds) * dt,          ds * dt };

    uint32_t dat[4];
    for (int c = 0; c < 4; c++) {
        int _s = s + (c & 1), _t = t + ((c >> 1) & 1);
        if (_s & ~w_mask) {
            if (st->clamp_s[tmu]) _s = _s < 0 ? 0 : (_s > w_mask ? w_mask : _s);
            else _s &= w_mask;
        }
        if (_t & ~h_mask) {
            if (st->clamp_t[tmu]) _t = _t < 0 ? 0 : (_t > h_mask ? h_mask : _t);
            else _t &= h_mask;
        }
        if (st->tex[tmu][st->lod])
            dat[c] = st->tex[tmu][st->lod][_s + (_t << shift)];
        else
            dat[c] = 0x808080ff;
    }

#define BLND(ch, sh) ((((dat[0] >> (sh)) & 0xff) * d[0] + \
                       ((dat[1] >> (sh)) & 0xff) * d[1] + \
                       ((dat[2] >> (sh)) & 0xff) * d[2] + \
                       ((dat[3] >> (sh)) & 0xff) * d[3]) >> 8)
    st->tex_b[tmu] = BLND(b,  0);
    st->tex_g[tmu] = BLND(g,  8);
    st->tex_r[tmu] = BLND(r, 16);
    st->tex_a[tmu] = BLND(a, 24);
#undef BLND
    (void)tex_lod;
}

/* =========================================================================
 * voodoo_tmu_fetch — perspective-correct texture coordinate calculation
 * Ported from 86Box voodoo_tmu_fetch()
 * ========================================================================= */
static void v3_tmu_fetch(v3_state_t *st, const voodoo3_params_t *p,
                         int tmu, bool bilinear)
{
    int     w_mask = st->tex_w_mask[tmu] ? *st->tex_w_mask[tmu] : 0xff;
    int     h_mask = st->tex_h_mask[tmu] ? *st->tex_h_mask[tmu] : 0xff;
    int     shift  = st->tex_shift[tmu]  ? *st->tex_shift[tmu]  : 8;
    int64_t tmuw   = tmu ? st->tmu1_w : st->tmu0_w;
    int64_t tmus   = tmu ? st->tmu1_s : st->tmu0_s;
    int64_t tmut   = tmu ? st->tmu1_t : st->tmu0_t;
    int     tex_lod_val = st->tex_lod[tmu] ? *st->tex_lod[tmu] : 0;
    int     s, t;

    if (p->tmu[tmu].textureMode & TEXMODE_PERSP_CORR) {
        int64_t w = tmuw ? (int64_t)((1ULL << 48) / (uint64_t)tmuw) : 0;
        s = (int32_t)((((tmus + (1 << 13)) >> 14) * w + (1 << 29)) >> 30);
        t = (int32_t)((((tmut + (1 << 13)) >> 14) * w + (1 << 29)) >> 30);

        st->lod = st->tmu[tmu].lod + (fastlog((uint64_t)llabs(w)) - (19 << 8));
    } else {
        s = (int32_t)(tmus >> (14 + 14));
        t = (int32_t)(tmut >> (14 + 14));
        st->lod = st->tmu[tmu].lod;
    }

    if (st->lod < st->lod_min[tmu]) st->lod = st->lod_min[tmu];
    if (st->lod > st->lod_max[tmu]) st->lod = st->lod_max[tmu];
    st->lod_frac[tmu] = st->lod & 0xff;
    st->lod >>= 8;

    /* Mirror */
    if (p->tmu[tmu].textureMode & TEXMODE_TMIRROR_S)
        if (s & 0x1000) s = ~s;
    if (p->tmu[tmu].textureMode & TEXMODE_TMIRROR_T)
        if (t & 0x1000) t = ~t;

    if (bilinear && (p->tmu[tmu].textureMode & 6)) {
        s -= 1 << (3 + tex_lod_val);
        t -= 1 << (3 + tex_lod_val);
        tex_read_bilinear(st, s >> tex_lod_val, t >> tex_lod_val,
                          tex_lod_val, w_mask, h_mask, shift - tex_lod_val, tmu);
    } else {
        tex_read(st, s >> (4 + tex_lod_val), t >> (4 + tex_lod_val),
                 w_mask, h_mask, shift, tmu);
    }
}

/* =========================================================================
 * Depth test helper
 * ========================================================================= */
static inline bool depth_test(int op, uint16_t new_d, uint16_t old_d)
{
    switch (op) {
    case DEPTH_OP_NEVER:  return false;
    case DEPTH_OP_LT:     return new_d <  old_d;
    case DEPTH_OP_EQ:     return new_d == old_d;
    case DEPTH_OP_LE:     return new_d <= old_d;
    case DEPTH_OP_GT:     return new_d >  old_d;
    case DEPTH_OP_NE:     return new_d != old_d;
    case DEPTH_OP_GE:     return new_d >= old_d;
    case DEPTH_OP_ALWAYS: return true;
    default:              return true;
    }
}

/* =========================================================================
 * Alpha blend helper — all 12 src/dst factor modes
 * ========================================================================= */
static inline void alpha_blend(int *r, int *g, int *b, int src_a,
                                uint8_t dst_r, uint8_t dst_g, uint8_t dst_b,
                                uint8_t dst_a, uint32_t alphaMode,
                                int colbfog_r, int colbfog_g, int colbfog_b)
{
    /*
     * Ported from 86Box ALPHA_BLEND macro (vid_voodoo_render.h).
     *
     * Cases 0-5 use a scalar factor (sf_r=sf_g=sf_b).
     * Cases 6 (AOM_COLOR) and 2 (A_COLOR) need per-channel factors
     * because the factor is derived from src or dst colour components.
     * Case 0xf: ASATURATE for src, ACOLORBEFOREFOG for dst.
     *
     * Result: src = src*src_factor/256, dst = dst*dst_factor/256, out = src+dst.
     */
    int src_fn = (int)ALPHA_SRC_FUNC(alphaMode);
    int dst_fn = (int)ALPHA_DST_FUNC(alphaMode);

    int sf_r, sf_g, sf_b;

    switch (src_fn) {
    case 0:  sf_r = sf_g = sf_b = 0;              break;
    case 1:  sf_r = sf_g = sf_b = 0xff;            break;
    case 2:  sf_r = sf_g = sf_b = src_a;           break;
    case 3:  sf_r = sf_g = sf_b = 0xff - src_a;    break;
    case 4:  sf_r = sf_g = sf_b = dst_a;           break;
    case 5:  sf_r = sf_g = sf_b = 0xff - dst_a;    break;
    case 6:  /* A_COLOR — per-channel: factor = dst_rgb */
        sf_r = dst_r; sf_g = dst_g; sf_b = dst_b; break;
    case 7:  /* AOM_COLOR — per-channel: factor = 1 - dst_rgb */
        sf_r = 0xff - dst_r; sf_g = 0xff - dst_g; sf_b = 0xff - dst_b; break;
    case 0xf: {
        /* ASATURATE: factor = min(src_a, 255-dst_a) */
        int _a = src_a < (0xff - dst_a) ? src_a : (0xff - dst_a);
        sf_r = sf_g = sf_b = _a; break;
    }
    default: sf_r = sf_g = sf_b = 0xff; break;
    }

    int df_r, df_g, df_b;

    switch (dst_fn) {
    case 0:  df_r = df_g = df_b = 0;              break;
    case 1:  df_r = df_g = df_b = 0xff;            break;
    case 2:  df_r = df_g = df_b = src_a;           break;
    case 3:  df_r = df_g = df_b = 0xff - src_a;    break;
    case 4:  df_r = df_g = df_b = dst_a;           break;
    case 5:  df_r = df_g = df_b = 0xff - dst_a;    break;
    case 6:  /* A_COLOR — per-channel: factor = src_rgb */
        df_r = *r; df_g = *g; df_b = *b; break;
    case 7:  /* AOM_COLOR — per-channel: factor = 1 - src_rgb */
        df_r = 0xff - *r; df_g = 0xff - *g; df_b = 0xff - *b; break;
    case 0xf: /* ACOLORBEFOREFOG: factor = pre-fog src colour */
        df_r = colbfog_r; df_g = colbfog_g; df_b = colbfog_b; break;
    default: df_r = df_g = df_b = 0; break;
    }

    *r = CLAMP(((*r * (sf_r + 1) + dst_r * (df_r + 1)) >> 8));
    *g = CLAMP(((*g * (sf_g + 1) + dst_g * (df_g + 1)) >> 8));
    *b = CLAMP(((*b * (sf_b + 1) + dst_b * (df_b + 1)) >> 8));
}

/* =========================================================================
 * Fog application
 * Ported from 86Box APPLY_FOG macro in vid_voodoo_regs.h
 * ========================================================================= */
static inline void apply_fog(int *r, int *g, int *b,
                              int32_t z, int32_t ia, int64_t w,
                              const voodoo3_params_t *p)
{
    /*
     * Ported from 86Box APPLY_FOG macro (vid_voodoo_render.h).
     *
     * FOG_CONSTANT: just add fogColor — no fog factor needed.
     *
     * Otherwise: compute fog_r/g/b = (fogColor - src) or 0 for ADD,
     * then multiply by fog_a and apply as ADD or MULT.
     * The fog_a++ matches 86Box: factor = 0 → 1/256, factor = 255 → 1.0.
     *
     * W-based fog (default, bits Z|ALPHA both 0):
     *   w_depth is a log2-encoded depth; fog table has 64 entries each with
     *   a .fog byte and a .dfog slope for sub-entry linear interpolation.
     *   86Box: fog_a = fogTable[idx].fog
     *                + (fogTable[idx].dfog * ((w_depth >> 2) & 0xff)) >> 10
     *
     * FOG_W (bits 3|4 both set, i.e. FOG_Z|FOG_ALPHA): fog_a from w high byte.
     *   86Box: fog_a = CLAMP((w >> 32) & 0xff)
     */
    uint32_t fog_mode = p->fogMode;

    if (fog_mode & FOG_CONSTANT) {
        *r = CLAMP(*r + p->fogColor.r);
        *g = CLAMP(*g + p->fogColor.g);
        *b = CLAMP(*b + p->fogColor.b);
        return;
    }

    int fog_r, fog_g, fog_b, fog_a;

    if (!(fog_mode & FOG_ADD)) {
        fog_r = p->fogColor.r;
        fog_g = p->fogColor.g;
        fog_b = p->fogColor.b;
    } else {
        fog_r = fog_g = fog_b = 0;
    }

    if (!(fog_mode & FOG_MULT)) {
        fog_r -= *r;
        fog_g -= *g;
        fog_b -= *b;
    }

    switch (fog_mode & (FOG_Z | FOG_ALPHA)) {
    case 0: {
        /* W-based fog — log-encoded depth + dfog interpolation */
        int w_depth;
        if (w & 0xffff00000000LL)
            w_depth = 0;
        else if (!(w & 0xffff0000LL))
            w_depth = 0xf001;
        else {
            int exp  = voodoo_fls((uint16_t)((uint32_t)w >> 16));
            int mant = (~(uint32_t)w >> (19 - exp)) & 0xfff;
            w_depth  = (exp << 12) + mant + 1;
            if (w_depth > 0xffff) w_depth = 0xffff;
        }
        unsigned idx = (unsigned)w_depth >> 10;
        if (idx >= 64) idx = 63;
        fog_a = p->fogTable[idx].fog
              + ((p->fogTable[idx].dfog * ((w_depth >> 2) & 0xff)) >> 10);
        break;
    }
    case FOG_Z:
        fog_a = (z >> 20) & 0xff;
        break;
    case FOG_ALPHA:
        fog_a = CLAMP(ia >> 12);
        break;
    default: /* FOG_Z | FOG_ALPHA = FOG_W */
        fog_a = CLAMP((int)((w >> 32) & 0xff));
        break;
    }

    fog_a++;  /* 0→1/256, 255→1.0  (matches 86Box fog_a++) */

    fog_r = (fog_r * fog_a) >> 8;
    fog_g = (fog_g * fog_a) >> 8;
    fog_b = (fog_b * fog_a) >> 8;

    if (fog_mode & FOG_MULT) {
        *r = CLAMP(fog_r);
        *g = CLAMP(fog_g);
        *b = CLAMP(fog_b);
    } else {
        *r = CLAMP(*r + fog_r);
        *g = CLAMP(*g + fog_g);
        *b = CLAMP(*b + fog_b);
    }
}

/* =========================================================================
 * voodoo3_half_triangle — render one half of a triangle (A→B or B→C)
 * Ported from 86Box voodoo_half_triangle()
 * ========================================================================= */
static void v3_half_triangle(Voodoo3State *s, const voodoo3_params_t *p,
                              v3_state_t *st, int ystart, int yend,
                              int odd_even)
{
    uint32_t fbz  = p->fbzMode;
    uint32_t fcp  = p->fbzColorPath;
    uint32_t alm  = p->alphaMode;
    uint32_t fogm = p->fogMode;
    bool     bilinear = s->bilinear && true;

    bool clip_en      = !!(fbz & FBZ_ENABLE_CLIPPING);
    bool depth_en     = !!(fbz & FBZ_DEPTH_ENABLE);
    bool depth_w      = !!(fbz & FBZ_W_BUFFER);
    bool rgb_wmask    = !!(fbz & FBZ_RGB_WMASK);
    bool depth_wmask  = !!(fbz & FBZ_DEPTH_WMASK);
    bool alpha_en_aux = !!(fbz & FBZ_ALPHA_ENABLE);
    bool fog_en       = !!(fogm & FOG_ENABLE);
    bool alpha_en     = !!(alm & ALPHA_ENABLE);
    bool blend_en     = !!(alm & ALPHA_BLEND_EN);
    bool chroma_en    = !!(fbz & FBZ_CHROMAKEY);
    bool stipple_en   = !!(fbz & FBZ_STIPPLE);
    bool stipple_patt = !!(fbz & FBZ_STIPPLE_PATT);
    bool dither_en    = !!(fbz & FBZ_DITHER);
    bool dither_2x2   = !!(fbz & FBZ_DITHER_2X2);
    bool dithersub_en = !!(fbz & FBZ_DITHER_SUB);
    bool y_origin     = !!(fbz & FBZ_Y_ORIGIN);
    bool tex_en       = FBZCP_TEXTURE_ENABLED(fcp);

    int depth_op    = (fbz >> FBZ_DEPTH_OP_SHIFT) & 7;
    int y_origin_v  = s->y_origin_swap;

    /* Decode fbzColorPath selectors once (same for all pixels on triangle) */
    int _rgb_sel        = (int)FBZCP_CC_RGBSELECT(fcp);
    int _a_sel          = (int)FBZCP_CC_ASELECT(fcp);
    int cc_localselect  = (int)FBZCP_CC_LOCALSELECT(fcp);
    int cca_localselect = (int)FBZCP_CCA_LOCALSELECT(fcp);
    int cc_localsel_ovr = (int)FBZCP_CC_LOCALSELECT_OVR(fcp);
    int cc_zero_other   = (int)FBZCP_CC_ZERO_OTHER(fcp);
    int cc_sub_clocal   = (int)FBZCP_CC_SUB_CLOCAL(fcp);
    int cc_mselect      = (int)FBZCP_CC_MSELECT(fcp);
    int cc_rev_blend    = (int)FBZCP_CC_REVERSE_BLEND(fcp);
    int cc_add          = (int)FBZCP_CC_ADD(fcp);
    int cc_add_alocal   = (int)FBZCP_CC_ADD_ALOCAL(fcp);
    int cc_invert       = (int)FBZCP_CC_INVERT_OUT(fcp);
    int cca_zero_other  = (int)FBZCP_CCA_ZERO_OTHER(fcp);
    int cca_sub_clocal  = (int)FBZCP_CCA_SUB_CLOCAL(fcp);
    int cca_mselect     = (int)FBZCP_CCA_MSELECT(fcp);
    int cca_rev_blend   = (int)FBZCP_CCA_REVERSE_BLEND(fcp);
    int cca_add         = (int)FBZCP_CCA_ADD(fcp);
    int cca_invert      = (int)FBZCP_CCA_INVERT_OUT(fcp);

    /* -----------------------------------------------------------------------
     * Debug: verify dither table, render path, and texture pointers.
     * These fire once per triangle (not per pixel) so overhead is minimal.
     * ----------------------------------------------------------------------- */
#ifdef DEBUG_VOODOO3_RENDER
    /* Dither table sanity: when dither is enabled the tables must be inited */
    if (dither_en) {
        /* voodoo3_dither_rb[0][0][0] should never be zero after table init */
        if (voodoo3_dither_rb[127][3][3] == 0 && voodoo3_dither_rb[1][0][0] == 0) {
            qemu_log_mask(LOG_GUEST_ERROR,
                "voodoo3: WARNING dither enabled but dither table looks uninitialised "
                "(fbzMode=0x%08x dither_2x2=%d dithersub=%d)\n",
                fbz, dither_2x2, dithersub_en);
        } else {
            qemu_log_mask(LOG_UNIMP,
                "voodoo3: render dither active: fbzMode=0x%08x "
                "dither_2x2=%d dithersub=%d\n",
                fbz, dither_2x2, dithersub_en);
        }
    }

    /* Render path: log whether texturing is active */
    qemu_log_mask(LOG_UNIMP,
        "voodoo3: render tri#%u tex_en=%d dither_en=%d depth_en=%d "
        "blend_en=%d fog_en=%d alpha_en=%d\n",
        s->tri_count, tex_en, dither_en, depth_en,
        blend_en, fog_en, alpha_en);

    /* Texture pointer validation: if tex_en is set, TMU0 pointers must exist */
    if (tex_en) {
        bool tmu0_ok = false, tmu1_ok = false;
        for (int _l = 0; _l <= V3_LOD_MAX; _l++) {
            if (st->tex[0][_l]) { tmu0_ok = true; break; }
        }
        if (!tmu0_ok) {
            qemu_log_mask(LOG_GUEST_ERROR,
                "voodoo3: tex_en=1 but TMU0 tex pointers all NULL "
                "(tri#%u textureMode0=0x%08x) — expect black/garbage\n",
                s->tri_count, p->tmu[0].textureMode);
        }
        /* Check TMU1 only when dual-TMU blend is active (passthrough mode off) */
        if ((p->tmu[0].textureMode & 0x00643000u) != 0x00241000u) {
            for (int _l = 0; _l <= V3_LOD_MAX; _l++) {
                if (st->tex[1][_l]) { tmu1_ok = true; break; }
            }
            if (!tmu1_ok) {
                qemu_log_mask(LOG_GUEST_ERROR,
                    "voodoo3: dual-TMU active but TMU1 tex pointers all NULL "
                    "(tri#%u textureMode1=0x%08x)\n",
                    s->tri_count, p->tmu[1].textureMode);
            }
        }
    } else if (!tex_en) {
        /* Sanity: if FBZCP_TEXTURE_ENABLED is clear, CC path must not reference
         * texture (cc_localselect values 0/1 are safe; value 2 = texture) */
        if (_rgb_sel == 2) {
            qemu_log_mask(LOG_GUEST_ERROR,
                "voodoo3: tex_en=0 but fbzColorPath CC_RGBSELECT=2 (texture) "
                "(fbzColorPath=0x%08x) — output will be undefined\n",
                fcp);
        }
    }
#endif /* DEBUG_VOODOO3_RENDER */

    /* Apply top clip */
    if (clip_en && ystart < p->clipLowY) {
        int dy = p->clipLowY - ystart;
        st->base_r   += p->dRdY * dy; st->base_g   += p->dGdY * dy;
        st->base_b   += p->dBdY * dy; st->base_a   += p->dAdY * dy;
        st->base_z   += p->dZdY * dy; st->base_w   += p->dWdY * dy;
        st->tmu[0].base_s += p->tmu[0].dSdY * dy;
        st->tmu[0].base_t += p->tmu[0].dTdY * dy;
        st->tmu[0].base_w += p->tmu[0].dWdY * dy;
        st->tmu[1].base_s += p->tmu[1].dSdY * dy;
        st->tmu[1].base_t += p->tmu[1].dTdY * dy;
        st->tmu[1].base_w += p->tmu[1].dWdY * dy;
        st->xstart += st->dx1 * dy;
        st->xend   += st->dx2 * dy;
        ystart = p->clipLowY;
    }
    if (clip_en && yend >= p->clipHighY)
        yend = p->clipHighY;

    for (st->y = ystart; st->y < yend; st->y++) {

        int      real_y = (st->y << 4) + 8;
        int      x, x2, dx;
        uint16_t *fb_row, *aux_row;

        st->ir = st->base_r; st->ig = st->base_g;
        st->ib = st->base_b; st->ia = st->base_a;
        st->z  = st->base_z; st->w  = st->base_w;
        st->tmu0_s = st->tmu[0].base_s; st->tmu0_t = st->tmu[0].base_t;
        st->tmu0_w = st->tmu[0].base_w;
        st->tmu1_s = st->tmu[1].base_s; st->tmu1_t = st->tmu[1].base_t;
        st->tmu1_w = st->tmu[1].base_w;

        /* Edge interpolation */
        x  = (st->vertexAx << 12) + ((st->dxAC * (real_y - st->vertexAy)) >> 4);
        if (real_y < st->vertexBy)
            x2 = (st->vertexAx << 12) + ((st->dxAB * (real_y - st->vertexAy)) >> 4);
        else
            x2 = (st->vertexBx << 12) + ((st->dxBC * (real_y - st->vertexBy)) >> 4);

        /* Y-origin flip */
        int screen_y = y_origin ? (y_origin_v - (real_y >> 4)) : (real_y >> 4);

        /*
         * Band-parallel scanline filter — ported from 86Box
         * voodoo_half_triangle() (vid_voodoo_render.c line 837):
         *
         *   if ((real_y & voodoo->odd_even_mask) != odd_even)
         *       goto next_line;
         *
         * Each render thread only draws the scanlines it owns.
         * With 1 thread: odd_even_mask=0, odd_even=0 → (y & 0)=0==0 always.
         * With 2 threads: mask=1 → thread 0 draws even rows, thread 1 odd.
         * With 4 threads: mask=3 → thread T draws rows where y%4 == T.
         *
         * We filter on screen_y (display-space row) rather than raw real_y
         * so the interleaving is correct when y_origin_flip is active.
         */
        if (((uint32_t)screen_y & s->odd_even_mask) != (uint32_t)odd_even)
            goto next_line;

        /* Sub-pixel correction for parameter interpolation */
        if (st->xdir > 0) x2 -= (1 << 16); else x  -= (1 << 16);
        dx = ((x + 0x7000) >> 16) - (((st->vertexAx << 12) + 0x7000) >> 16);
        x  = (x  + 0x7000) >> 16;
        x2 = (x2 + 0x7000) >> 16;

        /* Apply horizontal sub-pixel correction */
        st->ir += p->dRdX * dx; st->ig += p->dGdX * dx;
        st->ib += p->dBdX * dx; st->ia += p->dAdX * dx;
        st->z  += p->dZdX * dx; st->w  += p->dWdX * dx;
        st->tmu0_s += p->tmu[0].dSdX * dx; st->tmu0_t += p->tmu[0].dTdX * dx;
        st->tmu0_w += p->tmu[0].dWdX * dx;
        st->tmu1_s += p->tmu[1].dSdX * dx; st->tmu1_t += p->tmu[1].dTdX * dx;
        st->tmu1_w += p->tmu[1].dWdX * dx;

        /* Horizontal clip */
        if (clip_en) {
            if (st->xdir > 0) {
                if (x < p->clipLeft) {
                    int cdx = p->clipLeft - x;
                    st->ir += p->dRdX * cdx; st->ig += p->dGdX * cdx;
                    st->ib += p->dBdX * cdx; st->ia += p->dAdX * cdx;
                    st->z  += p->dZdX * cdx; st->w  += p->dWdX * cdx;
                    st->tmu0_s += p->tmu[0].dSdX * cdx;
                    st->tmu0_t += p->tmu[0].dTdX * cdx;
                    st->tmu0_w += p->tmu[0].dWdX * cdx;
                    st->tmu1_s += p->tmu[1].dSdX * cdx;
                    st->tmu1_t += p->tmu[1].dTdX * cdx;
                    st->tmu1_w += p->tmu[1].dWdX * cdx;
                    x = p->clipLeft;
                }
                if (x2 >= p->clipRight) x2 = p->clipRight - 1;
            } else {
                if (x >= p->clipRight) {
                    int cdx = (p->clipRight - 1) - x;
                    st->ir += p->dRdX * cdx; st->ig += p->dGdX * cdx;
                    st->ib += p->dBdX * cdx; st->ia += p->dAdX * cdx;
                    st->z  += p->dZdX * cdx; st->w  += p->dWdX * cdx;
                    st->tmu0_s += p->tmu[0].dSdX * cdx;
                    st->tmu0_t += p->tmu[0].dTdX * cdx;
                    st->tmu0_w += p->tmu[0].dWdX * cdx;
                    st->tmu1_s += p->tmu[1].dSdX * cdx;
                    st->tmu1_t += p->tmu[1].dTdX * cdx;
                    st->tmu1_w += p->tmu[1].dWdX * cdx;
                    x = p->clipRight - 1;
                }
                if (x2 < p->clipLeft) x2 = p->clipLeft;
            }
        }

        if (st->xdir > 0 && x2 < x) goto next_line;
        if (st->xdir < 0 && x2 > x) goto next_line;

        /* Compute row pointers into SGRAM */
        if (p->col_tiled)
            fb_row  = (uint16_t *)(s->fb_mem + p->draw_offset
                      + (screen_y >> 5) * p->row_width + (screen_y & 31) * 128);
        else
            fb_row  = (uint16_t *)(s->fb_mem + p->draw_offset
                      + (size_t)screen_y * p->row_width);

        if (p->aux_tiled)
            aux_row = (uint16_t *)(s->fb_mem + p->aux_offset
                      + (screen_y >> 5) * p->aux_row_width + (screen_y & 31) * 128);
        else
            aux_row = (uint16_t *)(s->fb_mem + p->aux_offset
                      + (size_t)screen_y * p->aux_row_width);

        /* Scanline pixel loop */
        do {
            /* Tiled x-offset */
            int x_t = (x & 63) | ((x >> 6) * 128 * 32 / 2);

            /* --- Stipple --- */
            if (stipple_en) {
                if (stipple_patt) {
                    int idx = ((screen_y & 3) << 3) | (~x & 7);
                    if (!(p->stipple & (1u << idx))) goto skip_pixel;
                } else {
                    st->stipple = (st->stipple << 1) | (st->stipple >> 31);
                    if (!(st->stipple & 0x80000000u)) goto skip_pixel;
                }
            }

            /* --- Depth calculation --- */
            {
                int32_t new_depth, w_depth;

                if ((uint64_t)(st->w >> 32) != 0)
                    w_depth = 0;
                else if (!(st->w & 0xffff0000LL))
                    w_depth = 0xf001;
                else {
                    int exp  = voodoo_fls((uint16_t)((uint32_t)st->w >> 16));
                    int mant = (~(uint32_t)st->w >> (19 - exp)) & 0xfff;
                    w_depth  = (exp << 12) + mant + 1;
                    if (w_depth > 0xffff) w_depth = 0xffff;
                }

                new_depth = depth_w ? w_depth : CLAMP16(st->z >> 12);

                if (fbz & FBZ_DEPTH_BIAS)
                    new_depth = CLAMP16(new_depth + (int16_t)(p->zaColor & 0xffff));

                if (depth_en) {
                    uint16_t old_d = p->aux_tiled ? aux_row[x_t] : aux_row[x];
                    uint16_t test_d = (fbz & FBZ_DEPTH_SOURCE)
                                     ? (uint16_t)(p->zaColor & 0xffff)
                                     : (uint16_t)new_depth;
                    if (!depth_test(depth_op, test_d, old_d))
                        goto skip_pixel;
                }

                /* --- Read destination pixel --- */
                uint16_t dst_raw = p->col_tiled ? fb_row[x_t] : fb_row[x];
                uint8_t  dest_r = (uint8_t)(((dst_raw >> 11) & 0x1f) * 255 / 31);
                uint8_t  dest_g = (uint8_t)(((dst_raw >>  5) & 0x3f) * 255 / 63);
                uint8_t  dest_b = (uint8_t)((dst_raw & 0x1f) * 255 / 31);
                uint8_t  dest_a = 0xff;
                if (alpha_en_aux)
                    dest_a = p->aux_tiled ? (uint8_t)aux_row[x_t]
                                          : (uint8_t)aux_row[x];

                /* --- Texture fetch --- */
                if (tex_en) {
                    uint32_t tm0 = p->tmu[0].textureMode;
                    uint32_t tm1 = p->tmu[1].textureMode;
                    if ((tm0 & TEXMODE_LOCAL_MASK) == TEXMODE_LOCAL) {
                        v3_tmu_fetch(st, p, 0, bilinear);
                    } else if ((tm0 & TEXMODE_LOCAL_MASK) == TEXMODE_PASSTHROUGH) {
                        v3_tmu_fetch(st, p, 1, bilinear);
                        st->tex_r[0] = st->tex_r[1];
                        st->tex_g[0] = st->tex_g[1];
                        st->tex_b[0] = st->tex_b[1];
                        st->tex_a[0] = st->tex_a[1];
                    } else {
                        v3_tmu_fetch(st, p, 1, bilinear);
                        v3_tmu_fetch(st, p, 0, bilinear);
                    }
                    (void)tm1;
                }

                /* --- Colour selection (clocal / cother) --- */
                uint8_t clocal_r, clocal_g, clocal_b, alocal;
                uint8_t cother_r = 0, cother_g = 0, cother_b = 0, aother;

                int sel = cc_localsel_ovr ? ((st->tex_a[0] & 0x80) ? 1 : 0)
                                          : cc_localselect;
                if (sel) {
                    clocal_r = (p->color0 >> 16) & 0xff;
                    clocal_g = (p->color0 >>  8) & 0xff;
                    clocal_b =  p->color0         & 0xff;
                } else {
                    clocal_r = (uint8_t)CLAMP(st->ir >> 12);
                    clocal_g = (uint8_t)CLAMP(st->ig >> 12);
                    clocal_b = (uint8_t)CLAMP(st->ib >> 12);
                }

                switch (_rgb_sel) {
                case 0: /* iterated RGB */
                    cother_r = (uint8_t)CLAMP(st->ir >> 12);
                    cother_g = (uint8_t)CLAMP(st->ig >> 12);
                    cother_b = (uint8_t)CLAMP(st->ib >> 12);
                    break;
                case 1: /* texture output */
                    cother_r = (uint8_t)st->tex_r[0];
                    cother_g = (uint8_t)st->tex_g[0];
                    cother_b = (uint8_t)st->tex_b[0];
                    break;
                case 2: /* color1 */
                    cother_r = (p->color1 >> 16) & 0xff;
                    cother_g = (p->color1 >>  8) & 0xff;
                    cother_b =  p->color1         & 0xff;
                    break;
                default: break;
                }

                /* Chroma-key */
                if (chroma_en &&
                    cother_r == p->chromaKey_r &&
                    cother_g == p->chromaKey_g &&
                    cother_b == p->chromaKey_b) {
                    s->fbiChromaFail++;
                    goto skip_pixel;
                }

                /* CCA local select */
                switch (cca_localselect) {
                case CCA_LOCALSEL_ITER_A:  alocal = (uint8_t)CLAMP(st->ia >> 12); break;
                case CCA_LOCALSEL_COLOR0:  alocal = (p->color0 >> 24) & 0xff;     break;
                case CCA_LOCALSEL_ITER_Z:  alocal = (uint8_t)CLAMP(st->z >> 20);  break;
                default:                   alocal = 0xff;                          break;
                }

                switch (_a_sel) {
                case A_SEL_ITER_A:  aother = (uint8_t)CLAMP(st->ia >> 12); break;
                case A_SEL_TEX:     aother = (uint8_t)st->tex_a[0];        break;
                case A_SEL_COLOR1:  aother = (p->color1 >> 24) & 0xff;     break;
                default:            aother = 0;                             break;
                }

                /* Alpha mask bit */
                if ((fbz & FBZ_ALPHA_MASK) && !(aother & 1))
                    goto skip_pixel;

                /* --- Colour combine (fbzColorPath) --- */
                int src_r, src_g, src_b, src_a;

                src_r = cc_zero_other ? 0 : cother_r;
                src_g = cc_zero_other ? 0 : cother_g;
                src_b = cc_zero_other ? 0 : cother_b;
                src_a = cca_zero_other ? 0 : aother;

                if (cc_sub_clocal)  { src_r -= clocal_r; src_g -= clocal_g; src_b -= clocal_b; }
                if (cca_sub_clocal) { src_a -= alocal; }

                /* Multiplier select */
                int msel_r, msel_g, msel_b, msel_a;

                /*
                 * Trilinear mipmap: when TEXMODE_TRILINEAR is set and the
                 * integer LOD is odd, the blend direction of the multiplier
                 * is inverted for this TMU0 stage.
                 * Ported from 86Box voodoo_render.c lines 544–550:
                 *   if ((textureMode[0] & TEXTUREMODE_TRILINEAR) && (lod & 1))
                 *       c_reverse = tc_reverse_blend;   // inverted
                 *   else
                 *       c_reverse = !tc_reverse_blend;  // normal
                 * In 86Box "tc_reverse_blend" is the raw fbzColorPath bit, and
                 * the pixel loop uses c_reverse (not the raw bit) for the XOR.
                 * We replicate that by computing eff_cc_rev_blend here.
                 */
                int eff_cc_rev_blend  = cc_rev_blend;
                int eff_cca_rev_blend = cca_rev_blend;
                if (tex_en &&
                    (p->tmu[0].textureMode & TEXMODE_TRILINEAR) &&
                    (st->lod & 1)) {
                    eff_cc_rev_blend  = !eff_cc_rev_blend;
                    eff_cca_rev_blend = !eff_cca_rev_blend;
                }

                switch (cc_mselect) {
                case CC_MSELECT_ZERO:
                    msel_r = msel_g = msel_b = 0;
                    break;
                case CC_MSELECT_CLOCAL:
                    msel_r = clocal_r; msel_g = clocal_g; msel_b = clocal_b;
                    break;
                case CC_MSELECT_AOTHER:
                    msel_r = msel_g = msel_b = aother;
                    break;
                case CC_MSELECT_ALOCAL:
                    msel_r = msel_g = msel_b = alocal;
                    break;
                case CC_MSELECT_DETAIL: {
                    /*
                     * Detail-texture blend factor.
                     * Ported from 86Box TC_MSELECT_DETAIL case
                     * (voodoo_render.c lines 473–477):
                     *   factor = (detail_bias[tmu] - lod) << detail_scale[tmu]
                     *   factor = clamp(factor, 0, detail_max[tmu])
                     * lod here is the integer LOD after the >>8 shift in
                     * v3_tmu_fetch (st->lod is already the integer part).
                     */
                    int f = (p->detail_bias[0] - st->lod) << p->detail_scale[0];
                    if (f < 0)                   f = 0;
                    if (f > p->detail_max[0])    f = p->detail_max[0];
                    msel_r = msel_g = msel_b = f;
                    break;
                }
                case CC_MSELECT_LOD_FRAC:
                    /*
                     * LOD fractional part as blend factor.
                     * Ported from 86Box TC_MSELECT_LOD_FRAC case
                     * (voodoo_render.c line 479):
                     *   factor = lod_frac[tmu]
                     */
                    msel_r = msel_g = msel_b = st->lod_frac[0];
                    break;
                default:
                    msel_r = msel_g = msel_b = 0;
                    break;
                }
                switch (cca_mselect) {
                case CCA_MSELECT_ZERO:
                    msel_a = 0;
                    break;
                case CCA_MSELECT_ALOCAL:
                case CCA_MSELECT_ALOCAL2:
                    msel_a = alocal;
                    break;
                case CCA_MSELECT_AOTHER:
                    msel_a = aother;
                    break;
                case CCA_MSELECT_DETAIL: {
                    /*
                     * Detail-texture alpha blend factor.
                     * Ported from 86Box TCA_MSELECT_DETAIL case
                     * (voodoo_render.c lines 522–525).
                     */
                    int f = (p->detail_bias[0] - st->lod) << p->detail_scale[0];
                    if (f < 0)                   f = 0;
                    if (f > p->detail_max[0])    f = p->detail_max[0];
                    msel_a = f;
                    break;
                }
                case CCA_MSELECT_LOD_FRAC:
                    /* Ported from 86Box TCA_MSELECT_LOD_FRAC (line 527). */
                    msel_a = st->lod_frac[0];
                    break;
                default:
                    msel_a = 0;
                    break;
                }

                if (!eff_cc_rev_blend)  { msel_r ^= 0xff; msel_g ^= 0xff; msel_b ^= 0xff; }
                if (!eff_cca_rev_blend) { msel_a ^= 0xff; }
                msel_r++; msel_g++; msel_b++; msel_a++;

                src_r = (src_r * msel_r) >> 8;
                src_g = (src_g * msel_g) >> 8;
                src_b = (src_b * msel_b) >> 8;
                src_a = (src_a * msel_a) >> 8;

                /* Add */
                switch (cc_add) {
                case CC_ADD_CLOCAL: src_r += clocal_r; src_g += clocal_g; src_b += clocal_b; break;
                case CC_ADD_ALOCAL: src_r += alocal;   src_g += alocal;   src_b += alocal;    break;
                default: break;
                }
                if (cc_add_alocal) { src_r += alocal; src_g += alocal; src_b += alocal; }
                if (cca_add) src_a += alocal;

                src_r = CLAMP(src_r); src_g = CLAMP(src_g);
                src_b = CLAMP(src_b); src_a = CLAMP(src_a);

                if (cc_invert)  { src_r ^= 0xff; src_g ^= 0xff; src_b ^= 0xff; }
                if (cca_invert) { src_a ^= 0xff; }

                /* --- Fog --- */
                /*
                 * Capture pre-fog colour for ACOLORBEFOREFOG alpha-blend mode.
                 * 86Box saves src_r/g/b before APPLY_FOG as colbfog_r/g/b.
                 */
                int colbfog_r = src_r, colbfog_g = src_g, colbfog_b = src_b;
                if (fog_en)
                    apply_fog(&src_r, &src_g, &src_b,
                              st->z, st->ia, st->w, p);

                /* --- Alpha test --- */
                if (alpha_en) {
                    int afunc = ALPHA_FUNC(alm);
                    int aref  = ALPHA_REF(alm);
                    bool pass;
                    switch (afunc) {
                    case 0: pass = false;         break;
                    case 1: pass = src_a < aref;  break;
                    case 2: pass = src_a == aref; break;
                    case 3: pass = src_a <= aref; break;
                    case 4: pass = src_a > aref;  break;
                    case 5: pass = src_a != aref; break;
                    case 6: pass = src_a >= aref; break;
                    default:pass = true;          break;
                    }
                    if (!pass) { s->fbiAFuncFail++; goto skip_pixel; }
                }

                /* --- Alpha blend --- */
                if (blend_en)
                    alpha_blend(&src_r, &src_g, &src_b, src_a,
                                dest_r, dest_g, dest_b, dest_a, alm,
                                colbfog_r, colbfog_g, colbfog_b);

                /* --- Dither & pack to RGB565 --- */
                if (dither_en) {
                    /*
                     * Use the hardware-accurate lookup tables from
                     * voodoo3_dither_tables.c (verbatim from 86Box
                     * vid_voodoo_dither.h, measured from real hardware).
                     *
                     * 4×4 mode: index with (screen_y & 3), (x & 3)
                     * 2×2 mode: index with (screen_y & 1), (x & 1)
                     *           — uses dedicated 2x2 tables, NOT the 4x4 ones
                     *
                     * FBZ_DITHER_SUB (fbzMode bit 19): apply subtraction
                     * dither to the destination (read-back) colour before
                     * blending, matching 86Box voodoo_half_triangle() logic.
                     */
                    if (dithersub_en) {
                        if (dither_2x2) {
                            dest_r = dithersub_rb2x2[dest_r][screen_y & 1][x & 1];
                            dest_g = dithersub_g2x2 [dest_g][screen_y & 1][x & 1];
                            dest_b = dithersub_rb2x2[dest_b][screen_y & 1][x & 1];
                        } else {
                            dest_r = dithersub_rb[dest_r][screen_y & 3][x & 3];
                            dest_g = dithersub_g [dest_g][screen_y & 3][x & 3];
                            dest_b = dithersub_rb[dest_b][screen_y & 3][x & 3];
                        }
                    }
                    if (dither_2x2) {
                        src_r = dither_rb2x2[src_r][screen_y & 1][x & 1];
                        src_g = dither_g2x2 [src_g][screen_y & 1][x & 1];
                        src_b = dither_rb2x2[src_b][screen_y & 1][x & 1];
                    } else {
                        src_r = dither_rb[src_r][screen_y & 3][x & 3];
                        src_g = dither_g [src_g][screen_y & 3][x & 3];
                        src_b = dither_rb[src_b][screen_y & 3][x & 3];
                    }
                } else {
                    src_r >>= 3; src_g >>= 2; src_b >>= 3;
                }

                /* --- Write pixel --- */
                if (rgb_wmask) {
                    uint16_t pix = (uint16_t)((src_r << 11) | (src_g << 5) | src_b);
                    if (p->col_tiled) fb_row[x_t] = pix;
                    else              fb_row[x]   = pix;

                    /* Mark scanline dirty for display output
                     * (mirrors 86Box dirty_line[] tracking in voodoo_half_triangle) */
                    if (p->draw_offset == p->front_offset) {
                        int _dy = screen_y;
                        if (_dy >= 0 && _dy < V3_DIRTY_LINES)
                            s->dirty_line[_dy] = 1;
                    }
                }

                /* --- Write depth / alpha --- */
                if (depth_wmask) {
                    uint16_t dval = alpha_en_aux ? (uint16_t)src_a
                                                 : (uint16_t)new_depth;
                    if (p->aux_tiled) aux_row[x_t] = dval;
                    else              aux_row[x]   = dval;
                }

                s->fbiPixelsOut++;
            }

skip_pixel:
            /* Step interpolants */
            if (st->xdir > 0) {
                st->ir += p->dRdX; st->ig += p->dGdX;
                st->ib += p->dBdX; st->ia += p->dAdX;
                st->z  += p->dZdX; st->w  += p->dWdX;
                st->tmu0_s += p->tmu[0].dSdX; st->tmu0_t += p->tmu[0].dTdX;
                st->tmu0_w += p->tmu[0].dWdX;
                st->tmu1_s += p->tmu[1].dSdX; st->tmu1_t += p->tmu[1].dTdX;
                st->tmu1_w += p->tmu[1].dWdX;
            } else {
                st->ir -= p->dRdX; st->ig -= p->dGdX;
                st->ib -= p->dBdX; st->ia -= p->dAdX;
                st->z  -= p->dZdX; st->w  -= p->dWdX;
                st->tmu0_s -= p->tmu[0].dSdX; st->tmu0_t -= p->tmu[0].dTdX;
                st->tmu0_w -= p->tmu[0].dWdX;
                st->tmu1_s -= p->tmu[1].dSdX; st->tmu1_t -= p->tmu[1].dTdX;
                st->tmu1_w -= p->tmu[1].dWdX;
            }

            x += st->xdir;
        } while (x != x2 + st->xdir);

        s->fbiPixelsIn += (abs(x2 - x) + 1);

next_line:
        /* Step to next scanline */
        st->base_r += p->dRdY; st->base_g += p->dGdY;
        st->base_b += p->dBdY; st->base_a += p->dAdY;
        st->base_z += p->dZdY; st->base_w += p->dWdY;
        st->tmu[0].base_s += p->tmu[0].dSdY;
        st->tmu[0].base_t += p->tmu[0].dTdY;
        st->tmu[0].base_w += p->tmu[0].dWdY;
        st->tmu[1].base_s += p->tmu[1].dSdY;
        st->tmu[1].base_t += p->tmu[1].dTdY;
        st->tmu[1].base_w += p->tmu[1].dWdY;
        st->xstart += st->dx1;
        st->xend   += st->dx2;
    }
}

/* =========================================================================
 * voodoo3_triangle — main entry point called from render thread
 *
 * Ported from 86Box voodoo_triangle().
 * Sets up scan-conversion state, computes LOD, calls v3_half_triangle().
 * ========================================================================= */
void voodoo3_triangle(Voodoo3State *s, const voodoo3_params_t *p, int odd_even)
{
    v3_state_t st = { 0 };
    int  dx, dy;
    int  vertexAy_adj, vertexCy_adj;

    s->tri_count++;

    /* Sub-pixel correction offsets */
    dx = 8 - (p->vertexAx & 0xf);
    if ((p->vertexAx & 0xf) > 8) dx += 16;
    dy = 8 - (p->vertexAy & 0xf);
    if ((p->vertexAy & 0xf) > 8) dy += 16;

    /* Load base interpolants */
    st.base_r = p->startR; st.base_g = p->startG;
    st.base_b = p->startB; st.base_a = p->startA;
    st.base_z = p->startZ; st.base_w = p->startW;
    st.tmu[0].base_s = p->tmu[0].startS;
    st.tmu[0].base_t = p->tmu[0].startT;
    st.tmu[0].base_w = p->tmu[0].startW;
    st.tmu[1].base_s = p->tmu[1].startS;
    st.tmu[1].base_t = p->tmu[1].startT;
    st.tmu[1].base_w = p->tmu[1].startW;

    /* Sub-pixel parameter adjustment */
    if (FBZCP_PARAM_ADJUST(p->fbzColorPath)) {
        st.base_r += (dx * p->dRdX + dy * p->dRdY) >> 4;
        st.base_g += (dx * p->dGdX + dy * p->dGdY) >> 4;
        st.base_b += (dx * p->dBdX + dy * p->dBdY) >> 4;
        st.base_a += (dx * p->dAdX + dy * p->dAdY) >> 4;
        st.base_z += (dx * p->dZdX + dy * p->dZdY) >> 4;
        st.tmu[0].base_s += (dx * p->tmu[0].dSdX + dy * p->tmu[0].dSdY) >> 4;
        st.tmu[0].base_t += (dx * p->tmu[0].dTdX + dy * p->tmu[0].dTdY) >> 4;
        st.tmu[0].base_w += (dx * p->tmu[0].dWdX + dy * p->tmu[0].dWdY) >> 4;
        st.tmu[1].base_s += (dx * p->tmu[1].dSdX + dy * p->tmu[1].dSdY) >> 4;
        st.tmu[1].base_t += (dx * p->tmu[1].dTdX + dy * p->tmu[1].dTdY) >> 4;
        st.tmu[1].base_w += (dx * p->tmu[1].dWdX + dy * p->tmu[1].dWdY) >> 4;
        st.base_w         += (dx * p->dWdX + dy * p->dWdY) >> 4;
    }

    /* Sign-extend 16-bit vertex coordinates */
#define SEXT16(v) ((int32_t)((v) & ~0xffff0000) | \
                   (((v) & 0x8000) ? 0xffff0000 : 0))
    st.vertexAx = SEXT16(p->vertexAx); st.vertexAy = SEXT16(p->vertexAy);
    st.vertexBx = SEXT16(p->vertexBx); st.vertexBy = SEXT16(p->vertexBy);
    st.vertexCx = SEXT16(p->vertexCx); st.vertexCy = SEXT16(p->vertexCy);
#undef SEXT16

    /* Edge dx/dy gradients (fixed-point, same as 86Box) */
    int32_t dAy = st.vertexBy - st.vertexAy;
    int32_t dBy = st.vertexCy - st.vertexAy;
    int32_t dCy = st.vertexCy - st.vertexBy;

    st.dxAB = dAy ? (int)(((int64_t)(st.vertexBx - st.vertexAx) << 16) / dAy) : 0;
    st.dxAC = dBy ? (int)(((int64_t)(st.vertexCx - st.vertexAx) << 16) / dBy) : 0;
    st.dxBC = dCy ? (int)(((int64_t)(st.vertexCx - st.vertexBx) << 16) / dCy) : 0;

    /* LOD calculation for each TMU (from 86Box voodoo_triangle) */
    for (int t = 0; t < 2; t++) {
        uint64_t tdx = (uint64_t)llabs(p->tmu[t].dSdX >> 14) *
                                  llabs(p->tmu[t].dSdX >> 14)
                     + (uint64_t)llabs(p->tmu[t].dTdX >> 14) *
                                  llabs(p->tmu[t].dTdX >> 14);
        uint64_t tdy = (uint64_t)llabs(p->tmu[t].dSdY >> 14) *
                                  llabs(p->tmu[t].dSdY >> 14)
                     + (uint64_t)llabs(p->tmu[t].dTdY >> 14) *
                                  llabs(p->tmu[t].dTdY >> 14);
        uint64_t tlod = tdx > tdy ? tdx : tdy;

        int LOD = 0;
        if (tlod) {
            LOD = (int)(log2((double)tlod / (double)(1ULL << 36)) * 256.0);
            LOD >>= 2;
        }

        /*
         * FIX 1: wire tLOD lodbias into LOD calculation.
         * p->tmu[t].lodbias is decoded in voodoo3.c (6-bit signed,
         * tLOD[17:12]) before the triangle is queued — use it here.
         *
         * FIX 5: decode lod_min / lod_max from the tLOD register instead of
         * hardcoding 0 / V3_LOD_MAX.
         * tLOD bit layout (Voodoo2/3 hardware spec, matches 86Box):
         *   bits [5:2]  = lod_min  (4-bit integer, 0..8)
         *   bits [11:8] = lod_max  (4-bit integer, 0..8)
         * Both are stored as raw integer LOD levels; multiply by 256 to
         * convert to the 8.8 fixed-point used by the clamp comparisons in
         * voodoo_tmu_fetch() (st->lod < st->lod_min[tmu]).
         */
        int lodbias = p->tmu[t].lodbias;
        st.tmu[t].lod = LOD + (lodbias << 6);

        int lod_min_reg = (int)((p->tmu[t].tLOD >> 2) & 0xf);
        int lod_max_reg = (int)((p->tmu[t].tLOD >> 8) & 0xf);
        if (lod_min_reg > V3_LOD_MAX) lod_min_reg = V3_LOD_MAX;
        if (lod_max_reg > V3_LOD_MAX) lod_max_reg = V3_LOD_MAX;
        st.lod_min[t] = lod_min_reg << 8;
        st.lod_max[t] = lod_max_reg << 8;
    }

    /* Wire decoded texture pointers (set by voodoo3_use_texture) */
    for (int _t = 0; _t < 2; _t++)
        for (int _l = 0; _l <= V3_LOD_MAX; _l++)
            st.tex[_t][_l] = p->tex_ptr[_t][_l];

    /*
     * Wire per-LOD geometry arrays into state.
     * We copy the int arrays from tex_params into local storage so
     * the int* pointers in v3_state_t remain valid for the lifetime
     * of this function.  Declared static is WRONG for multi-thread
     * use, so we use properly-scoped arrays.
     */
    int wm[2][V3_LOD_MAX+2], hm[2][V3_LOD_MAX+2];
    int sh[2][V3_LOD_MAX+2], tl[2][V3_LOD_MAX+2];
    for (int _t = 0; _t < 2; _t++) {
        for (int _l = 0; _l <= V3_LOD_MAX+1; _l++) {
            wm[_t][_l] = p->tex_params[_t].tex_w_mask[_l];
            hm[_t][_l] = p->tex_params[_t].tex_h_mask[_l];
            sh[_t][_l] = p->tex_params[_t].tex_shift[_l];
            tl[_t][_l] = p->tex_params[_t].tex_lod[_l];
        }
        st.tex_w_mask[_t] = wm[_t];
        st.tex_h_mask[_t] = hm[_t];
        st.tex_shift [_t] = sh[_t];
        st.tex_lod   [_t] = tl[_t];
    }

    st.stipple  = p->stipple;
    st.xstart   = st.xend = st.vertexAx << 8;
    st.xdir     = p->sign ? -1 : 1;
    st.dx1      = st.dxAB;
    st.dx2      = st.dxAC;

    vertexAy_adj = (st.vertexAy + 7) >> 4;
    vertexCy_adj = (st.vertexCy + 7) >> 4;

    v3_half_triangle(s, p, &st, vertexAy_adj, vertexCy_adj, odd_even);
}

/* =========================================================================
 * voodoo3_fb_writel — LFB pixel-write through the 3D pipeline
 *
 * Handles FIFO_WRITEL_FB entries queued by voodoo3_mmio_write() and by
 * CMDFIFO packet-5 type-2 (direct LFB writes from the Glide driver).
 *
 * Ported from 86Box src/video/vid_voodoo_fb.c : voodoo_fb_writel().
 * Original author: Sarah Walker.
 *
 * addr = framebuffer byte address (lower 23 bits of the FIFO command word;
 *        the FIFO_WRITEL_FB type bits have already been stripped by the caller)
 * val  = 32-bit pixel value as written by the guest
 *
 * The lfbMode register controls the pixel format.  When lfbMode bit 8
 * (pipeline enable) is set, the full Stipple / Depth / Chroma / Alpha
 * test + blend path runs; otherwise the pixel is written raw (no tests).
 * ========================================================================= */

/* LFB write-mask flags (86Box LFB_WRITE_*) */
#define LFB_WRITE_COLOUR  1
#define LFB_WRITE_DEPTH   2
#define LFB_WRITE_BOTH    3

/* 5-6-5 expand helpers (replicates the low bits into the LSBs for full 0..255) */
#define EXP5(v)  (((v) << 3) | ((v) >> 2))
#define EXP6(v)  (((v) << 2) | ((v) >> 4))

void voodoo3_fb_writel(Voodoo3State *s, uint32_t addr, uint32_t val)
{
    const voodoo3_params_t *p = &s->params;
    int      x, y;
    uint32_t write_addr, write_addr_aux;
    int      col_r[2], col_g[2], col_b[2];
    uint16_t depth_data[2];
    int      alpha_data[2];
    int      write_mask = 0;
    int      count = 1;
    int      fb_mask = (int)(s->fb_size - 1);

    /* Default depth / alpha from zaColor (86Box: depth_data = zaColor & 0xffff,
     * alpha_data = zaColor >> 24) */
    depth_data[0] = depth_data[1] = (uint16_t)(p->zaColor & 0xffffu);
    alpha_data[0] = alpha_data[1] = (int)((p->zaColor >> 24) & 0xffu);

    /* -----------------------------------------------------------------------
     * Decode lfbMode pixel format (bits [3:0]).
     * Mirrors 86Box voodoo_fb_writel() switch (voodoo->lfbMode & LFB_FORMAT_MASK).
     * For dual-pixel formats (RGB565 / RGB555 / ARGB1555 / depth-only),
     * count=2 and addr is a 2-pixel-wide word; single-pixel formats set count=1.
     * ----------------------------------------------------------------------- */
    switch (s->lfbMode & 0xfu) {

    case 0: /* RGB565 — two 16-bit pixels packed into val */
        col_r[0] = EXP5((val      ) >> 11 & 0x1f);
        col_g[0] = EXP6((val      ) >>  5 & 0x3f);
        col_b[0] = EXP5((val      )        & 0x1f);
        col_r[1] = EXP5((val >> 16) >> 11 & 0x1f);
        col_g[1] = EXP6((val >> 16) >>  5 & 0x3f);
        col_b[1] = EXP5((val >> 16)        & 0x1f);
        write_mask = LFB_WRITE_COLOUR; count = 2;
        break;

    case 1: /* RGB555 — two 16-bit pixels, no alpha */
        col_r[0] = EXP5((val      ) >> 10 & 0x1f);
        col_g[0] = EXP5((val      ) >>  5 & 0x1f);
        col_b[0] = EXP5((val      )        & 0x1f);
        col_r[1] = EXP5((val >> 16) >> 10 & 0x1f);
        col_g[1] = EXP5((val >> 16) >>  5 & 0x1f);
        col_b[1] = EXP5((val >> 16)        & 0x1f);
        write_mask = LFB_WRITE_COLOUR; count = 2;
        break;

    case 2: /* ARGB1555 — two 16-bit pixels with 1-bit alpha */
        col_r[0] = EXP5((val      ) >> 10 & 0x1f);
        col_g[0] = EXP5((val      ) >>  5 & 0x1f);
        col_b[0] = EXP5((val      )        & 0x1f);
        alpha_data[0] = ((val      ) & 0x8000u) ? 0xff : 0x00;
        col_r[1] = EXP5((val >> 16) >> 10 & 0x1f);
        col_g[1] = EXP5((val >> 16) >>  5 & 0x1f);
        col_b[1] = EXP5((val >> 16)        & 0x1f);
        alpha_data[1] = ((val >> 16) & 0x8000u) ? 0xff : 0x00;
        write_mask = LFB_WRITE_COLOUR; count = 2;
        break;

    case 4: /* ARGB8888 — one 32-bit pixel */
        col_b[0] =  (int)(val        & 0xffu);
        col_g[0] =  (int)((val >>  8) & 0xffu);
        col_r[0] =  (int)((val >> 16) & 0xffu);
        alpha_data[0] = (int)((val >> 24) & 0xffu);
        write_mask = LFB_WRITE_COLOUR; count = 1;
        addr >>= 1; /* ARGB8888 uses half the address space (2 bytes/pixel) */
        break;

    case 5: /* XRGB8888 — one 32-bit pixel, alpha ignored */
        col_b[0] =  (int)(val        & 0xffu);
        col_g[0] =  (int)((val >>  8) & 0xffu);
        col_r[0] =  (int)((val >> 16) & 0xffu);
        write_mask = LFB_WRITE_COLOUR; count = 1;
        addr >>= 1;
        break;

    case 12: /* depth+RGB565 — colour in low 16 bits, depth in high 16 bits */
        col_r[0] = EXP5((val & 0xffffu) >> 11 & 0x1f);
        col_g[0] = EXP6((val & 0xffffu) >>  5 & 0x3f);
        col_b[0] = EXP5((val & 0xffffu)        & 0x1f);
        depth_data[0] = (uint16_t)(val >> 16);
        write_mask = LFB_WRITE_BOTH; count = 1;
        addr >>= 1;
        break;

    case 13: /* depth+RGB555 */
        col_r[0] = EXP5((val & 0xffffu) >> 10 & 0x1f);
        col_g[0] = EXP5((val & 0xffffu) >>  5 & 0x1f);
        col_b[0] = EXP5((val & 0xffffu)        & 0x1f);
        depth_data[0] = (uint16_t)(val >> 16);
        write_mask = LFB_WRITE_BOTH; count = 1;
        addr >>= 1;
        break;

    case 14: /* depth+ARGB1555 */
        col_r[0] = EXP5((val & 0xffffu) >> 10 & 0x1f);
        col_g[0] = EXP5((val & 0xffffu) >>  5 & 0x1f);
        col_b[0] = EXP5((val & 0xffffu)        & 0x1f);
        alpha_data[0] = ((val & 0x8000u)) ? 0xff : 0x00;
        depth_data[0] = (uint16_t)(val >> 16);
        write_mask = LFB_WRITE_BOTH; count = 1;
        addr >>= 1;
        break;

    case 15: /* depth only — two 16-bit depth values */
        depth_data[0] = (uint16_t)(val & 0xffffu);
        depth_data[1] = (uint16_t)(val >> 16);
        write_mask = LFB_WRITE_DEPTH; count = 2;
        break;

    default:
        /* Unknown format — silently ignore (matches 86Box fatal() path
         * which we replace with a no-op for robustness). */
        return;
    }

    /* -----------------------------------------------------------------------
     * Compute pixel position — Banshee/V3 address encoding.
     *
     * Ported from 86Box voodoo_fb_writel() Banshee branch (vid_voodoo_fb.c):
     *   x = addr & 0xffe          bits[11:1]  — byte X within row
     *   y = (addr >> 12) & 0x3ff  bits[21:12] — row index
     *
     * The old Voodoo1/2 encoding (x = addr & 0x7fe, y = addr >> 11) is
     * WRONG for Banshee: it only allows 11-bit X (max 1023 pixels wide) and
     * the bit ranges overlap, corrupting Y for any resolution > 1024 wide.
     *
     * For single-pixel 32-bit formats (ARGB8888 / XRGB8888 / depth+colour)
     * the caller has already shifted addr >>= 1 to convert the 32-bit pixel
     * address to a 16-bit one, so the decode below is uniform across formats.
     * ----------------------------------------------------------------------- */
    x = (int)(addr & 0xffeu);           /* byte X offset within row (always even) */
    y = (int)((addr >> 12) & 0x3ffu);  /* row index */

    /* -----------------------------------------------------------------------
     * Write-buffer selection from lfbMode bits[5:4] (LFB_WRITE_MASK).
     *
     * Ported from 86Box:
     *   case LFB_WRITE_FRONT (0x00): fb_write_offset = front_offset
     *   case LFB_WRITE_BACK  (0x10): fb_write_offset = draw_offset  (back buf)
     *   default:                     fb_write_offset = front_offset
     *
     * Previously QEMU always used draw_offset, which was wrong when the Glide
     * driver selects the front buffer for direct LFB writes (e.g. 2D overlays).
     * ----------------------------------------------------------------------- */
    uint32_t fb_write_offset;
    switch (s->lfbMode & 0x30u) {
    case 0x10:  fb_write_offset = p->draw_offset;  break;  /* back  */
    default:    fb_write_offset = p->front_offset; break;  /* front */
    }

    /* Dirty-line tracking — front-buffer writes must update the display */
    if (fb_write_offset == p->front_offset && y < V3_DIRTY_LINES)
        s->dirty_line[y] = 1;

    /* Address computation — tiled or linear (86Box col_tiled / aux_tiled) */
    if (p->col_tiled)
        write_addr = fb_write_offset
            + (uint32_t)(x & 127) + (uint32_t)(x >> 7) * 128u * 32u
            + (uint32_t)(y & 31) * 128u + (uint32_t)(y >> 5) * p->row_width;
    else
        write_addr = fb_write_offset + (uint32_t)x + (uint32_t)y * p->row_width;

    if (p->aux_tiled)
        write_addr_aux = p->aux_offset
            + (uint32_t)(x & 127) + (uint32_t)(x >> 7) * 128u * 32u
            + (uint32_t)(y & 31) * 128u + (uint32_t)(y >> 5) * p->aux_row_width;
    else
        write_addr_aux = p->aux_offset + (uint32_t)x + (uint32_t)y * p->aux_row_width;

    /* -----------------------------------------------------------------------
     * Per-pixel pipeline (lfbMode bit 8 = pipeline-enable).
     * When set: run full Stipple / Depth / Chroma / Alpha test + blend.
     * When clear: raw write (no tests).
     * Ported from 86Box voodoo_fb_writel() if (voodoo->lfbMode & 0x100) branch.
     * ----------------------------------------------------------------------- */
    if (s->lfbMode & 0x100u) {

        bool stipple_en   = !!(p->fbzMode & FBZ_STIPPLE);
        bool stipple_patt = !!(p->fbzMode & FBZ_STIPPLE_PATT);
        bool depth_en     = !!(p->fbzMode & FBZ_DEPTH_ENABLE);
        bool chroma_en    = !!(p->fbzMode & FBZ_CHROMAKEY);
        bool alpha_en     = !!(p->alphaMode & ALPHA_ENABLE);
        bool blend_en     = !!(p->alphaMode & ALPHA_BLEND_EN);
        bool fog_en       = !!(p->fogMode & FOG_ENABLE);
        bool dither_en    = !!(p->fbzMode & FBZ_DITHER);
        bool dither_2x2   = !!(p->fbzMode & FBZ_DITHER_2X2);
        bool rgb_wmask    = !!(p->fbzMode & FBZ_RGB_WMASK);
        bool depth_wmask  = !!(p->fbzMode & FBZ_DEPTH_WMASK);
        int  depth_op     = (int)((p->fbzMode >> FBZ_DEPTH_OP_SHIFT) & 7);

        /* Stipple rolling state — LFB writes use the shared stipple register.
         * (86Box uses voodoo->params.stipple directly with <<1 rotate.) */
        uint32_t stipple_reg = p->stipple;

        for (int c = 0; c < count; c++) {
            int wr = col_r[c], wg = col_g[c], wb = col_b[c];
            int wa = alpha_data[c];
            uint16_t new_depth = depth_data[c];

            /* --- Stipple --- */
            if (stipple_en) {
                if (stipple_patt) {
                    /* Pattern mode: bit index from (y & 3, ~(x/2+c) & 7) */
                    int idx = ((y & 3) << 3) | (~((x >> 1) + c) & 7);
                    if (!(p->stipple & (1u << idx)))
                        goto skip_fb_pixel;
                } else {
                    /* Rotating mode: consume one bit per pixel */
                    stipple_reg = (stipple_reg << 1) | (stipple_reg >> 31);
                    if (!(stipple_reg & 0x80000000u))
                        goto skip_fb_pixel;
                }
            }

            /* --- Depth test --- */
            if (depth_en) {
                uint16_t old_d = *(const uint16_t *)(s->fb_mem +
                                  ((write_addr_aux) & (uint32_t)fb_mask));
                uint16_t test_d = (p->fbzMode & FBZ_DEPTH_SOURCE)
                                  ? (uint16_t)(p->zaColor & 0xffffu)
                                  : new_depth;
                if (!depth_test(depth_op, test_d, old_d))
                    goto skip_fb_pixel;
            }

            /* --- Chroma key --- */
            if (chroma_en &&
                wr == (int)p->chromaKey_r &&
                wg == (int)p->chromaKey_g &&
                wb == (int)p->chromaKey_b)
                goto skip_fb_pixel;

            /* --- Fog (uses new_depth as W proxy, ia=alpha for alpha-fog) --- */
            if (fog_en) {
                int32_t z_proxy  = (int32_t)new_depth << 12;
                int32_t ia_proxy = wa << 12;
                int64_t w_proxy  = new_depth;
                apply_fog(&wr, &wg, &wb, z_proxy, ia_proxy, w_proxy, p);
            }

            /* --- Alpha test --- */
            if (alpha_en) {
                int   afunc = (int)ALPHA_FUNC(p->alphaMode);
                int   aref  = (int)ALPHA_REF(p->alphaMode);
                bool  pass;
                switch (afunc) {
                case 0: pass = false;        break;
                case 1: pass = wa < aref;    break;
                case 2: pass = wa == aref;   break;
                case 3: pass = wa <= aref;   break;
                case 4: pass = wa > aref;    break;
                case 5: pass = wa != aref;   break;
                case 6: pass = wa >= aref;   break;
                default: pass = true;        break;
                }
                if (!pass) goto skip_fb_pixel;
            }

            /* --- Alpha blend --- */
            if (blend_en) {
                /* Read destination pixel for blend */
                uint16_t dst_raw = *(const uint16_t *)(s->fb_mem +
                                    ((write_addr) & (uint32_t)fb_mask));
                uint8_t dest_r = (uint8_t)(((dst_raw >> 11) & 0x1fu) * 255u / 31u);
                uint8_t dest_g = (uint8_t)(((dst_raw >>  5) & 0x3fu) * 255u / 63u);
                uint8_t dest_b = (uint8_t)((dst_raw & 0x1fu) * 255u / 31u);
                alpha_blend(&wr, &wg, &wb, wa, dest_r, dest_g, dest_b, 0xff,
                            p->alphaMode, wr, wg, wb);
            }

            /* --- Dither & write colour --- */
            if (rgb_wmask) {
                if (write_mask & (LFB_WRITE_COLOUR | LFB_WRITE_BOTH)) {
                    int pr, pg, pb;
                    if (dither_en) {
                        int px = (x >> 1) + c, py = y;
                        if (dither_2x2) {
                            pr = dither_rb2x2[wr][py & 1][px & 1];
                            pg = dither_g2x2 [wg][py & 1][px & 1];
                            pb = dither_rb2x2[wb][py & 1][px & 1];
                        } else {
                            pr = dither_rb[wr][py & 3][px & 3];
                            pg = dither_g [wg][py & 3][px & 3];
                            pb = dither_rb[wb][py & 3][px & 3];
                        }
                    } else {
                        pr = wr >> 3; pg = wg >> 2; pb = wb >> 3;
                    }
                    uint16_t pix = (uint16_t)((pr << 11) | (pg << 5) | pb);
                    *(uint16_t *)(s->fb_mem + ((write_addr) & (uint32_t)fb_mask)) = pix;
                }
            }

            /* --- Write depth --- */
            if (depth_wmask) {
                if (write_mask & (LFB_WRITE_DEPTH | LFB_WRITE_BOTH)) {
                    *(uint16_t *)(s->fb_mem +
                        ((write_addr_aux) & (uint32_t)fb_mask)) = new_depth;
                }
            }

skip_fb_pixel:
            write_addr     += 2u;
            write_addr_aux += 2u;
        }

    } else {
        /* -------------------------------------------------------------------
         * Raw write path: no pipeline tests, no blending.
         * Ported from 86Box voodoo_fb_writel() else branch.
         * ------------------------------------------------------------------- */
        for (int c = 0; c < count; c++) {
            if (write_mask & (LFB_WRITE_COLOUR | LFB_WRITE_BOTH)) {
                /* Pack to RGB565 — no dither in raw path */
                int pr = col_r[c] >> 3;
                int pg = col_g[c] >> 2;
                int pb = col_b[c] >> 3;
                *(uint16_t *)(s->fb_mem +
                    ((write_addr) & (uint32_t)fb_mask)) =
                    (uint16_t)((pr << 11) | (pg << 5) | pb);
            }
            if (write_mask & (LFB_WRITE_DEPTH | LFB_WRITE_BOTH)) {
                *(uint16_t *)(s->fb_mem +
                    ((write_addr_aux) & (uint32_t)fb_mask)) = depth_data[c];
            }
            write_addr     += 2u;
            write_addr_aux += 2u;
        }
    }
}

#undef LFB_WRITE_COLOUR
#undef LFB_WRITE_DEPTH
#undef LFB_WRITE_BOTH
#undef EXP5
#undef EXP6
