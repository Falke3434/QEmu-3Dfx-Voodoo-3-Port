/*
 * QEMU 3Dfx Voodoo 3 — Internal device structure header
 *
 * Shared between voodoo3.c, voodoo3_render.c, voodoo3_texture.c,
 * voodoo3_display.c, and voodoo3_setup.c.
 *
 * Copyright (C) 2026 <your name here>
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_VOODOO3_INT_H
#define HW_DISPLAY_VOODOO3_INT_H

/* Maximum LOD level */
#ifndef V3_LOD_MAX
#define V3_LOD_MAX  8
#endif

#include "qemu/osdep.h"
#include "hw/pci/pci_device.h"
#include "ui/console.h"
#include "qemu/timer.h"
#include "qemu/thread.h"
#include "hw/display/voodoo3_texture.h"  /* voodoo3_tex_params_t etc.  */
#include "hw/display/voodoo3_render.h"    /* voodoo3_triangle, voodoo3_triangle_setup */
#include "hw/display/voodoo3_display.h"  /* V3_DIRTY_LINES             */

/* Floating-point/integer union used in reg decode and setup */
typedef union { uint32_t i; float f; } fi_t;

/* Forward declarations (full definitions below) */
typedef struct voodoo3_params_t voodoo3_params_t;
typedef struct Voodoo3State     Voodoo3State;

/* voodoo3_queue_triangle() is defined in voodoo3.c but called from
 * voodoo3_setup.c — declare it here so all .c files can see it. */
void voodoo3_queue_triangle(Voodoo3State *s, voodoo3_params_t *p);

/* =========================================================================
 * Device variant constants (model property values)
 * ========================================================================= */
#define VOODOO3_MODEL_BANSHEE    0u
#define VOODOO3_MODEL_V3_1000    1u
#define VOODOO3_MODEL_V3_2000    2u
#define VOODOO3_MODEL_V3_3000    3u
#define VOODOO3_MODEL_V3_3500TV  4u

#define VOODOO3_CLUT_SIZE    256
#define PARAM_BUF_SIZE       256
#define MAX_RENDER_THREADS   4

typedef struct voodoo3_tmu_params_t {
    int64_t  startS, startT, startW;
    int64_t  dSdX, dTdX, dWdX;
    int64_t  dSdY, dTdY, dWdY;
    uint32_t textureMode;
    uint32_t texBaseAddr;
    int      tformat;
    int      lod_min, lod_max;
} voodoo3_tmu_params_t;

/* =========================================================================
 * Triangle parameter set — equivalent to 86Box voodoo_params_t
 * ========================================================================= */
typedef struct voodoo3_params_t {
    /* Vertex positions (4.12 fixed-point, signed 16-bit) */
    int32_t  vertexAx, vertexAy;
    int32_t  vertexBx, vertexBy;
    int32_t  vertexCx, vertexCy;

    /* Per-pixel colour gradients */
    int32_t  startR, startG, startB, startA, startZ;
    int32_t  dRdX, dGdX, dBdX, dAdX, dZdX;
    int32_t  dRdY, dGdY, dBdY, dAdY, dZdY;

    /* Homogeneous W (FBI) */
    int64_t  startW;
    int64_t  dWdX, dWdY;

    /* Per-TMU texture gradients */
    voodoo3_tmu_params_t tmu[2];

    /* Render-state registers */
    uint32_t fbzColorPath;
    uint32_t fbzMode;
    uint32_t fogMode;
    uint32_t alphaMode;
    uint32_t lfbMode;
    uint32_t stipple;
    uint32_t color0, color1;
    uint32_t zaColor;
    uint32_t swapbufferCMD;
    int      sign;           /* triangle winding */

    /* Fog table */
    struct { uint8_t fog, dfog; } fogTable[64];
    struct { uint8_t r, g, b; }  fogColor;
    uint32_t chromaKey;
    uint8_t  chromaKey_r, chromaKey_g, chromaKey_b;

    /* Clip rectangles */
    int      clipLeft, clipRight, clipLowY, clipHighY;
    int      clipLeft1, clipRight1, clipLowY1, clipHighY1;

    /* Buffer layout (Banshee/V3 only) */
    uint32_t draw_offset, front_offset, aux_offset;
    uint32_t row_width, aux_row_width;
    int      col_tiled, aux_tiled;

    /* Stats */
    uint32_t fbiPixelsIn, fbiChromaFail, fbiZFuncFail;
    uint32_t fbiAFuncFail, fbiPixelsOut;

    /*
     * Texture geometry + decoded cache pointers.
     * voodoo3_use_texture() fills tex_ptr before the triangle is queued.
     * tex_params is populated by voodoo3_recalc_tex() when textureMode /
     * tLOD / texBaseAddr registers are written.
     */
    voodoo3_tex_params_t tex_params[2];     /* per-TMU LOD geometry     */
    uint32_t            *tex_ptr[2][V3_LOD_MAX + 1]; /* decoded cache ptrs */
} voodoo3_params_t;

/* =========================================================================
 * Setup vertex (for sBeginTriCMD / sDrawTriCMD path)
 * ========================================================================= */
typedef struct voodoo3_vert_t {
    float sVx, sVy, sVz, sWb;
    float sRed, sGreen, sBlue, sAlpha;
    float sW0, sS0, sT0;
    float sW1, sS1, sT1;
} voodoo3_vert_t;

/* =========================================================================
 * Banshee 2D blitter state (mirrors banshee_t BLT fields)
 * ========================================================================= */
typedef struct voodoo3_blt_t {
    /* ---- Raw registers (from Banshee 2D reg map) ---- */
    uint32_t clip0Min, clip0Max;
    uint32_t dstBaseAddr, dstFormat;
    uint32_t srcColorkeyMin, srcColorkeyMax;
    uint32_t dstColorkeyMin, dstColorkeyMax;
    uint32_t bresError0, bresError1;
    uint32_t rop;
    uint32_t srcBaseAddr, srcFormat;
    uint32_t commandExtra;
    uint32_t lineStipple, lineStyle;
    uint32_t pattern0, pattern1;
    uint32_t srcSize;           /* bits[12:0]=W, bits[28:16]=H */
    uint32_t srcXY;             /* bits[12:0]=X, bits[28:16]=Y */
    uint32_t colorBack, colorFore;
    uint32_t dstSize;           /* bits[12:0]=W, bits[28:16]=H */
    uint32_t dstXY;             /* bits[12:0]=X, bits[28:16]=Y */
    uint32_t command;
    /* ---- Tiling flags (bit[31] of base addr registers) ---- */
    bool     dstTiled, srcTiled;
    /* ---- Launch-pending flag (86Box launch_pending) ---- */
    bool     launch_pending;
    /* ---- Decoded geometry (updated by reg writes) ---- */
    int      dstX, dstY, srcX, srcY;
    int      dstW, dstH, srcW, srcH;
    uint32_t dstStride, srcStride;
    int      dstBpp, srcBpp;
    /* ---- H2S pixel accumulation (host-to-screen BLT) ---- */
    uint8_t  host_data[8192];   /* per-row accumulation buffer */
    int      host_data_count;   /* bytes accumulated so far    */
    int      src_stride_dest;   /* source bytes per row        */
    int      cur_y;             /* current destination row     */
} voodoo3_blt_t;

/* =========================================================================
 * Main device state
 * ========================================================================= */
struct Voodoo3State {
    PCIDevice   parent_obj;         /* MUST be first */

    /* BARs */
    MemoryRegion mmio, lfb, io;

    /* Display */
    QemuConsole  *con;
    QEMUTimer    *vblank_timer;
    bool          in_vblank;
    int           screen_width, screen_height;
    bool          display_enabled;
    int           pix_format;

    /* SGRAM */
    uint8_t      *fb_mem;
    uint32_t      fb_size;

    /* Desktop surface */
    uint32_t      desktop_start, desktop_stride;
    bool          desktop_tiled;
    uint32_t      tile_base, tile_stride, tile_x;
    int           y_origin_swap;

    /* Hardware cursor */
    bool          cursor_ena;
    uint32_t      cur_pat_addr;
    int           cur_x, cur_y;
    uint32_t      cur_c0, cur_c1;
    /*
     * Shadow copy of the cursor bitmap, kept in host byte order.
     * 64 rows × 16 bytes = 1024 bytes.
     * Populated by voodoo3_lfb_write() when the write address falls within
     * [cur_pat_addr, cur_pat_addr+1024).  This avoids reading directly from
     * fb_mem where 32-bit writes from a big-endian CPU have been byte-swapped
     * by QEMU's DEVICE_LITTLE_ENDIAN memory region, corrupting the 1bpp mask.
     */
    uint8_t       cursor_buf[1024];

    /* CLUT */
    uint32_t      pallook[VOODOO3_CLUT_SIZE];
    int           dacAddr;

    /* Ext registers (Init/PLL/DAC/Video — named as in 86Box banshee_t) */
    uint32_t pciInit0, lfbMemoryConfig;
    uint32_t miscInit0, miscInit1;
    uint32_t dramInit0, dramInit1, agpInit0;
    uint32_t vgaInit0, vgaInit1;
    uint32_t pllCtrl0, pllCtrl1, pllCtrl2;
    uint32_t dacMode;
    uint32_t vidProcCfg, vidScreenSize;
    uint32_t vidDesktopStartAddr, vidDesktopOverlayStride;
    uint32_t vidChromaKeyMin, vidChromaKeyMax;
    uint32_t hwCurPatAddr, hwCurLoc, hwCurC0, hwCurC1;
    uint32_t intrCtrl;
    uint32_t command_2d, srcBaseAddr_2d;

    /* Generic register scratch (for misc unmapped regs) */
    uint32_t regs[512];

    /* Banshee CRTC / DAC indexed-register scratch (ext offsets 0xc0–0xda).
     * 64 CRTC ctrl entries (index via 0xd4, value via 0xd5) and
     * 64 frequency entries (index via 0xc4, value via 0xc5). */
    uint8_t  crtc_ctrl[64];     /* written via Ext_crtcCtrlIdx/Val pair  */
    uint8_t  crtc_freq[64];     /* written via Ext_crtcDoubleRate/Val     */
    uint8_t  dac_reset[64];     /* written via Ext_dacResetIdx/Val        */
    uint32_t crtc_idx;          /* current crtcCtrl index                 */
    uint32_t crtc_freq_idx;     /* current crtcFreq index                 */
    uint32_t dac_reset_idx;     /* current dacReset index                 */
    uint32_t vidSerialParallelPort;

    /* --- 3D state -------------------------------------------------------- */
    voodoo3_params_t params;        /* current triangle parameter set */
    uint32_t         lfbMode;       /* lfbMode register */
    int              rgb_sel;       /* fbzColorPath bits [1:0] */

    /* Setup-engine vertex buffer (sBeginTriCMD / sDrawTriCMD) */
    voodoo3_vert_t  verts[4];       /* [3] = staging, [0..2] = triangle */
    int             vertex_num;
    int             vertex_next_age;
    int             vertex_ages[3];
    int             num_verticies;
    int             cull_pingpong;
    uint32_t        sSetupMode;

    /* NCC dirty flags */
    int             ncc_dirty[2];

    /* Blitter state */
    voodoo3_blt_t   blt;

    /* Frame / buffer counters */
    int             cmd_written, cmd_read;
    int             tri_count;
    uint32_t        fbiPixelsIn, fbiChromaFail, fbiZFuncFail;
    uint32_t        fbiAFuncFail, fbiPixelsOut;
    bool            voodoo_busy;

    /* --- FIFO command ring ----------------------------------------------- */
#define V3_FIFO_SIZE 65536          /* entries, must be power of 2 */
    uint32_t  fifo_cmd[V3_FIFO_SIZE];
    uint32_t  fifo_val[V3_FIFO_SIZE];
    uint32_t  fifo_wr, fifo_rd;

    /* --- Triangle parameter ring buffer (one slot per render thread) ----- */
    voodoo3_params_t param_buf[PARAM_BUF_SIZE];
    uint32_t         param_wr;      /* written by FIFO thread */
    uint32_t         param_rd[MAX_RENDER_THREADS]; /* per-thread read pointer */

    /* --- Render threads -------------------------------------------------- */
    QemuThread   render_thread[MAX_RENDER_THREADS];
    QemuMutex    render_lock;
    QemuCond     render_cond;       /* wakes all render threads */
    QemuMutex    fifo_lock;
    QemuCond     fifo_cond;         /* wakes FIFO worker */
    bool         render_stop;
    uint32_t     render_threads_count;

    /* Device variant */
    uint32_t model;
    bool     is_agp, bilinear, dac_filter;

    /* PPC big-endian swap */

    /* --- Texture subsystem (ported from 86Box voodoo_t) ----------------- */

    /* Texture RAM: 4 MB per TMU (Voodoo 3 shares 8 MB SGRAM, split 4+4) */
    uint8_t             *tex_mem[2];
    uint32_t             tex_mem_size;   /* bytes per TMU = V3_TEX_MEM_SIZE   */
    uint32_t             tex_mask;       /* tex_mem_size - 1                  */

    /* Decoded texture cache (V3_TEX_CACHE_SIZE slots per TMU) */
    v3_tex_cache_entry_t tex_cache[2][V3_TEX_CACHE_SIZE];
    uint32_t             tex_lru[2];     /* simple eviction counter           */

    /* ARGB palette (256 entries per TMU) used for PAL8 / APAL8 / APAL88 */
    uint32_t             tex_palette[2][256];

    /* --- NCC (YIQ) table state (ported from 86Box nccTable / ncc_lookup) - */
    struct {
        uint32_t y[4];   /* Y luma  — 4 packed 8-bit entries per word */
        uint32_t i[4];   /* I chroma — 9-bit signed fields             */
        uint32_t q[4];   /* Q chroma — 9-bit signed fields             */
    } ncc_table[2][2];   /* [tmu][table_select]                        */
    uint32_t             ncc_lookup[2][2][256]; /* decoded ABGR32       */

    /* --- Dirty-line tracking (ported from 86Box dirty_line[]) ----------- */
    uint8_t              dirty_line[V3_DIRTY_LINES];

    /* --- Buffer swap state (ported from 86Box swap_pending etc.) -------- */
    bool                 swap_pending;
    int                  swap_interval;   /* vblanks to wait              */
    uint32_t             swap_offset;     /* draw_offset to flip to       */
    int                  retrace_count;   /* vblanks since swap requested */
    uint32_t             frame_count;     /* total frames rendered         */
};



/* Internal function declared in voodoo3.c, used by voodoo3_setup.c */

/* voodoo3_queue_triangle — submit a triangle to the render ring.
 * Defined as static in voodoo3.c but declared here for setup.c. */

#endif /* HW_DISPLAY_VOODOO3_INT_H */
