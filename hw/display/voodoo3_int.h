/*
 * QEMU 3Dfx Voodoo 3 — internal device structure header
 *
 * Shared between voodoo3.c, voodoo3_render.c, voodoo3_texture.c,
 * voodoo3_display.c, and voodoo3_setup.c.
 *
 * QEMU port: https://github.com/Falke3434/QEmu-3Dfx-Voodoo-3-Port
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_VOODOO3_INT_H
#define HW_DISPLAY_VOODOO3_INT_H

/* -----------------------------------------------------------------------
 * I2C / DDC — uses QEMU-native bitbang_i2c + I2CDDC infrastructure,
 * identical approach to hw/display/ati.c.
 * ----------------------------------------------------------------------- */
#include "hw/i2c/bitbang_i2c.h"
#include "hw/display/i2c-ddc.h"

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
#include "hw/display/voodoo3_gaps.h"     /* gap tracker (voodoo3_note_gap) */
#include "hw/display/vga_int.h"          /* legacy VGA core (optional)  */

/* Floating-point/integer union used in reg decode and setup */
typedef union { uint32_t i; float f; } fi_t;

/* Forward declarations (full definitions below) */
typedef struct voodoo3_params_t voodoo3_params_t;
typedef struct Voodoo3State     Voodoo3State;

/* voodoo3_queue_triangle() is defined in voodoo3.c but called from
 * voodoo3_setup.c — declare it here so all .c files can see it. */
void voodoo3_queue_triangle(Voodoo3State *s, voodoo3_params_t *p);

/* Reload the hardware-cursor pattern from SGRAM into cursor_buf. */
void voodoo3_cursor_reload(Voodoo3State *s);

/* Block until the render threads have finished every queued triangle. */
void voodoo3_wait_render_idle(Voodoo3State *s);

/* =========================================================================
 * Device variant constants (model property values)
 * ========================================================================= */
#define VOODOO3_MODEL_BANSHEE    0u
#define VOODOO3_MODEL_V3_1000    1u
#define VOODOO3_MODEL_V3_2000    2u
#define VOODOO3_MODEL_V3_3000    3u
#define VOODOO3_MODEL_V3_3500TV  4u

#define VOODOO3_CLUT_SIZE    512   /* 2 x 256: desktop/overlay CLUT (86Box pallook[512]) */
#define PARAM_BUF_SIZE       256
#define MAX_RENDER_THREADS   4

/* =========================================================================
 * vidProcCfg register bit definitions
 * Shared by voodoo3.c and voodoo3_display.c — defined here so both can use
 * them without the display module needing to include the main voodoo3.c.
 * Values from 3Dfx Voodoo3/Banshee register specification.
 * ========================================================================= */
#define VIDPROCCFG_VIDPROC_ENABLE        (1u <<  0)
#define VIDPROCCFG_CURSOR_MODE           (1u <<  1)  /* 0=Win AND/XOR, 1=X11 */
#define VIDPROCCFG_OVERLAY_ENABLE        (1u <<  8)
#define VIDPROCCFG_OVERLAY_CLUT_BYPASS   (1u << 11)
#define VIDPROCCFG_DESKTOP_CLUT_SEL      (1u << 12)  /* desktop uses pallook[256..511] */
#define VIDPROCCFG_OVERLAY_CLUT_SEL      (1u << 13)
#define VIDPROCCFG_H_SCALE_ENABLE        (1u << 14)
#define VIDPROCCFG_V_SCALE_ENABLE        (1u << 15)
#define VIDPROCCFG_FILTER_MODE_MASK      (3u << 16)
#define VIDPROCCFG_FILTER_MODE_POINT     (0u << 16)
#define VIDPROCCFG_FILTER_MODE_DITHER4X4 (1u << 16)
#define VIDPROCCFG_FILTER_MODE_DITHER2X2 (2u << 16)
#define VIDPROCCFG_FILTER_MODE_BILINEAR  (3u << 16)
#define VIDPROCCFG_DESKTOP_PIX_FMT(v)   (((v) >> 18) & 7u)
#define VIDPROCCFG_OVERLAY_PIX_FMT(v)   (((v) >> 21) & 7u)
#define VIDPROCCFG_DESKTOP_TILE          (1u << 24)
#define VIDPROCCFG_OVERLAY_TILE          (1u << 25)
#define VIDPROCCFG_HWCURSOR_ENA          (1u << 27)

/* Overlay pixel format values (from VIDPROCCFG_OVERLAY_PIX_FMT field) */
#define OVERLAY_FMT_565         1
#define OVERLAY_FMT_YUYV422     5
#define OVERLAY_FMT_UYVY422     6
#define OVERLAY_FMT_565_DITHER  7

/* vidDesktopOverlayStride register field masks */
#define VID_STRIDE_DESKTOP_MASK   0x00007fffu          /* bits[14:0]  */
#define VID_STRIDE_OVERLAY_SHIFT  16
#define VID_STRIDE_OVERLAY_MASK   (0x7fffu << VID_STRIDE_OVERLAY_SHIFT)

typedef struct voodoo3_tmu_params_t {
    int64_t  startS, startT, startW;
    int64_t  dSdX, dTdX, dWdX;
    int64_t  dSdY, dTdY, dWdY;
    uint32_t textureMode;
    uint32_t texBaseAddr;
    uint32_t texBaseAddr1;   /* second LOD base (LOD_SPLIT+ODD+MULTIBASEADDR) */
    uint32_t texBaseAddr2;   /* third  LOD base */
    uint32_t texBaseAddr38;  /* fourth LOD base */
    uint32_t tLOD;           /* tLOD register snapshot (lod_min/max/bias)    */
    int      lodbias;        /* signed 6-bit LOD bias decoded from tLOD[17:12] */
    int      tformat;
    int      lod_min, lod_max;
} voodoo3_tmu_params_t;

/* =========================================================================
 * Triangle parameter set — equivalent to 86Box voodoo_params_t
 * ========================================================================= */
typedef struct voodoo3_params_t {
    /* Vertex positions (12.4 fixed-point, signed 16-bit) */
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
    /* Raw register values for colBufferStride / auxBufferStride readback.
     * row_width / aux_row_width are transformed (tiled * 128 * 32) and
     * cannot be reconstructed, so we keep the original write value here. */
    uint32_t col_stride_raw, aux_stride_raw;

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
    /*
     * Texture-cache slot referenced by this triangle per TMU (-1 = none).
     * Render threads use it to release their reference (refcount_r[]) once
     * the triangle is finished, so the slot cannot be recycled underneath
     * a thread that is still sampling from it.
     */
    int8_t               tex_slot[2];

    /*
     * Detail-texture parameters — decoded from the tDetail register (0x308).
     * Ported from 86Box voodoo_params_t: detail_max[], detail_bias[], detail_scale[].
     *
     * detail_max[tmu]   = tDetail[7:0]   — clamp ceiling (0..255)
     * detail_bias[tmu]  = tDetail[13:8]  — LOD subtrahend (0..63)
     * detail_scale[tmu] = tDetail[16:14] — left-shift amount (0..7)
     *
     * Used in tc_mselect / tca_mselect = 4 (detail) of the TMU combine unit:
     *   factor = clamp((detail_bias - lod) << detail_scale, 0, detail_max)
     */
    int detail_max[2];
    int detail_bias[2];
    int detail_scale[2];
} voodoo3_params_t;

/*
 * Does TMU0's combine unit read the output of TMU1 (texture unit upstream of
 * it)?  Only when it does not discard the "other" input completely: RGB and
 * alpha zero_other both set (the usual "TMU0 samples its own texture" setup,
 * textureMode & 0x00201000 == 0x00201000) and neither mselect picks the
 * other TMU's alpha.  Used both to decide whether a texture has to be bound
 * for TMU1 (voodoo3_queue_triangle) and whether TMU1 has to be sampled
 * (rasterizer); the two must agree.
 */
static inline bool voodoo3_tmu1_needed(uint32_t tm0)
{
    bool tc_zero_other  = !!(tm0 & (1u << 12));
    bool tca_zero_other = !!(tm0 & (1u << 21));
    unsigned tc_msel    = (tm0 >> 14) & 7;
    unsigned tca_msel   = (tm0 >> 23) & 7;

    return !tc_zero_other || !tca_zero_other || tc_msel == 2 || tca_msel == 2;
}

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
/*
 * Clip rectangle — ported from 86Box banshee_blt clip_t.
 * clip[0] = clip0 (registers 0x08/0x0c), clip[1] = clip1 (0x4c/0x50).
 * COMMAND_CLIP_SEL (bit 23) selects which rect is active.
 */
typedef struct {
    int x_min, y_min, x_max, y_max;
} v3_clip_t;

typedef struct voodoo3_blt_t {
    /* ---- Raw registers (from Banshee 2D reg map) ---- */
    uint32_t clip0Min, clip0Max;
    uint32_t clip1Min, clip1Max;
    uint32_t dstBaseAddr, dstFormat;
    uint32_t srcColorkeyMin, srcColorkeyMax;
    uint32_t dstColorkeyMin, dstColorkeyMax;
    uint32_t bresError0, bresError1;
    uint32_t rop;
    uint32_t srcBaseAddr, srcFormat;
    uint32_t commandExtra;
    uint32_t lineStipple, lineStyle;
    uint32_t srcSize;           /* bits[12:0]=W, bits[28:16]=H */
    uint32_t srcXY;             /* bits[12:0]=X, bits[28:16]=Y */
    uint32_t colorBack, colorFore;
    uint32_t dstSize;           /* bits[12:0]=W, bits[28:16]=H */
    uint32_t dstXY;             /* bits[12:0]=X, bits[28:16]=Y */
    uint32_t command;

    /* ---- Clip rectangles (decoded, 86Box banshee_blt clip[2]) ---- */
    v3_clip_t clip[2];          /* [0]=clip0, [1]=clip1 */

    /* ---- Tiling flags (bit[31] of base addr registers) ---- */
    bool     dstTiled, srcTiled;
    uint32_t dstBaseAddr_tiled; /* non-zero if tiled (mirrors bit31 of dstBaseAddr raw) */
    uint32_t srcBaseAddr_tiled;

    /* ---- Launch-pending flag (86Box launch_pending) ---- */
    bool     launch_pending;

    /* ---- Decoded geometry (updated by reg writes) ---- */
    int      dstX, dstY, srcX, srcY;
    int      old_srcX;          /* saved srcX at launch (86Box) */
    int      dstSizeX, dstSizeY;
    int      srcSizeX, srcSizeY;
    uint32_t dst_stride;        /* destination stride in bytes (computed) */
    uint32_t src_stride;        /* source stride in bytes (computed) */
    int      dstBpp, srcBpp;
    int      src_bpp;           /* source bits-per-pixel (86Box blt.src_bpp) */

    /* ---- ROP array (86Box banshee_blt rops[4]) ---- */
    uint8_t  rops[4];           /* [0]=main ROP, [1..3]=colorkey ROP variants */

    /* ---- Pattern state (86Box banshee_blt colorPattern*) ---- */
    /*
     * colorPattern[64]: 8×8 32-bit pixel pattern (256 bytes).
     * Decoded views for faster 8/16/24-bpp access:
     *   colorPattern8[64]:  8×8 bytes
     *   colorPattern16[64]: 8×8 uint16
     *   colorPattern24[64]: 8×8 uint32 (low 24 bits used)
     * patoff_x/y: pattern offset from command register bits[20:17].
     */
    uint32_t colorPattern[64];
    uint32_t colorPattern24[64];
    uint16_t colorPattern16[64];
    uint8_t  colorPattern8[64];
    int      patoff_x, patoff_y;

    /* ---- H2S / stretch-blt pixel accumulation ---- */
    uint8_t  host_data[8192];   /* per-row accumulation buffer */
    int      host_data_count;   /* bytes accumulated so far    */
    int      host_data_remainder;
    int      host_data_size_src;  /* source row bytes (for stretch) */
    int      host_data_size_dest; /* dest row bytes */
    int      src_stride_src;      /* src stride for stretch src */
    int      src_stride_dest;     /* src stride for stretch dest */
    int      cur_x, cur_y;        /* current pixel/row position */

    /* ---- Bresenham error terms (decoded from bresError0/1) ---- */
    int      bres_error_0;      /* Y stretch error accumulator */

    /* ---- Line drawing state (86Box banshee_blt line_*) ---- */
    int      line_rep_cnt;      /* lineStyle[7:0]:  pixel repeat count */
    int      line_bit_mask_size;/* lineStyle[12:8]: stipple pattern size */
    int      line_pix_pos;      /* lineStyle[23:16]: current pixel pos */
    int      line_bit_pos;      /* lineStyle[28:24]: current bit pos */

    /* ---- Polyfill state (86Box banshee_blt lx/rx/ly/ry) ---- */
    int      lx[2], ly[2];     /* left  edge start/end vertices */
    int      rx[2], ry[2];     /* right edge start/end vertices */
    int      lx_cur, rx_cur;   /* current X intercepts */
    int      dx[2], dy[2];     /* deltas for left/right edges */
    int      x_inc[2];         /* X step direction for each edge */
    int      error[2];         /* Bresenham errors for each edge */
} voodoo3_blt_t;

/* =========================================================================
 * Main device state
 * ========================================================================= */
struct Voodoo3State {
    PCIDevice   parent_obj;         /* MUST be first */


    /* BARs */
    MemoryRegion mmio, io;
    /*
     * BAR1: Linear Framebuffer — RAM-backed device region.
     *
     * lfb_ram is mapped directly onto fb_mem via
     * memory_region_init_ram_device_ptr().  Guest reads/writes land in
     * fb_mem without a MMIO trap per access (same pattern as ATI
     * linear_aper in hw/display/ati.c).
     *
     * cmdfifo_mmio is a small MMIO subregion overlaid on lfb_ram at
     * whatever byte-offset the driver programmed into CMDFIFO_BASE_ADDR0.
     * It is repositioned at runtime via voodoo3_cmdfifo_reposition()
     * whenever the driver changes the ring-buffer location or size.
     * Priority 1 ensures it shadows the underlying RAM for those pages.
     */
    MemoryRegion lfb_bar;          /* BAR1 container (32 MiB)             */
    MemoryRegion lfb_ram;          /* SGRAM, RAM-backed (fb_size)         */
    MemoryRegion lfb_alias;        /* upper BAR1 half mirrors the SGRAM   */
    MemoryRegion cmdfifo_mmio;     /* MMIO overlay for CMDFIFO ring window */
    bool         cmdfifo_mmio_active; /* true when subregion is added      */
    /*
     * Tiled LFB aperture: MMIO overlay on BAR1 from tile_base to the end of
     * SGRAM.  CPU accesses in that window are translated with the
     * lfbMemoryConfig tile geometry (86Box banshee_read/write_linear).
     */
    MemoryRegion lfb_tiled_mmio;
    bool         lfb_tiled_active;
    uint32_t     lfb_tiled_base;   /* start offset of the active overlay */
    /* BAR0 3D-LFB window (0x1000000..0x1ffffff) with 8/16/32-bit access */
    MemoryRegion lfb3d_mmio;

    /* Native Voodoo3 display console (index 1, used after driver init) */
    QemuConsole  *con;
    QEMUTimer    *vblank_timer;
    bool          in_vblank;
    int           screen_width, screen_height;
    bool          display_enabled;
    /*
     * SDL-safe deferred resize.
     *
     * qemu_console_resize() / dpy_gfx_replace_surface() must be called from
     * the QEMU main thread and must not race an in-progress blit.  Under
     * -display sdl this is enforced by scheduling a Bottom Half instead of
     * calling qemu_console_resize() directly from MMIO-write paths or from
     * inside voodoo3_update_display_dirty().
     *
     * resize_bh      – BH handle allocated in realize, freed in unrealize.
     * resize_pending_w/h – dimensions requested; 0 = no resize pending.
     */
    QEMUBH       *resize_bh;
    int           resize_pending_w;
    int           resize_pending_h;
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
    bool          cur_loc_valid;  /* true after first hwCurLoc write */
    uint32_t      cur_pat_addr;
    int           cur_x, cur_y;
    int           cur_yoff;     /* sprite rows skipped when cursor clips above screen top */
    uint32_t      cur_c0, cur_c1;
    /*
     * Copy of the cursor bitmap (64 rows x 16 bytes), in the byte order the
     * chip sees.  Reloaded from SGRAM by voodoo3_cursor_reload() (pattern
     * address change, display refresh), undoing the LFB swizzle.
     *
     * Default (reset): plane0=0xFF, plane1=0x00 per row = fully transparent
     * in Windows AND/XOR mode (p0=1,p1=0 → skip).  This prevents a coloured
     * rectangle at (0,0) between cursor enable and the first shape write.
     */
    uint8_t       cursor_buf[1024];

    /* Last state seen by the display refresh (dirty-row bookkeeping) */
    bool          last_cur_vis;
    int           last_cur_x, last_cur_y, last_cur_yoff;
    uint32_t      last_cur_c0, last_cur_c1, last_vidproccfg;
    uint8_t       last_cursor_buf[1024];
    uint32_t      last_front_offset, last_row_width;
    int           last_col_tiled, last_pix_format;
    bool          full_redraw;
    uint32_t      leftOverlayBuf;  /* SST 0x250: buffer to show on swap      */
    uint32_t      overlay_addr;    /* current overlay scan-out address       */
    bool          ext_from_io;     /* ext write arrives via BAR2 (bytes)     */
    uint32_t      chromaRange;     /* SST 0x138 (stored only, range test not implemented) */
    bool          dac_pend;        /* BAR0 dacData write not yet committed   */
    uint32_t      dac_pend_val;
    /*
     * Pixel byte order in SGRAM (port convention: SGRAM holds pixels the way
     * the CPU wrote them).  Derived from the LFB swizzle bits in miscInit0:
     * Both flags are sticky (set once the driver uses the bit), because
     * AmigaOS and MorphOS switch the bits per access (see
     * voodoo3_update_lfb_swizzle()).  lfb_be32 starts true on big-endian
     * targets.
     *  - 16 bpp big-endian: bit 31 (AmigaOS 16-bit: 0xC0000000).  MorphOS
     *    uses RGB16PC (little-endian) and only ever sets bit 30.
     *  - 32 bpp big-endian: bit 30.  x86 Windows never sets it -> LE.
     */
    bool          lfb_be32;
    bool          lfb_be16;   /* sticky: miscInit0 bit 31 has been used */
    uint32_t      disp_gen;        /* bumped by display-affecting reg writes */
    uint32_t      last_disp_gen;

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

    /*
     * Screen-filter (scrfilter) state — ported from 86Box voodoo_t.scrfilter*
     * and voodoo_generate_vb_filters() in vid_voodoo_banshee.c.
     *
     * scrfilter_enabled : true when Video_maxRgbDelta > 0.
     * scrfilter_threshold : raw 24-bit RGB delta value (R<<16|G<<8|B).
     * scrfilter_threshold_old : previous value; table regenerated on change.
     *
     * vb_filter_v1_rb/g  : 4×1 / 2×2 filter LUT (256×256, per-channel).
     * vb_filter_bx_rb/g  : box pre-filter LUT    (256×256, per-channel).
     * purpleline         : per-channel scanline tint (256 entries × 3 ch).
     *
     * These are only allocated/populated when scrfilter_enabled is true.
     * All four 256×256 tables are ~256 KB total; kept as flat arrays for
     * direct indexing identical to 86Box's static arrays.
     */
    bool     scrfilter_enabled;
    uint32_t scrfilter_threshold;
    uint32_t scrfilter_threshold_old;
    uint8_t  vb_filter_v1_rb[256][256];
    uint8_t  vb_filter_v1_g [256][256];
    uint8_t  vb_filter_bx_rb[256][256];
    uint8_t  vb_filter_bx_g [256][256];
    uint16_t purpleline[256][3];

    /*
     * Video overlay state — ported from 86Box voodoo_t.overlay and
     * banshee_t.overlay_pix_fmt / overlay_buffer.
     *
     * Source: banshee_overlay_draw() in vid_voodoo_banshee.c,
     *         voodoo_t.overlay in vid_voodoo_common.h.
     */
    struct {
        /* Raw register storage */
        uint32_t vidOverlayStartCoords;
        uint32_t vidOverlayEndScreenCoords;
        uint32_t vidOverlayDudx;               /* X step, 20.12 fixed-point */
        uint32_t vidOverlayDudxOffsetSrcWidth;
        uint32_t vidOverlayDvdy;               /* Y step, 20.12 fixed-point */
        uint32_t vidOverlayDvdyOffset;
        /* Decoded geometry */
        int      start_x, start_y;             /* screen top-left */
        int      end_x,   end_y;               /* screen bottom-right */
        int      size_x,  size_y;              /* display size (pixels) */
        int      overlay_bytes;               /* source row width in bytes */
        /* Vertical sub-pixel accumulator (20.12 fixed-point source Y) */
        /* Pixel format: OVERLAY_FMT_565 / YUYV422 / UYVY422 */
        int      pix_fmt;
        /* Enable flag (vidProcCfg bit 8) */
        bool     ena;
        /* Two-line decode buffers for bilinear filtering (86Box overlay_buffer[2][4096]) */
        uint32_t buf[2][4096];
    } ov;

    /*
     * VGA register state — ported from 86Box svga_t fields used by
     * banshee_in() / banshee_out() and svga_in() / svga_out().
     *
     * The Banshee/V3 exposes standard VGA I/O ports 0x3b0..0x3df via two paths:
     *   1. BAR2 I/O offsets 0xb0..0xdf (and with legacy-vga=on the fixed
     *      PC ports 0x3b0..0x3df).
     *   2. BAR0 IO-remap window offsets 0xb0..0xdf, reads and writes
     *      routed to voodoo3_vga_in/out() as in 86Box (banshee_ext_out/in()
     *      -> svga_out/in()).
     *
     * misc_out: Miscellaneous Output Register (0x3c2 write / 0x3cc read).
     *   bit 0 = I/O address select (0=3Bx, 1=3Dx)
     *   bit 1 = RAM enable
     *   bits 3:2 = clock select (for pllCtrl recalc)
     *   bit 5 = page select (odd/even)
     *   bit 6 = horizontal sync polarity
     *   bit 7 = vertical sync polarity
     * 86Box svga_t: svga->miscout, initialised to 1 (I/O select = 3Dx).
     *
     * feat_reg: Feature Control Register (0x3da write / 0x3ca read).
     *
     * seq_idx / seq_regs[8]: Sequencer index (0x3c4) and data (0x3c5).
     *   [0]=Reset, [1]=Clocking Mode, [2]=Map Mask, [3]=Char Map Sel, [4]=Mem Mode
     *
     * gr_idx / gr_regs[16]: Graphics Controller index (0x3ce) and data (0x3cf).
     *   [5]=Mode, [6]=Misc (map select, chain4, odd/even)
     *
     * ar_idx / ar_regs[32] / ar_flip_flop: Attribute Controller (0x3c0).
     *   The ATC shares one port for index and data, toggled by ar_flip_flop.
     *   Reading 0x3da resets ar_flip_flop to index mode.
     *
     * dac_pel_mask: DAC PEL mask (0x3c6), default 0xff.
     * dac_read_addr / dac_write_addr: DAC address registers (0x3c7 / 0x3c8).
     * dac_rgb_idx: byte counter 0/1/2 for R/G/B triplet accumulation.
     * dac_rgb_buf[3]: partial RGB triplet buffer for DAC writes.
     * dac_state: 0=write mode, 3=read mode (matches 86Box svga->dac_state).
     *
     * CRTC registers live in crtc_ctrl[64]; the VGA path (0x3d4/0x3d5) and
     *   BAR0 writes to offsets 0xd4/0xd5 use the same array.
     */
    uint8_t  misc_out;             /* Misc Output Register              */
    uint8_t  feat_reg;             /* Feature Control Register          */
    uint8_t  seq_idx;              /* Sequencer index (0x3c4)           */
    uint8_t  seq_regs[8];          /* Sequencer registers [0..7]        */
    uint8_t  gr_idx;               /* Graphics Controller index (0x3ce) */
    uint8_t  gr_regs[16];          /* GRC registers [0..15]             */
    uint8_t  ar_idx;               /* ATC index register                */
    uint8_t  ar_regs[32];          /* ATC registers [0..31]             */
    bool     ar_flip_flop;         /* false=index, true=data            */
    uint8_t  dac_pel_mask;         /* DAC PEL mask (0x3c6), default 0xff */
    uint8_t  dac_read_addr;        /* DAC read address (0x3c7)          */
    uint8_t  dac_write_addr;       /* DAC write address (0x3c8)         */
    uint8_t  dac_rgb_idx;          /* R/G/B byte counter (0, 1, 2)      */
    uint8_t  dac_rgb_buf[3];       /* partial RGB triplet               */
    uint8_t  dac_state;            /* 0=write, 3=read (86Box dac_state) */

    /* Generic register scratch (for misc unmapped regs) */
    uint32_t regs[512];

    uint8_t  crtc_ctrl[64];     /* CRTC registers (0x3d4/0x3d5)           */
    uint32_t crtc_idx;          /* current CRTC index                     */
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
    uint32_t        ncc_gen[2];   /* incremented on each nccTable write; used to
                                   * invalidate cached NCC textures in the tex cache */

    /* Blitter state */
    voodoo3_blt_t   blt;

    /* Frame / buffer counters */
    int             cmd_read;       /* triangle commands seen (statistics) */
    int             tri_count;
    uint32_t        fbiPixelsIn, fbiChromaFail, fbiZFuncFail;
    uint32_t        fbiAFuncFail, fbiPixelsOut;

    /*
     * Gap tracker: how often the guest asked for something the model does
     * not implement, per kind and per offending value.  Updated with
     * qatomic_* from any thread; read through the "gaps" QOM property.
     * See voodoo3_gaps.h.  Diagnostic only, not migrated.
     */
    uint32_t gap_count[V3_GAP_MAX][V3_GAP_SLOTS];

    /*
     * VMState shadow for params.fogTable[64].
     *
     * params.fogTable is declared as `struct { uint8_t fog, dfog; }[64]`
     * — an array of anonymous structs.  VMSTATE_ macros cannot address
     * fields inside anonymous struct array elements.  We therefore mirror
     * the table as a flat 128-byte array (interleaved: fog0,dfog0,...,
     * fog63,dfog63 — identical byte layout to the C struct on all
     * platforms since {uint8_t,uint8_t} has no padding).
     *
     * voodoo3_3dstate_pre_save()  copies params.fogTable → fog_table_save.
     * voodoo3_3dstate_post_load() copies fog_table_save → params.fogTable.
     *
     * The verts[4] setup-vertex array is similarly mirrored as a flat
     * float array for the same reason (struct-of-floats without padding).
     */
    uint8_t  fog_table_save[128];   /* packed fogTable shadow (pre_save / post_load) */
    uint32_t verts_save[4 * 14];    /* packed verts[4] shadow: 14 floats per vertex (stored as uint32 bitwise) */

    /*
     * Texture-aperture and 3D-LFB writes are executed synchronously in the
     * calling thread (see voodoo3_lfb3d_write / voodoo3_mmio_write), which
     * keeps them ordered against triangle commands.
     */

    /* --- Triangle parameter ring buffer ---------------------------------- *
     * Single producer (serialised by queue_lock), one consumer per render   *
     * thread.  param_wr / param_rd[] are accessed with acquire/release       *
     * atomics; render threads never take a lock while rasterising.           */
    voodoo3_params_t param_buf[PARAM_BUF_SIZE];
    uint32_t         param_wr;      /* next slot to fill (producer)       */
    uint32_t         param_rd[MAX_RENDER_THREADS]; /* per-thread read ptr */

    /* --- Worker threads -------------------------------------------------- */
    QemuThread   render_thread[MAX_RENDER_THREADS];
    QemuEvent    render_event[MAX_RENDER_THREADS]; /* "work available"    */
    QemuEvent    render_space_event; /* ring slot freed / thread finished */
    QemuMutex    queue_lock;         /* serialises triangle producers      */
    QemuThread   fifo_thread;        /* CMDFIFO0/1 packet processor        */
    QemuEvent    fifo_event;         /* "CMDFIFO has new data"             */
    QemuEvent    fifo_idle_event;    /* FIFO thread went idle              */
    bool         fifo_busy;          /* FIFO thread is processing          */
    bool         threads_started;
    bool         render_stop;
    uint32_t     render_threads_count;
    /*
     * odd_even_mask — band-parallel scanline assignment (86Box style).
     *
     * Value = render_threads_count - 1.  Each render thread T renders only
     * scanlines where (screen_y & odd_even_mask) == T:
     *   1 thread:  mask=0 → all scanlines (no filtering)
     *   2 threads: mask=1 → thread 0 = even lines, thread 1 = odd lines
     *   4 threads: mask=3 → thread T = every 4th line starting at T
     *
     * Ported from 86Box voodoo->odd_even_mask (vid_voodoo_common.h).
     */
    uint32_t     odd_even_mask;

    /* Device variant */
    uint32_t model;
    bool     is_agp, bilinear, dac_filter;
    OnOffAuto lfb_tiling;           /* property: decode tiled LFB aperture
                                     * (auto: while the desktop is tiled) */

    /*
     * Legacy VGA (property "legacy-vga", default off).  When on, QEMU's VGA
     * core provides text/planar/256-colour modes at the fixed PC addresses
     * (ports 0x3B0-0x3DF, memory 0xA0000-0xBFFFF) and is shown while the
     * video processor is disabled (vidProcCfg bit 0 = 0), i.e. before a
     * native driver takes over.  Needed for the video BIOS on x86 hosts.
     * With the property off nothing of this is created.
     */
    bool          legacy_vga;
    bool          debug_trace;      /* property "debug": v3dbg tracing    */
    int64_t       last_kick_sync_ns;
    bool          vga_shown;        /* last refresh came from the VGA core */
    int           vga_leave_cnt;    /* refreshes with the desktop enabled  */
    VGACommonState vga;
    MemoryRegion  vga_ports;        /* 0x3B0-0x3DF in PCI I/O space        */
    MemoryRegion  vga_bank_mr;      /* 0xA0000 banked window into SGRAM    */

    /* --- Texture subsystem (ported from 86Box voodoo_t) ----------------- */

    /* Texture RAM: aliases fb_mem (Banshee/V3 unified SGRAM) */
    uint8_t             *tex_mem[2];
    uint32_t             tex_mem_size;   /* = fb_size                         */
    uint32_t             tex_mask;       /* tex_mem_size - 1                  */

    /*
     * SGRAM pages written behind the texture cache's back (CPU LFB writes,
     * 2D engine, CMDFIFO packet 5).  Set atomically by any thread; consumed
     * by the triangle producer before it looks up a texture.
     */
    unsigned long       *tex_dirty_pages;
    bool                 tex_dirty_any;
    /* SGRAM pages written by paths the display must pick up (tiled LFB) */
    unsigned long       *disp_dirty_pages;
    bool                 disp_dirty_any;

    /* Decoded texture cache (V3_TEX_CACHE_SIZE slots per TMU) */
    v3_tex_cache_entry_t tex_cache[2][V3_TEX_CACHE_SIZE];
    uint32_t             tex_lru[2];     /* simple eviction counter           */
    uint32_t             tex_epoch;      /* meant to be bumped whenever the
                                          * guest may have written texture
                                          * memory; cache hits re-verify their
                                          * source hash once per epoch.  Not
                                          * advanced anywhere yet (open item
                                          * in voodoo3-port-analyse.md)     */

    /* ARGB palette (256 entries per TMU) used for PAL8 / APAL8 / APAL88 */
    uint32_t             tex_palette[2][256];
    uint32_t             pal_gen[2];     /* bumped on every palette write */

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
    int                  swap_count;     /* swapPending writes not yet
                                          * retired (status bits 30:28) */
    int                  swap_interval;   /* vblanks to wait              */
    uint32_t             swap_offset;     /* draw_offset to flip to       */
    int                  retrace_count;   /* vblanks since swap requested */
    uint32_t             frame_count;     /* total frames rendered         */

    /* --- AGP host→VRAM DMA transfer registers (86Box banshee_t agp*) --- *
     * Written via banshee_cmd_write() Agp_* cases (BAR0+0x80000 area).    *
     * agpMoveCMD triggers the actual transfer; others set up the params.   */
    uint32_t             agpReqSize;            /* bytes to transfer        */
    uint32_t             agpHostAddressLow;     /* source host address      */
    uint32_t             agpHostAddressHigh;    /* width[13:0]+stride[27:14]*/
    uint32_t             agpGraphicsAddress;    /* dest VRAM byte address   */
    uint32_t             agpGraphicsStride;     /* dest stride in bytes     */
    uint32_t             agpMoveCMD;            /* trigger + dest type      */

    /* --- CMDFIFO0 state (BAR0 + 0x80000, 86Box banshee_t cmdfifo_*) ---- *
     *                                                                       *
     * The hardware CMDFIFO is an AGP/PCI ring buffer that the driver        *
     * configures via registers at BAR0+0x80020..0x80048.  The driver writes *
     * these on every mode-set; without them the driver's init sequence       *
     * spins forever on cmdFifoDepth and the system hangs/BSODs.             *
     *                                                                       *
     * FIFO0 is the primary command FIFO (PCI and AGP).                      *
     * FIFO1 is the secondary FIFO (AGP burst only).                         *
     * Both have the same register layout; FIFO1 starts at offset +0x30.     */
    uint32_t             cmdfifo_base;        /* FIFO0 base address           */
    uint32_t             cmdfifo_end;         /* FIFO0 end address (computed) */
    uint32_t             cmdfifo_size;        /* raw cmdBaseSize0 value        */
    bool                 cmdfifo_enabled;     /* bit 8 of cmdBaseSize0         */
    bool                 cmdfifo_no_holes;    /* bit 10: depth only via BUMP   */
    bool                 cmdfifo_in_agp;      /* bit 9 of cmdBaseSize0         */
    int                  cmdfifo_in_sub;      /* subroutine nesting depth      */
    uint32_t             cmdfifo_rp;          /* read pointer                  */
    uint32_t             cmdfifo_amin;        /* contiguous block lower bound  */
    uint32_t             cmdfifo_amax;        /* contiguous block upper bound  */
    uint32_t             cmdfifo_depth_rd;    /* depth read counter            */
    uint32_t             cmdfifo_depth_wr;    /* depth write counter           */
    uint32_t             cmdfifo_holecount;   /* outstanding holes in stream   */

    /* --- CMDFIFO1 state ------------------------------------------------- */
    uint32_t             cmdfifo_base_2;
    uint32_t             cmdfifo_end_2;
    uint32_t             cmdfifo_size_2;
    bool                 cmdfifo_enabled_2;
    bool                 cmdfifo_no_holes_2;
    bool                 cmdfifo_in_agp_2;
    int                  cmdfifo_in_sub_2;
    uint32_t             cmdfifo_rp_2;
    uint32_t             cmdfifo_amin_2;
    uint32_t             cmdfifo_amax_2;
    uint32_t             cmdfifo_depth_rd_2;
    uint32_t             cmdfifo_depth_wr_2;
    uint32_t             cmdfifo_holecount_2;

    /* --- VGA IRQ state -------------------------------------------------- */
    bool                 vblank_irq_pending; /* set by vblank cb, cleared by ISR */

    /* --- PLL / pixel clock state ---------------------------------------- *
     * pixel_clock_hz: current pixel clock frequency in Hz, computed from    *
     *   pllCtrl0 by voodoo3_pll_calc_freq().  Used to derive the vblank timer  *
     *   period so the emulated refresh rate tracks the programmed PLL.       *
     *                                                                        *
     * vblank_period_ns: nanoseconds per frame = (htotal * vtotal) /          *
     *   pixel_clock_hz * 1e9.  Stored so voodoo3_vblank_cb() can reschedule  *
     *   the timer without recomputing the PLL formula every vblank.          *
     *                                                                        *
     * 86Box equivalent: svga->clock (cycles per pixel) computed in           *
     *   banshee_recalctimings() and used by svga_recalctimings() to derive   *
     *   the scanline/frame timings.  We skip the scanline granularity and    *
     *   go straight to the full-frame period.                                */
    /* I2C/DDC — QEMU-native bitbang I2C + EDID slave (same as ATI) */
    bitbang_i2c_interface bbi2c_ddc;   /* DDC bus  (vidSerial bits 18-22) */
    bitbang_i2c_interface bbi2c_i2c;   /* I2C bus  (vidSerial bits 23-27) */
    I2CDDCState           i2cddc;

    double               pixel_clock_hz;    /* Hz, 0 = use VBLANK_HZ default */
    int64_t              vblank_period_ns;  /* ns per frame, 0 = use default  */
};


/* (voodoo3_queue_triangle() is declared near the top of this file) */

/* Pixel byte-order helpers (see lfb_be32 in Voodoo3State). */
static inline bool v3_be16(const Voodoo3State *s)
{
    return s->lfb_be16;
}
static inline bool v3_be32(const Voodoo3State *s)
{
    return s->lfb_be32;
}
/*
 * Byte rearrangement (k -> k ^ x) the chip's LFB swizzle applied to CPU
 * writes of the visible screen's data.  16 bpp: only when the driver uses
 * the word swizzle (AmigaOS 0xC0000000 -> x = 1); MorphOS writes RGB16PC
 * with the swizzle off (x = 0) even if it enabled bit 30 earlier.
 */
static inline uint32_t v3_lfb_x(const Voodoo3State *s)
{
    if (s->pix_format == 1) {
        return s->lfb_be16 ? ((s->lfb_be32 ? 3u : 0u) ^ 2u) : 0u;
    }
    /* 8/24/32 bpp: only the byte swizzle applies.  lfb_be16 is sticky from
     * an earlier 16-bit mode (AmigaOS 0xC0000000) and must not turn the
     * 32-bit rearrangement (x = 3) into x = 1. */
    return s->lfb_be32 ? 3u : 0u;
}

/* value to store host-natively so SGRAM gets the right byte order */
static inline uint16_t v3_px16(const Voodoo3State *s, uint32_t v)
{
    return v3_be16(s) ? cpu_to_be16((uint16_t)v) : cpu_to_le16((uint16_t)v);
}
static inline uint32_t v3_px32(const Voodoo3State *s, uint32_t v)
{
    return v3_be32(s) ? cpu_to_be32(v) : cpu_to_le32(v);
}
static inline uint16_t v3_ld16(const Voodoo3State *s, const void *p)
{
    return v3_be16(s) ? lduw_be_p(p) : lduw_le_p(p);
}
static inline void v3_st16(const Voodoo3State *s, void *p, uint16_t v)
{
    if (v3_be16(s)) {
        stw_be_p(p, v);
    } else {
        stw_le_p(p, v);
    }
}
static inline uint32_t v3_ld32(const Voodoo3State *s, const void *p)
{
    return v3_be32(s) ? ldl_be_p(p) : ldl_le_p(p);
}


/*
 * Record an engine write to SGRAM by address (display rows are derived from
 * the desktop start/stride; texture cache entries are invalidated).
 */
static inline void v3_mark_px(Voodoo3State *s, const void *ptr)
{
    unsigned long pg = (unsigned long)((const uint8_t *)ptr - s->fb_mem)
                       >> V3_VRAM_PAGE_SHIFT;
    if (!test_bit(pg, s->disp_dirty_pages)) {
        set_bit_atomic(pg, s->disp_dirty_pages);
        qatomic_set(&s->disp_dirty_any, true);
    }
    if (!test_bit(pg, s->tex_dirty_pages)) {
        set_bit_atomic(pg, s->tex_dirty_pages);
        qatomic_set(&s->tex_dirty_any, true);
    }
}


#endif /* HW_DISPLAY_VOODOO3_INT_H */
