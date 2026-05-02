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

/* -----------------------------------------------------------------------
 * I2C / DDC bit-bang state machine
 * vidSerialParallelPort GPIO: bit3=SCL-out, bit1=SDA-out, bit8=SDA-in
 * We act as DDC EEPROM slave at I2C address 0x50.
 * ----------------------------------------------------------------------- */
typedef enum {
    I2C_IDLE = 0,
    I2C_RECV_ADDR,
    I2C_SEND_ACK,
    I2C_RECV_REG,
    I2C_SEND_ACK2,
    I2C_SEND_DATA,
    I2C_WAIT_ACK,
} Voodoo3I2CState;

typedef struct {
    Voodoo3I2CState state;
    uint8_t  shift_reg;
    int      bit_count;
    uint8_t  addr;
    uint8_t  reg;
    int      data_idx;
    int      scl_last;
    int      sda_last;
    uint8_t  sda_out;
} Voodoo3I2C;

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
#include "vga_int.h"                      /* VGACommonState             */

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

    /*
     * Detail-texture parameters — decoded from the tDetail register (0x308).
     * Ported from 86Box voodoo_params_t: detail_max[], detail_bias[], detail_scale[].
     *
     * detail_max[tmu]   = tDetail[7:0]   — clamp ceiling (0..255)
     * detail_bias[tmu]  = tDetail[13:8]  — LOD subtrahend (0..63)
     * detail_scale[tmu] = tDetail[16:14] — left-shift amount (0..7)
     *
     * Used in TC_MSELECT_DETAIL / TCA_MSELECT_DETAIL colour-path cases:
     *   factor = clamp((detail_bias - lod) << detail_scale, 0, detail_max)
     */
    int detail_max[2];
    int detail_bias[2];
    int detail_scale[2];
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
    int      dstW, dstH, srcW, srcH;     /* dstSizeX/Y, srcSizeX/Y */
    int      dstSizeX, dstSizeY;
    int      srcSizeX, srcSizeY;
    uint32_t dstStride;         /* dst_stride in bytes (computed) */
    uint32_t srcStride;         /* src_stride in bytes (computed) */
    uint32_t dst_stride;        /* same as dstStride (86Box naming) */
    uint32_t src_stride;        /* same as srcStride (86Box naming) */
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
    int      bres_error_1;      /* X stretch error accumulator (unused) */

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

    /*
     * Embedded VGA core (pattern from hw/display/ati.c).
     *
     * vga_init() registers VGA I/O ports 0x3B0–0x3DF on the ISA bus and
     * maps VGA MMIO at 0xA0000–0xBFFFF.  These are required for:
     *
     *   • BIOS/UEFI POST:  probes 0x3C2 (Misc Output) + 0x3DA (Input Status)
     *   • Linux vesafb/efifb: programs CRTC regs before loading the KMS driver
     *   • Linux fbdev/vgacon: uses the standard VGA text-mode path on boot
     *   • Windows VGA miniport: accesses 0x3C0–0x3DF before the ICD loads
     *
     * Without these ports display_enabled never becomes true → black screen.
     *
     * The VGA console (vga.con, index 0) handles text mode and the early
     * graphical boot phase.  Once the native Voodoo3 driver sets
     * VIDPROCCFG_VIDPROC_ENABLE, we switch to the native console (s->con,
     * index 1) and stop forwarding to vga_update_display().
     *
     * vga.vram_size_mb defaults to 4 MB (set in voodoo3_pci_realize before
     * calling vga_common_init).  The Voodoo3's own SGRAM lives in fb_mem;
     * the VGA VRAM is only used for VGA-compat text/graphic modes.
     */
    VGACommonState vga;

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
    MemoryRegion lfb_ram;          /* RAM-backed, no trap — whole BAR1    */
    MemoryRegion cmdfifo_mmio;     /* MMIO overlay for CMDFIFO ring window */
    bool         cmdfifo_mmio_active; /* true when subregion is added      */

    /* Native Voodoo3 display console (index 1, used after driver init) */
    QemuConsole  *con;
    QEMUTimer    *vblank_timer;
    bool          in_vblank;
    int           screen_width, screen_height;
    bool          display_enabled;
    bool          driver_active;    /* true once driver has set VIDPROC_ENABLE=1;
                                     * disables VGA-fallback in vidProcCfg handler */
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
    int           cur_yoff;     /* sprite rows skipped when cursor clips above screen top */
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
        int32_t  src_y;
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
     *   1. Physical I/O ports 0x3c0..0x3df — registered with io_sethandler()
     *      in 86Box; in QEMU this maps to BAR2 offsets 0xb0..0xdf.
     *   2. BAR0 IO-remap window (offset 0xb0..0xdf), forwarded by
     *      banshee_ext_out/in() → banshee_out/in() → svga_out/in().
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
     * Note: CRTC registers reuse the existing crtc_ctrl[64] array.
     *   The ext path (0xd4/0xd5 via BAR0/BAR2) and the VGA path (0x3d4/0x3d5)
     *   both read/write the same crtc_ctrl[] — identical to 86Box where
     *   banshee_ext_outl(crtcCtrl) and banshee_out(0x3d4/0x3d5) both touch svga->crtc[].
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
    uint32_t        ncc_gen[2];   /* incremented on each nccTable write; used to
                                   * invalidate cached NCC textures in the tex cache */

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

    /* Diagnostic: counts status register reads after mode-set */
    uint32_t             status_read_count;

    /* --- PLL / pixel clock state ---------------------------------------- *
     * pixel_clock_hz: current pixel clock frequency in Hz, computed from    *
     *   pllCtrl0 by voodoo3_pll_recalc().  Used to derive the vblank timer  *
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
    /* I2C/DDC state + EDID */
    Voodoo3I2C           ddc;
    uint8_t              ddc_edid[128];

    double               pixel_clock_hz;    /* Hz, 0 = use VBLANK_HZ default */
    int64_t              vblank_period_ns;  /* ns per frame, 0 = use default  */
};

/* Internal function declared in voodoo3.c, used by voodoo3_setup.c */

/* voodoo3_queue_triangle — submit a triangle to the render ring.
 * Defined as static in voodoo3.c but declared here for setup.c. */

#endif /* HW_DISPLAY_VOODOO3_INT_H */
