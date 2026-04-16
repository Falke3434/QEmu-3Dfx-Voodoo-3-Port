/*
 * QEMU 3Dfx Voodoo 3 / Banshee PCI graphics emulation
 *
 * Ported and adapted from 86Box open-source PC emulator:
 *   vid_voodoo_banshee.c   — Banshee/V3 device, Init/PLL/DAC/Video regs
 *   vid_voodoo_reg.c       — SST-1 3D register decode
 *   vid_voodoo_render.c    — Triangle rasterizer + render thread
 *   vid_voodoo_setup.c     — Triangle setup (floating-point path)
 *   vid_voodoo_blitter.c   — Voodoo2 BLT engine
 *   vid_voodoo_banshee_blitter.c — Banshee/V3 2D engine
 *   vid_voodoo_display.c   — Display output / NCC
 *   vid_voodoo_fb.c        — Framebuffer read/write
 *   vid_voodoo_fifo.c      — Command FIFO
 *   vid_voodoo_texture.c   — Texture download
 *
 * Original 86Box authors: Sarah Walker <https://pcem-emulator.co.uk/>
 * Copyright (C) 2008-2024 Sarah Walker and 86Box contributors
 * Copyright (C) 2026 <your name here>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * -------------------------------------------------------------------------
 * PORTING STATUS
 * -------------------------------------------------------------------------
 * [x] QOM type + PCI config space
 * [x] All 3 BARs (MMIO 32MB, LFB 32MB prefetch, I/O 256B)
 * [x] BAR0 region decode: IO-remap / 2D / 3D / TEX / 3D-LFB
 * [x] Init/PLL/DAC/Video register decode  (banshee_ext_outl/inl)
 * [x] STATUS register with FIFO/busy/vblank flags (banshee_status)
 * [x] LFB tiled-address decode  (banshee_read/write_linear)
 * [x] Pixel-format-aware display output 8/16/24/32 bpp
 * [x] RGB565→BGRA8888 conversion
 * [x] Big-Endian byte-swap guards (be_fb) for PPC guests
 * [x] FIFO command queue (ring buffer)
 * [x] voodoo_params_t — full 3D parameter set ported from vid_voodoo_common.h
 * [x] Full SST-1 3D register decode (voodoo_reg_writel) — all vertex/grad/cmd
 * [x] Integer AND floating-point triangle parameter paths
 * [x] Triangle CMD dispatch → voodoo3_queue_triangle()
 * [x] ftriangleCMD / triangleCMD / sBeginTriCMD / sDrawTriCMD
 * [x] fbzMode / fbzColorPath / alphaMode / fogMode / lfbMode
 * [x] Clip registers (clipLeftRight / clipLowYHighY / clip1 / clip2)
 * [x] colBufferAddr / colBufferStride / auxBufferAddr / auxBufferStride
 * [x] clutData write
 * [x] Setup-mode vertex accumulator (sVx/sVy/sRed/... sDrawTriCMD)
 * [x] swapbufferCMD / fastfillCMD / nopCMD
 * [x] Banshee 2D blitter command decode (COMMAND_CMD_* constants)
 * [x] RectFill / ScreenToScreen / HostToScreen stubs wired to dispatcher
 * [x] 4 render QemuThreads (mirrors 86Box render_thread_1..4)
 * [x] Vblank QEMUTimer at ~60 Hz
 * [x] VMState for snapshots
 * [ ] Actual pixel-level triangle rasterizer (voodoo_triangle inner loop)
 * [ ] Texture fetch & filtering (voodoo_texture.c port)
 * [ ] Full Banshee 2D pixel operations (stretch-blt, line, polyline)
 * [ ] CMDFIFO (AGP ring buffer) processing
 * [ ] Hardware cursor compositing
 * [ ] DAC PLL clock recalculation
 * -------------------------------------------------------------------------
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qemu/thread.h"
#include "qapi/error.h"
#include "hw/pci/pci_device.h"
#include "hw/display/voodoo3.h"
#include "hw/display/voodoo3_int.h"
#include "hw/core/qdev-properties.h"
#include "hw/pci/pci.h"

/* glib defines CLAMP(x,lo,hi) - redefine for our single-arg use */
#ifdef CLAMP
#undef CLAMP
#endif
#define CLAMP(x)   ((x) < 0 ? 0 : ((x) > 255 ? 255 : (x)))
#define CLAMP16(x) ((x) < 0 ? 0 : ((x) > 65535 ? 65535 : (x)))

/* voodoo3_render.h, voodoo3_texture.h, voodoo3_display.h are all included
 * transitively via voodoo3_int.h */
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "migration/vmstate.h"

/* =========================================================================
 * PCI identity
 * ========================================================================= */
#define PCI_VENDOR_ID_3DFX          0x121A
#define PCI_DEVICE_ID_3DFX_BANSHEE  0x0003
#define PCI_DEVICE_ID_3DFX_VOODOO3  0x0005
#define PCI_SUBDEV_V3_2000_PCI      0x0036
#define PCI_SUBDEV_V3_3000_PCI      0x003A
#define PCI_SUBDEV_V3_2000_AGP      0x0038
#define PCI_SUBDEV_V3_3000_AGP      0x003C

/* =========================================================================
 * BAR sizes
 * ========================================================================= */
#define VOODOO3_MMIO_SIZE   (32 * MiB)
#define VOODOO3_LFB_SIZE    (32 * MiB)
#define VOODOO3_IO_SIZE     256
#define VOODOO3_FB_SIZE     (16 * MiB)
#define VOODOO3_CLUT_SIZE   256

/* =========================================================================
 * Ext register offsets (86Box banshee enum, BAR0 IO-remap window)
 * Source: 86Box vid_voodoo_banshee.c banshee_ext_outl/inl()
 * ========================================================================= */
#define Init_status                      0x00
#define Init_pciInit0                    0x04
#define Init_sipMonitor                  0x08
#define Init_lfbMemoryConfig             0x0c
#define Init_miscInit0                   0x10
#define Init_miscInit1                   0x14
#define Init_dramInit0                   0x18
#define Init_dramInit1                   0x1c
#define Init_agpInit0                    0x20
#define Init_tmugbInit                   0x24   /* TMU global buffer init — write-only */
#define Init_vgaInit0                    0x28
#define Init_vgaInit1                    0x2c
#define Init_2dCommand                   0x30
#define Init_2dSrcBaseAddr               0x34
#define Init_2dSrcFormat                 0x38   /* Banshee 2D source format */
#define Init_2dSrcSize                   0x3c   /* Banshee 2D source size   */
#define PLL_pllCtrl0                     0x40
#define PLL_pllCtrl1                     0x44
#define PLL_pllCtrl2                     0x48
#define DAC_dacMode                      0x4c
#define DAC_dacAddr                      0x50
#define DAC_dacData                      0x54
#define Video_maxRgbDelta                0x58
#define Video_vidProcCfg                 0x5c
#define Video_hwCurPatAddr               0x60
#define Video_hwCurLoc                   0x64
#define Video_hwCurC0                    0x68
#define Video_hwCurC1                    0x6c
#define Video_vidInFormat                0x70   /* video capture input format */
#define Video_vidSerialParallelPort      0x78
#define Video_vidInXDecimDeltas          0x80
#define Video_vidInError                 0x84
#define Video_vidInXStart                0x88
#define Video_vidChromaKeyMin            0x8c
#define Video_vidChromaKeyMax            0x90
#define Video_vidScreenSize              0x98
#define Video_vidOverlayStartCoords      0x9c
#define Video_vidOverlayEndScreenCoords  0xa0
#define Video_vidOverlayDudx             0xa4
#define Video_vidOverlayDudxOffsetSrcWidth 0xa8
#define Video_vidOverlayDvdy             0xac
#define Video_vidOverlayDvdyOffset       0xe0
#define Video_vidDesktopStartAddr        0xe4
#define Video_vidDesktopOverlayStride    0xe8

/*
 * Banshee CRTC / DAC index-data registers at ext offsets 0x70..0xda
 * (86Box banshee_ext_outl: dacIndexed*, crtcIndexed*, etc.)
 * 0x70 = miscInit2 / vidDesktopTileStride (write-only mirror)
 * 0xc0 = crtcFrequency0 (index select)
 * 0xc2 = crtcFrequency1
 * 0xc3 = crtcFrequency2
 * 0xc4 = crtcDoubleRate
 * 0xc5 = crtcValue        <- read/write of indexed value
 * 0xc6 = crtcBrightness / crtcContrast
 * 0xce = dacReset (index)
 * 0xcf = dacValue
 * 0xd4 = crtcCtrl (index)
 * 0xd5 = crtcCtrlValue
 * 0xda = dacStatus (read) — bit 3 = VSYNC, bit 0 = DAC ready
 */
#define Ext_miscInit2            0x70
#define Ext_crtcFreq0            0xc0
#define Ext_crtcFreq1            0xc2
#define Ext_crtcFreq2            0xc3
#define Ext_crtcDoubleRate       0xc4
#define Ext_crtcValue            0xc5
#define Ext_crtcBrightness       0xc6
#define Ext_dacResetIdx          0xce
#define Ext_dacResetVal          0xcf
#define Ext_crtcCtrlIdx          0xd4
#define Ext_crtcCtrlVal          0xd5
#define Ext_dacStatus            0xda

/* 2D blitter register sub-offsets (banshee 2D engine, BAR0 at 0x100000)
 * 0x054 = engineStatus (read) / srcBaseAddr (write-alias)
 * 0x080 = srcBaseAddr / expand write port
 * These are the correct Banshee 2D-engine field offsets from 86Box
 * vid_voodoo_banshee.c banshee_blt_write(). */
#define BLT2D_STATUS       0x054
#define BLT2D_STATUS_IDLE  0x00000000u   /* engine idle, FIFO empty */
#define BLT2D_SRCBASE      0x080

/* VIDPROCCFG bits */
#define VIDPROCCFG_VIDPROC_ENABLE   (1u << 0)
#define VIDPROCCFG_HWCURSOR_ENA     (1u << 27)
#define VIDPROCCFG_DESKTOP_PIX_FMT(v) (((v) >> 18) & 7u)
#define VIDPROCCFG_DESKTOP_TILE     (1u << 24)
#define MISCINIT0_Y_SWAP_SHIFT      18
#define MISCINIT0_Y_SWAP_MASK       (0xfffu << MISCINIT0_Y_SWAP_SHIFT)

/* Pixel format codes */
#define PIX_FORMAT_8       0
#define PIX_FORMAT_RGB565  1
#define PIX_FORMAT_RGB24   2
#define PIX_FORMAT_RGB32   3

/* BAR0 sub-region decode (banshee_reg_readl/writel) */
#define BAR0_IO_REMAP   0x0000000u
#define BAR0_2D_REGS    0x0100000u
#define BAR0_3D_LO      0x0200000u
#define BAR0_3D_HI      0x0500000u
#define BAR0_TEX0       0x0600000u
#define BAR0_TEX1       0x0800000u
#define BAR0_3D_LFB     0x1000000u

/* FIFO command type tags */
#define FIFO_WRITEL_REG    0x00000000u
#define FIFO_WRITEL_2DREG  0x00800000u
#define FIFO_WRITEL_FB     0x01000000u
#define FIFO_WRITEL_TEX    0x01800000u

#define VBLANK_HZ       60
#define MAX_RENDER_THREADS 4
#define PARAM_BUF_SIZE  256      /* must be power of 2 */

/* =========================================================================
 * SST-1 register offsets (from 86Box vid_voodoo_regs.h, abridged)
 * ========================================================================= */
#define SST_status          0x000
#define SST_intrCtrl        0x004
#define SST_fbzColorPath    0x008
#define SST_fogMode         0x00c
#define SST_alphaMode       0x010
#define SST_fbzMode         0x014
#define SST_lfbMode         0x018
#define SST_clipLeftRight   0x01c
#define SST_clipLowYHighY   0x020
#define SST_nopCMD          0x024
#define SST_fastfillCMD     0x028
#define SST_swapbufferCMD   0x02c
#define SST_fogColor        0x030
#define SST_zaColor         0x034
#define SST_chromaKey       0x038
#define SST_stipple         0x03c
#define SST_color0          0x040
#define SST_color1          0x044
#define SST_fogTable00      0x100
#define SST_fogTable1f      0x17c
/* These are the Banshee/Voodoo3 register names at 0x2c0-0x2d8.
 * SST_clipLeftRight1 etc. are Voodoo1/2 names for the same offsets.
 * In Banshee/V3, these offsets are reused as the setup vertex registers.
 * The 3D register write handler uses both names. */
#define SST_clipLeftRight1  0x2c0
#define SST_clipTopBottom1  0x2c4
#define SST_colBufferAddr   0x2c8
#define SST_colBufferStride 0x2cc
#define SST_auxBufferAddr   0x2d0
#define SST_auxBufferStride 0x2d4
#define SST_clutData        0x2d8  /* clutData alias for CLUT writes at sBlue addr */
#define SST_vertexAx        0x180
#define SST_vertexAy        0x184
#define SST_vertexBx        0x188
#define SST_vertexBy        0x18c
#define SST_vertexCx        0x190
#define SST_vertexCy        0x194
#define SST_startR          0x198
#define SST_startG          0x19c
#define SST_startB          0x1a0
#define SST_startZ          0x1a4
#define SST_startA          0x1a8
#define SST_startS          0x1ac
#define SST_startT          0x1b0
#define SST_startW          0x1b4
#define SST_dRdX            0x1b8
#define SST_dGdX            0x1bc
#define SST_dBdX            0x1c0
#define SST_dZdX            0x1c4
#define SST_dAdX            0x1c8
#define SST_dSdX            0x1cc
#define SST_dTdX            0x1d0
#define SST_dWdX            0x1d4
#define SST_dRdY            0x1d8
#define SST_dGdY            0x1dc
#define SST_dBdY            0x1e0
#define SST_dZdY            0x1e4
#define SST_dAdY            0x1e8
#define SST_dSdY            0x1ec
#define SST_dTdY            0x1f0
#define SST_dWdY            0x1f4
#define SST_triangleCMD     0x1f8
#define SST_fvertexAx       0x200
#define SST_fvertexAy       0x204
#define SST_fvertexBx       0x208
#define SST_fvertexBy       0x20c
#define SST_fvertexCx       0x210
#define SST_fvertexCy       0x214
#define SST_fstartR         0x218
#define SST_fstartG         0x21c
#define SST_fstartB         0x220
#define SST_fstartZ         0x224
#define SST_fstartA         0x228
#define SST_fstartS         0x22c
#define SST_fstartT         0x230
#define SST_fstartW         0x234
#define SST_fdRdX           0x238
#define SST_fdGdX           0x23c
#define SST_fdBdX           0x240
#define SST_fdZdX           0x244
#define SST_fdAdX           0x248
#define SST_fdSdX           0x24c
#define SST_fdTdX           0x250
#define SST_fdWdX           0x254
#define SST_fdRdY           0x258
#define SST_fdGdY           0x25c
#define SST_fdBdY           0x260
#define SST_fdZdY           0x264
#define SST_fdAdY           0x268
#define SST_fdSdY           0x26c
#define SST_fdTdY           0x270
#define SST_fdWdY           0x274
#define SST_ftriangleCMD    0x278
#define SST_sSetupMode      0x2c0
#define SST_sVx             0x2c4
#define SST_sVy             0x2c8
#define SST_sARGB           0x2cc
#define SST_sRed            0x2d0
#define SST_sGreen          0x2d4
#define SST_sBlue           0x2d8
#define SST_sAlpha          0x2dc
#define SST_sVz             0x2e0
#define SST_sWb             0x2e4
#define SST_sW0             0x2e8
#define SST_sS0             0x2ec
#define SST_sT0             0x2f0
#define SST_sW1             0x2f4
#define SST_sS1             0x2f8
#define SST_sT1             0x2fc
#define SST_sBeginTriCMD    0x300
#define SST_sDrawTriCMD     0x304

/* NCC table register offsets (86Box SST_nccTable0_* values) */
#define SST_nccTable0_Y0  0x100
#define SST_nccTable0_Y1  0x104
#define SST_nccTable0_Y2  0x108
#define SST_nccTable0_Y3  0x10c
#define SST_nccTable0_I0  0x110
#define SST_nccTable0_I1  0x114
#define SST_nccTable0_I2  0x118
#define SST_nccTable0_I3  0x11c
#define SST_nccTable0_Q0  0x120
#define SST_nccTable0_Q1  0x124
#define SST_nccTable0_Q2  0x128
#define SST_nccTable0_Q3  0x12c
/* nccTable1 is at +0x10 from nccTable0 in the TMU address space */

/* Banshee 2D command bits (from vid_voodoo_banshee_blitter.c) */
#define COMMAND_CMD_MASK        0xfu
#define COMMAND_CMD_NOP         (0u << 0)
#define COMMAND_CMD_S2S_BLT     (1u << 0)
#define COMMAND_CMD_S2S_STRETCH (2u << 0)
#define COMMAND_CMD_H2S_BLT     (3u << 0)
#define COMMAND_CMD_H2S_STRETCH (4u << 0)
#define COMMAND_CMD_RECTFILL    (5u << 0)
#define COMMAND_CMD_LINE        (6u << 0)
#define COMMAND_CMD_POLYLINE    (7u << 0)
#define COMMAND_CMD_POLYFILL    (8u << 0)
#define COMMAND_INITIATE        (1u << 8)
#define COMMAND_DX              (1u << 14)
#define COMMAND_DY              (1u << 15)

/* Setup mode bits (vid_voodoo_setup.c) */
#define SETUPMODE_RGB           (1 << 0)
#define SETUPMODE_ALPHA         (1 << 1)
#define SETUPMODE_Z             (1 << 2)
#define SETUPMODE_Wb            (1 << 3)
#define SETUPMODE_W0            (1 << 4)
#define SETUPMODE_S0_T0         (1 << 5)
#define SETUPMODE_W1            (1 << 6)
#define SETUPMODE_S1_T1         (1 << 7)
#define SETUPMODE_STRIP_MODE    (1 << 8)


/* =========================================================================
 * Helpers
 * ========================================================================= */
static inline void voodoo3_push_fifo(Voodoo3State *s,
                                     uint32_t cmd, uint32_t val)
{
    uint32_t next = (s->fifo_wr + 1) & (V3_FIFO_SIZE - 1);
    if (next == s->fifo_rd) {
        qemu_log_mask(LOG_UNIMP, "voodoo3: FIFO overflow — dropping\n");
        return;
    }
    s->fifo_cmd[s->fifo_wr] = cmd;
    s->fifo_val[s->fifo_wr] = val;
    s->fifo_wr = next;
    s->cmd_written++;
}

static uint32_t voodoo3_status(Voodoo3State *s)
{
    uint32_t ret = 0;
    int depth    = (int)((s->fifo_wr - s->fifo_rd) & (V3_FIFO_SIZE - 1));
    int busy     = (s->cmd_written != s->cmd_read) || s->voodoo_busy;
    int free     = 32 - depth;
    if (free < 0) free = 0;
    if (free > 0x1f) free = 0x1f;
    ret |= (uint32_t)(0x1f - free) & 0x1fu;
    if (depth > 0)   ret |= (1u << 5);
    if (s->in_vblank) ret |= (1u << 6);
    if (busy)         ret |= (0xfu << 7);
    return ret;
}

/* =========================================================================
 * Queue a rendered triangle into the param ring buffer.
 * The render threads pick it up from there.
 * Mirrors 86Box voodoo_queue_triangle().
 * ========================================================================= */
void voodoo3_queue_triangle(Voodoo3State *s, voodoo3_params_t *p)
{
    /*
     * Update NCC lookup tables if dirty (ported from 86Box triangleCMD handler
     * which calls voodoo_update_ncc() before queuing the triangle).
     */
    for (int _t = 0; _t < 2; _t++) {
        if (s->ncc_dirty[_t]) {
            voodoo3_update_ncc(s, _t);
        }
    }

    /*
     * Bind texture cache entries before queuing — ported from 86Box
     * voodoo_use_texture() call inside voodoo_queue_triangle().
     * This fills p->tex_ptr[][] so the rasterizer can fetch texels.
     */
    if (p->fbzColorPath & (1u << 27)) {   /* FBZCP_TEXTURE_ENABLED */
        voodoo3_use_texture(s, p, 0);
        voodoo3_use_texture(s, p, 1);
    }

    uint32_t idx = s->param_wr & (PARAM_BUF_SIZE - 1);
    memcpy(&s->param_buf[idx], p, sizeof(*p));
    s->param_wr++;
    s->voodoo_busy = true;
    qemu_cond_broadcast(&s->render_cond);
}

/* =========================================================================
 * Init / PLL / DAC / Video register write
 * Ported from 86Box banshee_ext_outl()
 * ========================================================================= */
/* =========================================================================
 * voodoo3_crtc_update — derive display geometry from VGA CRTC registers.
 *
 * AmigaOS 4 programs the display size and stride through VGA CRTC registers
 * (mirrored at ext 0xd4/0xd5) instead of vidScreenSize/vidProcCfg.
 * After every CRTC data write we recalculate and activate the display.
 *
 * Reference: PC99 VGA register documentation + 86Box svga_recalctimings().
 * ========================================================================= */
static void voodoo3_crtc_update(Voodoo3State *s)
{
    /* CRTC[0x01] = horizontal display end; width = (val+1)*8 pixels */
    int w = ((int)s->crtc_ctrl[0x01] + 1) * 8;

    /* CRTC[0x12] = vertical display end (low 8 bits)
     * CRTC[0x07] overflow: bit1 = vde[8], bit6 = vde[9]             */
    int vde = (int)s->crtc_ctrl[0x12]
            | (((int)s->crtc_ctrl[0x07] & 0x02) << 7)
            | (((int)s->crtc_ctrl[0x07] & 0x40) << 3);
    int h = vde + 1;

    /* CRTC[0x13] = logical scan-line width.
     * In standard VGA: row_bytes = CRTC[0x13] * 2  (word granularity).
     * For 256-colour / packed-pixel modes the effective stride is
     * CRTC[0x13] * 8  (byte doubled).  We pick the right one by comparing
     * against the pixel-width we already decoded from CRTC[0x01].
     *
     * Heuristic (from 86Box svga_recalctimings):
     *   stride8  = CRTC[0x13] * 4   (byte-granular packed pixel)
     *   stride16 = CRTC[0x13] * 4   (same register; bpp changes meaning)
     * For 8-bpp VGA:  CRTC[0x13] == w/4  → stride = w   bytes
     * For 16-bpp:     CRTC[0x13] == w/2  → stride = w*2 bytes
     *
     * Simpler rule that works for AmigaOS/MorphOS on Voodoo3:
     *   stride = CRTC[0x13] * 8
     * which gives stride = w for 8bpp (CRTC[0x13]=w/8) and
     *              stride = w*2 for 16bpp (CRTC[0x13]=w/4).
     * Pick pixel format by comparing stride/w ratio.
     */
    int crtc13 = (int)s->crtc_ctrl[0x13];

    if (w <= 0 || h <= 0 || crtc13 == 0)
        return;  /* CRTC not fully programmed yet */

    /* If EXTENDED_SHIFT_OUT is set (vgaInit0 bit 12), the display format
     * and stride come from vidProcCfg / vidDesktopOverlayStride, not CRTC.
     * Only update screen size from CRTC; leave pix_format/stride alone.  */
    bool ext_shift = !!(s->vgaInit0 & (1u << 12));

    int stride_candidate = crtc13 * 8;   /* byte-granular packed pixel */

    /* Derive pixel format from stride / width ratio */
    int new_fmt;
    uint32_t new_stride;

    if (ext_shift) {
        /* vidProcCfg controls format/stride; CRTC only gives us screen size */
        new_fmt    = s->pix_format;
        new_stride = s->desktop_stride > 0 ? s->desktop_stride
                                           : (uint32_t)(w * (s->pix_format == 3 ? 4 :
                                                             s->pix_format == 2 ? 3 :
                                                             s->pix_format == 1 ? 2 : 1));
    } else {
        int ratio = stride_candidate / w;

        if (ratio <= 1) {
            new_fmt    = 0;                   /* 8-bpp palette */
            new_stride = (uint32_t)w;
        } else if (ratio <= 2) {
            new_fmt    = 1;                   /* RGB565 16-bpp */
            new_stride = (uint32_t)(w * 2);
        } else if (ratio <= 3) {
            new_fmt    = 2;                   /* RGB24 */
            new_stride = (uint32_t)(w * 3);
        } else {
            new_fmt    = 3;                   /* ARGB32 */
            new_stride = (uint32_t)(w * 4);
        }
    }

    bool changed = (s->screen_width  != w  ||
                    s->screen_height != h  ||
                    s->pix_format    != new_fmt ||
                    s->desktop_stride != new_stride);

    s->screen_width   = w;
    s->screen_height  = h;
    s->pix_format     = new_fmt;
    s->desktop_stride = new_stride;
    s->params.row_width = new_stride;

    if (!s->display_enabled) {
        /*
         * CRTC-driven display enable: activate output as soon as the CRTC
         * has a valid geometry, even if VIDPROCCFG_VIDPROC_ENABLE was never
         * set.  This is the VGA-compat path for x86 guests (BIOS, Linux fbcon,
         * Windows safe-mode) which program CRTC registers but not vidProcCfg.
         */
        s->display_enabled = true;
        changed = true;
    }

    if (changed && s->con)
        qemu_console_resize(s->con, w, h);

    /* Force full redraw after mode change */
    if (changed)
        memset(s->dirty_line, 1, sizeof(s->dirty_line));
}

static void voodoo3_ext_write(Voodoo3State *s, uint32_t addr, uint32_t val)
{
    switch (addr & 0xff) {
    case Init_pciInit0:
        s->pciInit0 = val;
        break;
    case Init_lfbMemoryConfig:
        s->lfbMemoryConfig = val;
        s->tile_base       = (val & 0x1fff) << 12;
        s->tile_stride     = 1024u << ((val >> 13) & 7);
        s->tile_x          = ((val >> 16) & 0x7f) * 128u;
        break;
    case Init_miscInit0:
        s->miscInit0     = val;
        s->y_origin_swap = (int)((val & MISCINIT0_Y_SWAP_MASK)
                                 >> MISCINIT0_Y_SWAP_SHIFT);
        break;
    case Init_miscInit1:   s->miscInit1 = val; break;
    case Init_dramInit0:   s->dramInit0 = val; break;
    case Init_dramInit1:   s->dramInit1 = val; break;
    case Init_agpInit0:    s->agpInit0  = val; break;
    case Init_tmugbInit:   /* TMU global buffer — store, no effect needed */ break;
    case Init_2dCommand:   s->command_2d = val; break;
    case Init_2dSrcBaseAddr: s->srcBaseAddr_2d = val; break;
    case Init_2dSrcFormat: s->regs[Init_2dSrcFormat >> 2] = val; break;
    case Init_2dSrcSize:   s->regs[Init_2dSrcSize   >> 2] = val; break;
    case Init_vgaInit0:    s->vgaInit0  = val; break;
    case Init_vgaInit1:    s->vgaInit1  = val; break;

    case PLL_pllCtrl0: s->pllCtrl0 = val; break; /* TODO: pixel-clock recalc */
    case PLL_pllCtrl1: s->pllCtrl1 = val; break;
    case PLL_pllCtrl2: s->pllCtrl2 = val; break;

    case DAC_dacMode:
        s->dacMode = val;
        break;
    case DAC_dacAddr:
        s->dacAddr = (int)(val & 0x1ff);
        break;
    case DAC_dacData:
        if (s->dacAddr < VOODOO3_CLUT_SIZE)
            s->pallook[s->dacAddr] =
				(((val >> 16) & 0xff) << 16) |  /* R */
				(((val >>  8) & 0xff) <<  8) |  /* G */
				( (val        & 0xff)      );   /* B */
        break;

    case Video_vidInFormat:
        s->regs[Video_vidInFormat >> 2] = val;  /* capture format — unused */
        break;

    case Video_vidProcCfg:
        s->vidProcCfg      = val;
        s->display_enabled = !!(val & VIDPROCCFG_VIDPROC_ENABLE);
        s->cursor_ena      = !!(val & VIDPROCCFG_HWCURSOR_ENA);
        s->pix_format      = (int)VIDPROCCFG_DESKTOP_PIX_FMT(val);
        s->desktop_tiled   = !!(val & VIDPROCCFG_DESKTOP_TILE);
        /*
         * VGA-Fallback for x86/Linux guests:
         * Standard x86 VGA BIOSes and Linux framebuffer drivers may leave
         * VIDPROCCFG_VIDPROC_ENABLE=0 and rely on VGA-compatibility mode
         * for display output.  Without a full svga/VGA layer (which this port
         * lacks), the screen stays black.
         *
         * Workaround: if the CRTC has already been programmed with a valid
         * resolution, keep display_enabled=true regardless of VIDPROC_ENABLE.
         * This lets VGA-mode guests (x86 BIOS boot, Linux vga=... kernel param,
         * Windows safe-mode) see output through our CRTC-derived display path.
         *
         * Drivers that explicitly disable vidproc (e.g. during mode-switch)
         * and then re-enable it will work correctly because they re-set CRTC
         * and/or vidScreenSize before re-enabling VIDPROC_ENABLE.
         */
        if (!s->display_enabled && s->screen_width > 0 && s->screen_height > 0)
            s->display_enabled = true;
        /*
         * Keep params.col_tiled in sync with the desktop tiling flag from
         * vidProcCfg.  The display blit (voodoo3_update_display_dirty) reads
         * params.col_tiled, not desktop_tiled, so without this sync the
         * display output ignores the tiling mode set by vidProcCfg and
         * produces garbage interleaved scanlines whenever tiling is active.
         *
         * Note: vidDesktopOverlayStride also writes params.col_tiled from its
         * own bit[15]; the two registers must agree.  vidProcCfg's DESKTOP_TILE
         * bit is the authoritative enable — apply it here.
         */
        s->params.col_tiled = s->desktop_tiled;
        /* Force full redraw when video processor is re-enabled */
        if (s->display_enabled)
            memset(s->dirty_line, 1, sizeof(s->dirty_line));
        break;
    case Video_maxRgbDelta: /* filter threshold — ignored for now */ break;
    case Video_hwCurPatAddr:
        s->hwCurPatAddr = val;
        s->cur_pat_addr = val & 0xfffff0u;
        /* Pre-populate cursor_buf from VRAM in case pattern was written before
         * this register (e.g. driver sets up bitmap first, then sets address) */
        if (s->cur_pat_addr + 1024u <= s->fb_size)
            memcpy(s->cursor_buf, s->fb_mem + s->cur_pat_addr, 1024);
        break;
    case Video_hwCurLoc:
        s->hwCurLoc = val;
        s->cur_x    = (int)(val & 0x7ff) - 64;
        s->cur_y    = (int)((val >> 16) & 0x7ff) - 64;
        break;
    case Video_hwCurC0: s->cur_c0 = val; break;
    case Video_hwCurC1: s->cur_c1 = val; break;
    case Video_vidSerialParallelPort:
        s->vidSerialParallelPort = val;
        /* I2C/DDC GPIO — software sets bits, we just store and return 0 on read */
        break;
    case Video_vidChromaKeyMin: s->vidChromaKeyMin = val; break;
    case Video_vidChromaKeyMax: s->vidChromaKeyMax = val; break;
    case Video_vidScreenSize:
        s->vidScreenSize  = val;
        /*
         * 86Box: h_disp = (val & 0xfff) + 1,  v_disp = (val >> 12) & 0xfff
         * v_disp has NO +1 — the height field already encodes the line count
         * directly (not as height-1).  Adding +1 here produces a 1-line-too-tall
         * surface and a stretched/clipped picture.
         */
        s->screen_width   = (int)((val & 0xfff) + 1);
        s->screen_height  = (int)((val >> 12) & 0xfff);
        if (s->con && s->screen_width > 0 && s->screen_height > 0) {
            qemu_console_resize(s->con, s->screen_width, s->screen_height);
            memset(s->dirty_line, 1, sizeof(s->dirty_line));
        }
        break;
    case Video_vidInXDecimDeltas: s->regs[Video_vidInXDecimDeltas >> 2] = val; break;
    case Video_vidInError:        s->regs[Video_vidInError >> 2] = val; break;
    case Video_vidInXStart:       s->regs[Video_vidInXStart >> 2] = val; break;
    case Video_vidOverlayStartCoords:
    case Video_vidOverlayEndScreenCoords:
    case Video_vidOverlayDudx:
    case Video_vidOverlayDudxOffsetSrcWidth:
    case Video_vidOverlayDvdy:
    case Video_vidOverlayDvdyOffset:
        s->regs[(addr & 0xff) >> 2] = val;
        break;
    case Video_vidDesktopStartAddr:
        s->vidDesktopStartAddr = val & 0x00ffffff;
        s->desktop_start       = s->vidDesktopStartAddr;
        /*
         * Sync to params so voodoo3_update_display_dirty() sees the correct
         * framebuffer base.  In 2D/desktop mode (no 3D swap) these are the
         * authoritative source-of-truth for the display scanout.
         */
        s->params.front_offset = s->desktop_start;
        // s->params.draw_offset  = s->desktop_start;
        memset(s->dirty_line, 1, sizeof(s->dirty_line));
        break;
    case Video_vidDesktopOverlayStride:
        s->vidDesktopOverlayStride = val;
        /*
         * Bits [12:0]  = desktop stride in bytes (non-tiled mode, linear pitch).
         * Bits [14:13] = tiling granularity (0=linear, 1=128-byte, 2=256-byte).
         * Bit  [15]    = tiled mode enable.
         *
         * In tiled mode the "stride" field encodes the number of 128-byte tile
         * columns (same encoding as colBufferStride bits[6:0]).  row_width must
         * be set to  num_tile_cols * 128 * 32  so that the display blit and the
         * rasterizer both compute the correct per-tile-row-group byte offset:
         *
         *   offset_y = (y >> 5) * row_width + (y & 31) * 128
         *
         * Using val & 0x7fff directly in tiled mode gives a pitch that is
         * 128*32 = 4096× too small, producing interleaved garbage scanlines.
         *
         * Sync row_width and col_tiled to params immediately so the rasterizer
         * and display output agree on scanline pitch.
         */
        s->params.col_tiled = !!(val & (1u << 15));
        if (s->params.col_tiled) {
            /*
             * 86Box: desktop_stride_tiled = (val & 0x3fff) * 128 * 32
             * bits[13:0] = number of 128-byte column strips.
             * row_width = num_strips * 128 bytes/strip * 32 rows/band
             * Previous code used val & 0x7f (only 7 bits) → 128× too small
             * for screens wider than 64 tile columns (e.g. 1024px @ 16bpp).
             */
            uint32_t num_tile_cols  = val & 0x3fffu;
            s->desktop_stride       = num_tile_cols * 128u * 32u;
        } else {
            /*
             * Confirmed: bits[13:0] = direct byte stride (not shifted).
             * 86Box uses (val & 0x3fff) for the non-tiled stride field.
             * The old 0x1fff mask was wrong — it capped stride at 8191 bytes,
             * cutting off strides for resolutions ≥ 4096 px wide @ 2bpp.
             */
            s->desktop_stride       = val & 0x3fffu;  /* bits[13:0] */
        }
        s->params.row_width = s->desktop_stride;
        break;
    default:
        switch (addr & 0xff) {
        /* miscInit2 / vidDesktopTileStride — store, ignore */
        case Ext_miscInit2:
            s->regs[Ext_miscInit2 >> 2] = val;
            break;
        /* CRTC frequency index/data pair (0xc0–0xc6) */
        case Ext_crtcFreq0:
        case Ext_crtcFreq1:
        case Ext_crtcFreq2:
            s->regs[(addr & 0xfc) >> 2] = val;
            break;
        case Ext_crtcDoubleRate:     /* acts as index for Ext_crtcValue */
            s->crtc_freq_idx = val & 0x3f;
            break;
        case Ext_crtcValue:
            if (s->crtc_freq_idx < 64)
                s->crtc_freq[s->crtc_freq_idx] = (uint8_t)(val & 0xff);
            break;
        case Ext_crtcBrightness:
            s->regs[Ext_crtcBrightness >> 2] = val;
            break;
        /* DAC reset index/data pair (0xce / 0xcf) */
        case Ext_dacResetIdx:
            s->dac_reset_idx = val & 0x3f;
            break;
        case Ext_dacResetVal:
            if (s->dac_reset_idx < 64)
                s->dac_reset[s->dac_reset_idx] = (uint8_t)(val & 0xff);
            break;
        /* CRTC control index/data pair (0xd4 / 0xd5) */
        case Ext_crtcCtrlIdx:
            s->crtc_idx = val & 0x3f;
            break;
        case Ext_crtcCtrlVal:
            if (s->crtc_idx < 64) {
                s->crtc_ctrl[s->crtc_idx] = (uint8_t)(val & 0xff);
                voodoo3_crtc_update(s);
            }
            break;
        /* dacStatus is read-only — writes silently ignored */
        case Ext_dacStatus:
            break;
        default:
            qemu_log_mask(LOG_UNIMP,
                "voodoo3: ext write 0x%02x = 0x%08x (unimplemented)\n",
                addr & 0xff, val);
            break;
        }
        break;
    }
}

static uint32_t voodoo3_ext_read(Voodoo3State *s, uint32_t addr)
{
    switch (addr & 0xff) {
    case Init_status:              return voodoo3_status(s);
    case Init_pciInit0:            return s->pciInit0;
    case Init_lfbMemoryConfig:     return s->lfbMemoryConfig;
    case Init_miscInit0:           return s->miscInit0;
    case Init_miscInit1:           return s->miscInit1;
    case Init_dramInit0:           return s->dramInit0;
    case Init_dramInit1:           return s->dramInit1;
    case Init_agpInit0:            return s->agpInit0;
    case Init_vgaInit0:            return s->vgaInit0;
    case Init_vgaInit1:            return s->vgaInit1;
    case Init_2dCommand:           return s->command_2d;
    case Init_2dSrcBaseAddr:       return s->srcBaseAddr_2d;
    case PLL_pllCtrl0:             return s->pllCtrl0;
    case PLL_pllCtrl1:             return s->pllCtrl1;
    case PLL_pllCtrl2:             return s->pllCtrl2;
    case DAC_dacMode:              return s->dacMode;
    case DAC_dacAddr:              return (uint32_t)s->dacAddr;
    case DAC_dacData:
        return (s->dacAddr < VOODOO3_CLUT_SIZE)
               ? s->pallook[s->dacAddr] : 0xffffffff;
    case Video_vidProcCfg:         return s->vidProcCfg;
    case Video_vidScreenSize:      return s->vidScreenSize;
    case Video_vidDesktopStartAddr:     return s->vidDesktopStartAddr;
    case Video_vidDesktopOverlayStride: return s->vidDesktopOverlayStride;
    case Video_hwCurPatAddr:       return s->hwCurPatAddr;
    case Video_hwCurLoc:           return s->hwCurLoc;
    case Video_hwCurC0:            return s->cur_c0;
    case Video_hwCurC1:            return s->cur_c1;
    case Video_vidChromaKeyMin:    return s->vidChromaKeyMin;
    case Video_vidChromaKeyMax:    return s->vidChromaKeyMax;
    /*
     * vidSerialParallelPort (0x78): I2C / DDC / serial port status.
     * Bit 3 = SCL, bit 1 = SDA, bit 8 = I2C-ack.
     * AmigaOS 3dfxVoodoo.chip polls this waiting for the DDC bus to be
     * idle. Return the stored value with no busy bits set so the driver
     * doesn't spin forever.
     */
    case Video_vidSerialParallelPort:
        return s->vidSerialParallelPort & ~0x0000010cu; /* SCL/SDA idle */
    /* Ext_miscInit2 */
    case Ext_miscInit2:
        return s->regs[Ext_miscInit2 >> 2];
    /* CRTC frequency regs */
    case Ext_crtcFreq0:
    case Ext_crtcFreq1:
    case Ext_crtcFreq2:
        return s->regs[(addr & 0xfc) >> 2];
    case Ext_crtcDoubleRate:
        return s->crtc_freq_idx;
    case Ext_crtcValue:
        return (s->crtc_freq_idx < 64) ? s->crtc_freq[s->crtc_freq_idx] : 0;
    case Ext_dacResetIdx:
        return s->dac_reset_idx;
    case Ext_dacResetVal:
        return (s->dac_reset_idx < 64) ? s->dac_reset[s->dac_reset_idx] : 0;
    case Ext_crtcCtrlIdx:
        return s->crtc_idx;
    case Ext_crtcCtrlVal:
        return (s->crtc_idx < 64) ? s->crtc_ctrl[s->crtc_idx] : 0;
    /*
     * dacStatus (0xda): Bit 3 = VSYNC active, bit 0 = DAC ready.
     * Return 0 = DAC ready, not in VSYNC.  The driver polls this to
     * determine when the DAC palette write is safe.
     */
    case Ext_dacStatus:
        return s->in_vblank ? 0x08u : 0x00u;
    default:
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: ext read 0x%02x (unimplemented)\n", addr & 0xff);
        return 0xffffffff;
    }
}

/* =========================================================================
 * SST-1 3D register write dispatch
 *
 * Ported directly from 86Box voodoo_reg_writel() in vid_voodoo_reg.c.
 * All vertex, gradient, command, clip, buffer, and setup registers.
 * ========================================================================= */
static void voodoo3_3d_reg_write(Voodoo3State *s, uint32_t addr, uint32_t val)
{
    fi_t f;
    f.i = val;

    addr &= 0x3fc;

    switch (addr) {

    /* --- Render-state registers --- */
    case SST_fbzColorPath:
        s->params.fbzColorPath = val;
        s->rgb_sel             = (int)(val & 3);
        break;
    case SST_fogMode:   s->params.fogMode   = val; break;
    case SST_alphaMode: s->params.alphaMode = val; break;
    case SST_fbzMode:   s->params.fbzMode   = val; break;
    case SST_lfbMode:   s->lfbMode          = val; break;

    case SST_clipLeftRight:
        s->params.clipRight = (int)(val & 0xfff);
        s->params.clipLeft  = (int)((val >> 16) & 0xfff);
        break;
    case SST_clipLowYHighY:
        s->params.clipHighY = (int)(val & 0xfff);
        s->params.clipLowY  = (int)((val >> 16) & 0xfff);
        break;
    case SST_clipLeftRight1:
        s->params.clipRight1 = (int)(val & 0xfff);
        s->params.clipLeft1  = (int)((val >> 16) & 0xfff);
        break;
    case SST_clipTopBottom1:
        s->params.clipHighY1 = (int)(val & 0xfff);
        s->params.clipLowY1  = (int)((val >> 16) & 0xfff);
        break;

    case SST_fogColor:
        s->params.fogColor.r = (uint8_t)((val >> 16) & 0xff);
        s->params.fogColor.g = (uint8_t)((val >> 8)  & 0xff);
        s->params.fogColor.b = (uint8_t)(val & 0xff);
        break;
    case SST_zaColor:   s->params.zaColor   = val; break;
    case SST_chromaKey:
        s->params.chromaKey_r = (uint8_t)((val >> 16) & 0xff);
        s->params.chromaKey_g = (uint8_t)((val >>  8) & 0xff);
        s->params.chromaKey_b = (uint8_t)(val & 0xff);
        s->params.chromaKey   = val & 0xffffff;
        break;
    case SST_stipple: s->params.stipple = val; break;
    case SST_color0:  s->params.color0  = val; break;
    case SST_color1:  s->params.color1  = val; break;

    /* --- Fog table (64 entries packed as 2 per dword) --- */
    default:
        if (addr >= SST_fogTable00 && addr <= SST_fogTable1f) {
            unsigned idx = (addr - SST_fogTable00) >> 1;
            s->params.fogTable[idx].dfog     = (uint8_t)(val & 0xff);
            s->params.fogTable[idx].fog      = (uint8_t)((val >> 8)  & 0xff);
            if (idx + 1 < 64) {
                s->params.fogTable[idx+1].dfog = (uint8_t)((val >> 16) & 0xff);
                s->params.fogTable[idx+1].fog  = (uint8_t)((val >> 24) & 0xff);
            }
            break;
        }
        /* fall through to unimplemented log */
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: 3D reg 0x%03x = 0x%08x (unimplemented)\n", addr, val);
        break;

    /* --- Integer vertex / gradient registers --- */
    case SST_vertexAx: s->params.vertexAx = (int32_t)(int16_t)(val & 0xffff); break;
    case SST_vertexAy: s->params.vertexAy = (int32_t)(int16_t)(val & 0xffff); break;
    case SST_vertexBx: s->params.vertexBx = (int32_t)(int16_t)(val & 0xffff); break;
    case SST_vertexBy: s->params.vertexBy = (int32_t)(int16_t)(val & 0xffff); break;
    case SST_vertexCx: s->params.vertexCx = (int32_t)(int16_t)(val & 0xffff); break;
    case SST_vertexCy: s->params.vertexCy = (int32_t)(int16_t)(val & 0xffff); break;

    case SST_startR:   s->params.startR = (int32_t)(val & 0xffffff); break;
    case SST_startG:   s->params.startG = (int32_t)(val & 0xffffff); break;
    case SST_startB:   s->params.startB = (int32_t)(val & 0xffffff); break;
    case SST_startZ:   s->params.startZ = (int32_t)val;              break;
    case SST_startA:   s->params.startA = (int32_t)(val & 0xffffff); break;
    case SST_startS:
        s->params.tmu[0].startS = ((int64_t)(int32_t)val) << 14;
        s->params.tmu[1].startS = s->params.tmu[0].startS;
        break;
    case SST_startT:
        s->params.tmu[0].startT = ((int64_t)(int32_t)val) << 14;
        s->params.tmu[1].startT = s->params.tmu[0].startT;
        break;
    case SST_startW:
        s->params.startW        = (int64_t)(int32_t)val << 2;
        s->params.tmu[0].startW = s->params.startW;
        s->params.tmu[1].startW = s->params.startW;
        break;

#define SIGN24(v) (((v) & 0xffffff) | (((v) & 0x800000) ? 0xff000000u : 0u))
    case SST_dRdX: s->params.dRdX = (int32_t)SIGN24(val); break;
    case SST_dGdX: s->params.dGdX = (int32_t)SIGN24(val); break;
    case SST_dBdX: s->params.dBdX = (int32_t)SIGN24(val); break;
    case SST_dZdX: s->params.dZdX = (int32_t)val;          break;
    case SST_dAdX: s->params.dAdX = (int32_t)SIGN24(val); break;
    case SST_dSdX:
        s->params.tmu[0].dSdX = ((int64_t)(int32_t)val) << 14;
        s->params.tmu[1].dSdX = s->params.tmu[0].dSdX;
        break;
    case SST_dTdX:
        s->params.tmu[0].dTdX = ((int64_t)(int32_t)val) << 14;
        s->params.tmu[1].dTdX = s->params.tmu[0].dTdX;
        break;
    case SST_dWdX:
        s->params.dWdX          = (int64_t)(int32_t)val << 2;
        s->params.tmu[0].dWdX   = s->params.dWdX;
        s->params.tmu[1].dWdX   = s->params.dWdX;
        break;
    case SST_dRdY: s->params.dRdY = (int32_t)SIGN24(val); break;
    case SST_dGdY: s->params.dGdY = (int32_t)SIGN24(val); break;
    case SST_dBdY: s->params.dBdY = (int32_t)SIGN24(val); break;
    case SST_dZdY: s->params.dZdY = (int32_t)val;          break;
    case SST_dAdY: s->params.dAdY = (int32_t)SIGN24(val); break;
    case SST_dSdY:
        s->params.tmu[0].dSdY = ((int64_t)(int32_t)val) << 14;
        s->params.tmu[1].dSdY = s->params.tmu[0].dSdY;
        break;
    case SST_dTdY:
        s->params.tmu[0].dTdY = ((int64_t)(int32_t)val) << 14;
        s->params.tmu[1].dTdY = s->params.tmu[0].dTdY;
        break;
    case SST_dWdY:
        s->params.dWdY        = (int64_t)(int32_t)val << 2;
        s->params.tmu[0].dWdY = s->params.dWdY;
        s->params.tmu[1].dWdY = s->params.dWdY;
        break;
#undef SIGN24

    /* Integer triangle CMD */
    case SST_triangleCMD:
        s->params.sign = (int)(val >> 31);
        voodoo3_queue_triangle(s, &s->params);
        s->cmd_read++;
        break;

    /* --- Floating-point vertex / gradient registers --- */
    case SST_fvertexAx:
        f.i = val; s->params.vertexAx = (int32_t)(int16_t)(int32_t)(f.f * 16.0f) & 0xffff; break;
    case SST_fvertexAy:
        f.i = val; s->params.vertexAy = (int32_t)(int16_t)(int32_t)(f.f * 16.0f) & 0xffff; break;
    case SST_fvertexBx:
        f.i = val; s->params.vertexBx = (int32_t)(int16_t)(int32_t)(f.f * 16.0f) & 0xffff; break;
    case SST_fvertexBy:
        f.i = val; s->params.vertexBy = (int32_t)(int16_t)(int32_t)(f.f * 16.0f) & 0xffff; break;
    case SST_fvertexCx:
        f.i = val; s->params.vertexCx = (int32_t)(int16_t)(int32_t)(f.f * 16.0f) & 0xffff; break;
    case SST_fvertexCy:
        f.i = val; s->params.vertexCy = (int32_t)(int16_t)(int32_t)(f.f * 16.0f) & 0xffff; break;

    case SST_fstartR: f.i = val; s->params.startR = (int32_t)(f.f * 4096.0f); break;
    case SST_fstartG: f.i = val; s->params.startG = (int32_t)(f.f * 4096.0f); break;
    case SST_fstartB: f.i = val; s->params.startB = (int32_t)(f.f * 4096.0f); break;
    case SST_fstartZ: f.i = val; s->params.startZ = (int32_t)(f.f * 4096.0f); break;
    case SST_fstartA: f.i = val; s->params.startA = (int32_t)(f.f * 4096.0f); break;
    case SST_fstartS:
        f.i = val;
        s->params.tmu[0].startS = (int64_t)(f.f * 4294967296.0f);
        s->params.tmu[1].startS = s->params.tmu[0].startS;
        break;
    case SST_fstartT:
        f.i = val;
        s->params.tmu[0].startT = (int64_t)(f.f * 4294967296.0f);
        s->params.tmu[1].startT = s->params.tmu[0].startT;
        break;
    case SST_fstartW:
        f.i = val;
        s->params.startW        = (int64_t)(f.f * 4294967296.0f);
        s->params.tmu[0].startW = s->params.startW;
        s->params.tmu[1].startW = s->params.startW;
        break;

    case SST_fdRdX: f.i = val; s->params.dRdX = (int32_t)(f.f * 4096.0f); break;
    case SST_fdGdX: f.i = val; s->params.dGdX = (int32_t)(f.f * 4096.0f); break;
    case SST_fdBdX: f.i = val; s->params.dBdX = (int32_t)(f.f * 4096.0f); break;
    case SST_fdZdX: f.i = val; s->params.dZdX = (int32_t)(f.f * 4096.0f); break;
    case SST_fdAdX: f.i = val; s->params.dAdX = (int32_t)(f.f * 4096.0f); break;
    case SST_fdSdX:
        f.i = val;
        s->params.tmu[0].dSdX = (int64_t)(f.f * 4294967296.0f);
        s->params.tmu[1].dSdX = s->params.tmu[0].dSdX;
        break;
    case SST_fdTdX:
        f.i = val;
        s->params.tmu[0].dTdX = (int64_t)(f.f * 4294967296.0f);
        s->params.tmu[1].dTdX = s->params.tmu[0].dTdX;
        break;
    case SST_fdWdX:
        f.i = val;
        s->params.dWdX        = (int64_t)(f.f * 4294967296.0f);
        s->params.tmu[0].dWdX = s->params.dWdX;
        s->params.tmu[1].dWdX = s->params.dWdX;
        break;
    case SST_fdRdY: f.i = val; s->params.dRdY = (int32_t)(f.f * 4096.0f); break;
    case SST_fdGdY: f.i = val; s->params.dGdY = (int32_t)(f.f * 4096.0f); break;
    case SST_fdBdY: f.i = val; s->params.dBdY = (int32_t)(f.f * 4096.0f); break;
    case SST_fdZdY: f.i = val; s->params.dZdY = (int32_t)(f.f * 4096.0f); break;
    case SST_fdAdY: f.i = val; s->params.dAdY = (int32_t)(f.f * 4096.0f); break;
    case SST_fdSdY:
        f.i = val;
        s->params.tmu[0].dSdY = (int64_t)(f.f * 4294967296.0f);
        s->params.tmu[1].dSdY = s->params.tmu[0].dSdY;
        break;
    case SST_fdTdY:
        f.i = val;
        s->params.tmu[0].dTdY = (int64_t)(f.f * 4294967296.0f);
        s->params.tmu[1].dTdY = s->params.tmu[0].dTdY;
        break;
    case SST_fdWdY:
        f.i = val;
        s->params.dWdY        = (int64_t)(f.f * 4294967296.0f);
        s->params.tmu[0].dWdY = s->params.dWdY;
        s->params.tmu[1].dWdY = s->params.dWdY;
        break;

    /* Float triangle CMD */
    case SST_ftriangleCMD:
        s->params.sign = (int)(val >> 31);
        voodoo3_queue_triangle(s, &s->params);
        s->cmd_read++;
        break;

    /* --- Buffer management (Banshee/V3 only) --- */
    case SST_colBufferAddr:
        s->params.draw_offset  = val & 0xfffff0;
        break;
    case SST_colBufferStride:
		s->params.col_tiled = !!(val & (1u << 15));
		s->params.row_width = s->params.col_tiled
							  ? (val & 0x3fffu) * 128u * 32u
							  : (val & 0x3fffu);
        break;
    case SST_auxBufferAddr:
        s->params.aux_offset = val & 0xfffff0;
        break;
    case SST_auxBufferStride:
        s->params.aux_tiled     = (int)(val & (1u << 15));
        s->params.aux_row_width = s->params.aux_tiled
                                  ? (val & 0x7fu) * 128u * 32u
                                  : (val & 0x3fffu);
        break;

    /* --- CLUT --- */
    case SST_clutData: {
        unsigned idx = (val >> 24) & 0x3fu;
        if (idx < VOODOO3_CLUT_SIZE) {
            s->pallook[idx] = ((uint32_t)((val >> 16) & 0xff) << 16) |
                              ((uint32_t)((val >>  8) & 0xff) <<  8) |
                               (uint32_t)(val & 0xff);
        }
        break;
    }



    /* --- Immediate commands --- */
    case SST_nopCMD:
        s->cmd_read++;
        s->fbiPixelsIn = s->fbiChromaFail = s->fbiZFuncFail =
        s->fbiAFuncFail = s->fbiPixelsOut = 0;
        break;
    case SST_fastfillCMD:
        /* Ported from 86Box voodoo_fastfill() in vid_voodoo_blitter.c */
        voodoo3_fastfill(s);
        s->cmd_read++;
        break;
    case SST_swapbufferCMD:
        /*
         * Ported from 86Box swapbufferCMD handler + voodoo_callback().
         * Sets swap_pending; the actual flip happens in voodoo3_vblank_cb()
         * after swap_interval vblanks have elapsed.
         */
        voodoo3_swap_buffer(s, val);
        s->cmd_read++;
        break;

    /* --- Setup engine vertex accumulator --- */
        case SST_sAlpha: { fi_t g; g.i = val; s->verts[3].sAlpha = g.f; } break;
    case SST_sVz:    { fi_t g; g.i = val; s->verts[3].sVz    = g.f; } break;
    case SST_sWb:    { fi_t g; g.i = val; s->verts[3].sWb    = g.f; } break;
    case SST_sW0:    { fi_t g; g.i = val; s->verts[3].sW0    = g.f; } break;
    case SST_sS0:    { fi_t g; g.i = val; s->verts[3].sS0    = g.f; } break;
    case SST_sT0:    { fi_t g; g.i = val; s->verts[3].sT0    = g.f; } break;
    case SST_sW1:    { fi_t g; g.i = val; s->verts[3].sW1    = g.f; } break;
    case SST_sS1:    { fi_t g; g.i = val; s->verts[3].sS1    = g.f; } break;
    case SST_sT1:    { fi_t g; g.i = val; s->verts[3].sT1    = g.f; } break;

    case SST_sBeginTriCMD:
        /* Start a new triangle strip/fan — copy staging vertex to all three */
        s->verts[0] = s->verts[3];
        s->verts[1] = s->verts[3];
        s->verts[2] = s->verts[3];
        s->vertex_next_age = 0;
        s->vertex_ages[0]  = s->vertex_next_age++;
        s->num_verticies   = 1;
        s->cull_pingpong   = 0;
        break;

    case SST_sDrawTriCMD: {
        /*
         * Vertex accumulator — ported from 86Box SST_sDrawTriCMD handler.
         * Strip mode: replace oldest vertex.
         * Fan mode:   replace second-oldest vertex.
         */
        if (s->vertex_next_age < 3) {
            int vn = s->vertex_next_age;
            s->verts[vn]       = s->verts[3];
            s->vertex_ages[vn] = s->vertex_next_age++;
        } else {
            int vn = 0;
            if (!(s->sSetupMode & SETUPMODE_STRIP_MODE)) {
                /* Fan: find oldest */
                if (s->vertex_ages[0] < s->vertex_ages[1] &&
                    s->vertex_ages[0] < s->vertex_ages[2])       vn = 0;
                else if (s->vertex_ages[1] < s->vertex_ages[2])  vn = 1;
                else                                              vn = 2;
            } else {
                /* Strip: find second-oldest (pivot around oldest) */
                if ((s->vertex_ages[1] < s->vertex_ages[0] &&
                     s->vertex_ages[0] < s->vertex_ages[2]) ||
                    (s->vertex_ages[2] < s->vertex_ages[0] &&
                     s->vertex_ages[0] < s->vertex_ages[1]))      vn = 0;
                else if ((s->vertex_ages[0] < s->vertex_ages[1] &&
                          s->vertex_ages[1] < s->vertex_ages[2]) ||
                         (s->vertex_ages[2] < s->vertex_ages[1] &&
                          s->vertex_ages[1] < s->vertex_ages[0])) vn = 1;
                else                                               vn = 2;
            }
            s->verts[vn]       = s->verts[3];
            s->vertex_ages[vn] = s->vertex_next_age++;
        }
        s->num_verticies++;
        if (s->num_verticies >= 3) {
            /*
             * Triangle setup — ported from 86Box voodoo_triangle_setup().
             * Converts s->verts[0..2] (float) → fixed-point params and
             * calls voodoo3_queue_triangle() (hw/display/voodoo3_setup.c).
             */
            voodoo3_triangle_setup(s);
            s->cull_pingpong = !s->cull_pingpong;
            s->num_verticies = 2;
        }
        break;
    }

    } /* switch */
}

/* =========================================================================
 * Banshee 2D blitter dispatch
 * Ported from 86Box banshee_blt_execute() / banshee_blt_start()
 * ========================================================================= */
static void voodoo3_blt_execute(Voodoo3State *s)
{
    voodoo3_blt_t *blt = &s->blt;
    uint32_t cmd = blt->command & COMMAND_CMD_MASK;

    /*
     * dst/src geometry is now decoded directly in the register write handlers
     * (dstW/dstH/dstX/dstY/srcX/srcY from dstSize/dstXY/srcXY).
     * dstStride/srcStride are kept up-to-date by voodoo3_blt_update_*_stride().
     *
     * dstBpp: decode from dstFormat bits[2:0] (DST_FORMAT_COL field).
     *   0 = 8bpp, 1 = 16bpp, 2 = 24bpp, 3 = 32bpp  (same as pix_format).
     * Default to 16bpp (RGB565) which is what AmigaOS P96 uses.
     */
    /* DST_FORMAT_COL = bits[19:16] (86Box: DST_FORMAT_COL_MASK = 0xf<<16) */
    switch ((blt->dstFormat >> 16) & 0xfu) {
    case 1:  blt->dstBpp = 1; break;   /* 8-bpp  */
    case 3:  blt->dstBpp = 2; break;   /* 16-bpp */
    case 4:  blt->dstBpp = 3; break;   /* 24-bpp */
    case 5:  blt->dstBpp = 4; break;   /* 32-bpp */
    default: blt->dstBpp = 2; break;   /* default 16-bpp */
    }

    switch (cmd) {
    case COMMAND_CMD_NOP:
        break;

    case COMMAND_CMD_RECTFILL: {
        /*
         * Solid rectangle fill.
         * Ported from 86Box banshee_do_rectfill().
         */
        uint32_t color    = blt->colorFore;
        uint32_t dst_base = blt->dstBaseAddr & 0xffffffu;
        int bpp           = blt->dstBpp;
        int x, y;

        for (y = 0; y < blt->dstH && (blt->dstY + y) < s->screen_height; y++) {
            uint8_t *row = s->fb_mem + dst_base
                         + (size_t)(blt->dstY + y) * blt->dstStride
                         + (size_t)blt->dstX * bpp;
            for (x = 0; x < blt->dstW; x++) {
                switch (bpp) {
                case 1: row[x] = (uint8_t)color; break;
                case 2: ((uint16_t *)row)[x] = (uint16_t)color; break;
                case 3:
                    row[x*3+0] =  color        & 0xff;
                    row[x*3+1] = (color >>  8) & 0xff;
                    row[x*3+2] = (color >> 16) & 0xff;
                    break;
                case 4: ((uint32_t *)row)[x] = color; break;
                }
            }
            /* Mark dirty if drawing to front buffer */
            int abs_y = blt->dstY + y;
            if (dst_base == (uint32_t)s->params.front_offset
                && abs_y < V3_DIRTY_LINES)
                s->dirty_line[abs_y] = 1;
        }
        break;
    }

    case COMMAND_CMD_S2S_BLT:
    /*
     * Cmd 9 = Screen-to-Screen Transparent Blt (chroma-key).
     * Cmd 12 = variant used by AmigaOS OS4 P96 (Banshee extended cmd,
     *          functionally S2S with optional chroma; treat as plain S2S).
     * All three share the same copy engine; chroma-key filtering is a
     * TODO (colorkeyMin/Max are stored but not yet applied).
     */
    case 9:
    case 12:
    case COMMAND_CMD_S2S_STRETCH: {
        /*
         * Screen-to-screen blit.
         * Ported from 86Box banshee_do_screen_to_screen_blt().
         * Simple copy with correct src/dst stride, no ROP yet.
         */
        uint32_t src_base = blt->srcBaseAddr & 0xffffffu;
        uint32_t dst_base = blt->dstBaseAddr & 0xffffffu;
        int bpp = blt->dstBpp;
        int y;

        for (y = 0; y < blt->dstH; y++) {
            const uint8_t *src_row = s->fb_mem + src_base
                                   + (size_t)(blt->srcY + y) * blt->srcStride
                                   + (size_t)blt->srcX * bpp;
            uint8_t *dst_row = s->fb_mem + dst_base
                              + (size_t)(blt->dstY + y) * blt->dstStride
                              + (size_t)blt->dstX * bpp;
            memmove(dst_row, src_row, (size_t)blt->dstW * bpp);

            int abs_y = blt->dstY + y;
            if (dst_base == (uint32_t)s->params.front_offset
                && abs_y < V3_DIRTY_LINES)
                s->dirty_line[abs_y] = 1;
        }
        break;
    }

    case COMMAND_CMD_H2S_BLT:
        /* Host-to-screen: initialise the per-row accumulation state.
         * Pixel data arrives as 32-bit words written to the launch registers
         * (0x80..0xfc); each write calls voodoo3_blt_h2s_write(). */
        blt->host_data_count = 0;
        blt->cur_y           = 0;
        /* src_stride_dest: bytes per source row aligned to dword boundary */
        {
            /* SRC_FORMAT_COL = bits[19:16] of srcFormat (86Box spec) */
            int src_col = (int)((blt->srcFormat >> 16) & 0xf);
            int src_bpp;
            switch (src_col) {
            case 0:  src_bpp = 1; break;   /* 1-bpp mono   */
            case 1:  src_bpp = 1; break;   /* 8-bpp        */
            case 3:  src_bpp = 2; break;   /* 16-bpp       */
            case 4:  src_bpp = 3; break;   /* 24-bpp       */
            case 5:  src_bpp = 4; break;   /* 32-bpp       */
            default: src_bpp = blt->dstBpp; break; /* fallback */
            }
            blt->src_stride_dest = ((blt->dstW * src_bpp) + 3) & ~3;
            /* Also update dstBpp to match for correct memcpy */
            if (src_col != 0)   /* don't override for 1bpp mono */
                blt->dstBpp = src_bpp;
        }
        break;

    case COMMAND_CMD_LINE: {
        /*
         * Banshee 2D Line Draw — Bresenham algorithm.
         *
         * Register mapping (confirmed from 86Box vid_voodoo_banshee_blitter.c):
         *   dstXY      (0x6c): bits[12:0]=X0, bits[28:16]=Y0  (start point)
         *   dstSize    (0x68): bits[12:0]=dX (abs), bits[28:16]=dY (abs)
         *   bresError0 (0x28): initial error term E
         *   bresError1 (0x2c): K1 = 2*dMinor (error increment for minor step)
         *   colorFore  (0x64): line colour
         *   COMMAND_DX (bit 14): X direction (0=+, 1=-)
         *   COMMAND_DY (bit 15): Y direction (0=+, 1=-)
         *   dstSize axis convention: the LARGER delta is the major axis.
         *
         * The Launch Area write (0x80..0xfc) provides the END coordinate
         * (dstXY of the endpoint) — we use it just to trigger execution;
         * the pixel count is encoded in dstSize (the major-axis length).
         *
         * Bresenham state:
         *   x, y         = current pixel
         *   err          = bresError0 (signed 32-bit)
         *   k1           = bresError1 = 2 * dMinor
         *   k2           = 2 * dMinor - 2 * dMajor  (computed here)
         *   major_steps  = major-axis length in pixels
         *
         * Step rule (standard Bresenham, matches 86Box):
         *   if (err > 0) { step_minor(); err += k2; }
         *   else         {              err += k1; }
         *   always: step_major();
         */
        uint32_t color    = blt->colorFore;
        uint32_t dst_base = blt->dstBaseAddr & 0xffffffu;
        int bpp           = blt->dstBpp;
        int dx_mag        = blt->dstW;   /* |ΔX| */
        int dy_mag        = blt->dstH;   /* |ΔY| */
        int dx_sign       = (blt->command & COMMAND_DX) ? -1 : 1;
        int dy_sign       = (blt->command & COMMAND_DY) ? -1 : 1;

        /* Determine major / minor axes */
        bool x_major = (dx_mag >= dy_mag);
        int major_steps = x_major ? dx_mag : dy_mag;

        /* Bresenham error terms */
        int32_t err = (int32_t)blt->bresError0;
        int32_t k1  = (int32_t)blt->bresError1;   /* 2 * dMinor */
        int dMajor  = x_major ? dx_mag : dy_mag;
        int32_t k2  = k1 - 2 * dMajor;            /* 2*dMinor - 2*dMajor */

        int x = blt->dstX;
        int y = blt->dstY;

        for (int step = 0; step <= major_steps; step++) {
            /* Plot pixel (x, y) */
            if (x >= 0 && y >= 0 &&
                x < s->screen_width &&
                y < s->screen_height) {
                uint8_t *row = s->fb_mem + dst_base
                             + (size_t)y * blt->dstStride
                             + (size_t)x * bpp;
                switch (bpp) {
                case 1: *row = (uint8_t)color; break;
                case 2: *(uint16_t *)row = (uint16_t)color; break;
                case 3:
                    row[0] =  color        & 0xff;
                    row[1] = (color >>  8) & 0xff;
                    row[2] = (color >> 16) & 0xff;
                    break;
                case 4: *(uint32_t *)row = color; break;
                }
                if (dst_base == (uint32_t)s->params.front_offset
                    && y < V3_DIRTY_LINES)
                    s->dirty_line[y] = 1;
            }

            /* Bresenham step */
            if (err > 0) {
                /* Step minor axis */
                if (x_major) y += dy_sign;
                else         x += dx_sign;
                err += k2;
            } else {
                err += k1;
            }
            /* Step major axis */
            if (x_major) x += dx_sign;
            else         y += dy_sign;
        }
        break;
    }

    case COMMAND_CMD_POLYLINE:
    case COMMAND_CMD_POLYFILL:
    case COMMAND_CMD_H2S_STRETCH:
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: 2D cmd %u (stub)\n", cmd);
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "voodoo3: unknown 2D cmd %u\n", cmd);
        break;
    }
}

/*
 * Helper: recompute dst_stride from dstBaseAddr tiled flag + dstFormat.
 * Ported from 86Box banshee_blt_write() dstBaseAddr/dstFormat handlers.
 * DST_FORMAT_STRIDE_MASK = bits[12:0] = stride in bytes (non-tiled)
 *                        or number of 128-byte tile columns (tiled).
 */
/*
 * DST/SRC_FORMAT stride field: bits[13:0] = direct byte stride (86Box confirmed).
 * Mask is 0x3fff not 0x1fff — old value capped stride at 8191 bytes.
 */
#define DST_FORMAT_STRIDE_MASK  0x3fffu
#define SRC_FORMAT_STRIDE_MASK  0x3fffu

static void voodoo3_blt_update_dst_stride(Voodoo3State *s)
{
    if (s->blt.dstBaseAddr & 0x80000000u)
        s->blt.dstStride = (s->blt.dstFormat & DST_FORMAT_STRIDE_MASK) * 128u * 32u;
    else
        s->blt.dstStride = s->blt.dstFormat & DST_FORMAT_STRIDE_MASK;
}

static void voodoo3_blt_update_src_stride(Voodoo3State *s)
{
    if (s->blt.srcBaseAddr & 0x80000000u)
        s->blt.srcStride = (s->blt.srcFormat & SRC_FORMAT_STRIDE_MASK) * 128u * 32u;
    else
        s->blt.srcStride = s->blt.srcFormat & SRC_FORMAT_STRIDE_MASK;
}

static void voodoo3_2d_reg_write(Voodoo3State *s, uint32_t addr, uint32_t val)
{
    voodoo3_blt_t *blt = &s->blt;
    /*
     * Banshee 2D engine register map (BAR0 + 0x100000 base).
     * Ported from 86Box voodoo_2d_reg_writel() in vid_voodoo_banshee_blitter.c.
     *
     * Registers 0x80..0xfc are "launch" registers: writing them stores srcXY
     * (for S2S_BLT) or host pixel data (for H2S_BLT) and then triggers
     * execution if launch_pending is set.  This is how AmigaOS OS4 P96 drivers
     * initiate blits — they write the command to 0x70 (no INITIATE bit), then
     * write srcXY to 0x80 which fires the blt.
     */
    uint32_t off = addr & 0x1fc;

    /* ---- Launch registers 0x80..0xfc ---- */
    if (off >= 0x80 && off <= 0xfc) {
        if (s->blt.launch_pending) {
            /* Resolve dst_stride/src_stride before execute */
            voodoo3_blt_update_dst_stride(s);
            voodoo3_blt_update_src_stride(s);
            s->blt.launch_pending = 0;
        }
        /* Dispatch per-command launch data */
        switch (s->blt.command & COMMAND_CMD_MASK) {
        case COMMAND_CMD_S2S_BLT:
        case COMMAND_CMD_S2S_STRETCH:
            /* val = srcXY: bits[12:0]=srcX, bits[28:16]=srcY */
            s->blt.srcXY = val;
            s->blt.srcX  = (int)(val & 0x1fff);
            s->blt.srcY  = (int)((val >> 16) & 0x1fff);
            voodoo3_blt_execute(s);
            break;
        case COMMAND_CMD_H2S_BLT:
        case COMMAND_CMD_H2S_STRETCH:
        {
            /* Accumulate 4 bytes of host pixel data */
            if (blt->host_data_count + 4 <= (int)sizeof(blt->host_data)) {
                blt->host_data[blt->host_data_count+0] = (uint8_t)(val);
                blt->host_data[blt->host_data_count+1] = (uint8_t)(val >> 8);
                blt->host_data[blt->host_data_count+2] = (uint8_t)(val >> 16);
                blt->host_data[blt->host_data_count+3] = (uint8_t)(val >> 24);
                blt->host_data_count += 4;
            }
            /* When we have accumulated a full source row, write it out */
            while (blt->src_stride_dest > 0 &&
                   blt->host_data_count >= blt->src_stride_dest &&
                   blt->cur_y < blt->dstH) {
                /* Compute destination address in VRAM */
                uint32_t dst_addr = (blt->dstBaseAddr & 0xffffffu)
                    + (size_t)(blt->dstY + blt->cur_y) * blt->dstStride
                    + (size_t)blt->dstX * blt->dstBpp;
                if (dst_addr + blt->dstW * blt->dstBpp <= s->fb_size) {
                    memcpy(s->fb_mem + dst_addr,
                           blt->host_data,
                           (size_t)blt->dstW * blt->dstBpp);
                    /* Mark scanline dirty */
                    int abs_y = blt->dstY + blt->cur_y;
                    if (abs_y >= 0 && abs_y < V3_DIRTY_LINES)
                        s->dirty_line[abs_y] = 1;
                }
                blt->cur_y++;
                /* Shift remaining bytes to front */
                int remaining = blt->host_data_count - blt->src_stride_dest;
                if (remaining > 0)
                    memmove(blt->host_data,
                            blt->host_data + blt->src_stride_dest,
                            (size_t)remaining);
                blt->host_data_count = remaining;
            }
            break;
        }
        default:
            voodoo3_blt_execute(s);
            break;
        }
        return;
    }

    switch (off) {
    /* ---- Complete Banshee 2D register map from 86Box ---- */
    case 0x08:
        s->blt.clip0Min = val;
        break;
    case 0x0c:
        s->blt.clip0Max = val;
        break;
    case 0x10:
        /*
         * dstBaseAddr: bit[31] = tiled flag.
         * Recompute dst_stride whenever base addr or tiling changes.
         */
        s->blt.dstBaseAddr = val & 0xffffffu;
        s->blt.dstTiled    = !!(val & 0x80000000u);
        voodoo3_blt_update_dst_stride(s);
        break;
    case 0x14:
        s->blt.dstFormat = val;
        voodoo3_blt_update_dst_stride(s);
        break;
    case 0x18: s->blt.srcColorkeyMin = val & 0xffffffu; break;
    case 0x1c: s->blt.srcColorkeyMax = val & 0xffffffu; break;
    case 0x20: s->blt.dstColorkeyMin = val & 0xffffffu; break;
    case 0x24: s->blt.dstColorkeyMax = val & 0xffffffu; break;
    case 0x28: s->blt.bresError0  = val; break;
    case 0x2c: s->blt.bresError1  = val; break;
    case 0x30: s->blt.rop         = val; break;
    case 0x34:
        s->blt.srcBaseAddr = val & 0xffffffu;
        s->blt.srcTiled    = !!(val & 0x80000000u);
        voodoo3_blt_update_src_stride(s);
        break;
    case 0x38: s->blt.commandExtra = val; break;
    case 0x3c: s->blt.lineStipple  = val; break;
    case 0x40: s->blt.lineStyle    = val; break;
    case 0x44: s->blt.pattern0    = val; break;
    case 0x48: s->blt.pattern1    = val; break;
    case 0x4c: /* reserved */ break;
    case 0x50: /* reserved */ break;
    case 0x54:
        /*
         * srcFormat: colour depth, packing, stride.
         * 86Box: also recomputes src_stride here.
         */
        s->blt.srcFormat = val;
        voodoo3_blt_update_src_stride(s);
        break;
    case 0x58:
        s->blt.srcSize = val;
        s->blt.srcW    = (int)(val & 0x1fffu);
        s->blt.srcH    = (int)((val >> 16) & 0x1fffu);
        break;
    case 0x5c:
        s->blt.srcXY = val;
        s->blt.srcX  = (int)(val & 0x1fffu);
        s->blt.srcY  = (int)((val >> 16) & 0x1fffu);
        break;
    case 0x60: s->blt.colorBack = val; break;
    case 0x64: s->blt.colorFore = val; break;
    case 0x68:
        s->blt.dstSize = val;
        s->blt.dstW    = (int)(val & 0x1fffu);
        s->blt.dstH    = (int)((val >> 16) & 0x1fffu);
        break;
    case 0x6c:
        s->blt.dstXY = val;
        s->blt.dstX  = (int)(val & 0x1fffu);
        s->blt.dstY  = (int)((val >> 16) & 0x1fffu);
        break;
    case 0x70:
        /*
         * Command register.  86Box: sets launch_pending=1, then dispatches
         * only POLYFILL and H2S specially; all others wait for a launch-reg
         * write (0x80..0xfc) to actually fire.  If COMMAND_INITIATE is set,
         * fire immediately (same as 86Box default-case INITIATE path).
         */
        s->blt.command       = val;
        s->blt.launch_pending = 1;
        if (val & COMMAND_INITIATE) {
            voodoo3_blt_update_dst_stride(s);
            voodoo3_blt_update_src_stride(s);
            s->blt.launch_pending = 0;
            voodoo3_blt_execute(s);
        }
        break;
    /*
     * 0x054 = srcFormat in write path (handled by case 0x54 above).
     *         engineStatus is read-only; reads return 0 (idle) in the
     *         read handler. No write case needed here.
     * 0x080..0xfc = launch registers (handled at top of function).
     */
    default:
        s->regs[(off) >> 2] = val;
        break;
    }
}

/* =========================================================================
 * BAR0 MMIO read/write
 * ========================================================================= */
static uint64_t voodoo3_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    Voodoo3State *s = VOODOO3_PCI(opaque);
    uint32_t      ret;

    switch (addr & 0x1f00000) {
    case BAR0_IO_REMAP:
        ret = (addr & 0x80000) ? 0xffffffff
                               : voodoo3_ext_read(s, addr & 0xff);
        break;
    case BAR0_2D_REGS:
        switch (addr & 0x1fc) {
        case 0x000: ret = voodoo3_status(s);               break;
        case 0x004: ret = s->intrCtrl & 0x0030003f;        break;
        /*
         * 0x054 = engineStatus (86Box banshee 2D FIFO/engine status).
         * Bit 31 = trapezoidal engine busy, bit 9 = FIFO not empty.
         * Return 0 = engine idle so AmigaOS blt waits don't spin.
         */
        case BLT2D_STATUS: ret = 0x00000000u;               break;
        case 0x010: ret = s->blt.dstBaseAddr;               break;
        case 0x014: ret = s->blt.dstFormat;                 break;
        case 0x034: ret = s->blt.srcBaseAddr;               break;
        /*
         * 0x080 = srcBaseAddr2 / blt expand port (write-only in HW).
         * Return 0 on reads — AmigaOS graphic.library bitmap expand path
         * does NOT read this back; it's only written to.
         */
        case BLT2D_SRCBASE: ret = 0x00000000u;              break;
        case 0x070: ret = s->blt.command;                   break;
        default:
            ret = s->regs[(addr & 0x1fc) >> 2];
            break;
        }
        break;
    case BAR0_3D_LO:
    case 0x0300000:
    case 0x0400000:
    case BAR0_3D_HI:
        switch (addr & 0x3fc) {
        case SST_status:   ret = voodoo3_status(s);        break;
        case SST_intrCtrl: ret = s->intrCtrl & 0x0030003f; break;
        case SST_fbzColorPath: ret = s->params.fbzColorPath; break;
        case SST_fbzMode:  ret = s->params.fbzMode;         break;
        case SST_alphaMode:ret = s->params.alphaMode;       break;
        case SST_lfbMode:  ret = s->lfbMode;                break;
        default:
            qemu_log_mask(LOG_UNIMP,
                "voodoo3: 3D reg read 0x%03x\n", (unsigned)(addr & 0x3fc));
            ret = 0xffffffff;
            break;
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: MMIO read 0x%"HWADDR_PRIx"\n", addr);
        ret = 0xffffffff;
        break;
    }

    return (uint64_t)ret;
}

static void voodoo3_mmio_write(void *opaque, hwaddr addr,
                               uint64_t data, unsigned size)
{
    Voodoo3State *s   = VOODOO3_PCI(opaque);
    uint32_t      val = (uint32_t)data;

    switch (addr & 0x1f00000) {
    case BAR0_IO_REMAP:
        if (!(addr & 0x80000))
            voodoo3_ext_write(s, addr & 0xff, val);
        else
            qemu_log_mask(LOG_UNIMP,
                "voodoo3: CMDFIFO write 0x%"HWADDR_PRIx" = 0x%08x\n",
                addr, val);
        break;
    case BAR0_2D_REGS:
        if ((addr & 0x3fc) == SST_intrCtrl)
            s->intrCtrl = val & 0x0030003f;
        else
            voodoo3_2d_reg_write(s, addr, val);
        break;
    case BAR0_3D_LO:
    case 0x0300000:
    case 0x0400000:
    case BAR0_3D_HI:
        if ((addr & 0x3fc) == SST_intrCtrl)
            s->intrCtrl = val & 0x0030003f;
        else
            voodoo3_3d_reg_write(s, addr, val);
        break;
    case BAR0_TEX0:
    case 0x0700000:
    case BAR0_TEX1:
    case 0x0900000:
        /* Texture download — queue for texture engine */
        voodoo3_push_fifo(s, (uint32_t)(addr & 0x1ffffc) | FIFO_WRITEL_TEX, val);
        qemu_cond_signal(&s->fifo_cond);
        break;
    case 0x1000000: case 0x1100000: case 0x1200000: case 0x1300000:
    case 0x1400000: case 0x1500000: case 0x1600000: case 0x1700000:
    case 0x1800000: case 0x1900000: case 0x1a00000: case 0x1b00000:
    case 0x1c00000: case 0x1d00000: case 0x1e00000: case 0x1f00000:
        /* 3D LFB aperture — pixel data through render pipeline */
        voodoo3_push_fifo(s, (uint32_t)(addr & 0xfffffc) | FIFO_WRITEL_FB, val);
        qemu_cond_signal(&s->fifo_cond);
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: MMIO write 0x%"HWADDR_PRIx" = 0x%08x\n", addr, val);
        break;
    }
}

static const MemoryRegionOps voodoo3_mmio_ops = {
    .read  = voodoo3_mmio_read,
    .write = voodoo3_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid  = { .min_access_size = 1, .max_access_size = 4 },
    .impl   = { .min_access_size = 4, .max_access_size = 4 },
};

/* =========================================================================
 * BAR1: Linear Framebuffer (with tiled address decode)
 * ========================================================================= */
static hwaddr voodoo3_untile(Voodoo3State *s, hwaddr addr)
{
    if (s->tile_stride && addr >= s->tile_base) {
        hwaddr   rel  = addr - s->tile_base;
        uint32_t x    = (uint32_t)(rel & (s->tile_stride - 1));
        uint32_t y    = (uint32_t)(rel >> __builtin_ctz(s->tile_stride));
        addr = s->tile_base
             + (x & 127u)
             + ((x >> 7) * 128u * 32u)
             + ((y & 31u) * 128u)
             + (y >> 5) * s->tile_x * 32u;
    }
    return addr;
}

static uint64_t voodoo3_lfb_read(void *opaque, hwaddr addr, unsigned size)
{
    Voodoo3State *s = VOODOO3_PCI(opaque);
    uint64_t      val = 0;
    addr = voodoo3_untile(s, addr);
    if (addr + size > s->fb_size) return 0xffffffffffffffffULL;
    memcpy(&val, s->fb_mem + addr, size);
    return val;
}

static void voodoo3_lfb_write(void *opaque, hwaddr addr,
                              uint64_t data, unsigned size)
{
    Voodoo3State *s = VOODOO3_PCI(opaque);
    hwaddr phys = voodoo3_untile(s, addr);
    if (phys + size > s->fb_size) return;

    /*
     * If this write lands in the cursor pattern area, mirror the raw bytes
     * into cursor_buf[] BEFORE the QEMU endian layer re-orders them.
     * The cursor bitmap is 1bpp (8 pixels per byte, MSB first); we must
     * preserve the byte order as the driver intended, not as QEMU stores it
     * after applying DEVICE_LITTLE_ENDIAN byte-swapping on big-endian buses.
     *
     * We capture the bytes from `data` in the order the driver sent them:
     * for a 4-byte write on PPC, QEMU delivers data[] already swapped back
     * to LE, so we can simply copy size bytes from &data at offset phys-base.
     */
    if (s->cur_pat_addr && size >= 1) {
        uint32_t base = s->cur_pat_addr;
        if (phys >= base && phys + size <= base + 1024u) {
            uint32_t off = (uint32_t)(phys - base);

            for (unsigned i = 0; i < size && off + i < 1024u; i++)
                s->cursor_buf[off + i] = (uint8_t)(data >> (i * 8));
        }
    }

    memcpy(s->fb_mem + phys, &data, size);
}

static const MemoryRegionOps voodoo3_lfb_ops = {
    .read  = voodoo3_lfb_read,
    .write = voodoo3_lfb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /* Accept 8-byte guest writes (e.g. 64-bit STQ on PPC/x86-64);
     * the impl limit tells QEMU to split them into two 4-byte calls
     * before they reach voodoo3_lfb_write(), which only handles <= 4. */
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

/* =========================================================================
 * BAR2: Legacy I/O
 * ========================================================================= */
static uint64_t voodoo3_io_read(void *opaque, hwaddr addr, unsigned size)
{
    return (uint64_t)voodoo3_ext_read(VOODOO3_PCI(opaque), (uint32_t)(addr & 0xff));
}
static void voodoo3_io_write(void *opaque, hwaddr addr,
                             uint64_t data, unsigned size)
{
    voodoo3_ext_write(VOODOO3_PCI(opaque), (uint32_t)(addr & 0xff),
                      (uint32_t)data);
}
static const MemoryRegionOps voodoo3_io_ops = {
    .read  = voodoo3_io_read,
    .write = voodoo3_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* =========================================================================
 * Display output — pixel-format-aware blit to QEMU console surface
 * ========================================================================= */
/*
 * voodoo3_update_display — called by QEMU display subsystem on demand.
 * The actual work is done by voodoo3_update_display_dirty() in the
 * vblank callback, which tracks dirty lines for efficiency.
 * This function is kept as a fallback for full-screen invalidation.
 */
static void voodoo3_update_display(void *opaque)
{
    Voodoo3State *s = VOODOO3_PCI(opaque);
    /* Force full redraw by marking all lines dirty */
    memset(s->dirty_line, 1, sizeof(s->dirty_line));
    voodoo3_update_display_dirty(s);
}

static void voodoo3_invalidate_display(void *opaque) { (void)opaque; }

static const GraphicHwOps voodoo3_gfx_ops = {
    .gfx_update = voodoo3_update_display,
    .invalidate = voodoo3_invalidate_display,
};

/* =========================================================================
 * Vblank timer
 * ========================================================================= */
static void voodoo3_vblank_cb(void *opaque)
{
    Voodoo3State *s = VOODOO3_PCI(opaque);

    s->in_vblank = true;

    /* Check and execute pending buffer swap */
    voodoo3_do_swap_if_pending(s);

    /* Dirty-line-aware display update (ported from 86Box voodoo_callback) */
    if (s->display_enabled && s->con)
        voodoo3_update_display_dirty(s);

    s->in_vblank = false;

    timer_mod(s->vblank_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / VBLANK_HZ);
}

/* =========================================================================
 * Render threads
 *
 * Mirrors 86Box voodoo_render_thread_1..4 / render_thread().
 * Each thread handles every Nth triangle (band-parallel rendering).
 * The odd_even parameter selects which triangles this thread processes.
 *
 * TODO: call voodoo_triangle() once the rasterizer is ported.
 * ========================================================================= */
static void *voodoo3_render_thread(void *arg)
{
    uintptr_t     tid = (uintptr_t)((uint64_t *)arg)[0];
    Voodoo3State *s   = (Voodoo3State *)((uint64_t *)arg)[1];
    g_free(arg);

    qemu_mutex_lock(&s->render_lock);
    while (!s->render_stop) {
        qemu_cond_wait(&s->render_cond, &s->render_lock);
        if (s->render_stop) break;

        /*
         * Thread 0 processes the command FIFO (2D regs, TEX downloads).
         * Other threads only rasterize triangles from the param ring.
         * This mirrors 86Box where the FIFO thread is separate.
         */
        if (tid == 0) {
            while (s->fifo_rd != s->fifo_wr) {
                uint32_t cmd = s->fifo_cmd[s->fifo_rd];
                uint32_t val = s->fifo_val[s->fifo_rd];
                s->fifo_rd   = (s->fifo_rd + 1) & (V3_FIFO_SIZE - 1);
                s->cmd_read++;

                uint32_t type = cmd & 0xff800000u;
                if (type == FIFO_WRITEL_TEX) {
                    /* Texture download — ported from 86Box voodoo_tex_writel() */
                    int tmu = (cmd & 0x200000u) ? 1 : 0;
                    voodoo3_tex_download(s, cmd, val, tmu);
                }
                /* 2D-reg and FB writes handled directly by write callbacks */
            }
        }

        /* Rasterize all triangles assigned to this thread */
        while (s->param_rd[tid] != s->param_wr) {
            uint32_t idx = s->param_rd[tid] & (PARAM_BUF_SIZE - 1);
            voodoo3_params_t *p = &s->param_buf[idx];
            /* Full pixel rasterizer — ported from 86Box voodoo_triangle() */
            voodoo3_triangle(s, p);
            s->param_rd[tid]++;
        }

        /* Check if all threads are caught up */
        bool all_done = true;
        for (uint32_t i = 0; i < s->render_threads_count; i++) {
            if (s->param_rd[i] != s->param_wr) { all_done = false; break; }
        }
        if (all_done) s->voodoo_busy = false;
    }
    qemu_mutex_unlock(&s->render_lock);
    return NULL;
}

/* =========================================================================
 * Power-on register defaults
 * ========================================================================= */
static void voodoo3_reset_state(Voodoo3State *s)
{
    memset(s->regs,    0, sizeof(s->regs));
    memset(s->pallook, 0, sizeof(s->pallook));
    memset(&s->params, 0, sizeof(s->params));
    memset(&s->blt,    0, sizeof(s->blt));
    memset(s->verts,   0, sizeof(s->verts));

    s->miscInit0  = 0;
    s->miscInit1  = 0;
    s->pciInit0   = 0x01000100;
    s->dramInit0  = 0x00050000;
    s->pllCtrl0 = s->pllCtrl1 = s->pllCtrl2 = 0;
    s->dacMode  = 0;
    s->dacAddr  = 0;
    s->intrCtrl = 0;
    s->vidProcCfg       = 0;
    s->vidScreenSize    = 0;
    s->lfbMemoryConfig  = 0;
    s->tile_base = s->tile_stride = s->tile_x = 0;
    s->y_origin_swap = 0;

    s->screen_width    = 640;
    s->screen_height   = 480;
    s->desktop_stride  = 640 * 2;    /* RGB565 default: 2 bytes/pixel */
    s->desktop_start   = 0;
    s->pix_format      = PIX_FORMAT_RGB565;
    s->display_enabled = false;
    s->in_vblank       = false;

    /*
     * Sync params from desktop registers so the display scanout has a
     * valid framebuffer address and stride from the very first vblank,
     * even before any 3D colBufferAddr/colBufferStride writes arrive.
     */
    s->params.front_offset = s->desktop_start;
    s->params.draw_offset  = s->desktop_start;
    s->params.row_width    = s->desktop_stride;
    s->params.col_tiled    = 0;

    s->cmd_written = s->cmd_read = 0;
    s->fifo_wr = s->fifo_rd = 0;
    s->param_wr = 0;
    for (int i = 0; i < MAX_RENDER_THREADS; i++) s->param_rd[i] = 0;
    s->voodoo_busy  = false;
    s->tri_count    = 0;
    s->num_verticies = 0;
    s->vertex_next_age = 0;
    s->cull_pingpong   = 0;
    s->sSetupMode      = 0;
    s->ncc_dirty[0] = s->ncc_dirty[1] = 0;
    memset(s->ncc_table,  0, sizeof(s->ncc_table));
    memset(s->ncc_lookup, 0, sizeof(s->ncc_lookup));

    /* CRTC / DAC indexed register state */
    memset(s->crtc_ctrl,  0, sizeof(s->crtc_ctrl));
    memset(s->crtc_freq,  0, sizeof(s->crtc_freq));
    memset(s->dac_reset,  0, sizeof(s->dac_reset));
    s->crtc_idx       = 0;
    s->crtc_freq_idx  = 0;
    s->dac_reset_idx  = 0;
    s->vidSerialParallelPort = 0;
    s->swap_pending  = false;
    s->swap_interval = 0;
    s->swap_offset   = 0;
    s->retrace_count = 0;
    s->frame_count   = 0;
    memset(s->dirty_line, 1, sizeof(s->dirty_line));  /* force full redraw */

    if (s->fb_mem) memset(s->fb_mem, 0, s->fb_size);
    for (int t = 0; t < 2; t++) {
        if (s->tex_mem[t]) memset(s->tex_mem[t], 0, s->tex_mem_size);
        memset(s->tex_cache[t], 0, sizeof(s->tex_cache[t]));
        s->tex_lru[t] = 0;
    }
}

/* =========================================================================
 * PCI realize
 * ========================================================================= */
static void voodoo3_pci_realize(PCIDevice *pci_dev, Error **errp)
{
    Voodoo3State *s   = VOODOO3_PCI(pci_dev);
    uint8_t      *cfg = pci_dev->config;

    /* Initialize dither tables (safe to call multiple times) */
    voodoo3_init_dither_tables();

    pci_config_set_class(cfg, PCI_CLASS_DISPLAY_VGA);

    /* PCI IDs */
    pci_set_word(cfg + PCI_VENDOR_ID, PCI_VENDOR_ID_3DFX);
    qemu_log_mask(LOG_UNIMP,
        "voodoo3: selected model=%d (is_agp=%d)\n",
        s->model, s->is_agp);

    /* -----------------------------------------------------------------------
     * Device ID
     * Banshee uses 0x0003; all Voodoo3 variants (V3-1000..3500) use 0x0005.
     * Source: 86Box vid_voodoo_banshee.c + 3dfx PCI IDs.
     * ----------------------------------------------------------------------- */
    switch (s->model) {
    case VOODOO3_MODEL_BANSHEE:
        pci_set_word(cfg + PCI_DEVICE_ID, PCI_DEVICE_ID_3DFX_BANSHEE);
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: DEVICE_ID = BANSHEE (0x%04x)\n",
            PCI_DEVICE_ID_3DFX_BANSHEE);
        break;
    case VOODOO3_MODEL_V3_1000:
    case VOODOO3_MODEL_V3_2000:
    case VOODOO3_MODEL_V3_3000:
    case VOODOO3_MODEL_V3_3500TV:
    default:
        pci_set_word(cfg + PCI_DEVICE_ID, PCI_DEVICE_ID_3DFX_VOODOO3);
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: DEVICE_ID = VOODOO3 (0x%04x)\n",
            PCI_DEVICE_ID_3DFX_VOODOO3);
        break;
    }

    /* -----------------------------------------------------------------------
     * Subsystem Vendor + Device ID
     * Subsystem Vendor is always 0x121A (3dfx Interactive).
     * Subsystem Device IDs from 86Box vid_voodoo_banshee.c pci_regs[0x2e]:
     *   Banshee          0x0003
     *   V3-1000          0x0052  (PCI only — no AGP variant sold)
     *   V3-2000 PCI      0x0030  /  AGP  0x0038
     *   V3-3000 PCI      0x003A  /  AGP  0x003C
     *   V3-3500          0x0060  (AGP only)
     * ----------------------------------------------------------------------- */
    pci_set_word(cfg + PCI_SUBSYSTEM_VENDOR_ID, PCI_VENDOR_ID_3DFX);
    switch (s->model) {
    case VOODOO3_MODEL_BANSHEE:
        pci_set_word(cfg + PCI_SUBSYSTEM_ID, 0x0003);
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: SUBSYSTEM_ID = BANSHEE (0x0003)\n");
        break;
    case VOODOO3_MODEL_V3_1000:
        /* V3-1000 was PCI-only; no AGP variant exists */
        pci_set_word(cfg + PCI_SUBSYSTEM_ID, 0x0052);
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: SUBSYSTEM_ID = V3_1000 PCI (0x0052)\n");
        break;
    case VOODOO3_MODEL_V3_2000:
    {
        uint16_t subid = s->is_agp ? 0x0038u : 0x0030u;
        pci_set_word(cfg + PCI_SUBSYSTEM_ID, subid);
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: SUBSYSTEM_ID = V3_2000 (%s) = 0x%04x\n",
            s->is_agp ? "AGP" : "PCI", subid);
        break;
    }
    case VOODOO3_MODEL_V3_3500TV:
        /* V3-3500TV was AGP-only; always use AGP subsystem ID */
        pci_set_word(cfg + PCI_SUBSYSTEM_ID, 0x0060);
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: SUBSYSTEM_ID = V3_3500 AGP (0x0060)\n");
        break;
    case VOODOO3_MODEL_V3_3000:
    default:
    {
        uint16_t subid = s->is_agp ? 0x003Cu : 0x003Au;
        pci_set_word(cfg + PCI_SUBSYSTEM_ID, subid);
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: SUBSYSTEM_ID = V3_3000 (%s) = 0x%04x\n",
            s->is_agp ? "AGP" : "PCI", subid);
        break;
    }
    }

    cfg[PCI_LATENCY_TIMER] = 0x40;

    s->fb_size = VOODOO3_FB_SIZE;
    s->fb_mem  = g_malloc0(s->fb_size);
    if (!s->fb_mem) {
        error_setg(errp, "voodoo3: cannot allocate framebuffer"); return;
    }

    /* Allocate texture RAM (4 MB per TMU — shared SGRAM, split evenly) */
    s->tex_mem_size = V3_TEX_MEM_SIZE;
    s->tex_mask     = V3_TEX_MASK;
    for (int t = 0; t < 2; t++) {
        s->tex_mem[t] = g_malloc0(s->tex_mem_size);
        if (!s->tex_mem[t]) {
            error_setg(errp, "voodoo3: cannot allocate texture RAM TMU%d", t);
            return;
        }
        memset(s->tex_cache[t], 0, sizeof(s->tex_cache[t]));
        s->tex_lru[t] = 0;
    }

    memory_region_init_io(&s->mmio, OBJECT(s), &voodoo3_mmio_ops, s,
                          "voodoo3-mmio", VOODOO3_MMIO_SIZE);
    pci_register_bar(pci_dev, 0,
        PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_32,
        &s->mmio);

    memory_region_init_io(&s->lfb, OBJECT(s), &voodoo3_lfb_ops, s,
                          "voodoo3-lfb", VOODOO3_LFB_SIZE);
    pci_register_bar(pci_dev, 1,
        PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_32 |
        PCI_BASE_ADDRESS_MEM_PREFETCH, &s->lfb);

    memory_region_init_io(&s->io, OBJECT(s), &voodoo3_io_ops, s,
                          "voodoo3-io", VOODOO3_IO_SIZE);
    pci_register_bar(pci_dev, 2, PCI_BASE_ADDRESS_SPACE_IO, &s->io);

    s->con = graphic_console_init(DEVICE(pci_dev), 0, &voodoo3_gfx_ops, s);
    qemu_console_resize(s->con, 640, 480);

    s->vblank_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, voodoo3_vblank_cb, s);
    timer_mod(s->vblank_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / VBLANK_HZ);

    qemu_mutex_init(&s->render_lock);
    qemu_cond_init(&s->render_cond);
    qemu_mutex_init(&s->fifo_lock);
    qemu_cond_init(&s->fifo_cond);
    s->render_stop = false;

    uint32_t nthreads = s->render_threads_count;
    if (nthreads < 1) nthreads = 1;
    if (nthreads > MAX_RENDER_THREADS) nthreads = MAX_RENDER_THREADS;
    s->render_threads_count = nthreads;

    for (uint32_t i = 0; i < nthreads; i++) {
        uint64_t *arg = g_new(uint64_t, 2);
        arg[0] = i;
        arg[1] = (uintptr_t)s;
        char name[32];
        snprintf(name, sizeof(name), "voodoo3-render-%u", i);
        qemu_thread_create(&s->render_thread[i], name,
                           voodoo3_render_thread, arg, QEMU_THREAD_JOINABLE);
    }

    voodoo3_reset_state(s);

    /* -----------------------------------------------------------------------
     * Model-specific hardware defaults after reset.
     * pllCtrl0 encodes the pixel clock: freq = 14.318 MHz * (N+2) / ((M+2) * (1<<K))
     * where bits[1:0]=K, bits[7:2]=M, bits[15:8]=N.
     * The BIOS ROM overwrites this during POST; these are power-on defaults.
     *   Banshee   ~125 MHz  0x2907  (N=0x29, M=1, K=3)
     *   V3-1000   ~143 MHz  0x2d07  (N=0x2d, M=1, K=3)
     *   V3-2000   ~143 MHz  0x2d07
     *   V3-3000   ~166 MHz  0x3207  (N=0x32, M=1, K=3)
     *   V3-3500   ~183 MHz  0x3507  (N=0x35, M=1, K=3)
     * ----------------------------------------------------------------------- */
    switch (s->model) {
    case VOODOO3_MODEL_BANSHEE:
        s->pllCtrl0 = 0x2907;   /* ~125 MHz */
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: model BANSHEE -> pllCtrl0=0x%04x (~125 MHz)\n",
            s->pllCtrl0);
        break;
    case VOODOO3_MODEL_V3_1000:
        s->pllCtrl0 = 0x2d07;   /* ~143 MHz */
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: model V3_1000 -> pllCtrl0=0x%04x (~143 MHz)\n",
            s->pllCtrl0);
        break;
    case VOODOO3_MODEL_V3_2000:
        s->pllCtrl0 = 0x2d07;   /* ~143 MHz */
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: model V3_2000 -> pllCtrl0=0x%04x (~143 MHz)\n",
            s->pllCtrl0);
        break;
    case VOODOO3_MODEL_V3_3500TV:
        s->pllCtrl0 = 0x3507;   /* ~183 MHz */
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: model V3_3500 -> pllCtrl0=0x%04x (~183 MHz)\n",
            s->pllCtrl0);
        break;
    case VOODOO3_MODEL_V3_3000:
    default:
        s->pllCtrl0 = 0x3207;   /* ~166 MHz */
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: model V3_3000 -> pllCtrl0=0x%04x (~166 MHz)\n",
            s->pllCtrl0);
        break;
    }
}

/* =========================================================================
 * PCI exit
 * ========================================================================= */
static void voodoo3_pci_exit(PCIDevice *pci_dev)
{
    Voodoo3State *s = VOODOO3_PCI(pci_dev);

    qemu_mutex_lock(&s->render_lock);
    s->render_stop = true;
    qemu_cond_broadcast(&s->render_cond);
    qemu_mutex_unlock(&s->render_lock);

    for (uint32_t i = 0; i < s->render_threads_count; i++)
        qemu_thread_join(&s->render_thread[i]);

    qemu_mutex_destroy(&s->render_lock);
    qemu_cond_destroy(&s->render_cond);
    qemu_mutex_destroy(&s->fifo_lock);
    qemu_cond_destroy(&s->fifo_cond);

    timer_free(s->vblank_timer);
    g_free(s->fb_mem);
    s->fb_mem = NULL;
    for (int t = 0; t < 2; t++) {
        g_free(s->tex_mem[t]);
        s->tex_mem[t] = NULL;
    }
}

static void voodoo3_reset(DeviceState *dev)
{
    voodoo3_reset_state(VOODOO3_PCI(dev));
}

/* =========================================================================
 * VMState
 * ========================================================================= */
static const VMStateDescription vmstate_voodoo3 = {
    .name           = "voodoo3",
    .version_id     = 3,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, Voodoo3State),
        VMSTATE_UINT32_ARRAY(regs,     Voodoo3State, 512),
        VMSTATE_UINT32_ARRAY(pallook,  Voodoo3State, VOODOO3_CLUT_SIZE),
        VMSTATE_INT32(screen_width,    Voodoo3State),
        VMSTATE_INT32(screen_height,   Voodoo3State),
        VMSTATE_UINT32(desktop_stride, Voodoo3State),
        VMSTATE_UINT32(desktop_start,  Voodoo3State),
        VMSTATE_INT32(pix_format,      Voodoo3State),
        VMSTATE_BOOL(display_enabled,  Voodoo3State),
        VMSTATE_UINT32(vidProcCfg,     Voodoo3State),
        VMSTATE_UINT32(vidScreenSize,  Voodoo3State),
        VMSTATE_UINT32(miscInit0,      Voodoo3State),
        VMSTATE_UINT32(miscInit1,      Voodoo3State),
        VMSTATE_UINT32(pllCtrl0,       Voodoo3State),
        VMSTATE_UINT32(pllCtrl1,       Voodoo3State),
        VMSTATE_UINT32(pllCtrl2,       Voodoo3State),
        VMSTATE_UINT32(dacMode,        Voodoo3State),
        VMSTATE_INT32(dacAddr,         Voodoo3State),
        VMSTATE_UINT32(intrCtrl,       Voodoo3State),
        VMSTATE_UINT32(lfbMemoryConfig,Voodoo3State),
        VMSTATE_UINT32(tile_base,      Voodoo3State),
        VMSTATE_UINT32(tile_stride,    Voodoo3State),
        VMSTATE_UINT32(tile_x,         Voodoo3State),
        VMSTATE_UINT32(sSetupMode,     Voodoo3State),
        /* 2D blitter state (version 3+) */
        VMSTATE_UINT32(blt.command,        Voodoo3State),
        VMSTATE_UINT32(blt.commandExtra,   Voodoo3State),
        VMSTATE_UINT32(blt.dstBaseAddr,    Voodoo3State),
        VMSTATE_UINT32(blt.dstFormat,      Voodoo3State),
        VMSTATE_UINT32(blt.srcBaseAddr,    Voodoo3State),
        VMSTATE_UINT32(blt.srcFormat,      Voodoo3State),
        VMSTATE_UINT32(blt.dstSize,        Voodoo3State),
        VMSTATE_UINT32(blt.dstXY,          Voodoo3State),
        VMSTATE_UINT32(blt.srcSize,        Voodoo3State),
        VMSTATE_UINT32(blt.srcXY,          Voodoo3State),
        VMSTATE_UINT32(blt.colorFore,      Voodoo3State),
        VMSTATE_UINT32(blt.colorBack,      Voodoo3State),
        VMSTATE_UINT32(blt.rop,            Voodoo3State),
        VMSTATE_UINT32(blt.clip0Min,       Voodoo3State),
        VMSTATE_UINT32(blt.clip0Max,       Voodoo3State),
        VMSTATE_VARRAY_UINT32(fb_mem, Voodoo3State, fb_size, 1, vmstate_info_uint8, uint8_t),
        VMSTATE_END_OF_LIST()
    },
};

/* =========================================================================
 * QOM properties / class_init / type registration
 * ========================================================================= */
static const Property voodoo3_properties[] = {
    DEFINE_PROP_UINT32("model",    Voodoo3State, model, VOODOO3_MODEL_V3_3000),
    DEFINE_PROP_BOOL("agp",        Voodoo3State, is_agp,              false),
    DEFINE_PROP_BOOL("bilinear",   Voodoo3State, bilinear,            true),
    DEFINE_PROP_BOOL("dac-filter", Voodoo3State, dac_filter,          false),
    DEFINE_PROP_UINT32("render-threads", Voodoo3State, render_threads_count, 2),
};

static void voodoo3_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass    *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k  = PCI_DEVICE_CLASS(klass);

    k->realize   = voodoo3_pci_realize;
    k->exit      = voodoo3_pci_exit;
    k->vendor_id = PCI_VENDOR_ID_3DFX;
    k->device_id = PCI_DEVICE_ID_3DFX_VOODOO3;
    k->revision  = 0x01;
    k->class_id  = PCI_CLASS_DISPLAY_VGA;
    /*
     * ROM image: QEMU will map this automatically as expansion ROM (BAR 6).
     * Provide "vgabios-voodoo3.bin" in the QEMU data/firmware search path.
     * Override at runtime with -device voodoo3,romfile=<path>.
     * Setting romfile="" disables ROM mapping entirely.
     *
     * This uses the standard QEMU PCIDeviceClass::romfile API — no manual
     * MemoryRegion setup needed.  86Box didn't need this but QEMU guests
     * (x86 BIOS, UEFI) probe the expansion ROM for VGA BIOS entry points.
     */
    k->romfile   = "vgabios-voodoo3.bin";

    device_class_set_legacy_reset(dc, voodoo3_reset);
    dc->vmsd     = &vmstate_voodoo3;
    dc->desc     = "3Dfx Voodoo 3 / Banshee PCI/AGP graphics (ported from 86Box)";
    device_class_set_props(dc, voodoo3_properties);
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo voodoo3_info = {
    .name          = TYPE_VOODOO3_PCI,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(Voodoo3State),
    .class_init    = voodoo3_class_init,
    .interfaces    = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void voodoo3_register_types(void)
{
    type_register_static(&voodoo3_info);
}

type_init(voodoo3_register_types)
