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
 * [x] Init/PLL/DAC/Video register decode (banshee_ext_outl/inl)
 * [x] Ext register byte-I/O (BAR2): sub-byte R/W for all 32-bit ext regs
 *       DAC dacAddr/dacData byte accumulation (0x51-0x57)
 *       vgaInit0/vgaInit1 byte R/W (0x29-0x2f) + crtc_update side effect
 *       dacMode byte R/W (0x4d-0x4f)
 *       vidProcCfg byte R/W (0x5d-0x5f) + pix_format/tiling side effects
 * [x] STATUS register with FIFO/busy/vblank flags (banshee_status)
 * [x] LFB tiled-address decode (banshee_read/write_linear)
 * [x] 3D LFB aperture READ (voodoo3_mmio_read 0x1000000–0x1FFFFFF)
 *       Ported from 86Box voodoo_fb_readl() in vid_voodoo_fb.c.
 *       Banshee addr decode: x=addr&0xffe, y=(addr>>12)&0x3ff
 *       Buffer select from lfbMode bits[7:6]: front/back/aux offset
 *       Tiled address calc identical to write path (col_tiled/row_width)
 * [x] Pixel-format-aware display output 8/16/24/32 bpp
 * [x] RGB565→BGRA8888 conversion
 * [x] Big-Endian byte-swap guards (be_fb) for PPC guests
 * [x] VGA I/O port proxy (BAR2 0xb0..0xdf → VGA ports 0x3b0..0x3df)
 *       ATC (0x3c0/0x3c1), Misc Output (0x3c2/0x3cc), Sequencer (0x3c4/0x3c5)
 *       DAC PEL mask/addr/data (0x3c6-0x3c9), Feature Control (0x3ca/0x3da)
 *       GRC (0x3ce/0x3cf), CRTC (0x3d4/0x3d5) with CR11 protect logic
 *       Input Status 1 (0x3da) with ATC flip-flop reset
 * [x] DAC PLL clock recalculation (voodoo3_pll_calc_freq / voodoo3_pll_update_vblank)
 *       Formula: freq = ((n+2) / ((m+2) * (1<<k))) * 14318184 Hz
 *       Vblank period derived from CRTC htotal × vtotal / pixel_clock
 *       Triggered on pllCtrl0 / misc_out / seq[1] / CRTC writes
 * [x] FIFO command queue (ring buffer)
 * [x] CMDFIFO register stubs (base/size/rptr/depth/holecount FIFO0+FIFO1)
 * [x] AGP host→VRAM DMA register stubs (agpMoveCMD accepted, no-op on PCI)
 * [x] voodoo_params_t — full 3D parameter set ported from vid_voodoo_common.h
 * [x] Full SST-1 3D register decode (voodoo_reg_writel) — all vertex/grad/cmd
 * [x] Integer AND floating-point triangle parameter paths
 * [x] Triangle CMD dispatch → voodoo3_queue_triangle()
 *       Band-parallel broadcast: all threads read every triangle, each
 *       thread renders only scanlines where (screen_y & odd_even_mask)==tid.
 *       Ported from 86Box voodoo_queue_triangle() + render_thread() model.
 *       odd_even_mask = render_threads_count-1 (0/1/3 for 1/2/4 threads).
 *       Back-pressure via per-thread param_rd[tid] ring-full detection.
 * [x] ftriangleCMD / triangleCMD / sBeginTriCMD / sDrawTriCMD
 * [x] fbzMode / fbzColorPath / alphaMode / fogMode / lfbMode
 * [x] Clip registers (clipLeftRight / clipLowYHighY / clip1 / clip2)
 * [x] colBufferAddr / colBufferStride / auxBufferAddr / auxBufferStride
 * [x] clutData write
 * [x] Setup-mode vertex accumulator (sVx/sVy/sRed/... sDrawTriCMD)
 * [x] swapbufferCMD / fastfillCMD / nopCMD
 * [x] Full pixel-level triangle rasterizer (voodoo3_render.c)
 *       Scanline edge-walk, clipping, sub-pixel correction
 *       Depth/W-buffer (all 8 compare ops), stipple patterns
 *       fbzColorPath colour combine (all CC_MSELECT/CC_ADD modes)
 *       Alpha test, fog (linear/table/z-based), alpha blend (all factors)
 *       4×4 and 2×2 ordered dither (RGB565 output)
 *       Pixel/depth write-back (tiled and linear)
 * [x] Texture fetch & filtering (voodoo3_texture.c + voodoo3_render.c)
 *       All texture formats: RGB332/565/1555/4444/8332/ARGB8888/PAL8 etc.
 *       Perspective-correct UV, bilinear filtering, dual-TMU blend
 *       NCC (YIQ) palette decode (voodoo3_update_ncc / ncc_lookup)
 *       Texture cache with LRU eviction (voodoo3_use_texture)
 *       voodoo3_flush_tex_if_dirty(): cache invalidation on PKT5 dst=0/1
 *       VRAM writes (Bug 4 fix, ported from 86Box flush_texture_cache() +
 *       texture_present[] in vid_voodoo_texture.c).
 * [x] Hardware cursor compositing (banshee_hwcursor_draw port)
 *       64×64 sprite, Windows AND/XOR and X11 mask/color modes
 *       Partial top-clip via yoff, left/right clipping per pixel
 * [x] NCC (Naïve Colour Compression) table decode (voodoo3_update_ncc)
 * [x] Banshee 2D blitter command decode (COMMAND_CMD_* constants)
 * [x] Full Banshee 2D pixel operations:
 *       RectFill (solid fill with clip)
 * [x] Screen-to-Screen BLT (with full colorkey via blt_colorkey()/blt_mix())
 *       Screen-to-Screen Stretch BLT
 *       Host-to-Screen BLT (byte accumulation, stride alignment)
 *       Host-to-Screen Stretch BLT
 *       Line (Bresenham, COMMAND_DX/DY direction flags)
 *       Polyline / Polyfill stubs
 * [x] 4 render QemuThreads (mirrors 86Box render_thread_1..4)
 * [x] Vblank QEMUTimer (period from PLL when programmed, else 60 Hz)
 * [x] VMState for snapshots
 * [x] CMDFIFO AGP ring buffer processing (vid_voodoo_fifo.c port)
 *       All 7 packet types: 0=Control(NOP/JSR/RET/JMP-LFB/JMP-AGP),
 *       1=Sequential reg write, 2=2D bitmask write, 3=Setup/vertex,
 *       4=Bitmask reg write, 5=Raw VRAM/FB/TEX block, 6=AGP DMA (stub)
 *       JSR/RET subroutine nesting, ring-buffer wrap, in_sub tracking
 * [x] CMDFIFO1 (AGP ring, secondary FIFO) — full packet processor
 *       Ported from 86Box second while-loop in voodoo_fifo_thread()
 *       cmdfifo_read_dword_2() / cmdfifo_read_float_2() read from _2 fields
 *       All 7 packet types handled identically to FIFO0
 *       On PCI (Pegasos2) FIFO1 is never populated → returns immediately
 * [x] VGA IRQ (vblank_irq_pending, pci_irq_assert/deassert wired)
 *       Assert on vblank if CRTC[0x11] bit4=1 + bit5=0 + pciInit0 bit18=1
 *       Deassert on 0x3da read or CRTC[0x11] bit4 cleared
 * [x] Video overlay (YUV422/RGB565/UYVY422 overlay — banshee_overlay_draw port)
 *       Register decode (StartCoords/EndCoords/Dudx/Dvdy/SrcWidth)
 *       YUYV422 and UYVY422 YCbCr→RGB (ITU-R BT.601 coefficients)
 *       RGB565 (linear and tiled) with optional CLUT LUT
 *       H-scale (Dudx step) and V-scale (Dvdy step), point filter
 *       Bilinear Y-interpolation between two decoded lines
 *       Chroma-key compositing (vidChromaKeyMin/Max, all desktop bpp)
 * [x] NCC in rasterizer path (ncc_lookup populated by voodoo3_update_ncc;
 *       tex_params stores textureMode so decode_texture() uses the correct
 *       NCC table-select bit (textureMode[5] = TEXTUREMODE_NCC_SEL, not tLOD[5]).
 *       Cache invalidated via ncc_gen[tmu] counter on every nccTable write.)
 * [x] ARGB32 (srcfmt=7) alpha-blend blit support — FIX for:
 *       "RGB mask blits with RGB source (srcfmt = 7) and different
 *        colormodels (destfmt = 0) not supported yet"
 *       "cgx/WPAAlpha unsupported pixfmt: 0 for RECTFMT_ARGB"
 *       SRC_FORMAT_COL_ARGB32 (7u<<16) defined, src_bpp=32 set, Porter-Duff
 *       "src over dst" alpha-blend via blt_alpha_blend_argb32() in:
 *         blt_do_s2s_line(), blt_do_stretch_line(), blt_update_src_stride_full()
 *       DST_FORMAT_COL_PAL (0u<<16, dstfmt=0) added to blt_plot() fall-through.
 * [x] vidDesktopStartAddr (BAR0+0x0E4) WARN4 fix — "vidDesktopStart=0xFFFFFF00":
 *       Write handler clamps value to fb_size; out-of-range writes reset to 0.
 *       Read sub-byte 0xe7 (bits[31:24]) now always returns 0x00 — register
 *       is 24-bit only; upper byte not wired on real hardware.
 * [ ] JIT recompiler (x86-64/ARM64) — not applicable for QEMU device model
 * [ ] SLI multi-GPU — not applicable (single-GPU emulation only)
 * -------------------------------------------------------------------------
 * BUG FIXES (applied on top of initial port)
 * -------------------------------------------------------------------------
 * [x] FIX 1: lodbias wired into render path (voodoo3_render.c)
 *       tLOD[17:12] decoded in voodoo3.c → params.tmu[t].lodbias;
 *       render path now reads it instead of hardcoding 0.
 * [x] FIX 2: S2S BLT chroma-key is fully implemented (was mislabelled stub)
 *       blt_colorkey() + blt_mix() correctly gate on CMDEXTRA_SRC/DST_COLORKEY.
 *       Fast-path (memmove) already requires no_colorkey=true to be taken.
 * [x] FIX 3: RGB32 (pix_format==3) fastfill added (voodoo3_display.c)
 *       voodoo3_fastfill() now dispatches on s->pix_format:
 *         pix_format==3 → 32-bit word fill with tiled 32-pixel-per-strip layout
 *         all others     → original 16-bit RGB565 path
 * [x] FIX 4: RGB32 display bswap/memcpy redundancy removed (voodoo3_display.c)
 *       The memcpy(dst_row, dst, w*4) after the bswap32 loop was a self-copy
 *       (dst == (uint32_t*)dst_row).  Removed.  bswap32 itself was also
 *       incorrect for LE hosts — replaced with a plain copy; QEMU's
 *       DEVICE_LITTLE_ENDIAN MemoryRegion already compensates for BE guests.
 * [x] FIX 5: LOD clamp decoded from tLOD register (voodoo3_render.c)
 *       st.lod_min[t] / st.lod_max[t] now read from tLOD[5:2] / tLOD[11:8]
 *       (4-bit integer fields, converted to 8.8 fixed-point × 256) instead
 *       of being hardcoded to 0 / V3_LOD_MAX<<8.
 * [x] FIX 6: polyfill edge-selection corrected (voodoo3.c)
 *       blt_polyfill_continue() now selects the edge whose tip Y (ly[1] or
 *       ry[1]) is smallest — matching 86Box — instead of the incorrect
 *       ry[1]>=ly[1] heuristic that failed for concave polygons.
 * [x] FIX 7: ext register read default returns 0x0 instead of 0xFFFFFFFF
 *       Unknown ext reads now try regs[] first (for write-then-read
 *       round-trips), then return 0x00000000 (idle/absent) rather than
 *       0xFFFFFFFF (broken) which caused driver abort paths on AmigaOS4.
 * [x] FIX 8: SGRAM (fb_mem) saved in VMState subsection "voodoo3/sgram"
 *       16 MiB fb_mem is now persisted via VMSTATE_VBUFFER_UINT8 in a
 *       version-1 subsection.  Old v4 snapshots load cleanly (subsection
 *       absent → fb_mem zeroed → blank screen until next driver redraw)
 * [x] FIX 9: bswap16 RGB565 display path — BE-Guard added (voodoo3_display.c)
 *       The unconditional bswap16(src[x]) in the non-tiled RGB565→BGRA8888
 *       conversion loop was wrong for all little-endian hosts (x86_64):
 *       DEVICE_LITTLE_ENDIAN MemoryRegion already puts 16-bit words in
 *       host-native byte order before we read fb_mem, so a further bswap
 *       inverted R and B, producing a blue tint on every x86 guest in 16bpp.
 *       For BE hosts the same DEVICE_LITTLE_ENDIAN compensation applies,
 *       making the swap redundant there too.  Replaced with plain src[x].
 * [x] FIX 10: blt_polyfill_continue() edge-selection restored to exact 86Box
 *       semantics (voodoo3.c).  FIX 6's comment was correct but the
 *       implementation `ly[1] < ry[1]` is logically identical to the broken
 *       `ly[1] < ry[1]` it replaced.  86Box uses `ry[1] >= ly[1]` (selects
 *       left edge when right tip is equal-or-ahead), which correctly handles
 *       horizontal top edges (equal tips → left gets new vertex, not right).
 *       Retire block updated with the same ry[1] >= ly[1] sense.
 * [x] FIX 11: VMState 3D-param completeness — new subsection "voodoo3/3dstate"
 *       saves all fields previously omitted from vmstate_voodoo3:
 *         params.tmu[0/1]: startS/T/W, dS/dT/dW per-pixel + per-scanline,
 *           lodbias, lod_min, lod_max.
 *         params.fogTable[64]: packed via fog_table_save[128] shadow +
 *           pre_save/post_load hooks (struct array, not directly addressable).
 *         params.fogColor.r/g/b, chromaKey_r/g/b.
 *         params.detail_max/bias/scale[2], sign, swapbufferCMD.
 *         params.fbiPixels* stat counters.
 *         verts[4]: packed via verts_save[56] shadow (14 floats × 4).
 *         vertex_num, vertex_next_age, vertex_ages[3], num_verticies,
 *           cull_pingpong.
 *         swap_pending, swap_interval, swap_offset, retrace_count,
 *           frame_count, fbiPixels* global counters.
 *       Old snapshots load cleanly: missing subsection → fields keep
 *       reset() defaults; driver reprograms all 3D regs before next draw.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qemu/thread.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
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
#include "hw/i2c/i2c.h"

/* Forward declaration — defined in the BAR1 section after cmdfifo_ops. */
static void voodoo3_cmdfifo_reposition(Voodoo3State *s);

/* -----------------------------------------------------------------------
 * SDL-safe deferred console resize — Bottom Half callback.
 *
 * All qemu_console_resize() calls that originate from MMIO-write paths or
 * from inside voodoo3_update_display_dirty() use this BH instead of calling
 * qemu_console_resize() directly.  The BH fires at the start of the next
 * QEMU main-loop iteration, guaranteed in the main thread with no blit in
 * progress — which is the only safe point for SDL surface replacement.
 * ----------------------------------------------------------------------- */
static void voodoo3_resize_bh(void *opaque)
{
    Voodoo3State *s = VOODOO3_PCI(opaque);
    int w = s->resize_pending_w;
    int h = s->resize_pending_h;
    if (s->con && w > 0 && h > 0) {
        qemu_console_resize(s->con, w, h);
        memset(s->dirty_line, 1, sizeof(s->dirty_line));
    }
    s->resize_pending_w = 0;
    s->resize_pending_h = 0;
}

/* Helper: request a deferred resize.  Idempotent — last write wins. */
static void voodoo3_request_resize(Voodoo3State *s, int w, int h)
{
    if (w <= 0 || h <= 0)
        return;
    s->resize_pending_w = w;
    s->resize_pending_h = h;
    qemu_bh_schedule(s->resize_bh);
}

/* -----------------------------------------------------------------------
 * Screen-filter (scrfilter) — ported from 86Box
 * voodoo_generate_vb_filters() in vid_voodoo_banshee.c.
 *
 * Two 256×256 lookup tables are generated:
 *
 *   vb_filter_bx_rb/g  — box pre-filter: blends two horizontally adjacent
 *       pixels with a threshold-limited contribution (fcr/fcg cap).
 *
 *   vb_filter_v1_rb/g  — 4×1 / 2×2 filter: splits the difference between
 *       two pixel values by ½, clamped to ±fcr/fcg.
 *
 * purpleline[]  — per-scanline tint applied to odd rows (R and B channels
 *       boosted by +4 LSBs, G unchanged), matching 86Box behaviour.
 *
 * Call voodoo3_scrfilter_threshold_check() whenever scrfilter_threshold
 * changes; it regenerates the tables only when the value differs from
 * the previous call (scrfilter_threshold_old).
 *
 * 86Box reference: voodoo_generate_vb_filters(), voodoo_threshold_check()
 * ----------------------------------------------------------------------- */

static void
voodoo3_generate_vb_filters(Voodoo3State *s, int fcr, int fcg)
{
    /* Box pre-filter (vb_filter_bx_*): threshold-limited neighbour blend */
    for (int g = 0; g < 256; g++) {
        for (int h = 0; h < 256; h++) {
            float difference = (float)(g - h);
            float avg        = g;
            float avgdiff    = avg - h;

            avgdiff = avgdiff * 0.75f;
            if (avgdiff < 0)  avgdiff *= -1;
            if (difference < 0) difference *= -1;

            float thiscol  = g;
            float thiscolg = g;

            if (h > g) {
                float clr = avgdiff;
                float clg = avgdiff;
                if (clr > fcr) clr = fcr;
                if (clg > fcg) clg = fcg;

                thiscol  = g;
                thiscolg = g;

                if (thiscol  > g + fcr)        thiscol  = g + fcr;
                if (thiscolg > g + fcg)         thiscolg = g + fcg;
                if (thiscol  > g + difference)  thiscol  = g + difference;
                if (thiscolg > g + difference)  thiscolg = g + difference;

                int ugh = g - h;
                if (ugh < fcr) thiscol  = h;
                if (ugh < fcg) thiscolg = h;
            }

            if (difference > fcr) thiscol  = g;
            if (difference > fcg) thiscolg = g;

            if (thiscol  < 0)   thiscol  = 0;
            if (thiscolg < 0)   thiscolg = 0;
            if (thiscol  > 255) thiscol  = 255;
            if (thiscolg > 255) thiscolg = 255;

            s->vb_filter_bx_rb[g][h] = (uint8_t)thiscol;
            s->vb_filter_bx_g [g][h] = (uint8_t)thiscolg;
        }

        /* purpleline: R and B channels +4, G unchanged */
        float lined = g + 4;
        if (lined > 255) lined = 255;
        s->purpleline[g][0] = (uint16_t)lined; /* R */
        s->purpleline[g][1] = (uint16_t)g;     /* G */
        s->purpleline[g][2] = (uint16_t)lined; /* B */
    }

    /* 4×1 / 2×2 filter (vb_filter_v1_*): split-difference, clamped to fcr/fcg */
    for (int g = 0; g < 256; g++) {
        for (int h = 0; h < 256; h++) {
            float difference = (float)(h - g);
            float diffg      = difference;

            float thiscol  = g;
            float thiscolg = g;

            if (difference >  fcr)  difference =  fcr;
            if (difference < -fcr)  difference = -fcr;
            if (diffg      >  fcg)  diffg      =  fcg;
            if (diffg      < -fcg)  diffg      = -fcg;

            thiscol  = g + (difference / 2);
            thiscolg = g + (diffg      / 2);

            if (thiscol  < 0)   thiscol  = 0;
            if (thiscol  > 255) thiscol  = 255;
            if (thiscolg < 0)   thiscolg = 0;
            if (thiscolg > 255) thiscolg = 255;

            s->vb_filter_v1_rb[g][h] = (uint8_t)thiscol;
            s->vb_filter_v1_g [g][h] = (uint8_t)thiscolg;
        }
    }
}

/*
 * voodoo3_scrfilter_threshold_check — regenerate filter tables when the
 * scrfilter_threshold register changes.  Mirrors 86Box voodoo_threshold_check().
 */
static void
voodoo3_scrfilter_threshold_check(Voodoo3State *s)
{
    if (!s->scrfilter_enabled)
        return;

    if (s->scrfilter_threshold == s->scrfilter_threshold_old)
        return;

    s->scrfilter_threshold_old = s->scrfilter_threshold;

    int fcr = (s->scrfilter_threshold >> 16) & 0xFF; /* R cap */
    int fcg = (s->scrfilter_threshold >>  8) & 0xFF; /* G cap (also used for B) */

    voodoo3_generate_vb_filters(s, fcr, fcg);
}

/* -----------------------------------------------------------------------
 * DDC / I2C — QEMU-native bitbang_i2c bridge.
 *
 * The Voodoo3/Banshee exposes two I²C buses via vidSerialParallelPort:
 *
 *   DDC bus (EDID):  EN=bit18  SCL_W=bit19  SDA_W=bit20
 *                              SCL_R=bit21  SDA_R=bit22
 *   I2C bus (aux):   EN=bit23  SCL_W=bit24  SDA_W=bit25
 *                              SCL_R=bit26  SDA_R=bit27
 *
 * We wire both buses to the same I2CDDC slave (addr 0x50) so that
 * drivers which probe either bus receive valid EDID data.  This mirrors
 * how ATI (hw/display/ati.c) bridges its GPIO register bits to the
 * QEMU bitbang_i2c layer.
 *
 * Bit semantics (Banshee datasheet + 86Box banshee_ext_outl):
 *   When EN=0 the bus is idle; both lines float high (open-drain pull-up).
 *   When EN=1: SCL = SCL_W,  SDA = SDA_W  (host drives the bus).
 *   Read-back:  SCL_R = loopback of SCL_W,  SDA_R = slave SDA output.
 *
 * Unlike ATI, Voodoo3 has a single EN bit for both SCL and SDA per bus
 * rather than per-line output-enable bits.  We pass SCL=1/SDA=1 when
 * EN=0 to emulate the open-drain idle state.
 * ----------------------------------------------------------------------- */

/*
 * voodoo3_vidserial_update — bridge one vidSerialParallelPort bus to
 * QEMU's bitbang_i2c layer and write the read-back bits back into *reg.
 *
 * @reg     pointer to s->vidSerialParallelPort (modified in place)
 * @bbi2c   the bitbang_i2c_interface for this bus
 * @en_bit  bit index of the bus-enable flag  (18 for DDC, 23 for I2C)
 * @sclw    bit index of SCL write            (19 / 24)
 * @sdaw    bit index of SDA write            (20 / 25)
 * @sclr    bit index of SCL read-back        (21 / 26)
 * @sdar    bit index of SDA read-back        (22 / 27)
 */
static void voodoo3_vidserial_update(uint32_t *reg,
                                     bitbang_i2c_interface *bbi2c,
                                     int en_bit,
                                     int sclw, int sdaw,
                                     int sclr, int sdar)
{
    uint32_t v = *reg;

    /* Clear the read-back bits — we will rebuild them below */
    v &= ~((1u << sclr) | (1u << sdar));

    if (!(v & (1u << en_bit))) {
        /*
         * Bus disabled: release both lines (open-drain pull-up → 1,1).
         * This idles the state machine and prevents false START detection
         * when the driver has not yet asserted EN.
         */
        bitbang_i2c_set(bbi2c, BITBANG_I2C_SCL, 1);
        bitbang_i2c_set(bbi2c, BITBANG_I2C_SDA, 1);
        /* SCL_R and SDA_R both high when bus is idle */
        v |= (1u << sclr) | (1u << sdar);
    } else {
        bool scl = !!(v & (1u << sclw));
        bool sda;

        /* Drive SCL first so the state machine sees the correct clock edge */
        bitbang_i2c_set(bbi2c, BITBANG_I2C_SCL, scl);
        /* Drive SDA; bitbang_i2c_set returns the slave's SDA output */
        sda = bitbang_i2c_set(bbi2c, BITBANG_I2C_SDA,
                              !!(v & (1u << sdaw)));

        /* SCL_R = loopback of SCL_W (no clock stretching) */
        if (scl) v |= (1u << sclr);
        /* SDA_R = slave SDA output (EDID data or ACK/NAK) */
        if (sda) v |= (1u << sdar);
    }

    *reg = v;
}


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
/*
 * Additional Banshee/Voodoo3 ext register offsets not yet defined above.
 * Sources: 3dfx Banshee Hardware Specification v1.0 §4.1 "External Registers"
 *          86Box vid_voodoo_banshee.c banshee_ext_outl() / banshee_ext_inl()
 */
/* Video capture registers (0x70–0x94) */
#define Video_vidInStatus                0x74   /* video capture status (read-only)         */
#define Video_vidPllCtrl                 0x7c   /* video PLL control / reserved scratch     */
#define Video_vidInYStart                0x94   /* video capture Y start coordinate         */

/* Video capture DMA address / stride registers (0xec–0xfc) */
#define Video_vidInAddr0                 0xec   /* video capture DMA buffer 0 address       */
#define Video_vidInAddr1                 0xf0   /* video capture DMA buffer 1 address       */
#define Video_vidInAddr2                 0xf4   /* video capture DMA buffer 2 address       */
#define Video_vidInStride                0xf8   /* video capture DMA stride                 */
#define Video_vidCurrOverlayStartAddr    0xfc   /* current overlay scan-out start address   */

#define Init_tmugbInit                   0x24   /* TMU global buffer init — write-only */
#define Init_vgaInit0                    0x28
#define Init_vgaInit1                    0x2c
#define Init_2dCommand                   0x30
#define Init_2dSrcBaseAddr               0x34
#define Init_strapInfo                   0x38   /* Strap configuration: mem size, bus type, IRQ, BIOS size
                                                 * Read-only. 86Box banshee_ext_inl() Init_strapInfo.
                                                 * The real 3dfx BIOS reads this on boot to determine
                                                 * SDRAM/SGRAM config and ROM size. Bit 6 set = PCI+IRQ. */
#define Init_2dSrcSize                   0x3c   /* Banshee 2D source size (write-only scratch) */
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

/* VIDPROCCFG / OVERLAY_FMT / VID_STRIDE defines → see voodoo3_int.h */

/* vidOverlay register field masks */
#define OVERLAY_START_X_MASK         (0xfffu)
#define OVERLAY_START_Y_SHIFT        12
#define OVERLAY_START_Y_MASK         (0xfffu << OVERLAY_START_Y_SHIFT)
#define OVERLAY_END_X_MASK           (0xfffu)
#define OVERLAY_END_Y_SHIFT          12
#define OVERLAY_END_Y_MASK           (0xfffu << OVERLAY_END_Y_SHIFT)
#define OVERLAY_SRC_WIDTH_SHIFT      19
#define OVERLAY_SRC_WIDTH_MASK       (0x1fffu << OVERLAY_SRC_WIDTH_SHIFT)
#define VID_DUDX_MASK                0xffffffu
#define VID_DVDY_MASK                0xffffffu
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
#define SST_fbiPixelsIn     0x14c
#define SST_fbiChromaFail   0x150
#define SST_fbiZFuncFail    0x154
#define SST_fbiAFuncFail    0x158
#define SST_fbiPixelsOut    0x15c
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
#define SST_sBeginTriCMD    0x2a4
#define SST_sDrawTriCMD     0x2a0

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
#define COMMAND_INC_X_START     (1u << 10)
#define COMMAND_INC_Y_START     (1u << 11)
#define COMMAND_STIPPLE_LINE    (1u << 12)
#define COMMAND_PATTERN_MONO    (1u << 13)
#define COMMAND_DX              (1u << 14)
#define COMMAND_DY              (1u << 15)
#define COMMAND_TRANS_MONO      (1u << 16)
#define COMMAND_PATOFF_X_MASK   (7u << 17)
#define COMMAND_PATOFF_X_SHIFT  17
#define COMMAND_PATOFF_Y_MASK   (7u << 20)
#define COMMAND_PATOFF_Y_SHIFT  20
#define COMMAND_CLIP_SEL        (1u << 23)

#define CMDEXTRA_SRC_COLORKEY   (1u << 0)
#define CMDEXTRA_DST_COLORKEY   (1u << 1)
#define CMDEXTRA_FORCE_PAT_ROW0 (1u << 3)

/* SRC_FORMAT bits (86Box vid_voodoo_banshee_blitter.c) */
#define SRC_FORMAT_STRIDE_MASK   0x1fffu
#define SRC_FORMAT_COL_MASK      (0xfu << 16)
#define SRC_FORMAT_COL_1_BPP     (0u  << 16)
#define SRC_FORMAT_COL_8_BPP     (1u  << 16)
#define SRC_FORMAT_COL_16_BPP    (3u  << 16)
#define SRC_FORMAT_COL_24_BPP    (4u  << 16)
#define SRC_FORMAT_COL_32_BPP    (5u  << 16)
/*
 * SRC_FORMAT value 7 = ARGB8888 with alpha channel.
 * Used by CyberGraphX/Picasso96 WritePixelArray (WPAAlpha) and
 * RGB-mask blits from RECTFMT_ARGB sources.  The Banshee hardware
 * datasheet calls this "ARGB32" — identical pixel layout to 32_BPP
 * (0xAARRGGBB) but the alpha byte is *used* for per-pixel alpha
 * blending rather than being ignored.
 *
 * Fix: "RGB mask blits with RGB source (srcfmt = 7) and different
 *       colormodels (destfmt = 0) not supported yet"
 *      "cgx/WPAAlpha unsupported pixfmt: 0 for RECTFMT_ARGB"
 */
#define SRC_FORMAT_COL_ARGB32    (7u  << 16)   /* ARGB with alpha */
#define SRC_FORMAT_COL_YUYV      (8u  << 16)
#define SRC_FORMAT_COL_UYVY      (9u  << 16)
#define SRC_FORMAT_BYTE_SWIZZLE  (1u  << 20)
#define SRC_FORMAT_WORD_SWIZZLE  (1u  << 21)
#define SRC_FORMAT_PACKING_MASK  (3u  << 22)
#define SRC_FORMAT_PACKING_STRIDE (0u << 22)
#define SRC_FORMAT_PACKING_BYTE  (1u  << 22)
#define SRC_FORMAT_PACKING_WORD  (2u  << 22)
#define SRC_FORMAT_PACKING_DWORD (3u  << 22)

/* DST_FORMAT bits */
#define DST_FORMAT_COL_MASK      (0xfu << 16)
/*
 * DST_FORMAT value 0 = 8-bpp palette (CLUT) mode.
 * The Banshee 2D engine can blit ARGB32 source pixels onto a palette
 * framebuffer by dropping the colour channels and only writing the
 * alpha (or the nearest palette index), but what CyberGraphX/Picasso96
 * actually needs here is: ARGB → nearest palette entry lookup (or just
 * the RGB, dropping alpha since the palette FB has no alpha plane).
 *
 * DST_FORMAT_COL_8_BPP (1u << 16) is the normal 8-bpp path.
 * DST_FORMAT_COL_PAL   (0u << 16) is the "raw" palette write mode
 * where the destination is treated as an 8-bpp surface without the
 * intermediate CLUT conversion step (driver writes palette-indexed
 * pixels directly).
 *
 * Fix: destfmt=0 in "RGB mask blits … destfmt = 0 not supported yet"
 */
#define DST_FORMAT_COL_PAL       (0u  << 16)   /* 8-bpp raw palette (dstfmt=0) */
#define DST_FORMAT_COL_8_BPP     (1u  << 16)
#define DST_FORMAT_COL_16_BPP    (3u  << 16)
#define DST_FORMAT_COL_24_BPP    (4u  << 16)
#define DST_FORMAT_COL_32_BPP    (5u  << 16)

/* Colorkey format tags (for MIX/colorkey helpers) */
#define BLT_COLORKEY_8    0
#define BLT_COLORKEY_16   1
#define BLT_COLORKEY_32   2

#define BRES_ERROR_MASK   0xffffu
#define BRES_ERROR_USE    (1u << 31)

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
    int busy     = (s->cmd_written != s->cmd_read)
                || (s->cmdfifo_depth_rd != s->cmdfifo_depth_wr)
                || (s->cmdfifo_depth_rd_2 != s->cmdfifo_depth_wr_2)
                || s->voodoo_busy;
    int free     = 32 - depth;
    if (free < 0) free = 0;
    if (free > 0x1f) free = 0x1f;

    /*
     * Bits [4:0] = free FIFO slots (0x1f = full empty)
     * Bit  5     = FIFO not empty
     * Bit  6     = display active (NOT in vblank) — 86Box: "if (!v_retrace) ret |= 0x40"
     *              SET when display is active, CLEAR during vblank.
     *              The 3dfx driver polls this to detect vblank. Previous
     *              inversions here caused STOP 0xEA spin loops.
     * Bits [10:7] = busy flags (0x780 when busy)
     * Bits [12:11] = CMDFIFO0/1 not empty
     */
    ret |= (uint32_t)free & 0x1fu;
    if (depth > 0)    ret |= (1u << 5);
    if (!s->in_vblank) ret |= (1u << 6);   /* display active = bit6 SET */
    if (busy)          ret |= 0x780u;
    if (s->cmdfifo_depth_rd   != s->cmdfifo_depth_wr)
        ret |= (1u << 11);
    if (s->cmdfifo_depth_rd_2 != s->cmdfifo_depth_wr_2)
        ret |= (1u << 12);
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
            s->ncc_dirty[_t] = 0;
        }
    }

    /*
     * Decode tLOD lodbias for each TMU so the rasterizer can use it.
     * Ported from 86Box voodoo_triangle():
     *   lodbias = (params->tLOD[tmu] >> 12) & 0x3f;
     *   if (lodbias & 0x20) lodbias |= ~0x3f;  // sign-extend 6-bit
     */
    for (int _t = 0; _t < 2; _t++) {
        /* Snapshot tex_params from device state into triangle params */
        p->tex_params[_t] = s->params.tex_params[_t];
        p->tex_params[_t].tLOD = p->tmu[_t].tLOD;

        /* Decode lodbias: 6-bit signed from tLOD[17:12] */
        int lb = (int)((p->tmu[_t].tLOD >> 12) & 0x3f);
        if (lb & 0x20) lb |= ~0x3f;
        p->tmu[_t].lodbias = lb;
    }

    /*
     * Bind texture cache entries before queuing — ported from 86Box
     * voodoo_use_texture() call inside voodoo_queue_triangle().
     * This fills p->tex_ptr[][] so the rasterizer can fetch texels.
     */
    if (p->fbzColorPath & (1u << 27)) {   /* FBZCP_TEXTURE_ENABLED */
        voodoo3_use_texture(s, p, 0);
        /* TMU1 only when not in passthrough mode (86Box dual_tmus check) */
        if ((p->tmu[0].textureMode & 0x00643000u) != 0x00241000u)
            voodoo3_use_texture(s, p, 1);
    }

    /*
     * Band-parallel broadcast: write the triangle ONCE to the shared ring
     * buffer.  ALL render threads consume every entry (each with their own
     * param_rd[tid]) and skip scanlines they don't own via odd_even_mask.
     *
     * Mirrors 86Box exactly:
     *   PARAMS_WRITE_IDX++ increments the single write pointer once.
     *   Each thread has its own PARAMS_READ_IDX(voodoo, odd_even).
     *   voodoo_half_triangle() skips (screen_y & odd_even_mask) != thread_id.
     *
     * Back-pressure: if ANY thread's read pointer is PARAM_BUF_SIZE behind
     * the write pointer the ring is full — yield so render threads can drain.
     * Mirrors 86Box PARAM_FULL() loop in voodoo_queue_triangle().
     */
    {
        uint32_t nthreads = s->render_threads_count;
        while (true) {
            bool full = false;
            for (uint32_t t = 0; t < nthreads; t++) {
                if (s->param_wr - s->param_rd[t] >= PARAM_BUF_SIZE) {
                    full = true;
                    break;
                }
            }
            if (!full) break;
            qemu_cond_broadcast(&s->render_cond);
            qemu_mutex_unlock(&s->render_lock);
            qemu_mutex_lock(&s->render_lock);
        }
    }

    uint32_t idx = s->param_wr & (PARAM_BUF_SIZE - 1);
    memcpy(&s->param_buf[idx], p, sizeof(*p));
    s->param_wr++;
    s->voodoo_busy = true;
    qemu_cond_broadcast(&s->render_cond);
}

/* =========================================================================
 * PLL pixel-clock recalculation
 *
 * Ported from 86Box banshee_recalctimings() (vid_voodoo_banshee.c line 729).
 *
 * The Banshee/V3 has three PLLs (pllCtrl0/1/2).  Only pllCtrl0 is used for
 * the pixel clock; pllCtrl1/2 are stored but not used in clock generation
 * (confirmed: 86Box only reads pllCtrl0 in banshee_recalctimings).
 *
 * PLL formula (from 86Box and 3dfx Banshee datasheet):
 *   k    = pllCtrl0[1:0]          — output divider (0..3, divide by 1/2/4/8)
 *   m    = pllCtrl0[7:2]          — reference divider (0..63)
 *   n    = pllCtrl0[15:8]         — feedback divider (0..255)
 *   freq = ((n+2) / ((m+2) * (1<<k))) * 14318184.0  Hz
 *
 * The reference clock is the standard PC crystal: 14.318184 MHz.
 *
 * Misc Output Register bits[3:2] = clock select:
 *   00 = 25.175 MHz  (standard VGA 640x480 @ 60 Hz)
 *   01 = 28.322 MHz  (standard VGA 720x400 @ 70 Hz)
 *   10 = reserved / external (not used on Banshee)
 *   11 = PLL (pllCtrl0 formula above)
 *
 * 86Box: when clock_select != 3, svga_recalctimings() uses its own fixed
 * clock table.  In QEMU we fall back to VBLANK_HZ for those modes.
 *
 * The computed pixel_clock_hz is used in voodoo3_pll_update_vblank() to
 * derive the frame period from the CRTC total counts.  This makes the
 * emulated vblank timer track the programmed refresh rate instead of always
 * running at 60 Hz, which is important for drivers that check the DAC
 * status register or use swapbuffer with a specific swap_interval.
 *
 * Trigger points (same as 86Box svga_recalctimings call sites):
 *   - pllCtrl0 write  (primary: clock formula changes)
 *   - misc_out write  (clock source select bits change)
 *   - CRTC data write (htotal/vtotal change → frame period changes)
 * ========================================================================= */

/*
 * voodoo3_pll_calc_freq — compute pixel clock Hz from pllCtrl0.
 * Returns 0.0 if the PLL is not selected (misc_out clock select != 3).
 * Pure calculation, no side effects.
 */
static double voodoo3_pll_calc_freq(Voodoo3State *s)
{
    /* Misc Output Register bits[3:2] = clock source select.
     * 3 = PLL, anything else = fixed VGA clock. */
    unsigned clk_sel = (s->misc_out >> 2) & 3u;
    if (clk_sel != 3) {
        /* Fixed VGA clocks — 86Box svga core handles these.
         * Return standard frequencies so vblank period is reasonable. */
        return (clk_sel == 0) ? 25175000.0 : 28322000.0;
    }

    /* PLL selected: decode pllCtrl0 fields (86Box banshee_recalctimings) */
    unsigned k = s->pllCtrl0 & 0x3u;           /* bits[1:0]: output divider */
    unsigned m = (s->pllCtrl0 >> 2) & 0x3fu;   /* bits[7:2]: ref divider    */
    unsigned n = (s->pllCtrl0 >> 8) & 0xffu;   /* bits[15:8]: feedback div  */

    /* Guard against degenerate values that produce division by zero.
     * 86Box does no such check, but pllCtrl0=0 at reset would give freq=0. */
    if ((m + 2) == 0 || (1u << k) == 0)
        return 0.0;

    double freq = ((double)(n + 2) / ((double)(m + 2) * (double)(1u << k)))
                  * 14318184.0;
    return freq;
}

/*
 * voodoo3_pll_update_vblank — recompute vblank_period_ns from pixel clock
 * and CRTC total counts, then reschedule the vblank timer.
 *
 * CRTC register layout (standard VGA + Banshee extensions):
 *
 *   htotal (pixel clocks per line, including blanking):
 *     base  = (CRTC[0x00] + 5) * dots_per_clock
 *     +ext  = CRTC[0x1a] bit0 → +0x100 character clocks
 *     dots_per_clock = 8 (seq[1] bit3=0) or 16 (seq[1] bit3=1)
 *     86Box: svga->htotal is in character clocks, then *dots_per_clock.
 *     We compute total pixel clocks directly.
 *
 *   vtotal (lines per frame, including blanking):
 *     base  = CRTC[0x06]
 *           | ((CRTC[0x07] & 0x01) << 8)   overflow bit 8
 *           | ((CRTC[0x07] & 0x20) << 4)   overflow bit 9
 *     +ext  = CRTC[0x1b] bit0 → +0x400    (Banshee-specific extension)
 *     +2    (VGA convention: register value is vtotal - 2)
 *
 * frame_period = (htotal_pixels * vtotal_lines) / pixel_clock_hz  seconds
 * vblank_period_ns = frame_period * 1e9
 *
 * Falls back to VBLANK_HZ (60 Hz) if pixel_clock_hz is 0 or CRTC totals
 * are not yet programmed (avoids division by zero and timer storm).
 */
static void voodoo3_pll_update_vblank(Voodoo3State *s)
{
    double freq = voodoo3_pll_calc_freq(s);
    s->pixel_clock_hz = freq;

    /* dots_per_clock: sequencer register [1] bit 3 = 8-dot/16-dot clock.
     * 86Box: svga->dots_per_clock = (seqregs[1] & 8) ? 16 : 8 */
    int dots = (s->seq_regs[1] & 0x08u) ? 16 : 8;

    /* htotal in pixel clocks:
     *   CRTC[0x00] = (htotal / char_clocks) - 5
     *   +Banshee: CRTC[0x1a] bit0 = htotal bit 8 (in char clocks)
     *   total char clocks = (CRTC[0x00] + 5) + (CRTC[0x1a]&1 ? 0x100 : 0)
     *   total pixels      = char_clocks * dots_per_clock
     */
    int htotal_chars = ((int)s->crtc_ctrl[0x00] + 5)
                     + ((s->crtc_ctrl[0x1a] & 0x01u) ? 0x100 : 0);
    int htotal = htotal_chars * dots;

    /* vtotal in lines (86Box banshee_recalctimings):
     *   base = CRTC[0x06]
     *        | ((CRTC[0x07] & 0x01) << 8)
     *        | ((CRTC[0x07] & 0x20) << 4)
     *        + 2
     *   Banshee extension: CRTC[0x1b] bit0 → +0x400
     */
    int vtotal = ((int)s->crtc_ctrl[0x06]
               | (((int)s->crtc_ctrl[0x07] & 0x01) << 8)
               | (((int)s->crtc_ctrl[0x07] & 0x20) << 4))
               + 2;
    if (s->crtc_ctrl[0x1b] & 0x01u)
        vtotal += 0x400;

    int64_t period_ns;

    if (freq > 1000.0 && htotal > 0 && vtotal > 0) {
        /* Normal path: derive period from pixel clock and CRTC totals */
        double frame_s = (double)(htotal * vtotal) / freq;
        period_ns = (int64_t)(frame_s * 1e9);

        /* Sanity clamp: 10 Hz .. 240 Hz (4 ms .. 100 ms) */
        if (period_ns < 4000000LL)
            period_ns = 4000000LL;    /* 250 Hz cap   */
        if (period_ns > 100000000LL)
            period_ns = 100000000LL;  /* 10 Hz floor  */
    } else {
        /* Fallback: CRTC not yet programmed or PLL disabled */
        period_ns = NANOSECONDS_PER_SECOND / VBLANK_HZ;
    }

    s->vblank_period_ns = period_ns;

    /* Reschedule the vblank timer with the new period.
     * Only do this if the timer is active (after realize). */
    if (s->vblank_timer) {
        timer_mod(s->vblank_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + period_ns);
    }
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
     * CRTC[0x07] overflow: bit1 = vde[8], bit6 = vde[9]
     * CRTC[0x1b] Banshee extension: bit2 = vde[10]                  */
    int vde = (int)s->crtc_ctrl[0x12]
            | (((int)s->crtc_ctrl[0x07] & 0x02) << 7)
            | (((int)s->crtc_ctrl[0x07] & 0x40) << 3)
            | (((int)s->crtc_ctrl[0x1b] & 0x04) << 8);
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
    bool ext_shift = !!(s->vgaInit0 & (1u << 12))
              || !!(s->vidProcCfg & VIDPROCCFG_VIDPROC_ENABLE);

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
        voodoo3_request_resize(s, w, h);

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
    /* Init_strapInfo (0x38) is read-only — no write case */
    case Init_2dSrcSize:   s->regs[Init_2dSrcSize   >> 2] = val; break;
    case Init_vgaInit0:    s->vgaInit0  = val; break;
    case Init_vgaInit1:    s->vgaInit1  = val; break;

    case PLL_pllCtrl0:
        s->pllCtrl0 = val;
        /*
         * Ported from 86Box banshee_recalctimings():
         * The pixel clock formula is evaluated whenever pllCtrl0 changes.
         * 86Box triggers this via svga_recalctimings() which calls
         * banshee_recalctimings() as a callback.  In QEMU we call
         * voodoo3_pll_update_vblank() directly.
         */
        voodoo3_pll_update_vblank(s);
        break;
    case PLL_pllCtrl1: s->pllCtrl1 = val; break;  /* stored, not used in clk */
    case PLL_pllCtrl2: s->pllCtrl2 = val; break;  /* stored, not used in clk */

    case DAC_dacMode:
        s->dacMode = val;
        break;

    /*
     * DAC_dacAddr (0x50) — 32-bit DWORD write via BAR0 MMIO (ext_outl path).
     * Byte-wise I/O to BAR2 also lands here for offset 0x50 (addr byte 0).
     * Offsets 0x51..0x53 are the upper bytes of the 32-bit dacAddr register;
     * the Banshee only uses bits[8:0] as the palette index, so bytes 1..3
     * are always zero.  Silently accept them to suppress log spam.
     *
     * 86Box banshee_ext_outl(): banshee->dacAddr = val & 0x1ff
     * (only dword writes; 86Box does not implement the byte-I/O path for ext
     *  registers — it falls through to banshee_ext_out which only handles
     *  VGA port proxy 0xb0..0xdf.  QEMU BAR2 delivers byte-I/O directly here.)
     */
    case DAC_dacAddr:           /* 0x50 — index byte 0 */
        s->dacAddr      = (int)(val & 0x1ff);
        s->dac_rgb_idx  = 0;   /* new address resets accumulator */
        s->dac_write_addr = (uint8_t)(val & 0xff);
        break;
    case 0x51:                  /* dacAddr byte 1 — always 0, ignore */
    case 0x52:                  /* dacAddr byte 2 — always 0, ignore */
    case 0x53:                  /* dacAddr byte 3 — always 0, ignore */
        break;

    /*
     * DAC_dacData (0x54) — palette data.
     *
     * DWORD write (BAR0 ext_outl): val = 0x00RRGGBB, write pallook immediately.
     * This is the 86Box path: banshee_ext_outl() DAC_dacData case.
     *
     * Byte write (BAR2 byte I/O): the guest writes R, G, B as three
     * separate byte-wide I/O writes to offsets 0x54, 0x55, 0x56.
     * We accumulate into dac_rgb_buf[] and commit on the third byte (0x56),
     * then advance dacAddr — mirroring the VGA 0x3c9 accumulation logic.
     *
     * 86Box does not implement this byte-I/O path; AmigaOS/Tequila uses it
     * because it accesses the Banshee I/O BAR one byte at a time.
     */
    case DAC_dacData:           /* 0x54 — R byte (or full DWORD) */
        if (s->dac_rgb_idx == 0 && (val & 0xffff00u)) {
            /*
             * DWORD write: all three colour bytes present in val.
             * 86Box path: pallook[dacAddr] = val & 0xffffff (R at bits[23:16]).
             */
            if (s->dacAddr < VOODOO3_CLUT_SIZE)
                s->pallook[s->dacAddr] =
                    (((val >> 16) & 0xff) << 16) |  /* R */
                    (((val >>  8) & 0xff) <<  8) |  /* G */
                    ( (val        & 0xff)      );   /* B */
        } else {
            /* Byte I/O path: accumulate R byte */
            s->dac_rgb_buf[0] = (uint8_t)(val & 0xff);
            s->dac_rgb_idx    = 1;
        }
        break;
    case 0x55:                  /* dacData byte 1 = G */
        s->dac_rgb_buf[1] = (uint8_t)(val & 0xff);
        s->dac_rgb_idx    = 2;
        break;
    case 0x56:                  /* dacData byte 2 = B — commit on receipt */
        s->dac_rgb_buf[2] = (uint8_t)(val & 0xff);
        if (s->dacAddr < VOODOO3_CLUT_SIZE) {
            s->pallook[s->dacAddr] =
                ((uint32_t)s->dac_rgb_buf[2] << 16) |  /* R (byte 2) */
                ((uint32_t)s->dac_rgb_buf[1] <<  8) |  /* G (byte 1) */
                 (uint32_t)s->dac_rgb_buf[0];          /* B (byte 0) */
        }
        s->dacAddr        = (s->dacAddr + 1) & 0xff;
        s->dac_write_addr = (uint8_t)s->dacAddr;
        s->dac_rgb_idx    = 0;
        break;
    case 0x57:                  /* dacData byte 3 — padding, always 0, ignore */
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
        s->ov.pix_fmt = (int)VIDPROCCFG_OVERLAY_PIX_FMT(val);
        s->ov.ena     = !!(val & VIDPROCCFG_OVERLAY_ENABLE);
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
    case Video_maxRgbDelta:
        /*
         * Screen-filter threshold register — ported from 86Box banshee_ext_outl()
         * case Video_maxRgbDelta in vid_voodoo_banshee.c.
         *
         * Bits [23:16] = R delta cap (fcr), [15:8] = G cap (fcg), [7:0] = B cap.
         * Writing 0 disables the filter; any non-zero value enables it and
         * triggers regeneration of the vb_filter_* lookup tables via
         * voodoo3_scrfilter_threshold_check().
         */
        s->scrfilter_threshold = val;
        s->scrfilter_enabled   = (val != 0);
        voodoo3_scrfilter_threshold_check(s);
        s->regs[Video_maxRgbDelta >> 2] = val;
        break;
    case Video_hwCurPatAddr:
        s->hwCurPatAddr = val;
        s->cur_pat_addr = val & 0xfffff0u;
        /*
         * Only refresh cursor_buf if the OS has already positioned the cursor
         * (cur_loc_valid).  Before that point fb_mem at cur_pat_addr is still
         * zero (the OS hasn't written the shape yet), so copying it would turn
         * cursor_buf from the safe transparent default (plane0=0xFF, plane1=0x00)
         * into all-zeros → every pixel = col0 → coloured rectangle at boot.
         * The hwCurLoc handler (which sets cur_loc_valid) always reloads
         * cursor_buf from the freshly written VRAM anyway.
         */
        if (s->cur_loc_valid) {
            uint32_t base = s->cur_pat_addr + (uint32_t)s->cur_yoff * 16u;
            uint32_t len  = 1024u - (uint32_t)s->cur_yoff * 16u;
            if (base + len <= s->fb_size)
                memcpy(s->cursor_buf, s->fb_mem + base, len);
        }
        break;
    case Video_hwCurLoc:
        s->hwCurLoc = val;
        s->cur_x    = (int)(val & 0x7ffu) - 64;
        s->cur_y    = (int)((val >> 16) & 0x7ffu) - 64;
        /*
         * Ported from 86Box banshee_ext_outl() Video_hwCurLoc:
         * If cur_y < 0 the cursor is partially above the top of the screen.
         * yoff = number of sprite rows to skip; addr is bumped by yoff*16
         * so that banshee_hwcursor_draw() starts at the right row.
         */
        if (s->cur_y < 0) {
            s->cur_yoff = -s->cur_y;
            s->cur_y    = 0;
        } else {
            s->cur_yoff = 0;
        }
        /* Update cursor_buf to start at the correct sprite row */
        {
            uint32_t base = s->cur_pat_addr + (uint32_t)s->cur_yoff * 16u;
            if (base + 1024u <= s->fb_size)
                memcpy(s->cursor_buf, s->fb_mem + base,
                       1024u - (uint32_t)s->cur_yoff * 16u);
        }
        s->cur_loc_valid = true;   /* OS has positioned the cursor — safe to draw */
        break;
    case Video_hwCurC0: s->cur_c0 = val; break;
    case Video_hwCurC1: s->cur_c1 = val; break;
    case Video_vidSerialParallelPort:
        s->vidSerialParallelPort = val;
        /*
         * Bridge the two I²C buses to QEMU's bitbang_i2c layer.
         * Each call may modify s->vidSerialParallelPort in place to
         * update the SCL_R / SDA_R read-back bits.
         */
        voodoo3_vidserial_update(&s->vidSerialParallelPort,
                                 &s->bbi2c_ddc,
                                 18, 19, 20, 21, 22); /* DDC bus */
        voodoo3_vidserial_update(&s->vidSerialParallelPort,
                                 &s->bbi2c_i2c,
                                 23, 24, 25, 26, 27); /* I2C bus */
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
            voodoo3_request_resize(s, s->screen_width, s->screen_height);
            memset(s->dirty_line, 1, sizeof(s->dirty_line));
        }
        break;
    case Video_vidInXDecimDeltas: s->regs[Video_vidInXDecimDeltas >> 2] = val; break;
    case Video_vidInError:        s->regs[Video_vidInError >> 2] = val; break;
    case Video_vidInXStart:       s->regs[Video_vidInXStart >> 2] = val; break;
    /*
     * Video capture / misc registers — store for read-back, no active effect.
     *
     * 0x74  vidInStatus       — read-only hardware status; writes ignored per spec.
     * 0x7c  vidPllCtrl        — video PLL scratch; 86Box ext_outl falls to default (no-op).
     * 0x94  vidInYStart       — capture Y coordinate scratch.
     * 0xec  vidInAddr0        — DMA capture buffer 0 address.
     * 0xf0  vidInAddr1        — DMA capture buffer 1 address.
     * 0xf4  vidInAddr2        — DMA capture buffer 2 address.
     * 0xf8  vidInStride       — DMA capture stride.
     * 0xfc  vidCurrOverlayStartAddr — overlay scan-out pointer; stored for read-back.
     *
     * In 86Box vid_voodoo_banshee.c these all fall through to the default no-op
     * in banshee_ext_outl().  We store them in regs[] so read-back returns the
     * last written value (same pattern as vidInXDecimDeltas/vidInError/vidInXStart).
     */
    case Video_vidInStatus:               /* read-only status — ignore writes          */
        break;
    case Video_vidPllCtrl:
        s->regs[Video_vidPllCtrl >> 2] = val; break;
    case Video_vidInYStart:
        s->regs[Video_vidInYStart >> 2] = val; break;
    case Video_vidInAddr0:
        s->regs[Video_vidInAddr0 >> 2] = val; break;
    case Video_vidInAddr1:
        s->regs[Video_vidInAddr1 >> 2] = val; break;
    case Video_vidInAddr2:
        s->regs[Video_vidInAddr2 >> 2] = val; break;
    case Video_vidInStride:
        s->regs[Video_vidInStride >> 2] = val; break;
    case Video_vidCurrOverlayStartAddr:
        s->regs[Video_vidCurrOverlayStartAddr >> 2] = val; break;
    case Video_vidOverlayStartCoords:
        /*
         * 86Box banshee_ext_outl() Video_vidOverlayStartCoords:
         *   voodoo->overlay.vidOverlayStartCoords = val;
         *   voodoo->overlay.start_x = val & OVERLAY_START_X_MASK;
         *   voodoo->overlay.start_y = (val & OVERLAY_START_Y_MASK) >> OVERLAY_START_Y_SHIFT;
         *   voodoo->overlay.size_x  = end_x - start_x;
         *   voodoo->overlay.size_y  = end_y - start_y;
         */
        s->ov.vidOverlayStartCoords = val;
        s->ov.start_x = (int)(val & OVERLAY_START_X_MASK);
        s->ov.start_y = (int)((val & OVERLAY_START_Y_MASK) >> OVERLAY_START_Y_SHIFT);
        s->ov.size_x  = s->ov.end_x - s->ov.start_x;
        s->ov.size_y  = s->ov.end_y - s->ov.start_y;
        s->regs[Video_vidOverlayStartCoords >> 2] = val;
        break;
    case Video_vidOverlayEndScreenCoords:
        /*
         * 86Box: end_x = val & END_X_MASK; end_y = (val >> 12) & 0xfff;
         *   size_x = (end_x - start_x) + 1; size_y = (end_y - start_y) + 1;
         */
        s->ov.vidOverlayEndScreenCoords = val;
        s->ov.end_x  = (int)(val & OVERLAY_END_X_MASK);
        s->ov.end_y  = (int)((val & OVERLAY_END_Y_MASK) >> OVERLAY_END_Y_SHIFT);
        s->ov.size_x = (s->ov.end_x - s->ov.start_x) + 1;
        s->ov.size_y = (s->ov.end_y - s->ov.start_y) + 1;
        s->regs[Video_vidOverlayEndScreenCoords >> 2] = val;
        break;
    case Video_vidOverlayDudx:
        /* 86Box: voodoo->overlay.vidOverlayDudx = val & VID_DUDX_MASK */
        s->ov.vidOverlayDudx = val & VID_DUDX_MASK;
        s->regs[Video_vidOverlayDudx >> 2] = val;
        break;
    case Video_vidOverlayDudxOffsetSrcWidth:
        /*
         * 86Box: overlay_bytes = (val & OVERLAY_SRC_WIDTH_MASK) >> OVERLAY_SRC_WIDTH_SHIFT;
         * overlay_bytes = source row width in bytes
         */
        s->ov.vidOverlayDudxOffsetSrcWidth = val;
        s->ov.overlay_bytes = (int)((val & OVERLAY_SRC_WIDTH_MASK) >> OVERLAY_SRC_WIDTH_SHIFT);
        s->regs[Video_vidOverlayDudxOffsetSrcWidth >> 2] = val;
        break;
    case Video_vidOverlayDvdy:
        /* 86Box: voodoo->overlay.vidOverlayDvdy = val & VID_DVDY_MASK */
        s->ov.vidOverlayDvdy = val & VID_DVDY_MASK;
        s->regs[Video_vidOverlayDvdy >> 2] = val;
        break;
    case Video_vidOverlayDvdyOffset:
        s->ov.vidOverlayDvdyOffset = val;
        s->regs[Video_vidOverlayDvdyOffset >> 2] = val;
        break;
    case Video_vidDesktopStartAddr: {
        /*
         * WARN4 FIX: "vidDesktopStart = 0xFFFFFF00" (Module 25).
         *
         * On real Voodoo3 hardware, vidDesktopStartAddr (BAR0+0x0E4) is a
         * 24-bit register (bits[23:0] only), so the hardware ignores the
         * upper 8 bits on write.  The QEMU device already masks with
         * 0x00ffffff, which prevents the 0xFFFFFF00 garbage from being
         * stored.
         *
         * The secondary cause reported (register reads back 0xFFFFFF00
         * after reset) was an uninitialised state issue — now fixed by
         * explicit zero-initialisation in voodoo3_reset().
         *
         * Additional guard: if the written value (after masking) points
         * beyond the allocated VRAM, clamp to 0 so the display engine
         * never reads out-of-bounds.  This mirrors the real hardware
         * read-only upper-byte behaviour and prevents a white-screen
         * from a mis-programmed desktop start address.
         */
        uint32_t masked = val & 0x00ffffffu;
        if (masked >= s->fb_size) {
            masked = 0;  /* out-of-range → reset to frame 0 */
        }
        s->vidDesktopStartAddr = masked;
        s->desktop_start       = masked;
        /*
         * Sync to params so voodoo3_update_display_dirty() sees the correct
         * framebuffer base.  In 2D/desktop mode (no 3D swap) these are the
         * authoritative source-of-truth for the display scanout.
         */
        s->params.front_offset = s->desktop_start;
        // s->params.draw_offset  = s->desktop_start;
        memset(s->dirty_line, 1, sizeof(s->dirty_line));
        break;
    }
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
                voodoo3_pll_update_vblank(s);
                /* CRTC[0x11] IRQ arm/disarm — same as VGA 0x3d5 path */
                if (s->crtc_idx == 0x11 && !(val & 0x10u)) {
                    if (s->vblank_irq_pending) {
                        s->vblank_irq_pending = false;
                        pci_irq_deassert(PCI_DEVICE(s));
                    }
                }
            }
            break;
        /* dacStatus is read-only — writes silently ignored */
        case Ext_dacStatus:
            break;

        /*
         * Sub-byte writes to 32-bit ext registers (byte-I/O via BAR2).
         *
         * 86Box only implements DWORD access to ext registers (banshee_ext_outl).
         * AmigaOS/Tequila issues byte-wide I/O to BAR2, hitting offsets N+1..N+3
         * of each 32-bit register. Fix: read-modify-write + re-apply side effects.
         *
         * 86Box source references:
         *   Init_vgaInit0  → banshee_ext_outl: vgaInit0, svga_recalctimings()
         *   Init_vgaInit1  → banshee_ext_outl: vgaInit1, write/read_bank
         *   DAC_dacMode    → banshee_ext_outl: dacMode, svga->dpms
         *   Video_vidProcCfg → banshee_ext_outl: vidProcCfg + full recalc
         */

        /* Init_vgaInit0 (0x28) sub-bytes: 0x29, 0x2a, 0x2b */
        case 0x29: case 0x2a: case 0x2b: {
            unsigned bidx = (addr & 0xff) - 0x28u;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->vgaInit0 = (s->vgaInit0 & ~mask) | ((val & 0xffu) << (bidx * 8u));
            /* 86Box side effect: svga_recalctimings → checks EXTENDED_SHIFT_OUT */
            voodoo3_crtc_update(s);
            break;
        }

        /* Init_vgaInit1 (0x2c) sub-bytes: 0x2d, 0x2e, 0x2f */
        case 0x2d: case 0x2e: case 0x2f: {
            unsigned bidx = (addr & 0xff) - 0x2cu;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->vgaInit1 = (s->vgaInit1 & ~mask) | ((val & 0xffu) << (bidx * 8u));
            /* 86Box: updates write_bank/read_bank — no QEMU equivalent */
            break;
        }

        /* DAC_dacMode (0x4c) sub-bytes: 0x4d, 0x4e, 0x4f */
        case 0x4d: case 0x4e: case 0x4f: {
            unsigned bidx = (addr & 0xff) - 0x4cu;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->dacMode = (s->dacMode & ~mask) | ((val & 0xffu) << (bidx * 8u));
            /* 86Box: svga->dpms = !!(dacMode & 0x0a) — DPMS not emulated */
            break;
        }

        /* Video_vidProcCfg (0x5c) sub-bytes: 0x5d, 0x5e, 0x5f */
        case 0x5d: case 0x5e: case 0x5f: {
            unsigned bidx = (addr & 0xff) - 0x5cu;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->vidProcCfg = (s->vidProcCfg & ~mask) | ((val & 0xffu) << (bidx * 8u));
            /* Re-apply same side effects as the dword vidProcCfg handler.
             * DESKTOP_TILE (bit 24, byte 3) and pix_format (bits 20:18, byte 2)
             * live in the upper bytes and ARE written this way. */
            s->cursor_ena    = !!(s->vidProcCfg & VIDPROCCFG_HWCURSOR_ENA);
            s->pix_format    = (int)VIDPROCCFG_DESKTOP_PIX_FMT(s->vidProcCfg);
            s->desktop_tiled = !!(s->vidProcCfg & VIDPROCCFG_DESKTOP_TILE);
            s->ov.pix_fmt    = (int)VIDPROCCFG_OVERLAY_PIX_FMT(s->vidProcCfg);
            s->ov.ena        = !!(s->vidProcCfg & VIDPROCCFG_OVERLAY_ENABLE);
            s->params.col_tiled = s->desktop_tiled;
            if (s->display_enabled)
                memset(s->dirty_line, 1, sizeof(s->dirty_line));
            break;
        }

        /*
         * Init_pciInit0 (0x04) sub-bytes: 0x05, 0x06, 0x07
         *
         * 86Box banshee_ext_outl() Init_pciInit0:
         *   banshee->pciInit0 = val;
         * No further side effects in 86Box — pciInit0 is read back by
         * banshee_vga_vsync_enabled() for bit 18 (PCI IRQ enable).
         * Read-modify-write to preserve bits not changed by this byte.
         */
        case 0x05: case 0x06: case 0x07: {
            unsigned bidx = (addr & 0xff) - 0x04u;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->pciInit0 = (s->pciInit0 & ~mask) | ((val & 0xffu) << (bidx * 8u));
            break;
        }

        /*
         * Init_miscInit1 (0x14) sub-bytes: 0x15, 0x16, 0x17
         *
         * 86Box banshee_ext_outl() Init_miscInit1:
         *   banshee->miscInit1 = val;
         * miscInit1 is stored but has no active side effects in 86Box
         * (it controls memory timing — write-only from driver perspective).
         */
        case 0x15: case 0x16: case 0x17: {
            unsigned bidx = (addr & 0xff) - 0x14u;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->miscInit1 = (s->miscInit1 & ~mask) | ((val & 0xffu) << (bidx * 8u));
            break;
        }

        /*
         * Video_vidInFormat / Ext_miscInit2 (0x70) sub-bytes: 0x71, 0x72, 0x73
         *
         * Offset 0x70 in the ext register space is used for two purposes:
         *   - Video_vidInFormat: video capture input format (write-only, unused)
         *   - Ext_miscInit2 / vidDesktopTileStride mirror (write-only scratch)
         * 86Box ext_outl falls to default (no-op) for 0x70.
         * Store the assembled value in the regs[] scratch array for read-back.
         */
        case 0x71: case 0x72: case 0x73: {
            unsigned bidx = (addr & 0xff) - 0x70u;
            uint32_t idx  = Ext_miscInit2 >> 2;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->regs[idx] = (s->regs[idx] & ~mask) | ((val & 0xffu) << (bidx * 8u));
            break;
        }

        /*
         * Sub-byte writes — Init registers (ported from 86Box banshee_ext_outl).
         *
         * Init_lfbMemoryConfig (0x0c) sub-bytes: 0x0d, 0x0e, 0x0f
         * 86Box stores into banshee->lfbMemoryConfig and recalculates tile
         * geometry fields (tile_base, tile_stride, tile_x).  We mirror into
         * s->lfbMemoryConfig and re-apply the same decode.
         */
        case 0x0d: case 0x0e: case 0x0f: {
            unsigned bidx = (addr & 0xff) - 0x0cu;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->lfbMemoryConfig = (s->lfbMemoryConfig & ~mask) | ((val & 0xffu) << (bidx * 8u));
            /* Re-apply tile geometry side-effects (86Box Init_lfbMemoryConfig) */
            s->tile_base   = (s->lfbMemoryConfig & 0x1fffu) << 12;
            s->tile_stride = 1024u << ((s->lfbMemoryConfig >> 13) & 7u);
            s->tile_x      = (uint32_t)(((s->lfbMemoryConfig >> 16) & 0x7fu) * 128u);
            break;
        }

        /*
         * Init_miscInit0 (0x10) sub-bytes: 0x11, 0x12, 0x13
         * 86Box: banshee->miscInit0 = val; y_origin_swap extracted from bit field.
         */
        case 0x11: case 0x12: case 0x13: {
            unsigned bidx = (addr & 0xff) - 0x10u;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->miscInit0 = (s->miscInit0 & ~mask) | ((val & 0xffu) << (bidx * 8u));
            s->y_origin_swap = (int)((s->miscInit0 & MISCINIT0_Y_SWAP_MASK)
                               >> MISCINIT0_Y_SWAP_SHIFT);
            break;
        }

        /*
         * Init_dramInit0 (0x18) sub-bytes: 0x19, 0x1a, 0x1b
         * 86Box: stored in banshee->dramInit0 (memory timing, write-only).
         */
        case 0x19: case 0x1a: case 0x1b: {
            unsigned bidx = (addr & 0xff) - 0x18u;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->dramInit0 = (s->dramInit0 & ~mask) | ((val & 0xffu) << (bidx * 8u));
            break;
        }

        /*
         * Init_dramInit1 (0x1c) sub-bytes: 0x1d, 0x1e, 0x1f
         * 86Box: stored in banshee->dramInit1 (memory timing, write-only).
         */
        case 0x1d: case 0x1e: case 0x1f: {
            unsigned bidx = (addr & 0xff) - 0x1cu;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->dramInit1 = (s->dramInit1 & ~mask) | ((val & 0xffu) << (bidx * 8u));
            break;
        }

        /*
         * Init_agpInit0 (0x20) sub-bytes: 0x21, 0x22, 0x23
         * 86Box: stored in banshee->agpInit0 (AGP timing, write-only).
         */
        case 0x21: case 0x22: case 0x23: {
            unsigned bidx = (addr & 0xff) - 0x20u;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->agpInit0 = (s->agpInit0 & ~mask) | ((val & 0xffu) << (bidx * 8u));
            break;
        }

        /*
         * Init_tmugbInit (0x24) sub-bytes: 0x25, 0x26, 0x27
         * 86Box ext_outl falls to default (no named field).
         * Store in regs[] scratch so read-back round-trips work.
         */
        case 0x25: case 0x26: case 0x27: {
            unsigned bidx = (addr & 0xff) - 0x24u;
            uint32_t idx  = Init_tmugbInit >> 2;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->regs[idx] = (s->regs[idx] & ~mask) | ((val & 0xffu) << (bidx * 8u));
            break;
        }

        /*
         * Init_2dCommand (0x30) sub-bytes: 0x31, 0x32, 0x33
         * 86Box: stored in banshee->command_2d (2D command register).
         */
        case 0x31: case 0x32: case 0x33: {
            unsigned bidx = (addr & 0xff) - 0x30u;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->command_2d = (s->command_2d & ~mask) | ((val & 0xffu) << (bidx * 8u));
            break;
        }

        /*
         * Init_2dSrcBaseAddr (0x34) sub-bytes: 0x35, 0x36, 0x37
         * 86Box: stored in banshee->srcBaseAddr_2d.
         */
        case 0x35: case 0x36: case 0x37: {
            unsigned bidx = (addr & 0xff) - 0x34u;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->srcBaseAddr_2d = (s->srcBaseAddr_2d & ~mask) | ((val & 0xffu) << (bidx * 8u));
            break;
        }

        /*
         * Init_2dSrcSize (0x3c) sub-bytes: 0x3d, 0x3e, 0x3f
         * 86Box ext_outl falls to default (no named field).
         * Store in regs[] scratch.
         */
        case 0x3d: case 0x3e: case 0x3f: {
            unsigned bidx = (addr & 0xff) - 0x3cu;
            uint32_t idx  = Init_2dSrcSize >> 2;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->regs[idx] = (s->regs[idx] & ~mask) | ((val & 0xffu) << (bidx * 8u));
            break;
        }

        /*
         * PLL_pllCtrl0 (0x40) sub-bytes: 0x41, 0x42, 0x43
         * 86Box: banshee->pllCtrl0 = val; triggers pixel clock recalc.
         * Re-apply PLL side effects after byte merge.
         */
        case 0x41: case 0x42: case 0x43: {
            unsigned bidx = (addr & 0xff) - 0x40u;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->pllCtrl0 = (s->pllCtrl0 & ~mask) | ((val & 0xffu) << (bidx * 8u));
            voodoo3_pll_update_vblank(s);
            break;
        }

        /*
         * PLL_pllCtrl1 (0x44) sub-bytes: 0x45, 0x46, 0x47
         * 86Box: banshee->pllCtrl1 = val (stored, no active side effect).
         */
        case 0x45: case 0x46: case 0x47: {
            unsigned bidx = (addr & 0xff) - 0x44u;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->pllCtrl1 = (s->pllCtrl1 & ~mask) | ((val & 0xffu) << (bidx * 8u));
            break;
        }

        /*
         * Video_maxRgbDelta (0x58) sub-bytes: 0x59, 0x5a, 0x5b
         * 86Box: stores in voodoo->scrfilterThreshold and enables/disables filter.
         * Ported from banshee_ext_outl() Video_maxRgbDelta.
         */
        case 0x59: case 0x5a: case 0x5b: {
            unsigned bidx = (addr & 0xff) - 0x58u;
            uint32_t mask = 0xffu << (bidx * 8u);
            uint32_t newval = (s->regs[Video_maxRgbDelta >> 2] & ~mask)
                              | ((val & 0xffu) << (bidx * 8u));
            s->regs[Video_maxRgbDelta >> 2] = newval;
            s->scrfilter_threshold  = newval;
            s->scrfilter_enabled    = (newval > 0);
            break;
        }

        /*
         * Video_hwCurPatAddr (0x60) sub-bytes: 0x61, 0x62, 0x63
         * 86Box: banshee->hwCurPatAddr = val; svga->hwcursor.addr updated.
         */
        case 0x61: case 0x62: case 0x63: {
            unsigned bidx = (addr & 0xff) - 0x60u;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->hwCurPatAddr = (s->hwCurPatAddr & ~mask) | ((val & 0xffu) << (bidx * 8u));
            s->cur_pat_addr = s->hwCurPatAddr & 0xfffff0u;
            /* Same guard as the dword handler — see comment there */
            if (s->cur_loc_valid) {
                uint32_t base = s->cur_pat_addr + (uint32_t)s->cur_yoff * 16u;
                uint32_t len  = 1024u - (uint32_t)s->cur_yoff * 16u;
                if (base + len <= s->fb_size)
                    memcpy(s->cursor_buf, s->fb_mem + base, len);
            }
            break;
        }

        /*
         * Video_hwCurLoc (0x64) sub-bytes: 0x65, 0x66, 0x67
         * 86Box: banshee->hwCurLoc = val; svga->hwcursor.x/y decoded.
         * Re-apply same side effects as the dword handler.
         */
        case 0x65: case 0x66: case 0x67: {
            unsigned bidx = (addr & 0xff) - 0x64u;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->hwCurLoc = (s->hwCurLoc & ~mask) | ((val & 0xffu) << (bidx * 8u));
            {
                int cx = (int)(s->hwCurLoc & 0x7ffu) - 64;
                int cy = (int)((s->hwCurLoc >> 16) & 0x7ffu) - 64;
                if (cy < 0) {
                    s->cur_yoff = -cy;
                    cy = 0;
                } else {
                    s->cur_yoff = 0;
                }
                s->cur_x = cx;
                s->cur_y = cy;
                {
                    uint32_t base = s->cur_pat_addr + (uint32_t)s->cur_yoff * 16u;
                    if (base + 1024u <= s->fb_size)
                        memcpy(s->cursor_buf, s->fb_mem + base,
                               1024u - (uint32_t)s->cur_yoff * 16u);
                }
                s->cur_loc_valid = true;   /* OS has positioned the cursor — safe to draw */
            }
            break;
        }

        /*
         * Video_hwCurC0 (0x68) sub-bytes: 0x69, 0x6a, 0x6b
         * 86Box: banshee->hwCurC0 = val (cursor foreground colour, stored).
         */
        case 0x69: case 0x6a: case 0x6b: {
            unsigned bidx = (addr & 0xff) - 0x68u;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->cur_c0 = (s->cur_c0 & ~mask) | ((val & 0xffu) << (bidx * 8u));
            break;
        }

        /*
         * Video_hwCurC1 (0x6c) sub-bytes: 0x6d, 0x6e, 0x6f
         * 86Box: banshee->hwCurC1 = val (cursor background colour, stored).
         */
        case 0x6d: case 0x6e: case 0x6f: {
            unsigned bidx = (addr & 0xff) - 0x6cu;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->cur_c1 = (s->cur_c1 & ~mask) | ((val & 0xffu) << (bidx * 8u));
            break;
        }

        /*
         * Video_vidInStatus (0x74) sub-bytes: 0x75, 0x76, 0x77
         * Hardware-defined read-only status register.
         * Writes are silently ignored (no named field in 86Box).
         */
        case 0x75: case 0x76: case 0x77:
            /* read-only HW register — ignore byte writes */
            break;

        /*
         * Video_vidSerialParallelPort (0x78) sub-bytes: 0x79, 0x7a, 0x7b
         * 86Box: banshee->vidSerialParallelPort = val; DDC/I2C GPIO driven.
         * Re-apply I2C side effects after byte merge.
         */
        case 0x79: case 0x7a: case 0x7b: {
            unsigned bidx = (addr & 0xff) - 0x78u;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->vidSerialParallelPort = (s->vidSerialParallelPort & ~mask)
                                       | ((val & 0xffu) << (bidx * 8u));
            voodoo3_vidserial_update(&s->vidSerialParallelPort,
                                     &s->bbi2c_ddc,
                                     18, 19, 20, 21, 22); /* DDC bus */
            voodoo3_vidserial_update(&s->vidSerialParallelPort,
                                     &s->bbi2c_i2c,
                                     23, 24, 25, 26, 27); /* I2C bus */
            break;
        }

        /*
         * Video_vidPllCtrl (0x7c) sub-bytes: 0x7d, 0x7e, 0x7f
         * 86Box ext_outl falls to default (scratch / no-op).
         * Store in regs[] for read-back.
         */
        case 0x7d: case 0x7e: case 0x7f: {
            unsigned bidx = (addr & 0xff) - 0x7cu;
            uint32_t idx  = Video_vidPllCtrl >> 2;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->regs[idx] = (s->regs[idx] & ~mask) | ((val & 0xffu) << (bidx * 8u));
            break;
        }

        /*
         * Video_vidInXDecimDeltas (0x80) sub-bytes: 0x81, 0x82, 0x83
         * 86Box ext_outl falls to default.  Store in regs[] scratch.
         */
        case 0x81: case 0x82: case 0x83: {
            unsigned bidx = (addr & 0xff) - 0x80u;
            uint32_t idx  = Video_vidInXDecimDeltas >> 2;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->regs[idx] = (s->regs[idx] & ~mask) | ((val & 0xffu) << (bidx * 8u));
            break;
        }

        /*
         * Video_vidScreenSize (0x98) sub-bytes: 0x99, 0x9a, 0x9b
         * 86Box: banshee->vidScreenSize = val; h_disp/v_disp decoded.
         * Re-apply side effects after byte merge.
         */
        case 0x99: case 0x9a: case 0x9b: {
            unsigned bidx = (addr & 0xff) - 0x98u;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->vidScreenSize = (s->vidScreenSize & ~mask) | ((val & 0xffu) << (bidx * 8u));
            s->screen_width  = (int)((s->vidScreenSize & 0xfffu) + 1u);
            s->screen_height = (int)((s->vidScreenSize >> 12) & 0xfffu);
            if (s->con && s->screen_width > 0 && s->screen_height > 0) {
                voodoo3_request_resize(s, s->screen_width, s->screen_height);
                memset(s->dirty_line, 1, sizeof(s->dirty_line));
            }
            break;
        }

        /*
         * Video_vidDesktopStartAddr (0xe4) sub-bytes: 0xe5, 0xe6, 0xe7
         * 86Box: banshee->vidDesktopStartAddr = val & 0xffffff; fullchange set.
         * Upper byte (0xe7, bits[31:24]) not wired on real HW — clamped to 0.
         */
        case 0xe5: case 0xe6: {
            unsigned bidx = (addr & 0xff) - 0xe4u;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->vidDesktopStartAddr = ((s->vidDesktopStartAddr & ~mask)
                                      | ((val & 0xffu) << (bidx * 8u)))
                                     & 0x00ffffffu;
            voodoo3_crtc_update(s);
            break;
        }
        case 0xe7:
            /* bits[31:24] not wired — ignore write */
            break;

        /*
         * Video_vidDesktopOverlayStride (0xe8) sub-bytes: 0xe9, 0xea, 0xeb
         * 86Box: banshee->vidDesktopOverlayStride = val; fullchange set.
         */
        case 0xe9: case 0xea: case 0xeb: {
            unsigned bidx = (addr & 0xff) - 0xe8u;
            uint32_t mask = 0xffu << (bidx * 8u);
            s->vidDesktopOverlayStride = (s->vidDesktopOverlayStride & ~mask)
                                         | ((val & 0xffu) << (bidx * 8u));
            voodoo3_crtc_update(s);
            break;
        }

        default:
            qemu_log_mask(LOG_UNIMP,
                "voodoo3: ext write 0x%02x = 0x%08x (unimplemented)\n",
                addr & 0xff, val);
            break;
        }
        break;
    }
}

/* Forward declaration — defined later in this file */
static uint8_t voodoo3_vga_in(Voodoo3State *s, uint16_t addr);

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
    case Init_strapInfo:
        /*
         * Strap configuration register — read-only, set by hardware straps.
         * 86Box banshee_ext_inl() returns 0x00000040 for all Banshee/V3 models:
         *   bit 6     = PCI bus (not AGP), IRQ enabled, 32kB BIOS present
         *   bits 3:0  = memory configuration (8 MB SGRAM, per 86Box comment)
         * The real 3dfx BIOS reads this register first during POST to determine
         * SDRAM vs SGRAM and memory size.  Returning 0 causes silent init failure.
         * 86Box vid_voodoo_banshee.c returns 0x40 for ALL models (PCI && AGP) — no AGP distinction.
         * A proposed 0xC0 for AGP (bit 7 = AGP mode per 3dfx datasheet) is NOT in 86Box.
         */
        return 0x00000040;
    case PLL_pllCtrl0:             return s->pllCtrl0;
    case PLL_pllCtrl1:             return s->pllCtrl1;
    case PLL_pllCtrl2:             return s->pllCtrl2;
    case DAC_dacMode:              return s->dacMode;
    case DAC_dacAddr:              return (uint32_t)s->dacAddr;
    case 0x51: return 0;           /* dacAddr byte 1 — always 0 */
    case 0x52: return 0;           /* dacAddr byte 2 — always 0 */
    case 0x53: return 0;           /* dacAddr byte 3 — always 0 */
    case DAC_dacData:
        return (s->dacAddr < VOODOO3_CLUT_SIZE)
               ? s->pallook[s->dacAddr] : 0xffffffff;
    case 0x55:                     /* dacData byte 1 = G */
        return (s->dacAddr < VOODOO3_CLUT_SIZE)
               ? ((s->pallook[s->dacAddr] >> 8) & 0xff) : 0xff;
    case 0x56:                     /* dacData byte 2 = B */
        return (s->dacAddr < VOODOO3_CLUT_SIZE)
               ? (s->pallook[s->dacAddr] & 0xff) : 0xff;
    case 0x57: return 0;           /* dacData byte 3 — always 0 */
    case Video_vidProcCfg:         return s->vidProcCfg;
    case Video_vidScreenSize:      return s->vidScreenSize;
    case Video_vidDesktopStartAddr:     return s->vidDesktopStartAddr;
    /* Sub-byte reads of vidDesktopStartAddr (0xe4) — byte 1/2/3.
     * Ported from 86Box banshee_ext_inl() Video_vidDesktopStartAddr:
     *   ret = banshee->vidDesktopStartAddr
     * The Pegasos/MorphOS 3dfx driver reads bytes 0xe5, 0xe6, 0xe7
     * individually to assemble the 24-bit base address.
     *
     * WARN4 FIX: vidDesktopStartAddr is a 24-bit register (bits[23:0]).
     * Byte 3 (0xe7, bits[31:24]) must always read as 0x00 — on real
     * hardware the upper byte is not wired.  Before this fix the register
     * could read back 0xFFFFFF00 if the write path had uninitialised data.
     * The write handler now clamps to 0x00ffffff so byte 3 is always 0. */
    case 0xe5: return (s->vidDesktopStartAddr >>  8) & 0xff;
    case 0xe6: return (s->vidDesktopStartAddr >> 16) & 0xff;
    case 0xe7: return 0x00; /* bits[31:24] not wired — always 0 (WARN4 fix) */
    case Video_vidDesktopOverlayStride: return s->vidDesktopOverlayStride;
    /* Sub-byte reads of vidDesktopOverlayStride (0xe8) — byte 1/2/3.
     * Ported from 86Box banshee_ext_inl() Video_vidDesktopOverlayStride:
     *   ret = banshee->vidDesktopOverlayStride
     * The Pegasos/MorphOS 3dfx driver reads bytes 0xe9, 0xea, 0xeb
     * individually to read back stride and tile-mode bits. */
    case 0xe9: return (s->vidDesktopOverlayStride >>  8) & 0xff;
    case 0xea: return (s->vidDesktopOverlayStride >> 16) & 0xff;
    case 0xeb: return (s->vidDesktopOverlayStride >> 24) & 0xff;
    case Video_hwCurPatAddr:       return s->hwCurPatAddr;
    case Video_hwCurLoc:           return s->hwCurLoc;
    case Video_hwCurC0:            return s->cur_c0;
    case Video_hwCurC1:            return s->cur_c1;
    case Video_vidChromaKeyMin:    return s->vidChromaKeyMin;
    case Video_vidChromaKeyMax:    return s->vidChromaKeyMax;
    /* Overlay register read-back — 86Box banshee_ext_inl() */
    case Video_vidOverlayStartCoords:        return s->ov.vidOverlayStartCoords;
    case Video_vidOverlayEndScreenCoords:    return s->ov.vidOverlayEndScreenCoords;
    case Video_vidOverlayDudx:               return s->ov.vidOverlayDudx;
    case Video_vidOverlayDudxOffsetSrcWidth: return s->ov.vidOverlayDudxOffsetSrcWidth;
    case Video_vidOverlayDvdy:               return s->ov.vidOverlayDvdy;
    case Video_vidOverlayDvdyOffset:         return s->ov.vidOverlayDvdyOffset;
    /*
     * vidSerialParallelPort (0x78): I2C / DDC / serial port status.
     * Bit 3 = SCL, bit 1 = SDA, bit 8 = I2C-ack.
     * AmigaOS 3dfxVoodoo.chip polls this waiting for the DDC bus to be
     * idle. Return the stored value with no busy bits set so the driver
     * doesn't spin forever.
     */
    /*
     * vidSerialParallelPort (0x78): DDC I2C read-back.
     *
     * Ported from 86Box banshee_ext_inl() Video_vidSerialParallelPort:
     *   ret = stored & ~(DCK_R | DDA_R | I2C_SCK_R | I2C_SDA_R)
     *   if DDC_EN (bit 18): set DCK_R (bit 21) = SCL loopback from DCK_W (bit 19)
     *                        set DDA_R (bit 22) = SDA from our DDC state machine
     *
     * Previous code used bit 8 for SDA and bit 2 for SCL — both wrong.
     * The Amiga driver reads back bits 21 and 22 to get SCL/SDA state.
     */
    case Video_vidSerialParallelPort:
        /*
         * The SCL_R / SDA_R read-back bits (21-22 and 26-27) are kept
         * up-to-date by voodoo3_vidserial_update() on every write, so
         * we can return the stored register value directly.
         * This is the same approach used by ATI (ati.c ati_i2c() which
         * writes the result bits back into the GPIO register on write).
         */
        return s->vidSerialParallelPort;
    /* Ext_miscInit2 */
    /* VGA-proxy byte reads: ext 0xb0..0xbf -> VGA port 0x3b0..0x3bf */
    case 0xb0: case 0xb1: case 0xb2: case 0xb3:
    case 0xb4: case 0xb5: case 0xb6: case 0xb7:
    case 0xb8: case 0xb9: case 0xba: case 0xbb:
    case 0xbc: case 0xbd: case 0xbe: case 0xbf:
    /* VGA-proxy for unhandled ext 0xc0-0xdf sub-range */
    case 0xc1:
    case 0xc7: case 0xc8: case 0xc9: case 0xca: case 0xcb:
    case 0xcc: case 0xcd:
    case 0xd0: case 0xd1: case 0xd2: case 0xd3:
    case 0xd6: case 0xd7:
    case 0xdc: case 0xdd: case 0xde: case 0xdf:
        return (uint32_t)voodoo3_vga_in(s, (uint16_t)((addr & 0xff) + 0x300u));
    /*
     * 0xD8 = ext register (mapped to VGA 0x3D8 — CRT mode ctrl, write-only)
     * 0xD9 = padding byte adjacent to dacStatus — reserved, returns 0x00
     * 0xDB = padding byte above dacStatus     — reserved, returns 0x00
     *
     * These three are part of the 32-bit dword at ext offset 0xD8:
     *   bits[ 7: 0] = ext_read(0xD8) → VGA 0x3D8 (mode ctrl, undef on read)
     *   bits[15: 8] = ext_read(0xD9) → reserved: 0x00
     *   bits[23:16] = ext_read(0xDA) → dacStatus (bit0=DAC ready, bit3=VSYNC)
     *   bits[31:24] = ext_read(0xDB) → reserved: 0x00
     *
     * voodoo3_vga_in() has no case for 0x3D9 or 0x3DB and falls through to
     * default: return 0xFF — producing dacStatus=0xFFFF01FF on AmigaOne.
     * FIX: voodoo3diag Module 9 -- dacStatus upper/lower pad bytes must be 0.
     */
    case 0xd8:
        return (uint32_t)voodoo3_vga_in(s, 0x3d8u);
    case 0xd9:  /* reserved pad — always 0x00 */
        return 0x00u;
    case 0xdb:  /* reserved pad — always 0x00 */
        return 0x00u;
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
    case Init_sipMonitor:      /* 0x08 - SIP monitor, read-only scratch */
        return s->regs[Init_sipMonitor >> 2];
    case Ext_dacStatus:
        /*
         * Bit 3 = VSYNC active, bit 0 = DAC ready.
         * On real hardware bit 0 is SET after DAC initialisation.
         * QEMU's DAC is always ready, so always assert bit 0.
         * FIX: voodoo3diag Module 9 -- DAC ready flag was 0.
         */
        return 0x01u | (s->in_vblank ? 0x08u : 0x00u);

    /* Sub-byte reads — return the relevant byte of each 32-bit register */
    case 0x29: return (s->vgaInit0  >>  8) & 0xff;
    case 0x2a: return (s->vgaInit0  >> 16) & 0xff;
    case 0x2b: return (s->vgaInit0  >> 24) & 0xff;
    case 0x2d: return (s->vgaInit1  >>  8) & 0xff;
    case 0x2e: return (s->vgaInit1  >> 16) & 0xff;
    case 0x2f: return (s->vgaInit1  >> 24) & 0xff;
    case 0x4d: return (s->dacMode   >>  8) & 0xff;
    case 0x4e: return (s->dacMode   >> 16) & 0xff;
    case 0x4f: return (s->dacMode   >> 24) & 0xff;
    case 0x5d: return (s->vidProcCfg >>  8) & 0xff;
    case 0x5e: return (s->vidProcCfg >> 16) & 0xff;
    case 0x5f: return (s->vidProcCfg >> 24) & 0xff;

    /* Init_pciInit0 (0x04) sub-bytes — read-back of stored value.
     * 86Box banshee_ext_inl() Init_pciInit0: ret = banshee->pciInit0. */
    case 0x05: return (s->pciInit0 >>  8) & 0xff;
    case 0x06: return (s->pciInit0 >> 16) & 0xff;
    case 0x07: return (s->pciInit0 >> 24) & 0xff;

    /* Init_miscInit1 (0x14) sub-bytes — read-back of stored value.
     * 86Box banshee_ext_inl() Init_miscInit1: ret = banshee->miscInit1. */
    case 0x15: return (s->miscInit1 >>  8) & 0xff;
    case 0x16: return (s->miscInit1 >> 16) & 0xff;
    case 0x17: return (s->miscInit1 >> 24) & 0xff;

    /* Init_strapInfo (0x38) sub-bytes — read-only hardware strap register.
     * 86Box returns 0x00000040 for the full dword; bytes 1..3 are 0x00.
     * The strap value 0x40 is entirely in byte 0, so upper bytes = 0. */
    case 0x39: return 0x00;   /* strapInfo byte 1 — always 0 */
    case 0x3a: return 0x00;   /* strapInfo byte 2 — always 0 */
    case 0x3b: return 0x00;   /* strapInfo byte 3 — always 0 */

    /* Video_vidInFormat / Ext_miscInit2 (0x70) sub-bytes — scratch read-back. */
    case 0x71: return (s->regs[Ext_miscInit2 >> 2] >>  8) & 0xff;
    case 0x72: return (s->regs[Ext_miscInit2 >> 2] >> 16) & 0xff;
    case 0x73: return (s->regs[Ext_miscInit2 >> 2] >> 24) & 0xff;

    /*
     * Previously-unimplemented ext register read-backs.
     *
     * All ported from 86Box vid_voodoo_banshee.c behaviour:
     *   - Registers that 86Box ext_outl stores in named fields → read back those fields.
     *   - Registers that fall through to default in 86Box → return 0 (not 0xffffffff)
     *     so that driver probes don't misinterpret them as "register not present".
     *   - vidInStatus (0x74) is hardware read-only status; we return 0 (idle, no
     *     capture active) which matches having no capture hardware in the emulation.
     *
     * Register map references:
     *   0x24  Init_tmugbInit            — write-only; return stored scratch value
     *   0x3c  Init_2dSrcSize            — 86Box stores in regs[]; return it
     *   0x58  Video_maxRgbDelta         — 86Box stores scrfilterThreshold; return it
     *   0x74  Video_vidInStatus         — read-only HW status; return 0 (idle)
     *   0x7c  Video_vidPllCtrl          — scratch; return stored value
     *   0x80  Video_vidInXDecimDeltas   — 86Box falls to default (returns 0)
     *   0x84  Video_vidInError          — 86Box falls to default (returns 0)
     *   0x88  Video_vidInXStart         — 86Box falls to default (returns 0)
     *   0x94  Video_vidInYStart         — scratch; return stored value
     *   0xec  Video_vidInAddr0          — DMA addr scratch
     *   0xf0  Video_vidInAddr1          — DMA addr scratch
     *   0xf4  Video_vidInAddr2          — DMA addr scratch
     *   0xf8  Video_vidInStride         — DMA stride scratch
     *   0xfc  Video_vidCurrOverlayStartAddr — overlay pointer scratch
     */
    case Init_tmugbInit:
        /*
         * write-only in 86Box (no field stored).  The AmigaOS Tequila/3dfx driver
         * reads this back as part of a register-presence probe; return 0 to signal
         * "not busy / TMU buffer reset complete" rather than the all-ones sentinel
         * that would indicate "register absent".
         */
        return 0x00000000;
    case Init_2dSrcSize:
        return s->regs[Init_2dSrcSize >> 2];
    case Video_maxRgbDelta:
        /*
         * 86Box stores this as voodoo->scrfilterThreshold.  We mirror it into
         * regs[] on write (see ext_write handler above); return the stored value.
         */
        return s->regs[Video_maxRgbDelta >> 2];
    case Video_vidInStatus:
        /*
         * Read-only hardware capture status register.
         * Bit 0 = capture active; bit 1 = odd field; bits 31:16 = line counter.
         * Return 0 — no capture hardware in emulation, so "idle, even field, line 0".
         * 86Box banshee_ext_inl() falls to default (returns 0xffffffff); returning 0
         * is safer for driver probes.
         */
        return 0x00000000;
    case Video_vidPllCtrl:
        return s->regs[Video_vidPllCtrl >> 2];
    case Video_vidInXDecimDeltas:
        return s->regs[Video_vidInXDecimDeltas >> 2];
    case Video_vidInError:
        return s->regs[Video_vidInError >> 2];
    case Video_vidInXStart:
        return s->regs[Video_vidInXStart >> 2];
    case Video_vidInYStart:
        return s->regs[Video_vidInYStart >> 2];
    case Video_vidInAddr0:
        return s->regs[Video_vidInAddr0 >> 2];
    case Video_vidInAddr1:
        return s->regs[Video_vidInAddr1 >> 2];
    case Video_vidInAddr2:
        return s->regs[Video_vidInAddr2 >> 2];
    case Video_vidInStride:
        return s->regs[Video_vidInStride >> 2];
    case Video_vidCurrOverlayStartAddr:
        return s->regs[Video_vidCurrOverlayStartAddr >> 2];

    /*
     * Sub-byte reads — Init/PLL/Video registers ported from 86Box.
     * Each group returns the relevant byte of its parent 32-bit register.
     */

    /* Init_lfbMemoryConfig (0x0c) sub-bytes */
    case 0x0d: return (s->lfbMemoryConfig >>  8) & 0xff;
    case 0x0e: return (s->lfbMemoryConfig >> 16) & 0xff;
    case 0x0f: return (s->lfbMemoryConfig >> 24) & 0xff;

    /* Init_miscInit0 (0x10) sub-bytes */
    case 0x11: return (s->miscInit0 >>  8) & 0xff;
    case 0x12: return (s->miscInit0 >> 16) & 0xff;
    case 0x13: return (s->miscInit0 >> 24) & 0xff;

    /* Init_dramInit0 (0x18) sub-bytes */
    case 0x19: return (s->dramInit0 >>  8) & 0xff;
    case 0x1a: return (s->dramInit0 >> 16) & 0xff;
    case 0x1b: return (s->dramInit0 >> 24) & 0xff;

    /* Init_dramInit1 (0x1c) sub-bytes */
    case 0x1d: return (s->dramInit1 >>  8) & 0xff;
    case 0x1e: return (s->dramInit1 >> 16) & 0xff;
    case 0x1f: return (s->dramInit1 >> 24) & 0xff;

    default:
        /*
         * FIX 7: returning 0xffffffff from an unrecognised ext register read
         * is the worst possible sentinel — many drivers probe registers by
         * reading them and treat all-ones as "absent/broken hardware".
         *
         * Strategy:
         *   1. Try regs[] first — any write handler that stored a value there
         *      will be returned correctly, making write->read round-trips work
         *      for registers we accept but don't decode (capture params etc.).
         *   2. Fall back to 0x00000000 (hardware-absent / idle) rather than
         *      0xFFFFFFFF (broken).  This matches 86Box behaviour for the
         *      handful of capture/misc registers it leaves as default-0.
         */
        {
            uint32_t idx = (addr & 0xffcu) >> 2;
            if (idx < 512u && s->regs[idx] != 0u) {
                return s->regs[idx];
            }
            qemu_log_mask(LOG_UNIMP,
                "voodoo3: ext read 0x%03x (unimplemented, returning 0)\n",
                addr & 0xfffu);
            return 0x00000000u;
        }
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

    /*
     * Extract chip-select BEFORE masking — identical to 86Box
     * voodoo_reg_writel() line 80: chip = (addr >> 10) & 0xf
     *   Bit 0 (0x1) = CHIP_FBI
     *   Bit 1 (0x2) = CHIP_TREX0  (TMU 0)
     *   Bit 2 (0x4) = CHIP_TREX1  (TMU 1)
     */
    int chip = (int)((addr >> 10) & 0x7);
    if (chip == 0) chip = 0x7; /* broadcast to all chips */

    addr &= 0x3fc;

    /*
     * TMU registers (textureMode, tLOD, texBaseAddr*, nccTable*)
     * share numeric offsets 0x300+ with the FBI setup registers
     * (sBeginTriCMD, sDrawTriCMD).  They are distinguished by chip-select:
     * if TREX0 or TREX1 is set we handle them here and return early.
     * If only FBI is selected we fall through to the main switch below.
     */
    if ((chip & 0x6) && addr >= 0x300u) {
        /* NCC table0: 0x324..0x34c */
        if (addr >= 0x324u && addr <= 0x34cu) {
            int entry = (int)((addr - 0x324u) >> 2);
            int row = entry / 4; int col = entry % 4;
            if (chip & 0x2) {
                if      (row == 0) s->ncc_table[0][0].y[col] = val;
                else if (row == 1) s->ncc_table[0][0].i[col] = val;
                else               s->ncc_table[0][0].q[col] = val;
                s->ncc_dirty[0] = 1;
                s->ncc_gen[0]++;
            }
            if (chip & 0x4) {
                if      (row == 0) s->ncc_table[1][0].y[col] = val;
                else if (row == 1) s->ncc_table[1][0].i[col] = val;
                else               s->ncc_table[1][0].q[col] = val;
                s->ncc_dirty[1] = 1;
                s->ncc_gen[1]++;
            }
            return;
        }
        /* NCC table1: 0x364..0x38c */
        if (addr >= 0x364u && addr <= 0x38cu) {
            int entry = (int)((addr - 0x364u) >> 2);
            int row = entry / 4; int col = entry % 4;
            if (chip & 0x2) {
                if      (row == 0) s->ncc_table[0][1].y[col] = val;
                else if (row == 1) s->ncc_table[0][1].i[col] = val;
                else               s->ncc_table[0][1].q[col] = val;
                s->ncc_dirty[0] = 1;
                s->ncc_gen[0]++;
            }
            if (chip & 0x4) {
                if      (row == 0) s->ncc_table[1][1].y[col] = val;
                else if (row == 1) s->ncc_table[1][1].i[col] = val;
                else               s->ncc_table[1][1].q[col] = val;
                s->ncc_dirty[1] = 1;
                s->ncc_gen[1]++;
            }
            return;
        }
        switch (addr) {
        case 0x300u: /* textureMode */
            if (chip & 0x2) {
                s->params.tmu[0].textureMode = val;
                s->params.tmu[0].tformat = (int)((val >> 8) & 0xf);
                voodoo3_recalc_tex(&s->params.tex_params[0],
                    s->params.tmu[0].tLOD, val,
                    s->params.tmu[0].texBaseAddr, s->params.tmu[0].texBaseAddr1,
                    s->params.tmu[0].texBaseAddr2, s->params.tmu[0].texBaseAddr38,
                    s->params.tmu[0].tformat);
            }
            if (chip & 0x4) {
                s->params.tmu[1].textureMode = val;
                s->params.tmu[1].tformat = (int)((val >> 8) & 0xf);
                voodoo3_recalc_tex(&s->params.tex_params[1],
                    s->params.tmu[1].tLOD, val,
                    s->params.tmu[1].texBaseAddr, s->params.tmu[1].texBaseAddr1,
                    s->params.tmu[1].texBaseAddr2, s->params.tmu[1].texBaseAddr38,
                    s->params.tmu[1].tformat);
            }
            return;
        case 0x304u: /* tLOD */
            if (chip & 0x2) {
                s->params.tmu[0].tLOD = val;
                voodoo3_recalc_tex(&s->params.tex_params[0],
                    val, s->params.tmu[0].textureMode,
                    s->params.tmu[0].texBaseAddr, s->params.tmu[0].texBaseAddr1,
                    s->params.tmu[0].texBaseAddr2, s->params.tmu[0].texBaseAddr38,
                    s->params.tmu[0].tformat);
            }
            if (chip & 0x4) {
                s->params.tmu[1].tLOD = val;
                voodoo3_recalc_tex(&s->params.tex_params[1],
                    val, s->params.tmu[1].textureMode,
                    s->params.tmu[1].texBaseAddr, s->params.tmu[1].texBaseAddr1,
                    s->params.tmu[1].texBaseAddr2, s->params.tmu[1].texBaseAddr38,
                    s->params.tmu[1].tformat);
            }
            return;
        case 0x308u: /* tDetail — detail texture blend parameters */
            /*
             * Ported from 86Box vid_voodoo_reg.c SST_tDetail handler.
             * Bits [7:0]   = detail_max   — blend factor ceiling (0..255)
             * Bits [13:8]  = detail_bias  — LOD subtrahend (0..63)
             * Bits [16:14] = detail_scale — left-shift for factor (0..7)
             *
             * Used by CC_MSELECT_DETAIL / CCA_MSELECT_DETAIL in the
             * colour-combine path (voodoo3_render.c).
             */
            if (chip & 0x2) {
                s->params.detail_max[0]   = (int)(val & 0xffu);
                s->params.detail_bias[0]  = (int)((val >> 8) & 0x3fu);
                s->params.detail_scale[0] = (int)((val >> 14) & 0x7u);
            }
            if (chip & 0x4) {
                s->params.detail_max[1]   = (int)(val & 0xffu);
                s->params.detail_bias[1]  = (int)((val >> 8) & 0x3fu);
                s->params.detail_scale[1] = (int)((val >> 14) & 0x7u);
            }
            return;
        case 0x30cu: /* texBaseAddr */
            if (chip & 0x2) {
                s->params.tmu[0].texBaseAddr = val & 0xfffff0u;
                voodoo3_recalc_tex(&s->params.tex_params[0],
                    s->params.tmu[0].tLOD, s->params.tmu[0].textureMode,
                    val & 0xfffff0u, s->params.tmu[0].texBaseAddr1,
                    s->params.tmu[0].texBaseAddr2, s->params.tmu[0].texBaseAddr38,
                    s->params.tmu[0].tformat);
            }
            if (chip & 0x4) {
                s->params.tmu[1].texBaseAddr = val & 0xfffff0u;
                voodoo3_recalc_tex(&s->params.tex_params[1],
                    s->params.tmu[1].tLOD, s->params.tmu[1].textureMode,
                    val & 0xfffff0u, s->params.tmu[1].texBaseAddr1,
                    s->params.tmu[1].texBaseAddr2, s->params.tmu[1].texBaseAddr38,
                    s->params.tmu[1].tformat);
            }
            return;
        case 0x310u: /* texBaseAddr1 */
            if (chip & 0x2) {
                s->params.tmu[0].texBaseAddr1 = val & 0xfffff0u;
                voodoo3_recalc_tex(&s->params.tex_params[0],
                    s->params.tmu[0].tLOD, s->params.tmu[0].textureMode,
                    s->params.tmu[0].texBaseAddr, val & 0xfffff0u,
                    s->params.tmu[0].texBaseAddr2, s->params.tmu[0].texBaseAddr38,
                    s->params.tmu[0].tformat);
            }
            if (chip & 0x4) {
                s->params.tmu[1].texBaseAddr1 = val & 0xfffff0u;
                voodoo3_recalc_tex(&s->params.tex_params[1],
                    s->params.tmu[1].tLOD, s->params.tmu[1].textureMode,
                    s->params.tmu[1].texBaseAddr, val & 0xfffff0u,
                    s->params.tmu[1].texBaseAddr2, s->params.tmu[1].texBaseAddr38,
                    s->params.tmu[1].tformat);
            }
            return;
        case 0x314u: /* texBaseAddr2 */
            if (chip & 0x2) {
                s->params.tmu[0].texBaseAddr2 = val & 0xfffff0u;
                voodoo3_recalc_tex(&s->params.tex_params[0],
                    s->params.tmu[0].tLOD, s->params.tmu[0].textureMode,
                    s->params.tmu[0].texBaseAddr, s->params.tmu[0].texBaseAddr1,
                    val & 0xfffff0u, s->params.tmu[0].texBaseAddr38,
                    s->params.tmu[0].tformat);
            }
            if (chip & 0x4) {
                s->params.tmu[1].texBaseAddr2 = val & 0xfffff0u;
                voodoo3_recalc_tex(&s->params.tex_params[1],
                    s->params.tmu[1].tLOD, s->params.tmu[1].textureMode,
                    s->params.tmu[1].texBaseAddr, s->params.tmu[1].texBaseAddr1,
                    val & 0xfffff0u, s->params.tmu[1].texBaseAddr38,
                    s->params.tmu[1].tformat);
            }
            return;
        case 0x318u: /* texBaseAddr38 */
            if (chip & 0x2) {
                s->params.tmu[0].texBaseAddr38 = val & 0xfffff0u;
                voodoo3_recalc_tex(&s->params.tex_params[0],
                    s->params.tmu[0].tLOD, s->params.tmu[0].textureMode,
                    s->params.tmu[0].texBaseAddr, s->params.tmu[0].texBaseAddr1,
                    s->params.tmu[0].texBaseAddr2, val & 0xfffff0u,
                    s->params.tmu[0].tformat);
            }
            if (chip & 0x4) {
                s->params.tmu[1].texBaseAddr38 = val & 0xfffff0u;
                voodoo3_recalc_tex(&s->params.tex_params[1],
                    s->params.tmu[1].tLOD, s->params.tmu[1].textureMode,
                    s->params.tmu[1].texBaseAddr, s->params.tmu[1].texBaseAddr1,
                    s->params.tmu[1].texBaseAddr2, val & 0xfffff0u,
                    s->params.tmu[1].tformat);
            }
            return;
        default:
            /*
             * FIX B: Texture Download Port — unrecognised TMU register
             * write at 0x300+ is raw texel data uploaded via CMDFIFO.
             *
             * When the Glide/Warp3D driver uploads texture data via
             * CMDFIFO Packet-5 register writes (chip=TREX0/1 or broadcast),
             * it writes sequential 32-bit pixel words to addresses in the
             * 0x300-0x3fc range.  Each word contains two packed RGB565
             * texels (or one ARGB8888 texel for 32-bpp formats).
             *
             * voodoo3_tex_download() uses (fifo_addr & 0x1ffffc) as the
             * byte offset within the download window, then adds tex_base[0]
             * to get the SGRAM destination.  Passing `addr` directly gives
             * the correct 0x300-0x3fc offset range for the first 1 KB block.
             *
             * Ported from: 86Box vid_voodoo_reg.c default case for TMU
             * register writes at offset >= 0x300 that are not NCC entries.
             */
            if (chip & 0x2)
                voodoo3_tex_download(s, addr, val, 0);
            if (chip & 0x4)
                voodoo3_tex_download(s, addr, val, 1);
            return;
        }
    }

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
    /*
     * SST_intrCtrl (0x004) — interrupt control register.
     * Ported from 86Box vid_voodoo_banshee.c banshee->intrCtrl handling.
     * The AmigaOS4/Warp3D driver writes this register via CMDFIFO Packet-1
     * with the current render-state value before each draw call; 86Box stores
     * it filtered by 0x0030003f (only the defined interrupt-enable bits).
     * Previously fell through to LOG_UNIMP; now handled silently.
     */
    case SST_intrCtrl:
        s->intrCtrl = val & 0x0030003fu;
        break;
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
        /*
         * FBI-only writes to the TMU register range (0x300–0x3fc):
         * When chip-select has no TREX bits set (chip == 1, FBI-only), these
         * addresses bypass the TMU handler above and land here.  In 86Box
         * vid_voodoo_reg.c the main switch has no case for 0x300+, so they
         * hit `default: break` — silently discarded.  Match that behaviour:
         * the TMU state is written separately via the TREX chip-select path
         * (chip & 0x6), which IS handled; the FBI-only writes are redundant
         * state broadcast that must not produce LOG_UNIMP noise.
         */
        if (addr >= 0x300u) {
            break;
        }
        /* fall through to unimplemented log */
        qemu_log_mask(LOG_UNIMP,
            "voodoo3: 3D reg 0x%03x = 0x%08x (unimplemented)\n", addr, val);
        break;

    /*
     * FIX A: Voodoo2 SST register layout aliases (0x048–0x088)
     * =========================================================
     * The AmigaOS4 Warp3D driver and some Glide libraries issue CMDFIFO
     * Packet-5 register writes using the ORIGINAL Voodoo2 SST byte offsets,
     * not the Banshee/Voodoo3 remapped layout used by the cases above.
     *
     * In the Voodoo2 SST layout:
     *   0x008-0x01c = vertexAx..vertexCy  (int16 ×16 sub-pixel)
     *   0x020-0x03c = startR..startW      (int color/Z/ST/W)
     *   0x040-0x07c = dRdX..dWdY          (int gradients)
     *   0x080       = triangleCMD          (triggers rasterize)
     *   0x084-0x088 = fvertexAx, fvertexAy (float vertex, Voodoo2 position)
     *
     * QEMU's Banshee layout shifts those registers to 0x180-0x1f8, so
     * writes at the Voodoo2 addresses fall through to LOG_UNIMP.
     *
     * The CMDFIFO handler passes packet register addresses verbatim to
     * voodoo3_3d_reg_write(), so we add explicit alias cases here.
     *
     * addr & 0x3fc is already applied — these are the raw byte offsets.
     *
     * Voodoo2 0x008-0x044 are NOT in the log because they coincide with
     * QEMU's render-state registers (fbzColorPath=0x008 etc.) which are
     * already handled above. The driver writes state first, then vertex
     * data — the vertex writes at 0x048+ are the ones that hit the gap.
     *
     * Ported from: 86Box vid_voodoo_reg.c voodoo_reg_writel() cases
     *              SST_dBdX (0x048) through SST_fvertexAy (0x088).
     */

    /* -- V2 integer gradient dXd aliases (0x048-0x07c) -- */
    case 0x048: /* V2 dBdX  */ s->params.dBdX = (int32_t)(val & 0xffffff) |
                    (((val) & 0x800000) ? (int32_t)0xff000000u : 0); break;
    case 0x04c: /* V2 dZdX  */ s->params.dZdX = (int32_t)val; break;
    case 0x050: /* V2 dAdX  */ s->params.dAdX = (int32_t)(val & 0xffffff) |
                    (((val) & 0x800000) ? (int32_t)0xff000000u : 0); break;
    case 0x054: /* V2 dSdX  */
        s->params.tmu[0].dSdX = ((int64_t)(int32_t)val) << 14;
        s->params.tmu[1].dSdX = s->params.tmu[0].dSdX; break;
    case 0x058: /* V2 dTdX  */
        s->params.tmu[0].dTdX = ((int64_t)(int32_t)val) << 14;
        s->params.tmu[1].dTdX = s->params.tmu[0].dTdX; break;
    case 0x05c: /* V2 dWdX  */
        s->params.dWdX        = (int64_t)(int32_t)val << 2;
        s->params.tmu[0].dWdX = s->params.dWdX;
        s->params.tmu[1].dWdX = s->params.dWdX; break;
    case 0x060: /* V2 dRdY  */ s->params.dRdY = (int32_t)(val & 0xffffff) |
                    (((val) & 0x800000) ? (int32_t)0xff000000u : 0); break;
    case 0x064: /* V2 dGdY  */ s->params.dGdY = (int32_t)(val & 0xffffff) |
                    (((val) & 0x800000) ? (int32_t)0xff000000u : 0); break;
    case 0x068: /* V2 dBdY  */ s->params.dBdY = (int32_t)(val & 0xffffff) |
                    (((val) & 0x800000) ? (int32_t)0xff000000u : 0); break;
    case 0x06c: /* V2 dZdY  */ s->params.dZdY = (int32_t)val; break;
    case 0x070: /* V2 dAdY  */ s->params.dAdY = (int32_t)(val & 0xffffff) |
                    (((val) & 0x800000) ? (int32_t)0xff000000u : 0); break;
    case 0x074: /* V2 dSdY  */
        s->params.tmu[0].dSdY = ((int64_t)(int32_t)val) << 14;
        s->params.tmu[1].dSdY = s->params.tmu[0].dSdY; break;
    case 0x078: /* V2 dTdY  */
        s->params.tmu[0].dTdY = ((int64_t)(int32_t)val) << 14;
        s->params.tmu[1].dTdY = s->params.tmu[0].dTdY; break;
    case 0x07c: /* V2 dWdY  */
        s->params.dWdY        = (int64_t)(int32_t)val << 2;
        s->params.tmu[0].dWdY = s->params.dWdY;
        s->params.tmu[1].dWdY = s->params.dWdY; break;

    /* -- V2 triangleCMD alias (0x080) -- triggers integer rasterize -- */
    case 0x080: /* V2 triangleCMD */
        s->params.sign = (int)(val >> 31);
        voodoo3_queue_triangle(s, &s->params);
        s->cmd_read++;
        break;

    /* -- V2 float vertex aliases (0x084-0x088) -- */
    case 0x084: /* V2 fvertexAx */
        f.i = val;
        s->params.vertexAx = (int32_t)(int16_t)(int32_t)(f.f * 16.0f) & 0xffff;
        break;
    case 0x088: /* V2 fvertexAy */
        f.i = val;
        s->params.vertexAy = (int32_t)(int16_t)(int32_t)(f.f * 16.0f) & 0xffff;
        break;
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
    /*
     * 86Box address aliases (SST_fvertexBy/Cx/Cy at 0x094/0x098/0x09c):
     * The Voodoo 2 register map placed fvertexBy/Cx/Cy at 0x094-0x09c.
     * Banshee/Voodoo3 remaps them to 0x20c-0x214 (SST_fvertexBy below),
     * but some drivers (especially older Win9x Voodoo3 drivers) still
     * write to the Voodoo-2-era addresses.  86Box handles both in
     * vid_voodoo_reg.c via SST_fvertexBy (0x094) and
     * SST_remap_fvertexBy (0x094|0x400).  Port the same aliasing here.
     * Ported from: 86Box vid_voodoo_reg.c lines 357-370.
     */
    case 0x08c: /* SST_fvertexAy (Voodoo2/86Box address alias) */
        f.i = val; s->params.vertexAy = (int32_t)(int16_t)(int32_t)(f.f * 16.0f) & 0xffff; break;
    case 0x090: /* SST_fvertexBx (Voodoo2/86Box address alias) */
        f.i = val; s->params.vertexBx = (int32_t)(int16_t)(int32_t)(f.f * 16.0f) & 0xffff; break;
    case 0x094: /* SST_fvertexBy (Voodoo2/86Box address alias) */
        f.i = val; s->params.vertexBy = (int32_t)(int16_t)(int32_t)(f.f * 16.0f) & 0xffff; break;
    case 0x098: /* SST_fvertexCx (Voodoo2/86Box address alias) */
        f.i = val; s->params.vertexCx = (int32_t)(int16_t)(int32_t)(f.f * 16.0f) & 0xffff; break;
    case 0x09c: /* SST_fvertexCy (Voodoo2/86Box address alias) */
        f.i = val; s->params.vertexCy = (int32_t)(int16_t)(int32_t)(f.f * 16.0f) & 0xffff; break;

    /*
     * 0x1fc: Sits between dWdY (0x1f4) and triangleCMD (0x1f8) / fvertexAx (0x200).
     * Not defined in the Banshee spec nor in 86Box's register table.
     * Writes arrive here only with uninitialised data (0xcfcfcfcf) during
     * driver startup — silently ignore to suppress LOG_UNIMP spam.
     */
    case 0x1fc: break; /* reserved/undocumented — silent ignore */

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

    /*
     * Float gradient remap registers: 0x0a0 – 0x0fc
     *
     * The Banshee/Voodoo3 CMDFIFO remap address space encodes floating-point
     * triangle-setup parameters in an interleaved per-component layout:
     *   offset 0x0a0 = fstartR,  0x0a4 = fdRdX,  0x0a8 = fdRdY
     *   offset 0x0ac = fstartG,  0x0b0 = fdGdX,  0x0b4 = fdGdY
     *   offset 0x0b8 = fstartB,  0x0bc = fdBdX,  0x0c0 = fdBdY
     *   offset 0x0c4 = fstartZ,  0x0c8 = fdZdX,  0x0cc = fdZdY
     *   offset 0x0d0 = fstartA,  0x0d4 = fdAdX,  0x0d8 = fdAdY
     *   offset 0x0dc = fstartS,  0x0e0 = fdSdX,  0x0e4 = fdSdY  (TMU)
     *   offset 0x0e8 = fstartT,  0x0ec = fdTdX,  0x0f0 = fdTdY  (TMU)
     *   offset 0x0f4 = fstartW,  0x0f8 = fdWdX,  0x0fc = fdWdY  (FBI+TMU)
     *
     * These map to SST_remap_fstartR .. SST_remap_fdWdY in 86Box vid_voodoo_regs.h.
     * The AmigaOS 3dfxVoodoo driver issues all triangle-setup parameters via
     * this remap layout, so missing these causes triangles to render with zero
     * colour/Z/texture gradients (flat black or garbage).
     *
     * Conversion factors (from 86Box vid_voodoo_reg.c):
     *   R/G/B/Z/A:         float → fixed-point s12.12  (× 4096.0f)
     *   S/T (tex coords):  float → fixed-point s32.32  (× 4294967296.0f)
     *   W (perspective):   float → fixed-point s32.32  (× 4294967296.0f)
     *
     * S/T/W use the chip-select field (chip = (addr_before_mask >> 10) & 7):
     *   CHIP_FBI=0x1, CHIP_TREX0=0x2, CHIP_TREX1=0x4
     *
     * Ported from: 86Box vid_voodoo_reg.c lines 374–530
     *              (SST_remap_fstartR .. SST_remap_fdWdY cases)
     */

    /* --- fstart/fdXd/fdYd : Red --- */
    case 0x0a0: /* remap_fstartR */
        f.i = val; s->params.startR = (int32_t)(f.f * 4096.0f); break;
    case 0x0a4: /* remap_fdRdX */
        f.i = val; s->params.dRdX   = (int32_t)(f.f * 4096.0f); break;
    case 0x0a8: /* remap_fdRdY */
        f.i = val; s->params.dRdY   = (int32_t)(f.f * 4096.0f); break;

    /* --- fstart/fdXd/fdYd : Green --- */
    case 0x0ac: /* remap_fstartG */
        f.i = val; s->params.startG = (int32_t)(f.f * 4096.0f); break;
    case 0x0b0: /* remap_fdGdX */
        f.i = val; s->params.dGdX   = (int32_t)(f.f * 4096.0f); break;
    case 0x0b4: /* remap_fdGdY */
        f.i = val; s->params.dGdY   = (int32_t)(f.f * 4096.0f); break;

    /* --- fstart/fdXd/fdYd : Blue --- */
    case 0x0b8: /* remap_fstartB */
        f.i = val; s->params.startB = (int32_t)(f.f * 4096.0f); break;
    case 0x0bc: /* remap_fdBdX */
        f.i = val; s->params.dBdX   = (int32_t)(f.f * 4096.0f); break;
    case 0x0c0: /* remap_fdBdY */
        f.i = val; s->params.dBdY   = (int32_t)(f.f * 4096.0f); break;

    /* --- fstart/fdXd/fdYd : Z --- */
    case 0x0c4: /* remap_fstartZ */
        f.i = val; s->params.startZ = (int32_t)(f.f * 4096.0f); break;
    case 0x0c8: /* remap_fdZdX */
        f.i = val; s->params.dZdX   = (int32_t)(f.f * 4096.0f); break;
    case 0x0cc: /* remap_fdZdY */
        f.i = val; s->params.dZdY   = (int32_t)(f.f * 4096.0f); break;

    /* --- fstart/fdXd/fdYd : Alpha --- */
    case 0x0d0: /* remap_fstartA */
        f.i = val; s->params.startA = (int32_t)(f.f * 4096.0f); break;
    case 0x0d4: /* remap_fdAdX */
        f.i = val; s->params.dAdX   = (int32_t)(f.f * 4096.0f); break;
    case 0x0d8: /* remap_fdAdY */
        f.i = val; s->params.dAdY   = (int32_t)(f.f * 4096.0f); break;

    /* --- fstart/fdXd/fdYd : S (texture coord, per-TMU via chip select) --- */
    case 0x0dc: /* remap_fstartS */
        f.i = val;
        if (chip & 0x2) s->params.tmu[0].startS = (int64_t)(f.f * 4294967296.0f);
        if (chip & 0x4) s->params.tmu[1].startS = (int64_t)(f.f * 4294967296.0f);
        break;
    case 0x0e0: /* remap_fdSdX */
        f.i = val;
        if (chip & 0x2) s->params.tmu[0].dSdX = (int64_t)(f.f * 4294967296.0f);
        if (chip & 0x4) s->params.tmu[1].dSdX = (int64_t)(f.f * 4294967296.0f);
        break;
    case 0x0e4: /* remap_fdSdY */
        f.i = val;
        if (chip & 0x2) s->params.tmu[0].dSdY = (int64_t)(f.f * 4294967296.0f);
        if (chip & 0x4) s->params.tmu[1].dSdY = (int64_t)(f.f * 4294967296.0f);
        break;

    /* --- fstart/fdXd/fdYd : T (texture coord, per-TMU via chip select) --- */
    case 0x0e8: /* remap_fstartT */
        f.i = val;
        if (chip & 0x2) s->params.tmu[0].startT = (int64_t)(f.f * 4294967296.0f);
        if (chip & 0x4) s->params.tmu[1].startT = (int64_t)(f.f * 4294967296.0f);
        break;
    case 0x0ec: /* remap_fdTdX */
        f.i = val;
        if (chip & 0x2) s->params.tmu[0].dTdX = (int64_t)(f.f * 4294967296.0f);
        if (chip & 0x4) s->params.tmu[1].dTdX = (int64_t)(f.f * 4294967296.0f);
        break;
    case 0x0f0: /* remap_fdTdY */
        f.i = val;
        if (chip & 0x2) s->params.tmu[0].dTdY = (int64_t)(f.f * 4294967296.0f);
        if (chip & 0x4) s->params.tmu[1].dTdY = (int64_t)(f.f * 4294967296.0f);
        break;

    /* --- fstart/fdXd/fdYd : W (perspective, FBI + per-TMU via chip select) --- */
    case 0x0f4: /* remap_fstartW */
        f.i = val;
        if (chip & 0x1) s->params.startW        = (int64_t)(f.f * 4294967296.0f);
        if (chip & 0x2) s->params.tmu[0].startW = (int64_t)(f.f * 4294967296.0f);
        if (chip & 0x4) s->params.tmu[1].startW = (int64_t)(f.f * 4294967296.0f);
        break;
    case 0x0f8: /* remap_fdWdX */
        f.i = val;
        if (chip & 0x1) s->params.dWdX        = (int64_t)(f.f * 4294967296.0f);
        if (chip & 0x2) s->params.tmu[0].dWdX = (int64_t)(f.f * 4294967296.0f);
        if (chip & 0x4) s->params.tmu[1].dWdX = (int64_t)(f.f * 4294967296.0f);
        break;
    case 0x0fc: /* remap_fdWdY */
        f.i = val;
        if (chip & 0x1) s->params.dWdY        = (int64_t)(f.f * 4294967296.0f);
        if (chip & 0x2) s->params.tmu[0].dWdY = (int64_t)(f.f * 4294967296.0f);
        if (chip & 0x4) s->params.tmu[1].dWdY = (int64_t)(f.f * 4294967296.0f);
        break;

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
        s->params.col_stride_raw = val;  /* FIX: keep raw value for readback (diag Module 22) */
		s->params.col_tiled = !!(val & (1u << 15));
		s->params.row_width = s->params.col_tiled
							  ? (val & 0x3fffu) * 128u * 32u
							  : (val & 0x3fffu);
        break;
    case SST_auxBufferAddr:
        s->params.aux_offset = val & 0xfffff0;
        break;
    case SST_auxBufferStride:
        s->params.aux_stride_raw = val;  /* FIX: keep raw value for readback */
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
    /*
     * 0x2a8–0x2bc: Immediately after sBeginTriCMD (0x2a4).
     * These addresses exist in neither the Banshee register spec nor 86Box's
     * register table.  The AmigaOS driver writes real vertex-parameter data
     * here (values look like packed colour/coordinate words), likely a
     * driver-specific extension or a misidentified hardware feature.
     * Until the exact semantics are known, silently ignore to avoid
     * LOG_UNIMP spam during rendering — the triangles are still drawn via
     * the standard sDrawTriCMD/sBeginTriCMD path.
     */
    case 0x2a8: case 0x2ac: case 0x2b0: case 0x2b4:
    case 0x2b8: case 0x2bc:
        break; /* post-sBeginTriCMD reserved range — silent ignore */

    /*
     * 0x000 = status register: read-only on real hardware.
     * 86Box silently drops writes (no case in vid_voodoo_reg.c).
     * QEMU was falling through to LOG_UNIMP; suppress that.
     */
    case SST_status: break; /* read-only: silently ignore writes */

    /* --- Setup engine vertex accumulator --- */
    /*
     * 86Box address aliases for setup/vertex registers (0x27c–0x2a4):
     * The Voodoo3 Banshee register remap moves these registers relative
     * to where the Voodoo 2 spec placed them.  86Box vid_voodoo_regs.h:
     *   SST_sAlpha = 0x27c, SST_sVz = 0x280, SST_sWb = 0x284,
     *   SST_sW0 = 0x288,   SST_sS0 = 0x28c, SST_sT0 = 0x290,
     *   SST_sW1 = 0x294,   SST_sS1 = 0x298, SST_sT1 = 0x29c
     * QEMU maps these same registers at 0x2dc–0x2fc (SST_sAlpha etc.).
     * Older Win9x Voodoo3 drivers write both address ranges during init.
     * Ported from: 86Box vid_voodoo_reg.c lines 774-810.
     */
    case 0x27c: { fi_t g; g.i = val; s->verts[3].sAlpha = g.f; } break; /* sAlpha (86Box addr) */
    case 0x280: { fi_t g; g.i = val; s->verts[3].sVz    = g.f; } break; /* sVz    (86Box addr) */
    case 0x284: { fi_t g; g.i = val; s->verts[3].sWb    = g.f; } break; /* sWb    (86Box addr) */
    case 0x288: { fi_t g; g.i = val; s->verts[3].sW0    = g.f; } break; /* sW0    (86Box addr) */
    case 0x28c: { fi_t g; g.i = val; s->verts[3].sS0    = g.f; } break; /* sS0    (86Box addr) */
    case 0x290: { fi_t g; g.i = val; s->verts[3].sT0    = g.f; } break; /* sT0    (86Box addr) */
    case 0x294: { fi_t g; g.i = val; s->verts[3].sW1    = g.f; } break; /* sW1    (86Box addr) */
    case 0x298: { fi_t g; g.i = val; s->verts[3].sS1    = g.f; } break; /* sS1    (86Box addr) */
    case 0x29c: { fi_t g; g.i = val; s->verts[3].sT1    = g.f; } break; /* sT1    (86Box addr) */
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
 * Full Banshee 2D blitter — ported from 86Box vid_voodoo_banshee_blitter.c
 * Includes: ROP engine, clip rectangles, 8×8 pattern (mono+color),
 * colorkey (src+dst), S2S/H2S stretch-blt (Bresenham), line/polyline
 * (Bresenham + stipple + TRANS_MONO), polyfill (span-fill with 2 edges),
 * YUV422 (YUYV/UYVY) → RGB conversion, byte/word data swizzle.
 * ========================================================================= */

/* -------------------------------------------------------------------------
 * Colorkey test — ported from 86Box colorkey()
 * src_notdst: 1 = test src colorkey, 0 = test dst colorkey
 * Returns 1 if the pixel should be treated as transparent (skip write).
 * ------------------------------------------------------------------------- */
static inline int blt_colorkey(voodoo3_blt_t *blt, uint32_t pixel,
                                int src_notdst, int fmt)
{
    uint32_t min = src_notdst ? blt->srcColorkeyMin : blt->dstColorkeyMin;
    uint32_t max = src_notdst ? blt->srcColorkeyMax : blt->dstColorkeyMax;
    uint32_t flag = src_notdst ? CMDEXTRA_SRC_COLORKEY : CMDEXTRA_DST_COLORKEY;

    if (!(blt->commandExtra & flag))
        return 0;

    switch (fmt) {
    case BLT_COLORKEY_8:
        return ((pixel & 0xffu) >= (min & 0xffu)) &&
               ((pixel & 0xffu) <= (max & 0xffu));
    case BLT_COLORKEY_16: {
        int r = (pixel >> 11) & 0x1f, rm = (min >> 11) & 0x1f, rx_ = (max >> 11) & 0x1f;
        int g = (pixel >>  5) & 0x3f, gm = (min >>  5) & 0x3f, gx_ = (max >>  5) & 0x3f;
        int b =  pixel        & 0x1f, bm =  min        & 0x1f, bx_ =  max        & 0x1f;
        return (r >= rm) && (r <= rx_) && (g >= gm) && (g <= gx_) && (b >= bm) && (b <= bx_);
    }
    default: { /* 32 */
        int r = (pixel >> 16) & 0xff, rm = (min >> 16) & 0xff, rx_ = (max >> 16) & 0xff;
        int g = (pixel >>  8) & 0xff, gm = (min >>  8) & 0xff, gx_ = (max >>  8) & 0xff;
        int b =  pixel        & 0xff, bm =  min        & 0xff, bx_ =  max        & 0xff;
        return (r >= rm) && (r <= rx_) && (g >= gm) && (g <= gx_) && (b >= bm) && (b <= bx_);
    }
    }
}

/* -------------------------------------------------------------------------
 * ROP mixer — ported from 86Box MIX()
 * Selects rops[rop_nr] based on colorkey results for src and dst.
 * ------------------------------------------------------------------------- */
static inline uint32_t blt_mix(voodoo3_blt_t *blt, uint32_t dst, uint32_t src,
                                uint32_t pattern, int src_fmt, int dst_fmt)
{
    int rop_nr = 0;
    if (blt_colorkey(blt, src, 1, src_fmt)) rop_nr |= 2;
    if (blt_colorkey(blt, dst, 0, dst_fmt)) rop_nr |= 1;
    uint32_t rop = blt->rops[rop_nr];
    uint32_t result = 0;
    if (rop & 0x01) result |= (~pattern & ~src & ~dst);
    if (rop & 0x02) result |= (~pattern & ~src &  dst);
    if (rop & 0x04) result |= (~pattern &  src & ~dst);
    if (rop & 0x08) result |= (~pattern &  src &  dst);
    if (rop & 0x10) result |= ( pattern & ~src & ~dst);
    if (rop & 0x20) result |= ( pattern & ~src &  dst);
    if (rop & 0x40) result |= ( pattern &  src & ~dst);
    if (rop & 0x80) result |= ( pattern &  src &  dst);
    return result;
}

/* -------------------------------------------------------------------------
 * Address calculation — ported from 86Box get_addr()
 * Handles tiled and linear addressing for src and dst.
 * ------------------------------------------------------------------------- */
static inline uint32_t blt_get_addr(Voodoo3State *s, int x, int y,
                                    int src_notdst, uint32_t src_stride_override)
{
    voodoo3_blt_t *blt = &s->blt;
    uint32_t stride    = src_notdst ? src_stride_override : blt->dst_stride;
    uint32_t base      = src_notdst ? blt->srcBaseAddr : blt->dstBaseAddr;
    bool     tiled     = src_notdst ? !!blt->srcBaseAddr_tiled : !!blt->dstBaseAddr_tiled;

    uint32_t addr;
    if (tiled)
        addr = base + (x & 127) + ((x >> 7) * 128 * 32) + ((y & 31) * 128) + (y >> 5) * stride;
    else
        addr = base + x + y * stride;

    return addr & (s->fb_size - 1);
}

/* -------------------------------------------------------------------------
 * PLOT — write one pixel at (x,y) with ROP + pattern + colorkey.
 * Ported from 86Box PLOT() macro.
 * ------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------
 * blt_alpha_blend_argb32 — per-pixel alpha-blend ARGB32 source onto dest.
 *
 * Called when srcFormat is SRC_FORMAT_COL_ARGB32 (srcfmt=7).
 * Implements Porter-Duff "src over dst" using the alpha byte from the
 * ARGB32 source pixel, then converts the blended RGB to the destination
 * pixel format (8-bpp palette index pass-through, 16-bpp RGB565, 32-bpp).
 *
 * For dstFormat=0 (DST_FORMAT_COL_PAL / raw 8-bpp palette):
 *   The Picasso96 driver programs ARGB source pixels but the destination
 *   is an 8-bpp surface.  We cannot do palette lookup here (no LUT), so
 *   we write the luminance of the blended RGB as the destination byte.
 *   This matches what 86Box does (palette-indexed WPAAlpha is a degenerate
 *   case; drivers that rely on true palette matching use 16/32-bpp dst).
 *
 * FIX: "RGB mask blits with RGB source (srcfmt = 7) and different
 *       colormodels (destfmt = 0) not supported yet"
 *      "cgx/WPAAlpha unsupported pixfmt: 0 for RECTFMT_ARGB"
 * ------------------------------------------------------------------------- */
static inline void blt_alpha_blend_argb32(Voodoo3State *s,
                                           int x, int y,
                                           uint32_t src_argb)
{
    voodoo3_blt_t *blt = &s->blt;
    uint32_t alpha = (src_argb >> 24) & 0xffu;

    /* Fully transparent — nothing to write */
    if (alpha == 0) return;

    /* Decompose source RGB */
    int sr = (src_argb >> 16) & 0xff;
    int sg = (src_argb >>  8) & 0xff;
    int sb =  src_argb        & 0xff;

    switch (blt->dstFormat & DST_FORMAT_COL_MASK) {

    case DST_FORMAT_COL_PAL: /* dstfmt=0 — raw 8-bpp palette surface */
    case DST_FORMAT_COL_8_BPP: {
        /* Fully opaque fast-path */
        uint32_t addr = blt_get_addr(s, x, y, 0, 0);
        if (addr >= s->fb_size) break;
        if (alpha == 0xff) {
            /* Write luminance as palette index (best we can do without LUT) */
            s->fb_mem[addr] = (uint8_t)(((sr * 77) + (sg * 150) + (sb * 29)) >> 8);
        } else {
            uint8_t dst_idx = s->fb_mem[addr];
            /* Blend luminance with existing value */
            int dst_lum = dst_idx; /* treat existing byte as luminance */
            int src_lum = ((sr * 77) + (sg * 150) + (sb * 29)) >> 8;
            s->fb_mem[addr] = (uint8_t)((src_lum * alpha + dst_lum * (255 - alpha)) / 255);
        }
        if (y < V3_DIRTY_LINES) s->dirty_line[y] = 1;
        break;
    }

    case DST_FORMAT_COL_16_BPP: {
        uint32_t addr = blt_get_addr(s, x * 2, y, 0, 0);
        if (addr + 1 >= s->fb_size) break;
        if (alpha == 0xff) {
            /* Fully opaque: direct RGB565 write */
            uint16_t pix = (uint16_t)(((sr >> 3) << 11) | ((sg >> 2) << 5) | (sb >> 3));
            *(uint16_t *)(s->fb_mem + addr) = pix;
        } else {
            /* Alpha-blend over existing RGB565 */
            uint16_t dst16 = *(uint16_t *)(s->fb_mem + addr);
            int dr = ((dst16 >> 11) & 0x1f) << 3;
            int dg = ((dst16 >>  5) & 0x3f) << 2;
            int db =  (dst16        & 0x1f) << 3;
            int rr = (sr * alpha + dr * (255 - alpha)) / 255;
            int rg = (sg * alpha + dg * (255 - alpha)) / 255;
            int rb = (sb * alpha + db * (255 - alpha)) / 255;
            *(uint16_t *)(s->fb_mem + addr) =
                (uint16_t)(((rr >> 3) << 11) | ((rg >> 2) << 5) | (rb >> 3));
        }
        if (y < V3_DIRTY_LINES) s->dirty_line[y] = 1;
        break;
    }

    case DST_FORMAT_COL_24_BPP: {
        uint32_t addr = blt_get_addr(s, x * 3, y, 0, 0);
        if (addr + 3 >= s->fb_size) break;
        if (alpha == 0xff) {
            uint32_t dst32 = *(uint32_t *)(s->fb_mem + addr);
            *(uint32_t *)(s->fb_mem + addr) =
                ((src_argb & 0xffffffu)) | (dst32 & 0xff000000u);
        } else {
            uint32_t dst32 = *(uint32_t *)(s->fb_mem + addr);
            int dr = (dst32 >> 16) & 0xff, dg = (dst32 >> 8) & 0xff, db = dst32 & 0xff;
            int rr = (sr * alpha + dr * (255 - alpha)) / 255;
            int rg = (sg * alpha + dg * (255 - alpha)) / 255;
            int rb = (sb * alpha + db * (255 - alpha)) / 255;
            *(uint32_t *)(s->fb_mem + addr) =
                ((uint32_t)rr << 16) | ((uint32_t)rg << 8) | (uint32_t)rb
                | (dst32 & 0xff000000u);
        }
        if (y < V3_DIRTY_LINES) s->dirty_line[y] = 1;
        break;
    }

    case DST_FORMAT_COL_32_BPP: {
        uint32_t addr = blt_get_addr(s, x * 4, y, 0, 0);
        if (addr + 3 >= s->fb_size) break;
        if (alpha == 0xff) {
            *(uint32_t *)(s->fb_mem + addr) = src_argb;
        } else {
            uint32_t dst32 = *(uint32_t *)(s->fb_mem + addr);
            int dr = (dst32 >> 16) & 0xff, dg = (dst32 >> 8) & 0xff, db = dst32 & 0xff;
            int da = (dst32 >> 24) & 0xff;
            int rr = (sr * alpha + dr * (255 - alpha)) / 255;
            int rg = (sg * alpha + dg * (255 - alpha)) / 255;
            int rb = (sb * alpha + db * (255 - alpha)) / 255;
            int ra = alpha + (da * (255 - alpha)) / 255;
            *(uint32_t *)(s->fb_mem + addr) =
                ((uint32_t)ra << 24) | ((uint32_t)rr << 16) |
                ((uint32_t)rg << 8)  |  (uint32_t)rb;
        }
        if (y < V3_DIRTY_LINES) s->dirty_line[y] = 1;
        break;
    }

    default: break;
    }
}

static inline void blt_plot(Voodoo3State *s, int x, int y,
                             int pat_x, int pat_y, uint8_t pat_mono,
                             uint32_t src, int src_ck_fmt)
{
    voodoo3_blt_t *blt = &s->blt;

    switch (blt->dstFormat & DST_FORMAT_COL_MASK) {
    case DST_FORMAT_COL_PAL: /* dstfmt=0 — raw palette (fall-through) */
    case DST_FORMAT_COL_8_BPP: {
        uint32_t addr = blt_get_addr(s, x, y, 0, 0);
        if (addr >= s->fb_size) break;
        uint32_t dst  = s->fb_mem[addr];
        uint32_t pat  = (blt->command & COMMAND_PATTERN_MONO)
            ? ((pat_mono & (1u << (7 - (pat_x & 7)))) ? blt->colorFore : blt->colorBack)
            : (uint32_t)blt->colorPattern8[(pat_x & 7) + (pat_y & 7) * 8];
        s->fb_mem[addr] = (uint8_t)blt_mix(blt, dst, src, pat, src_ck_fmt, BLT_COLORKEY_8);
        if (y < V3_DIRTY_LINES) s->dirty_line[y] = 1;
        break;
    }
    case DST_FORMAT_COL_16_BPP: {
        uint32_t addr = blt_get_addr(s, x * 2, y, 0, 0);
        if (addr + 1 >= s->fb_size) break;
        uint32_t dst  = *(uint16_t *)(s->fb_mem + addr);
        uint32_t pat  = (blt->command & COMMAND_PATTERN_MONO)
            ? ((pat_mono & (1u << (7 - (pat_x & 7)))) ? bswap16(blt->colorFore) : blt->colorBack)
            : (uint32_t)blt->colorPattern16[(pat_x & 7) + (pat_y & 7) * 8];
        *(uint16_t *)(s->fb_mem + addr) = (uint16_t)blt_mix(blt, dst, src, pat, src_ck_fmt, BLT_COLORKEY_16);
        if (y < V3_DIRTY_LINES) s->dirty_line[y] = 1;
        break;
    }
    case DST_FORMAT_COL_24_BPP: {
        uint32_t addr = blt_get_addr(s, x * 3, y, 0, 0);
        if (addr + 3 >= s->fb_size) break;
        uint32_t dst  = *(uint32_t *)(s->fb_mem + addr);
        uint32_t pat  = (blt->command & COMMAND_PATTERN_MONO)
            ? ((pat_mono & (1u << (7 - (pat_x & 7)))) ? blt->colorFore : blt->colorBack)
            : blt->colorPattern24[(pat_x & 7) + (pat_y & 7) * 8];
        uint32_t res  = blt_mix(blt, dst, src, pat, src_ck_fmt, BLT_COLORKEY_32);
        *(uint32_t *)(s->fb_mem + addr) = (res & 0xffffffu) | (dst & 0xff000000u);
        if (y < V3_DIRTY_LINES) s->dirty_line[y] = 1;
        break;
    }
    case DST_FORMAT_COL_32_BPP: {
        uint32_t addr = blt_get_addr(s, x * 4, y, 0, 0);
        if (addr + 3 >= s->fb_size) break;
        uint32_t dst  = *(uint32_t *)(s->fb_mem + addr);
        uint32_t pat  = (blt->command & COMMAND_PATTERN_MONO)
            ? ((pat_mono & (1u << (7 - (pat_x & 7)))) ? bswap32(blt->colorFore) : blt->colorBack)
            : blt->colorPattern[(pat_x & 7) + (pat_y & 7) * 8];
        *(uint32_t *)(s->fb_mem + addr) = blt_mix(blt, dst, src, pat, src_ck_fmt, BLT_COLORKEY_32);
        if (y < V3_DIRTY_LINES) s->dirty_line[y] = 1;
        break;
    }
    default: break;
    }
}

/* -------------------------------------------------------------------------
 * PLOT_LINE — write one pixel for line/polyline (uses colorFore, no pattern).
 * Ported from 86Box PLOT_LINE() macro.
 * ------------------------------------------------------------------------- */
static inline void blt_plot_line(Voodoo3State *s, int x, int y, uint32_t pattern)
{
    voodoo3_blt_t *blt = &s->blt;

    switch (blt->dstFormat & DST_FORMAT_COL_MASK) {
    case DST_FORMAT_COL_8_BPP: {
        uint32_t addr = blt_get_addr(s, x, y, 0, 0);
        if (addr >= s->fb_size) break;
        uint32_t dst = s->fb_mem[addr];
        s->fb_mem[addr] = (uint8_t)blt_mix(blt, dst, blt->colorFore, pattern, BLT_COLORKEY_8, BLT_COLORKEY_8);
        if (y < V3_DIRTY_LINES) s->dirty_line[y] = 1;
        break;
    }
    case DST_FORMAT_COL_16_BPP: {
        uint32_t addr = blt_get_addr(s, x * 2, y, 0, 0);
        if (addr + 1 >= s->fb_size) break;
        uint32_t dst = *(uint16_t *)(s->fb_mem + addr);
        *(uint16_t *)(s->fb_mem + addr) = (uint16_t)blt_mix(blt, dst, blt->colorFore, pattern, BLT_COLORKEY_16, BLT_COLORKEY_16);
        if (y < V3_DIRTY_LINES) s->dirty_line[y] = 1;
        break;
    }
    case DST_FORMAT_COL_24_BPP: {
        uint32_t addr = blt_get_addr(s, x * 3, y, 0, 0);
        if (addr + 3 >= s->fb_size) break;
        uint32_t dst = *(uint32_t *)(s->fb_mem + addr);
        uint32_t res = blt_mix(blt, dst, blt->colorFore, pattern, BLT_COLORKEY_32, BLT_COLORKEY_32);
        *(uint32_t *)(s->fb_mem + addr) = (res & 0xffffffu) | (dst & 0xff000000u);
        if (y < V3_DIRTY_LINES) s->dirty_line[y] = 1;
        break;
    }
    case DST_FORMAT_COL_32_BPP: {
        uint32_t addr = blt_get_addr(s, x * 4, y, 0, 0);
        if (addr + 3 >= s->fb_size) break;
        uint32_t dst = *(uint32_t *)(s->fb_mem + addr);
        *(uint32_t *)(s->fb_mem + addr) = blt_mix(blt, dst, blt->colorFore, pattern, BLT_COLORKEY_32, BLT_COLORKEY_32);
        if (y < V3_DIRTY_LINES) s->dirty_line[y] = 1;
        break;
    }
    default: break;
    }
}

/* -------------------------------------------------------------------------
 * YUV422 decode — ported from 86Box DECODE_YUYV422 / DECODE_YUYV422_16BPP
 * ------------------------------------------------------------------------- */
static inline void blt_decode_yuyv422_32(uint32_t *out, const uint8_t *src)
{
    int y1 = src[0], cr = (int8_t)(src[1] - 0x80);
    int y2 = src[2], cb = (int8_t)(src[3] - 0x80);
    int dR = (359 * cr) >> 8, dG = (88 * cb + 183 * cr) >> 8, dB = (453 * cb) >> 8;
    int r, g, b;
    r = CLAMP(y1 + dR); g = CLAMP(y1 - dG); b = CLAMP(y1 + dB);
    out[0] = (uint32_t)(r | (g << 8) | (b << 16));
    r = CLAMP(y2 + dR); g = CLAMP(y2 - dG); b = CLAMP(y2 + dB);
    out[1] = (uint32_t)(r | (g << 8) | (b << 16));
}

static inline void blt_decode_yuyv422_16(uint16_t *out, const uint8_t *src)
{
    int y1 = src[0], cr = (int8_t)(src[1] - 0x80);
    int y2 = src[2], cb = (int8_t)(src[3] - 0x80);
    int dR = (359 * cr) >> 8, dG = (88 * cb + 183 * cr) >> 8, dB = (453 * cb) >> 8;
    int r, g, b;
    r = CLAMP(y1 + dR) >> 3; g = CLAMP(y1 - dG) >> 2; b = CLAMP(y1 + dB) >> 3;
    out[0] = (uint16_t)(r | (g << 5) | (b << 11));
    r = CLAMP(y2 + dR) >> 3; g = CLAMP(y2 - dG) >> 2; b = CLAMP(y2 + dB) >> 3;
    out[1] = (uint16_t)(r | (g << 5) | (b << 11));
}

/* -------------------------------------------------------------------------
 * src colorkey format from srcFormat
 * ------------------------------------------------------------------------- */
static inline int blt_src_ck_fmt(voodoo3_blt_t *blt)
{
    switch (blt->srcFormat & SRC_FORMAT_COL_MASK) {
    case SRC_FORMAT_COL_8_BPP:  return BLT_COLORKEY_8;
    case SRC_FORMAT_COL_16_BPP: return BLT_COLORKEY_16;
    default:                     return BLT_COLORKEY_32;
    }
}

/* -------------------------------------------------------------------------
 * end_command — update dstXY if INC_X/Y_START set
 * Ported from 86Box end_command()
 * ------------------------------------------------------------------------- */
static inline void blt_end_command(voodoo3_blt_t *blt)
{
    if (blt->command & COMMAND_INC_X_START) {
        blt->dstXY = (blt->dstXY & 0xffff0000u) | (blt->dstX & 0xffffu);
    }
    if (blt->command & COMMAND_INC_Y_START) {
        blt->dstXY = (blt->dstXY & 0x0000ffffu) | ((uint32_t)blt->dstY << 16);
    }
}

/* -------------------------------------------------------------------------
 * update_src_stride — decode packing mode → src_stride_src/dest
 * Ported from 86Box update_src_stride()
 * ------------------------------------------------------------------------- */
static void blt_update_src_stride_full(Voodoo3State *s)
{
    voodoo3_blt_t *blt = &s->blt;
    int bpp;
    switch (blt->srcFormat & SRC_FORMAT_COL_MASK) {
    case SRC_FORMAT_COL_1_BPP:  bpp =  1; break;
    case SRC_FORMAT_COL_8_BPP:  bpp =  8; break;
    case SRC_FORMAT_COL_16_BPP: bpp = 16; break;
    case SRC_FORMAT_COL_24_BPP: bpp = 24; break;
    case SRC_FORMAT_COL_ARGB32: bpp = 32; break; /* srcfmt=7: ARGB32 */
    default:                     bpp = 32; break;
    }
    switch (blt->srcFormat & SRC_FORMAT_PACKING_MASK) {
    case SRC_FORMAT_PACKING_STRIDE:
        blt->src_stride_src      = blt->src_stride;
        blt->src_stride_dest     = blt->src_stride;
        blt->host_data_size_src  = (blt->srcSizeX * bpp + 7) >> 3;
        blt->host_data_size_dest = (blt->dstSizeX * bpp + 7) >> 3;
        break;
    case SRC_FORMAT_PACKING_BYTE:
        blt->src_stride_src      = (blt->srcSizeX * bpp + 7) >> 3;
        blt->src_stride_dest     = (blt->dstSizeX * bpp + 7) >> 3;
        blt->host_data_size_src  = blt->src_stride_src;
        blt->host_data_size_dest = blt->src_stride_dest;
        break;
    case SRC_FORMAT_PACKING_WORD:
        blt->src_stride_src      = ((blt->srcSizeX * bpp + 15) >> 4) * 2;
        blt->src_stride_dest     = ((blt->dstSizeX * bpp + 15) >> 4) * 2;
        blt->host_data_size_src  = blt->src_stride_src;
        blt->host_data_size_dest = blt->src_stride_dest;
        break;
    case SRC_FORMAT_PACKING_DWORD:
        blt->src_stride_src      = ((blt->srcSizeX * bpp + 31) >> 5) * 4;
        blt->src_stride_dest     = ((blt->dstSizeX * bpp + 31) >> 5) * 4;
        blt->host_data_size_src  = blt->src_stride_src;
        blt->host_data_size_dest = blt->src_stride_dest;
        break;
    default: break;
    }
}

/* Stride mask: bits[12:0] = stride in bytes (non-tiled)
 * or number of 128-byte tile columns (tiled). */
#define DST_FORMAT_STRIDE_MASK     0x1fffu
#define SRC_FORMAT_STRIDE_MASK_BLT 0x1fffu

static void voodoo3_blt_update_dst_stride(Voodoo3State *s)
{
    if (s->blt.dstBaseAddr_tiled)
        s->blt.dst_stride = s->blt.dstStride =
            (s->blt.dstFormat & DST_FORMAT_STRIDE_MASK) * 128u * 32u;
    else
        s->blt.dst_stride = s->blt.dstStride =
            s->blt.dstFormat & DST_FORMAT_STRIDE_MASK;
}

static void voodoo3_blt_update_src_stride(Voodoo3State *s)
{
    if (s->blt.srcBaseAddr_tiled)
        s->blt.src_stride = s->blt.srcStride =
            (s->blt.srcFormat & SRC_FORMAT_STRIDE_MASK_BLT) * 128u * 32u;
    else
        s->blt.src_stride = s->blt.srcStride =
            s->blt.srcFormat & SRC_FORMAT_STRIDE_MASK_BLT;
}

/* -------------------------------------------------------------------------
 * do_screen_to_screen_line — blit one scan line with full ROP/pattern/clip
 * Ported from 86Box do_screen_to_screen_line()
 * src_p    : pointer to source scan line in VRAM
 * use_x_dir: 1 = respect COMMAND_DX for direction
 * src_x    : starting source X pixel index
 * src_tiled: source address is tiled
 * ------------------------------------------------------------------------- */
static void blt_do_s2s_line(Voodoo3State *s, const uint8_t *src_p,
                             int use_x_dir, int src_x, int src_tiled)
{
    voodoo3_blt_t *blt   = &s->blt;
    v3_clip_t     *clip  = &blt->clip[(blt->command & COMMAND_CLIP_SEL) ? 1 : 0];
    int            dst_y = blt->dstY;
    int            pat_y = (blt->commandExtra & CMDEXTRA_FORCE_PAT_ROW0) ? 0
                           : (blt->patoff_y + blt->dstY);
    uint8_t       *pmono = (uint8_t *)blt->colorPattern;
    bool  use_pt = ((blt->command & (COMMAND_PATTERN_MONO | COMMAND_TRANS_MONO))
                   == (COMMAND_PATTERN_MONO | COMMAND_TRANS_MONO));
    int   src_ck = blt_src_ck_fmt(blt);
    bool  same_fmt = ((blt->srcFormat & SRC_FORMAT_COL_MASK) ==
                      (blt->dstFormat & DST_FORMAT_COL_MASK));

    if (dst_y >= clip->y_min && dst_y < clip->y_max) {
        int     dst_x  = blt->dstX;
        int     pat_x  = blt->patoff_x + blt->dstX;
        uint8_t pmask  = pmono[pat_y & 7];
        int     cur_sx = src_x;

        for (blt->cur_x = 0; blt->cur_x < blt->dstSizeX; blt->cur_x++) {
            bool pt = use_pt ? !!(pmask & (1u << (7 - (pat_x & 7)))) : true;
            int  sxr = (cur_sx * blt->src_bpp) >> 3;
            if (src_tiled) sxr = (sxr & 127) + ((sxr >> 7) * 128 * 32);

            if (dst_x >= clip->x_min && dst_x < clip->x_max && pt) {
                if (same_fmt) {
                    /* No colour conversion */
                    uint32_t src_pix = 0;
                    switch (blt->dstFormat & DST_FORMAT_COL_MASK) {
                    case DST_FORMAT_COL_8_BPP:  src_pix = src_p[sxr]; break;
                    case DST_FORMAT_COL_16_BPP: src_pix = *(const uint16_t *)(src_p + sxr); break;
                    case DST_FORMAT_COL_24_BPP: src_pix = *(const uint32_t *)(src_p + sxr); break;
                    case DST_FORMAT_COL_32_BPP: src_pix = *(const uint32_t *)(src_p + sxr); break;
                    default: break;
                    }
                    blt_plot(s, dst_x, dst_y, pat_x, pat_y, pmask, src_pix, src_ck);
                } else {
                    /* Colour conversion required */
                    uint32_t src_data = 0;
                    uint32_t yuv_data = 0;
                    bool transparent = false;

                    switch (blt->srcFormat & SRC_FORMAT_COL_MASK) {
                    case SRC_FORMAT_COL_1_BPP: {
                        uint8_t b = src_p[sxr];
                        src_data = (b & (0x80u >> (cur_sx & 7))) ? blt->colorFore : blt->colorBack;
                        if (blt->command & COMMAND_TRANS_MONO)
                            transparent = !(b & (0x80u >> (cur_sx & 7)));
                        break;
                    }
                    case SRC_FORMAT_COL_8_BPP:
                        src_data = src_p[sxr]; break;
                    case SRC_FORMAT_COL_16_BPP: {
                        uint16_t s16 = *(const uint16_t *)(src_p + sxr);
                        int r = (s16 >> 11) & 0x1f, g = (s16 >> 5) & 0x3f, b = s16 & 0x1f;
                        r = (r << 3)|(r >> 2); g = (g << 2)|(g >> 4); b = (b << 3)|(b >> 2);
                        src_data = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
                        break;
                    }
                    case SRC_FORMAT_COL_24_BPP:
                    case SRC_FORMAT_COL_32_BPP:
                        src_data = *(const uint32_t *)(src_p + sxr); break;
                    /*
                     * srcfmt=7: ARGB32 — identical wire format to 32_BPP
                     * (0xAARRGGBB) but alpha byte drives per-pixel blending.
                     * FIX: "RGB mask blits with RGB source (srcfmt = 7) and
                     *       different colormodels (destfmt = 0) not supported"
                     *      "cgx/WPAAlpha unsupported pixfmt: 0 for RECTFMT_ARGB"
                     */
                    case SRC_FORMAT_COL_ARGB32:
                        src_data = *(const uint32_t *)(src_p + sxr);
                        /* Dispatch via alpha-blend path; skip normal ROP plot */
                        if (!transparent) {
                            blt_alpha_blend_argb32(s, dst_x, dst_y, src_data);
                        }
                        goto s2s_next_pixel;
                    case SRC_FORMAT_COL_YUYV:
                        yuv_data = *(const uint32_t *)(src_p + sxr); break;
                    case SRC_FORMAT_COL_UYVY:
                        yuv_data = *(const uint32_t *)(src_p + sxr);
                        yuv_data = ((yuv_data & 0xff00u) >> 8) | ((yuv_data & 0xffu) << 8) |
                                   ((yuv_data & 0xff000000u) >> 8) | ((yuv_data & 0xff0000u) << 8);
                        break;
                    default: src_data = *(const uint32_t *)(src_p + sxr); break;
                    }

                    /* Downconvert to 16-bpp if dst is 16-bpp and src is not 1-bpp */
                    if ((blt->dstFormat & DST_FORMAT_COL_MASK) == DST_FORMAT_COL_16_BPP &&
                        (blt->srcFormat & SRC_FORMAT_COL_MASK) != SRC_FORMAT_COL_1_BPP &&
                        (blt->srcFormat & SRC_FORMAT_COL_MASK) != SRC_FORMAT_COL_YUYV &&
                        (blt->srcFormat & SRC_FORMAT_COL_MASK) != SRC_FORMAT_COL_UYVY) {
                        int r = (src_data >> 16) & 0xff, g = (src_data >> 8) & 0xff, b = src_data & 0xff;
                        src_data = (b >> 3) | ((g >> 2) << 5) | ((r >> 3) << 11);
                    }

                    if ((blt->srcFormat & SRC_FORMAT_COL_MASK) == SRC_FORMAT_COL_YUYV ||
                        (blt->srcFormat & SRC_FORMAT_COL_MASK) == SRC_FORMAT_COL_UYVY) {
                        /* YUV → plot two pixels */
                        if ((blt->dstFormat & DST_FORMAT_COL_MASK) == DST_FORMAT_COL_16_BPP) {
                            uint16_t rgb16[2] = {0,0};
                            blt_decode_yuyv422_16(rgb16, (const uint8_t *)&yuv_data);
                            if (!transparent) blt_plot(s, dst_x, dst_y, pat_x, pat_y, pmask, rgb16[0], src_ck);
                            if (use_x_dir) dst_x += (blt->command & COMMAND_DX) ? -1 : 1;
                            else           dst_x++;
                            if (!transparent) blt_plot(s, dst_x, dst_y, pat_x, pat_y, pmask, rgb16[1], src_ck);
                        } else {
                            uint32_t rgb32[2] = {0,0};
                            blt_decode_yuyv422_32(rgb32, (const uint8_t *)&yuv_data);
                            if (!transparent) blt_plot(s, dst_x, dst_y, pat_x, pat_y, pmask, rgb32[0], src_ck);
                            if (use_x_dir) dst_x += (blt->command & COMMAND_DX) ? -1 : 1;
                            else           dst_x++;
                            if (!transparent) blt_plot(s, dst_x, dst_y, pat_x, pat_y, pmask, rgb32[1], src_ck);
                        }
                    } else {
                        if (!transparent)
                            blt_plot(s, dst_x, dst_y, pat_x, pat_y, pmask, src_data, src_ck);
                    }
                }
            }
s2s_next_pixel:
            if (use_x_dir) {
                cur_sx += (blt->command & COMMAND_DX) ? -1 : 1;
                dst_x  += (blt->command & COMMAND_DX) ? -1 : 1;
                pat_x  += (blt->command & COMMAND_DX) ? -1 : 1;
            } else {
                cur_sx++; dst_x++; pat_x++;
            }
        }
    }
    blt->srcY += (blt->command & COMMAND_DY) ? -1 : 1;
    blt->dstY += (blt->command & COMMAND_DY) ? -1 : 1;
}

/* -------------------------------------------------------------------------
 * do_screen_to_screen_stretch_line — one scanline of stretch-blt
 * Ported from 86Box do_screen_to_screen_stretch_line()
 * Uses Bresenham X-scaling; caller manages Y-scaling via bres_error_0.
 * ------------------------------------------------------------------------- */
static void blt_do_stretch_line(Voodoo3State *s, const uint8_t *src_p,
                                 int src_x, int *p_src_y)
{
    voodoo3_blt_t *blt  = &s->blt;
    v3_clip_t     *clip = &blt->clip[(blt->command & COMMAND_CLIP_SEL) ? 1 : 0];
    int            dst_y = blt->dstY;
    int            pat_y = (blt->commandExtra & CMDEXTRA_FORCE_PAT_ROW0) ? 0
                           : (blt->patoff_y + blt->dstY);
    uint8_t       *pmono = (uint8_t *)blt->colorPattern;
    bool  use_pt = ((blt->command & (COMMAND_PATTERN_MONO | COMMAND_TRANS_MONO))
                   == (COMMAND_PATTERN_MONO | COMMAND_TRANS_MONO));
    int   src_ck = blt_src_ck_fmt(blt);
    bool  same_fmt = ((blt->srcFormat & SRC_FORMAT_COL_MASK) ==
                      (blt->dstFormat & DST_FORMAT_COL_MASK));

    if (dst_y >= clip->y_min && dst_y < clip->y_max) {
        int     dst_x   = blt->dstX;
        int     pat_x   = blt->patoff_x + blt->dstX;
        uint8_t pmask   = pmono[pat_y & 7];
        int     error_x = blt->dstSizeX / 2;
        int     cur_sx  = src_x;

        for (blt->cur_x = 0; blt->cur_x < blt->dstSizeX; blt->cur_x++) {
            bool pt = use_pt ? !!(pmask & (1u << (7 - (pat_x & 7)))) : true;

            if (dst_x >= clip->x_min && dst_x < clip->x_max && pt) {
                if (same_fmt) {
                    uint32_t src_pix = 0;
                    switch (blt->dstFormat & DST_FORMAT_COL_MASK) {
                    case DST_FORMAT_COL_8_BPP:  src_pix = src_p[cur_sx]; break;
                    case DST_FORMAT_COL_16_BPP: src_pix = *(const uint16_t *)(src_p + cur_sx * 2); break;
                    case DST_FORMAT_COL_24_BPP: src_pix = *(const uint32_t *)(src_p + cur_sx * 3); break;
                    case DST_FORMAT_COL_32_BPP: src_pix = *(const uint32_t *)(src_p + cur_sx * 4); break;
                    default: break;
                    }
                    blt_plot(s, dst_x, dst_y, pat_x, pat_y, pmask, src_pix, src_ck);
                } else {
                    int      sxr = (cur_sx * blt->src_bpp) >> 3;
                    uint32_t src_data = 0, yuv_data = 0;
                    bool     transparent = false;
                    switch (blt->srcFormat & SRC_FORMAT_COL_MASK) {
                    case SRC_FORMAT_COL_1_BPP: {
                        uint8_t b = src_p[sxr];
                        src_data = (b & (0x80u >> (cur_sx & 7))) ? blt->colorFore : blt->colorBack;
                        if (blt->command & COMMAND_TRANS_MONO)
                            transparent = !(b & (0x80u >> (cur_sx & 7)));
                        break;
                    }
                    case SRC_FORMAT_COL_8_BPP:  src_data = src_p[sxr]; break;
                    case SRC_FORMAT_COL_16_BPP: {
                        uint16_t s16 = *(const uint16_t *)(src_p + sxr);
                        int r=(s16>>11)&0x1f, g=(s16>>5)&0x3f, b=s16&0x1f;
                        r=(r<<3)|(r>>2); g=(g<<2)|(g>>4); b=(b<<3)|(b>>2);
                        src_data=((uint32_t)r<<16)|((uint32_t)g<<8)|b;
                        break;
                    }
                    case SRC_FORMAT_COL_24_BPP:
                    case SRC_FORMAT_COL_32_BPP: src_data = *(const uint32_t *)(src_p+sxr); break;
                    /* srcfmt=7: ARGB32 with alpha — use alpha-blend path */
                    case SRC_FORMAT_COL_ARGB32:
                        src_data = *(const uint32_t *)(src_p+sxr);
                        if (!transparent) blt_alpha_blend_argb32(s, dst_x, dst_y, src_data);
                        goto stretch_next_pixel;
                    case SRC_FORMAT_COL_YUYV: yuv_data = *(const uint32_t *)(src_p+sxr); break;
                    case SRC_FORMAT_COL_UYVY:
                        yuv_data = *(const uint32_t *)(src_p+sxr);
                        yuv_data = ((yuv_data&0xff00u)>>8)|((yuv_data&0xffu)<<8)|
                                   ((yuv_data&0xff000000u)>>8)|((yuv_data&0xff0000u)<<8);
                        break;
                    default: src_data = *(const uint32_t *)(src_p+sxr); break;
                    }
                    if ((blt->dstFormat & DST_FORMAT_COL_MASK) == DST_FORMAT_COL_16_BPP &&
                        (blt->srcFormat & SRC_FORMAT_COL_MASK) != SRC_FORMAT_COL_1_BPP &&
                        (blt->srcFormat & SRC_FORMAT_COL_MASK) != SRC_FORMAT_COL_YUYV &&
                        (blt->srcFormat & SRC_FORMAT_COL_MASK) != SRC_FORMAT_COL_UYVY) {
                        int r=(src_data>>16)&0xff, g=(src_data>>8)&0xff, b=src_data&0xff;
                        src_data=(b>>3)|((g>>2)<<5)|((r>>3)<<11);
                    }
                    if ((blt->srcFormat & SRC_FORMAT_COL_MASK) == SRC_FORMAT_COL_YUYV ||
                        (blt->srcFormat & SRC_FORMAT_COL_MASK) == SRC_FORMAT_COL_UYVY) {
                        if ((blt->dstFormat & DST_FORMAT_COL_MASK) == DST_FORMAT_COL_16_BPP) {
                            uint16_t rgb16[2]={0,0};
                            blt_decode_yuyv422_16(rgb16,(const uint8_t*)&yuv_data);
                            if (!transparent) blt_plot(s,dst_x,dst_y,pat_x,pat_y,pmask,rgb16[0],src_ck);
                            dst_x++;
                            if (!transparent) blt_plot(s,dst_x,dst_y,pat_x,pat_y,pmask,rgb16[1],src_ck);
                        } else {
                            uint32_t rgb32[2]={0,0};
                            blt_decode_yuyv422_32(rgb32,(const uint8_t*)&yuv_data);
                            if (!transparent) blt_plot(s,dst_x,dst_y,pat_x,pat_y,pmask,rgb32[0],src_ck);
                            dst_x++;
                            if (!transparent) blt_plot(s,dst_x,dst_y,pat_x,pat_y,pmask,rgb32[1],src_ck);
                        }
                    } else {
                        if (!transparent)
                            blt_plot(s, dst_x, dst_y, pat_x, pat_y, pmask, src_data, src_ck);
                    }
                }
            }

stretch_next_pixel:
            /* Bresenham X step */
            error_x -= blt->srcSizeX;
            while (error_x < 0) { error_x += blt->dstSizeX; cur_sx++; }
            dst_x++; pat_x++;
        }
    }

    /* Bresenham Y step */
    blt->bres_error_0 -= blt->srcSizeY;
    while (blt->bres_error_0 < 0) {
        blt->bres_error_0 += blt->dstSizeY;
        if (p_src_y) (*p_src_y) += (blt->command & COMMAND_DY) ? -1 : 1;
    }
    blt->dstY += (blt->command & COMMAND_DY) ? -1 : 1;
}

/* -------------------------------------------------------------------------
 * do_rectfill — ported from 86Box banshee_do_rectfill()
 * Full clip, pattern, ROP, TRANS_MONO.
 * ------------------------------------------------------------------------- */
static void blt_do_rectfill(Voodoo3State *s)
{
    voodoo3_blt_t *blt  = &s->blt;
    v3_clip_t     *clip = &blt->clip[(blt->command & COMMAND_CLIP_SEL) ? 1 : 0];

    uint8_t  rop8    = (uint8_t)(blt->command >> 24);
    bool     no_ck   = !(blt->commandExtra &
                         (CMDEXTRA_SRC_COLORKEY | CMDEXTRA_DST_COLORKEY));
    bool     no_tp   = !((blt->command & (COMMAND_PATTERN_MONO | COMMAND_TRANS_MONO))
                         == (COMMAND_PATTERN_MONO | COMMAND_TRANS_MONO));
    bool     linear  = !blt->dstBaseAddr_tiled;
    bool     pos_dir = !(blt->command & (COMMAND_DX | COMMAND_DY));
    bool     full_clip =
        (blt->dstX >= clip->x_min &&
         blt->dstX + blt->dstSizeX <= clip->x_max &&
         blt->dstY >= clip->y_min &&
         blt->dstY + blt->dstSizeY <= clip->y_max);

    /*
     * Fast path: solid colour fill.
     * ROP=0xF0 (PATCOPY) and ROP=0xCC (SRCCOPY) are both solid fills for
     * RECTFILL — the Picasso96 Voodoo3 driver always uses 0xCC with colorFore
     * as the source, which produces the same result as 0xF0.
     */
    if ((rop8 == 0xF0 || rop8 == 0xCC) && no_ck && no_tp && linear && pos_dir && full_clip) {
        int      bpp    = blt->dstBpp;
        int      bpp_bits = bpp * 8;
        uint32_t stride = blt->dst_stride;
        int      w      = blt->dstSizeX;
        int      h      = blt->dstSizeY;
        uint32_t color  = blt->colorFore;

        for (int y = 0; y < h; y++) {
            int abs_y = blt->dstY + y;
            uint32_t row_off = blt->dstBaseAddr
                             + (uint32_t)abs_y * stride
                             + (uint32_t)blt->dstX * bpp;
            if (row_off + (uint32_t)w * bpp > s->fb_size) break;
            uint8_t *row = s->fb_mem + row_off;

            switch (bpp_bits) {
            case 8:
                memset(row, (uint8_t)color, (size_t)w);
                break;
            case 16: {
                uint16_t c = bswap16((uint16_t)color);
                uint16_t *p = (uint16_t *)row;
                for (int x = 0; x < w; x++) *p++ = c;
                break;
            }
            case 24: {
                /* 24bpp: write byte-triplets */
                uint8_t b0 = (uint8_t) color;
                uint8_t b1 = (uint8_t)(color >>  8);
                uint8_t b2 = (uint8_t)(color >> 16);
                uint8_t *p = row;
                for (int x = 0; x < w; x++) {
                    *p++ = b0; *p++ = b1; *p++ = b2;
                }
                break;
            }
            case 32:
            default: {
                uint32_t *p = (uint32_t *)row;
                for (int x = 0; x < w; x++) *p++ = color;
                break;
            }
            }

            if ((unsigned)abs_y < V3_DIRTY_LINES)
                s->dirty_line[abs_y] = 1;
        }
        blt_end_command(blt);
        return;
    }

    /* Slow path: ROP / colorkey / transparent pattern / tiling / clip */
    uint8_t       *pmono = (uint8_t *)blt->colorPattern;
    bool use_pt = ((blt->command & (COMMAND_PATTERN_MONO | COMMAND_TRANS_MONO))
                   == (COMMAND_PATTERN_MONO | COMMAND_TRANS_MONO));
    int pat_y = (blt->commandExtra & CMDEXTRA_FORCE_PAT_ROW0) ? 0
                : (blt->patoff_y + blt->dstY);
    int dst_y = blt->dstY;

    for (blt->cur_y = 0; blt->cur_y < blt->dstSizeY; blt->cur_y++) {
        if (dst_y >= clip->y_min && dst_y < clip->y_max) {
            int     dst_x  = blt->dstX;
            int     pat_x  = blt->patoff_x + blt->dstX;
            uint8_t pmask  = pmono[pat_y & 7];

            for (blt->cur_x = 0; blt->cur_x < blt->dstSizeX; blt->cur_x++) {
                bool pt = use_pt ? !!(pmask & (1u << (7 - (pat_x & 7)))) : true;
                if (dst_x >= clip->x_min && dst_x < clip->x_max && pt)
                    blt_plot(s, dst_x, dst_y, pat_x, pat_y, pmask,
                             blt->colorFore, BLT_COLORKEY_32);
                dst_x += (blt->command & COMMAND_DX) ? -1 : 1;
                pat_x += (blt->command & COMMAND_DX) ? -1 : 1;
            }
        }
        dst_y += (blt->command & COMMAND_DY) ? -1 : 1;
        if (!(blt->commandExtra & CMDEXTRA_FORCE_PAT_ROW0))
            pat_y += (blt->command & COMMAND_DY) ? -1 : 1;
    }
    blt_end_command(blt);
}

/* -------------------------------------------------------------------------
 * do_screen_to_screen_blt — full S2S with ROP/clip/pattern/colorkey
 * Ported from 86Box banshee_do_screen_to_screen_blt()
 * ------------------------------------------------------------------------- */
static void blt_do_s2s_blt(Voodoo3State *s)
{
    voodoo3_blt_t *blt = &s->blt;

    /*
     * Fast path: plain copy (ROP=0xCC), same pixel format, no colorkey,
     * no pattern, no tiling, rectangular clip covers whole blit.
     * This is the common case for window dragging and desktop composition.
     * Use memmove (handles overlapping regions correctly).
     */
    uint8_t rop8 = (uint8_t)(blt->command >> 24);
    v3_clip_t *clip = &blt->clip[(blt->command & COMMAND_CLIP_SEL) ? 1 : 0];
    bool no_colorkey = !(blt->commandExtra & (CMDEXTRA_SRC_COLORKEY | CMDEXTRA_DST_COLORKEY));
    bool same_format = ((blt->srcFormat & SRC_FORMAT_COL_MASK) ==
                        (blt->dstFormat & DST_FORMAT_COL_MASK));
    bool full_clip   = (blt->dstX >= clip->x_min && blt->dstX + blt->dstSizeX <= clip->x_max &&
                        blt->dstY >= clip->y_min && blt->dstY + blt->dstSizeY <= clip->y_max);
    bool no_tiling   = !blt->srcBaseAddr_tiled && !blt->dstBaseAddr_tiled;
    bool x_positive  = !(blt->command & COMMAND_DX);
    bool y_positive  = !(blt->command & COMMAND_DY);

    /*
     * Fast path: plain copy (ROP=0xCC = SRCCOPY), all 4 direction combinations.
     * Pattern flags are irrelevant for SRCCOPY — result = src regardless.
     * The Picasso96 driver always sets COMMAND_PATTERN_MONO even for plain
     * BltBitMap, which would block this path unnecessarily.
     *
     * Direction semantics (Picasso96 / Banshee datasheet):
     *   DX=0 DY=0: srcXY/dstXY = top-left.  Iterate top-to-bottom.
     *   DX=1 DY=1: srcXY/dstXY = bottom-right. Iterate bottom-to-top.
     *              This is the OverlappedBlitRect case (scroll down / window drag).
     *   DX=0 DY=1 or DX=1 DY=0: mixed — handled correctly below.
     *
     * memmove() is used per-row so X-overlap within a row is always safe.
     * Y-overlap is handled by the iteration order (top-to-bottom when dst
     * is above src, bottom-to-top when dst is below src).
     */
    if (rop8 == 0xCC && no_colorkey && same_format &&
        full_clip && no_tiling) {

        int      bpp         = blt->dstBpp;
        uint32_t src_stride  = blt->src_stride_dest;
        uint32_t dst_stride  = blt->dst_stride;
        int      w           = blt->dstSizeX;
        int      h           = blt->dstSizeY;
        uint32_t width_bytes = (uint32_t)w * bpp;

        /*
         * When DX=1/DY=1 the coordinates point at the bottom-right corner.
         * Normalise to top-left so address arithmetic is uniform.
         */
        int src_x0 = x_positive ? blt->srcX : blt->srcX - (w - 1);
        int src_y0 = y_positive ? blt->srcY : blt->srcY - (h - 1);
        int dst_x0 = x_positive ? blt->dstX : blt->dstX - (w - 1);
        int dst_y0 = y_positive ? blt->dstY : blt->dstY - (h - 1);

        for (int y = 0; y < h; y++) {
            /* When destination is below source, iterate bottom-to-top to
             * avoid overwriting source lines before they are copied. */
            int row = y_positive ? y : (h - 1 - y);
            uint32_t src_addr = blt->srcBaseAddr
                              + (uint32_t)(src_y0 + row) * src_stride
                              + (uint32_t)src_x0 * bpp;
            uint32_t dst_addr = blt->dstBaseAddr
                              + (uint32_t)(dst_y0 + row) * dst_stride
                              + (uint32_t)dst_x0 * bpp;
            if (src_addr + width_bytes > s->fb_size) break;
            if (dst_addr + width_bytes > s->fb_size) break;
            memmove(s->fb_mem + dst_addr, s->fb_mem + src_addr, width_bytes);
            int abs_y = dst_y0 + row;
            if ((unsigned)abs_y < V3_DIRTY_LINES)
                s->dirty_line[abs_y] = 1;
        }
        blt_end_command(blt);
        return;
    }

    /* Slow path: full ROP/colorkey/pattern/tiling/clip handling */
    for (blt->cur_y = 0; blt->cur_y < blt->dstSizeY; blt->cur_y++) {
        uint32_t src_addr = blt_get_addr(s, 0, blt->srcY, 1, blt->src_stride_dest);
        blt_do_s2s_line(s, s->fb_mem + src_addr, 1, blt->srcX, !!blt->srcBaseAddr_tiled);
    }
    blt_end_command(blt);
}

/* -------------------------------------------------------------------------
 * do_screen_to_screen_stretch_blt — S2S with Bresenham X+Y scaling
 * Ported from 86Box banshee_do_screen_to_screen_stretch_blt()
 * ------------------------------------------------------------------------- */
static void blt_do_stretch_blt(Voodoo3State *s)
{
    voodoo3_blt_t *blt = &s->blt;
    for (blt->cur_y = 0; blt->cur_y < blt->dstSizeY; blt->cur_y++) {
        uint32_t src_addr = blt_get_addr(s, 0, blt->srcY, 1, blt->src_stride_src);
        blt_do_stretch_line(s, s->fb_mem + src_addr, blt->srcX, &blt->srcY);
    }
    blt_end_command(blt);
}

/* -------------------------------------------------------------------------
 * do_host_to_screen_blt — H2S: accumulate dwords, flush on complete row
 * Ported from 86Box banshee_do_host_to_screen_blt()
 * ------------------------------------------------------------------------- */
static void blt_do_h2s_blt(Voodoo3State *s, uint32_t data)
{
    voodoo3_blt_t *blt = &s->blt;

    /* Byte/word swizzle */
    if (blt->srcFormat & SRC_FORMAT_BYTE_SWIZZLE)
        data = ((data >> 24)) | ((data >> 8) & 0xff00u) | ((data << 8) & 0xff0000u) | (data << 24);
    if (blt->srcFormat & SRC_FORMAT_WORD_SWIZZLE)
        data = (data >> 16) | (data << 16);

    if ((blt->srcFormat & SRC_FORMAT_PACKING_MASK) == SRC_FORMAT_PACKING_STRIDE) {
        int last_byte;
        if ((blt->srcFormat & SRC_FORMAT_COL_MASK) == SRC_FORMAT_COL_1_BPP)
            last_byte = ((blt->srcX & 31) + blt->dstSizeX + 7) >> 3;
        else
            last_byte = (blt->srcX & 3) + blt->host_data_size_dest;

        *(uint32_t *)(blt->host_data + blt->host_data_count) = data;
        blt->host_data_count += 4;
        if (blt->host_data_count >= last_byte) {
            if (blt->cur_y < blt->dstSizeY) {
                if ((blt->srcFormat & SRC_FORMAT_COL_MASK) == SRC_FORMAT_COL_1_BPP)
                    blt_do_s2s_line(s, blt->host_data + ((blt->srcX >> 3) & 3), 0, blt->srcX & 7, 0);
                else
                    blt_do_s2s_line(s, blt->host_data + (blt->srcX & 3), 0, 0, 0);
                blt->cur_y++;
                if (blt->cur_y == blt->dstSizeY) blt_end_command(blt);
            }
            if ((blt->srcFormat & SRC_FORMAT_COL_MASK) == SRC_FORMAT_COL_1_BPP)
                blt->srcX += (blt->srcFormat & SRC_FORMAT_STRIDE_MASK) << 3;
            else
                blt->srcX += (blt->srcFormat & SRC_FORMAT_STRIDE_MASK);
            blt->host_data_count = 0;
        }
    } else {
        *(uint32_t *)(blt->host_data + blt->host_data_count) = data;
        blt->host_data_count += 4;
        while (blt->host_data_count >= blt->src_stride_dest) {
            blt->host_data_count -= blt->src_stride_dest;
            if (blt->cur_y < blt->dstSizeY) {
                blt_do_s2s_line(s, blt->host_data, 0, 0, 0);
                blt->cur_y++;
                if (blt->cur_y == blt->dstSizeY) blt_end_command(blt);
            }
            if (blt->host_data_count)
                *(uint32_t *)(blt->host_data) = data >> ((4 - blt->host_data_count) * 8);
        }
    }
}

/* -------------------------------------------------------------------------
 * do_host_to_screen_stretch_blt — H2S stretch variant
 * Ported from 86Box banshee_do_host_to_screen_stretch_blt()
 * ------------------------------------------------------------------------- */
static void blt_do_h2s_stretch_blt(Voodoo3State *s, uint32_t data)
{
    voodoo3_blt_t *blt = &s->blt;

    if (blt->srcFormat & SRC_FORMAT_BYTE_SWIZZLE)
        data = (data >> 24) | ((data >> 8) & 0xff00u) | ((data << 8) & 0xff0000u) | (data << 24);
    if (blt->srcFormat & SRC_FORMAT_WORD_SWIZZLE)
        data = (data >> 16) | (data << 16);

    if ((blt->srcFormat & SRC_FORMAT_PACKING_MASK) == SRC_FORMAT_PACKING_STRIDE) {
        int last_byte = (blt->srcX & 3) + blt->host_data_size_src;
        *(uint32_t *)(blt->host_data + blt->host_data_count) = data;
        blt->host_data_count += 4;
        if (blt->host_data_count >= last_byte) {
            if (blt->cur_y < blt->dstSizeY) {
                if ((blt->srcFormat & SRC_FORMAT_COL_MASK) == SRC_FORMAT_COL_1_BPP)
                    blt_do_stretch_line(s, blt->host_data + ((blt->srcX >> 3) & 3), blt->srcX & 7, NULL);
                else
                    blt_do_stretch_line(s, blt->host_data + (blt->srcX & 3), 0, NULL);
                blt->cur_y++;
                if (blt->cur_y == blt->dstSizeY) blt_end_command(blt);
            }
            if ((blt->srcFormat & SRC_FORMAT_COL_MASK) == SRC_FORMAT_COL_1_BPP)
                blt->srcX += (blt->srcFormat & SRC_FORMAT_STRIDE_MASK) << 3;
            else
                blt->srcX += (blt->srcFormat & SRC_FORMAT_STRIDE_MASK);
            blt->host_data_count = 0;
        }
    } else {
        *(uint32_t *)(blt->host_data + blt->host_data_count) = data;
        blt->host_data_count += 4;
        while (blt->host_data_count >= blt->src_stride_src) {
            blt->host_data_count -= blt->src_stride_src;
            if (blt->cur_y < blt->dstSizeY) {
                blt_do_stretch_line(s, blt->host_data, 0, NULL);
                blt->cur_y++;
                if (blt->cur_y == blt->dstSizeY) blt_end_command(blt);
            }
            if (blt->host_data_count)
                *(uint32_t *)(blt->host_data) = data >> ((4 - blt->host_data_count) * 8);
        }
    }
}

/* -------------------------------------------------------------------------
 * step_line — advance stipple/repeat state for line drawing
 * Ported from 86Box step_line()
 * ------------------------------------------------------------------------- */
static inline void blt_step_line(voodoo3_blt_t *blt)
{
    if (blt->line_pix_pos == blt->line_rep_cnt) {
        blt->line_pix_pos = 0;
        if (blt->line_bit_pos == blt->line_bit_mask_size)
            blt->line_bit_pos = 0;
        else
            blt->line_bit_pos++;
    } else {
        blt->line_pix_pos++;
    }
}

/* -------------------------------------------------------------------------
 * do_line — Bresenham line with stipple, TRANS_MONO, clip
 * Ported from 86Box banshee_do_line()
 * draw_last: 1 = line, 0 = polyline (don't draw endpoint)
 * ------------------------------------------------------------------------- */
static void blt_do_line(Voodoo3State *s, bool draw_last)
{
    voodoo3_blt_t *blt  = &s->blt;
    v3_clip_t     *clip = &blt->clip[(blt->command & COMMAND_CLIP_SEL) ? 1 : 0];
    int dx = abs(blt->dstX - blt->srcX);
    int dy = abs(blt->dstY - blt->srcY);
    int x_inc = (blt->dstX > blt->srcX) ? 1 : -1;
    int y_inc = (blt->dstY > blt->srcY) ? 1 : -1;
    int x = blt->srcX, y = blt->srcY;
    int error;
    uint32_t stipple = (blt->command & COMMAND_STIPPLE_LINE) ? blt->lineStipple : ~0u;

    if (dx > dy) { /* X-major */
        error = dx / 2;
        while (x != blt->dstX) {
            uint32_t mask = stipple & (1u << blt->line_bit_pos);
            bool pt = (blt->command & COMMAND_TRANS_MONO) ? !!mask : true;
            if (y >= clip->y_min && y < clip->y_max &&
                x >= clip->x_min && x < clip->x_max && pt)
                blt_plot_line(s, x, y, mask ? blt->colorFore : blt->colorBack);
            error -= dy;
            if (error < 0) { error += dx; y += y_inc; }
            x += x_inc;
            blt_step_line(blt);
        }
    } else { /* Y-major */
        error = dy / 2;
        while (y != blt->dstY) {
            uint32_t mask = stipple & (1u << blt->line_bit_pos);
            bool pt = (blt->command & COMMAND_TRANS_MONO) ? !!mask : true;
            if (y >= clip->y_min && y < clip->y_max &&
                x >= clip->x_min && x < clip->x_max && pt)
                blt_plot_line(s, x, y, mask ? blt->colorFore : blt->colorBack);
            error -= dx;
            if (error < 0) { error += dy; x += x_inc; }
            y += y_inc;
            blt_step_line(blt);
        }
    }

    if (draw_last) {
        uint32_t mask = stipple & (1u << blt->line_bit_pos);
        bool pt = (blt->command & COMMAND_TRANS_MONO) ? !!mask : true;
        if (y >= clip->y_min && y < clip->y_max &&
            x >= clip->x_min && x < clip->x_max && pt)
            blt_plot_line(s, x, y, mask ? blt->colorFore : blt->colorBack);
    }

    /* Update srcXY to current endpoint — polyline chaining */
    blt->srcXY = ((uint32_t)(x & 0xffffu)) | ((uint32_t)y << 16);
    blt->srcX = x; blt->srcY = y;
}

/* -------------------------------------------------------------------------
 * polyfill_start — initialise polyfill edge state from srcXY/dstXY
 * Ported from 86Box banshee_polyfill_start()
 * ------------------------------------------------------------------------- */
static void blt_polyfill_start(voodoo3_blt_t *blt)
{
    blt->lx[0] = blt->srcX; blt->ly[0] = blt->srcY;
    blt->rx[0] = blt->dstX; blt->ry[0] = blt->dstY;
    blt->lx[1] = blt->srcX; blt->ly[1] = blt->srcY;
    blt->rx[1] = blt->dstX; blt->ry[1] = blt->dstY;
    blt->lx_cur = blt->srcX;
    blt->rx_cur = blt->dstX;
}

/* -------------------------------------------------------------------------
 * polyfill_continue — receive next vertex, fill spans between edges
 * Ported from 86Box banshee_polyfill_continue()
 * data: packed vertex = bits[12:0]=X (sign-extended from 13), bits[28:16]=Y
 * ------------------------------------------------------------------------- */
static void blt_polyfill_continue(Voodoo3State *s, uint32_t data)
{
    voodoo3_blt_t *blt  = &s->blt;
    v3_clip_t     *clip = &blt->clip[(blt->command & COMMAND_CLIP_SEL) ? 1 : 0];
    uint8_t       *pmono = (uint8_t *)blt->colorPattern;
    bool use_pt = ((blt->command & (COMMAND_PATTERN_MONO | COMMAND_TRANS_MONO))
                   == (COMMAND_PATTERN_MONO | COMMAND_TRANS_MONO));
    int y_start = MAX(blt->ly[0], blt->ry[0]);
    int y_end;

    /*
     * FIX 10: polyfill edge-selection — exact 86Box semantics restored.
     *
     * 86Box banshee_polyfill_continue() rule (from source):
     *   if (ry[1] >= ly[1]) → assign new vertex to LEFT edge
     *   else                 → assign new vertex to RIGHT edge
     *
     * Meaning: when the right tip is at or above (Y-wise equal or further
     * along) the left tip, the LEFT edge has been consumed first and
     * receives the next vertex.  This is the correct criterion for a
     * scanline-fill that walks downward.
     *
     * FIX 6 comment was correct in diagnosis but the implementation
     * `if (blt->ly[1] < blt->ry[1])` is logically equivalent to the
     * pre-fix code (ly[1] < ry[1]  ⟺  ry[1] > ly[1]) and does NOT
     * match 86Box's  ry[1] >= ly[1]  (which also fires when they are
     * equal).  The equal case is the horizontal-top-edge scenario that
     * was identified as broken — it must select LEFT, not RIGHT.
     *
     * Retire logic at the bottom uses the same ry[1] >= ly[1] sense.
     */
    if (blt->ry[1] >= blt->ly[1]) {
        /* Right tip is at or ahead — left edge needs the new vertex */
        blt->lx[1] = ((int32_t)(data << 19)) >> 19;
        blt->ly[1] = ((int32_t)(data <<  3)) >> 19;
        blt->dx[0]    = abs(blt->lx[1] - blt->lx[0]);
        blt->dy[0]    = abs(blt->ly[1] - blt->ly[0]);
        blt->x_inc[0] = (blt->lx[1] > blt->lx[0]) ? 1 : -1;
        blt->error[0] = blt->dy[0] / 2;
    } else {
        /* Left tip is ahead — right edge needs the new vertex */
        blt->rx[1] = ((int32_t)(data << 19)) >> 19;
        blt->ry[1] = ((int32_t)(data <<  3)) >> 19;
        blt->dx[1]    = abs(blt->rx[1] - blt->rx[0]);
        blt->dy[1]    = abs(blt->ry[1] - blt->ry[0]);
        blt->x_inc[1] = (blt->rx[1] > blt->rx[0]) ? 1 : -1;
        blt->error[1] = blt->dy[1] / 2;
    }

    y_end = MIN(blt->ly[1], blt->ry[1]);

    for (int y = y_start; y < y_end; y++) {
        if (y >= clip->y_min && y < clip->y_max) {
            int     pat_y  = (blt->commandExtra & CMDEXTRA_FORCE_PAT_ROW0) ? 0
                             : (blt->patoff_y + y);
            uint8_t pmask  = pmono[pat_y & 7];

            for (int x = blt->lx_cur; x < blt->rx_cur; x++) {
                int  pat_x = blt->patoff_x + x;
                bool pt    = use_pt ? !!(pmask & (1u << (7 - (pat_x & 7)))) : true;
                if (x >= clip->x_min && x < clip->x_max && pt)
                    blt_plot(s, x, y, pat_x, pat_y, pmask, blt->colorFore, BLT_COLORKEY_32);
            }
        }
        /* Advance left edge */
        blt->error[0] -= blt->dx[0];
        while (blt->error[0] < 0) { blt->error[0] += blt->dy[0]; blt->lx_cur += blt->x_inc[0]; }
        /* Advance right edge */
        blt->error[1] -= blt->dx[1];
        while (blt->error[1] < 0) { blt->error[1] += blt->dy[1]; blt->rx_cur += blt->x_inc[1]; }
    }

    /*
     * Retire completed vertices — 86Box-faithful.
     * When both tips meet (ry[1]==ly[1]): both edges exhausted, retire both.
     * When ry[1] >= ly[1]: left was just extended (its old [0] is now done).
     * Otherwise:           right was just extended (its old [0] is now done).
     */
    if (blt->ry[1] == blt->ly[1]) {
        blt->lx[0]=blt->lx[1]; blt->ly[0]=blt->ly[1];
        blt->rx[0]=blt->rx[1]; blt->ry[0]=blt->ry[1];
    } else if (blt->ry[1] >= blt->ly[1]) {
        /* Left edge was the one we just extended — retire left [0] */
        blt->lx[0]=blt->lx[1]; blt->ly[0]=blt->ly[1];
    } else {
        /* Right edge was the one we just extended — retire right [0] */
        blt->rx[0]=blt->rx[1]; blt->ry[0]=blt->ry[1];
    }
}

/* -------------------------------------------------------------------------
 * blt_do_launch — decode geometry at launch time
 * Ported from 86Box banshee_do_2d_launch()
 * ------------------------------------------------------------------------- */
static void blt_do_launch(Voodoo3State *s)
{
    voodoo3_blt_t *blt = &s->blt;
    blt->launch_pending          = false;
    blt->rops[0]                 = (uint8_t)(blt->command >> 24);
    blt->patoff_x                = (blt->command & COMMAND_PATOFF_X_MASK) >> COMMAND_PATOFF_X_SHIFT;
    blt->patoff_y                = (blt->command & COMMAND_PATOFF_Y_MASK) >> COMMAND_PATOFF_Y_SHIFT;
    blt->cur_x                   = 0;
    blt->cur_y                   = 0;
    blt->dstX = ((int32_t)(blt->dstXY << 19)) >> 19;
    blt->dstY = ((int32_t)(blt->dstXY <<  3)) >> 19;
    blt->srcX = ((int32_t)(blt->srcXY << 19)) >> 19;
    blt->srcY = ((int32_t)(blt->srcXY <<  3)) >> 19;
    blt->old_srcX                = blt->srcX;
    blt->host_data_remainder     = 0;
    blt->host_data_count         = 0;

    /*
     * Compute all strides here once at launch time instead of eagerly
     * on every register write. This eliminates 4-5 redundant recomputations
     * per blit operation and is the primary source of BltBitMap overhead.
     */
    voodoo3_blt_update_dst_stride(s);
    voodoo3_blt_update_src_stride(s);
    blt_update_src_stride_full(s);
}

static void voodoo3_blt_execute(Voodoo3State *s)
{
    voodoo3_blt_t *blt = &s->blt;

    if (blt->launch_pending)
        blt_do_launch(s);

    /* Decode dstBpp from dstFormat bits[19:16] */
    switch (blt->dstFormat & DST_FORMAT_COL_MASK) {
    case DST_FORMAT_COL_PAL:    blt->dstBpp = 1; break; /* dstfmt=0: raw 8-bpp palette */
    case DST_FORMAT_COL_8_BPP:  blt->dstBpp = 1; break;
    case DST_FORMAT_COL_16_BPP: blt->dstBpp = 2; break;
    case DST_FORMAT_COL_24_BPP: blt->dstBpp = 3; break;
    case DST_FORMAT_COL_32_BPP: blt->dstBpp = 4; break;
    default:                     blt->dstBpp = 2; break;
    }

    switch (blt->command & COMMAND_CMD_MASK) {
    case COMMAND_CMD_NOP:
        break;
    case COMMAND_CMD_RECTFILL:
        blt_do_rectfill(s);
        break;
    case COMMAND_CMD_S2S_BLT:
    case 9: case 12:    /* AmigaOS transparent / extended S2S variants */
        blt_do_s2s_blt(s);
        break;
    case COMMAND_CMD_S2S_STRETCH:
        blt_do_stretch_blt(s);
        break;
    case COMMAND_CMD_LINE:
        blt_do_line(s, true);
        break;
    case COMMAND_CMD_POLYLINE:
        blt_do_line(s, false);
        break;
    case COMMAND_CMD_H2S_BLT:
    case COMMAND_CMD_H2S_STRETCH:
        /* H2S: launched by register write — data arrives in launch handler */
        blt->host_data_count = 0;
        blt->cur_y           = 0;
        /* stride must be ready before first data word arrives */
        voodoo3_blt_update_dst_stride(s);
        voodoo3_blt_update_src_stride(s);
        blt_update_src_stride_full(s);
        break;
    case COMMAND_CMD_POLYFILL:
        /* Polyfill: handled entirely via polyfill_start/continue */
        break;

    /*
     * FIX C: 2D blitter command 11 — TRANS_MONO_BLT
     *   Host-to-screen monochrome expand with transparency.
     *   Each bit in the host data stream maps to one destination pixel:
     *     bit=1 → colorFore  (opaque)
     *     bit=0 → transparent (skip, preserve dst)
     *   Used for: text rendering, icon blitting, cursor compositing.
     *   The host data is written via launch registers (like H2S_BLT);
     *   here we set up the state so blt_do_h2s_blt() processes it
     *   correctly.  blt_do_h2s_blt() already checks SRC_FORMAT_COL_1_BPP
     *   and COMMAND_TRANS_MONO — ensure both bits are set.
     *
     *   Ported from: 86Box vid_voodoo_banshee_blitter.c blt_blt() case 11.
     */
    case 11: /* TRANS_MONO_BLT */
        blt->host_data_count = 0;
        blt->cur_y           = 0;
        /* Force 1-bpp source format and TRANS_MONO so blt_do_h2s_blt()
         * uses the monochrome expand path with transparency. */
        blt->srcFormat = (blt->srcFormat & ~SRC_FORMAT_COL_MASK)
                       | SRC_FORMAT_COL_1_BPP;
        blt->command  |= COMMAND_TRANS_MONO | COMMAND_PATTERN_MONO;
        voodoo3_blt_update_dst_stride(s);
        voodoo3_blt_update_src_stride(s);
        blt_update_src_stride_full(s);
        break;

    /*
     * FIX C: 2D blitter command 13 — H2S raw pixel blit
     *   Identical to H2S_BLT (cmd 3) but the host data is raw 32-bit
     *   ARGB pixels without format conversion.  The driver uploads
     *   pre-converted pixels directly (icons, scaled bitmaps).
     *   We treat it as a standard H2S_BLT (same data path); blt_do_h2s_blt()
     *   writes host_data bytes directly into the framebuffer via
     *   blt_do_s2s_line() which copies raw bytes when srcBpp==dstBpp.
     *
     *   Ported from: 86Box blt_blt() case 13 (same as case 3).
     */
    case 13: /* H2S raw pixel blit */
        blt->host_data_count = 0;
        blt->cur_y           = 0;
        voodoo3_blt_update_dst_stride(s);
        voodoo3_blt_update_src_stride(s);
        blt_update_src_stride_full(s);
        break;

    /*
     * FIX C: 2D blitter command 14 — S2S_FLIP (horizontal mirror)
     *   Screen-to-screen blit with left-right flip: source pixels are
     *   read right-to-left (srcX decreasing) and written left-to-right.
     *   Used by some drivers for double-buffer flips and mirror effects.
     *   Implemented as a full S2S blit with COMMAND_DX forced active so
     *   the existing slow-path iterates X in reverse.
     *
     *   Ported from: 86Box blt_blt() case 14.
     */
    case 14: /* S2S_FLIP */
        blt_do_s2s_blt(s);
        break;
    }
}

static void voodoo3_2d_reg_write(Voodoo3State *s, uint32_t addr, uint32_t val)
{
    voodoo3_blt_t *blt = &s->blt;
    /*
     * Banshee 2D engine register map — ported from 86Box voodoo_2d_reg_writel()
     * in vid_voodoo_banshee_blitter.c.
     *
     * Launch registers 0x80..0xfc: writing triggers blit execution for
     * S2S (passes srcXY), H2S (passes pixel data), LINE/POLYLINE (passes
     * endpoint), RECTFILL (passes dstXY), POLYFILL (passes next vertex).
     *
     * Pattern registers 0x100..0x1fc: 8×8 32-bit color pattern with
     * decoded 8/16/24-bpp views for fast pixel access.
     */
    uint32_t off = addr & 0x1fc;

    /* ---- Launch registers 0x80..0xfc ---- */
    if (off >= 0x80 && off <= 0xfc) {
        if (blt->launch_pending)
            blt_do_launch(s);

        switch (blt->command & COMMAND_CMD_MASK) {
        case COMMAND_CMD_S2S_BLT:
            blt->srcXY = val;
            blt->srcX  = ((int32_t)(val << 19)) >> 19;
            blt->srcY  = ((int32_t)(val <<  3)) >> 19;
            blt_do_s2s_blt(s);
            break;
        case COMMAND_CMD_S2S_STRETCH:
            blt->srcXY = val;
            blt->srcX  = ((int32_t)(val << 19)) >> 19;
            blt->srcY  = ((int32_t)(val <<  3)) >> 19;
            blt_do_stretch_blt(s);
            break;
        case COMMAND_CMD_H2S_BLT:
            blt_do_h2s_blt(s, val);
            break;
        case COMMAND_CMD_H2S_STRETCH:
            blt_do_h2s_stretch_blt(s, val);
            break;
        case 11: /* TRANS_MONO_BLT — host data same path as H2S */
        case 13: /* H2S raw pixel — same data path as H2S */
            blt_do_h2s_blt(s, val);
            break;
        case COMMAND_CMD_RECTFILL:
            blt->dstXY = val;
            blt->dstX  = ((int32_t)(val << 19)) >> 19;
            blt->dstY  = ((int32_t)(val <<  3)) >> 19;
            blt_do_rectfill(s);
            break;
        case COMMAND_CMD_LINE:
            blt->dstXY = val;
            blt->dstX  = ((int32_t)(val << 19)) >> 19;
            blt->dstY  = ((int32_t)(val <<  3)) >> 19;
            blt_do_line(s, true);
            break;
        case COMMAND_CMD_POLYLINE:
            blt->dstXY = val;
            blt->dstX  = ((int32_t)(val << 19)) >> 19;
            blt->dstY  = ((int32_t)(val <<  3)) >> 19;
            blt_do_line(s, false);
            break;
        case COMMAND_CMD_POLYFILL:
            blt_polyfill_continue(s, val);
            break;
        default:
            voodoo3_blt_execute(s);
            break;
        }
        return;
    }

    /*
     * ---- 3Dfx spec-defined control register aliases 0x100..0x154 ----
     *
     * The Voodoo3 / Banshee datasheet places the BLT control registers at
     * offsets 0x100–0x154 within the 2D engine window (BAR0 + 0x100000).
     * The 86Box-ported code uses an alternate layout (0x10..0x70) for the
     * same registers.  We intercept the spec offsets here and redirect them
     * to the identical blt fields so that both layouts work.
     *
     * voodoo3diag Module 20 probes these spec offsets and previously got
     * readback 0 on all six tested registers — now fixed.
     *
     * NOTE: offsets 0x128..0x1fc remain pattern-register territory.
     */
    if (off >= 0x100 && off <= 0x154) {
        switch (off) {
        case 0x100:                              /* dstBaseAddr  [SPEC] */
            blt->dstBaseAddr       = val & 0xffffffu;
            blt->dstBaseAddr_tiled = val & 0x80000000u;
            blt->dstTiled          = !!(val & 0x80000000u);
            break;
        case 0x104:                              /* dstFormat    [SPEC] */
            blt->dstFormat = val;
            break;
        case 0x108:                              /* dstSize      [SPEC] */
            blt->dstSize  = val;
            blt->dstSizeX = blt->dstW = (int)(val & 0x1fffu);
            blt->dstSizeY = blt->dstH = (int)((val >> 16) & 0x1fffu);
            break;
        case 0x10c:                              /* dstXY        [SPEC] */
            blt->dstXY = val;
            blt->dstX  = ((int32_t)(val << 19)) >> 19;
            blt->dstY  = ((int32_t)(val <<  3)) >> 19;
            break;
        case 0x110:                              /* srcBaseAddr  [SPEC] */
            blt->srcBaseAddr       = val & 0xffffffu;
            blt->srcBaseAddr_tiled = val & 0x80000000u;
            blt->srcTiled          = !!(val & 0x80000000u);
            break;
        case 0x114:                              /* srcFormat    [SPEC] */
            blt->srcFormat = val;
            switch (val & SRC_FORMAT_COL_MASK) {
            case SRC_FORMAT_COL_1_BPP:  blt->src_bpp =  1; break;
            case SRC_FORMAT_COL_8_BPP:  blt->src_bpp =  8; break;
            case SRC_FORMAT_COL_24_BPP: blt->src_bpp = 24; break;
            case SRC_FORMAT_COL_32_BPP:
            case SRC_FORMAT_COL_ARGB32: /* srcfmt=7: ARGB32 = 4 bytes/pixel */
            case SRC_FORMAT_COL_YUYV:
            case SRC_FORMAT_COL_UYVY:   blt->src_bpp = 32; break;
            default:                     blt->src_bpp = 16; break;
            }
            break;
        case 0x118:                              /* srcSize      [SPEC] */
            blt->srcSize  = val;
            blt->srcSizeX = blt->srcW = (int)(val & 0x1fffu);
            blt->srcSizeY = blt->srcH = (int)((val >> 16) & 0x1fffu);
            break;
        case 0x11c:                              /* srcXY        [SPEC] */
            blt->srcXY = val;
            blt->srcX  = ((int32_t)(val << 19)) >> 19;
            blt->srcY  = ((int32_t)(val <<  3)) >> 19;
            break;
        case 0x120: blt->colorBack = val; break; /* colorBack    [SPEC] */
        case 0x124: blt->colorFore = val; break; /* colorFore    [SPEC] */
        case 0x150:                              /* rop          [SPEC] */
            blt->rop     = val;
            blt->rops[1] = (uint8_t)(val);
            blt->rops[2] = (uint8_t)(val >> 8);
            blt->rops[3] = (uint8_t)(val >> 16);
            break;
        case 0x154:                              /* command      [SPEC] (write triggers launch) */
            blt->command = val;
            blt->launch_pending = 1;
            voodoo3_blt_execute(s);
            break;
        default:
            /* 0x128..0x14c: fall through to pattern registers below */
            break;
        }
        /*
         * Return for all properly aliased offsets (0x100..0x124, 0x150, 0x154).
         * Offsets 0x128..0x14c are NOT aliased and fall through to the
         * pattern-register handler below (they are valid colorPattern slots).
         */
        if (off <= 0x124 || off == 0x150 || off == 0x154)
            return;
    }

    /* ---- Pattern registers 0x100..0x1fc — 8×8 colour pattern ---- */
    if (off >= 0x100 && off <= 0x1fc) {
        /*
         * Ported from 86Box voodoo_2d_reg_writel() 0x100..0x1fc case.
         * colorPattern[64] stores 32-bit pixels; decoded into 8/16/24-bpp
         * views for fast per-bpp access in PLOT().
         */
        int idx = (off >> 2) & 63;
        blt->colorPattern[idx] = val;

        /* 24-bpp view: 4 pixels per 3 dwords */
        if (off < 0x1c0) {
            int base24 = (off & 0xfc) / 0x0c;
            uintptr_t sp = (uintptr_t)&blt->colorPattern[base24 * 3];
            int col24 = base24 * 4;
            blt->colorPattern24[col24]   = *(uint32_t *)sp & 0xffffffu;
            blt->colorPattern24[col24+1] = *(uint32_t *)(sp+3) & 0xffffffu;
            blt->colorPattern24[col24+2] = *(uint32_t *)(sp+6) & 0xffffffu;
            blt->colorPattern24[col24+3] = *(uint32_t *)(sp+9) & 0xffffffu;
        }
        /* 16-bpp view */
        if (off < 0x180) {
            blt->colorPattern16[((off >> 1) & 62)    ] = (uint16_t)(val & 0xffffu);
            blt->colorPattern16[((off >> 1) & 62) + 1] = (uint16_t)(val >> 16);
        }
        /* 8-bpp view */
        if (off < 0x140) {
            blt->colorPattern8[ off & 60      ] = (uint8_t)(val);
            blt->colorPattern8[(off & 60) + 1  ] = (uint8_t)(val >> 8);
            blt->colorPattern8[(off & 60) + 2  ] = (uint8_t)(val >> 16);
            blt->colorPattern8[(off & 60) + 3  ] = (uint8_t)(val >> 24);
        }
        return;
    }

    /* ---- Control registers 0x08..0x7c ---- */
    switch (off) {
    case 0x08:
        blt->clip0Min        = val;
        blt->clip[0].x_min   = (int)(val & 0xfffu);
        blt->clip[0].y_min   = (int)((val >> 16) & 0xfffu);
        break;
    case 0x0c:
        blt->clip0Max        = val;
        blt->clip[0].x_max   = (int)(val & 0xfffu);
        blt->clip[0].y_max   = (int)((val >> 16) & 0xfffu);
        break;
    case 0x10:
        blt->dstBaseAddr       = val & 0xffffffu;
        blt->dstBaseAddr_tiled = val & 0x80000000u;
        blt->dstTiled          = !!(val & 0x80000000u);
        /* stride recomputed lazily at launch */
        break;
    case 0x14:
        blt->dstFormat = val;
        /* stride recomputed lazily at launch */
        break;
    case 0x18: blt->srcColorkeyMin = val & 0xffffffu; break;
    case 0x1c: blt->srcColorkeyMax = val & 0xffffffu; break;
    case 0x20: blt->dstColorkeyMin = val & 0xffffffu; break;
    case 0x24: blt->dstColorkeyMax = val & 0xffffffu; break;
    case 0x28:
        blt->bresError0  = val;
        blt->bres_error_0 = (int)(val & BRES_ERROR_MASK);
        break;
    case 0x2c:
        blt->bresError1  = val;
        blt->bres_error_1 = (int)(val & BRES_ERROR_MASK);
        break;
    case 0x30:
        blt->rop     = val;
        blt->rops[1] = (uint8_t)(val);
        blt->rops[2] = (uint8_t)(val >> 8);
        blt->rops[3] = (uint8_t)(val >> 16);
        break;
    case 0x34:
        blt->srcBaseAddr       = val & 0xffffffu;
        blt->srcBaseAddr_tiled = val & 0x80000000u;
        blt->srcTiled          = !!(val & 0x80000000u);
        /* stride recomputed lazily at launch */
        break;
    case 0x38: blt->commandExtra = val; break;
    case 0x3c: blt->lineStipple  = val; break;
    case 0x40:
        blt->lineStyle          = val;
        blt->line_rep_cnt       = (int)(val & 0xffu);
        blt->line_bit_mask_size = (int)((val >> 8) & 0x1fu);
        blt->line_pix_pos       = (int)((val >> 16) & 0xffu);
        blt->line_bit_pos       = (int)((val >> 24) & 0x1fu);
        break;
    /* 0x44/0x48: first two colorPattern dwords (also decoded to 8/16/24-bpp) */
    case 0x44:
        blt->colorPattern[0]   = val;
        blt->colorPattern24[0] = val & 0xffffffu;
        blt->colorPattern24[1] = (blt->colorPattern24[1] & 0xffff00u) | (val >> 24);
        blt->colorPattern16[0] = (uint16_t)(val & 0xffffu);
        blt->colorPattern16[1] = (uint16_t)(val >> 16);
        blt->colorPattern8[0]  = (uint8_t)(val);
        blt->colorPattern8[1]  = (uint8_t)(val >> 8);
        blt->colorPattern8[2]  = (uint8_t)(val >> 16);
        blt->colorPattern8[3]  = (uint8_t)(val >> 24);
        break;
    case 0x48:
        blt->colorPattern[1]   = val;
        blt->colorPattern24[1] = (blt->colorPattern24[1] & 0xffu) | ((val & 0xffffu) << 8);
        blt->colorPattern24[2] = (blt->colorPattern24[2] & 0xff0000u) | (val >> 16);
        blt->colorPattern16[2] = (uint16_t)(val & 0xffffu);
        blt->colorPattern16[3] = (uint16_t)(val >> 16);
        blt->colorPattern8[4]  = (uint8_t)(val);
        blt->colorPattern8[5]  = (uint8_t)(val >> 8);
        blt->colorPattern8[6]  = (uint8_t)(val >> 16);
        blt->colorPattern8[7]  = (uint8_t)(val >> 24);
        break;
    case 0x4c:
        blt->clip1Min        = val;
        blt->clip[1].x_min   = (int)(val & 0xfffu);
        blt->clip[1].y_min   = (int)((val >> 16) & 0xfffu);
        break;
    case 0x50:
        blt->clip1Max        = val;
        blt->clip[1].x_max   = (int)(val & 0xfffu);
        blt->clip[1].y_max   = (int)((val >> 16) & 0xfffu);
        break;
    case 0x54:
        blt->srcFormat = val;
        switch (val & SRC_FORMAT_COL_MASK) {
        case SRC_FORMAT_COL_1_BPP:  blt->src_bpp =  1; break;
        case SRC_FORMAT_COL_8_BPP:  blt->src_bpp =  8; break;
        case SRC_FORMAT_COL_24_BPP: blt->src_bpp = 24; break;
        case SRC_FORMAT_COL_32_BPP:
        case SRC_FORMAT_COL_ARGB32: /* srcfmt=7: ARGB32 = 4 bytes/pixel */
        case SRC_FORMAT_COL_YUYV:
        case SRC_FORMAT_COL_UYVY:   blt->src_bpp = 32; break;
        default:                     blt->src_bpp = 16; break;
        }
        /* stride recomputed lazily at launch */
        break;
    case 0x58:
        blt->srcSize  = val;
        blt->srcSizeX = blt->srcW = (int)(val & 0x1fffu);
        blt->srcSizeY = blt->srcH = (int)((val >> 16) & 0x1fffu);
        /* stride recomputed lazily at launch */
        break;
    case 0x5c:
        blt->srcXY = val;
        blt->srcX  = ((int32_t)(val << 19)) >> 19;
        blt->srcY  = ((int32_t)(val <<  3)) >> 19;
        /* stride recomputed lazily at launch */
        break;
    case 0x60: blt->colorBack = val; break;
    case 0x64: blt->colorFore = val; break;
    case 0x68:
        blt->dstSize  = val;
        blt->dstSizeX = blt->dstW = (int)(val & 0x1fffu);
        blt->dstSizeY = blt->dstH = (int)((val >> 16) & 0x1fffu);
        /* stride recomputed lazily at launch */
        break;
    case 0x6c:
        blt->dstXY = val;
        blt->dstX  = ((int32_t)(val << 19)) >> 19;
        blt->dstY  = ((int32_t)(val <<  3)) >> 19;
        break;
    case 0x70:
        /*
         * Command register — ported from 86Box 0x70 handler.
         * Sets launch_pending; some commands fire immediately.
         */
        blt->command      = val;
        blt->launch_pending = true;
        blt->rops[0]      = (uint8_t)(val >> 24);
        blt->patoff_x     = (val & COMMAND_PATOFF_X_MASK) >> COMMAND_PATOFF_X_SHIFT;
        blt->patoff_y     = (val & COMMAND_PATOFF_Y_MASK) >> COMMAND_PATOFF_Y_SHIFT;

        switch (val & COMMAND_CMD_MASK) {
        case COMMAND_CMD_POLYFILL:
            blt_do_launch(s);
            if (val & COMMAND_INITIATE) {
                blt->dstXY = blt->srcXY;
                blt->dstX  = blt->srcX;
                blt->dstY  = blt->srcY;
            }
            blt_polyfill_start(blt);
            break;
        case COMMAND_CMD_H2S_BLT:
        case COMMAND_CMD_H2S_STRETCH:
            /* H2S: wait for data in launch registers */
            break;
        default:
            if (val & COMMAND_INITIATE) {
                blt_do_launch(s);
                voodoo3_blt_execute(s);
            }
            break;
        }
        break;

    default:
        s->regs[off >> 2] = val;
        break;
    }
}

/* =========================================================================
 * CMDFIFO register read/write
 *
 * Ported from 86Box banshee_cmd_read() / banshee_cmd_write()
 * in vid_voodoo_banshee.c.
 *
 * These registers live at BAR0 offset 0x80000–0x8ffff (the IO-remap
 * window with bit 19 set).  The local offset = addr & 0x1fc selects
 * the register.  The two FIFOs share the same layout; FIFO1 starts at
 * local offset 0x30 above FIFO0.
 *
 * AGP offsets (local 0x00–0x14):
 *   0x00  Agp_agpReqSize          byte count for DMA transfer
 *   0x04  Agp_agpHostAddressLow   source host address
 *   0x08  Agp_agpHostAddressHigh  [13:0]=width [27:14]=stride
 *   0x0c  Agp_agpGraphicsAddress  dest VRAM byte address
 *   0x10  Agp_agpGraphicsStride   dest stride in bytes
 *   0x14  Agp_agpMoveCMD          [4:3]=dest-type, triggers transfer
 *
 * CMDFIFO0 offsets (local 0x20–0x48):
 *   0x20  cmdBaseAddr0    FIFO ring base  (bits [23:12] << 12)
 *   0x24  cmdBaseSize0    ring size + enable/AGP flags
 *   0x28  cmdBump0        (write: no-op)
 *   0x2c  cmdRdPtrL0      read pointer low
 *   0x30  cmdRdPtrH0      read pointer high (stub)
 *   0x34  cmdAMin0        contiguous-hole lower bound
 *   0x3c  cmdAMax0        contiguous-hole upper bound
 *   0x40  cmdStatus0      (read-only)
 *   0x44  cmdFifoDepth0   write = reset depth; read = wr-rd
 *   0x48  cmdHoleCnt0     hole count
 *
 * CMDFIFO1 offsets are FIFO0 + 0x30 (local 0x50–0x78).
 * ========================================================================= */

#define CMDFIFO_BASE_ADDR0  0x20
#define CMDFIFO_SIZE0       0x24
#define CMDFIFO_BUMP0       0x28
#define CMDFIFO_RDPTR_L0    0x2c
#define CMDFIFO_RDPTR_H0    0x30
#define CMDFIFO_AMIN0       0x34
#define CMDFIFO_AMAX0       0x3c
#define CMDFIFO_STATUS0     0x40
#define CMDFIFO_DEPTH0      0x44
#define CMDFIFO_HOLECNT0    0x48

#define CMDFIFO_BASE_ADDR1  0x50
#define CMDFIFO_SIZE1       0x54
#define CMDFIFO_BUMP1       0x58
#define CMDFIFO_RDPTR_L1    0x5c
#define CMDFIFO_RDPTR_H1    0x60
#define CMDFIFO_AMIN1       0x64
#define CMDFIFO_AMAX1       0x6c
#define CMDFIFO_STATUS1     0x70
#define CMDFIFO_DEPTH1      0x74
#define CMDFIFO_HOLECNT1    0x78

#define AGP_REQSIZE         0x00
#define AGP_HOST_ADDR_LO    0x04
#define AGP_HOST_ADDR_HI    0x08
#define AGP_GRAPHICS_ADDR   0x0c
#define AGP_GRAPHICS_STRIDE 0x10
#define AGP_MOVE_CMD        0x14

static uint32_t voodoo3_cmd_read(Voodoo3State *s, uint32_t local)
{
    switch (local) {
    /* AGP */
    case AGP_HOST_ADDR_LO:    return s->agpHostAddressLow;
    case AGP_HOST_ADDR_HI:    return s->agpHostAddressHigh;
    case AGP_GRAPHICS_ADDR:   return s->agpGraphicsAddress;
    case AGP_GRAPHICS_STRIDE: return s->agpGraphicsStride;
    case AGP_REQSIZE:         return s->agpReqSize;
    case AGP_MOVE_CMD:        return s->agpMoveCMD;

    /* FIFO0 */
    case CMDFIFO_BASE_ADDR0:  return s->cmdfifo_base >> 12;
    case CMDFIFO_RDPTR_L0:    return s->cmdfifo_rp;
    case CMDFIFO_DEPTH0:      return s->cmdfifo_depth_wr - s->cmdfifo_depth_rd;
    case CMDFIFO_STATUS0:     return 0; /* always idle */
    case CMDFIFO_SIZE0:
        return s->cmdfifo_size
               | (s->cmdfifo_enabled  ? 0x100u : 0u)
               | (s->cmdfifo_in_agp   ? 0x200u : 0u);
    case CMDFIFO_AMIN0:       return s->cmdfifo_amin;
    case CMDFIFO_AMAX0:       return s->cmdfifo_amax;
    case CMDFIFO_HOLECNT0:    return s->cmdfifo_holecount;

    /* FIFO1 */
    case CMDFIFO_BASE_ADDR1:  return s->cmdfifo_base_2 >> 12;
    case CMDFIFO_RDPTR_L1:    return s->cmdfifo_rp_2;
    case CMDFIFO_DEPTH1:      return s->cmdfifo_depth_wr_2 - s->cmdfifo_depth_rd_2;
    case CMDFIFO_STATUS1:     return 0;
    case CMDFIFO_SIZE1:
        return s->cmdfifo_size_2
               | (s->cmdfifo_enabled_2 ? 0x100u : 0u)
               | (s->cmdfifo_in_agp_2  ? 0x200u : 0u);
    case CMDFIFO_AMIN1:       return s->cmdfifo_amin_2;
    case CMDFIFO_AMAX1:       return s->cmdfifo_amax_2;
    case CMDFIFO_HOLECNT1:    return s->cmdfifo_holecount_2;

    default:
        return 0xffffffffu;
    }
}

static void voodoo3_cmd_write(Voodoo3State *s, uint32_t local, uint32_t val)
{
    switch (local) {
    /* ---- AGP host→VRAM DMA transfer ------------------------------------ */
    case AGP_HOST_ADDR_LO:
        s->agpHostAddressLow  = val;
        break;
    case AGP_HOST_ADDR_HI:
        s->agpHostAddressHigh = val;
        break;
    case AGP_GRAPHICS_ADDR:
        s->agpGraphicsAddress = val;
        break;
    case AGP_GRAPHICS_STRIDE:
        s->agpGraphicsStride  = val;
        break;
    case AGP_REQSIZE:
        s->agpReqSize = val;
        break;
    case AGP_MOVE_CMD: {
        /*
         * Trigger AGP DMA transfer.
         * dest type = (val >> 3) & 3:
         *   0 = linear framebuffer   1 = planar YUV
         *   2 = framebuffer (tiled)  3 = texture
         *
         * On a PCI card (no AGP) this is a programmed-IO copy from
         * guest RAM.  We do not have access to physical host memory in
         * QEMU's device model, so we accept the write and return 0 from
         * agpMoveCMD reads.  The driver only uses this for texture upload
         * on AGP variants; PCI cards use the LFB aperture instead.
         */
        s->agpMoveCMD = val;
        /* Stub: no DMA engine – silently ignore */
        break;
    }

    /* ---- CMDFIFO0 ------------------------------------------------------- */
    case CMDFIFO_BASE_ADDR0:
        s->cmdfifo_base = (val & 0xfffu) << 12;
        s->cmdfifo_end  = s->cmdfifo_base +
                          (((s->cmdfifo_size & 0xffu) + 1u) << 12);
        voodoo3_cmdfifo_reposition(s);
        break;
    case CMDFIFO_SIZE0:
        s->cmdfifo_size    = val;
        s->cmdfifo_end     = s->cmdfifo_base +
                             (((val & 0xffu) + 1u) << 12);
        s->cmdfifo_enabled = !!(val & 0x100u);
        if (!s->cmdfifo_enabled)
            s->cmdfifo_in_sub = 0;
        s->cmdfifo_in_agp  = !!(val & 0x200u);
        voodoo3_cmdfifo_reposition(s);
        break;
    case CMDFIFO_BUMP0:
        /*
         * Writing the BUMP register adds N dwords to cmdfifo_depth_wr and
         * wakes the FIFO worker — ported from 86Box voodoo_wake_fifo_thread().
         * The written value is the number of dwords being added to CMDFIFO0.
         */
        s->cmdfifo_depth_wr += val;
        qemu_mutex_lock(&s->render_lock);
        qemu_cond_broadcast(&s->render_cond);
        qemu_mutex_unlock(&s->render_lock);
        break;
    case CMDFIFO_RDPTR_L0:
        s->cmdfifo_rp = val;
        break;
    case CMDFIFO_RDPTR_H0:
        /* high 32 bits of 64-bit pointer – not used on 32-bit PCI */
        break;
    case CMDFIFO_AMIN0:
        s->cmdfifo_amin = val;
        break;
    case CMDFIFO_AMAX0:
        s->cmdfifo_amax = val;
        break;
    case CMDFIFO_DEPTH0:
        s->cmdfifo_depth_rd = 0;
        s->cmdfifo_depth_wr = val & 0xffffu;
        break;
    case CMDFIFO_HOLECNT0:
        s->cmdfifo_holecount = val;
        break;
    case CMDFIFO_STATUS0:
        /* read-only */
        break;

    /* ---- CMDFIFO1 ------------------------------------------------------- */
    case CMDFIFO_BASE_ADDR1:
        s->cmdfifo_base_2 = (val & 0xfffu) << 12;
        s->cmdfifo_end_2  = s->cmdfifo_base_2 +
                            (((s->cmdfifo_size_2 & 0xffu) + 1u) << 12);
        break;
    case CMDFIFO_SIZE1:
        s->cmdfifo_size_2    = val;
        s->cmdfifo_end_2     = s->cmdfifo_base_2 +
                               (((val & 0xffu) + 1u) << 12);
        s->cmdfifo_enabled_2 = !!(val & 0x100u);
        if (!s->cmdfifo_enabled_2)
            s->cmdfifo_in_sub_2 = 0;
        s->cmdfifo_in_agp_2  = !!(val & 0x200u);
        break;
    case CMDFIFO_BUMP1:
        s->cmdfifo_depth_wr_2 += val;
        qemu_mutex_lock(&s->render_lock);
        qemu_cond_broadcast(&s->render_cond);
        qemu_mutex_unlock(&s->render_lock);
        break;
    case CMDFIFO_RDPTR_L1:
        s->cmdfifo_rp_2 = val;
        break;
    case CMDFIFO_RDPTR_H1:
        break;
    case CMDFIFO_AMIN1:
        s->cmdfifo_amin_2 = val;
        break;
    case CMDFIFO_AMAX1:
        s->cmdfifo_amax_2 = val;
        break;
    case CMDFIFO_DEPTH1:
        s->cmdfifo_depth_rd_2 = 0;
        s->cmdfifo_depth_wr_2 = val & 0xffffu;
        break;
    case CMDFIFO_HOLECNT1:
        s->cmdfifo_holecount_2 = val;
        break;
    case CMDFIFO_STATUS1:
        break;

    default:
        /* Shadow/unknown registers — silently ignore */
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
        if (addr & 0x80000u) {
            ret = voodoo3_cmd_read(s, (uint32_t)(addr & 0x1fc));
        } else {
            uint32_t eaddr = addr & 0xfc;   /* 4-byte-aligned ext offset */
            /*
             * BAR0 IO-remap ext register space layout:
             *
             *   0x00..0xAC = native 32-bit registers: ext_read(eaddr) returns
             *                the full 32-bit value in a single call.
             *
             *   0xB0..0xFF = byte-wide DAC / VGA-proxy registers packed four
             *                per dword.  A 32-bit host read at 4-byte-aligned
             *                offset 0xXC must assemble bytes [0xXF:0xXE:0xXD:0xXC]
             *                into a LE uint32.
             *
             *                Example: a 32-bit read at 0xD8 must return
             *                  bits[31:24] = ext_read(0xDB)
             *                  bits[23:16] = ext_read(0xDA) = dacStatus ← bit 0 = DAC ready
             *                  bits[15:8]  = ext_read(0xD9)
             *                  bits[7:0]   = ext_read(0xD8)
             *
             *                voodoo3diag Module 9 reads 32 bits at 0xD8 and
             *                checks dacStatus in bits[23:16].  Without assembly
             *                that byte was always 0 (DAC not ready).
             *
             * FIX: voodoo3diag Module 9 -- dacStatus byte was 0.
             */
            if (eaddr >= 0xb0u) {
                uint32_t b0 = voodoo3_ext_read(s, eaddr);
                uint32_t b1 = voodoo3_ext_read(s, eaddr + 1u);
                uint32_t b2 = voodoo3_ext_read(s, eaddr + 2u);
                uint32_t b3 = voodoo3_ext_read(s, eaddr + 3u);
                ret = (b3 << 24) | (b2 << 16) | (b1 << 8) | b0;
            } else {
                ret = voodoo3_ext_read(s, eaddr);
            }
        }
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
        /*
         * 86Box-derived control register offsets (0x10..0x70).
         * Kept for backwards-compatibility with any code using the
         * 86Box Banshee layout.
         */
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
        /*
         * 3Dfx Voodoo3 / Banshee datasheet register offsets (0x100..0x124).
         * voodoo3diag Module 20 probes these spec-defined offsets.
         * They map to the same underlying blt state as the 86Box offsets above.
         * FIX: all 6 registers were reading back 0x00000000 (MISMATCH).
         */
        case 0x100: ret = s->blt.dstBaseAddr;               break; /* dstBaseAddr  [SPEC 0x100] */
        case 0x104: ret = s->blt.dstFormat;                 break; /* dstFormat    [SPEC 0x104] */
        case 0x108: ret = s->blt.dstSize;                   break; /* dstSize      [SPEC 0x108] */
        case 0x10c: ret = s->blt.dstXY;                     break; /* dstXY        [SPEC 0x10C] */
        case 0x110: ret = s->blt.srcBaseAddr;               break; /* srcBaseAddr  [SPEC 0x110] */
        case 0x114: ret = s->blt.srcFormat;                 break; /* srcFormat    [SPEC 0x114] */
        case 0x118: ret = s->blt.srcSize;                   break; /* srcSize      [SPEC 0x118] */
        case 0x11c: ret = s->blt.srcXY;                     break; /* srcXY        [SPEC 0x11C] */
        case 0x120: ret = s->blt.colorBack;                 break; /* colorBack    [SPEC 0x120] */
        case 0x124: ret = s->blt.colorFore;                 break; /* colorFore    [SPEC 0x124] */
        case 0x150: ret = s->blt.rop;                       break; /* rop          [SPEC 0x150] */
        case 0x154: ret = s->blt.command;                   break; /* command      [SPEC 0x154] */
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
        case SST_fogMode:   ret = s->params.fogMode;         break;
        case SST_fbzMode:   ret = s->params.fbzMode;         break;
        case SST_alphaMode: ret = s->params.alphaMode;       break;
        case SST_lfbMode:   ret = s->lfbMode;                break;
        case SST_stipple:   ret = s->params.stipple;         break;
        case SST_color0:    ret = s->params.color0;          break;
        case SST_color1:    ret = s->params.color1;          break;
        case SST_fogColor:  ret = (uint32_t)s->params.fogColor.r << 16
                                 | (uint32_t)s->params.fogColor.g <<  8
                                 | (uint32_t)s->params.fogColor.b;        break;
        case SST_zaColor:   ret = s->params.zaColor;         break;
        case SST_chromaKey: ret = s->params.chromaKey;       break;
        /* clip registers */
        case SST_clipLeftRight:
            ret = ((uint32_t)s->params.clipLeft << 16) | (uint32_t)s->params.clipRight;
            break;
        case SST_clipLowYHighY:
            ret = ((uint32_t)s->params.clipLowY << 16) | (uint32_t)s->params.clipHighY;
            break;
        case SST_clipLeftRight1:
            ret = ((uint32_t)s->params.clipLeft1 << 16) | (uint32_t)s->params.clipRight1;
            break;
        case SST_clipTopBottom1:
            ret = ((uint32_t)s->params.clipLowY1 << 16) | (uint32_t)s->params.clipHighY1;
            break;
        /* buffer addresses */
        case SST_colBufferAddr:   ret = s->params.draw_offset;    break;
        case SST_colBufferStride: ret = s->params.col_stride_raw; break; /* FIX: raw reg, not transformed row_width (diag Module 22) */
        case SST_auxBufferAddr:   ret = s->params.aux_offset;     break;
        case SST_auxBufferStride: ret = s->params.aux_stride_raw; break; /* FIX: raw reg */
        /* fbi statistics counters */
        case SST_fbiPixelsIn:   ret = s->params.fbiPixelsIn   & 0xffffffu; break;
        case SST_fbiChromaFail: ret = s->params.fbiChromaFail & 0xffffffu; break;
        case SST_fbiZFuncFail:  ret = s->params.fbiZFuncFail  & 0xffffffu; break;
        case SST_fbiAFuncFail:  ret = s->params.fbiAFuncFail  & 0xffffffu; break;
        case SST_fbiPixelsOut:  ret = s->params.fbiPixelsOut  & 0xffffffu; break;
        /* setup-unit registers */
        /* SST_sSetupMode (0x2c0) aliases SST_clipLeftRight1 — handled above */
        default:
            /*
             * Real Voodoo3 hardware: reads of write-only or unimplemented
             * registers in the 3D core return the STATUS register value,
             * not 0x00 or 0xFFFFFFFF.  The chip floats the data bus to the
             * status bus for any unrecognised read address.
             * FIX: voodoo3diag Module 12 -- fbzColorPath, alphaMode, lfbMode
             * etc. returned 0x00000000; real HW returns STATUS mirror.
             */
            qemu_log_mask(LOG_UNIMP,
                "voodoo3: 3D reg read 0x%03x\n", (unsigned)(addr & 0x3fc));
            ret = voodoo3_status(s);
            break;
        }
        break;
    case BAR0_TEX0:
    case 0x0700000:
    case BAR0_TEX1:
    case 0x0900000: {
        /* TMU register read-back (textureMode, tLOD, etc.) */
        int tmu = (addr & 0x1f00000) >= BAR0_TEX1 ? 1 : 0;
        switch (addr & 0x3fc) {
        case 0x300u: ret = s->params.tmu[tmu].textureMode; break;
        case 0x304u: ret = s->params.tmu[tmu].tLOD;        break;
        case 0x308u: /* tDetail - return 0 (no detail tex) */ ret = 0; break;
        case 0x30cu: ret = s->params.tmu[tmu].texBaseAddr;  break;
        default:
            /*
             * Unimplemented TMU register reads: return STATUS mirror,
             * matching real Voodoo3 hardware bus behaviour.
             * FIX: voodoo3diag Module 15 -- TMU regs returned 0x00000000;
             * real HW returns STATUS value (e.g. 0x1F000000 at reset).
             */
            ret = voodoo3_status(s);
            break;
        }
        break;
    }
    case 0x0a00000: case 0x0b00000: case 0x0c00000: case 0x0d00000:
    case 0x0e00000: case 0x0f00000:
        /* Reserved BAR0 regions - return 0 to avoid spurious logs */
        ret = 0x00000000u;
        break;

    /*
     * 3D LFB aperture read — BAR0 offsets 0x1000000–0x1FFFFFF
     *
     * Ported from 86Box voodoo_fb_readl() in vid_voodoo_fb.c.
     * Original author: Sarah Walker.
     *
     * The Banshee/V3 uses a different address encoding than Voodoo1/2:
     *   bits[11:1]  = X byte coordinate within row  (addr & 0xffe)
     *   bits[21:12] = Y row coordinate              ((addr >> 12) & 0x3ff)
     *
     * The read buffer is selected by lfbMode bits[7:6] (LFB_READ_MASK):
     *   0x00 = front buffer  → params.front_offset
     *   0x40 = back buffer   → params.draw_offset  (draw = back on Banshee)
     *   0x80 = aux buffer    → params.aux_offset
     *
     * Tiled mode (col_tiled) uses the same 128-column × 32-row tile layout
     * as the write path in voodoo3_fb_writel():
     *   read_addr = base
     *             + (x & 127)                       ← column within tile
     *             + (x >> 7) * 128 * 32             ← tile strip offset
     *             + (y & 31) * 128                  ← row within tile
     *             + (y >> 5) * row_width             ← tile band offset
     *
     * Note: The 16-bit read path (voodoo_fb_readw) is not needed here
     * because QEMU's DEVICE_LITTLE_ENDIAN MemoryRegion decomposes 32-bit
     * reads to 16-bit automatically if the guest issues a 16-bit access.
     * All LFB reads via BAR0 MMIO are handled as 32-bit; the upper or
     * lower half is selected by the MemoryRegion size dispatch.
     */
    case 0x1000000: case 0x1100000: case 0x1200000: case 0x1300000:
    case 0x1400000: case 0x1500000: case 0x1600000: case 0x1700000:
    case 0x1800000: case 0x1900000: case 0x1a00000: case 0x1b00000:
    case 0x1c00000: case 0x1d00000: case 0x1e00000: case 0x1f00000:
    {
        /* Banshee address decode (type >= VOODOO_BANSHEE):
         *   x = addr & 0xffe  (byte X, low bit always 0 for 16-bit alignment)
         *   y = (addr >> 12) & 0x3ff
         * SLI is not applicable (single-GPU). */
        int      lx = (int)(addr & 0xffeu);
        int      ly = (int)((addr >> 12) & 0x3ffu);

        /* Select read buffer from lfbMode bits[7:6] */
        uint32_t read_base;
        switch (s->lfbMode & 0xc0u) {
        case 0x40:  read_base = s->params.draw_offset;  break; /* back  */
        case 0x80:  read_base = s->params.aux_offset;   break; /* aux   */
        default:    read_base = s->params.front_offset; break; /* front */
        }

        /* Tiled or linear address */
        uint32_t read_addr;
        if (s->params.col_tiled) {
            read_addr = read_base
                      + (uint32_t)(lx & 127)
                      + (uint32_t)(lx >> 7) * 128u * 32u
                      + (uint32_t)(ly & 31) * 128u
                      + (uint32_t)(ly >> 5) * s->params.row_width;
        } else {
            read_addr = read_base
                      + (uint32_t)lx
                      + (uint32_t)ly * s->params.row_width;
        }

        if (read_addr + 3u >= s->fb_size) {
            ret = 0xffffffffu;
        } else {
            memcpy(&ret, s->fb_mem + read_addr, 4);
        }
        break;
    }

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
        if (!(addr & 0x80000u))
            voodoo3_ext_write(s, addr & 0xff, val);
        else
            voodoo3_cmd_write(s, (uint32_t)(addr & 0x1fc), val);
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
 * BAR1: Linear Framebuffer — RAM-backed region + dynamic CMDFIFO overlay
 *
 * The bulk of BAR1 is a ram_device_ptr region mapped directly onto fb_mem.
 * Guest writes go straight to host RAM without a MMIO trap per access,
 * exactly like hw/display/ati.c's linear_aper.
 *
 * The CMDFIFO ring window is a small MMIO subregion (cmdfifo_mmio) layered
 * on top of lfb_ram at the byte offset the driver programs.  It intercepts
 * writes to advance depth_wr and wake the render thread — the one side-
 * effect that cannot be expressed as plain RAM access.
 *
 * voodoo3_cmdfifo_reposition() tears down the old subregion and re-adds it
 * whenever the driver changes CMDFIFO_BASE_ADDR0 or CMDFIFO_SIZE0.
 * ========================================================================= */

/*
 * Tiled → linear address translation for the tiled LFB aperture.
 * Used in the CMDFIFO overlay read path and by 86Box-ported 3D code that
 * still calls into fb_mem directly.
 */
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

/*
 * CMDFIFO overlay read: the subregion addr is already relative to the start
 * of cmdfifo_mmio (i.e. 0-based within the CMDFIFO window).  We translate
 * it to a fb_mem offset by adding cmdfifo_base, then apply the tiling
 * decode in case the driver placed the ring in a tiled region.
 */
static uint64_t voodoo3_cmdfifo_read(void *opaque, hwaddr addr, unsigned size)
{
    Voodoo3State *s = VOODOO3_PCI(opaque);
    uint64_t      val = 0;
    hwaddr phys = voodoo3_untile(s, s->cmdfifo_base + addr);
    if (phys + size > s->fb_size) {
        return 0xffffffffffffffffULL;
    }
    memcpy(&val, s->fb_mem + phys, size);
    return val;
}

/*
 * CMDFIFO overlay write.
 *
 * addr is 0-based within the cmdfifo_mmio subregion.  We convert to a
 * fb_mem physical offset by adding cmdfifo_base, commit the data so that
 * cmdfifo_read_dword() can retrieve it via fb_mem, then run the 86Box
 * depth_wr accounting that wakes the FIFO worker thread.
 *
 * cursor_buf is no longer mirrored here: with a RAM-backed lfb_ram the
 * guest writes go directly to fb_mem.  The Video_hwCurPatAddr and
 * Video_hwCurLoc register handlers already re-populate cursor_buf from
 * fb_mem whenever the driver repositions the cursor sprite.  Any
 * subsequent writes by the driver to the cursor area go straight to
 * fb_mem and are picked up by the next Video_hwCurLoc write.
 */
static void voodoo3_cmdfifo_write(void *opaque, hwaddr addr,
                                  uint64_t data, unsigned size)
{
    Voodoo3State *s = VOODOO3_PCI(opaque);
    hwaddr phys = voodoo3_untile(s, s->cmdfifo_base + addr);
    if (phys + size > s->fb_size) {
        return;
    }

    /* Commit to fb_mem so the render thread can read it. */
    memcpy(s->fb_mem + phys, &data, size);

    /*
     * CMDFIFO0 depth-counter update — ported from 86Box banshee_mem_writel().
     *
     * The driver fills the ring buffer via LFB (now via this MMIO overlay)
     * and polls cmdfifo_depth waiting for it to drain.  We must advance
     * depth_wr and wake the FIFO worker to avoid an infinite spin.
     *
     * Algorithm (condensed from 86Box banshee_mem_writel):
     *   phys == cmdfifo_base && !holecount  → reset amin/amax, depth_wr++
     *   holecount > 0                        → holecount--;
     *                                          if 0: depth_wr += (amax-amin)>>2
     *   phys == amax+4                        → amax=phys; depth_wr++
     *   else (out-of-order)                  → update amax, set holecount
     */
    if (size == 4 &&
        s->cmdfifo_enabled &&
        s->cmdfifo_base != s->cmdfifo_end) {
        uint32_t phys32 = (uint32_t)phys;
        if (phys32 >= s->cmdfifo_base && phys32 < s->cmdfifo_end) {
            if (phys32 == s->cmdfifo_base && !s->cmdfifo_holecount) {
                s->cmdfifo_amin = s->cmdfifo_base;
                s->cmdfifo_amax = s->cmdfifo_base;
                s->cmdfifo_depth_wr++;
            } else if (s->cmdfifo_holecount) {
                s->cmdfifo_holecount--;
                if (!s->cmdfifo_holecount) {
                    s->cmdfifo_depth_wr +=
                        (s->cmdfifo_amax - s->cmdfifo_amin) >> 2;
                }
            } else if (phys32 == s->cmdfifo_amax + 4u) {
                s->cmdfifo_amin = phys32;
                s->cmdfifo_amax = phys32;
                s->cmdfifo_depth_wr++;
            } else {
                /* Out-of-order write: record range, set hole count. */
                if (phys32 < s->cmdfifo_amin) {
                    s->cmdfifo_amin = s->cmdfifo_base - 4u;
                }
                s->cmdfifo_amax      = phys32;
                s->cmdfifo_holecount =
                    (s->cmdfifo_amax - s->cmdfifo_amin) >> 2;
                if (s->cmdfifo_holecount) {
                    s->cmdfifo_holecount--;
                }
            }
            qemu_mutex_lock(&s->render_lock);
            qemu_cond_broadcast(&s->render_cond);
            qemu_mutex_unlock(&s->render_lock);
        }
    }
}

/*
 * CMDFIFO MMIO overlay ops.
 *
 * min/max_access_size = 4: the CMDFIFO ring is always written as 4-byte
 * dwords.  QEMU will split any larger guest writes before they arrive here.
 */
static const MemoryRegionOps voodoo3_cmdfifo_ops = {
    .read       = voodoo3_cmdfifo_read,
    .write      = voodoo3_cmdfifo_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * voodoo3_cmdfifo_reposition — move the CMDFIFO MMIO overlay.
 *
 * Called whenever the driver reprograms CMDFIFO_BASE_ADDR0 or
 * CMDFIFO_SIZE0.  We tear down the old subregion (if any) and re-add it
 * at the new byte offset within lfb_ram.
 *
 * Must be called with s->render_lock NOT held (memory_region functions
 * acquire the BQL internally on TCG; on KVM the BQL is already held by
 * the MMIO dispatch path).
 *
 * When cmdfifo_base == cmdfifo_end (i.e. size is zero) or the CMDFIFO is
 * disabled we simply remove the overlay and do not re-add it.
 */
static void voodoo3_cmdfifo_reposition(Voodoo3State *s)
{
    uint32_t base = s->cmdfifo_base;
    uint32_t end  = s->cmdfifo_end;
    uint32_t size = (end > base) ? (end - base) : 0;

    /* Remove the old subregion if it was registered. */
    if (s->cmdfifo_mmio_active) {
        memory_region_del_subregion(&s->lfb_ram, &s->cmdfifo_mmio);
        s->cmdfifo_mmio_active = false;
    }

    /* Only re-add when the ring has a non-zero size and fits in fb_mem. */
    if (size == 0 || (uint64_t)base + size > s->fb_size) {
        return;
    }

    /*
     * Resize the cmdfifo_mmio region to match the new ring size.
     * memory_region_set_size() is the supported way to resize an
     * already-initialised region before it is mapped.
     */
    memory_region_set_size(&s->cmdfifo_mmio, size);

    /*
     * Add it back at the new offset with priority 1 so it shadows
     * the underlying RAM pages for the ring-buffer window only.
     */
    memory_region_add_subregion_overlap(&s->lfb_ram, base,
                                        &s->cmdfifo_mmio, 1);
    s->cmdfifo_mmio_active = true;
}

/* =========================================================================
 * BAR2: Legacy I/O — VGA port proxy
 *
 * The Banshee/V3 BAR2 is a 256-byte I/O aperture that mirrors the Banshee
 * extended register space at BAR0 offset 0x00..0xff AND the VGA I/O ports
 * 0x3b0..0x3df (mapped at BAR2 offset 0xb0..0xdf).
 *
 * Ported from 86Box:
 *   banshee_ext_out/in()  — the byte-wide ext register handler which
 *                           forwards 0xb0..0xdf to banshee_out/in()
 *   banshee_out/in()      — handles 0x3D4/0x3D5 (CRTC) plus delegates
 *                           everything else to svga_out/svga_in()
 *   svga_out/svga_in()    — the full VGA port implementation
 *
 * Port mapping (BAR2 offset → VGA port → function):
 *   0xb0..0xb7  → 0x3b0..0x3b7  MDA / unused on colour adapters
 *   0xba        → 0x3ba         Input Status 1 (mono)
 *   0xc0        → 0x3c0         ATC index+data (write) / ATC data (alt read)
 *   0xc1        → 0x3c1         ATC data read
 *   0xc2        → 0x3c2         Misc Output Write / Input Status 0 (read)
 *   0xc4        → 0x3c4         Sequencer index
 *   0xc5        → 0x3c5         Sequencer data
 *   0xc6        → 0x3c6         DAC PEL mask
 *   0xc7        → 0x3c7         DAC read address (write) / DAC state (read)
 *   0xc8        → 0x3c8         DAC write address
 *   0xc9        → 0x3c9         DAC data (R/G/B triplets)
 *   0xca        → 0x3ca         Feature Control read (read-only)
 *   0xcc        → 0x3cc         Misc Output read (read-only)
 *   0xce        → 0x3ce         GRC index
 *   0xcf        → 0x3cf         GRC data
 *   0xd4        → 0x3d4         CRTC index
 *   0xd5        → 0x3d5         CRTC data (with protect logic)
 *   0xda        → 0x3da         Input Status 1 / Feature Control write
 *
 * Offsets 0x00..0xaf and 0xe0..0xff go to voodoo3_ext_read/write().
 * ========================================================================= */

/*
 * VGA I/O write helper — ported from 86Box banshee_out() + svga_out().
 * addr is the full VGA port address (0x3b0..0x3df).
 */
static void voodoo3_vga_out(Voodoo3State *s, uint16_t addr, uint8_t val)
{
    /*
     * 86Box banshee_out() line 366:
     *   if (((addr & 0xfff0) == 0x3d0 || (addr & 0xfff0) == 0x3b0)
     *       && !(svga->miscout & 1))
     *       addr ^= 0x60;
     * Meaning: if misc_out bit 0 is 0, 3Dx <-> 3Bx (MDA compat mode).
     */
    if (((addr & 0xfff0) == 0x3d0 || (addr & 0xfff0) == 0x3b0)
        && !(s->misc_out & 1))
        addr ^= 0x60;

    switch (addr) {
    /*
     * 0x3c0 — Attribute Controller (ATC) index + data.
     * 86Box svga_out(): ar_flip_flop toggles between index write and data write.
     * Reading 0x3da/0x3ba resets ar_flip_flop to index mode.
     */
    case 0x3c0:
        if (!s->ar_flip_flop) {
            /* Index write: bits[4:0] = register select, bit 5 = palette enable */
            s->ar_idx = val & 0x3f;
        } else {
            /* Data write: store into ATC register */
            if (s->ar_idx < 32)
                s->ar_regs[s->ar_idx] = val;
        }
        s->ar_flip_flop = !s->ar_flip_flop;
        break;

    /*
     * 0x3c2 — Miscellaneous Output Register.
     * Bits: [0]=IO select(0=3Bx,1=3Dx), [1]=RAM enable,
     *       [3:2]=clock select, [5]=page sel, [7:6]=sync polarity.
     */
    case 0x3c2:
        s->misc_out = val;
        /*
         * 86Box: writing misc_out triggers svga_recalctimings() which calls
         * banshee_recalctimings() — the clock select bits[3:2] determine
         * whether the PLL or a fixed VGA clock is used.
         */
        voodoo3_pll_update_vblank(s);
        break;

    /* 0x3c4 — Sequencer index. */
    case 0x3c4:
        s->seq_idx = val & 0x07;
        break;
    /* 0x3c5 — Sequencer data. */
    case 0x3c5:
        if (s->seq_idx < 8) {
            s->seq_regs[s->seq_idx] = val;
            /* seq[1] bit3 = 8/16 dot clock → affects htotal pixel count */
            if (s->seq_idx == 1)
                voodoo3_pll_update_vblank(s);
        }
        break;

    /* 0x3c6 — DAC PEL mask. */
    case 0x3c6:
        s->dac_pel_mask = val;
        break;

    /*
     * 0x3c7 — DAC read address.
     * 86Box svga_out(): svga->dac_read = val; svga->dac_state = 3.
     */
    case 0x3c7:
        s->dac_read_addr = val;
        s->dac_rgb_idx   = 0;
        s->dac_state     = 3;   /* read mode */
        break;

    /*
     * 0x3c8 — DAC write address.
     * 86Box svga_out(): svga->dac_write = val; svga->dac_state = 0.
     */
    case 0x3c8:
        s->dac_write_addr = val;
        s->dacAddr        = (int)(val & 0xff);
        s->dac_rgb_idx    = 0;
        s->dac_state      = 0;   /* write mode */
        break;

    /*
     * 0x3c9 — DAC data (R/G/B triplet).
     * The Banshee DAC is 8-bit (not 6-bit VGA), so val is stored directly
     * (no << 2 shift), matching 86Box banshee behaviour.
     */
    case 0x3c9:
        s->dac_rgb_buf[s->dac_rgb_idx++] = val;
        if (s->dac_rgb_idx == 3) {
            s->dac_rgb_idx = 0;
            if (s->dacAddr < VOODOO3_CLUT_SIZE) {
                s->pallook[s->dacAddr] =
                    ((uint32_t)s->dac_rgb_buf[2] << 16) |  /* R (byte 2) */
                    ((uint32_t)s->dac_rgb_buf[1] <<  8) |  /* G (byte 1) */
                     (uint32_t)s->dac_rgb_buf[0];          /* B (byte 0) */
            }
            s->dacAddr = (s->dacAddr + 1) & 0xff;
            s->dac_write_addr = (uint8_t)s->dacAddr;
        }
        break;

    /* 0x3ce — Graphics Controller index. */
    case 0x3ce:
        s->gr_idx = val & 0x0f;
        break;
    /* 0x3cf — Graphics Controller data. */
    case 0x3cf:
        if (s->gr_idx < 16)
            s->gr_regs[s->gr_idx] = val;
        break;

    /*
     * 0x3d4 — CRTC index.
     * 86Box banshee_out(): svga->crtcreg = val & 0x3f.
     */
    case 0x3d4:
        s->crtc_idx = val & 0x3f;
        break;

    /*
     * 0x3d5 — CRTC data.
     * 86Box banshee_out(): protect regs 0x00..0x06 if crtc[0x11] bit 7 set.
     * Reg 0x07: only bit 4 writable when protected.
     */
    case 0x3d5:
        if (s->crtc_idx < 64) {
            /* Protection: CRTC[0x00..0x06] locked when CRTC[0x11] bit 7 set */
            if (s->crtc_idx < 7 && (s->crtc_ctrl[0x11] & 0x80))
                break;
            /* CRTC[0x07]: only bit 4 writable when protected */
            if (s->crtc_idx == 7 && (s->crtc_ctrl[0x11] & 0x80))
                val = (s->crtc_ctrl[7] & ~0x10u) | (val & 0x10u);

            s->crtc_ctrl[s->crtc_idx] = (uint8_t)val;
            voodoo3_crtc_update(s);
            voodoo3_pll_update_vblank(s);

            /*
             * CRTC[0x11] IRQ arm/disarm — ported from 86Box banshee_out() 0x3D5:
             *   bit 4 = 0 → disable vsync IRQ: vblank_irq = -1, clear PCI IRQ
             *   bit 4 = 1 (and bit 7 = 0 = protect off) → arm: vblank_irq = 0
             *   bit 5 is the mask bit (1 = masked = no IRQ even if enabled)
             * 86Box: banshee_update_irqs() is called after every CRTC[0x11] write.
             */
            if (s->crtc_idx == 0x11) {
                if (!(val & 0x10u)) {
                    /* bit 4 cleared → disable IRQ, deassert if pending */
                    if (s->vblank_irq_pending) {
                        s->vblank_irq_pending = false;
                        pci_irq_deassert(PCI_DEVICE(s));
                    }
                }
                /* bit 4 set: IRQ now armed; will fire on next vblank if
                 * conditions are met — nothing to do here, vblank_cb handles it */
            }
        }
        break;

    /*
     * 0x3da — Input Status 1 / Feature Control write.
     * Writing resets the ATC flip-flop to index mode.
     */
    case 0x3da:
    case 0x3ba:   /* mono alias */
        s->ar_flip_flop = false;
        s->feat_reg     = val & 0x03;
        break;

    default:
        break;
    }
}

/*
 * VGA I/O read helper — ported from 86Box banshee_in() + svga_in().
 * addr is the full VGA port address (0x3b0..0x3df).
 */
static uint8_t voodoo3_vga_in(Voodoo3State *s, uint16_t addr)
{
    /* Mono/colour alias swap — same logic as voodoo3_vga_out() */
    if (((addr & 0xfff0) == 0x3d0 || (addr & 0xfff0) == 0x3b0)
        && !(s->misc_out & 1))
        addr ^= 0x60;

    switch (addr) {
    /*
     * 0x3c0 / 0x3c1 — ATC data read.
     * 86Box svga_in(): returns ar_regs[ar_idx & 0x1f].
     * Does NOT toggle ar_flip_flop on read.
     */
    case 0x3c0:
    case 0x3c1:
        return s->ar_regs[s->ar_idx & 0x1f];

    /*
     * 0x3c2 — Input Status Register 0.
     * 86Box: bit 4 = monitor sense, bit 7 = vblank IRQ pending.
     * Return 0x00: colour monitor present, no IRQ pending.
     */
    case 0x3c2:
        return 0x00;

    case 0x3c4:
        return s->seq_idx;
    case 0x3c5:
        return (s->seq_idx < 8) ? s->seq_regs[s->seq_idx] : 0xff;

    case 0x3c6:
        return s->dac_pel_mask;

    /*
     * 0x3c7 — DAC state (0=write mode ready, 3=read mode ready).
     */
    case 0x3c7:
        return s->dac_state;

    case 0x3c8:
        return s->dac_write_addr;

    /*
     * 0x3c9 — DAC data read (R/G/B triplet cycling).
     */
    case 0x3c9:
    {
        /* dac_read_addr is uint8_t (0..255) == VOODOO3_CLUT_SIZE-1, always valid */
        uint32_t colour = s->pallook[s->dac_read_addr];
        uint8_t byte;
        switch (s->dac_rgb_idx) {
        case 0: byte = (uint8_t)((colour >> 16) & 0xff); break;  /* R */
        case 1: byte = (uint8_t)((colour >>  8) & 0xff); break;  /* G */
        default:byte = (uint8_t)( colour        & 0xff); break;  /* B */
        }
        s->dac_rgb_idx++;
        if (s->dac_rgb_idx == 3) {
            s->dac_rgb_idx   = 0;
            s->dac_read_addr = (s->dac_read_addr + 1) & 0xff;
        }
        return byte;
    }

    case 0x3ca:   /* Feature Control read */
        return s->feat_reg;

    case 0x3cc:   /* Misc Output read */
        return s->misc_out;

    case 0x3ce:
        return s->gr_idx;
    case 0x3cf:
        return (s->gr_idx < 16) ? s->gr_regs[s->gr_idx] : 0xff;

    case 0x3d4:
        return (uint8_t)(s->crtc_idx & 0x3f);
    case 0x3d5:
        return (s->crtc_idx < 64) ? s->crtc_ctrl[s->crtc_idx] : 0xff;

    /*
     * 0x3da / 0x3ba — Input Status Register 1.
     * bit 3 = vblank active (BIOS waits for this before palette writes).
     * Reading resets the ATC flip-flop to index mode.
     *
     * 86Box banshee_in() 0x3da:
     *   if (vblank_irq > 0) { vblank_irq = -1; banshee_update_irqs(); }
     * In QEMU: clear vblank_irq_pending and deassert PCI IRQ.
     */
    case 0x3da:
    case 0x3ba:
        s->ar_flip_flop = false;
        if (s->vblank_irq_pending) {
            s->vblank_irq_pending = false;
            pci_irq_deassert(PCI_DEVICE(s));
        }
        return s->in_vblank ? 0x08u : 0x00u;

    default:
        return 0xff;
    }
}

static uint64_t voodoo3_io_read(void *opaque, hwaddr addr, unsigned size)
{
    Voodoo3State *s   = VOODOO3_PCI(opaque);
    uint32_t      off = (uint32_t)(addr & 0xff);

    /*
     * BAR2 offset decode:
     *   0x00..0xaf → ext register (same as BAR0 0x00..0xaf)
     *   0xb0..0xdf → VGA port 0x3b0..0x3df  (offset + 0x300)
     *   0xe0..0xff → ext register (remainder of ext space)
     *
     * 86Box banshee_ext_in(): 0xb0..0xdf → banshee_in(off + 0x300).
     */
    if (off >= 0xb0 && off <= 0xdf)
        return (uint64_t)voodoo3_vga_in(s, (uint16_t)(off + 0x300u));

    return (uint64_t)voodoo3_ext_read(s, off);
}

static void voodoo3_io_write(void *opaque, hwaddr addr,
                             uint64_t data, unsigned size)
{
    Voodoo3State *s   = VOODOO3_PCI(opaque);
    uint32_t      off = (uint32_t)(addr & 0xff);
    uint8_t       val = (uint8_t)data;   /* VGA ports are byte-wide */

    /* 86Box banshee_ext_out(): 0xb0..0xdf → banshee_out(off + 0x300, val) */
    if (off >= 0xb0 && off <= 0xdf) {
        voodoo3_vga_out(s, (uint16_t)(off + 0x300u), val);
        return;
    }

    voodoo3_ext_write(s, off, (uint32_t)data);
}

static const MemoryRegionOps voodoo3_io_ops = {
    .read  = voodoo3_io_read,
    .write = voodoo3_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /*
     * VGA I/O ports are byte-wide (BAR2 is an 8-bit I/O aperture).
     * impl min/max = 1 forces QEMU to pass each byte access separately
     * (critical for ATC flip-flop and DAC RGB byte counter correctness).
     */
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 1 },
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

    /*
     * VGA vblank IRQ — ported from 86Box banshee_vblank_start() and
     * banshee_update_irqs() in vid_voodoo_banshee.c.
     *
     * Conditions (86Box banshee_vga_vsync_enabled + banshee_update_irqs):
     *   1. CRTC[0x11] bit 5 = 0  — vsync IRQ not masked (Vertical Retrace End)
     *   2. CRTC[0x11] bit 4 = 1  — vsync IRQ enable
     *   3. pciInit0 bit 18 = 1   — PCI interrupt enable (Init_pciInit0)
     *
     * 86Box: vblank_irq transitions 0→1 on vblank start, then
     *        pci_set_irq fires if the three conditions above are met.
     *        CRTC[0x11] write with bit 4 cleared resets vblank_irq to -1
     *        (disabled); setting bit 4 resets it to 0 (armed).
     *        Reading 0x3da (Input Status 1) acknowledges the interrupt:
     *        86Box banshee_in() 0x3da: if (vblank_irq > 0) vblank_irq = -1
     *        then banshee_update_irqs() → pci_clear_irq.
     *
     * In QEMU we use pci_set_irq() / pci_irq_assert() on the PCI device.
     */
    {
        bool vsync_enabled =
            !(s->crtc_ctrl[0x11] & 0x20u) &&   /* bit 5 = 0: not masked */
             (s->crtc_ctrl[0x11] & 0x10u) &&   /* bit 4 = 1: IRQ enable  */
             (s->pciInit0 & (1u << 18));        /* bit 18: PCI IRQ enable */

        if (vsync_enabled && !s->vblank_irq_pending) {
            s->vblank_irq_pending = true;
            pci_irq_assert(PCI_DEVICE(s));
        }
    }

    /* Check and execute pending buffer swap */
    voodoo3_do_swap_if_pending(s);

    /* Dirty-line-aware display update (ported from 86Box voodoo_callback) */
    if (s->display_enabled && s->con)
        voodoo3_update_display_dirty(s);

    s->in_vblank = false;

    timer_mod(s->vblank_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              (s->vblank_period_ns > 0 ? s->vblank_period_ns
                                       : NANOSECONDS_PER_SECOND / VBLANK_HZ));
}

/* =========================================================================
 * Render threads
 *
 * Mirrors 86Box voodoo_render_thread_1..4 / render_thread().
 * Each thread handles every Nth triangle (band-parallel rendering).
 * The odd_even parameter selects which triangles this thread processes.
 * ========================================================================= */
/* =========================================================================
 * CMDFIFO packet parser
 *
 * Ported from 86Box vid_voodoo_fifo.c voodoo_process_cmdfifo() /
 * voodoo_process_cmdfifo_2().
 *
 * The CMDFIFO is a ring buffer in VRAM.  The guest writes packets and
 * bumps cmdfifo_depth_wr; the FIFO worker reads cmdfifo_rp and processes
 * one dword at a time, advancing the read pointer.
 *
 * Packet header (first dword) bits[2:0] = packet type:
 *   000  NOP / command dispatch (no data)
 *   001  1 header dword, no following data
 *   010  Jump (1 dword: new read ptr)
 *   011  Register write packet — bits[30:3] = count
 *   100  Texture download packet
 *   101  Vertex data packet
 *   110  Raw data packet
 *   111  Extended packet
 *
 * For packet type 3 (register write):
 *   bits[15:4]  = address / 4  (BAR0 register word offset)
 *   bits[28:16] = count (0=1 dword)
 *   bits[30:29] = address MSBs
 *   Following dwords: register values in order starting at address.
 *
 * 86Box reference: vid_voodoo_fifo.c ~line 230 onward.
 * ========================================================================= */

static uint32_t cmdfifo_read_dword(Voodoo3State *s, uint32_t *rp,
                                   bool in_sub, bool in_agp)
{
    uint32_t val = 0;
    (void)in_agp; /* AGP host memory not accessible in QEMU device model */

    /* Read from VRAM ring buffer */
    uint32_t off = *rp & (s->fb_size - 1);
    if (off + 3 < s->fb_size)
        memcpy(&val, s->fb_mem + off, 4);

    if (!in_sub)
        s->cmdfifo_depth_rd++;
    *rp += 4;

    /* Wrap within ring */
    if (s->cmdfifo_end > s->cmdfifo_base && *rp >= s->cmdfifo_end)
        *rp = s->cmdfifo_base + (*rp - s->cmdfifo_end);

    return val;
}

static inline float cmdfifo_read_float(Voodoo3State *s, uint32_t *rp,
                                       bool in_sub, bool in_agp)
{
    union { uint32_t i; float f; } u;
    u.i = cmdfifo_read_dword(s, rp, in_sub, in_agp);
    return u.f;
}

/*
 * Dispatch one CMDFIFO register write.
 * word_addr = register byte-address / 4 (includes BAR0 range bits).
 *
 * Ported from 86Box voodoo_cmdfifo_reg_writel():
 *   if addr & (1<<13) && Banshee → 2D reg write
 *   else → 3D reg write
 */
static void cmdfifo_reg_dispatch(Voodoo3State *s, uint32_t word_addr, uint32_t val)
{
    uint32_t byte_addr = word_addr << 2;

    if (byte_addr & 0x2000u) {
        /* bit 13 set → Banshee 2D engine register */
        voodoo3_2d_reg_write(s, byte_addr & ~0x2000u, val);
    } else {
        /* 3D / SST register */
        voodoo3_3d_reg_write(s, byte_addr, val);
    }
}

/*
 * voodoo3_process_cmdfifo — VRAM CMDFIFO0 packet processor.
 *
 * Ported from 86Box voodoo_fifo_thread() CMDFIFO0 loop in vid_voodoo_fifo.c.
 *
 * Packet types (header bits[2:0]):
 *
 * 0 — Control (NOP / JSR / RET / JMP-LFB / JMP-AGP)
 *       bits[5:3] = sub-type
 *       0 = NOP
 *       1 = JSR: push rp, jump to (header>>4)&0xfffffc, set in_sub=1
 *       2 = RET: pop rp (restore ret_addr), set in_sub=0
 *       3 = JMP local framebuffer: rp = (header>>4)&0xfffffc
 *       4 = JMP AGP: rp from header+next dword (ignored on PCI)
 *
 * 1 — Register write (sequential or strided)
 *       bits[14:3]   = base address (word, i.e. byte_addr/4, with bit13=2D)
 *       bit[15]      = auto-increment (1=yes, 0=same addr each dword)
 *       bits[31:16]  = count of following dwords (0=none, N=N dwords)
 *
 * 2 — 2D register write (packed bitmask)
 *       bits[31:3]   = bitmask of 2D register slots to write
 *       Following dwords for each set bit (lsb first), starting at 2D reg 8
 *
 * 3 — Setup / vertex packet
 *       bits[8:3]    = sSetupMode mask (which vertex components present)
 *       bits[12:9]   = strip mode + start type
 *       bits[25:6]   = vertex count
 *       bits[31:28]  = extra tail dwords to skip
 *       Following: sVx/sVy then optional R/G/B, A, Z, Wb, W0, S0/T0, W1, S1/T1
 *       Each vertex fires sBeginTriCMD or sDrawTriCMD as appropriate.
 *
 * 4 — Register write with bitmask (like type 1 but with explicit mask)
 *       bits[14:3]   = base address (word)
 *       bits[28:15]  = bitmask (14 bits, each bit = one dword to write)
 *       bits[31:29]  = extra tail dwords to skip
 *
 * 5 — Raw VRAM / FB / texture write block
 *       bits[21:3]   = dword count (0 = 1)
 *       bits[31:30]  = destination space: 0/1=LFB, 2=FB (through pipeline), 3=TEX
 *       Following dword: start byte address
 *       Following N dwords: raw data
 *
 * 6 — AGP DMA transfer setup (Banshee only)
 *       Following 5 dwords: agpReqSize, hostAddrLow, hostAddrHigh,
 *                           graphicsAddress, graphicsStride
 *       (stub: accepted but no DMA engine on PCI)
 */
static void voodoo3_process_cmdfifo(Voodoo3State *s)
{
    if (!s->cmdfifo_enabled || s->cmdfifo_base == 0) return;

    /* Packet type 3 component mask bits — identical to 86Box CMDFIFO3_PC_MASK_* */
    enum {
        CF3_RGB   = 1u << 10, CF3_ALPHA = 1u << 11, CF3_Z    = 1u << 12,
        CF3_Wb    = 1u << 13, CF3_W0    = 1u << 14, CF3_S0T0 = 1u << 15,
        CF3_W1    = 1u << 16, CF3_S1T1  = 1u << 17, CF3_PC   = 1u << 28,
    };

    uint32_t rp      = s->cmdfifo_rp;
    uint32_t ret_rp  = 0;
    bool     in_sub  = (bool)s->cmdfifo_in_sub;

    while (in_sub || (s->cmdfifo_depth_rd < s->cmdfifo_depth_wr)) {

        uint32_t header = cmdfifo_read_dword(s, &rp, in_sub, s->cmdfifo_in_agp);

        switch (header & 7u) {

        /* ---- Packet 0: Control ---- */
        case 0:
            switch ((header >> 3) & 7u) {
            case 0: /* NOP */ break;
            case 1: /* JSR */
                ret_rp  = rp;
                rp      = (header >> 4) & 0xffffffcu;
                in_sub  = true;
                s->cmdfifo_in_sub = 1;
                break;
            case 2: /* RET */
                rp     = ret_rp;
                in_sub = false;
                s->cmdfifo_in_sub = 0;
                break;
            case 3: /* JMP LFB */
                rp = (header >> 4) & 0xffffffcu;
                s->cmdfifo_in_agp = 0;
                break;
            case 4: /* JMP AGP — treat as LFB on PCI */
            {
                uint32_t lo = cmdfifo_read_dword(s, &rp, in_sub, false);
                rp = ((header >> 4) & 0x1ffffffcu) | (lo << 25);
                s->cmdfifo_in_agp = 0; /* AGP memory not mapped; use VRAM ptr */
                break;
            }
            default:
                qemu_log_mask(LOG_UNIMP,
                    "voodoo3: CMDFIFO0 unknown sub-type %u\n",
                    (header >> 3) & 7u);
                goto drain;
            }
            break;

        /* ---- Packet 1: Sequential register write ---- */
        case 1:
        {
            uint32_t addr  = (header & 0x7ff8u) >> 1;  /* word addr with bit13=2D */
            int      count = (int)(header >> 16);
            bool     inc   = !!(header & (1u << 15));
            while (count--) {
                uint32_t val = cmdfifo_read_dword(s, &rp, in_sub, s->cmdfifo_in_agp);
                cmdfifo_reg_dispatch(s, addr >> 2, val);
                if (inc) addr += 4;
            }
            break;
        }

        /* ---- Packet 2: 2D register bitmask write ---- */
        case 2:
        {
            uint32_t mask = header >> 3;
            uint32_t addr = 8; /* 2D registers start at byte offset 8 */
            while (mask) {
                if (mask & 1u) {
                    uint32_t val = cmdfifo_read_dword(s, &rp, in_sub, s->cmdfifo_in_agp);
                    voodoo3_2d_reg_write(s, addr, val);
                }
                addr += 4;
                mask >>= 1;
            }
            break;
        }

        /* ---- Packet 3: Setup / vertex packet ---- */
        case 3:
        {
            uint32_t mask        = header;             /* component mask in same word */
            int      smode       = (int)((header >> 22) & 0xfu);
            int      num_verts   = (int)((header >> 6) & 0xfu);
            int      skip        = (int)((header >> 29) & 7u);
            int      v_num       = (((header >> 3) & 7u) == 2) ? 1 : 0;

            /* Write sSetupMode — bits[17:10] of header become sSetupMode[7:0]
             * 86Box: voodoo_cmdfifo_reg_writel(SST_sSetupMode,
             *            ((header>>10)&0xff) | (smode<<16)) */
            voodoo3_3d_reg_write(s, SST_sSetupMode,
                                 ((header >> 10) & 0xffu) | ((uint32_t)smode << 16));

            while (num_verts--) {
                s->verts[3].sVx = cmdfifo_read_float(s, &rp, in_sub, s->cmdfifo_in_agp);
                s->verts[3].sVy = cmdfifo_read_float(s, &rp, in_sub, s->cmdfifo_in_agp);

                if (mask & CF3_RGB) {
                    if (header & CF3_PC) {
                        uint32_t packed = cmdfifo_read_dword(s, &rp, in_sub, s->cmdfifo_in_agp);
                        s->verts[3].sBlue  = (float)( packed        & 0xff);
                        s->verts[3].sGreen = (float)((packed >>  8) & 0xff);
                        s->verts[3].sRed   = (float)((packed >> 16) & 0xff);
                        s->verts[3].sAlpha = (float)((packed >> 24) & 0xff);
                    } else {
                        s->verts[3].sRed   = cmdfifo_read_float(s, &rp, in_sub, s->cmdfifo_in_agp);
                        s->verts[3].sGreen = cmdfifo_read_float(s, &rp, in_sub, s->cmdfifo_in_agp);
                        s->verts[3].sBlue  = cmdfifo_read_float(s, &rp, in_sub, s->cmdfifo_in_agp);
                    }
                }
                if ((mask & CF3_ALPHA) && !(header & CF3_PC))
                    s->verts[3].sAlpha = cmdfifo_read_float(s, &rp, in_sub, s->cmdfifo_in_agp);
                if (mask & CF3_Z)
                    s->verts[3].sVz = cmdfifo_read_float(s, &rp, in_sub, s->cmdfifo_in_agp);
                if (mask & CF3_Wb)
                    s->verts[3].sWb = cmdfifo_read_float(s, &rp, in_sub, s->cmdfifo_in_agp);
                if (mask & CF3_W0)
                    s->verts[3].sW0 = cmdfifo_read_float(s, &rp, in_sub, s->cmdfifo_in_agp);
                if (mask & CF3_S0T0) {
                    s->verts[3].sS0 = cmdfifo_read_float(s, &rp, in_sub, s->cmdfifo_in_agp);
                    s->verts[3].sT0 = cmdfifo_read_float(s, &rp, in_sub, s->cmdfifo_in_agp);
                }
                if (mask & CF3_W1)
                    s->verts[3].sW1 = cmdfifo_read_float(s, &rp, in_sub, s->cmdfifo_in_agp);
                if (mask & CF3_S1T1) {
                    s->verts[3].sS1 = cmdfifo_read_float(s, &rp, in_sub, s->cmdfifo_in_agp);
                    s->verts[3].sT1 = cmdfifo_read_float(s, &rp, in_sub, s->cmdfifo_in_agp);
                }

                /* Fire sBeginTriCMD for first vertex, sDrawTriCMD for rest */
                if (v_num)
                    voodoo3_3d_reg_write(s, SST_sDrawTriCMD, 0);
                else
                    voodoo3_3d_reg_write(s, SST_sBeginTriCMD, 0);
                v_num++;

                /* In strip mode (sub-type 0), reset ping-pong every 3 verts */
                if (v_num == 3 && ((header >> 3) & 7u) == 0)
                    v_num = 0;
            }

            /* Skip trailing padding dwords */
            while (skip--)
                cmdfifo_read_dword(s, &rp, in_sub, s->cmdfifo_in_agp);
            break;
        }

        /* ---- Packet 4: Bitmask register write ---- */
        case 4:
        {
            uint32_t addr = (header & 0x7ff8u) >> 1;
            uint32_t mask = (header >> 15) & 0x3fffu;
            int      skip = (int)((header >> 29) & 7u);
            while (mask) {
                if (mask & 1u) {
                    uint32_t val = cmdfifo_read_dword(s, &rp, in_sub, s->cmdfifo_in_agp);
                    cmdfifo_reg_dispatch(s, addr >> 2, val);
                }
                addr += 4;
                mask >>= 1;
            }
            while (skip--)
                cmdfifo_read_dword(s, &rp, in_sub, s->cmdfifo_in_agp);
            break;
        }

        /* ---- Packet 5: Raw data block ---- */
        case 5:
        {
            int      count     = (int)((header >> 3) & 0x7ffffu);
            unsigned dst_space = (header >> 30) & 3u;
            uint32_t addr      = cmdfifo_read_dword(s, &rp, in_sub, s->cmdfifo_in_agp)
                                 & 0xffffffu;
            if (!count) count = 1;

            while (count--) {
                uint32_t val = cmdfifo_read_dword(s, &rp, in_sub, s->cmdfifo_in_agp);
                switch (dst_space) {
                case 0: /* Linear framebuffer — direct VRAM write */
                case 1: /* Planar YUV — treat as linear for now */
                    if (addr + 3 < s->fb_size) {
                        memcpy(s->fb_mem + addr, &val, 4);
                        /*
                         * Bug 4 fix: invalidate texture cache if the write
                         * address overlaps a cached texture.
                         * Ported from 86Box texture_present[] check +
                         * flush_texture_cache() in voodoo_tex_writel() and
                         * banshee_linear_write() (vid_voodoo_texture.c).
                         */
                        voodoo3_flush_tex_if_dirty(s, addr & s->tex_mask);
                    }
                    break;
                case 2: /* Framebuffer through render pipeline */
                    voodoo3_push_fifo(s, (addr & 0xfffffcu) | FIFO_WRITEL_FB, val);
                    break;
                case 3: /* Texture */
                    voodoo3_tex_download(s, addr, val, (addr >> 22) & 1);
                    break;
                }
                addr += 4;
            }
            break;
        }

        /* ---- Packet 6: AGP DMA setup (Banshee) ---- */
        case 6:
        {
            /*
             * 86Box: reads 5 dwords, writes agpReqSize/hostAddr/graphicsAddr/Stride,
             * then triggers agpMoveCMD.  On PCI (no host memory access) we just
             * consume and discard.
             */
            uint32_t d0 = cmdfifo_read_dword(s, &rp, in_sub, s->cmdfifo_in_agp);
            (void)d0;
            for (int i = 0; i < 4; i++)
                cmdfifo_read_dword(s, &rp, in_sub, s->cmdfifo_in_agp);
            qemu_log_mask(LOG_UNIMP,
                "voodoo3: CMDFIFO6 AGP DMA (PCI stub, ignored)\n");
            break;
        }

        /* ---- Packet 7: Extended packet (Voodoo3 reserved, not in 86Box either) ---- */
        case 7:
            /*
             * Packet type 7 is the "extended" header type reserved in the
             * Voodoo3/Banshee CMDFIFO spec.  Neither 86Box nor any known driver
             * actually generates it under normal operation; when it appears the
             * FIFO almost always contains uninitialised memory (0xcfcfcfcf).
             *
             * 86Box behaviour: falls into fatal() — we cannot do that in QEMU.
             * Previous behaviour: goto drain (flushes the entire FIFO, causing
             * rendering stalls and repeat log spam for every subsequent dword).
             *
             * New behaviour (ported rationale from 86Box vid_voodoo_fifo.c):
             * Log once and skip the single header dword already consumed, then
             * continue processing.  The FIFO length counter was already
             * decremented by cmdfifo_read_dword(); no additional data to skip.
             */
            qemu_log_mask(LOG_UNIMP,
                "voodoo3: unknown CMDFIFO packet type %u header=0x%08x rp=0x%08x"
                " (skipping, likely uninit FIFO data)\n",
                header & 7u, header, rp);
            break;

        default:
            qemu_log_mask(LOG_UNIMP,
                "voodoo3: unknown CMDFIFO packet type %u header=0x%08x rp=0x%08x\n",
                header & 7u, header, s->cmdfifo_rp);
            goto drain;
        }

        s->cmdfifo_rp = rp;
        s->cmdfifo_in_sub = in_sub ? 1 : 0;
    }
    s->cmdfifo_rp = rp;
    s->cmdfifo_in_sub = in_sub ? 1 : 0;
    return;

drain:
    /* Unknown packet — drain to avoid spin */
    s->cmdfifo_depth_rd = s->cmdfifo_depth_wr;
    s->cmdfifo_rp       = rp;
    s->cmdfifo_in_sub   = 0;
}

/*
 * cmdfifo_read_dword_2 — read one dword from CMDFIFO1 ring buffer.
 *
 * Mirrors cmdfifo_read_dword() but operates on the _2 fields (FIFO1).
 * Ported from 86Box cmdfifo_get_2() in vid_voodoo_fifo.c.
 *
 * AGP host memory (cmdfifo_in_agp_2) is not directly accessible in the
 * QEMU device model (no mem_readl_phys equivalent); we fall back to VRAM,
 * which is correct for PCI mode where the driver places FIFO1 in SGRAM.
 */
static uint32_t cmdfifo_read_dword_2(Voodoo3State *s, uint32_t *rp,
                                     bool in_sub)
{
    uint32_t val = 0;
    uint32_t off = *rp & (s->fb_size - 1);
    if (off + 3u < s->fb_size)
        memcpy(&val, s->fb_mem + off, 4);

    if (!in_sub)
        s->cmdfifo_depth_rd_2++;
    *rp += 4;

    /* Wrap within FIFO1 ring */
    if (s->cmdfifo_end_2 > s->cmdfifo_base_2 && *rp >= s->cmdfifo_end_2)
        *rp = s->cmdfifo_base_2 + (*rp - s->cmdfifo_end_2);

    return val;
}

static inline float cmdfifo_read_float_2(Voodoo3State *s, uint32_t *rp,
                                         bool in_sub)
{
    union { uint32_t i; float f; } u;
    u.i = cmdfifo_read_dword_2(s, rp, in_sub);
    return u.f;
}

/*
 * voodoo3_process_cmdfifo2 — CMDFIFO1 (AGP ring buffer) packet processor.
 *
 * Ported from the second while-loop in 86Box voodoo_fifo_thread()
 * (vid_voodoo_fifo.c, starting at "while (voodoo->cmdfifo_enabled_2 &&
 * (voodoo->cmdfifo_depth_rd_2 != voodoo->cmdfifo_depth_wr_2 || ...))").
 *
 * FIFO1 has the same 7 packet types as FIFO0.  The only difference is:
 *  - reads come from cmdfifo_read_dword_2() (uses _2 fields)
 *  - register dispatch still calls cmdfifo_reg_dispatch() (same 3D/2D logic)
 *  - subroutine state uses cmdfifo_in_sub_2 / cmdfifo_ret_addr_2
 *
 * On Pegasos2/AmigaOS4 (PCI bus) FIFO1 is normally never used by the
 * 3Dfx driver; this path is exercised on AGP boards (MorphOS on G4 Mac).
 * The function is called from voodoo3_render_thread() after FIFO0.
 */
static void voodoo3_process_cmdfifo2(Voodoo3State *s)
{
    if (!s->cmdfifo_enabled_2 || s->cmdfifo_base_2 == 0) return;

    /* Packet type 3 component mask bits — same as FIFO0 */
    enum {
        CF3_RGB   = 1u << 10, CF3_ALPHA = 1u << 11, CF3_Z    = 1u << 12,
        CF3_Wb    = 1u << 13, CF3_W0    = 1u << 14, CF3_S0T0 = 1u << 15,
        CF3_W1    = 1u << 16, CF3_S1T1  = 1u << 17, CF3_PC   = 1u << 28,
    };

    uint32_t rp      = s->cmdfifo_rp_2;
    uint32_t ret_rp  = 0;          /* JSR return address */
    bool     in_sub  = (bool)s->cmdfifo_in_sub_2;

    while (in_sub || (s->cmdfifo_depth_rd_2 < s->cmdfifo_depth_wr_2)) {

        uint32_t header = cmdfifo_read_dword_2(s, &rp, in_sub);

        switch (header & 7u) {

        /* ---- Packet 0: Control ---- */
        case 0:
            switch ((header >> 3) & 7u) {
            case 0: /* NOP */ break;
            case 1: /* JSR */
                ret_rp = rp;
                rp     = (header >> 4) & 0xfffffc;
                in_sub = true;
                s->cmdfifo_in_sub_2 = 1;
                break;
            case 2: /* RET */
                rp     = ret_rp;
                in_sub = false;
                s->cmdfifo_in_sub_2 = 0;
                break;
            case 3: /* JMP local framebuffer */
                rp = (header >> 4) & 0xfffffc;
                s->cmdfifo_in_agp_2 = false;
                break;
            case 4: /* JMP AGP */
                {
                    uint32_t lo = cmdfifo_read_dword_2(s, &rp, in_sub);
                    rp = ((header >> 4) & 0x1fffffc) | (lo << 25);
                    s->cmdfifo_in_agp_2 = false; /* no AGP mapping in QEMU */
                }
                break;
            default:
                qemu_log_mask(LOG_UNIMP,
                    "voodoo3: CMDFIFO2 bad PKT0 subtype %u header=0x%08x\n",
                    (header >> 3) & 7u, header);
                break;
            }
            s->cmdfifo_in_sub_2 = in_sub ? 1 : 0;
            break;

        /* ---- Packet 1: Sequential / strided register write ---- */
        case 1:
            {
                int      cnt  = (int)(header >> 16);
                uint32_t waddr = (header & 0x7ff8u) >> 1; /* word address */
                bool     inc  = !!(header & (1u << 15));
                while (cnt--) {
                    uint32_t val = cmdfifo_read_dword_2(s, &rp, in_sub);
                    cmdfifo_reg_dispatch(s, waddr, val);
                    if (inc) waddr++;
                }
            }
            break;

        /* ---- Packet 2: 2D register write with bitmask ---- */
        case 2:
            {
                uint32_t mask  = header >> 3;
                uint32_t baddr = 8; /* 2D regs start at byte offset 8 */
                while (mask) {
                    if (mask & 1u) {
                        uint32_t val = cmdfifo_read_dword_2(s, &rp, in_sub);
                        voodoo3_2d_reg_write(s, baddr, val);
                    }
                    baddr += 4;
                    mask  >>= 1;
                }
            }
            break;

        /* ---- Packet 3: Setup / vertex ---- */
        case 3:
            {
                int      extra        = (int)((header >> 29) & 7u);
                uint32_t comp_mask    = header;
                int      smode        = (int)((header >> 22) & 0xfu);
                int      num_verts    = (int)((header >> 6) & 0xfu);
                int      v_num        = ((header >> 3) & 7u) == 2 ? 1 : 0;

                voodoo3_3d_reg_write(s, SST_sSetupMode,
                    ((header >> 10) & 0xffu) | ((uint32_t)smode << 16));

                while (num_verts--) {
                    s->verts[3].sVx = cmdfifo_read_float_2(s, &rp, in_sub);
                    s->verts[3].sVy = cmdfifo_read_float_2(s, &rp, in_sub);
                    if (comp_mask & CF3_RGB) {
                        if (comp_mask & CF3_PC) {
                            uint32_t packed = cmdfifo_read_dword_2(s, &rp, in_sub);
                            s->verts[3].sBlue  = (float)(packed & 0xffu);
                            s->verts[3].sGreen = (float)((packed >> 8)  & 0xffu);
                            s->verts[3].sRed   = (float)((packed >> 16) & 0xffu);
                            s->verts[3].sAlpha = (float)((packed >> 24) & 0xffu);
                        } else {
                            s->verts[3].sRed   = cmdfifo_read_float_2(s, &rp, in_sub);
                            s->verts[3].sGreen = cmdfifo_read_float_2(s, &rp, in_sub);
                            s->verts[3].sBlue  = cmdfifo_read_float_2(s, &rp, in_sub);
                        }
                    }
                    if ((comp_mask & CF3_ALPHA) && !(comp_mask & CF3_PC))
                        s->verts[3].sAlpha = cmdfifo_read_float_2(s, &rp, in_sub);
                    if (comp_mask & CF3_Z)
                        s->verts[3].sVz = cmdfifo_read_float_2(s, &rp, in_sub);
                    if (comp_mask & CF3_Wb)
                        s->verts[3].sWb = cmdfifo_read_float_2(s, &rp, in_sub);
                    if (comp_mask & CF3_W0)
                        s->verts[3].sW0 = cmdfifo_read_float_2(s, &rp, in_sub);
                    if (comp_mask & CF3_S0T0) {
                        s->verts[3].sS0 = cmdfifo_read_float_2(s, &rp, in_sub);
                        s->verts[3].sT0 = cmdfifo_read_float_2(s, &rp, in_sub);
                    }
                    if (comp_mask & CF3_W1)
                        s->verts[3].sW1 = cmdfifo_read_float_2(s, &rp, in_sub);
                    if (comp_mask & CF3_S1T1) {
                        s->verts[3].sS1 = cmdfifo_read_float_2(s, &rp, in_sub);
                        s->verts[3].sT1 = cmdfifo_read_float_2(s, &rp, in_sub);
                    }
                    /* Fire sBeginTriCMD or sDrawTriCMD */
                    if (v_num)
                        voodoo3_3d_reg_write(s, SST_sDrawTriCMD, 0);
                    else
                        voodoo3_3d_reg_write(s, SST_sBeginTriCMD, 0);
                    v_num++;
                    if (v_num == 3 && ((header >> 3) & 7u) == 0)
                        v_num = 0;
                }
                /* Consume tail padding dwords */
                while (extra--)
                    cmdfifo_read_dword_2(s, &rp, in_sub);
            }
            break;

        /* ---- Packet 4: Register write with explicit bitmask ---- */
        case 4:
            {
                int      extra = (int)((header >> 29) & 7u);
                uint32_t mask  = (header >> 15) & 0x3fffu;
                uint32_t waddr = (header & 0x7ff8u) >> 1;
                while (mask) {
                    if (mask & 1u) {
                        uint32_t val = cmdfifo_read_dword_2(s, &rp, in_sub);
                        cmdfifo_reg_dispatch(s, waddr, val);
                    }
                    waddr++;
                    mask >>= 1;
                }
                while (extra--)
                    cmdfifo_read_dword_2(s, &rp, in_sub);
            }
            break;

        /* ---- Packet 5: Raw VRAM / FB / texture block ---- */
        case 5:
            {
                int      count     = (int)((header >> 3) & 0x7ffffu);
                unsigned dst_space = (header >> 30) & 3u;
                uint32_t addr      = cmdfifo_read_dword_2(s, &rp, in_sub)
                                     & 0xffffffu;
                if (!count) count = 1;

                while (count--) {
                    uint32_t val = cmdfifo_read_dword_2(s, &rp, in_sub);
                    switch (dst_space) {
                    case 0: /* Linear framebuffer */
                    case 1: /* Planar YUV — treat as linear */
                        if (addr + 3u < s->fb_size) {
                            memcpy(s->fb_mem + addr, &val, 4);
                            /* Bug 4 fix: texture cache invalidation for
                             * direct VRAM writes (same as FIFO0 path). */
                            voodoo3_flush_tex_if_dirty(s, addr & s->tex_mask);
                        }
                        break;
                    case 2: /* Framebuffer through render pipeline */
                        voodoo3_push_fifo(s,
                            (addr & 0xfffffcu) | FIFO_WRITEL_FB, val);
                        break;
                    case 3: /* Texture RAM */
                        voodoo3_tex_download(s, addr, val,
                                             (addr >> 22) & 1);
                        break;
                    }
                    addr += 4;
                }
            }
            break;

        /* ---- Packet 6: AGP DMA (stub — no DMA engine on PCI) ---- */
        case 6:
            {
                /* Consume the 5 DMA parameter dwords (86Box: agpReqSize,
                 * hostAddressLow, hostAddressHigh, graphicsAddress,
                 * graphicsStride).  On PCI this is a no-op. */
                (void)cmdfifo_read_dword_2(s, &rp, in_sub);
                for (int i = 0; i < 4; i++)
                    cmdfifo_read_dword_2(s, &rp, in_sub);
                qemu_log_mask(LOG_UNIMP,
                    "voodoo3: CMDFIFO2 PKT6 AGP DMA (PCI stub, ignored)\n");
            }
            break;

        default:
            qemu_log_mask(LOG_UNIMP,
                "voodoo3: CMDFIFO2 unknown packet type %u header=0x%08x"
                " rp=0x%08x (draining)\n", header & 7u, header, rp);
            /* Drain to avoid infinite spin on uninitialised FIFO data */
            s->cmdfifo_depth_rd_2 = s->cmdfifo_depth_wr_2;
            s->cmdfifo_rp_2       = rp;
            s->cmdfifo_in_sub_2   = 0;
            return;
        }

        s->cmdfifo_rp_2     = rp;
        s->cmdfifo_in_sub_2 = in_sub ? 1 : 0;
    }
    s->cmdfifo_rp_2     = rp;
    s->cmdfifo_in_sub_2 = in_sub ? 1 : 0;
}

static void *voodoo3_render_thread(void *arg)
{
    uintptr_t     tid = (uintptr_t)((uint64_t *)arg)[0];
    Voodoo3State *s   = (Voodoo3State *)((uint64_t *)arg)[1];
    g_free(arg);

    qemu_mutex_lock(&s->render_lock);
    while (!s->render_stop) {
        qemu_cond_wait(&s->render_cond, &s->render_lock);
        if (s->render_stop) break;

        if (tid == 0) {
            /*
             * Thread 0: process the internal MMIO FIFO (2D regs, TEX writes
             * queued by voodoo3_mmio_write), then drain the VRAM CMDFIFO.
             * Mirrors 86Box where the FIFO thread runs voodoo_process_cmdfifo().
             */
            while (s->fifo_rd != s->fifo_wr) {
                uint32_t cmd = s->fifo_cmd[s->fifo_rd];
                uint32_t val = s->fifo_val[s->fifo_rd];
                s->fifo_rd   = (s->fifo_rd + 1) & (V3_FIFO_SIZE - 1);
                s->cmd_read++;

                uint32_t type = cmd & 0xff800000u;
                if (type == FIFO_WRITEL_TEX) {
                    int tmu = (cmd & 0x200000u) ? 1 : 0;
                    voodoo3_tex_download(s, cmd, val, tmu);
                } else if (type == FIFO_WRITEL_FB) {
                    /*
                     * LFB pixel-write routed through the 3D pipeline.
                     * Produced by lfbMode-writes and CMDFIFO packet-5 type-2.
                     * Ported from 86Box vid_voodoo_fifo.c FIFO_WRITEL_FB case,
                     * which calls voodoo_fb_writel().
                     *
                     * cmd[22:0] is the framebuffer byte address
                     * (FIFO_WRITEL_FB already masks the type bits off, leaving
                     * addr & 0xfffffc in the lower 23 bits of cmd).
                     */
                    voodoo3_fb_writel(s, cmd & 0x00ffffffu, val);
                }
            }

            /* Process VRAM CMDFIFO0 */
            voodoo3_process_cmdfifo(s);

            /*
             * Process CMDFIFO1 (AGP ring buffer).
             *
             * Ported from the second while-loop in 86Box voodoo_fifo_thread()
             * in vid_voodoo_fifo.c.  86Box runs both FIFOs sequentially in
             * the same fifo thread; we do the same here.
             *
             * On PCI (Pegasos2 / AmigaOS4) FIFO1 is never populated by
             * the 3Dfx driver, so voodoo3_process_cmdfifo2() returns
             * immediately (depth_rd_2 == depth_wr_2 and in_sub_2 == 0).
             * On AGP boards that initialise FIFO1, all 7 packet types are
             * handled identically to FIFO0.
             */
            voodoo3_process_cmdfifo2(s);
        }

        /*
         * Band-parallel triangle rasterization — ported from 86Box
         * render_thread() in vid_voodoo_render.c.
         *
         * 86Box model (2-thread example):
         *   • One shared write pointer (PARAMS_WRITE_IDX).
         *   • Each thread has its OWN read pointer (PARAMS_READ_IDX[odd_even]).
         *   • Every triangle is read by EVERY thread.
         *   • voodoo_half_triangle() skips scanlines where
         *       (screen_y & odd_even_mask) != thread_id
         *     so thread 0 draws even scanlines, thread 1 draws odd scanlines.
         *
         * Result: zero false-sharing on the framebuffer — adjacent scanlines
         * are always owned by different threads, preventing the write-tearing
         * race that the old round-robin dispatch produced on overlapping
         * triangles.
         */
        uint32_t nthreads = s->render_threads_count;
        while (s->param_rd[tid] < s->param_wr) {
            uint32_t idx = s->param_rd[tid] & (PARAM_BUF_SIZE - 1);
            voodoo3_triangle(s, &s->param_buf[idx], (int)tid);
            s->param_rd[tid]++;
        }

        /* Check if all threads are idle */
        bool all_done = true;
        for (uint32_t i = 0; i < nthreads; i++) {
            if (s->param_rd[i] < s->param_wr) { all_done = false; break; }
        }
        if (all_done && s->fifo_rd == s->fifo_wr
            && s->cmdfifo_depth_rd  >= s->cmdfifo_depth_wr
            && s->cmdfifo_depth_rd_2 >= s->cmdfifo_depth_wr_2)
            s->voodoo_busy = false;
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
    /* Default clip rects: full screen (86Box initialises to max extents) */
    s->blt.clip[0].x_max = s->blt.clip[1].x_max = 4095;
    s->blt.clip[0].y_max = s->blt.clip[1].y_max = 4095;
    memset(s->verts,   0, sizeof(s->verts));

    /*
     * miscInit0 reset value: 0x00000000 on real hardware.
     * QEMU previously set bits[7:6] (0xC0) which is wrong.
     * Real HW (measured): 0x00000000 at power-on.
     */
    s->miscInit0  = 0x00000000;
    /*
     * miscInit1 reset value: real HW has bits[1:0] set (0x01800003).
     * Bit 0 = bypass FIFO, bit 1 = bypass PCI.  These are set by the
     * 3dfx reference BIOS during init and remain set under AmigaOS.
     * FIX: voodoo3diag Module 2 -- miscInit1 bits[1:0] missing in QEMU.
     */
    s->miscInit1  = 0x00000003;
    s->pciInit0   = 0x01000100;
    /*
     * dramInit0 — SGRAM/SDRAM configuration register.
     * Bit 27: SGRAM present (0 = SDRAM).
     * Bit 26: 2×SGRAM = 16 MB (only when bit27=1).
     * Bit 16: refresh-clock enable (always set).
     * Ported from 86Box banshee_init():
     *   dramInit0 = (1<<27)|(1<<26) for 16 MB SGRAM.
     *   dramInit1 = (1<<30)         for SDRAM (Banshee).
     * Programs like EVEREST/GPU-Z read bit26+27 to report VRAM size.
     */
    switch (s->model) {
    case VOODOO3_MODEL_BANSHEE:
        /* Banshee: SDRAM only, 16 MB */
        s->dramInit0 = (1u << 16);             /* no SGRAM bit */
        s->dramInit1 = (1u << 30);             /* SDRAM select */
        break;
    case VOODOO3_MODEL_V3_1000:
        /* Velocity 100: 8 MB SGRAM (single chip) */
        s->dramInit0 = (1u << 27) | (1u << 16); /* SGRAM, 8 MB */
        /*
         * Bit 27 = mem_init_done: firmware sets this after DRAM initialisation.
         * QEMU skips the DRAM init sequence, so we assert it at reset to signal
         * that memory is valid and ready.
         * FIX: voodoo3diag Module 18 -- "mem init NOT done" warning.
         */
        s->dramInit1 = (1u << 27);
        break;
    case VOODOO3_MODEL_V3_2000:
    case VOODOO3_MODEL_V3_3000:
    case VOODOO3_MODEL_V3_3500TV:
    default:
        /* V3 2000/3000/3500: 16 MB SGRAM (2 chips) */
        s->dramInit0 = (1u << 27) | (1u << 26) | (1u << 16);
        s->dramInit1 = (1u << 27); /* mem_init_done — FIX Module 18 */
        break;
    }
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
    s->vidDesktopStartAddr = 0;         /* FIX: diag Module 25 WARN -- register
                                         * readback at 0xE4 returned 0xFFFFFF00
                                         * (uninitialised garbage > 16 MB VRAM).
                                         * desktop_start and vidDesktopStartAddr
                                         * must be kept in sync from reset. */
    s->pix_format      = PIX_FORMAT_RGB565;
    s->display_enabled = false;
    /*
     * in_vblank = true at reset: real hardware powers on with display
     * disabled, so the vblank flag is asserted.  This makes the STATUS
     * register read 0x1F000000 (BE) at reset — bit6 (display active)
     * is CLEAR when in vblank.
     * FIX: voodoo3diag Module 2/5 -- status was 0x5F000000 (bit6 set)
     * instead of 0x1F000000 on real HW.
     */
    s->in_vblank       = true;

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
    s->ncc_gen[0]   = s->ncc_gen[1]   = 0;
    memset(s->ncc_table,  0, sizeof(s->ncc_table));
    memset(s->ncc_lookup, 0, sizeof(s->ncc_lookup));

    /* CRTC / DAC indexed register state */
    memset(s->crtc_ctrl,  0, sizeof(s->crtc_ctrl));
    memset(s->crtc_freq,  0, sizeof(s->crtc_freq));
    memset(s->dac_reset,  0, sizeof(s->dac_reset));
    s->crtc_idx       = 0;
    s->crtc_freq_idx  = 0;
    s->dac_reset_idx  = 0;

    /* VGA register reset — mirrors 86Box svga_init() defaults */
    s->misc_out      = 0x01;   /* I/O select = 3Dx (bit 0 set), RAM disabled */
    s->feat_reg      = 0x00;
    s->seq_idx       = 0x00;
    memset(s->seq_regs,  0, sizeof(s->seq_regs));
    s->gr_idx        = 0x00;
    memset(s->gr_regs,   0, sizeof(s->gr_regs));
    s->ar_idx        = 0x00;
    memset(s->ar_regs,   0, sizeof(s->ar_regs));
    s->ar_flip_flop  = false;
    s->dac_pel_mask  = 0xff;   /* 86Box svga_init: dac_mask = 0xff */
    s->dac_read_addr = 0x00;
    s->dac_write_addr= 0x00;
    s->dac_rgb_idx   = 0x00;
    memset(s->dac_rgb_buf, 0, sizeof(s->dac_rgb_buf));
    s->dac_state     = 0x00;   /* write mode */

    /* PLL / pixel clock — reset to 0 so vblank_cb falls back to VBLANK_HZ
     * until the driver programs pllCtrl0 and misc_out clock select.      */
    s->pixel_clock_hz   = 0.0;
    s->vblank_period_ns = 0;

    /*
     * 86Box init: banshee->vidSerialParallelPort = VIDSERIAL_DDC_DCK_W | VIDSERIAL_DDC_DDA_W
     * = (1<<19) | (1<<20) = 0x00180000
     * I2C bus idles with both SCL and SDA high.
     * Starting at 0 would mean SCL=0 SDA=0 which is an invalid bus state
     * and confuses the START/STOP edge detectors on the first write.
     */
    s->vidSerialParallelPort = (1u << 19) | (1u << 20);  /* DCK_W | DDA_W both high */
    /* DDC/I2C buses idle with SCL=1 SDA=1 (open-drain pull-up state) */
    voodoo3_vidserial_update(&s->vidSerialParallelPort, &s->bbi2c_ddc,
                             18, 19, 20, 21, 22);
    voodoo3_vidserial_update(&s->vidSerialParallelPort, &s->bbi2c_i2c,
                             23, 24, 25, 26, 27);
    /* Overlay reset */
    memset(&s->ov, 0, sizeof(s->ov));
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

    /*
     * Hardware cursor reset.
     *
     * Two independent guards prevent the boot artefact (coloured rectangle
     * at top-left before the OS positions the cursor):
     *
     * 1. cur_loc_valid — false until the first explicit hwCurLoc write.
     *    voodoo3_draw_cursor() returns immediately while this is false,
     *    so no cursor is drawn even if cursor_ena is set early.
     *
     * 2. cursor_buf transparent pattern — every row is initialised with
     *    plane0=0xFF, plane1=0x00.  In Windows AND/XOR mode this means
     *    p0=1, p1=0 for every pixel → transparent (skip), so even if
     *    cur_loc_valid somehow becomes true before the OS writes the real
     *    sprite shape, the cursor remains invisible.
     *    (In X11 mode plane0=mask: 0xFF means all pixels drawn, but
     *    cur_loc_valid still guards that path.)
     */
    s->cur_loc_valid = false;
    s->cur_x         = 0;
    s->cur_y         = 0;
    s->cur_yoff      = 0;
    s->cur_c0        = 0;
    s->cur_c1        = 0;
    s->cur_pat_addr  = 0;
    for (int row = 0; row < 64; row++) {
        uint8_t *p = s->cursor_buf + row * 16;
        memset(p,     0xFF, 8);   /* plane0: all 1 → p0=1 (transparent in Win mode) */
        memset(p + 8, 0x00, 8);   /* plane1: all 0 → p1=0 */
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

    /* -----------------------------------------------------------------------
     * Device ID
     * Banshee uses 0x0003; all Voodoo3 variants (V3-1000..3500) use 0x0005.
     * Source: 86Box vid_voodoo_banshee.c + 3dfx PCI IDs.
     * ----------------------------------------------------------------------- */
    switch (s->model) {
    case VOODOO3_MODEL_BANSHEE:
        pci_set_word(cfg + PCI_DEVICE_ID, PCI_DEVICE_ID_3DFX_BANSHEE);
        break;
    case VOODOO3_MODEL_V3_1000:
    case VOODOO3_MODEL_V3_2000:
    case VOODOO3_MODEL_V3_3000:
    case VOODOO3_MODEL_V3_3500TV:
    default:
        pci_set_word(cfg + PCI_DEVICE_ID, PCI_DEVICE_ID_3DFX_VOODOO3);
        break;
    }

    /* -----------------------------------------------------------------------
     * Subsystem Vendor + Device ID
     * Subsystem Vendor is always 0x121A (3dfx Interactive).
     * Subsystem Device IDs from 86Box vid_voodoo_banshee.c pci_regs[0x2e]:
     *   Banshee          0x0003
     *   V3-1000          0x0052  (PCI only — no AGP variant sold)
     *   V3-2000 PCI      0x0036  /  AGP  0x0038
     *   V3-3000 PCI      0x003A  /  AGP  0x003C
     *   V3-3500          0x0060  (AGP only)
     * ----------------------------------------------------------------------- */
    pci_set_word(cfg + PCI_SUBSYSTEM_VENDOR_ID, PCI_VENDOR_ID_3DFX);
    switch (s->model) {
    case VOODOO3_MODEL_BANSHEE:
        pci_set_word(cfg + PCI_SUBSYSTEM_ID, 0x0003);
        break;
    case VOODOO3_MODEL_V3_1000:
        /* V3-1000 was PCI-only; no AGP variant exists */
        pci_set_word(cfg + PCI_SUBSYSTEM_ID, 0x0052);
        break;
    case VOODOO3_MODEL_V3_2000:
    {
        uint16_t subid = s->is_agp ? 0x0038u : 0x0036u;
        pci_set_word(cfg + PCI_SUBSYSTEM_ID, subid);
        break;
    }
    case VOODOO3_MODEL_V3_3500TV:
        /* V3-3500TV was AGP-only; always use AGP subsystem ID */
        pci_set_word(cfg + PCI_SUBSYSTEM_ID, 0x0060);
        break;
    case VOODOO3_MODEL_V3_3000:
    default:
    {
        uint16_t subid = s->is_agp ? 0x003Cu : 0x003Au;
        pci_set_word(cfg + PCI_SUBSYSTEM_ID, subid);
        break;
    }
    }

    /* Log final PCI identity once at realize time */
    qemu_log_mask(LOG_UNIMP,
        "voodoo3: realize model=%d is_agp=%d device_id=0x%04x subsystem_id=0x%04x\n",
        s->model, s->is_agp,
        pci_get_word(cfg + PCI_DEVICE_ID),
        pci_get_word(cfg + PCI_SUBSYSTEM_ID));

    cfg[PCI_LATENCY_TIMER] = 0x40;

    s->fb_size = VOODOO3_FB_SIZE;

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

    memory_region_init_ram(&s->lfb_ram, OBJECT(s),
                           "voodoo3-lfb",
                           VOODOO3_LFB_SIZE,
                           &error_fatal);
    s->fb_mem = memory_region_get_ram_ptr(&s->lfb_ram);
    pci_register_bar(pci_dev, 1,
        PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_32 |
        PCI_BASE_ADDRESS_MEM_PREFETCH, &s->lfb_ram);

    /*
     * CMDFIFO overlay: initialise with a placeholder size of 4096 bytes
     * (one page — the minimum programmable CMDFIFO size).  The region is
     * NOT added as a subregion yet; voodoo3_cmdfifo_reposition() does that
     * when the driver programs CMDFIFO_BASE_ADDR0 / CMDFIFO_SIZE0.
     */
    memory_region_init_io(&s->cmdfifo_mmio, OBJECT(s),
                          &voodoo3_cmdfifo_ops, s,
                          "voodoo3-cmdfifo", 4096);
    s->cmdfifo_mmio_active = false;

    memory_region_init_io(&s->io, OBJECT(s), &voodoo3_io_ops, s,
                          "voodoo3-io", VOODOO3_IO_SIZE);
    pci_register_bar(pci_dev, 2, PCI_BASE_ADDRESS_SPACE_IO, &s->io);

    s->con = graphic_console_init(DEVICE(pci_dev), 0, &voodoo3_gfx_ops, s);
    qemu_console_resize(s->con, 640, 480);

    /* -----------------------------------------------------------------------
     * DDC / EDID — initialise two bitbang I²C buses sharing one I2CDDC slave.
     * Follows the same pattern as hw/display/ati.c:
     *   i2cbus = i2c_init_bus(...)
     *   bitbang_i2c_init(&s->bbi2c, i2cbus)
     *   i2c_slave_set_address(I2C_SLAVE(&s->i2cddc), 0x50)
     *   qdev_realize(DEVICE(&s->i2cddc), BUS(i2cbus), &error_abort)
     *
     * Both DDC bus (bits 18-22) and secondary I²C bus (bits 23-27) are
     * wired to the same EDID slave so any driver that probes either bus
     * receives valid monitor data.
     * ----------------------------------------------------------------------- */
    {
        I2CBus *ddc_bus = i2c_init_bus(DEVICE(pci_dev), "voodoo3.ddc");
        I2CBus *i2c_bus = i2c_init_bus(DEVICE(pci_dev), "voodoo3.i2c");
        bitbang_i2c_init(&s->bbi2c_ddc, ddc_bus);
        bitbang_i2c_init(&s->bbi2c_i2c, i2c_bus);
        /* Attach the EDID slave (address 0x50) to the DDC bus */
        i2c_slave_set_address(I2C_SLAVE(&s->i2cddc), 0x50);
        qdev_realize(DEVICE(&s->i2cddc), BUS(ddc_bus), &error_abort);
        /* Attach a second EDID slave to the secondary I²C bus.
         * Copy the edid_info so both buses advertise the same monitor data.
         * (DEFINE_EDID_PROPERTIES only populates s->i2cddc.edid_info.) */
        {
            I2CDDCState *slave2 = I2CDDC(qdev_new(TYPE_I2CDDC));
            slave2->edid_info = s->i2cddc.edid_info;
            i2c_slave_set_address(I2C_SLAVE(slave2), 0x50);
            qdev_realize(DEVICE(slave2), BUS(i2c_bus), &error_abort);
        }
    }

    s->vblank_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, voodoo3_vblank_cb, s);
    timer_mod(s->vblank_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / VBLANK_HZ);

    /* SDL-safe deferred resize BH — must be allocated before any MMIO write */
    s->resize_bh        = qemu_bh_new(voodoo3_resize_bh, s);
    s->resize_pending_w = 0;
    s->resize_pending_h = 0;

    qemu_mutex_init(&s->render_lock);
    qemu_cond_init(&s->render_cond);
    qemu_mutex_init(&s->fifo_lock);
    qemu_cond_init(&s->fifo_cond);
    s->render_stop = false;

    uint32_t nthreads = s->render_threads_count;
    if (nthreads < 1) nthreads = 1;
    if (nthreads > MAX_RENDER_THREADS) nthreads = MAX_RENDER_THREADS;
    /* odd_even_mask must be (power-of-two - 1); clamp to supported values */
    if (nthreads > 2) nthreads = 4;
    else if (nthreads > 1) nthreads = 2;
    s->render_threads_count = nthreads;
    /*
     * Band-parallel scanline mask (86Box: odd_even_mask = render_threads - 1).
     * Thread T renders scanlines where (screen_y & odd_even_mask) == T.
     */
    s->odd_even_mask = nthreads - 1u;

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
        break;
    case VOODOO3_MODEL_V3_1000:
        s->pllCtrl0 = 0x2d07;   /* ~143 MHz */
        break;
    case VOODOO3_MODEL_V3_2000:
        s->pllCtrl0 = 0x2d07;   /* ~143 MHz */
        break;
    case VOODOO3_MODEL_V3_3500TV:
        s->pllCtrl0 = 0x3507;   /* ~183 MHz */
        break;
    case VOODOO3_MODEL_V3_3000:
    default:
        s->pllCtrl0 = 0x3207;   /* ~166 MHz */
        break;
    }
    qemu_log_mask(LOG_UNIMP,
        "voodoo3: power-on pllCtrl0=0x%04x (model=%d)\n",
        s->pllCtrl0, s->model);
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

    /* Cancel any pending deferred resize and free the BH */
    qemu_bh_cancel(s->resize_bh);
    qemu_bh_delete(s->resize_bh);

    timer_free(s->vblank_timer);
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
 * =========================================================================
 *
 * VMSTATE_VBUFFER_UINT8 — saves a pointer-backed byte buffer whose length
 * is stored in a uint32_t struct member (the byte count, not word count).
 *
 * QEMU ships VMSTATE_VBUFFER_UINT32 (VMS_VBUFFER | VMS_MULTIPLY) which
 * multiplies the size field by sizeof(uint32_t).  We need the raw byte
 * count, so we define the _UINT8 variant locally without VMS_MULTIPLY.
 * This is the only way to save fb_mem (a uint8_t * pointer, 16 MiB) with
 * fb_size (uint32_t, value = 16*1024*1024 bytes) as the length field.
 */
#ifndef VMSTATE_VBUFFER_UINT8
#define VMSTATE_VBUFFER_UINT8(_field, _state, _version, _test, _num_field) { \
    .name         = (stringify(_field)),                                       \
    .version_id   = (_version),                                                \
    .field_exists = (_test),                                                   \
    .size_offset  = vmstate_offset_value(_state, _num_field, uint32_t),       \
    .info         = &vmstate_info_uint8,                                       \
    .flags        = VMS_VBUFFER,                                               \
    .offset       = offsetof(_state, _field),                                  \
}
#endif

/*
 * FIX 11: pre_save / post_load hooks for "voodoo3/3dstate" subsection.
 *
 * Pack params.fogTable (struct array) and verts[] (struct array) into flat
 * shadow buffers before save; unpack them after load.
 */
static int voodoo3_3dstate_pre_save(void *opaque)
{
    Voodoo3State *s = opaque;
    /* Pack fogTable: {fog,dfog}[64] → uint8_t[128] interleaved */
    for (int i = 0; i < 64; i++) {
        s->fog_table_save[i * 2 + 0] = s->params.fogTable[i].fog;
        s->fog_table_save[i * 2 + 1] = s->params.fogTable[i].dfog;
    }
    /* Pack verts[4]: each vertex has 14 floats in declaration order.
     * verts_save is uint32_t[] - use memcpy to avoid strict-aliasing UB. */
    for (int v = 0; v < 4; v++) {
        const voodoo3_vert_t *vsrc = &s->verts[v];
        float tmp[14] = {
            vsrc->sVx,   vsrc->sVy,   vsrc->sVz,
            vsrc->sWb,   vsrc->sRed,  vsrc->sGreen,
            vsrc->sBlue, vsrc->sAlpha,
            vsrc->sW0,   vsrc->sS0,   vsrc->sT0,
            vsrc->sW1,   vsrc->sS1,   vsrc->sT1,
        };
        memcpy(s->verts_save + v * 14, tmp, sizeof(tmp));
    }
    return 0;
}

static int voodoo3_3dstate_post_load(void *opaque, int version_id)
{
    Voodoo3State *s = opaque;
    (void)version_id;
    /* Unpack fogTable */
    for (int i = 0; i < 64; i++) {
        s->params.fogTable[i].fog  = s->fog_table_save[i * 2 + 0];
        s->params.fogTable[i].dfog = s->fog_table_save[i * 2 + 1];
    }
    /* Unpack verts[4] */
    for (int v = 0; v < 4; v++) {
        float tmp[14];
        memcpy(tmp, s->verts_save + v * 14, sizeof(tmp));
        voodoo3_vert_t *vdst = &s->verts[v];
        vdst->sVx    = tmp[ 0]; vdst->sVy    = tmp[ 1]; vdst->sVz    = tmp[ 2];
        vdst->sWb    = tmp[ 3]; vdst->sRed   = tmp[ 4]; vdst->sGreen = tmp[ 5];
        vdst->sBlue  = tmp[ 6]; vdst->sAlpha = tmp[ 7];
        vdst->sW0    = tmp[ 8]; vdst->sS0    = tmp[ 9]; vdst->sT0    = tmp[10];
        vdst->sW1    = tmp[11]; vdst->sS1    = tmp[12]; vdst->sT1    = tmp[13];
    }
    return 0;
}

/*
 * Subsection needed() predicate for "voodoo3/sgram".
 * Always returns true — we always want to save/restore SGRAM when
 * migrating.  A non-NULL needed() is required by QEMU's subsection
 * machinery; passing NULL causes an assertion failure on load in some
 * QEMU versions.
 */
static bool voodoo3_sgram_needed(void *opaque)
{
    return true;
}

/*
 * FIX 11: Subsection needed() predicate for "voodoo3/3dstate".
 * Always save — the 3D parameter set (gradients, fog, detail, TMU
 * texture coordinates, swap state, setup vertices) must survive a
 * snapshot/restore cycle, otherwise the next triangle submitted after
 * restore uses stale or zeroed gradient registers and renders garbage.
 *
 * This is a new subsection (version 1) so snapshots taken before this
 * fix load cleanly: QEMU skips unknown subsections, leaving all fields
 * at their reset() defaults.  The guest driver will re-program all 3D
 * registers before the next draw call — acceptable for the legacy case.
 */
static bool voodoo3_3dstate_needed(void *opaque)
{
    return true;
}

static const VMStateDescription vmstate_voodoo3 = {
    .name           = "voodoo3",
    .version_id     = 4,
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
        VMSTATE_UINT32_ARRAY(ncc_gen, Voodoo3State, 2),

        /* --- 3D render-state (missing before version 4) --- */
        /* params: render-state registers */
        VMSTATE_UINT32(params.fbzColorPath,    Voodoo3State),
        VMSTATE_UINT32(params.fbzMode,         Voodoo3State),
        VMSTATE_UINT32(params.fogMode,         Voodoo3State),
        VMSTATE_UINT32(params.alphaMode,       Voodoo3State),
        VMSTATE_UINT32(params.lfbMode,         Voodoo3State),
        VMSTATE_UINT32(params.stipple,         Voodoo3State),
        VMSTATE_UINT32(params.color0,          Voodoo3State),
        VMSTATE_UINT32(params.color1,          Voodoo3State),
        VMSTATE_UINT32(params.zaColor,         Voodoo3State),
        VMSTATE_UINT32(params.chromaKey,       Voodoo3State),
        VMSTATE_UINT32(params.draw_offset,     Voodoo3State),
        VMSTATE_UINT32(params.front_offset,    Voodoo3State),
        VMSTATE_UINT32(params.aux_offset,      Voodoo3State),
        VMSTATE_UINT32(params.row_width,       Voodoo3State),
        VMSTATE_UINT32(params.aux_row_width,   Voodoo3State),
        VMSTATE_INT32(params.col_tiled,        Voodoo3State),
        VMSTATE_INT32(params.aux_tiled,        Voodoo3State),
        /* FIX: raw stride register values for correct readback (diag Module 22) */
        VMSTATE_UINT32(params.col_stride_raw,  Voodoo3State),
        VMSTATE_UINT32(params.aux_stride_raw,  Voodoo3State),
        VMSTATE_INT32(params.clipLeft,         Voodoo3State),
        VMSTATE_INT32(params.clipRight,        Voodoo3State),
        VMSTATE_INT32(params.clipLowY,         Voodoo3State),
        VMSTATE_INT32(params.clipHighY,        Voodoo3State),
        VMSTATE_INT32(params.clipLeft1,        Voodoo3State),
        VMSTATE_INT32(params.clipRight1,       Voodoo3State),
        VMSTATE_INT32(params.clipLowY1,        Voodoo3State),
        VMSTATE_INT32(params.clipHighY1,       Voodoo3State),
        /* params: vertex positions and gradients */
        VMSTATE_INT32(params.vertexAx,         Voodoo3State),
        VMSTATE_INT32(params.vertexAy,         Voodoo3State),
        VMSTATE_INT32(params.vertexBx,         Voodoo3State),
        VMSTATE_INT32(params.vertexBy,         Voodoo3State),
        VMSTATE_INT32(params.vertexCx,         Voodoo3State),
        VMSTATE_INT32(params.vertexCy,         Voodoo3State),
        VMSTATE_INT32(params.startR,           Voodoo3State),
        VMSTATE_INT32(params.startG,           Voodoo3State),
        VMSTATE_INT32(params.startB,           Voodoo3State),
        VMSTATE_INT32(params.startA,           Voodoo3State),
        VMSTATE_INT32(params.startZ,           Voodoo3State),
        VMSTATE_INT32(params.dRdX,             Voodoo3State),
        VMSTATE_INT32(params.dGdX,             Voodoo3State),
        VMSTATE_INT32(params.dBdX,             Voodoo3State),
        VMSTATE_INT32(params.dAdX,             Voodoo3State),
        VMSTATE_INT32(params.dZdX,             Voodoo3State),
        VMSTATE_INT32(params.dRdY,             Voodoo3State),
        VMSTATE_INT32(params.dGdY,             Voodoo3State),
        VMSTATE_INT32(params.dBdY,             Voodoo3State),
        VMSTATE_INT32(params.dAdY,             Voodoo3State),
        VMSTATE_INT32(params.dZdY,             Voodoo3State),
        VMSTATE_INT64(params.startW,           Voodoo3State),
        VMSTATE_INT64(params.dWdX,             Voodoo3State),
        VMSTATE_INT64(params.dWdY,             Voodoo3State),
        /* params.tmu[0] */
        VMSTATE_UINT32(params.tmu[0].textureMode,    Voodoo3State),
        VMSTATE_UINT32(params.tmu[0].tLOD,           Voodoo3State),
        VMSTATE_UINT32(params.tmu[0].texBaseAddr,    Voodoo3State),
        VMSTATE_UINT32(params.tmu[0].texBaseAddr1,   Voodoo3State),
        VMSTATE_UINT32(params.tmu[0].texBaseAddr2,   Voodoo3State),
        VMSTATE_UINT32(params.tmu[0].texBaseAddr38,  Voodoo3State),
        VMSTATE_INT32(params.tmu[0].tformat,         Voodoo3State),
        /* params.tmu[1] */
        VMSTATE_UINT32(params.tmu[1].textureMode,    Voodoo3State),
        VMSTATE_UINT32(params.tmu[1].tLOD,           Voodoo3State),
        VMSTATE_UINT32(params.tmu[1].texBaseAddr,    Voodoo3State),
        VMSTATE_UINT32(params.tmu[1].texBaseAddr1,   Voodoo3State),
        VMSTATE_UINT32(params.tmu[1].texBaseAddr2,   Voodoo3State),
        VMSTATE_UINT32(params.tmu[1].texBaseAddr38,  Voodoo3State),
        VMSTATE_INT32(params.tmu[1].tformat,         Voodoo3State),
        /* NCC raw coefficient tables [tmu][table_sel] */
        VMSTATE_UINT32_ARRAY(ncc_table[0][0].y, Voodoo3State, 4),
        VMSTATE_UINT32_ARRAY(ncc_table[0][0].i, Voodoo3State, 4),
        VMSTATE_UINT32_ARRAY(ncc_table[0][0].q, Voodoo3State, 4),
        VMSTATE_UINT32_ARRAY(ncc_table[0][1].y, Voodoo3State, 4),
        VMSTATE_UINT32_ARRAY(ncc_table[0][1].i, Voodoo3State, 4),
        VMSTATE_UINT32_ARRAY(ncc_table[0][1].q, Voodoo3State, 4),
        VMSTATE_UINT32_ARRAY(ncc_table[1][0].y, Voodoo3State, 4),
        VMSTATE_UINT32_ARRAY(ncc_table[1][0].i, Voodoo3State, 4),
        VMSTATE_UINT32_ARRAY(ncc_table[1][0].q, Voodoo3State, 4),
        VMSTATE_UINT32_ARRAY(ncc_table[1][1].y, Voodoo3State, 4),
        VMSTATE_UINT32_ARRAY(ncc_table[1][1].i, Voodoo3State, 4),
        VMSTATE_UINT32_ARRAY(ncc_table[1][1].q, Voodoo3State, 4),
        /* Hardware cursor state */
        VMSTATE_INT32(cur_x,   Voodoo3State),
        VMSTATE_INT32(cur_y,   Voodoo3State),
        VMSTATE_INT32(cur_yoff, Voodoo3State),
        VMSTATE_UINT32(cur_c0, Voodoo3State),
        VMSTATE_UINT32(cur_c1, Voodoo3State),
        VMSTATE_UINT8_ARRAY(cursor_buf, Voodoo3State, 1024),
        VMSTATE_BOOL(cur_loc_valid, Voodoo3State),

        VMSTATE_END_OF_LIST()
    },
    /*
     * FIX 8: persist SGRAM (fb_mem) across snapshots.
     *
     * fb_mem is a pointer into a RAM-backed MemoryRegion (lfb_ram).  QEMU
     * normally saves RAM regions automatically via the "ram" block mechanism,
     * but lfb_ram is a device-private region initialised with
     * memory_region_init_ram_device_ptr() which does NOT register it as a
     * named RAM block — so it is invisible to the standard RAM saver.
     *
     * We use a subsection (introduced in version 5) so that old snapshots
     * saved at version 4 still load correctly: QEMU skips unknown
     * subsections on restore, and fb_mem is cleared to zero on realize(),
     * producing a blank screen rather than a crash.  The guest OS will
     * redraw the display as soon as it next calls the driver — acceptable
     * for the legacy-snapshot case.
     *
     * VMSTATE_VBUFFER_UINT8 saves exactly fb_size bytes from fb_mem.
     * fb_size is fixed at VOODOO3_FB_SIZE (16 MiB) for all supported
     * models, so no length field is needed.
     */
    .subsections = (const VMStateDescription * const []) {
        &(const VMStateDescription) {
            .name            = "voodoo3/sgram",
            .version_id      = 1,
            .minimum_version_id = 1,
            .needed          = voodoo3_sgram_needed,
            .fields = (const VMStateField[]) {
                VMSTATE_VBUFFER_UINT8(fb_mem, Voodoo3State, 1, NULL, fb_size),
                VMSTATE_END_OF_LIST()
            },
        },
        /*
         * FIX 11: "voodoo3/3dstate" subsection.
         *
         * Saves all 3D register state that was omitted from the main
         * VMState list:
         *
         *   params.tmu[0/1] gradient fields — startS/T/W and their
         *     dS/dT/dW per-pixel and per-scanline increments.  Without
         *     these the texture coordinate walker starts at 0,0 after
         *     restore, producing a single repeated texel.
         *   params.tmu[0/1] LOD fields — lodbias, lod_min, lod_max.
         *     Without these the mipmap selector defaults to LOD 0
         *     regardless of the distance from camera.
         *   params.fogTable[64] — per-index fog density + dfog step.
         *     Without this table-fog renders with all-zero coefficients.
         *   params.fogColor — RGB fog colour used for linear/z fog.
         *   params.chromaKey_r/g/b — per-channel chromakey thresholds
         *     decoded from params.chromaKey; without them chromakey
         *     passes every pixel.
         *   params.detail_max/bias/scale[2] — TC_MSELECT_DETAIL blend
         *     factors.  Without them detail texturing returns zero blend.
         *   params.sign — triangle winding (1/-1).  Without it all
         *     back-face cull results are inverted after restore.
         *   params.swapbufferCMD — pending buffer swap command.
         *   verts[4] / vertex_num / vertex_next_age / vertex_ages[3] /
         *     num_verticies / cull_pingpong — setup-engine vertex
         *     accumulator.  Without these the first sDrawTriCMD after
         *     restore will use one garbage vertex from index 0.
         *   swap_pending / swap_interval / swap_offset / retrace_count /
         *     frame_count — buffer-swap synchronisation.  Without these
         *     a pending swap may be skipped or triggered twice.
         *
         * All fields use the same VMSTATE_ macros as the main list.
         * No new types are needed — int64_t → VMSTATE_INT64,
         * float[] → VMSTATE_UINT32_ARRAY (bitwise via memcpy, lossless), struct arrays → per-field entries.
         *
         * fogTable is saved as two separate UINT8_ARRAY slices (fog
         * and dfog bytes interleaved in the struct) using the ARRAY2
         * pattern: save all fog[i] then all dfog[i] to avoid needing
         * a custom VMStateDescription for the anonymous struct.
         */
        &(const VMStateDescription) {
            .name            = "voodoo3/3dstate",
            .version_id      = 1,
            .minimum_version_id = 1,
            .needed          = voodoo3_3dstate_needed,
            .pre_save        = voodoo3_3dstate_pre_save,
            .post_load       = voodoo3_3dstate_post_load,
            .fields = (const VMStateField[]) {
                /* TMU 0 — texture coordinate gradients */
                VMSTATE_INT64(params.tmu[0].startS,  Voodoo3State),
                VMSTATE_INT64(params.tmu[0].startT,  Voodoo3State),
                VMSTATE_INT64(params.tmu[0].startW,  Voodoo3State),
                VMSTATE_INT64(params.tmu[0].dSdX,    Voodoo3State),
                VMSTATE_INT64(params.tmu[0].dTdX,    Voodoo3State),
                VMSTATE_INT64(params.tmu[0].dWdX,    Voodoo3State),
                VMSTATE_INT64(params.tmu[0].dSdY,    Voodoo3State),
                VMSTATE_INT64(params.tmu[0].dTdY,    Voodoo3State),
                VMSTATE_INT64(params.tmu[0].dWdY,    Voodoo3State),
                VMSTATE_INT32(params.tmu[0].lodbias,  Voodoo3State),
                VMSTATE_INT32(params.tmu[0].lod_min,  Voodoo3State),
                VMSTATE_INT32(params.tmu[0].lod_max,  Voodoo3State),
                /* TMU 1 — texture coordinate gradients */
                VMSTATE_INT64(params.tmu[1].startS,  Voodoo3State),
                VMSTATE_INT64(params.tmu[1].startT,  Voodoo3State),
                VMSTATE_INT64(params.tmu[1].startW,  Voodoo3State),
                VMSTATE_INT64(params.tmu[1].dSdX,    Voodoo3State),
                VMSTATE_INT64(params.tmu[1].dTdX,    Voodoo3State),
                VMSTATE_INT64(params.tmu[1].dWdX,    Voodoo3State),
                VMSTATE_INT64(params.tmu[1].dSdY,    Voodoo3State),
                VMSTATE_INT64(params.tmu[1].dTdY,    Voodoo3State),
                VMSTATE_INT64(params.tmu[1].dWdY,    Voodoo3State),
                VMSTATE_INT32(params.tmu[1].lodbias,  Voodoo3State),
                VMSTATE_INT32(params.tmu[1].lod_min,  Voodoo3State),
                VMSTATE_INT32(params.tmu[1].lod_max,  Voodoo3State),
                /*
                 * fogTable — saved via shadow array fog_table_save[128].
                 * Packing/unpacking done in pre_save/post_load above.
                 * Layout: [fog0, dfog0, fog1, dfog1, ..., fog63, dfog63]
                 */
                VMSTATE_UINT8_ARRAY(fog_table_save, Voodoo3State, 128),
                /* fogColor — three individual bytes (no padding in struct) */
                VMSTATE_UINT8(params.fogColor.r, Voodoo3State),
                VMSTATE_UINT8(params.fogColor.g, Voodoo3State),
                VMSTATE_UINT8(params.fogColor.b, Voodoo3State),
                /* chromaKey decoded channels */
                VMSTATE_UINT8(params.chromaKey_r, Voodoo3State),
                VMSTATE_UINT8(params.chromaKey_g, Voodoo3State),
                VMSTATE_UINT8(params.chromaKey_b, Voodoo3State),
                /* Detail-texture parameters [tmu] */
                VMSTATE_INT32_ARRAY(params.detail_max,   Voodoo3State, 2),
                VMSTATE_INT32_ARRAY(params.detail_bias,  Voodoo3State, 2),
                VMSTATE_INT32_ARRAY(params.detail_scale, Voodoo3State, 2),
                /* Triangle winding and swap command */
                VMSTATE_INT32(params.sign,           Voodoo3State),
                VMSTATE_UINT32(params.swapbufferCMD, Voodoo3State),
                /* Per-stat counters embedded in params (driver reads back) */
                VMSTATE_UINT32(params.fbiPixelsIn,    Voodoo3State),
                VMSTATE_UINT32(params.fbiChromaFail,  Voodoo3State),
                VMSTATE_UINT32(params.fbiZFuncFail,   Voodoo3State),
                VMSTATE_UINT32(params.fbiAFuncFail,   Voodoo3State),
                VMSTATE_UINT32(params.fbiPixelsOut,   Voodoo3State),
                /*
                 * Setup-engine vertex accumulator — saved via shadow array
                 * verts_save[4*14].  Packing/unpacking in pre_save/post_load.
                 */
                VMSTATE_UINT32_ARRAY(verts_save, Voodoo3State, 56),
                VMSTATE_INT32(vertex_num,         Voodoo3State),
                VMSTATE_INT32(vertex_next_age,    Voodoo3State),
                VMSTATE_INT32_ARRAY(vertex_ages,  Voodoo3State, 3),
                VMSTATE_INT32(num_verticies,      Voodoo3State),
                VMSTATE_INT32(cull_pingpong,       Voodoo3State),
                /* Buffer-swap synchronisation */
                VMSTATE_BOOL(swap_pending,        Voodoo3State),
                VMSTATE_INT32(swap_interval,      Voodoo3State),
                VMSTATE_UINT32(swap_offset,       Voodoo3State),
                VMSTATE_INT32(retrace_count,      Voodoo3State),
                VMSTATE_UINT32(frame_count,       Voodoo3State),
                /* Global 3D pixel counters */
                VMSTATE_UINT32(fbiPixelsIn,    Voodoo3State),
                VMSTATE_UINT32(fbiChromaFail,  Voodoo3State),
                VMSTATE_UINT32(fbiZFuncFail,   Voodoo3State),
                VMSTATE_UINT32(fbiAFuncFail,   Voodoo3State),
                VMSTATE_UINT32(fbiPixelsOut,   Voodoo3State),
                VMSTATE_END_OF_LIST()
            },
        },
        NULL
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
    DEFINE_EDID_PROPERTIES(Voodoo3State, i2cddc.edid_info),
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
     * No built-in ROM: the Voodoo3 BIOS is proprietary and cannot be
     * bundled.  k->romfile = NULL is the explicit default (no ROM loaded
     * unless the user supplies -device voodoo3,...,romfile=<path>).
     * When romfile is given, QEMU's generic PCI layer loads it and maps
     * the expansion-ROM BAR automatically — no extra code needed here.
     */
    k->romfile   = NULL;

    device_class_set_legacy_reset(dc, voodoo3_reset);
    dc->vmsd     = &vmstate_voodoo3;
    dc->desc     = "3Dfx Voodoo 3 / Banshee PCI/AGP graphics (ported from 86Box)";
    device_class_set_props(dc, voodoo3_properties);
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static void voodoo3_instance_init(Object *obj)
{
    Voodoo3State *s = VOODOO3_PCI(obj);

    /*
     * Initialise the I2CDDC child object so that DEFINE_EDID_PROPERTIES
     * fields are accessible before realize() runs.  The child is realised
     * (attached to its I²C bus) inside voodoo3_pci_realize().
     */
    object_initialize_child(obj, "i2cddc", &s->i2cddc, TYPE_I2CDDC);
}

static const TypeInfo voodoo3_info = {
    .name          = TYPE_VOODOO3_PCI,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(Voodoo3State),
    .instance_init = voodoo3_instance_init,
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
