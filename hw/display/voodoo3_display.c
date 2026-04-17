/*
 * QEMU 3Dfx Voodoo 3 — Display, FastFill, SwapBuffer, NCC
 *
 * Ported from 86Box:
 *   vid_voodoo_display.c  — voodoo_update_ncc(), dirty-line display output
 *   vid_voodoo_blitter.c  — voodoo_fastfill()
 *   vid_voodoo_reg.c      — swapbufferCMD logic
 *   vid_voodoo_render.c   — dither tables (derived from vid_voodoo_dither.h)
 *
 * Original author: Sarah Walker <https://pcem-emulator.co.uk/>
 * Copyright (C) 2008-2024 Sarah Walker and 86Box contributors
 * Copyright (C) 2026 <your name here>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/display/voodoo3_int.h"
#include "hw/display/voodoo3_display.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"

/* =========================================================================
 * 4×4 ordered dither tables for RGB565 output
 *
 * Generated from the algorithm in 86Box vid_voodoo_dither.h.
 * For each 8-bit input value, the table gives the 5-bit (R/B) or 6-bit (G)
 * dithered output for each position in a 4×4 pattern.
 *
 * dither_rb[v][y&3][x&3] → 5-bit output  (divide by 8 before packing)
 * dither_g [v][y&3][x&3] → 6-bit output  (divide by 4 before packing)
 * ========================================================================= */

/* Bayer 4×4 threshold matrix (0-15) */
static const int bayer4[4][4] = {
    {  0,  8,  2, 10 },
    { 12,  4, 14,  6 },
    {  3, 11,  1,  9 },
    { 15,  7, 13,  5 },
};

/* Build dither tables at runtime (avoids 256 KB of static data) */
static uint8_t v3_dither_rb_tbl[256][4][4];
static uint8_t v3_dither_g_tbl [256][4][4];
static bool    dither_tables_ready;

void voodoo3_init_dither_tables(void)
{
    if (dither_tables_ready) return;
    dither_tables_ready = true;

    for (int v = 0; v < 256; v++) {
        for (int y = 0; y < 4; y++) {
            for (int x = 0; x < 4; x++) {
                /* R/B: 8-bit → 5-bit with 4×4 Bayer dither
                 * threshold range 0-7 for 8→5 reduction (lose 3 bits) */
                int t_rb  = (bayer4[y][x] >> 1); /* 0-7 */
                int out_rb = (v + t_rb) >> 3;
                v3_dither_rb_tbl[v][y][x] = (uint8_t)(out_rb > 31 ? 31 : out_rb);

                /* G: 8-bit → 6-bit with 4×4 Bayer dither
                 * threshold range 0-3 for 8→6 reduction (lose 2 bits) */
                int t_g  = (bayer4[y][x] >> 2); /* 0-3 */
                int out_g = (v + t_g) >> 2;
                v3_dither_g_tbl[v][y][x] = (uint8_t)(out_g > 63 ? 63 : out_g);
            }
        }
    }
}

const uint8_t (*voodoo3_dither_rb)[4][4] = v3_dither_rb_tbl;
const uint8_t (*voodoo3_dither_g )[4][4] = v3_dither_g_tbl;

/* =========================================================================
 * NCC (Naïve Colour Compression) table update
 *
 * Ported from 86Box voodoo_update_ncc() in vid_voodoo_display.c.
 * Decodes the nccTable Y/I/Q registers into an 8-bit→RGBA lookup table
 * used by the TEX_Y4I2Q2 and TEX_A8Y4I2Q2 texture formats.
 * ========================================================================= */
#define NCC_CLAMP(x) ((x) < 0 ? 0 : ((x) > 255 ? 255 : (x)))

void voodoo3_update_ncc(Voodoo3State *s, int tmu)
{
    for (int tbl = 0; tbl < 2; tbl++) {
        for (int col = 0; col < 256; col++) {
            int y_idx = (col >> 4);
            int i_idx = (col >> 2) & 3;
            int q_idx =  col       & 3;

            /* Y component — packed 4×8-bit in y[0..3] */
            int y = (int)((s->ncc_table[tmu][tbl].y[y_idx >> 2]
                          >> ((y_idx & 3) * 8)) & 0xff);

            /* I component — signed 9-bit fields */
            int i_r = (int)((s->ncc_table[tmu][tbl].i[i_idx] >> 18) & 0x1ff);
            if (i_r & 0x100) i_r |= ~0x1ff;
            int i_g = (int)((s->ncc_table[tmu][tbl].i[i_idx] >>  9) & 0x1ff);
            if (i_g & 0x100) i_g |= ~0x1ff;
            int i_b = (int)( s->ncc_table[tmu][tbl].i[i_idx]        & 0x1ff);
            if (i_b & 0x100) i_b |= ~0x1ff;

            /* Q component — signed 9-bit fields */
            int q_r = (int)((s->ncc_table[tmu][tbl].q[q_idx] >> 18) & 0x1ff);
            if (q_r & 0x100) q_r |= ~0x1ff;
            int q_g = (int)((s->ncc_table[tmu][tbl].q[q_idx] >>  9) & 0x1ff);
            if (q_g & 0x100) q_g |= ~0x1ff;
            int q_b = (int)( s->ncc_table[tmu][tbl].q[q_idx]        & 0x1ff);
            if (q_b & 0x100) q_b |= ~0x1ff;

            s->ncc_lookup[tmu][tbl][col] =
                (uint32_t)NCC_CLAMP(y + i_b + q_b)        |
                ((uint32_t)NCC_CLAMP(y + i_g + q_g) <<  8) |
                ((uint32_t)NCC_CLAMP(y + i_r + q_r) << 16) |
                0xff000000u;
        }
    }
    s->ncc_dirty[tmu] = 0;
}

/* =========================================================================
 * FastFill — clear draw buffer and/or depth buffer within clip rectangle
 *
 * Ported from 86Box voodoo_fastfill() in vid_voodoo_blitter.c.
 * ========================================================================= */
#define FBZ_RGB_WMASK   (1 << 10)
#define FBZ_DEPTH_WMASK (1 << 12)
#define FBZ_Y_ORIGIN    (1 << 17)

void voodoo3_fastfill(Voodoo3State *s)
{
    const voodoo3_params_t *p = &s->params;
    int low_y, high_y;

    /* Y-origin flip for y_origin mode */
    if (p->fbzMode & FBZ_Y_ORIGIN) {
        high_y = s->y_origin_swap + 1 - p->clipLowY;
        low_y  = s->y_origin_swap + 1 - p->clipHighY;
    } else {
        low_y  = p->clipLowY;
        high_y = p->clipHighY;
    }

    if (low_y < 0) low_y = 0;
    if (high_y > s->screen_height) high_y = s->screen_height;

    /* --- Colour buffer fill --- */
    if (p->fbzMode & FBZ_RGB_WMASK) {
        uint8_t r = (uint8_t)(((p->color1 >> 16) & 0xff) >> 3);
        uint8_t g = (uint8_t)(((p->color1 >>  8) & 0xff) >> 2);
        uint8_t b = (uint8_t)( (p->color1        & 0xff) >> 3);
        uint16_t col = (uint16_t)((r << 11) | (g << 5) | b);

        for (int y = low_y; y < high_y; y++) {
            uint16_t *row;
            if (p->col_tiled)
                row = (uint16_t *)(s->fb_mem + p->draw_offset
                      + (size_t)(y >> 5) * p->row_width + (size_t)(y & 31) * 128);
            else
                row = (uint16_t *)(s->fb_mem + p->draw_offset
                      + (size_t)y * p->row_width);

            for (int x = p->clipLeft; x < p->clipRight; x++) {
                if (p->col_tiled) {
                    int xt = (x & 63) | ((x >> 6) * 128 * 32 / 2);
                    row[xt] = col;
                } else {
                    row[x] = col;
                }
            }

            /* Mark dirty if drawing to front buffer */
            if (p->draw_offset == p->front_offset && y < V3_DIRTY_LINES)
                s->dirty_line[y] = 1;
        }
    }

    /* --- Depth/aux buffer fill --- */
    if (p->fbzMode & FBZ_DEPTH_WMASK) {
        uint16_t depth = (uint16_t)(p->zaColor & 0xffff);

        for (int y = low_y; y < high_y; y++) {
            uint16_t *row;
            if (p->aux_tiled)
                row = (uint16_t *)(s->fb_mem + p->aux_offset
                      + (size_t)(y >> 5) * p->aux_row_width
                      + (size_t)(y & 31) * 128);
            else
                row = (uint16_t *)(s->fb_mem + p->aux_offset
                      + (size_t)y * p->aux_row_width);

            for (int x = p->clipLeft; x < p->clipRight; x++) {
                if (p->aux_tiled) {
                    int xt = (x & 63) | ((x >> 6) * 128 * 32 / 2);
                    row[xt] = depth;
                } else {
                    row[x] = depth;
                }
            }
        }
    }
}

/* =========================================================================
 * SwapBuffer — flip front/back buffers at vblank
 *
 * Ported from 86Box swapbufferCMD handler in vid_voodoo_reg.c and
 * voodoo_callback() in vid_voodoo_display.c.
 *
 * swap_interval: number of vblanks to wait before flipping (0 = immediate).
 * ========================================================================= */
void voodoo3_swap_buffer(Voodoo3State *s, uint32_t val)
{
    s->swap_interval = (int)((val >> 1) & 0xff);
    s->swap_offset   = s->params.draw_offset;
    s->swap_pending  = true;
    s->retrace_count = 0;
    s->frame_count++;
}

/* Called from vblank timer — perform the actual flip if interval elapsed */
void voodoo3_do_swap_if_pending(Voodoo3State *s)
{
    if (!s->swap_pending) return;

    s->retrace_count++;
    if (s->retrace_count <= s->swap_interval) return;

    /* Perform buffer flip */
    s->params.front_offset = s->swap_offset;
    s->swap_pending        = false;
    s->retrace_count       = 0;
    s->desktop_start       = s->swap_offset;
    /*
     * After a 3D swap the display scanout starts at the new front buffer.
     * Keep desktop_stride in sync with the current row_width so the display
     * blit uses the correct scanline pitch.  If row_width is 0 (not yet set
     * by colBufferStride), fall back to desktop_stride.
     */
    if (s->params.row_width == 0)
        s->params.row_width = s->desktop_stride;

    /* Mark all lines dirty so they are redrawn */
    memset(s->dirty_line, 1, sizeof(s->dirty_line));
}

/* =========================================================================
 * Display output with dirty-line tracking
 *
 * Ported from 86Box voodoo_callback() in vid_voodoo_display.c.
 *
 * Rather than blitting the entire framebuffer every vblank (expensive),
 * we only update lines that have been written by the rasterizer.
 * Each scanline rendered sets dirty_line[y]=1; we clear it after copying.
 *
 * This is the QEMU equivalent of 86Box's dirty_line[] + svga_doblit().
 * ========================================================================= */
void voodoo3_update_display_dirty(Voodoo3State *s)
{
    if (!s->display_enabled || s->screen_width <= 0 || s->screen_height <= 0)
        return;

    DisplaySurface *surf = qemu_console_surface(s->con);
    int w = s->screen_width;
    int h = s->screen_height;

    if (surface_width(surf) != w || surface_height(surf) != h) {
        qemu_console_resize(s->con, w, h);
        surf = qemu_console_surface(s->con);
        memset(s->dirty_line, 1, sizeof(s->dirty_line));
    }

    uint8_t *dst_base  = surface_data(surf);
    int      dst_bpp   = surface_bytes_per_pixel(surf);
    int      dst_pitch = surface_stride(surf);

    int dirty_lo = h, dirty_hi = -1;

    for (int y = 0; y < h && y < V3_DIRTY_LINES; y++) {
        if (!s->dirty_line[y]) continue;
        s->dirty_line[y] = 0;

        uint8_t *dst_row = dst_base + (size_t)y * dst_pitch;

        /*
         * Tiled framebuffer layout (Banshee/Voodoo3):
         *
         * The framebuffer is divided into 128-byte × 32-row tile strips.
         * For a pixel at (x, y) with bpp bytes per pixel:
         *
         *   tile_col        = x / (128 / bpp)     — which 128-byte column strip
         *   within_strip_x  = x % (128 / bpp)     — pixel offset within strip
         *   tile_row_group  = y / 32               — which group of 32 rows
         *   within_strip_y  = y % 32               — row within strip group
         *
         *   offset = tile_row_group * row_width          (row_width = num_tile_cols * 128 * 32)
         *          + tile_col       * 128 * 32
         *          + within_strip_y * 128
         *          + within_strip_x * bpp
         *
         * In non-tiled mode: offset = y * row_width + x * bpp  (linear)
         *
         * The row_width field holds the full byte-width of one 32-row band:
         *   non-tiled: bytes per scanline
         *   tiled:     num_tile_cols * 128 * 32
         */

        if (!s->params.col_tiled) {
            /* ---- Non-tiled: linear scanline ---- */
            const uint8_t *src_row = s->fb_mem + s->params.front_offset
                                   + (size_t)y * s->params.row_width;

            switch (s->pix_format) {
            case 0: /* 8bpp — palette lookup */
            {
                const uint8_t *src = src_row;
                if (dst_bpp == 4) {
                    uint32_t *dst = (uint32_t *)dst_row;
                    for (int x = 0; x < w; x++) {
                        dst[x] = 0xff000000u | s->pallook[src[x]];
                    }
                } else if (dst_bpp == 3) {
                    uint8_t *dst = dst_row;
                    for (int x = 0; x < w; x++) {
                        uint32_t c = s->pallook[src[x]];
                        dst[x*3+0] =  c        & 0xff;
                        dst[x*3+1] = (c >>  8) & 0xff;
                        dst[x*3+2] = (c >> 16) & 0xff;
                    }
                }
                break;
            }
            case 1: /* RGB565 */
            {
                const uint16_t *src = (const uint16_t *)src_row;
                if (dst_bpp == 4) {
                    uint32_t *dst = (uint32_t *)dst_row;
                    for (int x = 0; x < w; x++) {
                        uint16_t px = src[x];
                        uint32_t r  = ((px >> 11) & 0x1f); r = (r << 3) | (r >> 2);
                        uint32_t g  = ((px >>  5) & 0x3f); g = (g << 2) | (g >> 4);
                        uint32_t b  =  (px        & 0x1f); b = (b << 3) | (b >> 2);
                        dst[x] = rgb_to_pixel32bgr(r, g, b);
                    }
                } else if (dst_bpp == 2) {
                    memcpy(dst_row, src, (size_t)w * 2);
                } else {
                    uint8_t *dst = dst_row;
                    for (int x = 0; x < w; x++) {
                        uint16_t px = src[x];
                        uint32_t r  = (((px >> 11) & 0x1f) * 255 / 31);
                        uint32_t g  = (((px >>  5) & 0x3f) * 255 / 63);
                        uint32_t b  = ((px & 0x1f) * 255 / 31);
                        uint32_t out = 0xff000000u | (r << 16) | (g << 8) | b;
                        memcpy(dst + x * dst_bpp, &out, dst_bpp > 4 ? 4 : dst_bpp);
                    }
                }
                break;
            }
            case 2: /* RGB24 (packed 3-byte) */
            {
                const uint8_t *src = src_row;
                if (dst_bpp == 4) {
                    uint32_t *dst = (uint32_t *)dst_row;
                    for (int x = 0; x < w; x++) {
                        uint32_t b = src[x*3+0];
                        uint32_t g = src[x*3+1];
                        uint32_t r = src[x*3+2];
                        dst[x] = 0xff000000u | (r << 16) | (g << 8) | b;
                    }
                } else {
                    memcpy(dst_row, src, (size_t)w * 3);
                }
                break;
            }
            case 3: /* RGB32 / XRGB8888 */
            {
                const uint32_t *src = (const uint32_t *)src_row;
                if (dst_bpp == 4) {
                    memcpy(dst_row, src, (size_t)w * 4);
                } else if (dst_bpp == 3) {
                    uint8_t *dst = dst_row;
                    for (int x = 0; x < w; x++) {
                        uint32_t px = src[x];
                        dst[x*3+0] =  px        & 0xff;
                        dst[x*3+1] = (px >>  8) & 0xff;
                        dst[x*3+2] = (px >> 16) & 0xff;
                    }
                }
                break;
            }
            default: break;
            }

        } else {
            /* ---- Tiled mode: decode per-pixel tile address ---- */
            /*
             * bpp in bytes for the source format.
             * Note: pix_format 2 (RGB24) is 3 bytes — unusual with tiling,
             * but we handle it for completeness.
             */
            int src_bpp;
            switch (s->pix_format) {
            case 0:  src_bpp = 1; break;
            case 1:  src_bpp = 2; break;
            case 2:  src_bpp = 3; break;
            default: src_bpp = 4; break;
            }

            /*
             * pixels_per_strip = 128 / src_bpp
             * For RGB565 (2 bpp): 64 pixels per 128-byte strip
             * For RGB32  (4 bpp): 32 pixels per 128-byte strip
             */
            int pix_per_strip = 128 / src_bpp;

            /* Row-group base: which band of 32 rows we are in */
            size_t row_group_base = (size_t)(y >> 5) * s->params.row_width;
            /* Within-strip row offset (128 bytes per row within the strip) */
            size_t within_row    = (size_t)(y & 31) * 128;

            const uint8_t *fb_base = s->fb_mem + s->params.front_offset
                                   + row_group_base + within_row;

            uint8_t *dst8 = dst_row;

            for (int x = 0; x < w; x++) {
                /* Which 128-byte column strip, and pixel within it */
                int tile_col   = x / pix_per_strip;
                int strip_x    = x % pix_per_strip;

                /* Byte offset from fb_base to this pixel */
                const uint8_t *px_ptr = fb_base
                    + (size_t)tile_col * (128u * 32u)
                    + (size_t)strip_x  * src_bpp;

                uint32_t r, g, b, out;
                switch (s->pix_format) {
                case 0: /* 8bpp palette */
                    out = 0xff000000u | s->pallook[px_ptr[0]];
                    break;
                case 1: /* RGB565 */
                {
                    uint16_t px;
                    memcpy(&px, px_ptr, 2);
                    r = (px >> 11) & 0x1f; r = (r << 3) | (r >> 2);
                    g = (px >>  5) & 0x3f; g = (g << 2) | (g >> 4);
                    b =  px        & 0x1f; b = (b << 3) | (b >> 2);
                    out = 0xff000000u | (r << 16) | (g << 8) | b;
                    break;
                }
                case 2: /* RGB24 */
                    b = px_ptr[0]; g = px_ptr[1]; r = px_ptr[2];
                    out = 0xff000000u | (r << 16) | (g << 8) | b;
                    break;
                default: /* RGB32 */
                    memcpy(&out, px_ptr, 4);
                    out |= 0xff000000u;
                    break;
                }

                if (dst_bpp == 4) {
                    ((uint32_t *)dst8)[x] = out;
                } else if (dst_bpp == 3) {
                    dst8[x*3+0] =  out        & 0xff;
                    dst8[x*3+1] = (out >>  8) & 0xff;
                    dst8[x*3+2] = (out >> 16) & 0xff;
                } else if (dst_bpp == 2) {
                    /* Pack back to RGB565 */
                    uint16_t p565 = (uint16_t)(
                        (((out >> 16) & 0xff) >> 3) << 11 |
                        (((out >>  8) & 0xff) >> 2) << 5  |
                        (( out        & 0xff) >> 3));
                    ((uint16_t *)dst8)[x] = p565;
                }
            }
        }

        if (y < dirty_lo) dirty_lo = y;
        if (y > dirty_hi) dirty_hi = y;
    }

    if (dirty_lo <= dirty_hi) {
        voodoo3_draw_cursor(s, dst_base, dst_bpp, dst_pitch,
                            w, dirty_lo, dirty_hi);
        dpy_gfx_update(s->con, 0, dirty_lo, w, dirty_hi - dirty_lo + 1);
    }
}

/* =========================================================================
 * Hardware cursor compositing
 * Ported from 86Box banshee_hwcursor_draw() in vid_voodoo_banshee.c.
 *
 * 64x64 sprite in VRAM at cur_pat_addr, 16 bytes/row:
 *   bytes 0..7  = plane0 (mask/AND), bytes 8..15 = plane1 (color/XOR)
 * Mode (VIDPROCCFG bit 1):  0=Windows AND/XOR,  1=X11 mask/color
 * Position: cur_x/cur_y already de-biased (screen pixel 0,0 = value 64 in reg).
 * ========================================================================= */
void voodoo3_draw_cursor(Voodoo3State *s,
                         uint8_t *dst_base, int dst_bpp, int dst_pitch,
                         int w, int dirty_lo, int dirty_hi)
{
    if (!s->cursor_ena || !s->fb_mem) return;

    int cx      = s->cur_x;
    int cy      = s->cur_y;      /* already >= 0 after yoff adjustment */
    int yoff    = s->cur_yoff;   /* sprite rows already consumed (top-clip) */
    int x11_mode = !!(s->vidProcCfg & (1u << 1));  /* VIDPROCCFG_CURSOR_MODE */
    uint32_t col0 = s->cur_c0;
    uint32_t col1 = s->cur_c1;

    /* Vertical range on screen */
    int scr_y0 = cy;
    int scr_y1 = cy + (64 - yoff);
    if (scr_y0 > dirty_hi || scr_y1 < dirty_lo) return;
    if (scr_y0 < dirty_lo) scr_y0 = dirty_lo;
    if (scr_y1 > dirty_hi + 1) scr_y1 = dirty_hi + 1;
    if (scr_y1 > s->screen_height) scr_y1 = s->screen_height;

    for (int row = scr_y0; row < scr_y1; row++) {
        /*
         * sprite_row: which row of the 64-row sprite bitmap to use.
         * yoff rows were already skipped (cursor partially above screen top).
         * cursor_buf was loaded starting at row yoff, so cursor_buf[0]
         * corresponds to sprite row yoff → screen row cy.
         * buf_row = row - cy  (0-based index into cursor_buf).
         */
        int buf_row = row - cy;
        if (buf_row < 0 || buf_row >= (64 - yoff)) continue;

        /*
         * Each sprite row = 16 bytes: 8 bytes plane0 + 8 bytes plane1.
         * Ported from 86Box banshee_hwcursor_draw():
         *   plane0[c] = vram[addr + c]        (c = 0..7)
         *   plane1[c] = vram[addr + c + 8]    (c = 0..7)
         *   addr += 16  after each row
         */
        const uint8_t *plane0 = s->cursor_buf + (size_t)buf_row * 16;
        const uint8_t *plane1 = s->cursor_buf + (size_t)buf_row * 16 + 8;
        uint8_t *dst_row = dst_base + (size_t)row * dst_pitch;

        int x_off = cx;   /* screen X of the first cursor pixel in this row */

        for (int bx = 0; bx < 64; bx += 8) {
            if (x_off > -8) {
                uint8_t p0byte = plane0[bx >> 3];
                uint8_t p1byte = plane1[bx >> 3];
                for (int xx = 0; xx < 8; xx++) {
                    int scr_x = x_off + xx;
                    int p0 = (p0byte >> 7) & 1;
                    int p1 = (p1byte >> 7) & 1;
                    p0byte <<= 1;
                    p1byte <<= 1;

                    if (scr_x < 0 || scr_x >= w) continue;

                    uint32_t pixel;
                    if (x11_mode) {
                        /* X11 mode: plane0=mask (1=draw), plane1=color */
                        if (!p0) continue;
                        pixel = p1 ? col1 : col0;
                    } else {
                        /* Windows AND/XOR mode (86Box default):
                         *   p0=0, p1=0  → transparent (skip)
                         *   p0=0, p1=1  → color1 (foreground)
                         *   p1=0, p0=1  → color0 (background)  -- wait, no:
                         *
                         * 86Box logic:
                         *   if !(plane0 & bit) → draw plane1 ? col1 : col0
                         *   else if (plane1 & bit) → XOR pixel with 0xffffff
                         *   else → transparent
                         */
                        if (!p0) {
                            pixel = p1 ? col1 : col0;
                        } else if (p1) {
                            /* XOR invert */
                            if (dst_bpp == 4)
                                ((uint32_t *)dst_row)[scr_x] ^= 0x00ffffffu;
                            continue;
                        } else {
                            continue;  /* transparent */
                        }
                    }

                    if (dst_bpp == 4) {
                        ((uint32_t *)dst_row)[scr_x] = 0xff000000u | pixel;
                    } else if (dst_bpp == 3) {
                        dst_row[scr_x*3+0] =  pixel        & 0xff;
                        dst_row[scr_x*3+1] = (pixel >>  8) & 0xff;
                        dst_row[scr_x*3+2] = (pixel >> 16) & 0xff;
                    } else if (dst_bpp == 2) {
                        ((uint16_t *)dst_row)[scr_x] = (uint16_t)(
                            (((pixel>>16)&0xff)>>3)<<11 |
                            (((pixel>> 8)&0xff)>>2)<< 5 |
                            (( pixel     &0xff)>>3));
                    }
                }
            }
            x_off += 8;
        }
    }
}
