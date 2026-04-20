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
                        /*
                         * QEMU DisplaySurface is PIXMAN_x8r8g8b8 (little-endian):
                         * word = 0x00RRGGBB.  Use rgb_to_pixel32() not bgr variant.
                         */
                        dst[x] = rgb_to_pixel32(r, g, b);
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
        /* Overlay: composite video surface before cursor and screen update */
        voodoo3_overlay_draw(s, dst_base, dst_bpp, dst_pitch,
                             w, dirty_lo, dirty_hi);
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

/* =========================================================================
 * Video Overlay compositing
 *
 * Ported from 86Box banshee_overlay_draw() in vid_voodoo_banshee.c.
 *
 * The Banshee/V3 hardware overlay composites a separate video surface on
 * top of the desktop framebuffer.  The overlay surface lives in VRAM at
 * an address derived from vidDesktopOverlayStride bits[30:16] × pitch.
 *
 * Pixel formats (ov.pix_fmt / OVERLAY_FMT_*):
 *   YUYV422 (5): packed [Y0 Cr Y1 Cb] — 4 bytes per 2 pixels
 *   UYVY422 (6): packed [Cr Y0 Cb Y1] — 4 bytes per 2 pixels
 *   RGB565  (1): standard 16-bit RGB, optional CLUT look-up
 *   RGB565_DITHER (7): same, with dither hint (treated as RGB565 here)
 *
 * YCbCr → RGB conversion (ITU-R BT.601, same coefficients as 86Box):
 *   dR = (359 * Cr) >> 8
 *   dG = (88 * Cb + 183 * Cr) >> 8
 *   dB = (453 * Cb) >> 8
 *   R = clamp(Y + dR), G = clamp(Y - dG), B = clamp(Y + dB)
 *
 * Scaling:
 *   H: ov.vidOverlayDudx = source X step in 20.12 fixed-point
 *      (1<<20 = no scaling, <1<<20 = upscale, >1<<20 = downscale)
 *   V: ov.vidOverlayDvdy = source Y step in 20.12 fixed-point
 *      ov.src_y accumulates; integer part = source line index
 *
 * Filtering:
 *   BILINEAR: vertical bilinear between current and next source line.
 *   POINT / DITHER4X4 / DITHER2X2: nearest-neighbour (no filter tables).
 *
 * Chroma-key: if vidChromaKeyMin/Max match the desktop pixel at the same
 * screen coordinate, the overlay pixel is written; otherwise skipped.
 *
 * Called from voodoo3_update_display_dirty() after the desktop scanout
 * and before the cursor, for every dirty scanline inside the overlay rect.
 *
 * 86Box reference: banshee_overlay_draw(), DECODE_RGB565, DECODE_YUYV422,
 *                 DECODE_UYUV422, OVERLAY_SAMPLE, banshee_chroma_key().
 * ========================================================================= */

/* YCbCr clamping macro — identical to 86Box CLAMP(x) in overlay context */
#ifndef OV_CLAMP
#define OV_CLAMP(x) ((x) < 0 ? 0 : ((x) > 255 ? 255 : (x)))
#endif

/*
 * ov_decode_line — decode one source scanline into buf[buf_idx][].
 *
 * Ported from 86Box DECODE_RGB565 / DECODE_YUYV422 / DECODE_UYUV422 /
 * DECODE_RGB565_TILED macros.
 *
 * src_addr: byte offset into fb_mem of the start of this source line.
 * buf_idx:  which of the two decode buffers to fill (0 or 1).
 * The decoded output is XRGB8888 stored as (R<<16)|(G<<8)|B.
 */
static void ov_decode_line(Voodoo3State *s, uint32_t src_addr, int buf_idx)
{
    uint32_t *buf   = s->ov.buf[buf_idx];
    int       bytes = s->ov.overlay_bytes;
    int       wp    = 0;

    /* CLUT base: pallook[0..255] or pallook[256..511] per OVERLAY_CLUT_SEL */
    const uint32_t *clut = s->pallook +
        ((s->vidProcCfg & VIDPROCCFG_OVERLAY_CLUT_SEL) ? 256 : 0);
    bool clut_bypass = !!(s->vidProcCfg & VIDPROCCFG_OVERLAY_CLUT_BYPASS);
    bool tiled       = !!(s->vidProcCfg & VIDPROCCFG_OVERLAY_TILE);

    switch (s->ov.pix_fmt) {

    /* ---- YUYV422: Y0 Cr Y1 Cb per 4 bytes → 2 pixels ---- */
    case OVERLAY_FMT_YUYV422:
        for (int c = 0; c < bytes; c += 4) {
            uint32_t off = (src_addr + c) & (s->fb_size - 1);
            if (off + 3 >= s->fb_size) break;
            int y1 = s->fb_mem[off + 0];
            int Cr = (int)s->fb_mem[off + 1] - 0x80;
            int y2 = s->fb_mem[off + 2];
            int Cb = (int)s->fb_mem[off + 3] - 0x80;
            int dR = (359 * Cr) >> 8;
            int dG = (88 * Cb + 183 * Cr) >> 8;
            int dB = (453 * Cb) >> 8;
            int r, g, b;
            r = OV_CLAMP(y1 + dR); g = OV_CLAMP(y1 - dG); b = OV_CLAMP(y1 + dB);
            buf[wp++] = (uint32_t)((r << 16) | (g << 8) | b);
            r = OV_CLAMP(y2 + dR); g = OV_CLAMP(y2 - dG); b = OV_CLAMP(y2 + dB);
            buf[wp++] = (uint32_t)((r << 16) | (g << 8) | b);
        }
        break;

    /* ---- UYVY422: Cr Y0 Cb Y1 per 4 bytes → 2 pixels ---- */
    case OVERLAY_FMT_UYVY422:
        for (int c = 0; c < bytes; c += 4) {
            uint32_t off = (src_addr + c) & (s->fb_size - 1);
            if (off + 3 >= s->fb_size) break;
            int Cr = (int)s->fb_mem[off + 0] - 0x80;
            int y1 = s->fb_mem[off + 1];
            int Cb = (int)s->fb_mem[off + 2] - 0x80;
            int y2 = s->fb_mem[off + 3];
            int dR = (359 * Cr) >> 8;
            int dG = (88 * Cb + 183 * Cr) >> 8;
            int dB = (453 * Cb) >> 8;
            int r, g, b;
            r = OV_CLAMP(y1 + dR); g = OV_CLAMP(y1 - dG); b = OV_CLAMP(y1 + dB);
            buf[wp++] = (uint32_t)((r << 16) | (g << 8) | b);
            r = OV_CLAMP(y2 + dR); g = OV_CLAMP(y2 - dG); b = OV_CLAMP(y2 + dB);
            buf[wp++] = (uint32_t)((r << 16) | (g << 8) | b);
        }
        break;

    /* ---- RGB565 (tiled and linear) ---- */
    case OVERLAY_FMT_565:
    case OVERLAY_FMT_565_DITHER:
        if (tiled) {
            /*
             * 86Box DECODE_RGB565_TILED: each 128-byte strip holds 64 pixels.
             * offset = (c & 127) + (c >> 7) * 128 * 32
             */
            for (int c = 0; c < bytes; c += 2) {
                uint32_t tile_off = (uint32_t)(c & 127) + ((uint32_t)(c >> 7) * 128u * 32u);
                uint32_t addr = (src_addr + tile_off) & (s->fb_size - 1);
                if (addr + 1 >= s->fb_size) break;
                uint16_t px;
                memcpy(&px, s->fb_mem + addr, 2);
                int r = (px      ) & 0x1f;
                int g = (px >>  5) & 0x3f;
                int b = (px >> 11) & 0x1f;
                if (clut_bypass)
                    buf[wp++] = (uint32_t)((r << 3) | (g << 10) | (b << 19));
                else
                    buf[wp++] = (clut[r << 3] & 0x0000ffu) |
                                (clut[g << 2] & 0x00ff00u) |
                                (clut[b << 3] & 0xff0000u);
            }
        } else {
            for (int c = 0; c < bytes; c += 2) {
                uint32_t addr = (src_addr + c) & (s->fb_size - 1);
                if (addr + 1 >= s->fb_size) break;
                uint16_t px;
                memcpy(&px, s->fb_mem + addr, 2);
                int r = (px      ) & 0x1f;
                int g = (px >>  5) & 0x3f;
                int b = (px >> 11) & 0x1f;
                if (clut_bypass)
                    buf[wp++] = (uint32_t)((r << 3) | (g << 10) | (b << 19));
                else
                    buf[wp++] = (clut[r << 3] & 0x0000ffu) |
                                (clut[g << 2] & 0x00ff00u) |
                                (clut[b << 3] & 0xff0000u);
            }
        }
        break;

    default:
        break;
    }
}

/*
 * ov_chroma_key — returns true if overlay should be drawn at screen (x, y).
 *
 * Ported from 86Box banshee_chroma_key() in vid_voodoo_banshee.c.
 * Reads the desktop pixel at the given screen coordinate and checks if
 * it falls within [vidChromaKeyMin, vidChromaKeyMax] per channel.
 * Returns true (draw overlay) if chroma match; XORed with vidProcCfg bit 6.
 *
 * When chroma-key is not configured (vidProcCfg bit 5 = 0), always returns
 * true (draw overlay everywhere in the overlay rectangle).
 */
static bool ov_chroma_key(Voodoo3State *s, int scr_x, int scr_y)
{
    /* bit 5 = chroma-key enable */
    if (!(s->vidProcCfg & (1u << 5)))
        return true;

    uint32_t desktop_stride = s->vidDesktopOverlayStride & 0x3fffu;
    uint32_t src_addr = s->desktop_start + (uint32_t)scr_y * desktop_stride;

    bool match = false;
    switch (s->pix_format) {
    case 0: { /* 8bpp palette index */
        uint8_t idx = s->fb_mem[(src_addr + scr_x) & (s->fb_size - 1)];
        uint8_t lo  = s->vidChromaKeyMin & 0xff;
        uint8_t hi  = s->vidChromaKeyMax & 0xff;
        match = (idx >= lo && idx <= hi);
        break;
    }
    case 1: { /* RGB565 */
        uint32_t addr = (src_addr + (uint32_t)scr_x * 2u) & (s->fb_size - 1);
        uint16_t px = 0;
        if (addr + 1 < s->fb_size) memcpy(&px, s->fb_mem + addr, 2);
        int r = px & 0x1f, g = (px >> 5) & 0x3f, b = px >> 11;
        match = r >= (int)((s->vidChromaKeyMin >> 11) & 0x1f) &&
                r <= (int)((s->vidChromaKeyMax >> 11) & 0x1f) &&
                g >= (int)((s->vidChromaKeyMin >>  5) & 0x3f) &&
                g <= (int)((s->vidChromaKeyMax >>  5) & 0x3f) &&
                b >= (int)( s->vidChromaKeyMin        & 0x1f) &&
                b <= (int)( s->vidChromaKeyMax        & 0x1f);
        break;
    }
    case 2: { /* RGB24 */
        uint32_t addr = (src_addr + (uint32_t)scr_x * 3u) & (s->fb_size - 1);
        uint8_t r = s->fb_mem[(addr    ) & (s->fb_size-1)];
        uint8_t g = s->fb_mem[(addr + 1) & (s->fb_size-1)];
        uint8_t b = s->fb_mem[(addr + 2) & (s->fb_size-1)];
        match = r >= ((s->vidChromaKeyMin >> 16) & 0xff) &&
                r <= ((s->vidChromaKeyMax >> 16) & 0xff) &&
                g >= ((s->vidChromaKeyMin >>  8) & 0xff) &&
                g <= ((s->vidChromaKeyMax >>  8) & 0xff) &&
                b >= ( s->vidChromaKeyMin        & 0xff) &&
                b <= ( s->vidChromaKeyMax        & 0xff);
        break;
    }
    case 3: { /* RGB32 */
        uint32_t addr = (src_addr + (uint32_t)scr_x * 4u) & (s->fb_size - 1);
        uint8_t r = s->fb_mem[(addr    ) & (s->fb_size-1)];
        uint8_t g = s->fb_mem[(addr + 1) & (s->fb_size-1)];
        uint8_t b = s->fb_mem[(addr + 2) & (s->fb_size-1)];
        match = r >= ((s->vidChromaKeyMin >> 16) & 0xff) &&
                r <= ((s->vidChromaKeyMax >> 16) & 0xff) &&
                g >= ((s->vidChromaKeyMin >>  8) & 0xff) &&
                g <= ((s->vidChromaKeyMax >>  8) & 0xff) &&
                b >= ( s->vidChromaKeyMin        & 0xff) &&
                b <= ( s->vidChromaKeyMax        & 0xff);
        break;
    }
    default:
        return true;
    }

    /* XOR with invert bit (vidProcCfg bit 6) */
    return match ^ !!(s->vidProcCfg & (1u << 6));
}

/*
 * voodoo3_overlay_draw — composite overlay for scanlines [dirty_lo..dirty_hi].
 *
 * Ported from 86Box banshee_overlay_draw() in vid_voodoo_banshee.c.
 *
 * Called from voodoo3_update_display_dirty() after the desktop blit and
 * before the cursor for every dirty region that intersects the overlay.
 *
 * Scaling model (20.12 fixed-point, same as 86Box):
 *   ov.src_y tracks the current source line (integer = y >> 20, frac = y & 0xfffff).
 *   For each display line, src_y += vidOverlayDvdy (V scale step).
 *   For each display pixel, src_x += vidOverlayDudx (H scale step).
 *   src_y and src_x are reset to 0 on each new frame (vblank callback).
 *
 * Filtering (VIDPROCCFG_FILTER_MODE_*):
 *   POINT / DITHER4X4 / DITHER2X2 → nearest-neighbour (86Box scrfilter path
 *   is not ported; the emulator-option path = nearest-neighbour is used).
 *   BILINEAR → vertical blend between buf[0] (current line) and buf[1]
 *   (next source line) using the fractional part of src_y as coefficient.
 */
void voodoo3_overlay_draw(Voodoo3State *s,
                          uint8_t *dst_base, int dst_bpp, int dst_pitch,
                          int scr_w, int dirty_lo, int dirty_hi)
{
    if (!s->ov.ena || !s->fb_mem) return;
    if (s->ov.size_x <= 0 || s->ov.size_y <= 0) return;
    if (s->ov.overlay_bytes <= 0) return;

    /* Overlay pitch: bits[30:16] of vidDesktopOverlayStride */
    uint32_t ov_pitch = (s->vidDesktopOverlayStride & VID_STRIDE_OVERLAY_MASK)
                        >> VID_STRIDE_OVERLAY_SHIFT;
    if (s->vidProcCfg & VIDPROCCFG_OVERLAY_TILE)
        ov_pitch *= 128u * 32u;

    /* Base VRAM address of overlay surface — addr word 0 of overlay region.
     * 86Box: svga->overlay_latch.addr = (vidDesktopOverlayStride bits[30:16]) × stride.
     * We use ov_pitch as both the VRAM base stride and the line pitch.
     * The actual start address comes from the start coordinate relative
     * to the desktop start, consistent with 86Box behaviour. */
    uint32_t ov_base = s->desktop_start;   /* overlay shares VRAM with desktop */

    uint32_t filter = s->vidProcCfg & VIDPROCCFG_FILTER_MODE_MASK;
    bool h_scale    = !!(s->vidProcCfg & VIDPROCCFG_H_SCALE_ENABLE);
    bool v_scale    = !!(s->vidProcCfg & VIDPROCCFG_V_SCALE_ENABLE);

    int ov_x0 = s->ov.start_x;
    int ov_y0 = s->ov.start_y;
    int ov_w  = s->ov.size_x;

    /* Clamp overlay to screen */
    if (ov_x0 < 0) ov_x0 = 0;
    if (ov_x0 >= scr_w) return;
    if (ov_x0 + ov_w > scr_w) ov_w = scr_w - ov_x0;

    for (int scr_y = dirty_lo; scr_y <= dirty_hi; scr_y++) {
        /* Only draw within the overlay vertical extent */
        if (scr_y < s->ov.start_y || scr_y >= s->ov.end_y)
            continue;

        /* Source line index from accumulated src_y */
        int   src_line  = s->ov.src_y >> 20;
        /* Y fractional part for bilinear filter (0..0xfffff → 0..0xffff) */
        unsigned int y_coeff = (unsigned int)((s->ov.src_y & 0xfffffu) >> 4);

        /* Compute source addresses for current and next lines */
        uint32_t src_addr0, src_addr1;
        if (s->vidProcCfg & VIDPROCCFG_OVERLAY_TILE) {
            /* Tiled: 86Box: (y & 31)*128 + (y >> 5)*pitch */
            src_addr0 = ov_base + (uint32_t)(src_line       & 31) * 128u
                        + (uint32_t)(src_line       >> 5) * ov_pitch;
            src_addr1 = ov_base + (uint32_t)((src_line + 1) & 31) * 128u
                        + (uint32_t)((src_line + 1) >> 5) * ov_pitch;
        } else {
            src_addr0 = ov_base + (uint32_t) src_line       * ov_pitch;
            src_addr1 = ov_base + (uint32_t)(src_line + 1)  * ov_pitch;
        }

        /* Decode current line into buf[0] */
        ov_decode_line(s, src_addr0, 0);
        /* Decode next line into buf[1] for bilinear */
        if (filter == VIDPROCCFG_FILTER_MODE_BILINEAR)
            ov_decode_line(s, src_addr1, 1);

        uint8_t *dst_row = dst_base + (size_t)scr_y * dst_pitch;
        uint32_t src_x   = 0;  /* 20.12 fixed-point horizontal position */

        for (int x = 0; x < ov_w; x++) {
            int scr_x   = ov_x0 + x;
            int src_xi  = h_scale ? (int)(src_x >> 20) : x;

            /* Clamp src index to decoded buffer size */
            if (src_xi < 0) src_xi = 0;
            if (src_xi >= 4096) src_xi = 4095;

            uint32_t pixel;
            if (filter == VIDPROCCFG_FILTER_MODE_BILINEAR) {
                /* Vertical bilinear: blend buf[0][src_xi] and buf[1][src_xi] */
                uint32_t s0 = s->ov.buf[0][src_xi];
                uint32_t s1 = s->ov.buf[1][src_xi];
                unsigned int inv = 0x10000u - y_coeff;
                int r = (int)((((s0 >> 16) & 0xff) * inv +
                               ((s1 >> 16) & 0xff) * y_coeff) >> 16);
                int g = (int)((((s0 >>  8) & 0xff) * inv +
                               ((s1 >>  8) & 0xff) * y_coeff) >> 16);
                int b = (int)((( s0        & 0xff) * inv +
                               ( s1        & 0xff) * y_coeff) >> 16);
                pixel = (uint32_t)((r << 16) | (g << 8) | b);
            } else {
                /* Point / dither: nearest-neighbour */
                pixel = s->ov.buf[0][src_xi];
            }

            /* Chroma-key test */
            if (!ov_chroma_key(s, scr_x, scr_y - ov_y0))
                goto next_pixel;

            /* Write pixel to console surface */
            if (dst_bpp == 4) {
                ((uint32_t *)dst_row)[scr_x] = 0xff000000u | pixel;
            } else if (dst_bpp == 3) {
                dst_row[scr_x * 3 + 0] =  pixel        & 0xff;
                dst_row[scr_x * 3 + 1] = (pixel >>  8) & 0xff;
                dst_row[scr_x * 3 + 2] = (pixel >> 16) & 0xff;
            }

next_pixel:
            if (h_scale) src_x += s->ov.vidOverlayDudx;
        }

        /* Advance source Y accumulator */
        if (v_scale)
            s->ov.src_y += (int32_t)s->ov.vidOverlayDvdy;
        else
            s->ov.src_y += (1 << 20);  /* 1.0 in 20.12 = advance one source line */
    }
}
