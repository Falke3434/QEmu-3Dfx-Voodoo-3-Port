/*
 * QEMU 3Dfx Voodoo 3 — rasterizer and setup interface
 *
 * QEMU port: https://github.com/Falke3434/QEmu-3Dfx-Voodoo-3-Port
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_VOODOO3_RENDER_H
#define HW_DISPLAY_VOODOO3_RENDER_H

/* Maximum LOD level (mipmap depth 0..8) */
#define V3_LOD_MAX  8

/* voodoo3_triangle() — rasterize one triangle (hw/display/voodoo3_render.c) */
struct voodoo3_params_t;
struct Voodoo3State;
void voodoo3_triangle(struct Voodoo3State *s, const struct voodoo3_params_t *p,
                      int odd_even);

/* voodoo3_triangle_setup() — floating-point setup → fixed-point params */
void voodoo3_triangle_setup(struct Voodoo3State *s);

/*
 * voodoo3_fb_writel() — LFB pixel write through the 3D pipeline (3D LFB
 * aperture of BAR0, CMDFIFO packet 5).  Decodes the lfbMode pixel format,
 * applies stipple / depth / chroma / alpha tests and blending, then writes
 * RGB565.  Ported from 86Box voodoo_fb_writel().
 *
 * addr = byte offset within the 3D LFB aperture
 * val  = 32-bit pixel data as written by the guest
 */
void voodoo3_fb_writel(struct Voodoo3State *s, uint32_t addr, uint32_t val);
/* 16-bit LFB write (one pixel / one depth value) */
void voodoo3_fb_writew(struct Voodoo3State *s, uint32_t addr, uint16_t val);

#endif /* HW_DISPLAY_VOODOO3_RENDER_H */
