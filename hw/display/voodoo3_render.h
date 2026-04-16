/*
 * QEMU 3Dfx Voodoo 3 — Rasterizer + Setup interface header
 *
 * Copyright (C) 2026 <your name here>
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_VOODOO3_RENDER_H
#define HW_DISPLAY_VOODOO3_RENDER_H

/* Maximum LOD level (mipmap depth 0..8) */
#define V3_LOD_MAX  8

/* These functions are declared here for reference; callers must include
 * hw/display/voodoo3_int.h for the full type definitions. */

/* voodoo3_triangle() — rasterize one triangle (hw/display/voodoo3_render.c) */
struct voodoo3_params_t;
struct Voodoo3State;
void voodoo3_triangle(struct Voodoo3State *s, const struct voodoo3_params_t *p);

/* voodoo3_triangle_setup() — floating-point setup → fixed-point params */
void voodoo3_triangle_setup(struct Voodoo3State *s);

#endif /* HW_DISPLAY_VOODOO3_RENDER_H */
