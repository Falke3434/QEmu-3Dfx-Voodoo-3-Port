/*
 * QEMU 3Dfx Voodoo 3 — Triangle Setup (sBeginTriCMD / sDrawTriCMD path)
 *
 * Ported from 86Box vid_voodoo_setup.c
 * Original author: Sarah Walker <https://pcem-emulator.co.uk/>
 * Copyright (C) 2008-2024 Sarah Walker and 86Box contributors
 * Copyright (C) 2026 <your name here>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This file implements the floating-point vertex setup engine that converts
 * the sVx/sVy/sRed/... staging vertex (written via SST_sVx ... SST_sT1)
 * into fixed-point triangle parameters, then calls voodoo3_queue_triangle().
 *
 * The algorithm is a direct port of voodoo_triangle_setup() from 86Box.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/display/voodoo3_int.h"
#include "hw/display/voodoo3_render.h"
#include "hw/display/voodoo3_texture.h"

/* Setup-mode flags (from vid_voodoo_regs.h) */
#define SETUPMODE_RGB           (1 << 0)
#define SETUPMODE_ALPHA         (1 << 1)
#define SETUPMODE_Z             (1 << 2)
#define SETUPMODE_Wb            (1 << 3)
#define SETUPMODE_W0            (1 << 4)
#define SETUPMODE_S0_T0         (1 << 5)
#define SETUPMODE_W1            (1 << 6)
#define SETUPMODE_S1_T1         (1 << 7)
#define SETUPMODE_STRIP_MODE    (1 << 8)
#define SETUPMODE_CULLING_ENABLE    (1 << 9)
#define SETUPMODE_CULLING_SIGN      (1 << 10)
#define SETUPMODE_DISABLE_PINGPONG  (1 << 11)

/*
 * voodoo3_triangle_setup — convert the three floating-point setup vertices
 * in s->verts[0..2] into a fixed-point voodoo3_params_t and submit to the
 * triangle parameter ring buffer.
 *
 * Ported 1:1 from 86Box voodoo_triangle_setup().
 */
/* Forward declaration - defined in voodoo3.c */
/* voodoo3_queue_triangle declared in voodoo3_int.h */

void voodoo3_triangle_setup(Voodoo3State *s)
{
    /* Local aliases */
    const voodoo3_vert_t *v = s->verts;   /* [0]=A, [1]=B, [2]=C */

    /* Sort vertices so that A.y <= B.y <= C.y (screen space, Y grows down) */
    int va = 0, vb = 1, vc = 2;

    if (v[0].sVy < v[1].sVy) {
        if (v[1].sVy < v[2].sVy) {
            va = 0; vb = 1; vc = 2;
        } else {
            if (v[0].sVy < v[2].sVy) { va = 0; vb = 2; vc = 1; }
            else                      { va = 2; vb = 0; vc = 1; }
        }
    } else {
        if (v[1].sVy < v[2].sVy) {
            if (v[0].sVy < v[2].sVy) { va = 1; vb = 0; vc = 2; }
            else                      { va = 1; vb = 2; vc = 0; }
        } else {
            va = 2; vb = 1; vc = 0;
        }
    }

    /* Compute signed triangle area */
    float dxAB = v[0].sVx - v[1].sVx;
    float dxBC = v[1].sVx - v[2].sVx;
    float dyAB = v[0].sVy - v[1].sVy;
    float dyBC = v[1].sVy - v[2].sVy;
    float area  = dxAB * dyBC - dxBC * dyAB;

    if (area == 0.0f) return;

    /* Back-face / front-face culling */
    if (s->sSetupMode & SETUPMODE_CULLING_ENABLE) {
        int cull_sign = !!(s->sSetupMode & SETUPMODE_CULLING_SIGN);
        int sign      = (area < 0.0f) ? 1 : 0;

        if ((s->sSetupMode & (SETUPMODE_CULLING_ENABLE | SETUPMODE_DISABLE_PINGPONG))
                == SETUPMODE_CULLING_ENABLE
            && s->cull_pingpong)
            cull_sign = !cull_sign;

        if (cull_sign && sign)  return;
        if (!cull_sign && !sign) return;
    }

    /* Re-compute with sorted vertices */
    dxAB = v[va].sVx - v[vb].sVx;
    dxBC = v[vb].sVx - v[vc].sVx;
    dyAB = v[va].sVy - v[vb].sVy;
    dyBC = v[vb].sVy - v[vc].sVy;
    area = dxAB * dyBC - dxBC * dyAB;

    /* Reciprocal area for gradient computation */
    float inv = 1.0f / area;
    dxAB *= inv; dxBC *= inv;
    dyAB *= inv; dyBC *= inv;

    /* Helper macro: compute dX/dY gradient for a scalar attribute f */
#define GRAD_X(fa, fb, fc) \
    ((((fa) - (fb)) * dyBC - ((fb) - (fc)) * dyAB))
#define GRAD_Y(fa, fb, fc) \
    ((((fb) - (fc)) * dxAB - ((fa) - (fb)) * dxBC))

    /* Build parameter snapshot (copy from current params to preserve
     * render-state regs that are NOT set by the setup engine) */
    voodoo3_params_t p = s->params;

    /* Fixed-point vertex coordinates (4.12) */
    p.vertexAx = (int32_t)(int16_t)((int32_t)(v[va].sVx * 16.0f) & 0xffff);
    p.vertexAy = (int32_t)(int16_t)((int32_t)(v[va].sVy * 16.0f) & 0xffff);
    p.vertexBx = (int32_t)(int16_t)((int32_t)(v[vb].sVx * 16.0f) & 0xffff);
    p.vertexBy = (int32_t)(int16_t)((int32_t)(v[vb].sVy * 16.0f) & 0xffff);
    p.vertexCx = (int32_t)(int16_t)((int32_t)(v[vc].sVx * 16.0f) & 0xffff);
    p.vertexCy = (int32_t)(int16_t)((int32_t)(v[vc].sVy * 16.0f) & 0xffff);

    if (p.vertexAy > p.vertexBy || p.vertexBy > p.vertexCy) {
        qemu_log_mask(LOG_UNIMP,
            "voodoo3_triangle_setup: wrong order Ay=%d By=%d Cy=%d\n",
            p.vertexAy, p.vertexBy, p.vertexCy);
        return;
    }

    /* RGB colour gradients */
    if (s->sSetupMode & SETUPMODE_RGB) {
        p.startR = (int32_t)(v[va].sRed   * 4096.0f);
        p.startG = (int32_t)(v[va].sGreen * 4096.0f);
        p.startB = (int32_t)(v[va].sBlue  * 4096.0f);
        p.dRdX   = (int32_t)(GRAD_X(v[va].sRed,   v[vb].sRed,   v[vc].sRed)   * 4096.0f);
        p.dGdX   = (int32_t)(GRAD_X(v[va].sGreen, v[vb].sGreen, v[vc].sGreen) * 4096.0f);
        p.dBdX   = (int32_t)(GRAD_X(v[va].sBlue,  v[vb].sBlue,  v[vc].sBlue)  * 4096.0f);
        p.dRdY   = (int32_t)(GRAD_Y(v[va].sRed,   v[vb].sRed,   v[vc].sRed)   * 4096.0f);
        p.dGdY   = (int32_t)(GRAD_Y(v[va].sGreen, v[vb].sGreen, v[vc].sGreen) * 4096.0f);
        p.dBdY   = (int32_t)(GRAD_Y(v[va].sBlue,  v[vb].sBlue,  v[vc].sBlue)  * 4096.0f);
    }

    /* Alpha gradient */
    if (s->sSetupMode & SETUPMODE_ALPHA) {
        p.startA = (int32_t)(v[va].sAlpha * 4096.0f);
        p.dAdX   = (int32_t)(GRAD_X(v[va].sAlpha, v[vb].sAlpha, v[vc].sAlpha) * 4096.0f);
        p.dAdY   = (int32_t)(GRAD_Y(v[va].sAlpha, v[vb].sAlpha, v[vc].sAlpha) * 4096.0f);
    }

    /* Z gradient */
    if (s->sSetupMode & SETUPMODE_Z) {
        p.startZ = (int32_t)(v[va].sVz * 4096.0f);
        p.dZdX   = (int32_t)(GRAD_X(v[va].sVz, v[vb].sVz, v[vc].sVz) * 4096.0f);
        p.dZdY   = (int32_t)(GRAD_Y(v[va].sVz, v[vb].sVz, v[vc].sVz) * 4096.0f);
    }

    /* W (broadcast to both TMUs) */
    if (s->sSetupMode & SETUPMODE_Wb) {
        p.startW          = (int64_t)(v[va].sWb * 4294967296.0f);
        p.dWdX            = (int64_t)(GRAD_X(v[va].sWb, v[vb].sWb, v[vc].sWb) * 4294967296.0f);
        p.dWdY            = (int64_t)(GRAD_Y(v[va].sWb, v[vb].sWb, v[vc].sWb) * 4294967296.0f);
        p.tmu[0].startW = p.tmu[1].startW = p.startW;
        p.tmu[0].dWdX   = p.tmu[1].dWdX   = p.dWdX;
        p.tmu[0].dWdY   = p.tmu[1].dWdY   = p.dWdY;
    }

    /* TMU0 W */
    if (s->sSetupMode & SETUPMODE_W0) {
        p.tmu[0].startW = (int64_t)(v[va].sW0 * 4294967296.0f);
        p.tmu[0].dWdX   = (int64_t)(GRAD_X(v[va].sW0, v[vb].sW0, v[vc].sW0) * 4294967296.0f);
        p.tmu[0].dWdY   = (int64_t)(GRAD_Y(v[va].sW0, v[vb].sW0, v[vc].sW0) * 4294967296.0f);
        p.tmu[1].startW = p.tmu[0].startW;
        p.tmu[1].dWdX   = p.tmu[0].dWdX;
        p.tmu[1].dWdY   = p.tmu[0].dWdY;
    }

    /* TMU0 S/T */
    if (s->sSetupMode & SETUPMODE_S0_T0) {
        p.tmu[0].startS = (int64_t)(v[va].sS0 * 4294967296.0f);
        p.tmu[0].dSdX   = (int64_t)(GRAD_X(v[va].sS0, v[vb].sS0, v[vc].sS0) * 4294967296.0f);
        p.tmu[0].dSdY   = (int64_t)(GRAD_Y(v[va].sS0, v[vb].sS0, v[vc].sS0) * 4294967296.0f);
        p.tmu[0].startT = (int64_t)(v[va].sT0 * 4294967296.0f);
        p.tmu[0].dTdX   = (int64_t)(GRAD_X(v[va].sT0, v[vb].sT0, v[vc].sT0) * 4294967296.0f);
        p.tmu[0].dTdY   = (int64_t)(GRAD_Y(v[va].sT0, v[vb].sT0, v[vc].sT0) * 4294967296.0f);
        /* Mirror to TMU1 (overridden below if SETUPMODE_W1/S1_T1 set) */
        p.tmu[1].startS = p.tmu[0].startS; p.tmu[1].dSdX = p.tmu[0].dSdX;
        p.tmu[1].dSdY   = p.tmu[0].dSdY;
        p.tmu[1].startT = p.tmu[0].startT; p.tmu[1].dTdX = p.tmu[0].dTdX;
        p.tmu[1].dTdY   = p.tmu[0].dTdY;
    }

    /* TMU1 W */
    if (s->sSetupMode & SETUPMODE_W1) {
        p.tmu[1].startW = (int64_t)(v[va].sW1 * 4294967296.0f);
        p.tmu[1].dWdX   = (int64_t)(GRAD_X(v[va].sW1, v[vb].sW1, v[vc].sW1) * 4294967296.0f);
        p.tmu[1].dWdY   = (int64_t)(GRAD_Y(v[va].sW1, v[vb].sW1, v[vc].sW1) * 4294967296.0f);
    }

    /* TMU1 S/T */
    if (s->sSetupMode & SETUPMODE_S1_T1) {
        p.tmu[1].startS = (int64_t)(v[va].sS1 * 4294967296.0f);
        p.tmu[1].dSdX   = (int64_t)(GRAD_X(v[va].sS1, v[vb].sS1, v[vc].sS1) * 4294967296.0f);
        p.tmu[1].dSdY   = (int64_t)(GRAD_Y(v[va].sS1, v[vb].sS1, v[vc].sS1) * 4294967296.0f);
        p.tmu[1].startT = (int64_t)(v[va].sT1 * 4294967296.0f);
        p.tmu[1].dTdX   = (int64_t)(GRAD_X(v[va].sT1, v[vb].sT1, v[vc].sT1) * 4294967296.0f);
        p.tmu[1].dTdY   = (int64_t)(GRAD_Y(v[va].sT1, v[vb].sT1, v[vc].sT1) * 4294967296.0f);
    }

#undef GRAD_X
#undef GRAD_Y

    p.sign = (area < 0.0f) ? 1 : 0;

    voodoo3_queue_triangle(s, &p);
}
