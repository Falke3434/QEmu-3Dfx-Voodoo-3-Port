/*
 * voodoo3diag v13.2 -- 3Dfx Voodoo3 / Banshee diagnostics for AmigaOS 4.1
 *
 * Register layout and bit fields follow the Voodoo3 register specification
 * ("Avenger", rev. 0.97) and 86Box.
 *
 * v13.2:
 *  - dramInit0/1 decoded per spec (bit 27 of dramInit0 = SGRAM type
 *    16 Mbit, bit 26 = two chip sets; dramInit1 bit 27 = mctl_pagebreak,
 *    not a "memory initialised" flag)
 *  - vidProcCfg: tile space, overlay format, filter mode, chroma key
 *  - VGA module reads CRTC 0x1A/0x1B and compares the CRTC display end
 *    with vidScreenSize (height off by one?)
 *  - FUNC: colour pattern registers (pattern0Alias/pattern1Alias must
 *    alias colorPattern[0]/[1] at 0x100/0x104, writing 0x100/0x104 must
 *    not touch dstBaseAddr/dstFormat), and an opaque mono-pattern fill
 *    with ROP 0xF0 (two colours expected)
 *  - FUNC3D: nopCMD with bit 0 clear must keep the fbi counters, with
 *    bit 0 set must clear them (spec 8.17)
 *  - 2D commands 9..15 are never issued: 9..12 are undefined, 13..15 write
 *    the SGRAM mode/mask/colour registers (spec 7.x, command[3:0])
 *
 * v13.0 (rewrite of v12.4):
 *
 *  - Register map corrected against the Banshee/Voodoo3 spec and 86Box
 *    (ext/init registers, CMD/AGP block at BAR0+0x80000, 2D block at
 *    BAR0+0x100000, 3D block at BAR0+0x200000, TMU chip-select, 3D LFB at
 *    BAR0+0x1000000, VGA ports at BAR2+0xB0..0xDF).
 *  - Crash-safe logging: every line goes to the console AND to a log file
 *    (default RAM:voodoo3diag.log) and is flushed immediately.  A
 *    ">> MODULE n" marker is written before each module starts, so after a
 *    crash the log shows exactly where it happened.
 *  - Read-only by default.  Nothing that the running Picasso96 driver
 *    depends on is ever written (miscInit0/1, vidProcCfg, desktop registers,
 *    PLLs, CMDFIFO of an active driver).
 *  - Optional tests (switches):
 *      WRITE   harmless register round-trips (2D colour regs, CLUT entry 511,
 *              VGA DAC entry 255) -- always restored
 *      FUNC    2D engine functional tests inside a VRAM bitmap that this
 *              program allocates (fill, copy fwd/rev, ROP, mono expansion,
 *              byte-order calibration)
 *      FUNC3D  3D fastfill into the same bitmap (overwrites 3D state; do not
 *              run while a Warp3D/MiniGL program is active)
 *      FIFO    CMDFIFO test (only if CMDFIFO0 is currently disabled)
 *      ALL     all of the above
 *  - Every busy-wait has a timeout; on timeout the functional tests stop.
 *  - While the VRAM bitmap is locked nothing is printed (console output
 *    needs the graphics board and would deadlock); results are buffered and
 *    printed after the lock is released.
 *  - CSV switch: additional "CSV;module;test;status;value" lines for diffing
 *    runs on real hardware against QEMU.
 *
 * Usage:   voodoo3diag [LOG=file] [CSV] [WRITE] [FUNC] [FUNC3D] [FIFO] [ALL]
 *
 * Build (cross):  ppc-amigaos-gcc -O2 -Wall -o voodoo3diag voodoo3diag.c -lauto
 */

#include <exec/types.h>
#include <exec/memory.h>
#include <dos/dos.h>
#include <graphics/gfx.h>
#include <intuition/screens.h>
#include <expansion/pci.h>
#include <proto/exec.h>
#include <proto/dos.h>
#include <proto/expansion.h>
#include <proto/graphics.h>
#include <proto/intuition.h>

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define VERSION_STR "13.2"

struct PCIIFace *IPCI = NULL;

/* =========================================================================
 * PCI identity
 * ========================================================================= */
#define V3_VENDOR_ID          0x121A
#define V3_DEVICE_VOODOO3     0x0005
#define V3_DEVICE_BANSHEE     0x0003

/* PCI config offsets (avoid depending on SDK names) */
#define CFG_VENDOR            0x00
#define CFG_DEVICE            0x02
#define CFG_COMMAND           0x04
#define CFG_STATUS            0x06
#define CFG_REVISION          0x08
#define CFG_BAR0              0x10
#define CFG_BAR1              0x14
#define CFG_BAR2              0x18
#define CFG_SUBVENDOR         0x2C
#define CFG_SUBSYSTEM         0x2E
#define CFG_ROM               0x30
#define CFG_IRQ_LINE          0x3C
#define CFG_IRQ_PIN           0x3D

/* =========================================================================
 * BAR0 layout (Banshee/Voodoo3)
 * ========================================================================= */
#define BAR0_EXT              0x0000000UL   /* init/video ext regs, 0x00-0xFF */
#define BAR0_CMD              0x0080000UL   /* CMDFIFO / AGP registers        */
#define BAR0_2D               0x0100000UL   /* 2D engine                      */
#define BAR0_3D               0x0200000UL   /* 3D engine (FBI + TMU)          */
#define BAR0_TEX0             0x0600000UL   /* texture download TMU0          */
#define BAR0_3DLFB            0x1000000UL   /* 3D LFB aperture                */

#define CHIP_TMU0             0x0800UL      /* chip-select TMU0 in 3D space   */
#define CHIP_TMU1             0x1000UL      /* chip-select TMU1 in 3D space   */

/* ---- Ext / init registers (BAR0 + 0x00..0xFF, also BAR2 I/O) ----------- */
#define EXT_status            0x00
#define EXT_pciInit0          0x04
#define EXT_sipMonitor        0x08
#define EXT_lfbMemoryConfig   0x0C
#define EXT_miscInit0         0x10
#define EXT_miscInit1         0x14
#define EXT_dramInit0         0x18
#define EXT_dramInit1         0x1C
#define EXT_agpInit0          0x20
#define EXT_tmuGbeInit        0x24
#define EXT_vgaInit0          0x28
#define EXT_vgaInit1          0x2C
#define EXT_2dCommand         0x30
#define EXT_2dSrcBaseAddr     0x34
#define EXT_strapInfo         0x38
#define EXT_pllCtrl0          0x40
#define EXT_pllCtrl1          0x44
#define EXT_pllCtrl2          0x48
#define EXT_dacMode           0x4C
#define EXT_dacAddr           0x50
#define EXT_dacData           0x54
#define EXT_rgbMaxDelta       0x58
#define EXT_vidProcCfg        0x5C
#define EXT_hwCurPatAddr      0x60
#define EXT_hwCurLoc          0x64
#define EXT_hwCurC0           0x68
#define EXT_hwCurC1           0x6C
#define EXT_vidInFormat       0x70
#define EXT_vidInStatus       0x74
#define EXT_vidSerialParallel 0x78
#define EXT_vidChromaKeyMin   0x8C
#define EXT_vidChromaKeyMax   0x90
#define EXT_vidScreenSize     0x98
#define EXT_vidOvlStartCoords 0x9C
#define EXT_vidOvlEndCoords   0xA0
#define EXT_vidOvlDudx        0xA4
#define EXT_vidOvlDudxOffs    0xA8
#define EXT_vidOvlDvdy        0xAC
#define EXT_vidOvlDvdyOffs    0xE0
#define EXT_vidDesktopStart   0xE4
#define EXT_vidDesktopStride  0xE8

/* ---- CMD/AGP block (BAR0 + 0x80000) ------------------------------------- */
#define CMD_agpReqSize        0x00
#define CMD_agpHostAddrLo     0x04
#define CMD_agpHostAddrHi     0x08
#define CMD_agpGraphicsAddr   0x0C
#define CMD_agpGraphicsStride 0x10
#define CMD_agpMoveCMD        0x14
#define CMD_cmdBaseAddr0      0x20
#define CMD_cmdBaseSize0      0x24
#define CMD_cmdBump0          0x28
#define CMD_cmdRdPtrL0        0x2C
#define CMD_cmdRdPtrH0        0x30
#define CMD_cmdAMin0          0x34
#define CMD_cmdAMax0          0x3C
#define CMD_cmdFifoDepth0     0x44
#define CMD_cmdHoleCnt0       0x48
#define CMD_cmdBaseAddr1      0x50
#define CMD_cmdBaseSize1      0x54
#define CMD_cmdBump1          0x58
#define CMD_cmdRdPtrL1        0x5C
#define CMD_cmdFifoDepth1     0x74
#define CMD_cmdHoleCnt1       0x78
#define CMD_cmdFifoThresh     0x80

/* ---- 2D registers (BAR0 + 0x100000) ------------------------------------- */
#define R2D_status            0x00
#define R2D_intCtrl           0x04
#define R2D_clip0Min          0x08
#define R2D_clip0Max          0x0C
#define R2D_dstBaseAddr       0x10
#define R2D_dstFormat         0x14
#define R2D_srcColorkeyMin    0x18
#define R2D_srcColorkeyMax    0x1C
#define R2D_dstColorkeyMin    0x20
#define R2D_dstColorkeyMax    0x24
#define R2D_bresError0        0x28
#define R2D_bresError1        0x2C
#define R2D_rop               0x30
#define R2D_srcBaseAddr       0x34
#define R2D_commandExtra      0x38
#define R2D_lineStipple       0x3C
#define R2D_lineStyle         0x40
#define R2D_pattern0Alias     0x44
#define R2D_pattern1Alias     0x48
#define R2D_clip1Min          0x4C
#define R2D_clip1Max          0x50
#define R2D_srcFormat         0x54
#define R2D_srcSize           0x58
#define R2D_srcXY             0x5C
#define R2D_colorBack         0x60
#define R2D_colorFore         0x64
#define R2D_dstSize           0x68
#define R2D_dstXY             0x6C
#define R2D_command           0x70
#define R2D_launch            0x80   /* 0x80..0xFC launch area / host data */
#define R2D_pattern           0x100  /* 0x100..0x1FC colour pattern        */

/* 2D command word */
#define CMD2D_NOP             0x0
#define CMD2D_S2S             0x1
#define CMD2D_H2S             0x3
#define CMD2D_RECTFILL        0x5
#define CMD2D_INITIATE        (1UL << 8)
#define CMD2D_DX              (1UL << 14)  /* right-to-left */
#define CMD2D_DY              (1UL << 15)  /* bottom-to-top */
#define CMD2D_PATTERN_MONO    (1UL << 13)
#define CMD2D_TRANS_MONO      (1UL << 16)

/* surface formats (bits 19:16 of src/dstFormat) */
#define FMT_8BPP              1
#define FMT_16BPP             3
#define FMT_24BPP             4
#define FMT_32BPP             5
#define SRCFMT_1BPP           0

/* ---- 3D registers (BAR0 + 0x200000, chip bits 13:10) -------------------- */
#define R3D_status            0x000
#define R3D_intrCtrl          0x004
#define R3D_fbzColorPath      0x104
#define R3D_fogMode           0x108
#define R3D_alphaMode         0x10C
#define R3D_fbzMode           0x110
#define R3D_lfbMode           0x114
#define R3D_clipLeftRight     0x118
#define R3D_clipLowYHighY     0x11C
#define R3D_nopCMD            0x120
#define R3D_fastfillCMD       0x124
#define R3D_swapbufferCMD     0x128
#define R3D_fogColor          0x12C
#define R3D_zaColor           0x130
#define R3D_chromaKey         0x134
#define R3D_chromaRange       0x138
#define R3D_color0            0x144
#define R3D_color1            0x148
#define R3D_fbiPixelsIn       0x14C
#define R3D_fbiChromaFail     0x150
#define R3D_fbiZFuncFail      0x154
#define R3D_fbiAFuncFail      0x158
#define R3D_fbiPixelsOut      0x15C
#define R3D_colBufferAddr     0x1EC
#define R3D_colBufferStride   0x1F0
#define R3D_auxBufferAddr     0x1F4
#define R3D_auxBufferStride   0x1F8
#define R3D_leftOverlayBuf    0x250
#define R3D_textureMode       0x300
#define R3D_tLOD              0x304
#define R3D_texBaseAddr       0x30C

#define R3D_vertexAx          0x008
#define R3D_vertexAy          0x00C
#define R3D_vertexBx          0x010
#define R3D_vertexBy          0x014
#define R3D_vertexCx          0x018
#define R3D_vertexCy          0x01C
#define R3D_startR            0x020
#define R3D_startG            0x024
#define R3D_startB            0x028
#define R3D_startZ            0x02C
#define R3D_startA            0x030
#define R3D_dRdX              0x040
#define R3D_dGdX              0x044
#define R3D_dBdX              0x048
#define R3D_dZdX              0x04C
#define R3D_dAdX              0x050
#define R3D_dRdY              0x060
#define R3D_dGdY              0x064
#define R3D_dBdY              0x068
#define R3D_dZdY              0x06C
#define R3D_dAdY              0x070
#define R3D_triangleCMD       0x080

#define FBZ_CLIP_ENABLE       (1UL << 0)
#define FBZ_RGB_WMASK         (1UL << 9)
#define FBZ_DEPTH_WMASK       (1UL << 10)

/* ---- VGA ports through BAR2: port 0x3xx -> io + (0x3xx - 0x300) --------- */
#define VGA_IO(port)          ((ULONG)(port) - 0x300UL)

/* =========================================================================
 * Logging (console + file, flushed per line; buffered while locked)
 * ========================================================================= */
static FILE *g_log      = NULL;
static BOOL  g_csv      = FALSE;
static BOOL  g_defer    = FALSE;     /* TRUE while VRAM bitmap is locked */
static int   g_module   = 0;
static int   g_ok = 0, g_warn = 0, g_fail = 0, g_skip = 0;

#define DEFER_MAX   96
#define LINE_MAX_   200
static char  g_defer_buf[DEFER_MAX][LINE_MAX_];
static int   g_defer_n = 0;

static void out_line(const char *line)
{
    if (g_defer) {
        if (g_defer_n < DEFER_MAX) {
            strncpy(g_defer_buf[g_defer_n], line, LINE_MAX_ - 1);
            g_defer_buf[g_defer_n][LINE_MAX_ - 1] = 0;
            g_defer_n++;
        }
        return;
    }
    printf("%s\n", line);
    fflush(stdout);
    if (g_log) {
        fprintf(g_log, "%s\n", line);
        fflush(g_log);
    }
}

static void flush_deferred(void)
{
    int i;
    for (i = 0; i < g_defer_n; i++) {
        printf("%s\n", g_defer_buf[i]);
        if (g_log) {
            fprintf(g_log, "%s\n", g_defer_buf[i]);
        }
    }
    fflush(stdout);
    if (g_log) {
        fflush(g_log);
    }
    g_defer_n = 0;
}

static void outf(const char *fmt, ...)
{
    char buf[LINE_MAX_];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    out_line(buf);
}

enum { R_OK, R_WARN, R_FAIL, R_INFO, R_SKIP };

static void res(int r, const char *name, ULONG value, const char *fmt, ...)
{
    static const char *tag[] = { "[ OK ]", "[WARN]", "[FAIL]", "[INFO]", "[SKIP]" };
    static const char *csvtag[] = { "OK", "WARN", "FAIL", "INFO", "SKIP" };
    char detail[LINE_MAX_];
    char line[LINE_MAX_ * 2];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(detail, sizeof(detail), fmt, ap);
    va_end(ap);

    switch (r) {
    case R_OK:   g_ok++;   break;
    case R_WARN: g_warn++; break;
    case R_FAIL: g_fail++; break;
    case R_SKIP: g_skip++; break;
    default: break;
    }
    snprintf(line, sizeof(line), "  %s  %-30s %s", tag[r], name, detail);
    out_line(line);
    if (g_csv) {
        snprintf(line, sizeof(line), "CSV;%d;%s;%s;0x%08lX",
                 g_module, name, csvtag[r], (unsigned long)value);
        out_line(line);
    }
}

static void module_start(int n, const char *title)
{
    g_module = n;
    outf("");
    outf(">> MODULE %d -- %s", n, title);       /* crash marker */
    outf("------------------------------------------------------------");
}

static BOOL user_break(void)
{
    if (IExec->SetSignal(0L, SIGBREAKF_CTRL_C) & SIGBREAKF_CTRL_C) {
        outf("*** Break (Ctrl-C) ***");
        return TRUE;
    }
    return FALSE;
}

/* =========================================================================
 * MMIO helpers (PPC, little-endian device registers)
 * ========================================================================= */
static volatile UBYTE *g_mmio = NULL;   /* BAR0 */
static volatile UBYTE *g_lfb  = NULL;   /* BAR1 */
static ULONG           g_lfb_size = 0;
static struct PCIDevice *g_dev = NULL;
static ULONG           g_io   = 0;      /* BAR2 I/O base */
static BOOL            g_swz_wr = FALSE, g_swz_rd = FALSE; /* miscInit1 24/25 */

static inline ULONG ld_le32(volatile void *p)
{
    ULONG v;
    __asm__ volatile ("lwbrx %0,0,%1\n\teieio" : "=r"(v) : "r"(p) : "memory");
    return v;
}

static inline void st_le32(volatile void *p, ULONG v)
{
    __asm__ volatile ("stwbrx %0,0,%1\n\teieio" : : "r"(v), "r"(p) : "memory");
}

static inline ULONG ld_cpu32(volatile void *p)
{
    ULONG v = *(volatile ULONG *)p;
    __asm__ volatile ("eieio" : : : "memory");
    return v;
}

static inline void st_cpu32(volatile void *p, ULONG v)
{
    *(volatile ULONG *)p = v;
    __asm__ volatile ("eieio" : : : "memory");
}

static inline void mmio_sync(void)
{
    __asm__ volatile ("sync" : : : "memory");
}

/* ext registers are never register-swizzled */
static ULONG ext_rd(ULONG off)
{
    return g_mmio ? ld_le32(g_mmio + BAR0_EXT + off) : 0xDEADBEEFUL;
}

static ULONG cmd_rd(ULONG off)
{
    return g_mmio ? ld_le32(g_mmio + BAR0_CMD + off) : 0xDEADBEEFUL;
}

static void cmd_wr(ULONG off, ULONG v)
{
    if (g_mmio) st_le32(g_mmio + BAR0_CMD + off, v);
}

/* 2D/3D registers honour the register swizzle of miscInit1 (MorphOS mode) */
static ULONG eng_rd(ULONG base, ULONG off)
{
    if (!g_mmio) return 0xDEADBEEFUL;
    return g_swz_rd ? ld_cpu32(g_mmio + base + off) : ld_le32(g_mmio + base + off);
}

static void eng_wr(ULONG base, ULONG off, ULONG v)
{
    if (!g_mmio) return;
    if (g_swz_wr) st_cpu32(g_mmio + base + off, v);
    else          st_le32(g_mmio + base + off, v);
}

#define r2d_rd(o)     eng_rd(BAR0_2D, (o))
#define r2d_wr(o, v)  eng_wr(BAR0_2D, (o), (v))
#define r3d_rd(o)     eng_rd(BAR0_3D, (o))
#define r3d_wr(o, v)  eng_wr(BAR0_3D, (o), (v))

static void refresh_swizzle(void)
{
    ULONG m1 = ext_rd(EXT_miscInit1);
    g_swz_wr = (m1 & (1UL << 24)) != 0;
    g_swz_rd = (m1 & (1UL << 25)) != 0;
}

/* busy bits of the status register; returns FALSE on timeout */
static BOOL wait_idle(ULONG max_loops)
{
    ULONG i;
    if (!g_mmio) return FALSE;
    for (i = 0; i < max_loops; i++) {
        ULONG st = ext_rd(EXT_status);
        if (!(st & 0x780UL) && !(st & (3UL << 11))) {
            return TRUE;
        }
    }
    return FALSE;
}

/* =========================================================================
 * Decoders
 * ========================================================================= */
static double pll_mhz(ULONG pll)
{
    ULONG k = pll & 3, m = (pll >> 2) & 0x3f, n = (pll >> 8) & 0xff;
    return 14.31818 * (double)(n + 2) / (double)(m + 2) / (double)(1UL << k);
}

static const char *pixfmt_name(ULONG f)
{
    switch (f) {
    case 0: return "8bpp CLUT";
    case 1: return "16bpp RGB565";
    case 2: return "24bpp packed";
    case 3: return "32bpp";
    default: return "reserved";
    }
}

static const char *surf_fmt_name(ULONG f)
{
    switch (f) {
    case 0: return "1bpp mono";
    case 1: return "8bpp";
    case 3: return "16bpp";
    case 4: return "24bpp";
    case 5: return "32bpp";
    case 8: return "YUYV";
    case 9: return "UYVY";
    default: return "?";
    }
}

/* =========================================================================
 * MODULE 1 -- PCI configuration space
 * ========================================================================= */
static ULONG g_bar0 = 0, g_bar1 = 0, g_bar2 = 0;

static void mod_pci(struct PCIDevice *dev)
{
    UWORD ven, devid, cmd, sts, sv, sd;
    UBYTE rev, irq, pin;
    ULONG b0, b1, b2, rom;

    module_start(1, "PCI configuration space");
    ven   = dev->ReadConfigWord(CFG_VENDOR);
    devid = dev->ReadConfigWord(CFG_DEVICE);
    cmd   = dev->ReadConfigWord(CFG_COMMAND);
    sts   = dev->ReadConfigWord(CFG_STATUS);
    rev   = dev->ReadConfigByte(CFG_REVISION);
    sv    = dev->ReadConfigWord(CFG_SUBVENDOR);
    sd    = dev->ReadConfigWord(CFG_SUBSYSTEM);
    irq   = dev->ReadConfigByte(CFG_IRQ_LINE);
    pin   = dev->ReadConfigByte(CFG_IRQ_PIN);
    b0    = dev->ReadConfigLong(CFG_BAR0);
    b1    = dev->ReadConfigLong(CFG_BAR1);
    b2    = dev->ReadConfigLong(CFG_BAR2);
    rom   = dev->ReadConfigLong(CFG_ROM);

    g_bar0 = b0 & 0xFFFFFFF0UL;
    g_bar1 = b1 & 0xFFFFFFF0UL;
    g_bar2 = b2 & 0xFFFFFFFCUL;

    res(ven == V3_VENDOR_ID ? R_OK : R_FAIL, "VendorID", ven, "0x%04X (3Dfx = 0x121A)", ven);
    res((devid == V3_DEVICE_VOODOO3 || devid == V3_DEVICE_BANSHEE) ? R_OK : R_FAIL,
        "DeviceID", devid, "0x%04X (%s)", devid,
        devid == V3_DEVICE_VOODOO3 ? "Voodoo3" : devid == V3_DEVICE_BANSHEE ? "Banshee" : "?");
    res(R_INFO, "Revision", rev, "0x%02X", rev);
    res(R_INFO, "Subsystem", ((ULONG)sv << 16) | sd, "vendor 0x%04X id 0x%04X (%s)", sv, sd,
        sd == 0x0036 ? "V3 2000 PCI" : sd == 0x0038 ? "V3 2000 AGP" :
        sd == 0x003A ? "V3 3000 PCI" : sd == 0x003C ? "V3 3000 AGP" :
        sd == 0x0030 ? "V3 1000" : sd == 0x0034 ? "Banshee AGP" :
        sd == 0x0035 ? "Banshee PCI" : "unknown");
    res((cmd & 0x3) == 0x3 ? R_OK : R_WARN, "Command register", cmd,
        "0x%04X  io=%d mem=%d master=%d", cmd, cmd & 1, (cmd >> 1) & 1, (cmd >> 2) & 1);
    res(R_INFO, "Status register", sts, "0x%04X", sts);
    res((b0 & 1) == 0 && g_bar0 ? R_OK : R_FAIL, "BAR0 (MMIO 32MB)", b0, "0x%08lX", (unsigned long)b0);
    res((b1 & 1) == 0 && g_bar1 ? R_OK : R_FAIL, "BAR1 (LFB 32MB)", b1, "0x%08lX  prefetch=%d",
        (unsigned long)b1, (int)((b1 >> 3) & 1));
    res((b2 & 1) == 1 ? R_OK : R_FAIL, "BAR2 (I/O 256B)", b2, "0x%08lX", (unsigned long)b2);
    res(R_INFO, "Expansion ROM BAR", rom, "0x%08lX enabled=%d", (unsigned long)rom, (int)(rom & 1));
    res(R_INFO, "IRQ", irq, "line %d pin %d", irq, pin);
}

/* =========================================================================
 * MODULE 2 -- init / ext registers (read-only)
 * ========================================================================= */
static void mod_ext(void)
{
    static const struct { ULONG off; const char *name; } regs[] = {
        { EXT_status, "status" },          { EXT_pciInit0, "pciInit0" },
        { EXT_sipMonitor, "sipMonitor" },  { EXT_lfbMemoryConfig, "lfbMemoryConfig" },
        { EXT_miscInit0, "miscInit0" },    { EXT_miscInit1, "miscInit1" },
        { EXT_dramInit0, "dramInit0" },    { EXT_dramInit1, "dramInit1" },
        { EXT_agpInit0, "agpInit0" },      { EXT_tmuGbeInit, "tmuGbeInit" },
        { EXT_vgaInit0, "vgaInit0" },      { EXT_vgaInit1, "vgaInit1" },
        { EXT_2dCommand, "2dCommand" },    { EXT_2dSrcBaseAddr, "2dSrcBaseAddr" },
        { EXT_strapInfo, "strapInfo" },    { EXT_pllCtrl0, "pllCtrl0" },
        { EXT_pllCtrl1, "pllCtrl1" },      { EXT_pllCtrl2, "pllCtrl2" },
        { EXT_dacMode, "dacMode" },        { EXT_rgbMaxDelta, "rgbMaxDelta" },
        { EXT_vidProcCfg, "vidProcCfg" },  { EXT_hwCurPatAddr, "hwCurPatAddr" },
        { EXT_hwCurLoc, "hwCurLoc" },      { EXT_hwCurC0, "hwCurC0" },
        { EXT_hwCurC1, "hwCurC1" },        { EXT_vidSerialParallel, "vidSerialParallelPort" },
        { EXT_vidScreenSize, "vidScreenSize" },
        { EXT_vidDesktopStart, "vidDesktopStartAddr" },
        { EXT_vidDesktopStride, "vidDesktopOverlayStride" },
    };
    unsigned i;

    module_start(2, "Init / video registers (read-only)");
    for (i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
        ULONG v = ext_rd(regs[i].off);
        res(v == 0xFFFFFFFFUL ? R_WARN : R_INFO, regs[i].name, v, "off 0x%02lX = 0x%08lX",
            (unsigned long)regs[i].off, (unsigned long)v);
    }
}

/* =========================================================================
 * MODULE 3 -- status / clocks / memory / swizzle decode
 * ========================================================================= */
static void mod_decode(void)
{
    ULONG st, p0, p1, strap, d0, d1, m0, m1, lfbcfg, vga0;

    module_start(3, "Status, clocks, memory and byte-order decode");
    st     = ext_rd(EXT_status);
    p0     = ext_rd(EXT_pllCtrl0);
    p1     = ext_rd(EXT_pllCtrl1);
    strap  = ext_rd(EXT_strapInfo);
    d0     = ext_rd(EXT_dramInit0);
    d1     = ext_rd(EXT_dramInit1);
    m0     = ext_rd(EXT_miscInit0);
    m1     = ext_rd(EXT_miscInit1);
    lfbcfg = ext_rd(EXT_lfbMemoryConfig);
    vga0   = ext_rd(EXT_vgaInit0);

    res(R_INFO, "status", st, "fifo free=%lu vblank=%d busy=%d cmdfifo0=%d cmdfifo1=%d",
        (unsigned long)(st & 0x1f), !((st >> 6) & 1), (st & 0x780) ? 1 : 0,
        (int)((st >> 11) & 1), (int)((st >> 12) & 1));
    res((st & 0x780) ? R_WARN : R_OK, "engine idle", st, (st & 0x780) ? "busy at test time" : "idle");
    res(R_INFO, "pixel clock (pllCtrl0)", p0, "%.3f MHz", pll_mhz(p0));
    res(R_INFO, "memory clock (pllCtrl1)", p1, "%.3f MHz", pll_mhz(p1));
    res(R_INFO, "strapInfo", strap, "0x%08lX", (unsigned long)strap);
    res(R_INFO, "dramInit0", d0,
        "0x%08lX  sgram-16Mbit(27)=%d two-chipsets(26)=%d tCAS=%lu",
        (unsigned long)d0, (int)((d0 >> 27) & 1), (int)((d0 >> 26) & 1),
        (unsigned long)(((d0 >> 14) & 3) + 1));
    res(R_INFO, "dramInit1", d1,
        "0x%08lX  refresh=%d sdram(30)=%d pagebreak(27)=%d",
        (unsigned long)d1, (int)(d1 & 1), (int)((d1 >> 30) & 1), (int)((d1 >> 27) & 1));
    res(R_INFO, "miscInit0 LFB swizzle", m0,
        "0x%08lX  byte-swizzle(30)=%d word-swizzle(31)=%d  y-origin=%lu",
        (unsigned long)m0, (int)((m0 >> 30) & 1), (int)((m0 >> 31) & 1),
        (unsigned long)((m0 >> 18) & 0xfff));
    res(R_INFO, "miscInit1 reg swizzle", m1, "0x%08lX  wr-swizzle(24)=%d rd-swizzle(25)=%d",
        (unsigned long)m1, (int)((m1 >> 24) & 1), (int)((m1 >> 25) & 1));
    res(R_INFO, "lfbMemoryConfig", lfbcfg, "tile base 0x%06lX  stride %lu  tile_x %lu",
        (unsigned long)((lfbcfg & 0x1fff) << 12), (unsigned long)(1024UL << ((lfbcfg >> 13) & 7)),
        (unsigned long)(((lfbcfg >> 16) & 0x7f) * 128));
    res(R_INFO, "vgaInit0", vga0, "0x%08lX  ext-shift-out=%d", (unsigned long)vga0,
        (int)((vga0 >> 12) & 1));
}

/* =========================================================================
 * MODULE 4 -- video processor / desktop / cursor decode
 * ========================================================================= */
static ULONG g_scr_h = 0;   /* vidScreenSize height, for module 8 */

static void mod_video(void)
{
    ULONG vpc, ss, ds, dst, cpat, cloc, dm;
    ULONG w, h, fmt, stride, bpp;

    module_start(4, "Video processor, desktop and cursor");
    vpc  = ext_rd(EXT_vidProcCfg);
    ss   = ext_rd(EXT_vidScreenSize);
    ds   = ext_rd(EXT_vidDesktopStart);
    dst  = ext_rd(EXT_vidDesktopStride);
    cpat = ext_rd(EXT_hwCurPatAddr);
    cloc = ext_rd(EXT_hwCurLoc);
    dm   = ext_rd(EXT_dacMode);

    w = ss & 0xfff;
    h = (ss >> 12) & 0xfff;
    fmt = (vpc >> 18) & 7;
    stride = dst & 0x7fff;
    bpp = fmt == 0 ? 1 : fmt == 1 ? 2 : fmt == 2 ? 3 : 4;

    res(R_INFO, "vidProcCfg", vpc,
        "0x%08lX  enable=%d desktop=%s overlay=%d cursor=%d(%s) clut-bypass=%d clut-sel=%d 2x=%d",
        (unsigned long)vpc, (int)(vpc & 1), pixfmt_name(fmt), (int)((vpc >> 8) & 1),
        (int)((vpc >> 27) & 1), (vpc & 2) ? "X11" : "Windows", (int)((vpc >> 10) & 1),
        (int)((vpc >> 12) & 1), (int)((vpc >> 26) & 1));
    res(R_INFO, "vidProcCfg (2)", vpc,
        "desktop-tile=%d overlay-tile=%d overlay-fmt=%lu filter=%lu chroma=%d inv=%d h/v-scale=%d/%d",
        (int)((vpc >> 24) & 1), (int)((vpc >> 25) & 1), (unsigned long)((vpc >> 21) & 7),
        (unsigned long)((vpc >> 16) & 3), (int)((vpc >> 5) & 1), (int)((vpc >> 6) & 1),
        (int)((vpc >> 14) & 1), (int)((vpc >> 15) & 1));
    res(R_INFO, "screen size", ss, "%lux%lu", (unsigned long)w, (unsigned long)h);
    g_scr_h = h;
    res(R_INFO, "desktop start", ds, "0x%06lX", (unsigned long)(ds & 0xffffff));
    res(R_INFO, "desktop/overlay stride", dst, "desktop %lu bytes  overlay %lu bytes",
        (unsigned long)stride, (unsigned long)((dst >> 16) & 0x7fff));
    if (w && stride) {
        res(stride >= w * bpp ? R_OK : R_WARN, "stride >= width*bpp", stride,
            "%lu >= %lu", (unsigned long)stride, (unsigned long)(w * bpp));
    }
    res(R_INFO, "hw cursor", cpat, "pattern 0x%06lX  pos %lu,%lu",
        (unsigned long)(cpat & 0xffffff), (unsigned long)(cloc & 0x7ff),
        (unsigned long)((cloc >> 16) & 0x7ff));
    res(R_INFO, "dacMode", dm, "0x%08lX", (unsigned long)dm);
}

/* =========================================================================
 * MODULE 5 -- 2D registers (read-only decode)
 * ========================================================================= */
static void mod_2d_regs(void)
{
    ULONG v;
    module_start(5, "2D engine registers (read-only)");
    refresh_swizzle();
    v = r2d_rd(R2D_status);
    res(R_INFO, "2D status", v, "0x%08lX", (unsigned long)v);
    v = r2d_rd(R2D_dstBaseAddr);
    res(R_INFO, "dstBaseAddr", v, "0x%06lX", (unsigned long)(v & 0xffffff));
    v = r2d_rd(R2D_dstFormat);
    res(R_INFO, "dstFormat", v, "%s stride %lu", surf_fmt_name((v >> 16) & 0xf),
        (unsigned long)(v & 0x3fff));
    v = r2d_rd(R2D_srcBaseAddr);
    res(R_INFO, "srcBaseAddr", v, "0x%06lX", (unsigned long)(v & 0xffffff));
    v = r2d_rd(R2D_srcFormat);
    res(R_INFO, "srcFormat", v, "%s stride %lu byteswz=%d wordswz=%d packing=%lu",
        surf_fmt_name((v >> 16) & 0xf), (unsigned long)(v & 0x3fff), (int)((v >> 20) & 1),
        (int)((v >> 21) & 1), (unsigned long)((v >> 22) & 3));
    v = r2d_rd(R2D_clip0Min); res(R_INFO, "clip0Min", v, "0x%08lX", (unsigned long)v);
    v = r2d_rd(R2D_clip0Max); res(R_INFO, "clip0Max", v, "0x%08lX", (unsigned long)v);
    v = r2d_rd(R2D_colorBack); res(R_INFO, "colorBack", v, "0x%08lX", (unsigned long)v);
    v = r2d_rd(R2D_colorFore); res(R_INFO, "colorFore", v, "0x%08lX", (unsigned long)v);
    v = r2d_rd(R2D_dstSize);   res(R_INFO, "dstSize", v, "%lux%lu",
                                   (unsigned long)(v & 0x1fff), (unsigned long)((v >> 16) & 0x1fff));
    v = r2d_rd(R2D_command);   res(R_INFO, "command", v, "op=%lu rop=0x%02lX",
                                   (unsigned long)(v & 0xf), (unsigned long)(v >> 24));
}

/* =========================================================================
 * MODULE 6 -- CMDFIFO registers (read-only)
 * ========================================================================= */
static BOOL g_cmdfifo0_enabled = FALSE;

static void mod_cmdfifo_regs(void)
{
    ULONG base0, size0, rd0, depth0, hole0, base1, size1;

    module_start(6, "CMDFIFO registers (read-only)");
    base0  = cmd_rd(CMD_cmdBaseAddr0);
    size0  = cmd_rd(CMD_cmdBaseSize0);
    rd0    = cmd_rd(CMD_cmdRdPtrL0);
    depth0 = cmd_rd(CMD_cmdFifoDepth0);
    hole0  = cmd_rd(CMD_cmdHoleCnt0);
    base1  = cmd_rd(CMD_cmdBaseAddr1);
    size1  = cmd_rd(CMD_cmdBaseSize1);
    g_cmdfifo0_enabled = (size0 & 0x100) != 0;

    res(R_INFO, "CMDFIFO0 base/size", size0,
        "base 0x%06lX  size %lu KB  enable=%d agp=%d holes-off=%d",
        (unsigned long)((base0 & 0xfff) << 12), (unsigned long)(((size0 & 0xff) + 1) * 4),
        (int)((size0 >> 8) & 1), (int)((size0 >> 9) & 1), (int)((size0 >> 10) & 1));
    res(R_INFO, "CMDFIFO0 state", depth0, "rdptr 0x%06lX depth %lu holes %lu",
        (unsigned long)(rd0 & 0xffffff), (unsigned long)depth0, (unsigned long)hole0);
    res(R_INFO, "CMDFIFO1 base/size", size1, "base 0x%06lX  enable=%d",
        (unsigned long)((base1 & 0xfff) << 12), (int)((size1 >> 8) & 1));
}

/* =========================================================================
 * MODULE 7 -- 3D readable registers (status, counters, TMU)
 * ========================================================================= */
static void mod_3d_regs(void)
{
    ULONG v;
    module_start(7, "3D registers (readable subset)");
    refresh_swizzle();
    v = r3d_rd(R3D_status);
    res(R_INFO, "3D status", v, "0x%08lX (mirror of init status)", (unsigned long)v);
    v = r3d_rd(R3D_fbiPixelsIn);  res(R_INFO, "fbiPixelsIn", v, "%lu", (unsigned long)(v & 0xffffff));
    v = r3d_rd(R3D_fbiChromaFail);res(R_INFO, "fbiChromaFail", v, "%lu", (unsigned long)(v & 0xffffff));
    v = r3d_rd(R3D_fbiZFuncFail); res(R_INFO, "fbiZFuncFail", v, "%lu", (unsigned long)(v & 0xffffff));
    v = r3d_rd(R3D_fbiAFuncFail); res(R_INFO, "fbiAFuncFail", v, "%lu", (unsigned long)(v & 0xffffff));
    v = r3d_rd(R3D_fbiPixelsOut); res(R_INFO, "fbiPixelsOut", v, "%lu", (unsigned long)(v & 0xffffff));
    res(R_INFO, "note", 0, "most 3D registers are write-only; reads may return status");
}

/* =========================================================================
 * MODULE 8 -- VGA registers through BAR2 (read-only, index restored)
 * ========================================================================= */
static void mod_vga(void)
{
    UBYTE misc, seqidx, crtidx, seq[5], crt[0x1c];
    ULONG vde;
    int i;
    char buf[100];

    module_start(8, "VGA registers via BAR2 I/O (read-only)");
    if (!g_dev || !g_io) {
        res(R_SKIP, "VGA", 0, "no I/O BAR");
        return;
    }
    misc   = g_dev->InByte(g_io + VGA_IO(0x3CC));
    seqidx = g_dev->InByte(g_io + VGA_IO(0x3C4));
    crtidx = g_dev->InByte(g_io + VGA_IO(0x3D4));
    for (i = 0; i < 5; i++) {
        g_dev->OutByte(g_io + VGA_IO(0x3C4), (UBYTE)i);
        seq[i] = g_dev->InByte(g_io + VGA_IO(0x3C5));
    }
    for (i = 0; i < 0x1c; i++) {
        g_dev->OutByte(g_io + VGA_IO(0x3D4), (UBYTE)i);
        crt[i] = g_dev->InByte(g_io + VGA_IO(0x3D5));
    }
    g_dev->OutByte(g_io + VGA_IO(0x3C4), seqidx);
    g_dev->OutByte(g_io + VGA_IO(0x3D4), crtidx);

    res(R_INFO, "misc output (0x3CC)", misc, "0x%02X", misc);
    snprintf(buf, sizeof(buf), "%02X %02X %02X %02X %02X", seq[0], seq[1], seq[2], seq[3], seq[4]);
    res(R_INFO, "SEQ[0..4]", seq[0], "%s", buf);
    snprintf(buf, sizeof(buf), "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
             crt[0], crt[1], crt[2], crt[3], crt[4], crt[5], crt[6], crt[7], crt[8], crt[9],
             crt[0x12], crt[0x13]);
    res(R_INFO, "CRTC 0-9,12h,13h", crt[0], "%s", buf);
    res(R_INFO, "CRTC h-total/v-disp", crt[0], "htotal=%u vdisp-end(low)=%u ext 1Ah=%02X 1Bh=%02X",
        (unsigned)crt[0], (unsigned)crt[0x12], crt[0x1a], crt[0x1b]);
    /* display end: CR12 + CR07 bit1 (bit 8) + bit6 (bit 9) + CR1B bit2 (bit 10) */
    vde = crt[0x12] | ((ULONG)(crt[7] & 0x02) << 7) | ((ULONG)(crt[7] & 0x40) << 3) |
          ((ULONG)(crt[0x1b] & 0x04) << 8);
    if (g_scr_h) {
        res(vde + 1 == g_scr_h ? R_OK : R_INFO, "CRTC vs vidScreenSize", vde,
            "CRTC display lines %lu, vidScreenSize height %lu%s", (unsigned long)(vde + 1),
            (unsigned long)g_scr_h,
            vde + 1 == g_scr_h ? "" : "  (differ: driver programs CR12 = height, not height-1)");
    }
}

/* =========================================================================
 * MODULE 9 -- palette read (non-destructive)
 * ========================================================================= */
static void mod_palette_read(void)
{
    static const ULONG idx[] = { 0, 1, 15, 255, 256, 511 };
    ULONG save_addr;
    unsigned i;

    module_start(9, "Palette (CLUT) read via dacAddr/dacData");
    save_addr = ext_rd(EXT_dacAddr);
    for (i = 0; i < sizeof(idx) / sizeof(idx[0]); i++) {
        ULONG v;
        if (!g_mmio) break;
        st_le32(g_mmio + EXT_dacAddr, idx[i]);
        v = ext_rd(EXT_dacData);
        res(v == 0xFFFFFFFFUL ? R_WARN : R_INFO, "CLUT entry", v, "[%3lu] = 0x%06lX",
            (unsigned long)idx[i], (unsigned long)(v & 0xffffff));
    }
    if (g_mmio) st_le32(g_mmio + EXT_dacAddr, save_addr);
}

/* =========================================================================
 * MODULE 10 -- DDC (read-only)
 * ========================================================================= */
static void mod_ddc(void)
{
    ULONG v;
    module_start(10, "DDC / I2C port (read-only)");
    v = ext_rd(EXT_vidSerialParallel);
    res(R_INFO, "vidSerialParallelPort", v, "0x%08lX  DDC enable=%d SCL in=%d SDA in=%d",
        (unsigned long)v, (int)((v >> 18) & 1), (int)((v >> 21) & 1), (int)((v >> 22) & 1));
}

/* =========================================================================
 * MODULE 11 (WRITE) -- harmless register round-trips, always restored
 * ========================================================================= */
static void mod_write(void)
{
    ULONG save, v, save_addr;
    static const ULONG pat[] = { 0x00000000UL, 0x000000FFUL, 0x00FF0000UL, 0x00123456UL };
    unsigned i;
    UBYTE r, g, b, wa;

    module_start(11, "Register round-trips (WRITE, restored)");
    if (!g_mmio) { res(R_SKIP, "WRITE", 0, "no MMIO"); return; }
    refresh_swizzle();

    /* 2D colorFore: the driver writes it before every operation */
    if (!wait_idle(2000000)) { res(R_FAIL, "wait idle", 0, "timeout -- skipped"); return; }
    IExec->Forbid();
    save = r2d_rd(R2D_colorFore);
    r2d_wr(R2D_colorFore, 0x5A3C96E1UL);
    mmio_sync();
    v = r2d_rd(R2D_colorFore);
    r2d_wr(R2D_colorFore, save);
    IExec->Permit();
    res(v == 0x5A3C96E1UL ? R_OK : R_FAIL, "2D colorFore r/w", v, "wrote 0x5A3C96E1 read 0x%08lX",
        (unsigned long)v);

    /* CLUT entry 511 (upper half, not used by the desktop) via dacAddr/dacData */
    save_addr = ext_rd(EXT_dacAddr);
    st_le32(g_mmio + EXT_dacAddr, 511);
    save = ext_rd(EXT_dacData);
    for (i = 0; i < sizeof(pat) / sizeof(pat[0]); i++) {
        st_le32(g_mmio + EXT_dacAddr, 511);
        st_le32(g_mmio + EXT_dacData, pat[i]);
        st_le32(g_mmio + EXT_dacAddr, 511);
        v = ext_rd(EXT_dacData) & 0xffffff;
        res(v == pat[i] ? R_OK : R_FAIL, "CLUT[511] dword", v, "wrote 0x%06lX read 0x%06lX",
            (unsigned long)pat[i], (unsigned long)v);
    }
    st_le32(g_mmio + EXT_dacAddr, 511);
    st_le32(g_mmio + EXT_dacData, save);
    st_le32(g_mmio + EXT_dacAddr, save_addr);

    /* VGA DAC through BAR2: entry 255, order R,G,B (VGA standard) */
    if (g_dev && g_io) {
        UBYTE sr, sg, sb;
        wa = g_dev->InByte(g_io + VGA_IO(0x3C8));
        g_dev->OutByte(g_io + VGA_IO(0x3C7), 255);
        sr = g_dev->InByte(g_io + VGA_IO(0x3C9));
        sg = g_dev->InByte(g_io + VGA_IO(0x3C9));
        sb = g_dev->InByte(g_io + VGA_IO(0x3C9));
        g_dev->OutByte(g_io + VGA_IO(0x3C8), 255);
        g_dev->OutByte(g_io + VGA_IO(0x3C9), 0x12);
        g_dev->OutByte(g_io + VGA_IO(0x3C9), 0x34);
        g_dev->OutByte(g_io + VGA_IO(0x3C9), 0x56);
        g_dev->OutByte(g_io + VGA_IO(0x3C7), 255);
        r = g_dev->InByte(g_io + VGA_IO(0x3C9));
        g = g_dev->InByte(g_io + VGA_IO(0x3C9));
        b = g_dev->InByte(g_io + VGA_IO(0x3C9));
        g_dev->OutByte(g_io + VGA_IO(0x3C8), 255);
        g_dev->OutByte(g_io + VGA_IO(0x3C9), sr);
        g_dev->OutByte(g_io + VGA_IO(0x3C9), sg);
        g_dev->OutByte(g_io + VGA_IO(0x3C9), sb);
        g_dev->OutByte(g_io + VGA_IO(0x3C8), wa);
        res((r == 0x12 && g == 0x34 && b == 0x56) ? R_OK : R_FAIL, "VGA DAC R,G,B order",
            ((ULONG)r << 16) | ((ULONG)g << 8) | b,
            "wrote 12 34 56 read %02X %02X %02X", r, g, b);
        v = g_dev->InByte(g_io + VGA_IO(0x3C6));
        res(v == 0xFF ? R_OK : R_WARN, "VGA pixel mask (0x3C6)", v, "0x%02lX (expected 0xFF)",
            (unsigned long)v);
    }
}

/* =========================================================================
 * Functional tests (FUNC / FUNC3D / FIFO) in a VRAM bitmap
 * ========================================================================= */
#define TBM_W   64
#define TBM_H   32
#define TBM_EXTRA_ROWS 80     /* room for a 4K-aligned CMDFIFO ring */

static struct BitMap *g_tbm      = NULL;
static APTR           g_tlock    = NULL;
static UBYTE         *g_tbase    = NULL;  /* CPU address */
static ULONG          g_tbpr     = 0;
static ULONG          g_toff     = 0;     /* VRAM offset */
static ULONG          g_tbpp     = 0;
static BOOL           g_func_abort = FALSE;
static int            g_view_le = -1;    /* engine output as seen by CPU: 0=BE 1=LE */

static BOOL alloc_test_bitmap(void)
{
    struct Screen *scr;
    ULONG depth;

    scr = IIntuition->LockPubScreen(NULL);
    if (!scr) {
        res(R_SKIP, "test bitmap", 0, "no public screen");
        return FALSE;
    }
    depth = IGraphics->GetBitMapAttr(scr->RastPort.BitMap, BMA_DEPTH);
    g_tbm = IGraphics->AllocBitMap(TBM_W, TBM_H * 3 + TBM_EXTRA_ROWS, depth,
                                   BMF_CLEAR | BMF_DISPLAYABLE,
                                   scr->RastPort.BitMap);
    IIntuition->UnlockPubScreen(NULL, scr);
    if (!g_tbm) {
        res(R_SKIP, "test bitmap", 0, "AllocBitMap failed");
        return FALSE;
    }
    g_tbpp = depth <= 8 ? 1 : depth <= 16 ? 2 : 4;
    res(R_INFO, "test bitmap", depth, "%dx%d depth %lu allocated", TBM_W, TBM_H * 3 + TBM_EXTRA_ROWS,
        (unsigned long)depth);
    return TRUE;
}

static void free_test_bitmap(void)
{
    if (g_tbm) {
        IGraphics->WaitBlit();
        IGraphics->FreeBitMap(g_tbm);
        g_tbm = NULL;
    }
}

/* Lock: from here on nothing may be printed directly (deferred). */
static BOOL lock_test_bitmap(void)
{
    APTR addr = NULL;
    ULONG bpr = 0;

    g_tlock = IGraphics->LockBitMapTags(g_tbm, LBM_BaseAddress, &addr,
                                        LBM_BytesPerRow, &bpr, TAG_END);
    if (!g_tlock) {
        return FALSE;
    }
    g_defer = TRUE;
    g_tbase = (UBYTE *)addr;
    g_tbpr  = bpr;
    if (g_lfb && (ULONG)addr >= (ULONG)g_lfb && (ULONG)addr < (ULONG)g_lfb + g_lfb_size) {
        g_toff = ((ULONG)addr - (ULONG)g_lfb) & 0xFFFFFFUL;
        return TRUE;
    }
    return FALSE;     /* not in VRAM (or address not comparable) */
}

static void unlock_test_bitmap(void)
{
    if (g_tlock) {
        IGraphics->UnlockBitMap(g_tlock);
        g_tlock = NULL;
    }
    g_defer = FALSE;
    flush_deferred();
}

/* save / restore the 2D registers we touch */
static const ULONG g_2d_saved_regs[] = {
    R2D_clip0Min, R2D_clip0Max, R2D_dstBaseAddr, R2D_dstFormat, R2D_rop,
    R2D_srcBaseAddr, R2D_commandExtra, R2D_srcFormat, R2D_srcSize, R2D_srcXY,
    R2D_colorBack, R2D_colorFore, R2D_dstSize, R2D_dstXY
};
#define N2DSAVE (sizeof(g_2d_saved_regs) / sizeof(g_2d_saved_regs[0]))
static ULONG g_2d_save[N2DSAVE];

static void save_2d(void)
{
    unsigned i;
    for (i = 0; i < N2DSAVE; i++) g_2d_save[i] = r2d_rd(g_2d_saved_regs[i]);
}

static void restore_2d(void)
{
    unsigned i;
    for (i = 0; i < N2DSAVE; i++) r2d_wr(g_2d_saved_regs[i], g_2d_save[i]);
}

static ULONG fmt_code(void)
{
    return g_tbpp == 1 ? FMT_8BPP : g_tbpp == 2 ? FMT_16BPP : FMT_32BPP;
}

static ULONG px_read(int x, int y)    /* pixel as the CPU sees it */
{
    UBYTE *p = g_tbase + (ULONG)y * g_tbpr + (ULONG)x * g_tbpp;
    if (g_tbpp == 1) return *(volatile UBYTE *)p;
    if (g_tbpp == 2) return *(volatile UWORD *)p;
    return *(volatile ULONG *)p;
}

static void px_write(int x, int y, ULONG v)
{
    UBYTE *p = g_tbase + (ULONG)y * g_tbpr + (ULONG)x * g_tbpp;
    if (g_tbpp == 1) *(volatile UBYTE *)p = (UBYTE)v;
    else if (g_tbpp == 2) *(volatile UWORD *)p = (UWORD)v;
    else *(volatile ULONG *)p = v;
}

static void blt_setup_dst(void)
{
    r2d_wr(R2D_clip0Min, 0);
    r2d_wr(R2D_clip0Max, (0xfffUL << 16) | 0xfffUL);
    r2d_wr(R2D_dstBaseAddr, g_toff);
    r2d_wr(R2D_dstFormat, (fmt_code() << 16) | (g_tbpr & 0x3fff));
    r2d_wr(R2D_commandExtra, 0);
}

static BOOL blt_go(ULONG command)
{
    r2d_wr(R2D_command, command | CMD2D_INITIATE);
    mmio_sync();
    if (!wait_idle(2000000)) {
        g_func_abort = TRUE;
        return FALSE;
    }
    return TRUE;
}

static ULONG mask_bpp(ULONG v)
{
    return g_tbpp == 1 ? (v & 0xff) : g_tbpp == 2 ? (v & 0xffff) : v;
}

/* MODULE 12 (FUNC) -- 2D engine functional tests */
static void mod_func_2d(void)
{
    ULONG colour, got, cpu_le, expect;
    int x, y, bad;

    module_start(12, "2D engine functional tests (FUNC)");
    if (!g_tbm && !alloc_test_bitmap()) return;
    if (!wait_idle(2000000)) { res(R_FAIL, "wait idle", 0, "engine busy -- skipped"); return; }

    if (!lock_test_bitmap()) {
        unlock_test_bitmap();
        res(R_SKIP, "test bitmap", (ULONG)g_tbase, "not in VRAM aperture (addr 0x%08lX)",
            (unsigned long)g_tbase);
        return;
    }
    refresh_swizzle();
    IExec->Forbid();
    save_2d();

    res(R_INFO, "bitmap VRAM offset", g_toff, "0x%06lX  bpr %lu  bpp %lu",
        (unsigned long)g_toff, (unsigned long)g_tbpr, (unsigned long)g_tbpp);

    /* --- a) solid rectangle fill, ROP 0xCC ------------------------------ */
    colour = g_tbpp == 1 ? 0x5A : g_tbpp == 2 ? 0xF81F : 0x00123456UL;
    /* sentinels directly below and right of the rectangle */
    px_write(0, TBM_H, mask_bpp(0x3C3C3C3CUL));
    blt_setup_dst();
    r2d_wr(R2D_colorFore, colour);
    r2d_wr(R2D_dstSize, ((ULONG)TBM_H << 16) | TBM_W);
    r2d_wr(R2D_dstXY, 0);
    if (blt_go(0xCC000000UL | CMD2D_RECTFILL)) {
        got = px_read(5, 5);
        /* how the engine-written pixel appears to the CPU: report both */
        cpu_le = g_tbpp == 2 ? (((got & 0xff) << 8) | (got >> 8)) :
                 g_tbpp == 4 ? (((got & 0xff) << 24) | ((got & 0xff00) << 8) |
                                ((got >> 8) & 0xff00) | (got >> 24)) : got;
        if (mask_bpp(got) == mask_bpp(colour)) {
            g_view_le = 0;
            res(R_OK, "fill byte order", got, "CPU reads 0x%08lX = colour (big-endian view)",
                (unsigned long)got);
        } else if (mask_bpp(cpu_le) == mask_bpp(colour)) {
            g_view_le = 1;
            res(R_INFO, "fill byte order", got, "CPU reads 0x%08lX = colour byte-swapped (LE view)",
                (unsigned long)got);
        } else {
            res(R_FAIL, "fill byte order", got, "CPU reads 0x%08lX, colour 0x%08lX",
                (unsigned long)got, (unsigned long)colour);
        }
        bad = 0;
        for (y = 0; y < TBM_H; y++) for (x = 0; x < TBM_W; x++)
            if (px_read(x, y) != got) bad++;
        res(bad ? R_FAIL : R_OK, "fill coverage", bad, "%d of %d pixels differ", bad, TBM_W * TBM_H);
        res(px_read(0, TBM_H) == mask_bpp(0x3C3C3C3CUL) ? R_OK : R_FAIL, "fill bounds",
            px_read(0, TBM_H), "pixel below rectangle %s",
            px_read(0, TBM_H) == mask_bpp(0x3C3C3C3CUL) ? "untouched" : "overwritten");
    } else {
        res(R_FAIL, "rect fill", 0, "engine timeout");
    }

    /* --- b) screen-to-screen copy forward ------------------------------- */
    if (!g_func_abort) {
        for (y = 0; y < TBM_H; y++) for (x = 0; x < TBM_W; x++)
            px_write(x, y, mask_bpp((ULONG)(x * 3 + y * 7) * 0x01010101UL));
        blt_setup_dst();
        r2d_wr(R2D_srcBaseAddr, g_toff);
        r2d_wr(R2D_srcFormat, (fmt_code() << 16) | (g_tbpr & 0x3fff));
        r2d_wr(R2D_srcXY, 0);
        r2d_wr(R2D_dstXY, (ULONG)TBM_H << 16);              /* copy below */
        r2d_wr(R2D_dstSize, ((ULONG)TBM_H << 16) | TBM_W);
        if (blt_go(0xCC000000UL | CMD2D_S2S)) {
            bad = 0;
            for (y = 0; y < TBM_H; y++) for (x = 0; x < TBM_W; x++)
                if (px_read(x, y + TBM_H) != px_read(x, y)) bad++;
            res(bad ? R_FAIL : R_OK, "copy forward", bad, "%d pixels differ", bad);
        } else {
            res(R_FAIL, "copy forward", 0, "engine timeout");
        }
    }

    /* --- c) overlapping copy right-to-left/bottom-to-top (scroll by 3,2) - */
    if (!g_func_abort) {
        ULONG ref[TBM_H][8];
        for (y = 0; y < TBM_H; y++) for (x = 0; x < 8; x++) ref[y][x] = px_read(x, y);
        blt_setup_dst();
        r2d_wr(R2D_srcBaseAddr, g_toff);
        r2d_wr(R2D_srcFormat, (fmt_code() << 16) | (g_tbpr & 0x3fff));
        /* reversed: coordinates are the bottom-right pixel */
        r2d_wr(R2D_srcXY, ((ULONG)(TBM_H - 3) << 16) | (TBM_W - 4));
        r2d_wr(R2D_dstXY, ((ULONG)(TBM_H - 1) << 16) | (TBM_W - 1));
        r2d_wr(R2D_dstSize, ((ULONG)(TBM_H - 2) << 16) | (TBM_W - 3));
        if (blt_go(0xCC000000UL | CMD2D_S2S | CMD2D_DX | CMD2D_DY)) {
            bad = 0;
            for (y = 0; y < TBM_H - 2; y++) for (x = 0; x < 5; x++)
                if (px_read(x + 3, y + 2) != ref[y][x]) bad++;
            res(bad ? R_FAIL : R_OK, "copy overlapped -x-y", bad, "%d pixels differ", bad);
        } else {
            res(R_FAIL, "copy overlapped -x-y", 0, "engine timeout");
        }
    }

    /* --- d) ROP XOR fill (0x66 with colorFore as source) ----------------- */
    if (!g_func_abort) {
        ULONG before = px_read(10, 10);
        ULONG fore = g_tbpp == 1 ? 0xFF : g_tbpp == 2 ? 0xFFFF : 0x00FFFFFFUL;
        ULONG fore_cpu = fore;
        if (g_view_le == 1 && g_tbpp == 4) fore_cpu = 0xFFFFFF00UL;   /* LE view of 0x00FFFFFF */
        blt_setup_dst();
        r2d_wr(R2D_colorFore, fore);
        r2d_wr(R2D_dstSize, (1UL << 16) | 1);
        r2d_wr(R2D_dstXY, (10UL << 16) | 10);
        if (blt_go(0x66000000UL | CMD2D_RECTFILL)) {
            got = px_read(10, 10);
            expect = mask_bpp(before ^ fore_cpu);
            res(mask_bpp(got) == expect ? R_OK : R_FAIL, "ROP 0x66 xor fill", got,
                "before 0x%08lX after 0x%08lX expected 0x%08lX",
                (unsigned long)before, (unsigned long)got, (unsigned long)expect);
        } else {
            res(R_FAIL, "ROP 0x66", 0, "engine timeout");
        }
    }

    /* --- e) host-to-screen mono expansion, 32 pixels x 4 rows ------------ */
    if (!g_func_abort) {
        static const ULONG rows[4] = { 0xF0000000UL, 0x0F000000UL, 0x80000001UL, 0xAAAAAAAAUL };
        ULONG fg = g_tbpp == 1 ? 0x11 : g_tbpp == 2 ? 0x07E0 : 0x0000FF00UL;
        ULONG bg = g_tbpp == 1 ? 0x22 : g_tbpp == 2 ? 0x001F : 0x000000FFUL;
        ULONG fg_seen = 0, bg_seen = 0;
        int r;
        blt_setup_dst();
        r2d_wr(R2D_srcFormat, (SRCFMT_1BPP << 16) | (3UL << 22) | 4); /* dword packing */
        r2d_wr(R2D_srcXY, 0);
        r2d_wr(R2D_colorFore, fg);
        r2d_wr(R2D_colorBack, bg);
        r2d_wr(R2D_dstSize, (4UL << 16) | 32);
        r2d_wr(R2D_dstXY, ((ULONG)(2 * TBM_H) << 16) | 0);
        r2d_wr(R2D_command, 0xCC000000UL | CMD2D_H2S | CMD2D_INITIATE);
        for (r = 0; r < 4; r++) {
            /* host data in bus order: bit 7 of byte 0 = leftmost pixel */
            ULONG d = rows[r];
            ULONG bus = ((d >> 24) & 0xff) | ((d >> 8) & 0xff00) |
                        ((d << 8) & 0xff0000) | (d << 24);
            r2d_wr(R2D_launch, bus);
        }
        mmio_sync();
        if (wait_idle(2000000)) {
            fg_seen = px_read(0, 2 * TBM_H);
            bg_seen = px_read(5, 2 * TBM_H);
            res((fg_seen != bg_seen) ? R_OK : R_FAIL, "mono expansion", fg_seen,
                "row0 x0=0x%08lX (fg) x5=0x%08lX (bg)", (unsigned long)fg_seen,
                (unsigned long)bg_seen);
            res(px_read(31, 2 * TBM_H + 2) == fg_seen ? R_OK : R_FAIL, "mono bit order", 0,
                "row2 x31 %s", px_read(31, 2 * TBM_H + 2) == fg_seen ? "foreground (OK)" : "wrong");
        } else {
            g_func_abort = TRUE;
            res(R_FAIL, "mono expansion", 0, "engine timeout");
        }
    }

    /* --- f) colour pattern registers (spec: 0x44/0x48 alias colorPattern
     *        [0]/[1] at 0x100/0x104; 0x100.. are pattern registers only) - */
    if (!g_func_abort) {
        ULONG p0 = r2d_rd(R2D_pattern), p1 = r2d_rd(R2D_pattern + 4);
        ULONG dba = r2d_rd(R2D_dstBaseAddr), dfm = r2d_rd(R2D_dstFormat);
        ULONG a, b, dba2, dfm2;

        r2d_wr(R2D_pattern0Alias, 0x13579BDFUL);
        a = r2d_rd(R2D_pattern);
        r2d_wr(R2D_pattern + 4, 0x2468ACE0UL);
        b = r2d_rd(R2D_pattern1Alias);
        r2d_wr(R2D_pattern, 0x00123400UL);
        dba2 = r2d_rd(R2D_dstBaseAddr);
        dfm2 = r2d_rd(R2D_dstFormat);
        res(a == 0x13579BDFUL ? R_OK : R_FAIL, "pattern0Alias -> colorPattern[0]", a,
            "0x100 reads 0x%08lX (expected 0x13579BDF)", (unsigned long)a);
        res(b == 0x2468ACE0UL ? R_OK : R_FAIL, "colorPattern[1] -> pattern1Alias", b,
            "0x48 reads 0x%08lX (expected 0x2468ACE0)", (unsigned long)b);
        res(dba2 == dba && dfm2 == dfm ? R_OK : R_FAIL, "0x100 is a pattern register", dba2,
            "dstBaseAddr 0x%06lX -> 0x%06lX, dstFormat 0x%08lX -> 0x%08lX",
            (unsigned long)dba, (unsigned long)dba2, (unsigned long)dfm, (unsigned long)dfm2);
        r2d_wr(R2D_dstBaseAddr, dba);
        r2d_wr(R2D_dstFormat, dfm);
        r2d_wr(R2D_pattern, p0);
        r2d_wr(R2D_pattern + 4, p1);
    }

    /* --- g) opaque mono pattern, ROP 0xF0 (PATCOPY): pattern bit 1 =
     *        colorFore, bit 0 = colorBack; rows of 0xAA alternate ---------- */
    if (!g_func_abort) {
        ULONG p0 = r2d_rd(R2D_pattern0Alias), p1 = r2d_rd(R2D_pattern1Alias);
        ULONG fg = g_tbpp == 1 ? 0x33 : g_tbpp == 2 ? 0xF800 : 0x00FF0000UL;
        ULONG bg = g_tbpp == 1 ? 0x44 : g_tbpp == 2 ? 0x001F : 0x000000FFUL;
        ULONG c0, c1, c2;
        blt_setup_dst();
        r2d_wr(R2D_pattern0Alias, 0xAAAAAAAAUL);
        r2d_wr(R2D_pattern1Alias, 0xAAAAAAAAUL);
        r2d_wr(R2D_colorFore, fg);
        r2d_wr(R2D_colorBack, bg);
        r2d_wr(R2D_dstSize, (1UL << 16) | 8);
        r2d_wr(R2D_dstXY, (ULONG)(2 * TBM_H + 6) << 16);
        if (blt_go(0xF0000000UL | CMD2D_PATTERN_MONO | CMD2D_RECTFILL)) {
            c0 = px_read(0, 2 * TBM_H + 6);
            c1 = px_read(1, 2 * TBM_H + 6);
            c2 = px_read(2, 2 * TBM_H + 6);
            res(c0 != c1 && c0 == c2 ? R_OK : R_FAIL, "mono pattern fill 0xF0", c0,
                "x0=0x%08lX x1=0x%08lX x2=0x%08lX (x0==x2, x0!=x1 expected)",
                (unsigned long)c0, (unsigned long)c1, (unsigned long)c2);
        } else {
            res(R_FAIL, "mono pattern fill 0xF0", 0, "engine timeout");
        }
        r2d_wr(R2D_pattern0Alias, p0);
        r2d_wr(R2D_pattern1Alias, p1);
    }

    restore_2d();
    IExec->Permit();
    unlock_test_bitmap();
    if (g_func_abort) {
        outf("  functional tests aborted after engine timeout");
    }
}

/* MODULE 13 (FUNC3D) -- 3D fastfill into the test bitmap */
static ULONG g_tri_inside = 0, g_tri_outside = 0;
static ULONG g_nop0_out = 0, g_nop1_out = 0;
static int   g_tri_ok = 0;

static void mod_func_3d(void)
{
    ULONG got, pin, pout;
    int bad, x, y;

    module_start(13, "3D fastfill (FUNC3D, overwrites 3D state)");
    if (g_func_abort) { res(R_SKIP, "3D", 0, "previous engine timeout"); return; }
    if (!g_tbm && !alloc_test_bitmap()) return;
    if (g_tbpp != 2) {
        res(R_SKIP, "3D fastfill", g_tbpp, "needs a 16-bit screen (3D renders RGB565 only)");
        return;
    }
    if (!wait_idle(2000000)) { res(R_FAIL, "wait idle", 0, "engine busy -- skipped"); return; }
    if (!lock_test_bitmap()) {
        unlock_test_bitmap();
        res(R_SKIP, "test bitmap", 0, "not in VRAM aperture");
        return;
    }
    refresh_swizzle();
    IExec->Forbid();
    r3d_wr(R3D_nopCMD, 1);                          /* reset pixel counters */
    r3d_wr(R3D_colBufferAddr, g_toff);
    r3d_wr(R3D_colBufferStride, g_tbpr & 0x3fff);
    r3d_wr(R3D_clipLeftRight, (0UL << 16) | TBM_W);
    r3d_wr(R3D_clipLowYHighY, (0UL << 16) | TBM_H);
    r3d_wr(R3D_fbzMode, FBZ_RGB_WMASK | FBZ_CLIP_ENABLE);
    r3d_wr(R3D_color1, 0x00FF0000UL);               /* red -> RGB565 0xF800 */
    r3d_wr(R3D_fastfillCMD, 0);
    mmio_sync();
    if (!wait_idle(2000000)) {
        IExec->Permit();
        unlock_test_bitmap();
        g_func_abort = TRUE;
        res(R_FAIL, "3D fastfill", 0, "engine timeout");
        return;
    }
    got = px_read(3, 3);
    bad = 0;
    for (y = 0; y < TBM_H; y++) for (x = 0; x < TBM_W; x++) if (px_read(x, y) != got) bad++;
    /* flat green triangle A(4,4) B(60,4) C(4,28) over the red fill */
    {
        r3d_wr(R3D_nopCMD, 1);                      /* reset counters */
        r3d_wr(R3D_fbzColorPath, 0);                /* iterated RGB */
        r3d_wr(R3D_vertexAx, 4 << 4);  r3d_wr(R3D_vertexAy, 4 << 4);
        r3d_wr(R3D_vertexBx, 60 << 4); r3d_wr(R3D_vertexBy, 4 << 4);
        r3d_wr(R3D_vertexCx, 4 << 4);  r3d_wr(R3D_vertexCy, 28 << 4);
        r3d_wr(R3D_startR, 0);  r3d_wr(R3D_startG, 255UL << 12);
        r3d_wr(R3D_startB, 0);  r3d_wr(R3D_startA, 255UL << 12);
        r3d_wr(R3D_startZ, 0);
        r3d_wr(R3D_dRdX, 0); r3d_wr(R3D_dGdX, 0); r3d_wr(R3D_dBdX, 0);
        r3d_wr(R3D_dAdX, 0); r3d_wr(R3D_dZdX, 0);
        r3d_wr(R3D_dRdY, 0); r3d_wr(R3D_dGdY, 0); r3d_wr(R3D_dBdY, 0);
        r3d_wr(R3D_dAdY, 0); r3d_wr(R3D_dZdY, 0);
        r3d_wr(R3D_triangleCMD, 0);                 /* area > 0: sign 0 */
        mmio_sync();
        if (wait_idle(2000000)) {
            g_tri_inside  = px_read(8, 8);
            g_tri_outside = px_read(60, 28);
            g_tri_ok = 1;
        } else {
            g_func_abort = TRUE;
        }
    }
    pin  = r3d_rd(R3D_fbiPixelsIn);
    pout = r3d_rd(R3D_fbiPixelsOut);
    /* spec 8.17: nopCMD clears the fbi counters only with bit 0 set */
    r3d_wr(R3D_nopCMD, 0);
    mmio_sync();
    wait_idle(2000000);
    g_nop0_out = r3d_rd(R3D_fbiPixelsOut);
    r3d_wr(R3D_nopCMD, 1);
    mmio_sync();
    wait_idle(2000000);
    g_nop1_out = r3d_rd(R3D_fbiPixelsOut);
    IExec->Permit();
    unlock_test_bitmap();

    res((got == 0xF800 || got == 0x00F8) ? R_OK : R_FAIL, "fastfill colour", got,
        "CPU reads 0x%04lX (0xF800 BE view / 0x00F8 LE view)", (unsigned long)got);
    res(bad ? R_FAIL : R_OK, "fastfill coverage", bad, "%d of %d pixels differ", bad, TBM_W * TBM_H);
    if (g_tri_ok) {
        res((g_tri_inside == 0x07E0 || g_tri_inside == 0xE007) ? R_OK : R_FAIL, "triangle inside",
            g_tri_inside, "pixel (8,8) = 0x%04lX (green 0x07E0)", (unsigned long)g_tri_inside);
        res(g_tri_outside == got ? R_OK : R_FAIL, "triangle outside", g_tri_outside,
            "pixel (60,28) = 0x%04lX (still fill colour)", (unsigned long)g_tri_outside);
    } else {
        res(R_FAIL, "triangle", 0, "engine timeout");
    }
    res((pout & 0xffffff) ? R_OK : R_WARN, "pixel counters", pout,
        "after triangle: in=%lu out=%lu (expected about 700)", (unsigned long)(pin & 0xffffff),
        (unsigned long)(pout & 0xffffff));
    if (pout & 0xffffff) {
        res((g_nop0_out & 0xffffff) == (pout & 0xffffff) ? R_OK : R_FAIL, "nopCMD(0) keeps counters",
            g_nop0_out, "fbiPixelsOut %lu -> %lu", (unsigned long)(pout & 0xffffff),
            (unsigned long)(g_nop0_out & 0xffffff));
    }
    res((g_nop1_out & 0xffffff) == 0 ? R_OK : R_FAIL, "nopCMD(1) clears counters", g_nop1_out,
        "fbiPixelsOut after nopCMD(1) = %lu", (unsigned long)(g_nop1_out & 0xffffff));
}

/* MODULE 14 (FIFO) -- CMDFIFO type-2 packet writing 2D colorBack */
static ULONG g_fifo_ring_off = 0;

static void mod_fifo(void)
{
    ULONG saved_back, got, *ring;
    ULONG save_base, save_size, save_rd;
    int i;

    module_start(14, "CMDFIFO test (FIFO)");
    if (g_func_abort) { res(R_SKIP, "FIFO", 0, "previous engine timeout"); return; }
    if (g_cmdfifo0_enabled || (cmd_rd(CMD_cmdBaseSize0) & 0x100)) {
        res(R_SKIP, "FIFO", 0, "CMDFIFO0 is in use by the driver -- not touched");
        return;
    }
    if (!g_tbm && !alloc_test_bitmap()) return;
    if (!wait_idle(2000000)) { res(R_FAIL, "wait idle", 0, "engine busy -- skipped"); return; }
    if (!lock_test_bitmap()) {
        unlock_test_bitmap();
        res(R_SKIP, "test bitmap", 0, "not in VRAM aperture");
        return;
    }
    {
        /* 4K-aligned ring inside the extra rows of the test bitmap */
        ULONG start = g_toff + (ULONG)(TBM_H * 3) * g_tbpr;
        ULONG ring_off = (start + 0xfffUL) & ~0xfffUL;
        ULONG end = g_toff + (ULONG)(TBM_H * 3 + TBM_EXTRA_ROWS) * g_tbpr;
        if (ring_off + 4096 > end) {
            unlock_test_bitmap();
            res(R_SKIP, "FIFO", g_toff, "no 4K-aligned area in bitmap (bpr %lu)",
                (unsigned long)g_tbpr);
            return;
        }
        g_fifo_ring_off = ring_off;
    }
    refresh_swizzle();
    IExec->Forbid();
    saved_back = r2d_rd(R2D_colorBack);
    save_base  = cmd_rd(CMD_cmdBaseAddr0);
    save_size  = cmd_rd(CMD_cmdBaseSize0);
    save_rd    = cmd_rd(CMD_cmdRdPtrL0);

    /* packet type 2: header bits 2:0 = 2, mask bits 31:3; mask bit 0 is the
     * 2D register at 0x08 (86Box addr = 8), so colorBack (0x60) is bit 22 */
    ring = (ULONG *)(g_tbase + (g_fifo_ring_off - g_toff));
    for (i = 0; i < 16; i++) ring[i] = 0;
    {
        ULONG words[2];
        ULONG m0 = ext_rd(EXT_miscInit0);
        /* the chip fetches ring words little-endian from its own memory;
         * CPU stores are rearranged by the LFB swizzle (byte k -> k ^ x) */
        ULONG x = ((m0 & (1UL << 30)) ? 3UL : 0UL) ^ ((m0 & (1UL << 31)) ? 2UL : 0UL);
        int w, k;
        words[0] = 2UL | (1UL << (22 + 3));
        words[1] = 0x00C0FFEEUL;
        for (w = 0; w < 2; w++) {
            volatile UBYTE *b = (volatile UBYTE *)&ring[w];
            for (k = 0; k < 4; k++) b[k ^ x] = (UBYTE)(words[w] >> (8 * k));
        }
    }
    mmio_sync();
    cmd_wr(CMD_cmdBaseSize0, 0);                    /* disabled while set up */
    cmd_wr(CMD_cmdBaseAddr0, g_fifo_ring_off >> 12);
    cmd_wr(CMD_cmdRdPtrL0, g_fifo_ring_off);
    cmd_wr(CMD_cmdBaseSize0, 0x100UL | (1UL << 10)); /* 4K, enable, holes off */
    cmd_wr(CMD_cmdBump0, 2);
    mmio_sync();
    for (i = 0; i < 200000; i++) {
        if (cmd_rd(CMD_cmdRdPtrL0) == g_fifo_ring_off + 8) break;
    }
    got = r2d_rd(R2D_colorBack);

    /* restore */
    cmd_wr(CMD_cmdBaseSize0, 0);
    cmd_wr(CMD_cmdBaseAddr0, save_base);
    cmd_wr(CMD_cmdRdPtrL0, save_rd);
    cmd_wr(CMD_cmdBaseSize0, save_size);
    r2d_wr(R2D_colorBack, saved_back);
    IExec->Permit();
    unlock_test_bitmap();

    res(i < 200000 ? R_OK : R_FAIL, "CMDFIFO consumed", i, "read pointer %s",
        i < 200000 ? "advanced by 2 words" : "did not advance (timeout)");
    res(got == 0x00C0FFEEUL ? R_OK : R_FAIL, "type-2 packet", got,
        "colorBack = 0x%08lX (expected 0x00C0FFEE; swizzle x per miscInit0 may differ)",
        (unsigned long)got);
}

/* =========================================================================
 * main
 * ========================================================================= */
int main(void)
{
    LONG args[8] = { 0 };
    struct RDArgs *rda;
    const char *logname = "RAM:voodoo3diag.log";
    BOOL do_write, do_func, do_func3d, do_fifo;
    struct PCIDevice *dev = NULL;
    struct PCIResourceRange *rr0 = NULL, *rr1 = NULL, *rr2 = NULL;
    int rc;

    rda = IDOS->ReadArgs("LOG/K,CSV/S,WRITE/S,FUNC/S,FUNC3D/S,FIFO/S,ALL/S", args, NULL);
    if (!rda) {
        printf("Usage: voodoo3diag [LOG=file] [CSV] [WRITE] [FUNC] [FUNC3D] [FIFO] [ALL]\n");
        return 20;
    }
    if (args[0]) logname = (const char *)args[0];
    g_csv     = args[1] != 0;
    do_write  = args[2] != 0 || args[6] != 0;
    do_func   = args[3] != 0 || args[6] != 0;
    do_func3d = args[4] != 0 || args[6] != 0;
    do_fifo   = args[5] != 0 || args[6] != 0;

    g_log = fopen(logname, "w");

    outf("============================================================");
    outf(" voodoo3diag v" VERSION_STR " -- 3Dfx Voodoo3 / Banshee diagnostics");
    outf(" log: %s%s", g_log ? logname : "(could not open)", g_csv ? "  CSV on" : "");
    outf(" tests: read-only%s%s%s%s", do_write ? " +WRITE" : "", do_func ? " +FUNC" : "",
         do_func3d ? " +FUNC3D" : "", do_fifo ? " +FIFO" : "");
    outf("============================================================");

    IPCI = (struct PCIIFace *)IExec->GetInterface((struct Library *)ExpansionBase, "pci", 1, NULL);
    if (!IPCI) {
        outf("[FATAL] PCI interface not available");
        rc = 20;
        goto out;
    }
    dev = IPCI->FindDeviceTags(FDT_VendorID, V3_VENDOR_ID, FDT_DeviceID, V3_DEVICE_VOODOO3, TAG_DONE);
    if (!dev) {
        dev = IPCI->FindDeviceTags(FDT_VendorID, V3_VENDOR_ID, FDT_DeviceID, V3_DEVICE_BANSHEE,
                                   TAG_DONE);
    }
    if (!dev) {
        outf("[FATAL] no Voodoo3/Banshee found");
        rc = 20;
        goto out;
    }
    g_dev = dev;

    mod_pci(dev);

    rr0 = dev->GetResourceRange(0);
    rr1 = dev->GetResourceRange(1);
    rr2 = dev->GetResourceRange(2);
    if (rr0) g_mmio = (volatile UBYTE *)(ULONG)rr0->BaseAddress;
    if (rr1) { g_lfb = (volatile UBYTE *)(ULONG)rr1->BaseAddress; g_lfb_size = rr1->Size; }
    /* InByte/OutByte take PCI I/O port numbers: use the BAR2 value */
    g_io = g_bar2;
    (void)rr2;
    outf("  mapped: BAR0=0x%08lX BAR1=0x%08lX (size 0x%lX) IO=0x%08lX",
         (unsigned long)g_mmio, (unsigned long)g_lfb, (unsigned long)g_lfb_size,
         (unsigned long)g_io);
    if (!g_mmio) {
        outf("[FATAL] BAR0 not mapped -- register tests impossible");
        rc = 20;
        goto out;
    }
    if (g_lfb_size == 0 || g_lfb_size > 0x2000000UL) g_lfb_size = 0x2000000UL;

    refresh_swizzle();
    if (!user_break()) mod_ext();
    if (!user_break()) mod_decode();
    if (!user_break()) mod_video();
    if (!user_break()) mod_2d_regs();
    if (!user_break()) mod_cmdfifo_regs();
    if (!user_break()) mod_3d_regs();
    if (!user_break()) mod_vga();
    if (!user_break()) mod_palette_read();
    if (!user_break()) mod_ddc();
    if (do_write  && !user_break()) mod_write();
    if (do_func   && !user_break()) mod_func_2d();
    if (do_func3d && !user_break()) mod_func_3d();
    if (do_fifo   && !user_break()) mod_fifo();
    free_test_bitmap();

    outf("");
    outf("============================================================");
    outf(" RESULT: %d OK  %d WARN  %d FAIL  %d SKIP", g_ok, g_warn, g_fail, g_skip);
    outf("============================================================");
    rc = g_fail ? 10 : g_warn ? 5 : 0;

out:
    if (rr0) dev->FreeResourceRange(rr0);
    if (rr1) dev->FreeResourceRange(rr1);
    if (rr2) dev->FreeResourceRange(rr2);
    if (dev) IPCI->FreeDevice(dev);
    if (IPCI) IExec->DropInterface((struct Interface *)IPCI);
    if (g_log) fclose(g_log);
    IDOS->FreeArgs(rda);
    return rc;
}
