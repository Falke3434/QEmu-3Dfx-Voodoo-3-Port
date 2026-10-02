# Voodoo3 for QEMU (ported from 86Box)

3Dfx Voodoo3 / Banshee PCI device for QEMU, based on the 86Box implementation.
Main targets: AmigaOS 4.1 and MorphOS on Pegasos2, AmigaOne and Sam460ex
(qemu-system-ppc). x86 guests are supported with the legacy VGA option.

## Status

| Guest | 2D | 3D | Notes |
|---|---|---|---|
| AmigaOS 4.1 (Pegasos2, AmigaOne, Sam460) | ✅ 8/16/32 bit | ✅ Warp3D, MiniGL, CoW3D (GLQuake textures WIP) | voodoo3diag: all tests pass |
| MorphOS 3.19 (Pegasos2) | ✅ 8/16/24/32 bit | not tested | start without ROM |
| Windows XP (x86) | ✅ 3dfx driver, 1024x768x32 | not tested | `legacy-vga=on` |
| Linux / WinPE (x86) | ✅ | – | `legacy-vga=on` |

## Build

```bash
cd /path/to/qemu
python3 /path/to/apply.py
rm -rf build && mkdir build && cd build
../configure --target-list=ppc-softmmu,x86_64-softmmu
make -j$(nproc)
```

If your QEMU tree already contains an older `config VOODOO3` block in
`hw/display/Kconfig`, add the line `select VGA` to it (apply.py only appends
missing blocks) and reconfigure.

## Usage

```bash
# AmigaOS 4.1 FE (Pegasos2 / AmigaOne / Sam460ex)
qemu-system-ppc -M pegasos2 -vga none \
    -device voodoo3,model=3,romfile=roms/V3_3000_PCI_SD_2.15.06.rom [...]

# MorphOS (no ROM)
qemu-system-ppc -M pegasos2 -vga none -device voodoo3,model=3,romfile="" [...]

# x86 (Windows XP, Linux, DOS) with the Voodoo3 video BIOS
qemu-system-x86_64 -M pc -vga none \
    -device voodoo3,model=3,legacy-vga=on,romfile=roms/3k12sd.rom [...]
```

Notes:
- On Sam460ex the firmware uses the onboard SM501; switch the QEMU console to
  the Voodoo3 after boot.
- With `-accel whpx` the guest runs, but every register access of the card is
  a VM exit, so 2D/VGA drawing is slower than with TCG/KVM.

## Properties

| Property | Values | Default | Description |
|---|---|---|---|
| `model` | 0–4 | 3 | 0=Banshee, 1=V3-1000, 2=V3-2000, 3=V3-3000, 4=V3-3500 |
| `render-threads` | 1–4 | 2 | rasterizer threads |
| `bilinear` | on/off | on | bilinear texture filtering |
| `dac-filter` | on/off | off | DAC output filter |
| `agp` | on/off | off | AGP mode (experimental) |
| `lfb-tiling` | on/off | off | decode the tiled LFB aperture |
| `legacy-vga` | on/off | off | QEMU VGA core at the PC addresses (needed for x86 BIOS/DOS/boot) |
| `debug` | on/off | off | diagnostic trace (`v3dbg:` lines) with `-d unimp` |

## Legacy VGA / video BIOS (x86)

`legacy-vga=on` adds QEMU's VGA core: text, planar and 256-colour modes at the
fixed PC addresses (I/O 0x3B0-0x3DF, memory 0xA0000-0xBFFFF). It is shown
while the video processor is off (vidProcCfg bit 0 = 0), i.e. by the BIOS,
DOS, boot loaders and operating systems without a native Voodoo driver; once
a driver enables the desktop the normal Voodoo3 display takes over again.
The property is off by default, so AmigaOS 4.1 and MorphOS setups are not
affected.

## Files

```
hw/display/
  voodoo3.c           device: BARs, ext/VGA registers, CMDFIFO, 2D blitter, threads
  voodoo3_render.c    pixel rasterizer (triangle / half triangle)
  voodoo3_texture.c   texture decode, cache, dirty-page tracking
  voodoo3_display.c   display output, fastfill, swap, cursor, overlay
  voodoo3_setup.c     triangle setup engine (sBeginTriCMD/sDrawTriCMD)
  voodoo3_int.h       internal state
  voodoo3_render.h / voodoo3_texture.h / voodoo3_display.h
include/hw/display/
  voodoo3.h           public header
```

## Changelog

**x86 / Legacy VGA**
- `legacy-vga` property: QEMU VGA core, real Voodoo3 video BIOS under SeaBIOS
- BAR2 I/O with 8/16/32-bit accesses to the ext registers; BAR0 VGA proxy split
- Vertical retrace visible in status bit 6 / 0x3DA
- CMDFIFO ring at SGRAM offset 0 accepted, hole counting as in 86Box
  (Windows XP 3dfx driver runs, no more STOP 0xEA)
- FIFO thread no longer takes the BQL (no starvation while the guest polls)
- Desktop stride recomputed when stride or tiling bit changes
- No console resize fights between VGA core and Voodoo desktop (WHPX crash)
- VGA DAC R,G,B order, pixel mask 0x3C6, 6/8-bit DAC

**Stability**
- Bounds checks in display, rasterizer and blitter
- Status read wakes the CMDFIFO thread; no busy spinning without progress

**2D**
- Big-endian guests: LFB swizzle (miscInit0) and register swizzle (miscInit1)
- Pixel byte order follows the driver (AmigaOS BE, MorphOS RGB16PC, x86 LE)
- 512-entry CLUT, correct DAC dword/byte handling, desktop CLUT select
- Mono expansion, ROP fills and patterns with correct byte order
- 2D registers fully readable; full-screen refresh (no stripes)

**3D**
- 3D register map matches the Banshee spec / 86Box
- CMDFIFO: BUMP and hole modes, waits for data, swizzle-aware
- Fastfill, swap via overlay (leftOverlayBuf), separate desktop/3D stride
- Texture fixes: palette/NCC tables, swizzle, tLOD/textureMode bits, mirror
- Pixel counters fbiPixelsIn/Out

**Core**
- Lock-free render threads, dedicated CMDFIFO thread
- Textures in shared SGRAM, refcounted texture cache
- VMState for current QEMU (incl. VGA state)

## Tools

`voodoo3diag` (AmigaOS 4.1): register dump and functional tests for 2D, 3D,
palette and CMDFIFO; useful to compare QEMU with real hardware.

## License

This project is licensed under the GNU General Public License v2 (GPLv2).

This project contains code derived in part from the 86Box project:
https://github.com/86Box/86Box

All original code remains property of its respective authors.

## Credits

- 86Box Contributors (original Voodoo3 implementation)
- QEMU Project
