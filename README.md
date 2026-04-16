# QEMU Voodoo 3 ported from 86Box

## Anwenden

```bash
cd /path/to/qemu
python3 /path/to/apply.py
rm -rf build && mkdir build && cd build
../configure --target-list=ppc-softmmu,x86_64-softmmu
make -j$(nproc)
```

## Verwendung

Das Gerät ist wie jedes andere QEMU PCI-Device per `-device` nutzbar:

```bash
# x86 — Standard Voodoo 3 3000
qemu-system-x86_64 -device voodoo3,model=3 [...]

# PPC — AmigaOS 4.1 FE auf Pegasos2 oder AmigaOne
qemu-system-ppc -M pegasos2 -vga none \
    -device voodoo3,model=3 [...]

# Banshee
qemu-system-x86_64 -device voodoo3,model=0 [...]
```

## Properties

| Property | Werte | Default | Beschreibung |
|---|---|---|---|
| `model` | 0–4 | 3 | 0=Banshee, 1=V3-1000, 2=V3-2000, 3=V3-3000, 4=V3-3500 |
| `render-threads` | 1–4 | 2 | Rasterizer-Threads |
| `bilinear` | on/off | on | Bilinear-Filter |
| `agp` | on/off | off | AGP-Modus (experimentell) |

## Was eingebunden wird

Nur 3 Schritte — kein Machine-Patching, keine .mak-Einträge:

1. `hw/display/voodoo3*.c` + `*.h` → in `hw/display/` kopieren
2. `hw/display/Kconfig` → einen `config VOODOO3`-Block oben einfügen
3. `hw/display/meson.build` → einen `system_ss.add()`-Block oben einfügen

## Dateien

```
hw/display/
  voodoo3.c           Hauptgerät: BAR-Decode, FIFO, 2D-Blitter, Threads
  voodoo3_render.c    Pixel-Rasterizer (voodoo_triangle + half_triangle)
  voodoo3_texture.c   Textur-Decode, 16 Formate, Cache
  voodoo3_display.c   FastFill, SwapBuffer, NCC, Dither, Dirty-Line
  voodoo3_setup.c     Triangle-Setup (sBeginTriCMD/sDrawTriCMD)
  voodoo3_int.h       Interne Struct-Definitionen (shared)
  voodoo3_render.h    Interface
  voodoo3_texture.h   Interface + Typen
  voodoo3_display.h   Interface
include/hw/display/
  voodoo3.h           Öffentlicher Header + voodoo3_create() Helper
```

## Was portiert ist (aus 86Box)

- Vollständige SST-1 3D-Register (Integer + Float, beide Triangle-Pfade)
- Alle Clip-, Fog-, Alpha-, Stipple-, NCC-Register
- Pixel-Rasterizer: Depth, Alpha-Test, Alpha-Blend, Fog, Colour-Combine
- Perspektivkorrektes Texturing + Bilinear-Filter, Dual-TMU
- 16 Texturformate dekodiert (inkl. NCC YIQ-Formate)
- Banshee 2D: RectFill, ScreenToScreen
- FastFill, SwapBuffer mit Vblank-Intervall
- Dirty-Line Display-Ausgabe (nur veränderte Scanlines)
- 4×4 Bayer-Dither-Tabellen (Laufzeit-generiert)
- Big-Endian Byte-Swap für PPC-Gäste (AmigaOne, Pegasos2)
