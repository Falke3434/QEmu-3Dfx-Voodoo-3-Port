#!/usr/bin/env python3
import os
import shutil
import sys

ROOT = os.getcwd()
HERE = os.path.dirname(os.path.abspath(__file__))

FILES = [
    "hw/display/voodoo3.c",
    "hw/display/voodoo3_render.c",
    "hw/display/voodoo3_texture.c",
    "hw/display/voodoo3_display.c",
    "hw/display/voodoo3_setup.c",
    "hw/display/voodoo3_dither_tables.c",
    "hw/display/voodoo3_render.h",
    "hw/display/voodoo3_texture.h",
    "hw/display/voodoo3_display.h",
    "hw/display/voodoo3_int.h",
    "hw/display/voodoo3_gaps.h",
    "hw/display/voodoo3_dither_tables.h",
    "include/hw/display/voodoo3.h",
]

PATCHES = [
    ("hw/display/Kconfig.fragment", "hw/display/Kconfig", "config VOODOO3"),
    ("hw/display/meson.build.fragment", "hw/display/meson.build", "voodoo3.c"),
]


def copy_file(rel):
    src = os.path.join(HERE, rel)
    dst = os.path.join(ROOT, rel)
    if not os.path.exists(src):
        print(f"  MISSING {rel}")
        return False
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copy2(src, dst)
    print(f"  COPY    {rel}")
    return True


def append_once(fragment, target, marker):
    path = os.path.join(ROOT, target)
    if not os.path.exists(path):
        print(f"  SKIP    {target} (not found)")
        return False
    with open(path) as f:
        if marker in f.read():
            print(f"  SKIP    {target} (already patched)")
            return True
    with open(os.path.join(HERE, fragment)) as f:
        text = f.read()
    with open(path, "a") as f:
        f.write("\n" + text)
    print(f"  PATCH   {target}")
    return True


def main():
    if not os.path.isfile(os.path.join(ROOT, "hw/display/meson.build")):
        print("Run apply.py from the root of a QEMU source tree.")
        return 1
    print("=== Copying device files ===")
    ok = all([copy_file(f) for f in FILES])
    print("\n=== Patching build system ===")
    ok = all([append_once(*p) for p in PATCHES]) and ok
    print("""
=== Done ===

Rebuild QEMU (delete the build directory after Kconfig changes):

  rm -rf build && mkdir build && cd build
  ../configure --target-list=ppc-softmmu,x86_64-softmmu
  make -j$(nproc)

Examples:

  # AmigaOS 4.1 (Pegasos2 / AmigaOne / Sam460ex)
  qemu-system-ppc -M pegasos2 -vga none \\
      -device voodoo3,model=3,romfile=roms/V3_3000_PCI_SD_2.15.06.rom [...]

  # MorphOS (no ROM)
  qemu-system-ppc -M pegasos2 -vga none -device voodoo3,model=3,romfile="" [...]

  # x86 with the Voodoo3 video BIOS
  qemu-system-x86_64 -M pc -vga none \\
      -device voodoo3,model=3,legacy-vga=on,romfile=roms/3k12sd.rom [...]

Properties:
  model=0..4          Banshee(0), V3-1000(1), V3-2000(2), V3-3000(3), V3-3500(4)
  render-threads=1..4 rasterizer threads (default 2)
  bilinear=on/off     bilinear texture filtering (default on)
  dac-filter=on/off   DAC output filter (default off)
  agp=on/off          AGP identity (default off)
  lfb-tiling=auto/on/off  decode the tiled LFB aperture (default auto)
  legacy-vga=on/off   VGA core at the PC addresses (default off)
  debug=on/off        v3dbg trace with -d unimp (default off)
""")
    if not ok:
        print("Some steps failed, see above.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
