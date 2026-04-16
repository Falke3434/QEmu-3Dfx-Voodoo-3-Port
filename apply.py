#!/usr/bin/env python3
"""
apply.py — integrate the Voodoo 3 PCI device into a QEMU source tree.

Usage:
    1. Extract this archive somewhere, e.g. C:/qemu-voodoo3-final\
    2. cd into your QEMU repository root
    3. Run: python3 C:/qemu-voodoo3-final\apply.py

   Or on Windows with MSYS2:
    cd C:/msys64/home/falke/qemu
    python3 /c/qemu-voodoo3-final/apply.py

What this does:
  1. Copies hw/display/voodoo3*.c and *.h into hw/display/
  2. Copies include/hw/display/voodoo3.h into include/hw/display/
  3. Prepends one block to hw/display/Kconfig
  4. Prepends one block to hw/display/meson.build

That is ALL. No machine patches. No .mak changes.
The device is then available as -device voodoo3 on any PCI-capable guest.
"""

import os, sys, shutil

ROOT = os.getcwd()
HERE = os.path.dirname(os.path.abspath(__file__))
PATCH_ROOT = HERE   # files are in subdirs next to apply.py

def cp(src_rel, dst_rel):
    src = os.path.join(PATCH_ROOT, src_rel)
    dst = os.path.join(ROOT, dst_rel)
    if not os.path.exists(src):
        print(f"  MISSING source: {src_rel}")
        return
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copy2(src, dst)
    print(f"  COPY  {dst_rel}")

def prepend(fragment_rel, target_rel, marker):
    """Append fragment to target if marker not already present."""
    frag = open(os.path.join(PATCH_ROOT, fragment_rel)).read()
    target = os.path.join(ROOT, target_rel)
    if not os.path.exists(target):
        print(f"  SKIP  {target_rel} (not found)")
        return
    existing = open(target).read()
    if marker in existing:
        print(f"  SKIP  {target_rel} (already patched)")
        return
    # Append to end - safer than prepend, cannot break existing Kconfig structure
    with open(target, 'a') as f:
        f.write("\n" + frag)
    print(f"  PATCH {target_rel}")

print("=== Copying device files ===")
for f in ["voodoo3.c", "voodoo3_render.c", "voodoo3_texture.c",
          "voodoo3_display.c", "voodoo3_setup.c",
          "voodoo3_render.h", "voodoo3_texture.h",
          "voodoo3_display.h", "voodoo3_int.h"]:
    cp(f"hw/display/{f}", f"hw/display/{f}")

cp("include/hw/display/voodoo3.h", "include/hw/display/voodoo3.h")

print("\n=== Patching build system ===")
prepend("hw/display/Kconfig.fragment",    "hw/display/Kconfig",    "VOODOO3")
prepend("hw/display/meson.build.fragment","hw/display/meson.build","voodoo3.c")

print("""
=== Done ===

Rebuild QEMU (delete build dir first after Kconfig changes):

  rm -rf build && mkdir build && cd build
  ../configure --target-list=ppc-softmmu,x86_64-softmmu
  make -j$(nproc)

Usage examples:

  # x86 PC with Voodoo 3 3000
  qemu-system-x86_64 -device voodoo3,model=3 [...]

  # PPC AmigaOne / Pegasos2 with AmigaOS 4.1 FE
  qemu-system-ppc -M pegasos2 -vga none \\
      -device voodoo3,model=3,big-endian-framebuffer=on [...]

  # Banshee variant
  qemu-system-x86_64 -device voodoo3,model=0 [...]

Properties:
  model=0..4        Banshee(0), V3-1000(1), V3-2000(2), V3-3000(3), V3-3500(4)
  big-endian-framebuffer=on   Required for PPC guests (AmigaOne, Pegasos2)
  render-threads=N  Number of render threads (default: 2)
  bilinear=on/off   Bilinear texture filtering (default: on)
""")
