voodoo3diag 13.2 -- 3Dfx Voodoo3 / Banshee diagnostics for AmigaOS 4.1

  voodoo3diag [LOG=file] [CSV] [WRITE] [FUNC] [FUNC3D] [FIFO] [ALL]

Default: read-only (modules 1-10). Log goes to RAM:voodoo3diag.log and is
flushed after every line; ">> MODULE n" marks the start of each module,
so after a crash the log shows where it stopped.

  WRITE   harmless register round-trips (2D colorFore, CLUT entry 511,
          VGA DAC entry 255 with R,G,B order, pixel mask) -- restored
  FUNC    2D engine tests in a VRAM bitmap allocated by the program:
          rectangle fill (+ byte order of engine output as seen by the CPU),
          copy forward, overlapping copy -x/-y, ROP 0x66, mono expansion,
          colour pattern registers (aliases at 0x44/0x48, 0x100 must not
          touch other registers), opaque mono-pattern fill ROP 0xF0
  FUNC3D  3D fastfill and a flat triangle into the same bitmap, fbi
          counters and nopCMD bit 0 (16-bit screen only; overwrites 3D
          state -- do not run while a Warp3D/MiniGL program is active)
  FIFO    CMDFIFO type-2 packet test (skipped if the driver uses CMDFIFO0)
  ALL     everything
  CSV     extra "CSV;module;test;status;value" lines for diffing runs on
          real hardware against QEMU

Ctrl-C stops between modules. Every engine wait has a timeout.

  qtest_blitter.py  2D engine regression test without a guest: starts
          qemu-system-ppc -M pegasos2 in qtest mode and checks fills,
          copies, host blits, pattern registers and byte reads.
          python3 tools/qtest_blitter.py build/qemu-system-ppc pc-bios
