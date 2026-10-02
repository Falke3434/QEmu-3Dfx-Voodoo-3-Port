voodoo3diag 13.1 -- 3Dfx Voodoo3 / Banshee diagnostics for AmigaOS 4.1

  voodoo3diag [LOG=file] [CSV] [WRITE] [FUNC] [FUNC3D] [FIFO] [ALL]

Default: read-only (modules 1-10). Log goes to RAM:voodoo3diag.log and is
flushed after every line; ">> MODULE n" marks the start of each module,
so after a crash the log shows where it stopped.

  WRITE   harmless register round-trips (2D colorFore, CLUT entry 511,
          VGA DAC entry 255 with R,G,B order, pixel mask) -- restored
  FUNC    2D engine tests in a VRAM bitmap allocated by the program:
          rectangle fill (+ byte order of engine output as seen by the CPU),
          copy forward, overlapping copy -x/-y, ROP 0x66, mono expansion
  FUNC3D  3D fastfill into the same bitmap (16-bit screen only; overwrites
          3D state -- do not run while a Warp3D/MiniGL program is active)
  FIFO    CMDFIFO type-2 packet test (skipped if the driver uses CMDFIFO0)
  ALL     everything
  CSV     extra "CSV;module;test;status;value" lines for diffing runs on
          real hardware against QEMU

Ctrl-C stops between modules. Every engine wait has a timeout.
