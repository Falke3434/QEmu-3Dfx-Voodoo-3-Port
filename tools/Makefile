# Makefile for voodoo3diag -- AmigaOS 4.1 PPC
#
# Cross build (e.g. walkero/amigagccondocker:os4-gcc11):
#     make
# Native build on AmigaOS 4.1:
#     make CC=gcc
#
# Default C runtime is newlib; for clib2 use:  make CRT=clib2

CC      ?= ppc-amigaos-gcc
CRT     ?= newlib

CFLAGS  = -O2 -Wall -Wextra -Wno-unused-parameter -fno-strict-aliasing -mcrt=$(CRT)
LDFLAGS = -mcrt=$(CRT)
LIBS    = -lauto

TARGET  = voodoo3diag

all: $(TARGET)

$(TARGET): voodoo3diag.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS) $(LIBS)

clean:
	rm -f $(TARGET) *.o
