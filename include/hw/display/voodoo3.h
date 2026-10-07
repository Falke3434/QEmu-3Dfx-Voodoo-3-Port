/*
 * QEMU 3Dfx Voodoo 3 / Banshee — public header
 *
 * Ported from 86Box (vid_voodoo_banshee.c et al.)
 * Original 86Box authors: Sarah Walker et al.
 *
 * QEMU port: https://github.com/Falke3434/QEmu-3Dfx-Voodoo-3-Port
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef INCLUDE_HW_DISPLAY_VOODOO3_H
#define INCLUDE_HW_DISPLAY_VOODOO3_H

#include "hw/pci/pci_device.h"

#define TYPE_VOODOO3_PCI  "voodoo3"

typedef struct Voodoo3State Voodoo3State;
DECLARE_INSTANCE_CHECKER(Voodoo3State, VOODOO3_PCI, TYPE_VOODOO3_PCI)

#define VOODOO3_MODEL_BANSHEE    0u
#define VOODOO3_MODEL_V3_1000    1u
#define VOODOO3_MODEL_V3_2000    2u
#define VOODOO3_MODEL_V3_3000    3u
#define VOODOO3_MODEL_V3_3500TV  4u

#endif /* INCLUDE_HW_DISPLAY_VOODOO3_H */
