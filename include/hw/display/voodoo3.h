/*
 * QEMU 3Dfx Voodoo 3 / Banshee — public header
 *
 * Ported from 86Box (vid_voodoo_banshee.c et al.)
 * Original 86Box authors: Sarah Walker et al.
 *
 * Copyright (C) 2026 <your name here>
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef INCLUDE_HW_DISPLAY_VOODOO3_H
#define INCLUDE_HW_DISPLAY_VOODOO3_H

#include "hw/pci/pci_device.h"
#include "hw/core/qdev-properties.h"
#include "qapi/error.h"

#define TYPE_VOODOO3_PCI  "voodoo3"

typedef struct Voodoo3State Voodoo3State;
DECLARE_INSTANCE_CHECKER(Voodoo3State, VOODOO3_PCI, TYPE_VOODOO3_PCI)

#define VOODOO3_MODEL_BANSHEE    0u
#define VOODOO3_MODEL_V3_1000    1u
#define VOODOO3_MODEL_V3_2000    2u
#define VOODOO3_MODEL_V3_3000    3u
#define VOODOO3_MODEL_V3_3500TV  4u

static inline DeviceState *
voodoo3_create(PCIBus *bus, uint32_t model, bool is_agp, bool big_endian_fb)
{
    DeviceState *dev = qdev_new(TYPE_VOODOO3_PCI);
    qdev_prop_set_uint32(dev, "model", model);
    qdev_prop_set_bit(dev,   "agp",   is_agp);
    qdev_prop_set_bit(dev,   "big-endian-framebuffer", big_endian_fb);
    pci_realize_and_unref(PCI_DEVICE(dev), bus, &error_fatal);
    return dev;
}

#endif /* INCLUDE_HW_DISPLAY_VOODOO3_H */
