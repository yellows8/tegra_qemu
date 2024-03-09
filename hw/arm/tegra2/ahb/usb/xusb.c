/*
 * ARM NVIDIA Tegra2 emulation.
 *
 * Copyright (c) 2015 Dmitry Osipenko <digetx@gmail.com>
 * Copyright (c) yellows8
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, see <http://www.gnu.org/licenses/>.
 */

// Based on usb.c.

// NOTE: The memregion offsets/sizes used by qemu XHCI differs from what X1 uses.

#define CONFIG_ARCH_TEGRA_21x_SOC

#include "tegra_common.h"

#include "iomap.h"

#include "hw/usb/hcd-xhci.h"
#include "hw/usb/hcd-xhci-sysbus.h"

#include "tegra_trace.h"

#define TYPE_TEGRA_XUSB "tegra.xusb"
#define TEGRA_XUSB(obj) OBJECT_CHECK(tegra_xusb, (obj), TYPE_TEGRA_XUSB)
#define DEFINE_REG32(reg) reg##_t reg

typedef struct tegra_xusb_state {
    XHCISysbusState parent_obj;

    uint32_t regs_padctl[TEGRA_XUSB_PADCTL_SIZE>>2];
    uint32_t regs_device[(SZ_32K + SZ_8K)>>2];

    MemoryRegion iomem_padctl;
    MemoryRegion iomem_device;
} tegra_xusb;

static const VMStateDescription vmstate_tegra_xusb = {
    .name = TYPE_TEGRA_XUSB,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(xhci, XHCISysbusState, 2, vmstate_xhci, XHCIState),
        VMSTATE_UINT32_ARRAY(regs_padctl, tegra_xusb, TEGRA_XUSB_PADCTL_SIZE>>2),
        VMSTATE_UINT32_ARRAY(regs_device, tegra_xusb, (SZ_32K + SZ_8K)>>2),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_xusb_padctl_priv_read(void *opaque, hwaddr offset,
                                            unsigned size)
{
    tegra_xusb *s = opaque;
    uint64_t ret = 0;

    if (offset+size <= sizeof(s->regs_padctl)) ret = s->regs_padctl[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    TRACE_READ(s->iomem_padctl.addr, offset, ret);

    return ret;
}

static void tegra_xusb_padctl_priv_write(void *opaque, hwaddr offset,
                                         uint64_t value, unsigned size)
{
    tegra_xusb *s = opaque;

    TRACE_WRITE(s->iomem_padctl.addr, offset, 0, value);

    if (offset+size <= sizeof(s->regs_padctl)) s->regs_padctl[offset/sizeof(uint32_t)] = (s->regs_padctl[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;
}

static uint64_t tegra_xusb_device_priv_read(void *opaque, hwaddr offset,
                                            unsigned size)
{
    tegra_xusb *s = opaque;
    uint64_t ret = 0;

    if (offset+size <= sizeof(s->regs_device)) ret = s->regs_device[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    TRACE_READ(s->iomem_device.addr, offset, ret);

    return ret;
}

static void tegra_xusb_device_priv_write(void *opaque, hwaddr offset,
                                         uint64_t value, unsigned size)
{
    tegra_xusb *s = opaque;

    TRACE_WRITE(s->iomem_device.addr, offset, 0, value);

    if (offset+size <= sizeof(s->regs_device)) {
        if (offset == 0x8000) return; // Ignore read-only reg.
        s->regs_device[offset/sizeof(uint32_t)] = (s->regs_device[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;
    }
}

static void tegra_xusb_priv_reset(DeviceState *dev)
{
    tegra_xusb *s = TEGRA_XUSB(dev);

    memset(s->regs_padctl, 0, sizeof(s->regs_padctl));
    memset(s->regs_device, 0, sizeof(s->regs_device));

    // _T_XUSB_DEV_CFG_0, id reg
    s->regs_device[0x8000>>2] = 0x10DE | (0xFAD<<16);

    xhci_sysbus_reset(dev);
}

static const MemoryRegionOps tegra_xusb_padctl_mem_ops = {
    .read = tegra_xusb_padctl_priv_read,
    .write = tegra_xusb_padctl_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps tegra_xusb_device_mem_ops = {
    .read = tegra_xusb_device_priv_read,
    .write = tegra_xusb_device_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_xusb_priv_init(Object *obj)
{
    tegra_xusb *s = TEGRA_XUSB(obj);

    memory_region_init_io(&s->iomem_padctl, OBJECT(s), &tegra_xusb_padctl_mem_ops, s,
                          "tegra.xusb_padctl", TEGRA_XUSB_PADCTL_SIZE);
    memory_region_init_io(&s->iomem_device, OBJECT(s), &tegra_xusb_device_mem_ops, s,
                          "tegra.xusb_dev", SZ_32K + SZ_8K);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem_padctl);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem_device);
}

static void tegra_xusb_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_tegra_xusb;
    dc->reset = tegra_xusb_priv_reset;

    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

static const TypeInfo tegra_xusb_info = {
    .name = TYPE_TEGRA_XUSB,
    .parent = TYPE_XHCI_SYSBUS,
    .instance_size = sizeof(tegra_xusb),
    .instance_init = tegra_xusb_priv_init,
    .class_init = tegra_xusb_class_init,
};

static void tegra_xusb_register_types(void)
{
    type_register_static(&tegra_xusb_info);
}

type_init(tegra_xusb_register_types)
