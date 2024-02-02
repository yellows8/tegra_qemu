/*
 * ARM NVIDIA X1 emulation.
 *
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

// Based on tegra2 device code by digetx.

#define CONFIG_ARCH_TEGRA_21x_SOC

#include "tegra_common.h"

#include "hw/sysbus.h"

#include "mselect.h"

#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_MSELECT "tegra.mselect"
#define TEGRA_MSELECT(obj) OBJECT_CHECK(tegra_mselect, (obj), TYPE_TEGRA_MSELECT)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_mselect_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    DEFINE_REG32(config);
} tegra_mselect;

static const VMStateDescription vmstate_tegra_mselect = {
    .name = TYPE_TEGRA_MSELECT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(config.reg32, tegra_mselect),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_mselect_priv_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    tegra_mselect *s = opaque;
    uint64_t ret = 0;

    // Mirror all regs to CONFIG_OFFSET.

    ret = s->config.reg32;

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_mselect_priv_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    tegra_mselect *s = opaque;

    switch (offset) {
    case CONFIG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->config.reg32, value);
        s->config.reg32 = value;
        break;
    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }
}

static void tegra_mselect_priv_reset(DeviceState *dev)
{
    tegra_mselect *s = TEGRA_MSELECT(dev);

    s->config.reg32 = CONFIG_RESET;

    s->config.reg32 = 0xEAFFFFFE; // Validated by Erista NX_Bootloader with certain versions.
}

static const MemoryRegionOps tegra_mselect_mem_ops = {
    .read = tegra_mselect_priv_read,
    .write = tegra_mselect_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_mselect_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_mselect *s = TEGRA_MSELECT(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_mselect_mem_ops, s,
                          TYPE_TEGRA_MSELECT, TEGRA_MSELECT_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_mselect_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_mselect_priv_realize;
    dc->vmsd = &vmstate_tegra_mselect;
    dc->reset = tegra_mselect_priv_reset;
}

static const TypeInfo tegra_mselect_info = {
    .name = TYPE_TEGRA_MSELECT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_mselect),
    .class_init = tegra_mselect_class_init,
};

static void tegra_mselect_register_types(void)
{
    type_register_static(&tegra_mselect_info);
}

type_init(tegra_mselect_register_types)
