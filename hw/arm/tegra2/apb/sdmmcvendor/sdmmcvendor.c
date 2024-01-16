/*
 * ARM NVIDIA Tegra2/X1 emulation.
 *
 * Copyright (c) 2014-2015 Dmitry Osipenko <digetx@gmail.com>
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

#include "tegra_common.h"

#include "hw/sysbus.h"

#include "devices.h"
#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_SDMMCVENDOR "tegra.sdmmcvendor"
#define TEGRA_SDMMCVENDOR(obj) OBJECT_CHECK(tegra_sdmmcvendor, (obj), TYPE_TEGRA_SDMMCVENDOR)

typedef struct tegra_sdmmcvendor_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[0x100>>2];
} tegra_sdmmcvendor;

static const VMStateDescription vmstate_tegra_sdmmcvendor= {
    .name = "tegra.sdmmcvendor",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, tegra_sdmmcvendor, 0x100>>2),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_sdmmcvendor_priv_read(void *opaque, hwaddr offset,
                                              unsigned size)
{
    tegra_sdmmcvendor *s = opaque;
    uint64_t ret = 0;

    assert(offset < sizeof(s->regs));

    ret = s->regs[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    TRACE_READ(s->iomem.addr + 0x100, offset, ret);

    return ret;
}

static void tegra_sdmmcvendor_priv_write(void *opaque, hwaddr offset,
                                           uint64_t value, unsigned size)
{
    tegra_sdmmcvendor *s = opaque;

    assert(offset < sizeof(s->regs));

    TRACE_WRITE(s->iomem.addr + 0x100, offset, 0, value);

    s->regs[offset/sizeof(uint32_t)] = (s->regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;
}

static void tegra_sdmmcvendor_priv_reset(DeviceState *dev)
{
    tegra_sdmmcvendor *s = TEGRA_SDMMCVENDOR(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static const MemoryRegionOps tegra_sdmmcvendor_mem_ops = {
    .read = tegra_sdmmcvendor_priv_read,
    .write = tegra_sdmmcvendor_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_sdmmcvendor_realize(DeviceState *dev, Error **errp)
{
    tegra_sdmmcvendor *s = TEGRA_SDMMCVENDOR(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &tegra_sdmmcvendor_mem_ops, s,
                          TYPE_TEGRA_SDMMCVENDOR, 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void tegra_sdmmcvendor_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_tegra_sdmmcvendor;
    dc->realize = tegra_sdmmcvendor_realize;
    dc->reset = tegra_sdmmcvendor_priv_reset;
}

static const TypeInfo tegra_sdmmcvendor_info = {
    .name = TYPE_TEGRA_SDMMCVENDOR,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_sdmmcvendor),
    .class_init = tegra_sdmmcvendor_class_init,
};

static void tegra_sdmmcvendor_register_types(void)
{
    type_register_static(&tegra_sdmmcvendor_info);
}

type_init(tegra_sdmmcvendor_register_types)
