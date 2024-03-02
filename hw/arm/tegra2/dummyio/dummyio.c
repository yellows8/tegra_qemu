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

#include "tegra_common.h"

#include "hw/sysbus.h"

#include "iomap.h"
#include "tegra_trace.h"

#include "dummyio.h"

#define TYPE_TEGRA_DUMMYIO "tegra.dummyio"
#define TEGRA_DUMMYIO(obj) OBJECT_CHECK(tegra_dummyio, (obj), TYPE_TEGRA_DUMMYIO)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_dummyio_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t size;
    char *name;
    uint32_t regs[SZ_256K>>2];
} tegra_dummyio;

static const VMStateDescription vmstate_tegra_dummyio = {
    .name = TYPE_TEGRA_DUMMYIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, tegra_dummyio, SZ_256K>>2),
        VMSTATE_END_OF_LIST()
    }
};

void dummyio_set_reg(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    tegra_dummyio *s = opaque;

    if (offset+size <= s->size)
        s->regs[offset/sizeof(uint32_t)] = (s->regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;
}

static uint64_t tegra_dummyio_priv_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    tegra_dummyio *s = opaque;
    uint64_t ret = 0;

    if (offset+size <= s->size) ret = s->regs[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_dummyio_priv_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    tegra_dummyio *s = opaque;

    TRACE_WRITE(s->iomem.addr, offset, 0, value);

    dummyio_set_reg(s, offset, value, size);
}

static void tegra_dummyio_priv_reset(DeviceState *dev)
{
    tegra_dummyio *s = TEGRA_DUMMYIO(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static const MemoryRegionOps tegra_dummyio_mem_ops = {
    .read = tegra_dummyio_priv_read,
    .write = tegra_dummyio_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_dummyio_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_dummyio *s = TEGRA_DUMMYIO(dev);

    assert(s->size <= sizeof(s->regs));

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_dummyio_mem_ops, s,
                          s->name ? s->name : TYPE_TEGRA_DUMMYIO, s->size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static Property tegra_dummyio_properties[] = {
    DEFINE_PROP_UINT32("size", tegra_dummyio, size, SZ_256K), \
    DEFINE_PROP_STRING("name", tegra_dummyio, name),
    DEFINE_PROP_END_OF_LIST(),
};

static void tegra_dummyio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, tegra_dummyio_properties);
    dc->realize = tegra_dummyio_priv_realize;
    dc->vmsd = &vmstate_tegra_dummyio;
    dc->reset = tegra_dummyio_priv_reset;
}

static const TypeInfo tegra_dummyio_info = {
    .name = TYPE_TEGRA_DUMMYIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_dummyio),
    .class_init = tegra_dummyio_class_init,
};

static void tegra_dummyio_register_types(void)
{
    type_register_static(&tegra_dummyio_info);
}

type_init(tegra_dummyio_register_types)
