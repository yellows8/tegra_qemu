/*
 * ARM NVIDIA Tegra2 emulation.
 *
 * Copyright (c) 2014-2015 Dmitry Osipenko <digetx@gmail.com>
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

#include "tegra_common.h"

#include "hw/sysbus.h"
#include "cpu.h"

#include "sizes.h"
#include "tegra_cpu.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_PG "tegra.pg"
#define TEGRA_PG(obj) OBJECT_CHECK(tegra_pg, (obj), TYPE_TEGRA_PG)

typedef struct tegra_pg_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
} tegra_pg;

static uint64_t tegra_pg_priv_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    CPUState *cs = current_cpu;

//     assert(cs != NULL);
    if (cs == NULL)
        return 0;

    if (cs->cpu_index == TEGRA_BPMP)
        return 0xAAAAAAAA;

    return 0x55555555;
}

static void tegra_pg_priv_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    TPRINT("%s: %08X -> %08X\n", __func__, (uint32_t) value,
           (uint32_t) offset + 0x60000000);
}

static const MemoryRegionOps tegra_pg_mem_ops = {
    .read = tegra_pg_priv_read,
    .write = tegra_pg_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_pg_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_pg *s = TEGRA_PG(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_pg_mem_ops, s,
                          "tegra.pg", SZ_4K);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_pg_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_pg_priv_realize;
}

static const TypeInfo tegra_pg_info = {
    .name = TYPE_TEGRA_PG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_pg),
    .class_init = tegra_pg_class_init,
};

static void tegra_pg_register_types(void)
{
    type_register_static(&tegra_pg_info);
}

type_init(tegra_pg_register_types)
