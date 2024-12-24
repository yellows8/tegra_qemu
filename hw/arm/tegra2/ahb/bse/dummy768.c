/*
 * ARM NVIDIA Tegra2 emulation.
 *
 * Copyright (c) 2015 Dmitry Osipenko <digetx@gmail.com>
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

#include "clk_rst.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_BSE_DUMMY_768B "tegra.dummy768"
#define TEGRA_DUMMY(obj) OBJECT_CHECK(tegra_dummy, (obj), TYPE_TEGRA_BSE_DUMMY_768B)

typedef struct tegra_dummy_state {
    SysBusDevice parent_obj;

    uint32_t regs[192];
    MemoryRegion iomem;
} tegra_dummy;

static uint64_t tegra_dummy_read(void *opaque, hwaddr offset,
                                 unsigned size)
{
    tegra_dummy *s = opaque;
    uint32_t ret = s->regs[offset >> 2];
    int rst_set = tegra_rst_asserted(TEGRA20_CLK_VDE);
    int clk_en = tegra_clk_enabled(TEGRA20_CLK_VDE);

    TRACE_READ_EXT(s->iomem.addr, offset, ret, !clk_en, rst_set);

    return ret;
}

static void tegra_dummy_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    tegra_dummy *s = opaque;
    int rst_set = tegra_rst_asserted(TEGRA20_CLK_VDE);
    int clk_en = tegra_clk_enabled(TEGRA20_CLK_VDE);

    TRACE_WRITE_EXT(s->iomem.addr, offset, s->regs[offset >> 2], value,
                    !clk_en, rst_set);

    if (tegra_rst_asserted(TEGRA20_CLK_VDE)) {
        return;
    }

    if (!tegra_clk_enabled(TEGRA20_CLK_VDE)) {
        return;
    }

    s->regs[offset >> 2] = value;
}

static const MemoryRegionOps tegra_dummy_mem_ops = {
    .read = tegra_dummy_read,
    .write = tegra_dummy_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_bse_dummy_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_dummy *s = TEGRA_DUMMY(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &tegra_dummy_mem_ops, s,
                          "tegra.bse_dummy", 768);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_bse_dummy_priv_reset(DeviceState *dev)
{
    tegra_dummy *s = TEGRA_DUMMY(dev);
    int i;

    for (i = 0; i < 192; i++) {
        s->regs[i] = 0;
    }
}

static void tegra_bse_dummy_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_bse_dummy_priv_realize;
    device_class_set_legacy_reset(dc, tegra_bse_dummy_priv_reset);
}

static const TypeInfo tegra_dummy_info = {
    .name = TYPE_TEGRA_BSE_DUMMY_768B,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_dummy),
    .class_init = tegra_bse_dummy_class_init,
};

static void tegra_dummy_register_types(void)
{
    type_register_static(&tegra_dummy_info);
}

type_init(tegra_dummy_register_types)
