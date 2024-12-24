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

#include "vde.h"

static uint64_t tegra_sxe_read(void *opaque, hwaddr offset,
                                 unsigned size)
{
    tegra_sxe *s = opaque;
    int rst_set = tegra_rst_asserted(TEGRA20_CLK_VDE);
    int clk_en = tegra_clk_enabled(TEGRA20_CLK_VDE);
    int reg_n = offset >> 2;
    uint32_t ret = 0;

    switch (offset) {
    case 0xC8:
        if (s->regs[4] & 5) {
            ret = s->regs[reg_n];
        }
        break;
    default:
        ret = s->regs[reg_n];
        break;
    }

    if (!clk_en) {
        ret = 1;
    }

    TRACE_READ_EXT(s->iomem.addr, offset, ret, !clk_en, rst_set);

    return ret;
}

static void tegra_sxe_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    tegra_sxe *s = opaque;
    int rst_set = tegra_rst_asserted(TEGRA20_CLK_VDE);
    int clk_en = tegra_clk_enabled(TEGRA20_CLK_VDE);
    int reg_n = offset >> 2;

    TRACE_WRITE_EXT(s->iomem.addr, offset, s->regs[reg_n], value,
                    !clk_en, rst_set);

    if (tegra_rst_asserted(TEGRA20_CLK_VDE)) {
        return;
    }

    if (!tegra_clk_enabled(TEGRA20_CLK_VDE)) {
        return;
    }

    switch (offset){
    case 0:
        if (value & 0x20000000) {
            s->regs[50] = value << 16;
        }
        break;
    default:
        s->regs[reg_n] = value;
    }
}

static const MemoryRegionOps tegra_sxe_mem_ops = {
    .read = tegra_sxe_read,
    .write = tegra_sxe_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_vde_sxe_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_sxe *s = TEGRA_VDE_SXE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &tegra_sxe_mem_ops, s,
                          "tegra.vde_sxe", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_vde_sxe_priv_reset(DeviceState *dev)
{
    tegra_sxe *s = TEGRA_VDE_SXE(dev);
    int i;

    for (i = 0; i < 1024; i++) {
        s->regs[i] = 0;
    }

    s->regs[10] = 0x00004000;
    s->regs[14] = 0x000001FF;
    s->regs[51] = 0xFC000000;
}

static void tegra_vde_sxe_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_vde_sxe_priv_realize;
    device_class_set_legacy_reset(dc, tegra_vde_sxe_priv_reset);
}

static const TypeInfo tegra_sxe_info = {
    .name = TYPE_TEGRA_VDE_SXE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_sxe),
    .class_init = tegra_vde_sxe_class_init,
};

static void tegra_sxe_register_types(void)
{
    type_register_static(&tegra_sxe_info);
}

type_init(tegra_sxe_register_types)
