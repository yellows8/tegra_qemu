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
#include "remote_io.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_BSE_REMOTE "tegra.bse_remote"
#define TEGRA_BSE_REMOTE(obj) OBJECT_CHECK(bse_remote, (obj), TYPE_TEGRA_BSE_REMOTE)

typedef struct bse_remote_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq_ucq_error;
    qemu_irq irq_sync_token;
    qemu_irq irq_bse_v;
    qemu_irq irq_bse_a;
    qemu_irq irq_sxe;
} bse_remote;

static uint64_t bse_remote_read(void *opaque, hwaddr offset,
                                 unsigned size)
{
    bse_remote *s = opaque;
    uint32_t ret = remote_io_read(s->iomem.addr + offset, size << 3);
    int rst_set = tegra_rst_asserted(TEGRA20_CLK_VDE);
    int clk_en = tegra_clk_enabled(TEGRA20_CLK_VDE);
    uint32_t base;
    uint32_t mask;

    switch (offset) {
    case 0x0000 ... 0x00FF:
        base = 0x60010000;
        mask = 0xFF;
        break;
    case 0xA000 ... 0xAFFF:
        base = 0x6001A000;
        mask = 0xFFF;
        break;
    case 0xB000 ... 0xBFFF:
        base = 0x6001B000;
        mask = 0xFFF;
        break;
    case 0xC000 ... 0xC0FF:
        base = 0x6001C000;
        mask = 0xFF;
        break;
    case 0xC200 ... 0xC2FF:
        base = 0x6001C200;
        mask = 0xFF;
        break;
    case 0xC400 ... 0xC4FF:
        base = 0x6001C400;
        mask = 0xFF;
        break;
    case 0xC600 ... 0xC6FF:
        base = 0x6001C600;
        mask = 0xFF;
        break;
    case 0xC800 ... 0xC8FF:
        base = 0x6001C800;
        mask = 0xFF;
        break;
    case 0xCA00 ... 0xCAFF:
        base = 0x6001CA00;
        mask = 0xFF;
        break;
    case 0xCC00 ... 0xCCFF:
        base = 0x6001CC00;
        mask = 0xFF;
        break;
    case 0xD000 ... 0xD7FF:
        base = 0x6001D000;
        mask = 0x7FF;
        break;
    case 0xD800 ... 0xDAFF:
        base = 0x6001D800;
        mask = 0x3FF;
        break;
    default:
        base = s->iomem.addr;
        mask = 0xFFFF;
        break;
    }

    TRACE_READ_EXT(base, offset & mask, ret, !clk_en, rst_set);

    return ret;
}

static void bse_remote_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    bse_remote *s = opaque;
    int rst_set = tegra_rst_asserted(TEGRA20_CLK_VDE);
    int clk_en = tegra_clk_enabled(TEGRA20_CLK_VDE);
    uint32_t base;
    uint32_t mask;

    switch (offset) {
    case 0x0000 ... 0x00FF:
        base = 0x60010000;
        mask = 0xFF;
        break;
    case 0xA000 ... 0xAFFF:
        base = 0x6001A000;
        mask = 0xFFF;
        break;
    case 0xB000 ... 0xBFFF:
        base = 0x6001B000;
        mask = 0xFFF;
        break;
    case 0xC000 ... 0xC0FF:
        base = 0x6001C000;
        mask = 0xFF;
        break;
    case 0xC200 ... 0xC2FF:
        base = 0x6001C200;
        mask = 0xFF;
        break;
    case 0xC400 ... 0xC4FF:
        base = 0x6001C400;
        mask = 0xFF;
        break;
    case 0xC600 ... 0xC6FF:
        base = 0x6001C600;
        mask = 0xFF;
        break;
    case 0xC800 ... 0xC8FF:
        base = 0x6001C800;
        mask = 0xFF;
        break;
    case 0xCA00 ... 0xCAFF:
        base = 0x6001CA00;
        mask = 0xFF;
        break;
    case 0xCC00 ... 0xCCFF:
        base = 0x6001CC00;
        mask = 0xFF;
        break;
    case 0xD000 ... 0xD7FF:
        base = 0x6001D000;
        mask = 0x7FF;
        break;
    case 0xD800 ... 0xDAFF:
        base = 0x6001D800;
        mask = 0x3FF;
        break;
    default:
        base = s->iomem.addr;
        mask = 0xFFFF;
        break;
    }

    TRACE_WRITE_EXT(base, offset & mask, value, value, !clk_en, rst_set);

    remote_io_write(value, s->iomem.addr + offset, size << 3);
}

static const MemoryRegionOps bse_remote_mem_ops = {
    .read = bse_remote_read,
    .write = bse_remote_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_vde_bse_priv_realize(DeviceState *dev, Error **errp)
{
    bse_remote *s = TEGRA_BSE_REMOTE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &bse_remote_mem_ops, s,
                          "tegra.vde_bse", 0xDB00);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_ucq_error);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_sync_token);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_bse_v);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_bse_a);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_sxe);
}

static void tegra_vde_bse_priv_reset(DeviceState *dev)
{
    bse_remote *s = TEGRA_BSE_REMOTE(dev);

    remote_io_watch_irq(s->iomem.addr, &s->irq_ucq_error);
    remote_io_watch_irq(s->iomem.addr, &s->irq_sync_token);
    remote_io_watch_irq(s->iomem.addr, &s->irq_bse_v);
    remote_io_watch_irq(s->iomem.addr, &s->irq_bse_a);
    remote_io_watch_irq(s->iomem.addr, &s->irq_sxe);
}

static void bse_remote_remote_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_vde_bse_priv_realize;
    device_class_set_legacy_reset(dc, tegra_vde_bse_priv_reset);
}

static const TypeInfo bse_remote_info = {
    .name = TYPE_TEGRA_BSE_REMOTE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(bse_remote),
    .class_init = bse_remote_remote_class_init,
};

static void bse_remote_register_types(void)
{
    type_register_static(&bse_remote_info);
}

type_init(bse_remote_register_types)
