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

#include "arb_gnt_ictlr.h"
#include "arb.h"
#include "devices.h"
#include "iomap.h"
#include "tegra_trace.h"

static const VMStateDescription vmstate_tegra_arb_gnt_ictlr = {
    .name = "tegra.arb_gnt_ictlr",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(pri_ictlr_arbgnt_cpu_enable, tegra_arb_gnt_ictlr),
        VMSTATE_UINT32(pri_ictlr_arbgnt_cop_enable, tegra_arb_gnt_ictlr),
        VMSTATE_END_OF_LIST()
    }
};

uint32_t tegra_arb_sema_irq_enabled(uint8_t id)
{
    tegra_arb_gnt_ictlr *s = tegra_arb_gnt_ictlr_dev;

    assert(s != NULL && id < TEGRA_SEMA_NB);

    if (id == TEGRA_SEMA_CPU)
        return s->pri_ictlr_arbgnt_cpu_enable;

    return s->pri_ictlr_arbgnt_cop_enable;
}

static uint64_t tegra_arb_gnt_ictlr_priv_read(void *opaque, hwaddr offset,
                                              unsigned size)
{
    tegra_arb_gnt_ictlr *s = opaque;
    uint64_t ret = 0;

    switch (offset) {
    case CPU_STATUS_OFFSET:
        ret = tegra_arb_sema_gnt_status(TEGRA_SEMA_CPU);
        break;
    case CPU_ENABLE_OFFSET:
        ret = s->pri_ictlr_arbgnt_cpu_enable;
        break;
    case COP_STATUS_OFFSET:
        ret = tegra_arb_sema_gnt_status(TEGRA_SEMA_COP);
        break;
    case COP_ENABLE_OFFSET:
        ret = s->pri_ictlr_arbgnt_cop_enable;
        break;
    default:
        break;
    }

    TRACE_READ(TEGRA_ARBGNT_ICTLR_BASE, offset, ret);

    return ret;
}

static void tegra_arb_gnt_ictlr_priv_write(void *opaque, hwaddr offset,
                                           uint64_t value, unsigned size)
{
    tegra_arb_gnt_ictlr *s = opaque;

    switch (offset) {
    case CPU_ENABLE_OFFSET:
        TRACE_WRITE(TEGRA_ARBGNT_ICTLR_BASE, offset, s->pri_ictlr_arbgnt_cpu_enable, value);
        s->pri_ictlr_arbgnt_cpu_enable = value;
        break;

    case COP_ENABLE_OFFSET:
        TRACE_WRITE(TEGRA_ARBGNT_ICTLR_BASE, offset, s->pri_ictlr_arbgnt_cop_enable, value);
        s->pri_ictlr_arbgnt_cop_enable = value;
        break;

    default:
        TRACE_WRITE(TEGRA_ARBGNT_ICTLR_BASE, offset, 0, value);
        return;
    }

    tegra_arb_sema_update();
}

static void tegra_arb_gnt_ictlr_priv_reset(DeviceState *dev)
{
    tegra_arb_gnt_ictlr *s = TEGRA_ARBGNT_ICTLR(dev);

    s->pri_ictlr_arbgnt_cpu_enable = CPU_ENABLE_RESET;
    s->pri_ictlr_arbgnt_cop_enable = COP_ENABLE_RESET;
}

static const MemoryRegionOps tegra_arb_gnt_ictlr_mem_ops = {
    .read = tegra_arb_gnt_ictlr_priv_read,
    .write = tegra_arb_gnt_ictlr_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_arb_gnt_ictlr_realize(DeviceState *dev, Error **errp)
{
    tegra_arb_gnt_ictlr *s = TEGRA_ARBGNT_ICTLR(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &tegra_arb_gnt_ictlr_mem_ops, s,
                          "tegra.arb_gnt_ictlr", TEGRA_ARBGNT_ICTLR_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static Property tegra_arb_gnt_ictlr_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void tegra_arb_gnt_ictlr_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, tegra_arb_gnt_ictlr_properties);
    dc->vmsd = &vmstate_tegra_arb_gnt_ictlr;
    dc->realize = tegra_arb_gnt_ictlr_realize;
    device_class_set_legacy_reset(dc, tegra_arb_gnt_ictlr_priv_reset);
}

static const TypeInfo tegra_arb_gnt_ictlr_info = {
    .name = TYPE_TEGRA_ARBGNT_ICTLR,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_arb_gnt_ictlr),
    .class_init = tegra_arb_gnt_ictlr_class_init,
};

static void tegra_arb_gnt_ictlr_register_types(void)
{
    type_register_static(&tegra_arb_gnt_ictlr_info);
}

type_init(tegra_arb_gnt_ictlr_register_types)
