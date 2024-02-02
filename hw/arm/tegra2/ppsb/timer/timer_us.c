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

#include "hw/ptimer.h"
#include "hw/sysbus.h"

#include "timer_us.h"
#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_TIMER_US "tegra.timer_us"
#define TEGRA_TIMER_US(obj) OBJECT_CHECK(tegra_timer_us, (obj), TYPE_TEGRA_TIMER_US)

#define SCALE   1

static const VMStateDescription vmstate_tegra_timer_us = {
    .name = "tegra.timer_us",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(cntr_1us.reg32, tegra_timer_us),
        VMSTATE_UINT32(usec_cfg.reg32, tegra_timer_us),
        VMSTATE_UINT32(cntr_freeze.reg32, tegra_timer_us),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_timer_us_priv_read(void *opaque, hwaddr offset,
                                         unsigned size)
{
    tegra_timer_us *s = opaque;
    uint64_t ret = 0;

    switch (offset) {
    case CNTR_1US_OFFSET:
        s->cntr_1us.reg32 = ~ptimer_get_count(s->ptimer);
        ret = s->cntr_1us.reg32;
        break;
    case USEC_CFG_OFFSET:
        ret = s->usec_cfg.reg32;
        break;
    case CNTR_FREEZE_OFFSET:
        ret = s->cntr_freeze.reg32;
        break;
    default:
        TRACE_READ(s->iomem.addr, offset, 0);
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_timer_us_priv_write(void *opaque, hwaddr offset,
                                      uint64_t value, unsigned size)
{
    tegra_timer_us *s = opaque;

    switch (offset) {
    case USEC_CFG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->usec_cfg.reg32, value);
        s->usec_cfg.reg32 = value;
        break;
    case CNTR_FREEZE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->cntr_freeze.reg32, value);
        s->cntr_freeze.reg32 = value;
        break;
    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }
}

static void tegra_timer_us_priv_reset(DeviceState *dev)
{
    tegra_timer_us *s = TEGRA_TIMER_US(dev);

    ptimer_transaction_begin(s->ptimer);
    ptimer_run(s->ptimer, 0);
    ptimer_transaction_commit(s->ptimer);

    s->cntr_1us.reg32 = CNTR_1US_RESET;
    s->usec_cfg.reg32 = USEC_CFG_RESET;
    s->cntr_freeze.reg32 = CNTR_FREEZE_RESET;
}

static const MemoryRegionOps tegra_timer_us_mem_ops = {
    .read = tegra_timer_us_priv_read,
    .write = tegra_timer_us_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_timer_us_tick(void *opaque)
{
    /* Nothing to do on timer rollover */
}

static void tegra_timer_us_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_timer_us *s = TEGRA_TIMER_US(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_timer_us_mem_ops, s,
                          "tegra.timer_us", TEGRA_TMRUS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    s->ptimer = ptimer_init(tegra_timer_us_tick, s, PTIMER_POLICY_CONTINUOUS_TRIGGER);
    ptimer_transaction_begin(s->ptimer);
    ptimer_set_freq(s->ptimer, 1000000 * SCALE);
    ptimer_set_limit(s->ptimer, 0xffffffff, 1);
    ptimer_transaction_commit(s->ptimer);
}

static void tegra_timer_us_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_timer_us_priv_realize;
    dc->vmsd = &vmstate_tegra_timer_us;
    dc->reset = tegra_timer_us_priv_reset;
}

static const TypeInfo tegra_timer_us_info = {
    .name = TYPE_TEGRA_TIMER_US,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_timer_us),
    .class_init = tegra_timer_us_class_init,
};

static void tegra_timer_us_register_types(void)
{
    type_register_static(&tegra_timer_us_info);
}

type_init(tegra_timer_us_register_types)
