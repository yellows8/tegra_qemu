/*
 * ARM NVIDIA X1 emulation.
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

// Based on timer.c.

#define CONFIG_ARCH_TEGRA_21x_SOC

#include "tegra_common.h"

#include "hw/sysbus.h"
#include "hw/ptimer.h"

#include "timer_shared.h"
#include "timer.h"
#include "wdt.h"
#include "iomap.h"
#include "tegra_trace.h"
#include "devices.h"

#define TYPE_TEGRA_TIMER_SHARED "tegra.timer_shared"
#define TEGRA_TIMER_SHARED(obj) OBJECT_CHECK(tegra_timer_shared, (obj), TYPE_TEGRA_TIMER_SHARED)
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

static const VMStateDescription vmstate_tegra_timer_shared = {
    .name = TYPE_TEGRA_TIMER_SHARED,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(shared_timer_secure_cfg.reg32, tegra_timer_shared),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_timer_shared_priv_read(void *opaque, hwaddr offset,
                                             unsigned size)
{
    tegra_timer_shared *s = opaque;
    uint64_t ret = 0;

    switch (offset) {
    case SHARED_INTR_STATUS_OFFSET:
        shared_intr_status_t tmp = {};

        for (int i=0; i<5; i++) {
            tegra_wdt *wdt = tegra_wdt_devs[i];
            tegra_timer *tmr = tegra_timer_devs[6+i];

            if (wdt && wdt->status.interrupt_status) tmp.watchdog_src_bitmap |= (1<<i);
            if (tmr && tmr->irq_sts) tmp.timer_src_bitmap |= (1<<i);
        }

        ret = tmp.reg32;
        break;
    case SHARED_TIMER_SECURE_CFG_OFFSET:
        ret = s->shared_timer_secure_cfg.reg32;
        break;
    default:
        TRACE_READ(s->iomem.addr, offset, ret);
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_timer_shared_priv_write(void *opaque, hwaddr offset,
                                          uint64_t value, unsigned size)
{
    tegra_timer_shared *s = opaque;

    switch (offset) {
    case SHARED_INTR_STATUS_OFFSET: // Read-only reg, do nothing.
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    case SHARED_TIMER_SECURE_CFG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->shared_timer_secure_cfg.reg32, value & PCR_WRMASK);
        s->shared_timer_secure_cfg.reg32 = value;
        break;
    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }
}

static void tegra_timer_shared_priv_reset(DeviceState *dev)
{
    tegra_timer_shared *s = TEGRA_TIMER_SHARED(dev);

    s->shared_timer_secure_cfg.reg32 = SHARED_TIMER_SECURE_CFG_RESET;
}

static const MemoryRegionOps tegra_timer_shared_mem_ops = {
    .read = tegra_timer_shared_priv_read,
    .write = tegra_timer_shared_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_timer_shared_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_timer_shared *s = TEGRA_TIMER_SHARED(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_timer_shared_mem_ops, s,
                          TYPE_TEGRA_TIMER_SHARED, TEGRA_TMR_SHARED_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_timer_shared_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_timer_shared_priv_realize;
    dc->vmsd = &vmstate_tegra_timer_shared;
    dc->reset = tegra_timer_shared_priv_reset;
}

static const TypeInfo tegra_timer_shared_info = {
    .name = TYPE_TEGRA_TIMER_SHARED,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_timer_shared),
    .class_init = tegra_timer_shared_class_init,
};

static void tegra_timer_shared_register_types(void)
{
    type_register_static(&tegra_timer_shared_info);
}

type_init(tegra_timer_shared_register_types)
