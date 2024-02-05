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
#include "qemu/main-loop.h"

#include "timer.h"
#include "wdt.h"
#include "iomap.h"
#include "tegra_trace.h"
#include "devices.h"

#define TYPE_TEGRA_TIMER "tegra.timer"
#define TEGRA_TIMER(obj) OBJECT_CHECK(tegra_timer, (obj), TYPE_TEGRA_TIMER)
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

#define SCALE   1

#define PTIMER_POLICY                       \
    (PTIMER_POLICY_WRAP_AFTER_ONE_PERIOD |  \
     PTIMER_POLICY_CONTINUOUS_TRIGGER    |  \
     PTIMER_POLICY_NO_IMMEDIATE_TRIGGER  |  \
     PTIMER_POLICY_NO_IMMEDIATE_RELOAD   |  \
     PTIMER_POLICY_NO_COUNTER_ROUND_DOWN)

static const VMStateDescription vmstate_tegra_timer = {
    .name = "tegra.timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_PTIMER(ptimer, tegra_timer),
        VMSTATE_UINT32(ptv.reg32, tegra_timer),
        VMSTATE_UINT32(pcr.reg32, tegra_timer),
        VMSTATE_UINT32(id, tegra_timer),
        VMSTATE_BOOL(pcr_read, tegra_timer),
        VMSTATE_END_OF_LIST()
    }
};

static void tegra_timer_alarm(void *opaque)
{
    tegra_timer *s = opaque;

    s->ptv.en = s->ptv.per;

//     TPRINT("[%lu] tegra_timer_alarm!\n", qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
    if (!s->irq_sts) {
	TRACE_IRQ_RAISE(s->iomem.addr, s->irq);
    }

    s->irq_sts = 1;

    // Handle the watchdog timers which are configured for using the current timer.
    for (size_t i=0; i<ARRAY_SIZE(tegra_wdt_devs); i++) {
        tegra_wdt *wdt = tegra_wdt_devs[i];
        if (wdt && wdt->status.enabled && wdt->config.timer_source == s->id) {
            ptimer_set_limit(s->ptimer, wdt->config.period ? wdt->config.period : 0xFF, 1);
            tegra_wdt_alarm(wdt);
        }
    }
}

uint64_t tegra_timer_get_count(void *opaque)
{
    tegra_timer *s = opaque;

    return ptimer_get_count(s->ptimer);
}

static uint64_t tegra_timer_priv_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    tegra_timer *s = opaque;
    uint64_t ret = 0;

    switch (offset) {
    case PTV_OFFSET:
        ret = s->ptv.reg32;
        break;
    case PCR_OFFSET:
        s->pcr.tmr_pcv = tegra_timer_get_count(s);
        ret = s->pcr.reg32;
        s->pcr_read = true; // Used by wdt.
        break;
    default:
        TRACE_READ(s->iomem.addr, offset, ret);
        g_assert_not_reached();
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_timer_priv_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    tegra_timer *s = opaque;

    switch (offset) {
    case PTV_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ptv.reg32, value);
        s->ptv.reg32 = value;

        ptimer_transaction_begin(s->ptimer);
        ptimer_stop(s->ptimer);

        if (!s->ptv.en) {
            ptimer_transaction_commit(s->ptimer);
            break;
        }

//         TPRINT("[%lu] timer set to %d\n", qemu_clock_get_us(QEMU_CLOCK_VIRTUAL), s->ptv.tmr_ptv);

        //assert((s->ptv.tmr_ptv && s->ptv.per) || !s->ptv.per);

        ptimer_set_limit(s->ptimer, s->ptv.tmr_ptv + 1, 1);
        ptimer_run(s->ptimer, !s->ptv.per);
        ptimer_transaction_commit(s->ptimer);
        break;
    case PCR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pcr.reg32, value & PCR_WRMASK);
        WR_MASKED(s->pcr.reg32, value, PCR);

        if (s->pcr.intr_clr && s->irq_sts) {
            TRACE_IRQ_LOWER(s->iomem.addr, s->irq);
            s->irq_sts = 0;
//             TPRINT("timer irq cleared\n");;
        }
        break;
    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        g_assert_not_reached();
        break;
    }
}

static void tegra_timer_priv_reset(DeviceState *dev)
{
    tegra_timer *s = TEGRA_TIMER(dev);

    s->ptv.reg32 = PTV_RESET;
    s->pcr.reg32 = PCR_RESET;
    s->irq_sts = 0;
    s->pcr_read = false;
}

static const MemoryRegionOps tegra_timer_mem_ops = {
    .read = tegra_timer_priv_read,
    .write = tegra_timer_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_timer_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_timer *s = TEGRA_TIMER(dev);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_timer_mem_ops, s,
                          "tegra.timer", 0x8);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    s->ptimer = ptimer_init(tegra_timer_alarm, s, PTIMER_POLICY);
    ptimer_transaction_begin(s->ptimer);
    ptimer_set_freq(s->ptimer, 1000000 * SCALE);
    ptimer_transaction_commit(s->ptimer);
}

static Property tegra_timer_properties[] = {
    DEFINE_PROP_UINT32("id", tegra_timer, id, 0), \
    DEFINE_PROP_END_OF_LIST(),
};

static void tegra_timer_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, tegra_timer_properties);
    dc->realize = tegra_timer_priv_realize;
    dc->vmsd = &vmstate_tegra_timer;
    dc->reset = tegra_timer_priv_reset;
}

static const TypeInfo tegra_timer_info = {
    .name = TYPE_TEGRA_TIMER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_timer),
    .class_init = tegra_timer_class_init,
};

static void tegra_timer_register_types(void)
{
    type_register_static(&tegra_timer_info);
}

type_init(tegra_timer_register_types)
