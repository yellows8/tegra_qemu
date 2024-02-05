/*
 * ARM NVIDIA Tegra2 emulation.
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

#include "tegra_common.h"

#include "hw/ptimer.h"
#include "hw/sysbus.h"
#include "qemu/main-loop.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "sysemu/sysemu.h"
#include "qapi/qapi-commands.h"
#include "sysemu/watchdog.h"

#include "tegra_cpu.h"

#include "wdt.h"
#include "timer.h"
#include "../apb/pmc/pmc.h"
#include "devices.h"
#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_WDT "tegra.wdt"
#define TEGRA_WDT(obj) OBJECT_CHECK(tegra_wdt, (obj), TYPE_TEGRA_WDT)
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

#define SCALE   1

#define PTIMER_POLICY                       \
    (PTIMER_POLICY_WRAP_AFTER_ONE_PERIOD |  \
     PTIMER_POLICY_CONTINUOUS_TRIGGER    |  \
     PTIMER_POLICY_NO_IMMEDIATE_TRIGGER  |  \
     PTIMER_POLICY_NO_IMMEDIATE_RELOAD   |  \
     PTIMER_POLICY_NO_COUNTER_ROUND_DOWN)

static const VMStateDescription vmstate_tegra_wdt = {
    .name = "tegra.timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_PTIMER(ptimer, tegra_wdt),
        VMSTATE_UINT32(config.reg32, tegra_wdt),
        VMSTATE_UINT32(status.reg32, tegra_wdt),
        VMSTATE_UINT32(command.reg32, tegra_wdt),
        VMSTATE_UINT32(unlock_pattern.reg32, tegra_wdt),
        VMSTATE_INT32(expiration_count, tegra_wdt),
        VMSTATE_END_OF_LIST()
    }
};

static inline tegra_timer *tegra_wdt_get_timer_source(void *opaque)
{
    tegra_wdt *s = opaque;
    tegra_timer *timer = NULL;

    if (s->config.timer_source < ARRAY_SIZE(tegra_timer_devs)) {
        timer = tegra_timer_devs[s->config.timer_source];
    }

    return timer;
}

void tegra_wdt_alarm(void *opaque)
{
    tegra_wdt *s = opaque;

    s->expiration_count++;

//     TPRINT("[%lu] tegra_wdt_alarm!\n", qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));

    if (s->expiration_count==1) { // IRQ
        if (s->config.interrupt_en) {
            if (!s->status.interrupt_status) {
	        TRACE_IRQ_RAISE(s->iomem.addr, s->irq);
            }

            s->status.interrupt_status = 1;
        }
    }
    else {
        tegra_timer *timer = tegra_wdt_get_timer_source(s);
        if (timer && !timer->pcr_read) {
            if (s->expiration_count==2) { // FIQ
                if (s->config.fiq_enable) {
                    if (!s->status.fiq_status) {
	                TRACE_IRQ_RAISE(s->iomem.addr, s->fiq);
                    }

                    s->status.fiq_status = 1;
                }
            }
            else if (s->expiration_count==3) { // Directed reset
                for (int cpu_id=0; cpu_id<5; cpu_id++) {
                    if (s->config.core_reset_bitmap_en & (1<<cpu_id)) {
                        qemu_log_mask(LOG_GUEST_ERROR, "tegra.wdt: Asserting reset for cpu %d.\n", cpu_id);
                        tegra_cpu_reset_assert(cpu_id);
                    }
                }
            }
            else if (s->expiration_count==4) { // System reset
                if (s->status.interrupt_status) {
                    TRACE_IRQ_LOWER(s->iomem.addr, s->irq);
                    s->status.interrupt_status = 0;
                }

                if (s->status.fiq_status) {
                    TRACE_IRQ_LOWER(s->iomem.addr, s->fiq);
                    s->status.fiq_status = 0;
                }

                if (s->config.system_reset_enable || s->config.pmc2car_reset_enable) {
                    qemu_log_mask(LOG_GUEST_ERROR, "tegra.wdt: Requesting a system reset.\n");
                    Error *err = NULL;
                    tegra_pmc_set_rst_status(0x1); // RST_SOURCE = WATCHDOG
                    qmp_watchdog_set_action(WATCHDOG_ACTION_RESET, &err);
                    watchdog_perform_action();
                    if (err) error_report_err(err);
                }
                else {
                    for (int cpu_id=0; cpu_id<5; cpu_id++) {
                        if (s->config.core_reset_bitmap_en & (1<<cpu_id)) {
                            qemu_log_mask(LOG_GUEST_ERROR, "tegra.wdt: Deasserting reset for cpu %d.\n", cpu_id);
                            tegra_cpu_reset_deassert(cpu_id, 0);
                        }
                    }
                }
                s->status.enabled = 0;
            }
        }
    }
}

static uint64_t tegra_wdt_priv_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    tegra_wdt *s = opaque;
    uint64_t ret = 0;

    switch (offset) {
    case CONFIG_OFFSET:
        ret = s->config.reg32;
        break;
    case STATUS_OFFSET:
        status_t out = { .reg32 = s->status.reg32 };
        out.current_expiration_count = s->expiration_count+1;

        tegra_timer *timer = tegra_wdt_get_timer_source(s);
        if (timer) out.current_count = tegra_timer_get_count(timer);

        ret = out.reg32;
        break;
    case COMMAND_OFFSET:
        ret = s->command.reg32;
        break;
    case UNLOCK_PATTERN_OFFSET:
        ret = s->unlock_pattern.reg32;
        break;
    default:
        TRACE_READ(s->iomem.addr, offset, ret);
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_wdt_priv_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    tegra_wdt *s = opaque;

    switch (offset) {
    case CONFIG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->config.reg32, value);
        if (s->status.enabled)
            qemu_log_mask(LOG_GUEST_ERROR, "tegra.wdt: Ignoring write to CONFIG with the watchdog already enabled.\n");
        else {
            s->config.reg32 = value;
        }
        break;
    case STATUS_OFFSET: // STATUS is read-only.
        TRACE_WRITE(s->iomem.addr, offset, s->status.reg32, value);
        qemu_log_mask(LOG_GUEST_ERROR, "tegra.wdt: Ignoring write to read-only STATUS.\n");
        break;
    case COMMAND_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->command.reg32, value);
        s->command.reg32 = value;
        if (s->command.start_counter) { // enable counter
            s->status.enabled = 1;
            s->expiration_count = -1;
            s->status.interrupt_status = 0;
            s->status.fiq_status = 0;

            tegra_timer *timer = tegra_wdt_get_timer_source(s);
            if (timer) timer->pcr_read = false;
        }
        else if (s->command.disable_counter && s->unlock_pattern.unlock_pattern == 0xC45A) { // disable counter
            s->status.enabled = 0;
        }
        s->unlock_pattern.unlock_pattern = 0;
        break;
    case UNLOCK_PATTERN_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->unlock_pattern.reg32, value);
        s->unlock_pattern.reg32 = value;
        break;
    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }
}

static void tegra_wdt_priv_reset(DeviceState *dev)
{
    tegra_wdt *s = TEGRA_WDT(dev);

    s->config.reg32 = CONFIG_RESET;
    s->status.reg32 = STATUS_RESET;
    s->command.reg32 = COMMAND_RESET;
    s->unlock_pattern.reg32 = UNLOCK_PATTERN_RESET;
    s->expiration_count = -1;
}

static const MemoryRegionOps tegra_wdt_mem_ops = {
    .read = tegra_wdt_priv_read,
    .write = tegra_wdt_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_wdt_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_wdt *s = TEGRA_WDT(dev);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->fiq);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_wdt_mem_ops, s,
                          TYPE_TEGRA_WDT, TEGRA_WDT0_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_wdt_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_wdt_priv_realize;
    dc->vmsd = &vmstate_tegra_wdt;
    dc->reset = tegra_wdt_priv_reset;
}

static const TypeInfo tegra_wdt_info = {
    .name = TYPE_TEGRA_WDT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_wdt),
    .class_init = tegra_wdt_class_init,
};

static void tegra_wdt_register_types(void)
{
    type_register_static(&tegra_wdt_info);
}

type_init(tegra_wdt_register_types)
