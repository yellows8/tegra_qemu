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

#include "rtc.h"
#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_RTC "tegra.rtc"
#define TEGRA_RTC(obj) OBJECT_CHECK(tegra_rtc, (obj), TYPE_TEGRA_RTC)
#define DEFINE_REG32(reg) reg##_t reg

#define PTIMER_POLICY                       \
    (PTIMER_POLICY_WRAP_AFTER_ONE_PERIOD |  \
     PTIMER_POLICY_CONTINUOUS_TRIGGER    |  \
     PTIMER_POLICY_NO_IMMEDIATE_TRIGGER  |  \
     PTIMER_POLICY_NO_IMMEDIATE_RELOAD   |  \
     PTIMER_POLICY_NO_COUNTER_ROUND_DOWN)

typedef struct tegra_rtc_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    ptimer_state *p_rtc_sec;
    ptimer_state *p_rtc_ms;
    ptimer_state *p_sec_alarm0;
    ptimer_state *p_sec_alarm1;
    ptimer_state *p_ms_alarm;
    ptimer_state *p_sec_cnt_alarm;
    ptimer_state *p_ms_cnt_alarm;
    DEFINE_REG32(control);
    DEFINE_REG32(busy);
    DEFINE_REG32(seconds);
    DEFINE_REG32(shadow_seconds);
    DEFINE_REG32(milli_seconds);
    DEFINE_REG32(seconds_alarm0);
    DEFINE_REG32(seconds_alarm1);
    DEFINE_REG32(milli_seconds_alarm);
    DEFINE_REG32(seconds_countdown_alarm);
    DEFINE_REG32(milli_seconds_countdown_alarm);
    DEFINE_REG32(intr_mask);
    DEFINE_REG32(intr_status);
    DEFINE_REG32(intr_source);
    DEFINE_REG32(intr_set);
    DEFINE_REG32(correction_factor);
    qemu_irq irq;
} tegra_rtc;

static const VMStateDescription vmstate_tegra_rtc = {
    .name = "tegra.rtc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_PTIMER(p_rtc_sec, tegra_rtc),
        VMSTATE_PTIMER(p_rtc_ms, tegra_rtc),
        VMSTATE_PTIMER(p_sec_alarm0, tegra_rtc),
        VMSTATE_PTIMER(p_sec_alarm1, tegra_rtc),
        VMSTATE_PTIMER(p_ms_alarm, tegra_rtc),
        VMSTATE_PTIMER(p_sec_cnt_alarm, tegra_rtc),
        VMSTATE_PTIMER(p_ms_cnt_alarm, tegra_rtc),
        VMSTATE_UINT32(control.reg32, tegra_rtc),
        VMSTATE_UINT32(busy.reg32, tegra_rtc),
        VMSTATE_UINT32(seconds.reg32, tegra_rtc),
        VMSTATE_UINT32(shadow_seconds.reg32, tegra_rtc),
        VMSTATE_UINT32(milli_seconds.reg32, tegra_rtc),
        VMSTATE_UINT32(seconds_alarm0.reg32, tegra_rtc),
        VMSTATE_UINT32(seconds_alarm1.reg32, tegra_rtc),
        VMSTATE_UINT32(milli_seconds_alarm.reg32, tegra_rtc),
        VMSTATE_UINT32(seconds_countdown_alarm.reg32, tegra_rtc),
        VMSTATE_UINT32(milli_seconds_countdown_alarm.reg32, tegra_rtc),
        VMSTATE_UINT32(intr_mask.reg32, tegra_rtc),
        VMSTATE_UINT32(intr_status.reg32, tegra_rtc),
        VMSTATE_UINT32(intr_source.reg32, tegra_rtc),
        VMSTATE_UINT32(intr_set.reg32, tegra_rtc),
        VMSTATE_UINT32(correction_factor.reg32, tegra_rtc),
        VMSTATE_END_OF_LIST()
    }
};

static void tegra_rtc_sec_alarm0(void *opaque)
{
    tegra_rtc *s = opaque;

    s->intr_status.sec_alarm0 = 1;

    if (s->intr_mask.sec_alarm0)
        TRACE_IRQ_RAISE(s->iomem.addr, s->irq);
}

static void tegra_rtc_sec_alarm1(void *opaque)
{
    tegra_rtc *s = opaque;

    s->intr_status.sec_alarm1 = 1;

    if (s->intr_mask.sec_alarm1)
        TRACE_IRQ_RAISE(s->iomem.addr, s->irq);
}

static void tegra_rtc_ms_alarm(void *opaque)
{
    tegra_rtc *s = opaque;

    s->intr_status.msec_alarm = 1;

    if (s->intr_mask.msec_alarm)
        TRACE_IRQ_RAISE(s->iomem.addr, s->irq);
}

static void tegra_rtc_sec_cnt_alarm(void *opaque)
{
    tegra_rtc *s = opaque;

    s->intr_status.sec_cdn_alarm = 1;

    if (s->intr_mask.sec_cdn_alarm)
        TRACE_IRQ_RAISE(s->iomem.addr, s->irq);
}

static void tegra_rtc_ms_cnt_alarm(void *opaque)
{
    tegra_rtc *s = opaque;

    s->intr_status.msec_cdn_alarm = 1;

    if (s->intr_mask.msec_cdn_alarm)
        TRACE_IRQ_RAISE(s->iomem.addr, s->irq);
}

static void tegra_rtc_set_alarm(ptimer_state *clock, ptimer_state *alarm,
                                uint32_t time)
{
    ptimer_transaction_begin(alarm);
    ptimer_stop(alarm);

    if (time != 0) {
        uint32_t cnt = time - ~ptimer_get_count(clock);
        ptimer_set_limit(alarm, cnt, 1);
        ptimer_run(alarm, 0);
    }

    ptimer_transaction_commit(alarm);
}

static uint64_t tegra_rtc_priv_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    tegra_rtc *s = opaque;
    uint64_t ret = 0;

    switch (offset) {
    case CONTROL_OFFSET:
        ret = s->control.reg32;
        break;
    case BUSY_OFFSET:
        ret = s->busy.reg32;
        break;
    case SECONDS_OFFSET:
        s->seconds.reg32 = ~ptimer_get_count(s->p_rtc_sec);
        ret = s->seconds.reg32;
        break;
    case SHADOW_SECONDS_OFFSET:
        ret = s->shadow_seconds.reg32;
        break;
    case MILLI_SECONDS_OFFSET:
        s->shadow_seconds.reg32 = ~ptimer_get_count(s->p_rtc_sec);
        s->milli_seconds.milli_seconds = ~ptimer_get_count(s->p_rtc_ms);
        ret = s->milli_seconds.reg32;
        break;
    case SECONDS_ALARM0_OFFSET:
        ret = s->seconds_alarm0.reg32;
        break;
    case SECONDS_ALARM1_OFFSET:
        ret = s->seconds_alarm1.reg32;
        break;
    case MILLI_SECONDS_ALARM_OFFSET:
        ret = s->milli_seconds_alarm.reg32;
        break;
    case SECONDS_COUNTDOWN_ALARM_OFFSET:
        ret = s->seconds_countdown_alarm.reg32;
        break;
    case MILLI_SECONDS_COUNTDOWN_ALARM_OFFSET:
        ret = s->milli_seconds_countdown_alarm.reg32;
        break;
    case INTR_MASK_OFFSET:
        ret = s->intr_mask.reg32;
        break;
    case INTR_STATUS_OFFSET:
        ret = s->intr_status.reg32;
        break;
    case INTR_SOURCE_OFFSET:
        ret = s->intr_source.reg32 & s->intr_mask.reg32;
        break;
    case INTR_SET_OFFSET:
        break;
    case CORRECTION_FACTOR_OFFSET:
        ret = s->correction_factor.reg32;
        break;
    default:
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_rtc_priv_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    tegra_rtc *s = opaque;

    switch (offset) {
    case CONTROL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->control.reg32, value);

        if (!s->control.wr_sec_cnt)
            s->control.reg32 = value;
        break;
    case SECONDS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->seconds.reg32, value);
        s->seconds.reg32 = value;

        if (!s->control.wr_sec_cnt) {
            ptimer_transaction_begin(s->p_rtc_sec);
            ptimer_set_count(s->p_rtc_sec, ~value);
            ptimer_transaction_commit(s->p_rtc_sec);
        }
        break;
    case SECONDS_ALARM0_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->seconds_alarm0.reg32, value);
        s->seconds_alarm0.reg32 = value;

        tegra_rtc_set_alarm(s->p_rtc_sec, s->p_sec_alarm0, value);
        break;
    case SECONDS_ALARM1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->seconds_alarm1.reg32, value);
        s->seconds_alarm1.reg32 = value;

        tegra_rtc_set_alarm(s->p_rtc_sec, s->p_sec_alarm1, value);
        break;
    case MILLI_SECONDS_ALARM_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->milli_seconds_alarm.reg32, value);
        s->milli_seconds_alarm.reg32 = value;

        tegra_rtc_set_alarm(s->p_rtc_ms, s->p_ms_alarm,
                            s->milli_seconds_alarm.msec_match_value);
        break;
    case SECONDS_COUNTDOWN_ALARM_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->seconds_countdown_alarm.reg32, value);
        s->seconds_countdown_alarm.reg32 = value;

        ptimer_transaction_begin(s->p_sec_cnt_alarm);
        ptimer_stop(s->p_sec_cnt_alarm);

        if (!s->seconds_countdown_alarm.enable) {
            ptimer_transaction_commit(s->p_sec_cnt_alarm);
            break;
        }

        ptimer_set_limit(s->p_sec_cnt_alarm,
                         s->seconds_countdown_alarm.value, 1);
        ptimer_run(s->p_sec_cnt_alarm, !s->seconds_countdown_alarm.repeat);
        ptimer_transaction_commit(s->p_sec_cnt_alarm);
        break;
    case MILLI_SECONDS_COUNTDOWN_ALARM_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->milli_seconds_countdown_alarm.reg32, value);
        s->milli_seconds_countdown_alarm.reg32 = value;

        ptimer_transaction_begin(s->p_ms_cnt_alarm);
        ptimer_stop(s->p_ms_cnt_alarm);

        if (!s->milli_seconds_countdown_alarm.enable) {
            ptimer_transaction_commit(s->p_ms_cnt_alarm);
            break;
        }

        ptimer_set_limit(s->p_ms_cnt_alarm,
                         s->milli_seconds_countdown_alarm.value, 1);
        ptimer_run(s->p_ms_cnt_alarm, !s->milli_seconds_countdown_alarm.repeat);
        ptimer_transaction_commit(s->p_ms_cnt_alarm);
        break;
    case INTR_MASK_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->intr_mask.reg32, value);
        s->intr_mask.reg32 = value;
        break;
    case INTR_STATUS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->intr_status.reg32, value);
        s->intr_status.reg32 &= (~value);

        if (s->intr_mask.reg32 & s->intr_status.reg32 & 0x1f)
            break;
        TRACE_IRQ_LOWER(s->iomem.addr, s->irq);
        break;
    case INTR_SOURCE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->intr_source.reg32, value);
        s->intr_source.reg32 = value;
        break;
    case INTR_SET_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->intr_status.reg32,
                    s->intr_status.reg32 | value);
        s->intr_status.reg32 |= value;
        break;
    case CORRECTION_FACTOR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->correction_factor.reg32, value);
        s->correction_factor.reg32 = value;
        break;
    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }
}

static void tegra_rtc_priv_reset(DeviceState *dev)
{
    tegra_rtc *s = TEGRA_RTC(dev);

    ptimer_transaction_begin(s->p_rtc_sec);
    ptimer_run(s->p_rtc_sec, 0);
    ptimer_transaction_commit(s->p_rtc_sec);

    ptimer_transaction_begin(s->p_rtc_ms);
    ptimer_run(s->p_rtc_ms, 0);
    ptimer_transaction_commit(s->p_rtc_ms);

    s->control.reg32 = CONTROL_RESET;
    s->busy.reg32 = BUSY_RESET;
    s->seconds.reg32 = SECONDS_RESET;
    s->shadow_seconds.reg32 = SHADOW_SECONDS_RESET;
    s->milli_seconds.reg32 = MILLI_SECONDS_RESET;
    s->seconds_alarm0.reg32 = SECONDS_ALARM0_RESET;
    s->seconds_alarm1.reg32 = SECONDS_ALARM1_RESET;
    s->milli_seconds_alarm.reg32 = MILLI_SECONDS_ALARM_RESET;
    s->seconds_countdown_alarm.reg32 = SECONDS_COUNTDOWN_ALARM_RESET;
    s->milli_seconds_countdown_alarm.reg32 = MILLI_SECONDS_COUNTDOWN_ALARM_RESET;
    s->intr_mask.reg32 = INTR_MASK_RESET;
    s->intr_status.reg32 = INTR_STATUS_RESET;
    s->intr_source.reg32 = INTR_SOURCE_RESET;
    s->intr_set.reg32 = INTR_SET_RESET;
    s->correction_factor.reg32 = CORRECTION_FACTOR_RESET;
}

static const MemoryRegionOps tegra_rtc_mem_ops = {
    .read = tegra_rtc_priv_read,
    .write = tegra_rtc_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_rtc_tick(void *opaque)
{
    /* Nothing to do on timer rollover */
}

static void tegra_rtc_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_rtc *s = TEGRA_RTC(dev);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_rtc_mem_ops, s,
                          "tegra.rtc", TEGRA_RTC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    s->p_rtc_sec = ptimer_init(tegra_rtc_tick, s, PTIMER_POLICY);
    ptimer_transaction_begin(s->p_rtc_sec);
    ptimer_set_freq(s->p_rtc_sec, 1);
    ptimer_set_limit(s->p_rtc_sec, 0xffffffff, 1);
    ptimer_transaction_commit(s->p_rtc_sec);

    s->p_rtc_ms = ptimer_init(tegra_rtc_tick, s, PTIMER_POLICY);
    ptimer_transaction_begin(s->p_rtc_ms);
    ptimer_set_freq(s->p_rtc_ms, 1000);
    ptimer_set_limit(s->p_rtc_ms, 0x3ff, 1);
    ptimer_transaction_commit(s->p_rtc_ms);

    s->p_sec_alarm0 = ptimer_init(tegra_rtc_sec_alarm0, s, PTIMER_POLICY);
    ptimer_transaction_begin(s->p_sec_alarm0);
    ptimer_set_freq(s->p_sec_alarm0, 1);
    ptimer_transaction_commit(s->p_sec_alarm0);

    s->p_sec_alarm1 = ptimer_init(tegra_rtc_sec_alarm1, s, PTIMER_POLICY);
    ptimer_transaction_begin(s->p_sec_alarm1);
    ptimer_set_freq(s->p_sec_alarm1, 1);
    ptimer_transaction_commit(s->p_sec_alarm1);

    s->p_ms_alarm = ptimer_init(tegra_rtc_ms_alarm, s, PTIMER_POLICY);
    ptimer_transaction_begin(s->p_ms_alarm);
    ptimer_set_freq(s->p_ms_alarm, 1000);
    ptimer_transaction_commit(s->p_ms_alarm);

    s->p_sec_cnt_alarm = ptimer_init(tegra_rtc_sec_cnt_alarm, s, PTIMER_POLICY);
    ptimer_transaction_begin(s->p_sec_cnt_alarm);
    ptimer_set_freq(s->p_sec_cnt_alarm, 1);
    ptimer_transaction_commit(s->p_sec_cnt_alarm);

    s->p_ms_cnt_alarm = ptimer_init(tegra_rtc_ms_cnt_alarm, s, PTIMER_POLICY);
    ptimer_transaction_begin(s->p_ms_cnt_alarm);
    ptimer_set_freq(s->p_ms_cnt_alarm, 1000);
    ptimer_transaction_commit(s->p_ms_cnt_alarm);
}

static void tegra_rtc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_rtc_priv_realize;
    dc->vmsd = &vmstate_tegra_rtc;
    device_class_set_legacy_reset(dc, tegra_rtc_priv_reset);
}

static const TypeInfo tegra_rtc_info = {
    .name = TYPE_TEGRA_RTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_rtc),
    .class_init = tegra_rtc_class_init,
};

static void tegra_rtc_register_types(void)
{
    type_register_static(&tegra_rtc_info);
}

type_init(tegra_rtc_register_types)
