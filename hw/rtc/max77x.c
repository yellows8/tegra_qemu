/*
 * MAX77X serial rtc emulation
 *
 * Copyright (c) 2018 BALATON Zoltan
 * Copyright (c) yellows8
 *
 * This work is licensed under the GNU GPL license version 2 or later.
 *
 */

// Based on m41t80.c.

// TODO: IRQs are not supported.
// TODO: Writes to the time/date regs are essentially ignored since the RBUDR handling loads the qemu_get_timedate data into those regs.

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/bcd.h"
#include "hw/i2c/i2c.h"
#include "qom/object.h"
#include "sysemu/rtc.h"
#include "migration/vmstate.h"

#define TYPE_MAX77X "max77x"
OBJECT_DECLARE_SIMPLE_TYPE(MAX77XState, MAX77X)

struct MAX77XState {
    I2CSlave parent_obj;
    int8_t addr;

    uint8_t regs[0x1C];
    uint8_t regs_cache[0x1C];
};

static const VMStateDescription vmstate_max77x = {
    .name = "max77x",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, MAX77XState),
        VMSTATE_INT8(addr, MAX77XState),
        VMSTATE_UINT8_ARRAY(regs, MAX77XState, 0x1C),
        VMSTATE_UINT8_ARRAY(regs_cache, MAX77XState, 0x1C),
        VMSTATE_END_OF_LIST()
    }
};

static void max77x_realize(DeviceState *dev, Error **errp)
{
    MAX77XState *s = MAX77X(dev);

    s->addr = -1;

    // Configure reset values.
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->regs_cache, 0, sizeof(s->regs_cache));

    s->regs[0x01] = 0x3F; // RTCINTM
    s->regs[0x02] = 0x03; // RTCCNTLM
    s->regs[0x04] = 0x0A; // RTCUPDATE0

    s->regs[0x09] = 0x01; // RTCHOUR
    s->regs[0x0A] = 0x01; // RTCDOW
    s->regs[0x0B] = 0x01; // RTCMONTH
    s->regs[0x0D] = 0x01; // RTCDOM

    s->regs[0x11] = s->regs[0x18] = 0x01; // RTCDOWAx
    s->regs[0x12] = s->regs[0x19] = 0x01; // RTCMONTHAx
    s->regs[0x14] = s->regs[0x1B] = 0x01; // RTCDOMAx
}

// Check if the reg is accessed directly (true), or double-buffered (false).
static bool max77x_is_raw_reg(int8_t addr)
{
    return addr == 0x00 || addr == 0x04 || addr == 0x05 || addr == 0x06; // RTCINT, RTCUPDATE0, RTCUPDATE1, RTCSMPL
}

static void max77x_update_rtc(I2CSlave *i2c)
{
    MAX77XState *s = MAX77X(i2c);
    struct tm now;

    if (s->regs[0x04] & BIT(2)) return; // RTCUPDATE0 FREEZE_SEC

    bool is_bcd = (s->regs[0x03] & BIT(0)) != 0; // RTCCNTL BCD
    bool hrmode = (s->regs[0x03] & BIT(1)) != 0; // RTCCNTL HRMODE (0 = 12h, 1 = 24h)

    qemu_get_timedate(&now, 0);

    int seconds = is_bcd ? to_bcd(now.tm_sec) : now.tm_sec;
    int minutes = is_bcd ? to_bcd(now.tm_min) : now.tm_min;

    int hours = now.tm_hour;
    bool pm_flag = !hrmode && now.tm_hour >= 12;
    if (pm_flag) hours-= 12;
    hours = is_bcd ? to_bcd(hours) : hours;

    int month = now.tm_mon + 1;
    month = is_bcd ? to_bcd(month) : month;
    int year = now.tm_year % 100;
    int mday = is_bcd ? to_bcd(now.tm_mday) : now.tm_mday;

    s->regs[0x07+0] = seconds & 0x7F;
    s->regs[0x07+1] = minutes & 0x7F;
    s->regs[0x07+2] = (hours & 0x3F) | (pm_flag<<6);
    s->regs[0x07+3] = BIT(now.tm_wday) & 0x7F;
    s->regs[0x07+4] = month & 0x1F;
    s->regs[0x07+5] = is_bcd ? to_bcd(year) : year;
    s->regs[0x07+6] = mday & 0x3F;
}

static int max77x_send(I2CSlave *i2c, uint8_t data)
{
    MAX77XState *s = MAX77X(i2c);

    if (s->addr < 0) {
        s->addr = data;
    } else {
        int8_t addr = s->addr;
        s->addr = (s->addr + 1) % sizeof(s->regs);

        if (addr < sizeof(s->regs_cache)) {
            if (addr == 0x00) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: ignoring write to read-only register: 0x%02X\n",
                              __func__, addr);
                return 0;
            }
            if (addr == 0x03) { // RTCCNTL
                data &= s->regs[0x02] & 0x3; // RTCCNTLM
            }
            s->regs_cache[addr] = data;
            if (max77x_is_raw_reg(addr)) s->regs[addr] = s->regs_cache[addr];
            if (addr == 0x04) { // RTCUPDATE0
                if (s->regs[addr] & BIT(0)) { // UDR
                    memcpy(s->regs, s->regs_cache, sizeof(s->regs_cache));
                    s->regs[addr] &= ~BIT(0);
                    s->regs[0x05] |= BIT(1); // RTCUPDATE1 RBUDF
                }
                if (s->regs[addr] & BIT(4)) { // RBUDR
                    max77x_update_rtc(i2c);
                    memcpy(s->regs_cache, s->regs, sizeof(s->regs));
                    s->regs[addr] &= ~BIT(4);
                    s->regs[0x05] |= BIT(0); // RTCUPDATE1 UDF
                }
            }
        }
        else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid register: 0x%02X\n",
                          __func__, addr);
        }
    }
    return 0;
}

static uint8_t max77x_recv(I2CSlave *i2c)
{
    MAX77XState *s = MAX77X(i2c);

    if (s->addr < 0) {
        s->addr = 0;
    }
    int8_t addr = s->addr;
    s->addr = (s->addr + 1) % sizeof(s->regs);

    if (max77x_is_raw_reg(addr)) {
        uint8_t tmp = s->regs[addr];
        if (addr == 0x00) s->regs[addr] = 0;
        if (addr == 0x05 && (s->regs[0x04] & BIT(1))) s->regs[addr] &= ~0x3; // For RTCUPDATE1, clear low 2-bits when RTCUPDATE0 FCUR is set.
        return tmp;
    }

    if (addr < sizeof(s->regs_cache)) return s->regs_cache[addr];

    qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid register: 0x%02X\n",
                  __func__, addr);
    return 0;
}

static int max77x_event(I2CSlave *i2c, enum i2c_event event)
{
    MAX77XState *s = MAX77X(i2c);

    if (event == I2C_START_SEND) {
        s->addr = -1;
    }
    return 0;
}

static void max77x_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    dc->realize = max77x_realize;
    dc->vmsd = &vmstate_max77x;
    sc->send = max77x_send;
    sc->recv = max77x_recv;
    sc->event = max77x_event;
}

static const TypeInfo max77x_info = {
    .name          = TYPE_MAX77X,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(MAX77XState),
    .class_init    = max77x_class_init,
};

static void max77x_register_types(void)
{
    type_register_static(&max77x_info);
}

type_init(max77x_register_types)
