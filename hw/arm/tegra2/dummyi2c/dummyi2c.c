/*
 * Dummy i2c device
 *
 * Copyright (c) 2018 BALATON Zoltan
 * Copyright (c) yellows8
 *
 * This work is licensed under the GNU GPL license version 2 or later.
 *
 */

// Based on m41t80.c.

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/bcd.h"
#include "hw/i2c/i2c.h"
#include "qom/object.h"
#include "sysemu/rtc.h"
#include "migration/vmstate.h"

#include "dummyi2c.h"

#define TYPE_DUMMYI2C "dummyi2c"
OBJECT_DECLARE_SIMPLE_TYPE(DummyI2CState, DUMMYI2C)

struct DummyI2CState {
    I2CSlave parent_obj;
    int32_t addr;

    uint8_t regs[0x100];
};

static const VMStateDescription vmstate_dummyi2c = {
    .name = "dummyi2c",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, DummyI2CState),
        VMSTATE_INT32(addr, DummyI2CState),
        VMSTATE_UINT8_ARRAY(regs, DummyI2CState, 0x100),
        VMSTATE_END_OF_LIST()
    }
};

static void dummyi2c_realize(DeviceState *dev, Error **errp)
{
    DummyI2CState *s = DUMMYI2C(dev);

    s->addr = -1;

    memset(s->regs, 0, sizeof(s->regs));
}

void dummyi2c_set_regs(void *opaque, void* regs, size_t size)
{
    DummyI2CState *s = DUMMYI2C(opaque);

    if (size > sizeof(s->regs)) size = sizeof(s->regs);
    memcpy(s->regs, regs, size);
}

static int dummyi2c_send(I2CSlave *i2c, uint8_t data)
{
    DummyI2CState *s = DUMMYI2C(i2c);

    if (s->addr < 0) {
        s->addr = data;
    } else {
        int32_t addr = s->addr;
        s->addr = (s->addr + 1) % sizeof(s->regs);

        if (addr < sizeof(s->regs)) {
            s->regs[addr] = data;
        }
        else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid register: 0x%02X\n",
                          __func__, addr);
        }
    }
    return 0;
}

static uint8_t dummyi2c_recv(I2CSlave *i2c)
{
    DummyI2CState *s = DUMMYI2C(i2c);

    if (s->addr < 0) {
        s->addr = 0;
    }
    int32_t addr = s->addr;
    s->addr = (s->addr + 1) % sizeof(s->regs);

    if (addr < sizeof(s->regs)) return s->regs[addr];

    qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid register: 0x%02X\n",
                  __func__, addr);
    return 0;
}

static int dummyi2c_event(I2CSlave *i2c, enum i2c_event event)
{
    DummyI2CState *s = DUMMYI2C(i2c);

    if (event == I2C_START_SEND) {
        s->addr = -1;
    }
    return 0;
}

static void dummyi2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    dc->realize = dummyi2c_realize;
    dc->vmsd = &vmstate_dummyi2c;
    sc->send = dummyi2c_send;
    sc->recv = dummyi2c_recv;
    sc->event = dummyi2c_event;
}

static const TypeInfo dummyi2c_info = {
    .name          = TYPE_DUMMYI2C,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(DummyI2CState),
    .class_init    = dummyi2c_class_init,
};

static void dummyi2c_register_types(void)
{
    type_register_static(&dummyi2c_info);
}

type_init(dummyi2c_register_types)
