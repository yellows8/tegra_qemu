/*
 * MAX77XPMIC serial pmic emulation
 *
 * Copyright (c) 2018 BALATON Zoltan
 * Copyright (c) yellows8
 *
 * This work is licensed under the GNU GPL license version 2 or later.
 *
 */

// Based on m41t80.c.

// TODO: Wire an input GPIO to EN0 (power-button)?

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/bcd.h"
#include "hw/i2c/i2c.h"
#include "qom/object.h"
#include "migration/vmstate.h"
#include "sysemu/runstate.h"

#define TYPE_MAX77XPMIC "max77xpmic"
OBJECT_DECLARE_SIMPLE_TYPE(MAX77XPMICState, MAX77XPMIC)

struct MAX77XPMICState {
    I2CSlave parent_obj;
    int8_t addr;

    uint8_t regs[0x5B];
};

static const VMStateDescription vmstate_max77xpmic = {
    .name = "max77xpmic",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, MAX77XPMICState),
        VMSTATE_INT8(addr, MAX77XPMICState),
        VMSTATE_UINT8_ARRAY(regs, MAX77XPMICState, 0x5B),
        VMSTATE_END_OF_LIST()
    }
};

static void max77xpmic_realize(DeviceState *dev, Error **errp)
{
    MAX77XPMICState *s = MAX77XPMIC(dev);

    s->addr = -1;

    // Configure reset values.
    memset(s->regs, 0, sizeof(s->regs));

    s->regs[0x00] = 0x12; // CNFGGLBL1
    s->regs[0x01] = 0x03; // CNFGGLBL2
    s->regs[0x03] = 0x4C; // CNFG1_32K
    s->regs[0x04] = 0x41; // CNFGBBC
    s->regs[0x0C] = 0x40; // NVERC
    s->regs[0x0D] = 0x75; // IRQTOPM
    s->regs[0x0F] = 0xFF; // IRQMASKSD
    s->regs[0x10] = s->regs[0x11] = 0xFF; // IRQ_MSK_Lx
    s->regs[0x13] = 0x10; // STATLBT
    //s->regs[0x14] = 0xFF; // STATSD
    s->regs[0x1B] = 0x20; // VDVSSD0
    s->regs[0x1C] = 0x10; // VDVSSD1
    s->regs[0x1D] = s->regs[0x1E] = s->regs[0x1F] = s->regs[0x20] = 0x40; // CNFG1SDx
    s->regs[0x22] = 0x07; // CNFG2SD
    s->regs[0x23] = s->regs[0x25] = s->regs[0x27] = s->regs[0x29] = s->regs[0x2B] = s->regs[0x2D] = s->regs[0x2F] = s->regs[0x31] = s->regs[0x33] = 0x40; // CNFG1_Lx
    s->regs[0x24] = s->regs[0x26] = s->regs[0x28] = s->regs[0x2A] = s->regs[0x2C] = s->regs[0x2E] = s->regs[0x30] = s->regs[0x32] = s->regs[0x34] = 0x40; // CNFG2_Lx
    // CNFG1_GPIOx reset value should be non-zero in some cases but whatever.
    s->regs[0x42] = 0x07; // ONOFFCNFG2
}

static int max77xpmic_send(I2CSlave *i2c, uint8_t data)
{
    MAX77XPMICState *s = MAX77XPMIC(i2c);

    if (s->addr < 0) {
        s->addr = data;
    } else {
        int8_t addr = s->addr;
        s->addr = -1;

        if (addr < sizeof(s->regs)) {
            s->regs[addr] = data;

            if (addr == 0x41) { // ONOFFCNFG1
                if (data & BIT(7)) { // SFT_RST
                    qemu_log_mask(LOG_GUEST_ERROR, "%s: Requesting a system reset.\n",
                                  __func__);
                    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
                }
                else if (data & BIT(1)) { // PWR_OFF
                    qemu_log_mask(LOG_GUEST_ERROR, "%s: Requesting a system shutdown.\n",
                                  __func__);
                    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
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

static uint8_t max77xpmic_recv(I2CSlave *i2c)
{
    MAX77XPMICState *s = MAX77XPMIC(i2c);

    if (s->addr < 0) {
        s->addr = 0;
    }
    int8_t addr = s->addr;
    s->addr = (s->addr + 1) % sizeof(s->regs);

    if (addr < sizeof(s->regs)) {
        uint8_t tmp = s->regs[addr];
        if (addr == 0x0B || addr == 0x0C) s->regs[addr] = 0; // ONOFFIRQ, NVERC
        return tmp;
    }

    qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid register: 0x%02X\n",
                  __func__, addr);
    return 0;
}

static int max77xpmic_event(I2CSlave *i2c, enum i2c_event event)
{
    MAX77XPMICState *s = MAX77XPMIC(i2c);

    if (event == I2C_START_SEND) {
        s->addr = -1;
    }
    return 0;
}

static void max77xpmic_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    dc->realize = max77xpmic_realize;
    dc->vmsd = &vmstate_max77xpmic;
    sc->send = max77xpmic_send;
    sc->recv = max77xpmic_recv;
    sc->event = max77xpmic_event;
}

static const TypeInfo max77xpmic_info = {
    .name          = TYPE_MAX77XPMIC,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(MAX77XPMICState),
    .class_init    = max77xpmic_class_init,
};

static void max77xpmic_register_types(void)
{
    type_register_static(&max77xpmic_info);
}

type_init(max77xpmic_register_types)
