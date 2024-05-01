/*
 * Ftm3bd56 TouchPanel
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
#include "hw/i2c/i2c.h"
#include "qom/object.h"
#include "migration/vmstate.h"
#include "ui/console.h"

#define TYPE_FTM3BD56 "ftm3bd56"
OBJECT_DECLARE_SIMPLE_TYPE(FTM3BD56State, FTM3BD56)

// These defines are from hekate.

#define STMFTS_READ_INFO               0x80
#define STMFTS_READ_STATUS             0x84
#define STMFTS_READ_ONE_EVENT          0x85
#define STMFTS_READ_ALL_EVENT          0x86
#define STMFTS_LATEST_EVENT            0x87
#define STMFTS_SLEEP_IN                0x90
#define STMFTS_SLEEP_OUT               0x91
#define STMFTS_MS_MT_SENSE_OFF         0x92
#define STMFTS_MS_MT_SENSE_ON          0x93
#define STMFTS_SS_HOVER_SENSE_OFF      0x94
#define STMFTS_SS_HOVER_SENSE_ON       0x95
#define STMFTS_LP_TIMER_CALIB          0x97
#define STMFTS_MS_KEY_SENSE_OFF        0x9A
#define STMFTS_MS_KEY_SENSE_ON         0x9B
#define STMFTS_SYSTEM_RESET            0xA0
#define STMFTS_CLEAR_EVENT_STACK       0xA1
#define STMFTS_FULL_FORCE_CALIBRATION  0xA2
#define STMFTS_MS_CX_TUNING            0xA3
#define STMFTS_SS_CX_TUNING            0xA4
#define STMFTS_ITO_CHECK               0xA7
#define STMFTS_RELEASEINFO             0xAA
#define STMFTS_WRITE_REG               0xB6
#define STMFTS_SWITCH_SENSE_MODE       0xC3
#define STMFTS_NOISE_WRITE             0xC7
#define STMFTS_NOISE_READ              0xC8
#define STMFTS_RW_FRAMEBUFFER_REG      0xD0
#define STMFTS_SAVE_CX_TUNING          0xFC

#define STMFTS_DETECTION_CONFIG        0xB0
#define STMFTS_REQU_COMP_DATA          0xB8
#define STMFTS_VENDOR                  0xCF
#define STMFTS_FLASH_UNLOCK            0xF7
#define STMFTS_FLASH_WRITE_64K         0xF8
#define STMFTS_FLASH_STATUS            0xF9
#define STMFTS_FLASH_OP                0xFA
#define STMFTS_UNK5 0x62

/* cmd parameters */
#define STMFTS_VENDOR_GPIO_STATE       0x01
#define STMFTS_VENDOR_SENSE_MODE       0x02
#define STMFTS_STYLUS_MODE             0x00
#define STMFTS_FINGER_MODE             0x01
#define STMFTS_HOVER_MODE              0x02

/* events */
#define STMFTS_EV_NO_EVENT             0x00
#define STMFTS_EV_MULTI_TOUCH_DETECTED 0x02
#define STMFTS_EV_MULTI_TOUCH_ENTER    0x03
#define STMFTS_EV_MULTI_TOUCH_LEAVE    0x04
#define STMFTS_EV_MULTI_TOUCH_MOTION   0x05
#define STMFTS_EV_HOVER_ENTER          0x07
#define STMFTS_EV_HOVER_LEAVE          0x08
#define STMFTS_EV_HOVER_MOTION         0x09
#define STMFTS_EV_KEY_STATUS           0x0e
#define STMFTS_EV_ERROR                0x0f
#define STMFTS_EV_NOISE_READ           0x17
#define STMFTS_EV_NOISE_WRITE          0x18
#define STMFTS_EV_VENDOR               0x20

#define STMFTS_EV_CONTROLLER_READY     0x10
#define STMFTS_EV_STATUS               0x16
#define STMFTS_EV_DEBUG                0xDB

#define STMFTS_EV_STATUS_MS_CX_TUNING_DONE  1
#define STMFTS_EV_STATUS_SS_CX_TUNING_DONE  2
#define STMFTS_EV_STATUS_WRITE_CX_TUNE_DONE 4

/* multi touch related event masks */
#define STMFTS_MASK_EVENT_ID   0x0F
#define STMFTS_MASK_TOUCH_ID   0xF0
#define STMFTS_MASK_LEFT_EVENT 0x0F
#define STMFTS_MASK_X_MSB      0x0F
#define STMFTS_MASK_Y_LSB      0xF0

/* key related event masks */
#define STMFTS_MASK_KEY_NO_TOUCH 0x00
#define STMFTS_MASK_KEY_MENU     0x01
#define STMFTS_MASK_KEY_BACK     0x02

#define STMFTS_EVENT_SIZE     8
#define STMFTS_STACK_DEPTH   32
#define STMFTS_DATA_MAX_SIZE (STMFTS_EVENT_SIZE * STMFTS_STACK_DEPTH)
#define STMFTS_MAX_FINGERS   10

struct FTM3BD56State {
    I2CSlave parent_obj;
    QemuInputHandlerState *handler;
    int32_t addr;
    uint32_t num_args;
    uint32_t recv_pos;
    uint32_t events_read_pos;
    uint32_t events_write_pos;
    uint32_t events_num;

    uint32_t x, y;
    bool touched, last_touched;
    bool updated;

    uint8_t regs[0x10000];
    uint8_t cmd_args[0x8];
    uint8_t events[STMFTS_STACK_DEPTH][STMFTS_EVENT_SIZE];
};

static const VMStateDescription vmstate_ftm3bd56 = {
    .name = "ftm3bd56",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, FTM3BD56State),
        VMSTATE_INT32(addr, FTM3BD56State),
        VMSTATE_UINT32(num_args, FTM3BD56State),
        VMSTATE_UINT32(recv_pos, FTM3BD56State),
        VMSTATE_UINT32(events_read_pos, FTM3BD56State),
        VMSTATE_UINT32(events_write_pos, FTM3BD56State),
        VMSTATE_UINT32(events_num, FTM3BD56State),
        VMSTATE_UINT32(x, FTM3BD56State),
        VMSTATE_UINT32(y, FTM3BD56State),
        VMSTATE_BOOL(touched, FTM3BD56State),
        VMSTATE_BOOL(last_touched, FTM3BD56State),
        VMSTATE_BOOL(updated, FTM3BD56State),
        VMSTATE_UINT8_ARRAY(regs, FTM3BD56State, 0x10000),
        VMSTATE_UINT8_ARRAY(cmd_args, FTM3BD56State, 0x8),
        VMSTATE_UINT8_2DARRAY(events, FTM3BD56State, STMFTS_STACK_DEPTH, 0x8),
        VMSTATE_END_OF_LIST()
    }
};

static void ftm3bd56_write_event(FTM3BD56State *s, uint8_t event, uint8_t status, uint8_t *data)
{
    uint8_t tmpevent[STMFTS_EVENT_SIZE]={};

    if (s->events_num == STMFTS_STACK_DEPTH-1) return; // No more space.

    tmpevent[0] = event;
    tmpevent[1] = status;
    if (data) memcpy(&tmpevent[2], data, 6);

    memcpy(s->events[s->events_write_pos], tmpevent, sizeof(tmpevent));
    s->events_write_pos = (s->events_write_pos + 1) % STMFTS_STACK_DEPTH;
    s->events_num++;

    if (s->events_num == 1)
        s->events_read_pos = (s->events_read_pos + 1) % STMFTS_STACK_DEPTH;
}

static void ftm3bd56_input_event(DeviceState *dev, QemuConsole *src,
                                 InputEvent *evt)
{
    FTM3BD56State *s = (FTM3BD56State*)dev;
    InputMoveEvent *move;
    InputBtnEvent *btn;

    int width=0, height=0;
    uint32_t tmp=0;
    width = qemu_console_get_width(src, 1280);
    height = qemu_console_get_height(src, 720);

    switch (evt->type) {
    case INPUT_EVENT_KIND_ABS:
        move = evt->u.abs.data;
        if (move->axis == INPUT_AXIS_X) {
            tmp = qemu_input_scale_axis(move->value,
                                        INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX,
                                        0, width);
            if (s->x != tmp) {
                s->x = tmp;
                s->updated = true;
            }
        } else if (move->axis == INPUT_AXIS_Y) {
            tmp = qemu_input_scale_axis(move->value,
                                        INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX,
                                        0, height);
            if (s->y != tmp) {
                s->y = tmp;
                s->updated = true;
            }
        }
        break;

    case INPUT_EVENT_KIND_BTN:
        btn = evt->u.btn.data;
        if (btn->button == INPUT_BUTTON_LEFT) {
            if (s->touched != btn->down) {
                s->touched = btn->down;
                s->updated = true;
            }
        }
        break;

    default:
        break;
    }
}

static void ftm3bd56_input_sync(DeviceState *dev)
{
    FTM3BD56State *s = (FTM3BD56State*)dev;
    uint8_t data[6]={};
    uint8_t event=0;
    uint8_t status=0;

    if (s->updated) {
        s->updated = false;

        printf("ftm3bd56_input_sync: x = %d, y = %d, touched = %d, last_touched = %d\n", s->x, s->y, s->touched, s->last_touched);
        // TODO: fix rotation

        if (s->last_touched == false && s->touched == true)
            event = STMFTS_EV_MULTI_TOUCH_ENTER;
        else if (s->last_touched == true && s->touched == false)
            event = STMFTS_EV_MULTI_TOUCH_LEAVE;
        else if (s->last_touched == true && s->touched == true)
            event = STMFTS_EV_MULTI_TOUCH_MOTION;
        else
            event = STMFTS_EV_NO_EVENT;

        data[1] = (s->y & 0xF);
        data[0] = (s->y >> 4) & 0xFF;
        data[1] |= (s->x & 0xF) << 4;
        status = (s->x >> 4) & 0xFF;

        if (s->touched) {
            data[2] = 0x1;//0x0D;
            data[4] = 0x1;//0x81;
        }

        ftm3bd56_write_event(s, event, status, data);

        s->last_touched = s->touched;
    }
}

static const QemuInputHandler ftm3bd56_handler = {
    .name  = "QEMU ftm3bd56 TouchPanel",
    .mask  = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = ftm3bd56_input_event,
    .sync  = ftm3bd56_input_sync,
};

static void ftm3bd56_realize(DeviceState *dev, Error **errp)
{
    FTM3BD56State *s = FTM3BD56(dev);

    s->handler = qemu_input_handler_register((DeviceState *)s,
                                             &ftm3bd56_handler);
}

static void ftm3bd56_unrealize(DeviceState *dev)
{
    FTM3BD56State *s = FTM3BD56(dev);

    qemu_input_handler_unregister(s->handler);
}

static void ftm3bd56_reset(DeviceState *dev)
{
    FTM3BD56State *s = FTM3BD56(dev);

    s->addr = -1;
    s->num_args = 0;
    s->recv_pos = 0;

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->cmd_args, 0, sizeof(s->cmd_args));

    s->events_read_pos = STMFTS_STACK_DEPTH-1;
    s->events_write_pos = 0;
    s->events_num = 0;

    s->x = s->y = 0;
    s->touched = s->last_touched = false;
    s->updated = false;

    ftm3bd56_write_event(s, STMFTS_EV_CONTROLLER_READY, 0, NULL);

    s->regs[0x60] = 0x70; // Offset for fw-info.
    s->regs[0x61] = 0x0;

    s->regs[0x70+0] = 0x32; // fw_id
    s->regs[0x70+1] = 0x0;
    s->regs[0x70+2] = 0x0;
    s->regs[0x70+3] = 0x1;
}

static int ftm3bd56_send(I2CSlave *i2c, uint8_t data)
{
    FTM3BD56State *s = FTM3BD56(i2c);
    int32_t addr;

    if (s->addr < 0) {
        s->addr = data;
        s->num_args = 0;
        addr = s->addr;

        if (addr == STMFTS_MS_CX_TUNING) {
            ftm3bd56_write_event(s, STMFTS_EV_STATUS, STMFTS_EV_STATUS_MS_CX_TUNING_DONE, NULL);
        }
        else if (addr == STMFTS_SS_CX_TUNING) {
            ftm3bd56_write_event(s, STMFTS_EV_STATUS, STMFTS_EV_STATUS_SS_CX_TUNING_DONE, NULL);
        }
        else if (addr == STMFTS_SAVE_CX_TUNING) {
            ftm3bd56_write_event(s, STMFTS_EV_STATUS, STMFTS_EV_STATUS_WRITE_CX_TUNE_DONE, NULL);
        }
    } else {
        addr = s->addr;

        if (addr == STMFTS_RW_FRAMEBUFFER_REG && s->num_args >= 2) {
            uint32_t pos = 0;
            pos = (s->cmd_args[0]<<8) | (s->cmd_args[1]);
            pos += s->num_args - 2;

            if (pos < sizeof(s->regs)) {
                s->num_args++;
                s->regs[pos] = data;
                return 0;
            }

            qemu_log_mask(LOG_GUEST_ERROR, "%s: pos is too large for STMFTS_RW_FRAMEBUFFER_REG, ignoring.\n",
                      __func__);
            return 0;
        }

        if (s->num_args < sizeof(s->cmd_args)) {
            s->cmd_args[s->num_args++] = data;

            if (addr == STMFTS_ITO_CHECK && s->num_args == 2) {
                ftm3bd56_write_event(s, 0xF, 0x5, NULL);
            }
            else if (addr == STMFTS_WRITE_REG && s->num_args == 3) {
                if (s->cmd_args[0] == 0x0 && s->cmd_args[1] == 0x28 && s->cmd_args[2] == 0x80) { // System reset cmd
                    ftm3bd56_write_event(s, STMFTS_EV_CONTROLLER_READY, 0, NULL);
                }
            }
            else if (addr == STMFTS_VENDOR && s->num_args == 1) {
                if (s->cmd_args[0] == STMFTS_VENDOR_GPIO_STATE) {
                    // GPIOs for "NISSHA NFT-K12D":
                    uint8_t event_data[3] = {1, 1, 1};
                    ftm3bd56_write_event(s, STMFTS_EV_VENDOR, STMFTS_VENDOR_GPIO_STATE, event_data);
                }
            }
        }
        else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: too many args for cmd 0x%02X\n",
                          __func__, addr);
        }
    }
    return 0;
}

static uint8_t ftm3bd56_recv(I2CSlave *i2c)
{
    FTM3BD56State *s = FTM3BD56(i2c);

    if (s->addr < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: addr is not set, returning 0.\n",
                      __func__);
        return 0;
    }
    int32_t addr = s->addr;

    if (addr == STMFTS_READ_ONE_EVENT || addr == STMFTS_READ_ALL_EVENT || addr == STMFTS_LATEST_EVENT) {
        if ((s->recv_pos % STMFTS_EVENT_SIZE) + s->events_read_pos*STMFTS_EVENT_SIZE < sizeof(s->events)) {
            if (s->recv_pos % STMFTS_EVENT_SIZE == 7) {
                if (s->events_num) {
                    if (s->events_num > 1)
                        s->events_read_pos = (s->events_read_pos + 1) % STMFTS_STACK_DEPTH;
                    s->events_num--;
                }
            }
            return s->events[s->events_read_pos][s->recv_pos++ % STMFTS_EVENT_SIZE];
        }

        qemu_log_mask(LOG_GUEST_ERROR, "%s: recv_pos + events_read_pos is too large for STMFTS_READ_ONE_EVENT/STMFTS_READ_ALL_EVENT, returning 0.\n",
                      __func__);
        return 0;
    }
    else if (addr == STMFTS_RW_FRAMEBUFFER_REG && s->num_args >= 2) {
        if (s->recv_pos==0) {
            s->recv_pos++;
            return 0;
        }

        uint32_t pos = 0;
        pos = (s->cmd_args[0]<<8) | (s->cmd_args[1]);
        pos += s->recv_pos-1;

        if (pos < sizeof(s->regs)) {
            s->recv_pos++;
            return s->regs[pos];
        }

        qemu_log_mask(LOG_GUEST_ERROR, "%s: pos is too large for STMFTS_RW_FRAMEBUFFER_REG, returning 0.\n",
                      __func__);
        return 0;
    }
    else if (addr == STMFTS_WRITE_REG && s->num_args == 2) {
        if (s->cmd_args[0] == 0x0 && s->cmd_args[1] == 0x23) { // Total events
            return s->recv_pos++ != 1 ? 0 : s->events_num<<1;
        }
    }

    qemu_log_mask(LOG_GUEST_ERROR, "%s: unhandled addr, returning 0: 0x%02X\n",
                  __func__, addr);
    return 0;
}

static int ftm3bd56_event(I2CSlave *i2c, enum i2c_event event)
{
    FTM3BD56State *s = FTM3BD56(i2c);

    if (event == I2C_START_SEND) {
        s->addr = -1;
        s->num_args = 0;
    }
    else if (event == I2C_START_RECV) {
        s->recv_pos = 0;
    }
    return 0;
}

static void ftm3bd56_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    dc->reset = ftm3bd56_reset;
    dc->realize = ftm3bd56_realize;
    dc->unrealize = ftm3bd56_unrealize;
    dc->vmsd = &vmstate_ftm3bd56;
    sc->send = ftm3bd56_send;
    sc->recv = ftm3bd56_recv;
    sc->event = ftm3bd56_event;
}

static const TypeInfo ftm3bd56_info = {
    .name          = TYPE_FTM3BD56,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(FTM3BD56State),
    .class_init    = ftm3bd56_class_init,
};

static void ftm3bd56_register_types(void)
{
    type_register_static(&ftm3bd56_info);
}

type_init(ftm3bd56_register_types)
