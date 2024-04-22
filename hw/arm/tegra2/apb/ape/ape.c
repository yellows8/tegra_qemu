/*
 * ARM NVIDIA X1 emulation.
 *
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

// Based on tegra2 device code by digetx.

#include "tegra_common.h"

#include "hw/sysbus.h"
#include "hw/ptimer.h"

#include "qapi/error.h"
#include "qapi/qapi-commands.h"

#include "iomap.h"
#include "tegra_trace.h"
#include "devices.h"

#include "qemu/cutils.h"
#include "qemu/log.h"

#define TYPE_TEGRA_APE "tegra.ape"
#define TEGRA_APE(obj) OBJECT_CHECK(tegra_ape, (obj), TYPE_TEGRA_APE)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

#define NUM_MAILBOX 4
#define MAILBOX_INT_FULL 0
#define MAILBOX_INT_EMPTY 1

#define TIMER_FREQ 19200000 // NOTE: Same as CCPLEX freq, perhaps this should be a prop?

#define SCALE   1

#define TIMER_LIMIT (BIT(56)-1)

typedef struct tegra_ape_state {
    SysBusDevice parent_obj;

    qemu_irq irqs[2][NUM_MAILBOX];
    MemoryRegion iomem;
    ptimer_state *ptimer;

    uint64_t timer_count;
    bool timer_word_flag;
    bool mailbox_irq_raised[NUM_MAILBOX];

    uint32_t regs[((0x702F8000+0x1000)-TEGRA_APE_BASE)>>2];

} tegra_ape;

static const VMStateDescription vmstate_tegra_ape = {
    .name = TYPE_TEGRA_APE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_PTIMER(ptimer, tegra_ape),
        VMSTATE_UINT64(timer_count, tegra_ape),
        VMSTATE_BOOL(timer_word_flag, tegra_ape),
        VMSTATE_BOOL_ARRAY(mailbox_irq_raised, tegra_ape, NUM_MAILBOX),
        VMSTATE_UINT32_ARRAY(regs, tegra_ape, ((0x702F8000+0x1000)-TEGRA_APE_BASE)>>2),

        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_ape_priv_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    tegra_ape *s = opaque;
    uint64_t ret = 0;

    TRACE_READ(s->iomem.addr, offset, ret);

    if (offset+size <= sizeof(s->regs)) {
        ret = s->regs[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

        if (offset == 0x2C048) { // AMISC_TSC_0
            if (!s->timer_word_flag) s->timer_count = TIMER_LIMIT - ptimer_get_count(s->ptimer);
            ret = s->timer_count;
            if (!s->timer_word_flag) ret >>= 32;
            ret &= 0xFFFFFFFF;
            s->timer_word_flag = !s->timer_word_flag;
        }
        else if (offset == 0x2C04C) // AMISC_AMISC_DEBUG_0
            ret = s->timer_word_flag<<31;
    }

    return ret;
}

static void tegra_ape_priv_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    tegra_ape *s = opaque;

    TRACE_WRITE(s->iomem.addr, offset, 0, value);

    if (offset+size <= sizeof(s->regs)) {
        s->regs[offset/sizeof(uint32_t)] = (s->regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;

        if (offset >= 0x2C058 && offset+size <= 0x2C068) { // AMISC_SHRD_MBOX_0
            int mailbox = (offset-0x2C058)>>2;

            if (s->mailbox_irq_raised[mailbox]) { // Ack
                TRACE_IRQ_LOWER(s->iomem.addr, s->irqs[MAILBOX_INT_FULL][mailbox]);
                TRACE_IRQ_RAISE(s->iomem.addr, s->irqs[MAILBOX_INT_EMPTY][mailbox]);
                s->mailbox_irq_raised[mailbox] = false;
            }
            else if (value & BIT(31)) {
                TRACE_IRQ_LOWER(s->iomem.addr, s->irqs[MAILBOX_INT_EMPTY][mailbox]);
                TRACE_IRQ_RAISE(s->iomem.addr, s->irqs[MAILBOX_INT_FULL][mailbox]);
                s->mailbox_irq_raised[mailbox] = true;
            }
        }
    }
}

static void tegra_ape_priv_reset(DeviceState *dev)
{
    tegra_ape *s = TEGRA_APE(dev);

    memset(s->mailbox_irq_raised, 0, sizeof(s->mailbox_irq_raised));
    memset(s->regs, 0, sizeof(s->regs));

    s->regs[0x2C004/sizeof(uint32_t)] = 0xE0800000; // AMISC_ADSP_CONFIG_0
    s->regs[0x2C008/sizeof(uint32_t)] = 0x00C00000; // AMISC_ADSP_PERIP
    s->regs[0x2C00C/sizeof(uint32_t)] = 0x02000000; // AMISC_ADSP_L2_CONFIG_0
    s->regs[0x2C010/sizeof(uint32_t)] = 0x00C02000; // AMISC_ADSP_L2_REGFILEBASE_0

    s->timer_count = 0;
    s->timer_word_flag = false;

    ptimer_transaction_begin(s->ptimer);
    ptimer_run(s->ptimer, 0);
    ptimer_transaction_commit(s->ptimer);
}

static const MemoryRegionOps tegra_ape_mem_ops = {
    .read = tegra_ape_priv_read,
    .write = tegra_ape_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_ape_timer_tick(void *opaque)
{
    /* Nothing to do on timer rollover */
}

static void tegra_ape_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_ape *s = TEGRA_APE(dev);

    for (int i=0; i<2; i++) {
        for (int i2=0; i2<NUM_MAILBOX; i2++)
            sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irqs[i][i2]);
    }

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_ape_mem_ops, s,
                          TYPE_TEGRA_APE, (0x702F8000+0x1000)-TEGRA_APE_BASE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    s->ptimer = ptimer_init(tegra_ape_timer_tick, s, PTIMER_POLICY_CONTINUOUS_TRIGGER);
    ptimer_transaction_begin(s->ptimer);
    ptimer_set_freq(s->ptimer, TIMER_FREQ * SCALE);
    ptimer_set_limit(s->ptimer, TIMER_LIMIT, 1);
    ptimer_transaction_commit(s->ptimer);
}

static void tegra_ape_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_ape_priv_realize;
    dc->vmsd = &vmstate_tegra_ape;
    dc->reset = tegra_ape_priv_reset;
}

static const TypeInfo tegra_ape_info = {
    .name = TYPE_TEGRA_APE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_ape),
    .class_init = tegra_ape_class_init,
};

static void tegra_ape_register_types(void)
{
    type_register_static(&tegra_ape_info);
}

type_init(tegra_ape_register_types)
