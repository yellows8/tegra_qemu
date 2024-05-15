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
#include "qemu/main-loop.h"
#include "sysemu/runstate.h"

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

#define NUM_DMA_CHANNELS 22
#define DMA_CHANNEL_REGSIZE 0x80

#define TIMER_FREQ 19200000 // NOTE: Same as CCPLEX freq, perhaps this should be a prop?

#define SCALE   1

// The hardware uses 56-bits, but this is the largest bit-count which can be used with this freq without issues with ptimer.
#define TIMER_LIMIT (BIT(35)-1)

typedef struct tegra_ape_state {
    SysBusDevice parent_obj;

    qemu_irq irqs[2][NUM_MAILBOX];
    qemu_irq irqs_dma[NUM_DMA_CHANNELS];
    MemoryRegion iomem;
    ptimer_state *ptimer;
    QEMUBH *bh;

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

static uint32_t *tegra_ape_dma_get_chanregs(tegra_ape *s, size_t channel_id)
{
    return &s->regs[(0x22000 + channel_id*DMA_CHANNEL_REGSIZE)>>2];
}

static void tegra_ape_dma_update_irqs(tegra_ape *s)
{
    uint32_t value = 0;
    for (size_t channel_id=0; channel_id<NUM_DMA_CHANNELS; channel_id++) {
        uint32_t *chan_regs = tegra_ape_dma_get_chanregs(s, channel_id);
        bool flag = chan_regs[0x10>>2] != 0; // INT_STATUS_0

        qemu_set_irq(s->irqs_dma[channel_id], /*flag*/ false); // NOTE: Disabled since this causes issues when the guest ignores the IRQ. TRM doesn't seem to specify an IRQ-enable reg?
        if (flag) value |= BIT(channel_id);
    }

    s->regs[(0x22000+0xC28)>>2] = value; // ADMA_GLOBAL_CH_INT_STATUS_0
}

static bool tegra_ape_dma_process_channel(tegra_ape *s, size_t channel_id)
{
    uint32_t *chan_regs = tegra_ape_dma_get_chanregs(s, channel_id);
    bool flag = false;

    if (chan_regs[0x0>>2] & BIT(0)) { // CMD_0 TRANSFER_ENABLE
        flag = true;

        uint32_t ctrl = chan_regs[0x24>>2]; // CTRL_0
        uint32_t dir = (ctrl >> 12) & 0xF; // TRANSFER_DIRECTION
        uint32_t mode = (ctrl>>8) & 0x7; // TRANSFER_MODE
        bool flowctrl_enable = (ctrl & BIT(1)) != 0;
        uint32_t fifoctrl = chan_regs[0x2C>>2];

        bool transfer_done = true;

        // NOTE: Data-transfer would be here, but that needs MEMORY<>MEMORY transfers.

        if (mode != 2) { // non-CONTINUOUS
            flag = false;
            chan_regs[0x0>>2] &= ~BIT(0); // CMD_0 TRANSFER_ENABLE
            chan_regs[0xC>>2] &= ~BIT(0); // STATUS_0 TRANSFER_ENABLED
            chan_regs[0x30>>2] = 0; // TC_STATUS_0
        }
        else {
            if (!flowctrl_enable) chan_regs[0x30>>2] = 0; // TC_STATUS_0
            else {
                uint64_t transfer_fifosize = 0;

                if (dir==4 || dir==8) // MEMORY_TO_AHUB, AHUB_TO_AHUB
                    transfer_fifosize = 0x40 << ((fifoctrl>>8) & 0x1F); // TX_FIFO_SIZE in bytes
                else if (dir == 2) // AHUB_TO_MEMORY
                    transfer_fifosize = 0x40 << (fifoctrl & 0x1F); // RX_FIFO_SIZE in bytes

                if (transfer_fifosize) {
                    if (chan_regs[0x30>>2] >= transfer_fifosize) // TC_STATUS_0
                        chan_regs[0x30>>2] -= transfer_fifosize;
                    else if (chan_regs[0x30>>2]) // TC_STATUS_0
                        chan_regs[0x30>>2] = 0;
                    else
                        chan_regs[0x30>>2] = chan_regs[0x44>>2] - transfer_fifosize; // TC_STATUS_0 = TC_0 - transfer_fifosize

                    // NOTE: Actual data-transfer would need to update src-data-addr as well.

                    transfer_done = chan_regs[0x30>>2]==0;
                }
            }
        }

        if (transfer_done) {
            chan_regs[0x54>>2] = (chan_regs[0x54>>2] & 0xFFFF) + 1; // TRANSFER_STATUS_0 TRANSFER_DONE_COUNT
            chan_regs[0x10>>2] |= BIT(0); // INT_STATUS_0 TRANSFER_DONE
            tegra_ape_dma_update_irqs(s);
        }
    }

    return flag;
}

static int tegra_ape_dma_process_channels(tegra_ape *s)
{
    int count = 0;

    if (s->regs[(0x22000+0xC00)>>2] & BIT(0)) { // ADMA_GLOBAL_CMD_0 GLOBAL_ENABLE
        for (size_t i=0; i<NUM_DMA_CHANNELS; i++) {
            if (tegra_ape_dma_process_channel(s, i))
                count++;
        }
    }

    return count;
}

// DMA transfer has to be done from here in order to support CONTINUOUS mode.
static void tegra_ape_dma_run(void *opaque)
{
    tegra_ape *s = opaque;
    int count = 1;

    if (runstate_is_running())
        count = tegra_ape_dma_process_channels(s);

    if (count)
        qemu_bh_schedule_idle(s->bh);
}

static void tegra_ape_dma_enable_channel(tegra_ape *s, size_t channel_id)
{
    uint32_t *chan_regs = tegra_ape_dma_get_chanregs(s, channel_id);

    chan_regs[0xC>>2] |= BIT(0); // STATUS_0 TRANSFER_ENABLED
    chan_regs[0x30>>2] = chan_regs[0x44>>2]; // TC_STATUS_0 = TC_0
    qemu_bh_schedule_idle(s->bh);
}

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
        else if (offset >= 0x22000+0xC00 && offset+size <= 0x22000+0xC40) { // ADMA global regs
            if (offset == 0x22000+0xC10) { // ADMA_GLOBAL_STATUS_0
                ret &= ~BIT(0); // TRANSFER_ENABLED
                for (size_t channel_id=0; channel_id<NUM_DMA_CHANNELS; channel_id++) {
                    uint32_t *chan_regs = tegra_ape_dma_get_chanregs(s, channel_id);
                    if (chan_regs[0xC>>2] & BIT(0)) // STATUS_0 TRANSFER_ENABLED
                        ret |= BIT(0); // TRANSFER_ENABLED
                }
            }
            else if (offset == 0x22000+0xC2C) { // ADMA_GLOBAL_CH_ENABLE_STATUS_0
                ret = 0;
                for (size_t channel_id=0; channel_id<NUM_DMA_CHANNELS; channel_id++) {
                    uint32_t *chan_regs = tegra_ape_dma_get_chanregs(s, channel_id);
                    if (chan_regs[0xC>>2] & BIT(0)) // STATUS_0 TRANSFER_ENABLED
                        ret |= BIT(channel_id);
                }
            }
        }
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
        else if (offset >= 0x22000 && offset+size <= 0x24000) { // ADMA
            if (offset+size <= 0x22000 + NUM_DMA_CHANNELS*DMA_CHANNEL_REGSIZE) { // channel regs
                hwaddr chan_off = (offset - 0x22000) % DMA_CHANNEL_REGSIZE;
                size_t channel_id = (offset - 0x22000) / DMA_CHANNEL_REGSIZE;
                uint32_t *chan_regs = tegra_ape_dma_get_chanregs(s, channel_id);

                if (chan_off == 0x0) { // CMD_0
                    if (value & BIT(0)) { // TRANSFER_ENABLE
                        tegra_ape_dma_enable_channel(s, channel_id);
                    }
                }
                else if (chan_off == 0x4) { // SOFT_RESET_0
                    if (value & BIT(0)) { // ENABLE
                        s->regs[offset>>2] &= ~BIT(0);
                        memset(chan_regs, 0, DMA_CHANNEL_REGSIZE);
                    }
                }
                else if (chan_off == 0x18) { // INT_SET_0
                    if (value & BIT(0)) { // TRANSFER_DONE
                        chan_regs[0x10>>2] |= BIT(0); // INT_STATUS_0
                        tegra_ape_dma_update_irqs(s);
                    }
                }
                else if (chan_off == 0x1C) { // INT_CLEAR_0
                    if (value & BIT(0)) { // TRANSFER_DONE
                        chan_regs[0x10>>2] &= ~BIT(0); // INT_STATUS_0
                        tegra_ape_dma_update_irqs(s);
                    }
                }
            }
            else if (offset >= 0x22000+0xC00 && offset+size <= 0x22000+0xC40) { // ADMA global regs
                if (offset == 0x22000+0xC04) { // ADMA_GLOBAL_SOFT_RESET_0
                    if (value & BIT(0)) { // ENABLE
                        s->regs[offset>>2] &= ~BIT(0);
                    }
                }
            }
        }
    }
}

static void tegra_ape_start_timer(void *opaque)
{
    tegra_ape *s = opaque;

    ptimer_transaction_begin(s->ptimer);
    ptimer_stop(s->ptimer);
    ptimer_set_freq(s->ptimer, TIMER_FREQ * SCALE);
    ptimer_set_limit(s->ptimer, TIMER_LIMIT, 1);
    ptimer_run(s->ptimer, 0);
    ptimer_transaction_commit(s->ptimer);
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

    for (size_t channel_id=0; channel_id<NUM_DMA_CHANNELS; channel_id++) {
        uint32_t *chan_regs = tegra_ape_dma_get_chanregs(s, channel_id);

        chan_regs[0x24>>2] = 0x11004102; // CTRL_0
        chan_regs[0x28>>2] = 0x00500001; // CONFIG_0
        chan_regs[0x2C>>2] = channel_id < 4 ? 0x01010303 : 0x00000202; // AHUB_FIFO_CTRL_0
    }

    s->timer_count = 0;
    s->timer_word_flag = false;

    tegra_ape_start_timer(s);
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

    for (int i=0; i<NUM_DMA_CHANNELS; i++)
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irqs_dma[i]);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_ape_mem_ops, s,
                          TYPE_TEGRA_APE, (0x702F8000+0x1000)-TEGRA_APE_BASE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    s->ptimer = ptimer_init(tegra_ape_timer_tick, s, PTIMER_POLICY_CONTINUOUS_TRIGGER);
    s->bh = qemu_bh_new(tegra_ape_dma_run, s);
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
