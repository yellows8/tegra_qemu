/*
 * ARM NVIDIA Tegra2/X1 emulation.
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

#include "tegra_common.h"

#include "hw/sysbus.h"
#include "exec/address-spaces.h"
#include "sysemu/dma.h"
#include "qemu/cutils.h"
#include "qemu/log.h"

#include "apb_dma.h"
#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_APB_DMA "tegra.apb_dma"
#define TEGRA_APB_DMA(obj) OBJECT_CHECK(tegra_apb_dma, (obj), TYPE_TEGRA_APB_DMA)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

#define MAX_CHANNELS 32

typedef struct tegra_apb_dma_channel_state {
    qemu_irq irq;

    DEFINE_REG32(channel_csr);
    DEFINE_REG32(channel_sta);
    DEFINE_REG32(channel_dma_byte_sta);
    DEFINE_REG32(channel_csre);
    DEFINE_REG32(channel_ahb_ptr);
    DEFINE_REG32(channel_ahb_seq);
    DEFINE_REG32(channel_apb_ptr);
    DEFINE_REG32(channel_apb_seq);
    DEFINE_REG32(channel_wcount);
    DEFINE_REG32(channel_word_transfer);
} tegra_apb_dma_channel;

static const VMStateDescription vmstate_tegra_apb_dma_channel = {
    .name = "tegra.apb_dma_channel",
    .version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(channel_csr.reg32, tegra_apb_dma_channel),
        VMSTATE_UINT32(channel_sta.reg32, tegra_apb_dma_channel),
        VMSTATE_UINT32(channel_dma_byte_sta.reg32, tegra_apb_dma_channel),
        VMSTATE_UINT32(channel_csre.reg32, tegra_apb_dma_channel),
        VMSTATE_UINT32(channel_ahb_ptr.reg32, tegra_apb_dma_channel),
        VMSTATE_UINT32(channel_ahb_seq.reg32, tegra_apb_dma_channel),
        VMSTATE_UINT32(channel_apb_ptr.reg32, tegra_apb_dma_channel),
        VMSTATE_UINT32(channel_apb_seq.reg32, tegra_apb_dma_channel),
        VMSTATE_UINT32(channel_wcount.reg32, tegra_apb_dma_channel),
        VMSTATE_UINT32(channel_word_transfer.reg32, tegra_apb_dma_channel),
        VMSTATE_END_OF_LIST()
    }
};

typedef struct tegra_apb_dma_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    qemu_irq irqs[2];

    uint32_t channel_size;
    uint32_t num_channels;

    DEFINE_REG32(command);
    DEFINE_REG32(status);
    DEFINE_REG32(requestors_tx);
    DEFINE_REG32(requestors_rx);
    DEFINE_REG32(cntrl_reg);
    DEFINE_REG32(irq_sta_cpu);
    DEFINE_REG32(irq_sta_cop);
    DEFINE_REG32(irq_mask);
    DEFINE_REG32(irq_mask_set);
    DEFINE_REG32(irq_mask_clr);
    DEFINE_REG32(trig_reg);
    DEFINE_REG32(channel_trig_reg);
    DEFINE_REG32(dma_status);
    DEFINE_REG32(channel_en_reg);
    DEFINE_REG32(security_reg);
    DEFINE_REG32(channel_swid_0);
    DEFINE_REG32(chan_wt_reg0);
    DEFINE_REG32(chan_wt_reg1);
    DEFINE_REG32(chan_wt_reg2);
    DEFINE_REG32(chan_wt_reg3);
    DEFINE_REG32(channel_swid_1);
    tegra_apb_dma_channel channels[MAX_CHANNELS];
} tegra_apb_dma;

static const VMStateDescription vmstate_tegra_apb_dma = {
    .name = "tegra.apb_dma",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(channel_size, tegra_apb_dma),
        VMSTATE_UINT32(num_channels, tegra_apb_dma),
        VMSTATE_UINT32(command.reg32, tegra_apb_dma),
        VMSTATE_UINT32(status.reg32, tegra_apb_dma),
        VMSTATE_UINT32(requestors_tx.reg32, tegra_apb_dma),
        VMSTATE_UINT32(requestors_rx.reg32, tegra_apb_dma),
        VMSTATE_UINT32(cntrl_reg.reg32, tegra_apb_dma),
        VMSTATE_UINT32(irq_sta_cpu.reg32, tegra_apb_dma),
        VMSTATE_UINT32(irq_sta_cop.reg32, tegra_apb_dma),
        VMSTATE_UINT32(irq_mask.reg32, tegra_apb_dma),
        VMSTATE_UINT32(irq_mask_set.reg32, tegra_apb_dma),
        VMSTATE_UINT32(irq_mask_clr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(trig_reg.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_trig_reg.reg32, tegra_apb_dma),
        VMSTATE_UINT32(dma_status.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_en_reg.reg32, tegra_apb_dma),
        VMSTATE_UINT32(security_reg.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_swid_0.reg32, tegra_apb_dma),
        VMSTATE_UINT32(chan_wt_reg0.reg32, tegra_apb_dma),
        VMSTATE_UINT32(chan_wt_reg1.reg32, tegra_apb_dma),
        VMSTATE_UINT32(chan_wt_reg2.reg32, tegra_apb_dma),
        VMSTATE_UINT32(chan_wt_reg3.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_swid_1.reg32, tegra_apb_dma),
        VMSTATE_STRUCT_ARRAY(channels, tegra_apb_dma, MAX_CHANNELS, 0,
                             vmstate_tegra_apb_dma_channel, tegra_apb_dma_channel),
        VMSTATE_END_OF_LIST()
    }
};

static uint32_t tegra_apb_dma_get_mask(void *opaque)
{
    tegra_apb_dma *s = opaque;

    return s->irq_mask_set.reg32 & ~s->irq_mask_clr.reg32;
}

static void tegra_apb_dma_update_irq(void *opaque)
{
    tegra_apb_dma *s = opaque;
    uint32_t int_status=0, int_status_cop=0, int_status_cpu=0;

    for (uint32_t i=0; i<s->num_channels; i++) {
        tegra_apb_dma_channel *channel = &s->channels[i];
        bool flag = channel->channel_csr.ie_eoc && channel->channel_sta.ise_eoc; // ISE_EOC
        if (flag) {
            int_status |= BIT(i);
            if (channel->channel_ahb_seq.intr_enb)
                int_status_cpu |= BIT(i);
            else
                int_status_cop |= BIT(i);
        }
        qemu_set_irq(channel->irq, flag);
    }

    s->dma_status.reg32 = int_status;
    s->irq_sta_cop.reg32 = int_status_cop;
    s->irq_sta_cpu.reg32 = int_status_cpu;

    qemu_set_irq(s->irqs[0], (s->irq_sta_cop.reg32 & tegra_apb_dma_get_mask(s)) != 0);
    qemu_set_irq(s->irqs[1], (s->irq_sta_cpu.reg32 & tegra_apb_dma_get_mask(s)) != 0);
}

static void tegra_apb_dma_get_channel(void *opaque, hwaddr *offset, tegra_apb_dma_channel **channel, uint32_t *out_id)
{
    tegra_apb_dma *s = opaque;
    uint32_t id = 0;
    hwaddr off = *offset;

    *channel = NULL;

    if (off >= CHANNEL_CSR_OFFSET && off < CHANNEL_CSR_OFFSET + s->channel_size * s->num_channels) {
        id = (off - CHANNEL_CSR_OFFSET) / s->channel_size;
        *channel = &s->channels[id];
        off = ((off - CHANNEL_CSR_OFFSET) % s->channel_size) + CHANNEL_CSR_OFFSET;
        *offset = off;
    }

    if (out_id) *out_id = id;
}

static uint64_t tegra_apb_dma_priv_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    tegra_apb_dma *s = opaque;
    uint64_t ret = 0;
    tegra_apb_dma_channel *channel = NULL;

    tegra_apb_dma_get_channel(s, &offset, &channel, NULL);

    switch (offset) {
    case COMMAND_OFFSET:
        ret = s->command.reg32;
        break;
    case STATUS_OFFSET:
        ret = s->status.reg32;
        break;
    case REQUESTORS_TX_OFFSET:
        ret = s->requestors_tx.reg32;
        break;
    case REQUESTORS_RX_OFFSET:
        ret = s->requestors_rx.reg32;
        break;
    case CNTRL_REG_OFFSET:
        ret = s->cntrl_reg.reg32;
        break;
    case IRQ_STA_CPU_OFFSET:
        ret = s->irq_sta_cpu.reg32 & tegra_apb_dma_get_mask(s);
        break;
    case IRQ_STA_COP_OFFSET:
        ret = s->irq_sta_cop.reg32 & tegra_apb_dma_get_mask(s);
        break;
    case IRQ_MASK_OFFSET:
        ret = s->irq_mask.reg32 = tegra_apb_dma_get_mask(s);
        break;
    case TRIG_REG_OFFSET:
        ret = s->trig_reg.reg32;
        break;
    case CHANNEL_TRIG_REG_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->channel_trig_reg.reg32;
        break;
    case DMA_STATUS_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->dma_status.reg32;
        break;
    case CHANNEL_EN_REG_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->channel_en_reg.reg32;
        break;
    case SECURITY_REG_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->security_reg.reg32;
        break;
    case CHANNEL_SWID_0_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->channel_swid_0.reg32;
        break;
    case CHAN_WT_REG0_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->chan_wt_reg0.reg32;
        break;
    case CHAN_WT_REG1_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->chan_wt_reg1.reg32;
        break;
    case CHAN_WT_REG2_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->chan_wt_reg2.reg32;
        break;
    case CHAN_WT_REG3_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->chan_wt_reg3.reg32;
        break;
    case CHANNEL_SWID_1_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->channel_swid_1.reg32;
        break;
    case CHANNEL_CSR_OFFSET:
        ret = channel->channel_csr.reg32;
        break;
    case CHANNEL_STA_OFFSET:
        ret = channel->channel_sta.reg32;
        break;
    case CHANNEL_DMA_BYTE_STA_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = channel->channel_dma_byte_sta.reg32;
        break;
    case CHANNEL_CSRE_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = channel->channel_csre.reg32;
        break;
    case CHANNEL_AHB_PTR_OFFSET:
        ret = channel->channel_ahb_ptr.reg32;
        break;
    case CHANNEL_AHB_SEQ_OFFSET:
        ret = channel->channel_ahb_seq.reg32;
        break;
    case CHANNEL_APB_PTR_OFFSET:
        ret = channel->channel_apb_ptr.reg32;
        break;
    case CHANNEL_APB_SEQ_OFFSET:
        ret = channel->channel_apb_seq.reg32;
        break;
    case CHANNEL_WCOUNT_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = channel->channel_wcount.reg32;
        break;
    case CHANNEL_WORD_TRANSFER_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = channel->channel_word_transfer.reg32;
        break;
    default:
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_apb_dma_priv_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    tegra_apb_dma *s = opaque;
    tegra_apb_dma_channel *channel = NULL;
    uint32_t channel_id = 0;

    tegra_apb_dma_get_channel(s, &offset, &channel, &channel_id);

    switch (offset) {
    case COMMAND_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->command.reg32, value);
        s->command.reg32 = value;
        break;
    case CNTRL_REG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->cntrl_reg.reg32, value);
        s->cntrl_reg.reg32 = value;
        break;
    case IRQ_MASK_SET_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->irq_mask_set.reg32, value);
        s->irq_mask_set.reg32 = value;
        tegra_apb_dma_update_irq(s);
        break;
    case IRQ_MASK_CLR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->irq_mask_clr.reg32, value);
        s->irq_mask_clr.reg32 = value;
        tegra_apb_dma_update_irq(s);
        break;
    case CHANNEL_EN_REG_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) {
            TRACE_WRITE(s->iomem.addr, offset, s->channel_en_reg.reg32, value);
            s->channel_en_reg.reg32 = value;
        }
        break;
    case SECURITY_REG_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) {
            TRACE_WRITE(s->iomem.addr, offset, s->security_reg.reg32, value);
            s->security_reg.reg32 = value;
        }
        break;
    case CHANNEL_SWID_0_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) {
            TRACE_WRITE(s->iomem.addr, offset, s->channel_swid_0.reg32, value);
            s->channel_swid_0.reg32 = value;
        }
        break;
    case CHAN_WT_REG0_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) {
            TRACE_WRITE(s->iomem.addr, offset, s->chan_wt_reg0.reg32, value);
            s->chan_wt_reg0.reg32 = value;
        }
        break;
    case CHAN_WT_REG1_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) {
            TRACE_WRITE(s->iomem.addr, offset, s->chan_wt_reg1.reg32, value);
            s->chan_wt_reg1.reg32 = value;
        }
        break;
    case CHAN_WT_REG2_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) {
            TRACE_WRITE(s->iomem.addr, offset, s->chan_wt_reg2.reg32, value);
            s->chan_wt_reg2.reg32 = value;
        }
        break;
    case CHAN_WT_REG3_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) {
            TRACE_WRITE(s->iomem.addr, offset, s->chan_wt_reg3.reg32, value);
            s->chan_wt_reg3.reg32 = value;
        }
        break;
    case CHANNEL_SWID_1_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) {
            TRACE_WRITE(s->iomem.addr, offset, s->channel_swid_1.reg32, value);
            s->channel_swid_1.reg32 = value;
        }
        break;
    case CHANNEL_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, channel->channel_csr.reg32, value);
        channel->channel_csr.reg32 = value;

        if (channel->channel_csr.enb) { // Enable DMA transfer
            size_t bus_size = 1 << channel->channel_apb_seq.apb_bus_width; // bus width in bytes
            uint32_t wcount = (channel->channel_wcount.reg32>>2)+1;
            uint32_t apb_ptr_reg = channel->channel_apb_ptr.reg32 & ~0x3;
            uint32_t ahb_ptr_reg = channel->channel_ahb_ptr.ahb_base << 2;
            dma_addr_t tmplen = wcount<<2;
            uint8_t *databuf_ahb = NULL;

            DMADirection dmadir = channel->channel_csr.dir ? DMA_DIRECTION_FROM_DEVICE : DMA_DIRECTION_TO_DEVICE;
            databuf_ahb = dma_memory_map(&address_space_memory, ahb_ptr_reg, &tmplen,
                                         dmadir, MEMTXATTRS_UNSPECIFIED);

            channel->channel_word_transfer.reg32 = 0;
            channel->channel_dma_byte_sta.reg32 = 0;

            if (databuf_ahb==NULL)
                qemu_log_mask(LOG_GUEST_ERROR, "tegra.apb_dma: Failed to DMA map AHB buffer.\n");
            else if (!(apb_ptr_reg >= IO_APB_PHYS && apb_ptr_reg < IO_APB_PHYS+IO_APB_SIZE))
                qemu_log_mask(LOG_GUEST_ERROR, "tegra.apb_dma: Invalid APB addr: 0x%X.\n", apb_ptr_reg);
            else {
                size_t ahb_addr_wrap = channel->channel_ahb_seq.wrap;
                size_t apb_addr_wrap = channel->channel_apb_seq.apb_addr_wrap;
                if (apb_addr_wrap) apb_addr_wrap = 1<<(apb_addr_wrap-1);
                size_t chunk_size = apb_addr_wrap ? apb_addr_wrap : wcount;
                MemTxResult memres = MEMTX_OK;

                if (ahb_addr_wrap)
                    qemu_log_mask(LOG_GUEST_ERROR, "tegra.apb_dma: Ignoring ahb_addr_wrap = 0x%zX.\n", ahb_addr_wrap);

                if (channel->channel_ahb_seq.ahb_data_swap || channel->channel_apb_seq.apb_data_swap)
                    qemu_log_mask(LOG_GUEST_ERROR, "tegra.apb_dma: Ignoring ahb_data_swap = %d / apb_data_swap = %d.\n",
                                  channel->channel_ahb_seq.ahb_data_swap, channel->channel_apb_seq.apb_data_swap);

                for (size_t i=0; i<wcount; i+=chunk_size) {
                    if (i+chunk_size > wcount) chunk_size = wcount-i;
                    for (size_t chunki=0; chunki<chunk_size; chunki++) {
                        for (size_t busi=0; busi<4; busi+=bus_size) {
                            memres = address_space_rw(&address_space_memory, apb_ptr_reg,
                                                      MEMTXATTRS_UNSPECIFIED, &databuf_ahb[((i + chunki)<<2) + busi],
                                                      bus_size, channel->channel_csr.dir);
                            if (memres!=MEMTX_OK)
                                break;

                            //qemu_hexdump(stdout, "tegra_apb_dma_data", &databuf_ahb[((i + chunki)<<2) + busi], bus_size);
                        }
                    }

                    if (memres!=MEMTX_OK)
                        break;
                }

                if (memres!=MEMTX_OK)
                    qemu_log_mask(LOG_GUEST_ERROR, "tegra.apb_dma: address_space_rw(0x%X 0x%X 0x%X): %d\n", ahb_ptr_reg, apb_ptr_reg, wcount<<2, memres);
                else {
                    channel->channel_dma_byte_sta.reg32 = wcount<<2;
                    channel->channel_word_transfer.reg32 = (wcount-1)<<2;
                }
            }

            if (databuf_ahb) dma_memory_unmap(&address_space_memory, databuf_ahb, tmplen, dmadir, channel->channel_dma_byte_sta.reg32);

            channel->channel_csr.enb = 0;

            if (channel->channel_csr.ie_eoc)
                channel->channel_sta.ise_eoc = 1;

            if (tegra_board >= TEGRAX1_BOARD)
                s->status.reg32 &= ~BIT(channel_id);
        }

        tegra_apb_dma_update_irq(s);
        break;
    case CHANNEL_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, channel->channel_sta.reg32, value & CHANNEL_STA_WRMASK);
        if (tegra_board < TEGRAX1_BOARD)
            WR_MASKED(channel->channel_sta.reg32, value, CHANNEL_STA);
        else if (value & BIT(30)) { // ISE_EOC
            channel->channel_sta.reg32 &= ~BIT(30);
            tegra_apb_dma_update_irq(s);
        }
        break;
    case CHANNEL_CSRE_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) {
            TRACE_WRITE(s->iomem.addr, offset, channel->channel_csre.reg32, value);
            channel->channel_csre.reg32 = value;
        }
        break;
    case CHANNEL_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, channel->channel_ahb_ptr.reg32, value);
        channel->channel_ahb_ptr.reg32 = value;
        break;
    case CHANNEL_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, channel->channel_ahb_seq.reg32, value);
        channel->channel_ahb_seq.reg32 = value;
        break;
    case CHANNEL_APB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, channel->channel_apb_ptr.reg32, value);
        channel->channel_apb_ptr.reg32 = value;
        break;
    case CHANNEL_APB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, channel->channel_apb_seq.reg32, value);
        channel->channel_apb_seq.reg32 = value;
        break;
    case CHANNEL_WCOUNT_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) {
            TRACE_WRITE(s->iomem.addr, offset, channel->channel_wcount.reg32, value);
            channel->channel_wcount.reg32 = value;
        }
        break;
    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }
}

static void tegra_apb_dma_priv_reset(DeviceState *dev)
{
    tegra_apb_dma *s = TEGRA_APB_DMA(dev);

    s->command.reg32 = COMMAND_RESET;
    s->status.reg32 = STATUS_RESET;
    s->requestors_tx.reg32 = REQUESTORS_TX_RESET;
    s->requestors_rx.reg32 = REQUESTORS_RX_RESET;
    s->cntrl_reg.reg32 = CNTRL_REG_RESET;
    s->irq_sta_cpu.reg32 = IRQ_STA_CPU_RESET;
    s->irq_sta_cop.reg32 = IRQ_STA_COP_RESET;
    s->irq_mask.reg32 = IRQ_MASK_RESET;
    s->irq_mask_set.reg32 = IRQ_MASK_SET_RESET;
    s->irq_mask_clr.reg32 = IRQ_MASK_CLR_RESET;
    s->trig_reg.reg32 = TRIG_REG_RESET;

    s->channel_trig_reg.reg32 = CHANNEL_TRIG_REG_RESET;
    s->dma_status.reg32 = DMA_STATUS_RESET;
    s->channel_en_reg.reg32 = CHANNEL_EN_REG_RESET;
    s->security_reg.reg32 = SECURITY_REG_RESET;
    s->channel_swid_0.reg32 = CHANNEL_SWID_0_RESET;
    s->chan_wt_reg0.reg32 = CHAN_WT_REG0_RESET;
    s->chan_wt_reg1.reg32 = CHAN_WT_REG1_RESET;
    s->chan_wt_reg2.reg32 = CHAN_WT_REG2_RESET;
    s->chan_wt_reg3.reg32 = CHAN_WT_REG3_RESET;
    s->channel_swid_1.reg32 = CHANNEL_SWID_1_RESET;

    for (size_t i = 0; i < MAX_CHANNELS; i++) {
        tegra_apb_dma_channel *channel = &s->channels[i];
        channel->channel_csr.reg32 = CHANNEL_CSR_RESET;
        channel->channel_sta.reg32 = CHANNEL_STA_RESET;
        channel->channel_dma_byte_sta.reg32 = CHANNEL_DMA_BYTE_STA_RESET;
        channel->channel_csre.reg32 = CHANNEL_CSRE_RESET;
        channel->channel_ahb_ptr.reg32 = CHANNEL_AHB_PTR_RESET;
        channel->channel_ahb_seq.reg32 = CHANNEL_AHB_SEQ_RESET;
        channel->channel_apb_ptr.reg32 = CHANNEL_APB_PTR_RESET;
        channel->channel_apb_seq.reg32 = CHANNEL_APB_SEQ_RESET;
        channel->channel_wcount.reg32 = CHANNEL_WCOUNT_RESET;
        channel->channel_word_transfer.reg32 = CHANNEL_WORD_TRANSFER_RESET;
    }
}

static const MemoryRegionOps tegra_apb_dma_mem_ops = {
    .read = tegra_apb_dma_priv_read,
    .write = tegra_apb_dma_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_apb_dma_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_apb_dma *s = TEGRA_APB_DMA(dev);

    uint32_t channel_size = 0, num_channels = 0;

    if (tegra_board >= TEGRAX1_BOARD) {
        channel_size = 0x40;
        num_channels = 32;
    }
    else {
        channel_size = 0x20;
        num_channels = 16;
    }

    s->channel_size = channel_size;
    s->num_channels = num_channels;

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_apb_dma_mem_ops, s,
                          "tegra.apb_dma", TEGRA_APB_DMA_SIZE + (num_channels * channel_size));
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    for (uint32_t i=0; i<ARRAY_SIZE(s->irqs); i++)
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irqs[i]);

    for (uint32_t i=0; i<MAX_CHANNELS; i++)
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->channels[i].irq);
}

static void tegra_apb_dma_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_apb_dma_priv_realize;
    dc->vmsd = &vmstate_tegra_apb_dma;
    dc->reset = tegra_apb_dma_priv_reset;
}

static const TypeInfo tegra_apb_dma_info = {
    .name = TYPE_TEGRA_APB_DMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_apb_dma),
    .class_init = tegra_apb_dma_class_init,
};

static void tegra_apb_dma_register_types(void)
{
    type_register_static(&tegra_apb_dma_info);
}

type_init(tegra_apb_dma_register_types)
