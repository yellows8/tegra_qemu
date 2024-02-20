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

#include "hw/sysbus.h"

#include "apb_dma.h"
#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_APB_DMA "tegra.apb_dma"
#define TEGRA_APB_DMA(obj) OBJECT_CHECK(tegra_apb_dma, (obj), TYPE_TEGRA_APB_DMA)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_apb_dma_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
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
    DEFINE_REG32(channel_0_csr);
    DEFINE_REG32(channel_0_sta);
    DEFINE_REG32(channel_0_ahb_ptr);
    DEFINE_REG32(channel_0_ahb_seq);
    DEFINE_REG32(channel_0_apb_ptr);
    DEFINE_REG32(channel_0_apb_seq);
    DEFINE_REG32(channel_1_csr);
    DEFINE_REG32(channel_1_sta);
    DEFINE_REG32(channel_1_ahb_ptr);
    DEFINE_REG32(channel_1_ahb_seq);
    DEFINE_REG32(channel_1_apb_ptr);
    DEFINE_REG32(channel_1_apb_seq);
    DEFINE_REG32(channel_2_csr);
    DEFINE_REG32(channel_2_sta);
    DEFINE_REG32(channel_2_ahb_ptr);
    DEFINE_REG32(channel_2_ahb_seq);
    DEFINE_REG32(channel_2_apb_ptr);
    DEFINE_REG32(channel_2_apb_seq);
    DEFINE_REG32(channel_3_csr);
    DEFINE_REG32(channel_3_sta);
    DEFINE_REG32(channel_3_ahb_ptr);
    DEFINE_REG32(channel_3_ahb_seq);
    DEFINE_REG32(channel_3_apb_ptr);
    DEFINE_REG32(channel_3_apb_seq);
    DEFINE_REG32(channel_4_csr);
    DEFINE_REG32(channel_4_sta);
    DEFINE_REG32(channel_4_ahb_ptr);
    DEFINE_REG32(channel_4_ahb_seq);
    DEFINE_REG32(channel_4_apb_ptr);
    DEFINE_REG32(channel_4_apb_seq);
    DEFINE_REG32(channel_5_csr);
    DEFINE_REG32(channel_5_sta);
    DEFINE_REG32(channel_5_ahb_ptr);
    DEFINE_REG32(channel_5_ahb_seq);
    DEFINE_REG32(channel_5_apb_ptr);
    DEFINE_REG32(channel_5_apb_seq);
    DEFINE_REG32(channel_6_csr);
    DEFINE_REG32(channel_6_sta);
    DEFINE_REG32(channel_6_ahb_ptr);
    DEFINE_REG32(channel_6_ahb_seq);
    DEFINE_REG32(channel_6_apb_ptr);
    DEFINE_REG32(channel_6_apb_seq);
    DEFINE_REG32(channel_7_csr);
    DEFINE_REG32(channel_7_sta);
    DEFINE_REG32(channel_7_ahb_ptr);
    DEFINE_REG32(channel_7_ahb_seq);
    DEFINE_REG32(channel_7_apb_ptr);
    DEFINE_REG32(channel_7_apb_seq);
    DEFINE_REG32(channel_8_csr);
    DEFINE_REG32(channel_8_sta);
    DEFINE_REG32(channel_8_ahb_ptr);
    DEFINE_REG32(channel_8_ahb_seq);
    DEFINE_REG32(channel_8_apb_ptr);
    DEFINE_REG32(channel_8_apb_seq);
    DEFINE_REG32(channel_9_csr);
    DEFINE_REG32(channel_9_sta);
    DEFINE_REG32(channel_9_ahb_ptr);
    DEFINE_REG32(channel_9_ahb_seq);
    DEFINE_REG32(channel_9_apb_ptr);
    DEFINE_REG32(channel_9_apb_seq);
    DEFINE_REG32(channel_10_csr);
    DEFINE_REG32(channel_10_sta);
    DEFINE_REG32(channel_10_ahb_ptr);
    DEFINE_REG32(channel_10_ahb_seq);
    DEFINE_REG32(channel_10_apb_ptr);
    DEFINE_REG32(channel_10_apb_seq);
    DEFINE_REG32(channel_11_csr);
    DEFINE_REG32(channel_11_sta);
    DEFINE_REG32(channel_11_ahb_ptr);
    DEFINE_REG32(channel_11_ahb_seq);
    DEFINE_REG32(channel_11_apb_ptr);
    DEFINE_REG32(channel_11_apb_seq);
    DEFINE_REG32(channel_12_csr);
    DEFINE_REG32(channel_12_sta);
    DEFINE_REG32(channel_12_ahb_ptr);
    DEFINE_REG32(channel_12_ahb_seq);
    DEFINE_REG32(channel_12_apb_ptr);
    DEFINE_REG32(channel_12_apb_seq);
    DEFINE_REG32(channel_13_csr);
    DEFINE_REG32(channel_13_sta);
    DEFINE_REG32(channel_13_ahb_ptr);
    DEFINE_REG32(channel_13_ahb_seq);
    DEFINE_REG32(channel_13_apb_ptr);
    DEFINE_REG32(channel_13_apb_seq);
    DEFINE_REG32(channel_14_csr);
    DEFINE_REG32(channel_14_sta);
    DEFINE_REG32(channel_14_ahb_ptr);
    DEFINE_REG32(channel_14_ahb_seq);
    DEFINE_REG32(channel_14_apb_ptr);
    DEFINE_REG32(channel_14_apb_seq);
    DEFINE_REG32(channel_15_csr);
    DEFINE_REG32(channel_15_sta);
    DEFINE_REG32(channel_15_ahb_ptr);
    DEFINE_REG32(channel_15_ahb_seq);
    DEFINE_REG32(channel_15_apb_ptr);
    DEFINE_REG32(channel_15_apb_seq);
} tegra_apb_dma;

static const VMStateDescription vmstate_tegra_apb_dma = {
    .name = "tegra.apb_dma",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
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
        VMSTATE_UINT32(channel_0_csr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_0_sta.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_0_ahb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_0_ahb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_0_apb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_0_apb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_1_csr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_1_sta.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_1_ahb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_1_ahb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_1_apb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_1_apb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_2_csr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_2_sta.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_2_ahb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_2_ahb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_2_apb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_2_apb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_3_csr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_3_sta.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_3_ahb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_3_ahb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_3_apb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_3_apb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_4_csr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_4_sta.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_4_ahb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_4_ahb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_4_apb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_4_apb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_5_csr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_5_sta.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_5_ahb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_5_ahb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_5_apb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_5_apb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_6_csr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_6_sta.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_6_ahb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_6_ahb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_6_apb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_6_apb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_7_csr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_7_sta.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_7_ahb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_7_ahb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_7_apb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_7_apb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_8_csr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_8_sta.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_8_ahb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_8_ahb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_8_apb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_8_apb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_9_csr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_9_sta.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_9_ahb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_9_ahb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_9_apb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_9_apb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_10_csr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_10_sta.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_10_ahb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_10_ahb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_10_apb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_10_apb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_11_csr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_11_sta.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_11_ahb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_11_ahb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_11_apb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_11_apb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_12_csr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_12_sta.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_12_ahb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_12_ahb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_12_apb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_12_apb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_13_csr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_13_sta.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_13_ahb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_13_ahb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_13_apb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_13_apb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_14_csr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_14_sta.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_14_ahb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_14_ahb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_14_apb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_14_apb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_15_csr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_15_sta.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_15_ahb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_15_ahb_seq.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_15_apb_ptr.reg32, tegra_apb_dma),
        VMSTATE_UINT32(channel_15_apb_seq.reg32, tegra_apb_dma),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_apb_dma_priv_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    tegra_apb_dma *s = opaque;
    uint64_t ret = 0;

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
        ret = s->irq_sta_cpu.reg32;
        break;
    case IRQ_STA_COP_OFFSET:
        ret = s->irq_sta_cop.reg32;
        break;
    case IRQ_MASK_OFFSET:
        ret = s->irq_mask.reg32;
        break;
    case IRQ_MASK_SET_OFFSET:
        ret = s->irq_mask_set.reg32;
        break;
    case IRQ_MASK_CLR_OFFSET:
        ret = s->irq_mask_clr.reg32;
        break;
    case TRIG_REG_OFFSET:
        ret = s->trig_reg.reg32;
        break;
    case CHANNEL_0_CSR_OFFSET:
        ret = s->channel_0_csr.reg32;
        break;
    case CHANNEL_0_STA_OFFSET:
        ret = s->channel_0_sta.reg32;
        break;
    case CHANNEL_0_AHB_PTR_OFFSET:
        ret = s->channel_0_ahb_ptr.reg32;
        break;
    case CHANNEL_0_AHB_SEQ_OFFSET:
        ret = s->channel_0_ahb_seq.reg32;
        break;
    case CHANNEL_0_APB_PTR_OFFSET:
        ret = s->channel_0_apb_ptr.reg32;
        break;
    case CHANNEL_0_APB_SEQ_OFFSET:
        ret = s->channel_0_apb_seq.reg32;
        break;
    case CHANNEL_1_CSR_OFFSET:
        ret = s->channel_1_csr.reg32;
        break;
    case CHANNEL_1_STA_OFFSET:
        ret = s->channel_1_sta.reg32;
        break;
    case CHANNEL_1_AHB_PTR_OFFSET:
        ret = s->channel_1_ahb_ptr.reg32;
        break;
    case CHANNEL_1_AHB_SEQ_OFFSET:
        ret = s->channel_1_ahb_seq.reg32;
        break;
    case CHANNEL_1_APB_PTR_OFFSET:
        ret = s->channel_1_apb_ptr.reg32;
        break;
    case CHANNEL_1_APB_SEQ_OFFSET:
        ret = s->channel_1_apb_seq.reg32;
        break;
    case CHANNEL_2_CSR_OFFSET:
        ret = s->channel_2_csr.reg32;
        break;
    case CHANNEL_2_STA_OFFSET:
        ret = s->channel_2_sta.reg32;
        break;
    case CHANNEL_2_AHB_PTR_OFFSET:
        ret = s->channel_2_ahb_ptr.reg32;
        break;
    case CHANNEL_2_AHB_SEQ_OFFSET:
        ret = s->channel_2_ahb_seq.reg32;
        break;
    case CHANNEL_2_APB_PTR_OFFSET:
        ret = s->channel_2_apb_ptr.reg32;
        break;
    case CHANNEL_2_APB_SEQ_OFFSET:
        ret = s->channel_2_apb_seq.reg32;
        break;
    case CHANNEL_3_CSR_OFFSET:
        ret = s->channel_3_csr.reg32;
        break;
    case CHANNEL_3_STA_OFFSET:
        ret = s->channel_3_sta.reg32;
        break;
    case CHANNEL_3_AHB_PTR_OFFSET:
        ret = s->channel_3_ahb_ptr.reg32;
        break;
    case CHANNEL_3_AHB_SEQ_OFFSET:
        ret = s->channel_3_ahb_seq.reg32;
        break;
    case CHANNEL_3_APB_PTR_OFFSET:
        ret = s->channel_3_apb_ptr.reg32;
        break;
    case CHANNEL_3_APB_SEQ_OFFSET:
        ret = s->channel_3_apb_seq.reg32;
        break;
    case CHANNEL_4_CSR_OFFSET:
        ret = s->channel_4_csr.reg32;
        break;
    case CHANNEL_4_STA_OFFSET:
        ret = s->channel_4_sta.reg32;
        break;
    case CHANNEL_4_AHB_PTR_OFFSET:
        ret = s->channel_4_ahb_ptr.reg32;
        break;
    case CHANNEL_4_AHB_SEQ_OFFSET:
        ret = s->channel_4_ahb_seq.reg32;
        break;
    case CHANNEL_4_APB_PTR_OFFSET:
        ret = s->channel_4_apb_ptr.reg32;
        break;
    case CHANNEL_4_APB_SEQ_OFFSET:
        ret = s->channel_4_apb_seq.reg32;
        break;
    case CHANNEL_5_CSR_OFFSET:
        ret = s->channel_5_csr.reg32;
        break;
    case CHANNEL_5_STA_OFFSET:
        ret = s->channel_5_sta.reg32;
        break;
    case CHANNEL_5_AHB_PTR_OFFSET:
        ret = s->channel_5_ahb_ptr.reg32;
        break;
    case CHANNEL_5_AHB_SEQ_OFFSET:
        ret = s->channel_5_ahb_seq.reg32;
        break;
    case CHANNEL_5_APB_PTR_OFFSET:
        ret = s->channel_5_apb_ptr.reg32;
        break;
    case CHANNEL_5_APB_SEQ_OFFSET:
        ret = s->channel_5_apb_seq.reg32;
        break;
    case CHANNEL_6_CSR_OFFSET:
        ret = s->channel_6_csr.reg32;
        break;
    case CHANNEL_6_STA_OFFSET:
        ret = s->channel_6_sta.reg32;
        break;
    case CHANNEL_6_AHB_PTR_OFFSET:
        ret = s->channel_6_ahb_ptr.reg32;
        break;
    case CHANNEL_6_AHB_SEQ_OFFSET:
        ret = s->channel_6_ahb_seq.reg32;
        break;
    case CHANNEL_6_APB_PTR_OFFSET:
        ret = s->channel_6_apb_ptr.reg32;
        break;
    case CHANNEL_6_APB_SEQ_OFFSET:
        ret = s->channel_6_apb_seq.reg32;
        break;
    case CHANNEL_7_CSR_OFFSET:
        ret = s->channel_7_csr.reg32;
        break;
    case CHANNEL_7_STA_OFFSET:
        ret = s->channel_7_sta.reg32;
        break;
    case CHANNEL_7_AHB_PTR_OFFSET:
        ret = s->channel_7_ahb_ptr.reg32;
        break;
    case CHANNEL_7_AHB_SEQ_OFFSET:
        ret = s->channel_7_ahb_seq.reg32;
        break;
    case CHANNEL_7_APB_PTR_OFFSET:
        ret = s->channel_7_apb_ptr.reg32;
        break;
    case CHANNEL_7_APB_SEQ_OFFSET:
        ret = s->channel_7_apb_seq.reg32;
        break;
    case CHANNEL_8_CSR_OFFSET:
        ret = s->channel_8_csr.reg32;
        break;
    case CHANNEL_8_STA_OFFSET:
        ret = s->channel_8_sta.reg32;
        break;
    case CHANNEL_8_AHB_PTR_OFFSET:
        ret = s->channel_8_ahb_ptr.reg32;
        break;
    case CHANNEL_8_AHB_SEQ_OFFSET:
        ret = s->channel_8_ahb_seq.reg32;
        break;
    case CHANNEL_8_APB_PTR_OFFSET:
        ret = s->channel_8_apb_ptr.reg32;
        break;
    case CHANNEL_8_APB_SEQ_OFFSET:
        ret = s->channel_8_apb_seq.reg32;
        break;
    case CHANNEL_9_CSR_OFFSET:
        ret = s->channel_9_csr.reg32;
        break;
    case CHANNEL_9_STA_OFFSET:
        ret = s->channel_9_sta.reg32;
        break;
    case CHANNEL_9_AHB_PTR_OFFSET:
        ret = s->channel_9_ahb_ptr.reg32;
        break;
    case CHANNEL_9_AHB_SEQ_OFFSET:
        ret = s->channel_9_ahb_seq.reg32;
        break;
    case CHANNEL_9_APB_PTR_OFFSET:
        ret = s->channel_9_apb_ptr.reg32;
        break;
    case CHANNEL_9_APB_SEQ_OFFSET:
        ret = s->channel_9_apb_seq.reg32;
        break;
    case CHANNEL_10_CSR_OFFSET:
        ret = s->channel_10_csr.reg32;
        break;
    case CHANNEL_10_STA_OFFSET:
        ret = s->channel_10_sta.reg32;
        break;
    case CHANNEL_10_AHB_PTR_OFFSET:
        ret = s->channel_10_ahb_ptr.reg32;
        break;
    case CHANNEL_10_AHB_SEQ_OFFSET:
        ret = s->channel_10_ahb_seq.reg32;
        break;
    case CHANNEL_10_APB_PTR_OFFSET:
        ret = s->channel_10_apb_ptr.reg32;
        break;
    case CHANNEL_10_APB_SEQ_OFFSET:
        ret = s->channel_10_apb_seq.reg32;
        break;
    case CHANNEL_11_CSR_OFFSET:
        ret = s->channel_11_csr.reg32;
        break;
    case CHANNEL_11_STA_OFFSET:
        ret = s->channel_11_sta.reg32;
        break;
    case CHANNEL_11_AHB_PTR_OFFSET:
        ret = s->channel_11_ahb_ptr.reg32;
        break;
    case CHANNEL_11_AHB_SEQ_OFFSET:
        ret = s->channel_11_ahb_seq.reg32;
        break;
    case CHANNEL_11_APB_PTR_OFFSET:
        ret = s->channel_11_apb_ptr.reg32;
        break;
    case CHANNEL_11_APB_SEQ_OFFSET:
        ret = s->channel_11_apb_seq.reg32;
        break;
    case CHANNEL_12_CSR_OFFSET:
        ret = s->channel_12_csr.reg32;
        break;
    case CHANNEL_12_STA_OFFSET:
        ret = s->channel_12_sta.reg32;
        break;
    case CHANNEL_12_AHB_PTR_OFFSET:
        ret = s->channel_12_ahb_ptr.reg32;
        break;
    case CHANNEL_12_AHB_SEQ_OFFSET:
        ret = s->channel_12_ahb_seq.reg32;
        break;
    case CHANNEL_12_APB_PTR_OFFSET:
        ret = s->channel_12_apb_ptr.reg32;
        break;
    case CHANNEL_12_APB_SEQ_OFFSET:
        ret = s->channel_12_apb_seq.reg32;
        break;
    case CHANNEL_13_CSR_OFFSET:
        ret = s->channel_13_csr.reg32;
        break;
    case CHANNEL_13_STA_OFFSET:
        ret = s->channel_13_sta.reg32;
        break;
    case CHANNEL_13_AHB_PTR_OFFSET:
        ret = s->channel_13_ahb_ptr.reg32;
        break;
    case CHANNEL_13_AHB_SEQ_OFFSET:
        ret = s->channel_13_ahb_seq.reg32;
        break;
    case CHANNEL_13_APB_PTR_OFFSET:
        ret = s->channel_13_apb_ptr.reg32;
        break;
    case CHANNEL_13_APB_SEQ_OFFSET:
        ret = s->channel_13_apb_seq.reg32;
        break;
    case CHANNEL_14_CSR_OFFSET:
        ret = s->channel_14_csr.reg32;
        break;
    case CHANNEL_14_STA_OFFSET:
        ret = s->channel_14_sta.reg32;
        break;
    case CHANNEL_14_AHB_PTR_OFFSET:
        ret = s->channel_14_ahb_ptr.reg32;
        break;
    case CHANNEL_14_AHB_SEQ_OFFSET:
        ret = s->channel_14_ahb_seq.reg32;
        break;
    case CHANNEL_14_APB_PTR_OFFSET:
        ret = s->channel_14_apb_ptr.reg32;
        break;
    case CHANNEL_14_APB_SEQ_OFFSET:
        ret = s->channel_14_apb_seq.reg32;
        break;
    case CHANNEL_15_CSR_OFFSET:
        ret = s->channel_15_csr.reg32;
        break;
    case CHANNEL_15_STA_OFFSET:
        ret = s->channel_15_sta.reg32;
        break;
    case CHANNEL_15_AHB_PTR_OFFSET:
        ret = s->channel_15_ahb_ptr.reg32;
        break;
    case CHANNEL_15_AHB_SEQ_OFFSET:
        ret = s->channel_15_ahb_seq.reg32;
        break;
    case CHANNEL_15_APB_PTR_OFFSET:
        ret = s->channel_15_apb_ptr.reg32;
        break;
    case CHANNEL_15_APB_SEQ_OFFSET:
        ret = s->channel_15_apb_seq.reg32;
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
        break;
    case IRQ_MASK_CLR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->irq_mask_clr.reg32, value);
        s->irq_mask_clr.reg32 = value;
        break;
    case CHANNEL_0_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_0_csr.reg32, value);
        s->channel_0_csr.reg32 = value;
        break;
    case CHANNEL_0_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_0_sta.reg32, value & CHANNEL_0_STA_WRMASK);
WR_MASKED(       s->channel_0_sta.reg32, value, CHANNEL_0_STA);
        break;
    case CHANNEL_0_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_0_ahb_ptr.reg32, value);
        s->channel_0_ahb_ptr.reg32 = value;
        break;
    case CHANNEL_0_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_0_ahb_seq.reg32, value);
        s->channel_0_ahb_seq.reg32 = value;
        break;
    case CHANNEL_0_APB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_0_apb_ptr.reg32, value);
        s->channel_0_apb_ptr.reg32 = value;
        break;
    case CHANNEL_0_APB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_0_apb_seq.reg32, value);
        s->channel_0_apb_seq.reg32 = value;
        break;
    case CHANNEL_1_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_1_csr.reg32, value);
        s->channel_1_csr.reg32 = value;
        break;
    case CHANNEL_1_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_1_sta.reg32, value & CHANNEL_1_STA_WRMASK);
WR_MASKED(       s->channel_1_sta.reg32, value, CHANNEL_1_STA);
        break;
    case CHANNEL_1_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_1_ahb_ptr.reg32, value);
        s->channel_1_ahb_ptr.reg32 = value;
        break;
    case CHANNEL_1_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_1_ahb_seq.reg32, value);
        s->channel_1_ahb_seq.reg32 = value;
        break;
    case CHANNEL_1_APB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_1_apb_ptr.reg32, value);
        s->channel_1_apb_ptr.reg32 = value;
        break;
    case CHANNEL_1_APB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_1_apb_seq.reg32, value);
        s->channel_1_apb_seq.reg32 = value;
        break;
    case CHANNEL_2_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_2_csr.reg32, value);
        s->channel_2_csr.reg32 = value;
        break;
    case CHANNEL_2_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_2_sta.reg32, value & CHANNEL_2_STA_WRMASK);
WR_MASKED(       s->channel_2_sta.reg32, value, CHANNEL_2_STA);
        break;
    case CHANNEL_2_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_2_ahb_ptr.reg32, value);
        s->channel_2_ahb_ptr.reg32 = value;
        break;
    case CHANNEL_2_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_2_ahb_seq.reg32, value);
        s->channel_2_ahb_seq.reg32 = value;
        break;
    case CHANNEL_2_APB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_2_apb_ptr.reg32, value);
        s->channel_2_apb_ptr.reg32 = value;
        break;
    case CHANNEL_2_APB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_2_apb_seq.reg32, value);
        s->channel_2_apb_seq.reg32 = value;
        break;
    case CHANNEL_3_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_3_csr.reg32, value);
        s->channel_3_csr.reg32 = value;
        break;
    case CHANNEL_3_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_3_sta.reg32, value & CHANNEL_3_STA_WRMASK);
WR_MASKED(       s->channel_3_sta.reg32, value, CHANNEL_3_STA);
        break;
    case CHANNEL_3_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_3_ahb_ptr.reg32, value);
        s->channel_3_ahb_ptr.reg32 = value;
        break;
    case CHANNEL_3_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_3_ahb_seq.reg32, value);
        s->channel_3_ahb_seq.reg32 = value;
        break;
    case CHANNEL_3_APB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_3_apb_ptr.reg32, value);
        s->channel_3_apb_ptr.reg32 = value;
        break;
    case CHANNEL_3_APB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_3_apb_seq.reg32, value);
        s->channel_3_apb_seq.reg32 = value;
        break;
    case CHANNEL_4_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_4_csr.reg32, value);
        s->channel_4_csr.reg32 = value;
        break;
    case CHANNEL_4_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_4_sta.reg32, value & CHANNEL_4_STA_WRMASK);
WR_MASKED(       s->channel_4_sta.reg32, value, CHANNEL_4_STA);
        break;
    case CHANNEL_4_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_4_ahb_ptr.reg32, value);
        s->channel_4_ahb_ptr.reg32 = value;
        break;
    case CHANNEL_4_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_4_ahb_seq.reg32, value);
        s->channel_4_ahb_seq.reg32 = value;
        break;
    case CHANNEL_4_APB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_4_apb_ptr.reg32, value);
        s->channel_4_apb_ptr.reg32 = value;
        break;
    case CHANNEL_4_APB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_4_apb_seq.reg32, value);
        s->channel_4_apb_seq.reg32 = value;
        break;
    case CHANNEL_5_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_5_csr.reg32, value);
        s->channel_5_csr.reg32 = value;
        break;
    case CHANNEL_5_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_5_sta.reg32, value & CHANNEL_5_STA_WRMASK);
WR_MASKED(       s->channel_5_sta.reg32, value, CHANNEL_5_STA);
        break;
    case CHANNEL_5_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_5_ahb_ptr.reg32, value);
        s->channel_5_ahb_ptr.reg32 = value;
        break;
    case CHANNEL_5_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_5_ahb_seq.reg32, value);
        s->channel_5_ahb_seq.reg32 = value;
        break;
    case CHANNEL_5_APB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_5_apb_ptr.reg32, value);
        s->channel_5_apb_ptr.reg32 = value;
        break;
    case CHANNEL_5_APB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_5_apb_seq.reg32, value);
        s->channel_5_apb_seq.reg32 = value;
        break;
    case CHANNEL_6_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_6_csr.reg32, value);
        s->channel_6_csr.reg32 = value;
        break;
    case CHANNEL_6_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_6_sta.reg32, value & CHANNEL_6_STA_WRMASK);
WR_MASKED(       s->channel_6_sta.reg32, value, CHANNEL_6_STA);
        break;
    case CHANNEL_6_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_6_ahb_ptr.reg32, value);
        s->channel_6_ahb_ptr.reg32 = value;
        break;
    case CHANNEL_6_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_6_ahb_seq.reg32, value);
        s->channel_6_ahb_seq.reg32 = value;
        break;
    case CHANNEL_6_APB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_6_apb_ptr.reg32, value);
        s->channel_6_apb_ptr.reg32 = value;
        break;
    case CHANNEL_6_APB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_6_apb_seq.reg32, value);
        s->channel_6_apb_seq.reg32 = value;
        break;
    case CHANNEL_7_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_7_csr.reg32, value);
        s->channel_7_csr.reg32 = value;
        break;
    case CHANNEL_7_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_7_sta.reg32, value & CHANNEL_7_STA_WRMASK);
WR_MASKED(       s->channel_7_sta.reg32, value, CHANNEL_7_STA);
        break;
    case CHANNEL_7_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_7_ahb_ptr.reg32, value);
        s->channel_7_ahb_ptr.reg32 = value;
        break;
    case CHANNEL_7_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_7_ahb_seq.reg32, value);
        s->channel_7_ahb_seq.reg32 = value;
        break;
    case CHANNEL_7_APB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_7_apb_ptr.reg32, value);
        s->channel_7_apb_ptr.reg32 = value;
        break;
    case CHANNEL_7_APB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_7_apb_seq.reg32, value);
        s->channel_7_apb_seq.reg32 = value;
        break;
    case CHANNEL_8_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_8_csr.reg32, value);
        s->channel_8_csr.reg32 = value;
        break;
    case CHANNEL_8_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_8_sta.reg32, value & CHANNEL_8_STA_WRMASK);
WR_MASKED(       s->channel_8_sta.reg32, value, CHANNEL_8_STA);
        break;
    case CHANNEL_8_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_8_ahb_ptr.reg32, value);
        s->channel_8_ahb_ptr.reg32 = value;
        break;
    case CHANNEL_8_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_8_ahb_seq.reg32, value);
        s->channel_8_ahb_seq.reg32 = value;
        break;
    case CHANNEL_8_APB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_8_apb_ptr.reg32, value);
        s->channel_8_apb_ptr.reg32 = value;
        break;
    case CHANNEL_8_APB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_8_apb_seq.reg32, value);
        s->channel_8_apb_seq.reg32 = value;
        break;
    case CHANNEL_9_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_9_csr.reg32, value);
        s->channel_9_csr.reg32 = value;
        break;
    case CHANNEL_9_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_9_sta.reg32, value & CHANNEL_9_STA_WRMASK);
WR_MASKED(       s->channel_9_sta.reg32, value, CHANNEL_9_STA);
        break;
    case CHANNEL_9_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_9_ahb_ptr.reg32, value);
        s->channel_9_ahb_ptr.reg32 = value;
        break;
    case CHANNEL_9_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_9_ahb_seq.reg32, value);
        s->channel_9_ahb_seq.reg32 = value;
        break;
    case CHANNEL_9_APB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_9_apb_ptr.reg32, value);
        s->channel_9_apb_ptr.reg32 = value;
        break;
    case CHANNEL_9_APB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_9_apb_seq.reg32, value);
        s->channel_9_apb_seq.reg32 = value;
        break;
    case CHANNEL_10_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_10_csr.reg32, value);
        s->channel_10_csr.reg32 = value;
        break;
    case CHANNEL_10_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_10_sta.reg32, value & CHANNEL_10_STA_WRMASK);
WR_MASKED(       s->channel_10_sta.reg32, value, CHANNEL_10_STA);
        break;
    case CHANNEL_10_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_10_ahb_ptr.reg32, value);
        s->channel_10_ahb_ptr.reg32 = value;
        break;
    case CHANNEL_10_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_10_ahb_seq.reg32, value);
        s->channel_10_ahb_seq.reg32 = value;
        break;
    case CHANNEL_10_APB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_10_apb_ptr.reg32, value);
        s->channel_10_apb_ptr.reg32 = value;
        break;
    case CHANNEL_10_APB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_10_apb_seq.reg32, value);
        s->channel_10_apb_seq.reg32 = value;
        break;
    case CHANNEL_11_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_11_csr.reg32, value);
        s->channel_11_csr.reg32 = value;
        break;
    case CHANNEL_11_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_11_sta.reg32, value & CHANNEL_11_STA_WRMASK);
WR_MASKED(       s->channel_11_sta.reg32, value, CHANNEL_11_STA);
        break;
    case CHANNEL_11_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_11_ahb_ptr.reg32, value);
        s->channel_11_ahb_ptr.reg32 = value;
        break;
    case CHANNEL_11_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_11_ahb_seq.reg32, value);
        s->channel_11_ahb_seq.reg32 = value;
        break;
    case CHANNEL_11_APB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_11_apb_ptr.reg32, value);
        s->channel_11_apb_ptr.reg32 = value;
        break;
    case CHANNEL_11_APB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_11_apb_seq.reg32, value);
        s->channel_11_apb_seq.reg32 = value;
        break;
    case CHANNEL_12_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_12_csr.reg32, value);
        s->channel_12_csr.reg32 = value;
        break;
    case CHANNEL_12_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_12_sta.reg32, value & CHANNEL_12_STA_WRMASK);
WR_MASKED(       s->channel_12_sta.reg32, value, CHANNEL_12_STA);
        break;
    case CHANNEL_12_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_12_ahb_ptr.reg32, value);
        s->channel_12_ahb_ptr.reg32 = value;
        break;
    case CHANNEL_12_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_12_ahb_seq.reg32, value);
        s->channel_12_ahb_seq.reg32 = value;
        break;
    case CHANNEL_12_APB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_12_apb_ptr.reg32, value);
        s->channel_12_apb_ptr.reg32 = value;
        break;
    case CHANNEL_12_APB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_12_apb_seq.reg32, value);
        s->channel_12_apb_seq.reg32 = value;
        break;
    case CHANNEL_13_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_13_csr.reg32, value);
        s->channel_13_csr.reg32 = value;
        break;
    case CHANNEL_13_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_13_sta.reg32, value & CHANNEL_13_STA_WRMASK);
WR_MASKED(       s->channel_13_sta.reg32, value, CHANNEL_13_STA);
        break;
    case CHANNEL_13_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_13_ahb_ptr.reg32, value);
        s->channel_13_ahb_ptr.reg32 = value;
        break;
    case CHANNEL_13_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_13_ahb_seq.reg32, value);
        s->channel_13_ahb_seq.reg32 = value;
        break;
    case CHANNEL_13_APB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_13_apb_ptr.reg32, value);
        s->channel_13_apb_ptr.reg32 = value;
        break;
    case CHANNEL_13_APB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_13_apb_seq.reg32, value);
        s->channel_13_apb_seq.reg32 = value;
        break;
    case CHANNEL_14_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_14_csr.reg32, value);
        s->channel_14_csr.reg32 = value;
        break;
    case CHANNEL_14_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_14_sta.reg32, value & CHANNEL_14_STA_WRMASK);
WR_MASKED(       s->channel_14_sta.reg32, value, CHANNEL_14_STA);
        break;
    case CHANNEL_14_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_14_ahb_ptr.reg32, value);
        s->channel_14_ahb_ptr.reg32 = value;
        break;
    case CHANNEL_14_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_14_ahb_seq.reg32, value);
        s->channel_14_ahb_seq.reg32 = value;
        break;
    case CHANNEL_14_APB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_14_apb_ptr.reg32, value);
        s->channel_14_apb_ptr.reg32 = value;
        break;
    case CHANNEL_14_APB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_14_apb_seq.reg32, value);
        s->channel_14_apb_seq.reg32 = value;
        break;
    case CHANNEL_15_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_15_csr.reg32, value);
        s->channel_15_csr.reg32 = value;
        break;
    case CHANNEL_15_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_15_sta.reg32, value & CHANNEL_15_STA_WRMASK);
WR_MASKED(       s->channel_15_sta.reg32, value, CHANNEL_15_STA);
        break;
    case CHANNEL_15_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_15_ahb_ptr.reg32, value);
        s->channel_15_ahb_ptr.reg32 = value;
        break;
    case CHANNEL_15_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_15_ahb_seq.reg32, value);
        s->channel_15_ahb_seq.reg32 = value;
        break;
    case CHANNEL_15_APB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_15_apb_ptr.reg32, value);
        s->channel_15_apb_ptr.reg32 = value;
        break;
    case CHANNEL_15_APB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->channel_15_apb_seq.reg32, value);
        s->channel_15_apb_seq.reg32 = value;
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
    s->channel_0_csr.reg32 = CHANNEL_0_CSR_RESET;
    s->channel_0_sta.reg32 = CHANNEL_0_STA_RESET;
    s->channel_0_ahb_ptr.reg32 = CHANNEL_0_AHB_PTR_RESET;
    s->channel_0_ahb_seq.reg32 = CHANNEL_0_AHB_SEQ_RESET;
    s->channel_0_apb_ptr.reg32 = CHANNEL_0_APB_PTR_RESET;
    s->channel_0_apb_seq.reg32 = CHANNEL_0_APB_SEQ_RESET;
    s->channel_1_csr.reg32 = CHANNEL_1_CSR_RESET;
    s->channel_1_sta.reg32 = CHANNEL_1_STA_RESET;
    s->channel_1_ahb_ptr.reg32 = CHANNEL_1_AHB_PTR_RESET;
    s->channel_1_ahb_seq.reg32 = CHANNEL_1_AHB_SEQ_RESET;
    s->channel_1_apb_ptr.reg32 = CHANNEL_1_APB_PTR_RESET;
    s->channel_1_apb_seq.reg32 = CHANNEL_1_APB_SEQ_RESET;
    s->channel_2_csr.reg32 = CHANNEL_2_CSR_RESET;
    s->channel_2_sta.reg32 = CHANNEL_2_STA_RESET;
    s->channel_2_ahb_ptr.reg32 = CHANNEL_2_AHB_PTR_RESET;
    s->channel_2_ahb_seq.reg32 = CHANNEL_2_AHB_SEQ_RESET;
    s->channel_2_apb_ptr.reg32 = CHANNEL_2_APB_PTR_RESET;
    s->channel_2_apb_seq.reg32 = CHANNEL_2_APB_SEQ_RESET;
    s->channel_3_csr.reg32 = CHANNEL_3_CSR_RESET;
    s->channel_3_sta.reg32 = CHANNEL_3_STA_RESET;
    s->channel_3_ahb_ptr.reg32 = CHANNEL_3_AHB_PTR_RESET;
    s->channel_3_ahb_seq.reg32 = CHANNEL_3_AHB_SEQ_RESET;
    s->channel_3_apb_ptr.reg32 = CHANNEL_3_APB_PTR_RESET;
    s->channel_3_apb_seq.reg32 = CHANNEL_3_APB_SEQ_RESET;
    s->channel_4_csr.reg32 = CHANNEL_4_CSR_RESET;
    s->channel_4_sta.reg32 = CHANNEL_4_STA_RESET;
    s->channel_4_ahb_ptr.reg32 = CHANNEL_4_AHB_PTR_RESET;
    s->channel_4_ahb_seq.reg32 = CHANNEL_4_AHB_SEQ_RESET;
    s->channel_4_apb_ptr.reg32 = CHANNEL_4_APB_PTR_RESET;
    s->channel_4_apb_seq.reg32 = CHANNEL_4_APB_SEQ_RESET;
    s->channel_5_csr.reg32 = CHANNEL_5_CSR_RESET;
    s->channel_5_sta.reg32 = CHANNEL_5_STA_RESET;
    s->channel_5_ahb_ptr.reg32 = CHANNEL_5_AHB_PTR_RESET;
    s->channel_5_ahb_seq.reg32 = CHANNEL_5_AHB_SEQ_RESET;
    s->channel_5_apb_ptr.reg32 = CHANNEL_5_APB_PTR_RESET;
    s->channel_5_apb_seq.reg32 = CHANNEL_5_APB_SEQ_RESET;
    s->channel_6_csr.reg32 = CHANNEL_6_CSR_RESET;
    s->channel_6_sta.reg32 = CHANNEL_6_STA_RESET;
    s->channel_6_ahb_ptr.reg32 = CHANNEL_6_AHB_PTR_RESET;
    s->channel_6_ahb_seq.reg32 = CHANNEL_6_AHB_SEQ_RESET;
    s->channel_6_apb_ptr.reg32 = CHANNEL_6_APB_PTR_RESET;
    s->channel_6_apb_seq.reg32 = CHANNEL_6_APB_SEQ_RESET;
    s->channel_7_csr.reg32 = CHANNEL_7_CSR_RESET;
    s->channel_7_sta.reg32 = CHANNEL_7_STA_RESET;
    s->channel_7_ahb_ptr.reg32 = CHANNEL_7_AHB_PTR_RESET;
    s->channel_7_ahb_seq.reg32 = CHANNEL_7_AHB_SEQ_RESET;
    s->channel_7_apb_ptr.reg32 = CHANNEL_7_APB_PTR_RESET;
    s->channel_7_apb_seq.reg32 = CHANNEL_7_APB_SEQ_RESET;
    s->channel_8_csr.reg32 = CHANNEL_8_CSR_RESET;
    s->channel_8_sta.reg32 = CHANNEL_8_STA_RESET;
    s->channel_8_ahb_ptr.reg32 = CHANNEL_8_AHB_PTR_RESET;
    s->channel_8_ahb_seq.reg32 = CHANNEL_8_AHB_SEQ_RESET;
    s->channel_8_apb_ptr.reg32 = CHANNEL_8_APB_PTR_RESET;
    s->channel_8_apb_seq.reg32 = CHANNEL_8_APB_SEQ_RESET;
    s->channel_9_csr.reg32 = CHANNEL_9_CSR_RESET;
    s->channel_9_sta.reg32 = CHANNEL_9_STA_RESET;
    s->channel_9_ahb_ptr.reg32 = CHANNEL_9_AHB_PTR_RESET;
    s->channel_9_ahb_seq.reg32 = CHANNEL_9_AHB_SEQ_RESET;
    s->channel_9_apb_ptr.reg32 = CHANNEL_9_APB_PTR_RESET;
    s->channel_9_apb_seq.reg32 = CHANNEL_9_APB_SEQ_RESET;
    s->channel_10_csr.reg32 = CHANNEL_10_CSR_RESET;
    s->channel_10_sta.reg32 = CHANNEL_10_STA_RESET;
    s->channel_10_ahb_ptr.reg32 = CHANNEL_10_AHB_PTR_RESET;
    s->channel_10_ahb_seq.reg32 = CHANNEL_10_AHB_SEQ_RESET;
    s->channel_10_apb_ptr.reg32 = CHANNEL_10_APB_PTR_RESET;
    s->channel_10_apb_seq.reg32 = CHANNEL_10_APB_SEQ_RESET;
    s->channel_11_csr.reg32 = CHANNEL_11_CSR_RESET;
    s->channel_11_sta.reg32 = CHANNEL_11_STA_RESET;
    s->channel_11_ahb_ptr.reg32 = CHANNEL_11_AHB_PTR_RESET;
    s->channel_11_ahb_seq.reg32 = CHANNEL_11_AHB_SEQ_RESET;
    s->channel_11_apb_ptr.reg32 = CHANNEL_11_APB_PTR_RESET;
    s->channel_11_apb_seq.reg32 = CHANNEL_11_APB_SEQ_RESET;
    s->channel_12_csr.reg32 = CHANNEL_12_CSR_RESET;
    s->channel_12_sta.reg32 = CHANNEL_12_STA_RESET;
    s->channel_12_ahb_ptr.reg32 = CHANNEL_12_AHB_PTR_RESET;
    s->channel_12_ahb_seq.reg32 = CHANNEL_12_AHB_SEQ_RESET;
    s->channel_12_apb_ptr.reg32 = CHANNEL_12_APB_PTR_RESET;
    s->channel_12_apb_seq.reg32 = CHANNEL_12_APB_SEQ_RESET;
    s->channel_13_csr.reg32 = CHANNEL_13_CSR_RESET;
    s->channel_13_sta.reg32 = CHANNEL_13_STA_RESET;
    s->channel_13_ahb_ptr.reg32 = CHANNEL_13_AHB_PTR_RESET;
    s->channel_13_ahb_seq.reg32 = CHANNEL_13_AHB_SEQ_RESET;
    s->channel_13_apb_ptr.reg32 = CHANNEL_13_APB_PTR_RESET;
    s->channel_13_apb_seq.reg32 = CHANNEL_13_APB_SEQ_RESET;
    s->channel_14_csr.reg32 = CHANNEL_14_CSR_RESET;
    s->channel_14_sta.reg32 = CHANNEL_14_STA_RESET;
    s->channel_14_ahb_ptr.reg32 = CHANNEL_14_AHB_PTR_RESET;
    s->channel_14_ahb_seq.reg32 = CHANNEL_14_AHB_SEQ_RESET;
    s->channel_14_apb_ptr.reg32 = CHANNEL_14_APB_PTR_RESET;
    s->channel_14_apb_seq.reg32 = CHANNEL_14_APB_SEQ_RESET;
    s->channel_15_csr.reg32 = CHANNEL_15_CSR_RESET;
    s->channel_15_sta.reg32 = CHANNEL_15_STA_RESET;
    s->channel_15_ahb_ptr.reg32 = CHANNEL_15_AHB_PTR_RESET;
    s->channel_15_ahb_seq.reg32 = CHANNEL_15_AHB_SEQ_RESET;
    s->channel_15_apb_ptr.reg32 = CHANNEL_15_APB_PTR_RESET;
    s->channel_15_apb_seq.reg32 = CHANNEL_15_APB_SEQ_RESET;
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

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_apb_dma_mem_ops, s,
                          "tegra.apb_dma", TEGRA_APB_DMA_SIZE + (num_channels * channel_size));
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
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
