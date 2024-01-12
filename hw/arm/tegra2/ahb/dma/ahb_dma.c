/*
 * ARM NVIDIA Tegra2 emulation.
 *
 * Copyright (c) 2016 Dmitry Osipenko <digetx@gmail.com>
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

#include "ahb_dma.h"
#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_AHB_DMA "tegra.ahb_dma"
#define TEGRA_AHB_DMA(obj) OBJECT_CHECK(tegra_ahb_dma, (obj), TYPE_TEGRA_AHB_DMA)
#define DEFINE_REG32(reg) reg##_t reg

typedef struct tegra_ahb_dma_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    DEFINE_REG32(cmd);
    DEFINE_REG32(sta);
    DEFINE_REG32(tx_req);
    DEFINE_REG32(counter);
    DEFINE_REG32(irq_sta_cpu);
    DEFINE_REG32(irq_sta_cop);
    DEFINE_REG32(irq_mask);
    DEFINE_REG32(irq_mask_set);
    DEFINE_REG32(irq_mask_clr);
    DEFINE_REG32(rdwr_coherency);
    DEFINE_REG32(test_bus);
    DEFINE_REG32(ppcs_mccif_fifoctrl);
    DEFINE_REG32(timeout_wcoal_ppcs);
    DEFINE_REG32(ahbdmachan_channel_0_csr);
    DEFINE_REG32(ahbdmachan_channel_0_sta);
    DEFINE_REG32(ahbdmachan_channel_0_ahb_ptr);
    DEFINE_REG32(ahbdmachan_channel_0_ahb_seq);
    DEFINE_REG32(ahbdmachan_channel_0_xmb_ptr);
    DEFINE_REG32(ahbdmachan_channel_0_xmb_seq);
    DEFINE_REG32(ahbdmachan_channel_1_csr);
    DEFINE_REG32(ahbdmachan_channel_1_sta);
    DEFINE_REG32(ahbdmachan_channel_1_ahb_ptr);
    DEFINE_REG32(ahbdmachan_channel_1_ahb_seq);
    DEFINE_REG32(ahbdmachan_channel_1_xmb_ptr);
    DEFINE_REG32(ahbdmachan_channel_1_xmb_seq);
    DEFINE_REG32(ahbdmachan_channel_2_csr);
    DEFINE_REG32(ahbdmachan_channel_2_sta);
    DEFINE_REG32(ahbdmachan_channel_2_ahb_ptr);
    DEFINE_REG32(ahbdmachan_channel_2_ahb_seq);
    DEFINE_REG32(ahbdmachan_channel_2_xmb_ptr);
    DEFINE_REG32(ahbdmachan_channel_2_xmb_seq);
    DEFINE_REG32(ahbdmachan_channel_3_csr);
    DEFINE_REG32(ahbdmachan_channel_3_sta);
    DEFINE_REG32(ahbdmachan_channel_3_ahb_ptr);
    DEFINE_REG32(ahbdmachan_channel_3_ahb_seq);
    DEFINE_REG32(ahbdmachan_channel_3_xmb_ptr);
    DEFINE_REG32(ahbdmachan_channel_3_xmb_seq);
} tegra_ahb_dma;

static const VMStateDescription vmstate_tegra_ahb_dma = {
    .name = "tegra.ahb_dma",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(cmd.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(sta.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(tx_req.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(counter.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(irq_sta_cpu.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(irq_sta_cop.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(irq_mask.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(irq_mask_set.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(irq_mask_clr.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(rdwr_coherency.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(test_bus.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ppcs_mccif_fifoctrl.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(timeout_wcoal_ppcs.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_0_csr.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_0_sta.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_0_ahb_ptr.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_0_ahb_seq.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_0_xmb_ptr.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_0_xmb_seq.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_1_csr.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_1_sta.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_1_ahb_ptr.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_1_ahb_seq.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_1_xmb_ptr.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_1_xmb_seq.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_2_csr.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_2_sta.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_2_ahb_ptr.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_2_ahb_seq.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_2_xmb_ptr.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_2_xmb_seq.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_3_csr.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_3_sta.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_3_ahb_ptr.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_3_ahb_seq.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_3_xmb_ptr.reg32, tegra_ahb_dma),
        VMSTATE_UINT32(ahbdmachan_channel_3_xmb_seq.reg32, tegra_ahb_dma),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_ahb_dma_priv_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    tegra_ahb_dma *s = opaque;
    uint64_t ret = 0;

    assert(size == 4);

    switch (offset) {
    case CMD_OFFSET:
        ret = s->cmd.reg32;
        break;
    case STA_OFFSET:
        ret = s->sta.reg32;
        break;
    case TX_REQ_OFFSET:
        ret = s->tx_req.reg32;
        break;
    case COUNTER_OFFSET:
        ret = s->counter.reg32;
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
    case RDWR_COHERENCY_OFFSET:
        ret = s->rdwr_coherency.reg32;
        break;
    case TEST_BUS_OFFSET:
        ret = s->test_bus.reg32;
        break;
    case PPCS_MCCIF_FIFOCTRL_OFFSET:
        ret = s->ppcs_mccif_fifoctrl.reg32;
        break;
    case TIMEOUT_WCOAL_PPCS_OFFSET:
        ret = s->timeout_wcoal_ppcs.reg32;
        break;
    case AHBDMACHAN_CHANNEL_0_CSR_OFFSET:
        ret = s->ahbdmachan_channel_0_csr.reg32;
        break;
    case AHBDMACHAN_CHANNEL_0_STA_OFFSET:
        ret = s->ahbdmachan_channel_0_sta.reg32;
        break;
    case AHBDMACHAN_CHANNEL_0_AHB_PTR_OFFSET:
        ret = s->ahbdmachan_channel_0_ahb_ptr.reg32;
        break;
    case AHBDMACHAN_CHANNEL_0_AHB_SEQ_OFFSET:
        ret = s->ahbdmachan_channel_0_ahb_seq.reg32;
        break;
    case AHBDMACHAN_CHANNEL_0_XMB_PTR_OFFSET:
        ret = s->ahbdmachan_channel_0_xmb_ptr.reg32;
        break;
    case AHBDMACHAN_CHANNEL_0_XMB_SEQ_OFFSET:
        ret = s->ahbdmachan_channel_0_xmb_seq.reg32;
        break;
    case AHBDMACHAN_CHANNEL_1_CSR_OFFSET:
        ret = s->ahbdmachan_channel_1_csr.reg32;
        break;
    case AHBDMACHAN_CHANNEL_1_STA_OFFSET:
        ret = s->ahbdmachan_channel_1_sta.reg32;
        break;
    case AHBDMACHAN_CHANNEL_1_AHB_PTR_OFFSET:
        ret = s->ahbdmachan_channel_1_ahb_ptr.reg32;
        break;
    case AHBDMACHAN_CHANNEL_1_AHB_SEQ_OFFSET:
        ret = s->ahbdmachan_channel_1_ahb_seq.reg32;
        break;
    case AHBDMACHAN_CHANNEL_1_XMB_PTR_OFFSET:
        ret = s->ahbdmachan_channel_1_xmb_ptr.reg32;
        break;
    case AHBDMACHAN_CHANNEL_1_XMB_SEQ_OFFSET:
        ret = s->ahbdmachan_channel_1_xmb_seq.reg32;
        break;
    case AHBDMACHAN_CHANNEL_2_CSR_OFFSET:
        ret = s->ahbdmachan_channel_2_csr.reg32;
        break;
    case AHBDMACHAN_CHANNEL_2_STA_OFFSET:
        ret = s->ahbdmachan_channel_2_sta.reg32;
        break;
    case AHBDMACHAN_CHANNEL_2_AHB_PTR_OFFSET:
        ret = s->ahbdmachan_channel_2_ahb_ptr.reg32;
        break;
    case AHBDMACHAN_CHANNEL_2_AHB_SEQ_OFFSET:
        ret = s->ahbdmachan_channel_2_ahb_seq.reg32;
        break;
    case AHBDMACHAN_CHANNEL_2_XMB_PTR_OFFSET:
        ret = s->ahbdmachan_channel_2_xmb_ptr.reg32;
        break;
    case AHBDMACHAN_CHANNEL_2_XMB_SEQ_OFFSET:
        ret = s->ahbdmachan_channel_2_xmb_seq.reg32;
        break;
    case AHBDMACHAN_CHANNEL_3_CSR_OFFSET:
        ret = s->ahbdmachan_channel_3_csr.reg32;
        break;
    case AHBDMACHAN_CHANNEL_3_STA_OFFSET:
        ret = s->ahbdmachan_channel_3_sta.reg32;
        break;
    case AHBDMACHAN_CHANNEL_3_AHB_PTR_OFFSET:
        ret = s->ahbdmachan_channel_3_ahb_ptr.reg32;
        break;
    case AHBDMACHAN_CHANNEL_3_AHB_SEQ_OFFSET:
        ret = s->ahbdmachan_channel_3_ahb_seq.reg32;
        break;
    case AHBDMACHAN_CHANNEL_3_XMB_PTR_OFFSET:
        ret = s->ahbdmachan_channel_3_xmb_ptr.reg32;
        break;
    case AHBDMACHAN_CHANNEL_3_XMB_SEQ_OFFSET:
        ret = s->ahbdmachan_channel_3_xmb_seq.reg32;
        break;
    default:
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_ahb_dma_priv_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    tegra_ahb_dma *s = opaque;

    assert(size == 4);

    switch (offset) {
    case CMD_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->cmd.reg32, value);
        s->cmd.reg32 = value;
        break;
    case COUNTER_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->counter.reg32, value);
        s->counter.reg32 = value;
        break;
    case IRQ_MASK_SET_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->irq_mask_set.reg32, value);
        s->irq_mask_set.reg32 = value;
        break;
    case IRQ_MASK_CLR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->irq_mask_clr.reg32, value);
        s->irq_mask_clr.reg32 = value;
        break;
    case PPCS_MCCIF_FIFOCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ppcs_mccif_fifoctrl.reg32, value);
        s->ppcs_mccif_fifoctrl.reg32 = value;
        break;
    case TIMEOUT_WCOAL_PPCS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_wcoal_ppcs.reg32, value);
        s->timeout_wcoal_ppcs.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_0_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_0_csr.reg32, value);
        s->ahbdmachan_channel_0_csr.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_0_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_0_sta.reg32, value);
        s->ahbdmachan_channel_0_sta.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_0_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_0_ahb_ptr.reg32, value);
        s->ahbdmachan_channel_0_ahb_ptr.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_0_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_0_ahb_seq.reg32, value);
        s->ahbdmachan_channel_0_ahb_seq.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_0_XMB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_0_xmb_ptr.reg32, value);
        s->ahbdmachan_channel_0_xmb_ptr.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_0_XMB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_0_xmb_seq.reg32, value);
        s->ahbdmachan_channel_0_xmb_seq.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_1_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_1_csr.reg32, value);
        s->ahbdmachan_channel_1_csr.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_1_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_1_sta.reg32, value);
        s->ahbdmachan_channel_1_sta.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_1_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_1_ahb_ptr.reg32, value);
        s->ahbdmachan_channel_1_ahb_ptr.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_1_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_1_ahb_seq.reg32, value);
        s->ahbdmachan_channel_1_ahb_seq.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_1_XMB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_1_xmb_ptr.reg32, value);
        s->ahbdmachan_channel_1_xmb_ptr.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_1_XMB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_1_xmb_seq.reg32, value);
        s->ahbdmachan_channel_1_xmb_seq.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_2_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_2_csr.reg32, value);
        s->ahbdmachan_channel_2_csr.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_2_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_2_sta.reg32, value);
        s->ahbdmachan_channel_2_sta.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_2_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_2_ahb_ptr.reg32, value);
        s->ahbdmachan_channel_2_ahb_ptr.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_2_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_2_ahb_seq.reg32, value);
        s->ahbdmachan_channel_2_ahb_seq.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_2_XMB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_2_xmb_ptr.reg32, value);
        s->ahbdmachan_channel_2_xmb_ptr.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_2_XMB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_2_xmb_seq.reg32, value);
        s->ahbdmachan_channel_2_xmb_seq.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_3_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_3_csr.reg32, value);
        s->ahbdmachan_channel_3_csr.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_3_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_3_sta.reg32, value);
        s->ahbdmachan_channel_3_sta.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_3_AHB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_3_ahb_ptr.reg32, value);
        s->ahbdmachan_channel_3_ahb_ptr.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_3_AHB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_3_ahb_seq.reg32, value);
        s->ahbdmachan_channel_3_ahb_seq.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_3_XMB_PTR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_3_xmb_ptr.reg32, value);
        s->ahbdmachan_channel_3_xmb_ptr.reg32 = value;
        break;
    case AHBDMACHAN_CHANNEL_3_XMB_SEQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahbdmachan_channel_3_xmb_seq.reg32, value);
        s->ahbdmachan_channel_3_xmb_seq.reg32 = value;
        break;
    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }
}

static void tegra_ahb_dma_priv_reset(DeviceState *dev)
{
    tegra_ahb_dma *s = TEGRA_AHB_DMA(dev);

    s->cmd.reg32 = CMD_RESET;
    s->sta.reg32 = STA_RESET;
    s->tx_req.reg32 = TX_REQ_RESET;
    s->counter.reg32 = COUNTER_RESET;
    s->irq_sta_cpu.reg32 = IRQ_STA_CPU_RESET;
    s->irq_sta_cop.reg32 = IRQ_STA_COP_RESET;
    s->irq_mask.reg32 = IRQ_MASK_RESET;
    s->irq_mask_set.reg32 = IRQ_MASK_SET_RESET;
    s->irq_mask_clr.reg32 = IRQ_MASK_CLR_RESET;
    s->rdwr_coherency.reg32 = RDWR_COHERENCY_RESET;
    s->test_bus.reg32 = TEST_BUS_RESET;
    s->ppcs_mccif_fifoctrl.reg32 = PPCS_MCCIF_FIFOCTRL_RESET;
    s->timeout_wcoal_ppcs.reg32 = TIMEOUT_WCOAL_PPCS_RESET;
    s->ahbdmachan_channel_0_csr.reg32 = AHBDMACHAN_CHANNEL_0_CSR_RESET;
    s->ahbdmachan_channel_0_sta.reg32 = AHBDMACHAN_CHANNEL_0_STA_RESET;
    s->ahbdmachan_channel_0_ahb_ptr.reg32 = AHBDMACHAN_CHANNEL_0_AHB_PTR_RESET;
    s->ahbdmachan_channel_0_ahb_seq.reg32 = AHBDMACHAN_CHANNEL_0_AHB_SEQ_RESET;
    s->ahbdmachan_channel_0_xmb_ptr.reg32 = AHBDMACHAN_CHANNEL_0_XMB_PTR_RESET;
    s->ahbdmachan_channel_0_xmb_seq.reg32 = AHBDMACHAN_CHANNEL_0_XMB_SEQ_RESET;
    s->ahbdmachan_channel_1_csr.reg32 = AHBDMACHAN_CHANNEL_1_CSR_RESET;
    s->ahbdmachan_channel_1_sta.reg32 = AHBDMACHAN_CHANNEL_1_STA_RESET;
    s->ahbdmachan_channel_1_ahb_ptr.reg32 = AHBDMACHAN_CHANNEL_1_AHB_PTR_RESET;
    s->ahbdmachan_channel_1_ahb_seq.reg32 = AHBDMACHAN_CHANNEL_1_AHB_SEQ_RESET;
    s->ahbdmachan_channel_1_xmb_ptr.reg32 = AHBDMACHAN_CHANNEL_1_XMB_PTR_RESET;
    s->ahbdmachan_channel_1_xmb_seq.reg32 = AHBDMACHAN_CHANNEL_1_XMB_SEQ_RESET;
    s->ahbdmachan_channel_2_csr.reg32 = AHBDMACHAN_CHANNEL_2_CSR_RESET;
    s->ahbdmachan_channel_2_sta.reg32 = AHBDMACHAN_CHANNEL_2_STA_RESET;
    s->ahbdmachan_channel_2_ahb_ptr.reg32 = AHBDMACHAN_CHANNEL_2_AHB_PTR_RESET;
    s->ahbdmachan_channel_2_ahb_seq.reg32 = AHBDMACHAN_CHANNEL_2_AHB_SEQ_RESET;
    s->ahbdmachan_channel_2_xmb_ptr.reg32 = AHBDMACHAN_CHANNEL_2_XMB_PTR_RESET;
    s->ahbdmachan_channel_2_xmb_seq.reg32 = AHBDMACHAN_CHANNEL_2_XMB_SEQ_RESET;
    s->ahbdmachan_channel_3_csr.reg32 = AHBDMACHAN_CHANNEL_3_CSR_RESET;
    s->ahbdmachan_channel_3_sta.reg32 = AHBDMACHAN_CHANNEL_3_STA_RESET;
    s->ahbdmachan_channel_3_ahb_ptr.reg32 = AHBDMACHAN_CHANNEL_3_AHB_PTR_RESET;
    s->ahbdmachan_channel_3_ahb_seq.reg32 = AHBDMACHAN_CHANNEL_3_AHB_SEQ_RESET;
    s->ahbdmachan_channel_3_xmb_ptr.reg32 = AHBDMACHAN_CHANNEL_3_XMB_PTR_RESET;
    s->ahbdmachan_channel_3_xmb_seq.reg32 = AHBDMACHAN_CHANNEL_3_XMB_SEQ_RESET;
}

static const MemoryRegionOps tegra_ahb_dma_mem_ops = {
    .read = tegra_ahb_dma_priv_read,
    .write = tegra_ahb_dma_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_ahb_dma_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_ahb_dma *s = TEGRA_AHB_DMA(dev);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_ahb_dma_mem_ops, s,
                          "tegra.ahb_dma", TEGRA_AHB_DMA_SIZE * TEGRA_AHB_DMA_CH0_SIZE*4);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_ahb_dma_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_ahb_dma_priv_realize;
    dc->vmsd = &vmstate_tegra_ahb_dma;
    dc->reset = tegra_ahb_dma_priv_reset;
}

static const TypeInfo tegra_ahb_dma_info = {
    .name = TYPE_TEGRA_AHB_DMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_ahb_dma),
    .class_init = tegra_ahb_dma_class_init,
};

static void tegra_ahb_dma_register_types(void)
{
    type_register_static(&tegra_ahb_dma_info);
}

type_init(tegra_ahb_dma_register_types)
