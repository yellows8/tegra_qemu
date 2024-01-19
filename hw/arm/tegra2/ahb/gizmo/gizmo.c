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

#include "gizmo.h"
#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_AHB_GIZMO "tegra.ahb_gizmo"
#define TEGRA_AHB_GIZMO(obj) OBJECT_CHECK(tegra_ahb_gizmo, (obj), TYPE_TEGRA_AHB_GIZMO)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_ahb_gizmo_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t ahb_avpc_mccif_fifoctrl_offset;
    uint32_t ahb_timeout_wcoal_avpc_offset;
    DEFINE_REG32(ahb_mem);
    DEFINE_REG32(apb_dma);
    DEFINE_REG32(ahb_master_swid_0);
    DEFINE_REG32(ide);
    DEFINE_REG32(usb);
    DEFINE_REG32(ahb_xbar_bridge);
    DEFINE_REG32(cpu_ahb_bridge);
    DEFINE_REG32(cop_ahb_bridge);
    DEFINE_REG32(xbar_apb_ctlr);
    DEFINE_REG32(vcp_ahb_bridge);
    DEFINE_REG32(ahb_master_swid_1);
    DEFINE_REG32(nand);
    DEFINE_REG32(sdmmc4);
    DEFINE_REG32(xio);
    DEFINE_REG32(tzram);
    DEFINE_REG32(se);
    DEFINE_REG32(bsev);
    DEFINE_REG32(bsea);
    DEFINE_REG32(nor);
    DEFINE_REG32(usb2);
    DEFINE_REG32(usb3);
    DEFINE_REG32(sdmmc1);
    DEFINE_REG32(sdmmc2);
    DEFINE_REG32(sdmmc3);
    DEFINE_REG32(arc);
    DEFINE_REG32(wrq_empty);
    DEFINE_REG32(mem_prefetch_cfg5);
    DEFINE_REG32(mem_prefetch_cfg6);
    DEFINE_REG32(mem_prefetch_cfg7);
    DEFINE_REG32(mem_prefetch_cfg8);
    DEFINE_REG32(mem_prefetch_cfg_x);
    DEFINE_REG32(ahb_arbitration_xbar_ctrl);
    DEFINE_REG32(mem_prefetch_cfg3);
    DEFINE_REG32(mem_prefetch_cfg4);
    DEFINE_REG32(ahb_avp_ppcs_rd_coh_status);
    DEFINE_REG32(mem_prefetch_cfg1);
    DEFINE_REG32(mem_prefetch_cfg2);
    DEFINE_REG32(ahb_ahbslvmem_status);
    DEFINE_REG32(ahb_arbitration_ahb_mem_wrque_mst_id);
    DEFINE_REG32(ahb_arbitration_cpu_abort_info);
    DEFINE_REG32(ahb_arbitration_cpu_abort_addr);
    DEFINE_REG32(ahb_arbitration_cop_abort_info);
    DEFINE_REG32(ahb_arbitration_cop_abort_addr);
    DEFINE_REG32(ahb_spare_reg);
    DEFINE_REG32(ahb_avpc_mccif_fifoctrl);
    DEFINE_REG32(ahb_timeout_wcoal_avpc);
    DEFINE_REG32(ahb_mpcore_mccif_fifoctrl);
    DEFINE_REG32(ahb_arbitration_disable);
    DEFINE_REG32(ahb_arbitration_priority_ctrl);
    DEFINE_REG32(ahb_arbitration_usr_protect);
} tegra_ahb_gizmo;

static const VMStateDescription vmstate_tegra_ahb_gizmo = {
    .name = "tegra.ahb_gizmo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(ahb_avpc_mccif_fifoctrl_offset, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_timeout_wcoal_avpc_offset, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_mem.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(apb_dma.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_master_swid_0.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ide.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(usb.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_xbar_bridge.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(cpu_ahb_bridge.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(cop_ahb_bridge.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(xbar_apb_ctlr.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(vcp_ahb_bridge.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_master_swid_1.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(nand.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(sdmmc4.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(xio.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(tzram.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(se.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(bsev.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(bsea.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(nor.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(usb2.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(usb3.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(sdmmc1.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(sdmmc2.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(sdmmc3.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(arc.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(wrq_empty.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(mem_prefetch_cfg5.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(mem_prefetch_cfg6.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(mem_prefetch_cfg7.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(mem_prefetch_cfg8.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(mem_prefetch_cfg_x.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_arbitration_xbar_ctrl.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(mem_prefetch_cfg3.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(mem_prefetch_cfg4.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_avp_ppcs_rd_coh_status.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(mem_prefetch_cfg1.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(mem_prefetch_cfg2.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_ahbslvmem_status.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_arbitration_ahb_mem_wrque_mst_id.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_arbitration_cpu_abort_info.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_arbitration_cpu_abort_addr.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_arbitration_cop_abort_info.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_arbitration_cop_abort_addr.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_spare_reg.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_avpc_mccif_fifoctrl.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_timeout_wcoal_avpc.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_mpcore_mccif_fifoctrl.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_arbitration_disable.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_arbitration_priority_ctrl.reg32, tegra_ahb_gizmo),
        VMSTATE_UINT32(ahb_arbitration_usr_protect.reg32, tegra_ahb_gizmo),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_ahb_gizmo_priv_read(void *opaque, hwaddr offset,
                                          unsigned size)
{
    tegra_ahb_gizmo *s = opaque;
    uint64_t ret = 0;

    switch (offset) {
    case AHB_MEM_OFFSET:
        ret = s->ahb_mem.reg32;
        break;
    case APB_DMA_OFFSET:
        ret = s->apb_dma.reg32;
        break;
    case AHB_MASTER_SWID_0_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->ahb_master_swid_0.reg32;
        break;
    case IDE_OFFSET:
        ret = s->ide.reg32;
        break;
    case USB_OFFSET:
        ret = s->usb.reg32;
        break;
    case AHB_XBAR_BRIDGE_OFFSET:
        ret = s->ahb_xbar_bridge.reg32;
        break;
    case CPU_AHB_BRIDGE_OFFSET:
        ret = s->cpu_ahb_bridge.reg32;
        break;
    case COP_AHB_BRIDGE_OFFSET:
        ret = s->cop_ahb_bridge.reg32;
        break;
    case XBAR_APB_CTLR_OFFSET:
        ret = s->xbar_apb_ctlr.reg32;
        break;
    case VCP_AHB_BRIDGE_OFFSET:
        ret = s->vcp_ahb_bridge.reg32;
        break;
    case AHB_MASTER_SWID_1_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->ahb_master_swid_1.reg32;
        break;
    case NAND_OFFSET:
        ret = s->nand.reg32;
        break;
    case SDMMC4_OFFSET:
        ret = s->sdmmc4.reg32;
        break;
    case XIO_OFFSET:
        ret = s->xio.reg32;
        break;
    case TZRAM_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->tzram.reg32;
        break;
    case SE_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->se.reg32;
        break;
    case BSEV_OFFSET:
        ret = s->bsev.reg32;
        break;
    case BSEA_OFFSET:
        ret = s->bsea.reg32;
        break;
    case NOR_OFFSET:
        ret = s->nor.reg32;
        break;
    case USB2_OFFSET:
        ret = s->usb2.reg32;
        break;
    case USB3_OFFSET:
        ret = s->usb3.reg32;
        break;
    case SDMMC1_OFFSET:
        ret = s->sdmmc1.reg32;
        break;
    case SDMMC2_OFFSET:
        ret = s->sdmmc2.reg32;
        break;
    case SDMMC3_OFFSET:
        ret = s->sdmmc3.reg32;
        break;
    case ARC_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->arc.reg32;
        break;
    case WRQ_EMPTY_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->wrq_empty.reg32;
        break;
    case MEM_PREFETCH_CFG5_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->mem_prefetch_cfg5.reg32;
        break;
    case MEM_PREFETCH_CFG6_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->mem_prefetch_cfg6.reg32;
        break;
    case MEM_PREFETCH_CFG7_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->mem_prefetch_cfg7.reg32;
        break;
    case MEM_PREFETCH_CFG8_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->mem_prefetch_cfg8.reg32;
        break;
    case MEM_PREFETCH_CFG_X_OFFSET:
        ret = s->mem_prefetch_cfg_x.reg32;
        break;
    case AHB_ARBITRATION_XBAR_CTRL_OFFSET:
        ret = s->ahb_arbitration_xbar_ctrl.reg32;
        break;
    case MEM_PREFETCH_CFG3_OFFSET:
        ret = s->mem_prefetch_cfg3.reg32;
        break;
    case MEM_PREFETCH_CFG4_OFFSET:
        ret = s->mem_prefetch_cfg4.reg32;
        break;
    case AHB_AVP_PPCS_RD_COH_STATUS_OFFSET:
        ret = s->ahb_avp_ppcs_rd_coh_status.reg32;
        break;
    case MEM_PREFETCH_CFG1_OFFSET:
        ret = s->mem_prefetch_cfg1.reg32;
        break;
    case MEM_PREFETCH_CFG2_OFFSET:
        ret = s->mem_prefetch_cfg2.reg32;
        break;
    case AHB_AHBSLVMEM_STATUS_OFFSET:
        ret = s->ahb_ahbslvmem_status.reg32;
        break;
    case AHB_ARBITRATION_AHB_MEM_WRQUE_MST_ID_OFFSET:
        ret = s->ahb_arbitration_ahb_mem_wrque_mst_id.reg32;
        break;
    case AHB_ARBITRATION_CPU_ABORT_INFO_OFFSET:
        ret = s->ahb_arbitration_cpu_abort_info.reg32;
        break;
    case AHB_ARBITRATION_CPU_ABORT_ADDR_OFFSET:
        ret = s->ahb_arbitration_cpu_abort_addr.reg32;
        break;
    case AHB_ARBITRATION_COP_ABORT_INFO_OFFSET:
        ret = s->ahb_arbitration_cop_abort_info.reg32;
        break;
    case AHB_ARBITRATION_COP_ABORT_ADDR_OFFSET:
        ret = s->ahb_arbitration_cop_abort_addr.reg32;
        break;
    case AHB_SPARE_REG_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->ahb_spare_reg.reg32;
        break;
    case AHB_MPCORE_MCCIF_FIFOCTRL_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->ahb_mpcore_mccif_fifoctrl.reg32;
        break;
    case AHB_ARBITRATION_DISABLE_OFFSET:
        ret = s->ahb_arbitration_disable.reg32;
        break;
    case AHB_ARBITRATION_PRIORITY_CTRL_OFFSET:
        ret = s->ahb_arbitration_priority_ctrl.reg32;
        break;
    case AHB_ARBITRATION_USR_PROTECT_OFFSET:
        ret = s->ahb_arbitration_usr_protect.reg32;
        break;
    default:
        break;
    }

    if (offset == s->ahb_avpc_mccif_fifoctrl_offset) {
        ret = s->ahb_avpc_mccif_fifoctrl.reg32;
    }
    else if (offset == s->ahb_timeout_wcoal_avpc_offset) {
        ret = s->ahb_timeout_wcoal_avpc.reg32;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_ahb_gizmo_priv_write(void *opaque, hwaddr offset,
                                       uint64_t value, unsigned size)
{
    tegra_ahb_gizmo *s = opaque;

    switch (offset) {
    case AHB_MEM_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahb_mem.reg32, value);
        s->ahb_mem.reg32 = value;
        break;
    case APB_DMA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->apb_dma.reg32, value);
        s->apb_dma.reg32 = value;
        break;
    case AHB_MASTER_SWID_0_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahb_master_swid_0.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->ahb_master_swid_0.reg32 = value;
        break;
    case IDE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ide.reg32, value);
        s->ide.reg32 = value;
        break;
    case USB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->usb.reg32, value);
        s->usb.reg32 = value;
        break;
    case AHB_XBAR_BRIDGE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahb_xbar_bridge.reg32, value);
        s->ahb_xbar_bridge.reg32 = value;
        break;
    case CPU_AHB_BRIDGE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->cpu_ahb_bridge.reg32, value);
        s->cpu_ahb_bridge.reg32 = value;
        break;
    case COP_AHB_BRIDGE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->cop_ahb_bridge.reg32, value);
        s->cop_ahb_bridge.reg32 = value;
        break;
    case XBAR_APB_CTLR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->xbar_apb_ctlr.reg32, value);
        s->xbar_apb_ctlr.reg32 = value;
        break;
    case VCP_AHB_BRIDGE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->vcp_ahb_bridge.reg32, value);
        s->vcp_ahb_bridge.reg32 = value;
        break;
    case AHB_MASTER_SWID_1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahb_master_swid_1.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->ahb_master_swid_1.reg32 = value;
        break;
    case NAND_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->nand.reg32, value);
        s->nand.reg32 = value;
        break;
    case SDMMC4_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->sdmmc4.reg32, value);
        s->sdmmc4.reg32 = value;
        break;
    case XIO_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->xio.reg32, value);
        s->xio.reg32 = value;
        break;
    case TZRAM_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->tzram.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->tzram.reg32 = value;
        break;
    case SE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->se.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->se.reg32 = value;
        break;
    case BSEV_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bsev.reg32, value);
        s->bsev.reg32 = value;
        break;
    case BSEA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bsea.reg32, value);
        s->bsea.reg32 = value;
        break;
    case NOR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->nor.reg32, value);
        s->nor.reg32 = value;
        break;
    case USB2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->usb2.reg32, value);
        s->usb2.reg32 = value;
        break;
    case USB3_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->usb3.reg32, value);
        s->usb3.reg32 = value;
        break;
    case SDMMC1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->sdmmc1.reg32, value);
        s->sdmmc1.reg32 = value;
        break;
    case SDMMC2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->sdmmc2.reg32, value);
        s->sdmmc2.reg32 = value;
        break;
    case SDMMC3_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->sdmmc3.reg32, value);
        s->sdmmc3.reg32 = value;
        break;
    case ARC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->arc.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->arc.reg32 = value;
        break;
    case WRQ_EMPTY_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->wrq_empty.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->wrq_empty.reg32 = value;
        break;
    case MEM_PREFETCH_CFG5_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->mem_prefetch_cfg5.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->mem_prefetch_cfg5.reg32 = value;
        break;
    case MEM_PREFETCH_CFG6_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->mem_prefetch_cfg6.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->mem_prefetch_cfg6.reg32 = value;
        break;
    case MEM_PREFETCH_CFG7_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->mem_prefetch_cfg7.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->mem_prefetch_cfg7.reg32 = value;
        break;
    case MEM_PREFETCH_CFG8_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->mem_prefetch_cfg8.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->mem_prefetch_cfg8.reg32 = value;
        break;
    case MEM_PREFETCH_CFG_X_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->mem_prefetch_cfg_x.reg32, value);
        s->mem_prefetch_cfg_x.reg32 = value;
        break;
    case AHB_ARBITRATION_XBAR_CTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahb_arbitration_xbar_ctrl.reg32, value);
        s->ahb_arbitration_xbar_ctrl.reg32 = value;
        break;
    case MEM_PREFETCH_CFG3_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->mem_prefetch_cfg3.reg32, value & MEM_PREFETCH_CFG3_WRMASK);
        WR_MASKED(s->mem_prefetch_cfg3.reg32, value, MEM_PREFETCH_CFG3);
        break;
    case MEM_PREFETCH_CFG4_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->mem_prefetch_cfg4.reg32, value & MEM_PREFETCH_CFG4_WRMASK);
        WR_MASKED(s->mem_prefetch_cfg4.reg32, value, MEM_PREFETCH_CFG4);
        break;
    case MEM_PREFETCH_CFG1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->mem_prefetch_cfg1.reg32, value & MEM_PREFETCH_CFG1_WRMASK);
        WR_MASKED(s->mem_prefetch_cfg1.reg32, value, MEM_PREFETCH_CFG1);
        break;
    case MEM_PREFETCH_CFG2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->mem_prefetch_cfg2.reg32, value & MEM_PREFETCH_CFG2_WRMASK);
        WR_MASKED(s->mem_prefetch_cfg2.reg32, value, MEM_PREFETCH_CFG2);
        break;
    case AHB_ARBITRATION_AHB_MEM_WRQUE_MST_ID_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahb_arbitration_ahb_mem_wrque_mst_id.reg32, value);
        s->ahb_arbitration_ahb_mem_wrque_mst_id.reg32 = value;
        break;
    case AHB_SPARE_REG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahb_spare_reg.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->ahb_spare_reg.reg32 = value;
        break;
    case AHB_MPCORE_MCCIF_FIFOCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahb_mpcore_mccif_fifoctrl.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->ahb_mpcore_mccif_fifoctrl.reg32 = value;
        break;
    case AHB_ARBITRATION_DISABLE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahb_arbitration_disable.reg32, value);
        s->ahb_arbitration_disable.reg32 = value;
        break;
    case AHB_ARBITRATION_PRIORITY_CTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahb_arbitration_priority_ctrl.reg32, value);
        s->ahb_arbitration_priority_ctrl.reg32 = value;
        break;
    case AHB_ARBITRATION_USR_PROTECT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ahb_arbitration_usr_protect.reg32, value & AHB_ARBITRATION_USR_PROTECT_WRMASK);
        WR_MASKED(s->ahb_arbitration_usr_protect.reg32, value, AHB_ARBITRATION_USR_PROTECT);
        break;
    default:
        if (offset == s->ahb_avpc_mccif_fifoctrl_offset) {
            TRACE_WRITE(s->iomem.addr, offset, s->ahb_avpc_mccif_fifoctrl.reg32, value);
            s->ahb_avpc_mccif_fifoctrl.reg32 = value;
        }
        else if (offset == s->ahb_timeout_wcoal_avpc_offset) {
            TRACE_WRITE(s->iomem.addr, offset, s->ahb_timeout_wcoal_avpc.reg32, value);
            s->ahb_timeout_wcoal_avpc.reg32 = value;
        }
        else TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }
}

static void tegra_ahb_gizmo_priv_reset(DeviceState *dev)
{
    tegra_ahb_gizmo *s = TEGRA_AHB_GIZMO(dev);

    s->ahb_mem.reg32 = AHB_MEM_RESET;
    s->apb_dma.reg32 = APB_DMA_RESET;
    s->ahb_master_swid_0.reg32 = AHB_MASTER_SWID_0_RESET;
    s->ide.reg32 = IDE_RESET;
    s->usb.reg32 = USB_RESET;
    s->ahb_xbar_bridge.reg32 = AHB_XBAR_BRIDGE_RESET;
    s->cpu_ahb_bridge.reg32 = CPU_AHB_BRIDGE_RESET;
    s->cop_ahb_bridge.reg32 = COP_AHB_BRIDGE_RESET;
    s->xbar_apb_ctlr.reg32 = XBAR_APB_CTLR_RESET;
    s->vcp_ahb_bridge.reg32 = VCP_AHB_BRIDGE_RESET;
    s->ahb_master_swid_1.reg32 = AHB_MASTER_SWID_1_RESET;
    s->nand.reg32 = NAND_RESET;
    s->sdmmc4.reg32 = SDMMC4_RESET;
    s->xio.reg32 = XIO_RESET;
    s->tzram.reg32 = TZRAM_RESET;
    s->se.reg32 = SE_RESET;
    s->bsev.reg32 = BSEV_RESET;
    s->bsea.reg32 = BSEA_RESET;
    s->nor.reg32 = NOR_RESET;
    s->usb2.reg32 = USB2_RESET;
    s->usb3.reg32 = USB3_RESET;
    s->sdmmc1.reg32 = SDMMC1_RESET;
    s->sdmmc2.reg32 = SDMMC2_RESET;
    s->sdmmc3.reg32 = SDMMC3_RESET;
    s->arc.reg32 = ARC_RESET;
    s->wrq_empty.reg32 = WRQ_EMPTY_RESET;
    s->mem_prefetch_cfg5.reg32 = MEM_PREFETCH_CFG5_RESET;
    s->mem_prefetch_cfg6.reg32 = MEM_PREFETCH_CFG6_RESET;
    s->mem_prefetch_cfg7.reg32 = MEM_PREFETCH_CFG7_RESET;
    s->mem_prefetch_cfg8.reg32 = MEM_PREFETCH_CFG8_RESET;
    s->mem_prefetch_cfg_x.reg32 = MEM_PREFETCH_CFG_X_RESET;
    s->ahb_arbitration_xbar_ctrl.reg32 = AHB_ARBITRATION_XBAR_CTRL_RESET;
    s->mem_prefetch_cfg3.reg32 = MEM_PREFETCH_CFG3_RESET;
    s->mem_prefetch_cfg4.reg32 = MEM_PREFETCH_CFG4_RESET;
    s->ahb_avp_ppcs_rd_coh_status.reg32 = AHB_AVP_PPCS_RD_COH_STATUS_RESET;
    s->mem_prefetch_cfg1.reg32 = MEM_PREFETCH_CFG1_RESET;
    s->mem_prefetch_cfg2.reg32 = MEM_PREFETCH_CFG2_RESET;
    s->ahb_ahbslvmem_status.reg32 = AHB_AHBSLVMEM_STATUS_RESET;
    s->ahb_arbitration_ahb_mem_wrque_mst_id.reg32 = AHB_ARBITRATION_AHB_MEM_WRQUE_MST_ID_RESET;
    s->ahb_arbitration_cpu_abort_info.reg32 = AHB_ARBITRATION_CPU_ABORT_INFO_RESET;
    s->ahb_arbitration_cpu_abort_addr.reg32 = AHB_ARBITRATION_CPU_ABORT_ADDR_RESET;
    s->ahb_arbitration_cop_abort_info.reg32 = AHB_ARBITRATION_COP_ABORT_INFO_RESET;
    s->ahb_arbitration_cop_abort_addr.reg32 = AHB_ARBITRATION_COP_ABORT_ADDR_RESET;
    s->ahb_spare_reg.reg32 = AHB_SPARE_REG_RESET;
    s->ahb_avpc_mccif_fifoctrl.reg32 = AHB_AVPC_MCCIF_FIFOCTRL_RESET;
    s->ahb_timeout_wcoal_avpc.reg32 = AHB_TIMEOUT_WCOAL_AVPC_RESET;
    s->ahb_mpcore_mccif_fifoctrl.reg32 = AHB_MPCORE_MCCIF_FIFOCTRL_RESET;
    s->ahb_arbitration_disable.reg32 = AHB_ARBITRATION_DISABLE_RESET;
    s->ahb_arbitration_priority_ctrl.reg32 = AHB_ARBITRATION_PRIORITY_CTRL_RESET;
    s->ahb_arbitration_usr_protect.reg32 = AHB_ARBITRATION_USR_PROTECT_RESET;

    if (tegra_board >= TEGRAX1_BOARD) {
        s->ahb_avpc_mccif_fifoctrl_offset = AHB_AVPC_MCCIF_FIFOCTRL_TEGRAX1_OFFSET;
        s->ahb_timeout_wcoal_avpc_offset = AHB_TIMEOUT_WCOAL_AVPC_TEGRAX1_OFFSET;
    }
    else {
        s->ahb_avpc_mccif_fifoctrl_offset = AHB_AVPC_MCCIF_FIFOCTRL_TEGRA2_OFFSET;
        s->ahb_timeout_wcoal_avpc_offset = AHB_TIMEOUT_WCOAL_AVPC_TEGRA2_OFFSET;
    }
}

static const MemoryRegionOps tegra_ahb_gizmo_mem_ops = {
    .read = tegra_ahb_gizmo_priv_read,
    .write = tegra_ahb_gizmo_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_ahb_gizmo_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_ahb_gizmo *s = TEGRA_AHB_GIZMO(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_ahb_gizmo_mem_ops, s,
                          "tegra.ahb_gizmo", TEGRA_AHB_GIZMO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_ahb_gizmo_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_ahb_gizmo_priv_realize;
    dc->vmsd = &vmstate_tegra_ahb_gizmo;
    dc->reset = tegra_ahb_gizmo_priv_reset;
}

static const TypeInfo tegra_ahb_gizmo_info = {
    .name = TYPE_TEGRA_AHB_GIZMO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_ahb_gizmo),
    .class_init = tegra_ahb_gizmo_class_init,
};

static void tegra_ahb_gizmo_register_types(void)
{
    type_register_static(&tegra_ahb_gizmo_info);
}

type_init(tegra_ahb_gizmo_register_types)
