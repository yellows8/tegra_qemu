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

#include "mc.h"
#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_MC "tegra.mc"
#define TEGRA_MC(obj) OBJECT_CHECK(tegra_mc, (obj), TYPE_TEGRA_MC)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_mc_state {
    SysBusDevice parent_obj;

    uint32_t ram_size_kb;
    MemoryRegion iomem;
    uint32_t emem_cfg_offset;
    DEFINE_REG32(emem_cfg);
    DEFINE_REG32(emem_adr_cfg);
    DEFINE_REG32(emem_arb_cfg0);
    DEFINE_REG32(emem_arb_cfg1);
    DEFINE_REG32(emem_arb_cfg2);
    DEFINE_REG32(gart_config);
    DEFINE_REG32(gart_entry_addr);
    DEFINE_REG32(gart_entry_data);
    DEFINE_REG32(gart_error_req);
    DEFINE_REG32(gart_error_addr);
    DEFINE_REG32(timeout_ctrl);
    DEFINE_REG32(decerr_emem_others_status);
    DEFINE_REG32(decerr_emem_others_adr);
    DEFINE_REG32(client_ctrl);
    DEFINE_REG32(client_hotresetn);
    DEFINE_REG32(lowlatency_rawlogic_write_participants);
    DEFINE_REG32(bwshare_tmval);
    DEFINE_REG32(bwshare_emem_ctrl_0);
    DEFINE_REG32(bwshare_emem_ctrl_1);
    DEFINE_REG32(avpc_orrc);
    DEFINE_REG32(dc_orrc);
    DEFINE_REG32(dcb_orrc);
    DEFINE_REG32(epp_orrc);
    DEFINE_REG32(g2_orrc);
    DEFINE_REG32(hc_orrc);
    DEFINE_REG32(isp_orrc);
    DEFINE_REG32(mpcore_orrc);
    DEFINE_REG32(mpea_orrc);
    DEFINE_REG32(mpeb_orrc);
    DEFINE_REG32(mpec_orrc);
    DEFINE_REG32(nv_orrc);
    DEFINE_REG32(ppcs_orrc);
    DEFINE_REG32(vde_orrc);
    DEFINE_REG32(vi_orrc);
    DEFINE_REG32(fpri_ctrl_avpc);
    DEFINE_REG32(fpri_ctrl_dc);
    DEFINE_REG32(fpri_ctrl_dcb);
    DEFINE_REG32(fpri_ctrl_epp);
    DEFINE_REG32(fpri_ctrl_g2);
    DEFINE_REG32(fpri_ctrl_hc);
    DEFINE_REG32(fpri_ctrl_isp);
    DEFINE_REG32(fpri_ctrl_mpcore);
    DEFINE_REG32(fpri_ctrl_mpea);
    DEFINE_REG32(fpri_ctrl_mpeb);
    DEFINE_REG32(fpri_ctrl_mpec);
    DEFINE_REG32(fpri_ctrl_nv);
    DEFINE_REG32(fpri_ctrl_ppcs);
    DEFINE_REG32(fpri_ctrl_vde);
    DEFINE_REG32(fpri_ctrl_vi);
    DEFINE_REG32(timeout_avpc);
    DEFINE_REG32(timeout_dc);
    DEFINE_REG32(timeout_dcb);
    DEFINE_REG32(timeout_epp);
    DEFINE_REG32(timeout_g2);
    DEFINE_REG32(timeout_hc);
    DEFINE_REG32(timeout_isp);
    DEFINE_REG32(timeout_mpcore);
    DEFINE_REG32(timeout_mpea);
    DEFINE_REG32(timeout_mpeb);
    DEFINE_REG32(timeout_mpec);
    DEFINE_REG32(timeout_nv);
    DEFINE_REG32(timeout_ppcs);
    DEFINE_REG32(timeout_vde);
    DEFINE_REG32(timeout_vi);
    DEFINE_REG32(timeout_rcoal_avpc);
    DEFINE_REG32(timeout_rcoal_dc);
    DEFINE_REG32(timeout1_rcoal_dc);
    DEFINE_REG32(timeout_rcoal_dcb);
    DEFINE_REG32(timeout1_rcoal_dcb);
    DEFINE_REG32(timeout_rcoal_epp);
    DEFINE_REG32(timeout_rcoal_g2);
    DEFINE_REG32(timeout_rcoal_hc);
    DEFINE_REG32(timeout_rcoal_mpcore);
    DEFINE_REG32(timeout_rcoal_mpea);
    DEFINE_REG32(timeout_rcoal_mpeb);
    DEFINE_REG32(timeout_rcoal_mpec);
    DEFINE_REG32(timeout_rcoal_nv);
    DEFINE_REG32(timeout_rcoal_ppcs);
    DEFINE_REG32(timeout_rcoal_vde);
    DEFINE_REG32(timeout_rcoal_vi);
    DEFINE_REG32(rcoal_autodisable);
    DEFINE_REG32(bwshare_avpc);
    DEFINE_REG32(bwshare_dc);
    DEFINE_REG32(bwshare_dcb);
    DEFINE_REG32(bwshare_epp);
    DEFINE_REG32(bwshare_g2);
    DEFINE_REG32(bwshare_hc);
    DEFINE_REG32(bwshare_isp);
    DEFINE_REG32(bwshare_mpcore);
    DEFINE_REG32(bwshare_mpea);
    DEFINE_REG32(bwshare_mpeb);
    DEFINE_REG32(bwshare_mpec);
    DEFINE_REG32(bwshare_nv);
    DEFINE_REG32(bwshare_ppcs);
    DEFINE_REG32(bwshare_vde);
    DEFINE_REG32(bwshare_vi);
    DEFINE_REG32(intstatus);
    DEFINE_REG32(intmask);
    DEFINE_REG32(clken_override);
    DEFINE_REG32(security_cfg0);
    DEFINE_REG32(security_cfg1);
    DEFINE_REG32(security_violation_status);
    DEFINE_REG32(security_violation_adr);
    DEFINE_REG32(security_cfg2);
    DEFINE_REG32(stat_control);
    DEFINE_REG32(stat_status);
    DEFINE_REG32(stat_emc_addr_low);
    DEFINE_REG32(stat_emc_addr_high);
    DEFINE_REG32(stat_emc_clock_limit);
    DEFINE_REG32(stat_emc_clocks);
    DEFINE_REG32(stat_emc_control_0);
    DEFINE_REG32(stat_emc_control_1);
    DEFINE_REG32(stat_emc_hist_limit_0);
    DEFINE_REG32(stat_emc_hist_limit_1);
    DEFINE_REG32(stat_emc_count_0);
    DEFINE_REG32(stat_emc_count_1);
    DEFINE_REG32(stat_emc_hist_0);
    DEFINE_REG32(stat_emc_hist_1);
    DEFINE_REG32(axi_decerr_ovr);
    DEFINE_REG32(lowlatency_config);
    DEFINE_REG32(ap_ctrl_0);
    DEFINE_REG32(ap_ctrl_1);
    DEFINE_REG32(client_activity_monitor_emem_0);
    DEFINE_REG32(client_activity_monitor_emem_1);
    uint32_t regs[(0xC00-0x274)>>2];
} tegra_mc;

static const VMStateDescription vmstate_tegra_mc = {
    .name = "tegra.mc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(emem_cfg_offset, tegra_mc),
        VMSTATE_UINT32(emem_cfg.reg32, tegra_mc),
        VMSTATE_UINT32(emem_adr_cfg.reg32, tegra_mc),
        VMSTATE_UINT32(emem_arb_cfg0.reg32, tegra_mc),
        VMSTATE_UINT32(emem_arb_cfg1.reg32, tegra_mc),
        VMSTATE_UINT32(emem_arb_cfg2.reg32, tegra_mc),
        VMSTATE_UINT32(gart_config.reg32, tegra_mc),
        VMSTATE_UINT32(gart_entry_addr.reg32, tegra_mc),
        VMSTATE_UINT32(gart_entry_data.reg32, tegra_mc),
        VMSTATE_UINT32(gart_error_req.reg32, tegra_mc),
        VMSTATE_UINT32(gart_error_addr.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_ctrl.reg32, tegra_mc),
        VMSTATE_UINT32(decerr_emem_others_status.reg32, tegra_mc),
        VMSTATE_UINT32(decerr_emem_others_adr.reg32, tegra_mc),
        VMSTATE_UINT32(client_ctrl.reg32, tegra_mc),
        VMSTATE_UINT32(client_hotresetn.reg32, tegra_mc),
        VMSTATE_UINT32(lowlatency_rawlogic_write_participants.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_tmval.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_emem_ctrl_0.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_emem_ctrl_1.reg32, tegra_mc),
        VMSTATE_UINT32(avpc_orrc.reg32, tegra_mc),
        VMSTATE_UINT32(dc_orrc.reg32, tegra_mc),
        VMSTATE_UINT32(dcb_orrc.reg32, tegra_mc),
        VMSTATE_UINT32(epp_orrc.reg32, tegra_mc),
        VMSTATE_UINT32(g2_orrc.reg32, tegra_mc),
        VMSTATE_UINT32(hc_orrc.reg32, tegra_mc),
        VMSTATE_UINT32(isp_orrc.reg32, tegra_mc),
        VMSTATE_UINT32(mpcore_orrc.reg32, tegra_mc),
        VMSTATE_UINT32(mpea_orrc.reg32, tegra_mc),
        VMSTATE_UINT32(mpeb_orrc.reg32, tegra_mc),
        VMSTATE_UINT32(mpec_orrc.reg32, tegra_mc),
        VMSTATE_UINT32(nv_orrc.reg32, tegra_mc),
        VMSTATE_UINT32(ppcs_orrc.reg32, tegra_mc),
        VMSTATE_UINT32(vde_orrc.reg32, tegra_mc),
        VMSTATE_UINT32(vi_orrc.reg32, tegra_mc),
        VMSTATE_UINT32(fpri_ctrl_avpc.reg32, tegra_mc),
        VMSTATE_UINT32(fpri_ctrl_dc.reg32, tegra_mc),
        VMSTATE_UINT32(fpri_ctrl_dcb.reg32, tegra_mc),
        VMSTATE_UINT32(fpri_ctrl_epp.reg32, tegra_mc),
        VMSTATE_UINT32(fpri_ctrl_g2.reg32, tegra_mc),
        VMSTATE_UINT32(fpri_ctrl_hc.reg32, tegra_mc),
        VMSTATE_UINT32(fpri_ctrl_isp.reg32, tegra_mc),
        VMSTATE_UINT32(fpri_ctrl_mpcore.reg32, tegra_mc),
        VMSTATE_UINT32(fpri_ctrl_mpea.reg32, tegra_mc),
        VMSTATE_UINT32(fpri_ctrl_mpeb.reg32, tegra_mc),
        VMSTATE_UINT32(fpri_ctrl_mpec.reg32, tegra_mc),
        VMSTATE_UINT32(fpri_ctrl_nv.reg32, tegra_mc),
        VMSTATE_UINT32(fpri_ctrl_ppcs.reg32, tegra_mc),
        VMSTATE_UINT32(fpri_ctrl_vde.reg32, tegra_mc),
        VMSTATE_UINT32(fpri_ctrl_vi.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_avpc.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_dc.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_dcb.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_epp.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_g2.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_hc.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_isp.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_mpcore.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_mpea.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_mpeb.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_mpec.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_nv.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_ppcs.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_vde.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_vi.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_rcoal_avpc.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_rcoal_dc.reg32, tegra_mc),
        VMSTATE_UINT32(timeout1_rcoal_dc.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_rcoal_dcb.reg32, tegra_mc),
        VMSTATE_UINT32(timeout1_rcoal_dcb.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_rcoal_epp.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_rcoal_g2.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_rcoal_hc.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_rcoal_mpcore.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_rcoal_mpea.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_rcoal_mpeb.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_rcoal_mpec.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_rcoal_nv.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_rcoal_ppcs.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_rcoal_vde.reg32, tegra_mc),
        VMSTATE_UINT32(timeout_rcoal_vi.reg32, tegra_mc),
        VMSTATE_UINT32(rcoal_autodisable.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_avpc.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_dc.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_dcb.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_epp.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_g2.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_hc.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_isp.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_mpcore.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_mpea.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_mpeb.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_mpec.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_nv.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_ppcs.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_vde.reg32, tegra_mc),
        VMSTATE_UINT32(bwshare_vi.reg32, tegra_mc),
        VMSTATE_UINT32(intstatus.reg32, tegra_mc),
        VMSTATE_UINT32(intmask.reg32, tegra_mc),
        VMSTATE_UINT32(clken_override.reg32, tegra_mc),
        VMSTATE_UINT32(security_cfg0.reg32, tegra_mc),
        VMSTATE_UINT32(security_cfg1.reg32, tegra_mc),
        VMSTATE_UINT32(security_violation_status.reg32, tegra_mc),
        VMSTATE_UINT32(security_violation_adr.reg32, tegra_mc),
        VMSTATE_UINT32(security_cfg2.reg32, tegra_mc),
        VMSTATE_UINT32(stat_control.reg32, tegra_mc),
        VMSTATE_UINT32(stat_status.reg32, tegra_mc),
        VMSTATE_UINT32(stat_emc_addr_low.reg32, tegra_mc),
        VMSTATE_UINT32(stat_emc_addr_high.reg32, tegra_mc),
        VMSTATE_UINT32(stat_emc_clock_limit.reg32, tegra_mc),
        VMSTATE_UINT32(stat_emc_clocks.reg32, tegra_mc),
        VMSTATE_UINT32(stat_emc_control_0.reg32, tegra_mc),
        VMSTATE_UINT32(stat_emc_control_1.reg32, tegra_mc),
        VMSTATE_UINT32(stat_emc_hist_limit_0.reg32, tegra_mc),
        VMSTATE_UINT32(stat_emc_hist_limit_1.reg32, tegra_mc),
        VMSTATE_UINT32(stat_emc_count_0.reg32, tegra_mc),
        VMSTATE_UINT32(stat_emc_count_1.reg32, tegra_mc),
        VMSTATE_UINT32(stat_emc_hist_0.reg32, tegra_mc),
        VMSTATE_UINT32(stat_emc_hist_1.reg32, tegra_mc),
        VMSTATE_UINT32(axi_decerr_ovr.reg32, tegra_mc),
        VMSTATE_UINT32(lowlatency_config.reg32, tegra_mc),
        VMSTATE_UINT32(ap_ctrl_0.reg32, tegra_mc),
        VMSTATE_UINT32(ap_ctrl_1.reg32, tegra_mc),
        VMSTATE_UINT32(client_activity_monitor_emem_0.reg32, tegra_mc),
        VMSTATE_UINT32(client_activity_monitor_emem_1.reg32, tegra_mc),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_mc_priv_read(void *opaque, hwaddr offset,
                                   unsigned size)
{
    tegra_mc *s = opaque;
    uint64_t ret = 0;

    switch (offset) {
    case EMEM_ADR_CFG_OFFSET:
        ret = s->emem_adr_cfg.reg32;
        break;
    case EMEM_ARB_CFG0_OFFSET:
        ret = s->emem_arb_cfg0.reg32;
        break;
    case EMEM_ARB_CFG1_OFFSET:
        ret = s->emem_arb_cfg1.reg32;
        break;
    case EMEM_ARB_CFG2_OFFSET:
        ret = s->emem_arb_cfg2.reg32;
        break;
    case GART_CONFIG_OFFSET:
        ret = s->gart_config.reg32;
        break;
    case GART_ENTRY_ADDR_OFFSET:
        ret = s->gart_entry_addr.reg32;
        break;
    case GART_ENTRY_DATA_OFFSET:
        ret = s->gart_entry_data.reg32;
        break;
    case GART_ERROR_REQ_OFFSET:
        ret = s->gart_error_req.reg32;
        break;
    case GART_ERROR_ADDR_OFFSET:
        ret = s->gart_error_addr.reg32;
        break;
    case TIMEOUT_CTRL_OFFSET:
        ret = s->timeout_ctrl.reg32;
        break;
    case DECERR_EMEM_OTHERS_STATUS_OFFSET:
        ret = s->decerr_emem_others_status.reg32;
        break;
    case DECERR_EMEM_OTHERS_ADR_OFFSET:
        ret = s->decerr_emem_others_adr.reg32;
        break;
    case CLIENT_CTRL_OFFSET:
        ret = s->client_ctrl.reg32;
        break;
    case CLIENT_HOTRESETN_OFFSET:
        ret = s->client_hotresetn.reg32;
        break;
    case LOWLATENCY_RAWLOGIC_WRITE_PARTICIPANTS_OFFSET:
        ret = s->lowlatency_rawlogic_write_participants.reg32;
        break;
    case BWSHARE_TMVAL_OFFSET:
        ret = s->bwshare_tmval.reg32;
        break;
    case BWSHARE_EMEM_CTRL_0_OFFSET:
        ret = s->bwshare_emem_ctrl_0.reg32;
        break;
    case BWSHARE_EMEM_CTRL_1_OFFSET:
        ret = s->bwshare_emem_ctrl_1.reg32;
        break;
    case AVPC_ORRC_OFFSET:
        ret = s->avpc_orrc.reg32;
        break;
    case DC_ORRC_OFFSET:
        ret = s->dc_orrc.reg32;
        break;
    case DCB_ORRC_OFFSET:
        ret = s->dcb_orrc.reg32;
        break;
    case EPP_ORRC_OFFSET:
        ret = s->epp_orrc.reg32;
        break;
    case G2_ORRC_OFFSET:
        ret = s->g2_orrc.reg32;
        break;
    case HC_ORRC_OFFSET:
        ret = s->hc_orrc.reg32;
        break;
    case ISP_ORRC_OFFSET:
        ret = s->isp_orrc.reg32;
        break;
    case MPCORE_ORRC_OFFSET:
        ret = s->mpcore_orrc.reg32;
        break;
    case MPEA_ORRC_OFFSET:
        ret = s->mpea_orrc.reg32;
        break;
    case MPEB_ORRC_OFFSET:
        ret = s->mpeb_orrc.reg32;
        break;
    case MPEC_ORRC_OFFSET:
        ret = s->mpec_orrc.reg32;
        break;
    case NV_ORRC_OFFSET:
        ret = s->nv_orrc.reg32;
        break;
    case PPCS_ORRC_OFFSET:
        ret = s->ppcs_orrc.reg32;
        break;
    case VDE_ORRC_OFFSET:
        ret = s->vde_orrc.reg32;
        break;
    case VI_ORRC_OFFSET:
        ret = s->vi_orrc.reg32;
        break;
    case FPRI_CTRL_AVPC_OFFSET:
        ret = s->fpri_ctrl_avpc.reg32;
        break;
    case FPRI_CTRL_DC_OFFSET:
        ret = s->fpri_ctrl_dc.reg32;
        break;
    case FPRI_CTRL_DCB_OFFSET:
        ret = s->fpri_ctrl_dcb.reg32;
        break;
    case FPRI_CTRL_EPP_OFFSET:
        ret = s->fpri_ctrl_epp.reg32;
        break;
    case FPRI_CTRL_G2_OFFSET:
        ret = s->fpri_ctrl_g2.reg32;
        break;
    case FPRI_CTRL_HC_OFFSET:
        ret = s->fpri_ctrl_hc.reg32;
        break;
    case FPRI_CTRL_ISP_OFFSET:
        ret = s->fpri_ctrl_isp.reg32;
        break;
    case FPRI_CTRL_MPCORE_OFFSET:
        ret = s->fpri_ctrl_mpcore.reg32;
        break;
    case FPRI_CTRL_MPEA_OFFSET:
        ret = s->fpri_ctrl_mpea.reg32;
        break;
    case FPRI_CTRL_MPEB_OFFSET:
        ret = s->fpri_ctrl_mpeb.reg32;
        break;
    case FPRI_CTRL_MPEC_OFFSET:
        ret = s->fpri_ctrl_mpec.reg32;
        break;
    case FPRI_CTRL_NV_OFFSET:
        ret = s->fpri_ctrl_nv.reg32;
        break;
    case FPRI_CTRL_PPCS_OFFSET:
        ret = s->fpri_ctrl_ppcs.reg32;
        break;
    case FPRI_CTRL_VDE_OFFSET:
        ret = s->fpri_ctrl_vde.reg32;
        break;
    case FPRI_CTRL_VI_OFFSET:
        ret = s->fpri_ctrl_vi.reg32;
        break;
    case TIMEOUT_AVPC_OFFSET:
        ret = s->timeout_avpc.reg32;
        break;
    case TIMEOUT_DC_OFFSET:
        ret = s->timeout_dc.reg32;
        break;
    case TIMEOUT_DCB_OFFSET:
        ret = s->timeout_dcb.reg32;
        break;
    case TIMEOUT_EPP_OFFSET:
        ret = s->timeout_epp.reg32;
        break;
    case TIMEOUT_G2_OFFSET:
        ret = s->timeout_g2.reg32;
        break;
    case TIMEOUT_HC_OFFSET:
        ret = s->timeout_hc.reg32;
        break;
    case TIMEOUT_ISP_OFFSET:
        ret = s->timeout_isp.reg32;
        break;
    case TIMEOUT_MPCORE_OFFSET:
        ret = s->timeout_mpcore.reg32;
        break;
    case TIMEOUT_MPEA_OFFSET:
        ret = s->timeout_mpea.reg32;
        break;
    case TIMEOUT_MPEB_OFFSET:
        ret = s->timeout_mpeb.reg32;
        break;
    case TIMEOUT_MPEC_OFFSET:
        ret = s->timeout_mpec.reg32;
        break;
    case TIMEOUT_NV_OFFSET:
        ret = s->timeout_nv.reg32;
        break;
    case TIMEOUT_PPCS_OFFSET:
        ret = s->timeout_ppcs.reg32;
        break;
    case TIMEOUT_VDE_OFFSET:
        ret = s->timeout_vde.reg32;
        break;
    case TIMEOUT_VI_OFFSET:
        ret = s->timeout_vi.reg32;
        break;
    case TIMEOUT_RCOAL_AVPC_OFFSET:
        ret = s->timeout_rcoal_avpc.reg32;
        break;
    case TIMEOUT_RCOAL_DC_OFFSET:
        ret = s->timeout_rcoal_dc.reg32;
        break;
    case TIMEOUT1_RCOAL_DC_OFFSET:
        ret = s->timeout1_rcoal_dc.reg32;
        break;
    case TIMEOUT_RCOAL_DCB_OFFSET:
        ret = s->timeout_rcoal_dcb.reg32;
        break;
    case TIMEOUT1_RCOAL_DCB_OFFSET:
        ret = s->timeout1_rcoal_dcb.reg32;
        break;
    case TIMEOUT_RCOAL_EPP_OFFSET:
        ret = s->timeout_rcoal_epp.reg32;
        break;
    case TIMEOUT_RCOAL_G2_OFFSET:
        ret = s->timeout_rcoal_g2.reg32;
        break;
    case TIMEOUT_RCOAL_HC_OFFSET:
        ret = s->timeout_rcoal_hc.reg32;
        break;
    case TIMEOUT_RCOAL_MPCORE_OFFSET:
        ret = s->timeout_rcoal_mpcore.reg32;
        break;
    case TIMEOUT_RCOAL_MPEA_OFFSET:
        ret = s->timeout_rcoal_mpea.reg32;
        break;
    case TIMEOUT_RCOAL_MPEB_OFFSET:
        ret = s->timeout_rcoal_mpeb.reg32;
        break;
    case TIMEOUT_RCOAL_MPEC_OFFSET:
        ret = s->timeout_rcoal_mpec.reg32;
        break;
    case TIMEOUT_RCOAL_NV_OFFSET:
        ret = s->timeout_rcoal_nv.reg32;
        break;
    case TIMEOUT_RCOAL_PPCS_OFFSET:
        ret = s->timeout_rcoal_ppcs.reg32;
        break;
    case TIMEOUT_RCOAL_VDE_OFFSET:
        ret = s->timeout_rcoal_vde.reg32;
        break;
    case TIMEOUT_RCOAL_VI_OFFSET:
        ret = s->timeout_rcoal_vi.reg32;
        break;
    case RCOAL_AUTODISABLE_OFFSET:
        ret = s->rcoal_autodisable.reg32;
        break;
    case BWSHARE_AVPC_OFFSET:
        ret = s->bwshare_avpc.reg32;
        break;
    case BWSHARE_DC_OFFSET:
        ret = s->bwshare_dc.reg32;
        break;
    case BWSHARE_DCB_OFFSET:
        ret = s->bwshare_dcb.reg32;
        break;
    case BWSHARE_EPP_OFFSET:
        ret = s->bwshare_epp.reg32;
        break;
    case BWSHARE_G2_OFFSET:
        ret = s->bwshare_g2.reg32;
        break;
    case BWSHARE_HC_OFFSET:
        ret = s->bwshare_hc.reg32;
        break;
    case BWSHARE_ISP_OFFSET:
        ret = s->bwshare_isp.reg32;
        break;
    case BWSHARE_MPCORE_OFFSET:
        ret = s->bwshare_mpcore.reg32;
        break;
    case BWSHARE_MPEA_OFFSET:
        ret = s->bwshare_mpea.reg32;
        break;
    case BWSHARE_MPEB_OFFSET:
        ret = s->bwshare_mpeb.reg32;
        break;
    case BWSHARE_MPEC_OFFSET:
        ret = s->bwshare_mpec.reg32;
        break;
    case BWSHARE_NV_OFFSET:
        ret = s->bwshare_nv.reg32;
        break;
    case BWSHARE_PPCS_OFFSET:
        ret = s->bwshare_ppcs.reg32;
        break;
    case BWSHARE_VDE_OFFSET:
        ret = s->bwshare_vde.reg32;
        break;
    case BWSHARE_VI_OFFSET:
        ret = s->bwshare_vi.reg32;
        break;
    case INTSTATUS_OFFSET:
        ret = s->intstatus.reg32;
        break;
    case INTMASK_OFFSET:
        ret = s->intmask.reg32;
        break;
    case CLKEN_OVERRIDE_OFFSET:
        ret = s->clken_override.reg32;
        break;
    case SECURITY_CFG0_OFFSET:
        ret = s->security_cfg0.reg32;
        break;
    case SECURITY_CFG1_OFFSET:
        ret = s->security_cfg1.reg32;
        break;
    case SECURITY_VIOLATION_STATUS_OFFSET:
        ret = s->security_violation_status.reg32;
        break;
    case SECURITY_VIOLATION_ADR_OFFSET:
        ret = s->security_violation_adr.reg32;
        break;
    case SECURITY_CFG2_OFFSET:
        ret = s->security_cfg2.reg32;
        break;
    case STAT_CONTROL_OFFSET:
        ret = s->stat_control.reg32;
        break;
    case STAT_STATUS_OFFSET:
        ret = s->stat_status.reg32;
        break;
    case STAT_EMC_ADDR_LOW_OFFSET:
        ret = s->stat_emc_addr_low.reg32;
        break;
    case STAT_EMC_ADDR_HIGH_OFFSET:
        ret = s->stat_emc_addr_high.reg32;
        break;
    case STAT_EMC_CLOCK_LIMIT_OFFSET:
        ret = s->stat_emc_clock_limit.reg32;
        break;
    case STAT_EMC_CLOCKS_OFFSET:
        ret = s->stat_emc_clocks.reg32;
        break;
    case STAT_EMC_CONTROL_0_OFFSET:
        ret = s->stat_emc_control_0.reg32;
        break;
    case STAT_EMC_CONTROL_1_OFFSET:
        ret = s->stat_emc_control_1.reg32;
        break;
    case STAT_EMC_HIST_LIMIT_0_OFFSET:
        ret = s->stat_emc_hist_limit_0.reg32;
        break;
    case STAT_EMC_HIST_LIMIT_1_OFFSET:
        ret = s->stat_emc_hist_limit_1.reg32;
        break;
    case STAT_EMC_COUNT_0_OFFSET:
        ret = s->stat_emc_count_0.reg32;
        break;
    case STAT_EMC_COUNT_1_OFFSET:
        ret = s->stat_emc_count_1.reg32;
        break;
    case STAT_EMC_HIST_0_OFFSET:
        ret = s->stat_emc_hist_0.reg32;
        break;
    case STAT_EMC_HIST_1_OFFSET:
        ret = s->stat_emc_hist_1.reg32;
        break;
    case AXI_DECERR_OVR_OFFSET:
        ret = s->axi_decerr_ovr.reg32;
        break;
    case LOWLATENCY_CONFIG_OFFSET:
        ret = s->lowlatency_config.reg32;
        break;
    case AP_CTRL_0_OFFSET:
        ret = s->ap_ctrl_0.reg32;
        break;
    case AP_CTRL_1_OFFSET:
        ret = s->ap_ctrl_1.reg32;
        break;
    case CLIENT_ACTIVITY_MONITOR_EMEM_0_OFFSET:
        ret = s->client_activity_monitor_emem_0.reg32;
        break;
    case CLIENT_ACTIVITY_MONITOR_EMEM_1_OFFSET:
        ret = s->client_activity_monitor_emem_1.reg32;
        break;
    case 0x274 ... 0xBFC:
        ret = s->regs[(offset-0x274)>>2];
        break;
    default:
        if (offset == s->emem_cfg_offset) ret = s->emem_cfg.reg32;
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_mc_priv_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    tegra_mc *s = opaque;

    switch (offset) {
    case EMEM_ADR_CFG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->emem_adr_cfg.reg32, value);
        s->emem_adr_cfg.reg32 = value;
        break;
    case EMEM_ARB_CFG0_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->emem_arb_cfg0.reg32, value);
        s->emem_arb_cfg0.reg32 = value;
        break;
    case EMEM_ARB_CFG1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->emem_arb_cfg1.reg32, value);
        s->emem_arb_cfg1.reg32 = value;
        break;
    case EMEM_ARB_CFG2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->emem_arb_cfg2.reg32, value);
        s->emem_arb_cfg2.reg32 = value;
        break;
    case GART_CONFIG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gart_config.reg32, value);
        s->gart_config.reg32 = value;
        break;
    case GART_ENTRY_ADDR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gart_entry_addr.reg32, value);
        s->gart_entry_addr.reg32 = value;
        break;
    case GART_ENTRY_DATA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gart_entry_data.reg32, value);
        s->gart_entry_data.reg32 = value;
        break;
    case TIMEOUT_CTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_ctrl.reg32, value);
        s->timeout_ctrl.reg32 = value;
        break;
    case CLIENT_CTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->client_ctrl.reg32, value);
        s->client_ctrl.reg32 = value;
        break;
    case CLIENT_HOTRESETN_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->client_hotresetn.reg32, value);
        s->client_hotresetn.reg32 = value;
        break;
    case LOWLATENCY_RAWLOGIC_WRITE_PARTICIPANTS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->lowlatency_rawlogic_write_participants.reg32, value);
        s->lowlatency_rawlogic_write_participants.reg32 = value;
        break;
    case BWSHARE_TMVAL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_tmval.reg32, value);
        s->bwshare_tmval.reg32 = value;
        break;
    case BWSHARE_EMEM_CTRL_0_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_emem_ctrl_0.reg32, value);
        s->bwshare_emem_ctrl_0.reg32 = value;
        break;
    case BWSHARE_EMEM_CTRL_1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_emem_ctrl_1.reg32, value);
        s->bwshare_emem_ctrl_1.reg32 = value;
        break;
    case FPRI_CTRL_AVPC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fpri_ctrl_avpc.reg32, value);
        s->fpri_ctrl_avpc.reg32 = value;
        break;
    case FPRI_CTRL_DC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fpri_ctrl_dc.reg32, value);
        s->fpri_ctrl_dc.reg32 = value;
        break;
    case FPRI_CTRL_DCB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fpri_ctrl_dcb.reg32, value);
        s->fpri_ctrl_dcb.reg32 = value;
        break;
    case FPRI_CTRL_EPP_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fpri_ctrl_epp.reg32, value);
        s->fpri_ctrl_epp.reg32 = value;
        break;
    case FPRI_CTRL_G2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fpri_ctrl_g2.reg32, value);
        s->fpri_ctrl_g2.reg32 = value;
        break;
    case FPRI_CTRL_HC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fpri_ctrl_hc.reg32, value);
        s->fpri_ctrl_hc.reg32 = value;
        break;
    case FPRI_CTRL_ISP_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fpri_ctrl_isp.reg32, value);
        s->fpri_ctrl_isp.reg32 = value;
        break;
    case FPRI_CTRL_MPCORE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fpri_ctrl_mpcore.reg32, value);
        s->fpri_ctrl_mpcore.reg32 = value;
        break;
    case FPRI_CTRL_MPEA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fpri_ctrl_mpea.reg32, value);
        s->fpri_ctrl_mpea.reg32 = value;
        break;
    case FPRI_CTRL_MPEB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fpri_ctrl_mpeb.reg32, value);
        s->fpri_ctrl_mpeb.reg32 = value;
        break;
    case FPRI_CTRL_MPEC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fpri_ctrl_mpec.reg32, value);
        s->fpri_ctrl_mpec.reg32 = value;
        break;
    case FPRI_CTRL_NV_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fpri_ctrl_nv.reg32, value);
        s->fpri_ctrl_nv.reg32 = value;
        break;
    case FPRI_CTRL_PPCS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fpri_ctrl_ppcs.reg32, value);
        s->fpri_ctrl_ppcs.reg32 = value;
        break;
    case FPRI_CTRL_VDE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fpri_ctrl_vde.reg32, value);
        s->fpri_ctrl_vde.reg32 = value;
        break;
    case FPRI_CTRL_VI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fpri_ctrl_vi.reg32, value);
        s->fpri_ctrl_vi.reg32 = value;
        break;
    case TIMEOUT_AVPC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_avpc.reg32, value);
        s->timeout_avpc.reg32 = value;
        break;
    case TIMEOUT_DC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_dc.reg32, value);
        s->timeout_dc.reg32 = value;
        break;
    case TIMEOUT_DCB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_dcb.reg32, value);
        s->timeout_dcb.reg32 = value;
        break;
    case TIMEOUT_EPP_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_epp.reg32, value);
        s->timeout_epp.reg32 = value;
        break;
    case TIMEOUT_G2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_g2.reg32, value);
        s->timeout_g2.reg32 = value;
        break;
    case TIMEOUT_HC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_hc.reg32, value);
        s->timeout_hc.reg32 = value;
        break;
    case TIMEOUT_ISP_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_isp.reg32, value);
        s->timeout_isp.reg32 = value;
        break;
    case TIMEOUT_MPCORE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_mpcore.reg32, value);
        s->timeout_mpcore.reg32 = value;
        break;
    case TIMEOUT_MPEA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_mpea.reg32, value);
        s->timeout_mpea.reg32 = value;
        break;
    case TIMEOUT_MPEB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_mpeb.reg32, value);
        s->timeout_mpeb.reg32 = value;
        break;
    case TIMEOUT_MPEC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_mpec.reg32, value);
        s->timeout_mpec.reg32 = value;
        break;
    case TIMEOUT_NV_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_nv.reg32, value);
        s->timeout_nv.reg32 = value;
        break;
    case TIMEOUT_PPCS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_ppcs.reg32, value);
        s->timeout_ppcs.reg32 = value;
        break;
    case TIMEOUT_VDE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_vde.reg32, value);
        s->timeout_vde.reg32 = value;
        break;
    case TIMEOUT_VI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_vi.reg32, value);
        s->timeout_vi.reg32 = value;
        break;
    case TIMEOUT_RCOAL_AVPC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_rcoal_avpc.reg32, value);
        s->timeout_rcoal_avpc.reg32 = value;
        break;
    case TIMEOUT_RCOAL_DC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_rcoal_dc.reg32, value);
        s->timeout_rcoal_dc.reg32 = value;
        break;
    case TIMEOUT1_RCOAL_DC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout1_rcoal_dc.reg32, value);
        s->timeout1_rcoal_dc.reg32 = value;
        break;
    case TIMEOUT_RCOAL_DCB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_rcoal_dcb.reg32, value);
        s->timeout_rcoal_dcb.reg32 = value;
        break;
    case TIMEOUT1_RCOAL_DCB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout1_rcoal_dcb.reg32, value);
        s->timeout1_rcoal_dcb.reg32 = value;
        break;
    case TIMEOUT_RCOAL_EPP_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_rcoal_epp.reg32, value);
        s->timeout_rcoal_epp.reg32 = value;
        break;
    case TIMEOUT_RCOAL_G2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_rcoal_g2.reg32, value);
        s->timeout_rcoal_g2.reg32 = value;
        break;
    case TIMEOUT_RCOAL_HC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_rcoal_hc.reg32, value);
        s->timeout_rcoal_hc.reg32 = value;
        break;
    case TIMEOUT_RCOAL_MPCORE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_rcoal_mpcore.reg32, value);
        s->timeout_rcoal_mpcore.reg32 = value;
        break;
    case TIMEOUT_RCOAL_MPEA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_rcoal_mpea.reg32, value);
        s->timeout_rcoal_mpea.reg32 = value;
        break;
    case TIMEOUT_RCOAL_MPEB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_rcoal_mpeb.reg32, value);
        s->timeout_rcoal_mpeb.reg32 = value;
        break;
    case TIMEOUT_RCOAL_MPEC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_rcoal_mpec.reg32, value);
        s->timeout_rcoal_mpec.reg32 = value;
        break;
    case TIMEOUT_RCOAL_NV_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_rcoal_nv.reg32, value);
        s->timeout_rcoal_nv.reg32 = value;
        break;
    case TIMEOUT_RCOAL_PPCS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_rcoal_ppcs.reg32, value);
        s->timeout_rcoal_ppcs.reg32 = value;
        break;
    case TIMEOUT_RCOAL_VDE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_rcoal_vde.reg32, value);
        s->timeout_rcoal_vde.reg32 = value;
        break;
    case TIMEOUT_RCOAL_VI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timeout_rcoal_vi.reg32, value);
        s->timeout_rcoal_vi.reg32 = value;
        break;
    case RCOAL_AUTODISABLE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rcoal_autodisable.reg32, value);
        s->rcoal_autodisable.reg32 = value;
        break;
    case BWSHARE_AVPC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_avpc.reg32, value);
        s->bwshare_avpc.reg32 = value;
        break;
    case BWSHARE_DC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_dc.reg32, value);
        s->bwshare_dc.reg32 = value;
        break;
    case BWSHARE_DCB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_dcb.reg32, value);
        s->bwshare_dcb.reg32 = value;
        break;
    case BWSHARE_EPP_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_epp.reg32, value);
        s->bwshare_epp.reg32 = value;
        break;
    case BWSHARE_G2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_g2.reg32, value);
        s->bwshare_g2.reg32 = value;
        break;
    case BWSHARE_HC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_hc.reg32, value);
        s->bwshare_hc.reg32 = value;
        break;
    case BWSHARE_ISP_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_isp.reg32, value);
        s->bwshare_isp.reg32 = value;
        break;
    case BWSHARE_MPCORE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_mpcore.reg32, value);
        s->bwshare_mpcore.reg32 = value;
        break;
    case BWSHARE_MPEA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_mpea.reg32, value);
        s->bwshare_mpea.reg32 = value;
        break;
    case BWSHARE_MPEB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_mpeb.reg32, value);
        s->bwshare_mpeb.reg32 = value;
        break;
    case BWSHARE_MPEC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_mpec.reg32, value);
        s->bwshare_mpec.reg32 = value;
        break;
    case BWSHARE_NV_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_nv.reg32, value);
        s->bwshare_nv.reg32 = value;
        break;
    case BWSHARE_PPCS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_ppcs.reg32, value);
        s->bwshare_ppcs.reg32 = value;
        break;
    case BWSHARE_VDE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_vde.reg32, value);
        s->bwshare_vde.reg32 = value;
        break;
    case BWSHARE_VI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bwshare_vi.reg32, value);
        s->bwshare_vi.reg32 = value;
        break;
    case INTSTATUS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->intstatus.reg32, value);
        s->intstatus.reg32 = value;
        break;
    case INTMASK_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->intmask.reg32, value);
        s->intmask.reg32 = value;
        break;
    case CLKEN_OVERRIDE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clken_override.reg32, value);
        s->clken_override.reg32 = value;
        break;
    case SECURITY_CFG0_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->security_cfg0.reg32, value);
        s->security_cfg0.reg32 = value;
        break;
    case SECURITY_CFG1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->security_cfg1.reg32, value);
        s->security_cfg1.reg32 = value;
        break;
    case SECURITY_VIOLATION_STATUS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->security_violation_status.reg32, value & SECURITY_VIOLATION_STATUS_WRMASK);
        WR_MASKED(s->security_violation_status.reg32, value, SECURITY_VIOLATION_STATUS);
        break;
    case SECURITY_CFG2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->security_cfg2.reg32, value);
        s->security_cfg2.reg32 = value;
        break;
    case STAT_CONTROL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_control.reg32, value);
        s->stat_control.reg32 = value;
        break;
    case STAT_STATUS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_status.reg32, value & STAT_STATUS_WRMASK);
        WR_MASKED(s->stat_status.reg32, value, STAT_STATUS);
        break;
    case STAT_EMC_ADDR_LOW_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_emc_addr_low.reg32, value);
        s->stat_emc_addr_low.reg32 = value;
        break;
    case STAT_EMC_ADDR_HIGH_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_emc_addr_high.reg32, value);
        s->stat_emc_addr_high.reg32 = value;
        break;
    case STAT_EMC_CLOCK_LIMIT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_emc_clock_limit.reg32, value);
        s->stat_emc_clock_limit.reg32 = value;
        break;
    case STAT_EMC_CONTROL_0_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_emc_control_0.reg32, value);
        s->stat_emc_control_0.reg32 = value;
        break;
    case STAT_EMC_CONTROL_1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_emc_control_1.reg32, value);
        s->stat_emc_control_1.reg32 = value;
        break;
    case STAT_EMC_HIST_LIMIT_0_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_emc_hist_limit_0.reg32, value);
        s->stat_emc_hist_limit_0.reg32 = value;
        break;
    case STAT_EMC_HIST_LIMIT_1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_emc_hist_limit_1.reg32, value);
        s->stat_emc_hist_limit_1.reg32 = value;
        break;
    case AXI_DECERR_OVR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->axi_decerr_ovr.reg32, value);
        s->axi_decerr_ovr.reg32 = value;
        break;
    case LOWLATENCY_CONFIG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->lowlatency_config.reg32, value);
        s->lowlatency_config.reg32 = value;
        break;
    case AP_CTRL_0_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ap_ctrl_0.reg32, value);
        s->ap_ctrl_0.reg32 = value;
        break;
    case AP_CTRL_1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ap_ctrl_1.reg32, value);
        s->ap_ctrl_1.reg32 = value;
        break;
    case CLIENT_ACTIVITY_MONITOR_EMEM_0_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->client_activity_monitor_emem_0.reg32, value);
        s->client_activity_monitor_emem_0.reg32 = value;
        break;
    case CLIENT_ACTIVITY_MONITOR_EMEM_1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->client_activity_monitor_emem_1.reg32, value);
        s->client_activity_monitor_emem_1.reg32 = value;
        break;
    case 0x274 ... 0xBFC:
        TRACE_WRITE(s->iomem.addr, offset, s->regs[(offset-0x274)>>2], value);
        s->regs[(offset-0x274)>>2] = value;
        break;
    default:
        if (offset == s->emem_cfg_offset) {
            TRACE_WRITE(s->iomem.addr, offset, s->emem_cfg.reg32, value);
            s->emem_cfg.reg32 = value;
        }
        else
            TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }
}

static void tegra_mc_priv_reset(DeviceState *dev)
{
    tegra_mc *s = TEGRA_MC(dev);

    if (tegra_board >= TEGRAX1_BOARD) {
        s->emem_cfg_offset = EMEM_CFG_TEGRAX1_OFFSET;
        s->emem_cfg.reg32 = s->ram_size_kb / SZ_1K;
    }
    else {
        s->emem_cfg_offset = EMEM_CFG_TEGRA2_OFFSET;
        s->emem_cfg.reg32 = s->ram_size_kb;
    }
    s->emem_adr_cfg.reg32 = EMEM_ADR_CFG_RESET;
    s->emem_arb_cfg0.reg32 = EMEM_ARB_CFG0_RESET;
    s->emem_arb_cfg1.reg32 = EMEM_ARB_CFG1_RESET;
    s->emem_arb_cfg2.reg32 = EMEM_ARB_CFG2_RESET;
    s->gart_config.reg32 = GART_CONFIG_RESET;
    s->gart_entry_addr.reg32 = GART_ENTRY_ADDR_RESET;
    s->gart_entry_data.reg32 = GART_ENTRY_DATA_RESET;
    s->gart_error_req.reg32 = GART_ERROR_REQ_RESET;
    s->gart_error_addr.reg32 = GART_ERROR_ADDR_RESET;
    s->timeout_ctrl.reg32 = TIMEOUT_CTRL_RESET;
    s->decerr_emem_others_status.reg32 = DECERR_EMEM_OTHERS_STATUS_RESET;
    s->decerr_emem_others_adr.reg32 = DECERR_EMEM_OTHERS_ADR_RESET;
    s->client_ctrl.reg32 = CLIENT_CTRL_RESET;
    s->client_hotresetn.reg32 = CLIENT_HOTRESETN_RESET;
    s->lowlatency_rawlogic_write_participants.reg32 = LOWLATENCY_RAWLOGIC_WRITE_PARTICIPANTS_RESET;
    s->bwshare_tmval.reg32 = BWSHARE_TMVAL_RESET;
    s->bwshare_emem_ctrl_0.reg32 = BWSHARE_EMEM_CTRL_0_RESET;
    s->bwshare_emem_ctrl_1.reg32 = BWSHARE_EMEM_CTRL_1_RESET;
    s->avpc_orrc.reg32 = AVPC_ORRC_RESET;
    s->dc_orrc.reg32 = DC_ORRC_RESET;
    s->dcb_orrc.reg32 = DCB_ORRC_RESET;
    s->epp_orrc.reg32 = EPP_ORRC_RESET;
    s->g2_orrc.reg32 = G2_ORRC_RESET;
    s->hc_orrc.reg32 = HC_ORRC_RESET;
    s->isp_orrc.reg32 = ISP_ORRC_RESET;
    s->mpcore_orrc.reg32 = MPCORE_ORRC_RESET;
    s->mpea_orrc.reg32 = MPEA_ORRC_RESET;
    s->mpeb_orrc.reg32 = MPEB_ORRC_RESET;
    s->mpec_orrc.reg32 = MPEC_ORRC_RESET;
    s->nv_orrc.reg32 = NV_ORRC_RESET;
    s->ppcs_orrc.reg32 = PPCS_ORRC_RESET;
    s->vde_orrc.reg32 = VDE_ORRC_RESET;
    s->vi_orrc.reg32 = VI_ORRC_RESET;
    s->fpri_ctrl_avpc.reg32 = FPRI_CTRL_AVPC_RESET;
    s->fpri_ctrl_dc.reg32 = FPRI_CTRL_DC_RESET;
    s->fpri_ctrl_dcb.reg32 = FPRI_CTRL_DCB_RESET;
    s->fpri_ctrl_epp.reg32 = FPRI_CTRL_EPP_RESET;
    s->fpri_ctrl_g2.reg32 = FPRI_CTRL_G2_RESET;
    s->fpri_ctrl_hc.reg32 = FPRI_CTRL_HC_RESET;
    s->fpri_ctrl_isp.reg32 = FPRI_CTRL_ISP_RESET;
    s->fpri_ctrl_mpcore.reg32 = FPRI_CTRL_MPCORE_RESET;
    s->fpri_ctrl_mpea.reg32 = FPRI_CTRL_MPEA_RESET;
    s->fpri_ctrl_mpeb.reg32 = FPRI_CTRL_MPEB_RESET;
    s->fpri_ctrl_mpec.reg32 = FPRI_CTRL_MPEC_RESET;
    s->fpri_ctrl_nv.reg32 = FPRI_CTRL_NV_RESET;
    s->fpri_ctrl_ppcs.reg32 = FPRI_CTRL_PPCS_RESET;
    s->fpri_ctrl_vde.reg32 = FPRI_CTRL_VDE_RESET;
    s->fpri_ctrl_vi.reg32 = FPRI_CTRL_VI_RESET;
    s->timeout_avpc.reg32 = TIMEOUT_AVPC_RESET;
    s->timeout_dc.reg32 = TIMEOUT_DC_RESET;
    s->timeout_dcb.reg32 = TIMEOUT_DCB_RESET;
    s->timeout_epp.reg32 = TIMEOUT_EPP_RESET;
    s->timeout_g2.reg32 = TIMEOUT_G2_RESET;
    s->timeout_hc.reg32 = TIMEOUT_HC_RESET;
    s->timeout_isp.reg32 = TIMEOUT_ISP_RESET;
    s->timeout_mpcore.reg32 = TIMEOUT_MPCORE_RESET;
    s->timeout_mpea.reg32 = TIMEOUT_MPEA_RESET;
    s->timeout_mpeb.reg32 = TIMEOUT_MPEB_RESET;
    s->timeout_mpec.reg32 = TIMEOUT_MPEC_RESET;
    s->timeout_nv.reg32 = TIMEOUT_NV_RESET;
    s->timeout_ppcs.reg32 = TIMEOUT_PPCS_RESET;
    s->timeout_vde.reg32 = TIMEOUT_VDE_RESET;
    s->timeout_vi.reg32 = TIMEOUT_VI_RESET;
    s->timeout_rcoal_avpc.reg32 = TIMEOUT_RCOAL_AVPC_RESET;
    s->timeout_rcoal_dc.reg32 = TIMEOUT_RCOAL_DC_RESET;
    s->timeout1_rcoal_dc.reg32 = TIMEOUT1_RCOAL_DC_RESET;
    s->timeout_rcoal_dcb.reg32 = TIMEOUT_RCOAL_DCB_RESET;
    s->timeout1_rcoal_dcb.reg32 = TIMEOUT1_RCOAL_DCB_RESET;
    s->timeout_rcoal_epp.reg32 = TIMEOUT_RCOAL_EPP_RESET;
    s->timeout_rcoal_g2.reg32 = TIMEOUT_RCOAL_G2_RESET;
    s->timeout_rcoal_hc.reg32 = TIMEOUT_RCOAL_HC_RESET;
    s->timeout_rcoal_mpcore.reg32 = TIMEOUT_RCOAL_MPCORE_RESET;
    s->timeout_rcoal_mpea.reg32 = TIMEOUT_RCOAL_MPEA_RESET;
    s->timeout_rcoal_mpeb.reg32 = TIMEOUT_RCOAL_MPEB_RESET;
    s->timeout_rcoal_mpec.reg32 = TIMEOUT_RCOAL_MPEC_RESET;
    s->timeout_rcoal_nv.reg32 = TIMEOUT_RCOAL_NV_RESET;
    s->timeout_rcoal_ppcs.reg32 = TIMEOUT_RCOAL_PPCS_RESET;
    s->timeout_rcoal_vde.reg32 = TIMEOUT_RCOAL_VDE_RESET;
    s->timeout_rcoal_vi.reg32 = TIMEOUT_RCOAL_VI_RESET;
    s->rcoal_autodisable.reg32 = RCOAL_AUTODISABLE_RESET;
    s->bwshare_avpc.reg32 = BWSHARE_AVPC_RESET;
    s->bwshare_dc.reg32 = BWSHARE_DC_RESET;
    s->bwshare_dcb.reg32 = BWSHARE_DCB_RESET;
    s->bwshare_epp.reg32 = BWSHARE_EPP_RESET;
    s->bwshare_g2.reg32 = BWSHARE_G2_RESET;
    s->bwshare_hc.reg32 = BWSHARE_HC_RESET;
    s->bwshare_isp.reg32 = BWSHARE_ISP_RESET;
    s->bwshare_mpcore.reg32 = BWSHARE_MPCORE_RESET;
    s->bwshare_mpea.reg32 = BWSHARE_MPEA_RESET;
    s->bwshare_mpeb.reg32 = BWSHARE_MPEB_RESET;
    s->bwshare_mpec.reg32 = BWSHARE_MPEC_RESET;
    s->bwshare_nv.reg32 = BWSHARE_NV_RESET;
    s->bwshare_ppcs.reg32 = BWSHARE_PPCS_RESET;
    s->bwshare_vde.reg32 = BWSHARE_VDE_RESET;
    s->bwshare_vi.reg32 = BWSHARE_VI_RESET;
    s->intstatus.reg32 = INTSTATUS_RESET;
    s->intmask.reg32 = INTMASK_RESET;
    s->clken_override.reg32 = CLKEN_OVERRIDE_RESET;
    s->security_cfg0.reg32 = SECURITY_CFG0_RESET;
    s->security_cfg1.reg32 = SECURITY_CFG1_RESET;
    s->security_violation_status.reg32 = SECURITY_VIOLATION_STATUS_RESET;
    s->security_violation_adr.reg32 = SECURITY_VIOLATION_ADR_RESET;
    s->security_cfg2.reg32 = SECURITY_CFG2_RESET;
    s->stat_control.reg32 = STAT_CONTROL_RESET;
    s->stat_status.reg32 = STAT_STATUS_RESET;
    s->stat_emc_addr_low.reg32 = STAT_EMC_ADDR_LOW_RESET;
    s->stat_emc_addr_high.reg32 = STAT_EMC_ADDR_HIGH_RESET;
    s->stat_emc_clock_limit.reg32 = STAT_EMC_CLOCK_LIMIT_RESET;
    s->stat_emc_clocks.reg32 = STAT_EMC_CLOCKS_RESET;
    s->stat_emc_control_0.reg32 = STAT_EMC_CONTROL_0_RESET;
    s->stat_emc_control_1.reg32 = STAT_EMC_CONTROL_1_RESET;
    s->stat_emc_hist_limit_0.reg32 = STAT_EMC_HIST_LIMIT_0_RESET;
    s->stat_emc_hist_limit_1.reg32 = STAT_EMC_HIST_LIMIT_1_RESET;
    s->stat_emc_count_0.reg32 = STAT_EMC_COUNT_0_RESET;
    s->stat_emc_count_1.reg32 = STAT_EMC_COUNT_1_RESET;
    s->stat_emc_hist_0.reg32 = STAT_EMC_HIST_0_RESET;
    s->stat_emc_hist_1.reg32 = STAT_EMC_HIST_1_RESET;
    s->axi_decerr_ovr.reg32 = AXI_DECERR_OVR_RESET;
    s->lowlatency_config.reg32 = LOWLATENCY_CONFIG_RESET;
    s->ap_ctrl_0.reg32 = AP_CTRL_0_RESET;
    s->ap_ctrl_1.reg32 = AP_CTRL_1_RESET;
    s->client_activity_monitor_emem_0.reg32 = CLIENT_ACTIVITY_MONITOR_EMEM_0_RESET;
    s->client_activity_monitor_emem_1.reg32 = CLIENT_ACTIVITY_MONITOR_EMEM_1_RESET;
    memset(s->regs, 0, sizeof(s->regs));
}

static const MemoryRegionOps tegra_mc_mem_ops = {
    .read = tegra_mc_priv_read,
    .write = tegra_mc_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_mc_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_mc *s = TEGRA_MC(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_mc_mem_ops, s,
                          "tegra.mc", TEGRA_MC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static Property tegra_mc_properties[] = {
    DEFINE_PROP_UINT32("ram_size_kb", tegra_mc, ram_size_kb, EMEM_CFG_TEGRAX1_RESET*SZ_1K), \
    DEFINE_PROP_END_OF_LIST(),
};

static void tegra_mc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, tegra_mc_properties);
    dc->realize = tegra_mc_priv_realize;
    dc->vmsd = &vmstate_tegra_mc;
    dc->reset = tegra_mc_priv_reset;
}

static const TypeInfo tegra_mc_info = {
    .name = TYPE_TEGRA_MC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_mc),
    .class_init = tegra_mc_class_init,
};

static void tegra_mc_register_types(void)
{
    type_register_static(&tegra_mc_info);
}

type_init(tegra_mc_register_types)
