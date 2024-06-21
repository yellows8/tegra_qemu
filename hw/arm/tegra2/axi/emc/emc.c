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

#include "emc.h"
#include "iomap.h"
#include "devices.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_EMC "tegra.emc"
#define TEGRA_EMC(obj) OBJECT_CHECK(tegra_emc, (obj), TYPE_TEGRA_EMC)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_emc_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    DEFINE_REG32(intstatus);
    DEFINE_REG32(intmask);
    DEFINE_REG32(dbg);
    DEFINE_REG32(cfg);
    DEFINE_REG32(adr_cfg);
    DEFINE_REG32(adr_cfg_1);
    DEFINE_REG32(refctrl);
    DEFINE_REG32(pin);
    DEFINE_REG32(timing_control);
    DEFINE_REG32(rc);
    DEFINE_REG32(rfc);
    DEFINE_REG32(ras);
    DEFINE_REG32(rp);
    DEFINE_REG32(r2w);
    DEFINE_REG32(w2r);
    DEFINE_REG32(r2p);
    DEFINE_REG32(w2p);
    DEFINE_REG32(rd_rcd);
    DEFINE_REG32(wr_rcd);
    DEFINE_REG32(rrd);
    DEFINE_REG32(rext);
    DEFINE_REG32(wdv);
    DEFINE_REG32(quse);
    DEFINE_REG32(qrst);
    DEFINE_REG32(qsafe);
    DEFINE_REG32(rdv);
    DEFINE_REG32(refresh);
    DEFINE_REG32(burst_refresh_num);
    DEFINE_REG32(pdex2wr);
    DEFINE_REG32(pdex2rd);
    DEFINE_REG32(pchg2pden);
    DEFINE_REG32(act2pden);
    DEFINE_REG32(ar2pden);
    DEFINE_REG32(rw2pden);
    DEFINE_REG32(txsr);
    DEFINE_REG32(tcke);
    DEFINE_REG32(tfaw);
    DEFINE_REG32(trpab);
    DEFINE_REG32(tclkstable);
    DEFINE_REG32(tclkstop);
    DEFINE_REG32(trefbw);
    DEFINE_REG32(quse_extra);
    DEFINE_REG32(odt_write);
    DEFINE_REG32(odt_read);
    DEFINE_REG32(mrs);
    DEFINE_REG32(emrs);
    DEFINE_REG32(ref);
    DEFINE_REG32(pre);
    DEFINE_REG32(nop);
    DEFINE_REG32(self_ref);
    DEFINE_REG32(dpd);
    DEFINE_REG32(mrw);
    DEFINE_REG32(mrr);
    DEFINE_REG32(fbio_cfg1);
    DEFINE_REG32(fbio_dqsib_dly);
    DEFINE_REG32(fbio_dqsib_dly_msb);
    DEFINE_REG32(fbio_cfg5);
    DEFINE_REG32(fbio_quse_dly);
    DEFINE_REG32(fbio_quse_dly_msb);
    DEFINE_REG32(fbio_cfg6);
    DEFINE_REG32(dqs_trimmer_rd0);
    DEFINE_REG32(dqs_trimmer_rd1);
    DEFINE_REG32(dqs_trimmer_rd2);
    DEFINE_REG32(dqs_trimmer_rd3);
    DEFINE_REG32(ll_arb_config);
    DEFINE_REG32(t_min_critical_hp);
    DEFINE_REG32(t_min_critical_timeout);
    DEFINE_REG32(t_min_load);
    DEFINE_REG32(t_max_critical_hp);
    DEFINE_REG32(t_max_critical_timeout);
    DEFINE_REG32(t_max_load);
    DEFINE_REG32(auto_cal_config);
    DEFINE_REG32(auto_cal_interval);
    DEFINE_REG32(auto_cal_status);
    DEFINE_REG32(req_ctrl);
    DEFINE_REG32(emc_status);
    DEFINE_REG32(cfg_2);
    DEFINE_REG32(cfg_dig_dll);
    DEFINE_REG32(dll_xform_dqs);
    DEFINE_REG32(dll_xform_quse);
    DEFINE_REG32(dig_dll_upper_status);
    DEFINE_REG32(dig_dll_lower_status);
    DEFINE_REG32(ctt_term_ctrl);
    DEFINE_REG32(zcal_ref_cnt);
    DEFINE_REG32(zcal_wait_cnt);
    DEFINE_REG32(zcal_mrw_cmd);
    DEFINE_REG32(cmdq);
    DEFINE_REG32(fbio_spare);
    DEFINE_REG32(fbio_wrptr_eq_2);
    DEFINE_REG32(clken_override);
    DEFINE_REG32(stat_control);
    DEFINE_REG32(stat_status);
    DEFINE_REG32(stat_llmc_addr_low);
    DEFINE_REG32(stat_llmc_addr_high);
    DEFINE_REG32(stat_llmc_clock_limit);
    DEFINE_REG32(stat_llmc_clocks);
    DEFINE_REG32(stat_llmc_control);
    DEFINE_REG32(stat_llmc_hist_limit);
    DEFINE_REG32(stat_llmc_count);
    DEFINE_REG32(stat_llmc_hist);
    DEFINE_REG32(stat_pwr_clock_limit);
    DEFINE_REG32(stat_pwr_clocks);
    DEFINE_REG32(stat_pwr_count);
    DEFINE_REG32(stat_dram_clock_limit_lo);
    DEFINE_REG32(stat_dram_clock_limit_hi);
    DEFINE_REG32(stat_dram_clocks_lo);
    DEFINE_REG32(stat_dram_clocks_hi);
    DEFINE_REG32(stat_dram_dev0_activate_cnt_lo);
    DEFINE_REG32(stat_dram_dev0_activate_cnt_hi);
    DEFINE_REG32(stat_dram_dev0_read_cnt_lo);
    DEFINE_REG32(stat_dram_dev0_read_cnt_hi);
    DEFINE_REG32(stat_dram_dev0_write_cnt_lo);
    DEFINE_REG32(stat_dram_dev0_write_cnt_hi);
    DEFINE_REG32(stat_dram_dev0_ref_cnt_lo);
    DEFINE_REG32(stat_dram_dev0_ref_cnt_hi);
    DEFINE_REG32(stat_dram_dev0_cumm_banks_active_cke_eq1_lo);
    DEFINE_REG32(stat_dram_dev0_cumm_banks_active_cke_eq1_hi);
    DEFINE_REG32(stat_dram_dev0_cumm_banks_active_cke_eq0_lo);
    DEFINE_REG32(stat_dram_dev0_cumm_banks_active_cke_eq0_hi);
    DEFINE_REG32(stat_dram_dev0_cke_eq1_clks_lo);
    DEFINE_REG32(stat_dram_dev0_cke_eq1_clks_hi);
    DEFINE_REG32(stat_dram_dev0_extclks_cke_eq1_lo);
    DEFINE_REG32(stat_dram_dev0_extclks_cke_eq1_hi);
    DEFINE_REG32(stat_dram_dev0_extclks_cke_eq0_lo);
    DEFINE_REG32(stat_dram_dev0_extclks_cke_eq0_hi);
    DEFINE_REG32(stat_dram_dev1_activate_cnt_lo);
    DEFINE_REG32(stat_dram_dev1_activate_cnt_hi);
    DEFINE_REG32(stat_dram_dev1_read_cnt_lo);
    DEFINE_REG32(stat_dram_dev1_read_cnt_hi);
    DEFINE_REG32(stat_dram_dev1_write_cnt_lo);
    DEFINE_REG32(stat_dram_dev1_write_cnt_hi);
    DEFINE_REG32(stat_dram_dev1_ref_cnt_lo);
    DEFINE_REG32(stat_dram_dev1_ref_cnt_hi);
    DEFINE_REG32(stat_dram_dev1_cumm_banks_active_cke_eq1_lo);
    DEFINE_REG32(stat_dram_dev1_cumm_banks_active_cke_eq1_hi);
    DEFINE_REG32(stat_dram_dev1_cumm_banks_active_cke_eq0_lo);
    DEFINE_REG32(stat_dram_dev1_cumm_banks_active_cke_eq0_hi);
    DEFINE_REG32(stat_dram_dev1_cke_eq1_clks_lo);
    DEFINE_REG32(stat_dram_dev1_cke_eq1_clks_hi);
    DEFINE_REG32(stat_dram_dev1_extclks_cke_eq1_lo);
    DEFINE_REG32(stat_dram_dev1_extclks_cke_eq1_hi);
    DEFINE_REG32(stat_dram_dev1_extclks_cke_eq0_lo);
    DEFINE_REG32(stat_dram_dev1_extclks_cke_eq0_hi);
    DEFINE_REG32(stat_dram_dev0_no_banks_active_cke_eq1_lo);
    DEFINE_REG32(stat_dram_dev0_no_banks_active_cke_eq1_hi);
    DEFINE_REG32(stat_dram_dev0_no_banks_active_cke_eq0_lo);
    DEFINE_REG32(stat_dram_dev0_no_banks_active_cke_eq0_hi);
    DEFINE_REG32(stat_dram_dev1_no_banks_active_cke_eq1_lo);
    DEFINE_REG32(stat_dram_dev1_no_banks_active_cke_eq1_hi);
    DEFINE_REG32(stat_dram_dev1_no_banks_active_cke_eq0_lo);
    DEFINE_REG32(stat_dram_dev1_no_banks_active_cke_eq0_hi);
    DEFINE_REG32(cfg_clktrim);
    DEFINE_REG32(cfg_clktrim_1);
    DEFINE_REG32(cfg_clktrim_2);
    uint32_t regs[0xED4>>2];
} tegra_emc;

static const VMStateDescription vmstate_tegra_emc = {
    .name = "tegra.emc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(intstatus.reg32, tegra_emc),
        VMSTATE_UINT32(intmask.reg32, tegra_emc),
        VMSTATE_UINT32(dbg.reg32, tegra_emc),
        VMSTATE_UINT32(cfg.reg32, tegra_emc),
        VMSTATE_UINT32(adr_cfg.reg32, tegra_emc),
        VMSTATE_UINT32(adr_cfg_1.reg32, tegra_emc),
        VMSTATE_UINT32(refctrl.reg32, tegra_emc),
        VMSTATE_UINT32(pin.reg32, tegra_emc),
        VMSTATE_UINT32(timing_control.reg32, tegra_emc),
        VMSTATE_UINT32(rc.reg32, tegra_emc),
        VMSTATE_UINT32(rfc.reg32, tegra_emc),
        VMSTATE_UINT32(ras.reg32, tegra_emc),
        VMSTATE_UINT32(rp.reg32, tegra_emc),
        VMSTATE_UINT32(r2w.reg32, tegra_emc),
        VMSTATE_UINT32(w2r.reg32, tegra_emc),
        VMSTATE_UINT32(r2p.reg32, tegra_emc),
        VMSTATE_UINT32(w2p.reg32, tegra_emc),
        VMSTATE_UINT32(rd_rcd.reg32, tegra_emc),
        VMSTATE_UINT32(wr_rcd.reg32, tegra_emc),
        VMSTATE_UINT32(rrd.reg32, tegra_emc),
        VMSTATE_UINT32(rext.reg32, tegra_emc),
        VMSTATE_UINT32(wdv.reg32, tegra_emc),
        VMSTATE_UINT32(quse.reg32, tegra_emc),
        VMSTATE_UINT32(qrst.reg32, tegra_emc),
        VMSTATE_UINT32(qsafe.reg32, tegra_emc),
        VMSTATE_UINT32(rdv.reg32, tegra_emc),
        VMSTATE_UINT32(refresh.reg32, tegra_emc),
        VMSTATE_UINT32(burst_refresh_num.reg32, tegra_emc),
        VMSTATE_UINT32(pdex2wr.reg32, tegra_emc),
        VMSTATE_UINT32(pdex2rd.reg32, tegra_emc),
        VMSTATE_UINT32(pchg2pden.reg32, tegra_emc),
        VMSTATE_UINT32(act2pden.reg32, tegra_emc),
        VMSTATE_UINT32(ar2pden.reg32, tegra_emc),
        VMSTATE_UINT32(rw2pden.reg32, tegra_emc),
        VMSTATE_UINT32(txsr.reg32, tegra_emc),
        VMSTATE_UINT32(tcke.reg32, tegra_emc),
        VMSTATE_UINT32(tfaw.reg32, tegra_emc),
        VMSTATE_UINT32(trpab.reg32, tegra_emc),
        VMSTATE_UINT32(tclkstable.reg32, tegra_emc),
        VMSTATE_UINT32(tclkstop.reg32, tegra_emc),
        VMSTATE_UINT32(trefbw.reg32, tegra_emc),
        VMSTATE_UINT32(quse_extra.reg32, tegra_emc),
        VMSTATE_UINT32(odt_write.reg32, tegra_emc),
        VMSTATE_UINT32(odt_read.reg32, tegra_emc),
        VMSTATE_UINT32(mrs.reg32, tegra_emc),
        VMSTATE_UINT32(emrs.reg32, tegra_emc),
        VMSTATE_UINT32(ref.reg32, tegra_emc),
        VMSTATE_UINT32(pre.reg32, tegra_emc),
        VMSTATE_UINT32(nop.reg32, tegra_emc),
        VMSTATE_UINT32(self_ref.reg32, tegra_emc),
        VMSTATE_UINT32(dpd.reg32, tegra_emc),
        VMSTATE_UINT32(mrw.reg32, tegra_emc),
        VMSTATE_UINT32(mrr.reg32, tegra_emc),
        VMSTATE_UINT32(fbio_cfg1.reg32, tegra_emc),
        VMSTATE_UINT32(fbio_dqsib_dly.reg32, tegra_emc),
        VMSTATE_UINT32(fbio_dqsib_dly_msb.reg32, tegra_emc),
        VMSTATE_UINT32(fbio_cfg5.reg32, tegra_emc),
        VMSTATE_UINT32(fbio_quse_dly.reg32, tegra_emc),
        VMSTATE_UINT32(fbio_quse_dly_msb.reg32, tegra_emc),
        VMSTATE_UINT32(fbio_cfg6.reg32, tegra_emc),
        VMSTATE_UINT32(dqs_trimmer_rd0.reg32, tegra_emc),
        VMSTATE_UINT32(dqs_trimmer_rd1.reg32, tegra_emc),
        VMSTATE_UINT32(dqs_trimmer_rd2.reg32, tegra_emc),
        VMSTATE_UINT32(dqs_trimmer_rd3.reg32, tegra_emc),
        VMSTATE_UINT32(ll_arb_config.reg32, tegra_emc),
        VMSTATE_UINT32(t_min_critical_hp.reg32, tegra_emc),
        VMSTATE_UINT32(t_min_critical_timeout.reg32, tegra_emc),
        VMSTATE_UINT32(t_min_load.reg32, tegra_emc),
        VMSTATE_UINT32(t_max_critical_hp.reg32, tegra_emc),
        VMSTATE_UINT32(t_max_critical_timeout.reg32, tegra_emc),
        VMSTATE_UINT32(t_max_load.reg32, tegra_emc),
        VMSTATE_UINT32(auto_cal_config.reg32, tegra_emc),
        VMSTATE_UINT32(auto_cal_interval.reg32, tegra_emc),
        VMSTATE_UINT32(auto_cal_status.reg32, tegra_emc),
        VMSTATE_UINT32(req_ctrl.reg32, tegra_emc),
        VMSTATE_UINT32(emc_status.reg32, tegra_emc),
        VMSTATE_UINT32(cfg_2.reg32, tegra_emc),
        VMSTATE_UINT32(cfg_dig_dll.reg32, tegra_emc),
        VMSTATE_UINT32(dll_xform_dqs.reg32, tegra_emc),
        VMSTATE_UINT32(dll_xform_quse.reg32, tegra_emc),
        VMSTATE_UINT32(dig_dll_upper_status.reg32, tegra_emc),
        VMSTATE_UINT32(dig_dll_lower_status.reg32, tegra_emc),
        VMSTATE_UINT32(ctt_term_ctrl.reg32, tegra_emc),
        VMSTATE_UINT32(zcal_ref_cnt.reg32, tegra_emc),
        VMSTATE_UINT32(zcal_wait_cnt.reg32, tegra_emc),
        VMSTATE_UINT32(zcal_mrw_cmd.reg32, tegra_emc),
        VMSTATE_UINT32(cmdq.reg32, tegra_emc),
        VMSTATE_UINT32(fbio_spare.reg32, tegra_emc),
        VMSTATE_UINT32(fbio_wrptr_eq_2.reg32, tegra_emc),
        VMSTATE_UINT32(clken_override.reg32, tegra_emc),
        VMSTATE_UINT32(stat_control.reg32, tegra_emc),
        VMSTATE_UINT32(stat_status.reg32, tegra_emc),
        VMSTATE_UINT32(stat_llmc_addr_low.reg32, tegra_emc),
        VMSTATE_UINT32(stat_llmc_addr_high.reg32, tegra_emc),
        VMSTATE_UINT32(stat_llmc_clock_limit.reg32, tegra_emc),
        VMSTATE_UINT32(stat_llmc_clocks.reg32, tegra_emc),
        VMSTATE_UINT32(stat_llmc_control.reg32, tegra_emc),
        VMSTATE_UINT32(stat_llmc_hist_limit.reg32, tegra_emc),
        VMSTATE_UINT32(stat_llmc_count.reg32, tegra_emc),
        VMSTATE_UINT32(stat_llmc_hist.reg32, tegra_emc),
        VMSTATE_UINT32(stat_pwr_clock_limit.reg32, tegra_emc),
        VMSTATE_UINT32(stat_pwr_clocks.reg32, tegra_emc),
        VMSTATE_UINT32(stat_pwr_count.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_clock_limit_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_clock_limit_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_clocks_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_clocks_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_activate_cnt_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_activate_cnt_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_read_cnt_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_read_cnt_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_write_cnt_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_write_cnt_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_ref_cnt_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_ref_cnt_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_cumm_banks_active_cke_eq1_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_cumm_banks_active_cke_eq1_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_cumm_banks_active_cke_eq0_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_cumm_banks_active_cke_eq0_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_cke_eq1_clks_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_cke_eq1_clks_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_extclks_cke_eq1_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_extclks_cke_eq1_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_extclks_cke_eq0_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_extclks_cke_eq0_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_activate_cnt_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_activate_cnt_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_read_cnt_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_read_cnt_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_write_cnt_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_write_cnt_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_ref_cnt_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_ref_cnt_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_cumm_banks_active_cke_eq1_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_cumm_banks_active_cke_eq1_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_cumm_banks_active_cke_eq0_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_cumm_banks_active_cke_eq0_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_cke_eq1_clks_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_cke_eq1_clks_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_extclks_cke_eq1_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_extclks_cke_eq1_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_extclks_cke_eq0_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_extclks_cke_eq0_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_no_banks_active_cke_eq1_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_no_banks_active_cke_eq1_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_no_banks_active_cke_eq0_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev0_no_banks_active_cke_eq0_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_no_banks_active_cke_eq1_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_no_banks_active_cke_eq1_hi.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_no_banks_active_cke_eq0_lo.reg32, tegra_emc),
        VMSTATE_UINT32(stat_dram_dev1_no_banks_active_cke_eq0_hi.reg32, tegra_emc),
        VMSTATE_UINT32(cfg_clktrim.reg32, tegra_emc),
        VMSTATE_UINT32(cfg_clktrim_1.reg32, tegra_emc),
        VMSTATE_UINT32(cfg_clktrim_2.reg32, tegra_emc),
        VMSTATE_UINT32_ARRAY(regs, tegra_emc, 0xED4>>2),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_emc_priv_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    tegra_emc *s = opaque;
    uint64_t ret = 0;
    tegra_emc *emc_chan = NULL;

    switch (offset) {
    case INTSTATUS_OFFSET:
        ret = s->intstatus.reg32;
        break;
    case INTMASK_OFFSET:
        ret = s->intmask.reg32;
        break;
    case DBG_OFFSET:
        ret = s->dbg.reg32;
        break;
    case CFG_OFFSET:
        ret = s->cfg.reg32;
        break;
    case ADR_CFG_OFFSET:
        ret = s->adr_cfg.reg32;
        break;
    case ADR_CFG_1_OFFSET:
        ret = s->adr_cfg_1.reg32;
        break;
    case REFCTRL_OFFSET:
        ret = s->refctrl.reg32;
        break;
    case PIN_OFFSET:
        ret = s->pin.reg32;
        break;
    case TIMING_CONTROL_OFFSET:
        ret = s->timing_control.reg32;
        break;
    case RC_OFFSET:
        ret = s->rc.reg32;
        break;
    case RFC_OFFSET:
        ret = s->rfc.reg32;
        break;
    case RAS_OFFSET:
        ret = s->ras.reg32;
        break;
    case RP_OFFSET:
        ret = s->rp.reg32;
        break;
    case R2W_OFFSET:
        ret = s->r2w.reg32;
        break;
    case W2R_OFFSET:
        ret = s->w2r.reg32;
        break;
    case R2P_OFFSET:
        ret = s->r2p.reg32;
        break;
    case W2P_OFFSET:
        ret = s->w2p.reg32;
        break;
    case RD_RCD_OFFSET:
        ret = s->rd_rcd.reg32;
        break;
    case WR_RCD_OFFSET:
        ret = s->wr_rcd.reg32;
        break;
    case RRD_OFFSET:
        ret = s->rrd.reg32;
        break;
    case REXT_OFFSET:
        ret = s->rext.reg32;
        break;
    case WDV_OFFSET:
        ret = s->wdv.reg32;
        break;
    case QUSE_OFFSET:
        ret = s->quse.reg32;
        break;
    case QRST_OFFSET:
        ret = s->qrst.reg32;
        break;
    case QSAFE_OFFSET:
        ret = s->qsafe.reg32;
        break;
    case RDV_OFFSET:
        ret = s->rdv.reg32;
        break;
    case REFRESH_OFFSET:
        ret = s->refresh.reg32;
        break;
    case BURST_REFRESH_NUM_OFFSET:
        ret = s->burst_refresh_num.reg32;
        break;
    case PDEX2WR_OFFSET:
        ret = s->pdex2wr.reg32;
        break;
    case PDEX2RD_OFFSET:
        ret = s->pdex2rd.reg32;
        break;
    case PCHG2PDEN_OFFSET:
        ret = s->pchg2pden.reg32;
        break;
    case ACT2PDEN_OFFSET:
        ret = s->act2pden.reg32;
        break;
    case AR2PDEN_OFFSET:
        ret = s->ar2pden.reg32;
        break;
    case RW2PDEN_OFFSET:
        ret = s->rw2pden.reg32;
        break;
    case TXSR_OFFSET:
        ret = s->txsr.reg32;
        break;
    case TCKE_OFFSET:
        ret = s->tcke.reg32;
        break;
    case TFAW_OFFSET:
        ret = s->tfaw.reg32;
        break;
    case TRPAB_OFFSET:
        ret = s->trpab.reg32;
        break;
    case TCLKSTABLE_OFFSET:
        ret = s->tclkstable.reg32;
        break;
    case TCLKSTOP_OFFSET:
        ret = s->tclkstop.reg32;
        break;
    case TREFBW_OFFSET:
        ret = s->trefbw.reg32;
        break;
    case QUSE_EXTRA_OFFSET:
        ret = s->quse_extra.reg32;
        break;
    case ODT_WRITE_OFFSET:
        ret = s->odt_write.reg32;
        break;
    case ODT_READ_OFFSET:
        ret = s->odt_read.reg32;
        break;
    case MRS_OFFSET:
        ret = s->mrs.reg32;
        break;
    case EMRS_OFFSET:
        ret = s->emrs.reg32;
        break;
    case REF_OFFSET:
        ret = s->ref.reg32;
        break;
    case PRE_OFFSET:
        ret = s->pre.reg32;
        break;
    case NOP_OFFSET:
        ret = s->nop.reg32;
        break;
    case SELF_REF_OFFSET:
        ret = s->self_ref.reg32;
        break;
    case DPD_OFFSET:
        ret = s->dpd.reg32;
        break;
    case MRW_OFFSET:
        ret = s->mrw.reg32;
        break;
    case MRR_OFFSET:
        ret = s->mrr.reg32;
        if (tegra_board >= TEGRAX1_BOARD) {
            s->emc_status.reg32 &= ~0x100000; // MRR_DIVLD
            s->emc_status.reg32 |= 0xF << 16; // MRR_FIFO_SPACE
            emc_chan = tegra_emc0_dev;
            emc_chan->emc_status.reg32 &= ~0x100000;
            emc_chan->emc_status.reg32 |= 0xF << 16;
            emc_chan = tegra_emc1_dev;
            emc_chan->emc_status.reg32 &= ~0x100000;
            emc_chan->emc_status.reg32 |= 0xF << 16;
        }
        break;
    case FBIO_CFG1_OFFSET:
        ret = s->fbio_cfg1.reg32;
        break;
    case FBIO_DQSIB_DLY_OFFSET:
        ret = s->fbio_dqsib_dly.reg32;
        break;
    case FBIO_DQSIB_DLY_MSB_OFFSET:
        ret = s->fbio_dqsib_dly_msb.reg32;
        break;
    case FBIO_CFG5_OFFSET:
        ret = s->fbio_cfg5.reg32;
        break;
    case FBIO_QUSE_DLY_OFFSET:
        ret = s->fbio_quse_dly.reg32;
        break;
    case FBIO_QUSE_DLY_MSB_OFFSET:
        ret = s->fbio_quse_dly_msb.reg32;
        break;
    case FBIO_CFG6_OFFSET:
        ret = s->fbio_cfg6.reg32;
        break;
    case DQS_TRIMMER_RD0_OFFSET:
        ret = s->dqs_trimmer_rd0.reg32;
        break;
    case DQS_TRIMMER_RD1_OFFSET:
        ret = s->dqs_trimmer_rd1.reg32;
        break;
    case DQS_TRIMMER_RD2_OFFSET:
        ret = s->dqs_trimmer_rd2.reg32;
        break;
    case DQS_TRIMMER_RD3_OFFSET:
        ret = s->dqs_trimmer_rd3.reg32;
        break;
    case LL_ARB_CONFIG_OFFSET:
        ret = s->ll_arb_config.reg32;
        break;
    case T_MIN_CRITICAL_HP_OFFSET:
        ret = s->t_min_critical_hp.reg32;
        break;
    case T_MIN_CRITICAL_TIMEOUT_OFFSET:
        ret = s->t_min_critical_timeout.reg32;
        break;
    case T_MIN_LOAD_OFFSET:
        ret = s->t_min_load.reg32;
        break;
    case T_MAX_CRITICAL_HP_OFFSET:
        ret = s->t_max_critical_hp.reg32;
        break;
    case T_MAX_CRITICAL_TIMEOUT_OFFSET:
        ret = s->t_max_critical_timeout.reg32;
        break;
    case T_MAX_LOAD_OFFSET:
        ret = s->t_max_load.reg32;
        break;
    case AUTO_CAL_CONFIG_OFFSET:
        ret = s->auto_cal_config.reg32;
        break;
    case AUTO_CAL_INTERVAL_OFFSET:
        ret = s->auto_cal_interval.reg32;
        break;
    case AUTO_CAL_STATUS_OFFSET:
        ret = s->auto_cal_status.reg32;
        break;
    case REQ_CTRL_OFFSET:
        ret = s->req_ctrl.reg32;
        break;
    case EMC_STATUS_OFFSET:
        ret = s->emc_status.reg32;
        break;
    case CFG_2_OFFSET:
        ret = s->cfg_2.reg32;
        break;
    case CFG_DIG_DLL_OFFSET:
        ret = s->cfg_dig_dll.reg32;
        break;
    case DLL_XFORM_DQS_OFFSET:
        ret = s->dll_xform_dqs.reg32;
        break;
    case DLL_XFORM_QUSE_OFFSET:
        ret = s->dll_xform_quse.reg32;
        break;
    case DIG_DLL_UPPER_STATUS_OFFSET:
        ret = s->dig_dll_upper_status.reg32;
        break;
    case DIG_DLL_LOWER_STATUS_OFFSET:
        ret = s->dig_dll_lower_status.reg32;
        break;
    case CTT_TERM_CTRL_OFFSET:
        ret = s->ctt_term_ctrl.reg32;
        break;
    case ZCAL_REF_CNT_OFFSET:
        ret = s->zcal_ref_cnt.reg32;
        break;
    case ZCAL_WAIT_CNT_OFFSET:
        ret = s->zcal_wait_cnt.reg32;
        break;
    case ZCAL_MRW_CMD_OFFSET:
        ret = s->zcal_mrw_cmd.reg32;
        break;
    case CMDQ_OFFSET:
        ret = s->cmdq.reg32;
        break;
    case FBIO_SPARE_OFFSET:
        ret = s->fbio_spare.reg32;
        break;
    case FBIO_WRPTR_EQ_2_OFFSET:
        ret = s->fbio_wrptr_eq_2.reg32;
        break;
    case CLKEN_OVERRIDE_OFFSET:
        ret = s->clken_override.reg32;
        break;
    case STAT_CONTROL_OFFSET:
        ret = s->stat_control.reg32;
        break;
    case STAT_STATUS_OFFSET:
        ret = s->stat_status.reg32;
        break;
    case STAT_LLMC_ADDR_LOW_OFFSET:
        ret = s->stat_llmc_addr_low.reg32;
        break;
    case STAT_LLMC_ADDR_HIGH_OFFSET:
        ret = s->stat_llmc_addr_high.reg32;
        break;
    case STAT_LLMC_CLOCK_LIMIT_OFFSET:
        ret = s->stat_llmc_clock_limit.reg32;
        break;
    case STAT_LLMC_CLOCKS_OFFSET:
        ret = s->stat_llmc_clocks.reg32;
        break;
    case STAT_LLMC_CONTROL_OFFSET:
        ret = s->stat_llmc_control.reg32;
        break;
    case STAT_LLMC_HIST_LIMIT_OFFSET:
        ret = s->stat_llmc_hist_limit.reg32;
        break;
    case STAT_LLMC_COUNT_OFFSET:
        ret = s->stat_llmc_count.reg32;
        break;
    case STAT_LLMC_HIST_OFFSET:
        ret = s->stat_llmc_hist.reg32;
        break;
    case STAT_PWR_CLOCK_LIMIT_OFFSET:
        ret = s->stat_pwr_clock_limit.reg32;
        break;
    case STAT_PWR_CLOCKS_OFFSET:
        ret = s->stat_pwr_clocks.reg32;
        break;
    case STAT_PWR_COUNT_OFFSET:
        ret = s->stat_pwr_count.reg32;
        break;
    case STAT_DRAM_CLOCK_LIMIT_LO_OFFSET:
        ret = s->stat_dram_clock_limit_lo.reg32;
        break;
    case STAT_DRAM_CLOCK_LIMIT_HI_OFFSET:
        ret = s->stat_dram_clock_limit_hi.reg32;
        break;
    case STAT_DRAM_CLOCKS_LO_OFFSET:
        ret = s->stat_dram_clocks_lo.reg32;
        break;
    case STAT_DRAM_CLOCKS_HI_OFFSET:
        ret = s->stat_dram_clocks_hi.reg32;
        break;
    case STAT_DRAM_DEV0_ACTIVATE_CNT_LO_OFFSET:
        ret = s->stat_dram_dev0_activate_cnt_lo.reg32;
        break;
    case STAT_DRAM_DEV0_ACTIVATE_CNT_HI_OFFSET:
        ret = s->stat_dram_dev0_activate_cnt_hi.reg32;
        break;
    case STAT_DRAM_DEV0_READ_CNT_LO_OFFSET:
        ret = s->stat_dram_dev0_read_cnt_lo.reg32;
        break;
    case STAT_DRAM_DEV0_READ_CNT_HI_OFFSET:
        ret = s->stat_dram_dev0_read_cnt_hi.reg32;
        break;
    case STAT_DRAM_DEV0_WRITE_CNT_LO_OFFSET:
        ret = s->stat_dram_dev0_write_cnt_lo.reg32;
        break;
    case STAT_DRAM_DEV0_WRITE_CNT_HI_OFFSET:
        ret = s->stat_dram_dev0_write_cnt_hi.reg32;
        break;
    case STAT_DRAM_DEV0_REF_CNT_LO_OFFSET:
        ret = s->stat_dram_dev0_ref_cnt_lo.reg32;
        break;
    case STAT_DRAM_DEV0_REF_CNT_HI_OFFSET:
        ret = s->stat_dram_dev0_ref_cnt_hi.reg32;
        break;
    case STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ1_LO_OFFSET:
        ret = s->stat_dram_dev0_cumm_banks_active_cke_eq1_lo.reg32;
        break;
    case STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ1_HI_OFFSET:
        ret = s->stat_dram_dev0_cumm_banks_active_cke_eq1_hi.reg32;
        break;
    case STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ0_LO_OFFSET:
        ret = s->stat_dram_dev0_cumm_banks_active_cke_eq0_lo.reg32;
        break;
    case STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ0_HI_OFFSET:
        ret = s->stat_dram_dev0_cumm_banks_active_cke_eq0_hi.reg32;
        break;
    case STAT_DRAM_DEV0_CKE_EQ1_CLKS_LO_OFFSET:
        ret = s->stat_dram_dev0_cke_eq1_clks_lo.reg32;
        break;
    case STAT_DRAM_DEV0_CKE_EQ1_CLKS_HI_OFFSET:
        ret = s->stat_dram_dev0_cke_eq1_clks_hi.reg32;
        break;
    case STAT_DRAM_DEV0_EXTCLKS_CKE_EQ1_LO_OFFSET:
        ret = s->stat_dram_dev0_extclks_cke_eq1_lo.reg32;
        break;
    case STAT_DRAM_DEV0_EXTCLKS_CKE_EQ1_HI_OFFSET:
        ret = s->stat_dram_dev0_extclks_cke_eq1_hi.reg32;
        break;
    case STAT_DRAM_DEV0_EXTCLKS_CKE_EQ0_LO_OFFSET:
        ret = s->stat_dram_dev0_extclks_cke_eq0_lo.reg32;
        break;
    case STAT_DRAM_DEV0_EXTCLKS_CKE_EQ0_HI_OFFSET:
        ret = s->stat_dram_dev0_extclks_cke_eq0_hi.reg32;
        break;
    case STAT_DRAM_DEV1_ACTIVATE_CNT_LO_OFFSET:
        ret = s->stat_dram_dev1_activate_cnt_lo.reg32;
        break;
    case STAT_DRAM_DEV1_ACTIVATE_CNT_HI_OFFSET:
        ret = s->stat_dram_dev1_activate_cnt_hi.reg32;
        break;
    case STAT_DRAM_DEV1_READ_CNT_LO_OFFSET:
        ret = s->stat_dram_dev1_read_cnt_lo.reg32;
        break;
    case STAT_DRAM_DEV1_READ_CNT_HI_OFFSET:
        ret = s->stat_dram_dev1_read_cnt_hi.reg32;
        break;
    case STAT_DRAM_DEV1_WRITE_CNT_LO_OFFSET:
        ret = s->stat_dram_dev1_write_cnt_lo.reg32;
        break;
    case STAT_DRAM_DEV1_WRITE_CNT_HI_OFFSET:
        ret = s->stat_dram_dev1_write_cnt_hi.reg32;
        break;
    case STAT_DRAM_DEV1_REF_CNT_LO_OFFSET:
        ret = s->stat_dram_dev1_ref_cnt_lo.reg32;
        break;
    case STAT_DRAM_DEV1_REF_CNT_HI_OFFSET:
        ret = s->stat_dram_dev1_ref_cnt_hi.reg32;
        break;
    case STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ1_LO_OFFSET:
        ret = s->stat_dram_dev1_cumm_banks_active_cke_eq1_lo.reg32;
        break;
    case STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ1_HI_OFFSET:
        ret = s->stat_dram_dev1_cumm_banks_active_cke_eq1_hi.reg32;
        break;
    case STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ0_LO_OFFSET:
        ret = s->stat_dram_dev1_cumm_banks_active_cke_eq0_lo.reg32;
        break;
    case STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ0_HI_OFFSET:
        ret = s->stat_dram_dev1_cumm_banks_active_cke_eq0_hi.reg32;
        break;
    case STAT_DRAM_DEV1_CKE_EQ1_CLKS_LO_OFFSET:
        ret = s->stat_dram_dev1_cke_eq1_clks_lo.reg32;
        break;
    case STAT_DRAM_DEV1_CKE_EQ1_CLKS_HI_OFFSET:
        ret = s->stat_dram_dev1_cke_eq1_clks_hi.reg32;
        break;
    case STAT_DRAM_DEV1_EXTCLKS_CKE_EQ1_LO_OFFSET:
        ret = s->stat_dram_dev1_extclks_cke_eq1_lo.reg32;
        break;
    case STAT_DRAM_DEV1_EXTCLKS_CKE_EQ1_HI_OFFSET:
        ret = s->stat_dram_dev1_extclks_cke_eq1_hi.reg32;
        break;
    case STAT_DRAM_DEV1_EXTCLKS_CKE_EQ0_LO_OFFSET:
        ret = s->stat_dram_dev1_extclks_cke_eq0_lo.reg32;
        break;
    case STAT_DRAM_DEV1_EXTCLKS_CKE_EQ0_HI_OFFSET:
        ret = s->stat_dram_dev1_extclks_cke_eq0_hi.reg32;
        break;
    case STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ1_LO_OFFSET:
        ret = s->stat_dram_dev0_no_banks_active_cke_eq1_lo.reg32;
        break;
    case STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ1_HI_OFFSET:
        ret = s->stat_dram_dev0_no_banks_active_cke_eq1_hi.reg32;
        break;
    case STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ0_LO_OFFSET:
        ret = s->stat_dram_dev0_no_banks_active_cke_eq0_lo.reg32;
        break;
    case STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ0_HI_OFFSET:
        ret = s->stat_dram_dev0_no_banks_active_cke_eq0_hi.reg32;
        break;
    case STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ1_LO_OFFSET:
        ret = s->stat_dram_dev1_no_banks_active_cke_eq1_lo.reg32;
        break;
    case STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ1_HI_OFFSET:
        ret = s->stat_dram_dev1_no_banks_active_cke_eq1_hi.reg32;
        break;
    case STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ0_LO_OFFSET:
        ret = s->stat_dram_dev1_no_banks_active_cke_eq0_lo.reg32;
        break;
    case STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ0_HI_OFFSET:
        ret = s->stat_dram_dev1_no_banks_active_cke_eq0_hi.reg32;
        break;
    case CFG_CLKTRIM_OFFSET:
        ret = s->cfg_clktrim.reg32;
        break;
    case CFG_CLKTRIM_1_OFFSET:
        ret = s->cfg_clktrim_1.reg32;
        break;
    case CFG_CLKTRIM_2_OFFSET:
        ret = s->cfg_clktrim_2.reg32;
        break;
    default:
        if (offset+size <= sizeof(s->regs)) ret = s->regs[offset>>2];
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_emc_priv_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    tegra_emc *s = opaque;
    tegra_emc *emc_chan = NULL;

    switch (offset) {
    case INTSTATUS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->intstatus.reg32, value);
        s->intstatus.reg32 = value;
        if (tegra_board >= TEGRAX1_BOARD) {
            emc_chan = tegra_emc0_dev;
            emc_chan->intstatus.reg32 = value;
            emc_chan = tegra_emc1_dev;
            emc_chan->intstatus.reg32 = value;
        }
        break;
    case INTMASK_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->intmask.reg32, value);
        s->intmask.reg32 = value;
        break;
    case DBG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->dbg.reg32, value);
        s->dbg.reg32 = value;
        break;
    case CFG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->cfg.reg32, value);
        s->cfg.reg32 = value;
        break;
    case ADR_CFG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->adr_cfg.reg32, value);
        s->adr_cfg.reg32 = value;
        break;
    case ADR_CFG_1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->adr_cfg_1.reg32, value);
        s->adr_cfg_1.reg32 = value;
        break;
    case REFCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->refctrl.reg32, value);
        s->refctrl.reg32 = value;
        break;
    case PIN_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pin.reg32, value);
        s->pin.reg32 = value;
        break;
    case TIMING_CONTROL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->timing_control.reg32, value);
        s->timing_control.reg32 = value;
        break;
    case RC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rc.reg32, value);
        s->rc.reg32 = value;
        break;
    case RFC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rfc.reg32, value);
        s->rfc.reg32 = value;
        break;
    case RAS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ras.reg32, value);
        s->ras.reg32 = value;
        break;
    case RP_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rp.reg32, value);
        s->rp.reg32 = value;
        break;
    case R2W_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->r2w.reg32, value);
        s->r2w.reg32 = value;
        break;
    case W2R_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->w2r.reg32, value);
        s->w2r.reg32 = value;
        break;
    case R2P_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->r2p.reg32, value);
        s->r2p.reg32 = value;
        break;
    case W2P_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->w2p.reg32, value);
        s->w2p.reg32 = value;
        break;
    case RD_RCD_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rd_rcd.reg32, value);
        s->rd_rcd.reg32 = value;
        break;
    case WR_RCD_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->wr_rcd.reg32, value);
        s->wr_rcd.reg32 = value;
        break;
    case RRD_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rrd.reg32, value);
        s->rrd.reg32 = value;
        break;
    case REXT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rext.reg32, value);
        s->rext.reg32 = value;
        break;
    case WDV_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->wdv.reg32, value);
        s->wdv.reg32 = value;
        break;
    case QUSE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->quse.reg32, value);
        s->quse.reg32 = value;
        break;
    case QRST_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->qrst.reg32, value);
        s->qrst.reg32 = value;
        break;
    case QSAFE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->qsafe.reg32, value);
        s->qsafe.reg32 = value;
        break;
    case RDV_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rdv.reg32, value);
        s->rdv.reg32 = value;
        break;
    case REFRESH_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->refresh.reg32, value);
        s->refresh.reg32 = value;
        break;
    case BURST_REFRESH_NUM_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->burst_refresh_num.reg32, value);
        s->burst_refresh_num.reg32 = value;
        break;
    case PDEX2WR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pdex2wr.reg32, value);
        s->pdex2wr.reg32 = value;
        break;
    case PDEX2RD_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pdex2rd.reg32, value);
        s->pdex2rd.reg32 = value;
        break;
    case PCHG2PDEN_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pchg2pden.reg32, value);
        s->pchg2pden.reg32 = value;
        break;
    case ACT2PDEN_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->act2pden.reg32, value);
        s->act2pden.reg32 = value;
        break;
    case AR2PDEN_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ar2pden.reg32, value);
        s->ar2pden.reg32 = value;
        break;
    case RW2PDEN_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rw2pden.reg32, value);
        s->rw2pden.reg32 = value;
        break;
    case TXSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->txsr.reg32, value);
        s->txsr.reg32 = value;
        break;
    case TCKE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->tcke.reg32, value);
        s->tcke.reg32 = value;
        break;
    case TFAW_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->tfaw.reg32, value);
        s->tfaw.reg32 = value;
        break;
    case TRPAB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->trpab.reg32, value);
        s->trpab.reg32 = value;
        break;
    case TCLKSTABLE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->tclkstable.reg32, value);
        s->tclkstable.reg32 = value;
        break;
    case TCLKSTOP_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->tclkstop.reg32, value);
        s->tclkstop.reg32 = value;
        break;
    case TREFBW_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->trefbw.reg32, value);
        s->trefbw.reg32 = value;
        break;
    case QUSE_EXTRA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->quse_extra.reg32, value);
        s->quse_extra.reg32 = value;
        break;
    case ODT_WRITE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->odt_write.reg32, value);
        s->odt_write.reg32 = value;
        break;
    case ODT_READ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->odt_read.reg32, value);
        s->odt_read.reg32 = value;
        break;
    case MRS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->mrs.reg32, value);
        s->mrs.reg32 = value;
        break;
    case EMRS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->emrs.reg32, value);
        s->emrs.reg32 = value;
        break;
    case REF_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ref.reg32, value);
        s->ref.reg32 = value;
        break;
    case PRE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pre.reg32, value);
        s->pre.reg32 = value;
        break;
    case NOP_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->nop.reg32, value);
        s->nop.reg32 = value;
        break;
    case SELF_REF_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->self_ref.reg32, value);
        s->self_ref.reg32 = value;

        s->emc_status.dram_in_self_refresh = 0;
        emc_chan = tegra_emc0_dev;
        if (emc_chan) emc_chan->emc_status.dram_in_self_refresh = 0;
        emc_chan = tegra_emc1_dev;
        if (emc_chan) emc_chan->emc_status.dram_in_self_refresh = 0;

        if (s->self_ref.self_ref_cmd) {
            int i;

            uint32_t tmp = ~s->self_ref.sref_dev_selectn & 0x3;
            int numdev = tegra_board < TEGRAX1_BOARD ? s->adr_cfg.emem_numdev : s->adr_cfg.reg32 & 0x1;
            for (i = 0; i < numdev + 1; i++) {
                if (tmp & (1<<i)) {
                    s->emc_status.dram_in_self_refresh |= 1 << i;
                    emc_chan = tegra_emc0_dev;
                    if (emc_chan) emc_chan->emc_status.dram_in_self_refresh |= 1 << i;
                    emc_chan = tegra_emc1_dev;
                    if (emc_chan) emc_chan->emc_status.dram_in_self_refresh |= 1 << i;
                }
            }
        }

        break;
    case DPD_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->dpd.reg32, value);
        s->dpd.reg32 = value;
        break;
    case MRW_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->mrw.reg32, value);
        s->mrw.reg32 = value;
        break;
    case MRR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->mrr.reg32, value);
        s->mrr.reg32 = value;
        if (tegra_board >= TEGRAX1_BOARD) {
            s->emc_status.reg32 |= 0x100000; // MRR_DIVLD
            emc_chan = tegra_emc0_dev;
            emc_chan->emc_status.reg32 |= 0x100000;
            emc_chan = tegra_emc1_dev;
            emc_chan->emc_status.reg32 |= 0x100000;
        }
        break;
    case FBIO_CFG1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fbio_cfg1.reg32, value);
        s->fbio_cfg1.reg32 = value;
        break;
    case FBIO_DQSIB_DLY_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fbio_dqsib_dly.reg32, value);
        s->fbio_dqsib_dly.reg32 = value;
        break;
    case FBIO_DQSIB_DLY_MSB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fbio_dqsib_dly_msb.reg32, value);
        s->fbio_dqsib_dly_msb.reg32 = value;
        break;
    case FBIO_CFG5_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fbio_cfg5.reg32, value);
        s->fbio_cfg5.reg32 = value;
        break;
    case FBIO_QUSE_DLY_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fbio_quse_dly.reg32, value);
        s->fbio_quse_dly.reg32 = value;
        break;
    case FBIO_QUSE_DLY_MSB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fbio_quse_dly_msb.reg32, value);
        s->fbio_quse_dly_msb.reg32 = value;
        break;
    case FBIO_CFG6_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fbio_cfg6.reg32, value);
        s->fbio_cfg6.reg32 = value;
        break;
    case LL_ARB_CONFIG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ll_arb_config.reg32, value);
        s->ll_arb_config.reg32 = value;
        break;
    case T_MIN_CRITICAL_HP_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->t_min_critical_hp.reg32, value);
        s->t_min_critical_hp.reg32 = value;
        break;
    case T_MIN_CRITICAL_TIMEOUT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->t_min_critical_timeout.reg32, value);
        s->t_min_critical_timeout.reg32 = value;
        break;
    case T_MIN_LOAD_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->t_min_load.reg32, value);
        s->t_min_load.reg32 = value;
        break;
    case T_MAX_CRITICAL_HP_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->t_max_critical_hp.reg32, value);
        s->t_max_critical_hp.reg32 = value;
        break;
    case T_MAX_CRITICAL_TIMEOUT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->t_max_critical_timeout.reg32, value);
        s->t_max_critical_timeout.reg32 = value;
        break;
    case T_MAX_LOAD_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->t_max_load.reg32, value);
        s->t_max_load.reg32 = value;
        break;
    case AUTO_CAL_CONFIG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->auto_cal_config.reg32, value & AUTO_CAL_CONFIG_WRMASK);
        WR_MASKED(s->auto_cal_config.reg32, value, AUTO_CAL_CONFIG);
        break;
    case AUTO_CAL_INTERVAL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->auto_cal_interval.reg32, value);
        s->auto_cal_interval.reg32 = value;
        break;
    case REQ_CTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->req_ctrl.reg32, value);
        s->req_ctrl.reg32 = value;
        break;
    case CFG_2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->cfg_2.reg32, value);
        s->cfg_2.reg32 = value;
        break;
    case CFG_DIG_DLL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->cfg_dig_dll.reg32, value & CFG_DIG_DLL_WRMASK);
        WR_MASKED(s->cfg_dig_dll.reg32, value, CFG_DIG_DLL);
        if (tegra_board >= TEGRAX1_BOARD) {
            uint32_t mask = 0x28000; // EMC_DIG_DLL_STATUS: DLL_LOCK, DLL_PRIV_UPDATED
            if (tegra_board == TEGRAX1PLUS_BOARD) mask |= 1 << 2; // TX1+: EMC_DIG_DLL_STATUS: DLL_LOCK
            s->dll_xform_quse.reg32 |= mask;
            emc_chan = tegra_emc0_dev;
            emc_chan->cfg_dig_dll.reg32 = s->cfg_dig_dll.reg32;
            emc_chan->dll_xform_quse.reg32 |= mask;
            emc_chan = tegra_emc1_dev;
            emc_chan->cfg_dig_dll.reg32 = s->cfg_dig_dll.reg32;
            emc_chan->dll_xform_quse.reg32 |= mask;
        }
        break;
    case DLL_XFORM_DQS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->dll_xform_dqs.reg32, value);
        s->dll_xform_dqs.reg32 = value;
        break;
    case DLL_XFORM_QUSE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->dll_xform_quse.reg32, value);
        s->dll_xform_quse.reg32 = value;
        break;
    case CTT_TERM_CTRL_OFFSET:
        //TRACE_WRITE(s->iomem.addr, offset, s->ctt_term_ctrl.reg32, value & CTT_TERM_CTRL_WRMASK);
        if (tegra_board < TEGRAX1_BOARD)
            WR_MASKED(s->ctt_term_ctrl.reg32, value, CTT_TERM_CTRL);
        else
            s->ctt_term_ctrl.reg32 = value;
        break;
    case ZCAL_REF_CNT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->zcal_ref_cnt.reg32, value);
        s->zcal_ref_cnt.reg32 = value;
        break;
    case ZCAL_WAIT_CNT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->zcal_wait_cnt.reg32, value);
        s->zcal_wait_cnt.reg32 = value;
        break;
    case ZCAL_MRW_CMD_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->zcal_mrw_cmd.reg32, value);
        s->zcal_mrw_cmd.reg32 = value;
        break;
    case CMDQ_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->cmdq.reg32, value);
        s->cmdq.reg32 = value;
        break;
    case FBIO_SPARE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fbio_spare.reg32, value);
        s->fbio_spare.reg32 = value;
        break;
    case FBIO_WRPTR_EQ_2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fbio_wrptr_eq_2.reg32, value & FBIO_WRPTR_EQ_2_WRMASK);
        WR_MASKED(s->fbio_wrptr_eq_2.reg32, value, FBIO_WRPTR_EQ_2);
        break;
    case CLKEN_OVERRIDE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clken_override.reg32, value);
        s->clken_override.reg32 = value;
        break;
    case STAT_CONTROL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_control.reg32, value);
        s->stat_control.reg32 = value;
        break;
    case STAT_STATUS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_status.reg32, value & STAT_STATUS_WRMASK);
        WR_MASKED(s->stat_status.reg32, value, STAT_STATUS);
        break;
    case STAT_LLMC_ADDR_LOW_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_llmc_addr_low.reg32, value);
        s->stat_llmc_addr_low.reg32 = value;
        break;
    case STAT_LLMC_ADDR_HIGH_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_llmc_addr_high.reg32, value);
        s->stat_llmc_addr_high.reg32 = value;
        break;
    case STAT_LLMC_CLOCK_LIMIT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_llmc_clock_limit.reg32, value);
        s->stat_llmc_clock_limit.reg32 = value;
        break;
    case STAT_LLMC_CONTROL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_llmc_control.reg32, value);
        s->stat_llmc_control.reg32 = value;
        break;
    case STAT_LLMC_HIST_LIMIT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_llmc_hist_limit.reg32, value);
        s->stat_llmc_hist_limit.reg32 = value;
        break;
    case STAT_PWR_CLOCK_LIMIT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_pwr_clock_limit.reg32, value);
        s->stat_pwr_clock_limit.reg32 = value;
        break;
    case STAT_DRAM_CLOCK_LIMIT_LO_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_clock_limit_lo.reg32, value);
        s->stat_dram_clock_limit_lo.reg32 = value;
        break;
    case STAT_DRAM_CLOCK_LIMIT_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_clock_limit_hi.reg32, value);
        s->stat_dram_clock_limit_hi.reg32 = value;
        break;
    case STAT_DRAM_CLOCKS_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_clocks_hi.reg32, value & STAT_DRAM_CLOCKS_HI_WRMASK);
        WR_MASKED(s->stat_dram_clocks_hi.reg32, value, STAT_DRAM_CLOCKS_HI);
        break;
    case STAT_DRAM_DEV0_ACTIVATE_CNT_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev0_activate_cnt_hi.reg32, value & STAT_DRAM_DEV0_ACTIVATE_CNT_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev0_activate_cnt_hi.reg32, value, STAT_DRAM_DEV0_ACTIVATE_CNT_HI);
        break;
    case STAT_DRAM_DEV0_READ_CNT_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev0_read_cnt_hi.reg32, value & STAT_DRAM_DEV0_READ_CNT_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev0_read_cnt_hi.reg32, value, STAT_DRAM_DEV0_READ_CNT_HI);
        break;
    case STAT_DRAM_DEV0_WRITE_CNT_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev0_write_cnt_hi.reg32, value & STAT_DRAM_DEV0_WRITE_CNT_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev0_write_cnt_hi.reg32, value, STAT_DRAM_DEV0_WRITE_CNT_HI);
        break;
    case STAT_DRAM_DEV0_REF_CNT_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev0_ref_cnt_hi.reg32, value & STAT_DRAM_DEV0_REF_CNT_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev0_ref_cnt_hi.reg32, value, STAT_DRAM_DEV0_REF_CNT_HI);
        break;
    case STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ1_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev0_cumm_banks_active_cke_eq1_hi.reg32, value & STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ1_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev0_cumm_banks_active_cke_eq1_hi.reg32, value, STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ1_HI);
        break;
    case STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ0_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev0_cumm_banks_active_cke_eq0_hi.reg32, value & STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ0_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev0_cumm_banks_active_cke_eq0_hi.reg32, value, STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ0_HI);
        break;
    case STAT_DRAM_DEV0_CKE_EQ1_CLKS_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev0_cke_eq1_clks_hi.reg32, value & STAT_DRAM_DEV0_CKE_EQ1_CLKS_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev0_cke_eq1_clks_hi.reg32, value, STAT_DRAM_DEV0_CKE_EQ1_CLKS_HI);
        break;
    case STAT_DRAM_DEV0_EXTCLKS_CKE_EQ1_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev0_extclks_cke_eq1_hi.reg32, value & STAT_DRAM_DEV0_EXTCLKS_CKE_EQ1_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev0_extclks_cke_eq1_hi.reg32, value, STAT_DRAM_DEV0_EXTCLKS_CKE_EQ1_HI);
        break;
    case STAT_DRAM_DEV0_EXTCLKS_CKE_EQ0_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev0_extclks_cke_eq0_hi.reg32, value & STAT_DRAM_DEV0_EXTCLKS_CKE_EQ0_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev0_extclks_cke_eq0_hi.reg32, value, STAT_DRAM_DEV0_EXTCLKS_CKE_EQ0_HI);
        break;
    case STAT_DRAM_DEV1_ACTIVATE_CNT_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev1_activate_cnt_hi.reg32, value & STAT_DRAM_DEV1_ACTIVATE_CNT_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev1_activate_cnt_hi.reg32, value, STAT_DRAM_DEV1_ACTIVATE_CNT_HI);
        break;
    case STAT_DRAM_DEV1_READ_CNT_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev1_read_cnt_hi.reg32, value & STAT_DRAM_DEV1_READ_CNT_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev1_read_cnt_hi.reg32, value, STAT_DRAM_DEV1_READ_CNT_HI);
        break;
    case STAT_DRAM_DEV1_WRITE_CNT_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev1_write_cnt_hi.reg32, value & STAT_DRAM_DEV1_WRITE_CNT_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev1_write_cnt_hi.reg32, value, STAT_DRAM_DEV1_WRITE_CNT_HI);
        break;
    case STAT_DRAM_DEV1_REF_CNT_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev1_ref_cnt_hi.reg32, value & STAT_DRAM_DEV1_REF_CNT_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev1_ref_cnt_hi.reg32, value, STAT_DRAM_DEV1_REF_CNT_HI);
        break;
    case STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ1_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev1_cumm_banks_active_cke_eq1_hi.reg32, value & STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ1_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev1_cumm_banks_active_cke_eq1_hi.reg32, value, STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ1_HI);
        break;
    case STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ0_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev1_cumm_banks_active_cke_eq0_hi.reg32, value & STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ0_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev1_cumm_banks_active_cke_eq0_hi.reg32, value, STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ0_HI);
        break;
    case STAT_DRAM_DEV1_CKE_EQ1_CLKS_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev1_cke_eq1_clks_hi.reg32, value & STAT_DRAM_DEV1_CKE_EQ1_CLKS_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev1_cke_eq1_clks_hi.reg32, value, STAT_DRAM_DEV1_CKE_EQ1_CLKS_HI);
        break;
    case STAT_DRAM_DEV1_EXTCLKS_CKE_EQ1_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev1_extclks_cke_eq1_hi.reg32, value & STAT_DRAM_DEV1_EXTCLKS_CKE_EQ1_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev1_extclks_cke_eq1_hi.reg32, value, STAT_DRAM_DEV1_EXTCLKS_CKE_EQ1_HI);
        break;
    case STAT_DRAM_DEV1_EXTCLKS_CKE_EQ0_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev1_extclks_cke_eq0_hi.reg32, value & STAT_DRAM_DEV1_EXTCLKS_CKE_EQ0_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev1_extclks_cke_eq0_hi.reg32, value, STAT_DRAM_DEV1_EXTCLKS_CKE_EQ0_HI);
        break;
    case STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ1_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev0_no_banks_active_cke_eq1_hi.reg32, value & STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ1_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev0_no_banks_active_cke_eq1_hi.reg32, value, STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ1_HI);
        break;
    case STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ0_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev0_no_banks_active_cke_eq0_hi.reg32, value & STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ0_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev0_no_banks_active_cke_eq0_hi.reg32, value, STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ0_HI);
        break;
    case STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ1_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev1_no_banks_active_cke_eq1_hi.reg32, value & STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ1_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev1_no_banks_active_cke_eq1_hi.reg32, value, STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ1_HI);
        break;
    case STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ0_HI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->stat_dram_dev1_no_banks_active_cke_eq0_hi.reg32, value & STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ0_HI_WRMASK);
        WR_MASKED(s->stat_dram_dev1_no_banks_active_cke_eq0_hi.reg32, value, STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ0_HI);
        break;
    case CFG_CLKTRIM_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->cfg_clktrim.reg32, value);
        s->cfg_clktrim.reg32 = value;
        break;
    case CFG_CLKTRIM_1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->cfg_clktrim_1.reg32, value);
        s->cfg_clktrim_1.reg32 = value;
        break;
    case CFG_CLKTRIM_2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->cfg_clktrim_2.reg32, value);
        s->cfg_clktrim_2.reg32 = value;
        break;
    default:
        if (offset+size <= sizeof(s->regs)) {
            TRACE_WRITE(s->iomem.addr, offset, s->regs[offset>>2], value);
            s->regs[offset>>2] = value;
        }
        else
            TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }
}

static void tegra_emc_priv_reset(DeviceState *dev)
{
    tegra_emc *s = TEGRA_EMC(dev);

    s->intstatus.reg32 = INTSTATUS_RESET;
    s->intmask.reg32 = INTMASK_RESET;
    s->dbg.reg32 = DBG_RESET;
    s->cfg.reg32 = CFG_RESET;
    s->adr_cfg.reg32 = ADR_CFG_RESET;
    s->adr_cfg_1.reg32 = ADR_CFG_1_RESET;
    s->refctrl.reg32 = REFCTRL_RESET;
    s->pin.reg32 = PIN_RESET;
    s->timing_control.reg32 = TIMING_CONTROL_RESET;
    s->rc.reg32 = RC_RESET;
    s->rfc.reg32 = RFC_RESET;
    s->ras.reg32 = RAS_RESET;
    s->rp.reg32 = RP_RESET;
    s->r2w.reg32 = R2W_RESET;
    s->w2r.reg32 = W2R_RESET;
    s->r2p.reg32 = R2P_RESET;
    s->w2p.reg32 = W2P_RESET;
    s->rd_rcd.reg32 = RD_RCD_RESET;
    s->wr_rcd.reg32 = WR_RCD_RESET;
    s->rrd.reg32 = RRD_RESET;
    s->rext.reg32 = REXT_RESET;
    s->wdv.reg32 = WDV_RESET;
    s->quse.reg32 = QUSE_RESET;
    s->qrst.reg32 = QRST_RESET;
    s->qsafe.reg32 = QSAFE_RESET;
    s->rdv.reg32 = RDV_RESET;
    s->refresh.reg32 = REFRESH_RESET;
    s->burst_refresh_num.reg32 = BURST_REFRESH_NUM_RESET;
    s->pdex2wr.reg32 = PDEX2WR_RESET;
    s->pdex2rd.reg32 = PDEX2RD_RESET;
    s->pchg2pden.reg32 = PCHG2PDEN_RESET;
    s->act2pden.reg32 = ACT2PDEN_RESET;
    s->ar2pden.reg32 = AR2PDEN_RESET;
    s->rw2pden.reg32 = RW2PDEN_RESET;
    s->txsr.reg32 = TXSR_RESET;
    s->tcke.reg32 = TCKE_RESET;
    s->tfaw.reg32 = TFAW_RESET;
    s->trpab.reg32 = TRPAB_RESET;
    s->tclkstable.reg32 = TCLKSTABLE_RESET;
    s->tclkstop.reg32 = TCLKSTOP_RESET;
    s->trefbw.reg32 = TREFBW_RESET;
    s->quse_extra.reg32 = QUSE_EXTRA_RESET;
    s->odt_write.reg32 = ODT_WRITE_RESET;
    s->odt_read.reg32 = ODT_READ_RESET;
    s->mrs.reg32 = MRS_RESET;
    s->emrs.reg32 = EMRS_RESET;
    s->ref.reg32 = REF_RESET;
    s->pre.reg32 = PRE_RESET;
    s->nop.reg32 = NOP_RESET;
    s->self_ref.reg32 = SELF_REF_RESET;
    s->dpd.reg32 = DPD_RESET;
    s->mrw.reg32 = MRW_RESET;
    s->mrr.reg32 = MRR_RESET;
    s->fbio_cfg1.reg32 = FBIO_CFG1_RESET;
    s->fbio_dqsib_dly.reg32 = FBIO_DQSIB_DLY_RESET;
    s->fbio_dqsib_dly_msb.reg32 = FBIO_DQSIB_DLY_MSB_RESET;
    s->fbio_cfg5.reg32 = FBIO_CFG5_RESET;
    s->fbio_quse_dly.reg32 = FBIO_QUSE_DLY_RESET;
    s->fbio_quse_dly_msb.reg32 = FBIO_QUSE_DLY_MSB_RESET;
    s->fbio_cfg6.reg32 = FBIO_CFG6_RESET;
    s->dqs_trimmer_rd0.reg32 = DQS_TRIMMER_RD0_RESET;
    s->dqs_trimmer_rd1.reg32 = DQS_TRIMMER_RD1_RESET;
    s->dqs_trimmer_rd2.reg32 = DQS_TRIMMER_RD2_RESET;
    s->dqs_trimmer_rd3.reg32 = DQS_TRIMMER_RD3_RESET;
    s->ll_arb_config.reg32 = LL_ARB_CONFIG_RESET;
    s->t_min_critical_hp.reg32 = T_MIN_CRITICAL_HP_RESET;
    s->t_min_critical_timeout.reg32 = T_MIN_CRITICAL_TIMEOUT_RESET;
    s->t_min_load.reg32 = T_MIN_LOAD_RESET;
    s->t_max_critical_hp.reg32 = T_MAX_CRITICAL_HP_RESET;
    s->t_max_critical_timeout.reg32 = T_MAX_CRITICAL_TIMEOUT_RESET;
    s->t_max_load.reg32 = T_MAX_LOAD_RESET;
    s->auto_cal_config.reg32 = AUTO_CAL_CONFIG_RESET;
    s->auto_cal_interval.reg32 = AUTO_CAL_INTERVAL_RESET;
    s->auto_cal_status.reg32 = AUTO_CAL_STATUS_RESET;
    s->req_ctrl.reg32 = REQ_CTRL_RESET;
    s->emc_status.reg32 = EMC_STATUS_RESET;
    s->cfg_2.reg32 = CFG_2_RESET;
    s->cfg_dig_dll.reg32 = CFG_DIG_DLL_RESET;
    s->dll_xform_dqs.reg32 = DLL_XFORM_DQS_RESET;
    s->dll_xform_quse.reg32 = DLL_XFORM_QUSE_RESET;
    s->dig_dll_upper_status.reg32 = DIG_DLL_UPPER_STATUS_RESET;
    s->dig_dll_lower_status.reg32 = DIG_DLL_LOWER_STATUS_RESET;
    s->ctt_term_ctrl.reg32 = CTT_TERM_CTRL_RESET;
    s->zcal_ref_cnt.reg32 = ZCAL_REF_CNT_RESET;
    s->zcal_wait_cnt.reg32 = ZCAL_WAIT_CNT_RESET;
    s->zcal_mrw_cmd.reg32 = ZCAL_MRW_CMD_RESET;
    s->cmdq.reg32 = CMDQ_RESET;
    s->fbio_spare.reg32 = FBIO_SPARE_RESET;
    s->fbio_wrptr_eq_2.reg32 = FBIO_WRPTR_EQ_2_RESET;
    s->clken_override.reg32 = CLKEN_OVERRIDE_RESET;
    s->stat_control.reg32 = STAT_CONTROL_RESET;
    s->stat_status.reg32 = STAT_STATUS_RESET;
    s->stat_llmc_addr_low.reg32 = STAT_LLMC_ADDR_LOW_RESET;
    s->stat_llmc_addr_high.reg32 = STAT_LLMC_ADDR_HIGH_RESET;
    s->stat_llmc_clock_limit.reg32 = STAT_LLMC_CLOCK_LIMIT_RESET;
    s->stat_llmc_clocks.reg32 = STAT_LLMC_CLOCKS_RESET;
    s->stat_llmc_control.reg32 = STAT_LLMC_CONTROL_RESET;
    s->stat_llmc_hist_limit.reg32 = STAT_LLMC_HIST_LIMIT_RESET;
    s->stat_llmc_count.reg32 = STAT_LLMC_COUNT_RESET;
    s->stat_llmc_hist.reg32 = STAT_LLMC_HIST_RESET;
    s->stat_pwr_clock_limit.reg32 = STAT_PWR_CLOCK_LIMIT_RESET;
    s->stat_pwr_clocks.reg32 = STAT_PWR_CLOCKS_RESET;
    s->stat_pwr_count.reg32 = STAT_PWR_COUNT_RESET;
    s->stat_dram_clock_limit_lo.reg32 = STAT_DRAM_CLOCK_LIMIT_LO_RESET;
    s->stat_dram_clock_limit_hi.reg32 = STAT_DRAM_CLOCK_LIMIT_HI_RESET;
    s->stat_dram_clocks_lo.reg32 = STAT_DRAM_CLOCKS_LO_RESET;
    s->stat_dram_clocks_hi.reg32 = STAT_DRAM_CLOCKS_HI_RESET;
    s->stat_dram_dev0_activate_cnt_lo.reg32 = STAT_DRAM_DEV0_ACTIVATE_CNT_LO_RESET;
    s->stat_dram_dev0_activate_cnt_hi.reg32 = STAT_DRAM_DEV0_ACTIVATE_CNT_HI_RESET;
    s->stat_dram_dev0_read_cnt_lo.reg32 = STAT_DRAM_DEV0_READ_CNT_LO_RESET;
    s->stat_dram_dev0_read_cnt_hi.reg32 = STAT_DRAM_DEV0_READ_CNT_HI_RESET;
    s->stat_dram_dev0_write_cnt_lo.reg32 = STAT_DRAM_DEV0_WRITE_CNT_LO_RESET;
    s->stat_dram_dev0_write_cnt_hi.reg32 = STAT_DRAM_DEV0_WRITE_CNT_HI_RESET;
    s->stat_dram_dev0_ref_cnt_lo.reg32 = STAT_DRAM_DEV0_REF_CNT_LO_RESET;
    s->stat_dram_dev0_ref_cnt_hi.reg32 = STAT_DRAM_DEV0_REF_CNT_HI_RESET;
    s->stat_dram_dev0_cumm_banks_active_cke_eq1_lo.reg32 = STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ1_LO_RESET;
    s->stat_dram_dev0_cumm_banks_active_cke_eq1_hi.reg32 = STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ1_HI_RESET;
    s->stat_dram_dev0_cumm_banks_active_cke_eq0_lo.reg32 = STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ0_LO_RESET;
    s->stat_dram_dev0_cumm_banks_active_cke_eq0_hi.reg32 = STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ0_HI_RESET;
    s->stat_dram_dev0_cke_eq1_clks_lo.reg32 = STAT_DRAM_DEV0_CKE_EQ1_CLKS_LO_RESET;
    s->stat_dram_dev0_cke_eq1_clks_hi.reg32 = STAT_DRAM_DEV0_CKE_EQ1_CLKS_HI_RESET;
    s->stat_dram_dev0_extclks_cke_eq1_lo.reg32 = STAT_DRAM_DEV0_EXTCLKS_CKE_EQ1_LO_RESET;
    s->stat_dram_dev0_extclks_cke_eq1_hi.reg32 = STAT_DRAM_DEV0_EXTCLKS_CKE_EQ1_HI_RESET;
    s->stat_dram_dev0_extclks_cke_eq0_lo.reg32 = STAT_DRAM_DEV0_EXTCLKS_CKE_EQ0_LO_RESET;
    s->stat_dram_dev0_extclks_cke_eq0_hi.reg32 = STAT_DRAM_DEV0_EXTCLKS_CKE_EQ0_HI_RESET;
    s->stat_dram_dev1_activate_cnt_lo.reg32 = STAT_DRAM_DEV1_ACTIVATE_CNT_LO_RESET;
    s->stat_dram_dev1_activate_cnt_hi.reg32 = STAT_DRAM_DEV1_ACTIVATE_CNT_HI_RESET;
    s->stat_dram_dev1_read_cnt_lo.reg32 = STAT_DRAM_DEV1_READ_CNT_LO_RESET;
    s->stat_dram_dev1_read_cnt_hi.reg32 = STAT_DRAM_DEV1_READ_CNT_HI_RESET;
    s->stat_dram_dev1_write_cnt_lo.reg32 = STAT_DRAM_DEV1_WRITE_CNT_LO_RESET;
    s->stat_dram_dev1_write_cnt_hi.reg32 = STAT_DRAM_DEV1_WRITE_CNT_HI_RESET;
    s->stat_dram_dev1_ref_cnt_lo.reg32 = STAT_DRAM_DEV1_REF_CNT_LO_RESET;
    s->stat_dram_dev1_ref_cnt_hi.reg32 = STAT_DRAM_DEV1_REF_CNT_HI_RESET;
    s->stat_dram_dev1_cumm_banks_active_cke_eq1_lo.reg32 = STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ1_LO_RESET;
    s->stat_dram_dev1_cumm_banks_active_cke_eq1_hi.reg32 = STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ1_HI_RESET;
    s->stat_dram_dev1_cumm_banks_active_cke_eq0_lo.reg32 = STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ0_LO_RESET;
    s->stat_dram_dev1_cumm_banks_active_cke_eq0_hi.reg32 = STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ0_HI_RESET;
    s->stat_dram_dev1_cke_eq1_clks_lo.reg32 = STAT_DRAM_DEV1_CKE_EQ1_CLKS_LO_RESET;
    s->stat_dram_dev1_cke_eq1_clks_hi.reg32 = STAT_DRAM_DEV1_CKE_EQ1_CLKS_HI_RESET;
    s->stat_dram_dev1_extclks_cke_eq1_lo.reg32 = STAT_DRAM_DEV1_EXTCLKS_CKE_EQ1_LO_RESET;
    s->stat_dram_dev1_extclks_cke_eq1_hi.reg32 = STAT_DRAM_DEV1_EXTCLKS_CKE_EQ1_HI_RESET;
    s->stat_dram_dev1_extclks_cke_eq0_lo.reg32 = STAT_DRAM_DEV1_EXTCLKS_CKE_EQ0_LO_RESET;
    s->stat_dram_dev1_extclks_cke_eq0_hi.reg32 = STAT_DRAM_DEV1_EXTCLKS_CKE_EQ0_HI_RESET;
    s->stat_dram_dev0_no_banks_active_cke_eq1_lo.reg32 = STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ1_LO_RESET;
    s->stat_dram_dev0_no_banks_active_cke_eq1_hi.reg32 = STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ1_HI_RESET;
    s->stat_dram_dev0_no_banks_active_cke_eq0_lo.reg32 = STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ0_LO_RESET;
    s->stat_dram_dev0_no_banks_active_cke_eq0_hi.reg32 = STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ0_HI_RESET;
    s->stat_dram_dev1_no_banks_active_cke_eq1_lo.reg32 = STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ1_LO_RESET;
    s->stat_dram_dev1_no_banks_active_cke_eq1_hi.reg32 = STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ1_HI_RESET;
    s->stat_dram_dev1_no_banks_active_cke_eq0_lo.reg32 = STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ0_LO_RESET;
    s->stat_dram_dev1_no_banks_active_cke_eq0_hi.reg32 = STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ0_HI_RESET;
    s->cfg_clktrim.reg32 = CFG_CLKTRIM_RESET;
    s->cfg_clktrim_1.reg32 = CFG_CLKTRIM_1_RESET;
    s->cfg_clktrim_2.reg32 = CFG_CLKTRIM_2_RESET;

    s->emc_status.no_outstanding_transactions = 1;

    memset(s->regs, 0, sizeof(s->regs));
}

static const MemoryRegionOps tegra_emc_mem_ops = {
    .read = tegra_emc_priv_read,
    .write = tegra_emc_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_emc_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_emc *s = TEGRA_EMC(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_emc_mem_ops, s,
                          "tegra.emc", TEGRA_EMC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_emc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_emc_priv_realize;
    dc->vmsd = &vmstate_tegra_emc;
    dc->reset = tegra_emc_priv_reset;
}

static const TypeInfo tegra_emc_info = {
    .name = TYPE_TEGRA_EMC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_emc),
    .class_init = tegra_emc_class_init,
};

static void tegra_emc_register_types(void)
{
    type_register_static(&tegra_emc_info);
}

type_init(tegra_emc_register_types)
