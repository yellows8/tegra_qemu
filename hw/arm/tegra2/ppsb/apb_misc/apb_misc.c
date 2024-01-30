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

#include "apb_misc.h"
#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_APB_MISC "tegra.apb_misc"
#define TEGRA_APB_MISC(obj) OBJECT_CHECK(tegra_apb_misc, (obj), TYPE_TEGRA_APB_MISC)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_apb_misc_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    DEFINE_REG32(pp_strapping_opt_a);
    DEFINE_REG32(pp_tristate_reg_a);
    DEFINE_REG32(pp_tristate_reg_b);
    DEFINE_REG32(pp_tristate_reg_c);
    DEFINE_REG32(pp_tristate_reg_d);
    DEFINE_REG32(pp_config_ctl);
    DEFINE_REG32(pp_misc_usb_otg);
    DEFINE_REG32(pp_pinmux_global);
    DEFINE_REG32(pp_usb_phy_param);
    DEFINE_REG32(pp_usb_phy_vbus_sensors);
    DEFINE_REG32(pp_usb_phy_vbus_wakeup_id);
    DEFINE_REG32(pp_usb_phy_alt_vbus_sts);
    DEFINE_REG32(pp_pin_mux_ctl_a);
    DEFINE_REG32(pp_pin_mux_ctl_b);
    DEFINE_REG32(pp_pin_mux_ctl_c);
    DEFINE_REG32(pp_pin_mux_ctl_d);
    DEFINE_REG32(pp_pin_mux_ctl_e);
    DEFINE_REG32(pp_pin_mux_ctl_f);
    DEFINE_REG32(pp_pin_mux_ctl_g);
    DEFINE_REG32(pp_pin_mux_ctl_h);
    DEFINE_REG32(pp_pullupdown_reg_a);
    DEFINE_REG32(pp_pullupdown_reg_b);
    DEFINE_REG32(pp_pullupdown_reg_c);
    DEFINE_REG32(pp_pullupdown_reg_d);
    DEFINE_REG32(pp_pullupdown_reg_e);
    DEFINE_REG32(pp_misc_usb_clk_rst_ctl);
    DEFINE_REG32(gp_modereg);
    DEFINE_REG32(gp_hidrev);
    DEFINE_REG32(gp_asdbgreg);
    DEFINE_REG32(gp_emu_revid);
    DEFINE_REG32(gp_transactor_scratch);
    DEFINE_REG32(gp_aocfg1padctrl);
    DEFINE_REG32(gp_aocfg2padctrl);
    DEFINE_REG32(gp_atcfg1padctrl);
    DEFINE_REG32(gp_atcfg2padctrl);
    DEFINE_REG32(gp_cdev1cfgpadctrl);
    DEFINE_REG32(gp_cdev2cfgpadctrl);
    DEFINE_REG32(gp_csuscfgpadctrl);
    DEFINE_REG32(gp_dap1cfgpadctrl);
    DEFINE_REG32(gp_dap2cfgpadctrl);
    DEFINE_REG32(gp_dap3cfgpadctrl);
    DEFINE_REG32(gp_dap4cfgpadctrl);
    DEFINE_REG32(gp_dbgcfgpadctrl);
    DEFINE_REG32(gp_lcdcfg1padctrl);
    DEFINE_REG32(gp_lcdcfg2padctrl);
    DEFINE_REG32(gp_sdio2cfgpadctrl);
    DEFINE_REG32(gp_sdio3cfgpadctrl);
    DEFINE_REG32(gp_spicfgpadctrl);
    DEFINE_REG32(gp_uaacfgpadctrl);
    DEFINE_REG32(gp_uabcfgpadctrl);
    DEFINE_REG32(gp_uart2cfgpadctrl);
    DEFINE_REG32(gp_uart3cfgpadctrl);
    DEFINE_REG32(gp_vicfg1padctrl);
    DEFINE_REG32(gp_vicfg2padctrl);
    DEFINE_REG32(gp_xm2cfgapadctrl);
    DEFINE_REG32(gp_xm2cfgcpadctrl);
    DEFINE_REG32(gp_xm2cfgdpadctrl);
    DEFINE_REG32(gp_xm2clkcfgpadctrl);
    DEFINE_REG32(gp_xm2comppadctrl);
    DEFINE_REG32(gp_xm2vttgenpadctrl);
    DEFINE_REG32(gp_padctl_dft);
    DEFINE_REG32(gp_sdio1cfgpadctrl);
    DEFINE_REG32(gp_xm2cfgcpadctrl2);
    DEFINE_REG32(gp_xm2cfgdpadctrl2);
    DEFINE_REG32(gp_crtcfgpadctrl);
    DEFINE_REG32(gp_ddccfgpadctrl);
    DEFINE_REG32(gp_gmacfgpadctrl);
    DEFINE_REG32(gp_gmbcfgpadctrl);
    DEFINE_REG32(gp_gmccfgpadctrl);
    DEFINE_REG32(gp_gmdcfgpadctrl);
    DEFINE_REG32(gp_gmecfgpadctrl);
    DEFINE_REG32(gp_owrcfgpadctrl);
    DEFINE_REG32(gp_uadcfgpadctrl);
    DEFINE_REG32(das_dap_ctrl_sel);
    DEFINE_REG32(das_dac_input_data_clk_sel);
    DEFINE_REG32(pp_misc_save_the_day);
    DEFINE_REG32(async_corepwrconfig);
    DEFINE_REG32(async_emcpaden);
    DEFINE_REG32(sc1x_pads_vip_vclkctrl);
    DEFINE_REG32(async_vclkctrl);
    DEFINE_REG32(async_tvdacvhsyncctrl);
    DEFINE_REG32(async_tvdaccntl);
    DEFINE_REG32(async_tvdacstatus);
    DEFINE_REG32(async_tvdacdinconfig);
    DEFINE_REG32(async_int_status);
    DEFINE_REG32(async_int_mask);
    DEFINE_REG32(async_int_polarity);
    DEFINE_REG32(async_int_type_select);
    DEFINE_REG32(das_dap_ctrl_sel_1);
    DEFINE_REG32(das_dap_ctrl_sel_2);
    DEFINE_REG32(das_dap_ctrl_sel_3);
    DEFINE_REG32(das_dap_ctrl_sel_4);
    DEFINE_REG32(das_dac_input_data_clk_sel_1);
    DEFINE_REG32(das_dac_input_data_clk_sel_2);
    uint32_t regs[(0xB7C-GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2];
} tegra_apb_misc;

static const VMStateDescription vmstate_tegra_apb_misc = {
    .name = "tegra.apb_misc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(pp_strapping_opt_a.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_tristate_reg_a.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_tristate_reg_b.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_tristate_reg_c.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_tristate_reg_d.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_config_ctl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_misc_usb_otg.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_pinmux_global.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_usb_phy_param.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_usb_phy_vbus_sensors.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_usb_phy_vbus_wakeup_id.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_usb_phy_alt_vbus_sts.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_pin_mux_ctl_a.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_pin_mux_ctl_b.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_pin_mux_ctl_c.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_pin_mux_ctl_d.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_pin_mux_ctl_e.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_pin_mux_ctl_f.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_pin_mux_ctl_g.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_pin_mux_ctl_h.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_pullupdown_reg_a.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_pullupdown_reg_b.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_pullupdown_reg_c.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_pullupdown_reg_d.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_pullupdown_reg_e.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_misc_usb_clk_rst_ctl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_modereg.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_hidrev.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_asdbgreg.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_emu_revid.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_transactor_scratch.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_aocfg1padctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_aocfg2padctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_atcfg1padctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_atcfg2padctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_cdev1cfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_cdev2cfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_csuscfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_dap1cfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_dap2cfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_dap3cfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_dap4cfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_dbgcfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_lcdcfg1padctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_lcdcfg2padctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_sdio2cfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_sdio3cfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_spicfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_uaacfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_uabcfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_uart2cfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_uart3cfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_vicfg1padctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_vicfg2padctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_xm2cfgapadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_xm2cfgcpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_xm2cfgdpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_xm2clkcfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_xm2comppadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_xm2vttgenpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_padctl_dft.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_sdio1cfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_xm2cfgcpadctrl2.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_xm2cfgdpadctrl2.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_crtcfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_ddccfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_gmacfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_gmbcfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_gmccfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_gmdcfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_gmecfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_owrcfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(gp_uadcfgpadctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(das_dap_ctrl_sel.reg32, tegra_apb_misc),
        VMSTATE_UINT32(das_dac_input_data_clk_sel.reg32, tegra_apb_misc),
        VMSTATE_UINT32(pp_misc_save_the_day.reg32, tegra_apb_misc),
        VMSTATE_UINT32(async_corepwrconfig.reg32, tegra_apb_misc),
        VMSTATE_UINT32(async_emcpaden.reg32, tegra_apb_misc),
        VMSTATE_UINT32(sc1x_pads_vip_vclkctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(async_vclkctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(async_tvdacvhsyncctrl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(async_tvdaccntl.reg32, tegra_apb_misc),
        VMSTATE_UINT32(async_tvdacstatus.reg32, tegra_apb_misc),
        VMSTATE_UINT32(async_tvdacdinconfig.reg32, tegra_apb_misc),
        VMSTATE_UINT32(async_int_status.reg32, tegra_apb_misc),
        VMSTATE_UINT32(async_int_mask.reg32, tegra_apb_misc),
        VMSTATE_UINT32(async_int_polarity.reg32, tegra_apb_misc),
        VMSTATE_UINT32(async_int_type_select.reg32, tegra_apb_misc),
        VMSTATE_UINT32(das_dap_ctrl_sel_1.reg32, tegra_apb_misc),
        VMSTATE_UINT32(das_dap_ctrl_sel_2.reg32, tegra_apb_misc),
        VMSTATE_UINT32(das_dap_ctrl_sel_3.reg32, tegra_apb_misc),
        VMSTATE_UINT32(das_dap_ctrl_sel_4.reg32, tegra_apb_misc),
        VMSTATE_UINT32(das_dac_input_data_clk_sel_1.reg32, tegra_apb_misc),
        VMSTATE_UINT32(das_dac_input_data_clk_sel_2.reg32, tegra_apb_misc),
        VMSTATE_UINT32_ARRAY(regs, tegra_apb_misc, (0xB7C-GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_apb_misc_priv_read(void *opaque, hwaddr offset,
                                         unsigned size)
{
    tegra_apb_misc *s = opaque;
    uint64_t ret = 0;

    switch (offset) {
    case PP_STRAPPING_OPT_A_OFFSET:
        ret = s->pp_strapping_opt_a.reg32;
        break;
    case PP_TRISTATE_REG_A_OFFSET:
        ret = s->pp_tristate_reg_a.reg32;
        break;
    case PP_TRISTATE_REG_B_OFFSET:
        ret = s->pp_tristate_reg_b.reg32;
        break;
    case PP_TRISTATE_REG_C_OFFSET:
        ret = s->pp_tristate_reg_c.reg32;
        break;
    case PP_TRISTATE_REG_D_OFFSET:
        ret = s->pp_tristate_reg_d.reg32;
        break;
    case PP_CONFIG_CTL_OFFSET:
        ret = s->pp_config_ctl.reg32;
        break;
    case PP_MISC_USB_OTG_OFFSET:
        ret = s->pp_misc_usb_otg.reg32;
        break;
    case PP_PINMUX_GLOBAL_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->pp_pinmux_global.reg32;
        break;
    case PP_USB_PHY_PARAM_OFFSET:
        ret = s->pp_usb_phy_param.reg32;
        break;
    case PP_USB_PHY_VBUS_SENSORS_OFFSET:
        ret = s->pp_usb_phy_vbus_sensors.reg32;
        break;
    case PP_USB_PHY_VBUS_WAKEUP_ID_OFFSET:
        ret = s->pp_usb_phy_vbus_wakeup_id.reg32;
        break;
    case PP_USB_PHY_ALT_VBUS_STS_OFFSET:
        ret = s->pp_usb_phy_alt_vbus_sts.reg32;
        break;
    case PP_PIN_MUX_CTL_A_OFFSET:
        ret = s->pp_pin_mux_ctl_a.reg32;
        break;
    case PP_PIN_MUX_CTL_B_OFFSET:
        ret = s->pp_pin_mux_ctl_b.reg32;
        break;
    case PP_PIN_MUX_CTL_C_OFFSET:
        ret = s->pp_pin_mux_ctl_c.reg32;
        break;
    case PP_PIN_MUX_CTL_D_OFFSET:
        ret = s->pp_pin_mux_ctl_d.reg32;
        break;
    case PP_PIN_MUX_CTL_E_OFFSET:
        ret = s->pp_pin_mux_ctl_e.reg32;
        break;
    case PP_PIN_MUX_CTL_F_OFFSET:
        ret = s->pp_pin_mux_ctl_f.reg32;
        break;
    case PP_PIN_MUX_CTL_G_OFFSET:
        ret = s->pp_pin_mux_ctl_g.reg32;
        break;
    case PP_PIN_MUX_CTL_H_OFFSET:
        ret = s->pp_pin_mux_ctl_h.reg32;
        break;
    case PP_PULLUPDOWN_REG_A_OFFSET:
        ret = s->pp_pullupdown_reg_a.reg32;
        break;
    case PP_PULLUPDOWN_REG_B_OFFSET:
        ret = s->pp_pullupdown_reg_b.reg32;
        break;
    case PP_PULLUPDOWN_REG_C_OFFSET:
        ret = s->pp_pullupdown_reg_c.reg32;
        break;
    case PP_PULLUPDOWN_REG_D_OFFSET:
        ret = s->pp_pullupdown_reg_d.reg32;
        break;
    case PP_PULLUPDOWN_REG_E_OFFSET:
        ret = s->pp_pullupdown_reg_e.reg32;
        break;
    case PP_MISC_USB_CLK_RST_CTL_OFFSET:
        ret = s->pp_misc_usb_clk_rst_ctl.reg32;
        break;
    case GP_MODEREG_OFFSET:
        ret = s->gp_modereg.reg32;
        break;
    case GP_HIDREV_OFFSET:
        ret = s->gp_hidrev.reg32;
        break;
    case GP_ASDBGREG_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->gp_asdbgreg.reg32;
        break;
    case GP_EMU_REVID_OFFSET:
        ret = s->gp_emu_revid.reg32;
        break;
    case GP_TRANSACTOR_SCRATCH_OFFSET:
        ret = s->gp_transactor_scratch.reg32;
        break;
    case GP_AOCFG1PADCTRL_OFFSET:
        ret = s->gp_aocfg1padctrl.reg32;
        break;
    case GP_AOCFG2PADCTRL_OFFSET:
        ret = s->gp_aocfg2padctrl.reg32;
        break;
    case GP_ATCFG1PADCTRL_OFFSET:
        ret = s->gp_atcfg1padctrl.reg32;
        break;
    case GP_ATCFG2PADCTRL_OFFSET:
        ret = s->gp_atcfg2padctrl.reg32;
        break;
    case GP_CDEV1CFGPADCTRL_OFFSET:
        ret = s->gp_cdev1cfgpadctrl.reg32;
        break;
    case GP_CDEV2CFGPADCTRL_OFFSET:
        ret = s->gp_cdev2cfgpadctrl.reg32;
        break;
    case GP_CSUSCFGPADCTRL_OFFSET:
        ret = s->gp_csuscfgpadctrl.reg32;
        break;
    case GP_DAP1CFGPADCTRL_OFFSET:
        ret = s->gp_dap1cfgpadctrl.reg32;
        break;
    case GP_DAP2CFGPADCTRL_OFFSET:
        ret = s->gp_dap2cfgpadctrl.reg32;
        break;
    case GP_DAP3CFGPADCTRL_OFFSET:
        ret = s->gp_dap3cfgpadctrl.reg32;
        break;
    case GP_DAP4CFGPADCTRL_OFFSET:
        ret = s->gp_dap4cfgpadctrl.reg32;
        break;
    case GP_DBGCFGPADCTRL_OFFSET:
        ret = s->gp_dbgcfgpadctrl.reg32;
        break;
    case GP_LCDCFG1PADCTRL_OFFSET:
        ret = s->gp_lcdcfg1padctrl.reg32;
        break;
    case GP_LCDCFG2PADCTRL_OFFSET:
        ret = s->gp_lcdcfg2padctrl.reg32;
        break;
    case GP_SDIO2CFGPADCTRL_OFFSET:
        ret = s->gp_sdio2cfgpadctrl.reg32;
        break;
    case GP_SDIO3CFGPADCTRL_OFFSET:
        ret = s->gp_sdio3cfgpadctrl.reg32;
        break;
    case GP_SPICFGPADCTRL_OFFSET:
        ret = s->gp_spicfgpadctrl.reg32;
        break;
    case GP_UAACFGPADCTRL_OFFSET:
        ret = s->gp_uaacfgpadctrl.reg32;
        break;
    case GP_UABCFGPADCTRL_OFFSET:
        ret = s->gp_uabcfgpadctrl.reg32;
        break;
    case GP_UART2CFGPADCTRL_OFFSET:
        ret = s->gp_uart2cfgpadctrl.reg32;
        break;
    case GP_UART3CFGPADCTRL_OFFSET:
        ret = s->gp_uart3cfgpadctrl.reg32;
        break;
    case GP_VICFG1PADCTRL_OFFSET:
        ret = s->gp_vicfg1padctrl.reg32;
        break;
    case GP_VICFG2PADCTRL_OFFSET:
        ret = s->gp_vicfg2padctrl.reg32;
        break;
    case GP_XM2CFGAPADCTRL_OFFSET:
        ret = s->gp_xm2cfgapadctrl.reg32;
        break;
    case GP_XM2CFGCPADCTRL_OFFSET:
        ret = s->gp_xm2cfgcpadctrl.reg32;
        break;
    case GP_XM2CFGDPADCTRL_OFFSET:
        ret = s->gp_xm2cfgdpadctrl.reg32;
        break;
    case GP_XM2CLKCFGPADCTRL_OFFSET:
        ret = s->gp_xm2clkcfgpadctrl.reg32;
        break;
    case GP_XM2COMPPADCTRL_OFFSET:
        ret = s->gp_xm2comppadctrl.reg32;
        break;
    case GP_XM2VTTGENPADCTRL_OFFSET:
        ret = s->gp_xm2vttgenpadctrl.reg32;
        break;
    case GP_PADCTL_DFT_OFFSET:
        ret = s->gp_padctl_dft.reg32;
        break;
    case GP_SDIO1CFGPADCTRL_OFFSET:
        ret = s->gp_sdio1cfgpadctrl.reg32;
        break;
    case GP_XM2CFGCPADCTRL2_OFFSET:
        ret = s->gp_xm2cfgcpadctrl2.reg32;
        break;
    case GP_XM2CFGDPADCTRL2_OFFSET:
        ret = s->gp_xm2cfgdpadctrl2.reg32;
        break;
    case GP_CRTCFGPADCTRL_OFFSET:
        ret = s->gp_crtcfgpadctrl.reg32;
        break;
    case GP_DDCCFGPADCTRL_OFFSET:
        ret = s->gp_ddccfgpadctrl.reg32;
        break;
    case GP_GMACFGPADCTRL_OFFSET:
        ret = s->gp_gmacfgpadctrl.reg32;
        break;
    case GP_GMBCFGPADCTRL_OFFSET:
        ret = s->gp_gmbcfgpadctrl.reg32;
        break;
    case GP_GMCCFGPADCTRL_OFFSET:
        ret = s->gp_gmccfgpadctrl.reg32;
        break;
    case GP_GMDCFGPADCTRL_OFFSET:
        ret = s->gp_gmdcfgpadctrl.reg32;
        break;
    case GP_GMECFGPADCTRL_OFFSET:
        ret = s->gp_gmecfgpadctrl.reg32;
        break;
    case GP_OWRCFGPADCTRL_OFFSET:
        ret = s->gp_owrcfgpadctrl.reg32;
        break;
    case GP_UADCFGPADCTRL_OFFSET:
        ret = s->gp_uadcfgpadctrl.reg32;
        break;
    case GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET ... GP_QSPI_SCK_LPBK_CONTROL_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->regs[(offset - GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2];
        break;
    case DAS_DAP_CTRL_SEL_OFFSET:
        ret = s->das_dap_ctrl_sel.reg32;
        break;
    case DAS_DAC_INPUT_DATA_CLK_SEL_OFFSET:
        ret = s->das_dac_input_data_clk_sel.reg32;
        break;
    case PP_MISC_SAVE_THE_DAY_OFFSET:
        ret = s->pp_misc_save_the_day.reg32;
        break;
    case ASYNC_COREPWRCONFIG_OFFSET:
        ret = s->async_corepwrconfig.reg32;
        break;
    case ASYNC_EMCPADEN_OFFSET:
        ret = s->async_emcpaden.reg32;
        break;
    case SC1X_PADS_VIP_VCLKCTRL_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->sc1x_pads_vip_vclkctrl.reg32;
        break;
    case ASYNC_VCLKCTRL_OFFSET:
        ret = s->async_vclkctrl.reg32;
        break;
    case ASYNC_TVDACVHSYNCCTRL_OFFSET:
        ret = s->async_tvdacvhsyncctrl.reg32;
        break;
    case ASYNC_TVDACCNTL_OFFSET:
        ret = s->async_tvdaccntl.reg32;
        break;
    case ASYNC_TVDACSTATUS_OFFSET:
        ret = s->async_tvdacstatus.reg32;
        break;
    case ASYNC_TVDACDINCONFIG_OFFSET:
        ret = s->async_tvdacdinconfig.reg32;
        break;
    case ASYNC_INT_STATUS_OFFSET:
        ret = s->async_int_status.reg32;
        break;
    case ASYNC_INT_MASK_OFFSET:
        ret = s->async_int_mask.reg32;
        break;
    case ASYNC_INT_POLARITY_OFFSET:
        ret = s->async_int_polarity.reg32;
        break;
    case ASYNC_INT_TYPE_SELECT_OFFSET:
        ret = s->async_int_type_select.reg32;
        break;
    case DAS_DAP_CTRL_SEL_1_OFFSET:
        ret = s->das_dap_ctrl_sel_1.reg32;
        break;
    case DAS_DAP_CTRL_SEL_2_OFFSET:
        ret = s->das_dap_ctrl_sel_2.reg32;
        break;
    case DAS_DAP_CTRL_SEL_3_OFFSET:
        ret = s->das_dap_ctrl_sel_3.reg32;
        break;
    case DAS_DAP_CTRL_SEL_4_OFFSET:
        ret = s->das_dap_ctrl_sel_4.reg32;
        break;
    case DAS_DAC_INPUT_DATA_CLK_SEL_1_OFFSET:
        ret = s->das_dac_input_data_clk_sel_1.reg32;
        break;
    case DAS_DAC_INPUT_DATA_CLK_SEL_2_OFFSET:
        ret = s->das_dac_input_data_clk_sel_2.reg32;
        break;
    default:
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_apb_misc_priv_write(void *opaque, hwaddr offset,
                                      uint64_t value, unsigned size)
{
    tegra_apb_misc *s = opaque;

    switch (offset) {
    case PP_STRAPPING_OPT_A_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_strapping_opt_a.reg32, value);
        s->pp_strapping_opt_a.reg32 = value;
        break;
    case PP_TRISTATE_REG_A_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_tristate_reg_a.reg32, value);
        s->pp_tristate_reg_a.reg32 = value;
        break;
    case PP_TRISTATE_REG_B_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_tristate_reg_b.reg32, value);
        s->pp_tristate_reg_b.reg32 = value;
        break;
    case PP_TRISTATE_REG_C_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_tristate_reg_c.reg32, value);
        s->pp_tristate_reg_c.reg32 = value;
        break;
    case PP_TRISTATE_REG_D_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_tristate_reg_d.reg32, value);
        s->pp_tristate_reg_d.reg32 = value;
        break;
    case PP_CONFIG_CTL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_config_ctl.reg32, value);
        s->pp_config_ctl.reg32 = value;
        break;
    case PP_MISC_USB_OTG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_misc_usb_otg.reg32, value & PP_MISC_USB_OTG_WRMASK);
        WR_MASKED(s->pp_misc_usb_otg.reg32, value, PP_MISC_USB_OTG);
        break;
    case PP_PINMUX_GLOBAL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_pinmux_global.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->pp_pinmux_global.reg32 = value;
        break;
    case PP_USB_PHY_PARAM_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_usb_phy_param.reg32, value);
        s->pp_usb_phy_param.reg32 = value;
        break;
    case PP_USB_PHY_VBUS_SENSORS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_usb_phy_vbus_sensors.reg32, value & PP_USB_PHY_VBUS_SENSORS_WRMASK);
        WR_MASKED(s->pp_usb_phy_vbus_sensors.reg32, value, PP_USB_PHY_VBUS_SENSORS);
        break;
    case PP_USB_PHY_VBUS_WAKEUP_ID_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_usb_phy_vbus_wakeup_id.reg32, value & PP_USB_PHY_VBUS_WAKEUP_ID_WRMASK);
        WR_MASKED(s->pp_usb_phy_vbus_wakeup_id.reg32, value, PP_USB_PHY_VBUS_WAKEUP_ID);
        break;
    case PP_PIN_MUX_CTL_A_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_pin_mux_ctl_a.reg32, value);
        s->pp_pin_mux_ctl_a.reg32 = value;
        break;
    case PP_PIN_MUX_CTL_B_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_pin_mux_ctl_b.reg32, value);
        s->pp_pin_mux_ctl_b.reg32 = value;
        break;
    case PP_PIN_MUX_CTL_C_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_pin_mux_ctl_c.reg32, value);
        s->pp_pin_mux_ctl_c.reg32 = value;
        break;
    case PP_PIN_MUX_CTL_D_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_pin_mux_ctl_d.reg32, value);
        s->pp_pin_mux_ctl_d.reg32 = value;
        break;
    case PP_PIN_MUX_CTL_E_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_pin_mux_ctl_e.reg32, value);
        s->pp_pin_mux_ctl_e.reg32 = value;
        break;
    case PP_PIN_MUX_CTL_F_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_pin_mux_ctl_f.reg32, value);
        s->pp_pin_mux_ctl_f.reg32 = value;
        break;
    case PP_PIN_MUX_CTL_G_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_pin_mux_ctl_g.reg32, value);
        s->pp_pin_mux_ctl_g.reg32 = value;
        break;
    case PP_PIN_MUX_CTL_H_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_pin_mux_ctl_h.reg32, value);
        s->pp_pin_mux_ctl_h.reg32 = value;
        break;
    case PP_PULLUPDOWN_REG_A_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_pullupdown_reg_a.reg32, value);
        s->pp_pullupdown_reg_a.reg32 = value;
        break;
    case PP_PULLUPDOWN_REG_B_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_pullupdown_reg_b.reg32, value);
        s->pp_pullupdown_reg_b.reg32 = value;
        break;
    case PP_PULLUPDOWN_REG_C_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_pullupdown_reg_c.reg32, value);
        s->pp_pullupdown_reg_c.reg32 = value;
        break;
    case PP_PULLUPDOWN_REG_D_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_pullupdown_reg_d.reg32, value);
        s->pp_pullupdown_reg_d.reg32 = value;
        break;
    case PP_PULLUPDOWN_REG_E_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_pullupdown_reg_e.reg32, value);
        s->pp_pullupdown_reg_e.reg32 = value;
        break;
    case PP_MISC_USB_CLK_RST_CTL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_misc_usb_clk_rst_ctl.reg32, value);
        s->pp_misc_usb_clk_rst_ctl.reg32 = value;
        break;
    case GP_ASDBGREG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_asdbgreg.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->gp_asdbgreg.reg32 = value;
        break;
    case GP_TRANSACTOR_SCRATCH_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_transactor_scratch.reg32, value);
        s->gp_transactor_scratch.reg32 = value;
        break;
    case GP_AOCFG1PADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_aocfg1padctrl.reg32, value);
        s->gp_aocfg1padctrl.reg32 = value;
        break;
    case GP_AOCFG2PADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_aocfg2padctrl.reg32, value);
        s->gp_aocfg2padctrl.reg32 = value;
        break;
    case GP_ATCFG1PADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_atcfg1padctrl.reg32, value);
        s->gp_atcfg1padctrl.reg32 = value;
        break;
    case GP_ATCFG2PADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_atcfg2padctrl.reg32, value);
        s->gp_atcfg2padctrl.reg32 = value;
        break;
    case GP_CDEV1CFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_cdev1cfgpadctrl.reg32, value);
        s->gp_cdev1cfgpadctrl.reg32 = value;
        break;
    case GP_CDEV2CFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_cdev2cfgpadctrl.reg32, value);
        s->gp_cdev2cfgpadctrl.reg32 = value;
        break;
    case GP_CSUSCFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_csuscfgpadctrl.reg32, value);
        s->gp_csuscfgpadctrl.reg32 = value;
        break;
    case GP_DAP1CFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_dap1cfgpadctrl.reg32, value);
        s->gp_dap1cfgpadctrl.reg32 = value;
        break;
    case GP_DAP2CFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_dap2cfgpadctrl.reg32, value);
        s->gp_dap2cfgpadctrl.reg32 = value;
        break;
    case GP_DAP3CFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_dap3cfgpadctrl.reg32, value);
        s->gp_dap3cfgpadctrl.reg32 = value;
        break;
    case GP_DAP4CFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_dap4cfgpadctrl.reg32, value);
        s->gp_dap4cfgpadctrl.reg32 = value;
        break;
    case GP_DBGCFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_dbgcfgpadctrl.reg32, value);
        s->gp_dbgcfgpadctrl.reg32 = value;
        break;
    case GP_LCDCFG1PADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_lcdcfg1padctrl.reg32, value);
        s->gp_lcdcfg1padctrl.reg32 = value;
        break;
    case GP_LCDCFG2PADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_lcdcfg2padctrl.reg32, value);
        s->gp_lcdcfg2padctrl.reg32 = value;
        break;
    case GP_SDIO2CFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_sdio2cfgpadctrl.reg32, value);
        s->gp_sdio2cfgpadctrl.reg32 = value;
        break;
    case GP_SDIO3CFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_sdio3cfgpadctrl.reg32, value);
        s->gp_sdio3cfgpadctrl.reg32 = value;
        break;
    case GP_SPICFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_spicfgpadctrl.reg32, value);
        s->gp_spicfgpadctrl.reg32 = value;
        break;
    case GP_UAACFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_uaacfgpadctrl.reg32, value);
        s->gp_uaacfgpadctrl.reg32 = value;
        break;
    case GP_UABCFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_uabcfgpadctrl.reg32, value);
        s->gp_uabcfgpadctrl.reg32 = value;
        break;
    case GP_UART2CFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_uart2cfgpadctrl.reg32, value);
        s->gp_uart2cfgpadctrl.reg32 = value;
        break;
    case GP_UART3CFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_uart3cfgpadctrl.reg32, value);
        s->gp_uart3cfgpadctrl.reg32 = value;
        break;
    case GP_VICFG1PADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_vicfg1padctrl.reg32, value);
        s->gp_vicfg1padctrl.reg32 = value;
        break;
    case GP_VICFG2PADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_vicfg2padctrl.reg32, value);
        s->gp_vicfg2padctrl.reg32 = value;
        break;
    case GP_XM2CFGAPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_xm2cfgapadctrl.reg32, value);
        s->gp_xm2cfgapadctrl.reg32 = value;
        break;
    case GP_XM2CFGCPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_xm2cfgcpadctrl.reg32, value);
        s->gp_xm2cfgcpadctrl.reg32 = value;
        break;
    case GP_XM2CFGDPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_xm2cfgdpadctrl.reg32, value);
        s->gp_xm2cfgdpadctrl.reg32 = value;
        break;
    case GP_XM2CLKCFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_xm2clkcfgpadctrl.reg32, value);
        s->gp_xm2clkcfgpadctrl.reg32 = value;
        break;
    case GP_XM2COMPPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_xm2comppadctrl.reg32, value);
        s->gp_xm2comppadctrl.reg32 = value;
        break;
    case GP_XM2VTTGENPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_xm2vttgenpadctrl.reg32, value);
        s->gp_xm2vttgenpadctrl.reg32 = value;
        break;
    case GP_PADCTL_DFT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_padctl_dft.reg32, value);
        s->gp_padctl_dft.reg32 = value;
        break;
    case GP_SDIO1CFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_sdio1cfgpadctrl.reg32, value);
        s->gp_sdio1cfgpadctrl.reg32 = value;
        break;
    case GP_XM2CFGCPADCTRL2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_xm2cfgcpadctrl2.reg32, value);
        s->gp_xm2cfgcpadctrl2.reg32 = value;
        break;
    case GP_XM2CFGDPADCTRL2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_xm2cfgdpadctrl2.reg32, value);
        s->gp_xm2cfgdpadctrl2.reg32 = value;
        break;
    case GP_CRTCFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_crtcfgpadctrl.reg32, value);
        s->gp_crtcfgpadctrl.reg32 = value;
        break;
    case GP_DDCCFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_ddccfgpadctrl.reg32, value);
        s->gp_ddccfgpadctrl.reg32 = value;
        break;
    case GP_GMACFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_gmacfgpadctrl.reg32, value);
        s->gp_gmacfgpadctrl.reg32 = value;
        break;
    case GP_GMBCFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_gmbcfgpadctrl.reg32, value);
        s->gp_gmbcfgpadctrl.reg32 = value;
        break;
    case GP_GMCCFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_gmccfgpadctrl.reg32, value);
        s->gp_gmccfgpadctrl.reg32 = value;
        break;
    case GP_GMDCFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_gmdcfgpadctrl.reg32, value);
        s->gp_gmdcfgpadctrl.reg32 = value;
        break;
    case GP_GMECFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_gmecfgpadctrl.reg32, value);
        s->gp_gmecfgpadctrl.reg32 = value;
        break;
    case GP_OWRCFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_owrcfgpadctrl.reg32, value);
        s->gp_owrcfgpadctrl.reg32 = value;
        break;
    case GP_UADCFGPADCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gp_uadcfgpadctrl.reg32, value);
        s->gp_uadcfgpadctrl.reg32 = value;
        break;
    case GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET ... GP_QSPI_SCK_LPBK_CONTROL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->regs[(offset - GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2] , value);
        if (tegra_board >= TEGRAX1_BOARD) s->regs[(offset - GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2] = value;
        break;
    case DAS_DAP_CTRL_SEL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->das_dap_ctrl_sel.reg32, value);
        s->das_dap_ctrl_sel.reg32 = value;
        break;
    case DAS_DAC_INPUT_DATA_CLK_SEL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->das_dac_input_data_clk_sel.reg32, value);
        s->das_dac_input_data_clk_sel.reg32 = value;
        break;
    case PP_MISC_SAVE_THE_DAY_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pp_misc_save_the_day.reg32, value);
        s->pp_misc_save_the_day.reg32 = value;
        break;
    case ASYNC_COREPWRCONFIG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->async_corepwrconfig.reg32, value);
        s->async_corepwrconfig.reg32 = value;
        break;
    case ASYNC_EMCPADEN_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->async_emcpaden.reg32, value);
        s->async_emcpaden.reg32 = value;
        break;
    case SC1X_PADS_VIP_VCLKCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->sc1x_pads_vip_vclkctrl.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->sc1x_pads_vip_vclkctrl.reg32 = value;
        break;
    case ASYNC_VCLKCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->async_vclkctrl.reg32, value);
        s->async_vclkctrl.reg32 = value;
        break;
    case ASYNC_TVDACVHSYNCCTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->async_tvdacvhsyncctrl.reg32, value);
        s->async_tvdacvhsyncctrl.reg32 = value;
        break;
    case ASYNC_TVDACCNTL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->async_tvdaccntl.reg32, value);
        s->async_tvdaccntl.reg32 = value;
        break;
    case ASYNC_TVDACSTATUS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->async_tvdacstatus.reg32, value & ASYNC_TVDACSTATUS_WRMASK);
        WR_MASKED(s->async_tvdacstatus.reg32, value, ASYNC_TVDACSTATUS);
        break;
    case ASYNC_TVDACDINCONFIG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->async_tvdacdinconfig.reg32, value);
        s->async_tvdacdinconfig.reg32 = value;
        break;
    case ASYNC_INT_STATUS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->async_int_status.reg32, value & ASYNC_INT_STATUS_WRMASK);
        WR_MASKED(s->async_int_status.reg32, value, ASYNC_INT_STATUS);
        break;
    case ASYNC_INT_MASK_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->async_int_mask.reg32, value);
        s->async_int_mask.reg32 = value;
        break;
    case ASYNC_INT_POLARITY_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->async_int_polarity.reg32, value);
        s->async_int_polarity.reg32 = value;
        break;
    case ASYNC_INT_TYPE_SELECT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->async_int_type_select.reg32, value);
        s->async_int_type_select.reg32 = value;
        break;
    case DAS_DAP_CTRL_SEL_1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->das_dap_ctrl_sel_1.reg32, value);
        s->das_dap_ctrl_sel_1.reg32 = value;
        break;
    case DAS_DAP_CTRL_SEL_2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->das_dap_ctrl_sel_2.reg32, value);
        s->das_dap_ctrl_sel_2.reg32 = value;
        break;
    case DAS_DAP_CTRL_SEL_3_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->das_dap_ctrl_sel_3.reg32, value);
        s->das_dap_ctrl_sel_3.reg32 = value;
        break;
    case DAS_DAP_CTRL_SEL_4_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->das_dap_ctrl_sel_4.reg32, value);
        s->das_dap_ctrl_sel_4.reg32 = value;
        break;
    case DAS_DAC_INPUT_DATA_CLK_SEL_1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->das_dac_input_data_clk_sel_1.reg32, value);
        s->das_dac_input_data_clk_sel_1.reg32 = value;
        break;
    case DAS_DAC_INPUT_DATA_CLK_SEL_2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->das_dac_input_data_clk_sel_2.reg32, value);
        s->das_dac_input_data_clk_sel_2.reg32 = value;
        break;
    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }
}

static void tegra_apb_misc_priv_reset(DeviceState *dev)
{
    tegra_apb_misc *s = TEGRA_APB_MISC(dev);

    s->pp_tristate_reg_a.reg32 = PP_TRISTATE_REG_A_RESET;
    s->pp_tristate_reg_b.reg32 = PP_TRISTATE_REG_B_RESET;
    s->pp_tristate_reg_c.reg32 = PP_TRISTATE_REG_C_RESET;
    s->pp_tristate_reg_d.reg32 = PP_TRISTATE_REG_D_RESET;
    s->pp_misc_usb_otg.reg32 = PP_MISC_USB_OTG_RESET;
    s->pp_pinmux_global.reg32 = PP_PINMUX_GLOBAL_RESET;
    s->pp_usb_phy_param.reg32 = PP_USB_PHY_PARAM_RESET;
    s->pp_usb_phy_vbus_sensors.reg32 = PP_USB_PHY_VBUS_SENSORS_RESET;
    s->pp_usb_phy_vbus_wakeup_id.reg32 = PP_USB_PHY_VBUS_WAKEUP_ID_RESET;
    s->pp_usb_phy_alt_vbus_sts.reg32 = PP_USB_PHY_ALT_VBUS_STS_RESET;
    s->pp_pin_mux_ctl_a.reg32 = PP_PIN_MUX_CTL_A_RESET;
    s->pp_pin_mux_ctl_b.reg32 = PP_PIN_MUX_CTL_B_RESET;
    s->pp_pin_mux_ctl_c.reg32 = PP_PIN_MUX_CTL_C_RESET;
    s->pp_pin_mux_ctl_d.reg32 = PP_PIN_MUX_CTL_D_RESET;
    s->pp_pin_mux_ctl_e.reg32 = PP_PIN_MUX_CTL_E_RESET;
    s->pp_pin_mux_ctl_f.reg32 = PP_PIN_MUX_CTL_F_RESET;
    s->pp_pin_mux_ctl_g.reg32 = PP_PIN_MUX_CTL_G_RESET;
    s->pp_pin_mux_ctl_h.reg32 = PP_PIN_MUX_CTL_H_RESET;
    s->pp_pullupdown_reg_a.reg32 = PP_PULLUPDOWN_REG_A_RESET;
    s->pp_pullupdown_reg_b.reg32 = PP_PULLUPDOWN_REG_B_RESET;
    s->pp_pullupdown_reg_c.reg32 = PP_PULLUPDOWN_REG_C_RESET;
    s->pp_pullupdown_reg_d.reg32 = PP_PULLUPDOWN_REG_D_RESET;
    s->pp_pullupdown_reg_e.reg32 = PP_PULLUPDOWN_REG_E_RESET;
    s->pp_misc_usb_clk_rst_ctl.reg32 = PP_MISC_USB_CLK_RST_CTL_RESET;
    s->gp_modereg.reg32 = GP_MODEREG_RESET;
    s->gp_hidrev.reg32 = GP_HIDREV_RESET;
    s->gp_asdbgreg.reg32 = GP_ASDBGREG_RESET;
    s->gp_emu_revid.reg32 = GP_EMU_REVID_RESET;
    s->gp_transactor_scratch.reg32 = GP_TRANSACTOR_SCRATCH_RESET;
    s->gp_aocfg2padctrl.reg32 = GP_AOCFG2PADCTRL_RESET;
    s->gp_atcfg1padctrl.reg32 = GP_ATCFG1PADCTRL_RESET;
    s->gp_atcfg2padctrl.reg32 = GP_ATCFG2PADCTRL_RESET;
    s->gp_cdev1cfgpadctrl.reg32 = GP_CDEV1CFGPADCTRL_RESET;
    s->gp_cdev2cfgpadctrl.reg32 = GP_CDEV2CFGPADCTRL_RESET;
    s->gp_dbgcfgpadctrl.reg32 = GP_DBGCFGPADCTRL_RESET;
    s->gp_lcdcfg1padctrl.reg32 = GP_LCDCFG1PADCTRL_RESET;
    s->gp_lcdcfg2padctrl.reg32 = GP_LCDCFG2PADCTRL_RESET;
    s->gp_sdio2cfgpadctrl.reg32 = GP_SDIO2CFGPADCTRL_RESET;
    s->gp_sdio3cfgpadctrl.reg32 = GP_SDIO3CFGPADCTRL_RESET;
    s->gp_spicfgpadctrl.reg32 = GP_SPICFGPADCTRL_RESET;
    s->gp_uaacfgpadctrl.reg32 = GP_UAACFGPADCTRL_RESET;
    s->gp_uabcfgpadctrl.reg32 = GP_UABCFGPADCTRL_RESET;
    s->gp_uart2cfgpadctrl.reg32 = GP_UART2CFGPADCTRL_RESET;
    s->gp_uart3cfgpadctrl.reg32 = GP_UART3CFGPADCTRL_RESET;
    s->gp_vicfg1padctrl.reg32 = GP_VICFG1PADCTRL_RESET;
    s->gp_vicfg2padctrl.reg32 = GP_VICFG2PADCTRL_RESET;
    s->gp_xm2cfgapadctrl.reg32 = GP_XM2CFGAPADCTRL_RESET;
    s->gp_xm2cfgcpadctrl.reg32 = GP_XM2CFGCPADCTRL_RESET;
    s->gp_xm2cfgdpadctrl.reg32 = GP_XM2CFGDPADCTRL_RESET;
    s->das_dap_ctrl_sel.reg32 = DAS_DAP_CTRL_SEL_RESET;
    s->das_dac_input_data_clk_sel.reg32 = DAS_DAC_INPUT_DATA_CLK_SEL_RESET;
    s->pp_misc_save_the_day.reg32 = PP_MISC_SAVE_THE_DAY_RESET;
    s->async_corepwrconfig.reg32 = ASYNC_COREPWRCONFIG_RESET;
    s->async_emcpaden.reg32 = ASYNC_EMCPADEN_RESET;
    s->sc1x_pads_vip_vclkctrl.reg32 = SC1X_PADS_VIP_VCLKCTRL_RESET;
    s->async_vclkctrl.reg32 = ASYNC_VCLKCTRL_RESET;
    s->async_tvdacvhsyncctrl.reg32 = ASYNC_TVDACVHSYNCCTRL_RESET;
    s->async_tvdaccntl.reg32 = ASYNC_TVDACCNTL_RESET;
    s->async_tvdacstatus.reg32 = ASYNC_TVDACSTATUS_RESET;
    s->async_tvdacdinconfig.reg32 = ASYNC_TVDACDINCONFIG_RESET;
    s->async_int_status.reg32 = ASYNC_INT_STATUS_RESET;
    s->async_int_mask.reg32 = ASYNC_INT_MASK_RESET;
    s->async_int_polarity.reg32 = ASYNC_INT_POLARITY_RESET;
    s->async_int_type_select.reg32 = ASYNC_INT_TYPE_SELECT_RESET;
    s->das_dap_ctrl_sel_1.reg32 = DAS_DAP_CTRL_SEL_1_RESET;
    s->das_dap_ctrl_sel_2.reg32 = DAS_DAP_CTRL_SEL_2_RESET;
    s->das_dap_ctrl_sel_3.reg32 = DAS_DAP_CTRL_SEL_3_RESET;
    s->das_dap_ctrl_sel_4.reg32 = DAS_DAP_CTRL_SEL_4_RESET;
    s->das_dac_input_data_clk_sel_1.reg32 = DAS_DAC_INPUT_DATA_CLK_SEL_1_RESET;
    s->das_dac_input_data_clk_sel_2.reg32 = DAS_DAC_INPUT_DATA_CLK_SEL_2_RESET;

    if (tegra_board >= TEGRAX1_BOARD) {
        s->pp_config_ctl.reg32 = PP_CONFIG_CTL_TEGRAX1_RESET;
        s->gp_aocfg1padctrl.reg32 = GP_AOCFG1PADCTRL_TEGRAX1_RESET;
        s->gp_csuscfgpadctrl.reg32 = GP_CSUSCFGPADCTRL_TEGRAX1_RESET;
        s->gp_dap1cfgpadctrl.reg32 = GP_DAP1CFGPADCTRL_TEGRAX1_RESET;
        s->gp_dap2cfgpadctrl.reg32 = GP_DAP2CFGPADCTRL_TEGRAX1_RESET;
        s->gp_dap3cfgpadctrl.reg32 = GP_DAP3CFGPADCTRL_TEGRAX1_RESET;
        s->gp_dap4cfgpadctrl.reg32 = GP_DAP4CFGPADCTRL_TEGRAX1_RESET;
        s->gp_xm2clkcfgpadctrl.reg32 = GP_XM2CLKCFGPADCTRL_TEGRAX1_RESET;
        s->gp_xm2comppadctrl.reg32 = GP_XM2COMPPADCTRL_TEGRAX1_RESET;
        s->gp_xm2vttgenpadctrl.reg32 = GP_XM2VTTGENPADCTRL_TEGRAX1_RESET;
        s->gp_padctl_dft.reg32 = GP_PADCTL_DFT_TEGRAX1_RESET;
        s->gp_sdio1cfgpadctrl.reg32 = GP_SDIO1CFGPADCTRL_TEGRAX1_RESET;
        s->gp_xm2cfgcpadctrl2.reg32 = GP_XM2CFGCPADCTRL2_TEGRAX1_RESET;
        s->gp_xm2cfgdpadctrl2.reg32 = GP_XM2CFGDPADCTRL2_TEGRAX1_RESET;
        s->gp_crtcfgpadctrl.reg32 = GP_CRTCFGPADCTRL_TEGRAX1_RESET;
        s->gp_ddccfgpadctrl.reg32 = GP_DDCCFGPADCTRL_TEGRAX1_RESET;
        s->gp_gmacfgpadctrl.reg32 = GP_GMACFGPADCTRL_TEGRAX1_RESET;
        s->gp_gmbcfgpadctrl.reg32 = GP_GMBCFGPADCTRL_TEGRAX1_RESET;
        s->gp_gmccfgpadctrl.reg32 = GP_GMCCFGPADCTRL_TEGRAX1_RESET;
        s->gp_gmdcfgpadctrl.reg32 = GP_GMDCFGPADCTRL_TEGRAX1_RESET;
        s->gp_gmecfgpadctrl.reg32 = GP_GMECFGPADCTRL_TEGRAX1_RESET;
        s->gp_owrcfgpadctrl.reg32 = GP_OWRCFGPADCTRL_TEGRAX1_RESET;
        s->gp_uadcfgpadctrl.reg32 = GP_UADCFGPADCTRL_TEGRAX1_RESET;
    }
    else {
        s->pp_config_ctl.reg32 = PP_CONFIG_CTL_TEGRA2_RESET;
        s->gp_aocfg1padctrl.reg32 = GP_AOCFG1PADCTRL_TEGRA2_RESET;
        s->gp_csuscfgpadctrl.reg32 = GP_CSUSCFGPADCTRL_TEGRA2_RESET;
        s->gp_dap1cfgpadctrl.reg32 = GP_DAP1CFGPADCTRL_TEGRA2_RESET;
        s->gp_dap2cfgpadctrl.reg32 = GP_DAP2CFGPADCTRL_TEGRA2_RESET;
        s->gp_dap3cfgpadctrl.reg32 = GP_DAP3CFGPADCTRL_TEGRA2_RESET;
        s->gp_dap4cfgpadctrl.reg32 = GP_DAP4CFGPADCTRL_TEGRA2_RESET;
        s->gp_xm2clkcfgpadctrl.reg32 = GP_XM2CLKCFGPADCTRL_TEGRA2_RESET;
        s->gp_xm2comppadctrl.reg32 = GP_XM2COMPPADCTRL_TEGRA2_RESET;
        s->gp_xm2vttgenpadctrl.reg32 = GP_XM2VTTGENPADCTRL_TEGRA2_RESET;
        s->gp_padctl_dft.reg32 = GP_PADCTL_DFT_TEGRA2_RESET;
        s->gp_sdio1cfgpadctrl.reg32 = GP_SDIO1CFGPADCTRL_TEGRA2_RESET;
        s->gp_xm2cfgcpadctrl2.reg32 = GP_XM2CFGCPADCTRL2_TEGRA2_RESET;
        s->gp_xm2cfgdpadctrl2.reg32 = GP_XM2CFGDPADCTRL2_TEGRA2_RESET;
        s->gp_crtcfgpadctrl.reg32 = GP_CRTCFGPADCTRL_TEGRA2_RESET;
        s->gp_ddccfgpadctrl.reg32 = GP_DDCCFGPADCTRL_TEGRA2_RESET;
        s->gp_gmacfgpadctrl.reg32 = GP_GMACFGPADCTRL_TEGRA2_RESET;
        s->gp_gmbcfgpadctrl.reg32 = GP_GMBCFGPADCTRL_TEGRA2_RESET;
        s->gp_gmccfgpadctrl.reg32 = GP_GMCCFGPADCTRL_TEGRA2_RESET;
        s->gp_gmdcfgpadctrl.reg32 = GP_GMDCFGPADCTRL_TEGRA2_RESET;
        s->gp_gmecfgpadctrl.reg32 = GP_GMECFGPADCTRL_TEGRA2_RESET;
        s->gp_owrcfgpadctrl.reg32 = GP_OWRCFGPADCTRL_TEGRA2_RESET;
        s->gp_uadcfgpadctrl.reg32 = GP_UADCFGPADCTRL_TEGRA2_RESET;
    }

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[(GP_QSPI_COMP_CFGPADCTRL_OFFSET-GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2] = GP_QSPI_COMP_CFGPADCTRL_RESET;
    s->regs[(GP_QSPI_SCK_CFGPADCTRL_OFFSET-GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2] = GP_QSPI_SCK_CFGPADCTRL_RESET;
    s->regs[(GP_SDMMC1_PAD_CFGPADCTRL_OFFSET-GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2] = GP_SDMMC1_PAD_CFGPADCTRL_RESET;
    s->regs[(GP_EMMC2_PAD_CFGPADCTRL_OFFSET-GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2] = GP_EMMC2_PAD_CFGPADCTRL_RESET;
    s->regs[(GP_EMMC2_PAD_DRV_TYPE_CFGPADCTRL_OFFSET-GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2] = GP_EMMC2_PAD_DRV_TYPE_CFGPADCTRL_RESET;
    s->regs[(GP_EMMC2_PAD_PUPD_CFGPADCTRL_OFFSET-GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2] = GP_EMMC2_PAD_PUPD_CFGPADCTRL_RESET;
    s->regs[(GP_SDMMC3_PAD_CFGPADCTRL_OFFSET-GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2] = GP_SDMMC3_PAD_CFGPADCTRL_RESET;
    s->regs[(GP_EMMC4_PAD_CFGPADCTRL_OFFSET-GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2] = GP_EMMC4_PAD_CFGPADCTRL_RESET;
    s->regs[(GP_EMMC4_PAD_DRV_TYPE_CFGPADCTRL_OFFSET-GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2] = GP_EMMC4_PAD_DRV_TYPE_CFGPADCTRL_RESET;
    s->regs[(GP_EMMC4_PAD_PUPD_CFGPADCTRL_OFFSET-GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2] = GP_EMMC4_PAD_PUPD_CFGPADCTRL_RESET;
    s->regs[(GP_QSPI_COMP_CONTROL_OFFSET-GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2] = GP_QSPI_COMP_CONTROL_RESET;
    s->regs[(GP_VGPIO_GPIO_MUX_SEL_OFFSET-GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2] = GP_VGPIO_GPIO_MUX_SEL_RESET;
    s->regs[(GP_QSPI_SCK_LPBK_CONTROL_OFFSET-GP_BUTTON_VOL_DOWN_CFGPADCTRL_OFFSET)>>2] = GP_QSPI_SCK_LPBK_CONTROL_RESET;

    s->gp_hidrev.chipid = 0x20;
    s->gp_hidrev.hidfam = 0x7;
    s->gp_hidrev.majorrev = 0x1;
    s->gp_hidrev.minorrev = 0x4;
}

static const MemoryRegionOps tegra_apb_misc_mem_ops = {
    .read = tegra_apb_misc_priv_read,
    .write = tegra_apb_misc_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_apb_misc_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_apb_misc *s = TEGRA_APB_MISC(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_apb_misc_mem_ops, s,
                          "tegra.apb_misc", TEGRA_APB_MISC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static Property tegra_apb_misc_properties[] = {
    DEFINE_PROP_UINT32("pp-strapping-opt-a", tegra_apb_misc, pp_strapping_opt_a.reg32, PP_STRAPPING_OPT_A_RESET | 7<<10), \
    DEFINE_PROP_END_OF_LIST(),
};

static void tegra_apb_misc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, tegra_apb_misc_properties);
    dc->realize = tegra_apb_misc_priv_realize;
    dc->vmsd = &vmstate_tegra_apb_misc;
    dc->reset = tegra_apb_misc_priv_reset;
}

static const TypeInfo tegra_apb_misc_info = {
    .name = TYPE_TEGRA_APB_MISC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_apb_misc),
    .class_init = tegra_apb_misc_class_init,
};

static void tegra_apb_misc_register_types(void)
{
    type_register_static(&tegra_apb_misc_info);
}

type_init(tegra_apb_misc_register_types)
