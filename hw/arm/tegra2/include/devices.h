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

#ifndef TEGRA_DEVICES_H
#define TEGRA_DEVICES_H

extern void * tegra_a9mpcore_dev;
extern void * gic_dev;
extern void * tegra_apb_dma_dev;
extern void * tegra_ahb_gizmo_dev;
extern void * tegra_ahbapb_debugbus_dev;
extern void * tegra_apb_misc_dev;
extern void * tegra_pinmuxaux_dev;
extern void * tegra_car_dev;
extern void * tegra_emc_dev;
extern void * tegra_emc0_dev;
extern void * tegra_emc1_dev;
extern void * tegra_sata_dev;
extern void * tegra_hda_dev;
extern void * tegra_ape_dev;
extern void * tegra_dds_dev;
extern void * tegra_flow_dev;
extern void * tegra_fuse_dev;
extern void * tegra_se_dev;
extern void * tegra_se2_dev;
extern void * tegra_tsensor_dev;
extern void * tegra_cec_dev;
extern void * tegra_sysctr0_dev;
extern void * tegra_sysctr1_dev;
extern void * tegra_soctherm_dev;
extern void * tegra_avpcache_dev;
extern void * tegra_gpios_dev;
extern void * tegra_mc_dev;
extern void * tegra_mc0_dev;
extern void * tegra_mc1_dev;
extern void * tegra_pmc_dev;
extern void * tegra_rtc_dev;
extern void * tegra_sdmmc1_dev;
extern void * tegra_sdmmc2_dev;
extern void * tegra_sdmmc3_dev;
extern void * tegra_sdmmc4_dev;
extern void * tegra_sdmmc1_vendor_dev;
extern void * tegra_sdmmc2_vendor_dev;
extern void * tegra_sdmmc3_vendor_dev;
extern void * tegra_sdmmc4_vendor_dev;
extern void * tegra_speedo_dev;
extern void * tegra_dp2_dev;
extern void * tegra_apb2jtag_dev;
extern void * tegra_timer_devs[14];
extern void * tegra_timer_us_dev;
extern void * tegra_wdt_devs[5];
extern void * tegra_timer_shared_dev;
extern void * tegra_uarta_dev;
extern void * tegra_uartb_dev;
extern void * tegra_uartc_dev;
extern void * tegra_uartd_dev;
extern void * tegra_uarta_vendor_dev;
extern void * tegra_uartb_vendor_dev;
extern void * tegra_uartc_vendor_dev;
extern void * tegra_uartd_vendor_dev;
extern void * tegra_iobist_dev;
extern void * tegra_pwm_dev;
extern void * tegra_dca_dev;
extern void * tegra_dcb_dev;
extern void * tegra_dsi_dev;
extern void * tegra_dsib_dev;
extern void * tegra_sor_dev;
extern void * tegra_sor1_dev;
extern void * tegra_tsec_dev;
extern void * tegra_tsecb_dev;
extern void * tegra_vic_dev;
extern void * tegra_nvenc_dev;
extern void * tegra_nvdec_dev;
extern void * tegra_nvjpg_dev;
extern void * tegra_dpaux_dev;
extern void * tegra_dpaux1_dev;
extern void * tegra_mipical_dev;
extern void * tegra_dvfs_dev;
extern void * tegra_cluster_clock_dev;
extern void * tegra_ehci1_dev;
extern void * tegra_ehci2_dev;
extern void * tegra_ehci3_dev;
extern void * tegra_bsea_dev;
extern void * tegra_bsev_dev;
extern void * tegra_idc1_dev;
extern void * tegra_idc2_dev;
extern void * tegra_idc3_dev;
extern void * tegra_idc4_dev;
extern void * tegra_idc5_dev;
extern void * tegra_idc6_dev;
extern void * tegra_dvc_dev;
extern void * tegra_spi_devs[6];
extern void * tegra_qspi_dev;
extern void * tegra_i2c_tmpsensor_dev;
extern void * tegra_i2c_rtc_dev;
extern void * tegra_i2c_pmic_dev;
extern void * tegra_i2c_subpmic_devs[4];
extern void * tegra_i2c_charger_dev;
extern void * tegra_i2c_fuel_dev;
extern void * tegra_grhost_dev;
extern void * tegra_arb_sema_dev;
extern void * tegra_arb_gnt_ictlr_dev;
extern void * tegra_irq_dispatcher_dev;
extern void * tegra_mselect_dev;
extern void * tegra_evp_dev;
extern void * tegra_sb_dev;
extern void * tegra_actmon_dev;
extern void * tegra_ictlr_dev;
extern void * tegra_gr2d_dev;
extern void * tegra_host1x_dev;
extern void * tegra_vi_dev;
extern void * tegra_csi_dev;
extern void * tegra_isp_dev;
extern void * tegra_ispb_dev;
extern void * tegra_vii2c_dev;
extern void * tegra_cop_mmu_dev;
extern void * tegra_res_sema_dev;
extern void * tegra_ucq_dev;
extern void * tegra_sxe_dev;
extern void * tegra_mbe_dev;
extern void * tegra_ppe_dev;
extern void * tegra_mce_dev;
extern void * tegra_tfe_dev;
extern void * tegra_ppb_dev;
extern void * tegra_vdma_dev;
extern void * tegra_ucq2_dev;
extern void * tegra_bsea2_dev;
extern void * tegra_frameid_dev;
extern void *tegra_ahb_dma_dev;

void tegra_a9mpcore_reset(void);
void tegra_device_reset(void *dev_);

#endif // TEGRA_DEVICES_H
