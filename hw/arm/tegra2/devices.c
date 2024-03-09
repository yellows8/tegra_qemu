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

#include "hw/cpu/a9mpcore.h"
#include "sysemu/sysemu.h"
#include "hw/irq.h"

#include "devices.h"
#include "tegra_trace.h"

void *tegra_a9mpcore_dev = NULL;
void *gic_dev = NULL;
void *tegra_apb_dma_dev = NULL;
void *tegra_ahb_gizmo_dev = NULL;
void *tegra_ahbapb_debugbus_dev = NULL;
void *tegra_apb_misc_dev = NULL;
void *tegra_pinmuxaux_dev = NULL;
void *tegra_car_dev = NULL;
void *tegra_emc_dev = NULL;
void *tegra_emc0_dev = NULL;
void *tegra_emc1_dev = NULL;
void *tegra_sata_dev = NULL;
void *tegra_hda_dev = NULL;
void *tegra_ape_dev = NULL;
void *tegra_dds_dev = NULL;
void *tegra_flow_dev = NULL;
void *tegra_fuse_dev = NULL;
void *tegra_se_dev = NULL;
void *tegra_se2_dev = NULL;
void *tegra_tsensor_dev = NULL;
void *tegra_cec_dev = NULL;
void *tegra_sysctr0_dev = NULL;
void *tegra_sysctr1_dev = NULL;
void *tegra_soctherm_dev = NULL;
void *tegra_avpcache_dev = NULL;
void *tegra_gpios_dev = NULL;
void *tegra_mc_dev = NULL;
void *tegra_mc0_dev = NULL;
void *tegra_mc1_dev = NULL;
void *tegra_pmc_dev = NULL;
void *tegra_rtc_dev = NULL;
void *tegra_sdmmc1_dev = NULL;
void *tegra_sdmmc2_dev = NULL;
void *tegra_sdmmc3_dev = NULL;
void *tegra_sdmmc4_dev = NULL;
void *tegra_sdmmc1_vendor_dev = NULL;
void *tegra_sdmmc2_vendor_dev = NULL;
void *tegra_sdmmc3_vendor_dev = NULL;
void *tegra_sdmmc4_vendor_dev = NULL;
void *tegra_speedo_dev = NULL;
void *tegra_dp2_dev = NULL;
void *tegra_apb2jtag_dev = NULL;
void *tegra_timer_devs[14] = {};
void *tegra_timer_us_dev = NULL;
void *tegra_wdt_devs[5] = {};
void *tegra_timer_shared_dev = NULL;
void *tegra_uarta_dev = NULL;
void *tegra_uartb_dev = NULL;
void *tegra_uartc_dev = NULL;
void *tegra_uartd_dev = NULL;
void *tegra_uarta_vendor_dev = NULL;
void *tegra_uartb_vendor_dev = NULL;
void *tegra_uartc_vendor_dev = NULL;
void *tegra_uartd_vendor_dev = NULL;
void *tegra_iobist_dev = NULL;
void *tegra_pwm_dev = NULL;
void *tegra_dca_dev = NULL;
void *tegra_dcb_dev = NULL;
void *tegra_dsi_dev = NULL;
void *tegra_dsib_dev = NULL;
void *tegra_sor_dev = NULL;
void *tegra_sor1_dev = NULL;
void *tegra_tsec_dev = NULL;
void *tegra_tsecb_dev = NULL;
void *tegra_vic_dev = NULL;
void *tegra_nvenc_dev = NULL;
void *tegra_nvdec_dev = NULL;
void *tegra_nvjpg_dev = NULL;
void *tegra_dpaux_dev = NULL;
void *tegra_dpaux1_dev = NULL;
void *tegra_mipical_dev = NULL;
void *tegra_dvfs_dev = NULL;
void *tegra_cluster_clock_dev = NULL;
void *tegra_ehci1_dev = NULL;
void *tegra_ehci2_dev = NULL;
void *tegra_ehci3_dev = NULL;
void *tegra_xusb_dev = NULL;
void *tegra_bsea_dev = NULL;
void *tegra_bsev_dev = NULL;
void *tegra_idc1_dev = NULL;
void *tegra_idc2_dev = NULL;
void *tegra_idc3_dev = NULL;
void *tegra_idc4_dev = NULL;
void *tegra_idc5_dev = NULL;
void *tegra_idc6_dev = NULL;
void *tegra_dvc_dev = NULL;
void *tegra_spi_devs[6] = {};
void *tegra_qspi_dev = NULL;
void *tegra_i2c_tmpsensor_dev = NULL;
void *tegra_i2c_rtc_dev = NULL;
void *tegra_i2c_pmic_dev = NULL;
void *tegra_i2c_subpmic_devs[4] = {};
void *tegra_i2c_charger_dev = NULL;
void *tegra_i2c_fuel_dev = NULL;
void *tegra_grhost_dev = NULL;
void *tegra_arb_sema_dev = NULL;
void *tegra_arb_gnt_ictlr_dev = NULL;
void *tegra_irq_dispatcher_dev = NULL;
void *tegra_mselect_dev = NULL;
void *tegra_evp_dev = NULL;
void *tegra_sb_dev = NULL;
void *tegra_actmon_dev = NULL;
void *tegra_ictlr_dev = NULL;
void *tegra_gr2d_dev = NULL;
void *tegra_host1x_dev = NULL;
void *tegra_vi_dev = NULL;
void *tegra_csi_dev = NULL;
void *tegra_isp_dev = NULL;
void *tegra_ispb_dev = NULL;
void *tegra_vii2c_dev = NULL;
void *tegra_gpu_dev = NULL;
void *tegra_cop_mmu_dev = NULL;
void *tegra_res_sema_dev = NULL;
void *tegra_ucq_dev = NULL;
void *tegra_sxe_dev = NULL;
void *tegra_mbe_dev = NULL;
void *tegra_ppe_dev = NULL;
void *tegra_mce_dev = NULL;
void *tegra_tfe_dev = NULL;
void *tegra_ppb_dev = NULL;
void *tegra_vdma_dev = NULL;
void *tegra_ucq2_dev = NULL;
void *tegra_bsea2_dev = NULL;
void *tegra_frameid_dev = NULL;
void *tegra_ahb_dma_dev = NULL;

void tegra_a9mpcore_reset(void)
{
    if (tegra_board < TEGRAX1_BOARD) {
        A9MPPrivState *a9mpcore = A9MPCORE_PRIV(tegra_a9mpcore_dev);

        tegra_device_reset(&a9mpcore->scu);
        tegra_device_reset(&a9mpcore->gtimer);
        tegra_device_reset(&a9mpcore->mptimer);
        tegra_device_reset(&a9mpcore->wdt);
    }

    /* FIXME: GIC reset is broken */
//     device_reset( DEVICE(&a9mpcore->gic) );
}

void tegra_device_reset(void *dev_)
{
    DeviceState *dev = DEVICE(dev_);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int i = 0;

    if (dev_ == NULL) {
        return;
    }

    device_cold_reset(dev);

    while (sysbus_has_irq(sbd, i)) {
        /* Clear the IRQ state.  */
        qemu_irq_lower( sysbus_get_connected_irq(sbd, i++) );
    }
}
