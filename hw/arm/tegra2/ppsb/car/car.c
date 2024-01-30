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
#include "sysemu/runstate.h"
#include "sysemu/sysemu.h"

#include "car.h"
#include "clk_rst.h"
#include "devices.h"
#include "iomap.h"
#include "remote_io.h"
#include "tegra_cpu.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_CAR "tegra.car"
#define TEGRA_CAR(obj) OBJECT_CHECK(tegra_car, (obj), TYPE_TEGRA_CAR)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

#define PLL_LOCKED (1 << 27)

typedef struct tegra_car_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t rst_cpu_cmplx_clr_offset;
    DEFINE_REG32(rst_source);
    DEFINE_REG32(rst_devices_l);
    DEFINE_REG32(rst_devices_h);
    DEFINE_REG32(rst_devices_u);
    DEFINE_REG32(clk_out_enb_l);
    DEFINE_REG32(clk_out_enb_h);
    DEFINE_REG32(clk_out_enb_u);
    DEFINE_REG32(cclk_burst_policy);
    DEFINE_REG32(super_cclk_divider);
    DEFINE_REG32(sclk_burst_policy);
    DEFINE_REG32(super_sclk_divider);
    DEFINE_REG32(clk_system_rate);
    DEFINE_REG32(prog_dly_clk);
    DEFINE_REG32(audio_sync_clk_rate);
    DEFINE_REG32(cop_clk_skip_policy);
    DEFINE_REG32(clk_mask_arm);
    DEFINE_REG32(misc_clk_enb);
    DEFINE_REG32(clk_cpu_cmplx);
    DEFINE_REG32(osc_ctrl);
    DEFINE_REG32(pll_lfsr);
    DEFINE_REG32(osc_freq_det);
    DEFINE_REG32(osc_freq_det_status);
    DEFINE_REG32(plle_ss_cntl);
    DEFINE_REG32(plle_misc1);
    DEFINE_REG32(pllc_base);
    DEFINE_REG32(pllc_out);
    DEFINE_REG32(pllc_misc);
    DEFINE_REG32(pllc_misc_1);
    DEFINE_REG32(pllm_base);
    DEFINE_REG32(pllm_out);
    DEFINE_REG32(pllm_misc1);
    DEFINE_REG32(pllm_misc2);
    DEFINE_REG32(pllp_base);
    DEFINE_REG32(pllp_outa);
    DEFINE_REG32(pllp_outb);
    DEFINE_REG32(pllp_misc);
    DEFINE_REG32(plla_base);
    DEFINE_REG32(plla_out);
    DEFINE_REG32(plla_misc1);
    DEFINE_REG32(plla_misc);
    DEFINE_REG32(pllu_base);
    DEFINE_REG32(pllu_outa);
    DEFINE_REG32(pllu_misc1);
    DEFINE_REG32(pllu_misc);
    DEFINE_REG32(plld_base);
    DEFINE_REG32(plld_misc1);
    DEFINE_REG32(plld_misc);
    DEFINE_REG32(pllx_base);
    DEFINE_REG32(pllx_misc);
    DEFINE_REG32(plle_base);
    DEFINE_REG32(plle_misc);
    DEFINE_REG32(plle_ss_cntl1);
    DEFINE_REG32(plle_ss_cntl2);
    DEFINE_REG32(lvl2_clk_gate_ovra);
    DEFINE_REG32(lvl2_clk_gate_ovrb);
    DEFINE_REG32(clk_source_i2s1);
    DEFINE_REG32(clk_source_i2s2);
    DEFINE_REG32(clk_source_spdif_out);
    DEFINE_REG32(clk_source_spdif_in);
    DEFINE_REG32(clk_source_pwm);
    DEFINE_REG32(clk_source_spi1);
    DEFINE_REG32(clk_source_spi22);
    DEFINE_REG32(clk_source_spi3);
    DEFINE_REG32(clk_source_xio);
    DEFINE_REG32(clk_source_i2c1);
    DEFINE_REG32(clk_source_dvc_i2c);
    DEFINE_REG32(clk_source_twc);
    DEFINE_REG32(clk_source_sbc1);
    DEFINE_REG32(clk_source_disp1);
    DEFINE_REG32(clk_source_disp2);
    DEFINE_REG32(clk_source_cve);
    DEFINE_REG32(clk_source_ide);
    DEFINE_REG32(clk_source_vi);
    DEFINE_REG32(clk_source_sdmmc1);
    DEFINE_REG32(clk_source_sdmmc2);
    DEFINE_REG32(clk_source_g3d);
    DEFINE_REG32(clk_source_g2d);
    DEFINE_REG32(clk_source_ndflash);
    DEFINE_REG32(clk_source_sdmmc4);
    DEFINE_REG32(clk_source_vfir);
    DEFINE_REG32(clk_source_epp);
    DEFINE_REG32(clk_source_mpe);
    DEFINE_REG32(clk_source_mipi);
    DEFINE_REG32(clk_source_uart1);
    DEFINE_REG32(clk_source_uart2);
    DEFINE_REG32(clk_source_host1x);
    DEFINE_REG32(clk_source_tvo);
    DEFINE_REG32(clk_source_hdmi);
    DEFINE_REG32(clk_source_tvdac);
    DEFINE_REG32(clk_source_i2c2);
    DEFINE_REG32(clk_source_emc);
    DEFINE_REG32(clk_source_uart3);
    DEFINE_REG32(clk_source_vi_sensor);
    DEFINE_REG32(clk_source_spi4);
    DEFINE_REG32(clk_source_i2c3);
    DEFINE_REG32(clk_source_sdmmc3);
    DEFINE_REG32(clk_source_uart4);
    DEFINE_REG32(clk_source_uart5);
    DEFINE_REG32(clk_source_vde);
    DEFINE_REG32(clk_source_owr);
    DEFINE_REG32(clk_source_nor);
    DEFINE_REG32(clk_source_csite);
    DEFINE_REG32(clk_source_osc);
    DEFINE_REG32(clk_source_la);
    DEFINE_REG32(rst_cpu_cmplx_set);
    DEFINE_REG32(utmip_pll_cfg0);
    DEFINE_REG32(rst_controller_pllc4_base);
    DEFINE_REG32(rst_controller_pllmb_base);
    uint32_t regs[0x748>>2];
} tegra_car;

static const VMStateDescription vmstate_tegra_car = {
    .name = "tegra.car",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(rst_cpu_cmplx_clr_offset, tegra_car),
        VMSTATE_UINT32(rst_source.reg32, tegra_car),
        VMSTATE_UINT32(rst_devices_l.reg32, tegra_car),
        VMSTATE_UINT32(rst_devices_h.reg32, tegra_car),
        VMSTATE_UINT32(rst_devices_u.reg32, tegra_car),
        VMSTATE_UINT32(clk_out_enb_l.reg32, tegra_car),
        VMSTATE_UINT32(clk_out_enb_h.reg32, tegra_car),
        VMSTATE_UINT32(clk_out_enb_u.reg32, tegra_car),
        VMSTATE_UINT32(cclk_burst_policy.reg32, tegra_car),
        VMSTATE_UINT32(super_cclk_divider.reg32, tegra_car),
        VMSTATE_UINT32(sclk_burst_policy.reg32, tegra_car),
        VMSTATE_UINT32(super_sclk_divider.reg32, tegra_car),
        VMSTATE_UINT32(clk_system_rate.reg32, tegra_car),
        VMSTATE_UINT32(prog_dly_clk.reg32, tegra_car),
        VMSTATE_UINT32(audio_sync_clk_rate.reg32, tegra_car),
        VMSTATE_UINT32(cop_clk_skip_policy.reg32, tegra_car),
        VMSTATE_UINT32(clk_mask_arm.reg32, tegra_car),
        VMSTATE_UINT32(misc_clk_enb.reg32, tegra_car),
        VMSTATE_UINT32(clk_cpu_cmplx.reg32, tegra_car),
        VMSTATE_UINT32(osc_ctrl.reg32, tegra_car),
        VMSTATE_UINT32(pll_lfsr.reg32, tegra_car),
        VMSTATE_UINT32(osc_freq_det.reg32, tegra_car),
        VMSTATE_UINT32(osc_freq_det_status.reg32, tegra_car),
        VMSTATE_UINT32(plle_ss_cntl.reg32, tegra_car),
        VMSTATE_UINT32(plle_misc1.reg32, tegra_car),
        VMSTATE_UINT32(pllc_base.reg32, tegra_car),
        VMSTATE_UINT32(pllc_out.reg32, tegra_car),
        VMSTATE_UINT32(pllc_misc.reg32, tegra_car),
        VMSTATE_UINT32(pllc_misc_1.reg32, tegra_car),
        VMSTATE_UINT32(pllm_base.reg32, tegra_car),
        VMSTATE_UINT32(pllm_out.reg32, tegra_car),
        VMSTATE_UINT32(pllm_misc1.reg32, tegra_car),
        VMSTATE_UINT32(pllm_misc2.reg32, tegra_car),
        VMSTATE_UINT32(pllp_base.reg32, tegra_car),
        VMSTATE_UINT32(pllp_outa.reg32, tegra_car),
        VMSTATE_UINT32(pllp_outb.reg32, tegra_car),
        VMSTATE_UINT32(pllp_misc.reg32, tegra_car),
        VMSTATE_UINT32(plla_base.reg32, tegra_car),
        VMSTATE_UINT32(plla_out.reg32, tegra_car),
        VMSTATE_UINT32(plla_misc1.reg32, tegra_car),
        VMSTATE_UINT32(plla_misc.reg32, tegra_car),
        VMSTATE_UINT32(pllu_base.reg32, tegra_car),
        VMSTATE_UINT32(pllu_outa.reg32, tegra_car),
        VMSTATE_UINT32(pllu_misc1.reg32, tegra_car),
        VMSTATE_UINT32(pllu_misc.reg32, tegra_car),
        VMSTATE_UINT32(plld_base.reg32, tegra_car),
        VMSTATE_UINT32(plld_misc1.reg32, tegra_car),
        VMSTATE_UINT32(plld_misc.reg32, tegra_car),
        VMSTATE_UINT32(pllx_base.reg32, tegra_car),
        VMSTATE_UINT32(pllx_misc.reg32, tegra_car),
        VMSTATE_UINT32(plle_base.reg32, tegra_car),
        VMSTATE_UINT32(plle_misc.reg32, tegra_car),
        VMSTATE_UINT32(plle_ss_cntl1.reg32, tegra_car),
        VMSTATE_UINT32(plle_ss_cntl2.reg32, tegra_car),
        VMSTATE_UINT32(lvl2_clk_gate_ovra.reg32, tegra_car),
        VMSTATE_UINT32(lvl2_clk_gate_ovrb.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_i2s1.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_i2s2.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_spdif_out.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_spdif_in.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_pwm.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_spi1.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_spi22.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_spi3.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_xio.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_i2c1.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_dvc_i2c.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_twc.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_sbc1.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_disp1.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_disp2.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_cve.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_ide.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_vi.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_sdmmc1.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_sdmmc2.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_g3d.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_g2d.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_ndflash.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_sdmmc4.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_vfir.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_epp.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_mpe.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_mipi.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_uart1.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_uart2.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_host1x.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_tvo.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_hdmi.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_tvdac.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_i2c2.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_emc.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_uart3.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_vi_sensor.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_spi4.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_i2c3.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_sdmmc3.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_uart4.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_uart5.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_vde.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_owr.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_nor.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_csite.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_osc.reg32, tegra_car),
        VMSTATE_UINT32(rst_cpu_cmplx_set.reg32, tegra_car),
        VMSTATE_UINT32(utmip_pll_cfg0.reg32, tegra_car),
        VMSTATE_UINT32(rst_controller_pllc4_base.reg32, tegra_car),
        VMSTATE_UINT32(rst_controller_pllmb_base.reg32, tegra_car),
        VMSTATE_UINT32(clk_source_la.reg32, tegra_car),
        VMSTATE_UINT32_ARRAY(regs, tegra_car, 0x748>>2),
        VMSTATE_END_OF_LIST()
    }
};

int tegra_clk_enabled(int id)
{
    tegra_car *s = tegra_car_dev;
    int ret = 0;

    switch (id) {
    case 0 ... 31:
        ret = s->clk_out_enb_l.reg32 & (1 << id);
        break;
    case 32 ... 63:
        ret = s->clk_out_enb_h.reg32 & (1 << (id - 32));
        break;
    case 64 ... 95:
        ret = s->clk_out_enb_u.reg32 & (1 << (id - 64));
        break;
    default:
        g_assert_not_reached();
    }

    return !!ret;
}

int tegra_rst_asserted(int id)
{
    tegra_car *s = tegra_car_dev;
    int ret = 0;

    switch (id) {
    case 0 ... 31:
        ret = s->rst_devices_l.reg32 & (1 << id);
        break;
    case 32 ... 63:
        ret = s->rst_devices_h.reg32 & (1 << (id - 32));
        break;
    case 64 ... 95:
        ret = s->rst_devices_u.reg32 & (1 << (id - 64));
        break;
    default:
        g_assert_not_reached();
    }

    return !!ret;
}

static void set_rst_devices_l(uint32_t value)
{
    rst_dev_l_set_t rst = { .reg32 = value };

    if (rst.set_cop_rst) {
        tegra_cpu_reset_assert(TEGRA_BPMP);
    }

    if (rst.set_trig_sys_rst) {
        TPRINT("clk_rst reboot request!\n");
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }

    if (rst.set_tmr_rst) {
        TPRINT("car: resetting timers\n");
        tegra_device_reset( DEVICE(tegra_timer1_dev) );
        tegra_device_reset( DEVICE(tegra_timer2_dev) );
        tegra_device_reset( DEVICE(tegra_timer_us_dev) );
        tegra_device_reset( DEVICE(tegra_timer3_dev) );
        tegra_device_reset( DEVICE(tegra_timer4_dev) );
    }

    if (rst.set_gpio_rst) {
        TPRINT("car: resetting gpio\n");
        tegra_device_reset( DEVICE(tegra_gpios_dev) );
    }

    if (rst.set_vcp_rst) {
        remote_io_rst_set(TEGRA20_CLK_VCP, 1);
    }
}

static void clr_rst_devices_l(uint32_t value)
{
    rst_dev_l_clr_t rst = { .reg32 = value };

    if (rst.clr_cop_rst) {
        tegra_cpu_reset_deassert(TEGRA_BPMP, 0);
    }

    if (rst.clr_vcp_rst) {
        remote_io_rst_set(TEGRA20_CLK_VCP, 0);
    }
}

static void set_rst_devices_h(uint32_t value, uint32_t clk_enb)
{
    rst_dev_h_set_t rst = { .reg32 = value };
    clk_out_enb_h_t clk = { .reg32 = clk_enb };

    if (rst.set_bsea_rst & clk.clk_enb_bsea) {
        TPRINT("car: resetting BSEA\n");
        tegra_device_reset( DEVICE(tegra_bsea_dev) );
    }

    if (rst.set_bsea_rst) {
        remote_io_rst_set(TEGRA20_CLK_BSEA, 1);
    }

    if (rst.set_bsev_rst & clk.clk_enb_bsev) {
        TPRINT("car: resetting BSEV\n");
        tegra_device_reset( DEVICE(tegra_bsev_dev) );
    }

    if (rst.set_bsev_rst) {
        remote_io_rst_set(TEGRA20_CLK_BSEV, 1);
    }

    if (rst.set_vde_rst & clk.clk_enb_vde) {
        TPRINT("car: resetting VDE\n");
        tegra_device_reset( DEVICE(tegra_sxe_dev) );
        tegra_device_reset( DEVICE(tegra_bsev_dev) );
        tegra_device_reset( DEVICE(tegra_mbe_dev) );
        tegra_device_reset( DEVICE(tegra_ppe_dev) );
        tegra_device_reset( DEVICE(tegra_mce_dev) );
        tegra_device_reset( DEVICE(tegra_tfe_dev) );
        tegra_device_reset( DEVICE(tegra_ppb_dev) );
        tegra_device_reset( DEVICE(tegra_vdma_dev) );
        tegra_device_reset( DEVICE(tegra_ucq2_dev) );
        tegra_device_reset( DEVICE(tegra_bsea2_dev) );
        tegra_device_reset( DEVICE(tegra_frameid_dev) );
    }

    if (rst.set_vde_rst) {
        remote_io_rst_set(TEGRA20_CLK_VDE, 1);
    }

    if (rst.set_ahbdma_rst) {
        remote_io_rst_set(TEGRA20_CLK_AHBDMA, 1);
    }
}

static void clr_rst_devices_h(uint32_t value, uint32_t clk_enb)
{
    rst_dev_h_clr_t rst = { .reg32 = value };

    if (rst.clr_bsea_rst) {
        remote_io_rst_set(TEGRA20_CLK_BSEA, 0);
    }

    if (rst.clr_bsev_rst) {
        remote_io_rst_set(TEGRA20_CLK_BSEV, 0);
    }

    if (rst.clr_vde_rst) {
        remote_io_rst_set(TEGRA20_CLK_VDE, 0);
    }

    if (rst.clr_ahbdma_rst) {
        remote_io_rst_set(TEGRA20_CLK_AHBDMA, 0);
    }
}

static void set_rst_devices_u(uint32_t value)
{
}

static void clr_rst_devices_u(uint32_t value)
{
}

static uint64_t tegra_car_priv_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    tegra_car *s = opaque;
    uint64_t ret = 0;

    switch (offset) {
    case RST_SOURCE_OFFSET:
        ret = s->rst_source.reg32;
        break;
    case RST_DEV_L_SET_OFFSET:
    case RST_DEV_L_CLR_OFFSET:
    case RST_DEVICES_L_OFFSET:
        ret = s->rst_devices_l.reg32;
        break;
    case RST_DEV_H_SET_OFFSET:
    case RST_DEV_H_CLR_OFFSET:
    case RST_DEVICES_H_OFFSET:
        ret = s->rst_devices_h.reg32;
        break;
    case RST_DEV_U_SET_OFFSET:
    case RST_DEV_U_CLR_OFFSET:
    case RST_DEVICES_U_OFFSET:
        ret = s->rst_devices_u.reg32;
        break;
    case CLK_ENB_L_SET_OFFSET:
    case CLK_ENB_L_CLR_OFFSET:
    case CLK_OUT_ENB_L_OFFSET:
        ret = s->clk_out_enb_l.reg32;
        break;
    case CLK_ENB_H_SET_OFFSET:
    case CLK_ENB_H_CLR_OFFSET:
    case CLK_OUT_ENB_H_OFFSET:
        ret = s->clk_out_enb_h.reg32;
        break;
    case CLK_ENB_U_SET_OFFSET:
    case CLK_ENB_U_CLR_OFFSET:
    case CLK_OUT_ENB_U_OFFSET:
        ret = s->clk_out_enb_u.reg32;
        break;
    case CCLK_BURST_POLICY_OFFSET:
        ret = s->cclk_burst_policy.reg32;
        break;
    case SUPER_CCLK_DIVIDER_OFFSET:
        ret = s->super_cclk_divider.reg32;
        break;
    case SCLK_BURST_POLICY_OFFSET:
        ret = s->sclk_burst_policy.reg32;
        break;
    case SUPER_SCLK_DIVIDER_OFFSET:
        ret = s->super_sclk_divider.reg32;
        break;
    case CLK_SYSTEM_RATE_OFFSET:
        ret = s->clk_system_rate.reg32;
        break;
    case PROG_DLY_CLK_OFFSET:
        ret = s->prog_dly_clk.reg32;
        break;
    case AUDIO_SYNC_CLK_RATE_OFFSET:
        ret = s->audio_sync_clk_rate.reg32;
        break;
    case COP_CLK_SKIP_POLICY_OFFSET:
        ret = s->cop_clk_skip_policy.reg32;
        break;
    case CLK_MASK_ARM_OFFSET:
        ret = s->clk_mask_arm.reg32;
        break;
    case MISC_CLK_ENB_OFFSET:
        ret = s->misc_clk_enb.reg32;
        break;
    case CLK_CPU_CMPLX_OFFSET:
        ret = s->clk_cpu_cmplx.reg32;
        break;
    case OSC_CTRL_OFFSET:
        ret = s->osc_ctrl.reg32;
        break;
    case PLL_LFSR_OFFSET:
        ret = s->pll_lfsr.reg32;
        break;
    case OSC_FREQ_DET_OFFSET:
        ret = s->osc_freq_det.reg32;
        break;
    case OSC_FREQ_DET_STATUS_OFFSET:
        ret = s->osc_freq_det_status.reg32;
        break;
    case PLLE_SS_CNTL_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->plle_ss_cntl.reg32;
        break;
    case PLLE_MISC1_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->plle_misc1.reg32;
        break;
    case PLLC_BASE_OFFSET:
        ret = s->pllc_base.reg32;
        break;
    case PLLC_OUT_OFFSET:
        ret = s->pllc_out.reg32;
        break;
    case PLLC_MISC_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->pllc_misc.reg32;
        break;
    case PLLC_MISC_1_OFFSET:
        ret = s->pllc_misc_1.reg32;
        break;
    case PLLM_BASE_OFFSET:
        ret = s->pllm_base.reg32;
        break;
    case PLLM_OUT_OFFSET:
        ret = s->pllm_out.reg32;
        break;
    case PLLM_MISC1_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->pllm_misc1.reg32;
        break;
    case PLLM_MISC2_OFFSET:
        ret = s->pllm_misc2.reg32;
        break;
    case PLLP_BASE_OFFSET:
        ret = s->pllp_base.reg32;
        break;
    case PLLP_OUTA_OFFSET:
        ret = s->pllp_outa.reg32;
        break;
    case PLLP_OUTB_OFFSET:
        ret = s->pllp_outb.reg32;
        break;
    case PLLP_MISC_OFFSET:
        ret = s->pllp_misc.reg32;
        break;
    case PLLA_BASE_OFFSET:
        ret = s->plla_base.reg32;
        break;
    case PLLA_OUT_OFFSET:
        ret = s->plla_out.reg32;
        break;
    case PLLA_MISC1_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->plla_misc1.reg32;
        break;
    case PLLA_MISC_OFFSET:
        ret = s->plla_misc.reg32;
        break;
    case PLLU_BASE_OFFSET:
        ret = s->pllu_base.reg32;
        break;
    case PLLU_OUTA_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->pllu_outa.reg32;
        break;
    case PLLU_MISC1_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->pllu_misc1.reg32;
        break;
    case PLLU_MISC_OFFSET:
        ret = s->pllu_misc.reg32;
        break;
    case PLLD_BASE_OFFSET:
        ret = s->plld_base.reg32;
        break;
    case PLLD_MISC1_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->plld_misc1.reg32;
        break;
    case PLLD_MISC_OFFSET:
        ret = s->plld_misc.reg32;
        break;
    case PLLX_BASE_OFFSET:
        ret = s->pllx_base.reg32;
        break;
    case PLLX_MISC_OFFSET:
        ret = s->pllx_misc.reg32;
        break;
    case PLLE_BASE_OFFSET:
        ret = s->plle_base.reg32;
        break;
    case PLLE_MISC_OFFSET:
        ret = s->plle_misc.reg32;
        break;
    case PLLE_SS_CNTL1_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->plle_ss_cntl1.reg32;
        break;
    case PLLE_SS_CNTL2_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->plle_ss_cntl2.reg32;
        break;
    case LVL2_CLK_GATE_OVRA_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->lvl2_clk_gate_ovra.reg32;
        break;
    case LVL2_CLK_GATE_OVRB_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->lvl2_clk_gate_ovrb.reg32;
        break;
    case CLK_SOURCE_I2S1_OFFSET:
        ret = s->clk_source_i2s1.reg32;
        break;
    case CLK_SOURCE_I2S2_OFFSET:
        ret = s->clk_source_i2s2.reg32;
        break;
    case CLK_SOURCE_SPDIF_OUT_OFFSET:
        ret = s->clk_source_spdif_out.reg32;
        break;
    case CLK_SOURCE_SPDIF_IN_OFFSET:
        ret = s->clk_source_spdif_in.reg32;
        break;
    case CLK_SOURCE_PWM_OFFSET:
        ret = s->clk_source_pwm.reg32;
        break;
    case CLK_SOURCE_SPI1_OFFSET:
        ret = s->clk_source_spi1.reg32;
        break;
    case CLK_SOURCE_SPI22_OFFSET:
        ret = s->clk_source_spi22.reg32;
        break;
    case CLK_SOURCE_SPI3_OFFSET:
        ret = s->clk_source_spi3.reg32;
        break;
    case CLK_SOURCE_XIO_OFFSET:
        ret = s->clk_source_xio.reg32;
        break;
    case CLK_SOURCE_I2C1_OFFSET:
        ret = s->clk_source_i2c1.reg32;
        break;
    case CLK_SOURCE_DVC_I2C_OFFSET:
        ret = s->clk_source_dvc_i2c.reg32;
        break;
    case CLK_SOURCE_TWC_OFFSET:
        ret = s->clk_source_twc.reg32;
        break;
    case CLK_SOURCE_SBC1_OFFSET:
        ret = s->clk_source_sbc1.reg32;
        break;
    case CLK_SOURCE_DISP1_OFFSET:
        ret = s->clk_source_disp1.reg32;
        break;
    case CLK_SOURCE_DISP2_OFFSET:
        ret = s->clk_source_disp2.reg32;
        break;
    case CLK_SOURCE_CVE_OFFSET:
        ret = s->clk_source_cve.reg32;
        break;
    case CLK_SOURCE_IDE_OFFSET:
        ret = s->clk_source_ide.reg32;
        break;
    case CLK_SOURCE_VI_OFFSET:
        ret = s->clk_source_vi.reg32;
        break;
    case CLK_SOURCE_SDMMC1_OFFSET:
        ret = s->clk_source_sdmmc1.reg32;
        break;
    case CLK_SOURCE_SDMMC2_OFFSET:
        ret = s->clk_source_sdmmc2.reg32;
        break;
    case CLK_SOURCE_G3D_OFFSET:
        ret = s->clk_source_g3d.reg32;
        break;
    case CLK_SOURCE_G2D_OFFSET:
        ret = s->clk_source_g2d.reg32;
        break;
    case CLK_SOURCE_NDFLASH_OFFSET:
        ret = s->clk_source_ndflash.reg32;
        break;
    case CLK_SOURCE_SDMMC4_OFFSET:
        ret = s->clk_source_sdmmc4.reg32;
        break;
    case CLK_SOURCE_VFIR_OFFSET:
        ret = s->clk_source_vfir.reg32;
        break;
    case CLK_SOURCE_EPP_OFFSET:
        ret = s->clk_source_epp.reg32;
        break;
    case CLK_SOURCE_MPE_OFFSET:
        ret = s->clk_source_mpe.reg32;
        break;
    case CLK_SOURCE_MIPI_OFFSET:
        ret = s->clk_source_mipi.reg32;
        break;
    case CLK_SOURCE_UART1_OFFSET:
        ret = s->clk_source_uart1.reg32;
        break;
    case CLK_SOURCE_UART2_OFFSET:
        ret = s->clk_source_uart2.reg32;
        break;
    case CLK_SOURCE_HOST1X_OFFSET:
        ret = s->clk_source_host1x.reg32;
        break;
    case CLK_SOURCE_TVO_OFFSET:
        ret = s->clk_source_tvo.reg32;
        break;
    case CLK_SOURCE_HDMI_OFFSET:
        ret = s->clk_source_hdmi.reg32;
        break;
    case CLK_SOURCE_TVDAC_OFFSET:
        ret = s->clk_source_tvdac.reg32;
        break;
    case CLK_SOURCE_I2C2_OFFSET:
        ret = s->clk_source_i2c2.reg32;
        break;
    case CLK_SOURCE_EMC_OFFSET:
        ret = s->clk_source_emc.reg32;
        break;
    case CLK_SOURCE_UART3_OFFSET:
        ret = s->clk_source_uart3.reg32;
        break;
    case CLK_SOURCE_VI_SENSOR_OFFSET:
        ret = s->clk_source_vi_sensor.reg32;
        break;
    case CLK_SOURCE_SPI4_OFFSET:
        ret = s->clk_source_spi4.reg32;
        break;
    case CLK_SOURCE_I2C3_OFFSET:
        ret = s->clk_source_i2c3.reg32;
        break;
    case CLK_SOURCE_SDMMC3_OFFSET:
        ret = s->clk_source_sdmmc3.reg32;
        break;
    case CLK_SOURCE_UART4_OFFSET:
        ret = s->clk_source_uart4.reg32;
        break;
    case CLK_SOURCE_UART5_OFFSET:
        ret = s->clk_source_uart5.reg32;
        break;
    case CLK_SOURCE_VDE_OFFSET:
        ret = s->clk_source_vde.reg32;
        break;
    case CLK_SOURCE_OWR_OFFSET:
        ret = s->clk_source_owr.reg32;
        break;
    case CLK_SOURCE_NOR_OFFSET:
        ret = s->clk_source_nor.reg32;
        break;
    case CLK_SOURCE_CSITE_OFFSET:
        ret = s->clk_source_csite.reg32;
        break;
    case CLK_SOURCE_OSC_OFFSET:
        ret = s->clk_source_osc.reg32;
        break;
    case RST_CPU_CMPLX_SET_OFFSET:
        ret = s->rst_cpu_cmplx_set.reg32;
        break;
    case UTMIP_PLL_CFG0_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->utmip_pll_cfg0.reg32;
        break;
    case RST_CONTROLLER_PLLC4_BASE_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->rst_controller_pllc4_base.reg32;
        break;
    case RST_CONTROLLER_PLLMB_BASE_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->rst_controller_pllmb_base.reg32;
        break;
    case CLK_SOURCE_LA_OFFSET:
        ret = s->clk_source_la.reg32;
        break;
    default:
        if (offset == s->rst_cpu_cmplx_clr_offset) ret = s->rst_cpu_cmplx_set.reg32;
        else if (tegra_board >= TEGRAX1_BOARD && offset <= sizeof(s->regs)-4) ret = s->regs[offset>>2];
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_car_priv_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    tegra_car *s = opaque;

    switch (offset) {
    case RST_SOURCE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rst_source.reg32, value & RST_SOURCE_WRMASK);
        WR_MASKED(s->rst_source.reg32, value, RST_SOURCE);
        break;
    case RST_DEVICES_L_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rst_devices_l.reg32, value);

        set_rst_devices_l(((s->rst_devices_l.reg32 ^ value) & value) & (s->clk_out_enb_l.reg32 | 6));
        clr_rst_devices_l(((s->rst_devices_l.reg32 ^ value) & value) & (s->clk_out_enb_l.reg32 | 6));
        s->rst_devices_l.reg32 = value;
        break;
    case RST_DEVICES_H_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rst_devices_h.reg32, value);

        set_rst_devices_h(((s->rst_devices_h.reg32 ^ value) & value), s->clk_out_enb_h.reg32);
        clr_rst_devices_h(((s->rst_devices_h.reg32 ^ value) & value), s->clk_out_enb_h.reg32);
        s->rst_devices_h.reg32 = value;
        break;
    case RST_DEVICES_U_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rst_devices_u.reg32, value);

        set_rst_devices_u(((s->rst_devices_u.reg32 ^ value) & value) & s->clk_out_enb_u.reg32);
        clr_rst_devices_u(((s->rst_devices_u.reg32 ^ value) & value) & s->clk_out_enb_u.reg32);
        s->rst_devices_u.reg32 = value;
        break;
    case CLK_OUT_ENB_L_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_out_enb_l.reg32, value);
        s->clk_out_enb_l.reg32 = value;

        remote_io_clk_set(TEGRA20_CLK_VCP, s->clk_out_enb_l.clk_enb_vcp);

        set_rst_devices_l(((s->clk_out_enb_l.reg32 & (value | 6)) ^ value) & s->rst_devices_l.reg32);
        break;
    case CLK_OUT_ENB_H_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_out_enb_h.reg32, value);
        s->clk_out_enb_h.reg32 = value;

        remote_io_clk_set(TEGRA20_CLK_AHBDMA, s->clk_out_enb_h.clk_enb_ahbdma);
        remote_io_clk_set(TEGRA20_CLK_BSEA, s->clk_out_enb_h.clk_enb_bsea);
        remote_io_clk_set(TEGRA20_CLK_BSEV, s->clk_out_enb_h.clk_enb_bsev);
        remote_io_clk_set(TEGRA20_CLK_VDE, s->clk_out_enb_h.clk_enb_vde);

        set_rst_devices_h(((s->clk_out_enb_h.reg32 & value) ^ value), s->rst_devices_h.reg32);
        break;
    case CLK_OUT_ENB_U_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_out_enb_u.reg32, value);
        s->clk_out_enb_u.reg32 = value;

        set_rst_devices_u(((s->clk_out_enb_u.reg32 & value) ^ value) & s->rst_devices_u.reg32);
        break;
    case CCLK_BURST_POLICY_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->cclk_burst_policy.reg32, value);
        s->cclk_burst_policy.reg32 = value;
        break;
    case SUPER_CCLK_DIVIDER_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->super_cclk_divider.reg32, value);
        s->super_cclk_divider.reg32 = value;
        break;
    case SCLK_BURST_POLICY_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->sclk_burst_policy.reg32, value);
        s->sclk_burst_policy.reg32 = value;
        break;
    case SUPER_SCLK_DIVIDER_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->super_sclk_divider.reg32, value);
        s->super_sclk_divider.reg32 = value;
        break;
    case CLK_SYSTEM_RATE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_system_rate.reg32, value);
        s->clk_system_rate.reg32 = value;
        break;
    case PROG_DLY_CLK_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->prog_dly_clk.reg32, value);
        s->prog_dly_clk.reg32 = value;
        break;
    case AUDIO_SYNC_CLK_RATE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->audio_sync_clk_rate.reg32, value);
        s->audio_sync_clk_rate.reg32 = value;
        break;
    case COP_CLK_SKIP_POLICY_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->cop_clk_skip_policy.reg32, value);
        s->cop_clk_skip_policy.reg32 = value;
        break;
    case CLK_MASK_ARM_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_mask_arm.reg32, value & CLK_MASK_ARM_WRMASK);
        WR_MASKED(s->clk_mask_arm.reg32, value, CLK_MASK_ARM);
        break;
    case MISC_CLK_ENB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->misc_clk_enb.reg32, value);
        s->misc_clk_enb.reg32 = value;
        break;
    case CLK_CPU_CMPLX_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_cpu_cmplx.reg32, value);
        s->clk_cpu_cmplx.reg32 = value;
        break;
    case OSC_CTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->osc_ctrl.reg32, value);
        s->osc_ctrl.reg32 = value;
        break;
    case OSC_FREQ_DET_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->osc_freq_det.reg32, value);
        s->osc_freq_det.reg32 = value;
        break;
    case PLLE_SS_CNTL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->plle_ss_cntl.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->plle_ss_cntl.reg32 = value;
        break;
    case PLLE_MISC1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->plle_misc1.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->plle_misc1.reg32 = value;
        break;
    case PLLC_BASE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllc_base.reg32, value & PLLC_BASE_WRMASK);
        WR_MASKED(s->pllc_base.reg32, value, PLLC_BASE);
        break;
    case PLLC_OUT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllc_out.reg32, value);
        s->pllc_out.reg32 = value;
        break;
    case PLLC_MISC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllc_misc.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->pllc_misc.reg32 = value;
        break;
    case PLLC_MISC_1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllc_misc_1.reg32, value);
        s->pllc_misc_1.reg32 = value;
        break;
    case PLLM_BASE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllm_base.reg32, value & PLLM_BASE_WRMASK);
        WR_MASKED(s->pllm_base.reg32, value, PLLM_BASE);
        break;
    case PLLM_OUT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllm_out.reg32, value);
        s->pllm_out.reg32 = value;
        break;
    case PLLM_MISC1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllm_misc1.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->pllm_misc1.reg32 = value;
        break;
    case PLLM_MISC2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllm_misc2.reg32, value);
        s->pllm_misc2.reg32 = value;
        break;
    case PLLP_BASE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllp_base.reg32, value & PLLP_BASE_WRMASK);
        WR_MASKED(s->pllp_base.reg32, value, PLLP_BASE);
        break;
    case PLLP_OUTA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllp_outa.reg32, value);
        s->pllp_outa.reg32 = value;
        break;
    case PLLP_OUTB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllp_outb.reg32, value);
        s->pllp_outb.reg32 = value;
        break;
    case PLLP_MISC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllp_misc.reg32, value);
        s->pllp_misc.reg32 = value;
        break;
    case PLLA_BASE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->plla_base.reg32, value & PLLA_BASE_WRMASK);
        WR_MASKED(s->plla_base.reg32, value, PLLA_BASE);
        break;
    case PLLA_OUT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->plla_out.reg32, value);
        s->plla_out.reg32 = value;
        break;
    case PLLA_MISC1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->plla_misc1.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->plla_misc1.reg32 = value;
        break;
    case PLLA_MISC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->plla_misc.reg32, value);
        s->plla_misc.reg32 = value;
        break;
    case PLLU_BASE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllu_base.reg32, value & PLLU_BASE_WRMASK);
        WR_MASKED(s->pllu_base.reg32, value, PLLU_BASE);
        break;
    case PLLU_OUTA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllu_outa.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->pllu_outa.reg32 = value;
        break;
    case PLLU_MISC1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllu_misc1.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->pllu_misc1.reg32 = value;
        break;
    case PLLU_MISC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllu_misc.reg32, value);
        s->pllu_misc.reg32 = value;
        break;
    case PLLD_BASE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->plld_base.reg32, value & PLLD_BASE_WRMASK);
        WR_MASKED(s->plld_base.reg32, value, PLLD_BASE);
        break;
    case PLLD_MISC1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->plld_misc1.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->plld_misc1.reg32 = value;
        break;
    case PLLD_MISC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->plld_misc.reg32, value);
        s->plld_misc.reg32 = value;
        break;
    case PLLX_BASE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllx_base.reg32, value & PLLX_BASE_WRMASK);
        WR_MASKED(s->pllx_base.reg32, value, PLLX_BASE);
        break;
    case PLLX_MISC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllx_misc.reg32, value);
        s->pllx_misc.reg32 = value;
        break;
    case PLLE_BASE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->plle_base.reg32, value);
        s->plle_base.reg32 = value;
        break;
    case PLLE_MISC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->plle_misc.reg32, value & PLLE_MISC_WRMASK);
        WR_MASKED(s->plle_misc.reg32, value, PLLE_MISC);
        break;
    case PLLE_SS_CNTL1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->plle_ss_cntl1.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->plle_ss_cntl1.reg32 = value;
        break;
    case PLLE_SS_CNTL2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->plle_ss_cntl2.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->plle_ss_cntl2.reg32 = value;
        break;
    case LVL2_CLK_GATE_OVRA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->lvl2_clk_gate_ovra.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->lvl2_clk_gate_ovra.reg32 = value;
        break;
    case LVL2_CLK_GATE_OVRB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->lvl2_clk_gate_ovrb.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->lvl2_clk_gate_ovrb.reg32 = value;
        break;
    case CLK_SOURCE_I2S1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_i2s1.reg32, value);
        s->clk_source_i2s1.reg32 = value;
        break;
    case CLK_SOURCE_I2S2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_i2s2.reg32, value);
        s->clk_source_i2s2.reg32 = value;
        break;
    case CLK_SOURCE_SPDIF_OUT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_spdif_out.reg32, value);
        s->clk_source_spdif_out.reg32 = value;
        break;
    case CLK_SOURCE_SPDIF_IN_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_spdif_in.reg32, value);
        s->clk_source_spdif_in.reg32 = value;
        break;
    case CLK_SOURCE_PWM_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_pwm.reg32, value);
        s->clk_source_pwm.reg32 = value;
        break;
    case CLK_SOURCE_SPI1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_spi1.reg32, value);
        s->clk_source_spi1.reg32 = value;
        break;
    case CLK_SOURCE_SPI22_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_spi22.reg32, value);
        s->clk_source_spi22.reg32 = value;
        break;
    case CLK_SOURCE_SPI3_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_spi3.reg32, value);
        s->clk_source_spi3.reg32 = value;
        break;
    case CLK_SOURCE_XIO_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_xio.reg32, value);
        s->clk_source_xio.reg32 = value;
        break;
    case CLK_SOURCE_I2C1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_i2c1.reg32, value);
        s->clk_source_i2c1.reg32 = value;
        break;
    case CLK_SOURCE_DVC_I2C_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_dvc_i2c.reg32, value);
        s->clk_source_dvc_i2c.reg32 = value;
        break;
    case CLK_SOURCE_TWC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_twc.reg32, value);
        s->clk_source_twc.reg32 = value;
        break;
    case CLK_SOURCE_SBC1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_sbc1.reg32, value);
        s->clk_source_sbc1.reg32 = value;
        break;
    case CLK_SOURCE_DISP1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_disp1.reg32, value);
        s->clk_source_disp1.reg32 = value;
        break;
    case CLK_SOURCE_DISP2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_disp2.reg32, value);
        s->clk_source_disp2.reg32 = value;
        break;
    case CLK_SOURCE_CVE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_cve.reg32, value);
        s->clk_source_cve.reg32 = value;
        break;
    case CLK_SOURCE_IDE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_ide.reg32, value);
        s->clk_source_ide.reg32 = value;
        break;
    case CLK_SOURCE_VI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_vi.reg32, value);
        s->clk_source_vi.reg32 = value;
        break;
    case CLK_SOURCE_SDMMC1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_sdmmc1.reg32, value);
        s->clk_source_sdmmc1.reg32 = value;
        break;
    case CLK_SOURCE_SDMMC2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_sdmmc2.reg32, value);
        s->clk_source_sdmmc2.reg32 = value;
        break;
    case CLK_SOURCE_G3D_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_g3d.reg32, value);
        s->clk_source_g3d.reg32 = value;
        break;
    case CLK_SOURCE_G2D_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_g2d.reg32, value);
        s->clk_source_g2d.reg32 = value;
        break;
    case CLK_SOURCE_NDFLASH_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_ndflash.reg32, value);
        s->clk_source_ndflash.reg32 = value;
        break;
    case CLK_SOURCE_SDMMC4_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_sdmmc4.reg32, value);
        s->clk_source_sdmmc4.reg32 = value;
        break;
    case CLK_SOURCE_VFIR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_vfir.reg32, value);
        s->clk_source_vfir.reg32 = value;
        break;
    case CLK_SOURCE_EPP_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_epp.reg32, value);
        s->clk_source_epp.reg32 = value;
        break;
    case CLK_SOURCE_MPE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_mpe.reg32, value);
        s->clk_source_mpe.reg32 = value;
        break;
    case CLK_SOURCE_MIPI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_mipi.reg32, value);
        s->clk_source_mipi.reg32 = value;
        break;
    case CLK_SOURCE_UART1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_uart1.reg32, value);
        s->clk_source_uart1.reg32 = value;
        break;
    case CLK_SOURCE_UART2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_uart2.reg32, value);
        s->clk_source_uart2.reg32 = value;
        break;
    case CLK_SOURCE_HOST1X_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_host1x.reg32, value);
        s->clk_source_host1x.reg32 = value;
        break;
    case CLK_SOURCE_TVO_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_tvo.reg32, value);
        s->clk_source_tvo.reg32 = value;
        break;
    case CLK_SOURCE_HDMI_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_hdmi.reg32, value);
        s->clk_source_hdmi.reg32 = value;
        break;
    case CLK_SOURCE_TVDAC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_tvdac.reg32, value);
        s->clk_source_tvdac.reg32 = value;
        break;
    case CLK_SOURCE_I2C2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_i2c2.reg32, value);
        s->clk_source_i2c2.reg32 = value;
        break;
    case CLK_SOURCE_EMC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_emc.reg32, value);
        s->clk_source_emc.reg32 = value;
        break;
    case CLK_SOURCE_UART3_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_uart3.reg32, value);
        s->clk_source_uart3.reg32 = value;
        break;
    case CLK_SOURCE_VI_SENSOR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_vi_sensor.reg32, value);
        s->clk_source_vi_sensor.reg32 = value;
        break;
    case CLK_SOURCE_SPI4_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_spi4.reg32, value);
        s->clk_source_spi4.reg32 = value;
        break;
    case CLK_SOURCE_I2C3_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_i2c3.reg32, value);
        s->clk_source_i2c3.reg32 = value;
        break;
    case CLK_SOURCE_SDMMC3_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_sdmmc3.reg32, value);
        s->clk_source_sdmmc3.reg32 = value;
        break;
    case CLK_SOURCE_UART4_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_uart4.reg32, value);
        s->clk_source_uart4.reg32 = value;
        break;
    case CLK_SOURCE_UART5_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_uart5.reg32, value);
        s->clk_source_uart5.reg32 = value;
        break;
    case CLK_SOURCE_VDE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_vde.reg32, value);
        s->clk_source_vde.reg32 = value;
        break;
    case CLK_SOURCE_OWR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_owr.reg32, value);
        s->clk_source_owr.reg32 = value;
        break;
    case CLK_SOURCE_NOR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_nor.reg32, value);
        s->clk_source_nor.reg32 = value;
        break;
    case CLK_SOURCE_CSITE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_csite.reg32, value);
        s->clk_source_csite.reg32 = value;
        break;
    case CLK_SOURCE_OSC_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_osc.reg32, value);
        s->clk_source_osc.reg32 = value;
        break;
    case RST_DEV_L_SET_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rst_devices_l.reg32, value);

        set_rst_devices_l(((s->rst_devices_l.reg32 & value) ^ value) & (s->clk_out_enb_l.reg32 | 6));
        s->rst_devices_l.reg32 |= value;
        break;
    case RST_DEV_L_CLR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rst_devices_l.reg32, value);

        clr_rst_devices_l(s->rst_devices_l.reg32 & value & (s->clk_out_enb_l.reg32 | 6));
        s->rst_devices_l.reg32 &= ~value;
        break;
    case RST_DEV_H_SET_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rst_devices_h.reg32, value);

        set_rst_devices_h(((s->rst_devices_h.reg32 & value) ^ value), s->clk_out_enb_h.reg32);
        s->rst_devices_h.reg32 |= value;
        break;
    case RST_DEV_H_CLR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rst_devices_h.reg32, value);

        clr_rst_devices_h(s->rst_devices_h.reg32 & value, s->clk_out_enb_h.reg32);
        s->rst_devices_h.reg32 &= ~value;
        break;
    case RST_DEV_U_SET_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rst_devices_u.reg32, value);

        set_rst_devices_u(((s->rst_devices_u.reg32 & value) ^ value) & s->clk_out_enb_u.reg32);
        s->rst_devices_u.reg32 |= value;
        break;
    case RST_DEV_U_CLR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rst_devices_u.reg32, value);

        clr_rst_devices_u(s->rst_devices_u.reg32 & value & s->clk_out_enb_u.reg32);
        s->rst_devices_u.reg32 &= ~value;
        break;
    case CLK_ENB_L_SET_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_out_enb_l.reg32, value);

        set_rst_devices_l(((s->clk_out_enb_l.reg32 & (value | 6)) ^ value) & s->rst_devices_l.reg32);
        s->clk_out_enb_l.reg32 |= value;

        remote_io_clk_set(TEGRA20_CLK_VCP, s->clk_out_enb_l.clk_enb_vcp);
        break;
    case CLK_ENB_L_CLR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_out_enb_l.reg32, value);
        s->clk_out_enb_l.reg32 &= ~value;

        remote_io_clk_set(TEGRA20_CLK_VCP, s->clk_out_enb_l.clk_enb_vcp);
        break;
    case CLK_ENB_H_SET_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_out_enb_h.reg32, value);

        set_rst_devices_h(((s->clk_out_enb_h.reg32 & value) ^ value), s->rst_devices_h.reg32);
        s->clk_out_enb_h.reg32 |= value;

        remote_io_clk_set(TEGRA20_CLK_AHBDMA, s->clk_out_enb_h.clk_enb_ahbdma);
        remote_io_clk_set(TEGRA20_CLK_BSEA, s->clk_out_enb_h.clk_enb_bsea);
        remote_io_clk_set(TEGRA20_CLK_BSEV, s->clk_out_enb_h.clk_enb_bsev);
        remote_io_clk_set(TEGRA20_CLK_VDE, s->clk_out_enb_h.clk_enb_vde);
        break;
    case CLK_ENB_H_CLR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_out_enb_h.reg32, value);
        s->clk_out_enb_h.reg32 &= ~value;

        remote_io_clk_set(TEGRA20_CLK_AHBDMA, s->clk_out_enb_h.clk_enb_ahbdma);
        remote_io_clk_set(TEGRA20_CLK_BSEA, s->clk_out_enb_h.clk_enb_bsea);
        remote_io_clk_set(TEGRA20_CLK_BSEV, s->clk_out_enb_h.clk_enb_bsev);
        remote_io_clk_set(TEGRA20_CLK_VDE, s->clk_out_enb_h.clk_enb_vde);
        break;
    case CLK_ENB_U_SET_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_out_enb_u.reg32, value);

        set_rst_devices_u(((s->clk_out_enb_u.reg32 & value) ^ value) & s->rst_devices_u.reg32);
        s->clk_out_enb_u.reg32 |= value;
        break;
    case CLK_ENB_U_CLR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_out_enb_u.reg32, value);
        s->clk_out_enb_u.reg32 &= ~value;
        break;
    case RST_CPU_CMPLX_SET_OFFSET:
    {
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        rst_cpu_cmplx_set_t rst = { .reg32 = value };

        s->rst_cpu_cmplx_set.reg32 |= value;

        if (rst.set_cpureset0) {
            tegra_cpu_reset_assert(TEGRA_CCPLEX_CORE0);
        }

        if (rst.set_cpureset1) {
            tegra_cpu_reset_assert(TEGRA_CCPLEX_CORE1);
        }

        if (tegra_board >= TEGRAX1_BOARD) {
            if (rst.set_cpureset2) {
                tegra_cpu_reset_assert(TEGRA_CCPLEX_CORE2);
            }

            if (rst.set_cpureset3) {
                tegra_cpu_reset_assert(TEGRA_CCPLEX_CORE3);
            }
        }
        break;
    }
    case UTMIP_PLL_CFG0_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->utmip_pll_cfg0.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->utmip_pll_cfg0.reg32 = value;
        break;
    case RST_CONTROLLER_PLLC4_BASE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rst_controller_pllc4_base.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->rst_controller_pllc4_base.reg32 = value;
        break;
    case RST_CONTROLLER_PLLMB_BASE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rst_controller_pllmb_base.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD)  {
            value &= ~(3<<26);
            value |= s->rst_controller_pllmb_base.reg32 & (3<<26);
            s->rst_controller_pllmb_base.reg32 = value;
        }
        break;
    case CLK_SOURCE_LA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clk_source_la.reg32, value);
        s->clk_source_la.reg32 = value;
        break;
    default:
        if (offset != s->rst_cpu_cmplx_clr_offset && offset <= sizeof(s->regs)-4) {
            TRACE_WRITE(s->iomem.addr, offset, s->regs[offset>>2], value);
            s->regs[offset>>2] = value;
        }
        else TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }

    if (offset == s->rst_cpu_cmplx_clr_offset) {
        rst_cpu_cmplx_clr_t rst = { .reg32 = value };

        s->rst_cpu_cmplx_set.reg32 &= ~value;

        if (rst.clr_cpureset0) {
            tegra_cpu_reset_deassert(TEGRA_CCPLEX_CORE0, 0);
        }

        if (rst.clr_cpureset1) {
            tegra_cpu_reset_deassert(TEGRA_CCPLEX_CORE1, 0);
        }

        if (tegra_board >= TEGRAX1_BOARD) {
            if (rst.clr_cpureset2) {
                tegra_cpu_reset_deassert(TEGRA_CCPLEX_CORE2, 0);
            }

            if (rst.clr_cpureset3) {
                tegra_cpu_reset_deassert(TEGRA_CCPLEX_CORE3, 0);
            }
        }
    }
}

void tegra_car_orr_reg(void *opaque, hwaddr offset,
                       int64_t value, unsigned size)
{
    tegra_car_priv_write(opaque, offset, tegra_car_priv_read(opaque, offset, 4) | value, 4);
}

static void tegra_car_priv_reset(DeviceState *dev)
{
    tegra_car *s = TEGRA_CAR(dev);

    s->rst_source.reg32 = RST_SOURCE_RESET;
    s->rst_devices_l.reg32 = RST_DEVICES_L_RESET;
    s->rst_devices_h.reg32 = RST_DEVICES_H_RESET;
    s->rst_devices_u.reg32 = RST_DEVICES_U_RESET;
    s->clk_out_enb_l.reg32 = CLK_OUT_ENB_L_RESET;
    s->clk_out_enb_h.reg32 = CLK_OUT_ENB_H_RESET;
    s->clk_out_enb_u.reg32 = CLK_OUT_ENB_U_RESET;
    s->cclk_burst_policy.reg32 = CCLK_BURST_POLICY_RESET;
    s->super_cclk_divider.reg32 = SUPER_CCLK_DIVIDER_RESET;
    s->sclk_burst_policy.reg32 = SCLK_BURST_POLICY_RESET;
    s->super_sclk_divider.reg32 = SUPER_SCLK_DIVIDER_RESET;
    s->clk_system_rate.reg32 = CLK_SYSTEM_RATE_RESET;
    s->prog_dly_clk.reg32 = PROG_DLY_CLK_RESET;
    s->audio_sync_clk_rate.reg32 = AUDIO_SYNC_CLK_RATE_RESET;
    s->cop_clk_skip_policy.reg32 = COP_CLK_SKIP_POLICY_RESET;
    s->clk_mask_arm.reg32 = CLK_MASK_ARM_RESET;
    s->misc_clk_enb.reg32 = MISC_CLK_ENB_RESET;
    s->clk_cpu_cmplx.reg32 = CLK_CPU_CMPLX_RESET;
    s->osc_ctrl.reg32 = OSC_CTRL_RESET;
    s->pll_lfsr.reg32 = PLL_LFSR_RESET;
    s->osc_freq_det.reg32 = OSC_FREQ_DET_RESET;
    s->osc_freq_det_status.reg32 = OSC_FREQ_DET_STATUS_RESET;
    s->plle_ss_cntl.reg32 = PLLE_SS_CNTL_RESET;
    s->plle_misc1.reg32 = PLLE_MISC1_OFFSET;
    s->pllc_base.reg32 = PLLC_BASE_RESET | PLL_LOCKED;
    s->pllc_out.reg32 = PLLC_OUT_RESET;
    s->pllc_misc.reg32 = PLLC_MISC_RESET;
    s->pllm_base.reg32 = PLLM_BASE_RESET | PLL_LOCKED;
    s->pllm_out.reg32 = PLLM_OUT_RESET;
    s->pllm_misc1.reg32 = PLLM_MISC1_RESET;
    s->pllp_base.reg32 = PLLP_BASE_RESET | PLL_LOCKED;
    s->pllp_outa.reg32 = PLLP_OUTA_RESET;
    s->pllp_outb.reg32 = PLLP_OUTB_RESET;
    s->pllp_misc.reg32 = PLLP_MISC_RESET;
    s->plla_base.reg32 = PLLA_BASE_RESET | PLL_LOCKED;
    s->plla_out.reg32 = PLLA_OUT_RESET;
    s->plla_misc1.reg32 = PLLA_MISC1_RESET;
    s->pllu_outa.reg32 = PLLU_OUTA_RESET;
    s->pllu_misc1.reg32 = PLLU_MISC1_RESET;
    s->plld_base.reg32 = PLLD_BASE_RESET | PLL_LOCKED;
    s->plld_misc1.reg32 = PLLD_MISC1_RESET;
    s->plld_misc.reg32 = PLLD_MISC_RESET;
    s->pllx_base.reg32 = PLLX_BASE_RESET | PLL_LOCKED;
    s->pllx_misc.reg32 = PLLX_MISC_RESET;
    s->plle_base.reg32 = PLLE_BASE_RESET | PLL_LOCKED;
    s->plle_misc.reg32 = PLLE_MISC_RESET;
    s->plle_ss_cntl1.reg32 = PLLE_SS_CNTL1_RESET;
    s->plle_ss_cntl2.reg32 = PLLE_SS_CNTL2_RESET;
    s->lvl2_clk_gate_ovra.reg32 = LVL2_CLK_GATE_OVRA_RESET;
    s->lvl2_clk_gate_ovrb.reg32 = LVL2_CLK_GATE_OVRB_RESET;
    s->clk_source_i2s1.reg32 = CLK_SOURCE_I2S1_RESET;
    s->clk_source_i2s2.reg32 = CLK_SOURCE_I2S2_RESET;
    s->clk_source_spdif_out.reg32 = CLK_SOURCE_SPDIF_OUT_RESET;
    s->clk_source_spdif_in.reg32 = CLK_SOURCE_SPDIF_IN_RESET;
    s->clk_source_pwm.reg32 = CLK_SOURCE_PWM_RESET;
    s->clk_source_spi1.reg32 = CLK_SOURCE_SPI1_RESET;
    s->clk_source_spi22.reg32 = CLK_SOURCE_SPI22_RESET;
    s->clk_source_spi3.reg32 = CLK_SOURCE_SPI3_RESET;
    s->clk_source_xio.reg32 = CLK_SOURCE_XIO_RESET;
    s->clk_source_i2c1.reg32 = CLK_SOURCE_I2C1_RESET;
    s->clk_source_dvc_i2c.reg32 = CLK_SOURCE_DVC_I2C_RESET;
    s->clk_source_twc.reg32 = CLK_SOURCE_TWC_RESET;
    s->clk_source_sbc1.reg32 = CLK_SOURCE_SBC1_RESET;
    s->clk_source_disp1.reg32 = CLK_SOURCE_DISP1_RESET;
    s->clk_source_disp2.reg32 = CLK_SOURCE_DISP2_RESET;
    s->clk_source_cve.reg32 = CLK_SOURCE_CVE_RESET;
    s->clk_source_ide.reg32 = CLK_SOURCE_IDE_RESET;
    s->clk_source_vi.reg32 = CLK_SOURCE_VI_RESET;
    s->clk_source_sdmmc1.reg32 = CLK_SOURCE_SDMMC1_RESET;
    s->clk_source_sdmmc2.reg32 = CLK_SOURCE_SDMMC2_RESET;
    s->clk_source_g3d.reg32 = CLK_SOURCE_G3D_RESET;
    s->clk_source_g2d.reg32 = CLK_SOURCE_G2D_RESET;
    s->clk_source_ndflash.reg32 = CLK_SOURCE_NDFLASH_RESET;
    s->clk_source_sdmmc4.reg32 = CLK_SOURCE_SDMMC4_RESET;
    s->clk_source_vfir.reg32 = CLK_SOURCE_VFIR_RESET;
    s->clk_source_epp.reg32 = CLK_SOURCE_EPP_RESET;
    s->clk_source_mpe.reg32 = CLK_SOURCE_MPE_RESET;
    s->clk_source_mipi.reg32 = CLK_SOURCE_MIPI_RESET;
    s->clk_source_uart1.reg32 = CLK_SOURCE_UART1_RESET;
    s->clk_source_uart2.reg32 = CLK_SOURCE_UART2_RESET;
    s->clk_source_host1x.reg32 = CLK_SOURCE_HOST1X_RESET;
    s->clk_source_tvo.reg32 = CLK_SOURCE_TVO_RESET;
    s->clk_source_hdmi.reg32 = CLK_SOURCE_HDMI_RESET;
    s->clk_source_tvdac.reg32 = CLK_SOURCE_TVDAC_RESET;
    s->clk_source_i2c2.reg32 = CLK_SOURCE_I2C2_RESET;
    s->clk_source_emc.reg32 = CLK_SOURCE_EMC_RESET;
    s->clk_source_uart3.reg32 = CLK_SOURCE_UART3_RESET;
    s->clk_source_vi_sensor.reg32 = CLK_SOURCE_VI_SENSOR_RESET;
    s->clk_source_spi4.reg32 = CLK_SOURCE_SPI4_RESET;
    s->clk_source_i2c3.reg32 = CLK_SOURCE_I2C3_RESET;
    s->clk_source_sdmmc3.reg32 = CLK_SOURCE_SDMMC3_RESET;
    s->clk_source_uart4.reg32 = CLK_SOURCE_UART4_RESET;
    s->clk_source_uart5.reg32 = CLK_SOURCE_UART5_RESET;
    s->clk_source_vde.reg32 = CLK_SOURCE_VDE_RESET;
    s->clk_source_owr.reg32 = CLK_SOURCE_OWR_RESET;
    s->clk_source_nor.reg32 = CLK_SOURCE_NOR_RESET;
    s->clk_source_csite.reg32 = CLK_SOURCE_CSITE_RESET;
    s->clk_source_osc.reg32 = CLK_SOURCE_OSC_RESET;
    s->utmip_pll_cfg0.reg32 = UTMIP_PLL_CFG0_RESET;
    s->rst_controller_pllc4_base.reg32 = RST_CONTROLLER_PLLC4_BASE_RESET;
    s->rst_controller_pllmb_base.reg32 = RST_CONTROLLER_PLLMB_BASE_RESET | PLL_LOCKED;
    s->clk_source_la.reg32 = CLK_SOURCE_LA_RESET;

    /* Enable UARTA */
    s->rst_devices_l.swr_uart1_rst = 0;
    s->clk_out_enb_l.clk_enb_uart1 = 1;

    /* Enable BSEA */
    s->rst_devices_h.swr_bsea_rst = 0;
    s->clk_out_enb_h.clk_enb_bsea = 1;

    /* Enable BSEV */
    s->rst_devices_h.swr_bsev_rst = 0;
    s->clk_out_enb_h.clk_enb_bsev = 1;
    s->rst_devices_h.swr_vde_rst = 0;

    s->osc_freq_det_status.osc_freq_det_cnt = 1587;

    if (tegra_board >= TEGRAX1_BOARD) {
        s->pllc_misc_1.reg32 = PLLC_MISC_1_TEGRAX1_RESET;
        s->pllm_misc2.reg32 = PLLM_MISC2_TEGRAX1_RESET;
        s->plla_misc.reg32 = PLLA_MISC_TEGRAX1_RESET;
        s->pllu_misc.reg32 = PLLU_MISC_TEGRAX1_RESET;
        s->pllu_base.reg32 = PLLU_BASE_TEGRAX1_RESET | PLL_LOCKED;
        s->rst_cpu_cmplx_clr_offset = RST_CPU_CMPLX_CLR_TEGRAX1_OFFSET;
        s->rst_cpu_cmplx_set.reg32 = RST_CPU_CMPLX_SET_TEGRAX1_RESET;
    }
    else {
        s->pllc_misc_1.reg32 = PLLC_MISC_1_TEGRA2_RESET;
        s->pllm_misc2.reg32 = PLLM_MISC2_TEGRA2_RESET;
        s->plla_misc.reg32 = PLLA_MISC_TEGRA2_RESET;
        s->pllu_misc.reg32 = PLLU_MISC_TEGRA2_RESET;
        s->pllu_base.reg32 = PLLU_BASE_TEGRA2_RESET | PLL_LOCKED;
        s->rst_cpu_cmplx_clr_offset = RST_CPU_CMPLX_CLR_TEGRA2_OFFSET;
        s->rst_cpu_cmplx_set.reg32 = RST_CPU_CMPLX_SET_TEGRA2_RESET;
    }

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[RST_CPUG_CMPLX_SET_OFFSET>>2] = RST_CPUG_CMPLX_SET_RESET;
}

static const MemoryRegionOps tegra_car_mem_ops = {
    .read = tegra_car_priv_read,
    .write = tegra_car_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_car_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_car *s = TEGRA_CAR(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_car_mem_ops, s,
                          "tegra.car", TEGRA_CLK_RESET_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_car_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_car_priv_realize;
    dc->vmsd = &vmstate_tegra_car;
    dc->reset = tegra_car_priv_reset;
}

static const TypeInfo tegra_car_info = {
    .name = TYPE_TEGRA_CAR,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_car),
    .class_init = tegra_car_class_init,
};

static void tegra_car_register_types(void)
{
    type_register_static(&tegra_car_info);
}

type_init(tegra_car_register_types)
