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
#include "qemu/log.h"

#include "pmc.h"
#include "iomap.h"
#include "tegra_trace.h"
#include "tegra_cpu.h"
#include "devices.h"

#define TYPE_TEGRA_PMC "tegra.pmc"
#define TEGRA_PMC(obj) OBJECT_CHECK(tegra_pmc, (obj), TYPE_TEGRA_PMC)
#define DEFINE_REG32(reg) reg##_t reg

static uint32_t tegra_pmc_regdef_tegrax1_reset_table[] = {
    /*TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_CNTRL_0, 0x0, 0x00410A00)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_WAKE_LVL_0, 0x10, 0xFF9FFFFF)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_DPD_PADS_ORIDE_0, 0x1C, 0x00200000)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PWRGATE_TIMER_OFF_0, 0x28, 0xEDCBA987)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PWRGOOD_TIMER_0, 0x3C, 0x003F007F)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_BLINK_TIMER_0, 0x40, 0xFFFFFFFF)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_NO_IOPOWER_0, 0x44, 0x00010080)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PWR_DET_0, 0x48, 0x00FEB425)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PWR_DET_LATCH_0, 0x4C, 0x00000001)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_CPUPWRGOOD_TIMER_0, 0xC8, 0x0000FFFF)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_CPUPWROFF_TIMER_0, 0xCC, 0x0000FFFF)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PG_MASK_0, 0xD0, 0xFFFFFFFF)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PG_MASK_1_0, 0xD4, 0xFFFFFFFF)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PWR_DET_VAL_0, 0xE4, 0x00FCBC2D)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_DDR_PWR_0, 0xE8, 0x0000000F)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_USB_AO_0, 0xF0, 0x01FFFFFF)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_CRYPTO_OP_0, 0xF4, 0x00000001)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PLLP_WB0_OVERRIDE_0, 0xF8, 0x00018000)*/

    // Erista NX_Bootloader with certain versions expects the regs commented out below to be 0.

    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_WAKE2_LVL_0, 0x164, 0xF9FFFFE7)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PG_MASK_2_0, 0x174, 0xFFFFFFFF)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PG_MASK_CE1_0, 0x178, 0x000000FF)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PG_MASK_CE2_0, 0x17C, 0x000000FF)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PG_MASK_CE3_0, 0x180, 0x000000FF)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PWRGATE_TIMER_CE_0_0, 0x184, 0xEDCBA987)
    /*TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PWRGATE_TIMER_CE_1_0, 0x188, 0x1240E30A)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PWRGATE_TIMER_CE_2_0, 0x18C, 0x1C698594)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PWRGATE_TIMER_CE_3_0, 0x190, 0x2692281E)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PWRGATE_TIMER_CE_4_0, 0x194, 0x30BACAA8)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PWRGATE_TIMER_CE_5_0, 0x198, 0x3AE36D32)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PWRGATE_TIMER_CE_6_0, 0x19C, 0x00000F3B)*/
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_OSC_EDPD_OVER_0, 0x1A4, 0x0000007E)
    //TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_SATA_PWRGT_0, 0x1AC, 0x0000003F)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_IO_DPD2_STATUS_0, 0x1C4, 0x02000000)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_SEL_DPD_TIM_0, 0x1C8, 0x0000007F)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_VDDP_SEL_0, 0x1CC, 0x00000003)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PLLM_WB0_OVERRIDE_FREQ_0, 0x1DC, 0x00002A02)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PWRGATE_TIMER_MULT_0, 0x1E4, 0x0000001B)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_UHSIC_SAVED_STATE_0, 0x1F0, 0x0F0F0F0F)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_TERM_PAD_CFG_0, 0x1F8, 0x00041041)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG_0, 0x1FC, 0xC0C0C0C0)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_SLEEPWALK_P0_0, 0x204, 0x23232363)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_SLEEPWALK_P1_0, 0x208, 0x23232363)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_SLEEPWALK_P2_0, 0x20C, 0x23232363)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UHSIC_SLEEPWALK_P0_0, 0x210, 0x16161616)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_UHSIC_FAKE_0, 0x218, 0x01111111)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_POR_DPD_CTRL_0, 0x264, 0x80000003)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_UHSIC_LINE_WAKEUP_0, 0x26C, 0x0000001F)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_BIAS_MASTER_CNTRL_0, 0x270, 0x0000000D)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_TD_PWRGATE_INTER_PART_TIMER_0, 0x278, 0x0000000F)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_UHSIC2_SAVED_STATE_0, 0x280, 0x00000F07)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_UHSIC2_SLEEP_CFG_0, 0x284, 0x000000C0)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UHSIC2_SLEEPWALK_P1_0, 0x28C, 0x06060606)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_UHSIC2_FAKE_0, 0x294, 0x00001101)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_UHSIC2_LINE_WAKEUP_0, 0x298, 0x00000001)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PG_MASK_CE0_0, 0x2A4, 0x000000FF)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PG_MASK_3_0, 0x2A8, 0xFFFFFFFF)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PG_MASK_4_0, 0x2AC, 0xFFFFFFFF)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_TSC_MULT_0, 0x2B4, 0x000016E0)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_CPU_VSENSE_OVERRIDE_0, 0x2B8, 0x0000001F)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_GLB_AMAP_CFG_0, 0x2BC, 0x00020000)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_GPU_RG_CNTRL_0, 0x2D4, 0x00000001)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PG_MASK_5_0, 0x2DC, 0xFFFFFFFF)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_PG_MASK_6_0, 0x2E0, 0x000000FF)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_CNTRL2_0, 0x440, 0x00000001)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_IO_DPD_OFF_MASK_0, 0x444, 0x01F00000)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_IO_DPD2_OFF_MASK_0, 0x448, 0x1DF30000)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_FUSE_CONTROL_0, 0x450, 0x00020200)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_IO_DPD3_REQ_0, 0x45C, 0x0FFF0000)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_IO_DPD3_STATUS_0, 0x460, 0x0FFF0000)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_DIRECT_THERMTRIP_CFG_0, 0x474, 0x00000010)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_TSOSC_DELAY_0, 0x478, 0x0000007F)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_DEBUG_AUTHENTICATION_0, 0x480, 0x0000003F)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_AOTAG_CFG_0, 0x484, 0x00000003)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_AOTAG_THRESH1_CFG_0, 0x488, 0x007F80FA)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_AOTAG_THRESH2_CFG_0, 0x48C, 0x00000005)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_AOTAG_THRESH3_CFG_0, 0x490, 0x000000FA)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_TSENSOR_CONFIG0_0, 0x49C, 0x00000001)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_PAD_CFG0_0, 0x4C0, 0x28400000)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_PAD_CFG1_0, 0x4C4, 0x28400000)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_PAD_CFG2_0, 0x4C8, 0x28400000)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_PAD_CFG3_0, 0x4CC, 0x28400000)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_UHSIC_SLEEP_CFG1_0, 0x4D0, 0x000000C0)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_RAMDUMP_CTL_STATUS_0, 0x4DC, 0x000000FC)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_UTMIP_SLEEPWALK_P3_0, 0x4E0, 0x23232363)
    TEGRA_REGDEF_TABLE_RESET(APBDEV_PMC_DDR_CNTRL_0, 0x4E4, 0x0007FF9F)
};

typedef struct tegra_pmc_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    DEFINE_REG32(cntrl);
    DEFINE_REG32(sec_disable);
    DEFINE_REG32(pmc_swrst);
    DEFINE_REG32(wake_mask);
    DEFINE_REG32(wake_lvl);
    DEFINE_REG32(wake_status);
    DEFINE_REG32(sw_wake_status);
    DEFINE_REG32(dpd_pads_oride);
    DEFINE_REG32(dpd_sample);
    DEFINE_REG32(dpd_enable);
    DEFINE_REG32(pwrgate_timer_off);
    DEFINE_REG32(clamp_status);
    DEFINE_REG32(pwrgate_toggle);
    DEFINE_REG32(remove_clamping_cmd);
    DEFINE_REG32(pwrgate_status);
    DEFINE_REG32(pwrgood_timer);
    DEFINE_REG32(blink_timer);
    DEFINE_REG32(no_iopower);
    DEFINE_REG32(pwr_det);
    DEFINE_REG32(pwr_det_latch);
    DEFINE_REG32(scratch0);
    DEFINE_REG32(scratch1);
    DEFINE_REG32(scratch2);
    DEFINE_REG32(scratch3);
    DEFINE_REG32(scratch4);
    DEFINE_REG32(scratch5);
    DEFINE_REG32(scratch6);
    DEFINE_REG32(scratch7);
    DEFINE_REG32(scratch8);
    DEFINE_REG32(scratch9);
    DEFINE_REG32(scratch10);
    DEFINE_REG32(scratch11);
    DEFINE_REG32(scratch12);
    DEFINE_REG32(scratch13);
    DEFINE_REG32(scratch14);
    DEFINE_REG32(scratch15);
    DEFINE_REG32(scratch16);
    DEFINE_REG32(scratch17);
    DEFINE_REG32(scratch18);
    DEFINE_REG32(scratch19);
    DEFINE_REG32(scratch20);
    DEFINE_REG32(scratch21);
    DEFINE_REG32(scratch22);
    DEFINE_REG32(scratch23);
    DEFINE_REG32(secure_scratch0);
    DEFINE_REG32(secure_scratch1);
    DEFINE_REG32(secure_scratch2);
    DEFINE_REG32(secure_scratch3);
    DEFINE_REG32(secure_scratch4);
    DEFINE_REG32(secure_scratch5);
    DEFINE_REG32(cpupwrgood_timer);
    DEFINE_REG32(cpupwroff_timer);
    DEFINE_REG32(pg_mask);
    DEFINE_REG32(pg_mask_1);
    DEFINE_REG32(auto_wake_lvl);
    DEFINE_REG32(auto_wake_lvl_mask);
    DEFINE_REG32(wake_delay);
    DEFINE_REG32(pwr_det_val);
    DEFINE_REG32(ddr_pwr);
    DEFINE_REG32(usb_debounce_del);
    DEFINE_REG32(usb_ao);
    DEFINE_REG32(crypto_op);
    DEFINE_REG32(pllp_wb0_override);
    DEFINE_REG32(scratch24);
    DEFINE_REG32(scratch25);
    DEFINE_REG32(scratch26);
    DEFINE_REG32(scratch27);
    DEFINE_REG32(scratch28);
    DEFINE_REG32(scratch29);
    DEFINE_REG32(scratch30);
    DEFINE_REG32(scratch31);
    DEFINE_REG32(scratch32);
    DEFINE_REG32(scratch33);
    DEFINE_REG32(scratch34);
    DEFINE_REG32(scratch35);
    DEFINE_REG32(scratch36);
    DEFINE_REG32(scratch37);
    DEFINE_REG32(scratch38);
    DEFINE_REG32(scratch39);
    DEFINE_REG32(scratch40);
    DEFINE_REG32(scratch41);
    DEFINE_REG32(scratch42);
    DEFINE_REG32(bondout_mirror0);
    DEFINE_REG32(bondout_mirror1);
    DEFINE_REG32(bondout_mirror2);
    DEFINE_REG32(sys_33v_en);
    DEFINE_REG32(bondout_mirror_access);
    DEFINE_REG32(gate);

    uint32_t regs[(0xC00-0x160)>>2];

} tegra_pmc;

static const VMStateDescription vmstate_tegra_pmc = {
    .name = "tegra.pmc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(cntrl.reg32, tegra_pmc),
        VMSTATE_UINT32(sec_disable.reg32, tegra_pmc),
        VMSTATE_UINT32(pmc_swrst.reg32, tegra_pmc),
        VMSTATE_UINT32(wake_mask.reg32, tegra_pmc),
        VMSTATE_UINT32(wake_lvl.reg32, tegra_pmc),
        VMSTATE_UINT32(wake_status.reg32, tegra_pmc),
        VMSTATE_UINT32(sw_wake_status.reg32, tegra_pmc),
        VMSTATE_UINT32(dpd_pads_oride.reg32, tegra_pmc),
        VMSTATE_UINT32(dpd_sample.reg32, tegra_pmc),
        VMSTATE_UINT32(dpd_enable.reg32, tegra_pmc),
        VMSTATE_UINT32(pwrgate_timer_off.reg32, tegra_pmc),
        VMSTATE_UINT32(clamp_status.reg32, tegra_pmc),
        VMSTATE_UINT32(pwrgate_toggle.reg32, tegra_pmc),
        VMSTATE_UINT32(remove_clamping_cmd.reg32, tegra_pmc),
        VMSTATE_UINT32(pwrgate_status.reg32, tegra_pmc),
        VMSTATE_UINT32(pwrgood_timer.reg32, tegra_pmc),
        VMSTATE_UINT32(blink_timer.reg32, tegra_pmc),
        VMSTATE_UINT32(no_iopower.reg32, tegra_pmc),
        VMSTATE_UINT32(pwr_det.reg32, tegra_pmc),
        VMSTATE_UINT32(pwr_det_latch.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch0.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch1.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch2.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch3.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch4.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch5.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch6.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch7.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch8.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch9.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch10.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch11.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch12.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch13.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch14.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch15.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch16.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch17.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch18.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch19.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch20.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch21.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch22.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch23.reg32, tegra_pmc),
        VMSTATE_UINT32(secure_scratch0.reg32, tegra_pmc),
        VMSTATE_UINT32(secure_scratch1.reg32, tegra_pmc),
        VMSTATE_UINT32(secure_scratch2.reg32, tegra_pmc),
        VMSTATE_UINT32(secure_scratch3.reg32, tegra_pmc),
        VMSTATE_UINT32(secure_scratch4.reg32, tegra_pmc),
        VMSTATE_UINT32(secure_scratch5.reg32, tegra_pmc),
        VMSTATE_UINT32(cpupwrgood_timer.reg32, tegra_pmc),
        VMSTATE_UINT32(cpupwroff_timer.reg32, tegra_pmc),
        VMSTATE_UINT32(pg_mask.reg32, tegra_pmc),
        VMSTATE_UINT32(pg_mask_1.reg32, tegra_pmc),
        VMSTATE_UINT32(auto_wake_lvl.reg32, tegra_pmc),
        VMSTATE_UINT32(auto_wake_lvl_mask.reg32, tegra_pmc),
        VMSTATE_UINT32(wake_delay.reg32, tegra_pmc),
        VMSTATE_UINT32(pwr_det_val.reg32, tegra_pmc),
        VMSTATE_UINT32(ddr_pwr.reg32, tegra_pmc),
        VMSTATE_UINT32(usb_debounce_del.reg32, tegra_pmc),
        VMSTATE_UINT32(usb_ao.reg32, tegra_pmc),
        VMSTATE_UINT32(crypto_op.reg32, tegra_pmc),
        VMSTATE_UINT32(pllp_wb0_override.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch24.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch25.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch26.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch27.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch28.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch29.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch30.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch31.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch32.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch33.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch34.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch35.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch36.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch37.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch38.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch39.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch40.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch41.reg32, tegra_pmc),
        VMSTATE_UINT32(scratch42.reg32, tegra_pmc),
        VMSTATE_UINT32(bondout_mirror0.reg32, tegra_pmc),
        VMSTATE_UINT32(bondout_mirror1.reg32, tegra_pmc),
        VMSTATE_UINT32(bondout_mirror2.reg32, tegra_pmc),
        VMSTATE_UINT32(sys_33v_en.reg32, tegra_pmc),
        VMSTATE_UINT32(bondout_mirror_access.reg32, tegra_pmc),
        VMSTATE_UINT32(gate.reg32, tegra_pmc),
        VMSTATE_UINT32_ARRAY(regs, tegra_pmc, (0xC00-0x160)>>2),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_pmc_priv_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    tegra_pmc *s = opaque;
    uint64_t ret = 0;

    switch (offset) {
    case CNTRL_OFFSET:
        ret = s->cntrl.reg32;
        break;
    case SEC_DISABLE_OFFSET:
        ret = s->sec_disable.reg32;
        break;
    case PMC_SWRST_OFFSET:
        ret = s->pmc_swrst.reg32;
        break;
    case WAKE_MASK_OFFSET:
        ret = s->wake_mask.reg32;
        break;
    case WAKE_LVL_OFFSET:
        ret = s->wake_lvl.reg32;
        break;
    case WAKE_STATUS_OFFSET:
        ret = s->wake_status.reg32;
        break;
    case SW_WAKE_STATUS_OFFSET:
        ret = s->sw_wake_status.reg32;
        break;
    case DPD_PADS_ORIDE_OFFSET:
        ret = s->dpd_pads_oride.reg32;
        break;
    case DPD_SAMPLE_OFFSET:
        ret = s->dpd_sample.reg32;
        break;
    case DPD_ENABLE_OFFSET:
        ret = s->dpd_enable.reg32;
        break;
    case PWRGATE_TIMER_OFF_OFFSET:
        ret = s->pwrgate_timer_off.reg32;
        break;
    case CLAMP_STATUS_OFFSET:
        ret = s->clamp_status.reg32;
        break;
    case PWRGATE_TOGGLE_OFFSET:
        ret = s->pwrgate_toggle.reg32;
        break;
    case REMOVE_CLAMPING_CMD_OFFSET:
        ret = s->remove_clamping_cmd.reg32;
        break;
    case PWRGATE_STATUS_OFFSET:
        pwrgate_status_t tmp = s->pwrgate_status;

        if (tegra_board >= TEGRAX1_BOARD) {
            bool crail = 0, flag = 0;
            for (int cpu_id=0; cpu_id < TEGRAX1_CCPLEX_NCORES; cpu_id++) {
                int partid = cpu_id == 0 ? 14 : 9 + cpu_id-1;

                flag = (tegra_cpu_is_powergated(cpu_id)==0);
                crail |= flag;

                tmp.reg32 &= ~(1<<partid);
                tmp.reg32 |= flag<<partid;
            }
            tmp.cpu = crail; // CRAIL: CPU Rail
        }
        else {
            s->pwrgate_status.cpu = (tegra_cpu_is_powergated(TEGRA_CCPLEX_CORE0) && tegra_cpu_is_powergated(TEGRA_CCPLEX_CORE1))==0;
        }

        ret = tmp.reg32;
        break;
    case PWRGOOD_TIMER_OFFSET:
        ret = s->pwrgood_timer.reg32;
        break;
    case BLINK_TIMER_OFFSET:
        ret = s->blink_timer.reg32;
        break;
    case NO_IOPOWER_OFFSET:
        ret = s->no_iopower.reg32;
        break;
    case PWR_DET_OFFSET:
        ret = s->pwr_det.reg32;
        break;
    case PWR_DET_LATCH_OFFSET:
        ret = s->pwr_det_latch.reg32;
        break;
    case SCRATCH0_OFFSET:
        ret = s->scratch0.reg32;
        break;
    case SCRATCH1_OFFSET:
        ret = s->scratch1.reg32;
        break;
    case SCRATCH2_OFFSET:
        ret = s->scratch2.reg32;
        break;
    case SCRATCH3_OFFSET:
        ret = s->scratch3.reg32;
        break;
    case SCRATCH4_OFFSET:
        ret = s->scratch4.reg32;
        break;
    case SCRATCH5_OFFSET:
        ret = s->scratch5.reg32;
        break;
    case SCRATCH6_OFFSET:
        ret = s->scratch6.reg32;
        break;
    case SCRATCH7_OFFSET:
        ret = s->scratch7.reg32;
        break;
    case SCRATCH8_OFFSET:
        ret = s->scratch8.reg32;
        break;
    case SCRATCH9_OFFSET:
        ret = s->scratch9.reg32;
        break;
    case SCRATCH10_OFFSET:
        ret = s->scratch10.reg32;
        break;
    case SCRATCH11_OFFSET:
        ret = s->scratch11.reg32;
        break;
    case SCRATCH12_OFFSET:
        ret = s->scratch12.reg32;
        break;
    case SCRATCH13_OFFSET:
        ret = s->scratch13.reg32;
        break;
    case SCRATCH14_OFFSET:
        ret = s->scratch14.reg32;
        break;
    case SCRATCH15_OFFSET:
        ret = s->scratch15.reg32;
        break;
    case SCRATCH16_OFFSET:
        ret = s->scratch16.reg32;
        break;
    case SCRATCH17_OFFSET:
        ret = s->scratch17.reg32;
        break;
    case SCRATCH18_OFFSET:
        ret = s->scratch18.reg32;
        break;
    case SCRATCH19_OFFSET:
        ret = s->scratch19.reg32;
        break;
    case SCRATCH20_OFFSET:
        ret = s->scratch20.reg32;
        break;
    case SCRATCH21_OFFSET:
        ret = s->scratch21.reg32;
        break;
    case SCRATCH22_OFFSET:
        ret = s->scratch22.reg32;
        break;
    case SCRATCH23_OFFSET:
        ret = s->scratch23.reg32;
        break;
    case SECURE_SCRATCH0_OFFSET:
        ret = s->secure_scratch0.reg32;
        break;
    case SECURE_SCRATCH1_OFFSET:
        ret = s->secure_scratch1.reg32;
        break;
    case SECURE_SCRATCH2_OFFSET:
        ret = s->secure_scratch2.reg32;
        break;
    case SECURE_SCRATCH3_OFFSET:
        ret = s->secure_scratch3.reg32;
        break;
    case SECURE_SCRATCH4_OFFSET:
        ret = s->secure_scratch4.reg32;
        break;
    case SECURE_SCRATCH5_OFFSET:
        ret = s->secure_scratch5.reg32;
        break;
    case CPUPWRGOOD_TIMER_OFFSET:
        ret = s->cpupwrgood_timer.reg32;
        break;
    case CPUPWROFF_TIMER_OFFSET:
        ret = s->cpupwroff_timer.reg32;
        break;
    case PG_MASK_OFFSET:
        ret = s->pg_mask.reg32;
        break;
    case PG_MASK_1_OFFSET:
        ret = s->pg_mask_1.reg32;
        break;
    case AUTO_WAKE_LVL_OFFSET:
        ret = s->auto_wake_lvl.reg32;
        break;
    case AUTO_WAKE_LVL_MASK_OFFSET:
        ret = s->auto_wake_lvl_mask.reg32;
        break;
    case WAKE_DELAY_OFFSET:
        ret = s->wake_delay.reg32;
        break;
    case PWR_DET_VAL_OFFSET:
        ret = s->pwr_det_val.reg32;
        break;
    case DDR_PWR_OFFSET:
        ret = s->ddr_pwr.reg32;
        break;
    case USB_DEBOUNCE_DEL_OFFSET:
        ret = s->usb_debounce_del.reg32;
        break;
    case USB_AO_OFFSET:
        ret = s->usb_ao.reg32;
        break;
    case CRYPTO_OP_OFFSET:
        ret = s->crypto_op.reg32;
        break;
    case PLLP_WB0_OVERRIDE_OFFSET:
        ret = s->pllp_wb0_override.reg32;
        break;
    case SCRATCH24_OFFSET:
        ret = s->scratch24.reg32;
        break;
    case SCRATCH25_OFFSET:
        ret = s->scratch25.reg32;
        break;
    case SCRATCH26_OFFSET:
        ret = s->scratch26.reg32;
        break;
    case SCRATCH27_OFFSET:
        ret = s->scratch27.reg32;
        break;
    case SCRATCH28_OFFSET:
        ret = s->scratch28.reg32;
        break;
    case SCRATCH29_OFFSET:
        ret = s->scratch29.reg32;
        break;
    case SCRATCH30_OFFSET:
        ret = s->scratch30.reg32;
        break;
    case SCRATCH31_OFFSET:
        ret = s->scratch31.reg32;
        break;
    case SCRATCH32_OFFSET:
        ret = s->scratch32.reg32;
        break;
    case SCRATCH33_OFFSET:
        ret = s->scratch33.reg32;
        break;
    case SCRATCH34_OFFSET:
        ret = s->scratch34.reg32;
        break;
    case SCRATCH35_OFFSET:
        ret = s->scratch35.reg32;
        break;
    case SCRATCH36_OFFSET:
        ret = s->scratch36.reg32;
        break;
    case SCRATCH37_OFFSET:
        ret = s->scratch37.reg32;
        break;
    case SCRATCH38_OFFSET:
        ret = s->scratch38.reg32;
        break;
    case SCRATCH39_OFFSET:
        ret = s->scratch39.reg32;
        break;
    case SCRATCH40_OFFSET:
        ret = s->scratch40.reg32;
        break;
    case SCRATCH41_OFFSET:
        ret = s->scratch41.reg32;
        break;
    case SCRATCH42_OFFSET:
        ret = s->scratch42.reg32;
        break;
    case BONDOUT_MIRROR0_OFFSET:
        ret = s->bondout_mirror0.reg32;
        break;
    case BONDOUT_MIRROR1_OFFSET:
        ret = s->bondout_mirror1.reg32;
        break;
    case BONDOUT_MIRROR2_OFFSET:
        ret = s->bondout_mirror2.reg32;
        break;
    case SYS_33V_EN_OFFSET:
        ret = s->sys_33v_en.reg32;
        break;
    case BONDOUT_MIRROR_ACCESS_OFFSET:
        ret = s->bondout_mirror_access.reg32;
        break;
    case GATE_OFFSET:
        ret = s->gate.reg32;
        break;
    default:
        if (offset>=0x160) {
            if (offset != UTMIP_UHSIC_TRIGGERS_OFFSET && offset != UTMIP_UHSIC2_TRIGGERS_OFFSET && offset != AOTAG_INTR_DIS_OFFSET) { // These regs always returns 0.
                ret = s->regs[(offset-0x160)>>2];
            }
        }
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_pmc_priv_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    tegra_pmc *s = opaque;

    switch (offset) {
    case CNTRL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->cntrl.reg32, value);
        s->cntrl.reg32 = value;

        if (s->cntrl.reg32 & 0x10) {
            qemu_log_mask(LOG_GUEST_ERROR, "tegra.pmc: Requesting a system reset.\n");
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
        break;
    case SEC_DISABLE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->sec_disable.reg32, value);
        s->sec_disable.reg32 = value;
        break;
    case PMC_SWRST_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pmc_swrst.reg32, value);
        s->pmc_swrst.reg32 = value;
        break;
    case WAKE_MASK_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->wake_mask.reg32, value);
        s->wake_mask.reg32 = value;
        break;
    case WAKE_LVL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->wake_lvl.reg32, value);
        s->wake_lvl.reg32 = value;
        break;
    case WAKE_STATUS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->wake_status.reg32, value);
        s->wake_status.reg32 = value;
        break;
    case SW_WAKE_STATUS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->sw_wake_status.reg32, value);
        s->sw_wake_status.reg32 = value;
        break;
    case DPD_PADS_ORIDE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->dpd_pads_oride.reg32, value);
        s->dpd_pads_oride.reg32 = value;
        break;
    case DPD_SAMPLE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->dpd_sample.reg32, value);
        s->dpd_sample.reg32 = value;
        break;
    case DPD_ENABLE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->dpd_enable.reg32, value);
        s->dpd_enable.reg32 = value;
        break;
    case CLAMP_STATUS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->clamp_status.reg32, value);
        s->clamp_status.reg32 = value;
        break;
    case PWRGATE_TOGGLE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pwrgate_toggle.reg32, value);
        s->pwrgate_toggle.reg32 = value;

        if (s->pwrgate_toggle.start) {
            s->pwrgate_toggle.start = 0;
            s->pwrgate_status.reg32 ^= (1 << s->pwrgate_toggle.partid);

            if (tegra_board >= TEGRAX1_BOARD) {
                if (s->pwrgate_toggle.partid >= 9 && s->pwrgate_toggle.partid <= 14) {
                    int cpu_id = s->pwrgate_toggle.partid < 14 ? s->pwrgate_toggle.partid - 8 : 0;
                    if (cpu_id < TEGRAX1_CCPLEX_NCORES) {
                        if (tegra_cpu_is_powergated(cpu_id))
                            tegra_cpu_unpowergate(cpu_id);
                        else
                            tegra_cpu_powergate(cpu_id);
                    }
                }
            }
            else {
                if (s->pwrgate_toggle.partid == 0) {
                    if (tegra_cpu_is_powergated(TEGRA_CCPLEX_CORE0) && tegra_cpu_is_powergated(TEGRA_CCPLEX_CORE1)) {
                        tegra_cpu_unpowergate(TEGRA_CCPLEX_CORE0);
                        tegra_cpu_unpowergate(TEGRA_CCPLEX_CORE1);
                    }
                    else {
                        tegra_cpu_powergate(TEGRA_CCPLEX_CORE0);
                        tegra_cpu_powergate(TEGRA_CCPLEX_CORE1);
                    }
                }
            }
        }
        break;
    case REMOVE_CLAMPING_CMD_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->remove_clamping_cmd.reg32, value);
        s->remove_clamping_cmd.reg32 = value;
        break;
    case PWRGOOD_TIMER_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pwrgood_timer.reg32, value);
        s->pwrgood_timer.reg32 = value;
        break;
    case BLINK_TIMER_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->blink_timer.reg32, value);
        s->blink_timer.reg32 = value;
        break;
    case NO_IOPOWER_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->no_iopower.reg32, value);
        s->no_iopower.reg32 = value;
        break;
    case PWR_DET_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pwr_det.reg32, value);
        s->pwr_det.reg32 = value;
        break;
    case PWR_DET_LATCH_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pwr_det_latch.reg32, value);
        s->pwr_det_latch.reg32 = value;
        break;
    case SCRATCH0_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch0.reg32, value);
        s->scratch0.reg32 = value;
        break;
    case SCRATCH1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch1.reg32, value);
        s->scratch1.reg32 = value;
        break;
    case SCRATCH2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch2.reg32, value);
        s->scratch2.reg32 = value;
        break;
    case SCRATCH3_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch3.reg32, value);
        s->scratch3.reg32 = value;
        break;
    case SCRATCH4_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch4.reg32, value);
        s->scratch4.reg32 = value;
        break;
    case SCRATCH5_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch5.reg32, value);
        s->scratch5.reg32 = value;
        break;
    case SCRATCH6_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch6.reg32, value);
        s->scratch6.reg32 = value;
        break;
    case SCRATCH7_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch7.reg32, value);
        s->scratch7.reg32 = value;
        break;
    case SCRATCH8_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch8.reg32, value);
        s->scratch8.reg32 = value;
        break;
    case SCRATCH9_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch9.reg32, value);
        s->scratch9.reg32 = value;
        break;
    case SCRATCH10_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch10.reg32, value);
        s->scratch10.reg32 = value;
        break;
    case SCRATCH11_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch11.reg32, value);
        s->scratch11.reg32 = value;
        break;
    case SCRATCH12_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch12.reg32, value);
        s->scratch12.reg32 = value;
        break;
    case SCRATCH13_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch13.reg32, value);
        s->scratch13.reg32 = value;
        break;
    case SCRATCH14_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch14.reg32, value);
        s->scratch14.reg32 = value;
        break;
    case SCRATCH15_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch15.reg32, value);
        s->scratch15.reg32 = value;
        break;
    case SCRATCH16_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch16.reg32, value);
        s->scratch16.reg32 = value;
        break;
    case SCRATCH17_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch17.reg32, value);
        s->scratch17.reg32 = value;
        break;
    case SCRATCH18_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch18.reg32, value);
        s->scratch18.reg32 = value;
        break;
    case SCRATCH19_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch19.reg32, value);
        s->scratch19.reg32 = value;
        break;
    case SCRATCH20_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch20.reg32, value);
//         s->scratch20.reg32 = value;
        break;
    case SCRATCH21_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch21.reg32, value);
        s->scratch21.reg32 = value;
        break;
    case SCRATCH22_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch22.reg32, value);
        s->scratch22.reg32 = value;
        break;
    case SCRATCH23_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch23.reg32, value);
        s->scratch23.reg32 = value;
        break;
    case SECURE_SCRATCH0_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->secure_scratch0.reg32, value);
        s->secure_scratch0.reg32 = value;
        break;
    case SECURE_SCRATCH1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->secure_scratch1.reg32, value);
        s->secure_scratch1.reg32 = value;
        break;
    case SECURE_SCRATCH2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->secure_scratch2.reg32, value);
        s->secure_scratch2.reg32 = value;
        break;
    case SECURE_SCRATCH3_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->secure_scratch3.reg32, value);
        s->secure_scratch3.reg32 = value;
        break;
    case SECURE_SCRATCH4_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->secure_scratch4.reg32, value);
        s->secure_scratch4.reg32 = value;
        break;
    case SECURE_SCRATCH5_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->secure_scratch5.reg32, value);
        s->secure_scratch5.reg32 = value;
        break;
    case CPUPWRGOOD_TIMER_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->cpupwrgood_timer.reg32, value);
        s->cpupwrgood_timer.reg32 = value;
        break;
    case CPUPWROFF_TIMER_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->cpupwroff_timer.reg32, value);
        s->cpupwroff_timer.reg32 = value;
        break;
    case PG_MASK_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pg_mask.reg32, value);
        s->pg_mask.reg32 = value;
        break;
    case PG_MASK_1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pg_mask_1.reg32, value);
        s->pg_mask_1.reg32 = value;
        break;
    case AUTO_WAKE_LVL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->auto_wake_lvl.reg32, value);
        s->auto_wake_lvl.reg32 = value;
        break;
    case AUTO_WAKE_LVL_MASK_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->auto_wake_lvl_mask.reg32, value);
        s->auto_wake_lvl_mask.reg32 = value;
        break;
    case WAKE_DELAY_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->wake_delay.reg32, value);
        s->wake_delay.reg32 = value;
        break;
    case PWR_DET_VAL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pwr_det_val.reg32, value);
        s->pwr_det_val.reg32 = value;
        break;
    case DDR_PWR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ddr_pwr.reg32, value);
        s->ddr_pwr.reg32 = value;
        break;
    case USB_DEBOUNCE_DEL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->usb_debounce_del.reg32, value);
        s->usb_debounce_del.reg32 = value;
        break;
    case USB_AO_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->usb_ao.reg32, value);
        s->usb_ao.reg32 = value;
        break;
    case CRYPTO_OP_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->crypto_op.reg32, value);
        s->crypto_op.reg32 = value;
        break;
    case PLLP_WB0_OVERRIDE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->pllp_wb0_override.reg32, value);
        s->pllp_wb0_override.reg32 = value;
        break;
    case SCRATCH24_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch24.reg32, value);
        s->scratch24.reg32 = value;
        break;
    case SCRATCH25_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch25.reg32, value);
        s->scratch25.reg32 = value;
        break;
    case SCRATCH26_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch26.reg32, value);
        s->scratch26.reg32 = value;
        break;
    case SCRATCH27_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch27.reg32, value);
        s->scratch27.reg32 = value;
        break;
    case SCRATCH28_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch28.reg32, value);
        s->scratch28.reg32 = value;
        break;
    case SCRATCH29_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch29.reg32, value);
        s->scratch29.reg32 = value;
        break;
    case SCRATCH30_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch30.reg32, value);
        s->scratch30.reg32 = value;
        break;
    case SCRATCH31_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch31.reg32, value);
        s->scratch31.reg32 = value;
        break;
    case SCRATCH32_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch32.reg32, value);
        s->scratch32.reg32 = value;
        break;
    case SCRATCH33_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch33.reg32, value);
        s->scratch33.reg32 = value;
        break;
    case SCRATCH34_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch34.reg32, value);
        s->scratch34.reg32 = value;
        break;
    case SCRATCH35_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch35.reg32, value);
        s->scratch35.reg32 = value;
        break;
    case SCRATCH36_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch36.reg32, value);
        s->scratch36.reg32 = value;
        break;
    case SCRATCH37_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch37.reg32, value);
        s->scratch37.reg32 = value;
        break;
    case SCRATCH38_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch38.reg32, value);
        s->scratch38.reg32 = value;
        break;
    case SCRATCH39_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch39.reg32, value);
        s->scratch39.reg32 = value;
        break;
    case SCRATCH40_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch40.reg32, value);
        s->scratch40.reg32 = value;
        break;
    case SCRATCH41_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch41.reg32, value);
        s->scratch41.reg32 = value;
        break;
    case SCRATCH42_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->scratch42.reg32, value);
        s->scratch42.reg32 = value;
        break;
    case BONDOUT_MIRROR0_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bondout_mirror0.reg32, value);
        s->bondout_mirror0.reg32 = value;
        break;
    case BONDOUT_MIRROR1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bondout_mirror1.reg32, value);
        s->bondout_mirror1.reg32 = value;
        break;
    case BONDOUT_MIRROR2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bondout_mirror2.reg32, value);
        s->bondout_mirror2.reg32 = value;
        break;
    case SYS_33V_EN_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->sys_33v_en.reg32, value);
        s->sys_33v_en.reg32 = value;
        break;
    case BONDOUT_MIRROR_ACCESS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->bondout_mirror_access.reg32, value);
        s->bondout_mirror_access.reg32 = value;
        break;
    case GATE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->gate.reg32, value);
        s->gate.reg32 = value;
        break;
    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        if (offset>=0x160) {
            if (offset == AOTAG_INTR_DIS_OFFSET)
                s->regs[(AOTAG_INTR_EN_OFFSET-0x160)>>2] &= ~(value & 0x3);
            else
                s->regs[(offset-0x160)>>2] = value;
        }
        break;
    }
}

void tegra_pmc_set_rst_status(uint32_t value)
{
    tegra_pmc *s = tegra_pmc_dev;

    s->regs[(RST_STATUS_OFFSET-0x160)>>2] = value;
}

void tegra_pmc_reset(DeviceState *dev, ShutdownCause cause)
{
    tegra_pmc *s = TEGRA_PMC(dev);

    s->sec_disable.reg32 = SEC_DISABLE_RESET;
    s->pmc_swrst.reg32 = PMC_SWRST_RESET;
    s->wake_mask.reg32 = WAKE_MASK_RESET;
    s->wake_status.reg32 = WAKE_STATUS_RESET;
    s->sw_wake_status.reg32 = SW_WAKE_STATUS_RESET;
    s->dpd_pads_oride.reg32 = DPD_PADS_ORIDE_RESET;
    s->dpd_sample.reg32 = DPD_SAMPLE_RESET;
    s->dpd_enable.reg32 = DPD_ENABLE_RESET;
    s->clamp_status.reg32 = CLAMP_STATUS_RESET;
    s->pwrgate_toggle.reg32 = PWRGATE_TOGGLE_RESET;
    s->remove_clamping_cmd.reg32 = REMOVE_CLAMPING_CMD_RESET;
    s->pwrgate_status.reg32 = PWRGATE_STATUS_RESET;
    s->blink_timer.reg32 = BLINK_TIMER_RESET;
    s->pwr_det_latch.reg32 = PWR_DET_LATCH_RESET;
    s->cpupwrgood_timer.reg32 = CPUPWRGOOD_TIMER_RESET;
    s->cpupwroff_timer.reg32 = CPUPWROFF_TIMER_RESET;
    s->pg_mask.reg32 = PG_MASK_RESET;
    s->auto_wake_lvl.reg32 = AUTO_WAKE_LVL_RESET;
    s->auto_wake_lvl_mask.reg32 = AUTO_WAKE_LVL_MASK_RESET;
    s->wake_delay.reg32 = WAKE_DELAY_RESET;
    s->usb_debounce_del.reg32 = USB_DEBOUNCE_DEL_RESET;
    s->crypto_op.reg32 = CRYPTO_OP_RESET;
    s->bondout_mirror0.reg32 = BONDOUT_MIRROR0_RESET;
    s->bondout_mirror1.reg32 = BONDOUT_MIRROR1_RESET;
    s->bondout_mirror2.reg32 = BONDOUT_MIRROR2_RESET;
    s->sys_33v_en.reg32 = SYS_33V_EN_RESET;
    s->bondout_mirror_access.reg32 = BONDOUT_MIRROR_ACCESS_RESET;
    s->gate.reg32 = GATE_RESET;

    // Skip init for SCRATCH-regs/default-zero regs for guest-reset. TODO: Is this correct / is there more regs to skip etc?
    if (cause != SHUTDOWN_CAUSE_GUEST_RESET) {
        s->scratch0.reg32 = SCRATCH0_RESET;
        s->scratch1.reg32 = SCRATCH1_RESET;
        s->scratch2.reg32 = SCRATCH2_RESET;
        s->scratch3.reg32 = SCRATCH3_RESET;
        s->scratch4.reg32 = SCRATCH4_RESET;
        s->scratch5.reg32 = SCRATCH5_RESET;
        s->scratch6.reg32 = SCRATCH6_RESET;
        s->scratch7.reg32 = SCRATCH7_RESET;
        s->scratch8.reg32 = SCRATCH8_RESET;
        s->scratch9.reg32 = SCRATCH9_RESET;
        s->scratch10.reg32 = SCRATCH10_RESET;
        s->scratch11.reg32 = SCRATCH11_RESET;
        s->scratch12.reg32 = SCRATCH12_RESET;
        s->scratch13.reg32 = SCRATCH13_RESET;
        s->scratch14.reg32 = SCRATCH14_RESET;
        s->scratch15.reg32 = SCRATCH15_RESET;
        s->scratch16.reg32 = SCRATCH16_RESET;
        s->scratch17.reg32 = SCRATCH17_RESET;
        s->scratch18.reg32 = SCRATCH18_RESET;
        s->scratch19.reg32 = SCRATCH19_RESET;
        s->scratch20.reg32 = SCRATCH20_RESET;
        s->scratch21.reg32 = SCRATCH21_RESET;
        s->scratch22.reg32 = SCRATCH22_RESET;
        s->scratch23.reg32 = SCRATCH23_RESET;
        s->scratch24.reg32 = SCRATCH24_RESET;
        s->scratch25.reg32 = SCRATCH25_RESET;
        s->scratch26.reg32 = SCRATCH26_RESET;
        s->scratch27.reg32 = SCRATCH27_RESET;
        s->scratch28.reg32 = SCRATCH28_RESET;
        s->scratch29.reg32 = SCRATCH29_RESET;
        s->scratch30.reg32 = SCRATCH30_RESET;
        s->scratch31.reg32 = SCRATCH31_RESET;
        s->scratch32.reg32 = SCRATCH32_RESET;
        s->scratch33.reg32 = SCRATCH33_RESET;
        s->scratch34.reg32 = SCRATCH34_RESET;
        s->scratch35.reg32 = SCRATCH35_RESET;
        s->scratch36.reg32 = SCRATCH36_RESET;
        s->scratch37.reg32 = SCRATCH37_RESET;
        s->scratch38.reg32 = SCRATCH38_RESET;
        s->scratch39.reg32 = SCRATCH39_RESET;
        s->scratch40.reg32 = SCRATCH40_RESET;
        s->scratch41.reg32 = SCRATCH41_RESET;
        s->scratch42.reg32 = SCRATCH42_RESET;
        s->secure_scratch0.reg32 = SECURE_SCRATCH0_RESET;
        s->secure_scratch1.reg32 = SECURE_SCRATCH1_RESET;
        s->secure_scratch2.reg32 = SECURE_SCRATCH2_RESET;
        s->secure_scratch3.reg32 = SECURE_SCRATCH3_RESET;
        s->secure_scratch4.reg32 = SECURE_SCRATCH4_RESET;
        s->secure_scratch5.reg32 = SECURE_SCRATCH5_RESET;

        /* Set ODMDATA for UARTD */
        s->scratch20.reg32 = (3 << 18) | (3 << 15);

        memset(s->regs, 0, sizeof(s->regs));
    }
    else {
        memset(s->regs, 0, RST_STATUS_OFFSET-0x160);
        memset(&s->regs[(RST_STATUS_OFFSET+0x4-0x160)>>2], 0, BONDOUT_MIRROR3_OFFSET-(RST_STATUS_OFFSET+0x4));
        memset(&s->regs[(UTMIP_UHSIC_LINE_WAKEUP_OFFSET-0x160)>>2], 0, SECURE_SCRATCH8_OFFSET-UTMIP_UHSIC_LINE_WAKEUP_OFFSET);
        memset(&s->regs[(CNTRL2_OFFSET-0x160)>>2], 0, SCRATCH56_OFFSET-CNTRL2_OFFSET);
    }

    if (tegra_board >= TEGRAX1_BOARD) {
        s->cntrl.reg32 = CNTRL_TEGRAX1_RESET;
        s->wake_lvl.reg32 = WAKE_LVL_TEGRAX1_RESET;
        s->pwrgate_timer_off.reg32 = PWRGATE_TIMER_OFF_TEGRAX1_RESET;
        s->pwrgood_timer.reg32 = PWRGOOD_TIMER_TEGRAX1_RESET;
        s->no_iopower.reg32 = NO_IOPOWER_TEGRAX1_RESET;
        s->pwr_det.reg32 = PWR_DET_TEGRAX1_RESET;
        s->pg_mask_1.reg32 = PG_MASK_1_TEGRAX1_RESET;
        s->pwr_det_val.reg32 = PWR_DET_VAL_TEGRAX1_RESET;
        s->ddr_pwr.reg32 = DDR_PWR_TEGRAX1_RESET;
        s->usb_ao.reg32 = USB_AO_TEGRAX1_RESET;
        s->pllp_wb0_override.reg32 = PLLP_WB0_OVERRIDE_TEGRAX1_RESET;

        for (size_t i=0; i<sizeof(tegra_pmc_regdef_tegrax1_reset_table)/sizeof(uint32_t); i+=2) {
            s->regs[(tegra_pmc_regdef_tegrax1_reset_table[i]-0x160)>>2] = tegra_pmc_regdef_tegrax1_reset_table[i+1];
        }

        // Setup regs as validated by Erista NX_Bootloader with certain versions.
        s->pwr_det_val.reg32 = 0x007C3C2C;
        s->regs[(AOTAG_CFG_OFFSET-0x160)>>2] = 0x00000001;
    }
    else {
        s->cntrl.reg32 = CNTRL_TEGRA2_RESET;
        s->wake_lvl.reg32 = WAKE_LVL_TEGRA2_RESET;
        s->pwrgate_timer_off.reg32 = PWRGATE_TIMER_OFF_TEGRA2_RESET;
        s->pwrgood_timer.reg32 = PWRGOOD_TIMER_TEGRA2_RESET;
        s->no_iopower.reg32 = NO_IOPOWER_TEGRA2_RESET;
        s->pwr_det.reg32 = PWR_DET_TEGRA2_RESET;
        s->pg_mask_1.reg32 = PG_MASK_1_TEGRA2_RESET;
        s->pwr_det_val.reg32 = PWR_DET_VAL_TEGRA2_RESET;
        s->ddr_pwr.reg32 = DDR_PWR_TEGRA2_RESET;
        s->usb_ao.reg32 = USB_AO_TEGRA2_RESET;
        s->pllp_wb0_override.reg32 = PLLP_WB0_OVERRIDE_TEGRA2_RESET;
    }
}

static const MemoryRegionOps tegra_pmc_mem_ops = {
    .read = tegra_pmc_priv_read,
    .write = tegra_pmc_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_pmc_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_pmc *s = TEGRA_PMC(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_pmc_mem_ops, s,
                          "tegra.pmc", TEGRA_PMC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_pmc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_pmc_priv_realize;
    dc->vmsd = &vmstate_tegra_pmc;
}

static const TypeInfo tegra_pmc_info = {
    .name = TYPE_TEGRA_PMC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_pmc),
    .class_init = tegra_pmc_class_init,
};

static void tegra_pmc_register_types(void)
{
    type_register_static(&tegra_pmc_info);
}

type_init(tegra_pmc_register_types)
