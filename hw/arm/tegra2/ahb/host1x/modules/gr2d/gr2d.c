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

#include "gr2d.h"

#include "host1x_module.h"

#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_GR2D "tegra.gr2d"
#define TEGRA_GR2D(obj) OBJECT_CHECK(tegra_gr2d, (obj), TYPE_TEGRA_GR2D)

static const VMStateDescription vmstate_gr2d_ctx = {
    .name = "tegra-gr2d-ctx",
    .version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(g2sb_incr_syncpt_cntrl.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_incr_syncpt_error.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2classchannel_regonly.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2trigger.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2trigger1.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2trigger2.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2cmdsel.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2raise.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2hostset.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2hostfifo.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2vdda.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2vddaini.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2hdda.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2hddainils.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2cscfirst.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2cscsecond.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2cscthird.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2cmkeyl.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2cmkeyu.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2uba_a.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2vba_a.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2sbformat.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2controlsb.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2controlsecond.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2controlmain.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2ropfade.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2alphablend.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2cliplefttop.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2cliprightbot.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2patpack.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2patpack_size.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2patba.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2patos.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2patbgc.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2patfgc.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2patkey.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2dstba.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2dstba_b.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2dstba_c.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2dstst.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2srcpack.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2srcpack_size.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2srcba.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2srcba_b.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2srcst.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2srcbgc.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2srcfgc.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2srckey.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2srcsize.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2dstsize.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2srcps.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2dstps.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2cbdes.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2cbstride.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2linesetting.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2linedeltan.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2linedeltam.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2linepos.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2linelen.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2cscfourth.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2srcst_b.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2uvstride.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2cbdes2.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2tilemode.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2patbase.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2srcba_sb_surfbase.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2dstba_sb_surfbase.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2dstba_b_sb_surfbase.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2vba_a_sb_surfbase.reg32, gr2d_ctx),
        VMSTATE_UINT32(g2sb_g2uba_a_sb_surfbase.reg32, gr2d_ctx),
        VMSTATE_BOOL(transpose, gr2d_ctx),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_tegra_gr2d = {
    .name = "tegra.gr2d",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(regs.ctx, tegra_gr2d, 7, 0, vmstate_gr2d_ctx, gr2d_ctx),
        VMSTATE_UINT32(regs.g2sb_switch_g2interrupt.reg32, tegra_gr2d),
        VMSTATE_UINT32(regs.g2sb_switch_g2intenable.reg32, tegra_gr2d),
        VMSTATE_UINT32(regs.g2sb_switch_g2currentcontext.reg32, tegra_gr2d),
        VMSTATE_UINT32(regs.g2sb_switch_g2nxtcxtswitch.reg32, tegra_gr2d),
        VMSTATE_UINT32(regs.g2sb_switch_g2globalcontrol.reg32, tegra_gr2d),
        VMSTATE_UINT32(regs.g2sb_switch_g2globalcontrolb.reg32, tegra_gr2d),
        VMSTATE_UINT32(regs.g2sb_switch_g2workingstat.reg32, tegra_gr2d),
        VMSTATE_UINT32(regs.g2sb_switch_g2bufthreshold.reg32, tegra_gr2d),
        VMSTATE_UINT32(regs.g2sb_switch_clken_overide.reg32, tegra_gr2d),
        VMSTATE_UINT32(regs.g2sb_switch_g2_mccif_fifoctrl.reg32, tegra_gr2d),
        VMSTATE_UINT32(regs.g2sb_switch_timeout_wcoal_g2.reg32, tegra_gr2d),
        VMSTATE_UINT32(regs.falcon_addr, tegra_gr2d),
        VMSTATE_UINT32_ARRAY(regs.priv, tegra_gr2d, 0x22C18>>2),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_gr2d_priv_read(void *opaque, hwaddr offset, unsigned size)
{
    tegra_gr2d *s = opaque;

    return host1x_module_read(&s->gr2d_module[0], offset >> 2);
}

static void tegra_gr2d_priv_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    tegra_gr2d *s = opaque;

    host1x_module_write(&s->gr2d_module[0], offset >> 2, value);
}

static void tegra_gr2d_priv_reset(DeviceState *dev)
{
    tegra_gr2d *s = TEGRA_GR2D(dev);
    gr2d_regs *regs = &s->regs;
    int i;

    for (i = 0; i < ARRAY_SIZE(s->regs.ctx); i++) {
        gr2d_ctx *ctx = &s->regs.ctx[i];

        ctx->g2sb_incr_syncpt_cntrl.reg32 = G2SB_INCR_SYNCPT_CNTRL_RESET;
        ctx->g2sb_incr_syncpt_error.reg32 = G2SB_INCR_SYNCPT_ERROR_RESET;
        ctx->g2sb_g2classchannel_regonly.reg32 = G2SB_G2CLASSCHANNEL_REGONLY_RESET;
        ctx->g2sb_g2trigger.reg32 = G2SB_G2TRIGGER_RESET;
        ctx->g2sb_g2trigger1.reg32 = G2SB_G2TRIGGER1_RESET;
        ctx->g2sb_g2trigger2.reg32 = G2SB_G2TRIGGER2_RESET;
        ctx->g2sb_g2cmdsel.reg32 = G2SB_G2CMDSEL_RESET;
        ctx->g2sb_g2raise.reg32 = G2SB_G2RAISE_RESET;
        ctx->g2sb_g2hostset.reg32 = G2SB_G2HOSTSET_RESET;
        ctx->g2sb_g2hostfifo.reg32 = G2SB_G2HOSTFIFO_RESET;
        ctx->g2sb_g2vdda.reg32 = G2SB_G2VDDA_RESET;
        ctx->g2sb_g2vddaini.reg32 = G2SB_G2VDDAINI_RESET;
        ctx->g2sb_g2hdda.reg32 = G2SB_G2HDDA_RESET;
        ctx->g2sb_g2hddainils.reg32 = G2SB_G2HDDAINILS_RESET;
        ctx->g2sb_g2cscfirst.reg32 = G2SB_G2CSCFIRST_RESET;
        ctx->g2sb_g2cscsecond.reg32 = G2SB_G2CSCSECOND_RESET;
        ctx->g2sb_g2cscthird.reg32 = G2SB_G2CSCTHIRD_RESET;
        ctx->g2sb_g2cmkeyl.reg32 = G2SB_G2CMKEYL_RESET;
        ctx->g2sb_g2cmkeyu.reg32 = G2SB_G2CMKEYU_RESET;
        ctx->g2sb_g2uba_a.reg32 = G2SB_G2UBA_A_RESET;
        ctx->g2sb_g2vba_a.reg32 = G2SB_G2VBA_A_RESET;
        ctx->g2sb_g2sbformat.reg32 = G2SB_G2SBFORMAT_RESET;
        ctx->g2sb_g2controlsb.reg32 = G2SB_G2CONTROLSB_RESET;
        ctx->g2sb_g2controlsecond.reg32 = G2SB_G2CONTROLSECOND_RESET;
        ctx->g2sb_g2controlmain.reg32 = G2SB_G2CONTROLMAIN_RESET;
        ctx->g2sb_g2ropfade.reg32 = G2SB_G2ROPFADE_RESET;
        ctx->g2sb_g2alphablend.reg32 = G2SB_G2ALPHABLEND_RESET;
        ctx->g2sb_g2cliplefttop.reg32 = G2SB_G2CLIPLEFTTOP_RESET;
        ctx->g2sb_g2cliprightbot.reg32 = G2SB_G2CLIPRIGHTBOT_RESET;
        ctx->g2sb_g2patpack.reg32 = G2SB_G2PATPACK_RESET;
        ctx->g2sb_g2patpack_size.reg32 = G2SB_G2PATPACK_SIZE_RESET;
        ctx->g2sb_g2patba.reg32 = G2SB_G2PATBA_RESET;
        ctx->g2sb_g2patos.reg32 = G2SB_G2PATOS_RESET;
        ctx->g2sb_g2patbgc.reg32 = G2SB_G2PATBGC_RESET;
        ctx->g2sb_g2patfgc.reg32 = G2SB_G2PATFGC_RESET;
        ctx->g2sb_g2patkey.reg32 = G2SB_G2PATKEY_RESET;
        ctx->g2sb_g2dstba.reg32 = G2SB_G2DSTBA_RESET;
        ctx->g2sb_g2dstba_b.reg32 = G2SB_G2DSTBA_B_RESET;
        ctx->g2sb_g2dstba_c.reg32 = G2SB_G2DSTBA_C_RESET;
        ctx->g2sb_g2dstst.reg32 = G2SB_G2DSTST_RESET;
        ctx->g2sb_g2srcpack.reg32 = G2SB_G2SRCPACK_RESET;
        ctx->g2sb_g2srcpack_size.reg32 = G2SB_G2SRCPACK_SIZE_RESET;
        ctx->g2sb_g2srcba.reg32 = G2SB_G2SRCBA_RESET;
        ctx->g2sb_g2srcba_b.reg32 = G2SB_G2SRCBA_B_RESET;
        ctx->g2sb_g2srcst.reg32 = G2SB_G2SRCST_RESET;
        ctx->g2sb_g2srcbgc.reg32 = G2SB_G2SRCBGC_RESET;
        ctx->g2sb_g2srcfgc.reg32 = G2SB_G2SRCFGC_RESET;
        ctx->g2sb_g2srckey.reg32 = G2SB_G2SRCKEY_RESET;
        ctx->g2sb_g2srcsize.reg32 = G2SB_G2SRCSIZE_RESET;
        ctx->g2sb_g2dstsize.reg32 = G2SB_G2DSTSIZE_RESET;
        ctx->g2sb_g2srcps.reg32 = G2SB_G2SRCPS_RESET;
        ctx->g2sb_g2dstps.reg32 = G2SB_G2DSTPS_RESET;
        ctx->g2sb_g2cbdes.reg32 = G2SB_G2CBDES_RESET;
        ctx->g2sb_g2cbstride.reg32 = G2SB_G2CBSTRIDE_RESET;
        ctx->g2sb_g2linesetting.reg32 = G2SB_G2LINESETTING_RESET;
        ctx->g2sb_g2linedeltan.reg32 = G2SB_G2LINEDELTAN_RESET;
        ctx->g2sb_g2linedeltam.reg32 = G2SB_G2LINEDELTAM_RESET;
        ctx->g2sb_g2linepos.reg32 = G2SB_G2LINEPOS_RESET;
        ctx->g2sb_g2linelen.reg32 = G2SB_G2LINELEN_RESET;
        ctx->g2sb_g2cscfourth.reg32 = G2SB_G2CSCFOURTH_RESET;
        ctx->g2sb_g2srcst_b.reg32 = G2SB_G2SRCST_B_RESET;
        ctx->g2sb_g2uvstride.reg32 = G2SB_G2UVSTRIDE_RESET;
        ctx->g2sb_g2cbdes2.reg32 = G2SB_G2CBDES2_RESET;
        ctx->g2sb_g2tilemode.reg32 = G2SB_G2TILEMODE_RESET;
        ctx->g2sb_g2patbase.reg32 = G2SB_G2PATBASE_RESET;
        ctx->g2sb_g2srcba_sb_surfbase.reg32 = G2SB_G2SRCBA_SB_SURFBASE_RESET;
        ctx->g2sb_g2dstba_sb_surfbase.reg32 = G2SB_G2DSTBA_SB_SURFBASE_RESET;
        ctx->g2sb_g2dstba_b_sb_surfbase.reg32 = G2SB_G2DSTBA_B_SB_SURFBASE_RESET;
        ctx->g2sb_g2vba_a_sb_surfbase.reg32 = G2SB_G2VBA_A_SB_SURFBASE_RESET;
        ctx->g2sb_g2uba_a_sb_surfbase.reg32 = G2SB_G2UBA_A_SB_SURFBASE_RESET;
    }

    regs->g2sb_switch_g2interrupt.reg32 = G2SB_SWITCH_G2INTERRUPT_RESET;
    regs->g2sb_switch_g2intenable.reg32 = G2SB_SWITCH_G2INTENABLE_RESET;
    regs->g2sb_switch_g2currentcontext.reg32 = G2SB_SWITCH_G2CURRENTCONTEXT_RESET;
    regs->g2sb_switch_g2nxtcxtswitch.reg32 = G2SB_SWITCH_G2NXTCXTSWITCH_RESET;
    regs->g2sb_switch_g2globalcontrol.reg32 = G2SB_SWITCH_G2GLOBALCONTROL_RESET;
    regs->g2sb_switch_g2globalcontrolb.reg32 = G2SB_SWITCH_G2GLOBALCONTROLB_RESET;
    regs->g2sb_switch_g2workingstat.reg32 = G2SB_SWITCH_G2WORKINGSTAT_RESET;
    regs->g2sb_switch_g2bufthreshold.reg32 = G2SB_SWITCH_G2BUFTHRESHOLD_RESET;
    regs->g2sb_switch_clken_overide.reg32 = G2SB_SWITCH_CLKEN_OVERIDE_RESET;
    regs->g2sb_switch_g2_mccif_fifoctrl.reg32 = G2SB_SWITCH_G2_MCCIF_FIFOCTRL_RESET;
    regs->g2sb_switch_timeout_wcoal_g2.reg32 = G2SB_SWITCH_TIMEOUT_WCOAL_G2_RESET;
    regs->falcon_addr = 0;
    memset(regs->priv, 0, sizeof(regs->priv));
}

static const MemoryRegionOps tegra_gr2d_mem_ops = {
    .read = tegra_gr2d_priv_read,
    .write = tegra_gr2d_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_gr2d_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_gr2d *s = TEGRA_GR2D(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_gr2d_mem_ops, s,
                          "tegra.gr2d", SZ_256K);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    s->gr2d_module[0].class_id = tegra_board >= TEGRAX1_BOARD ? 0x5D : 0x50,
    s->gr2d_module[0].reg_write = gr2d_write;
    s->gr2d_module[0].reg_read = gr2d_read;
    register_host1x_bus_module(&s->gr2d_module[0], &s->regs);

    if (tegra_board >= TEGRAX1_BOARD) return;

    s->gr2d_module[1].class_id = 0x51,
    s->gr2d_module[1].reg_write = gr2d_write;
    s->gr2d_module[1].reg_read = gr2d_read;
    register_host1x_bus_module(&s->gr2d_module[1], &s->regs);

    s->gr2d_module[2].class_id = 0x54,
    s->gr2d_module[2].reg_write = gr2d_write;
    s->gr2d_module[2].reg_read = gr2d_read;
    register_host1x_bus_module(&s->gr2d_module[2], &s->regs);

    s->gr2d_module[3].class_id = 0x55,
    s->gr2d_module[3].reg_write = gr2d_write;
    s->gr2d_module[3].reg_read = gr2d_read;
    register_host1x_bus_module(&s->gr2d_module[3], &s->regs);

    s->gr2d_module[4].class_id = 0x56,
    s->gr2d_module[4].reg_write = gr2d_write;
    s->gr2d_module[4].reg_read = gr2d_read;
    register_host1x_bus_module(&s->gr2d_module[4], &s->regs);

    s->gr2d_sb_module[0].class_id = 0x52,
    s->gr2d_sb_module[0].reg_write = gr2d_write;
    s->gr2d_sb_module[0].reg_read = gr2d_read;
    register_host1x_bus_module(&s->gr2d_sb_module[0], &s->regs);

    s->gr2d_sb_module[1].class_id = 0x58,
    s->gr2d_sb_module[1].reg_write = gr2d_write;
    s->gr2d_sb_module[1].reg_read = gr2d_read;
    register_host1x_bus_module(&s->gr2d_sb_module[1], &s->regs);

    s->gr2d_sb_module[2].class_id = 0x5a,
    s->gr2d_sb_module[2].reg_write = gr2d_write;
    s->gr2d_sb_module[2].reg_read = gr2d_read;
    register_host1x_bus_module(&s->gr2d_sb_module[2], &s->regs);
}

static void tegra_gr2d_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_gr2d_priv_realize;
    dc->vmsd = &vmstate_tegra_gr2d;
    dc->reset = tegra_gr2d_priv_reset;
}

static const TypeInfo tegra_gr2d_info = {
    .name = TYPE_TEGRA_GR2D,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_gr2d),
    .class_init = tegra_gr2d_class_init,
};

static void tegra_gr2d_register_types(void)
{
    type_register_static(&tegra_gr2d_info);
}

type_init(tegra_gr2d_register_types)
