/*
 * ARM NVIDIA Tegra2 emulation.
 *
 * Copyright (c) 2015 Dmitry Osipenko <digetx@gmail.com>
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

#include "clk_rst.h"
#include "tegra_trace.h"

#include "vde.h"

static uint64_t tegra_frameid_read(void *opaque, hwaddr offset,
                                 unsigned size)
{
    tegra_frameid *s = opaque;
    uint32_t ret = s->regs[offset >> 2];
    int rst_set = tegra_rst_asserted(TEGRA20_CLK_VDE);
    int clk_en = tegra_clk_enabled(TEGRA20_CLK_VDE);

    if (!clk_en) {
        ret = 1;
    }

    TRACE_READ_EXT(s->iomem.addr, offset, ret, !clk_en, rst_set);

    return ret;
}

static void tegra_frameid_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    tegra_frameid *s = opaque;
    int rst_set = tegra_rst_asserted(TEGRA20_CLK_VDE);
    int clk_en = tegra_clk_enabled(TEGRA20_CLK_VDE);

    TRACE_WRITE_EXT(s->iomem.addr, offset, s->regs[offset >> 2], value,
                    !clk_en, rst_set);

    if (tegra_rst_asserted(TEGRA20_CLK_VDE)) {
        return;
    }

    if (!tegra_clk_enabled(TEGRA20_CLK_VDE)) {
        return;
    }

    s->regs[offset >> 2] = value;
}

static const MemoryRegionOps tegra_frameid_mem_ops = {
    .read = tegra_frameid_read,
    .write = tegra_frameid_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_vde_frameid_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_frameid *s = TEGRA_VDE_FRAMEID(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &tegra_frameid_mem_ops, s,
                          "tegra.vde_frameid", 768);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_vde_frameid_priv_reset(DeviceState *dev)
{
    tegra_frameid *s = TEGRA_VDE_FRAMEID(dev);
    int i;

    for (i = 0; i < 768; i++) {
        s->regs[i] = 0;
    }

    s->regs[0] = 0x00300980;
    s->regs[1] = 0x002FC8A0;
    s->regs[2] = 0x002FF3E0;
    s->regs[3] = 0x802C1214;
    s->regs[4] = 0x80144221;
    s->regs[5] = 0x808A2840;
    s->regs[6] = 0x0002A566;
    s->regs[7] = 0x80518290;
    s->regs[8] = 0x80848216;
    s->regs[9] = 0x80144199;
    s->regs[10] = 0x8002651C;
    s->regs[11] = 0x80868232;
    s->regs[12] = 0x805B9035;
    s->regs[13] = 0x805208BD;
    s->regs[14] = 0x80100180;
    s->regs[15] = 0x800C0313;
    s->regs[16] = 0x8020786B;
    s->regs[17] = 0x80E90460;
    s->regs[18] = 0x80011921;
    s->regs[19] = 0x80430B3A;
    s->regs[20] = 0x80100001;
    s->regs[21] = 0x80CA0069;
    s->regs[22] = 0x80490008;
    s->regs[23] = 0x80580026;
    s->regs[24] = 0x80580026;
    s->regs[25] = 0x80580026;
    s->regs[26] = 0x80580026;
    s->regs[27] = 0x80580026;
    s->regs[28] = 0x80580026;
    s->regs[29] = 0x80580026;
    s->regs[30] = 0x80580026;
    s->regs[31] = 0x80580026;
    s->regs[32] = 0x0050002D;
    s->regs[33] = 0x0050002D;
    s->regs[34] = 0x0050002D;
    s->regs[35] = 0x00120301;
    s->regs[36] = 0x00530010;
    s->regs[37] = 0x01890041;
    s->regs[38] = 0x037C009F;
    s->regs[39] = 0x03810154;
    s->regs[40] = 0x00320202;
    s->regs[41] = 0x00320202;
    s->regs[42] = 0x00320202;
    s->regs[43] = 0x00320202;
    s->regs[44] = 0x00320202;
    s->regs[45] = 0x00320202;
    s->regs[46] = 0x00320202;
    s->regs[47] = 0x00320202;
    s->regs[48] = 0x00320202;
    s->regs[49] = 0x00320202;
    s->regs[50] = 0x00320202;
    s->regs[51] = 0x00320202;
    s->regs[52] = 0x00320202;
    s->regs[53] = 0x00320202;
    s->regs[54] = 0x00320202;
    s->regs[55] = 0x00320202;
    s->regs[56] = 0x00320202;
    s->regs[57] = 0x00320202;
    s->regs[58] = 0x00320202;
    s->regs[59] = 0x00320202;
    s->regs[60] = 0x00320202;
    s->regs[61] = 0x00320202;
    s->regs[62] = 0x00320202;
    s->regs[63] = 0x00320202;
    s->regs[64] = 0x003017E6;
    s->regs[65] = 0x002FD706;
    s->regs[66] = 0x00300246;
    s->regs[67] = 0x0001843C;
    s->regs[68] = 0x00000310;
    s->regs[69] = 0x0002A507;
    s->regs[70] = 0x00CA400E;
    s->regs[71] = 0x00C421D2;
    s->regs[72] = 0x008A08C1;
    s->regs[73] = 0x00240011;
    s->regs[74] = 0x00404000;
    s->regs[75] = 0x000107A0;
    s->regs[76] = 0x00061842;
    s->regs[77] = 0x00E66088;
    s->regs[78] = 0x00D0C892;
    s->regs[79] = 0x00EF418A;
    s->regs[80] = 0x00030208;
    s->regs[81] = 0x0060200A;
    s->regs[82] = 0x00105980;
    s->regs[83] = 0x00587516;
    s->regs[84] = 0x00870004;
    s->regs[85] = 0x00386088;
    s->regs[86] = 0x0034E361;
    s->regs[87] = 0x00C1A008;
    s->regs[88] = 0x00C1A008;
    s->regs[89] = 0x00C1A008;
    s->regs[90] = 0x00C1A008;
    s->regs[91] = 0x00C1A008;
    s->regs[92] = 0x00C1A008;
    s->regs[93] = 0x00C1A008;
    s->regs[94] = 0x00C1A008;
    s->regs[95] = 0x00C1A008;
    s->regs[96] = 0x00301B81;
    s->regs[97] = 0x002FDAA1;
    s->regs[98] = 0x003005E1;
    s->regs[99] = 0x00A24E2A;
    s->regs[100] = 0x001805C0;
    s->regs[101] = 0x00614851;
    s->regs[102] = 0x00424C91;
    s->regs[103] = 0x00E58040;
    s->regs[104] = 0x00064004;
    s->regs[105] = 0x00140348;
    s->regs[106] = 0x00402CF4;
    s->regs[107] = 0x00201658;
    s->regs[108] = 0x00414802;
    s->regs[109] = 0x00082090;
    s->regs[110] = 0x000122A5;
    s->regs[111] = 0x00A0100E;
    s->regs[112] = 0x00800920;
    s->regs[113] = 0x00A00A16;
    s->regs[114] = 0x0000045B;
    s->regs[115] = 0x00810A2E;
    s->regs[116] = 0x00844488;
    s->regs[117] = 0x00240038;
    s->regs[118] = 0x0048B912;
    s->regs[119] = 0x00480E93;
    s->regs[120] = 0x00480E93;
    s->regs[121] = 0x00480E93;
    s->regs[122] = 0x00480E93;
    s->regs[123] = 0x00480E93;
    s->regs[124] = 0x00480E93;
    s->regs[125] = 0x00480E93;
    s->regs[126] = 0x00480E93;
    s->regs[127] = 0x00480E93;
    s->regs[128] = 0x00000007;
    s->regs[160] = 0x00000A00;
    s->regs[161] = 0x00000A00;
    s->regs[162] = 0x00000A00;
    s->regs[163] = 0x00008000;
    s->regs[164] = 0x00000120;
    s->regs[165] = 0x00005050;
    s->regs[166] = 0x00002900;
    s->regs[167] = 0x00001200;
    s->regs[168] = 0x00003C80;
    s->regs[169] = 0x0000A080;
    s->regs[170] = 0x00009190;
    s->regs[171] = 0x00006420;
    s->regs[172] = 0x0000A800;
    s->regs[173] = 0x00000520;
    s->regs[175] = 0x00000040;
    s->regs[176] = 0x0000C110;
    s->regs[177] = 0x0000E000;
    s->regs[178] = 0x0000C100;
    s->regs[179] = 0x00004400;
    s->regs[180] = 0x00008850;
    s->regs[181] = 0x00006030;
    s->regs[182] = 0x00000290;
    s->regs[183] = 0x00001AD0;
    s->regs[184] = 0x00001AD0;
    s->regs[185] = 0x00001AD0;
    s->regs[186] = 0x00001AD0;
    s->regs[187] = 0x00001AD0;
    s->regs[188] = 0x00001AD0;
    s->regs[189] = 0x00001AD0;
    s->regs[190] = 0x00001AD0;
    s->regs[191] = 0x00001AD0;
}

static void tegra_vde_frameid_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_vde_frameid_priv_realize;
    device_class_set_legacy_reset(dc, tegra_vde_frameid_priv_reset);
}

static const TypeInfo tegra_frameid_info = {
    .name = TYPE_TEGRA_VDE_FRAMEID,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_frameid),
    .class_init = tegra_vde_frameid_class_init,
};

static void tegra_frameid_register_types(void)
{
    type_register_static(&tegra_frameid_info);
}

type_init(tegra_frameid_register_types)
