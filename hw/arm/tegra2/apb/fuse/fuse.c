/*
 * ARM NVIDIA Tegra2/X1 emulation.
 *
 * Copyright (c) 2014-2015 Dmitry Osipenko <digetx@gmail.com>
 * Copyright (c) yellows8
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

#include "fuse.h"
#include "iomap.h"
#include "tegra_trace.h"

#include "crypto/secret_common.h"
#include "qapi/error.h"

#define TYPE_TEGRA_FUSE "tegra.fuse"
#define TEGRA_FUSE(obj) OBJECT_CHECK(tegra_fuse, (obj), TYPE_TEGRA_FUSE)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_fuse_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    DEFINE_REG32(fuse_fusebypass);
    DEFINE_REG32(fuse_write_access_sw);
    DEFINE_REG32(fuse_jtag_secureid_0);
    DEFINE_REG32(fuse_jtag_secureid_1);
    DEFINE_REG32(fuse_sku_info);
    DEFINE_REG32(fuse_process_calib);
    DEFINE_REG32(fuse_io_calib);
    DEFINE_REG32(fuse_dac_crt_calib);
    DEFINE_REG32(fuse_dac_hdtv_calib);
    DEFINE_REG32(fuse_dac_sdtv_calib);
    DEFINE_REG32(fuse_reserved_production);
    DEFINE_REG32(fuse_reserved_odm0);
    DEFINE_REG32(fuse_reserved_odm1);
    DEFINE_REG32(fuse_reserved_odm2);
    DEFINE_REG32(fuse_reserved_odm3);
    DEFINE_REG32(fuse_reserved_odm4);
    DEFINE_REG32(fuse_reserved_odm5);
    DEFINE_REG32(fuse_reserved_odm6);
    DEFINE_REG32(fuse_reserved_odm7);
    DEFINE_REG32(fuse_spare_bit_0);
    DEFINE_REG32(fuse_spare_bit_1);
    DEFINE_REG32(fuse_spare_bit_2);
    DEFINE_REG32(fuse_spare_bit_3);
    DEFINE_REG32(fuse_spare_bit_4);
    DEFINE_REG32(fuse_spare_bit_5);
    DEFINE_REG32(fuse_spare_bit_6);
    DEFINE_REG32(fuse_spare_bit_7);
    DEFINE_REG32(fuse_spare_bit_8);
    DEFINE_REG32(fuse_spare_bit_9);
    DEFINE_REG32(fuse_spare_bit_10);
    DEFINE_REG32(fuse_spare_bit_11);
    DEFINE_REG32(fuse_spare_bit_12);
    DEFINE_REG32(fuse_spare_bit_13);
    DEFINE_REG32(fuse_spare_bit_14);
    DEFINE_REG32(fuse_spare_bit_15);
    DEFINE_REG32(fuse_spare_bit_16);
    DEFINE_REG32(fuse_spare_bit_17);
    DEFINE_REG32(fuse_spare_bit_18);
    DEFINE_REG32(fuse_spare_bit_19);
    DEFINE_REG32(fuse_spare_bit_20);
    DEFINE_REG32(fuse_spare_bit_21);
    DEFINE_REG32(fuse_spare_bit_22);
    DEFINE_REG32(fuse_spare_bit_23);
    DEFINE_REG32(fuse_spare_bit_24);
    DEFINE_REG32(fuse_spare_bit_25);
    DEFINE_REG32(fuse_spare_bit_26);
    DEFINE_REG32(fuse_spare_bit_27);
    DEFINE_REG32(fuse_spare_bit_28);
    DEFINE_REG32(fuse_spare_bit_29);
    DEFINE_REG32(fuse_spare_bit_30);
    DEFINE_REG32(fuse_spare_bit_31);
    DEFINE_REG32(fuse_spare_bit_32);
    DEFINE_REG32(fuse_spare_bit_33);
    DEFINE_REG32(fuse_spare_bit_34);
    DEFINE_REG32(fuse_spare_bit_35);
    DEFINE_REG32(fuse_spare_bit_36);
    DEFINE_REG32(fuse_spare_bit_37);
    DEFINE_REG32(fuse_spare_bit_38);
    DEFINE_REG32(fuse_spare_bit_39);
    DEFINE_REG32(fuse_spare_bit_40);
    DEFINE_REG32(fuse_spare_bit_41);
    DEFINE_REG32(fuse_spare_bit_42);
    DEFINE_REG32(fuse_spare_bit_43);
    DEFINE_REG32(fuse_spare_bit_44);
    DEFINE_REG32(fuse_spare_bit_45);
    DEFINE_REG32(fuse_spare_bit_46);
    DEFINE_REG32(fuse_spare_bit_47);
    DEFINE_REG32(fuse_spare_bit_48);
    DEFINE_REG32(fuse_spare_bit_49);
    DEFINE_REG32(fuse_spare_bit_50);
    DEFINE_REG32(fuse_spare_bit_51);
    DEFINE_REG32(fuse_spare_bit_52);
    DEFINE_REG32(fuse_spare_bit_53);
    DEFINE_REG32(fuse_spare_bit_54);
    DEFINE_REG32(fuse_spare_bit_55);
    DEFINE_REG32(fuse_spare_bit_56);
    DEFINE_REG32(fuse_spare_bit_57);
    DEFINE_REG32(fuse_spare_bit_58);
    DEFINE_REG32(fuse_spare_bit_59);
    DEFINE_REG32(fuse_spare_bit_60);
    DEFINE_REG32(fuse_spare_bit_61);
} tegra_fuse;

static const VMStateDescription vmstate_tegra_fuse = {
    .name = "tegra.fuse",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(fuse_fusebypass.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_write_access_sw.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_jtag_secureid_0.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_jtag_secureid_1.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_sku_info.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_process_calib.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_io_calib.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_dac_crt_calib.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_dac_hdtv_calib.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_dac_sdtv_calib.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_reserved_production.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_reserved_odm0.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_reserved_odm1.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_reserved_odm2.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_reserved_odm3.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_reserved_odm4.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_reserved_odm5.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_reserved_odm6.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_reserved_odm7.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_0.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_1.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_2.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_3.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_4.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_5.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_6.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_7.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_8.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_9.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_10.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_11.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_12.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_13.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_14.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_15.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_16.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_17.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_18.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_19.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_20.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_21.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_22.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_23.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_24.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_25.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_26.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_27.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_28.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_29.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_30.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_31.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_32.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_33.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_34.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_35.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_36.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_37.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_38.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_39.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_40.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_41.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_42.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_43.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_44.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_45.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_46.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_47.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_48.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_49.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_50.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_51.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_52.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_53.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_54.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_55.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_56.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_57.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_58.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_59.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_60.reg32, tegra_fuse),
        VMSTATE_UINT32(fuse_spare_bit_61.reg32, tegra_fuse),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_fuse_priv_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    tegra_fuse *s = opaque;
    uint64_t ret = 0;

    switch (offset) {
    case FUSE_FUSEBYPASS_OFFSET:
        ret = s->fuse_fusebypass.reg32;
        break;
    case FUSE_WRITE_ACCESS_SW_OFFSET:
        ret = s->fuse_write_access_sw.reg32;
        break;
    case FUSE_JTAG_SECUREID_0_OFFSET:
        ret = s->fuse_jtag_secureid_0.reg32;
        break;
    case FUSE_JTAG_SECUREID_1_OFFSET:
        ret = s->fuse_jtag_secureid_1.reg32;
        break;
    case FUSE_SKU_INFO_OFFSET:
        ret = s->fuse_sku_info.reg32;
        break;
    case FUSE_PROCESS_CALIB_OFFSET:
        ret = s->fuse_process_calib.reg32;
        break;
    case FUSE_IO_CALIB_OFFSET:
        ret = s->fuse_io_calib.reg32;
        break;
    case FUSE_DAC_CRT_CALIB_OFFSET:
        ret = s->fuse_dac_crt_calib.reg32;
        break;
    case FUSE_DAC_HDTV_CALIB_OFFSET:
        ret = s->fuse_dac_hdtv_calib.reg32;
        break;
    case FUSE_DAC_SDTV_CALIB_OFFSET:
        ret = s->fuse_dac_sdtv_calib.reg32;
        break;
    case FUSE_RESERVED_PRODUCTION_OFFSET:
        ret = s->fuse_reserved_production.reg32;
        break;
    case FUSE_RESERVED_ODM0_OFFSET:
        ret = s->fuse_reserved_odm0.reg32;
        break;
    case FUSE_RESERVED_ODM1_OFFSET:
        ret = s->fuse_reserved_odm1.reg32;
        break;
    case FUSE_RESERVED_ODM2_OFFSET:
        ret = s->fuse_reserved_odm2.reg32;
        break;
    case FUSE_RESERVED_ODM3_OFFSET:
        ret = s->fuse_reserved_odm3.reg32;
        break;
    case FUSE_RESERVED_ODM4_OFFSET:
        ret = s->fuse_reserved_odm4.reg32;
        break;
    case FUSE_RESERVED_ODM5_OFFSET:
        ret = s->fuse_reserved_odm5.reg32;
        break;
    case FUSE_RESERVED_ODM6_OFFSET:
        ret = s->fuse_reserved_odm6.reg32;
        break;
    case FUSE_RESERVED_ODM7_OFFSET:
        ret = s->fuse_reserved_odm7.reg32;
        break;
    case FUSE_SPARE_BIT_0_OFFSET:
        ret = s->fuse_spare_bit_0.reg32;
        break;
    case FUSE_SPARE_BIT_1_OFFSET:
        ret = s->fuse_spare_bit_1.reg32;
        break;
    case FUSE_SPARE_BIT_2_OFFSET:
        ret = s->fuse_spare_bit_2.reg32;
        break;
    case FUSE_SPARE_BIT_3_OFFSET:
        ret = s->fuse_spare_bit_3.reg32;
        break;
    case FUSE_SPARE_BIT_4_OFFSET:
        ret = s->fuse_spare_bit_4.reg32;
        break;
    case FUSE_SPARE_BIT_5_OFFSET:
        ret = s->fuse_spare_bit_5.reg32;
        break;
    case FUSE_SPARE_BIT_6_OFFSET:
        ret = s->fuse_spare_bit_6.reg32;
        break;
    case FUSE_SPARE_BIT_7_OFFSET:
        ret = s->fuse_spare_bit_7.reg32;
        break;
    case FUSE_SPARE_BIT_8_OFFSET:
        ret = s->fuse_spare_bit_8.reg32;
        break;
    case FUSE_SPARE_BIT_9_OFFSET:
        ret = s->fuse_spare_bit_9.reg32;
        break;
    case FUSE_SPARE_BIT_10_OFFSET:
        ret = s->fuse_spare_bit_10.reg32;
        break;
    case FUSE_SPARE_BIT_11_OFFSET:
        ret = s->fuse_spare_bit_11.reg32;
        break;
    case FUSE_SPARE_BIT_12_OFFSET:
        ret = s->fuse_spare_bit_12.reg32;
        break;
    case FUSE_SPARE_BIT_13_OFFSET:
        ret = s->fuse_spare_bit_13.reg32;
        break;
    case FUSE_SPARE_BIT_14_OFFSET:
        ret = s->fuse_spare_bit_14.reg32;
        break;
    case FUSE_SPARE_BIT_15_OFFSET:
        ret = s->fuse_spare_bit_15.reg32;
        break;
    case FUSE_SPARE_BIT_16_OFFSET:
        ret = s->fuse_spare_bit_16.reg32;
        break;
    case FUSE_SPARE_BIT_17_OFFSET:
        ret = s->fuse_spare_bit_17.reg32;
        break;
    case FUSE_SPARE_BIT_18_OFFSET:
        ret = s->fuse_spare_bit_18.reg32;
        break;
    case FUSE_SPARE_BIT_19_OFFSET:
        ret = s->fuse_spare_bit_19.reg32;
        break;
    case FUSE_SPARE_BIT_20_OFFSET:
        ret = s->fuse_spare_bit_20.reg32;
        break;
    case FUSE_SPARE_BIT_21_OFFSET:
        ret = s->fuse_spare_bit_21.reg32;
        break;
    case FUSE_SPARE_BIT_22_OFFSET:
        ret = s->fuse_spare_bit_22.reg32;
        break;
    case FUSE_SPARE_BIT_23_OFFSET:
        ret = s->fuse_spare_bit_23.reg32;
        break;
    case FUSE_SPARE_BIT_24_OFFSET:
        ret = s->fuse_spare_bit_24.reg32;
        break;
    case FUSE_SPARE_BIT_25_OFFSET:
        ret = s->fuse_spare_bit_25.reg32;
        break;
    case FUSE_SPARE_BIT_26_OFFSET:
        ret = s->fuse_spare_bit_26.reg32;
        break;
    case FUSE_SPARE_BIT_27_OFFSET:
        ret = s->fuse_spare_bit_27.reg32;
        break;
    case FUSE_SPARE_BIT_28_OFFSET:
        ret = s->fuse_spare_bit_28.reg32;
        break;
    case FUSE_SPARE_BIT_29_OFFSET:
        ret = s->fuse_spare_bit_29.reg32;
        break;
    case FUSE_SPARE_BIT_30_OFFSET:
        ret = s->fuse_spare_bit_30.reg32;
        break;
    case FUSE_SPARE_BIT_31_OFFSET:
        ret = s->fuse_spare_bit_31.reg32;
        break;
    case FUSE_SPARE_BIT_32_OFFSET:
        ret = s->fuse_spare_bit_32.reg32;
        break;
    case FUSE_SPARE_BIT_33_OFFSET:
        ret = s->fuse_spare_bit_33.reg32;
        break;
    case FUSE_SPARE_BIT_34_OFFSET:
        ret = s->fuse_spare_bit_34.reg32;
        break;
    case FUSE_SPARE_BIT_35_OFFSET:
        ret = s->fuse_spare_bit_35.reg32;
        break;
    case FUSE_SPARE_BIT_36_OFFSET:
        ret = s->fuse_spare_bit_36.reg32;
        break;
    case FUSE_SPARE_BIT_37_OFFSET:
        ret = s->fuse_spare_bit_37.reg32;
        break;
    case FUSE_SPARE_BIT_38_OFFSET:
        ret = s->fuse_spare_bit_38.reg32;
        break;
    case FUSE_SPARE_BIT_39_OFFSET:
        ret = s->fuse_spare_bit_39.reg32;
        break;
    case FUSE_SPARE_BIT_40_OFFSET:
        ret = s->fuse_spare_bit_40.reg32;
        break;
    case FUSE_SPARE_BIT_41_OFFSET:
        ret = s->fuse_spare_bit_41.reg32;
        break;
    case FUSE_SPARE_BIT_42_OFFSET:
        ret = s->fuse_spare_bit_42.reg32;
        break;
    case FUSE_SPARE_BIT_43_OFFSET:
        ret = s->fuse_spare_bit_43.reg32;
        break;
    case FUSE_SPARE_BIT_44_OFFSET:
        ret = s->fuse_spare_bit_44.reg32;
        break;
    case FUSE_SPARE_BIT_45_OFFSET:
        ret = s->fuse_spare_bit_45.reg32;
        break;
    case FUSE_SPARE_BIT_46_OFFSET:
        ret = s->fuse_spare_bit_46.reg32;
        break;
    case FUSE_SPARE_BIT_47_OFFSET:
        ret = s->fuse_spare_bit_47.reg32;
        break;
    case FUSE_SPARE_BIT_48_OFFSET:
        ret = s->fuse_spare_bit_48.reg32;
        break;
    case FUSE_SPARE_BIT_49_OFFSET:
        ret = s->fuse_spare_bit_49.reg32;
        break;
    case FUSE_SPARE_BIT_50_OFFSET:
        ret = s->fuse_spare_bit_50.reg32;
        break;
    case FUSE_SPARE_BIT_51_OFFSET:
        ret = s->fuse_spare_bit_51.reg32;
        break;
    case FUSE_SPARE_BIT_52_OFFSET:
        ret = s->fuse_spare_bit_52.reg32;
        break;
    case FUSE_SPARE_BIT_53_OFFSET:
        ret = s->fuse_spare_bit_53.reg32;
        break;
    case FUSE_SPARE_BIT_54_OFFSET:
        ret = s->fuse_spare_bit_54.reg32;
        break;
    case FUSE_SPARE_BIT_55_OFFSET:
        ret = s->fuse_spare_bit_55.reg32;
        break;
    case FUSE_SPARE_BIT_56_OFFSET:
        ret = s->fuse_spare_bit_56.reg32;
        break;
    case FUSE_SPARE_BIT_57_OFFSET:
        ret = s->fuse_spare_bit_57.reg32;
        break;
    case FUSE_SPARE_BIT_58_OFFSET:
        ret = s->fuse_spare_bit_58.reg32;
        break;
    case FUSE_SPARE_BIT_59_OFFSET:
        ret = s->fuse_spare_bit_59.reg32;
        break;
    case FUSE_SPARE_BIT_60_OFFSET:
        ret = s->fuse_spare_bit_60.reg32;
        break;
    case FUSE_SPARE_BIT_61_OFFSET:
        ret = s->fuse_spare_bit_61.reg32;
        break;
    default:
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_fuse_priv_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    tegra_fuse *s = opaque;

    switch (offset) {
    case FUSE_FUSEBYPASS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_fusebypass.reg32, value);
        s->fuse_fusebypass.reg32 = value;
        break;
    case FUSE_WRITE_ACCESS_SW_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_write_access_sw.reg32, value);
        s->fuse_write_access_sw.reg32 = value;
        break;
    case FUSE_JTAG_SECUREID_0_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_jtag_secureid_0.reg32, value);
        s->fuse_jtag_secureid_0.reg32 = value;
        break;
    case FUSE_JTAG_SECUREID_1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_jtag_secureid_1.reg32, value);
        s->fuse_jtag_secureid_1.reg32 = value;
        break;
    case FUSE_SKU_INFO_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_sku_info.reg32, value);
        s->fuse_sku_info.reg32 = value;
        break;
    case FUSE_PROCESS_CALIB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_process_calib.reg32, value);
        s->fuse_process_calib.reg32 = value;
        break;
    case FUSE_IO_CALIB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_io_calib.reg32, value);
        s->fuse_io_calib.reg32 = value;
        break;
    case FUSE_DAC_CRT_CALIB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_dac_crt_calib.reg32, value);
        s->fuse_dac_crt_calib.reg32 = value;
        break;
    case FUSE_DAC_HDTV_CALIB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_dac_hdtv_calib.reg32, value);
        s->fuse_dac_hdtv_calib.reg32 = value;
        break;
    case FUSE_DAC_SDTV_CALIB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_dac_sdtv_calib.reg32, value);
        s->fuse_dac_sdtv_calib.reg32 = value;
        break;
    case FUSE_RESERVED_PRODUCTION_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_reserved_production.reg32, value);
        s->fuse_reserved_production.reg32 = value;
        break;
    case FUSE_RESERVED_ODM0_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_reserved_odm0.reg32, value);
        s->fuse_reserved_odm0.reg32 = value;
        break;
    case FUSE_RESERVED_ODM1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_reserved_odm1.reg32, value);
        s->fuse_reserved_odm1.reg32 = value;
        break;
    case FUSE_RESERVED_ODM2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_reserved_odm2.reg32, value);
        s->fuse_reserved_odm2.reg32 = value;
        break;
    case FUSE_RESERVED_ODM3_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_reserved_odm3.reg32, value);
        s->fuse_reserved_odm3.reg32 = value;
        break;
    case FUSE_RESERVED_ODM4_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_reserved_odm4.reg32, value);
        s->fuse_reserved_odm4.reg32 = value;
        break;
    case FUSE_RESERVED_ODM5_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_reserved_odm5.reg32, value);
        s->fuse_reserved_odm5.reg32 = value;
        break;
    case FUSE_RESERVED_ODM6_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_reserved_odm6.reg32, value);
        s->fuse_reserved_odm6.reg32 = value;
        break;
    case FUSE_RESERVED_ODM7_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_reserved_odm7.reg32, value);
        s->fuse_reserved_odm7.reg32 = value;
        break;
    case FUSE_SPARE_BIT_0_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_0.reg32, value & FUSE_SPARE_BIT_0_WRMASK);
        WR_MASKED(s->fuse_spare_bit_0.reg32, value, FUSE_SPARE_BIT_0);
        break;
    case FUSE_SPARE_BIT_1_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_1.reg32, value & FUSE_SPARE_BIT_1_WRMASK);
        WR_MASKED(s->fuse_spare_bit_1.reg32, value, FUSE_SPARE_BIT_1);
        break;
    case FUSE_SPARE_BIT_2_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_2.reg32, value & FUSE_SPARE_BIT_2_WRMASK);
        WR_MASKED(s->fuse_spare_bit_2.reg32, value, FUSE_SPARE_BIT_2);
        break;
    case FUSE_SPARE_BIT_3_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_3.reg32, value & FUSE_SPARE_BIT_3_WRMASK);
        WR_MASKED(s->fuse_spare_bit_3.reg32, value, FUSE_SPARE_BIT_3);
        break;
    case FUSE_SPARE_BIT_4_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_4.reg32, value & FUSE_SPARE_BIT_4_WRMASK);
        WR_MASKED(s->fuse_spare_bit_4.reg32, value, FUSE_SPARE_BIT_4);
        break;
    case FUSE_SPARE_BIT_5_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_5.reg32, value & FUSE_SPARE_BIT_5_WRMASK);
        WR_MASKED(s->fuse_spare_bit_5.reg32, value, FUSE_SPARE_BIT_5);
        break;
    case FUSE_SPARE_BIT_6_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_6.reg32, value & FUSE_SPARE_BIT_6_WRMASK);
        WR_MASKED(s->fuse_spare_bit_6.reg32, value, FUSE_SPARE_BIT_6);
        break;
    case FUSE_SPARE_BIT_7_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_7.reg32, value & FUSE_SPARE_BIT_7_WRMASK);
        WR_MASKED(s->fuse_spare_bit_7.reg32, value, FUSE_SPARE_BIT_7);
        break;
    case FUSE_SPARE_BIT_8_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_8.reg32, value & FUSE_SPARE_BIT_8_WRMASK);
        WR_MASKED(s->fuse_spare_bit_8.reg32, value, FUSE_SPARE_BIT_8);
        break;
    case FUSE_SPARE_BIT_9_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_9.reg32, value & FUSE_SPARE_BIT_9_WRMASK);
        WR_MASKED(s->fuse_spare_bit_9.reg32, value, FUSE_SPARE_BIT_9);
        break;
    case FUSE_SPARE_BIT_10_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_10.reg32, value & FUSE_SPARE_BIT_10_WRMASK);
        WR_MASKED(s->fuse_spare_bit_10.reg32, value, FUSE_SPARE_BIT_10);
        break;
    case FUSE_SPARE_BIT_11_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_11.reg32, value & FUSE_SPARE_BIT_11_WRMASK);
        WR_MASKED(s->fuse_spare_bit_11.reg32, value, FUSE_SPARE_BIT_11);
        break;
    case FUSE_SPARE_BIT_12_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_12.reg32, value & FUSE_SPARE_BIT_12_WRMASK);
        WR_MASKED(s->fuse_spare_bit_12.reg32, value, FUSE_SPARE_BIT_12);
        break;
    case FUSE_SPARE_BIT_13_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_13.reg32, value & FUSE_SPARE_BIT_13_WRMASK);
        WR_MASKED(s->fuse_spare_bit_13.reg32, value, FUSE_SPARE_BIT_13);
        break;
    case FUSE_SPARE_BIT_14_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_14.reg32, value & FUSE_SPARE_BIT_14_WRMASK);
        WR_MASKED(s->fuse_spare_bit_14.reg32, value, FUSE_SPARE_BIT_14);
        break;
    case FUSE_SPARE_BIT_15_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_15.reg32, value & FUSE_SPARE_BIT_15_WRMASK);
        WR_MASKED(s->fuse_spare_bit_15.reg32, value, FUSE_SPARE_BIT_15);
        break;
    case FUSE_SPARE_BIT_16_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_16.reg32, value & FUSE_SPARE_BIT_16_WRMASK);
        WR_MASKED(s->fuse_spare_bit_16.reg32, value, FUSE_SPARE_BIT_16);
        break;
    case FUSE_SPARE_BIT_17_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_17.reg32, value & FUSE_SPARE_BIT_17_WRMASK);
        WR_MASKED(s->fuse_spare_bit_17.reg32, value, FUSE_SPARE_BIT_17);
        break;
    case FUSE_SPARE_BIT_18_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_18.reg32, value & FUSE_SPARE_BIT_18_WRMASK);
        WR_MASKED(s->fuse_spare_bit_18.reg32, value, FUSE_SPARE_BIT_18);
        break;
    case FUSE_SPARE_BIT_19_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_19.reg32, value & FUSE_SPARE_BIT_19_WRMASK);
        WR_MASKED(s->fuse_spare_bit_19.reg32, value, FUSE_SPARE_BIT_19);
        break;
    case FUSE_SPARE_BIT_20_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_20.reg32, value & FUSE_SPARE_BIT_20_WRMASK);
        WR_MASKED(s->fuse_spare_bit_20.reg32, value, FUSE_SPARE_BIT_20);
        break;
    case FUSE_SPARE_BIT_21_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_21.reg32, value & FUSE_SPARE_BIT_21_WRMASK);
        WR_MASKED(s->fuse_spare_bit_21.reg32, value, FUSE_SPARE_BIT_21);
        break;
    case FUSE_SPARE_BIT_22_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_22.reg32, value & FUSE_SPARE_BIT_22_WRMASK);
        WR_MASKED(s->fuse_spare_bit_22.reg32, value, FUSE_SPARE_BIT_22);
        break;
    case FUSE_SPARE_BIT_23_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_23.reg32, value & FUSE_SPARE_BIT_23_WRMASK);
        WR_MASKED(s->fuse_spare_bit_23.reg32, value, FUSE_SPARE_BIT_23);
        break;
    case FUSE_SPARE_BIT_24_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_24.reg32, value & FUSE_SPARE_BIT_24_WRMASK);
        WR_MASKED(s->fuse_spare_bit_24.reg32, value, FUSE_SPARE_BIT_24);
        break;
    case FUSE_SPARE_BIT_25_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_25.reg32, value & FUSE_SPARE_BIT_25_WRMASK);
        WR_MASKED(s->fuse_spare_bit_25.reg32, value, FUSE_SPARE_BIT_25);
        break;
    case FUSE_SPARE_BIT_26_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_26.reg32, value & FUSE_SPARE_BIT_26_WRMASK);
        WR_MASKED(s->fuse_spare_bit_26.reg32, value, FUSE_SPARE_BIT_26);
        break;
    case FUSE_SPARE_BIT_27_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_27.reg32, value & FUSE_SPARE_BIT_27_WRMASK);
        WR_MASKED(s->fuse_spare_bit_27.reg32, value, FUSE_SPARE_BIT_27);
        break;
    case FUSE_SPARE_BIT_28_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_28.reg32, value & FUSE_SPARE_BIT_28_WRMASK);
        WR_MASKED(s->fuse_spare_bit_28.reg32, value, FUSE_SPARE_BIT_28);
        break;
    case FUSE_SPARE_BIT_29_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_29.reg32, value & FUSE_SPARE_BIT_29_WRMASK);
        WR_MASKED(s->fuse_spare_bit_29.reg32, value, FUSE_SPARE_BIT_29);
        break;
    case FUSE_SPARE_BIT_30_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_30.reg32, value & FUSE_SPARE_BIT_30_WRMASK);
        WR_MASKED(s->fuse_spare_bit_30.reg32, value, FUSE_SPARE_BIT_30);
        break;
    case FUSE_SPARE_BIT_31_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_31.reg32, value & FUSE_SPARE_BIT_31_WRMASK);
        WR_MASKED(s->fuse_spare_bit_31.reg32, value, FUSE_SPARE_BIT_31);
        break;
    case FUSE_SPARE_BIT_32_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_32.reg32, value & FUSE_SPARE_BIT_32_WRMASK);
        WR_MASKED(s->fuse_spare_bit_32.reg32, value, FUSE_SPARE_BIT_32);
        break;
    case FUSE_SPARE_BIT_33_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_33.reg32, value & FUSE_SPARE_BIT_33_WRMASK);
        WR_MASKED(s->fuse_spare_bit_33.reg32, value, FUSE_SPARE_BIT_33);
        break;
    case FUSE_SPARE_BIT_34_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_34.reg32, value & FUSE_SPARE_BIT_34_WRMASK);
        WR_MASKED(s->fuse_spare_bit_34.reg32, value, FUSE_SPARE_BIT_34);
        break;
    case FUSE_SPARE_BIT_35_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_35.reg32, value & FUSE_SPARE_BIT_35_WRMASK);
        WR_MASKED(s->fuse_spare_bit_35.reg32, value, FUSE_SPARE_BIT_35);
        break;
    case FUSE_SPARE_BIT_36_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_36.reg32, value & FUSE_SPARE_BIT_36_WRMASK);
        WR_MASKED(s->fuse_spare_bit_36.reg32, value, FUSE_SPARE_BIT_36);
        break;
    case FUSE_SPARE_BIT_37_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_37.reg32, value & FUSE_SPARE_BIT_37_WRMASK);
        WR_MASKED(s->fuse_spare_bit_37.reg32, value, FUSE_SPARE_BIT_37);
        break;
    case FUSE_SPARE_BIT_38_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_38.reg32, value & FUSE_SPARE_BIT_38_WRMASK);
        WR_MASKED(s->fuse_spare_bit_38.reg32, value, FUSE_SPARE_BIT_38);
        break;
    case FUSE_SPARE_BIT_39_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_39.reg32, value & FUSE_SPARE_BIT_39_WRMASK);
        WR_MASKED(s->fuse_spare_bit_39.reg32, value, FUSE_SPARE_BIT_39);
        break;
    case FUSE_SPARE_BIT_40_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_40.reg32, value & FUSE_SPARE_BIT_40_WRMASK);
        WR_MASKED(s->fuse_spare_bit_40.reg32, value, FUSE_SPARE_BIT_40);
        break;
    case FUSE_SPARE_BIT_41_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_41.reg32, value & FUSE_SPARE_BIT_41_WRMASK);
        WR_MASKED(s->fuse_spare_bit_41.reg32, value, FUSE_SPARE_BIT_41);
        break;
    case FUSE_SPARE_BIT_42_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_42.reg32, value & FUSE_SPARE_BIT_42_WRMASK);
        WR_MASKED(s->fuse_spare_bit_42.reg32, value, FUSE_SPARE_BIT_42);
        break;
    case FUSE_SPARE_BIT_43_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_43.reg32, value & FUSE_SPARE_BIT_43_WRMASK);
        WR_MASKED(s->fuse_spare_bit_43.reg32, value, FUSE_SPARE_BIT_43);
        break;
    case FUSE_SPARE_BIT_44_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_44.reg32, value & FUSE_SPARE_BIT_44_WRMASK);
        WR_MASKED(s->fuse_spare_bit_44.reg32, value, FUSE_SPARE_BIT_44);
        break;
    case FUSE_SPARE_BIT_45_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_45.reg32, value & FUSE_SPARE_BIT_45_WRMASK);
        WR_MASKED(s->fuse_spare_bit_45.reg32, value, FUSE_SPARE_BIT_45);
        break;
    case FUSE_SPARE_BIT_46_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_46.reg32, value & FUSE_SPARE_BIT_46_WRMASK);
        WR_MASKED(s->fuse_spare_bit_46.reg32, value, FUSE_SPARE_BIT_46);
        break;
    case FUSE_SPARE_BIT_47_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_47.reg32, value & FUSE_SPARE_BIT_47_WRMASK);
        WR_MASKED(s->fuse_spare_bit_47.reg32, value, FUSE_SPARE_BIT_47);
        break;
    case FUSE_SPARE_BIT_48_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_48.reg32, value & FUSE_SPARE_BIT_48_WRMASK);
        WR_MASKED(s->fuse_spare_bit_48.reg32, value, FUSE_SPARE_BIT_48);
        break;
    case FUSE_SPARE_BIT_49_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_49.reg32, value & FUSE_SPARE_BIT_49_WRMASK);
        WR_MASKED(s->fuse_spare_bit_49.reg32, value, FUSE_SPARE_BIT_49);
        break;
    case FUSE_SPARE_BIT_50_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_50.reg32, value & FUSE_SPARE_BIT_50_WRMASK);
        WR_MASKED(s->fuse_spare_bit_50.reg32, value, FUSE_SPARE_BIT_50);
        break;
    case FUSE_SPARE_BIT_51_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_51.reg32, value & FUSE_SPARE_BIT_51_WRMASK);
        WR_MASKED(s->fuse_spare_bit_51.reg32, value, FUSE_SPARE_BIT_51);
        break;
    case FUSE_SPARE_BIT_52_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_52.reg32, value & FUSE_SPARE_BIT_52_WRMASK);
        WR_MASKED(s->fuse_spare_bit_52.reg32, value, FUSE_SPARE_BIT_52);
        break;
    case FUSE_SPARE_BIT_53_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_53.reg32, value & FUSE_SPARE_BIT_53_WRMASK);
        WR_MASKED(s->fuse_spare_bit_53.reg32, value, FUSE_SPARE_BIT_53);
        break;
    case FUSE_SPARE_BIT_54_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_54.reg32, value & FUSE_SPARE_BIT_54_WRMASK);
        WR_MASKED(s->fuse_spare_bit_54.reg32, value, FUSE_SPARE_BIT_54);
        break;
    case FUSE_SPARE_BIT_55_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_55.reg32, value & FUSE_SPARE_BIT_55_WRMASK);
        WR_MASKED(s->fuse_spare_bit_55.reg32, value, FUSE_SPARE_BIT_55);
        break;
    case FUSE_SPARE_BIT_56_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_56.reg32, value & FUSE_SPARE_BIT_56_WRMASK);
        WR_MASKED(s->fuse_spare_bit_56.reg32, value, FUSE_SPARE_BIT_56);
        break;
    case FUSE_SPARE_BIT_57_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_57.reg32, value & FUSE_SPARE_BIT_57_WRMASK);
        WR_MASKED(s->fuse_spare_bit_57.reg32, value, FUSE_SPARE_BIT_57);
        break;
    case FUSE_SPARE_BIT_58_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_58.reg32, value & FUSE_SPARE_BIT_58_WRMASK);
        WR_MASKED(s->fuse_spare_bit_58.reg32, value, FUSE_SPARE_BIT_58);
        break;
    case FUSE_SPARE_BIT_59_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_59.reg32, value & FUSE_SPARE_BIT_59_WRMASK);
        WR_MASKED(s->fuse_spare_bit_59.reg32, value, FUSE_SPARE_BIT_59);
        break;
    case FUSE_SPARE_BIT_60_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_60.reg32, value & FUSE_SPARE_BIT_60_WRMASK);
        WR_MASKED(s->fuse_spare_bit_60.reg32, value, FUSE_SPARE_BIT_60);
        break;
    case FUSE_SPARE_BIT_61_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->fuse_spare_bit_61.reg32, value & FUSE_SPARE_BIT_61_WRMASK);
        WR_MASKED(s->fuse_spare_bit_61.reg32, value, FUSE_SPARE_BIT_61);
        break;
    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }
}

static void tegra_fuse_priv_reset(DeviceState *dev)
{
    tegra_fuse *s = TEGRA_FUSE(dev);

    s->fuse_fusebypass.reg32 = 0x1;
    s->fuse_write_access_sw.reg32 = 0x0;
    s->fuse_jtag_secureid_0.reg32 = FUSE_JTAG_SECUREID_0_RESET;
    s->fuse_jtag_secureid_1.reg32 = FUSE_JTAG_SECUREID_1_RESET;
    s->fuse_sku_info.reg32 = 0x8;
    s->fuse_process_calib.reg32 = FUSE_PROCESS_CALIB_RESET;
    s->fuse_io_calib.reg32 = FUSE_IO_CALIB_RESET;
    s->fuse_dac_crt_calib.reg32 = FUSE_DAC_CRT_CALIB_RESET;
    s->fuse_dac_hdtv_calib.reg32 = FUSE_DAC_HDTV_CALIB_RESET;
    s->fuse_dac_sdtv_calib.reg32 = 0xC8;
    s->fuse_reserved_production.reg32 = 0x2;
    s->fuse_reserved_odm0.reg32 = 0x0;
    s->fuse_reserved_odm1.reg32 = 0x0;
    s->fuse_reserved_odm2.reg32 = 0x0;
    s->fuse_reserved_odm3.reg32 = 0x0;
    s->fuse_reserved_odm4.reg32 = 0x0;
    s->fuse_reserved_odm5.reg32 = 0x0;
    s->fuse_reserved_odm6.reg32 = 0x0;
    s->fuse_reserved_odm7.reg32 = 0x0;
    s->fuse_spare_bit_0.reg32 = FUSE_SPARE_BIT_0_RESET;
    s->fuse_spare_bit_1.reg32 = FUSE_SPARE_BIT_1_RESET;
    s->fuse_spare_bit_2.reg32 = FUSE_SPARE_BIT_2_RESET;
    s->fuse_spare_bit_3.reg32 = FUSE_SPARE_BIT_3_RESET;
    s->fuse_spare_bit_4.reg32 = FUSE_SPARE_BIT_4_RESET;
    s->fuse_spare_bit_5.reg32 = FUSE_SPARE_BIT_5_RESET;
    s->fuse_spare_bit_6.reg32 = FUSE_SPARE_BIT_6_RESET;
    s->fuse_spare_bit_7.reg32 = 0x1;
    s->fuse_spare_bit_8.reg32 = FUSE_SPARE_BIT_8_RESET;
    s->fuse_spare_bit_9.reg32 = FUSE_SPARE_BIT_9_RESET;
    s->fuse_spare_bit_10.reg32 = 0x1;
    s->fuse_spare_bit_11.reg32 = FUSE_SPARE_BIT_11_RESET;
    s->fuse_spare_bit_12.reg32 = 0x1;
    s->fuse_spare_bit_13.reg32 = FUSE_SPARE_BIT_13_RESET;
    s->fuse_spare_bit_14.reg32 = FUSE_SPARE_BIT_14_RESET;
    s->fuse_spare_bit_15.reg32 = 0x1;
    s->fuse_spare_bit_16.reg32 = FUSE_SPARE_BIT_16_RESET;
    s->fuse_spare_bit_17.reg32 = FUSE_SPARE_BIT_17_RESET;
    s->fuse_spare_bit_18.reg32 = 0x1;
    s->fuse_spare_bit_19.reg32 = 0x1;
    s->fuse_spare_bit_20.reg32 = FUSE_SPARE_BIT_20_RESET;
    s->fuse_spare_bit_21.reg32 = FUSE_SPARE_BIT_21_RESET;
    s->fuse_spare_bit_22.reg32 = 0x1;
    s->fuse_spare_bit_23.reg32 = FUSE_SPARE_BIT_23_RESET;
    s->fuse_spare_bit_24.reg32 = 0x1;
    s->fuse_spare_bit_25.reg32 = FUSE_SPARE_BIT_25_RESET;
    s->fuse_spare_bit_26.reg32 = 0x1;
    s->fuse_spare_bit_27.reg32 = FUSE_SPARE_BIT_27_RESET;
    s->fuse_spare_bit_28.reg32 = FUSE_SPARE_BIT_28_RESET;
    s->fuse_spare_bit_29.reg32 = FUSE_SPARE_BIT_29_RESET;
    s->fuse_spare_bit_30.reg32 = FUSE_SPARE_BIT_30_RESET;
    s->fuse_spare_bit_31.reg32 = FUSE_SPARE_BIT_31_RESET;
    s->fuse_spare_bit_32.reg32 = 0x1;
    s->fuse_spare_bit_33.reg32 = FUSE_SPARE_BIT_33_RESET;
    s->fuse_spare_bit_34.reg32 = 0x1;
    s->fuse_spare_bit_35.reg32 = FUSE_SPARE_BIT_35_RESET;
    s->fuse_spare_bit_36.reg32 = 0x1;
    s->fuse_spare_bit_37.reg32 = FUSE_SPARE_BIT_37_RESET;
    s->fuse_spare_bit_38.reg32 = FUSE_SPARE_BIT_38_RESET;
    s->fuse_spare_bit_39.reg32 = FUSE_SPARE_BIT_39_RESET;
    s->fuse_spare_bit_40.reg32 = FUSE_SPARE_BIT_40_RESET;
    s->fuse_spare_bit_41.reg32 = FUSE_SPARE_BIT_41_RESET;
    s->fuse_spare_bit_42.reg32 = 0x1;
    s->fuse_spare_bit_43.reg32 = FUSE_SPARE_BIT_43_RESET;
    s->fuse_spare_bit_44.reg32 = 0x1;
    s->fuse_spare_bit_45.reg32 = 0x1;
    s->fuse_spare_bit_46.reg32 = FUSE_SPARE_BIT_46_RESET;
    s->fuse_spare_bit_47.reg32 = FUSE_SPARE_BIT_47_RESET;
    s->fuse_spare_bit_48.reg32 = FUSE_SPARE_BIT_48_RESET;
    s->fuse_spare_bit_49.reg32 = FUSE_SPARE_BIT_49_RESET;
    s->fuse_spare_bit_50.reg32 = 0x1;
    s->fuse_spare_bit_51.reg32 = FUSE_SPARE_BIT_51_RESET;
    s->fuse_spare_bit_52.reg32 = 0x1;
    s->fuse_spare_bit_53.reg32 = 0x1;
    s->fuse_spare_bit_54.reg32 = FUSE_SPARE_BIT_54_RESET;
    s->fuse_spare_bit_55.reg32 = FUSE_SPARE_BIT_55_RESET;
    s->fuse_spare_bit_56.reg32 = FUSE_SPARE_BIT_56_RESET;
    s->fuse_spare_bit_57.reg32 = FUSE_SPARE_BIT_57_RESET;
    s->fuse_spare_bit_58.reg32 = FUSE_SPARE_BIT_58_RESET;
    s->fuse_spare_bit_59.reg32 = FUSE_SPARE_BIT_59_RESET;
    s->fuse_spare_bit_60.reg32 = FUSE_SPARE_BIT_60_RESET;
    s->fuse_spare_bit_61.reg32 = FUSE_SPARE_BIT_61_RESET;

    // Load the fuse_cache secret into the fuse-cache regs. End-of-file == end of fuse regs. Loading cache regs from iopage is also supported.
    Error *err = NULL;
    uint8_t *data=NULL;
    size_t datalen = 0;
    if (qcrypto_secret_lookup("fuse_cache", &data, &datalen, &err)==0) {
        if (datalen > 0x368 && datalen!=0x1000) {
            error_setg(&err, "Fuse: Invalid datalen for secret fuse_cache, datalen=0x%lx expected <=0x368/0x1000.", datalen);
        }
        else {
            uint32_t *dataptr = (uint32_t*)data;
            size_t datacount = datalen/4;
            size_t databaseoff = 0;

            if (datalen==0x1000) {
                databaseoff = 0x898>>2;
            }

            for (size_t offset=databaseoff; offset<datacount; offset++) {
                hwaddr hwoff = offset*4;
                if (datalen!=0x1000) hwoff = 0x400-datalen+offset*4;
                else hwoff-= 0x800;
                tegra_fuse_priv_write(s, hwoff, dataptr[offset], 4);
            }
        }
        g_free(data);
    }
    if (err) error_report_err(err);
}

static const MemoryRegionOps tegra_fuse_mem_ops = {
    .read = tegra_fuse_priv_read,
    .write = tegra_fuse_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_fuse_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_fuse *s = TEGRA_FUSE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_fuse_mem_ops, s,
                          "tegra.fuse", TEGRA_FUSE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_fuse_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_fuse_priv_realize;
    dc->vmsd = &vmstate_tegra_fuse;
    dc->reset = tegra_fuse_priv_reset;
}

static const TypeInfo tegra_fuse_info = {
    .name = TYPE_TEGRA_FUSE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_fuse),
    .class_init = tegra_fuse_class_init,
};

static void tegra_fuse_register_types(void)
{
    type_register_static(&tegra_fuse_info);
}

type_init(tegra_fuse_register_types)
