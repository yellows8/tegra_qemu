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

/* Autogenerated from TRM v02p */

#ifndef TEGRA_FUSE_H
#define TEGRA_FUSE_H

#define FUSE_FUSEBYPASS_OFFSET 0x24
#define FUSE_FUSEBYPASS_RESET  0x00000000
typedef union fuse_fusebypass_u {
    struct {
        unsigned int fusebypass_val:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_fusebypass_t;

#define FUSE_WRITE_ACCESS_SW_OFFSET 0x30
#define FUSE_WRITE_ACCESS_SW_RESET  0x00000001
typedef union fuse_write_access_sw_u {
    struct {
        unsigned int write_access_sw_ctrl:1;
        unsigned int undefined_bits_1_15:15;
        unsigned int write_access_sw_status:1;
        unsigned int undefined_bits_17_31:15;
    };

    uint32_t reg32;
} fuse_write_access_sw_t;

#define FUSE_JTAG_SECUREID_0_OFFSET 0x108
#define FUSE_JTAG_SECUREID_0_RESET  0x00000000
typedef union fuse_jtag_secureid_0_u {
    struct {
        unsigned int jtag_secureid_0:32;
    };

    uint32_t reg32;
} fuse_jtag_secureid_0_t;

#define FUSE_JTAG_SECUREID_1_OFFSET 0x10C
#define FUSE_JTAG_SECUREID_1_RESET  0x00000000
typedef union fuse_jtag_secureid_1_u {
    struct {
        unsigned int jtag_secureid_1:32;
    };

    uint32_t reg32;
} fuse_jtag_secureid_1_t;

#define FUSE_SKU_INFO_OFFSET 0x110
#define FUSE_SKU_INFO_RESET  0x00000000
typedef union fuse_sku_info_u {
    struct {
        unsigned int sku_info:8;
        unsigned int undefined_bits_8_31:24;
    };

    uint32_t reg32;
} fuse_sku_info_t;

#define FUSE_PROCESS_CALIB_OFFSET 0x114
#define FUSE_PROCESS_CALIB_RESET  0x00000000
typedef union fuse_process_calib_u {
    struct {
        unsigned int process_calib:2;
        unsigned int undefined_bits_2_31:30;
    };

    uint32_t reg32;
} fuse_process_calib_t;

#define FUSE_IO_CALIB_OFFSET 0x118
#define FUSE_IO_CALIB_RESET  0x00000000
typedef union fuse_io_calib_u {
    struct {
        unsigned int io_calib:10;
        unsigned int undefined_bits_10_31:22;
    };

    uint32_t reg32;
} fuse_io_calib_t;

#define FUSE_DAC_CRT_CALIB_OFFSET 0x11C
#define FUSE_DAC_CRT_CALIB_RESET  0x00000000
typedef union fuse_dac_crt_calib_u {
    struct {
        unsigned int dac_crt_calib:8;
        unsigned int undefined_bits_8_31:24;
    };

    uint32_t reg32;
} fuse_dac_crt_calib_t;

#define FUSE_DAC_HDTV_CALIB_OFFSET 0x120
#define FUSE_DAC_HDTV_CALIB_RESET  0x00000000
typedef union fuse_dac_hdtv_calib_u {
    struct {
        unsigned int dac_hdtv_calib:8;
        unsigned int undefined_bits_8_31:24;
    };

    uint32_t reg32;
} fuse_dac_hdtv_calib_t;

#define FUSE_DAC_SDTV_CALIB_OFFSET 0x124
#define FUSE_DAC_SDTV_CALIB_RESET  0x00000000
typedef union fuse_dac_sdtv_calib_u {
    struct {
        unsigned int dac_sdtv_calib:8;
        unsigned int undefined_bits_8_31:24;
    };

    uint32_t reg32;
} fuse_dac_sdtv_calib_t;

#define FUSE_RESERVED_PRODUCTION_OFFSET 0x14C
#define FUSE_RESERVED_PRODUCTION_RESET  0x00000000
typedef union fuse_reserved_production_u {
    struct {
        unsigned int reserved_production:4;
        unsigned int undefined_bits_4_31:28;
    };

    uint32_t reg32;
} fuse_reserved_production_t;

#define FUSE_RESERVED_ODM0_OFFSET 0x1C8
#define FUSE_RESERVED_ODM0_RESET  0x00000000
typedef union fuse_reserved_odm0_u {
    uint32_t reg32;
} fuse_reserved_odm0_t;

#define FUSE_RESERVED_ODM1_OFFSET 0x1CC
#define FUSE_RESERVED_ODM1_RESET  0x00000000
typedef union fuse_reserved_odm1_u {
    uint32_t reg32;
} fuse_reserved_odm1_t;

#define FUSE_RESERVED_ODM2_OFFSET 0x1D0
#define FUSE_RESERVED_ODM2_RESET  0x00000000
typedef union fuse_reserved_odm2_u {
    uint32_t reg32;
} fuse_reserved_odm2_t;

#define FUSE_RESERVED_ODM3_OFFSET 0x1D4
#define FUSE_RESERVED_ODM3_RESET  0x00000000
typedef union fuse_reserved_odm3_u {
    uint32_t reg32;
} fuse_reserved_odm3_t;

#define FUSE_RESERVED_ODM4_OFFSET 0x1D8
#define FUSE_RESERVED_ODM4_RESET  0x00000000
typedef union fuse_reserved_odm4_u {
    uint32_t reg32;
} fuse_reserved_odm4_t;

#define FUSE_RESERVED_ODM5_OFFSET 0x1DC
#define FUSE_RESERVED_ODM5_RESET  0x00000000
typedef union fuse_reserved_odm5_u {
    uint32_t reg32;
} fuse_reserved_odm5_t;

#define FUSE_RESERVED_ODM6_OFFSET 0x1E0
#define FUSE_RESERVED_ODM6_RESET  0x00000000
typedef union fuse_reserved_odm6_u {
    uint32_t reg32;
} fuse_reserved_odm6_t;

#define FUSE_RESERVED_ODM7_OFFSET 0x1E4
#define FUSE_RESERVED_ODM7_RESET  0x00000000
typedef union fuse_reserved_odm7_u {
    uint32_t reg32;
} fuse_reserved_odm7_t;

#define FUSE_SPARE_BIT_0_OFFSET 0x200
#define FUSE_SPARE_BIT_0_RESET  0x00000000
#define FUSE_SPARE_BIT_0_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_0_u {
    struct {
        unsigned int spare_bit_0:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_0_t;

#define FUSE_SPARE_BIT_1_OFFSET 0x204
#define FUSE_SPARE_BIT_1_RESET  0x00000000
#define FUSE_SPARE_BIT_1_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_1_u {
    struct {
        unsigned int spare_bit_1:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_1_t;

#define FUSE_SPARE_BIT_2_OFFSET 0x208
#define FUSE_SPARE_BIT_2_RESET  0x00000000
#define FUSE_SPARE_BIT_2_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_2_u {
    struct {
        unsigned int spare_bit_2:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_2_t;

#define FUSE_SPARE_BIT_3_OFFSET 0x20C
#define FUSE_SPARE_BIT_3_RESET  0x00000000
#define FUSE_SPARE_BIT_3_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_3_u {
    struct {
        unsigned int spare_bit_3:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_3_t;

#define FUSE_SPARE_BIT_4_OFFSET 0x210
#define FUSE_SPARE_BIT_4_RESET  0x00000000
#define FUSE_SPARE_BIT_4_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_4_u {
    struct {
        unsigned int spare_bit_4:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_4_t;

#define FUSE_SPARE_BIT_5_OFFSET 0x214
#define FUSE_SPARE_BIT_5_RESET  0x00000000
#define FUSE_SPARE_BIT_5_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_5_u {
    struct {
        unsigned int spare_bit_5:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_5_t;

#define FUSE_SPARE_BIT_6_OFFSET 0x218
#define FUSE_SPARE_BIT_6_RESET  0x00000000
#define FUSE_SPARE_BIT_6_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_6_u {
    struct {
        unsigned int spare_bit_6:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_6_t;

#define FUSE_SPARE_BIT_7_OFFSET 0x21C
#define FUSE_SPARE_BIT_7_RESET  0x00000000
#define FUSE_SPARE_BIT_7_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_7_u {
    struct {
        unsigned int spare_bit_7:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_7_t;

#define FUSE_SPARE_BIT_8_OFFSET 0x220
#define FUSE_SPARE_BIT_8_RESET  0x00000000
#define FUSE_SPARE_BIT_8_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_8_u {
    struct {
        unsigned int spare_bit_8:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_8_t;

#define FUSE_SPARE_BIT_9_OFFSET 0x224
#define FUSE_SPARE_BIT_9_RESET  0x00000000
#define FUSE_SPARE_BIT_9_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_9_u {
    struct {
        unsigned int spare_bit_9:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_9_t;

#define FUSE_SPARE_BIT_10_OFFSET 0x228
#define FUSE_SPARE_BIT_10_RESET  0x00000000
#define FUSE_SPARE_BIT_10_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_10_u {
    struct {
        unsigned int spare_bit_10:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_10_t;

#define FUSE_SPARE_BIT_11_OFFSET 0x22C
#define FUSE_SPARE_BIT_11_RESET  0x00000000
#define FUSE_SPARE_BIT_11_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_11_u {
    struct {
        unsigned int spare_bit_11:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_11_t;

#define FUSE_SPARE_BIT_12_OFFSET 0x230
#define FUSE_SPARE_BIT_12_RESET  0x00000000
#define FUSE_SPARE_BIT_12_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_12_u {
    struct {
        unsigned int spare_bit_12:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_12_t;

#define FUSE_SPARE_BIT_13_OFFSET 0x234
#define FUSE_SPARE_BIT_13_RESET  0x00000000
#define FUSE_SPARE_BIT_13_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_13_u {
    struct {
        unsigned int spare_bit_13:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_13_t;

#define FUSE_SPARE_BIT_14_OFFSET 0x238
#define FUSE_SPARE_BIT_14_RESET  0x00000000
#define FUSE_SPARE_BIT_14_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_14_u {
    struct {
        unsigned int spare_bit_14:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_14_t;

#define FUSE_SPARE_BIT_15_OFFSET 0x23C
#define FUSE_SPARE_BIT_15_RESET  0x00000000
#define FUSE_SPARE_BIT_15_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_15_u {
    struct {
        unsigned int spare_bit_15:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_15_t;

#define FUSE_SPARE_BIT_16_OFFSET 0x240
#define FUSE_SPARE_BIT_16_RESET  0x00000000
#define FUSE_SPARE_BIT_16_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_16_u {
    struct {
        unsigned int spare_bit_16:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_16_t;

#define FUSE_SPARE_BIT_17_OFFSET 0x244
#define FUSE_SPARE_BIT_17_RESET  0x00000000
#define FUSE_SPARE_BIT_17_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_17_u {
    struct {
        unsigned int spare_bit_17:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_17_t;

#define FUSE_SPARE_BIT_18_OFFSET 0x248
#define FUSE_SPARE_BIT_18_RESET  0x00000000
#define FUSE_SPARE_BIT_18_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_18_u {
    struct {
        unsigned int spare_bit_18:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_18_t;

#define FUSE_SPARE_BIT_19_OFFSET 0x24C
#define FUSE_SPARE_BIT_19_RESET  0x00000000
#define FUSE_SPARE_BIT_19_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_19_u {
    struct {
        unsigned int spare_bit_19:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_19_t;

#define FUSE_SPARE_BIT_20_OFFSET 0x250
#define FUSE_SPARE_BIT_20_RESET  0x00000000
#define FUSE_SPARE_BIT_20_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_20_u {
    struct {
        unsigned int spare_bit_20:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_20_t;

#define FUSE_SPARE_BIT_21_OFFSET 0x254
#define FUSE_SPARE_BIT_21_RESET  0x00000000
#define FUSE_SPARE_BIT_21_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_21_u {
    struct {
        unsigned int spare_bit_21:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_21_t;

#define FUSE_SPARE_BIT_22_OFFSET 0x258
#define FUSE_SPARE_BIT_22_RESET  0x00000000
#define FUSE_SPARE_BIT_22_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_22_u {
    struct {
        unsigned int spare_bit_22:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_22_t;

#define FUSE_SPARE_BIT_23_OFFSET 0x25C
#define FUSE_SPARE_BIT_23_RESET  0x00000000
#define FUSE_SPARE_BIT_23_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_23_u {
    struct {
        unsigned int spare_bit_23:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_23_t;

#define FUSE_SPARE_BIT_24_OFFSET 0x260
#define FUSE_SPARE_BIT_24_RESET  0x00000000
#define FUSE_SPARE_BIT_24_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_24_u {
    struct {
        unsigned int spare_bit_24:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_24_t;

#define FUSE_SPARE_BIT_25_OFFSET 0x264
#define FUSE_SPARE_BIT_25_RESET  0x00000000
#define FUSE_SPARE_BIT_25_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_25_u {
    struct {
        unsigned int spare_bit_25:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_25_t;

#define FUSE_SPARE_BIT_26_OFFSET 0x268
#define FUSE_SPARE_BIT_26_RESET  0x00000000
#define FUSE_SPARE_BIT_26_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_26_u {
    struct {
        unsigned int spare_bit_26:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_26_t;

#define FUSE_SPARE_BIT_27_OFFSET 0x26C
#define FUSE_SPARE_BIT_27_RESET  0x00000000
#define FUSE_SPARE_BIT_27_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_27_u {
    struct {
        unsigned int spare_bit_27:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_27_t;

#define FUSE_SPARE_BIT_28_OFFSET 0x270
#define FUSE_SPARE_BIT_28_RESET  0x00000000
#define FUSE_SPARE_BIT_28_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_28_u {
    struct {
        unsigned int spare_bit_28:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_28_t;

#define FUSE_SPARE_BIT_29_OFFSET 0x274
#define FUSE_SPARE_BIT_29_RESET  0x00000000
#define FUSE_SPARE_BIT_29_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_29_u {
    struct {
        unsigned int spare_bit_29:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_29_t;

#define FUSE_SPARE_BIT_30_OFFSET 0x278
#define FUSE_SPARE_BIT_30_RESET  0x00000000
#define FUSE_SPARE_BIT_30_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_30_u {
    struct {
        unsigned int spare_bit_30:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_30_t;

#define FUSE_SPARE_BIT_31_OFFSET 0x27C
#define FUSE_SPARE_BIT_31_RESET  0x00000000
#define FUSE_SPARE_BIT_31_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_31_u {
    struct {
        unsigned int spare_bit_31:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_31_t;

#define FUSE_SPARE_BIT_32_OFFSET 0x280
#define FUSE_SPARE_BIT_32_RESET  0x00000000
#define FUSE_SPARE_BIT_32_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_32_u {
    struct {
        unsigned int spare_bit_32:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_32_t;

#define FUSE_SPARE_BIT_33_OFFSET 0x284
#define FUSE_SPARE_BIT_33_RESET  0x00000000
#define FUSE_SPARE_BIT_33_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_33_u {
    struct {
        unsigned int spare_bit_33:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_33_t;

#define FUSE_SPARE_BIT_34_OFFSET 0x288
#define FUSE_SPARE_BIT_34_RESET  0x00000000
#define FUSE_SPARE_BIT_34_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_34_u {
    struct {
        unsigned int spare_bit_34:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_34_t;

#define FUSE_SPARE_BIT_35_OFFSET 0x28C
#define FUSE_SPARE_BIT_35_RESET  0x00000000
#define FUSE_SPARE_BIT_35_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_35_u {
    struct {
        unsigned int spare_bit_35:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_35_t;

#define FUSE_SPARE_BIT_36_OFFSET 0x290
#define FUSE_SPARE_BIT_36_RESET  0x00000000
#define FUSE_SPARE_BIT_36_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_36_u {
    struct {
        unsigned int spare_bit_36:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_36_t;

#define FUSE_SPARE_BIT_37_OFFSET 0x294
#define FUSE_SPARE_BIT_37_RESET  0x00000000
#define FUSE_SPARE_BIT_37_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_37_u {
    struct {
        unsigned int spare_bit_37:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_37_t;

#define FUSE_SPARE_BIT_38_OFFSET 0x298
#define FUSE_SPARE_BIT_38_RESET  0x00000000
#define FUSE_SPARE_BIT_38_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_38_u {
    struct {
        unsigned int spare_bit_38:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_38_t;

#define FUSE_SPARE_BIT_39_OFFSET 0x29C
#define FUSE_SPARE_BIT_39_RESET  0x00000000
#define FUSE_SPARE_BIT_39_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_39_u {
    struct {
        unsigned int spare_bit_39:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_39_t;

#define FUSE_SPARE_BIT_40_OFFSET 0x2A0
#define FUSE_SPARE_BIT_40_RESET  0x00000000
#define FUSE_SPARE_BIT_40_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_40_u {
    struct {
        unsigned int spare_bit_40:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_40_t;

#define FUSE_SPARE_BIT_41_OFFSET 0x2A4
#define FUSE_SPARE_BIT_41_RESET  0x00000000
#define FUSE_SPARE_BIT_41_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_41_u {
    struct {
        unsigned int spare_bit_41:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_41_t;

#define FUSE_SPARE_BIT_42_OFFSET 0x2A8
#define FUSE_SPARE_BIT_42_RESET  0x00000000
#define FUSE_SPARE_BIT_42_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_42_u {
    struct {
        unsigned int spare_bit_42:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_42_t;

#define FUSE_SPARE_BIT_43_OFFSET 0x2AC
#define FUSE_SPARE_BIT_43_RESET  0x00000000
#define FUSE_SPARE_BIT_43_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_43_u {
    struct {
        unsigned int spare_bit_43:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_43_t;

#define FUSE_SPARE_BIT_44_OFFSET 0x2B0
#define FUSE_SPARE_BIT_44_RESET  0x00000000
#define FUSE_SPARE_BIT_44_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_44_u {
    struct {
        unsigned int spare_bit_44:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_44_t;

#define FUSE_SPARE_BIT_45_OFFSET 0x2B4
#define FUSE_SPARE_BIT_45_RESET  0x00000000
#define FUSE_SPARE_BIT_45_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_45_u {
    struct {
        unsigned int spare_bit_45:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_45_t;

#define FUSE_SPARE_BIT_46_OFFSET 0x2B8
#define FUSE_SPARE_BIT_46_RESET  0x00000000
#define FUSE_SPARE_BIT_46_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_46_u {
    struct {
        unsigned int spare_bit_46:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_46_t;

#define FUSE_SPARE_BIT_47_OFFSET 0x2BC
#define FUSE_SPARE_BIT_47_RESET  0x00000000
#define FUSE_SPARE_BIT_47_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_47_u {
    struct {
        unsigned int spare_bit_47:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_47_t;

#define FUSE_SPARE_BIT_48_OFFSET 0x2C0
#define FUSE_SPARE_BIT_48_RESET  0x00000000
#define FUSE_SPARE_BIT_48_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_48_u {
    struct {
        unsigned int spare_bit_48:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_48_t;

#define FUSE_SPARE_BIT_49_OFFSET 0x2C4
#define FUSE_SPARE_BIT_49_RESET  0x00000000
#define FUSE_SPARE_BIT_49_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_49_u {
    struct {
        unsigned int spare_bit_49:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_49_t;

#define FUSE_SPARE_BIT_50_OFFSET 0x2C8
#define FUSE_SPARE_BIT_50_RESET  0x00000000
#define FUSE_SPARE_BIT_50_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_50_u {
    struct {
        unsigned int spare_bit_50:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_50_t;

#define FUSE_SPARE_BIT_51_OFFSET 0x2CC
#define FUSE_SPARE_BIT_51_RESET  0x00000000
#define FUSE_SPARE_BIT_51_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_51_u {
    struct {
        unsigned int spare_bit_51:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_51_t;

#define FUSE_SPARE_BIT_52_OFFSET 0x2D0
#define FUSE_SPARE_BIT_52_RESET  0x00000000
#define FUSE_SPARE_BIT_52_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_52_u {
    struct {
        unsigned int spare_bit_52:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_52_t;

#define FUSE_SPARE_BIT_53_OFFSET 0x2D4
#define FUSE_SPARE_BIT_53_RESET  0x00000000
#define FUSE_SPARE_BIT_53_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_53_u {
    struct {
        unsigned int spare_bit_53:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_53_t;

#define FUSE_SPARE_BIT_54_OFFSET 0x2D8
#define FUSE_SPARE_BIT_54_RESET  0x00000000
#define FUSE_SPARE_BIT_54_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_54_u {
    struct {
        unsigned int spare_bit_54:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_54_t;

#define FUSE_SPARE_BIT_55_OFFSET 0x2DC
#define FUSE_SPARE_BIT_55_RESET  0x00000000
#define FUSE_SPARE_BIT_55_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_55_u {
    struct {
        unsigned int spare_bit_55:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_55_t;

#define FUSE_SPARE_BIT_56_OFFSET 0x2E0
#define FUSE_SPARE_BIT_56_RESET  0x00000000
#define FUSE_SPARE_BIT_56_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_56_u {
    struct {
        unsigned int spare_bit_56:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_56_t;

#define FUSE_SPARE_BIT_57_OFFSET 0x2E4
#define FUSE_SPARE_BIT_57_RESET  0x00000000
#define FUSE_SPARE_BIT_57_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_57_u {
    struct {
        unsigned int spare_bit_57:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_57_t;

#define FUSE_SPARE_BIT_58_OFFSET 0x2E8
#define FUSE_SPARE_BIT_58_RESET  0x00000000
#define FUSE_SPARE_BIT_58_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_58_u {
    struct {
        unsigned int spare_bit_58:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_58_t;

#define FUSE_SPARE_BIT_59_OFFSET 0x2EC
#define FUSE_SPARE_BIT_59_RESET  0x00000000
#define FUSE_SPARE_BIT_59_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_59_u {
    struct {
        unsigned int spare_bit_59:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_59_t;

#define FUSE_SPARE_BIT_60_OFFSET 0x2F0
#define FUSE_SPARE_BIT_60_RESET  0x00000000
#define FUSE_SPARE_BIT_60_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_60_u {
    struct {
        unsigned int spare_bit_60:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_60_t;

#define FUSE_SPARE_BIT_61_OFFSET 0x2F4
#define FUSE_SPARE_BIT_61_RESET  0x00000000
#define FUSE_SPARE_BIT_61_WRMASK 0xFFFFFFFE
typedef union fuse_spare_bit_61_u {
    struct {
        unsigned int spare_bit_61:1;
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} fuse_spare_bit_61_t;

#endif // TEGRA_FUSE_H
