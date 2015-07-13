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

/* Autogenerated from TRM v02p */

#ifndef TEGRA_HOST1X_CHANNELS_H
#define TEGRA_HOST1X_CHANNELS_H

#include "hw/sysbus.h"

#include "fifo.h"
#include "host1x_cdma.h"

#undef DEFINE_REG32
#define DEFINE_REG32(reg) reg##_t reg

#define FIFOSTAT_OFFSET 0x0
#define FIFOSTAT_RESET  0x00000000
typedef union fifostat_u {
    struct {
        unsigned int cfnumempty:10;         /* Command FIFO free count */
        unsigned int cfempty:1;             /* Indicates whether the command FIFO is empty or not 0 = NOTEMPTY 1 = EMPTY */
        unsigned int cfgather:1;            /* Indicates whether GATHER is active. If a GATHER command issued via PIO, software must wait for the GATHER to be IDLE before issuing another command. 0 = IDLE 1 = BUSY */
        unsigned int cfgather3d:1;          /* Indicates whether GATHER is active. If a GATHER command issued via PIO, software must wait for the GATHER to be IDLE before issuing another command. 0 = IDLE 1 = BUSY */
        unsigned int undefined_bits_13_15:3;
        unsigned int regfnumempty:5;        /* Register write/read FIFO free count */
        unsigned int undefined_bits_21_23:3;
        unsigned int outfentries:5;         /* Number of entries available for reading in this channel's output FIFO */
        unsigned int undefined_bits_29_30:2;
        unsigned int indrdy:1;              /* Indicates that INDCOUNT==0, so it should be OK to issue another read */
    };

    uint32_t reg32;
} fifostat_t;

#define INDOFF_OFFSET 0x4
#define INDOFF_RESET  0x00000000
typedef union indoff_u {
    struct {
        unsigned int indoffupd:1;           /* Optionally disable the update of INDOFFSET when writing this register 0 = UPDATE 1 = NO_UPDATE */
        unsigned int undefined_bit_1:1;
        unsigned int indcustom:24;
        unsigned int undefined_bit_26:1;
        unsigned int indswap:2;             /* Indirect framebuffer access swap control. 00 = No byte swap 01 = 16-bit byte swap ([31:0] -> {[23:16],[31:24],[7:0],[15:8]}) 10 = 32-bit byte swap ([31:0] -> {[7:0],[15:8],[23:16],[31:24]}) 11 = 32-bit word swap ([31:0] -> {[15:8],[7:0],[31:24],[23:16]}) 0 = NONE 1 = BYTE16 2 = BYTE32 3 = WORD32 */
        unsigned int buf32b:1;              /* buffer up 32 bits of register data before sending it. Otherwise, register writes will be sent as soon as they are received. Does not support byte writes in 16-bit host. Does not affect framebuffer writes. 0 = NOBUF 1 = BUF */
        unsigned int acctype:1;             /* access type: indirect register or indirect framebuffer 0 = REG 1 = FB */
        unsigned int autoinc:1;             /* auto increment of read/write address 0 = DISABLE 1 = ENABLE */
    };

    struct {
        unsigned int stub_bitsx:2;
        unsigned int regoffset:16;
        unsigned int modid:8;
        unsigned int restx:8;
    };

    struct {
        unsigned int stub_bits:2;
        unsigned int fboffset:24;
        unsigned int rest:8;
    };

    uint32_t reg32;
} indoff_t;

#define INDCNT_OFFSET 0x8
#define INDCNT_RESET  0x00000000
typedef union indcnt_u {
    struct {
        unsigned int indcount:16;
        unsigned int undefined_bits_16_31:16;
    };

    uint32_t reg32;
} indcnt_t;

#define INDDATA_OFFSET 0xC
#define INDDATA_RESET  0x00000000
typedef union inddata_u {
    struct {
        unsigned int inddata:32;            /* read or write data */
    };

    uint32_t reg32;
} inddata_t;

#define DMASTART_OFFSET 0x14
#define DMASTART_RESET  0x00000000
typedef union dmastart_u {
    struct {
        unsigned int undefined_bits_0_1:2;
        unsigned int dmastart:30;           /* cmdbuf FB offset */
    };

    uint32_t reg32;
} dmastart_t;

#define DMAPUT_OFFSET 0x18
#define DMAPUT_RESET  0x00000000
typedef union dmaput_u {
    struct {
        unsigned int undefined_bits_0_1:2;
        unsigned int dmaput:30;             /* cmdbuf FB offset */
    };

    uint32_t reg32;
} dmaput_t;

#define DMAGET_OFFSET 0x1C
#define DMAGET_RESET  0x00000000
typedef union dmaget_u {
    struct {
        unsigned int undefined_bits_0_1:2;
        unsigned int dmaget:30;             /* cmdbuf FB offset */
    };

    uint32_t reg32;
} dmaget_t;

#define DMAEND_OFFSET 0x20
#define DMAEND_RESET  0x00000000
typedef union dmaend_u {
    struct {
        unsigned int undefined_bits_0_1:2;
        unsigned int dmaend:30;             /* cmdbuf FB offset */
    };

    uint32_t reg32;
} dmaend_t;

#define DMACTRL_OFFSET 0x24
#define DMACTRL_RESET  0x00000001
typedef union dmactrl_u {
    struct {
        unsigned int dmastop:1;             /* Stop DMA from fetching on this channel. NOTE: a Command DMA channel needs to be enabled for PIO-gather to work!! 0 = RUN 1 = STOP */
        unsigned int dmagetrst:1;           /* Reset GET pointer to '0'. Useful for cleaning up crashed channels. 0 = DISABLE 1 = ENABLE */
        unsigned int dmainitget:1;          /* Reset GET pointer to the value of DMAPUT when DMAGETRST is asserted. 0 = DISABLE 1 = ENABLE */
        unsigned int undefined_bits_3_31:29;
    };

    uint32_t reg32;
} dmactrl_t;

#define INDOFF2_OFFSET 0x8C
#define INDOFF2_RESET  0x00000000
typedef union indoff2_u {
    struct {
        unsigned int undefined_bitsx_0_1:2;
        unsigned int regoffset:16;
        unsigned int modid:8;
        unsigned int rest:8;
    };

    struct {
        unsigned int undefined_bits_0_1:2;
        unsigned int fboffset:30;
    };

    uint32_t reg32;
} indoff2_t;

#define TICKCOUNT_HI_OFFSET 0x90
#define TICKCOUNT_HI_RESET  0x00000000
typedef union tickcount_hi_u {
    struct {
        unsigned int ticks_hi:32;
    };

    uint32_t reg32;
} tickcount_hi_t;

#define TICKCOUNT_LO_OFFSET 0x94
#define TICKCOUNT_LO_RESET  0x00000000
typedef union tickcount_lo_u {
    struct {
        unsigned int ticks_lo:32;
    };

    uint32_t reg32;
} tickcount_lo_t;

#define CHANNELCTRL_OFFSET 0x98
#define CHANNELCTRL_RESET  0x00000000
typedef union channelctrl_u {
    struct {
        unsigned int enabletickcnt:1;       /* enable or disable tick counter 0 = DISABLE 1 = ENABLE */
        unsigned int undefined_bits_1_31:31;
    };

    uint32_t reg32;
} channelctrl_t;

#define RAISE_OFFSET 0x10
#define RAISE_RESET  0x00000000
typedef union raise_u {
    struct {
        unsigned int raise:32;              /* This channel's RAISE vector */
    };

    uint32_t reg32;
} raise_t;

#define FBBUFBASE_OFFSET 0x28
#define FBBUFBASE_RESET  0x00000000
typedef union fbbufbase_u {
    struct {
        unsigned int undefined_bits_0_1:2;
        unsigned int fbbufbase:30;
    };

    uint32_t reg32;
} fbbufbase_t;

#define CMDSWAP_OFFSET 0x2C
#define CMDSWAP_RESET  0x00000000
typedef union cmdswap_u {
    struct {
        unsigned int cmdswap:2;             /* Indirect framebuffer access swap control. 00 = No byte swap 01 = 16-bit byte swap ([31:0] -> {[23:16],[31:24],[7:0],[15:8]}) 10 = 32-bit byte swap ([31:0] -> {[7:0],[15:8],[23:16],[31:24]}) 11 = 32-bit word swap ([31:0] -> {[15:8],[7:0],[31:24],[23:16]}) */
        unsigned int undefined_bits_2_31:30;
    };

    uint32_t reg32;
} cmdswap_t;

typedef struct tegra_host1x_channel_state {
    SysBusDevice parent_obj;

    struct host1x_cdma cdma;
    struct host1x_fifo fifo;

    MemoryRegion iomem;
    DEFINE_REG32(fifostat);
    DEFINE_REG32(indoff);
    DEFINE_REG32(indcnt);
    DEFINE_REG32(inddata);
    DEFINE_REG32(dmastart);
    DEFINE_REG32(dmaput);
    DEFINE_REG32(dmaend);
    DEFINE_REG32(dmactrl);
    DEFINE_REG32(indoff2);
    DEFINE_REG32(tickcount_hi);
    DEFINE_REG32(tickcount_lo);
    DEFINE_REG32(channelctrl);
    DEFINE_REG32(raise);
    DEFINE_REG32(fbbufbase);
    DEFINE_REG32(cmdswap);
    uint32_t indoffset;
    uint8_t class_id;
} tegra_host1x_channel;

#endif // TEGRA_HOST1X_CHANNELS_H
