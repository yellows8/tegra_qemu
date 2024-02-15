/*
 * QEMU Tegra I2C
 *
 * Copyright (c) 2012 Andreas Färber
 * Copyright 2011 Google Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see
 * <http://www.gnu.org/licenses/gpl-2.0.html>
 */
#ifndef HW_I2C_TEGRA_H
#define HW_I2C_TEGRA_H

#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"

#define TYPE_TEGRA_I2C "tegra-i2c"

#define TEGRA_I2C(obj) OBJECT_CHECK(TegraI2CState, (obj), TYPE_TEGRA_I2C)

#define TEGRA_I2C_FIFO_SIZE_LOG2 5
#define TEGRA_I2C_FIFO_SIZE (1 << TEGRA_I2C_FIFO_SIZE_LOG2)

typedef enum {
    I2C_HEADER0,
    I2C_HEADER1,
    I2C_HEADER_SPECIFIC,
    I2C_PAYLOAD
} TegraI2CFSM;

typedef struct TegraI2CState {
    /*< private >*/
    SysBusDevice parent;
    /*< public >*/

    I2CBus *bus;
    MemoryRegion iomem;
    bool is_dvc;
    uint32_t dvc_ctrl[3];
    uint32_t dvc_status;
    uint32_t config;
    uint32_t cmd_addr0;
    uint32_t cmd_addr1;
    uint32_t sl_config;
    uint32_t sl_addr1;
    uint32_t sl_addr2;
    uint32_t tlow_sext;
    uint32_t sl_delay_count;
    uint32_t packet_transfer_status;
    uint32_t fifo_control;
    uint32_t int_mask;
    uint32_t int_status;
    uint32_t clk_divisor;
    uint8_t rx_fifo[TEGRA_I2C_FIFO_SIZE];
    uint32_t bus_clear_config;
    uint32_t bus_clear_status;
    uint32_t config_load;
    uint32_t regs[0x10>>2];
    int32_t rx_ptr;
    int32_t rx_len;
    uint8_t payload_size;
    uint8_t payload_transfered;
    uint32_t header;
    uint32_t header_specific;
    TegraI2CFSM state;
    qemu_irq irq;
} TegraI2CState;

#endif
