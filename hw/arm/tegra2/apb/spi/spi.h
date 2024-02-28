/*
 * ARM NVIDIA X1 emulation.
 *
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

#ifndef TEGRA_SPI_H
#define TEGRA_SPI_H

#define SPI_COMMAND_OFFSET 0x0
#define SPI_COMMAND_RESET 0x43D00000
#define SPI_TIMING_REG2_OFFSET 0xC
#define SPI_TIMING_REG2_RESET 0x20202020
#define SPI_TRANSFER_STATUS_OFFSET 0x10
#define SPI_TRANSFER_STATUS_RESET 0x00FF0000
#define SPI_FIFO_STATUS_OFFSET 0x14
#define SPI_FIFO_STATUS_RESET 0x00400005
#define SPI_DMA_CTL_OFFSET 0x20
#define SPI_DMA_BLK_SIZE_OFFSET 0x24
#define SPI_INTR_MASK_OFFSET 0x18C
#define SPI_SPARE_CTLR_OFFSET 0x190
#define SPI_SPARE_CTLR_RESET 0x0FFF0000

#endif // TEGRA_SPI_H
