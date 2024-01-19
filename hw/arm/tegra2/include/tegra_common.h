/*
 * ARM NVIDIA Tegra2 emulation.
 *
 * Copyright (c) 2021 Dmitry Osipenko <digetx@gmail.com>
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

#ifndef TEGRA_COMMON_H
#define TEGRA_COMMON_H

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"

enum tegra_board_type {
    TEGRA2_BOARD_QEMU,
    TEGRA2_BOARD_PICASSO,
    TEGRAX1_BOARD,
    TEGRAX1PLUS_BOARD,
};

extern enum tegra_board_type tegra_board;

#endif
