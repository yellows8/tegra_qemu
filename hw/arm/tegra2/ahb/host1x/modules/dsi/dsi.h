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

#ifndef TEGRA_DSI_H
#define TEGRA_DSI_H

#define HOST_DSI_CONTROL_OFFSET 0xF
#define HOST_DSI_CONTROL_RESET 0x00000043
typedef union host_dsi_control_u {
    uint32_t reg32;
} host_dsi_control_t;

#define DSI_TRIGGER_OFFSET 0x13
#define DSI_TRIGGER_RESET 0x00000000
typedef union dsi_trigger_u {
    uint32_t reg32;
} dsi_trigger_t;

#endif // TEGRA_DSI_H
