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

#ifndef TEGRA_SB_H
#define TEGRA_SB_H

#define CSR_OFFSET 0x0
#define CSR_RESET 0x00000000

#define PIROM_START_OFFSET 0x4
#define PIROM_START_RESET 0x00001000

#define AA64_RESET_LOW_OFFSET 0x30
#define AA64_RESET_HIGH_OFFSET 0x34

uint64_t tegra_sb_get_cpu_reset_vector(void);

ssize_t tegra_sb_load_irom_file(void *opaque, const char *path);
void tegra_sb_load_irom_fixed(void *opaque, const void* buffer, size_t size);

#endif // TEGRA_SB_H
