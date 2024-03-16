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

#ifndef TEGRA_CPU_H
#define TEGRA_CPU_H

#include "qemu/notify.h"

enum {
    TEGRA_CCPLEX_CORE0 = 0,
    TEGRA_CCPLEX_CORE1,
    TEGRA_CCPLEX_CORE2,
    TEGRA_CCPLEX_CORE3,
    TEGRA_BPMP,
    TEGRA_APE,
    TEGRA_NCPUS,
    TEGRA_CCPLEX_NCORES = TEGRA_CCPLEX_CORE3+1,
    TEGRA2_CCPLEX_NCORES = TEGRA_CCPLEX_CORE1+1,
    TEGRAX1_CCPLEX_NCORES = TEGRA_CCPLEX_CORE3+1,
    TEGRA2_NCPUS = TEGRA2_CCPLEX_NCORES+1,
    TEGRAX1_MAIN_NCPUS = TEGRAX1_CCPLEX_NCORES+1,
    TEGRAX1_NCPUS = TEGRAX1_MAIN_NCPUS+1,
};

void tegra_cpu_halt(int cpu_id);
void tegra_cpu_unhalt(int cpu_id);
void tegra_cpu_reset_assert(int cpu_id);
void tegra_cpu_reset_deassert(int cpu_id, int flow);
void tegra_cpu_set_rvbar(uint64_t value);
int tegra_cpu_is_powergated(int cpu_id);
void tegra_cpu_powergate(int cpu_id);
void tegra_cpu_unpowergate(int cpu_id);
uint32_t tegra_get_wfe_bitmap(int type);
void tegra_flow_wfe_handle(int cpu_id, int type);
void tegra_cpu_reset_init(void);
int tegra_sibling_cpu(int cpu_id);
int tegra_cpu_halted(int cpu_id);
void set_is_tegra_cpu(int cpu_id);

#endif // TEGRA_CPU_H
