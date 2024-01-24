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

#ifndef TEGRA_HOST1X_PRIV_H
#define TEGRA_HOST1X_PRIV_H

#define NV_HOST1X_SYNCPT_THESH_WIDTH    16
#define NV_HOST1X_SYNCPT_NB_BASES       64
#define NV_HOST1X_SYNCPT_NB_BASES_TEGRA2   8
#define NV_HOST1X_SYNCPT_NB_BASES_TEGRAX1  64
#define NV_HOST1X_SYNCPT_NB_PTS         192
#define NV_HOST1X_SYNCPT_NB_PTS_TEGRA2  32
#define NV_HOST1X_SYNCPT_NB_PTS_X1      192
#define NV_HOST1X_NB_MLOCKS             16
#define CHANNELS_NB                     14
#define CHANNELS_NB_TEGRA2              8
#define CHANNELS_NB_TEGRAX1             14

#define FOREACH_BIT_SET(val, itr, size)     \
    if (val != 0)                           \
        for (itr = 0; itr < size; itr++)    \
            if ((val >> itr) & 1)

#define FOREACH_CPU(itr) \
    for (itr = HOST1X_CPU; itr < HOST1X_CPUS_NB; itr++)

enum hcpu {
    HOST1X_CPU = 0,
    HOST1X_COP,
    HOST1X_CPUS_NB,
};

#endif // TEGRA_HOST1X_PRIV_H
