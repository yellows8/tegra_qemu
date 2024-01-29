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

// The below defines are based on: https://github.com/Atmosphere-NX/Atmosphere/blob/master/libraries/libexosphere/source/tsec/tsec_registers.hpp

#ifndef TEGRA_TSEC_H
#define TEGRA_TSEC_H

#define NV_SOR_DP_HDCP_BKSV_LSB_OFFSET   0x1E8
#define NV_SOR_TMDS_HDCP_BKSV_LSB_OFFSET 0x21C
#define NV_SOR_TMDS_HDCP_CN_MSB_OFFSET   0x208
#define NV_SOR_TMDS_HDCP_CN_LSB_OFFSET   0x20C

#define TSEC_FALCON_IRQMSET_OFFSET      0x1010
#define TSEC_FALCON_IRQDEST_OFFSET      0x101C
#define TSEC_FALCON_MAILBOX0_OFFSET     0x1040
#define TSEC_FALCON_MAILBOX1_OFFSET     0x1044
#define TSEC_FALCON_ITFEN_OFFSET        0x1048
#define TSEC_FALCON_ADDR                0x10AC
#define TSEC_FALCON_CPUCTL_OFFSET       0x1100
#define TSEC_FALCON_BOOTVEC_OFFSET      0x1104
#define TSEC_FALCON_DMACTL_OFFSET       0x110C
#define TSEC_FALCON_DMATRFBASE_OFFSET   0x1110
#define TSEC_FALCON_DMATRFMOFFS_OFFSET  0x1114
#define TSEC_FALCON_DMATRFCMD_OFFSET    0x1118
#define TSEC_FALCON_DMATRFFBOFFS_OFFSET 0x111C

enum tegra_tsec_engine {
    TEGRA_TSEC_ENGINE_TSEC,
    TEGRA_TSEC_ENGINE_TSECB,
    TEGRA_TSEC_ENGINE_VIC,
    TEGRA_TSEC_ENGINE_NVDEC,
    TEGRA_TSEC_ENGINE_NVENC,
    TEGRA_TSEC_ENGINE_NVJPG,
};

#endif // TEGRA_TSEC_H
