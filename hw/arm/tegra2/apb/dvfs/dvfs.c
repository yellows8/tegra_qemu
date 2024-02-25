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

// Based on tegra2 device code by digetx.

#define CONFIG_ARCH_TEGRA_21x_SOC

#include "tegra_common.h"

#include "hw/sysbus.h"

#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_DVFS "tegra.dvfs"
#define TEGRA_DVFS(obj) OBJECT_CHECK(tegra_dvfs, (obj), TYPE_TEGRA_DVFS)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_dvfs_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[0x78>>2];
} tegra_dvfs;

static const VMStateDescription vmstate_tegra_dvfs = {
    .name = TYPE_TEGRA_DVFS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, tegra_dvfs, 0x78>>2),
        VMSTATE_END_OF_LIST()
    }
};

static uint32_t tegra_dvfs_regdef_tegrax1_reset_table[] = {
    TEGRA_REGDEF_TABLE_RESET(CL_DVFS_CONFIG_0, 0x4, 0x00000006)
    TEGRA_REGDEF_TABLE_RESET(CL_DVFS_PARAMS_0, 0x8, 0x000000F0)
    TEGRA_REGDEF_TABLE_RESET(CL_DVFS_FREQ_REQ_0, 0x14, 0x0000FF40)
    TEGRA_REGDEF_TABLE_RESET(CL_DVFS_DROOP_CTRL_0, 0x1C, 0x00000810)
    TEGRA_REGDEF_TABLE_RESET(CL_DVFS_OUTPUT_CFG_0, 0x20, 0x50200000)
    TEGRA_REGDEF_TABLE_RESET(CL_DVFS_I2C_CFG_0, 0x40, 0x0010F000)
    TEGRA_REGDEF_TABLE_RESET(CL_DVFS_I2C_CLK_DIVISOR_REGISTER_0, 0x16C, 0x00190001)
    TEGRA_REGDEF_TABLE_RESET(DVFS_DFLL_THROTTLE_CTRL_0, 0x64, 0x00000006)
    TEGRA_REGDEF_TABLE_RESET(DVFS_DFLL_THROTTLE_LIGHT_0, 0x68, 0xFF402000)
    TEGRA_REGDEF_TABLE_RESET(DVFS_DFLL_THROTTLE_MEDIUM_0, 0x6C, 0xFF402000)
    TEGRA_REGDEF_TABLE_RESET(DVFS_DFLL_THROTTLE_HEAVY_0, 0x70, 0xFF402000)
    TEGRA_REGDEF_TABLE_RESET(DVFS_CC4_HVC_0, 0x74, 0x00000101)
};

static uint64_t tegra_dvfs_priv_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    tegra_dvfs *s = opaque;
    uint64_t ret = 0;

    if (offset + size <= sizeof(s->regs)) ret = s->regs[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_dvfs_priv_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    tegra_dvfs *s = opaque;

    TRACE_WRITE(s->iomem.addr, offset, 0, value);

    if (offset + size <= sizeof(s->regs)) {
        s->regs[offset/sizeof(uint32_t)] = (s->regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;

        if (offset == 0x28) { // CL_DVFS_MONITOR_CTRL_0
            s->regs[0x2C>>2] = ((value & 0x3) != 0) << 16; // Set CL_DVFS_MONITOR_DATA_0 DFLL_MONITOR_DATA_NEW using DFLL_MONITOR_CTRL_SELECT != DISABLE.
        }
    }
}

static void tegra_dvfs_priv_reset(DeviceState *dev)
{
    tegra_dvfs *s = TEGRA_DVFS(dev);

    memset(s->regs, 0, sizeof(s->regs));

    if (tegra_board >= TEGRAX1_BOARD) {
        for (size_t i=0; i<sizeof(tegra_dvfs_regdef_tegrax1_reset_table)/sizeof(uint32_t); i+=2) {
            s->regs[tegra_dvfs_regdef_tegrax1_reset_table[i]>>2] = tegra_dvfs_regdef_tegrax1_reset_table[i+1];
        }
    }
}

static const MemoryRegionOps tegra_dvfs_mem_ops = {
    .read = tegra_dvfs_priv_read,
    .write = tegra_dvfs_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_dvfs_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_dvfs *s = TEGRA_DVFS(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_dvfs_mem_ops, s,
                          TYPE_TEGRA_DVFS, TEGRA_CL_DVFS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_dvfs_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_dvfs_priv_realize;
    dc->vmsd = &vmstate_tegra_dvfs;
    dc->reset = tegra_dvfs_priv_reset;
}

static const TypeInfo tegra_dvfs_info = {
    .name = TYPE_TEGRA_DVFS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_dvfs),
    .class_init = tegra_dvfs_class_init,
};

static void tegra_dvfs_register_types(void)
{
    type_register_static(&tegra_dvfs_info);
}

type_init(tegra_dvfs_register_types)
