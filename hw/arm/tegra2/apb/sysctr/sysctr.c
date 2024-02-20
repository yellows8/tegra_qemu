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

#define CONFIG_ARM_ARCH_TIMER

#include "tegra_common.h"

#include "hw/sysbus.h"

#include "iomap.h"
#include "tegra_trace.h"

#include "sysctr.h"

#define TYPE_TEGRA_SYSCTR "tegra.sysctr"
#define TEGRA_SYSCTR(obj) OBJECT_CHECK(tegra_sysctr, (obj), TYPE_TEGRA_SYSCTR)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_sysctr_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[TEGRA_TSC_SIZE>>2];
} tegra_sysctr;

static const VMStateDescription vmstate_tegra_sysctr = {
    .name = "tegra.sysctr",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, tegra_sysctr, TEGRA_TSC_SIZE>>2),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_sysctr_priv_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    tegra_sysctr *s = opaque;
    uint64_t ret = 0;

    assert(offset < sizeof(s->regs));

    ret = s->regs[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_sysctr_priv_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    tegra_sysctr *s = opaque;

    assert(offset < sizeof(s->regs));

    TRACE_WRITE(s->iomem.addr, offset, 0, value);

    s->regs[offset/sizeof(uint32_t)] = (s->regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;
}

static void tegra_sysctr_priv_reset(DeviceState *dev)
{
    tegra_sysctr *s = TEGRA_SYSCTR(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[SYSCTR_CNTFID0_OFFSET>>2] = 19200000; // Only needed when emulation was started after BPMP bootloader.
}

static const MemoryRegionOps tegra_sysctr_mem_ops = {
    .read = tegra_sysctr_priv_read,
    .write = tegra_sysctr_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_sysctr_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_sysctr *s = TEGRA_SYSCTR(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_sysctr_mem_ops, s,
                          "tegra.sysctr", TEGRA_TSC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_sysctr_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_sysctr_priv_realize;
    dc->vmsd = &vmstate_tegra_sysctr;
    dc->reset = tegra_sysctr_priv_reset;
}

static const TypeInfo tegra_sysctr_info = {
    .name = TYPE_TEGRA_SYSCTR,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_sysctr),
    .class_init = tegra_sysctr_class_init,
};

static void tegra_sysctr_register_types(void)
{
    type_register_static(&tegra_sysctr_info);
}

type_init(tegra_sysctr_register_types)
