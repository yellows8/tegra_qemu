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

// NOTE: Not intended to render anything.

#include "tegra_common.h"

#include "hw/sysbus.h"

#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_GPU "tegra.gpu"
#define TEGRA_GPU(obj) OBJECT_CHECK(tegra_gpu, (obj), TYPE_TEGRA_GPU)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_gpu_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[0x2000000>>2];
} tegra_gpu;

static const VMStateDescription vmstate_tegra_gpu = {
    .name = TYPE_TEGRA_GPU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, tegra_gpu, 0x2000000>>2),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_gpu_priv_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    tegra_gpu *s = opaque;
    uint64_t ret = 0;

    if (offset+size <= sizeof(s->regs))
        ret = s->regs[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_gpu_priv_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    tegra_gpu *s = opaque;

    TRACE_WRITE(s->iomem.addr, offset, 0, value);

    if (offset+size <= sizeof(s->regs)) {
        if (offset < 4) return; // Ignore id reg.
        s->regs[offset/sizeof(uint32_t)] = (s->regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;

        if (offset == 0x70000 && (value & BIT(0)))
            s->regs[offset>>2] &= ~BIT(0);
        else if (offset == 0x100C80 && (value & BIT(12)))
            s->regs[offset>>2] |= 0x1<<16;
        else if (offset == 0x100C80+0x3C && (value & BIT(31)))
            s->regs[0x100C80>>2] |= BIT(15);
        else if (offset == 0x10A004 && (value & BIT(4)))
            s->regs[0x10A008>>2] &= ~BIT(4);
        else if (offset == 0x10A100 && (value & BIT(1))) {
            s->regs[offset>>2] |= BIT(4);
            s->regs[0x10A040>>2] = 0;
        }
        else if (offset == 0x137014 && (value & BIT(30)))
            s->regs[offset>>2] |= BIT(31);
        else if (offset == 0x13701C && (value & BIT(31)))
            s->regs[0x1328A0>>2] |= BIT(24);
        else if (offset == 0x409130 && (value & BIT(1)))
            s->regs[0x409800>>2] = 0x1;
        else if (offset == 0x409400 && (value & BIT(12)))
            s->regs[offset>>2] &= ~BIT(12);
        else if ((offset == 0x409A10 || offset == 0x409B00) && (value & 0x1F))
            s->regs[offset>>2] &= ~0x1F;
    }
}

static void tegra_gpu_priv_reset(DeviceState *dev)
{
    tegra_gpu *s = TEGRA_GPU(dev);

    memset(s->regs, 0, sizeof(s->regs));

    s->regs[0x0] = tegra_board >= TEGRAX1PLUS_BOARD ? 0x12E000A1 : 0x12B000A1;

    s->regs[0x1010000>>2] = 0x33;
}

static const MemoryRegionOps tegra_gpu_mem_ops = {
    .read = tegra_gpu_priv_read,
    .write = tegra_gpu_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_gpu_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_gpu *s = TEGRA_GPU(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_gpu_mem_ops, s,
                          TYPE_TEGRA_GPU, sizeof(s->regs));
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_gpu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_gpu_priv_realize;
    dc->vmsd = &vmstate_tegra_gpu;
    dc->reset = tegra_gpu_priv_reset;
}

static const TypeInfo tegra_gpu_info = {
    .name = TYPE_TEGRA_GPU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_gpu),
    .class_init = tegra_gpu_class_init,
};

static void tegra_gpu_register_types(void)
{
    type_register_static(&tegra_gpu_info);
}

type_init(tegra_gpu_register_types)
