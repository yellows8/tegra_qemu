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

#include "tegra_common.h"

#include "hw/sysbus.h"

#include "iomap.h"
#include "tegra_trace.h"

#include "tsec.h"

#define TYPE_TEGRA_TSEC "tegra.tsec"
#define TEGRA_TSEC(obj) OBJECT_CHECK(tegra_tsec, (obj), TYPE_TEGRA_TSEC)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_tsec_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[TEGRA_TSEC_SIZE>>2];
} tegra_tsec;

static const VMStateDescription vmstate_tegra_tsec = {
    .name = "tegra.tsec",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, tegra_tsec, TEGRA_TSEC_SIZE>>2),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_tsec_priv_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    tegra_tsec *s = opaque;
    uint64_t ret = 0;

    assert(offset < sizeof(s->regs));

    ret = s->regs[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    if (offset == TSEC_FALCON_DMATRFCMD_OFFSET) {
        ret |= 1<<1; // BUSY = IDLE
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_tsec_priv_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    tegra_tsec *s = opaque;

    assert(offset < sizeof(s->regs));

    TRACE_WRITE(s->iomem.addr, offset, 0, value);

    if (offset == TSEC_FALCON_CPUCTL_OFFSET) {
        if (value & 0x2) {
            value |= 1<<4; // When STARTCPU is set, enable HALTED.
            s->regs[TSEC_FALCON_MAILBOX1_OFFSET>>2] = 0xB0B0B0B0; // TsecResult_Success
        }
    }

    s->regs[offset/sizeof(uint32_t)] = (s->regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;
}

static void tegra_tsec_priv_reset(DeviceState *dev)
{
    tegra_tsec *s = TEGRA_TSEC(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static const MemoryRegionOps tegra_tsec_mem_ops = {
    .read = tegra_tsec_priv_read,
    .write = tegra_tsec_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_tsec_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_tsec *s = TEGRA_TSEC(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_tsec_mem_ops, s,
                          "tegra.tsec", TEGRA_TSEC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_tsec_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_tsec_priv_realize;
    dc->vmsd = &vmstate_tegra_tsec;
    dc->reset = tegra_tsec_priv_reset;
}

static const TypeInfo tegra_tsec_info = {
    .name = TYPE_TEGRA_TSEC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_tsec),
    .class_init = tegra_tsec_class_init,
};

static void tegra_tsec_register_types(void)
{
    type_register_static(&tegra_tsec_info);
}

type_init(tegra_tsec_register_types)
