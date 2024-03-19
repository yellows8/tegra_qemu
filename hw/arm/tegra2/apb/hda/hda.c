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

#include "qapi/error.h"
#include "qapi/qapi-commands.h"

#include "iomap.h"
#include "tegra_trace.h"
#include "devices.h"

#include "qemu/cutils.h"
#include "qemu/log.h"

#define TYPE_TEGRA_HDA "tegra.hda"
#define TEGRA_HDA(obj) OBJECT_CHECK(tegra_hda, (obj), TYPE_TEGRA_HDA)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_hda_state {
    SysBusDevice parent_obj;

    qemu_irq irq;
    MemoryRegion iomem;

    uint32_t regs[TEGRA_HDA_SIZE>>2];

} tegra_hda;

static const VMStateDescription vmstate_tegra_hda = {
    .name = TYPE_TEGRA_HDA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, tegra_hda, TEGRA_HDA_SIZE>>2),

        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_hda_priv_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    tegra_hda *s = opaque;
    uint64_t ret = 0;

    TRACE_READ(s->iomem.addr, offset, ret);

    if (offset+size <= sizeof(s->regs)) {
        ret = s->regs[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);
    }

    return ret;
}

static void tegra_hda_priv_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    tegra_hda *s = opaque;

    TRACE_WRITE(s->iomem.addr, offset, 0, value);

    if (offset+size <= sizeof(s->regs)) {
        if (offset == 0x8000+0x24) { // Interrupts status
            s->regs[offset>>2] &= ~value;
            if (!s->regs[offset>>2]) TRACE_IRQ_LOWER(s->iomem.addr, s->irq);
        }

        s->regs[offset/sizeof(uint32_t)] = (s->regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;

        if (offset == 0x8000+0x48) {
            TRACE_IRQ_RAISE(s->iomem.addr, s->irq);

            s->regs[(0x8000+0x24)>>2] |= BIT(31) | BIT(30);
            s->regs[(0x8000+0x5C)>>2] |= BIT(8);
            s->regs[(0x8000+0x58)>>2] = (s->regs[(0x8000+0x58)>>2] + 1) & 0xFF;
        }
    }
}

static void tegra_hda_priv_reset(DeviceState *dev)
{
    tegra_hda *s = TEGRA_HDA(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static const MemoryRegionOps tegra_hda_mem_ops = {
    .read = tegra_hda_priv_read,
    .write = tegra_hda_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_hda_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_hda *s = TEGRA_HDA(dev);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_hda_mem_ops, s,
                          TYPE_TEGRA_HDA, TEGRA_HDA_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_hda_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_hda_priv_realize;
    dc->vmsd = &vmstate_tegra_hda;
    dc->reset = tegra_hda_priv_reset;
}

static const TypeInfo tegra_hda_info = {
    .name = TYPE_TEGRA_HDA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_hda),
    .class_init = tegra_hda_class_init,
};

static void tegra_hda_register_types(void)
{
    type_register_static(&tegra_hda_info);
}

type_init(tegra_hda_register_types)
