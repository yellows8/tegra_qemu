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

#include "avpcache.h"

#define TYPE_TEGRA_AVPCACHE "tegra.avpcache"
#define TEGRA_AVPCACHE(obj) OBJECT_CHECK(tegra_avpcache, (obj), TYPE_TEGRA_AVPCACHE)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_avpcache_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[0x1000/4];
} tegra_avpcache;

static const VMStateDescription vmstate_tegra_avpcache = {
    .name = "tegra.avpcache",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, tegra_avpcache, 0x1000>>2),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_avpcache_priv_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    tegra_avpcache *s = opaque;
    uint64_t ret = 0;

    assert(offset < sizeof(s->regs));

    ret = s->regs[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_avpcache_priv_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    tegra_avpcache *s = opaque;

    assert(offset < sizeof(s->regs));

    TRACE_WRITE(s->iomem.addr, offset, 0, value);

    s->regs[offset/sizeof(uint32_t)] = (s->regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;

    if (offset == AVP_CACHE_MAINT_2_OFFSET) {
        s->regs[AVP_CACHE_INT_RAW_EVENT_OFFESET>>2] = 1;
    }
    else if (offset == AVP_CACHE_INT_CLEAR_OFFSET) {
        s->regs[AVP_CACHE_INT_RAW_EVENT_OFFESET>>2] &= ~value;
    }
}

static void tegra_avpcache_priv_reset(DeviceState *dev)
{
    tegra_avpcache *s = TEGRA_AVPCACHE(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static const MemoryRegionOps tegra_avpcache_mem_ops = {
    .read = tegra_avpcache_priv_read,
    .write = tegra_avpcache_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_avpcache_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_avpcache *s = TEGRA_AVPCACHE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_avpcache_mem_ops, s,
                          "tegra.avpcache", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_avpcache_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_avpcache_priv_realize;
    dc->vmsd = &vmstate_tegra_avpcache;
    dc->reset = tegra_avpcache_priv_reset;
}

static const TypeInfo tegra_avpcache_info = {
    .name = TYPE_TEGRA_AVPCACHE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_avpcache),
    .class_init = tegra_avpcache_class_init,
};

static void tegra_avpcache_register_types(void)
{
    type_register_static(&tegra_avpcache_info);
}

type_init(tegra_avpcache_register_types)
