/*
 * ARM NVIDIA Tegra2 emulation.
 *
 * Copyright (c) 2015 Dmitry Osipenko <digetx@gmail.com>
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

#define CACHED_READ

#include "tegra_common.h"

#include "hw/sysbus.h"
#include "cpu.h"

#include "clk_rst.h"
#include "remote_io.h"
#include "sizes.h"
#include "tegra_cpu.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_REMOTE_MEM "tegra.remote_mem"
#define TEGRA_REMOTE_MEM(obj) OBJECT_CHECK(remote_mem, (obj), TYPE_TEGRA_REMOTE_MEM)

static int read_io_mem_cache_invalid[2];

typedef struct remote_mem_state {
    SysBusDevice parent_obj;

#ifdef CACHED_READ
    uint8_t mem_cache[2][16384];
    uint32_t read_cache_start_addr[2];
#endif
    MemoryRegion iomem;
} remote_mem;

void remote_io_read_cache_invalidate(void)
{
    read_io_mem_cache_invalid[0] = 1;
    read_io_mem_cache_invalid[1] = 1;
}

#ifdef CACHED_READ
static int cache_miss(remote_mem *s, int on_avp, hwaddr offset, unsigned size)
{
    uint32_t start = s->read_cache_start_addr[on_avp];
    uint32_t end = MIN(start + ARRAY_SIZE(s->mem_cache[on_avp]), SZ_256M);

    if (read_io_mem_cache_invalid[on_avp]) {
        return 1;
    }

    if (offset < start || offset + size > end) {
        return 1;
    }

    return 0;
}

static void refill_cache(remote_mem *s, int on_avp, hwaddr start)
{
    uint32_t end = MIN(start + ARRAY_SIZE(s->mem_cache[on_avp]), SZ_256M);
    uint32_t size = end - start;

    remote_io_read_mem_range(s->mem_cache[on_avp], s->iomem.addr + start, size);

    s->read_cache_start_addr[on_avp] = start;
    read_io_mem_cache_invalid[on_avp] = 0;
}

static uint32_t read_cache(remote_mem *s, int on_avp,
                           hwaddr offset, unsigned size)
{
    uint32_t start = s->read_cache_start_addr[on_avp];
    uint16_t *local16 = (uint16_t *) (s->mem_cache[on_avp] + (offset - start));
    uint32_t *local32 = (uint32_t *) (s->mem_cache[on_avp] + (offset - start));
    uint32_t ret;

    switch (size) {
    case 1:
        ret = s->mem_cache[on_avp][offset - start];
        break;
    case 2:
        ret = *(local16);
        break;
    case 4:
        ret = *(local32);
        break;
    default:
        g_assert_not_reached();
    }

    return ret;
}

static void write_cache(remote_mem *s, int on_avp, hwaddr offset,
                        uint32_t value, unsigned size)
{
    uint32_t start = s->read_cache_start_addr[on_avp];
    uint16_t *local16 = (uint16_t *) (s->mem_cache[on_avp] + (offset - start));
    uint32_t *local32 = (uint32_t *) (s->mem_cache[on_avp] + (offset - start));

    switch (size) {
    case 1:
        s->mem_cache[on_avp][offset - start] = value;
        return;
    case 2:
        *(local16) = value;
        return;
    case 4:
        *(local32) = value;
        return;
    default:
        g_assert_not_reached();
    }
}
#endif

static uint64_t remote_mem_read(void *opaque, hwaddr offset,
                                unsigned size)
{
    remote_mem *s = TEGRA_REMOTE_MEM(opaque);
    uint32_t ret;

#ifdef CACHED_READ
    int on_avp = (current_cpu && current_cpu->cpu_index == TEGRA_BPMP);

    if (!cache_miss(s, !on_avp, offset, size)) {
        ret = read_cache(s, !on_avp, offset, size);
    } else {
        if (cache_miss(s, on_avp, offset, size)) {
            refill_cache(s, on_avp, offset);
        }

        ret = read_cache(s, on_avp, offset, size);
    }
#else
    ret = remote_io_read(s->iomem.addr + offset, size << 3);
#endif

    TRACE_READ_MEM(s->iomem.addr, offset, ret, size);

    return ret;
}

static void remote_mem_write(void *opaque, hwaddr offset,
                             uint64_t value, unsigned size)
{
    remote_mem *s = TEGRA_REMOTE_MEM(opaque);
#ifdef CACHED_READ
    int on_avp = (current_cpu && current_cpu->cpu_index == TEGRA_BPMP);
#endif

    TRACE_WRITE_MEM(s->iomem.addr, offset, value, size);

#ifdef CACHED_READ
    if (!cache_miss(s, on_avp, offset, size)) {
        write_cache(s, on_avp, offset, value, size);
    }

    if (!cache_miss(s, !on_avp, offset, size)) {
        write_cache(s, !on_avp, offset, value, size);
    }
#endif

    remote_io_write(value, s->iomem.addr + offset, size << 3);
}

static const MemoryRegionOps remote_mem_mem_ops = {
    .read = remote_mem_read,
    .write = remote_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void remote_mem_priv_realize(DeviceState *dev, Error **errp)
{
    remote_mem *s = TEGRA_REMOTE_MEM(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &remote_mem_mem_ops, s,
                          "tegra.remote_mem", SZ_256M);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void remote_mem_remote_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = remote_mem_priv_realize;
}

static const TypeInfo remote_mem_info = {
    .name = TYPE_TEGRA_REMOTE_MEM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(remote_mem),
    .class_init = remote_mem_remote_class_init,
};

static void remote_mem_register_types(void)
{
    type_register_static(&remote_mem_info);
}

type_init(remote_mem_register_types)
