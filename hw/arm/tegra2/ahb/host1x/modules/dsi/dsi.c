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
#include "host1x_module.h"

#include "dsi.h"

#define TYPE_TEGRA_DSI "tegra.dsi"
#define TEGRA_DSI(obj) OBJECT_CHECK(tegra_dsi, (obj), TYPE_TEGRA_DSI)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_dsi_state {
    SysBusDevice parent_obj;

    struct host1x_module module;

    MemoryRegion iomem;
    uint32_t regs[TEGRA_DSI_SIZE>>2];
} tegra_dsi;

static const VMStateDescription vmstate_tegra_dsi = {
    .name = "tegra.dsi",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, tegra_dsi, TEGRA_DSI_SIZE>>2),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_dsi_priv_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    tegra_dsi *s = opaque;
    uint64_t ret = 0;

    assert(offset < sizeof(s->regs));

    offset>>=2;
    ret = s->regs[offset] & ((1ULL<<size*8)-1);

    TRACE_READ(s->iomem.addr, offset<<2, ret);

    return ret;
}

static void tegra_dsi_priv_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    tegra_dsi *s = opaque;

    assert(offset < sizeof(s->regs));

    TRACE_WRITE(s->iomem.addr, offset, 0, value);

    offset>>=2;

    if (offset == HOST_DSI_CONTROL_OFFSET) {
        value &= ~((1<<3) | (1<<7)); // IMM_BTA, PERIPH_RESET
    }
    else if (offset == DSI_TRIGGER_OFFSET) {
        value &= ~0x3; // Clear the operation-complete bits.
    }

    s->regs[offset] = (s->regs[offset] & ~((1ULL<<size*8)-1)) | value;
}

static void tegra_dsi_module_write(struct host1x_module *module,
                                  uint32_t offset, uint32_t data)
{
    tegra_dsi_priv_write(module->opaque, offset<<2, data, 4);
}

static uint32_t tegra_dsi_module_read(struct host1x_module *module,
                                     uint32_t offset)
{
    return tegra_dsi_priv_read(module->opaque, offset<<2, 4);
}

static void tegra_dsi_priv_reset(DeviceState *dev)
{
    tegra_dsi *s = TEGRA_DSI(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[HOST_DSI_CONTROL_OFFSET] = DSI_TRIGGER_RESET;
    s->regs[DSI_TRIGGER_OFFSET] = HOST_DSI_CONTROL_RESET;
}

static const MemoryRegionOps tegra_dsi_mem_ops = {
    .read = tegra_dsi_priv_read,
    .write = tegra_dsi_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_dsi_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_dsi *s = TEGRA_DSI(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_dsi_mem_ops, s,
                          "tegra.dsi", TEGRA_DSI_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    s->module.reg_write = tegra_dsi_module_write;
    s->module.reg_read = tegra_dsi_module_read;
    register_host1x_bus_module(&s->module, s);
}

static Property tegra_dsi_properties[] = {
    DEFINE_PROP_UINT8("class_id", tegra_dsi, module.class_id, 0x79),
    DEFINE_PROP_END_OF_LIST(),
};

static void tegra_dsi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, tegra_dsi_properties);
    dc->realize = tegra_dsi_priv_realize;
    dc->vmsd = &vmstate_tegra_dsi;
    device_class_set_legacy_reset(dc, tegra_dsi_priv_reset);
}

static const TypeInfo tegra_dsi_info = {
    .name = TYPE_TEGRA_DSI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_dsi),
    .class_init = tegra_dsi_class_init,
};

static void tegra_dsi_register_types(void)
{
    type_register_static(&tegra_dsi_info);
}

type_init(tegra_dsi_register_types)
