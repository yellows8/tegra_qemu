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

#include "tegra_common.h"

#include "hw/sysbus.h"
#include "cpu.h"

#include "arb_sema.h"
#include "arb.h"
#include "devices.h"
#include "iomap.h"
#include "tegra_cpu.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_ARB_SEMA "tegra.arb_sema"
#define TEGRA_ARB_SEMA(obj) OBJECT_CHECK(tegra_arb_sema, (obj), TYPE_TEGRA_ARB_SEMA)

typedef struct tegra_arb_sema_state {
    SysBusDevice parent_obj;

    qemu_irq irq_cpu;
    qemu_irq irq_cop;

    MemoryRegion iomem;
    uint32_t arb_sema_smp_gnt_st[TEGRA_SEMA_NB];
    uint32_t arb_sema_smp_get[TEGRA_SEMA_NB];
    uint32_t arb_sema_smp_req_st[TEGRA_SEMA_NB];
} tegra_arb_sema;

static const VMStateDescription vmstate_tegra_arb_sema = {
    .name = "tegra.arb_sema",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(arb_sema_smp_gnt_st, tegra_arb_sema, TEGRA_SEMA_NB),
        VMSTATE_UINT32_ARRAY(arb_sema_smp_get, tegra_arb_sema, TEGRA_SEMA_NB),
        VMSTATE_UINT32_ARRAY(arb_sema_smp_req_st, tegra_arb_sema, TEGRA_SEMA_NB),
        VMSTATE_END_OF_LIST()
    }
};

uint32_t tegra_arb_sema_gnt_status(uint8_t id)
{
    tegra_arb_sema *s = tegra_arb_sema_dev;

    assert(s != NULL && id < TEGRA_SEMA_NB);

    return s->arb_sema_smp_gnt_st[id];
}

static uint64_t tegra_arb_sema_priv_read(void *opaque, hwaddr offset,
                                         unsigned size)
{
    tegra_arb_sema *s = opaque;
    CPUState *cs = current_cpu;
    uint64_t ret = 0;
    int id;

    id = (cs && cs->cpu_index == TEGRA_BPMP) ? TEGRA_SEMA_COP : TEGRA_SEMA_CPU;

    switch (offset) {
    case SMP_GNT_ST_OFFSET:
        ret = s->arb_sema_smp_gnt_st[id];
        break;
    case SMP_PUT_OFFSET:
    case SMP_GET_OFFSET:
        ret = s->arb_sema_smp_get[id];
        break;
    case SMP_REQ_ST_OFFSET:
        ret = s->arb_sema_smp_get[id] & ~s->arb_sema_smp_gnt_st[id];
        break;
    default:
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

void tegra_arb_sema_update(void)
{
    tegra_arb_sema *s = tegra_arb_sema_dev;
    int i;

    for (i = 0; i < TEGRA_SEMA_NB; i++) {
        if (s->arb_sema_smp_get[i] & s->arb_sema_smp_gnt_st[i]) {
            if (i == TEGRA_SEMA_CPU) {
                if (tegra_arb_sema_irq_enabled(i))
                    TRACE_IRQ_RAISE(s->iomem.addr, s->irq_cpu);
                else
                    TRACE_IRQ_LOWER(s->iomem.addr, s->irq_cpu);
            }

            if (i == TEGRA_SEMA_COP) {
                if (tegra_arb_sema_irq_enabled(i))
                    TRACE_IRQ_RAISE(s->iomem.addr, s->irq_cop);
                else
                    TRACE_IRQ_LOWER(s->iomem.addr, s->irq_cop);
            }
        }
    }
}

static void tegra_arb_sema_priv_write(void *opaque, hwaddr offset,
                                      uint64_t value, unsigned size)
{
    tegra_arb_sema *s = opaque;
    CPUState *cs = current_cpu;
    int id;

    id = (cs && cs->cpu_index == TEGRA_BPMP) ? TEGRA_SEMA_COP : TEGRA_SEMA_CPU;

    switch (offset) {
    case SMP_GET_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->arb_sema_smp_get[id], value);
        s->arb_sema_smp_get[id] = value;
        break;

    case SMP_PUT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->arb_sema_smp_gnt_st[id], value);
        s->arb_sema_smp_gnt_st[id] &= ~value;
        break;

    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        return;
    }

    s->arb_sema_smp_gnt_st[id] = s->arb_sema_smp_get[id];
    s->arb_sema_smp_gnt_st[id] &= ~s->arb_sema_smp_gnt_st[!id];

    tegra_arb_sema_update();
}

static void tegra_arb_sema_priv_reset(DeviceState *dev)
{
    tegra_arb_sema *s = TEGRA_ARB_SEMA(dev);
    int i;

    for (i = 0; i < TEGRA_SEMA_NB; i++) {
        s->arb_sema_smp_gnt_st[i] = SMP_GNT_ST_RESET;
        s->arb_sema_smp_get[i] = SMP_GET_RESET;
        s->arb_sema_smp_req_st[i] = SMP_REQ_ST_RESET;
    }
}

static const MemoryRegionOps tegra_arb_sema_mem_ops = {
    .read = tegra_arb_sema_priv_read,
    .write = tegra_arb_sema_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_arb_sema_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_arb_sema *s = TEGRA_ARB_SEMA(dev);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_cop);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_cpu);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_arb_sema_mem_ops, s,
                          "tegra.arb_sema", TEGRA_ARB_SEMA_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_arb_sema_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_arb_sema_priv_realize;
    dc->vmsd = &vmstate_tegra_arb_sema;
    dc->reset = tegra_arb_sema_priv_reset;
}

static const TypeInfo tegra_arb_sema_info = {
    .name = TYPE_TEGRA_ARB_SEMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_arb_sema),
    .class_init = tegra_arb_sema_class_init,
};

static void tegra_arb_sema_register_types(void)
{
    type_register_static(&tegra_arb_sema_info);
}

type_init(tegra_arb_sema_register_types)
