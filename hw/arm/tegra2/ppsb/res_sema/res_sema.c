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

#include "tegra_common.h"

#include "hw/sysbus.h"
#include "cpu.h"

#include "res_sema.h"
#include "iomap.h"
#include "tegra_cpu.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_RES_SEMA "tegra.res_sema"
#define TEGRA_RES_SEMA(obj) OBJECT_CHECK(tegra_res_sema, (obj), TYPE_TEGRA_RES_SEMA)
#define DEFINE_REG32(reg) reg##_t reg

typedef struct tegra_res_sema_state {
    SysBusDevice parent_obj;

    qemu_irq irq_cpu_inbox_full;
    qemu_irq irq_cpu_outbox_empty;
    qemu_irq irq_cop_inbox_empty;
    qemu_irq irq_cop_outbox_full;

    MemoryRegion iomem;
    DEFINE_REG32(res_sema_shrd_smp_sta);
    DEFINE_REG32(res_sema_shrd_inbox);
    DEFINE_REG32(res_sema_shrd_outbox);
    uint8_t irq_sts;
} tegra_res_sema;

static const VMStateDescription vmstate_tegra_res_sema = {
    .name = "tegra.res_sema",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(res_sema_shrd_smp_sta.reg32, tegra_res_sema),
        VMSTATE_UINT32(res_sema_shrd_inbox.reg32, tegra_res_sema),
        VMSTATE_UINT32(res_sema_shrd_outbox.reg32, tegra_res_sema),
        VMSTATE_UINT8(irq_sts, tegra_res_sema),
        VMSTATE_END_OF_LIST()
    }
};

static void tegra_res_sema_upd_irq_sts(tegra_res_sema *s, int n, int l,
                                       qemu_irq i)
{
    uint8_t mask = 1 << n;

    if (l && !(s->irq_sts & mask)) {
        s->irq_sts |= mask;
        TRACE_IRQ_RAISE(s->iomem.addr, i);
    } else if (!l && (s->irq_sts & mask)) {
        s->irq_sts &= ~mask;
        TRACE_IRQ_LOWER(s->iomem.addr, i);
    }
}

static void tegra_res_sema_upd_irqs(tegra_res_sema *s, int inbox)
{
    int l;

    if (inbox) {
        l = s->res_sema_shrd_inbox.tag && s->res_sema_shrd_inbox.ie_ibf;
        tegra_res_sema_upd_irq_sts(s, 0, l, s->irq_cpu_inbox_full);

        l = !s->res_sema_shrd_inbox.tag && s->res_sema_shrd_inbox.ie_ibe;
        tegra_res_sema_upd_irq_sts(s, 1, l, s->irq_cop_inbox_empty);
    } else {
        l = s->res_sema_shrd_outbox.tag && s->res_sema_shrd_outbox.ie_obf;
        tegra_res_sema_upd_irq_sts(s, 2, l, s->irq_cop_outbox_full);

        l = !s->res_sema_shrd_outbox.tag && s->res_sema_shrd_outbox.ie_obe;
        tegra_res_sema_upd_irq_sts(s, 3, l, s->irq_cpu_outbox_empty);
    }
}

static uint64_t tegra_res_sema_priv_read(void *opaque, hwaddr offset,
                                         unsigned size)
{
    tegra_res_sema *s = opaque;
    uint64_t ret = 0;

    if (current_cpu == NULL) {
        return ret;
    }

    switch (offset) {
    case RES_SEMA_SHRD_SMP_STA_OFFSET:
        ret = s->res_sema_shrd_smp_sta.reg32;
        break;
    case RES_SEMA_SHRD_INBOX_OFFSET:
        ret = s->res_sema_shrd_inbox.reg32;
        break;
    case RES_SEMA_SHRD_OUTBOX_OFFSET:
        ret = s->res_sema_shrd_outbox.reg32;
        break;
    default:
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_res_sema_priv_write(void *opaque, hwaddr offset,
                                      uint64_t value, unsigned size)
{
    tegra_res_sema *s = opaque;
    int cpu_id;

    if (current_cpu == NULL) {
        return;
    }

    cpu_id = current_cpu->cpu_index;

    switch (offset) {
    case RES_SEMA_SHRD_SMP_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->res_sema_shrd_smp_sta.reg32, value);
        s->res_sema_shrd_smp_sta.reg32 |= value;
        break;
    case RES_SEMA_SHRD_SMP_CLR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->res_sema_shrd_smp_sta.reg32, value);
        s->res_sema_shrd_smp_sta.reg32 &= ~value;
        break;
    case RES_SEMA_SHRD_INBOX_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->res_sema_shrd_inbox.reg32, value);
        s->res_sema_shrd_inbox.reg32 = value;

        if (cpu_id != TEGRA_BPMP) {
            s->res_sema_shrd_inbox.tag = 0;
        }

        tegra_res_sema_upd_irqs(s, 1);
        break;
    case RES_SEMA_SHRD_OUTBOX_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->res_sema_shrd_outbox.reg32, value);
        s->res_sema_shrd_outbox.reg32 = value;

        if (cpu_id == TEGRA_BPMP) {
            s->res_sema_shrd_outbox.tag = 0;
        }

        tegra_res_sema_upd_irqs(s, 0);
        break;
    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }
}

static void tegra_res_sema_priv_reset(DeviceState *dev)
{
    tegra_res_sema *s = TEGRA_RES_SEMA(dev);

    s->res_sema_shrd_smp_sta.reg32 = RES_SEMA_SHRD_SMP_STA_RESET;
    s->res_sema_shrd_inbox.reg32 = RES_SEMA_SHRD_INBOX_RESET;
    s->res_sema_shrd_outbox.reg32 = RES_SEMA_SHRD_OUTBOX_RESET;
    s->irq_sts = 0;
}

static const MemoryRegionOps tegra_res_sema_mem_ops = {
    .read = tegra_res_sema_priv_read,
    .write = tegra_res_sema_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_res_sema_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_res_sema *s = TEGRA_RES_SEMA(dev);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_cpu_inbox_full);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_cop_inbox_empty);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_cop_outbox_full);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_cpu_outbox_empty);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_res_sema_mem_ops, s,
                          "tegra.res_sema", TEGRA_RES_SEMA_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_res_sema_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_res_sema_priv_realize;
    dc->vmsd = &vmstate_tegra_res_sema;
    dc->reset = tegra_res_sema_priv_reset;
}

static const TypeInfo tegra_res_sema_info = {
    .name = TYPE_TEGRA_RES_SEMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_res_sema),
    .class_init = tegra_res_sema_class_init,
};

static void tegra_res_sema_register_types(void)
{
    type_register_static(&tegra_res_sema_info);
}

type_init(tegra_res_sema_register_types)
