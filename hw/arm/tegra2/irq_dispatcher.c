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

#define CONFIG_ARCH_TEGRA_21x_SOC

#include "tegra_common.h"

#include "hw/cpu/a9mpcore.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "cpu.h"

#include "devices.h"
#include "irqs.h"
#include "ictlr.h"
#include "tegra_cpu.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_IRQ_DISPATCHER "tegra.irq_dispatcher"
#define TEGRA_IRQ_DISPATCHER(obj) OBJECT_CHECK(tegra_irq_dispatcher, \
                                            (obj), TYPE_TEGRA_IRQ_DISPATCHER)

typedef struct tegra_irq_dispatcher {
    SysBusDevice parent_obj;
    qemu_irq cpu_irqs[TEGRA2_A9_NCORES][2];
    qemu_irq cop_irqs[2];
    int cpu_irq_gic_lvl;
    int cpu_irq_lic_lvl;
} tegra_irq_dispatcher;

static void tegra_irq_dispatcher_set_irq_dev(void *opaque, int irq, int level)
{
    A9MPPrivState *a9mpcore = A9MPCORE_PRIV(tegra_a9mpcore_dev);
    tegra_ictlr *ictlr = TEGRA_ICTLR(tegra_ictlr_dev);

//     TPRINT("%s irq=%d lvl=%d\n", __func__, irq, level);

    qemu_set_irq(qdev_get_gpio_in(DEVICE(&a9mpcore->gic), irq), level);
    qemu_set_irq(qdev_get_gpio_in(DEVICE(ictlr), irq), level);
}

static void tegra_irq_dispatcher_set_irq_gic(void *opaque, int irq, int level)
{
    tegra_irq_dispatcher *s = TEGRA_IRQ_DISPATCHER(opaque);
    int cpu_id = irq - INT_MAIN_NR;

    assert(cpu_id < TEGRA2_A9_NCORES);

    if (cpu_id == TEGRA2_A9_CORE0) {
        s->cpu_irq_gic_lvl = level;
        level |= s->cpu_irq_lic_lvl;
    }

//     TPRINT("%s cpu=%d irq=%d lvl=%d\n", __func__, cpu_id, irq, level);

    qemu_set_irq(s->cpu_irqs[cpu_id][ARM_CPU_IRQ], level);

    if (level && !tegra_cpu_is_powergated(cpu_id)) {
        tegra_flow_on_irq(cpu_id);
    }
}

static void tegra_irq_dispatcher_set_cpu_irq_lic(void *opaque, int irq, int level)
{
//     tegra_irq_dispatcher *s = TEGRA_IRQ_DISPATCHER(opaque);
//     int irq_type = irq & 1;

//     s->cpu_irq_lic_lvl = level;
//     level |= s->cpu_irq_gic_lvl;

//     TPRINT("%s irq=%d type=%s lvl=%d\n",
//            __func__, irq, irq_type ? "FIQ":"IRQ", level);

    if (level) {
        tegra_flow_on_irq(TEGRA2_A9_CORE0);
    }

    /* LIC is only used to wake CPU.  */
//     qemu_set_irq(s->cpu_irqs[TEGRA2_A9_CORE0][irq_type], level);
}

static void tegra_irq_dispatcher_set_cop_irq_lic(void *opaque, int irq, int level)
{
    tegra_irq_dispatcher *s = TEGRA_IRQ_DISPATCHER(opaque);
    int irq_type = irq & 1;

//     TPRINT("%s irq=%d type=%s lvl=%d\n",
//            __func__, irq, irq_type ? "FIQ":"IRQ", level);

    qemu_set_irq(s->cop_irqs[irq_type], level);

    if (level) {
        tegra_flow_on_irq(TEGRA2_COP);
    }
}

static void tegra_irq_dispatcher_realize(DeviceState *dev, Error **errp)
{
    /* Initialize IRQs coming from devices to dispatcher */
    qdev_init_gpio_in(dev, tegra_irq_dispatcher_set_irq_dev, INT_MAIN_NR);
    qdev_init_gpio_in(dev, tegra_irq_dispatcher_set_irq_gic, 2);
    qdev_init_gpio_in(dev, tegra_irq_dispatcher_set_cpu_irq_lic, 2);
    qdev_init_gpio_in(dev, tegra_irq_dispatcher_set_cop_irq_lic, 2);
}

static void tegra_irq_dispatcher_priv_reset(DeviceState *dev)
{
    tegra_irq_dispatcher *s = TEGRA_IRQ_DISPATCHER(dev);

    s->cpu_irq_lic_lvl = s->cpu_irq_gic_lvl = 0;
}

static void tegra_irq_dispatcher_init(Object *obj)
{
    tegra_irq_dispatcher *s = TEGRA_IRQ_DISPATCHER(obj);
    int i;

    for (i = 0; i < TEGRA2_A9_NCORES; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->cpu_irqs[i][ARM_CPU_IRQ]);
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->cpu_irqs[i][ARM_CPU_FIQ]);
    }

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->cop_irqs[ARM_CPU_IRQ]);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->cop_irqs[ARM_CPU_FIQ]);
}

static void tegra_irq_dispatcher_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_irq_dispatcher_realize;
    dc->reset = tegra_irq_dispatcher_priv_reset;
}

static const TypeInfo tegra_irq_dispatcher_info = {
    .name          = "tegra.irq_dispatcher",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_init = tegra_irq_dispatcher_init,
    .instance_size = sizeof(tegra_irq_dispatcher),
    .class_init    = tegra_irq_dispatcher_class_init,
};

static void tegra_irq_dispatcher_register_types(void)
{
    type_register_static(&tegra_irq_dispatcher_info);
}

type_init(tegra_irq_dispatcher_register_types)
