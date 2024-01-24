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

#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/sysbus.h"

#include "devices.h"
#include "iomap.h"

#include "host1x_channel.h"
#include "host1x_hwlock.h"
#include "host1x_module.h"
#include "host1x_priv.h"

#include "tegra_trace.h"

#define TYPE_TEGRA_GRHOST "tegra.grhost"
#define TEGRA_GRHOST(obj) OBJECT_CHECK(tegra_grhost, (obj), TYPE_TEGRA_GRHOST)

typedef struct tegra_grhost_state {
    SysBusDevice parent_obj;

    tegra_host1x_channel channels[CHANNELS_NB];

    MemoryRegion container;
    qemu_irq cop_syncpts_irq;
    qemu_irq cpu_syncpts_irq;
    qemu_irq cop_general_irq;
    qemu_irq cpu_general_irq;
} tegra_grhost;

static void tegra_grhost_add_channel(MemoryRegion *container,
                                     tegra_host1x_channel *ch, int id)
{
    object_initialize(ch, sizeof(*ch), "tegra.host1x_channel");
    sysbus_realize(SYS_BUS_DEVICE(ch), &error_fatal);
    memory_region_add_subregion(container, id * SZ_16K,
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(ch), 0));
    host1x_init_cdma(&ch->cdma, id);
}

static void tegra_grhost_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_grhost *s = TEGRA_GRHOST(dev);
    int i;

    memory_region_init(&s->container, OBJECT(s), "grhost-container",
                       TEGRA_GRHOST_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->container);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->cop_syncpts_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->cpu_syncpts_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->cop_general_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->cpu_general_irq);

    host1x_init_syncpts_irq(&s->cpu_syncpts_irq, &s->cop_syncpts_irq);
    host1x_init_syncpts();
    host1x_init_mlocks();
    host1x_init_dma();

    int count = tegra_board >= TEGRAX1_BOARD ? CHANNELS_NB_TEGRA2 : CHANNELS_NB_TEGRAX1;

    for (i = 0; i < count; i++)
        tegra_grhost_add_channel(&s->container, &s->channels[i], i);
}

static void tegra_grhost_priv_reset(DeviceState *dev)
{
    host1x_reset_mlocks();
    host1x_reset_hwlocks();
    host1x_reset_syncpt_irqs();
    host1x_reset_modules_irqs();
}

static void tegra_grhost_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_grhost_priv_realize;
    dc->reset = tegra_grhost_priv_reset;
}

static const TypeInfo tegra_grhost_info = {
    .name = TYPE_TEGRA_GRHOST,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_grhost),
    .class_init = tegra_grhost_class_init,
};

static void tegra_grhost_register_types(void)
{
    type_register_static(&tegra_grhost_info);
}

type_init(tegra_grhost_register_types)
