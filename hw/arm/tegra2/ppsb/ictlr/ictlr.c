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

#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/sysbus.h"

#include "devices.h"
#include "irqs.h"
#include "ictlr.h"
#include "iomap.h"
#include "tegra_cpu.h"
#include "tegra_trace.h"

static const VMStateDescription vmstate_tegra_ictlr = {
    .name = "tegra.ictlr",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(num_cpu, tegra_ictlr),
        VMSTATE_UINT32(num_irq, tegra_ictlr),
        VMSTATE_UINT32(num_banks, tegra_ictlr),
        VMSTATE_UINT32_2DARRAY(virq_cpu, tegra_ictlr, TEGRA_CCPLEX_NCORES, TEGRA_ICTLR_NUM_BANKS),
        VMSTATE_UINT32_ARRAY(virq_cop, tegra_ictlr, TEGRA_ICTLR_NUM_BANKS),
        VMSTATE_UINT32_2DARRAY(vfiq_cpu, tegra_ictlr, TEGRA_CCPLEX_NCORES, TEGRA_ICTLR_NUM_BANKS),
        VMSTATE_UINT32_ARRAY(vfiq_cop, tegra_ictlr, TEGRA_ICTLR_NUM_BANKS),
        VMSTATE_UINT32_ARRAY(isr, tegra_ictlr, TEGRA_ICTLR_NUM_BANKS),
        VMSTATE_UINT32_ARRAY(fir, tegra_ictlr, TEGRA_ICTLR_NUM_BANKS),
        VMSTATE_UINT32_2DARRAY(cpu_ier, tegra_ictlr, TEGRA_CCPLEX_NCORES, TEGRA_ICTLR_NUM_BANKS),
        VMSTATE_UINT32_2DARRAY(cpu_iep_class, tegra_ictlr, TEGRA_CCPLEX_NCORES, TEGRA_ICTLR_NUM_BANKS),
        VMSTATE_UINT32_ARRAY(cop_ier, tegra_ictlr, TEGRA_ICTLR_NUM_BANKS),
        VMSTATE_UINT32_ARRAY(cop_iep_class, tegra_ictlr, TEGRA_ICTLR_NUM_BANKS),
        VMSTATE_END_OF_LIST()
    }
};

static int tegra_ictlr_is_irq_pending(tegra_ictlr *s, uint32_t *reg, int fiq)
{
    int i;

    for (i = 0; i < s->num_banks; i++) {
        if (reg[i]) {
//             TPRINT("%s bank=%d %s reg=0x%08X\n",
//                    __func__, i, fiq ? "FIQ" : "IRQ", reg[i]);
            return 1;
        }
    }

    return 0;
}

int tegra_ictlr_is_irq_pending_on_cpu(int cpu_id)
{
    tegra_ictlr *s = tegra_ictlr_dev;
    int ret = 0;

    g_assert(tegra_ictlr_dev != NULL);
    g_assert(cpu_id < s->num_cpu);

    switch (cpu_id) {
    case TEGRA_CCPLEX_CORE0:
    case TEGRA_CCPLEX_CORE1:
    case TEGRA_CCPLEX_CORE2:
    case TEGRA_CCPLEX_CORE3:
        ret |= tegra_ictlr_is_irq_pending(s, s->virq_cpu[cpu_id], 0);
        ret |= tegra_ictlr_is_irq_pending(s, s->vfiq_cpu[cpu_id], 1);
        break;
    case TEGRA_BPMP:
        ret |= tegra_ictlr_is_irq_pending(s, s->virq_cop, 0);
        ret |= tegra_ictlr_is_irq_pending(s, s->vfiq_cop, 1);
        break;
    default:
        g_assert_not_reached();
    }

    return ret;
}

static void tegra_ictlr_update_irq(tegra_ictlr *s, uint32_t *virq, uint32_t *ier,
                                   uint32_t *iep, int is_fiq, qemu_irq irq,
                                   int bank)
{
    uint32_t new_sts, old_sts;
    int old_irq_lvl = !!(virq[0] | virq[1] | virq[2] | virq[3]);
    int new_irq_lvl = 0;
    int i;

    old_sts = virq[bank];

    new_sts = (s->fir[bank] | s->isr[bank]) & ier[bank];
    new_sts = (is_fiq ? iep[bank] : ~iep[bank]) & new_sts;

    if (old_sts == new_sts) {
        /* VIRQ's state not changed at all.  */
        return;
    }

    /* Some bit changed, update valid IRQ's status register.  */
    virq[bank] = new_sts;

    if (!!old_sts == !!new_sts) {
        /* Bank IRQ level state not changed.  */
        return;
    }

    if (old_irq_lvl == !!new_sts) {
        /* No change if new bank level is equal to overall old one.  */
        return;
    }

    /* TODO: Just track CPU IRQ status.  */
    for (i = 0; i < s->num_banks; i++) {
        new_irq_lvl = (i == bank) ? !!new_sts : !!virq[i];

        /* Any bank with VIRQ != 0 would interrupt CPU.  */
        if (new_irq_lvl) {
            break;
        }
    }

    /* Stop here if overall IRQ state not changed.  */
    if (old_irq_lvl == new_irq_lvl) {
        return;
    }

    qemu_set_irq(irq, new_irq_lvl);
}

static void tegra_ictlr_update_irqs(tegra_ictlr *s, int bank)
{
    for (uint32_t i=0; i<s->num_cpu; i++) {
        tegra_ictlr_update_irq(s, s->virq_cpu[i], s->cpu_ier[i], s->cpu_iep_class[i],
                               0, s->cpu_irq, bank);

        tegra_ictlr_update_irq(s, s->vfiq_cpu[i], s->cpu_ier[i], s->cpu_iep_class[i],
                               1, s->cpu_fiq, bank);
    }

    tegra_ictlr_update_irq(s, s->virq_cop, s->cop_ier, s->cop_iep_class,
                           0, s->cop_irq, bank);

    tegra_ictlr_update_irq(s, s->vfiq_cop, s->cop_ier, s->cop_iep_class,
                           1, s->cop_fiq, bank);
}

static void tegra_ictlr_irq_handler(void *opaque, int irq, int level)
{
    tegra_ictlr *s = opaque;
    uint32_t irq_mask = 1 << (irq & 0x1F);
    int bank = irq >> 5;

//     TPRINT("%s: irq %d level %d\n", __func__, irq, level);

    g_assert(irq < s->num_irq);

    if (level)
        s->isr[bank] |= irq_mask;
    else
        s->isr[bank] &= ~irq_mask;

    tegra_ictlr_update_irqs(s, bank);
}

static uint64_t tegra_ictlr_read(void *opaque, hwaddr offset, unsigned size)
{
    tegra_ictlr *s = opaque;
    int bank = (offset >> 8);
    int cpu_id = 0;
    uint64_t ret = 0;

    if (bank >= s->num_banks)
        goto out;

    if ( offset >= ICTLR_VIRQ_CPU1_OFFSET && offset <= ICTLR_CPU3_IEP_CLASS_OFFSET) {
        if (s->num_cpu==1) goto out;
        cpu_id = ((offset-ICTLR_VIRQ_CPU1_OFFSET) / 0x18) + 1;
        offset -= ICTLR_VIRQ_CPU1_OFFSET;
        offset = (offset % 0x18) + ICTLR_VIRQ_CPU1_OFFSET;
    }

    switch (offset & 0xff) {
    case ICTLR_VIRQ_CPU_OFFSET:
    case ICTLR_VIRQ_CPU1_OFFSET:
        ret = s->virq_cpu[cpu_id][bank];
        break;
    case ICTLR_VIRQ_COP_OFFSET:
        ret = s->virq_cop[bank];
        break;
    case ICTLR_VFIQ_CPU_OFFSET:
    case ICTLR_VFIQ_CPU1_OFFSET:
        ret = s->vfiq_cpu[cpu_id][bank];
        break;
    case ICTLR_VFIQ_COP_OFFSET:
        ret = s->vfiq_cop[bank];
        break;
    case ICTLR_ISR_OFFSET:
        ret = s->isr[bank];
        break;
    case ICTLR_FIR_OFFSET:
        ret = s->fir[bank];
        break;
    case ICTLR_CPU_IER_OFFSET:
    case ICTLR_CPU1_IER_OFFSET:
        ret = s->cpu_ier[cpu_id][bank];
        break;
    case ICTLR_CPU_IEP_CLASS_OFFSET:
    case ICTLR_CPU1_IEP_CLASS_OFFSET:
        ret = s->cpu_iep_class[cpu_id][bank];
        break;
    case ICTLR_COP_IER_OFFSET:
        ret = s->cop_ier[bank];
        break;
    case ICTLR_COP_IEP_CLASS_OFFSET:
        ret = s->cop_iep_class[bank];
        break;
    default:
        break;
    }

out:
    TRACE_READ(s->iomem.addr + bank * 0x100, offset, ret);

    return ret;
}

static void tegra_ictlr_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    tegra_ictlr *s = opaque;
    int bank = (offset >> 8);
    int cpu_id = 0;

    if (bank >= s->num_banks) {
        TRACE_WRITE(s->iomem.addr + bank * 0x100, offset, 0, value);
        return;
    }

    if ( offset >= ICTLR_VIRQ_CPU1_OFFSET && offset <= ICTLR_CPU3_IEP_CLASS_OFFSET) {
        if (s->num_cpu==1) {
            TRACE_WRITE(s->iomem.addr + bank * 0x100, offset, 0, value);
            return;
        }
        cpu_id = ((offset-ICTLR_VIRQ_CPU1_OFFSET) / 0x18) + 1;
        offset -= ICTLR_VIRQ_CPU1_OFFSET;
        offset = (offset % 0x18) + ICTLR_VIRQ_CPU1_OFFSET;
    }

    switch (offset & 0xff) {
    case ICTLR_FIR_SET_OFFSET:
        TRACE_WRITE(s->iomem.addr + bank * 0x100, offset, s->fir[bank], value);
        s->fir[bank] |= value;
        break;

    case ICTLR_FIR_CLR_OFFSET:
        TRACE_WRITE(s->iomem.addr + bank * 0x100, offset, s->fir[bank], value);
        s->fir[bank] &= ~value;
        break;

    case ICTLR_CPU_IER_SET_OFFSET:
    case ICTLR_CPU1_IER_OFFSET:
        TRACE_WRITE(s->iomem.addr + bank * 0x100, offset, s->cpu_ier[cpu_id][bank], value);
        s->cpu_ier[cpu_id][bank] |= value;
        break;

    case ICTLR_CPU_IER_CLR_OFFSET:
    case ICTLR_CPU1_IER_CLR_OFFSET:
        TRACE_WRITE(s->iomem.addr + bank * 0x100, offset, s->cpu_ier[cpu_id][bank], value);
        s->cpu_ier[cpu_id][bank] &= ~value;
        break;

    case ICTLR_CPU_IEP_CLASS_OFFSET:
    case ICTLR_CPU1_IEP_CLASS_OFFSET:
        TRACE_WRITE(s->iomem.addr + bank * 0x100, offset, s->cpu_iep_class[cpu_id][bank], value);
        s->cpu_iep_class[cpu_id][bank] = value;
        break;

    case ICTLR_COP_IER_SET_OFFSET:
        TRACE_WRITE(s->iomem.addr + bank * 0x100, offset, s->cop_ier[bank], value);
        s->cop_ier[bank] |= value;
        break;

    case ICTLR_COP_IER_CLR_OFFSET:
        TRACE_WRITE(s->iomem.addr + bank * 0x100, offset, s->cop_ier[bank], value);
        s->cop_ier[bank] &= ~value;
        break;

    case ICTLR_COP_IEP_CLASS_OFFSET:
        TRACE_WRITE(s->iomem.addr + bank * 0x100, offset, s->cop_iep_class[bank], value);
        s->cop_iep_class[bank] = value;
        break;

    default:
        TRACE_WRITE(s->iomem.addr + bank * 0x100, offset, 0, value);
        return;
    }

    tegra_ictlr_update_irqs(s, bank);
}

static void tegra_ictlr_reset(DeviceState *dev)
{
    tegra_ictlr *s = TEGRA_ICTLR(dev);

    memset(s->virq_cpu, 0, sizeof(s->virq_cpu));
    memset(s->virq_cop, 0, sizeof(s->virq_cop));
    memset(s->vfiq_cpu, 0, sizeof(s->vfiq_cpu));
    memset(s->vfiq_cop, 0, sizeof(s->vfiq_cop));
    memset(s->isr, 0, sizeof(s->isr));
    memset(s->fir, 0, sizeof(s->fir));
    memset(s->cpu_ier, 0, sizeof(s->cpu_ier));
    memset(s->cpu_iep_class, 0, sizeof(s->cpu_iep_class));
    memset(s->cop_ier, 0, sizeof(s->cop_ier));
    memset(s->cop_iep_class, 0, sizeof(s->cop_iep_class));
}

static void tegra_ictlr_realize(DeviceState *dev, Error **errp)
{
    tegra_ictlr *s = TEGRA_ICTLR(dev);
    SysBusDevice *arb_gnt_ictlr = SYS_BUS_DEVICE(&s->arb_gnt_ictlr);

    qdev_init_gpio_in(dev, tegra_ictlr_irq_handler, INT_MAIN_NR);
    sysbus_realize(arb_gnt_ictlr, &error_fatal);

    memory_region_add_subregion(&s->iomem, 0x40,
                                sysbus_mmio_get_region(arb_gnt_ictlr, 0));
}

static const MemoryRegionOps tegra_ictlr_mem_ops = {
    .read = tegra_ictlr_read,
    .write = tegra_ictlr_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_ictlr_init(Object *obj)
{
    tegra_ictlr *s = TEGRA_ICTLR(obj);

    s->num_banks = s->num_irq / 32;
    g_assert(s->num_cpu <= TEGRA_CCPLEX_NCORES);
    g_assert(s->num_banks <= TEGRA_ICTLR_NUM_BANKS);
    if (s->num_cpu < TEGRAX1_CCPLEX_NCORES) s->num_cpu = 1;

    object_initialize_child(obj, "arb_gnt_ictlr", &s->arb_gnt_ictlr,
                            TYPE_TEGRA_ARBGNT_ICTLR);

    memory_region_init_io(&s->iomem, obj, &tegra_ictlr_mem_ops, s,
                          "tegra.ictlr", s->num_banks*0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    tegra_arb_gnt_ictlr_dev = &s->arb_gnt_ictlr;

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->cpu_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->cpu_fiq);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->cop_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->cop_fiq);
}

static Property tegra_ictlr_properties[] = {
    DEFINE_PROP_UINT32("num-cpu", tegra_ictlr, num_cpu, TEGRA_CCPLEX_NCORES), \
    DEFINE_PROP_UINT32("num-irq", tegra_ictlr, num_irq, INT_MAIN_NR), \
    DEFINE_PROP_END_OF_LIST(),
};

static void tegra_ictlr_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, tegra_ictlr_properties);
    dc->vmsd = &vmstate_tegra_ictlr;
    dc->realize = tegra_ictlr_realize;
    dc->reset = tegra_ictlr_reset;
}

static const TypeInfo tegra_ictlr_info = {
    .name = TYPE_TEGRA_ICTLR,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_ictlr),
    .instance_init = tegra_ictlr_init,
    .class_init = tegra_ictlr_class_init,
};

static void tegra_ictlr_register_types(void)
{
    type_register_static(&tegra_ictlr_info);
}

type_init(tegra_ictlr_register_types)
