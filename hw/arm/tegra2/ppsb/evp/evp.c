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

#include "hw/arm/boot.h"
#include "hw/core/tcg-cpu-ops.h"
#include "hw/sysbus.h"
#include "cpu.h"
#include "semihosting/semihost.h"
#include "sysemu/sysemu.h"

#include "devices.h"
#include "iomap.h"
#include "tegra_cpu.h"
#include "tegra_trace.h"

#include "evp.h"

#define TYPE_TEGRA_EVP "tegra.evp"
#define TEGRA_EVP(obj) OBJECT_CHECK(tegra_evp, (obj), TYPE_TEGRA_EVP)

typedef struct tegra_evp_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t evp_regs[2][36];
} tegra_evp;

static const VMStateDescription vmstate_tegra_evp = {
    .name = "tegra.evp",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_2DARRAY(evp_regs, tegra_evp, 2, 36),
        VMSTATE_END_OF_LIST()
    }
};

static int tegra_evp_cpu_index(uint64_t offset)
{
    CPUState *cs = current_cpu;
    int cpu_index = 0;

//     assert(cs != NULL);
    if (cs == NULL)
        return cpu_index;

    if (offset < EVP_CPU_RESET_VECTOR_OFFSET) {
        if (cs->cpu_index == TEGRA2_COP)
            cpu_index = 1;
    } else {
        cpu_index = (offset >> 9) & 1;
    }

    return cpu_index;
}

static uint64_t tegra_evp_priv_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    tegra_evp *s = opaque;
    uint64_t ret = 0;
    int cpu_index;

    if (offset & 3)
        goto out;

    switch (offset) {
    case EVP_RESET_VECTOR_OFFSET ... EVP_PRI_FIQ_VEC_3_OFFSET:
        cpu_index = tegra_evp_cpu_index(offset);
        ret = s->evp_regs[cpu_index][(offset & 0xff) >> 2];
        break;
    case EVP_CPU_RESET_VECTOR_OFFSET ... EVP_CPU_PRI_FIQ_VEC_3_OFFSET:
        ret = s->evp_regs[0][(offset & 0xff) >> 2];
        break;
    case EVP_COP_RESET_VECTOR_OFFSET ... EVP_COP_PRI_FIQ_VEC_3_OFFSET:
        ret = s->evp_regs[1][(offset & 0xff) >> 2];
        break;
    default:
        break;
    }

out:
    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_evp_priv_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    tegra_evp *s = opaque;

    switch (offset) {
    case EVP_CPU_RESET_VECTOR_OFFSET ... EVP_COP_PRI_FIQ_VEC_2_OFFSET:
        if (!(offset & 3)) {
            int cpu_index = tegra_evp_cpu_index(offset);
            int reg_id = (offset & 0xff) >> 2;

            TRACE_WRITE(s->iomem.addr, offset, s->evp_regs[cpu_index][reg_id],
                        value);
            s->evp_regs[cpu_index][reg_id] = value;
            break;
        }
        /* Fallthrough */
    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }
}

static void tegra_evp_priv_reset(DeviceState *dev)
{
    tegra_evp *s = TEGRA_EVP(dev);
    int i;

    for (i = 0; i < 2; i++) {
        s->evp_regs[i][0]  = EVP_RESET_VECTOR_RESET;
        s->evp_regs[i][1]  = EVP_UNDEF_VECTOR_RESET;
        s->evp_regs[i][2]  = EVP_SWI_VECTOR_RESET;
        s->evp_regs[i][3]  = EVP_PREFETCH_ABORT_VECTOR_RESET;
        s->evp_regs[i][4]  = EVP_DATA_ABORT_VECTOR_RESET;
        s->evp_regs[i][5]  = EVP_RSVD_VECTOR_RESET;
        s->evp_regs[i][6]  = EVP_IRQ_VECTOR_RESET;
        s->evp_regs[i][7]  = EVP_FIQ_VECTOR_RESET;
        s->evp_regs[i][8]  = EVP_IRQ_STS_RESET;
        s->evp_regs[i][9]  = EVP_PRI_IRQ_STS_RESET;
        s->evp_regs[i][10] = EVP_FIQ_STS_RESET;
        s->evp_regs[i][11] = EVP_PRI_FIQ_STS_RESET;
        s->evp_regs[i][12] = EVP_PRI_IRQ_NUM_0_RESET;
        s->evp_regs[i][13] = EVP_PRI_IRQ_VEC_0_RESET;
        s->evp_regs[i][14] = EVP_PRI_IRQ_NUM_1_RESET;
        s->evp_regs[i][15] = EVP_PRI_IRQ_VEC_1_RESET;
        s->evp_regs[i][16] = EVP_PRI_IRQ_NUM_2_RESET;
        s->evp_regs[i][17] = EVP_PRI_IRQ_VEC_2_RESET;
        s->evp_regs[i][18] = EVP_PRI_IRQ_NUM_3_RESET;
        s->evp_regs[i][19] = EVP_PRI_IRQ_VEC_3_RESET;
        s->evp_regs[i][20] = EVP_PRI_IRQ_NUM_4_RESET;
        s->evp_regs[i][21] = EVP_PRI_IRQ_VEC_4_RESET;
        s->evp_regs[i][22] = EVP_PRI_IRQ_NUM_5_RESET;
        s->evp_regs[i][23] = EVP_PRI_IRQ_VEC_5_RESET;
        s->evp_regs[i][24] = EVP_PRI_IRQ_NUM_6_RESET;
        s->evp_regs[i][25] = EVP_PRI_IRQ_VEC_6_RESET;
        s->evp_regs[i][26] = EVP_PRI_IRQ_NUM_7_RESET;
        s->evp_regs[i][27] = EVP_PRI_IRQ_VEC_7_RESET;
        s->evp_regs[i][28] = EVP_PRI_FIQ_NUM_0_RESET;
        s->evp_regs[i][29] = EVP_PRI_FIQ_VEC_0_RESET;
        s->evp_regs[i][30] = EVP_PRI_FIQ_NUM_1_RESET;
        s->evp_regs[i][31] = EVP_PRI_FIQ_VEC_1_RESET;
        s->evp_regs[i][32] = EVP_PRI_FIQ_NUM_2_RESET;
        s->evp_regs[i][33] = EVP_PRI_FIQ_VEC_2_RESET;
        s->evp_regs[i][34] = EVP_PRI_FIQ_NUM_3_RESET;
        s->evp_regs[i][35] = EVP_PRI_FIQ_VEC_3_RESET;
    }
}

static const MemoryRegionOps tegra_evp_mem_ops = {
    .read = tegra_evp_priv_read,
    .write = tegra_evp_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_cpu_do_interrupt(ARMCPU *cpu, void *opaque)
{
    CPUARMState *env = &cpu->env;
    tegra_evp *s = tegra_evp_dev;
    uint32_t irq_vector_addr;
    CPUState *cs = opaque;

    switch (cs->exception_index) {
    case EXCP_UDEF:
        irq_vector_addr = s->evp_regs[1][1];
        break;
    case EXCP_SWI:
        if (semihosting_enabled(false)) {
            if (env->regs[15] != 0x08 && env->regs[15] != 0xFFFF0008) {
                return;
            }
        }
        /* Fallthrough */
    case EXCP_SMC:
        irq_vector_addr = s->evp_regs[1][2];
        break;
    case EXCP_PREFETCH_ABORT:
        irq_vector_addr = s->evp_regs[1][3];
        hw_error("Base updated abort model not implemented");
        break;
    case EXCP_DATA_ABORT:
        irq_vector_addr = s->evp_regs[1][4];
        hw_error("Base updated abort model not implemented");
        break;
    case EXCP_IRQ:
        irq_vector_addr = s->evp_regs[1][6];
        break;
    case EXCP_FIQ:
        irq_vector_addr = s->evp_regs[1][7];
        break;
    default:
        return;
    }

    env->regs[15] = irq_vector_addr;

    /* ARM7TDMI switches to arm mode.  */
    env->thumb = 0;
}

#ifndef CONFIG_TCG
#error "TCG must be enabled"
#endif

static void tegra_evp_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_evp *s = TEGRA_EVP(dev);
    CPUState *cs = qemu_get_cpu(TEGRA2_COP);
    ARMCPU *cpu = ARM_CPU(cs);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_evp_mem_ops, s,
                          "tegra.evp", TEGRA_EXCEPTION_VECTORS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    /* FIXME: lame */
    device_cold_reset( DEVICE(s) );

    /* COP follows all EVP vectors.  */
    arm_register_el_change_hook(cpu, tegra_cpu_do_interrupt, cs);
}

static void tegra_evp_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_evp_priv_realize;
    dc->vmsd = &vmstate_tegra_evp;
    dc->reset = tegra_evp_priv_reset;
}

static const TypeInfo tegra_evp_info = {
    .name = TYPE_TEGRA_EVP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_evp),
    .class_init = tegra_evp_class_init,
};

static void tegra_evp_register_types(void)
{
    type_register_static(&tegra_evp_info);
}

type_init(tegra_evp_register_types)
