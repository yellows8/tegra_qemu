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
#include "devices.h"
#include "iomap.h"

#include "evp.h"

#define TYPE_TEGRA_EVP "tegra.evp"
#define TEGRA_EVP(obj) OBJECT_CHECK(tegra_evp, (obj), TYPE_TEGRA_EVP)

typedef struct tegra_evp_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion lovec_mem;
    uint32_t cpu_reset_vector;
    uint32_t bpmp_reset_vector;
    uint32_t cold_bpmp_reset_vector;
    uint32_t evp_regs[2][36];
} tegra_evp;

static const VMStateDescription vmstate_tegra_evp = {
    .name = "tegra.evp",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(cpu_reset_vector, tegra_evp),
        VMSTATE_UINT32(bpmp_reset_vector, tegra_evp),
        VMSTATE_UINT32(cold_bpmp_reset_vector, tegra_evp),
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
        if (cs->cpu_index == TEGRA_BPMP)
            cpu_index = 1;
    } else {
        cpu_index = (offset >> 9) & 1;
    }

    return cpu_index;
}

uint64_t tegra_evp_get_cpu_reset_vector(void) {
    tegra_evp *s = TEGRA_EVP(tegra_evp_dev);

    return s->evp_regs[0][0];
}

void tegra_evp_set_bpmp_reset_vector(uint64_t value) {
    tegra_evp *s = TEGRA_EVP(tegra_evp_dev);

    s->evp_regs[1][0] = value;
}

bool tegra_evp_is_bpmp_reset_vector_default(void) {
    tegra_evp *s = TEGRA_EVP(tegra_evp_dev);

    return s->bpmp_reset_vector == TEGRA_IROM_BASE;
}

bool tegra_evp_is_cold_bpmp_reset_vector_default(void) {
    tegra_evp *s = TEGRA_EVP(tegra_evp_dev);

    return s->cold_bpmp_reset_vector == TEGRA_IROM_BASE;
}

static uint64_t tegra_evp_priv_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    tegra_evp *s = opaque;
    uint64_t ret = 0;
    int cpu_index;

    if (offset & 3)
        goto out;

    offset &= 0x3FF;

    hwaddr tmpoff = offset & 0xFF;
    switch (tmpoff) {
    case EVP_RESET_VECTOR_OFFSET ... EVP_FIQ_VECTOR_OFFSET:
    case EVP_PRI_IRQ_STS_OFFSET:
    case EVP_PRI_FIQ_STS_OFFSET:
    case EVP_PRI_IRQ_NUM_0_OFFSET ... EVP_PRI_FIQ_VEC_3_OFFSET:
        cpu_index = tegra_evp_cpu_index(offset);
        ret = s->evp_regs[cpu_index][tmpoff >> 2];
        if (tmpoff == EVP_PRI_IRQ_STS_OFFSET || tmpoff == EVP_PRI_FIQ_STS_OFFSET ||
            (tmpoff >= EVP_PRI_IRQ_NUM_0_OFFSET && (tmpoff & 0x4) == 0))
            ret &= 0xFF;
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

    offset &= 0x3FF;

    switch (offset & 0xFF) {
    case EVP_RESET_VECTOR_OFFSET ... EVP_FIQ_VECTOR_OFFSET:
    case EVP_PRI_IRQ_STS_OFFSET:
    case EVP_PRI_FIQ_STS_OFFSET:
    case EVP_PRI_IRQ_NUM_0_OFFSET ... EVP_PRI_FIQ_VEC_3_OFFSET:
        if (!(offset & 3) && (offset & 0x300) != 0x0) {
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

static uint64_t tegra_evp_lovec_priv_read(void *opaque, hwaddr offset,
                                          unsigned size)
{
    tegra_evp *s = opaque;
    uint64_t ret = 0;

    if (offset < 0x40) ret = 0xe59ff3f8; // Load pc from ptrs starting at offset 0x400.
    else if (offset < 0x400) ret = 0xeafffffe; // "b ."
    else ret = tegra_evp_priv_read(s, offset-0x400, size);

    TRACE_READ(s->lovec_mem.addr, offset, ret);

    return ret;
}

static void tegra_evp_lovec_priv_write(void *opaque, hwaddr offset,
                                       uint64_t value, unsigned size)
{
    tegra_evp *s = opaque;

    // Only offset >=0x400 is writable.
    if (offset >= 0x400) tegra_evp_priv_write(s, offset-0x400, value, size);

    TRACE_WRITE(s->lovec_mem.addr, offset, 0, value);
}

void tegra_evp_reset(DeviceState *dev, ShutdownCause cause)
{
    tegra_evp *s = TEGRA_EVP(dev);
    int i;

    uint32_t resetval = tegra_board < TEGRAX1_BOARD ? 0x80 : 0xC0;

    s->evp_regs[0][0] = s->cpu_reset_vector;
    s->evp_regs[1][0] = cause != SHUTDOWN_CAUSE_GUEST_RESET ? s->cold_bpmp_reset_vector : s->bpmp_reset_vector;

    for (i = 0; i < 1; i++) {
        s->evp_regs[i][1]  = EVP_UNDEF_VECTOR_RESET;
        s->evp_regs[i][2]  = EVP_SWI_VECTOR_RESET;
        s->evp_regs[i][3]  = EVP_PREFETCH_ABORT_VECTOR_RESET;
        s->evp_regs[i][4]  = EVP_DATA_ABORT_VECTOR_RESET;
        s->evp_regs[i][5]  = EVP_RSVD_VECTOR_RESET;
        s->evp_regs[i][6]  = EVP_IRQ_VECTOR_RESET;
        s->evp_regs[i][7]  = EVP_FIQ_VECTOR_RESET;
        s->evp_regs[i][8]  = EVP_IRQ_STS_RESET;
        s->evp_regs[i][9]  = resetval;
        s->evp_regs[i][10] = EVP_FIQ_STS_RESET;
        s->evp_regs[i][11] = resetval;
        s->evp_regs[i][12] = resetval;
        s->evp_regs[i][13] = EVP_PRI_IRQ_VEC_0_RESET;
        s->evp_regs[i][14] = resetval;
        s->evp_regs[i][15] = EVP_PRI_IRQ_VEC_1_RESET;
        s->evp_regs[i][16] = resetval;
        s->evp_regs[i][17] = EVP_PRI_IRQ_VEC_2_RESET;
        s->evp_regs[i][18] = resetval;
        s->evp_regs[i][19] = EVP_PRI_IRQ_VEC_3_RESET;
        s->evp_regs[i][20] = resetval;
        s->evp_regs[i][21] = EVP_PRI_IRQ_VEC_4_RESET;
        s->evp_regs[i][22] = resetval;
        s->evp_regs[i][23] = EVP_PRI_IRQ_VEC_5_RESET;
        s->evp_regs[i][24] = resetval;
        s->evp_regs[i][25] = EVP_PRI_IRQ_VEC_6_RESET;
        s->evp_regs[i][26] = resetval;
        s->evp_regs[i][27] = EVP_PRI_IRQ_VEC_7_RESET;
        s->evp_regs[i][28] = resetval;
        s->evp_regs[i][29] = EVP_PRI_FIQ_VEC_0_RESET;
        s->evp_regs[i][30] = resetval;
        s->evp_regs[i][31] = EVP_PRI_FIQ_VEC_1_RESET;
        s->evp_regs[i][32] = resetval;
        s->evp_regs[i][33] = EVP_PRI_FIQ_VEC_2_RESET;
        s->evp_regs[i][34] = resetval;
        s->evp_regs[i][35] = EVP_PRI_FIQ_VEC_3_RESET;
    }

    if (tegra_board >= TEGRAX1_BOARD) {
        for (int cpu_index = 0; cpu_index < 2; cpu_index++) {
            for (i = 1; i < 0x20>>2; i++) {
                s->evp_regs[cpu_index][i] = TEGRA_IROM_BASE + (i<<2);
            }
        }
    }
}

static const MemoryRegionOps tegra_evp_mem_ops = {
    .read = tegra_evp_priv_read,
    .write = tegra_evp_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps tegra_evp_lovec_ops = {
    .read = tegra_evp_lovec_priv_read,
    .write = tegra_evp_lovec_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

#if 0
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
#endif

#ifndef CONFIG_TCG
#error "TCG must be enabled"
#endif

static void tegra_evp_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_evp *s = TEGRA_EVP(dev);
    //CPUState *cs = qemu_get_cpu(TEGRA_BPMP);
    //ARMCPU *cpu = ARM_CPU(cs);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_evp_mem_ops, s,
                          "tegra.evp", TEGRA_EXCEPTION_VECTORS_SIZE);
    memory_region_init_io(&s->lovec_mem, OBJECT(dev), &tegra_evp_lovec_ops, s,
                          "tegra.evp.lovec", 0x1400);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->lovec_mem);

    /* FIXME: lame */
    //device_cold_reset( DEVICE(s) );

    /* COP follows all EVP vectors.  */
    //arm_register_el_change_hook(cpu, tegra_cpu_do_interrupt, cs);
}

static Property tegra_evp_properties[] = {
    DEFINE_PROP_UINT32("cpu-reset-vector", tegra_evp, cpu_reset_vector, EVP_RESET_VECTOR_RESET), \
    DEFINE_PROP_UINT32("bpmp-reset-vector", tegra_evp, bpmp_reset_vector, TEGRA_IROM_BASE), \
    DEFINE_PROP_UINT32("cold-bpmp-reset-vector", tegra_evp, cold_bpmp_reset_vector, TEGRA_IROM_BASE), \
    DEFINE_PROP_END_OF_LIST(),
};

static void tegra_evp_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, tegra_evp_properties);
    dc->realize = tegra_evp_priv_realize;
    dc->vmsd = &vmstate_tegra_evp;
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
