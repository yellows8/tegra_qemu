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

#include "hw/arm/boot.h"
#include "hw/sysbus.h"
#include "cpu.h"
#include "exec/exec-all.h"

#include "tcg/tcg-op.h"
#include "sysemu/tcg.h"

#include "devices.h"
#include "sizes.h"
#include "tegra_cpu.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_COP_MMU "tegra.cop_mmu"
#define TEGRA_COP_MMU(obj) OBJECT_CHECK(tegra_cop_mmu, (obj), TYPE_TEGRA_COP_MMU)

#define PTE0_COMPARE    0xF000
#define PTE0_TRANSLATE  0xF004

#define TRANSLATE_DATA  (1 << 11)
#define TRANSLATE_CODE  (1 << 10)
#define TRANSLATE_WR    (1 << 9)
#define TRANSLATE_RD    (1 << 8)
#define TRANSLATE_HIT   (1 << 7)
#define TRANSLATE_EN    (1 << 2)

typedef struct tegra_cop_mmu_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    uint16_t translate_virt_base;
    uint16_t translate_phys_base;
    uint16_t translate_flags;
    uint16_t translate_mask;
} tegra_cop_mmu;

static const VMStateDescription vmstate_tegra_cop_mmu = {
    .name = "tegra.cop_mmu",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT16(translate_virt_base, tegra_cop_mmu),
        VMSTATE_UINT16(translate_phys_base, tegra_cop_mmu),
        VMSTATE_UINT16(translate_flags, tegra_cop_mmu),
        VMSTATE_UINT16(translate_mask, tegra_cop_mmu),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_cop_mmu_priv_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    tegra_cop_mmu *s = opaque;
    uint64_t ret = 0;

    if (current_cpu != qemu_get_cpu(TEGRA_BPMP)) {
        return ret;
    }

    switch (offset) {
    case PTE0_COMPARE:
        ret = (s->translate_virt_base << 16) | s->translate_mask;
        break;
    case PTE0_TRANSLATE:
        ret = (s->translate_phys_base << 16) | s->translate_flags;
        break;
    default:
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_cop_mmu_priv_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    tegra_cop_mmu *s = opaque;
    uint32_t old __attribute__ ((unused));

    /* MMU is in main address space for simplicity. Avoid CPU access.  */
    if (current_cpu != qemu_get_cpu(TEGRA_BPMP)) {
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        return;
    }

    switch (offset) {
    case PTE0_COMPARE:
        old = (s->translate_virt_base << 16) | s->translate_mask;
        TRACE_WRITE(s->iomem.addr, offset, old, value);

        s->translate_virt_base = value >> 16;
        s->translate_mask = value & 0x3FFF;
        break;
    case PTE0_TRANSLATE:
        old = (s->translate_phys_base << 16) | s->translate_flags;
        TRACE_WRITE(s->iomem.addr, offset, old, value);

        s->translate_phys_base = (value >> 16) & 0x3FFF;
        s->translate_flags = value & 0xFFF;

        if (s->translate_flags & TRANSLATE_EN) {
            /* Too much churn to implement correctly.  */
            g_assert(s->translate_flags & TRANSLATE_DATA);
            g_assert(s->translate_flags & TRANSLATE_HIT);
            g_assert(s->translate_flags & TRANSLATE_RD);
            g_assert(s->translate_flags & TRANSLATE_WR);
            g_assert(s->translate_flags & TRANSLATE_CODE);
        }
        break;
    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        return;
    }

    tlb_flush(current_cpu);

    if (tcg_enabled()) {
        tcg_gen_goto_tb(0);
        tcg_gen_exit_tb(NULL, 0);
    }
}

static void tegra_cop_mmu_priv_reset(DeviceState *dev)
{
    tegra_cop_mmu *s = TEGRA_COP_MMU(dev);

    s->translate_virt_base = 0;
    s->translate_phys_base = 0;
    s->translate_flags = 0;
    s->translate_mask = 0;
}

static const MemoryRegionOps tegra_cop_mmu_mem_ops = {
    .read = tegra_cop_mmu_priv_read,
    .write = tegra_cop_mmu_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static hwaddr tegra_cop_mmu_translate(hwaddr addr, int access_type)
{
    tegra_cop_mmu *s = tegra_cop_mmu_dev;
    hwaddr phys_base = s->translate_phys_base & s->translate_mask;
    hwaddr virt_base = s->translate_virt_base;
//     hwaddr orig = addr;

    if (!(s->translate_flags & TRANSLATE_EN)) {
        return addr;
    }

    if (((addr >> 16) & s->translate_mask) != virt_base) {
//         printf("NOT! translated 0x%08X\n", (uint32_t) addr);
        return addr;
    }

    addr &= ~((0xC000 | s->translate_mask) << 16);
    addr |= (0x8000 | phys_base) << 16;

//     printf("translate 0x%08X -> 0x%08X\n", (uint32_t) orig, (uint32_t) addr);

    return addr;
}

static void tegra_cop_mmu_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_cop_mmu *s = TEGRA_COP_MMU(dev);
    CPUState *cs = qemu_get_cpu(TEGRA_BPMP);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_cop_mmu_mem_ops, s,
                          "tegra.cop_mmu", SZ_64K);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    ARM_CPU(cs)->translate_addr = tegra_cop_mmu_translate;
}

static void tegra_cop_mmu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_cop_mmu_priv_realize;
    dc->vmsd = &vmstate_tegra_cop_mmu;
    dc->reset = tegra_cop_mmu_priv_reset;
}

static const TypeInfo tegra_cop_mmu_info = {
    .name = TYPE_TEGRA_COP_MMU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_cop_mmu),
    .class_init = tegra_cop_mmu_class_init,
};

static void tegra_cop_mmu_register_types(void)
{
    type_register_static(&tegra_cop_mmu_info);
}

type_init(tegra_cop_mmu_register_types)
