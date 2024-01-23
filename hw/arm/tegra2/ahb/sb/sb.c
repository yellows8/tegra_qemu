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

#define CONFIG_ARCH_TEGRA_21x_SOC

#include "tegra_common.h"

#include "hw/sysbus.h"
#include "exec/address-spaces.h"

#include "iomap.h"
#include "tegra_cpu.h"
#include "tegra_trace.h"
#include "devices.h"

#include "sb.h"

#define TYPE_TEGRA_SB "tegra.sb"
#define TEGRA_SB(obj) OBJECT_CHECK(tegra_sb, (obj), TYPE_TEGRA_SB)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

#define IPATCH_MAX 32 // Not sure what the actual max is but whatever.

typedef struct tegra_sb_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion ipatch_iomem;
    uint32_t regs[TEGRA_SB_SIZE>>2];
    uint32_t ipatch_regs[1+IPATCH_MAX];
    uint16_t ipatch_original_values[IPATCH_MAX];
} tegra_sb;

static const VMStateDescription vmstate_tegra_sb = {
    .name = "tegra.sb",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, tegra_sb, TEGRA_SB_SIZE>>2),
        VMSTATE_UINT32_ARRAY(ipatch_regs, tegra_sb, 1+IPATCH_MAX),
        VMSTATE_UINT16_ARRAY(ipatch_original_values, tegra_sb, IPATCH_MAX),
        VMSTATE_END_OF_LIST()
    }
};

uint64_t tegra_sb_get_cpu_reset_vector(void) {
    tegra_sb *s = TEGRA_SB(tegra_sb_dev);

    return *((uint64_t*)&s->regs[AA64_RESET_LOW_OFFSET>>2]) & 0xFFFFFFFFFFFULL;
}

static uint64_t tegra_sb_priv_read(void *opaque, hwaddr offset,
                                   unsigned size)
{
    tegra_sb *s = opaque;
    uint64_t ret = 0;

    assert(offset < sizeof(s->regs));

    ret = s->regs[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_sb_priv_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    tegra_sb *s = opaque;

    assert(offset < sizeof(s->regs));

    TRACE_WRITE(s->iomem.addr, offset, 0, value);

    uint32_t csr_old = s->regs[CSR_OFFSET>>2];
    uint32_t patch_value = 0xeafffffe; // "b ."
    if (offset == CSR_OFFSET) value |= s->regs[CSR_OFFSET>>2] & 0x10; // PIROM_DISABLE can only be set.

    s->regs[offset/sizeof(uint32_t)] = (s->regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;

    if (offset == CSR_OFFSET) {
        if ((csr_old & s->regs[CSR_OFFSET>>2]) ^ 0x10) { // PIROM_DISABLE is now set.
            hwaddr addr = TEGRA_IROM_BASE + (s->regs[PIROM_START_OFFSET>>2] & ~0x7F);
            for (; addr < TEGRA_IROM_BASE + TEGRA_IROM_SIZE; addr+=4) {
                address_space_write_rom(&address_space_memory, addr,
                                        MEMTXATTRS_UNSPECIFIED, &patch_value,
                                        sizeof(patch_value));
            }
        }
    }
}

static uint64_t tegra_ipatch_priv_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    tegra_sb *s = opaque;
    uint64_t ret = 0;

    if (offset+size <= sizeof(s->ipatch_regs)) ret = s->ipatch_regs[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_ipatch_priv_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    tegra_sb *s = opaque;

    TRACE_WRITE(s->iomem.addr, offset, 0, value);

     if (offset+size <= sizeof(s->ipatch_regs)) {
         uint32_t old_control = s->ipatch_regs[0];
         s->ipatch_regs[offset/sizeof(uint32_t)] = (s->ipatch_regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;

         if (offset == 0 && old_control != s->ipatch_regs[0]) { // We assume the IPATCH slot regs don't change while the slot is already enabled.
             uint32_t tmp_control = old_control ^ s->ipatch_regs[0];
             for (uint32_t i=0; i<IPATCH_MAX; i++) {
                 if (tmp_control & (1<<i)) {
                     bool was_enabled = (old_control & (1<<i)) != 0;
                     uint32_t slot_value = s->ipatch_regs[1+i];
                     uint16_t patch_value = slot_value & 0xFFFF;
                     uint32_t patch_offset = ((slot_value >> 16) & 0xFFFF) << 1;
                     hwaddr patch_addr = TEGRA_IROM_BASE + patch_offset;

                     if (!was_enabled) { // disabled -> enabled
                         address_space_rw(&address_space_memory, patch_addr,
                                          MEMTXATTRS_UNSPECIFIED, &s->ipatch_original_values[i],
                                          sizeof(s->ipatch_original_values[i]), false);
                     }
                     else patch_value = s->ipatch_original_values[i]; // enabled -> disabled

                     address_space_write_rom(&address_space_memory, patch_addr,
                                             MEMTXATTRS_UNSPECIFIED, &patch_value,
                                             sizeof(patch_value));
                 }
             }
         }
     }
}

static void tegra_sb_priv_reset(DeviceState *dev)
{
    tegra_sb *s = TEGRA_SB(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->ipatch_regs, 0, sizeof(s->ipatch_regs));
    s->regs[PIROM_START_OFFSET>>2] = PIROM_START_RESET;
}

static const MemoryRegionOps tegra_sb_mem_ops = {
    .read = tegra_sb_priv_read,
    .write = tegra_sb_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps tegra_ipatch_mem_ops = {
    .read = tegra_ipatch_priv_read,
    .write = tegra_ipatch_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_sb_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_sb *s = TEGRA_SB(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_sb_mem_ops, s,
                          "tegra.sb", TEGRA_SB_SIZE);
    memory_region_init_io(&s->ipatch_iomem, OBJECT(dev), &tegra_ipatch_mem_ops, s,
                          "tegra.ipatch", TEGRA_IPATCH_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->ipatch_iomem);
}

static void tegra_sb_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_sb_priv_realize;
    dc->vmsd = &vmstate_tegra_sb;
    dc->reset = tegra_sb_priv_reset;
}

static const TypeInfo tegra_sb_info = {
    .name = TYPE_TEGRA_SB,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_sb),
    .class_init = tegra_sb_class_init,
};

static void tegra_sb_register_types(void)
{
    type_register_static(&tegra_sb_info);
}

type_init(tegra_sb_register_types)
