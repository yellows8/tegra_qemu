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
#include "hw/loader.h"
#include "sysemu/runstate.h"
#include "qapi/qmp/qdict.h"
#include "qapi/qmp/qnum.h"
#include "qapi/visitor.h"
#include "qemu/log.h"
#include "qemu/cutils.h"
#include "qapi/error.h"
#include "qemu/error-report.h"

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

#define ARM_INFINITE_LOOP 0xEAFFFFFEULL // "b ."

typedef struct tegra_sb_state {
    SysBusDevice parent_obj;

    QDict cold_reset_ipatches;

    MemoryRegion iomem;
    MemoryRegion ipatch_iomem;
    MemoryRegion irom_iomem;
    uint32_t regs[TEGRA_SB_SIZE>>2];
    uint32_t ipatch_regs[1+IPATCH_MAX];
    uint8_t irom[TEGRA_IROM_SIZE];
} tegra_sb;

static const VMStateDescription vmstate_tegra_sb = {
    .name = "tegra.sb",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, tegra_sb, TEGRA_SB_SIZE>>2),
        VMSTATE_UINT32_ARRAY(ipatch_regs, tegra_sb, 1+IPATCH_MAX),
        VMSTATE_UINT8_ARRAY(irom, tegra_sb, TEGRA_IROM_SIZE),
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
    if (offset == CSR_OFFSET) value |= s->regs[CSR_OFFSET>>2] & 0x10; // PIROM_DISABLE can only be set.

    s->regs[offset/sizeof(uint32_t)] = (s->regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;

    if (offset == CSR_OFFSET) {
        if ((csr_old & s->regs[CSR_OFFSET>>2]) ^ 0x10) { // PIROM_DISABLE is now set.
            hwaddr addr = TEGRA_IROM_BASE + (s->regs[PIROM_START_OFFSET>>2] & ~0x7F);
            cpu_flush_icache_range(addr, TEGRA_IROM_BASE + TEGRA_IROM_SIZE - addr);
            // The actual prot-reads are handled with the irom-read func below.
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
         s->ipatch_regs[offset/sizeof(uint32_t)] = (s->ipatch_regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;

         cpu_flush_icache_range(TEGRA_IROM_BASE, TEGRA_IROM_SIZE);
     }
}

static uint64_t tegra_irom_priv_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    tegra_sb *s = opaque;
    uint64_t ret = 0;
    uint8_t *ret8 = (uint8_t*)&ret;
    uint64_t mask = (1ULL<<size*8)-1;

    if (offset+size <= sizeof(s->irom)) memcpy(&ret, &s->irom[offset], size);

    if (s->regs[CSR_OFFSET>>2] & 0x10) { // PIROM_DISABLE
        hwaddr base = s->regs[PIROM_START_OFFSET>>2] & ~0x7F;
        if (offset+size >= base) { // Not really correct since unaligned access isn't handled, but whatever.
            ret = ARM_INFINITE_LOOP & mask;
        }
    }
    else {
        uint32_t tmp_control = s->ipatch_regs[0];
        for (uint32_t i=0; i<IPATCH_MAX; i++) {
            if (tmp_control & (1<<i)) {
                uint32_t slot_value = s->ipatch_regs[1+i];
                uint16_t patch_value = slot_value & 0xFFFF;
                uint32_t patch_offset = ((slot_value >> 16) & 0xFFFF) << 1;

                if (offset <= patch_offset && offset+size > patch_offset) {
                    size_t tmpsize = size - (patch_offset - offset);
                    if (tmpsize > sizeof(patch_value)) tmpsize = sizeof(patch_value);
                    memcpy(&ret8[patch_offset - offset], &patch_value, tmpsize);
                }
            }
        }
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_irom_priv_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    tegra_sb *s = opaque;

    TRACE_WRITE(s->iomem.addr, offset, 0, value);

    // This is read-only.
}

static void tegra_ipatch_set_cold_reset_ipatches(Object *obj, Visitor *v, const char *name,
                                                 void *opaque, Error **errp)
{
    tegra_sb *s = TEGRA_SB(obj);
    uint64_t value=0;
    char *keyname = NULL;
    char *strvalue = NULL;
    char *strptr = NULL;
    char tmpstr[256]={};

    if (!visit_type_str(v, name, &strvalue, errp)) {
        return;
    }

    strptr = tmpstr;
    keyname = tmpstr;
    pstrcpy(tmpstr, sizeof(tmpstr), strvalue);
    qemu_strsep(&strptr, ":");
    strvalue = strptr;

    if (strvalue == NULL) {
        error_setg(errp, "error reading %s '%s'", name, tmpstr);
        return;
    }

    unsigned long _value=0;
    if (qemu_strtoul(strvalue, NULL, 16, &_value)) {
        error_setg(errp, "error reading %s '%s'", name, strvalue);
        return;
    }
    value = _value;

    QNum *num = qnum_from_uint(value);
    if (num) {
        qdict_put(&s->cold_reset_ipatches, keyname, num);
    }
}

ssize_t tegra_sb_load_irom_file(void *opaque, const char *path)
{
    tegra_sb *s = opaque;

    ssize_t size = get_image_size(path);
    if (size > 0)
        size = load_image_size(path, s->irom, sizeof(s->irom));
    return size;
}

void tegra_sb_load_irom_fixed(void *opaque, const void* buffer, size_t size)
{
    tegra_sb *s = opaque;

    memcpy(s->irom, buffer, size < sizeof(s->irom) ? size : sizeof(s->irom));
}

void tegra_sb_priv_reset(DeviceState *dev, ShutdownCause cause)
{
    tegra_sb *s = TEGRA_SB(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->ipatch_regs, 0, sizeof(s->ipatch_regs));
    s->regs[PIROM_START_OFFSET>>2] = PIROM_START_RESET;

    cpu_flush_icache_range(TEGRA_IROM_BASE, TEGRA_IROM_SIZE);

    // Write the input prop reg values if needed.
    if (cause != SHUTDOWN_CAUSE_GUEST_RESET) {
        for (const QDictEntry *ent = qdict_first(&s->cold_reset_ipatches); ent; ent = qdict_next(&s->cold_reset_ipatches, ent)) {
            const char *keyname = qdict_entry_key(ent);
            QObject *keyobj = qdict_entry_value(ent);

            if (keyname && keyobj) {
                QNum *num = qobject_to(QNum, keyobj);

                if (num) {
                    unsigned long _id=0;
                    uint64_t id=0, value=0;
                    if (!qemu_strtoul(keyname, NULL, 16, &_id) && (id = _id) < IPATCH_MAX) {
                        value = qnum_get_uint(num);
                        qemu_log_mask(LOG_GUEST_ERROR, "tegra.sb: Writing ipatch for cold-reset-ipatches: ipatch id 0x%" PRIx64 " = reg value 0x%" PRIx64 ".\n", id, value);
                        s->ipatch_regs[1+id] = value;
                        s->ipatch_regs[0] |= BIT(id);
                    }
                }
            }
        }
    }
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

static const MemoryRegionOps tegra_irom_mem_ops = {
    .read = tegra_irom_priv_read,
    .write = tegra_irom_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_sb_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_sb *s = TEGRA_SB(dev);

    memset(s->irom, 0, sizeof(s->irom));

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_sb_mem_ops, s,
                          "tegra.sb", TEGRA_SB_SIZE);
    memory_region_init_io(&s->ipatch_iomem, OBJECT(dev), &tegra_ipatch_mem_ops, s,
                          "tegra.ipatch", TEGRA_IPATCH_SIZE);
    memory_region_init_io(&s->irom_iomem, OBJECT(dev), &tegra_irom_mem_ops, s,
                          "tegra.irom", TEGRA_IROM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->ipatch_iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->irom_iomem);
}

static void tegra_sb_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_sb_priv_realize;
    dc->vmsd = &vmstate_tegra_sb;

    object_class_property_add(klass, "cold-reset-ipatches", "uint",
                              NULL,
                              tegra_ipatch_set_cold_reset_ipatches, NULL, NULL);
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
