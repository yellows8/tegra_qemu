/* "Unimplemented" device
 *
 * This is a dummy device which accepts and logs all accesses.
 * It's useful for stubbing out regions of an SoC or board
 * map which correspond to devices that have not yet been
 * implemented. This is often sufficient to placate initial
 * guest device driver probing such that the system will
 * come up.
 *
 * Copyright Linaro Limited, 2017
 * Written by Peter Maydell
 */

// Based on unimp, returns a fixed value.

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/misc/fixedmem.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"

static uint64_t fixedmem_read(void *opaque, hwaddr offset, unsigned size)
{
    FixedmemDeviceState *s = FIXEDMEM_DEVICE(opaque);

    qemu_log_mask(LOG_UNIMP, "%s: fixedmem device read  "
                  "(size %d, offset 0x%0*" HWADDR_PRIx ")\n",
                  s->name, size, s->offset_fmt_width, offset);
    uint64_t ret = s->value;

    offset &= 0x7;
    if (offset) {
        ret = (ret >> offset*8) | (ret << (64-(offset*8)));
    }

    if (size < 8)
        ret &= ((1ULL<<size*8)-1);

    return ret;
}

static void fixedmem_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    FixedmemDeviceState *s = FIXEDMEM_DEVICE(opaque);

    qemu_log_mask(LOG_UNIMP, "%s: fixedmem device write "
                  "(size %d, offset 0x%0*" HWADDR_PRIx
                  ", value 0x%0*" PRIx64 ")\n",
                  s->name, size, s->offset_fmt_width, offset, size << 1, value);
}

static const MemoryRegionOps fixedmem_ops = {
    .read = fixedmem_read,
    .write = fixedmem_write,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void fixedmem_realize(DeviceState *dev, Error **errp)
{
    FixedmemDeviceState *s = FIXEDMEM_DEVICE(dev);

    if (s->size == 0) {
        error_setg(errp, "property 'size' not specified or zero");
        return;
    }

    if (s->name == NULL) {
        error_setg(errp, "property 'name' not specified");
        return;
    }

    s->offset_fmt_width = DIV_ROUND_UP(64 - clz64(s->size - 1), 4);

    memory_region_init_io(&s->iomem, OBJECT(s), &fixedmem_ops, s,
                          s->name, s->size);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static Property fixedmem_properties[] = {
    DEFINE_PROP_UINT64("size", FixedmemDeviceState, size, 0),
    DEFINE_PROP_STRING("name", FixedmemDeviceState, name),
    DEFINE_PROP_UINT64("value", FixedmemDeviceState, value, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void fixedmem_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = fixedmem_realize;
    device_class_set_props(dc, fixedmem_properties);
}

static const TypeInfo fixedmem_info = {
    .name = TYPE_FIXEDMEM_DEVICE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(FixedmemDeviceState),
    .class_init = fixedmem_class_init,
};

static void fixedmem_register_types(void)
{
    type_register_static(&fixedmem_info);
}

type_init(fixedmem_register_types)
