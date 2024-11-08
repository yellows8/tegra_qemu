/*
 * "Unimplemented" device
 *
 * Copyright Linaro Limited, 2017
 * Written by Peter Maydell
 */

// Based on unimp.

#ifndef HW_MISC_FIXEDMEM_H
#define HW_MISC_FIXEDMEM_H

#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "qom/object.h"

#define TYPE_FIXEDMEM_DEVICE "fixedmem-device"

OBJECT_DECLARE_SIMPLE_TYPE(FixedmemDeviceState, FIXEDMEM_DEVICE)

struct FixedmemDeviceState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    unsigned offset_fmt_width;
    char *name;
    uint64_t size;
    uint64_t value;
};

/**
 * create_fixedmem_device: create and map a dummy device
 * @name: name of the device for debug logging
 * @base: base address of the device's MMIO region
 * @size: size of the device's MMIO region
 * @value: value for device's MMIO reads
 *
 * This utility function creates and maps an instance of fixedmem-device,
 * which is a dummy device which simply logs all guest accesses to
 * it via the qemu_log LOG_UNIMP debug log.
 * The device is mapped at priority -1000, which means that you can
 * use it to cover a large region and then map other devices on top of it
 * if necessary.
 */
static inline void create_fixedmem_device(const char *name,
                                               hwaddr base,
                                               hwaddr size,
                                               uint64_t value)
{
    DeviceState *dev = qdev_new(TYPE_FIXEDMEM_DEVICE);

    qdev_prop_set_string(dev, "name", name);
    qdev_prop_set_uint64(dev, "size", size);
    qdev_prop_set_uint64(dev, "value", value);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, base, -1000);
}

#endif
