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

#include "hw/usb/hcd-ehci.h"

#include "usb.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_USB "tegra.usb"
#define TEGRA_USB(obj) OBJECT_CHECK(tegra_usb, (obj), TYPE_TEGRA_USB)
#define DEFINE_REG32(reg) reg##_t reg

typedef struct tegra_usb_state {
    EHCISysBusState parent_obj;

    uint32_t regs_config[0x100>>2];

    uint32_t regs_controller[(0x260-0x1B4)>>2];
    DEFINE_REG32(usb1_if_usb_susp_ctrl);
    uint32_t regs_susp[(0x85C-0x404)>>2];
    uint32_t regs_queue[0x1000>>2];

    MemoryRegion iomem_config;
    MemoryRegion iomem_controller;
    MemoryRegion iomem;
    MemoryRegion iomem_queue;
} tegra_usb;

static const VMStateDescription vmstate_tegra_usb = {
    .name = "tegra.usb",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(ehci, EHCISysBusState, 2, vmstate_ehci, EHCIState),
        VMSTATE_UINT32(usb1_if_usb_susp_ctrl.reg32, tegra_usb),
        VMSTATE_UINT32_ARRAY(regs_config, tegra_usb, 0x100>>2),
        VMSTATE_UINT32_ARRAY(regs_controller, tegra_usb, (0x260-0x1B4)>>2),
        VMSTATE_UINT32_ARRAY(regs_susp, tegra_usb, (0x85C-0x404)>>2),
        VMSTATE_UINT32_ARRAY(regs_queue, tegra_usb, 0x1000>>2),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_usb_config_priv_read(void *opaque, hwaddr offset,
                                           unsigned size)
{
    tegra_usb *s = opaque;
    uint64_t ret = 0;

    assert(offset+size <= sizeof(s->regs_config));

    ret = s->regs_config[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    TRACE_READ(s->iomem_config.addr, offset, ret);

    return ret;
}

static void tegra_usb_config_priv_write(void *opaque, hwaddr offset,
                                        uint64_t value, unsigned size)
{
    tegra_usb *s = opaque;

    assert(offset+size <= sizeof(s->regs_config));

    TRACE_WRITE(s->iomem_config.addr, offset, 0, value);

    if (offset >= 0x80) {
        s->regs_config[offset/sizeof(uint32_t)] = (s->regs_config[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;
    }
}

static uint64_t tegra_usb_controller_priv_read(void *opaque, hwaddr offset,
                                          unsigned size)
{
    tegra_usb *s = opaque;
    uint64_t ret = 0;

    assert(offset+size <= sizeof(s->regs_controller));

    ret = s->regs_controller[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    TRACE_READ(s->iomem_controller.addr, offset, ret);

    return ret;
}

static void tegra_usb_controller_priv_write(void *opaque, hwaddr offset,
                                       uint64_t value, unsigned size)
{
    tegra_usb *s = opaque;

    assert(offset+size <= sizeof(s->regs_controller));

    TRACE_WRITE(s->iomem_controller.addr, offset, 0, value);

    s->regs_controller[offset/sizeof(uint32_t)] = (s->regs_controller[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;
}

static uint64_t tegra_usb_priv_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    tegra_usb *s = opaque;
    uint64_t ret = 0;

    switch (offset + 0x400) {
    case USB1_IF_USB_SUSP_CTRL_OFFSET:
        ret = s->usb1_if_usb_susp_ctrl.reg32;
        break;
    default:
        if (offset+size <= sizeof(s->regs_susp)+0x4) {
            offset-=0x4;
            ret = s->regs_config[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);
        }
        break;
    }

//     TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_usb_priv_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    tegra_usb *s = opaque;
    usb1_if_usb_susp_ctrl_t old = s->usb1_if_usb_susp_ctrl;

    switch (offset + 0x400) {
    case USB1_IF_USB_SUSP_CTRL_OFFSET:
//         TRACE_WRITE(s->iomem.addr, offset, s->usb1_if_usb_susp_ctrl.reg32, value & USB1_IF_USB_SUSP_CTRL_WRMASK);
        s->usb1_if_usb_susp_ctrl.reg32 = value & USB1_IF_USB_SUSP_CTRL_WRMASK;

        s->usb1_if_usb_susp_ctrl.usb_susp_set ^= old.usb_susp_set;

        if (s->usb1_if_usb_susp_ctrl.utmip_reset ||
            s->usb1_if_usb_susp_ctrl.usb_susp_set) {

            s->usb1_if_usb_susp_ctrl.usb_phy_clk_valid = 0;

        } else if (!s->usb1_if_usb_susp_ctrl.utmip_reset ||
                   s->usb1_if_usb_susp_ctrl.usb_susp_clr) {

            s->usb1_if_usb_susp_ctrl.usb_phy_clk_valid = 1;
        }
        break;
    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        if (offset+size <= sizeof(s->regs_susp)+0x4) {
            offset-=0x4;
            s->regs_susp[offset/sizeof(uint32_t)] = (s->regs_susp[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;
        }
        break;
    }
}

static uint64_t tegra_usb_queue_priv_read(void *opaque, hwaddr offset,
                                          unsigned size)
{
    tegra_usb *s = opaque;
    uint64_t ret = 0;

    assert(offset+size <= sizeof(s->regs_queue));

    ret = s->regs_queue[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    TRACE_READ(s->iomem_queue.addr, offset, ret);

    return ret;
}

static void tegra_usb_queue_priv_write(void *opaque, hwaddr offset,
                                       uint64_t value, unsigned size)
{
    tegra_usb *s = opaque;

    assert(offset+size <= sizeof(s->regs_queue));

    TRACE_WRITE(s->iomem_queue.addr, offset, 0, value);

    s->regs_queue[offset/sizeof(uint32_t)] = (s->regs_queue[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;
}

static void tegra_usb_priv_reset(DeviceState *dev)
{
    EHCISysBusState *i = SYS_BUS_EHCI(dev);
    tegra_usb *s = TEGRA_USB(dev);
    EHCIState *ehci = &i->ehci;

    s->usb1_if_usb_susp_ctrl.reg32 = USB1_IF_USB_SUSP_CTRL_RESET;

    memset(s->regs_config, 0, sizeof(s->regs_config));
    memset(s->regs_controller, 0, sizeof(s->regs_controller));
    memset(s->regs_susp, 0, sizeof(s->regs_susp));
    memset(s->regs_queue, 0, sizeof(s->regs_queue));

    ehci_reset(ehci);
}

static const MemoryRegionOps tegra_usb_config_mem_ops = {
    .read = tegra_usb_config_priv_read,
    .write = tegra_usb_config_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps tegra_usb_controller_mem_ops = {
    .read = tegra_usb_controller_priv_read,
    .write = tegra_usb_controller_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps tegra_usb_mem_ops = {
    .read = tegra_usb_priv_read,
    .write = tegra_usb_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps tegra_usb_queue_mem_ops = {
    .read = tegra_usb_queue_priv_read,
    .write = tegra_usb_queue_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_usb_priv_init(Object *obj)
{
    EHCISysBusState *i = SYS_BUS_EHCI(obj);
    tegra_usb *s = TEGRA_USB(obj);
    EHCIState *ehci = &i->ehci;

    if (tegra_board >= TEGRAX1_BOARD) {
        ehci->capsbase  = 0x100;
        ehci->opregbase = 0x130;
        ehci->portscbase = 0x174 - ehci->opregbase;
        ehci->portnr = 1;
    }
    else {
        ehci->capsbase  = 0x100;
        ehci->opregbase = 0x140;
    }

    memory_region_init_io(&s->iomem_config, OBJECT(s), &tegra_usb_config_mem_ops, s,
                          "tegra.usb_config", 0x100);
    memory_region_init_io(&s->iomem_controller, OBJECT(s), &tegra_usb_controller_mem_ops, s,
                          "tegra.usb_controller", 0x260-0x1B4);
    memory_region_init_io(&s->iomem, OBJECT(s), &tegra_usb_mem_ops, s,
                          "tegra.usb_susp_ctrl", 0x85C-0x400);
    memory_region_init_io(&s->iomem_queue, OBJECT(s), &tegra_usb_queue_mem_ops, s,
                          "tegra.usb_queue", 0x1000);
    memory_region_add_subregion(&ehci->mem, 0x0, &s->iomem_config);
    memory_region_add_subregion(&ehci->mem, 0x1B4, &s->iomem_controller);
    memory_region_add_subregion(&ehci->mem, 0x400, &s->iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem_queue);
}

static void tegra_usb_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_tegra_usb;
    dc->reset = tegra_usb_priv_reset;

    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

static const TypeInfo tegra_usb_info = {
    .name = TYPE_TEGRA_USB,
    .parent = TYPE_SYS_BUS_EHCI,
    .instance_size = sizeof(tegra_usb),
    .instance_init = tegra_usb_priv_init,
    .class_init = tegra_usb_class_init,
};

static void tegra_usb_register_types(void)
{
    type_register_static(&tegra_usb_info);
}

type_init(tegra_usb_register_types)
