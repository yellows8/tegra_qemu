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

#include "hw/sysbus.h"

#include "uart.h"
#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_UART "tegra.uart"
#define TEGRA_UART(obj) OBJECT_CHECK(tegra_uart, (obj), TYPE_TEGRA_UART)
#define DEFINE_REG32(reg) reg##_t reg

typedef struct tegra_uart_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    DEFINE_REG32(thr_dlab);
    DEFINE_REG32(ier_dlab);
    DEFINE_REG32(iir_fcr);
    DEFINE_REG32(lcr);
    DEFINE_REG32(mcr);
    DEFINE_REG32(lsr);
    DEFINE_REG32(msr);
    DEFINE_REG32(spr);
    DEFINE_REG32(irda_csr);
    DEFINE_REG32(rx_fifo_cfg);
    DEFINE_REG32(mie);
    DEFINE_REG32(vendor_status);
    DEFINE_REG32(asr);
} tegra_uart;

static const VMStateDescription vmstate_tegra_uart = {
    .name = "tegra.uart",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(thr_dlab.reg32, tegra_uart),
        VMSTATE_UINT32(ier_dlab.reg32, tegra_uart),
        VMSTATE_UINT32(iir_fcr.reg32, tegra_uart),
        VMSTATE_UINT32(lcr.reg32, tegra_uart),
        VMSTATE_UINT32(mcr.reg32, tegra_uart),
        VMSTATE_UINT32(lsr.reg32, tegra_uart),
        VMSTATE_UINT32(msr.reg32, tegra_uart),
        VMSTATE_UINT32(spr.reg32, tegra_uart),
        VMSTATE_UINT32(irda_csr.reg32, tegra_uart),
        VMSTATE_UINT32(rx_fifo_cfg.reg32, tegra_uart),
        VMSTATE_UINT32(mie.reg32, tegra_uart),
        VMSTATE_UINT32(vendor_status.reg32, tegra_uart),
        VMSTATE_UINT32(asr.reg32, tegra_uart),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_uart_priv_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    tegra_uart *s = opaque;
    uint64_t ret = 0;

    offset+= 0x20; // MMIO is mapped to base+0x20.

    switch (offset) {
    case THR_DLAB_OFFSET:
        ret = s->thr_dlab.reg32;
        break;
    case IER_DLAB_OFFSET:
        ret = s->ier_dlab.reg32;
        break;
    case IIR_FCR_OFFSET:
        ret = s->iir_fcr.reg32;
        break;
    case LCR_OFFSET:
        ret = s->lcr.reg32;
        break;
    case MCR_OFFSET:
        ret = s->mcr.reg32;
        break;
    case LSR_OFFSET:
        ret = s->lsr.reg32;
        break;
    case MSR_OFFSET:
        ret = s->msr.reg32;
        break;
    case SPR_OFFSET:
        ret = s->spr.reg32;
        break;
    case IRDA_CSR_OFFSET:
        ret = s->irda_csr.reg32;
        break;
    case RX_FIFO_CFG_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->rx_fifo_cfg.reg32;
        break;
    case MIE_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->mie.reg32;
        break;
    case VENDOR_STATUS_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = s->vendor_status.reg32;
        break;
    case ASR_OFFSET:
        ret = s->asr.reg32;
        break;
    default:
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_uart_priv_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    tegra_uart *s = opaque;

    offset+= 0x20;

    switch (offset) {
    case THR_DLAB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->thr_dlab.reg32, value);
        s->thr_dlab.reg32 = value;
        break;
    case IER_DLAB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->ier_dlab.reg32, value);
        s->ier_dlab.reg32 = value;
        break;
    case IIR_FCR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->iir_fcr.reg32, value);
        s->iir_fcr.reg32 = value;
        break;
    case LCR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->lcr.reg32, value);
        s->lcr.reg32 = value;
        break;
    case MCR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->mcr.reg32, value);
        s->mcr.reg32 = value;
        break;
    case MSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->msr.reg32, value);
        s->msr.reg32 = value;
        break;
    case SPR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->spr.reg32, value);
        s->spr.reg32 = value;
        break;
    case IRDA_CSR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->irda_csr.reg32, value);
        s->irda_csr.reg32 = value;
        break;
    case RX_FIFO_CFG_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->rx_fifo_cfg.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->rx_fifo_cfg.reg32 = value;
        break;
    case MIE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->mie.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->mie.reg32 = value;
        break;
    case VENDOR_STATUS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->vendor_status.reg32, value);
        if (tegra_board >= TEGRAX1_BOARD) s->vendor_status.reg32 = value;
        break;
    case ASR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->asr.reg32, value);
        s->asr.reg32 = value;
        break;
    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }
}

static void tegra_uart_priv_reset(DeviceState *dev)
{
    tegra_uart *s = TEGRA_UART(dev);

    s->thr_dlab.reg32 = THR_DLAB_RESET;
    s->ier_dlab.reg32 = IER_DLAB_RESET;
    s->iir_fcr.reg32 = IIR_FCR_RESET;
    s->lcr.reg32 = LCR_RESET;
    s->mcr.reg32 = MCR_RESET;
    s->lsr.reg32 = LSR_RESET;
    s->msr.reg32 = MSR_RESET;
    s->spr.reg32 = SPR_RESET;
    s->irda_csr.reg32 = IRDA_CSR_RESET;
    s->rx_fifo_cfg.reg32 = RX_FIFO_CFG_RESET;
    s->mie.reg32 = MIE_RESET;
    s->vendor_status.reg32 = VENDOR_STATUS_RESET;
    s->asr.reg32 = ASR_RESET;
}

static const MemoryRegionOps tegra_uart_mem_ops = {
    .read = tegra_uart_priv_read,
    .write = tegra_uart_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_uart_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_uart *s = TEGRA_UART(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_uart_mem_ops, s,
                          "tegra.uart", 0x20);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_uart_priv_realize;
    dc->vmsd = &vmstate_tegra_uart;
    dc->reset = tegra_uart_priv_reset;
}

static const TypeInfo tegra_uart_info = {
    .name = TYPE_TEGRA_UART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_uart),
    .class_init = tegra_uart_class_init,
};

static void tegra_uart_register_types(void)
{
    type_register_static(&tegra_uart_info);
}

type_init(tegra_uart_register_types)
