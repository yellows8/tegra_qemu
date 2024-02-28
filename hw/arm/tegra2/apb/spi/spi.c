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

#include "tegra_common.h"

#include "hw/sysbus.h"

#include "qapi/error.h"
#include "qapi/qapi-commands.h"

#include "iomap.h"
#include "tegra_trace.h"
#include "devices.h"

#include "qemu/cutils.h"
#include "qemu/log.h"

#include "spi.h"

#define TYPE_TEGRA_SPI "tegra.spi"
#define TEGRA_SPI(obj) OBJECT_CHECK(tegra_spi, (obj), TYPE_TEGRA_SPI)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_spi_state {
    SysBusDevice parent_obj;

    qemu_irq irq;
    MemoryRegion iomem;

    uint32_t regs[0x194>>2];

} tegra_spi;

static const VMStateDescription vmstate_tegra_spi = {
    .name = TYPE_TEGRA_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, tegra_spi, 0x194>>2),

        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_spi_priv_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    tegra_spi *s = opaque;
    uint64_t ret = 0;

    TRACE_READ(s->iomem.addr, offset, ret);

    if (offset+size <= sizeof(s->regs)) ret = s->regs[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    return ret;
}

static void tegra_spi_priv_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    tegra_spi *s = opaque;

    TRACE_WRITE(s->iomem.addr, offset, 0, value);

    // TODO: IRQ

    if (offset+size <= sizeof(s->regs)) {
        if (offset == SPI_COMMAND_OFFSET) {
            if (value & BIT(31)) { // PIO = GO
                value &= ~BIT(31);
                s->regs[SPI_TRANSFER_STATUS_OFFSET>>2] |= BIT(30); //RDY
                s->regs[SPI_TRANSFER_STATUS_OFFSET>>2] = (s->regs[SPI_TRANSFER_STATUS_OFFSET>>2] & ~0xFFFF) | (s->regs[SPI_DMA_BLK_SIZE_OFFSET>>2] & 0xFFFF);
            }
        }
        else if (offset == SPI_TRANSFER_STATUS_OFFSET) {
            if (value & BIT(30)) value &= ~BIT(30); // RDY
        }
        else if (offset == SPI_FIFO_STATUS_OFFSET) {
            value &= ~0x3FFF000F;
            value |= s->regs[SPI_FIFO_STATUS_OFFSET>>2] & 0x3FFF000F;
        }
        else if (offset == SPI_DMA_CTL_OFFSET) {
            if (value & BIT(31)) { // DMA = enable
                value &= ~BIT(31);
                s->regs[SPI_TRANSFER_STATUS_OFFSET>>2] |= BIT(30); //RDY
                s->regs[SPI_TRANSFER_STATUS_OFFSET>>2] = (s->regs[SPI_TRANSFER_STATUS_OFFSET>>2] & ~0xFFFF) | (s->regs[SPI_DMA_BLK_SIZE_OFFSET>>2] & 0xFFFF);
            }
        }

        s->regs[offset/sizeof(uint32_t)] = (s->regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;
    }
}

static void tegra_spi_priv_reset(DeviceState *dev)
{
    tegra_spi *s = TEGRA_SPI(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[SPI_COMMAND_OFFSET>>2] = SPI_COMMAND_RESET;
    s->regs[SPI_TIMING_REG2_OFFSET>>2] = SPI_TIMING_REG2_RESET;
    s->regs[SPI_TRANSFER_STATUS_OFFSET>>2] = SPI_TRANSFER_STATUS_RESET;
    s->regs[SPI_FIFO_STATUS_OFFSET>>2] = SPI_FIFO_STATUS_RESET;
    s->regs[SPI_SPARE_CTLR_OFFSET>>2] = SPI_SPARE_CTLR_RESET;
}

static const MemoryRegionOps tegra_spi_mem_ops = {
    .read = tegra_spi_priv_read,
    .write = tegra_spi_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_spi_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_spi *s = TEGRA_SPI(dev);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_spi_mem_ops, s,
                          TYPE_TEGRA_SPI, TEGRA_SPI1_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_spi_priv_realize;
    dc->vmsd = &vmstate_tegra_spi;
    dc->reset = tegra_spi_priv_reset;
}

static const TypeInfo tegra_spi_info = {
    .name = TYPE_TEGRA_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_spi),
    .class_init = tegra_spi_class_init,
};

static void tegra_spi_register_types(void)
{
    type_register_static(&tegra_spi_info);
}

type_init(tegra_spi_register_types)
