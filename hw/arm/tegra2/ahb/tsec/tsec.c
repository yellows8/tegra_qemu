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

#include "crypto/secret_common.h"
#include "qapi/error.h"
#include "exec/address-spaces.h"
#include "sysemu/dma.h"

#include "iomap.h"
#include "tegra_trace.h"

#include "tsec.h"

#define TYPE_TEGRA_TSEC "tegra.tsec"
#define TEGRA_TSEC(obj) OBJECT_CHECK(tegra_tsec, (obj), TYPE_TEGRA_TSEC)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_tsec_state {
    SysBusDevice parent_obj;

    qemu_irq irq;
    MemoryRegion iomem;
    int32_t engine;
    uint32_t regs[TEGRA_TSEC_SIZE>>2];
    bool outdata_set;
    uint32_t outdata[4];
} tegra_tsec;

static const VMStateDescription vmstate_tegra_tsec = {
    .name = "tegra.tsec",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_INT32(engine, tegra_tsec),
        VMSTATE_UINT32_ARRAY(regs, tegra_tsec, TEGRA_TSEC_SIZE>>2),
        VMSTATE_BOOL(outdata_set, tegra_tsec),
        VMSTATE_UINT32_ARRAY(outdata, tegra_tsec, 4),
        VMSTATE_END_OF_LIST()
    }
};

// TODO: Is this correct? How to handle TSEC_FALCON_ADDR_MSB?
static hwaddr tegra_tsec_get_priv_offset(tegra_tsec *s, hwaddr offset, unsigned size) {
    offset = (offset - 0x1000) << 6;
    offset |= (s->regs[TSEC_FALCON_ADDR>>2] & 0x3F) << 2;
    assert(offset+size <= sizeof(s->regs));

    return offset;
}

static uint64_t tegra_tsec_priv_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    tegra_tsec *s = opaque;
    uint64_t ret = 0;

    assert(offset+size <= sizeof(s->regs));

    if (s->engine == TEGRA_TSEC_ENGINE_VIC && offset >= 0x1400) {
        offset = tegra_tsec_get_priv_offset(s, offset, size);
    }

    ret = s->regs[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    if (offset == TSEC_FALCON_DMATRFCMD_OFFSET) {
        ret |= 1<<1; // BUSY = IDLE
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_tsec_priv_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    tegra_tsec *s = opaque;

    assert(offset+size <= sizeof(s->regs));

    if (s->engine == TEGRA_TSEC_ENGINE_VIC && offset >= 0x1400) {
        offset = tegra_tsec_get_priv_offset(s, offset, size);
    }

    TRACE_WRITE(s->iomem.addr, offset, 0, value);

    if (offset == TSEC_FALCON_CPUCTL_OFFSET) {
        if (value & 0x2) {
            value |= 1<<4; // When STARTCPU is set, enable HALTED.
            if (s->engine == TEGRA_TSEC_ENGINE_TSEC) {
                if (s->outdata_set) {
                    size_t datasize = NV_SOR_TMDS_HDCP_BKSV_LSB_OFFSET+4-NV_SOR_DP_HDCP_BKSV_LSB_OFFSET;
                    dma_addr_t databuf_outsize=datasize;
                    uint32_t *databuf_out = dma_memory_map(&address_space_memory, TEGRA_SOR1_BASE + NV_SOR_DP_HDCP_BKSV_LSB_OFFSET,
                                                       &databuf_outsize, DMA_DIRECTION_FROM_DEVICE, MEMTXATTRS_UNSPECIFIED);

                    if (databuf_out && databuf_outsize==datasize) {
                        databuf_out[0] = s->outdata[0];
                        databuf_out[(NV_SOR_TMDS_HDCP_BKSV_LSB_OFFSET-NV_SOR_DP_HDCP_BKSV_LSB_OFFSET)>>2] = s->outdata[1];
                        databuf_out[(NV_SOR_TMDS_HDCP_CN_MSB_OFFSET-NV_SOR_DP_HDCP_BKSV_LSB_OFFSET)>>2] = s->outdata[2];
                        databuf_out[(NV_SOR_TMDS_HDCP_CN_LSB_OFFSET-NV_SOR_DP_HDCP_BKSV_LSB_OFFSET)>>2] = s->outdata[3];
                    }
                    else {
                        Error *err = NULL;
                        error_setg(&err, "tegra.tsec: Failed to DMA map the SOR output regs.");
                        if (err) error_report_err(err);
                    }

                    if (databuf_out) dma_memory_unmap(&address_space_memory, databuf_out, databuf_outsize, DMA_DIRECTION_TO_DEVICE, datasize);
                }
                s->regs[TSEC_FALCON_MAILBOX1_OFFSET>>2] = 0xB0B0B0B0; // TsecResult_Success
            }
        }
    }

    s->regs[offset/sizeof(uint32_t)] = (s->regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;
}

static void tegra_tsec_priv_reset(DeviceState *dev)
{
    tegra_tsec *s = TEGRA_TSEC(dev);

    memset(s->regs, 0, sizeof(s->regs));

    s->outdata_set = false;
    memset(s->outdata, 0, sizeof(s->outdata));

    // Load the tegra.tsec.outdata secret into outdata.
    if (s->engine == TEGRA_TSEC_ENGINE_TSEC) {
        Error *err = NULL;
        uint8_t *data=NULL;
        size_t datalen = 0;
        if (qcrypto_secret_lookup("tegra.tsec.outdata", &data, &datalen, &err)==0) {
            if (datalen > sizeof(s->outdata)) {
                error_setg(&err, "tegra.tsec: Invalid datalen for secret tegra.tsec.outdata, datalen=0x%lx expected <=0x%lx.", datalen, sizeof(s->outdata));
            }
            else {
                memcpy(s->outdata, data, datalen);
                s->outdata_set = true;
            }
            g_free(data);
        }
        if (err) error_report_err(err);
    }
}

static const MemoryRegionOps tegra_tsec_mem_ops = {
    .read = tegra_tsec_priv_read,
    .write = tegra_tsec_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_tsec_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_tsec *s = TEGRA_TSEC(dev);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_tsec_mem_ops, s,
                          "tegra.tsec", TEGRA_TSEC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static Property tegra_tsec_properties[] = {
    DEFINE_PROP_INT32("engine", tegra_tsec, engine, TEGRA_TSEC_ENGINE_TSEC), \
    DEFINE_PROP_END_OF_LIST(),
};

static void tegra_tsec_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, tegra_tsec_properties);
    dc->realize = tegra_tsec_priv_realize;
    dc->vmsd = &vmstate_tegra_tsec;
    dc->reset = tegra_tsec_priv_reset;
}

static const TypeInfo tegra_tsec_info = {
    .name = TYPE_TEGRA_TSEC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_tsec),
    .class_init = tegra_tsec_class_init,
};

static void tegra_tsec_register_types(void)
{
    type_register_static(&tegra_tsec_info);
}

type_init(tegra_tsec_register_types)
