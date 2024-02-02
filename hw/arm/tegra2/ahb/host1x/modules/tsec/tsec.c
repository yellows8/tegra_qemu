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
#include "host1x_module.h"
#include "exec/address-spaces.h"
#include "sysemu/dma.h"
#include "qemu/log.h"
#include "cpu.h"

#include "iomap.h"
#include "tegra_trace.h"
#include "tegra_cpu.h"
#include "devices.h"

#include "tsec.h"
#include "../apb/se/se.h"
#include "../ppsb/evp/evp.h"
#include "../ppsb/flow/flow.h"
#include "../ppsb/car/car.h"

#define TYPE_TEGRA_TSEC "tegra.tsec"
#define TEGRA_TSEC(obj) OBJECT_CHECK(tegra_tsec, (obj), TYPE_TEGRA_TSEC)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_tsec_state {
    SysBusDevice parent_obj;

    qemu_irq irq;
    MemoryRegion iomem;
    struct host1x_module module;

    int32_t engine;
    uint32_t regs[TEGRA_TSEC_SIZE>>2];
    bool outdata_set;
    bool package1_key_set;
    bool tsec_root_key_set;
    uint32_t outdata[4];
    uint32_t package1_key[4];
    uint32_t tsec_root_key[4];
} tegra_tsec;

// From hactool.
typedef struct {
    uint32_t magic;
    uint32_t wb_size;
    uint32_t wb_ep;
    uint32_t _0xC;
    uint32_t bl_size;
    uint32_t bl_ep;
    uint32_t sm_size;
    uint32_t sm_ep;
    unsigned char data[];
} pk11_t;

static const VMStateDescription vmstate_tegra_tsec = {
    .name = "tegra.tsec",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_INT32(engine, tegra_tsec),
        VMSTATE_UINT32_ARRAY(regs, tegra_tsec, TEGRA_TSEC_SIZE>>2),
        VMSTATE_BOOL(outdata_set, tegra_tsec),
        VMSTATE_BOOL(package1_key_set, tegra_tsec),
        VMSTATE_BOOL(tsec_root_key_set, tegra_tsec),
        VMSTATE_UINT32_ARRAY(outdata, tegra_tsec, 4),
        VMSTATE_UINT32_ARRAY(package1_key, tegra_tsec, 4),
        VMSTATE_UINT32_ARRAY(tsec_root_key, tegra_tsec, 4),
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
                    struct host1x_module *module = get_host1x_module(0x7C);

                    if (module) {
                        host1x_module_write(module, NV_SOR_DP_HDCP_BKSV_LSB_OFFSET>>2, s->outdata[0]);
                        host1x_module_write(module, NV_SOR_TMDS_HDCP_BKSV_LSB_OFFSET>>2, s->outdata[1]);
                        host1x_module_write(module, NV_SOR_TMDS_HDCP_CN_MSB_OFFSET>>2, s->outdata[2]);
                        host1x_module_write(module, NV_SOR_TMDS_HDCP_CN_LSB_OFFSET>>2, s->outdata[3]);
                    }
                    else {
                        Error *err = NULL;
                        error_setg(&err, "tegra.tsec: Failed to get the SOR1 host1x module.");
                        if (err) error_report_err(err);
                    }
                }

                if (s->tsec_root_key_set)
                    tegra_se_set_aes_keyslot(13, s->tsec_root_key, sizeof(s->tsec_root_key));
                if (s->outdata_set) // tsec_key
                    tegra_se_set_aes_keyslot(12, s->outdata, sizeof(s->outdata));

                if (s->package1_key_set) { // Decrypt PK11 if the key is set.
                    struct {
                        uint32_t pk11_size;
                        uint32_t _0x4[0xC>>2];
                        uint8_t iv[0x10];
                    } pk11_info = {};

                    hwaddr inbuf = 0x40010000 + 0x6FE0;
                    dma_memory_read(&address_space_memory, inbuf, &pk11_info, sizeof(pk11_info), MEMTXATTRS_UNSPECIFIED);
                    inbuf+= sizeof(pk11_info);

                    pk11_t header={};

                    if(tegra_se_crypto_operation(s->package1_key, pk11_info.iv, QCRYPTO_CIPHER_ALG_AES_128, QCRYPTO_CIPHER_MODE_CBC, false, inbuf, &header, sizeof(header))==0) {
                        if (tswap32(header.magic) != 0x31314B50) { // PK11
                            Error *err = NULL;
                            error_setg(&err, "tegra.tsec: Decryption failed, invalid PK11 magicnum.");
                            if (err) error_report_err(err);
                        }
                        else {
                            uint32_t pk11_size = tswap32(pk11_info.pk11_size);
                            uint32_t bl_ep = tswap32(header.bl_ep);

                            tegra_se_crypto_operation(s->package1_key, pk11_info.iv, QCRYPTO_CIPHER_ALG_AES_128, QCRYPTO_CIPHER_MODE_CBC, false, inbuf, NULL, pk11_size);

                            tegra_se_lock_aes_keyslot(13, 0x100);

                            if (bl_ep >= pk11_size) {
                                qemu_log_mask(LOG_GUEST_ERROR, "tegra.tsec: Invalid PK11 bl_ep, bl_ep = 0x%x pk11_size = 0x%x.\n", bl_ep, pk11_size);
                            }
                            else {
                                inbuf+= sizeof(header) + bl_ep;

                                // Halt the BPMP, then set the reset vector used when the BPMP is reset below.
                                tegra_flow_event_write(tegra_flow_dev, HALT_COP_EVENTS_OFFSET, 2<<29, TEGRA_BPMP); // FLOW_MODE_WAITEVENT
                                tegra_evp_set_bpmp_reset_vector(inbuf);

                                // Run the same CAR pokes as tsec-fw. Note that tsec-fw would do this before the halt write above.
                                tegra_car_orr_reg(tegra_car_dev, RST_DEVICES_L_OFFSET, 0x9050C202, 4);
                                tegra_car_orr_reg(tegra_car_dev, RST_DEVICES_H_OFFSET, 0x4000006, 4);
                                tegra_car_orr_reg(tegra_car_dev, RST_DEVICES_U_OFFSET, 0x82000460, 4);

                                tegra_car_orr_reg(tegra_car_dev, RST_DEVICES_V_OFFSET, 0x8, 4);
                                tegra_car_orr_reg(tegra_car_dev, RST_CPUG_CMPLX_SET_OFFSET, 0x20000000, 4);
                                tegra_car_orr_reg(tegra_car_dev, RST_CPUG_CMPLX_SET_OFFSET, 0x61FF000F, 4);

                                tegra_car_clear_reg(tegra_car_dev, RST_DEVICES_L_OFFSET, 0x2, 4);
                                tegra_flow_event_write(tegra_flow_dev, HALT_COP_EVENTS_OFFSET, 0, TEGRA_BPMP);
                            }
                        }
                    }
                }

                s->regs[TSEC_FALCON_MAILBOX1_OFFSET>>2] = 0xB0B0B0B0; // TsecResult_Success
            }
        }
    }

    s->regs[offset/sizeof(uint32_t)] = (s->regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;
}

static void tegra_tsec_module_write(struct host1x_module *module,
                                    uint32_t offset, uint32_t data)
{
    tegra_tsec_priv_write(module->opaque, offset<<2, data, 4);
}

static uint32_t tegra_tsec_module_read(struct host1x_module *module,
                                       uint32_t offset)
{
    return tegra_tsec_priv_read(module->opaque, offset<<2, 4);
}

static void tegra_tsec_load_key(const char *name, void* outdata, size_t maxsize, bool *flag)
{
    Error *err = NULL;
    uint8_t *data=NULL;
    size_t datalen = 0;
    *flag = false;
    if (qcrypto_secret_lookup(name, &data, &datalen, &err)==0) {
        if (datalen > maxsize) {
            error_setg(&err, "tegra.tsec: Invalid datalen for secret %s, datalen=0x%lx expected <=0x%lx.", name, datalen, maxsize);
        }
        else {
            memcpy(outdata, data, datalen);
            *flag = true;
        }
        g_free(data);
        data = NULL;
        datalen = 0;
    }
    if (err) error_report_err(err);
    err = NULL;
}

static void tegra_tsec_priv_reset(DeviceState *dev)
{
    tegra_tsec *s = TEGRA_TSEC(dev);

    memset(s->regs, 0, sizeof(s->regs));

    s->outdata_set = false;
    s->package1_key_set = false;
    memset(s->outdata, 0, sizeof(s->outdata));
    memset(s->package1_key, 0, sizeof(s->package1_key));

    // Load the tegra.tsec.outdata and tegra.tsec.package1_key secret into outdata/package1_key.
    if (s->engine == TEGRA_TSEC_ENGINE_TSEC) {
        tegra_tsec_load_key("tegra.tsec.outdata", s->outdata, sizeof(s->outdata), &s->outdata_set);

        tegra_tsec_load_key("tegra.tsec.package1_key", s->package1_key, sizeof(s->package1_key), &s->package1_key_set);
        tegra_tsec_load_key("tegra.tsec.tsec_root_key", s->tsec_root_key, sizeof(s->tsec_root_key), &s->tsec_root_key_set);
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

    s->module.reg_write = tegra_tsec_module_write;
    s->module.reg_read = tegra_tsec_module_read;
    register_host1x_bus_module(&s->module, s);
}

static Property tegra_tsec_properties[] = {
    DEFINE_PROP_UINT8("class_id", tegra_tsec, module.class_id, 0xE0),
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
