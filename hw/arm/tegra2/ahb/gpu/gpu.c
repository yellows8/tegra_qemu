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

// NOTE: Not intended to render anything.

// Various reg names are from linux nvgpu.

#include "tegra_common.h"

#include "hw/sysbus.h"
#include "qemu/log.h"

#include "host1x_syncpts.h"

#include "../host1x/modules/dc/registers/dc.h"
#include "devices.h"

#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_GPU "tegra.gpu"
#define TEGRA_GPU(obj) OBJECT_CHECK(tegra_gpu, (obj), TYPE_TEGRA_GPU)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

typedef struct tegra_gpu_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[0x2000000>>2];
} tegra_gpu;

static const VMStateDescription vmstate_tegra_gpu = {
    .name = TYPE_TEGRA_GPU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, tegra_gpu, 0x2000000>>2),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t tegra_gpu_priv_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    tegra_gpu *s = opaque;
    uint64_t ret = 0;

    if (offset+size <= sizeof(s->regs))
        ret = s->regs[offset/sizeof(uint32_t)] & ((1ULL<<size*8)-1);

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_gpu_priv_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    tegra_gpu *s = opaque;

    TRACE_WRITE(s->iomem.addr, offset, 0, value);

    // NOTE: Mailbox-ret values for sizes below are dummy values, not the proper values from hardware.

    if (offset+size <= sizeof(s->regs)) {
        if (offset < 4) return; // Ignore id reg.
        s->regs[offset/sizeof(uint32_t)] = (s->regs[offset/sizeof(uint32_t)] & ~((1ULL<<size*8)-1)) | value;

        if ((offset == 0x00070000 || offset == 0x00070004 || offset == 0x00070010) && (value & BIT(0))) // flush_fb_flush/flush_l2_system_invalidate/flush_l2_flush_dirty_r pending
            s->regs[offset>>2] &= ~(BIT(0) | BIT(1)); // Clear pending (from busy) and outstanding.
        else if (offset == 0x100C80 && (value & BIT(12)))
            s->regs[offset>>2] |= 0x1<<16;
        else if (offset == 0x100C80+0x3C && (value & BIT(31)))
            s->regs[0x100C80>>2] |= BIT(15);
        else if (offset == 0x10A004 && (value & BIT(4)))
            s->regs[0x10A008>>2] &= ~BIT(4);
        else if (offset == 0x10A100 && (value & BIT(1))) {
            s->regs[offset>>2] |= BIT(4);
            s->regs[0x10A040>>2] = 0;
        }
        else if (offset == 0x137014 && (value & BIT(30)))
            s->regs[offset>>2] |= BIT(31);
        else if (offset == 0x13701C && (value & BIT(31)))
            s->regs[0x1328A0>>2] |= BIT(24);
        else if (offset == 0x409130 && (value & BIT(1)))
            s->regs[0x409800>>2] = 0x1; // gr_fecs_ctxsw_mailbox(0) = eUcodeHandshakeInitComplete
        else if (offset == 0x409400 && (value & BIT(12)))
            s->regs[offset>>2] &= ~BIT(12);
        else if (offset == 0x409504) { // gr_fecs_method_push
            if (value == 0x00000004) { // gr_fecs_method_push_adr_halt_pipeline_v
                s->regs[(0x409800+(1*4))>>2] = 0x1; // gr_fecs_ctxsw_mailbox(1) = gr_fecs_ctxsw_mailbox_value_pass_v
            }
            else if (value == 0x00000003) { // gr_fecs_method_push_adr_bind_pointer_v
                s->regs[(0x409800+(0*4))>>2] = 0x10; // gr_fecs_ctxsw_mailbox(0)
            }
            else if (value == 0x00000009) { // gr_fecs_method_push_adr_wfi_golden_save_v
                s->regs[(0x409800+(0*4))>>2] = 0x1; // gr_fecs_ctxsw_mailbox(0)
            }
            else if (value == 0x00000015) { // gr_fecs_method_push_adr_restore_golden_v
                s->regs[(0x409800+(0*4))>>2] = 0x1; // gr_fecs_ctxsw_mailbox(0) = gr_fecs_ctxsw_mailbox_value_pass_v
            }
            else if (value == 0x00000010 || value == 0x00000016 || value == 0x00000025) { // gr_fecs_method_push_adr_discover_image_size_v, gr_fecs_method_push_adr_discover_zcull_image_size_v, gr_fecs_method_push_adr_discover_pm_image_size_v
                s->regs[(0x409800+(0*4))>>2] = 0x1; // gr_fecs_ctxsw_mailbox(0)
                //if (value == 0x00000010) s->regs[(0x409800+(0*4))>>2] = 0x4;
                // NOTE: These use the mailbox ret as size values.
            }
            else if (value == 0x00000030) { // gr_fecs_method_push_adr_discover_reglist_image_size_v
                s->regs[(0x409800+(0*4))>>2] = 0x1; // gr_fecs_ctxsw_mailbox(0)
                // NOTE: Mailbox ret is used as a size value.
            }
            else if (value == 0x00000031 || value == 0x00000032) { // gr_fecs_method_push_adr_set_reglist_bind_instance_v, gr_fecs_method_push_adr_set_reglist_virtual_address_v
                s->regs[(0x409800+(4*4))>>2] = 0x1; // gr_fecs_ctxsw_mailbox(4)
            }
            else if (value == 0x00000038 || value == 0x00000038) { // gr_fecs_method_push_adr_stop_ctxsw_v, gr_fecs_method_push_adr_start_ctxsw_v
                s->regs[(0x409800+(1*4))>>2] = 0x1; // gr_fecs_ctxsw_mailbox(1) = gr_fecs_ctxsw_mailbox_value_pass_v
            }
        }
        else if ((offset == 0x409A10 || offset == 0x409B00) && (value & 0x1F))
            s->regs[offset>>2] &= ~0x1F;
        #if 0 // Not needed with the workaround in host1x_channel.
        else if ((s->regs[0x2254>>2] & 0x10000000)) { // fifo_bar1_base fifo_bar1_base_valid_true_f
            uint32_t base = (s->regs[0x2254>>2] & 0xFFFFFFF)<<12;
            base+= 0x1000000;
            size_t num_channels = 0x80;
            size_t entrysize = 1<<9;
            if (offset >= base && offset < base+num_channels*entrysize) {
                if (((offset - base) & (entrysize-1)) == 35*4) { // ram_userd_gp_put_w (GPFIFO)
                    // HACK: The GPU updates syncpts via GPFIFO. Avoid parsing GPFIFO etc and just update all syncpts where threshold is set.
                    for (uint32_t syncpt=0; syncpt<NV_HOST1X_SYNCPT_NB_PTS; syncpt++) {
                        //uint32_t tmp=0, tmp2;
                        uint32_t threshold = host1x_get_syncpt_threshold(syncpt);
                        if (threshold!=0 /*&&
                            (tegra_dca_dev && tegra_dc_get_vblank_syncpt(tegra_dca_dev, &tmp) && syncpt!=tmp) &&
                            (tegra_dcb_dev && tegra_dc_get_vblank_syncpt(tegra_dcb_dev, &tmp2) && syncpt!=tmp2)*/)
                        {
                            host1x_set_syncpt_count(syncpt, threshold);
                            qemu_log_mask(LOG_GUEST_ERROR, "tegra.gpu: Set syncpt_count for syncpt 0x%x to 0x%x, for GPFIO handling.\n", syncpt, threshold);
                        }
                    }
                }
            }
        }
        #endif
    }
}

static void tegra_gpu_priv_reset(DeviceState *dev)
{
    tegra_gpu *s = TEGRA_GPU(dev);

    memset(s->regs, 0, sizeof(s->regs));

    s->regs[0x0] = tegra_board >= TEGRAX1PLUS_BOARD ? 0x12E000A1 : 0x12B000A1;

    s->regs[0x1010000>>2] = 0x33;

    // TODO: Use actual values from hardware?
    s->regs[0x120074>>2] = 0x1;
    s->regs[0x120078>>2] = 0x1;
    s->regs[0x22430>>2] = 0x1;
    s->regs[0x22434>>2] = 0x1;
    s->regs[0x22438>>2] = 0x1;
}

static const MemoryRegionOps tegra_gpu_mem_ops = {
    .read = tegra_gpu_priv_read,
    .write = tegra_gpu_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_gpu_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_gpu *s = TEGRA_GPU(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_gpu_mem_ops, s,
                          TYPE_TEGRA_GPU, sizeof(s->regs));
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tegra_gpu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_gpu_priv_realize;
    dc->vmsd = &vmstate_tegra_gpu;
    dc->reset = tegra_gpu_priv_reset;
}

static const TypeInfo tegra_gpu_info = {
    .name = TYPE_TEGRA_GPU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_gpu),
    .class_init = tegra_gpu_class_init,
};

static void tegra_gpu_register_types(void)
{
    type_register_static(&tegra_gpu_info);
}

type_init(tegra_gpu_register_types)
