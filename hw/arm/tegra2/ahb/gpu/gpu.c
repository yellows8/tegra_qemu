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

// Various reg names etc are from linux nvgpu.
// Various method related names are from deko3d.

#include "tegra_common.h"

#include "hw/sysbus.h"
#include "sysemu/dma.h"
#include "qemu/log.h"
#include "qemu/cutils.h"

#include "host1x_syncpts.h"

#include "../host1x/modules/dc/registers/dc.h"
#include "devices.h"

#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_GPU "tegra.gpu"
#define TEGRA_GPU(obj) OBJECT_CHECK(tegra_gpu, (obj), TYPE_TEGRA_GPU)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

#define NV_GMMU_VA_RANGE          38

//#define TEGRA_GPU_DEBUG

typedef struct tegra_gpu_state {
    SysBusDevice parent_obj;

    qemu_irq irq_stall;
    qemu_irq irq_nonstall;
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

typedef union tegra_gpu_cmdhdr {
    struct {
        unsigned int method_address:12;
        unsigned int reserved:1;
        unsigned int method_subchannel:3;
        unsigned int arg:13;
        unsigned int sec_op:3;
    };

    uint32_t data;
} tegra_gpu_cmdhdr;

typedef union tegra_gpu_method_setreportsemaphore {
    struct {
        unsigned int operation:2;
        unsigned int flush_disable:1;
        unsigned int reduction_enable:1;
        unsigned int fence_enable:1;
        unsigned int unk:4;
        unsigned int reduction_op:3;
        unsigned int unit:4;
        unsigned int sync_condition:1;
        unsigned int format:2;
        unsigned int unk2:1;
        unsigned int awaken_enable:1;
        unsigned int unk3:2;
        unsigned int counter:5;
        unsigned int structure_size:1;
        unsigned int unk4:3;
    };

    uint32_t data;
} tegra_gpu_method_setreportsemaphore;

#ifdef TEGRA_GPU_DEBUG
static void tegra_gpu_log_hexdump(const char *prefix,
                                  const void *bufptr, size_t size)
{
    FILE *f = qemu_log_trylock();
    if (f) {
        qemu_hexdump(f, prefix, bufptr, size);
        qemu_log_unlock(f);
    }
}
#endif

static MemTxResult tegra_gpu_translate_gmmu(tegra_gpu *s, dma_addr_t addr,
                                            dma_addr_t *out_addr,
                                            IOMMUAccessFlags flag, dma_addr_t pdb, bool big_page_size)
{
    MemTxResult res = MEMTX_OK;

    *out_addr = 0;

    pdb &= (1UL<<34)-1; // TODO: handle properly?

    size_t pde_lowbit = big_page_size ? 27 : 26;
    size_t pte_lowbit = 12;

    if (addr >= SZ_1G*16) // big pages range
        pte_lowbit = big_page_size ? 17 : 16;

    uint64_t pde_i = (addr & ((1UL<<NV_GMMU_VA_RANGE)-1)) >> pde_lowbit;
    uint64_t pte_i = (addr & ((1UL<<pde_lowbit)-1)) >> pte_lowbit;

    uint32_t pde[2]={};
    uint32_t pte[2]={};

    dma_addr_t pte_base=0;

    res = dma_memory_read(&address_space_memory, pdb + pde_i*8,
                          pde, sizeof(pde),
                          MEMTXATTRS_UNSPECIFIED);

    if (res == MEMTX_OK) {
        if ((pde[0] & 0x1) && (pde[1] & 0x8)) { // gmmu_pde_aperture_big_video_memory_f, gmmu_pde_vol_big_true_f
            pte_base = ((dma_addr_t)pde[0]>>4) << 12;
        }
        else if ((pde[1] & 0x1) && (pde[1] & 0x4)) { // gmmu_pde_aperture_small_video_memory_f, gmmu_pde_vol_small_true_f
            pte_base = ((dma_addr_t)pde[1]>>4) << 12;
        }
        else
            res = MEMTX_DECODE_ERROR;
    }

    if (res == MEMTX_OK) {
        pte_base &= (1UL<<34)-1;

        res = dma_memory_read(&address_space_memory, pte_base + pte_i*8,
                              pte, sizeof(pte),
                              MEMTXATTRS_UNSPECIFIED);
    }

    if (res == MEMTX_OK) {
        if (pte[0] & 0x1) { // gmmu_pte_valid_true_f
            if (((pte[0] & 0x4) && (flag & IOMMU_WO)) ||      // gmmu_pte_read_only_true_f
                ((pte[1] & 0x40000000) && (flag & IOMMU_RO))) // gmmu_pte_read_disable_true_f
                res = MEMTX_ACCESS_ERROR;
        }
        else
            res = MEMTX_DECODE_ERROR;
    }

    if (res == MEMTX_OK) {
        *out_addr = ((dma_addr_t)pte[0]>>4) << 12;
        *out_addr &= (1UL<<34)-1;
        *out_addr |= addr & ((1UL<<pte_lowbit)-1);
    }

    return res;
}

#if 0
static void tegra_gpu_dump_gmmu_pages(tegra_gpu *s, dma_addr_t pdb, bool big_page_size, dma_addr_t gva, dma_addr_t size)
{
    MemTxResult res = MEMTX_OK;
    dma_addr_t ptr = 0;
    dma_addr_t databuf_size = 0x1000;
    dma_addr_t tmplen_in=databuf_size;

    for (dma_addr_t i=0; i<size; i+=databuf_size) {
        res = tegra_gpu_translate_gmmu(s, gva + i,
                                       &ptr,
                                       IOMMU_RO, pdb, big_page_size);
        if (res != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR, "tegra_gpu_dump_gmmu_pages: tegra_gpu_translate_gmmu(0x%lX) failed: 0x%x.\n", gva + i, res);
            continue;
        }

        void* databuf = dma_memory_map(&address_space_memory, ptr, &tmplen_in, DMA_DIRECTION_TO_DEVICE, MEMTXATTRS_UNSPECIFIED);

        if (databuf) {
            qemu_log_mask(LOG_GUEST_ERROR, "tegra_gpu_dump_gmmu_pages: Dumping gva 0x%lX:\n", gva + i);
            tegra_gpu_log_hexdump("tegra_gpu_dump_gmmu_pages", databuf, databuf_size);

            dma_memory_unmap(&address_space_memory, databuf, databuf_size, DMA_DIRECTION_TO_DEVICE, databuf_size);
        }
        else {
            qemu_log_mask(LOG_GUEST_ERROR, "tegra_gpu_dump_gmmu_pages: dma_memory_map(0x%lX) failed.\n", ptr);
        }
    }
}
#endif

static MemTxResult tegra_gpu_rw_gmmu(tegra_gpu *s,
                                     dma_addr_t *gva, dma_addr_t *gva_tmp, dma_addr_t *ptr,
                                     dma_addr_t pdb, bool big_page_size,
                                     void* data, size_t size,
                                     DMADirection dir)
{
    MemTxResult res = MEMTX_OK;
    if (*ptr==0 || (((*gva) & ~0xFFF) != ((*gva_tmp) & ~0xFFF))) {
        *gva = (*gva_tmp) & ~0xFFF;
        res = tegra_gpu_translate_gmmu(s, *gva,
                                       ptr,
                                       IOMMU_RO, pdb, big_page_size);
        if (res != MEMTX_OK) return res;
    }

    res = dma_memory_rw(&address_space_memory, (*ptr) | ((*gva_tmp) & 0xFFF),
                          data, size,
                          dir,
                          MEMTXATTRS_UNSPECIFIED);
    return res;
}

static dma_addr_t tegra_gpu_pb_cmd_get_iova(uint32_t *args)
{
    return (((dma_addr_t)args[0])<<32) | args[1];
}

static MemTxResult tegra_gpu_parse_pb_cmd(tegra_gpu *s,
                                               tegra_gpu_cmdhdr cmdhdr, uint32_t *args, size_t argscount,
                                               dma_addr_t pdb, bool big_page_size)
{
    MemTxResult res = MEMTX_OK;
    uint32_t addr = cmdhdr.method_address;
    dma_addr_t gva = 0, iova = 0, ptr = 0;

    if (addr == 0x6C0) { // SetReportSemaphore*
        if (argscount != 4) {
            qemu_log_mask(LOG_GUEST_ERROR, "tegra_gpu_parse_pb_cmd: The argscount for SetReportSemaphore is 0x%lx, expected 0x%x.\n", argscount, 4);
        }
        else {
            tegra_gpu_method_setreportsemaphore *sema = (void*)&args[3];

            if (sema->operation == 0) { // Release
                if (sema->reduction_enable) {
                    qemu_log_mask(LOG_GUEST_ERROR, "tegra_gpu_parse_pb_cmd: SetReportSemaphore ReductionEnable is not supported, ignoring.\n");
                }
                else {
                    iova = tegra_gpu_pb_cmd_get_iova(&args[0]);

                    // Write SetReportSemaphorePayload to the iova.
                    res = tegra_gpu_rw_gmmu(s,
                                            &gva, &iova, &ptr,
                                            pdb, big_page_size,
                                            &args[2], sizeof(args[2]),
                                            DMA_DIRECTION_FROM_DEVICE);
                }
            }
            else {
                qemu_log_mask(LOG_GUEST_ERROR, "tegra_gpu_parse_pb_cmd: SetReportSemaphore Operation 0x%x is not supported, ignoring.\n", sema->operation);
            }
        }
    }

    return res;
}

static MemTxResult tegra_gpu_parse_gpfifo_cmds(tegra_gpu *s,
                                               dma_addr_t gpu_iova, size_t size, uint32_t desc_high,
                                               dma_addr_t pdb, bool big_page_size)
{
    MemTxResult res = MEMTX_OK;
    dma_addr_t gpfifo_gva = 0, gpfifo_gva_tmp = 0, gpfifo_ptr = 0;
    tegra_gpu_cmdhdr cmdhdr={};
    static uint32_t gpfifo_args[0x2000>>2]={};
    size_t argscount=0;

    #ifdef TEGRA_GPU_DEBUG
    qemu_log_mask(LOG_GUEST_ERROR, "tegra_gpu_parse_gpfifo_cmds: gpu_iova = 0x%lx, size = 0x%lx, desc_high = 0x%x\n", gpu_iova, size, desc_high);
    #endif

    for (size_t i=0; i<size; i++) {
        gpfifo_gva_tmp = gpu_iova + i*4;

        res = tegra_gpu_rw_gmmu(s,
                                &gpfifo_gva, &gpfifo_gva_tmp, &gpfifo_ptr,
                                pdb, big_page_size,
                                &cmdhdr, sizeof(cmdhdr),
                                DMA_DIRECTION_TO_DEVICE);
        if (res != MEMTX_OK) break;

        argscount = 1;
        if (cmdhdr.sec_op == 4 || cmdhdr.sec_op == 7 || cmdhdr.data == 0x0) // IMMD_DATA_METHOD, END_PB_SEGMENT
            argscount = 0;
        else if (cmdhdr.sec_op == 1 || cmdhdr.sec_op == 3 || cmdhdr.sec_op == 5) // INC_METHOD, NON_INC_METHOD, ONE_INC
            argscount = cmdhdr.arg;

        #ifdef TEGRA_GPU_DEBUG
        qemu_log_mask(LOG_GUEST_ERROR, "tegra_gpu_parse_gpfifo_cmds hdr: data = 0x%x, method_address = 0x%x, reserved = 0x%x, method_subchannel = 0x%x, arg = 0x%x, sec_op = 0x%x\n", cmdhdr.data, cmdhdr.method_address, cmdhdr.reserved, cmdhdr.method_subchannel, cmdhdr.arg, cmdhdr.sec_op);
        #endif

        for (size_t i2=0; i2<argscount; i++, i2++) {
            if (i+1 >= size) {
                qemu_log_mask(LOG_GUEST_ERROR, "tegra_gpu_parse_gpfifo_cmds: Aborting since argscount 0x%lx (i2=0x%lx) is larger then the remaining size.\n", argscount, i2);
                return res;
            }
            gpfifo_gva_tmp = gpu_iova + (i+1)*4;
            res = tegra_gpu_rw_gmmu(s,
                                    &gpfifo_gva, &gpfifo_gva_tmp, &gpfifo_ptr,
                                    pdb, big_page_size,
                                    &gpfifo_args[i2], sizeof(gpfifo_args[i2]),
                                    DMA_DIRECTION_TO_DEVICE);
            if (res != MEMTX_OK) break;
        }
        if (res != MEMTX_OK) break;

        #ifdef TEGRA_GPU_DEBUG
        if (argscount) tegra_gpu_log_hexdump("tegra_gpu_parse_gpfifo_cmds args", gpfifo_args, argscount*sizeof(uint32_t));
        #endif

        if (cmdhdr.sec_op == 7) // END_PB_SEGMENT
            break;

        res = tegra_gpu_parse_pb_cmd(s,
                                     cmdhdr, gpfifo_args, argscount,
                                     pdb, big_page_size);
        if (res != MEMTX_OK) break;
    }

    return res;
}

static MemTxResult tegra_gpu_parse_gpfifo(tegra_gpu *s, uint32_t original_value, uint32_t value,
                                          dma_addr_t gpfifo_base, dma_addr_t gpfifo_entries, dma_addr_t pdb, bool big_page_size)
{
    MemTxResult res = MEMTX_OK;
    dma_addr_t gpfifo_gva = 0, gpfifo_gva_tmp = 0, gpfifo_ptr = 0;
    dma_addr_t data_gva = 0;
    uint32_t gpfifo[2]={}; // struct gpfifo_entry
    size_t size = 0;

    for (uint64_t i=original_value % gpfifo_entries; i!=value; i = (i+1) % gpfifo_entries) {
        gpfifo_gva_tmp = gpfifo_base + i*8;

        res = tegra_gpu_rw_gmmu(s,
                                &gpfifo_gva, &gpfifo_gva_tmp, &gpfifo_ptr,
                                pdb, big_page_size,
                                gpfifo, sizeof(gpfifo),
                                DMA_DIRECTION_TO_DEVICE);
        if (res != MEMTX_OK) break;

        data_gva = gpfifo[0] | (((dma_addr_t)gpfifo[1] & 0xFF) << 32); // gpu_iova
        size = (gpfifo[1] & ~BIT(31)) >> 10;

        res = tegra_gpu_parse_gpfifo_cmds(s, data_gva, size, gpfifo[1], pdb, big_page_size);
        if (res != MEMTX_OK) break;
    }

    return res;
}

static void tegra_gpu_process_gpfifo(tegra_gpu *s, uint32_t channel_id, uint32_t original_value, uint32_t value)
{
    uint32_t ccsr = s->regs[(0x00800000 + channel_id*8)>>2]; // ccsr_channel_inst

    if (ccsr & 0x80000000) { // ccsr_channel_inst_bind_true_f
        dma_addr_t inst_ptr = (((dma_addr_t)ccsr) & 0xFFFFFFF) << 12;
        inst_ptr &= (1UL<<34)-1;

        uint32_t inst[134]={};
        uint32_t *gp_base = &inst[18]; // ram_fc_gp_base/ram_fc_gp_base_hi_w
        uint32_t *page_dir_base = &inst[128]; // ram_in_page_dir_base_lo_w/ram_in_page_dir_base_hi_w

        if (dma_memory_read(&address_space_memory, inst_ptr,
                            inst, sizeof(inst),
                            MEMTXATTRS_UNSPECIFIED) == MEMTX_OK) {
            dma_addr_t gpfifo_base = gp_base[0] & (0x1FFFFFFF << 3);
            gpfifo_base |= ((dma_addr_t)gp_base[1] & 0xFF) << 32;

            dma_addr_t gpfifo_entries = (gp_base[1] >> 16) & 0x1F; // pbdma_gp_base_hi_limit2_f
            gpfifo_entries = 1<<gpfifo_entries;

            dma_addr_t pdb = page_dir_base[0] & (0xFFFFF << 12);
            pdb |= ((dma_addr_t)page_dir_base[1] & 0xFF) << 32;

            bool big_page_size = (page_dir_base[0] & 0x800) == 0; // big_page_size. Bit clear for 128kb, set for 64kb.

            #ifdef TEGRA_GPU_DEBUG
            qemu_log_mask(LOG_GUEST_ERROR, "tegra_gpu_process_gpfifo: gpfifo_base = 0x%lx, gpfifo_entries = 0x%lx, pdb = 0x%lx, big_page_size = %d\n", gpfifo_base, gpfifo_entries, pdb, big_page_size);
            #endif

            MemTxResult res = tegra_gpu_parse_gpfifo(s, original_value, value, gpfifo_base, gpfifo_entries, pdb, big_page_size);
            if (res != MEMTX_OK) qemu_log_mask(LOG_GUEST_ERROR, "tegra_gpu_process_gpfifo: tegra_gpu_parse_gpfifo(): 0x%x\n", res);
        }
    }

    // Not needed with the workaround in host1x_channel.
    // HACK: The GPU updates syncpts via GPFIFO. Avoid parsing GPFIFO etc and just update all syncpts where threshold is set.
    #if 0
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
    #endif
}

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

        if (offset == 0x2638) {
            s->regs[offset>>2] = value;

            s->regs[0x00002a00>>2] |= value;
            if (s->regs[0x00002a00>>2])
                TRACE_IRQ_RAISE(s->iomem.addr, s->irq_nonstall);
            s->regs[0x20290>>2] = 0;
        }
        else if (offset == 0x00002a00) { // fifo_intr_runlist_r
            s->regs[offset>>2] &= ~value;
            if (s->regs[offset>>2] == 0)
                TRACE_IRQ_LOWER(s->iomem.addr, s->irq_nonstall);
            return;
        }

        uint32_t original_value = s->regs[offset/sizeof(uint32_t)];

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
        else if ((s->regs[0x2254>>2] & 0x10000000)) { // fifo_bar1_base fifo_bar1_base_valid_true_f
            uint32_t base = (s->regs[0x2254>>2] & 0xFFFFFFF)<<12;
            base+= 0x1000000;
            size_t num_channels = 0x80;
            size_t entrysize = 1<<9;
            if (offset >= base && offset < base+num_channels*entrysize) {
                if (((offset - base) & (entrysize-1)) == 35*4) { // ram_userd_gp_put_w (GPFIFO)
                    uint32_t channel_id = (offset - base) / entrysize;

                    tegra_gpu_process_gpfifo(s, channel_id, original_value, value);

	            s->regs[(base + channel_id*entrysize + 34*4)>>2] = value; // ram_userd_gp_get_w
                }
            }
        }
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

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_stall);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_nonstall);

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
