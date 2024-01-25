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


#include "modules/host1x/host1x.h"

#include "host1x_channel.h"
#include "host1x_fifo.h"
#include "host1x_hwlock.h"
#include "host1x_module.h"
#include "host1x_syncpts.h"

#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_HOST1X_CHANNEL "tegra.host1x_channel"
#define TEGRA_HOST1X_CHANNEL(obj) OBJECT_CHECK(tegra_host1x_channel, (obj), TYPE_TEGRA_HOST1X_CHANNEL)

#define CHANNEL_BASE    (0x50000000 | (s->cdma.ch_id * SZ_16K))

static const VMStateDescription vmstate_tegra_host1x_channel = {
    .name = "tegra.host1x_channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(sync_base, tegra_host1x_channel),
        VMSTATE_UINT32(sync_size, tegra_host1x_channel),
        VMSTATE_UINT32(syncpt_offset, tegra_host1x_channel),
        VMSTATE_UINT32(syncpt_count, tegra_host1x_channel),
        VMSTATE_UINT32(bases_count, tegra_host1x_channel),
        VMSTATE_UINT32(fifostat.reg32, tegra_host1x_channel),
        VMSTATE_UINT32(indoff.reg32, tegra_host1x_channel),
        VMSTATE_UINT32(indcnt.reg32, tegra_host1x_channel),
        VMSTATE_UINT32(dmastart.reg32, tegra_host1x_channel),
        VMSTATE_UINT32(dmaput.reg32, tegra_host1x_channel),
        VMSTATE_UINT32(dmaend.reg32, tegra_host1x_channel),
        VMSTATE_UINT32(dmactrl.reg32, tegra_host1x_channel),
        VMSTATE_UINT32(indoff2.reg32, tegra_host1x_channel),
        VMSTATE_UINT32(tickcount_hi.reg32, tegra_host1x_channel),
        VMSTATE_UINT32(tickcount_lo.reg32, tegra_host1x_channel),
        VMSTATE_UINT32(channelctrl.reg32, tegra_host1x_channel),
        VMSTATE_UINT32(raise.reg32, tegra_host1x_channel),
        VMSTATE_UINT32(fbbufbase.reg32, tegra_host1x_channel),
        VMSTATE_UINT32(cmdswap.reg32, tegra_host1x_channel),
        VMSTATE_UINT32(indoffset, tegra_host1x_channel),
        VMSTATE_UINT8(class_id, tegra_host1x_channel),
        VMSTATE_END_OF_LIST()
    }
};

static uint32_t host1x_sync_read_reg(tegra_host1x_channel *s, hwaddr base, hwaddr offset)
{
    uint32_t ret = 0;

    switch (offset) {
    case INTSTATUS_OFFSET:
        ret = host1x_get_syncpts_irq_status();
        break;
    case INTMASK_OFFSET:
        ret = host1x_get_modules_irq_mask();
        break;
    case INTC0MASK_OFFSET:
        ret = host1x_get_modules_irq_cpu_mask();
        break;
    case INTC1MASK_OFFSET:
        ret = host1x_get_modules_irq_cop_mask();
        break;
    case HINTSTATUS_OFFSET:
        break;
    case HINTMASK_OFFSET:
        break;
    case HINTSTATUS_EXT_OFFSET:
        break;
    case HINTMASK_EXT_OFFSET:
        break;
    case SYNCPT_THRESH_CPU0_INT_STATUS_OFFSET:
        ret = host1x_get_syncpts_cpu_irq_status();
        break;
    case SYNCPT_THRESH_CPU1_INT_STATUS_OFFSET:
        ret = host1x_get_syncpts_cop_irq_status();
        break;
    case SYNCPT_THRESH_INT_MASK_OFFSET:
        ret = host1x_get_syncpts_dst_mask_low();
        break;
    case SYNCPT_THRESH_INT_MASK_1_OFFSET:
        ret = host1x_get_syncpts_dst_mask_high();
        break;
    case SYNCPT_THRESH_INT_DISABLE_OFFSET:
        break;
    case SYNCPT_THRESH_INT_ENABLE_CPU0_OFFSET:
        break;
    case SYNCPT_THRESH_INT_ENABLE_CPU1_OFFSET:
        break;
    case CF0_SETUP_OFFSET ... CF7_SETUP_OFFSET:
        break;
    case CF_SETUPDONE_OFFSET:
        break;
    case CMDPROC_CTRL_OFFSET:
        break;
    case CMDPROC_STAT_OFFSET:
        break;
    case CMDPROC_STOP_OFFSET:
        break;
    case CH_TEARDOWN_OFFSET:
        break;
    case MOD_TEARDOWN_OFFSET:
        break;
    case CH0_STATUS_OFFSET ... CH7_STATUS_OFFSET:
        break;
    case DISPLAY_STATUS_OFFSET:
        break;
    case DISPLAYB_STATUS_OFFSET:
        break;
    case EPP_STATUS_OFFSET:
        break;
    case GR3D_STATUS_OFFSET:
        break;
    case ISP_STATUS_OFFSET:
        break;
    case MPE_STATUS_OFFSET:
        break;
    case TVO_STATUS_OFFSET:
        break;
    case DSI_STATUS_OFFSET:
        break;
    case HDMI_STATUS_OFFSET:
        break;
    case VI_STATUS_OFFSET:
        break;
    case GR2D_STATUS_OFFSET:
        break;
    case DIRECT_MODULE_CONFIG_OFFSET:
        break;
    case USEC_CLK_OFFSET:
        break;
    case CTXSW_TIMEOUT_CFG_OFFSET:
        break;
    case INDREG_DMA_CTRL_OFFSET:
        break;
    case CHANNEL_PRIORITY_OFFSET:
        break;
    case CDMA_ASM_TIMEOUT_OFFSET:
        break;
    case CDMA_MISC_OFFSET:
        break;
    case IP_BUSY_TIMEOUT_OFFSET:
        break;
    case IP_READ_TIMEOUT_ADDR_OFFSET:
        break;
    case IP_WRITE_TIMEOUT_ADDR_OFFSET:
        break;
    case MCCIF_THCTRL_OFFSET:
        break;
    case HC_MCCIF_FIFOCTRL_OFFSET:
        break;
    case TIMEOUT_WCOAL_HC_OFFSET:
        break;
    case HWLOCK0_OFFSET ... HWLOCK7_OFFSET:
        ret = host1x_acquire_hwlock((offset & 0xf) >> 2);
        break;
    case MLOCK_OFFSET ... MLOCK_15_OFFSET:
        ret = host1x_cpu_acquire_mlock((offset & 0x3f) >> 2);
        break;
    case MLOCK_OWNER_OFFSET ... MLOCK_OWNER_15_OFFSET:
        ret = host1x_cpu_get_mlock_val((offset & 0x3f) >> 2);
        break;
    case MLOCK_ERROR_OFFSET:
        break;
    case SYNCPT_INT_THRESH_OFFSET ... SYNCPT_INT_THRESH_31_OFFSET:
        ret = host1x_get_syncpt_threshold((offset & 0xff) >> 2);
        break;
    case SYNCPT_CPU_INCR_OFFSET:
        break;
    case CBREAD0_OFFSET ... CBREAD7_OFFSET:
        break;
    case REGF_DATA_OFFSET:
        break;
    case REGF_ADDR_OFFSET:
        break;
    case WAITOVR_OFFSET:
        break;
    case CFPEEK_CTRL_OFFSET:
        break;
    case CFPEEK_READ_OFFSET:
        break;
    case CFPEEK_PTRS_OFFSET:
        break;
    case CBSTAT0_OFFSET ... CBSTAT7_OFFSET:
        break;
    case CDMA_STATS_WORDS_FETCHED_OFFSET:
        break;
    case CDMA_STATS_WORDS_DISCARDED_OFFSET:
        break;
    case CFG_OFFSET:
        break;
    case RDMA_MISC_OFFSET:
        break;
    case RDMA_ARB_COUNT_OFFSET:
        break;
    case RDMA_CONFIG_OFFSET:
        break;
    case RDMA_WRAP_OFFSET:
        break;
    case RDMA_STATUS0_OFFSET:
        break;
    case RDMA_BUFFER_THRESHOLD0_OFFSET:
        break;
    case RDMA_CONF0_OFFSET:
        break;
    case RDMA_SWAP0_OFFSET:
        break;
    case RDMA_LINE0_OFFSET:
        break;
    case RDMA_CLID0_OFFSET:
        break;
    case RDMA_BADDR0_OFFSET:
        break;
    case RDMA_DMATRIGGER0_OFFSET:
        break;
    default:
        if (offset >= s->syncpt_offset && offset <= s->syncpt_offset + s->syncpt_count*4 - 4)
            ret = host1x_get_syncpt_count((offset - s->syncpt_offset) >> 2);
        else if (offset >= SYNCPT_BASE_OFFSET && offset <= SYNCPT_BASE_OFFSET + s->bases_count*4 - 4)
            ret = host1x_get_syncpt_base((offset - SYNCPT_BASE_OFFSET) >> 2);
        break;
    }

    TRACE_READ(base, offset, ret);

    return ret;
}

static void host1x_sync_write_reg(tegra_host1x_channel *s, hwaddr base, hwaddr offset, uint32_t value)
{
    unsigned i;

    switch (offset) {
    case INTMASK_OFFSET:
        TRACE_WRITE(base, offset, host1x_get_modules_irq_mask(), value);
        host1x_set_modules_irq_mask(value);
        break;
    case INTC0MASK_OFFSET:
        TRACE_WRITE(base, offset, host1x_get_modules_irq_cpu_mask(), value);
        host1x_set_modules_percpu_irq_mask(HOST1X_CPU, value);
        break;
    case INTC1MASK_OFFSET:
        TRACE_WRITE(base, offset, host1x_get_modules_irq_cop_mask(), value);
        host1x_set_modules_percpu_irq_mask(HOST1X_COP, value);
        break;
    case HINTSTATUS_OFFSET:
        TRACE_WRITE(base, offset, 0, value & HINTSTATUS_WRMASK);
        break;
    case HINTMASK_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case HINTSTATUS_EXT_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case HINTMASK_EXT_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case SYNCPT_THRESH_CPU0_INT_STATUS_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        host1x_clear_syncpts_irq_status(HOST1X_CPU, value);
        break;
    case SYNCPT_THRESH_CPU1_INT_STATUS_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        host1x_clear_syncpts_irq_status(HOST1X_COP, value);
        break;
    case SYNCPT_THRESH_INT_MASK_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        host1x_set_syncpts_irq_dst_mask(0, value);
        break;
    case SYNCPT_THRESH_INT_MASK_1_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        host1x_set_syncpts_irq_dst_mask(1, value);
        break;
    case SYNCPT_THRESH_INT_DISABLE_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        host1x_clear_syncpts_irq_dst_mask(value);
        break;
    case SYNCPT_THRESH_INT_ENABLE_CPU0_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        host1x_enable_syncpts_irq_mask(HOST1X_CPU, value);
        break;
    case SYNCPT_THRESH_INT_ENABLE_CPU1_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        host1x_enable_syncpts_irq_mask(HOST1X_COP, value);
        break;
    case CF0_SETUP_OFFSET ... CF7_SETUP_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case CF_SETUPDONE_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case CMDPROC_CTRL_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case CMDPROC_STOP_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case CH_TEARDOWN_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case MOD_TEARDOWN_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case DIRECT_MODULE_CONFIG_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case USEC_CLK_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case CTXSW_TIMEOUT_CFG_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case INDREG_DMA_CTRL_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case CHANNEL_PRIORITY_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case CDMA_ASM_TIMEOUT_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case CDMA_MISC_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case IP_BUSY_TIMEOUT_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case MCCIF_THCTRL_OFFSET:
        TRACE_WRITE(base, offset, 0, value & MCCIF_THCTRL_WRMASK);
        break;
    case HC_MCCIF_FIFOCTRL_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case TIMEOUT_WCOAL_HC_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case HWLOCK0_OFFSET ... HWLOCK7_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        host1x_release_hwlock((offset & 0xf) >> 2);
        break;
    case MLOCK_OFFSET ... MLOCK_15_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        host1x_cpu_release_mlock((offset & 0x3f) >> 2);
        break;
    case MLOCK_ERROR_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case SYNCPT_INT_THRESH_OFFSET ... SYNCPT_INT_THRESH_31_OFFSET:
    {
        syncpt_int_thresh_t thresh = { .reg32 = value };
        TRACE_WRITE(base, offset, 0, value);
        host1x_set_syncpt_threshold((offset & 0xff) >> 2, thresh.int_thresh);
        break;
    }
    case SYNCPT_CPU_INCR_OFFSET:
        TRACE_WRITE(base, offset, 0, value);

        FOREACH_BIT_SET(value, i, NV_HOST1X_SYNCPT_NB_PTS)
            host1x_incr_syncpt(i);
        break;
    case WAITOVR_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case CFPEEK_CTRL_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case CFG_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case RDMA_MISC_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case RDMA_ARB_COUNT_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case RDMA_CONFIG_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case RDMA_WRAP_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case RDMA_STATUS0_OFFSET:
        TRACE_WRITE(base, offset, 0, value & RDMA_STATUS0_WRMASK);
        break;
    case RDMA_BUFFER_THRESHOLD0_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case RDMA_CONF0_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case RDMA_SWAP0_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case RDMA_LINE0_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case RDMA_CLID0_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case RDMA_BADDR0_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    case RDMA_DMATRIGGER0_OFFSET:
        TRACE_WRITE(base, offset, 0, value);
        break;
    default:
        TRACE_WRITE(base, offset, 0, value);
        if (offset >= s->syncpt_offset && offset <= s->syncpt_offset + s->syncpt_count*4 - 4)
            host1x_set_syncpt_count((offset - s->syncpt_offset) >> 2, value);
        else if (offset >= SYNCPT_BASE_OFFSET && offset <= SYNCPT_BASE_OFFSET + s->bases_count*4 - 4) {
            syncpt_base_t syncpt_base = { .reg32 = value };
            host1x_set_syncpt_base((offset - SYNCPT_BASE_OFFSET) >> 2, syncpt_base.base_0);
        }
        break;
    }
}

static uint32_t ind_swap(uint32_t value, int type)
{
    uint32_t ret;

    switch (type) {
        case 1:
            ret = (bswap16(value >> 16) << 16) | bswap16(value & 0xffff);
            break;
        case 2:
            ret = bswap32(value);
            break;
        case 3:
            ret = ror32(value, 16);
            break;
        default:
            ret = value;
            break;
    }

    return ret;
}

static uint64_t tegra_host1x_channel_priv_read(void *opaque, hwaddr offset,
                                               unsigned size)
{
    tegra_host1x_channel *s = opaque;
    uint64_t ret = 0;

    switch (offset) {
    case FIFOSTAT_OFFSET:
        s->fifostat.outfentries = host1x_get_fifo_entries_nb(s->fifo);
        ret = s->fifostat.reg32;
        break;
    case INDOFF_OFFSET:
        ret = s->indoff.reg32;
        break;
    case INDCNT_OFFSET:
        ret = s->indcnt.reg32;
        break;
    case INDDATA_OFFSET:
        ret = host1x_fifo_pop(s->fifo);
        break;
    case DMASTART_OFFSET:
        ret = s->dmastart.reg32;
        break;
    case DMAPUT_OFFSET:
        ret = s->dmaput.reg32;
        break;
    case DMAGET_OFFSET:
        ret = s->cdma.gather.get << 2;
        break;
    case DMAEND_OFFSET:
        ret = s->dmaend.reg32;
        break;
    case DMACTRL_OFFSET:
        ret = s->dmactrl.reg32;
        break;
    case INDOFF2_OFFSET:
        ret = s->indoff2.reg32;
        break;
    case TICKCOUNT_HI_OFFSET:
        ret = s->tickcount_hi.reg32;
        break;
    case TICKCOUNT_LO_OFFSET:
        ret = s->tickcount_lo.reg32;
        break;
    case CHANNELCTRL_OFFSET:
        ret = s->channelctrl.reg32;
        break;
    case RAISE_OFFSET:
        ret = s->raise.reg32;
        break;
    case FBBUFBASE_OFFSET:
        ret = s->fbbufbase.reg32;
        break;
    case CMDSWAP_OFFSET:
        ret = s->cmdswap.reg32;
        break;
    default:
        if (offset >= s->sync_base && offset+size <= s->sync_base + s->sync_size) return host1x_sync_read_reg(s, CHANNEL_BASE + s->sync_base, offset - s->sync_base);
        break;
    }

    TRACE_READ(CHANNEL_BASE, offset, ret);

    return ret;
}

static void tegra_host1x_channel_priv_write(void *opaque, hwaddr offset,
                                            uint64_t value, unsigned size)
{
    tegra_host1x_channel *s = opaque;

    switch (offset) {
    case INDOFF_OFFSET:
        TRACE_WRITE(CHANNEL_BASE, offset, s->indoff.reg32, value);
        s->indoff.reg32 = value;

        if (s->indoff.indoffupd == UPDATE) {
            if (s->indoff.acctype == REG) {
                s->indoffset = s->indoff.regoffset;
                s->class_id = decode_class_id(s->indoff.modid);
            } else {
                s->indoffset = s->indoff.fboffset;
            }
        }
        break;
    case INDCNT_OFFSET:
        TRACE_WRITE(CHANNEL_BASE, offset, s->indcnt.reg32, value);
        s->indcnt.reg32 = value;

        g_assert_not_reached();
        break;
    case INDDATA_OFFSET:
        TRACE_WRITE(CHANNEL_BASE, offset, s->inddata.reg32, value);

        if (s->indoff.acctype == REG) {
            /* Indirect host1x module reg write */
            struct host1x_module *module = get_host1x_module(s->class_id);
            host1x_cdma_ptr = &s->cdma;

            host1x_module_write(module, s->indoffset, value);
        } else {
            /* Indirect memory write */
            uint32_t *mem = host1x_dma_ptr;

            mem[s->indoffset] = ind_swap(value, s->indoff.indswap);
        }

        if (s->indoff.autoinc) {
            s->indoffset++;
            s->indoffset &= 0x3fffffff;
        }
        break;
    case DMASTART_OFFSET:
        TRACE_WRITE(CHANNEL_BASE, offset, s->dmastart.reg32, value);
        s->dmastart.reg32 = value;

        host1x_cdma_set_base(&s->cdma, s->dmastart.dmastart);
        break;
    case DMAPUT_OFFSET:
        TRACE_WRITE(CHANNEL_BASE, offset, s->dmaput.reg32, value);
        s->dmaput.reg32 = value;

        host1x_cdma_set_put(&s->cdma, s->dmaput.dmaput);
        break;
    case DMAEND_OFFSET:
        TRACE_WRITE(CHANNEL_BASE, offset, s->dmaend.reg32, value);
        s->dmaend.reg32 = value;

        host1x_cdma_set_end(&s->cdma, s->dmaend.dmaend);
        break;
    case DMACTRL_OFFSET:
        TRACE_WRITE(CHANNEL_BASE, offset, s->dmactrl.reg32, value);
        s->dmactrl.reg32 = value;

        host1x_cdma_control(&s->cdma, s->dmactrl.dmastop, s->dmactrl.dmagetrst,
                            s->dmactrl.dmainitget);
        break;
    case INDOFF2_OFFSET:
        TRACE_WRITE(CHANNEL_BASE, offset, s->indoff2.reg32, value);
        s->indoff2.reg32 = value;

        if (s->indoff.acctype == REG) {
            s->indoffset = s->indoff2.regoffset;
            s->class_id = decode_class_id(s->indoff2.modid);
        } else {
            s->indoffset = s->indoff2.fboffset;
        }
        break;
    case TICKCOUNT_HI_OFFSET:
        TRACE_WRITE(CHANNEL_BASE, offset, s->tickcount_hi.reg32, value);
        s->tickcount_hi.reg32 = value;
        break;
    case TICKCOUNT_LO_OFFSET:
        TRACE_WRITE(CHANNEL_BASE, offset, s->tickcount_lo.reg32, value);
        s->tickcount_lo.reg32 = value;
        break;
    case CHANNELCTRL_OFFSET:
        TRACE_WRITE(CHANNEL_BASE, offset, s->channelctrl.reg32, value);
        s->channelctrl.reg32 = value;
        break;
    case RAISE_OFFSET:
        TRACE_WRITE(CHANNEL_BASE, offset, s->raise.reg32, value);
        s->raise.reg32 = value;
        break;
    case FBBUFBASE_OFFSET:
        TRACE_WRITE(CHANNEL_BASE, offset, s->fbbufbase.reg32, value);
        s->fbbufbase.reg32 = value;
        break;
    case CMDSWAP_OFFSET:
        TRACE_WRITE(CHANNEL_BASE, offset, s->cmdswap.reg32, value);
        s->cmdswap.reg32 = value;
        break;
    default:
        TRACE_WRITE(CHANNEL_BASE, offset, 0, value);
        if (offset >= s->sync_base && offset+size <= s->sync_base + s->sync_size) host1x_sync_write_reg(s, CHANNEL_BASE + s->sync_base, offset - s->sync_base, value);
        break;
    }
}

static void tegra_host1x_channel_priv_reset(DeviceState *dev)
{
    tegra_host1x_channel *s = TEGRA_HOST1X_CHANNEL(dev);

    host1x_cdma_control(&s->cdma, 1, 1, 0);

    s->fifostat.reg32 = FIFOSTAT_RESET;
    s->indoff.reg32 = INDOFF_RESET;
    s->indcnt.reg32 = INDCNT_RESET;
    s->dmastart.reg32 = DMASTART_RESET;
    s->dmaput.reg32 = DMAPUT_RESET;
    s->dmaend.reg32 = DMAEND_RESET;
    s->dmactrl.reg32 = DMACTRL_RESET;
    s->indoff2.reg32 = INDOFF2_RESET;
    s->tickcount_hi.reg32 = TICKCOUNT_HI_RESET;
    s->tickcount_lo.reg32 = TICKCOUNT_LO_RESET;
    s->channelctrl.reg32 = CHANNELCTRL_RESET;
    s->raise.reg32 = RAISE_RESET;
    s->fbbufbase.reg32 = FBBUFBASE_RESET;
    s->cmdswap.reg32 = CMDSWAP_RESET;

    if (tegra_board >= TEGRAX1_BOARD) {
        s->sync_base = 0x2100;
        s->sync_size = 0x1800;
        s->syncpt_offset = 0xF80;
        s->syncpt_count = NV_HOST1X_SYNCPT_NB_PTS_X1;
        s->bases_count = NV_HOST1X_SYNCPT_NB_BASES_TEGRAX1;
    }
    else {
        s->sync_base = 0x3000;
        s->sync_size = 0x8E8;
        s->syncpt_offset = SYNCPT_OFFSET;
        s->syncpt_count = NV_HOST1X_SYNCPT_NB_PTS_TEGRA2;
        s->bases_count = NV_HOST1X_SYNCPT_NB_BASES_TEGRA2;
    }

    host1x_cdma_set_base(&s->cdma, s->dmastart.dmastart);
    host1x_cdma_set_put(&s->cdma, s->dmaput.dmaput);
    host1x_cdma_set_end(&s->cdma, s->dmaend.dmaend);

    host1x_fifo_reset(s->fifo);
}

static const MemoryRegionOps tegra_host1x_channel_mem_ops = {
    .read = tegra_host1x_channel_priv_read,
    .write = tegra_host1x_channel_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_host1x_channel_realize(DeviceState *dev, Error **errp)
{
    tegra_host1x_channel *s = TEGRA_HOST1X_CHANNEL(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &tegra_host1x_channel_mem_ops,
                          s, "tegra.host1x_channel", SZ_16K);
    sysbus_init_mmio(sbd, &s->iomem);

    s->fifo = host1x_fifo_create(32);
}

static Property tegra_host1x_channel_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void tegra_host1x_channel_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, tegra_host1x_channel_properties);
    dc->vmsd = &vmstate_tegra_host1x_channel;
    dc->realize = tegra_host1x_channel_realize;
    dc->reset = tegra_host1x_channel_priv_reset;
}

static const TypeInfo tegra_host1x_channel_info = {
    .name = TYPE_TEGRA_HOST1X_CHANNEL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_host1x_channel),
    .class_init = tegra_host1x_channel_class_init,
};

static void tegra_host1x_channel_register_types(void)
{
    type_register_static(&tegra_host1x_channel_info);
}

type_init(tegra_host1x_channel_register_types)
