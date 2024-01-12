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


#include "host1x_cdma.h"
#include "host1x_cmd_processor.h"
#include "host1x_module.h"

#include "tegra_trace.h"

/* TODO: get rid of globals */
void *host1x_dma_ptr;
__thread struct host1x_cdma *host1x_cdma_ptr;

static void *host1x_cdma_thr(void *opaque)
{
    struct host1x_cdma *cdma = opaque;
    struct host1x_dma_gather *gather = &cdma->gather;

    host1x_cdma_ptr = cdma;

    for (;;) {
        qemu_event_wait(&cdma->start_ev);
        qemu_event_reset(&cdma->start_ev);

        gather->base = cdma->base;

        TPRINT("cdma%d start base=0x%08X get=0x%08X put=0x%08X end=0x%08X\n",
               cdma->ch_id, gather->base << 2, gather->get << 2,
               gather->put << 2, cdma->end << 2);

        TRACE_CDMA_START(cdma->ch_id);

        process_cmd_buf(gather);

        TRACE_CDMA_STOP(cdma->ch_id);

        TPRINT("cdma%d stop\n", cdma->ch_id);
        qemu_event_set(&cdma->stop_ev);
    }

    return NULL;
}

static void host1x_cdma_run(struct host1x_cdma *cdma)
{
    struct host1x_dma_gather *gather = &cdma->gather;

    if (!cdma->enabled)
        return;

    if (gather->get == gather->put)
        return;

    qemu_event_reset(&cdma->stop_ev);
    qemu_event_set(&cdma->start_ev);
}

void host1x_cdma_set_base(struct host1x_cdma *cdma, uint32_t base)
{
    cdma->base = base;
    host1x_cdma_run(cdma);
}

void host1x_cdma_set_put(struct host1x_cdma *cdma, uint32_t put)
{
    cdma->gather.put = put;
    host1x_cdma_run(cdma);
}

void host1x_cdma_set_end(struct host1x_cdma *cdma, uint32_t end)
{
    cdma->end = end;
}

void host1x_cdma_control(struct host1x_cdma *cdma,
                         bool dma_stop, bool dma_get_rst, bool dma_init_get)
{
    if (dma_stop) {
        cdma->enabled = 0;
        host1x_wake_mlocked_channels();
        host1x_unlock_syncpt_waiter_forced(&cdma->waiter);
    }

    qemu_event_wait(&cdma->stop_ev);

    if (dma_get_rst)
        cdma->gather.get = dma_init_get ? cdma->gather.put : 0;

    if (!dma_stop) {
        cdma->enabled = 1;
        host1x_cdma_run(cdma);
    }
}

void host1x_init_cdma(struct host1x_cdma *cdma, uint8_t ch_id)
{
    qemu_event_init(&cdma->start_ev, 0);
    qemu_event_init(&cdma->stop_ev, 1);
    qemu_thread_create(&cdma->thread, "cdma", host1x_cdma_thr, cdma,
                       QEMU_THREAD_DETACHED);
    cdma->gather.cdma = cdma;
    cdma->gather.inlined = 0;
    cdma->module = NULL;
    cdma->ch_id = ch_id;
    cdma->enabled = 0;

    host1x_init_syncpt_waiter(&cdma->waiter);
}

void host1x_init_dma(void)
{
    dma_addr_t dma_sz = 0x3fffffff;

    host1x_dma_ptr = dma_memory_map(&address_space_memory, 0, &dma_sz,
                                    DMA_DIRECTION_TO_DEVICE, MEMTXATTRS_UNSPECIFIED);

    g_assert(host1x_dma_ptr != NULL);
}
