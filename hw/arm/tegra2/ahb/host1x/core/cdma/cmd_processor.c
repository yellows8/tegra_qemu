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

#include "qemu/main-loop.h"
#include "sysemu/dma.h"

#include "host1x_cdma.h"
#include "host1x_cmd_processor.h"
#include "host1x_module.h"
#include "host1x_priv.h"

#include "tegra_trace.h"

static int dma_range_is_valid(struct host1x_dma_gather *gather)
{
    struct host1x_cdma *cdma = gather->cdma;

    if (unlikely((gather->get) == cdma->end)) {
        TPRINT("%s: error cdma=%d reached end=0x%X gather_inlined=%d "
                                                        "base=0x%X get=0x%X\n",
               __func__, cdma->ch_id, cdma->end << 2,
               gather->inlined, gather->base << 2, gather->get << 2);

        g_assert_not_reached();

        cdma->enabled = 0;
    }

    return cdma->enabled;
}

static int dma_get_is_valid(struct host1x_dma_gather *gather)
{
    int stop_gather = (gather->get == gather->put);

    if (stop_gather)
        TPRINT("%s: gather done cdma=%d enabled=%d " \
               "gather_inlined=%d get=0x%X put=0x%X\n",
               __func__, gather->cdma->ch_id, gather->cdma->enabled,
               gather->inlined, gather->get << 2, gather->put << 2);

    return !stop_gather;
}

static int cdma_stopped(struct host1x_dma_gather *gather)
{
    struct host1x_cdma *cdma = gather->cdma;

    if (!cdma->enabled)
        return 1;

    if (!dma_get_is_valid(gather))
        return 1;

    if (!dma_range_is_valid(gather))
        return 1;

    return 0;
}

static void module_feed(struct host1x_dma_gather *gather,
                        uint16_t offset, uint16_t count, bool incr)
{
    struct host1x_cdma *cdma = gather->cdma;
    struct host1x_module *module = cdma->module;
    uint32_t *cmd_buf = host1x_dma_ptr + (gather->base << 2);
    uint32_t i;

    for (i = 0; i < count; i++) {
        if (cdma_stopped(gather))
            return;

        host1x_module_write(module, offset, cmd_buf[gather->get++]);

        if (incr)
            offset++;
    }
}

static void module_feed_masked(struct host1x_dma_gather *gather,
                               uint16_t offset, uint16_t mask, uint8_t mask_size)
{
    struct host1x_cdma *cdma = gather->cdma;
    struct host1x_module *module = cdma->module;
    uint32_t *cmd_buf = host1x_dma_ptr + (gather->base << 2);
    uint32_t i;

    FOREACH_BIT_SET(mask, i, mask_size) {
        if (cdma_stopped(gather))
            return;

        host1x_module_write(module, offset + i, cmd_buf[gather->get++]);
    }
}

void process_cmd_buf(struct host1x_dma_gather *gather)
{
    struct host1x_cdma *cdma = gather->cdma;
    uint32_t *cmd_buf = host1x_dma_ptr + (gather->base << 2);

    while ( !cdma_stopped(gather) ) {
        uint32_t cmd = cmd_buf[gather->get++];
        uint8_t opcode = CMD_OPCODE(cmd);

        TRACE_CDMA(cmd, gather->inlined, cdma->ch_id);

//         TPRINT("cdma=%d inlined=%d get=0x%X cmd=0x%08X\n",
//                cdma->ch_id, gather->inlined, gather->get - 1, cmd);

        bql_lock();

        switch (opcode) {
        case SETCL:
        {
            setcl_op op = { .reg32 = cmd };

            cdma->module = get_host1x_module(op.class_id);

            module_feed_masked(gather, op.offset, op.mask, 8);
            break;
        }
        case INCR:
        case NONINCR:
        {
            incr_op op = { .reg32 = cmd };

            module_feed(gather, op.offset, op.count, opcode & 1);
            break;
        }
        case MASK:
        {
            mask_op op = { .reg32 = cmd };

            module_feed_masked(gather, op.offset, op.mask, 16);
            break;
        }
        case IMM:
        {
            struct host1x_module *module = cdma->module;
            imm_op op = { .reg32 = cmd };

            host1x_module_write(module, op.offset, op.immdata);
            break;
        }
        case GATHER:
        {
            struct host1x_dma_gather gather_inlined;
            gather_op op = { .reg32 = cmd };

            g_assert(!gather->inlined);
            g_assert(dma_get_is_valid(gather));
            g_assert(dma_range_is_valid(gather));

            if (cdma_stopped(gather)) {
                bql_unlock();
                return;
            }

            gather_inlined.get = 0;
            gather_inlined.inlined = 1;
            gather_inlined.cdma = cdma;
            gather_inlined.base = cmd_buf[gather->get++] >> 2;
            gather_inlined.put = op.count;

            TPRINT("gather base=0x%08X count=%d insert=%d\n",
                   gather_inlined.base << 2, op.count, op.insert);

            if (op.count == 0)
                break;

            if (!dma_get_is_valid(&gather_inlined))
                break;

            if (op.insert)
                module_feed(&gather_inlined, op.offset, op.count, op.incr);
            else {
                bql_unlock();
                process_cmd_buf(&gather_inlined);
                bql_lock();
            }
            break;
        }
        case EXTEND:
        {
            extend_op op = { .reg32 = cmd };

            switch (op.subop) {
            case ACQUIRE_MLOCK:
                host1x_ch_acquire_mlock(cdma, op.value & 0xf);
                break;
            case RELEASE_MLOCK:
                host1x_ch_release_mlock(cdma, op.value & 0xf);
                break;
            default:
                TPRINT("%s: unknown extended subop=0x%x data=0x%08x\n",
                       __func__, op.subop, cmd);
                g_assert_not_reached();
            }
            break;
        }
        case RESTART:
        {
            restart_op op = { .reg32 = cmd };

//             printf("restart base=0x%08X get=0x%08X new_get=0x%08X\n",
//                    cdma->gather.base << 2, cdma->gather.get << 2, op.offset << 2);

            cdma->gather.get = op.offset << 2;

            if (gather->inlined) {
                bql_unlock();
                return;
            }
            break;
        }
        case CHDONE:
            break;
        default:
            TPRINT("%s: unknown opcode=0x%x data=0x%08x\n",
                   __func__, opcode, cmd);
            g_assert_not_reached();
        }

        bql_unlock();
    }
}
