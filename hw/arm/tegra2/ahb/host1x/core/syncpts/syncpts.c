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


#include "host1x_syncpts.h"
#include "syncpts.h"

enum stype {
    COUNTER_INCR,
    COUNTER_CHANGE,
    THRESHOLD_CHANGE,
};

enum btype {
    BASE_INCR,
    BASE_CHANGE,
};

struct host1x_syncpt syncpts[NV_HOST1X_SYNCPT_NB_PTS];
struct host1x_syncpt_base syncpt_bases[NV_HOST1X_SYNCPT_NB_BASES];

/* Host1x uses HW optimization for syncpt/threshold comparison.  */
int counter_reached_threshold(uint32_t counter, uint32_t threshold)
{
    uint32_t sub = threshold - counter;
    int overflow = !!(sub & (1 << (NV_HOST1X_SYNCPT_THESH_WIDTH - 1)));
    int eq = !(sub & ((1 << (NV_HOST1X_SYNCPT_THESH_WIDTH - 1)) - 1));

    return ((overflow && !eq) || (!overflow && eq));
}

static void handle_syncpt_update(uint32_t id, enum stype op, uint32_t val)
{
    struct host1x_syncpt *syncpt = &syncpts[id];
    struct host1x_syncpt_waiter *waiter, *waiter_next;

    assert(id < NV_HOST1X_SYNCPT_NB_PTS);

    syncpt_lock(syncpt);

    switch (op) {
    case COUNTER_INCR:
        syncpt->counter++;

        QLIST_FOREACH_SAFE(waiter, &syncpt->waiters_incr, next, waiter_next) {
            host1x_unlock_syncpt_waiter(waiter);
        }
        break;
    case COUNTER_CHANGE:
        syncpt->counter = val;
        break;
    case THRESHOLD_CHANGE:
        syncpt->threshold = val;
        break;
    };

    if (counter_reached_threshold(syncpt->counter, syncpt->threshold))
        host1x_set_syncpt_irq(id);

    host1x_update_threshold_waiters(syncpt);

    syncpt_unlock(syncpt);
}

static void handle_base_update(uint32_t id, enum btype op, uint32_t val)
{
    struct host1x_syncpt_base *syncpt_base = &syncpt_bases[id];

    assert(id < NV_HOST1X_SYNCPT_NB_BASES);

    syncpt_lock(syncpt_base);

    switch (op) {
    case BASE_INCR:
        syncpt_base->base += val;
        break;
    case BASE_CHANGE:
        syncpt_base->base = val;
        break;
    };

    host1x_update_threshold_waiters_base(id);

    syncpt_unlock(syncpt_base);
}

void host1x_incr_syncpt(uint32_t syncpt_id)
{
    handle_syncpt_update(syncpt_id, COUNTER_INCR, 0);
}

void host1x_set_syncpt_count(uint32_t syncpt_id, uint32_t val)
{
    handle_syncpt_update(syncpt_id, COUNTER_CHANGE, val);
}

void host1x_set_syncpt_threshold(uint32_t syncpt_id, uint32_t val)
{
    handle_syncpt_update(syncpt_id, THRESHOLD_CHANGE, val);
}

void host1x_incr_syncpt_base(uint32_t syncpt_base_id, uint32_t val)
{
    handle_base_update(syncpt_base_id, BASE_INCR, val);
}

void host1x_set_syncpt_base(uint32_t syncpt_base_id, uint32_t val)
{
    handle_base_update(syncpt_base_id, BASE_CHANGE, val);
}

/* WARNING: Assumed atomic change.  */
uint32_t host1x_get_syncpt_count(uint32_t syncpt_id)
{
    assert(syncpt_id < NV_HOST1X_SYNCPT_NB_PTS);
    return syncpts[syncpt_id].counter;
}

uint32_t host1x_get_syncpt_threshold(uint32_t syncpt_id)
{
    assert(syncpt_id < NV_HOST1X_SYNCPT_NB_PTS);
    return syncpts[syncpt_id].threshold;
}

uint32_t host1x_get_syncpt_base(uint32_t syncpt_base_id)
{
    assert(syncpt_base_id < NV_HOST1X_SYNCPT_NB_BASES);
    return syncpt_bases[syncpt_base_id].base;
}

void host1x_init_syncpts(void)
{
    int i;

    for (i = 0; i < NV_HOST1X_SYNCPT_NB_PTS; i++)
        qemu_mutex_init(&syncpts[i].mutex);

    for (i = 0; i < NV_HOST1X_SYNCPT_NB_BASES; i++)
        qemu_mutex_init(&syncpt_bases[i].mutex);
}

/* Channels must be teared down first */
void host1x_reset_syncpts(void)
{
    int i;

    for (i = 0; i < NV_HOST1X_SYNCPT_NB_PTS; i++) {
        syncpts[i].counter = 0;
        syncpts[i].threshold = 0;
        QLIST_INIT(&syncpts[i].waiters);
        QLIST_INIT(&syncpts[i].waiters_base);
        QLIST_INIT(&syncpts[i].waiters_incr);
    }

    for (i = 0; i < NV_HOST1X_SYNCPT_NB_BASES; i++) {
        syncpt_bases[i].base = 0;
    }
}

int host1x_syncpt_threshold_is_crossed(uint32_t syncpt_id)
{
    g_assert(syncpt_id < NV_HOST1X_SYNCPT_NB_PTS);

    struct host1x_syncpt *syncpt = &syncpts[syncpt_id];

    return counter_reached_threshold(syncpt->counter, syncpt->threshold);
}
