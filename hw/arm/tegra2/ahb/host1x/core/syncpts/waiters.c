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

#include "host1x_syncpts.h"
#include "syncpts.h"

void host1x_init_syncpt_waiter(struct host1x_syncpt_waiter *waiter)
{
    qemu_event_init(&waiter->syncpt_ev, 1);
}

void host1x_unlock_syncpt_waiter(struct host1x_syncpt_waiter *waiter)
{
    QLIST_REMOVE(waiter, next);
    qemu_event_set(&waiter->syncpt_ev);
}

void host1x_unlock_syncpt_waiter_forced(struct host1x_syncpt_waiter *waiter)
{
    struct host1x_syncpt_waiter *waiter__, *waiter_next;
    int i;

    for (i = 0; i < NV_HOST1X_SYNCPT_NB_PTS; i++) {
        struct host1x_syncpt *syncpt = &syncpts[i];

        qemu_mutex_lock(&syncpt->mutex);

        QLIST_FOREACH_SAFE(waiter__, &syncpt->waiters, next, waiter_next) {
            if (waiter != waiter__)
                continue;

            host1x_unlock_syncpt_waiter(waiter);
            qemu_mutex_unlock(&syncpt->mutex);
            return;
        }

        QLIST_FOREACH_SAFE(waiter__, &syncpt->waiters_base, next, waiter_next) {
            if (waiter != waiter__)
                continue;

            host1x_unlock_syncpt_waiter(waiter);
            qemu_mutex_unlock(&syncpt->mutex);
            return;
        }

        QLIST_FOREACH_SAFE(waiter__, &syncpt->waiters_incr, next, waiter_next) {
            if (waiter != waiter__)
                continue;

            host1x_unlock_syncpt_waiter(waiter);
            qemu_mutex_unlock(&syncpt->mutex);
            return;
        }

        qemu_mutex_unlock(&syncpt->mutex);
    }
}

void host1x_update_threshold_waiters(struct host1x_syncpt *syncpt)
{
    struct host1x_syncpt_waiter *waiter, *waiter_next;

    QLIST_FOREACH_SAFE(waiter, &syncpt->waiters, next, waiter_next) {
        if (counter_reached_threshold(syncpt->counter, waiter->threshold))
            host1x_unlock_syncpt_waiter(waiter);
    }

    QLIST_FOREACH_SAFE(waiter, &syncpt->waiters_base, next, waiter_next) {
        struct host1x_syncpt_base *syncpt_base = &syncpt_bases[waiter->base_id];
        uint32_t threshold;

        syncpt_lock(syncpt_base);

        threshold = syncpt_base->base + waiter->threshold;

        if (counter_reached_threshold(syncpt->counter, threshold))
            host1x_unlock_syncpt_waiter(waiter);

        syncpt_unlock(syncpt_base);
    }
}

void host1x_wait_syncpt(struct host1x_syncpt_waiter *waiter,
                        uint32_t syncpt_id, uint32_t threshold)
{
    struct host1x_syncpt *syncpt = &syncpts[syncpt_id];

    assert(syncpt_id < NV_HOST1X_SYNCPT_NB_PTS);

    syncpt_lock(syncpt);

    waiter->threshold = threshold;

    if (!counter_reached_threshold(syncpt->counter, waiter->threshold)) {
        QLIST_INSERT_HEAD(&syncpt->waiters, waiter, next);
        qemu_event_reset(&waiter->syncpt_ev);
    }

    syncpt_unlock(syncpt);

    bql_unlock();
    qemu_event_wait(&waiter->syncpt_ev);
    bql_lock();
}

void host1x_wait_syncpt_incr(struct host1x_syncpt_waiter *waiter,
                             uint32_t syncpt_id)
{
    struct host1x_syncpt *syncpt = &syncpts[syncpt_id];

    assert(syncpt_id < NV_HOST1X_SYNCPT_NB_PTS);

    syncpt_lock(syncpt);

    QLIST_INSERT_HEAD(&syncpt->waiters_incr, waiter, next);
    qemu_event_reset(&waiter->syncpt_ev);

    syncpt_unlock(syncpt);

    bql_unlock();
    qemu_event_wait(&waiter->syncpt_ev);
    bql_lock();
}

void host1x_update_threshold_waiters_base(uint32_t syncpt_base_id)
{
    struct host1x_syncpt_base *syncpt_base = &syncpt_bases[syncpt_base_id];
    struct host1x_syncpt_waiter *waiter, *waiter_next;
    int i;

    for (i = 0; i < NV_HOST1X_SYNCPT_NB_PTS; i++) {
        struct host1x_syncpt *syncpt = &syncpts[i];

        syncpt_lock(syncpt);

        QLIST_FOREACH_SAFE(waiter, &syncpt->waiters_base, next, waiter_next) {
            uint32_t threshold;

            if (waiter->base_id != syncpt_base_id)
                continue;

            syncpt_lock(syncpt_base);

            threshold = syncpt_base->base + waiter->threshold;

            if (counter_reached_threshold(syncpt->counter, threshold))
                host1x_unlock_syncpt_waiter(waiter);

            syncpt_unlock(syncpt_base);
        }

        syncpt_unlock(syncpt);
    }
}

void host1x_wait_syncpt_base(struct host1x_syncpt_waiter *waiter,
                             uint32_t syncpt_id, uint32_t syncpt_base_id,
                             uint32_t offset)
{
    struct host1x_syncpt_base *syncpt_base = &syncpt_bases[syncpt_base_id];
    struct host1x_syncpt *syncpt = &syncpts[syncpt_id];
    uint32_t threshold;

    assert(syncpt_id < NV_HOST1X_SYNCPT_NB_PTS);
    assert(syncpt_base_id < NV_HOST1X_SYNCPT_NB_BASES);

    syncpt_lock(syncpt);

    waiter->threshold = offset;
    waiter->base_id = syncpt_base_id;

    threshold = syncpt_base->base + waiter->threshold;

    if (!counter_reached_threshold(syncpt->counter, threshold)) {
        QLIST_INSERT_HEAD(&syncpt->waiters_base, waiter, next);
        qemu_event_reset(&waiter->syncpt_ev);
    }

    syncpt_unlock(syncpt);

    bql_unlock();
    qemu_event_wait(&waiter->syncpt_ev);
    bql_lock();
}
