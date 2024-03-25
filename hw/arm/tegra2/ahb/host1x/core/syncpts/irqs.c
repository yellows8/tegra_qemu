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

#include "hw/hw.h"

#include "host1x_syncpts.h"
#include "tegra_cpu.h"

#include "tegra_trace.h"

#define lock_irqs()     qemu_mutex_lock(&irq_mutex)
#define unlock_irqs()   qemu_mutex_unlock(&irq_mutex)

static uint32_t syncpts_irq_sts;
static uint32_t syncpts_percpu_irq_sts[HOST1X_CPUS_NB][NV_HOST1X_SYNCPT_NB_PTS/32];
static uint32_t syncpts_percpu_dst_mask[HOST1X_CPUS_NB][NV_HOST1X_SYNCPT_NB_PTS/32];

static qemu_irq *cpu_syncpts_irq;
static qemu_irq *cop_syncpts_irq;

static QemuMutex irq_mutex;

inline uint32_t host1x_get_syncpts_irq_status(void)
{
    return syncpts_irq_sts;
}

inline uint32_t host1x_get_syncpts_cpu_irq_status(uint32_t index)
{
    g_assert(index <= ARRAY_SIZE(syncpts_percpu_irq_sts[HOST1X_CPU]));
    return syncpts_percpu_irq_sts[HOST1X_CPU][index];
}

inline uint32_t host1x_get_syncpts_cop_irq_status(uint32_t index)
{
    g_assert(index <= ARRAY_SIZE(syncpts_percpu_irq_sts[HOST1X_COP]));
    return syncpts_percpu_irq_sts[HOST1X_COP][index];
}

uint32_t host1x_get_syncpts_dst_mask(uint32_t index)
{
    enum hcpu cpu_id;
    uint32_t ret = 0;
    unsigned i;
    int fb, lb;

    g_assert(index < ARRAY_SIZE(syncpts_percpu_dst_mask[cpu_id]));

    if ((index & 1) == 0) {
        fb = 0;
        lb = 16;
    }
    else {
        fb = 16;
        lb = 32;
    }

    lock_irqs();

    FOREACH_CPU(cpu_id) {
        for (i = fb; i < lb; i++) {
            if (syncpts_percpu_dst_mask[cpu_id][index>>1] & (1 << i)) {
                ret |= 1 << (i * 2 + cpu_id);
            }
        }
    }

    unlock_irqs();

    return ret;
}

static void host1x_set_irq_status_bit(enum hcpu cpu_id, bool enable)
{
    uint32_t irq_mask = (1 << 30) << cpu_id;
    int sts_updated;

    if (enable) {
        sts_updated = !(syncpts_irq_sts & irq_mask);
        syncpts_irq_sts |= irq_mask;
    } else {
        sts_updated = !!(syncpts_irq_sts & irq_mask);
        syncpts_irq_sts &= ~irq_mask;
    }

    if (!sts_updated)
        return;

    switch (cpu_id) {
    case HOST1X_CPU:
        TRACE_IRQ_SET(0x50003000, *cpu_syncpts_irq, enable);
        break;
    case HOST1X_COP:
        TRACE_IRQ_SET(0x50003000, *cop_syncpts_irq, enable);
        break;
    default:
        g_assert_not_reached();
    }
}

void host1x_enable_syncpts_irq_mask(enum hcpu cpu_id, uint32_t index, uint32_t enable_mask)
{
    enum hcpu cpu;
    unsigned i;

    g_assert(index < ARRAY_SIZE(syncpts_percpu_dst_mask[cpu_id]));

    lock_irqs();

    FOREACH_CPU(cpu) {
        if (cpu == cpu_id) {
            syncpts_percpu_dst_mask[cpu][index] |= enable_mask;
        } else {
            syncpts_percpu_dst_mask[cpu][index] &= ~enable_mask;
        }
    }

    unlock_irqs();

    FOREACH_BIT_SET(enable_mask, i, NV_HOST1X_SYNCPT_NB_PTS) {
        uint32_t id = index*32 + i;
        if (host1x_syncpt_threshold_is_crossed(id)) {
            host1x_set_syncpt_irq(id);
        }
    }
}

void host1x_set_syncpts_irq_dst_mask(uint32_t index, uint32_t mask)
{
    enum hcpu cpu_id;
    uint32_t dst_mask;
    unsigned i;

    lock_irqs();

    FOREACH_BIT_SET(mask, i, NV_HOST1X_SYNCPT_NB_PTS) {
        cpu_id = ((1 << i) & 0x55555555) ? HOST1X_CPU : HOST1X_COP;
        dst_mask = 1 << ((i - cpu_id) / 2);

        syncpts_percpu_dst_mask[cpu_id][index>>1] |= dst_mask << ((index & 1) ? 16 : 0);
    }

    unlock_irqs();
}

void host1x_clear_syncpts_irq_dst_mask(uint32_t index, uint32_t clear_mask)
{
    lock_irqs();

    syncpts_percpu_dst_mask[HOST1X_CPU][index] &= ~clear_mask;
    syncpts_percpu_dst_mask[HOST1X_COP][index] &= ~clear_mask;

    unlock_irqs();
}

void host1x_set_syncpt_irq(uint8_t syncpt_id)
{
    uint32_t irq_mask = 1 << (syncpt_id % 32);
    uint32_t index = syncpt_id/32;
    enum hcpu cpu_id;

    g_assert(syncpt_id < NV_HOST1X_SYNCPT_NB_PTS);

    lock_irqs();

    FOREACH_CPU(cpu_id) {
        if (syncpts_percpu_dst_mask[cpu_id][index] & irq_mask) {
            syncpts_percpu_irq_sts[cpu_id][index] |= irq_mask;
            host1x_set_irq_status_bit(cpu_id, 1);
        }
    }

    unlock_irqs();
}

void host1x_clear_syncpts_irq_status(enum hcpu cpu_id, uint32_t index, uint32_t clear_mask)
{
    unsigned i;

    g_assert(index < ARRAY_SIZE(syncpts_percpu_irq_sts[cpu_id]) && index < ARRAY_SIZE(syncpts_percpu_dst_mask[cpu_id]));

    lock_irqs();

    FOREACH_BIT_SET(clear_mask, i, 32) {
        if (!(syncpts_percpu_dst_mask[cpu_id][index] & (1 << i))) {
            continue;
        }

        /* Don't clear if IRQ line is not enabled and active.  */
        if (!host1x_syncpt_threshold_is_crossed(index*32 + i)) {
            clear_mask &= ~(1 << i);
        }
    }

    syncpts_percpu_irq_sts[cpu_id][index] &= ~clear_mask;

    if (syncpts_percpu_irq_sts[cpu_id][index] == 0) {
        host1x_set_irq_status_bit(cpu_id, 0);
    }

    unlock_irqs();
}

void host1x_init_syncpts_irq(qemu_irq *cpu_irq, qemu_irq *cop_irq)
{
    qemu_mutex_init(&irq_mutex);

    cpu_syncpts_irq = cpu_irq;
    cop_syncpts_irq = cop_irq;
}

void host1x_reset_syncpt_irqs(void)
{
    enum hcpu cpu_id;

    syncpts_irq_sts = 0;

    FOREACH_CPU(cpu_id) {
        memset(syncpts_percpu_irq_sts[cpu_id], 0, sizeof(syncpts_percpu_irq_sts[cpu_id]));
        memset(syncpts_percpu_dst_mask[cpu_id], 0, sizeof(syncpts_percpu_dst_mask[cpu_id]));
    }
}
