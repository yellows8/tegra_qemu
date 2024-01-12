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

#include "qemu/thread.h"
#include "qemu/main-loop.h"

#include "host1x_fifo.h"

struct host1x_fifo {
    uint32_t *data;
    unsigned int size;
    unsigned int entries_nb;
    unsigned int first;
    unsigned int last;
    QemuMutex mutex;
    QemuCond free_cond;
};

void * host1x_fifo_create(unsigned int fifo_size)
{
    struct host1x_fifo *fifo = calloc(1, sizeof(struct host1x_fifo));

    assert(fifo != NULL);

    fifo->size = fifo_size - 1;
    fifo->data = malloc(fifo_size * sizeof(*fifo->data));

    assert(fifo->data != NULL);

    qemu_mutex_init(&fifo->mutex);
    qemu_cond_init(&fifo->free_cond);

    return fifo;
}

unsigned int host1x_get_fifo_entries_nb(struct host1x_fifo *fifo)
{
    return fifo->entries_nb;
}

void host1x_fifo_push(struct host1x_fifo *fifo, uint32_t data)
{
    bool lock = false;
    qemu_mutex_lock(&fifo->mutex);

    while (fifo->entries_nb == fifo->size) {
        bql_unlock();
        lock = true;

        qemu_cond_wait(&fifo->free_cond, &fifo->mutex);
    }

    fifo->data[fifo->last++] = data;
    fifo->entries_nb++;

    if (fifo->last > fifo->size)
        fifo->last = 0;

    qemu_mutex_unlock(&fifo->mutex);

    if (lock) {
        bql_lock();
    }
}

uint32_t host1x_fifo_pop(struct host1x_fifo *fifo)
{
    uint32_t ret;

    qemu_mutex_lock(&fifo->mutex);

    assert(fifo->entries_nb != 0);

    ret = fifo->data[fifo->first++];
    fifo->entries_nb--;

    if (fifo->first > fifo->size)
        fifo->first = 0;

    qemu_mutex_unlock(&fifo->mutex);

    qemu_cond_signal(&fifo->free_cond);

    return ret;
}

void host1x_fifo_reset(struct host1x_fifo *fifo)
{
    assert(qemu_mutex_trylock(&fifo->mutex) == 0);

    fifo->entries_nb = 0;
    fifo->first = 0;
    fifo->last = 0;

    qemu_mutex_unlock(&fifo->mutex);
}
