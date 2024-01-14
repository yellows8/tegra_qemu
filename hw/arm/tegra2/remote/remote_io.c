/*
 * ARM NVIDIA Tegra2 emulation.
 *
 * Copyright (c) 2015 Dmitry Osipenko <digetx@gmail.com>
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

#define CONFIG_ARCH_TEGRA_2x_SOC

#include "tegra_common.h"

#include "qemu/sockets.h"
#include "qemu/thread.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/hw.h"
#include "cpu.h"

#include "irqs.h"
#include "remote_io.h"
#include "tegra_cpu.h"
#include "tegra_trace.h"

#define RST_DEV_L_SET           0x60006300
#define CLK_ENB_L_SET           0x60006320

#define __pak   __attribute__((packed, aligned(1)))

#define FOREACH_BIT_SET(val, itr, size)     \
    if (val != 0)                           \
        for (itr = 0; itr < size; itr++)    \
            if ((val >> itr) & 1)

#define REMOTE_IO_PKT_SIZE  10

#define REMOTE_IO_READ      0x0

struct __pak remote_io_read_req {
    uint8_t magic;
    uint32_t address;
    unsigned size:7;
    unsigned on_avp:1;
    uint32_t __pad32;
};

#define REMOTE_IO_READ_RESP 0x1

struct __pak remote_io_read_resp {
    uint8_t magic;
    uint32_t data;
    uint32_t __pad32;
    uint8_t __pad8;
};

#define REMOTE_IO_WRITE     0x2

struct __pak remote_io_write_req {
    uint8_t magic;
    uint32_t address;
    uint32_t value;
    unsigned size:7;
    unsigned on_avp:1;
};

#define REMOTE_IO_IRQ_WATCH 0x3

struct __pak remote_io_irq_watch_req {
    uint8_t magic;
    uint32_t irq_nb;
    uint32_t __pad32;
    uint8_t __pad8;
};

#define REMOTE_IO_IRQ_STS   0x4

struct __pak remote_io_irq_notify {
    uint8_t magic;
    uint8_t bank;
    uint32_t upd;
    uint32_t sts;
};

#define REMOTE_IO_READ_MEM_RANGE 0x5

struct __pak remote_io_read_range_req {
    uint8_t magic;
    uint32_t address;
    uint32_t size;
    uint8_t __pad8;
};

#define REMOTE_IO_READ_MEM_RANGE_RESP 0x6

struct remote_irq {
    qemu_irq *irq;
    uint32_t base_addr;
};

static struct remote_irq remote_irqs[INT_MAIN_NR];

static const char *remote_addr;
static int sock = -1;

static QemuThread recv_thread;
static QemuMutex io_mutex;
static QemuEvent read_ev;
static uint32_t read_val;
static void *read_range_data;
static uint32_t read_range_size = UINT32_MAX;

static void remote_io_connect(void)
{
    int i;

    if (sock != -1) {
        shutdown(sock, SHUT_RDWR);
        close(sock);
        sock = -1;
    }

    while (sock < 0) {
        Error *err = NULL;

        printf("remote_io: connecting to %s ...\n", remote_addr);

        sock = inet_connect(remote_addr, &err);

        if (sock < 0) {
            error_report_err(err);
            sleep(1);
        }
    }

    for (i = 0; i < INT_MAIN_NR; i++) {
        if (remote_irqs[i].base_addr != 0) {
            remote_io_watch_irq(remote_irqs[i].base_addr,
                                remote_irqs[i].irq);
        }
    }
}

static void remote_irq_handle(struct remote_io_irq_notify *inotify)
{
    int i;

    remote_io_read_cache_invalidate();

    FOREACH_BIT_SET(inotify->upd, i, 32) {
        int irq_nb = inotify->bank * 32 + i;

        TRACE_IRQ_SET(remote_irqs[irq_nb].base_addr, *remote_irqs[irq_nb].irq,
                      !!(inotify->sts & (1 << i)));
    }
}

static void * remote_io_recv_handler(void *arg)
{
    struct remote_io_irq_notify *inotify;
    struct remote_io_read_resp *resp;
    char buf[REMOTE_IO_PKT_SIZE];
    int magic;

    for (;;) {
        magic = tegra_recv_all(sock, buf, sizeof(buf), 0);

        if (magic < 1) {
            sleep(1);
            continue;
        }

        if (magic < sizeof(buf)) {
            hw_error("%s failed ret %d\n", __func__, magic);
        }

        magic = buf[0];

        switch (magic) {
        case REMOTE_IO_READ_RESP:
            resp = (void *) buf;
            read_val = resp->data;

            qemu_event_set(&read_ev);
            break;
        case REMOTE_IO_IRQ_STS:
            inotify = (void *) buf;

            remote_irq_handle(inotify);
            break;
        case REMOTE_IO_READ_MEM_RANGE_RESP:
            if (tegra_recv_all(sock, read_range_data, read_range_size, 0) < read_range_size) {
                hw_error("%s read_range_size failed\n", __func__);
            }

            qemu_event_set(&read_ev);
            break;
        default:
            hw_error("%s bad magic %d\n", __func__, magic);
        }
    }

    return NULL;
}

void remote_io_read_mem_range(uint8_t *data, uint32_t addr, uint32_t size)
{
    struct remote_io_read_range_req req = {
        .magic = REMOTE_IO_READ_MEM_RANGE,
        .address = addr,
        .size = size,
    };
    int ret;

    if (sock == -1) {
        return;
    }

    qemu_mutex_lock(&io_mutex);
    qemu_event_reset(&read_ev);

    read_range_data = data;
    read_range_size = size;

retry:
    ret = tegra_send_all(sock, &req, sizeof(req));

    if (ret < 1) {
        remote_io_connect();
        goto retry;
    }

    if (ret < sizeof(req)) {
        hw_error("%s failed\n", __func__);
    }

    qemu_event_wait(&read_ev);

    read_range_size = UINT32_MAX;

    qemu_mutex_unlock(&io_mutex);
}

uint32_t remote_io_read(uint32_t addr, int size)
{
    struct remote_io_read_req req = {
        .magic = REMOTE_IO_READ,
        .address = addr,
        .size = size,
        .on_avp = (current_cpu && current_cpu->cpu_index == TEGRA_BPMP),
    };
    int64_t ret;

    if (sock == -1) {
        return 0;
    }

    qemu_mutex_lock(&io_mutex);
    qemu_event_reset(&read_ev);

retry:
    ret = tegra_send_all(sock, &req, sizeof(req));

    if (ret < 1) {
        remote_io_connect();
        goto retry;
    }

    if (ret < sizeof(req)) {
        hw_error("%s failed\n", __func__);
    }

    qemu_event_wait(&read_ev);
    ret = read_val;
    qemu_mutex_unlock(&io_mutex);

    return ret;
}

void remote_io_write(uint32_t value, uint32_t addr, int size)
{
    struct remote_io_write_req req = {
        .magic = REMOTE_IO_WRITE,
        .value = value,
        .address = addr,
        .size = size,
        .on_avp = (current_cpu && current_cpu->cpu_index == TEGRA_BPMP),
    };
    int ret;

    if (sock == -1) {
        return;
    }

    if (addr > 0x60000000) {
        remote_io_read_cache_invalidate();
    }

retry:
    ret = tegra_send_all(sock, &req, sizeof(req));

    if (ret < 1) {
        remote_io_connect();
        goto retry;
    }

    if (ret < sizeof(req)) {
        hw_error("%s failed\n", __func__);
    }
}

void remote_io_watch_irq(uint32_t base_addr, qemu_irq *irq)
{
    struct remote_io_irq_watch_req req = {
        .magic = REMOTE_IO_IRQ_WATCH,
        .irq_nb = (*irq)->n,
    };

    if (sock == -1) {
        return;
    }

    remote_irqs[(*irq)->n].base_addr = base_addr;
    remote_irqs[(*irq)->n].irq = irq;

    if (tegra_send_all(sock, &req, sizeof(req)) < 0) {
        hw_error("%s failed\n", __func__);
    }
}

void remote_io_rst_set(uint8_t id, int enb)
{
    unsigned bank = id >> 5;
    uint32_t addr = RST_DEV_L_SET + (bank << 3) + (!enb << 2);
    uint32_t val = 1 << (id & 0x1F);

    remote_io_write(val, addr, 32);
}

void remote_io_clk_set(uint8_t id, int enb)
{
    unsigned bank = id >> 5;
    uint32_t addr = CLK_ENB_L_SET + (bank << 3) + (!enb << 2);
    uint32_t val = 1 << (id & 0x1F);

    remote_io_write(val, addr, 32);
}

void remote_io_init(const char *addr)
{
    if (addr == NULL || sock != -1) {
        return;
    }

    qemu_mutex_init(&io_mutex);
    qemu_event_init(&read_ev, 0);

    remote_addr = addr;

    remote_io_connect();

    qemu_thread_create(&recv_thread, "remote_io_recv", remote_io_recv_handler,
                       NULL, QEMU_THREAD_DETACHED);

    remote_io_read_cache_invalidate();
}
