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

#define LOCAL_SOCKET

#include "tegra_common.h"

#include "qapi/error.h"
#include "qemu/error-report.h"
#include "sysemu/sysemu.h"
#include "hw/ptimer.h"
#include "hw/sysbus.h"
#include "qemu/sockets.h"
#include "qemu/thread.h"
#include "cpu.h"

#include "ahb/host1x/include/host1x_cdma.h"
#include "ppsb/timer/timer_us.h"
#include "ppsb/timer/timer.h"
#include "devices.h"
#include "tegra_trace.h"

#define SOCKET_FILE     "/tmp/trace.sock"

#define HOST1X_CDMA	0x1010

#define PACKET_TRACE_RW 0x11111111
#define PACKET_TRACE_RW_V2 0x11111112
struct __attribute__((packed, aligned(1))) trace_pkt_rw {
    uint32_t magic;
    uint32_t hwaddr;
    uint32_t offset;
    uint32_t value;
    uint32_t new_value;
    uint32_t is_write;
    uint32_t time;
    uint32_t cpu_pc;
    uint32_t cpu_id;
};

#define PACKET_TRACE_IRQ 0x22223333
struct __attribute__((packed, aligned(1))) trace_pkt_irq {
    uint32_t magic;
    uint32_t hwaddr;
    uint32_t hwirq;
    uint32_t status;
    uint32_t time;
    uint32_t cpu_pc;
    uint32_t cpu_id;
    uint64_t __pad;
};

#define PACKET_TRACE_TXT 0x33334444
struct __attribute__((packed, aligned(1))) trace_pkt_txt {
    uint32_t magic;
    uint32_t text_sz;
    uint64_t __pad0;
    uint64_t __pad1;
    uint64_t __pad2;
    uint32_t __pad4;
//     uint32_t time;
    char text[1]; // \0
};

#define PACKET_TRACE_CDMA 0x44445555
struct __attribute__((packed, aligned(1))) trace_pkt_cdma {
    uint32_t magic;
    uint32_t time;
    uint32_t data;
    uint32_t is_gather;
    uint32_t ch_id;
    uint64_t __pad0;
    uint64_t __pad1;
};

static int msgsock = -1;
static QemuMutex send_mutex;

int tegra_send_all(int fd, const void *_buf, int len1)
{
    int ret, len;
    const uint8_t *buf;
    bool try_again = true;

    if (msgsock == -1)
        return 0;

    qemu_mutex_lock(&send_mutex);
retry:
    buf = _buf;
    len = len1;
    while (len > 0) {
        ret = write(fd, buf, len);
        if (ret < 0) {
            if (errno != EINTR && errno != EAGAIN) {
                if (try_again) {
                    tegra_trace_init();
                    try_again = false;
                    goto retry;
                }
                qemu_mutex_unlock(&send_mutex);
                return -1;
            }
        } else if (ret == 0) {
            break;
        } else {
            buf += ret;
            len -= ret;
        }
    }
    qemu_mutex_unlock(&send_mutex);
    return len1 - len;
}

int tegra_recv_all(int fd, void *_buf, int len1, bool single_read)
{
    int ret, len;
    uint8_t *buf = _buf;

    len = len1;
    while ((len > 0) && (ret = read(fd, buf, len)) != 0) {
        if (ret < 0) {
            if (errno != EINTR && errno != EAGAIN) {
                return -1;
            }
            continue;
        } else {
            if (single_read) {
                return ret;
            }
            buf += ret;
            len -= ret;
        }
    }
    return len1 - len;
}

__attribute__ ((format (printf, 1, 2))) void tegra_trace_text_message(const char* format, ...)
{
#ifdef _GNU_SOURCE
    struct trace_pkt_txt *W;
    va_list args;
    char *txt;
    size_t sz;
    int ret;

    va_start(args, format);
    ret = vasprintf(&txt, format, args);
    va_end(args);

    if (ret < 1)
        return;

    sz = sizeof(*W) + strlen(txt);
    W = malloc(sz);
    W->magic = htonl(PACKET_TRACE_TXT);
//     W->time = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
    W->text_sz = htonl(strlen(txt) + 1);
    memcpy(W->text, txt, strlen(txt) + 1);
    free(txt);

    if (tegra_send_all(msgsock, W, sz) < 0) {
        printf("%s", W->text);
    }

    free(W);
#endif
}

void tegra_trace_irq(uint32_t hwaddr, uint32_t hwirq, uint32_t status)
{
    uint32_t time = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
    struct trace_pkt_irq W = {
        htonl(PACKET_TRACE_IRQ),
        htonl(hwaddr),
        htonl(hwirq),
        htonl(status),
        htonl(time),
        htonl(0),
        htonl(0)
    };

    tegra_send_all(msgsock, &W, sizeof(W));
}

void tegra_trace_write(uint32_t hwaddr, uint32_t offset,
                       uint32_t value, uint32_t new_value, uint32_t is_write)
{
    CPUState *cs = CPU(current_cpu);
    ARMCPU *cpu = ARM_CPU(cs);
    uint32_t cpu_pc = cpu ? cpu->env.regs[15] : 0;
    uint32_t time = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
    uint32_t cpu_id = host1x_cdma_ptr ? HOST1X_CDMA : (cs ? cs->cpu_index : 0xFF);
    struct trace_pkt_rw W = {
        htonl((is_write > 1) ? PACKET_TRACE_RW_V2 : PACKET_TRACE_RW),
        htonl(hwaddr),
        htonl(offset),
        htonl(value),
        htonl(new_value),
        htonl(is_write),
        htonl(time),
        htonl(cpu_pc),
        htonl(cpu_id)
    };

    tegra_send_all(msgsock, &W, sizeof(W));
}

void tegra_trace_cdma(uint32_t data, uint32_t is_gather, uint32_t ch_id)
{
    uint32_t time = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
    struct trace_pkt_cdma W = {
        htonl(PACKET_TRACE_CDMA),
        htonl(time),
        htonl(data),
        htonl(is_gather),
        htonl(ch_id)
    };

    tegra_send_all(msgsock, &W, sizeof(W));
}

#define CMD_CHANGE_TIMERS_FREQ      0x122

#ifdef TEGRA_TRACE
static void * trace_viewer_cmd_handler(void *arg)
{
    tegra_timer_us **timer_us = (void *) &tegra_timer_us_dev;
    tegra_timer **timer3 = (void *) &tegra_timer3_dev;
    tegra_timer **timer2 = (void *) &tegra_timer2_dev;
    uint32_t freq;
    uint32_t cmd;

    for (;;) {
        if (msgsock == -1 || tegra_recv_all(msgsock, &cmd, sizeof(cmd), 0) < sizeof(cmd)) {
            sleep(1);
            continue;
        }

        switch (cmd) {
        case CMD_CHANGE_TIMERS_FREQ:
            tegra_recv_all(msgsock, &freq, sizeof(freq), 0);
            /* Does't include ARM's MPtimer!  */
            ptimer_transaction_begin((*timer_us)->ptimer);
            ptimer_set_freq((*timer_us)->ptimer, freq);
            ptimer_transaction_commit((*timer_us)->ptimer);

            ptimer_transaction_begin((*timer3)->ptimer);
            ptimer_set_freq((*timer3)->ptimer, freq);
            ptimer_transaction_commit((*timer3)->ptimer);

            ptimer_transaction_begin((*timer2)->ptimer);
            ptimer_set_freq((*timer2)->ptimer, freq);
            ptimer_transaction_commit((*timer2)->ptimer);
            break;
        default:
            if (errno == 0) {
                fprintf(stderr, "%s unknown cmd 0x%X\n", __func__, cmd);
            }
        }
    }

    return NULL;
}
#endif // TEGRA_TRACE

void tegra_trace_init(void)
{
#ifdef TEGRA_TRACE
    SocketAddress *saddr;
    QemuThread trace_cmd_thread;
    static int sock = -1;

    if (msgsock != -1) {
        close(msgsock);
    }

    if (sock != -1) {
        goto WAIT;
    }

    qemu_mutex_init(&send_mutex);

#ifdef LOCAL_SOCKET
    saddr = socket_parse("unix:" SOCKET_FILE, &error_abort);
#else
    saddr = socket_parse("0.0.0.0:19191", &error_abort);
#endif // LOCAL_SOCKET

    sock = socket_listen(saddr, 1, &error_abort);
    socket_set_fast_reuse(sock);

    if (listen(sock, 1) < 0) {
        error_setg_errno(&error_abort, errno, "Failed to listen on socket");
    }

    qemu_thread_create(&trace_cmd_thread, "trace_cmd_handler",
                       trace_viewer_cmd_handler,
                       NULL, QEMU_THREAD_DETACHED);

WAIT:
    printf("Waiting for trace viewer connection...\n");
    msgsock = qemu_accept(sock, NULL, NULL);
    g_assert(msgsock != -1);
#endif // TEGRA_TRACE
}
