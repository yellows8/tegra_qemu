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

/* WARNING: HACK */

#include "hw/irq.h"

#ifdef TEGRA_TRACE

typedef union tegra_trace_rw_u {
    struct {
        unsigned int rw:1;
        unsigned int clk_disabled:1;
        unsigned int in_reset:1;
        unsigned int size:3;
        unsigned int __pad:26;
    };

    uint32_t val;
} tegra_trace_rw_t;

static inline uint32_t ttrw(int rw, int c, int r, int s)
{
    tegra_trace_rw_t ret = {
        .rw = rw,
        .clk_disabled = c,
        .in_reset = r,
        .size = s,
    };

    return ret.val;
}

#define TRACE_READ_MEM(a, o, v, s)          tegra_trace_write(a, o, v, 0, ttrw(0,0,0,s))
#define TRACE_WRITE_MEM(a, o, v, s)         tegra_trace_write(a, o, v, v, ttrw(1,0,0,s))
#define TRACE_READ(a, o, v)                 tegra_trace_write(a, o, v, 0, ttrw(0,0,0,4))
#define TRACE_WRITE(a, o, v, n)             tegra_trace_write(a, o, v, n, ttrw(1,0,0,4))
#define TRACE_READ_EXT(a, o, v, c, r)       tegra_trace_write(a, o, v, 0, ttrw(0,c,r,4))
#define TRACE_WRITE_EXT(a, o, v, n, c, r)   tegra_trace_write(a, o, v, n, ttrw(1,c,r,4))
#define TRACE_IRQ_RAISE(a, i)               do {tegra_trace_irq(a, i->n, 1); qemu_irq_raise(i);} while (0)
#define TRACE_IRQ_LOWER(a, i)               do {tegra_trace_irq(a, i->n, 0); qemu_irq_lower(i);} while (0)
#define TRACE_IRQ_SET(a, i, v)              do {tegra_trace_irq(a, (i)->n, v); qemu_set_irq(i,v);} while (0)
#define TPRINT(f, ...)                      tegra_trace_text_message(f, ## __VA_ARGS__)
#define TRACE_CDMA(d, g, c)                 tegra_trace_cdma(d, g, c)
#define TRACE_CDMA_START(c)                 tegra_trace_cdma(0xA << 28, 0, c)
#define TRACE_CDMA_STOP(c)                  tegra_trace_cdma(0xB << 28, 0, c)
#else
#define TRACE_READ_MEM(a, o, v, s)          do {(void)(a); (void)(o); (void)(v); (void)(s);} while (0)
#define TRACE_WRITE_MEM(a, o, v, s)         do {(void)(a); (void)(o); (void)(v); (void)(s);} while (0)
#define TRACE_READ(a, o, v)                 do {(void)(a); (void)(o); (void)(v);} while (0)
#define TRACE_WRITE(a, o, v, n)             do {(void)(a); (void)(o); (void)(v); (void)(n);} while (0)
#define TRACE_READ_EXT(a, o, v, c, r)       do {(void)(a); (void)(o); (void)(v); (void)(c); (void)(r);} while (0)
#define TRACE_WRITE_EXT(a, o, v, n, c, r)   do {(void)(a); (void)(o); (void)(v); (void)(n); (void)(c); (void)(r);} while (0)
#define TRACE_IRQ_RAISE(a, i)               do {(void)(a); qemu_irq_raise(i);} while (0)
#define TRACE_IRQ_LOWER(a, i)               do {(void)(a); qemu_irq_lower(i);} while (0)
#define TRACE_IRQ_SET(a, i, v)              do {(void)(a); qemu_set_irq(i,v);} while (0)
#define TPRINT(f, ...)                      do {} while (0)
#define TRACE_CDMA(d, g, c)                 do {(void)(d); (void)(g); (void)(c);} while (0)
#define TRACE_CDMA_START(c)                 do {(void)(c);} while (0)
#define TRACE_CDMA_STOP(c)                  do {(void)(c);} while (0)
#endif //TEGRA_TRACE

int tegra_send_all(int fd, const void *_buf, int len1);

int tegra_recv_all(int fd, void *_buf, int len1, bool single_read);

void tegra_trace_irq(uint32_t hwaddr, uint32_t hwirq, uint32_t status);

void tegra_trace_write(uint32_t hwaddr, uint32_t offset,
                       uint32_t value, uint32_t new_value, uint32_t is_write);

void tegra_trace_cdma(uint32_t data, uint32_t is_gather, uint32_t ch_id);

void tegra_trace_init(void);

void tegra_trace_text_message(const char* format, ...);
