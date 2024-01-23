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

#include "exec/memory.h"
#include "ui/console.h"

#include "registers/color_palette.h"
#include "registers/color_space.h"
#include "registers/digital_vibrance.h"
#include "registers/horizontal_filtering.h"
#include "registers/vertical_filtering.h"
#include "window.h"

#include "host1x_priv.h"

#define OFFSET_IN_RANGE(offset, handler)                    \
    ((handler.begin <= offset) && (offset <= handler.end))

static int tegra_dc_to_pixman(int format)
{
    switch (format) {
    case 0: // P1
        break;
        g_assert_not_reached();
    case 1: // P2
        g_assert_not_reached();
    case 2: // P4
        g_assert_not_reached();
    case 3: // P8
        g_assert_not_reached();
    case 4: // B4G4R4A4
        return PIXMAN_a4b4g4r4;
    case 5: // B5G5R5A
        return PIXMAN_a1b5g5r5;
    case 6: // B5G6R5
        return PIXMAN_b5g6r5;
    case 7: // AB5G5R5
        return PIXMAN_a1b5g5r5;
    case 12: // B8G8R8A8
        return PIXMAN_a8r8g8b8;
    case 13: // R8G8B8A8
        return PIXMAN_a8b8g8r8;
    case 14: // B6x2G6x2R6x2A8
        return PIXMAN_a8b8g8r8;
    case 15: // R6x2G6x2B6x2A8
        return PIXMAN_x14r6g6b6;
    case 16: // YCbCr422
        g_assert_not_reached();
    case 17: // YUV422
        g_assert_not_reached();
    case 18: // YCbCr420P
	return -1;
        g_assert_not_reached();
    case 19: // YUV420P
        g_assert_not_reached();
    case 20: // YCbCr422P
        g_assert_not_reached();
    case 21: // YUV422P
        g_assert_not_reached();
    case 22: // YCbCr422R
        g_assert_not_reached();
    case 23: // YUV422R
        g_assert_not_reached();
    case 24: // YCbCr422RA
        g_assert_not_reached();
    case 25: // YUV422RA
        g_assert_not_reached();
    }

    return PIXMAN_a8b8g8r8;
}

static win_regs * alloc_regs(int cap)
{
    win_regs *regs = g_malloc0(sizeof(win_regs));
    unsigned regs_sz;

    switch (cap) {
    case CAP_COLOR_PALETTE:
        regs->shadow_type = ACTIVE;
        regs->io_handler = pallette_handler;
        regs_sz = sizeof(color_palette);
        break;
    case CAP_HORIZONTAL_FILTERING:
        regs->shadow_type = ACTIVE;
        regs->io_handler = h_filter_handler;
        regs_sz = sizeof(h_filter);
        break;
    case CAP_COLOR_SPACE_CONVERSION:
        regs->shadow_type = ACTIVE;
        regs->io_handler = csc_handler;
        regs_sz = sizeof(csc);
        break;
    case CAP_VERTICAL_FILTERING:
        regs->shadow_type = ACTIVE;
        regs->io_handler = v_filter_handler;
        regs_sz = sizeof(v_filter);
        break;
    case CAP_DIGITAL_VIBRANCE:
        regs->shadow_type = ASSEMBLY;
        regs->io_handler = dv_handler;
        regs_sz = sizeof(dv);
        break;
    default:
        g_assert_not_reached();
    };

    switch (regs->shadow_type) {
    case ARMED:
//         regs->armed = g_malloc(regs_sz);
        /* Fallthrough. */
    case ASSEMBLY:
        regs->assembly = g_malloc(regs_sz);
        /* Fallthrough. */
    case ACTIVE:
        regs->active = g_malloc(regs_sz);
    };

    regs->regs_sz = regs_sz;

    return regs;
}

uint32_t read_window(display_window *win, uint32_t offset, int st)
{
    win_regs *regs;

    QLIST_FOREACH(regs, &win->regs_list, next) {
        if (OFFSET_IN_RANGE(offset, regs->io_handler)) {
            switch (st) {
            case ASSEMBLY:
                if (regs->shadow_type != ACTIVE)
                    return regs->io_handler.read(regs->assembly, offset);
                /* Fallthrough. */
            case ACTIVE:
                return regs->io_handler.read(regs->active, offset);
            };
        }
    }

    if (OFFSET_IN_RANGE(offset, win_common_handler)) {
        if (st == ACTIVE) {
            return win_common_handler.read(&win->regs_active, offset);
        } else {
            return win_common_handler.read(&win->regs_assembly, offset);
        }
    }

    return 0;
}

static void qemu_unmap_displaysurface_guestmem(pixman_image_t *image,
                                               void *unused)
{
    void *data = pixman_image_get_data(image);
    uint32_t size = pixman_image_get_stride(image) *
        pixman_image_get_height(image);
    cpu_physical_memory_unmap(data, size, 0, 0);
}

static DisplaySurface *
qemu_create_displaysurface_guestmem(int width, int height,
                                    pixman_format_code_t format,
                                    int linesize, uint64_t addr)
{
    DisplaySurface *surface;
    hwaddr size;
    void *data;

    if (linesize == 0) {
        linesize = width * PIXMAN_FORMAT_BPP(format) / 8;
    }

    size = (hwaddr)linesize * height;
    data = cpu_physical_memory_map(addr, &size, 0);
    if (size != (hwaddr)linesize * height) {
        cpu_physical_memory_unmap(data, size, 0, 0);
        return NULL;
    }
    if (data == NULL) return NULL;

    surface = qemu_create_displaysurface_from
        (width, height, format, linesize, data);
    pixman_image_set_destroy_function
        (surface->image, qemu_unmap_displaysurface_guestmem, NULL);

    return surface;
}

static void update_window_surface(display_window *win)
{
    uint32_t starting_address = 0;

    if (tegra_dc_to_pixman(win->regs_active.win_color_depth.color_depth) == -1) return;

    starting_address += win->regs_active.winbuf_start_addr.reg32;

//     starting_address += win->regs_active.win_buf_stride.reg32 * buf_index;

    starting_address += win->regs_active.winbuf_addr_v_offset.reg32 * \
                        win->regs_active.win_line_stride.line_stride;

    starting_address += win->regs_active.winbuf_addr_h_offset.reg32;

    qemu_free_displaysurface(win->surface);

    win->surface = qemu_create_displaysurface_guestmem(
            win->regs_active.win_size.h_size,
            win->regs_active.win_size.v_size,
tegra_dc_to_pixman(win->regs_active.win_color_depth.color_depth),
            win->regs_active.win_line_stride.line_stride,
            starting_address);
}

void write_window(display_window *win, uint32_t offset, uint32_t value, int st)
{
    win_regs *regs;

    QLIST_FOREACH(regs, &win->regs_list, next) {
        if (OFFSET_IN_RANGE(offset, regs->io_handler)) {
            switch (st) {
            case ACTIVE:
                regs->io_handler.write(regs->active, offset, value);
                /* Fallthrough. */
            case ASSEMBLY:
                if (regs->shadow_type != ACTIVE)
                    regs->io_handler.write(regs->assembly, offset, value);
                return;
            };
        }
    }

    if (OFFSET_IN_RANGE(offset, win_common_handler)) {
        if (st == ACTIVE) {
            win_common_handler.write(&win->regs_active, offset, value);

            switch (offset) {
            case WINBUF_START_ADDR_OFFSET:
//             case WIN_BUF_STRIDE_OFFSET:
            /* Fallthrough. */
            case WINBUF_START_ADDR_V_OFFSET:
            case WINBUF_ADDR_H_OFFSET_OFFSET:
                update_window_surface(win);
                break;
            default:
                break;
            }
        }

        win_common_handler.write(&win->regs_assembly, offset, value);
    }
}

void latch_window_assembly(display_window *win)
{
    win_regs *regs;

    QLIST_FOREACH(regs, &win->regs_list, next) {
        if (regs->shadow_type == ACTIVE)
            continue;

        memcpy(regs->active, regs->assembly, regs->regs_sz);
    }

    update_window_surface(win);
}

void init_window(display_window *win, int caps)
{
    win_regs *regs;
    int cap;

    win->caps = caps;

    memset(win, 0, sizeof(display_window));

    FOREACH_BIT_SET(caps, cap, CAPS_NB) {
        regs = alloc_regs(cap);
        QLIST_INSERT_HEAD(&win->regs_list, regs, next);
    }
}

void reset_window(display_window *win)
{
    win_regs *regs;

    QLIST_FOREACH(regs, &win->regs_list, next) {
        if (regs->io_handler.reset != NULL) {
            switch (regs->shadow_type) {
            case ARMED:
//                 regs->armed = g_malloc(regs_sz);
            /* Fallthrough. */
            case ASSEMBLY:
                regs->io_handler.reset(regs->assembly);
            /* Fallthrough. */
            case ACTIVE:
                regs->io_handler.reset(regs->active);
            };
        }
    }

    win_common_handler.reset(&win->regs_active);
    win_common_handler.reset(&win->regs_assembly);
}
