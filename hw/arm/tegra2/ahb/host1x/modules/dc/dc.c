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
#include "hw/ptimer.h"
#include "hw/sysbus.h"
#include "ui/console.h"
#include "sysemu/sysemu.h"

#include "registers/dc.h"
#include "window.h"

#include "host1x_module.h"
#include "host1x_syncpts.h"

#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_DC "tegra.dc"
#define TEGRA_DC(obj) OBJECT_CHECK(tegra_dc, (obj), TYPE_TEGRA_DC)

#define WIN_A_CAPS  (CAP_COLOR_PALETTE_BIT          \
                   | CAP_DIGITAL_VIBRANCE_BIT)

#define WIN_B_CAPS  (CAP_COLOR_PALETTE_BIT          \
                   | CAP_DIGITAL_VIBRANCE_BIT       \
                   | CAP_COLOR_SPACE_CONVERSION_BIT \
                   | CAP_HORIZONTAL_FILTERING_BIT   \
                   | CAP_VERTICAL_FILTERING_BIT)

#define WIN_C_CAPS  (CAP_COLOR_PALETTE_BIT          \
                   | CAP_DIGITAL_VIBRANCE_BIT       \
                   | CAP_COLOR_SPACE_CONVERSION_BIT \
                   | CAP_HORIZONTAL_FILTERING_BIT)

typedef struct tegra_dc_state {
    SysBusDevice parent_obj;

    QemuConsole *console;
    qemu_irq irq;

    uint32_t disp_width;
    uint32_t disp_height;

    MemoryRegion iomem;

    dc_regs dc;

    display_window win_a;
    display_window win_b;
    display_window win_c;
    display_window win_d;
    display_window win_t;

    struct host1x_module module;

    uint8_t disp_refresh_rate;
    ptimer_state *ptimer;
} tegra_dc;

static uint64_t tegra_dc_priv_read(void *opaque, hwaddr offset,
                                   unsigned size)
{
    tegra_dc *s = opaque;
    enum shadow_type st = (s->dc.cmd_state_access.read_mux) ? ASSEMBLY : ACTIVE;
    uint32_t ret = 0;

    offset >>= 2;

    switch (offset) {
    case 0x0 ... 0x4FF:
        if (offset < 0x80 || offset >= 0x300)
            ret = dc_handler.read(&s->dc, offset);
        else if (tegra_board >= TEGRAX1_BOARD) {
            if (offset < 0xC0)
                ret = read_window(&s->win_d, offset - 0x80 + 0x700, st);
            else if (offset >= 0xC0 && offset < 0x100)
                ret = read_window(&s->win_d, offset - 0xC0 + 0x800, st);
            else if (offset >= 0x100 && offset < 0x140)
                ret = read_window(&s->win_t, offset - 0x100 + 0x700, st);
            else if (offset >= 0x140 && offset < 0x180)
                ret = read_window(&s->win_t, offset - 0x140 + 0x800, st);
            // WINDOW H is RESERVED, so ignore it.
        }
        break;

    case 0x500 ... 0x80a:
        ret = s->dc.cmd_display_window_header.window_a_select;
        ret += s->dc.cmd_display_window_header.window_b_select;
        ret += s->dc.cmd_display_window_header.window_c_select;
        if (tegra_board >= TEGRAX1_BOARD) ret += s->dc.cmd_display_window_header.window_d_select;

        /* SW error check.  */
        //g_assert(ret <= 1);

        if (s->dc.cmd_display_window_header.window_a_select) {
            ret = read_window(&s->win_a, offset, st);
            break;
        }

        if (s->dc.cmd_display_window_header.window_b_select) {
            ret = read_window(&s->win_b, offset, st);
            offset += 0x1000;
            break;
        }

        if (s->dc.cmd_display_window_header.window_c_select) {
            ret = read_window(&s->win_c, offset, st);
            offset += 0x2000;
            break;
        }

        if (tegra_board >= TEGRAX1_BOARD && offset >= 0x700 && s->dc.cmd_display_window_header.window_d_select) {
            ret = read_window(&s->win_d, offset, st);
            offset += 0x3000;
            break;
        }
        break;

    case 0xa00 ... 0xbff:
        offset -= 0xa00;

        if (offset <= 0x138) {
            offset = 0x500 + offset;
        } else if (offset >= 0x180 && offset <= 0x199) {
            offset = 0x700 + (offset - 0x180);
        } else if (offset >= 0x1c0 && offset <= 0x1F9) {
            offset = 0x800 + (offset - 0x1c0);
        }

        ret = read_window(&s->win_a, offset, st);
        break;

    case 0xc00 ... 0xdff:
        offset -= 0xc00;

        if (offset <= 0x138) {
            offset = 0x500 + offset;
        } else if (offset >= 0x180 && offset <= 0x199) {
            offset = 0x700 + (offset - 0x180);
        } else if (offset >= 0x1c0 && offset <= 0x1F9) {
            offset = 0x800 + (offset - 0x1c0);
        }

        ret = read_window(&s->win_b, offset, st);
        offset += 0x1000;
        break;

    case 0xe00 ... 0xfff:
        offset -= 0xe00;

        if (offset <= 0x138) {
            offset = 0x500 + offset;
        } else if (offset >= 0x180 && offset <= 0x199) {
            offset = 0x700 + (offset - 0x180);
        } else if (offset >= 0x1c0 && offset <= 0x1F9) {
            offset = 0x800 + (offset - 0x1c0);
        }

        ret = read_window(&s->win_c, offset, st);
        offset += 0x2000;
        break;

    default:
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_dc_priv_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    tegra_dc *s = opaque;
    enum shadow_type st = (s->dc.cmd_state_access.write_mux) ? ASSEMBLY : ACTIVE;
    uint32_t old = tegra_dc_priv_read(s, offset, size);

    offset >>= 2;

    switch (offset) {
    case 0x0 ... 0x4FF:
        TRACE_WRITE(s->iomem.addr, offset, old, value);
        if (offset < 0x80 || offset >= 0x300) {
            dc_handler.write(&s->dc, offset, value);

            if (offset == CMD_INT_STATUS_OFFSET) {
                if (!s->dc.cmd_int_status.reg32) {
                    TRACE_IRQ_LOWER(s->iomem.addr, s->irq);
                }
            }

            if (offset == CMD_DISPLAY_COMMAND_OFFSET) {
                cmd_display_command_t old_cmd = { .reg32 = old };

                if (old_cmd.display_ctrl_mode !=
                        s->dc.cmd_display_command.display_ctrl_mode)
                {
                    ptimer_transaction_begin(s->ptimer);
                    switch (s->dc.cmd_display_command.display_ctrl_mode) {
                    case 0:
                        ptimer_stop(s->ptimer);
                        break;
                    case 1:
                        ptimer_set_limit(s->ptimer, 1, 1);
                        ptimer_run(s->ptimer, 0);
                        break;
                    case 2:
                        ptimer_set_limit(s->ptimer, 1, 1);
                        ptimer_run(s->ptimer, 1);
                        break;
                    default:
                        g_assert_not_reached();
                    }
                    ptimer_transaction_commit(s->ptimer);
                }
            }

            if (offset == CMD_STATE_CONTROL_OFFSET) {
                if (s->dc.cmd_state_control.win_a_act_req)
                    latch_window_assembly(&s->win_a);
                if (s->dc.cmd_state_control.win_b_act_req)
                    latch_window_assembly(&s->win_b);
                if (s->dc.cmd_state_control.win_c_act_req)
                    latch_window_assembly(&s->win_c);
                if (tegra_board >= TEGRAX1_BOARD && s->dc.cmd_state_control.win_d_act_req)
                    latch_window_assembly(&s->win_d);
            }
        }
        else if (tegra_board >= TEGRAX1_BOARD) {
            if (offset < 0xC0)
                write_window(&s->win_d, offset - 0x80 + 0x700, value, st);
            else if (offset >= 0xC0 && offset < 0x100)
                write_window(&s->win_d, offset - 0xC0 + 0x800, value, st);
            else if (offset >= 0x100 && offset < 0x140)
                write_window(&s->win_t, offset - 0x100 + 0x700, value, st);
            else if (offset >= 0x140 && offset < 0x180)
                write_window(&s->win_t, offset - 0x140 + 0x800, value, st);
            // WINDOW H is RESERVED, so ignore it.
        }

        break;

    case 0x500 ... 0x80a:
        if (s->dc.cmd_display_window_header.window_a_select) {
            TRACE_WRITE(s->iomem.addr, offset, old, value);
            write_window(&s->win_a, offset, value, st);
        }

        if (s->dc.cmd_display_window_header.window_b_select) {
            TRACE_WRITE(s->iomem.addr, offset + 0x1000, old, value);
            write_window(&s->win_b, offset, value, st);
        }

        if (s->dc.cmd_display_window_header.window_c_select) {
            TRACE_WRITE(s->iomem.addr, offset + 0x2000, old, value);
            write_window(&s->win_c, offset, value, st);
        }

        if (tegra_board >= TEGRAX1_BOARD && offset >= 0x700 && s->dc.cmd_display_window_header.window_d_select) {
            TRACE_WRITE(s->iomem.addr, offset + 0x3000, old, value);
            write_window(&s->win_d, offset, value, st);
            break;
        }
        break;

    case 0xa00 ... 0xbff:
        offset -= 0xa00;

        if (offset <= 0x138) {
            offset = 0x500 + offset;
        } else if (offset >= 0x180 && offset <= 0x199) {
            offset = 0x700 + (offset - 0x180);
        } else if (offset >= 0x1c0 && offset <= 0x1F9) {
            offset = 0x800 + (offset - 0x1c0);
        }

        TRACE_WRITE(s->iomem.addr, offset, old, value);
        write_window(&s->win_a, offset, value, st);
        break;

    case 0xc00 ... 0xdff:
        offset -= 0xc00;

        if (offset <= 0x138) {
            offset = 0x500 + offset;
        } else if (offset >= 0x180 && offset <= 0x199) {
            offset = 0x700 + (offset - 0x180);
        } else if (offset >= 0x1c0 && offset <= 0x1F9) {
            offset = 0x800 + (offset - 0x1c0);
        }

        TRACE_WRITE(s->iomem.addr, offset + 0x1000, old, value);
        write_window(&s->win_b, offset, value, st);
        break;

    case 0xe00 ... 0xfff:
        offset -= 0xe00;

        if (offset <= 0x138) {
            offset = 0x500 + offset;
        } else if (offset >= 0x180 && offset <= 0x199) {
            offset = 0x700 + (offset - 0x180);
        } else if (offset >= 0x1c0 && offset <= 0x1F9) {
            offset = 0x800 + (offset - 0x1c0);
        }

        TRACE_WRITE(s->iomem.addr, offset + 0x2000, old, value);
        write_window(&s->win_c, offset, value, st);
        break;

    default:
        TRACE_WRITE(s->iomem.addr, offset, old, value);
        break;
    }
}

static void tegra_dc_priv_reset(DeviceState *dev)
{
    tegra_dc *s = TEGRA_DC(dev);

    dc_handler.reset(&s->dc);
    reset_window(&s->win_a);
    reset_window(&s->win_b);
    reset_window(&s->win_c);
    reset_window(&s->win_d);
    reset_window(&s->win_t);
}

static const MemoryRegionOps tegra_dc_mem_ops = {
    .read = tegra_dc_priv_read,
    .write = tegra_dc_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

void tegra_dc_pixman_transform_flags(pixman_image_t *image, bool flipx, bool flipy, bool transpose, int *x, int *y, int *width, int *height)
{
    struct pixman_transform transform={};
    pixman_fixed_t fw = pixman_int_to_fixed(*width), fh = pixman_int_to_fixed(*height);
    pixman_transform_init_identity(&transform);

    if (!flipx && !flipy && !transpose) return;

    pixman_transform_rotate(&transform, NULL, transpose ? 0 : -pixman_fixed_1,
                                              flipy ? 0 : (flipx ? pixman_fixed_1 : -pixman_fixed_1));

    if (!flipx && !flipy && transpose)
        pixman_transform_translate(&transform, NULL, 0, fh);
    else
        pixman_transform_translate(&transform, NULL, flipx ? fw : 0,
                                                     flipy ? fh : 0);

    pixman_image_set_transform(image, &transform);

    if (transpose) {
        int tmp = *y;
        *y = *x;
        *x = tmp;
        tmp = *height;
        *height = *width;
        *width = tmp;
    }
}

void tegra_dc_pixman_transform_rotate(pixman_image_t *image, int rotate, int *x, int *y, int *width, int *height)
{
    bool flipx = false, flipy = false, transpose = false;

    switch (rotate) {
    case 90:
        transpose = true;
        break;
    case 180:
        flipx = true;
        flipy = true;
        break;
    case 270:
        flipx = true;
        transpose = true;
        break;
    }

    tegra_dc_pixman_transform_flags(image, flipx, flipy, transpose, x, y, width, height);
}

static void tegra_dc_compose_window(QemuConsole *console, display_window *win)
{
    if (!win->regs_active.win_options.win_enable || !win->surface)
        return;

    int x = win->regs_active.win_position.h_position;
    int y = win->regs_active.win_position.v_position;
    int width = win->regs_active.win_size.h_size;
    int height = win->regs_active.win_size.v_size;

    if (graphic_rotate) {
        tegra_dc_pixman_transform_rotate(win->surface->image, graphic_rotate, &x, &y, &width, &height);
    }

    pixman_image_composite(PIXMAN_OP_SRC,
                           win->surface->image, NULL,
                           qemu_console_surface(console)->image,
                           0, 0, 0, 0,
                           x, y,
                           width, height);
}

static void tegra_dc_vblank(void *opaque)
{
    tegra_dc *s = opaque;
    bool flag=false;

    if (s->dc.cmd_display_command.display_ctrl_mode == 0) {
        return;
    }

    if (s->dc.cmd_cont_syncpt_vsync.vsync_en) {
        host1x_incr_syncpt(s->dc.cmd_cont_syncpt_vsync.vsync_indx);
    }

    if (s->dc.cmd_int_enable.v_blank_int_enable) {
        s->dc.cmd_int_status.v_blank_int = 1;
        if (s->dc.cmd_int_mask.v_blank_int_mask)
            flag = true;
    }

    if (s->dc.cmd_int_enable.frame_end_int_enable) {
        s->dc.cmd_int_status.frame_end_int = 1;
        if (s->dc.cmd_int_mask.frame_end_int_mask)
            flag = true;
    }

    if (flag)
        TRACE_IRQ_RAISE(s->iomem.addr, s->irq);
}

bool tegra_dc_get_vblank_syncpt(void *opaque, uint32_t *out)
{
    tegra_dc *s = opaque;

    if (s->dc.cmd_display_command.display_ctrl_mode == 0) {
        return false;
    }

    if (s->dc.cmd_cont_syncpt_vsync.vsync_en) {
        *out = s->dc.cmd_cont_syncpt_vsync.vsync_indx;
        return true;
    }

    return false;
}

static void tegra_dc_module_write(struct host1x_module *module,
                                  uint32_t offset, uint32_t data)
{
    tegra_dc *s = module->opaque;

    TRACE_WRITE(module->class_id, offset, data, data);

    dc_handler.write(&s->dc, offset, data);
}

static uint32_t tegra_dc_module_read(struct host1x_module *module,
                                     uint32_t offset)
{
    tegra_dc *s = module->opaque;
    uint32_t ret = dc_handler.read(&s->dc, offset);

    TRACE_READ(module->class_id, offset, ret);

    return ret;
}

// NOTE: Blending is not handled.
static void tegra_dc_compose(void *opaque)
{
    tegra_dc *s = opaque;

    if (s->dc.cmd_display_command.display_ctrl_mode == 0) {
        return;
    }

    int width = s->dc.disp_disp_active.h_disp_active;
    int height = s->dc.disp_disp_active.v_disp_active;
    if (graphic_rotate == 90 || graphic_rotate == 270) {
        width = s->dc.disp_disp_active.v_disp_active;
        height = s->dc.disp_disp_active.h_disp_active;
    }

    qemu_console_resize(s->console, width, height); // This will internally do nothing if resize isn't needed.

    // Fill the image with the background color. pixman color is 16bit.
    if (tegra_board >= TEGRAX1_BOARD) {
        pixman_rectangle16_t rect = { .x = 0, .y = 0, .width = width, .height = height };
        pixman_color_t color = { .red = s->dc.disp_blend_background_color.bkgnd_red<<8,
                               .green = s->dc.disp_blend_background_color.bkgnd_green<<8,
                               .blue = s->dc.disp_blend_background_color.bkgnd_blue<<8,
                               .alpha = s->dc.disp_blend_background_color.bkgnd_alpha<<8 };

        pixman_image_fill_rectangles(PIXMAN_OP_SRC,
                                     qemu_console_surface(s->console)->image,
                                     &color,
                                     1,
                                     &rect);
    }

    tegra_dc_compose_window(s->console, &s->win_a);
    tegra_dc_compose_window(s->console, &s->win_b);
    tegra_dc_compose_window(s->console, &s->win_c);
    tegra_dc_compose_window(s->console, &s->win_d);
    tegra_dc_compose_window(s->console, &s->win_t);

    dpy_gfx_update(s->console, 0, 0, width, height);
}

static const GraphicHwOps tegra_dc_ops = {
    .gfx_update = tegra_dc_compose,
};

static void tegra_dc_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_dc *s = TEGRA_DC(dev);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_dc_mem_ops, s,
                          "tegra.dc", TEGRA_DISPLAY_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    init_window(&s->win_a, WIN_A_CAPS);
    init_window(&s->win_b, WIN_B_CAPS);
    init_window(&s->win_c, WIN_C_CAPS);
    init_window(&s->win_d, WIN_C_CAPS);
    init_window(&s->win_t, WIN_C_CAPS);

    s->ptimer = ptimer_init(tegra_dc_vblank, s, PTIMER_POLICY_CONTINUOUS_TRIGGER);
    ptimer_transaction_begin(s->ptimer);
    ptimer_set_freq(s->ptimer, s->disp_refresh_rate);
    ptimer_transaction_commit(s->ptimer);

    s->module.reg_write = tegra_dc_module_write;
    s->module.reg_read = tegra_dc_module_read;
    register_host1x_bus_module(&s->module, s);

    s->console = graphic_console_init(DEVICE(dev), 0, &tegra_dc_ops, s);
}

static Property tegra_dc_properties[] = {
    DEFINE_PROP_UINT32("display_width",  tegra_dc, disp_width, 1366),
    DEFINE_PROP_UINT32("display_height", tegra_dc, disp_height, 768),
    DEFINE_PROP_UINT8("refresh_rate", tegra_dc, disp_refresh_rate, 60),
    DEFINE_PROP_UINT8("class_id", tegra_dc, module.class_id, 0x70),
    DEFINE_PROP_END_OF_LIST(),
};

static void tegra_dc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, tegra_dc_properties);
    dc->realize = tegra_dc_priv_realize;
    dc->reset = tegra_dc_priv_reset;
}

static const TypeInfo tegra_dc_info = {
    .name = TYPE_TEGRA_DC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_dc),
    .class_init = tegra_dc_class_init,
};

static void tegra_dc_register_types(void)
{
    type_register_static(&tegra_dc_info);
}

type_init(tegra_dc_register_types)
