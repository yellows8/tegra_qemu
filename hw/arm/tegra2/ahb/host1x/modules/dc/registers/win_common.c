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

#include <glib.h>

#include "win_common.h"

static uint32_t win_common_read(void *regs, uint32_t offset)
{
    win_common_regs *win = regs;
    uint32_t ret = 0;

    switch (offset) {
    case WIN_OPTIONS_OFFSET:
        ret = win->win_options.reg32;
        break;
    case WIN_BYTE_SWAP_OFFSET:
        ret = win->win_byte_swap.reg32;
        break;
    case WIN_BUFFER_CONTROL_OFFSET:
        ret = win->win_buffer_control.reg32;
        break;
    case WIN_COLOR_DEPTH_OFFSET:
        ret = win->win_color_depth.reg32;
        break;
    case WIN_POSITION_OFFSET:
        ret = win->win_position.reg32;
        break;
    case WIN_SIZE_OFFSET:
        ret = win->win_size.reg32;
        break;
    case WIN_PRESCALED_SIZE_OFFSET:
        ret = win->win_prescaled_size.reg32;
        break;
    case WIN_H_INITIAL_DDA_OFFSET:
        ret = win->win_h_initial_dda.reg32;
        break;
    case WIN_V_INITIAL_DDA_OFFSET:
        ret = win->win_v_initial_dda.reg32;
        break;
    case WIN_DDA_INCREMENT_OFFSET:
        ret = win->win_dda_increment.reg32;
        break;
    case WIN_LINE_STRIDE_OFFSET:
        ret = win->win_line_stride.reg32;
        break;
    case WIN_BUF_STRIDE_OFFSET:
        ret = win->win_buf_stride.reg32;
        break;
    case WIN_UV_BUF_STRIDE_OFFSET:
        ret = win->win_uv_buf_stride.reg32;
        break;
    case WIN_BUFFER_ADDR_MODE_OFFSET:
        ret = win->win_buffer_addr_mode.reg32;
        break;
    case WIN_BLEND_NOKEY_OFFSET:
        ret = win->win_blend_nokey.reg32;
        break;
    case WIN_BLEND_1WIN_OFFSET:
        ret = win->win_blend_1win.reg32;
        break;
    case WIN_BLEND_2WIN_B_OFFSET:
        ret = win->win_blend_2win_b.reg32;
        break;
    case WIN_BLEND_2WIN_C_OFFSET:
        ret = win->win_blend_2win_c.reg32;
        break;
    case WIN_BLEND_3WIN_AC_OFFSET:
        ret = win->win_blend_3win_ac.reg32;
        break;
    case WIN_HP_FETCH_CONTROL_OFFSET:
        ret = win->win_hp_fetch_control.reg32;
        break;
    case WINBUF_START_ADDR_OFFSET:
    case WINBUF_START_ADDR_NS_OFFSET:
        ret = win->winbuf_start_addr.reg32;
        break;
    case WINBUF_START_ADDR_U_OFFSET:
    case WINBUF_START_ADDR_U_NS_OFFSET:
        ret = win->winbuf_start_addr_u.reg32;
        break;
    case WINBUF_START_ADDR_V_OFFSET:
    case WINBUF_START_ADDR_V_NS_OFFSET:
        ret = win->winbuf_start_addr_v.reg32;
        break;
    case WINBUF_ADDR_H_OFFSET_OFFSET:
    case WINBUF_ADDR_H_OFFSET_NS_OFFSET:
        ret = win->winbuf_addr_h_offset.reg32;
        break;
    case WINBUF_ADDR_V_OFFSET_OFFSET:
    case WINBUF_ADDR_V_OFFSET_NS_OFFSET:
        ret = win->winbuf_addr_v_offset.reg32;
        break;
    case WINBUF_UFLOW_STATUS_OFFSET:
        ret = win->winbuf_uflow_status.reg32;
        break;
    default:
        //g_assert_not_reached();
    }

    return ret;
}

static void win_common_write(void *regs, uint32_t offset, uint32_t value)
{
    win_common_regs *win = regs;

    switch (offset) {
    case WIN_OPTIONS_OFFSET:
        win->win_options.reg32 = value;
        break;
    case WIN_BYTE_SWAP_OFFSET:
        win->win_byte_swap.reg32 = value;
        break;
    case WIN_BUFFER_CONTROL_OFFSET:
        win->win_buffer_control.reg32 = value;
        break;
    case WIN_COLOR_DEPTH_OFFSET:
        win->win_color_depth.reg32 = value;
        break;
    case WIN_POSITION_OFFSET:
        win->win_position.reg32 = value;
        break;
    case WIN_SIZE_OFFSET:
        win->win_size.reg32 = value;
        break;
    case WIN_PRESCALED_SIZE_OFFSET:
        win->win_prescaled_size.reg32 = value;
        break;
    case WIN_H_INITIAL_DDA_OFFSET:
        win->win_h_initial_dda.reg32 = value;
        break;
    case WIN_V_INITIAL_DDA_OFFSET:
        win->win_v_initial_dda.reg32 = value;
        break;
    case WIN_DDA_INCREMENT_OFFSET:
        win->win_dda_increment.reg32 = value;
        break;
    case WIN_LINE_STRIDE_OFFSET:
        win->win_line_stride.reg32 = value;
        break;
    case WIN_BUF_STRIDE_OFFSET:
        win->win_buf_stride.reg32 = value;
        break;
    case WIN_UV_BUF_STRIDE_OFFSET:
        win->win_uv_buf_stride.reg32 = value;
        break;
    case WIN_BUFFER_ADDR_MODE_OFFSET:
        win->win_buffer_addr_mode.reg32 = value;
        break;
    case WIN_BLEND_NOKEY_OFFSET:
        win->win_blend_nokey.reg32 = value;
        break;
    case WIN_BLEND_1WIN_OFFSET:
        win->win_blend_1win.reg32 = value;
        break;
    case WIN_BLEND_2WIN_B_OFFSET:
        win->win_blend_2win_b.reg32 = value;
        break;
    case WIN_BLEND_2WIN_C_OFFSET:
        win->win_blend_2win_c.reg32 = value;
        break;
    case WIN_BLEND_3WIN_AC_OFFSET:
        win->win_blend_3win_ac.reg32 = value;
        break;
    case WIN_HP_FETCH_CONTROL_OFFSET:
        win->win_hp_fetch_control.reg32 = value;
        break;
    case WINBUF_START_ADDR_OFFSET:
    case WINBUF_START_ADDR_NS_OFFSET:
        win->winbuf_start_addr.reg32 = value;
        break;
    case WINBUF_START_ADDR_U_OFFSET:
    case WINBUF_START_ADDR_U_NS_OFFSET:
        win->winbuf_start_addr_u.reg32 = value;
        break;
    case WINBUF_START_ADDR_V_OFFSET:
    case WINBUF_START_ADDR_V_NS_OFFSET:
        win->winbuf_start_addr_v.reg32 = value;
        break;
    case WINBUF_ADDR_H_OFFSET_OFFSET:
    case WINBUF_ADDR_H_OFFSET_NS_OFFSET:
        win->winbuf_addr_h_offset.reg32 = value;
        break;
    case WINBUF_ADDR_V_OFFSET_OFFSET:
    case WINBUF_ADDR_V_OFFSET_NS_OFFSET:
        win->winbuf_addr_v_offset.reg32 = value;
        break;
    case WINBUF_UFLOW_STATUS_OFFSET:
        win->winbuf_uflow_status.reg32 = value;
        break;
    default:
        //g_assert_not_reached();
    }
}

static void win_common_reset(void *regs)
{
    win_common_regs *win = regs;

    win->win_options.reg32 = WIN_OPTIONS_RESET;
    win->win_byte_swap.reg32 = WIN_BYTE_SWAP_RESET;
    win->win_buffer_control.reg32 = WIN_BUFFER_CONTROL_RESET;
    win->win_color_depth.reg32 = WIN_COLOR_DEPTH_RESET;
    win->win_position.reg32 = WIN_POSITION_RESET;
    win->win_size.reg32 = WIN_SIZE_RESET;
    win->win_prescaled_size.reg32 = WIN_PRESCALED_SIZE_RESET;
    win->win_h_initial_dda.reg32 = WIN_H_INITIAL_DDA_RESET;
    win->win_v_initial_dda.reg32 = WIN_V_INITIAL_DDA_RESET;
    win->win_dda_increment.reg32 = WIN_DDA_INCREMENT_RESET;
    win->win_line_stride.reg32 = WIN_LINE_STRIDE_RESET;
    win->win_buf_stride.reg32 = WIN_BUF_STRIDE_RESET;
    win->win_buffer_addr_mode.reg32 = WIN_BUFFER_ADDR_MODE_RESET;
    win->win_blend_nokey.reg32 = WIN_BLEND_NOKEY_RESET;
    win->win_blend_1win.reg32 = WIN_BLEND_1WIN_RESET;
    win->win_blend_2win_b.reg32 = WIN_BLEND_2WIN_B_RESET;
    win->win_blend_2win_c.reg32 = WIN_BLEND_2WIN_C_RESET;
    win->win_blend_3win_ac.reg32 = WIN_BLEND_3WIN_AC_RESET;
    win->win_hp_fetch_control.reg32 = WIN_HP_FETCH_CONTROL_RESET;
    win->winbuf_start_addr.reg32 = WINBUF_START_ADDR_RESET;
    win->winbuf_addr_h_offset.reg32 = WINBUF_ADDR_H_OFFSET_RESET;
    win->winbuf_addr_v_offset.reg32 = WINBUF_ADDR_V_OFFSET_RESET;
    win->winbuf_uflow_status.reg32 = WINBUF_UFLOW_STATUS_RESET;
}

regs_io_handler win_common_handler = {
    .read  = win_common_read,
    .write = win_common_write,
    .reset = win_common_reset,
    .begin = 0x700,
    .end   = 0x80A,
};
