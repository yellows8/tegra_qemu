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

#include "host1x_syncpts.h"

#include <pixman.h>
#include "dc.h"

static uint32_t dc_read(void *regs, uint32_t offset)
{
    dc_regs *dc = regs;
    uint32_t ret = 0;

    switch (offset) {
    case CMD_GENERAL_INCR_SYNCPT_OFFSET:
    case CMD_GENERAL_INCR_SYNCPT_CNTRL_OFFSET:
    case CMD_GENERAL_INCR_SYNCPT_ERROR_OFFSET:
    case CMD_WIN_A_INCR_SYNCPT_OFFSET:
    case CMD_WIN_A_INCR_SYNCPT_CNTRL_OFFSET:
    case CMD_WIN_A_INCR_SYNCPT_ERROR_OFFSET:
    case CMD_WIN_B_INCR_SYNCPT_OFFSET:
    case CMD_WIN_B_INCR_SYNCPT_CNTRL_OFFSET:
    case CMD_WIN_B_INCR_SYNCPT_ERROR_OFFSET:
    case CMD_WIN_C_INCR_SYNCPT_OFFSET:
    case CMD_WIN_C_INCR_SYNCPT_CNTRL_OFFSET:
    case CMD_WIN_C_INCR_SYNCPT_ERROR_OFFSET:
        break;
    case CMD_CONT_SYNCPT_VSYNC_OFFSET:
        ret = dc->cmd_cont_syncpt_vsync.reg32;
        break;
    case CMD_CTXSW_OFFSET:
        ret = dc->cmd_ctxsw.reg32;
        break;
    case CMD_DISPLAY_COMMAND_OPTION0_OFFSET:
        ret = dc->cmd_display_command_option0.reg32;
        break;
    case CMD_DISPLAY_COMMAND_OFFSET:
        ret = dc->cmd_display_command.reg32;
        break;
    case CMD_SIGNAL_RAISE_OFFSET:
        ret = dc->cmd_signal_raise.reg32;
        break;
    case CMD_DISPLAY_POWER_CONTROL_OFFSET:
        ret = dc->cmd_display_power_control.reg32;
        break;
    case CMD_INT_STATUS_OFFSET:
        ret = dc->cmd_int_status.reg32;
        break;
    case CMD_INT_MASK_OFFSET:
        ret = dc->cmd_int_mask.reg32;
        break;
    case CMD_INT_ENABLE_OFFSET:
        ret = dc->cmd_int_enable.reg32;
        break;
    case CMD_INT_TYPE_OFFSET:
        ret = dc->cmd_int_type.reg32;
        break;
    case CMD_INT_POLARITY_OFFSET:
        ret = dc->cmd_int_polarity.reg32;
        break;
    case CMD_SIGNAL_RAISE1_OFFSET:
        ret = dc->cmd_signal_raise1.reg32;
        break;
    case CMD_SIGNAL_RAISE2_OFFSET:
        ret = dc->cmd_signal_raise2.reg32;
        break;
    case CMD_SIGNAL_RAISE3_OFFSET:
        ret = dc->cmd_signal_raise3.reg32;
        break;
    case CMD_STATE_ACCESS_OFFSET:
        ret = dc->cmd_state_access.reg32;
        break;
    case CMD_STATE_CONTROL_OFFSET:
        ret = dc->cmd_state_control.reg32;
        break;
    case CMD_DISPLAY_WINDOW_HEADER_OFFSET:
        ret = dc->cmd_display_window_header.reg32;
        break;
    case CMD_REG_ACT_CONTROL_OFFSET:
        ret = dc->cmd_reg_act_control.reg32;
        break;
    case COM_CRC_CONTROL_OFFSET:
        ret = dc->com_crc_control.reg32;
        break;
    case COM_CRC_CHECKSUM_OFFSET:
        ret = dc->com_crc_checksum.reg32;
        break;
    case COM_PIN_OUTPUT_ENABLE0_OFFSET:
        ret = dc->com_pin_output_enable0.reg32;
        break;
    case COM_PIN_OUTPUT_ENABLE1_OFFSET:
        ret = dc->com_pin_output_enable1.reg32;
        break;
    case COM_PIN_OUTPUT_ENABLE2_OFFSET:
        ret = dc->com_pin_output_enable2.reg32;
        break;
    case COM_PIN_OUTPUT_ENABLE3_OFFSET:
        ret = dc->com_pin_output_enable3.reg32;
        break;
    case COM_PIN_OUTPUT_POLARITY0_OFFSET:
        ret = dc->com_pin_output_polarity0.reg32;
        break;
    case COM_PIN_OUTPUT_POLARITY1_OFFSET:
        ret = dc->com_pin_output_polarity1.reg32;
        break;
    case COM_PIN_OUTPUT_POLARITY2_OFFSET:
        ret = dc->com_pin_output_polarity2.reg32;
        break;
    case COM_PIN_OUTPUT_POLARITY3_OFFSET:
        ret = dc->com_pin_output_polarity3.reg32;
        break;
    case COM_PIN_OUTPUT_DATA0_OFFSET:
        ret = dc->com_pin_output_data0.reg32;
        break;
    case COM_PIN_OUTPUT_DATA1_OFFSET:
        ret = dc->com_pin_output_data1.reg32;
        break;
    case COM_PIN_OUTPUT_DATA2_OFFSET:
        ret = dc->com_pin_output_data2.reg32;
        break;
    case COM_PIN_OUTPUT_DATA3_OFFSET:
        ret = dc->com_pin_output_data3.reg32;
        break;
    case COM_PIN_INPUT_ENABLE0_OFFSET:
        ret = dc->com_pin_input_enable0.reg32;
        break;
    case COM_PIN_INPUT_ENABLE1_OFFSET:
        ret = dc->com_pin_input_enable1.reg32;
        break;
    case COM_PIN_INPUT_ENABLE2_OFFSET:
        ret = dc->com_pin_input_enable2.reg32;
        break;
    case COM_PIN_INPUT_ENABLE3_OFFSET:
        ret = dc->com_pin_input_enable3.reg32;
        break;
    case COM_PIN_INPUT_DATA0_OFFSET:
        ret = dc->com_pin_input_data0.reg32;
        break;
    case COM_PIN_INPUT_DATA1_OFFSET:
        ret = dc->com_pin_input_data1.reg32;
        break;
    case COM_PIN_OUTPUT_SELECT0_OFFSET:
        ret = dc->com_pin_output_select0.reg32;
        break;
    case COM_PIN_OUTPUT_SELECT1_OFFSET:
        ret = dc->com_pin_output_select1.reg32;
        break;
    case COM_PIN_OUTPUT_SELECT2_OFFSET:
        ret = dc->com_pin_output_select2.reg32;
        break;
    case COM_PIN_OUTPUT_SELECT3_OFFSET:
        ret = dc->com_pin_output_select3.reg32;
        break;
    case COM_PIN_OUTPUT_SELECT4_OFFSET:
        ret = dc->com_pin_output_select4.reg32;
        break;
    case COM_PIN_OUTPUT_SELECT5_OFFSET:
        ret = dc->com_pin_output_select5.reg32;
        break;
    case COM_PIN_OUTPUT_SELECT6_OFFSET:
        ret = dc->com_pin_output_select6.reg32;
        break;
    case COM_PIN_MISC_CONTROL_OFFSET:
        ret = dc->com_pin_misc_control.reg32;
        break;
    case COM_PM0_CONTROL_OFFSET:
        ret = dc->com_pm0_control.reg32;
        break;
    case COM_PM0_DUTY_CYCLE_OFFSET:
        ret = dc->com_pm0_duty_cycle.reg32;
        break;
    case COM_PM1_CONTROL_OFFSET:
        ret = dc->com_pm1_control.reg32;
        break;
    case COM_PM1_DUTY_CYCLE_OFFSET:
        ret = dc->com_pm1_duty_cycle.reg32;
        break;
    case COM_SPI_CONTROL_OFFSET:
        ret = dc->com_spi_control.reg32;
        break;
    case COM_SPI_START_BYTE_OFFSET:
        ret = dc->com_spi_start_byte.reg32;
        break;
    case COM_HSPI_WRITE_DATA_AB_OFFSET:
        ret = dc->com_hspi_write_data_ab.reg32;
        break;
    case COM_HSPI_WRITE_DATA_CD_OFFSET:
        ret = dc->com_hspi_write_data_cd.reg32;
        break;
    case COM_HSPI_CS_DC_OFFSET:
        ret = dc->com_hspi_cs_dc.reg32;
        break;
    case COM_SCRATCH_REGISTER_A_OFFSET:
        ret = dc->com_scratch_register_a.reg32;
        break;
    case COM_SCRATCH_REGISTER_B_OFFSET:
        ret = dc->com_scratch_register_b.reg32;
        break;
    case COM_GPIO_CTRL_OFFSET:
        ret = dc->com_gpio_ctrl.reg32;
        break;
    case COM_GPIO_DEBOUNCE_COUNTER_OFFSET:
        ret = dc->com_gpio_debounce_counter.reg32;
        break;
    case COM_CRC_CHECKSUM_LATCHED_OFFSET:
        ret = dc->com_crc_checksum_latched.reg32;
        break;
    case DISP_DISP_SIGNAL_OPTIONS0_OFFSET:
        ret = dc->disp_disp_signal_options0.reg32;
        break;
    case DISP_DISP_SIGNAL_OPTIONS1_OFFSET:
        ret = dc->disp_disp_signal_options1.reg32;
        break;
    case DISP_DISP_WIN_OPTIONS_OFFSET:
        ret = dc->disp_disp_win_options.reg32;
        break;
    case DISP_MEM_HIGH_PRIORITY_OFFSET:
        ret = dc->disp_mem_high_priority.reg32;
        break;
    case DISP_MEM_HIGH_PRIORITY_TIMER_OFFSET:
        ret = dc->disp_mem_high_priority_timer.reg32;
        break;
    case DISP_DISP_TIMING_OPTIONS_OFFSET:
        ret = dc->disp_disp_timing_options.reg32;
        break;
    case DISP_REF_TO_SYNC_OFFSET:
        ret = dc->disp_ref_to_sync.reg32;
        break;
    case DISP_SYNC_WIDTH_OFFSET:
        ret = dc->disp_sync_width.reg32;
        break;
    case DISP_BACK_PORCH_OFFSET:
        ret = dc->disp_back_porch.reg32;
        break;
    case DISP_DISP_ACTIVE_OFFSET:
        ret = dc->disp_disp_active.reg32;
        break;
    case DISP_FRONT_PORCH_OFFSET:
        ret = dc->disp_front_porch.reg32;
        break;
    case DISP_H_PULSE0_CONTROL_OFFSET:
        ret = dc->disp_h_pulse0_control.reg32;
        break;
    case DISP_H_PULSE0_POSITION_A_OFFSET:
        ret = dc->disp_h_pulse0_position_a.reg32;
        break;
    case DISP_H_PULSE0_POSITION_B_OFFSET:
        ret = dc->disp_h_pulse0_position_b.reg32;
        break;
    case DISP_H_PULSE0_POSITION_C_OFFSET:
        ret = dc->disp_h_pulse0_position_c.reg32;
        break;
    case DISP_H_PULSE0_POSITION_D_OFFSET:
        ret = dc->disp_h_pulse0_position_d.reg32;
        break;
    case DISP_H_PULSE1_CONTROL_OFFSET:
        ret = dc->disp_h_pulse1_control.reg32;
        break;
    case DISP_H_PULSE1_POSITION_A_OFFSET:
        ret = dc->disp_h_pulse1_position_a.reg32;
        break;
    case DISP_H_PULSE1_POSITION_B_OFFSET:
        ret = dc->disp_h_pulse1_position_b.reg32;
        break;
    case DISP_H_PULSE1_POSITION_C_OFFSET:
        ret = dc->disp_h_pulse1_position_c.reg32;
        break;
    case DISP_H_PULSE1_POSITION_D_OFFSET:
        ret = dc->disp_h_pulse1_position_d.reg32;
        break;
    case DISP_H_PULSE2_CONTROL_OFFSET:
        ret = dc->disp_h_pulse2_control.reg32;
        break;
    case DISP_H_PULSE2_POSITION_A_OFFSET:
        ret = dc->disp_h_pulse2_position_a.reg32;
        break;
    case DISP_H_PULSE2_POSITION_B_OFFSET:
        ret = dc->disp_h_pulse2_position_b.reg32;
        break;
    case DISP_H_PULSE2_POSITION_C_OFFSET:
        ret = dc->disp_h_pulse2_position_c.reg32;
        break;
    case DISP_H_PULSE2_POSITION_D_OFFSET:
        ret = dc->disp_h_pulse2_position_d.reg32;
        break;
    case DISP_V_PULSE0_CONTROL_OFFSET:
        ret = dc->disp_v_pulse0_control.reg32;
        break;
    case DISP_V_PULSE0_POSITION_A_OFFSET:
        ret = dc->disp_v_pulse0_position_a.reg32;
        break;
    case DISP_V_PULSE0_POSITION_B_OFFSET:
        ret = dc->disp_v_pulse0_position_b.reg32;
        break;
    case DISP_V_PULSE0_POSITION_C_OFFSET:
        ret = dc->disp_v_pulse0_position_c.reg32;
        break;
    case DISP_V_PULSE1_CONTROL_OFFSET:
        ret = dc->disp_v_pulse1_control.reg32;
        break;
    case DISP_V_PULSE1_POSITION_A_OFFSET:
        ret = dc->disp_v_pulse1_position_a.reg32;
        break;
    case DISP_V_PULSE1_POSITION_B_OFFSET:
        ret = dc->disp_v_pulse1_position_b.reg32;
        break;
    case DISP_V_PULSE1_POSITION_C_OFFSET:
        ret = dc->disp_v_pulse1_position_c.reg32;
        break;
    case DISP_V_PULSE2_CONTROL_OFFSET:
        ret = dc->disp_v_pulse2_control.reg32;
        break;
    case DISP_V_PULSE2_POSITION_A_OFFSET:
        ret = dc->disp_v_pulse2_position_a.reg32;
        break;
    case DISP_V_PULSE3_CONTROL_OFFSET:
        ret = dc->disp_v_pulse3_control.reg32;
        break;
    case DISP_V_PULSE3_POSITION_A_OFFSET:
        ret = dc->disp_v_pulse3_position_a.reg32;
        break;
    case DISP_M0_CONTROL_OFFSET:
        ret = dc->disp_m0_control.reg32;
        break;
    case DISP_M1_CONTROL_OFFSET:
        ret = dc->disp_m1_control.reg32;
        break;
    case DISP_DI_CONTROL_OFFSET:
        ret = dc->disp_di_control.reg32;
        break;
    case DISP_PP_CONTROL_OFFSET:
        ret = dc->disp_pp_control.reg32;
        break;
    case DISP_PP_SELECT_A_OFFSET:
        ret = dc->disp_pp_select_a.reg32;
        break;
    case DISP_PP_SELECT_B_OFFSET:
        ret = dc->disp_pp_select_b.reg32;
        break;
    case DISP_PP_SELECT_C_OFFSET:
        ret = dc->disp_pp_select_c.reg32;
        break;
    case DISP_PP_SELECT_D_OFFSET:
        ret = dc->disp_pp_select_d.reg32;
        break;
    case DISP_DISP_CLOCK_CONTROL_OFFSET:
        ret = dc->disp_disp_clock_control.reg32;
        break;
    case DISP_DISP_INTERFACE_CONTROL_OFFSET:
        ret = dc->disp_disp_interface_control.reg32;
        break;
    case DISP_DISP_COLOR_CONTROL_OFFSET:
        ret = dc->disp_disp_color_control.reg32;
        break;
    case DISP_SHIFT_CLOCK_OPTIONS_OFFSET:
        ret = dc->disp_shift_clock_options.reg32;
        break;
    case DISP_DATA_ENABLE_OPTIONS_OFFSET:
        ret = dc->disp_data_enable_options.reg32;
        break;
    case DISP_SERIAL_INTERFACE_OPTIONS_OFFSET:
        ret = dc->disp_serial_interface_options.reg32;
        break;
    case DISP_LCD_SPI_OPTIONS_OFFSET:
        ret = dc->disp_lcd_spi_options.reg32;
        break;
    case DISP_BORDER_COLOR_OFFSET:
        ret = dc->disp_border_color.reg32;
        break;
    case DISP_COLOR_KEY0_LOWER_OFFSET:
        ret = dc->disp_color_key0_lower.reg32;
        break;
    case DISP_COLOR_KEY0_UPPER_OFFSET:
        ret = dc->disp_color_key0_upper.reg32;
        break;
    case DISP_COLOR_KEY1_LOWER_OFFSET:
        ret = dc->disp_color_key1_lower.reg32;
        break;
    case DISP_COLOR_KEY1_UPPER_OFFSET:
        ret = dc->disp_color_key1_upper.reg32;
        break;
    case DISP_CURSOR_FOREGROUND_OFFSET:
        ret = dc->disp_cursor_foreground.reg32;
        break;
    case DISP_CURSOR_BACKGROUND_OFFSET:
        ret = dc->disp_cursor_background.reg32;
        break;
    case DISP_CURSOR_START_ADDR_OFFSET:
    case DISP_CURSOR_START_ADDR_NS_OFFSET:
        ret = dc->disp_cursor_start_addr.reg32;
        break;
    case DISP_CURSOR_POSITION_OFFSET:
    case DISP_CURSOR_POSITION_NS_OFFSET:
        ret = dc->disp_cursor_position.reg32;
        break;
    case DISP_INIT_SEQ_CONTROL_OFFSET:
        ret = dc->disp_init_seq_control.reg32;
        break;
    case DISP_SPI_INIT_SEQ_DATA_A_OFFSET:
        ret = dc->disp_spi_init_seq_data_a.reg32;
        break;
    case DISP_SPI_INIT_SEQ_DATA_B_OFFSET:
        ret = dc->disp_spi_init_seq_data_b.reg32;
        break;
    case DISP_SPI_INIT_SEQ_DATA_C_OFFSET:
        ret = dc->disp_spi_init_seq_data_c.reg32;
        break;
    case DISP_SPI_INIT_SEQ_DATA_D_OFFSET:
        ret = dc->disp_spi_init_seq_data_d.reg32;
        break;
    case DISP_DC_MCCIF_FIFOCTRL_OFFSET:
        ret = dc->disp_dc_mccif_fifoctrl.reg32;
        break;
    case DISP_MCCIF_DISPLAY0A_HYST_OFFSET:
        ret = dc->disp_mccif_display0a_hyst.reg32;
        break;
    case DISP_MCCIF_DISPLAY0B_HYST_OFFSET:
        ret = dc->disp_mccif_display0b_hyst.reg32;
        break;
    case DISP_MCCIF_DISPLAY0C_HYST_OFFSET:
        ret = dc->disp_mccif_display0c_hyst.reg32;
        break;
    case DISP_MCCIF_DISPLAY1B_HYST_OFFSET:
        ret = dc->disp_mccif_display1b_hyst.reg32;
        break;
    case DISP_DAC_CRT_CTRL_OFFSET:
        ret = dc->disp_dac_crt_ctrl.reg32;
        break;
    case DISP_DISP_MISC_CONTROL_OFFSET:
        ret = dc->disp_disp_misc_control.reg32;
        break;
    case DISP_BLEND_BACKGROUND_COLOR_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) ret = dc->disp_blend_background_color.reg32;
        break;
    default:
        //g_assert_not_reached();
    }

    return ret;
}

static void dc_write(void *regs, uint32_t offset, uint32_t value)
{
    dc_regs *dc = regs;

    switch (offset) {
    case CMD_GENERAL_INCR_SYNCPT_OFFSET:
    {
        cmd_general_incr_syncpt_t cmd = { .reg32 = value };

        host1x_incr_syncpt(cmd.general_indx);
        break;
    }
    case CMD_GENERAL_INCR_SYNCPT_CNTRL_OFFSET:
//         dc->cmd_general_incr_syncpt_cntrl.reg32 = value;
        break;
    case CMD_GENERAL_INCR_SYNCPT_ERROR_OFFSET:
//         dc->cmd_general_incr_syncpt_error.reg32 = value;
        break;
    case CMD_WIN_A_INCR_SYNCPT_OFFSET:
    {
        cmd_win_a_incr_syncpt_t cmd = { .reg32 = value };

        host1x_incr_syncpt(cmd.win_a_indx);
        break;
    }
    case CMD_WIN_A_INCR_SYNCPT_CNTRL_OFFSET:
//         dc->cmd_win_a_incr_syncpt_cntrl.reg32 = value;
        break;
    case CMD_WIN_A_INCR_SYNCPT_ERROR_OFFSET:
//         dc->cmd_win_a_incr_syncpt_error.reg32 = value;
        break;
    case CMD_WIN_B_INCR_SYNCPT_OFFSET:
    {
        cmd_win_b_incr_syncpt_t cmd = { .reg32 = value };

        host1x_incr_syncpt(cmd.win_b_indx);
        break;
    }
    case CMD_WIN_B_INCR_SYNCPT_CNTRL_OFFSET:
//         dc->cmd_win_b_incr_syncpt_cntrl.reg32 = value;
        break;
    case CMD_WIN_B_INCR_SYNCPT_ERROR_OFFSET:
//         dc->cmd_win_b_incr_syncpt_error.reg32 = value;
        break;
    case CMD_WIN_C_INCR_SYNCPT_OFFSET:
    {
        cmd_win_c_incr_syncpt_t cmd = { .reg32 = value };

        host1x_incr_syncpt(cmd.win_c_indx);
        break;
    }
    case CMD_WIN_C_INCR_SYNCPT_CNTRL_OFFSET:
//         dc->cmd_win_c_incr_syncpt_cntrl.reg32 = value;
        break;
    case CMD_WIN_C_INCR_SYNCPT_ERROR_OFFSET:
//         dc->cmd_win_c_incr_syncpt_error.reg32 = value;
        break;
    case CMD_CONT_SYNCPT_VSYNC_OFFSET:
        dc->cmd_cont_syncpt_vsync.reg32 = value;
        break;
    case CMD_CTXSW_OFFSET:
        dc->cmd_ctxsw.reg32 = value;
        break;
    case CMD_DISPLAY_COMMAND_OPTION0_OFFSET:
        dc->cmd_display_command_option0.reg32 = value;
        break;
    case CMD_DISPLAY_COMMAND_OFFSET:
        dc->cmd_display_command.reg32 = value;
        break;
    case CMD_SIGNAL_RAISE_OFFSET:
        dc->cmd_signal_raise.reg32 = value;
        break;
    case CMD_DISPLAY_POWER_CONTROL_OFFSET:
        dc->cmd_display_power_control.reg32 = value;
        break;
    case CMD_INT_STATUS_OFFSET:
        dc->cmd_int_status.reg32 &= (~value & 0x1DFF9F);
        break;
    case CMD_INT_MASK_OFFSET:
        dc->cmd_int_mask.reg32 = value;
        break;
    case CMD_INT_ENABLE_OFFSET:
        dc->cmd_int_enable.reg32 = value;
        break;
    case CMD_INT_TYPE_OFFSET:
        dc->cmd_int_type.reg32 = value;
        break;
    case CMD_INT_POLARITY_OFFSET:
        dc->cmd_int_polarity.reg32 = value;
        break;
    case CMD_SIGNAL_RAISE1_OFFSET:
        dc->cmd_signal_raise1.reg32 = value;
        break;
    case CMD_SIGNAL_RAISE2_OFFSET:
        dc->cmd_signal_raise2.reg32 = value;
        break;
    case CMD_SIGNAL_RAISE3_OFFSET:
        dc->cmd_signal_raise3.reg32 = value;
        break;
    case CMD_STATE_ACCESS_OFFSET:
        dc->cmd_state_access.reg32 = value;
        break;
    case CMD_STATE_CONTROL_OFFSET:
        dc->cmd_state_control.reg32 = value;
        break;
    case CMD_DISPLAY_WINDOW_HEADER_OFFSET:
        dc->cmd_display_window_header.reg32 = value;
        break;
    case CMD_REG_ACT_CONTROL_OFFSET:
        dc->cmd_reg_act_control.reg32 = value;
        break;
    case COM_CRC_CONTROL_OFFSET:
        dc->com_crc_control.reg32 = value;
        break;
    case COM_PIN_OUTPUT_ENABLE0_OFFSET:
        dc->com_pin_output_enable0.reg32 = value;
        break;
    case COM_PIN_OUTPUT_ENABLE1_OFFSET:
        dc->com_pin_output_enable1.reg32 = value;
        break;
    case COM_PIN_OUTPUT_ENABLE2_OFFSET:
        dc->com_pin_output_enable2.reg32 = value;
        break;
    case COM_PIN_OUTPUT_ENABLE3_OFFSET:
        dc->com_pin_output_enable3.reg32 = value;
        break;
    case COM_PIN_OUTPUT_POLARITY0_OFFSET:
        dc->com_pin_output_polarity0.reg32 = value;
        break;
    case COM_PIN_OUTPUT_POLARITY1_OFFSET:
        dc->com_pin_output_polarity1.reg32 = value;
        break;
    case COM_PIN_OUTPUT_POLARITY2_OFFSET:
        dc->com_pin_output_polarity2.reg32 = value;
        break;
    case COM_PIN_OUTPUT_POLARITY3_OFFSET:
        dc->com_pin_output_polarity3.reg32 = value;
        break;
    case COM_PIN_OUTPUT_DATA0_OFFSET:
        dc->com_pin_output_data0.reg32 = value;
        break;
    case COM_PIN_OUTPUT_DATA1_OFFSET:
        dc->com_pin_output_data1.reg32 = value;
        break;
    case COM_PIN_OUTPUT_DATA2_OFFSET:
        dc->com_pin_output_data2.reg32 = value;
        break;
    case COM_PIN_OUTPUT_DATA3_OFFSET:
        dc->com_pin_output_data3.reg32 = value;
        break;
    case COM_PIN_INPUT_ENABLE0_OFFSET:
        dc->com_pin_input_enable0.reg32 = value;
        break;
    case COM_PIN_INPUT_ENABLE1_OFFSET:
        dc->com_pin_input_enable1.reg32 = value;
        break;
    case COM_PIN_INPUT_ENABLE2_OFFSET:
        dc->com_pin_input_enable2.reg32 = value;
        break;
    case COM_PIN_INPUT_ENABLE3_OFFSET:
        dc->com_pin_input_enable3.reg32 = value;
        break;
    case COM_PIN_OUTPUT_SELECT0_OFFSET:
        dc->com_pin_output_select0.reg32 = value;
        break;
    case COM_PIN_OUTPUT_SELECT1_OFFSET:
        dc->com_pin_output_select1.reg32 = value;
        break;
    case COM_PIN_OUTPUT_SELECT2_OFFSET:
        dc->com_pin_output_select2.reg32 = value;
        break;
    case COM_PIN_OUTPUT_SELECT3_OFFSET:
        dc->com_pin_output_select3.reg32 = value;
        break;
    case COM_PIN_OUTPUT_SELECT4_OFFSET:
        dc->com_pin_output_select4.reg32 = value;
        break;
    case COM_PIN_OUTPUT_SELECT5_OFFSET:
        dc->com_pin_output_select5.reg32 = value;
        break;
    case COM_PIN_OUTPUT_SELECT6_OFFSET:
        dc->com_pin_output_select6.reg32 = value;
        break;
    case COM_PIN_MISC_CONTROL_OFFSET:
        dc->com_pin_misc_control.reg32 = value;
        break;
    case COM_PM0_CONTROL_OFFSET:
        dc->com_pm0_control.reg32 = value;
        break;
    case COM_PM0_DUTY_CYCLE_OFFSET:
        dc->com_pm0_duty_cycle.reg32 = value;
        break;
    case COM_PM1_CONTROL_OFFSET:
        dc->com_pm1_control.reg32 = value;
        break;
    case COM_PM1_DUTY_CYCLE_OFFSET:
        dc->com_pm1_duty_cycle.reg32 = value;
        break;
    case COM_SPI_CONTROL_OFFSET:
        dc->com_spi_control.reg32 = value;
        break;
    case COM_SPI_START_BYTE_OFFSET:
        dc->com_spi_start_byte.reg32 = value;
        break;
    case COM_HSPI_WRITE_DATA_AB_OFFSET:
        dc->com_hspi_write_data_ab.reg32 = value;
        break;
    case COM_HSPI_WRITE_DATA_CD_OFFSET:
        dc->com_hspi_write_data_cd.reg32 = value;
        break;
    case COM_HSPI_CS_DC_OFFSET:
        dc->com_hspi_cs_dc.reg32 = value;
        break;
    case COM_SCRATCH_REGISTER_A_OFFSET:
        dc->com_scratch_register_a.reg32 = value;
        break;
    case COM_SCRATCH_REGISTER_B_OFFSET:
        dc->com_scratch_register_b.reg32 = value;
        break;
    case COM_GPIO_CTRL_OFFSET:
        dc->com_gpio_ctrl.reg32 = value;
        break;
    case COM_GPIO_DEBOUNCE_COUNTER_OFFSET:
        dc->com_gpio_debounce_counter.reg32 = value;
        break;
    case DISP_DISP_SIGNAL_OPTIONS0_OFFSET:
        dc->disp_disp_signal_options0.reg32 = value;
        break;
    case DISP_DISP_SIGNAL_OPTIONS1_OFFSET:
        dc->disp_disp_signal_options1.reg32 = value;
        break;
    case DISP_DISP_WIN_OPTIONS_OFFSET:
        dc->disp_disp_win_options.reg32 = value;
        break;
    case DISP_MEM_HIGH_PRIORITY_OFFSET:
        dc->disp_mem_high_priority.reg32 = value;
        break;
    case DISP_MEM_HIGH_PRIORITY_TIMER_OFFSET:
        dc->disp_mem_high_priority_timer.reg32 = value;
        break;
    case DISP_DISP_TIMING_OPTIONS_OFFSET:
        dc->disp_disp_timing_options.reg32 = value;
        break;
    case DISP_REF_TO_SYNC_OFFSET:
        dc->disp_ref_to_sync.reg32 = value;
        break;
    case DISP_SYNC_WIDTH_OFFSET:
        dc->disp_sync_width.reg32 = value;
        break;
    case DISP_BACK_PORCH_OFFSET:
        dc->disp_back_porch.reg32 = value;
        break;
    case DISP_DISP_ACTIVE_OFFSET:
        dc->disp_disp_active.reg32 = value;
        break;
    case DISP_FRONT_PORCH_OFFSET:
        dc->disp_front_porch.reg32 = value;
        break;
    case DISP_H_PULSE0_CONTROL_OFFSET:
        dc->disp_h_pulse0_control.reg32 = value;
        break;
    case DISP_H_PULSE0_POSITION_A_OFFSET:
        dc->disp_h_pulse0_position_a.reg32 = value;
        break;
    case DISP_H_PULSE0_POSITION_B_OFFSET:
        dc->disp_h_pulse0_position_b.reg32 = value;
        break;
    case DISP_H_PULSE0_POSITION_C_OFFSET:
        dc->disp_h_pulse0_position_c.reg32 = value;
        break;
    case DISP_H_PULSE0_POSITION_D_OFFSET:
        dc->disp_h_pulse0_position_d.reg32 = value;
        break;
    case DISP_H_PULSE1_CONTROL_OFFSET:
        dc->disp_h_pulse1_control.reg32 = value;
        break;
    case DISP_H_PULSE1_POSITION_A_OFFSET:
        dc->disp_h_pulse1_position_a.reg32 = value;
        break;
    case DISP_H_PULSE1_POSITION_B_OFFSET:
        dc->disp_h_pulse1_position_b.reg32 = value;
        break;
    case DISP_H_PULSE1_POSITION_C_OFFSET:
        dc->disp_h_pulse1_position_c.reg32 = value;
        break;
    case DISP_H_PULSE1_POSITION_D_OFFSET:
        dc->disp_h_pulse1_position_d.reg32 = value;
        break;
    case DISP_H_PULSE2_CONTROL_OFFSET:
        dc->disp_h_pulse2_control.reg32 = value;
        break;
    case DISP_H_PULSE2_POSITION_A_OFFSET:
        dc->disp_h_pulse2_position_a.reg32 = value;
        break;
    case DISP_H_PULSE2_POSITION_B_OFFSET:
        dc->disp_h_pulse2_position_b.reg32 = value;
        break;
    case DISP_H_PULSE2_POSITION_C_OFFSET:
        dc->disp_h_pulse2_position_c.reg32 = value;
        break;
    case DISP_H_PULSE2_POSITION_D_OFFSET:
        dc->disp_h_pulse2_position_d.reg32 = value;
        break;
    case DISP_V_PULSE0_CONTROL_OFFSET:
        dc->disp_v_pulse0_control.reg32 = value;
        break;
    case DISP_V_PULSE0_POSITION_A_OFFSET:
        dc->disp_v_pulse0_position_a.reg32 = value;
        break;
    case DISP_V_PULSE0_POSITION_B_OFFSET:
        dc->disp_v_pulse0_position_b.reg32 = value;
        break;
    case DISP_V_PULSE0_POSITION_C_OFFSET:
        dc->disp_v_pulse0_position_c.reg32 = value;
        break;
    case DISP_V_PULSE1_CONTROL_OFFSET:
        dc->disp_v_pulse1_control.reg32 = value;
        break;
    case DISP_V_PULSE1_POSITION_A_OFFSET:
        dc->disp_v_pulse1_position_a.reg32 = value;
        break;
    case DISP_V_PULSE1_POSITION_B_OFFSET:
        dc->disp_v_pulse1_position_b.reg32 = value;
        break;
    case DISP_V_PULSE1_POSITION_C_OFFSET:
        dc->disp_v_pulse1_position_c.reg32 = value;
        break;
    case DISP_V_PULSE2_CONTROL_OFFSET:
        dc->disp_v_pulse2_control.reg32 = value;
        break;
    case DISP_V_PULSE2_POSITION_A_OFFSET:
        dc->disp_v_pulse2_position_a.reg32 = value;
        break;
    case DISP_V_PULSE3_CONTROL_OFFSET:
        dc->disp_v_pulse3_control.reg32 = value;
        break;
    case DISP_V_PULSE3_POSITION_A_OFFSET:
        dc->disp_v_pulse3_position_a.reg32 = value;
        break;
    case DISP_M0_CONTROL_OFFSET:
        dc->disp_m0_control.reg32 = value;
        break;
    case DISP_M1_CONTROL_OFFSET:
        dc->disp_m1_control.reg32 = value;
        break;
    case DISP_DI_CONTROL_OFFSET:
        dc->disp_di_control.reg32 = value;
        break;
    case DISP_PP_CONTROL_OFFSET:
        dc->disp_pp_control.reg32 = value;
        break;
    case DISP_PP_SELECT_A_OFFSET:
        dc->disp_pp_select_a.reg32 = value;
        break;
    case DISP_PP_SELECT_B_OFFSET:
        dc->disp_pp_select_b.reg32 = value;
        break;
    case DISP_PP_SELECT_C_OFFSET:
        dc->disp_pp_select_c.reg32 = value;
        break;
    case DISP_PP_SELECT_D_OFFSET:
        dc->disp_pp_select_d.reg32 = value;
        break;
    case DISP_DISP_CLOCK_CONTROL_OFFSET:
        dc->disp_disp_clock_control.reg32 = value;
        break;
    case DISP_DISP_INTERFACE_CONTROL_OFFSET:
        dc->disp_disp_interface_control.reg32 = value;
        break;
    case DISP_DISP_COLOR_CONTROL_OFFSET:
        dc->disp_disp_color_control.reg32 = value;
        break;
    case DISP_SHIFT_CLOCK_OPTIONS_OFFSET:
        dc->disp_shift_clock_options.reg32 = value;
        break;
    case DISP_DATA_ENABLE_OPTIONS_OFFSET:
        dc->disp_data_enable_options.reg32 = value;
        break;
    case DISP_SERIAL_INTERFACE_OPTIONS_OFFSET:
        dc->disp_serial_interface_options.reg32 = value;
        break;
    case DISP_LCD_SPI_OPTIONS_OFFSET:
        dc->disp_lcd_spi_options.reg32 = value;
        break;
    case DISP_BORDER_COLOR_OFFSET:
        dc->disp_border_color.reg32 = value;
        break;
    case DISP_COLOR_KEY0_LOWER_OFFSET:
        dc->disp_color_key0_lower.reg32 = value;
        break;
    case DISP_COLOR_KEY0_UPPER_OFFSET:
        dc->disp_color_key0_upper.reg32 = value;
        break;
    case DISP_COLOR_KEY1_LOWER_OFFSET:
        dc->disp_color_key1_lower.reg32 = value;
        break;
    case DISP_COLOR_KEY1_UPPER_OFFSET:
        dc->disp_color_key1_upper.reg32 = value;
        break;
    case DISP_CURSOR_FOREGROUND_OFFSET:
        dc->disp_cursor_foreground.reg32 = value;
        break;
    case DISP_CURSOR_BACKGROUND_OFFSET:
        dc->disp_cursor_background.reg32 = value;
        break;
    case DISP_CURSOR_START_ADDR_OFFSET:
    case DISP_CURSOR_START_ADDR_NS_OFFSET:
        dc->disp_cursor_start_addr.reg32 = value;
        break;
    case DISP_CURSOR_POSITION_OFFSET:
    case DISP_CURSOR_POSITION_NS_OFFSET:
        dc->disp_cursor_position.reg32 = value;
        break;
    case DISP_INIT_SEQ_CONTROL_OFFSET:
        dc->disp_init_seq_control.reg32 = value;
        break;
    case DISP_SPI_INIT_SEQ_DATA_A_OFFSET:
        dc->disp_spi_init_seq_data_a.reg32 = value;
        break;
    case DISP_SPI_INIT_SEQ_DATA_B_OFFSET:
        dc->disp_spi_init_seq_data_b.reg32 = value;
        break;
    case DISP_SPI_INIT_SEQ_DATA_C_OFFSET:
        dc->disp_spi_init_seq_data_c.reg32 = value;
        break;
    case DISP_SPI_INIT_SEQ_DATA_D_OFFSET:
        dc->disp_spi_init_seq_data_d.reg32 = value;
        break;
    case DISP_DC_MCCIF_FIFOCTRL_OFFSET:
        dc->disp_dc_mccif_fifoctrl.reg32 = value;
        break;
    case DISP_MCCIF_DISPLAY0A_HYST_OFFSET:
        dc->disp_mccif_display0a_hyst.reg32 = value;
        break;
    case DISP_MCCIF_DISPLAY0B_HYST_OFFSET:
        dc->disp_mccif_display0b_hyst.reg32 = value;
        break;
    case DISP_MCCIF_DISPLAY0C_HYST_OFFSET:
        dc->disp_mccif_display0c_hyst.reg32 = value;
        break;
    case DISP_MCCIF_DISPLAY1B_HYST_OFFSET:
        dc->disp_mccif_display1b_hyst.reg32 = value;
        break;
    case DISP_DAC_CRT_CTRL_OFFSET:
        dc->disp_dac_crt_ctrl.reg32 = value;
        break;
    case DISP_DISP_MISC_CONTROL_OFFSET:
        dc->disp_disp_misc_control.reg32 = value;
        break;
    case DISP_BLEND_BACKGROUND_COLOR_OFFSET:
        if (tegra_board >= TEGRAX1_BOARD) dc->disp_blend_background_color.reg32 = value;
        break;
    default:
        //g_assert_not_reached();
    }
}

static void dc_reset(void *regs)
{
    dc_regs *dc = regs;

    dc->cmd_cont_syncpt_vsync.reg32 = CMD_CONT_SYNCPT_VSYNC_RESET;
    dc->cmd_ctxsw.reg32 = CMD_CTXSW_RESET;
    dc->cmd_display_command_option0.reg32 = CMD_DISPLAY_COMMAND_OPTION0_RESET;
    dc->cmd_display_command.reg32 = CMD_DISPLAY_COMMAND_RESET;
    dc->cmd_signal_raise.reg32 = CMD_SIGNAL_RAISE_RESET;
    dc->cmd_display_power_control.reg32 = CMD_DISPLAY_POWER_CONTROL_RESET;
    dc->cmd_int_status.reg32 = CMD_INT_STATUS_RESET;
    dc->cmd_int_mask.reg32 = CMD_INT_MASK_RESET;
    dc->cmd_int_enable.reg32 = CMD_INT_ENABLE_RESET;
    dc->cmd_int_type.reg32 = CMD_INT_TYPE_RESET;
    dc->cmd_int_polarity.reg32 = CMD_INT_POLARITY_RESET;
    dc->cmd_signal_raise1.reg32 = CMD_SIGNAL_RAISE1_RESET;
    dc->cmd_signal_raise2.reg32 = CMD_SIGNAL_RAISE2_RESET;
    dc->cmd_signal_raise3.reg32 = CMD_SIGNAL_RAISE3_RESET;
    dc->cmd_state_access.reg32 = CMD_STATE_ACCESS_RESET;
    dc->cmd_state_control.reg32 = CMD_STATE_CONTROL_RESET;
    dc->cmd_display_window_header.reg32 = CMD_DISPLAY_WINDOW_HEADER_RESET;
    dc->cmd_reg_act_control.reg32 = CMD_REG_ACT_CONTROL_RESET;
    dc->com_crc_control.reg32 = COM_CRC_CONTROL_RESET;
    dc->com_crc_checksum.reg32 = COM_CRC_CHECKSUM_RESET;
    dc->com_pin_output_enable0.reg32 = COM_PIN_OUTPUT_ENABLE0_RESET;
    dc->com_pin_output_enable1.reg32 = COM_PIN_OUTPUT_ENABLE1_RESET;
    dc->com_pin_output_enable2.reg32 = COM_PIN_OUTPUT_ENABLE2_RESET;
    dc->com_pin_output_enable3.reg32 = COM_PIN_OUTPUT_ENABLE3_RESET;
    dc->com_pin_output_polarity0.reg32 = COM_PIN_OUTPUT_POLARITY0_RESET;
    dc->com_pin_output_polarity1.reg32 = COM_PIN_OUTPUT_POLARITY1_RESET;
    dc->com_pin_output_polarity2.reg32 = COM_PIN_OUTPUT_POLARITY2_RESET;
    dc->com_pin_output_polarity3.reg32 = COM_PIN_OUTPUT_POLARITY3_RESET;
    dc->com_pin_output_data0.reg32 = COM_PIN_OUTPUT_DATA0_RESET;
    dc->com_pin_output_data1.reg32 = COM_PIN_OUTPUT_DATA1_RESET;
    dc->com_pin_output_data2.reg32 = COM_PIN_OUTPUT_DATA2_RESET;
    dc->com_pin_output_data3.reg32 = COM_PIN_OUTPUT_DATA3_RESET;
    dc->com_pin_input_enable0.reg32 = COM_PIN_INPUT_ENABLE0_RESET;
    dc->com_pin_input_enable1.reg32 = COM_PIN_INPUT_ENABLE1_RESET;
    dc->com_pin_input_enable2.reg32 = COM_PIN_INPUT_ENABLE2_RESET;
    dc->com_pin_input_enable3.reg32 = COM_PIN_INPUT_ENABLE3_RESET;
    dc->com_pin_input_data0.reg32 = COM_PIN_INPUT_DATA0_RESET;
    dc->com_pin_input_data1.reg32 = COM_PIN_INPUT_DATA1_RESET;
    dc->com_pin_output_select0.reg32 = COM_PIN_OUTPUT_SELECT0_RESET;
    dc->com_pin_output_select1.reg32 = COM_PIN_OUTPUT_SELECT1_RESET;
    dc->com_pin_output_select2.reg32 = COM_PIN_OUTPUT_SELECT2_RESET;
    dc->com_pin_output_select3.reg32 = COM_PIN_OUTPUT_SELECT3_RESET;
    dc->com_pin_output_select4.reg32 = COM_PIN_OUTPUT_SELECT4_RESET;
    dc->com_pin_output_select5.reg32 = COM_PIN_OUTPUT_SELECT5_RESET;
    dc->com_pin_output_select6.reg32 = COM_PIN_OUTPUT_SELECT6_RESET;
    dc->com_pin_misc_control.reg32 = COM_PIN_MISC_CONTROL_RESET;
    dc->com_pm0_control.reg32 = COM_PM0_CONTROL_RESET;
    dc->com_pm0_duty_cycle.reg32 = COM_PM0_DUTY_CYCLE_RESET;
    dc->com_pm1_control.reg32 = COM_PM1_CONTROL_RESET;
    dc->com_pm1_duty_cycle.reg32 = COM_PM1_DUTY_CYCLE_RESET;
    dc->com_spi_control.reg32 = COM_SPI_CONTROL_RESET;
    dc->com_spi_start_byte.reg32 = COM_SPI_START_BYTE_RESET;
    dc->com_hspi_write_data_ab.reg32 = COM_HSPI_WRITE_DATA_AB_RESET;
    dc->com_hspi_write_data_cd.reg32 = COM_HSPI_WRITE_DATA_CD_RESET;
    dc->com_hspi_cs_dc.reg32 = COM_HSPI_CS_DC_RESET;
    dc->com_scratch_register_a.reg32 = COM_SCRATCH_REGISTER_A_RESET;
    dc->com_scratch_register_b.reg32 = COM_SCRATCH_REGISTER_B_RESET;
    dc->com_gpio_ctrl.reg32 = COM_GPIO_CTRL_RESET;
    dc->com_gpio_debounce_counter.reg32 = COM_GPIO_DEBOUNCE_COUNTER_RESET;
    dc->com_crc_checksum_latched.reg32 = COM_CRC_CHECKSUM_LATCHED_RESET;
    dc->disp_disp_signal_options0.reg32 = DISP_DISP_SIGNAL_OPTIONS0_RESET;
    dc->disp_disp_signal_options1.reg32 = DISP_DISP_SIGNAL_OPTIONS1_RESET;
    dc->disp_disp_win_options.reg32 = DISP_DISP_WIN_OPTIONS_RESET;
    dc->disp_mem_high_priority.reg32 = DISP_MEM_HIGH_PRIORITY_RESET;
    dc->disp_mem_high_priority_timer.reg32 = DISP_MEM_HIGH_PRIORITY_TIMER_RESET;
    dc->disp_disp_timing_options.reg32 = DISP_DISP_TIMING_OPTIONS_RESET;
    dc->disp_ref_to_sync.reg32 = DISP_REF_TO_SYNC_RESET;
    dc->disp_sync_width.reg32 = DISP_SYNC_WIDTH_RESET;
    dc->disp_back_porch.reg32 = DISP_BACK_PORCH_RESET;
    dc->disp_disp_active.reg32 = DISP_DISP_ACTIVE_RESET;
    dc->disp_front_porch.reg32 = DISP_FRONT_PORCH_RESET;
    dc->disp_h_pulse0_control.reg32 = DISP_H_PULSE0_CONTROL_RESET;
    dc->disp_h_pulse0_position_a.reg32 = DISP_H_PULSE0_POSITION_A_RESET;
    dc->disp_h_pulse0_position_b.reg32 = DISP_H_PULSE0_POSITION_B_RESET;
    dc->disp_h_pulse0_position_c.reg32 = DISP_H_PULSE0_POSITION_C_RESET;
    dc->disp_h_pulse0_position_d.reg32 = DISP_H_PULSE0_POSITION_D_RESET;
    dc->disp_h_pulse1_control.reg32 = DISP_H_PULSE1_CONTROL_RESET;
    dc->disp_h_pulse1_position_a.reg32 = DISP_H_PULSE1_POSITION_A_RESET;
    dc->disp_h_pulse1_position_b.reg32 = DISP_H_PULSE1_POSITION_B_RESET;
    dc->disp_h_pulse1_position_c.reg32 = DISP_H_PULSE1_POSITION_C_RESET;
    dc->disp_h_pulse1_position_d.reg32 = DISP_H_PULSE1_POSITION_D_RESET;
    dc->disp_h_pulse2_control.reg32 = DISP_H_PULSE2_CONTROL_RESET;
    dc->disp_h_pulse2_position_a.reg32 = DISP_H_PULSE2_POSITION_A_RESET;
    dc->disp_h_pulse2_position_b.reg32 = DISP_H_PULSE2_POSITION_B_RESET;
    dc->disp_h_pulse2_position_c.reg32 = DISP_H_PULSE2_POSITION_C_RESET;
    dc->disp_h_pulse2_position_d.reg32 = DISP_H_PULSE2_POSITION_D_RESET;
    dc->disp_v_pulse0_control.reg32 = DISP_V_PULSE0_CONTROL_RESET;
    dc->disp_v_pulse0_position_a.reg32 = DISP_V_PULSE0_POSITION_A_RESET;
    dc->disp_v_pulse0_position_b.reg32 = DISP_V_PULSE0_POSITION_B_RESET;
    dc->disp_v_pulse0_position_c.reg32 = DISP_V_PULSE0_POSITION_C_RESET;
    dc->disp_v_pulse1_control.reg32 = DISP_V_PULSE1_CONTROL_RESET;
    dc->disp_v_pulse1_position_a.reg32 = DISP_V_PULSE1_POSITION_A_RESET;
    dc->disp_v_pulse1_position_b.reg32 = DISP_V_PULSE1_POSITION_B_RESET;
    dc->disp_v_pulse1_position_c.reg32 = DISP_V_PULSE1_POSITION_C_RESET;
    dc->disp_v_pulse2_control.reg32 = DISP_V_PULSE2_CONTROL_RESET;
    dc->disp_v_pulse2_position_a.reg32 = DISP_V_PULSE2_POSITION_A_RESET;
    dc->disp_v_pulse3_control.reg32 = DISP_V_PULSE3_CONTROL_RESET;
    dc->disp_v_pulse3_position_a.reg32 = DISP_V_PULSE3_POSITION_A_RESET;
    dc->disp_m0_control.reg32 = DISP_M0_CONTROL_RESET;
    dc->disp_m1_control.reg32 = DISP_M1_CONTROL_RESET;
    dc->disp_di_control.reg32 = DISP_DI_CONTROL_RESET;
    dc->disp_pp_control.reg32 = DISP_PP_CONTROL_RESET;
    dc->disp_pp_select_a.reg32 = DISP_PP_SELECT_A_RESET;
    dc->disp_pp_select_b.reg32 = DISP_PP_SELECT_B_RESET;
    dc->disp_pp_select_c.reg32 = DISP_PP_SELECT_C_RESET;
    dc->disp_pp_select_d.reg32 = DISP_PP_SELECT_D_RESET;
    dc->disp_disp_clock_control.reg32 = DISP_DISP_CLOCK_CONTROL_RESET;
    dc->disp_disp_interface_control.reg32 = DISP_DISP_INTERFACE_CONTROL_RESET;
    dc->disp_disp_color_control.reg32 = DISP_DISP_COLOR_CONTROL_RESET;
    dc->disp_shift_clock_options.reg32 = DISP_SHIFT_CLOCK_OPTIONS_RESET;
    dc->disp_data_enable_options.reg32 = DISP_DATA_ENABLE_OPTIONS_RESET;
    dc->disp_serial_interface_options.reg32 = DISP_SERIAL_INTERFACE_OPTIONS_RESET;
    dc->disp_lcd_spi_options.reg32 = DISP_LCD_SPI_OPTIONS_RESET;
    dc->disp_border_color.reg32 = DISP_BORDER_COLOR_RESET;
    dc->disp_color_key0_lower.reg32 = DISP_COLOR_KEY0_LOWER_RESET;
    dc->disp_color_key0_upper.reg32 = DISP_COLOR_KEY0_UPPER_RESET;
    dc->disp_color_key1_lower.reg32 = DISP_COLOR_KEY1_LOWER_RESET;
    dc->disp_color_key1_upper.reg32 = DISP_COLOR_KEY1_UPPER_RESET;
    dc->disp_cursor_foreground.reg32 = DISP_CURSOR_FOREGROUND_RESET;
    dc->disp_cursor_background.reg32 = DISP_CURSOR_BACKGROUND_RESET;
    dc->disp_cursor_start_addr.reg32 = DISP_CURSOR_START_ADDR_RESET;
    dc->disp_cursor_position.reg32 = DISP_CURSOR_POSITION_RESET;
    dc->disp_init_seq_control.reg32 = DISP_INIT_SEQ_CONTROL_RESET;
    dc->disp_spi_init_seq_data_a.reg32 = DISP_SPI_INIT_SEQ_DATA_A_RESET;
    dc->disp_spi_init_seq_data_b.reg32 = DISP_SPI_INIT_SEQ_DATA_B_RESET;
    dc->disp_spi_init_seq_data_c.reg32 = DISP_SPI_INIT_SEQ_DATA_C_RESET;
    dc->disp_spi_init_seq_data_d.reg32 = DISP_SPI_INIT_SEQ_DATA_D_RESET;
    dc->disp_dc_mccif_fifoctrl.reg32 = DISP_DC_MCCIF_FIFOCTRL_RESET;
    dc->disp_mccif_display0a_hyst.reg32 = DISP_MCCIF_DISPLAY0A_HYST_RESET;
    dc->disp_mccif_display0b_hyst.reg32 = DISP_MCCIF_DISPLAY0B_HYST_RESET;
    dc->disp_mccif_display0c_hyst.reg32 = DISP_MCCIF_DISPLAY0C_HYST_RESET;
    dc->disp_mccif_display1b_hyst.reg32 = DISP_MCCIF_DISPLAY1B_HYST_RESET;
    dc->disp_dac_crt_ctrl.reg32 = DISP_DAC_CRT_CTRL_RESET;
    dc->disp_disp_misc_control.reg32 = DISP_DISP_MISC_CONTROL_RESET;
}

regs_io_handler dc_handler = {
    .read  = dc_read,
    .write = dc_write,
    .reset = dc_reset,
    .begin = 0x0,
    .end   = 0x4FF,
};
