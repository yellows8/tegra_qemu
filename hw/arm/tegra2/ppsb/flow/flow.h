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

/* Autogenerated from TRM v02p */

#ifndef TEGRA_FLOW_CTRL_H
#define TEGRA_FLOW_CTRL_H

#define HALT_CPU0_EVENTS_OFFSET 0x0
#define HALT_CPU_EVENTS_RESET  0x00000000
typedef union halt_events_u {
    struct {
        unsigned int zero:8;
        unsigned int fiq_0:1;
        unsigned int fiq_1:1;
        unsigned int irq_0:1;
        unsigned int irq_1:1;
        unsigned int ibf:1;
        unsigned int ibe:1;
        unsigned int obf:1;
        unsigned int obe:1;
        unsigned int xrq_a:1;
        unsigned int xrq_b:1;
        unsigned int xrq_c:1;
        unsigned int xrq_d:1;
        unsigned int smp30:1;
        unsigned int smp31:1;
        unsigned int x_rdy:1;
        unsigned int sec:1;
        unsigned int msec:1;
        unsigned int usec:1;
        unsigned int x32k:1;
        unsigned int sclk:1;
        unsigned int jtag:1;
        unsigned int mode:3;                /* 0 = FLOW_MODE_NONE 1 = FLOW_MODE_RUN_AND_INT 2 = FLOW_MODE_WAITEVENT 3 = FLOW_MODE_WAITEVENT_AND_INT 4 = FLOW_MODE_STOP_UNTIL_IRQ 5 = FLOW_MODE_STOP_UNTIL_IRQ_AND_INT 6 = FLOW_MODE_STOP_UNTIL_EVENT_AND_IRQ 2 = FLOW_MODE_STOP 3 = FLOW_MODE_STOP_AND_INT 4 = FLOW_MODE_STOP_UNTIL_INT 5 = FLOW_MODE_STOP_UNTIL_INT_AND_INT 6 = FLOW_MODE_STOP_OR_INT */
    };

    uint32_t reg32;
} halt_events_t;

#define HALT_COP_EVENTS_OFFSET 0x4
#define HALT_COP_EVENTS_RESET  0x00000000

#define CPU0_CSR_OFFSET 0x8
#define CPU_CSR_RESET   0x00000000
#define CPU_CSR_WRMASK  0xF0043FFF
typedef union csr_u {
    struct {
        unsigned int enable:1;
        unsigned int event_enable:1;
        unsigned int undefined_bits_2_3:2;
        unsigned int wait_wfe_bitmap:2;
        unsigned int undefined_bits_6_13:8;
        unsigned int event_flag:1;
        unsigned int intr_flag:1;           /* TRUE when Interrupt is Active -- Write-1-to-Clear */
        unsigned int pwr_off_sts:1;
        unsigned int f2c_mpcore_rst:1;
        unsigned int undefined_bit_18:1;
        unsigned int f2p_req:1;
        unsigned int f2p_pwrup:1;
        unsigned int p2f_ack:1;
        unsigned int halt:1;
        unsigned int wait_event:1;
        unsigned int pwr_state:4;
        unsigned int undefined_bits_28_31:4;
    };

    uint32_t reg32;
} csr_t;

#define COP_CSR_OFFSET 0xC
#define COP_CSR_RESET  0x00000000
#define COP_CSR_WRMASK 0x00008000

#define XRQ_EVENTS_OFFSET 0x10
#define XRQ_EVENTS_RESET  0x00000000
typedef union xrq_events_u {
    struct {
        unsigned int xrq_a7_xrq_a0:8;       /* Setting a bit to 1 enables event triggering for the corresponding bit in GPIO port A. The assertion level is determined by GPIO_INT.LVL.A. If more than one XRQ.A bit is set, the events are ORed together. The resultant event is enabled by setting the XRQ.A bit in the HALT_CPU.EVENTS or HALT_COP.EVENTS registers */
        unsigned int xrq_b7_xrq_b0:8;       /* Setting a bit to 1 enables event triggering for the corresponding bit in GPIO port B. The assertion level is determined by GPIO_INT.LVL.B. If more than one XRQ.B bit is set, the events are ORed together. The resultant event is enabled by setting the XRQ.B bit in the HALT_CPU.EVENTS or HALT_COP.EVENTS registers */
        unsigned int xrq_c7_xrq_c0:8;       /* Setting a bit to 1 enables event triggering for the corresponding bit in GPIO port C. The assertion level is determined by GPIO_INT.LVL.C. If more than one XRQ.C bit is set, the events are ORed together. The resultant event is enabled by setting the XRQ.C bit in the HALT_CPU.EVENTS or HALT_COP.EVENTS registers */
        unsigned int xrq_d7_xrq_d0:8;       /* Setting a bit to 1 enables event triggering for the corresponding bit in GPIO port D. The assertion level is determined by GPIO_INT.LVL.D. If more than one XRQ.D bit is set, the events are ORed together. The resultant event is enabled by setting the XRQ.D bit in the HALT_CPU.EVENTS or HALT_COP.EVENTS registers */
    };

    uint32_t reg32;
} xrq_events_t;

#define HALT_CPU1_EVENTS_OFFSET 0x14
#define CPU1_CSR_OFFSET 0x18

#define RAM_REPAIR_OFFSET 0x40
#define RAM_REPAIR_RESET  0x00000004
typedef union ram_repair_u {
    uint32_t reg32;
} ram_repair_t;

#endif // TEGRA_FLOW_CTRL_H
