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
#include "hw/arm/boot.h"
#include "hw/cpu/a9mpcore.h"
#include "hw/intc/gic_internal.h"
#include "hw/ptimer.h"
#include "hw/sysbus.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "sysemu/sysemu.h"

#include "clk_rst.h"
#include "devices.h"
#include "flow.h"
#include "ictlr.h"
#include "iomap.h"
#include "irqs.h"
#include "tegra_cpu.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_FLOW_CTRL "tegra.flow"
#define TEGRA_FLOW_CTRL(obj) OBJECT_CHECK(tegra_flow, (obj), TYPE_TEGRA_FLOW_CTRL)
#define DEFINE_REG32(reg) reg##_t reg
#define WR_MASKED(r, d, m)  r = (r & ~m##_WRMASK) | (d & m##_WRMASK)

/* No flow control */
#define FLOW_MODE_NONE                      0
/* Keep running but generate interrupt when event conditions met */
#define FLOW_MODE_RUN_AND_INT               1
/* Stop running until event conditions met */
#define FLOW_MODE_WAITEVENT                 2
/* Same as FLOW_MODE_WAITEVENT but generate an interrupt when eventd */
#define FLOW_MODE_WAITEVENT_AND_INT         3
/* Stop until an interrupt controller interrupt occurs */
#define FLOW_MODE_STOP_UNTIL_IRQ            4
/* Same as FLOW_MODE_STOP_UNTIL_IRQ but generate another interrupt when eventd */
#define FLOW_MODE_STOP_UNTIL_IRQ_AND_INT    5
/* Stop until event conditions met AND an interrupt controller interrupt occurs */
#define FLOW_MODE_STOP_UNTIL_EVENT_AND_IRQ  6

#define INTERRUPT   1
#define WAITEVENT   2
#define WAIT_IRQ    4
#define STOP        6

typedef struct tegra_flow_state {
    SysBusDevice parent_obj;

    qemu_irq irq_cpu_event;
    qemu_irq irq_cop_event;

    MemoryRegion iomem;
    ptimer_state *ptimer[TEGRA2_NCPUS];
    DEFINE_REG32(halt_events)[TEGRA2_NCPUS];
    DEFINE_REG32(csr)[TEGRA2_NCPUS];
    DEFINE_REG32(xrq_events);
    uint8_t cop_stalled;
} tegra_flow;

static const VMStateDescription vmstate_tegra_flow = {
    .name = "tegra.flow",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_PTIMER_ARRAY(ptimer, tegra_flow, TEGRA2_NCPUS),
        VMSTATE_ARRAY(halt_events, tegra_flow, TEGRA2_NCPUS, 0,
                      vmstate_info_uint32, halt_events_t),
        VMSTATE_ARRAY(csr, tegra_flow, TEGRA2_NCPUS, 0,
                      vmstate_info_uint32, csr_t),
        VMSTATE_UINT32(xrq_events.reg32, tegra_flow),
        VMSTATE_UINT8(cop_stalled, tegra_flow),
        VMSTATE_END_OF_LIST()
    }
};

typedef struct tegra_flow_timer_arg {
    tegra_flow *s;
    int cpu_id;
} tegra_flow_timer_arg;

static int tegra_flow_have_pending_irq(int cpu_id)
{
    A9MPPrivState *a9mpcore = A9MPCORE_PRIV(tegra_a9mpcore_dev);
    GICState *s = &a9mpcore->gic;
    int i;

    if (tegra_ictlr_is_irq_pending_on_cpu(cpu_id)) {
        return 1;
    }

    if (cpu_id == TEGRA2_COP || tegra_cpu_is_powergated(cpu_id)) {
        return 0;
    }

    for (i = INT_CPU_IRQS_NR; i < INT_GIC_NR; i++) {
        if (!GIC_DIST_TEST_ENABLED(i, 1 << cpu_id))
            continue;

        if (gic_test_pending(s, i, 1 << cpu_id)) {
            TPRINT("tegra_flow: irq %d pending\n", i);
            return 1;
        }
    }

    return 0;
}

static int tegra_flow_arm_event(tegra_flow *s, int cpu_id, int wait)
{
    int ev_cnt = s->halt_events[cpu_id].zero;
    int unimplemented = 1;

    if (s->csr[cpu_id].event_flag) {
        if (cpu_id != TEGRA2_COP) {
            goto fired;
        }
    }

    if (s->halt_events[cpu_id].irq_0) {
        int sibling = tegra_sibling_cpu(cpu_id);

        if (tegra_flow_have_pending_irq(cpu_id)) {
            goto fired;
        }

        if (sibling != cpu_id) {
            if (tegra_flow_have_pending_irq(sibling)) {
                goto fired;
            }
        }

        unimplemented = 0;
    }

    if (s->halt_events[cpu_id].irq_1) {
        if (tegra_flow_have_pending_irq(TEGRA2_COP)) {
            goto fired;
        }

        unimplemented = 0;
    }

    if (!ev_cnt) {
        goto armed;
    }

    if (s->halt_events[cpu_id].sclk && tegra_clk_enabled(TEGRA20_CLK_SCLK)) {
        s->halt_events[cpu_id].zero = 0;
        goto armed;
    }

    if (s->halt_events[cpu_id].x32k && tegra_clk_enabled(TEGRA20_CLK_CLK_32K)) {
        s->halt_events[cpu_id].zero = 0;
        goto armed;
    }

    if (s->halt_events[cpu_id].usec) {
        TPRINT("tegra_flow: cpu %d armed %d usec delay\n", cpu_id, ev_cnt);
        ptimer_transaction_begin(s->ptimer[cpu_id]);
        ptimer_set_limit(s->ptimer[cpu_id], 1, 1);
        ptimer_run(s->ptimer[cpu_id], 0);
        ptimer_transaction_commit(s->ptimer[cpu_id]);
        goto armed;
    }

    if (s->halt_events[cpu_id].msec) {
        TPRINT("tegra_flow: cpu %d armed %d msec delay\n", cpu_id, ev_cnt);
        ptimer_transaction_begin(s->ptimer[cpu_id]);
        ptimer_set_limit(s->ptimer[cpu_id], 1000, 1);
        ptimer_run(s->ptimer[cpu_id], 0);
        ptimer_transaction_commit(s->ptimer[cpu_id]);
        goto armed;
    }

    if (s->halt_events[cpu_id].sec) {
        TPRINT("tegra_flow: cpu %d armed %d sec delay\n", cpu_id, ev_cnt);
        ptimer_transaction_begin(s->ptimer[cpu_id]);
        ptimer_set_limit(s->ptimer[cpu_id], 1000000, 1);
        ptimer_run(s->ptimer[cpu_id], 0);
        ptimer_transaction_commit(s->ptimer[cpu_id]);
        goto armed;
    }

    if (unimplemented) {
        TPRINT("WARNING!!! Unimplemented WAITEVENT 0x%08X armed! cpu %d",
               s->halt_events[cpu_id].reg32, cpu_id);
    }

armed:
    s->csr[cpu_id].wait_event = !!wait;

    return 1;

fired:
    s->csr[cpu_id].event_flag = 1;

    return 0;
}

static void tegra_flow_gen_interrupt(tegra_flow *s, int cpu_id)
{
    s->csr[cpu_id].intr_flag = 1;
//     s->halt_events[cpu_id].mode &= ~INTERRUPT;

    /* ??? TODO: Check if IRQ is deprecated on Tegra2.  */
    if (cpu_id == TEGRA2_COP) {
        TRACE_IRQ_RAISE(s->iomem.addr, s->irq_cop_event);
    } else {
        TRACE_IRQ_RAISE(s->iomem.addr, s->irq_cpu_event);
    }
}

static void __tegra_flow_clear_waitevent(tegra_flow *s, int cpu_id, int clr_wait)
{
    s->csr[cpu_id].event_flag = 1;

    ptimer_stop(s->ptimer[cpu_id]);

    if (s->halt_events[cpu_id].mode & INTERRUPT) {
        tegra_flow_gen_interrupt(s, cpu_id);
    }

    if (cpu_id == TEGRA2_COP && s->cop_stalled) {
        s->csr[TEGRA2_COP].wait_event = 0;
        s->cop_stalled = 0;
        return;
    }

    if (clr_wait) {
        s->halt_events[cpu_id].mode &= ~WAITEVENT;
        s->csr[cpu_id].wait_event = 0;
    }
}

static void tegra_flow_clear_waitevent(tegra_flow *s, int cpu_id, int clr_wait)
{
    ptimer_transaction_begin(s->ptimer[cpu_id]);
    __tegra_flow_clear_waitevent(s, cpu_id, clr_wait);
    ptimer_transaction_commit(s->ptimer[cpu_id]);
}

static uint64_t tegra_flow_priv_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    tegra_flow *s = opaque;
    uint64_t ret = 0;

    switch (offset) {
    case HALT_CPU0_EVENTS_OFFSET:
        ret = s->halt_events[TEGRA2_A9_CORE0].reg32;
        break;
    case HALT_CPU1_EVENTS_OFFSET:
        ret = s->halt_events[TEGRA2_A9_CORE1].reg32;
        break;
    case HALT_COP_EVENTS_OFFSET:
        ret = s->halt_events[TEGRA2_COP].reg32;
        break;

    case CPU0_CSR_OFFSET:
        s->csr[TEGRA2_A9_CORE0].halt = tegra_cpu_halted(TEGRA2_A9_CORE0);
        s->csr[TEGRA2_A9_CORE0].pwr_off_sts = tegra_cpu_is_powergated(TEGRA2_A9_CORE0);
        ret = s->csr[TEGRA2_A9_CORE0].reg32;
        break;
    case CPU1_CSR_OFFSET:
        s->csr[TEGRA2_A9_CORE1].halt = tegra_cpu_halted(TEGRA2_A9_CORE1);
        s->csr[TEGRA2_A9_CORE1].pwr_off_sts = tegra_cpu_is_powergated(TEGRA2_A9_CORE1);
        ret = s->csr[TEGRA2_A9_CORE1].reg32;
        break;
    case COP_CSR_OFFSET:
        s->csr[TEGRA2_COP].halt = tegra_cpu_halted(TEGRA2_COP);
        s->csr[TEGRA2_COP].pwr_off_sts = tegra_cpu_is_powergated(TEGRA2_COP);
        ret = s->csr[TEGRA2_COP].reg32;
        break;

    case XRQ_EVENTS_OFFSET:
        ret = s->xrq_events.reg32;
        break;
    default:
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    if (current_cpu && current_cpu->cpu_index == TEGRA2_COP) {
        if (s->halt_events[TEGRA2_COP].mode & INTERRUPT) {
            s->halt_events[TEGRA2_COP].mode |= WAITEVENT;
            s->cop_stalled = 1;

            TPRINT("COP stalled");

            if (!tegra_flow_arm_event(s, TEGRA2_COP, 1)) {
                tegra_flow_clear_waitevent(s, TEGRA2_COP, 0);
            } else {
                tegra_cpu_halt(TEGRA2_COP);
            }
        }
    }

    return ret;
}

static void tegra_flow_run_cpu(int cpu_id)
{
    if (tegra_cpu_is_powergated(cpu_id)) {
        tegra_cpu_unpowergate(cpu_id);
    } else {
        tegra_cpu_unhalt(cpu_id);
    }
}

void tegra_flow_on_irq(int cpu_id)
{
    tegra_flow *s = tegra_flow_dev;
    CPUState *csX;

//     TPRINT("%s cpu %d\n", __func__, cpu_id);

    CPU_FOREACH(csX) {
        int cpu_idX = csX->cpu_index;

        if (!(s->csr[cpu_idX].wait_event)) {
            continue;
        }

        if (s->csr[cpu_idX].event_flag) {
            if (cpu_idX != TEGRA2_COP) {
                goto event;
            }
        }

        if (cpu_id != TEGRA2_COP) {
            if (s->halt_events[cpu_idX].irq_0) {
                goto event;
            }
        } else {
            if (s->halt_events[cpu_idX].irq_1) {
                goto event;
            }
        }

        continue;

event:
        tegra_flow_clear_waitevent(s, cpu_idX, tegra_cpu_halted(cpu_idX));

        if (!(s->halt_events[cpu_idX].mode & STOP)) {
            tegra_flow_run_cpu(cpu_idX);
        }
    }

    if (!(s->halt_events[cpu_id].mode & WAIT_IRQ)) {
        if (tegra_cpu_is_powergated(cpu_id)) {
            cpu_id = tegra_sibling_cpu(cpu_id);

            g_assert(s->halt_events[cpu_id].mode & STOP);
        } else {
            return;
        }
    }

    if (s->halt_events[cpu_id].mode & WAIT_IRQ) {
        s->halt_events[cpu_id].mode &= ~WAIT_IRQ;

        if (!(s->halt_events[cpu_id].mode & STOP)) {
            tegra_flow_run_cpu(cpu_id);
        }
    }
}

static __attribute__ ((__unused__)) const char* tegra_flow_mode_name(int mode)
{
    switch (mode) {
    case FLOW_MODE_NONE:
        return "FLOW_MODE_NONE";
    case FLOW_MODE_RUN_AND_INT:
        return "FLOW_MODE_RUN_AND_INT";
    case FLOW_MODE_WAITEVENT:
        return "FLOW_MODE_WAITEVENT";
    case FLOW_MODE_WAITEVENT_AND_INT:
        return "FLOW_MODE_WAITEVENT_AND_INT";
    case FLOW_MODE_STOP_UNTIL_IRQ:
        return "FLOW_MODE_STOP_UNTIL_IRQ";
    case FLOW_MODE_STOP_UNTIL_IRQ_AND_INT:
        return "FLOW_MODE_STOP_UNTIL_IRQ_AND_INT";
    case FLOW_MODE_STOP_UNTIL_EVENT_AND_IRQ:
        return "FLOW_MODE_STOP_UNTIL_EVENT_AND_IRQ";
    default:
        TPRINT("wrong flow mode\n");
        g_assert_not_reached();
    }

    return "";
}

static void tegra_flow_timer_event(void *opaque)
{
    tegra_flow_timer_arg *arg = opaque;
    tegra_flow *s = arg->s;
    int cpu_id = arg->cpu_id;

    s->halt_events[cpu_id].zero = MAX(0, s->halt_events[cpu_id].zero - 1);

    TPRINT("tegra_flow: event on cpu %d zero=%u event_flag=%d mode=%s\n",
           cpu_id, s->halt_events[cpu_id].zero, s->csr[cpu_id].event_flag,
           tegra_flow_mode_name(s->halt_events[cpu_id].mode));

    if (s->halt_events[cpu_id].zero) {
        return;
    }

    __tegra_flow_clear_waitevent(s, cpu_id, tegra_cpu_halted(cpu_id));

    if (!(s->halt_events[cpu_id].mode & STOP)) {
        tegra_flow_run_cpu(cpu_id);
    }
}

static int tegra_flow_powergate(tegra_flow *s, int cpu_id, int is_sibling)
{
    if (!s->csr[cpu_id].enable) {
        return 0;
    }

    if (!(s->halt_events[cpu_id].mode & STOP)) {
        return 0;
    }

    if (s->csr[cpu_id].wait_wfe_bitmap) {
        uint32_t wfe_bitmap = tegra_get_wfe_bitmap();
        wfe_bitmap &= s->csr[cpu_id].wait_wfe_bitmap;

        if (s->csr[cpu_id].wait_wfe_bitmap != wfe_bitmap) {
//             tegra_cpu_halt(cpu_id);
            return 0;
        }
    } else if (is_sibling) {
        return 0;
    }

    tegra_cpu_powergate(cpu_id);

    if (s->halt_events[cpu_id].mode & WAITEVENT) {
        if (!tegra_flow_arm_event(s, cpu_id, 1)) {
            tegra_flow_clear_waitevent(s, cpu_id, 1);
        }
    }

    if (s->halt_events[cpu_id].mode & WAIT_IRQ) {
        if (tegra_flow_have_pending_irq(cpu_id)) {
            s->halt_events[cpu_id].mode &= ~WAIT_IRQ;
        }
    }

    if (!(s->halt_events[cpu_id].mode & STOP)) {
        tegra_cpu_unpowergate(cpu_id);
    }

    return 1;
}

static void tegra_flow_update_mode(tegra_flow *s, int cpu_id, int in_wfe)
{
    int is_cop = (cpu_id == TEGRA2_COP);

//     TPRINT("%s mode=%s in_wfe=%d cpu %d\n", __func__,
//            tegra_flow_mode_name(s->halt_events[cpu_id].mode), in_wfe, cpu_id);

    if (in_wfe) {
        CPUState *cs = CPU(qemu_get_cpu(cpu_id));
        int sibling = tegra_sibling_cpu(cpu_id);

        bql_lock();

        if (tegra_flow_powergate(s, cpu_id, 0) ||
            tegra_flow_powergate(s, sibling, 1))
        {
            bql_unlock();
            cpu_loop_exit(cs);
        }
    }

    if (!in_wfe) {
        switch (s->halt_events[cpu_id].mode) {
        case FLOW_MODE_RUN_AND_INT:
            if (!tegra_flow_arm_event(s, cpu_id, 0)) {
                tegra_flow_gen_interrupt(s, cpu_id);
            }
            /* Fallthrough */
        case FLOW_MODE_NONE:
            tegra_cpu_unhalt(cpu_id);
            return;
        }
    }

    if (s->halt_events[cpu_id].mode & WAITEVENT) {
        int wait = (is_cop || in_wfe);

        if (!tegra_flow_arm_event(s, cpu_id, wait)) {
            tegra_flow_clear_waitevent(s, cpu_id, wait);
        }
    }

    if (!is_cop && !in_wfe) {
        return;
    }

    if (s->halt_events[cpu_id].mode & WAIT_IRQ) {
        if (tegra_flow_have_pending_irq(cpu_id)) {
            s->halt_events[cpu_id].mode &= ~WAIT_IRQ;
        }
    }

    if (s->halt_events[cpu_id].mode & STOP) {
        tegra_cpu_halt(cpu_id);
    }

    if (in_wfe) {
        bql_unlock();
    }
}

void tegra_flow_wfe_handle(int cpu_id)
{
    tegra_flow *s = tegra_flow_dev;

    tegra_flow_update_mode(s, cpu_id, 1);
}

static void tegra_flow_event_write(tegra_flow *s, hwaddr offset,
                                   uint32_t value, int cpu_id)
{
    TRACE_WRITE(s->iomem.addr, offset, s->halt_events[cpu_id].reg32, value);

    s->halt_events[cpu_id].reg32 = value;
    tegra_flow_update_mode(s, cpu_id, 0);
}

static void tegra_flow_csr_write(tegra_flow *s, hwaddr offset,
                                 uint32_t value, int cpu_id)
{
    csr_t old_csr = s->csr[cpu_id];

    TRACE_WRITE(s->iomem.addr, offset, s->csr[cpu_id].reg32, value);

    if (cpu_id != TEGRA2_COP) {
        WR_MASKED(s->csr[cpu_id].reg32, value, CPU_CSR);
    } else {
        WR_MASKED(s->csr[cpu_id].reg32, value, COP_CSR);
    }

//     s->csr[cpu_id].reg32 &= ~(value & 0x3000);

    if (s->csr[cpu_id].event_flag) {
        s->csr[cpu_id].event_flag = 0;
    } else {
        s->csr[cpu_id].event_flag = old_csr.event_flag;
    }

    if (old_csr.intr_flag && s->csr[cpu_id].intr_flag) {
        switch (cpu_id) {
        case TEGRA2_A9_CORE0:
        case TEGRA2_A9_CORE1:
            if (s->csr[ tegra_sibling_cpu(cpu_id) ].intr_flag) {
                break;
            }
            TRACE_IRQ_LOWER(s->iomem.addr, s->irq_cpu_event);
            break;
        case TEGRA2_COP:
            TRACE_IRQ_LOWER(s->iomem.addr, s->irq_cop_event);
            break;
        }
        s->csr[cpu_id].intr_flag = 0;
    } else {
        s->csr[cpu_id].intr_flag = old_csr.intr_flag;
    }
}

static void tegra_flow_priv_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    tegra_flow *s = opaque;

    switch (offset) {
    case HALT_CPU0_EVENTS_OFFSET:
        tegra_flow_event_write(s, offset, value, TEGRA2_A9_CORE0);
        break;
    case HALT_CPU1_EVENTS_OFFSET:
        tegra_flow_event_write(s, offset, value, TEGRA2_A9_CORE1);
        break;
    case HALT_COP_EVENTS_OFFSET:
        tegra_flow_event_write(s, offset, value, TEGRA2_COP);
        break;

    case CPU0_CSR_OFFSET:
        tegra_flow_csr_write(s, offset, value, TEGRA2_A9_CORE0);
        break;
    case CPU1_CSR_OFFSET:
        tegra_flow_csr_write(s, offset, value, TEGRA2_A9_CORE1);
        break;
    case COP_CSR_OFFSET:
        tegra_flow_csr_write(s, offset, value, TEGRA2_COP);
        break;

    case XRQ_EVENTS_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, s->xrq_events.reg32, value);
        s->xrq_events.reg32 = value;
        break;

    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }
}

static void tegra_flow_priv_reset(DeviceState *dev)
{
    tegra_flow *s = TEGRA_FLOW_CTRL(dev);

    s->halt_events[TEGRA2_A9_CORE0].reg32 = HALT_CPU_EVENTS_RESET;
    s->halt_events[TEGRA2_A9_CORE1].reg32 = HALT_CPU_EVENTS_RESET;
    s->csr[TEGRA2_A9_CORE0].reg32 = CPU_CSR_RESET;
    s->csr[TEGRA2_A9_CORE1].reg32 = CPU_CSR_RESET;

    s->halt_events[TEGRA2_COP].reg32 = HALT_COP_EVENTS_RESET;
    s->csr[TEGRA2_COP].reg32 = COP_CSR_RESET;

    s->xrq_events.reg32 = XRQ_EVENTS_RESET;
    s->cop_stalled = 0;
}

static const MemoryRegionOps tegra_flow_mem_ops = {
    .read = tegra_flow_priv_read,
    .write = tegra_flow_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tegra_flow_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_flow *s = TEGRA_FLOW_CTRL(dev);
    int i;

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_cpu_event);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_cop_event);

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_flow_mem_ops, s,
                          "tegra.flow", TEGRA_FLOW_CTRL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    for (i = 0; i < TEGRA2_NCPUS; i++) {
        tegra_flow_timer_arg *arg = g_malloc0(sizeof(tegra_flow_timer_arg));

        arg->s = s;
        arg->cpu_id = i;

        s->ptimer[i] = ptimer_init(tegra_flow_timer_event, arg,
                                   PTIMER_POLICY_CONTINUOUS_TRIGGER);
        ptimer_transaction_begin(s->ptimer[i]);
        ptimer_set_freq(s->ptimer[i], 1000000);
        ptimer_transaction_commit(s->ptimer[i]);
    }
}

static void tegra_flow_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_flow_priv_realize;
    dc->vmsd = &vmstate_tegra_flow;
    dc->reset = tegra_flow_priv_reset;
}

static const TypeInfo tegra_flow_info = {
    .name = TYPE_TEGRA_FLOW_CTRL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_flow),
    .class_init = tegra_flow_class_init,
};

static void tegra_flow_register_types(void)
{
    type_register_static(&tegra_flow_info);
}

type_init(tegra_flow_register_types)
