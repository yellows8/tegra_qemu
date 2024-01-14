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

#include "hw/hw.h"
#include "hw/arm/boot.h"
#include "sysemu/reset.h"
#include "sysemu/sysemu.h"
#include "target/arm/arm-powerctl.h"
#include "cpu.h"

#include "devices.h"
#include "tegra_cpu.h"
#include "tegra_trace.h"

#include "tegra_cpu_priv.h"

#undef TPRINT
#define TPRINT(...) {}

static int tegra_A9_powergated;
static int tegra_AVP_powergated;

static int tcpu_in_reset[TEGRA_NCPUS];

static void tegra_dump_cpus_pc(void)
{
#if defined(TEGRA_TRACE) && 0
    CPUState *cs;

    const char * tegra_cpu_name[TEGRA_NCPUS] = {
        [TEGRA2_A9_CORE0] = "CPU0",
        [TEGRA2_A9_CORE1] = "CPU1",
        [TEGRA2_COP]  = "COP",
    };

    TPRINT("*** dump cpus pc ***\n");

    CPU_FOREACH(cs) {
        ARMCPU *cpu = ARM_CPU(cs);

        TPRINT("%s: pc= 0x%08X halted=%d powered_off=%d\n",
               tegra_cpu_name[cs->cpu_index], cpu->env.regs[15],
               cs->halted, cpu->power_state == PSCI_OFF);
    }

    TPRINT("----------------------------------\n");
#endif
}

static void tegra_cpu_pwrgate_reset(void *opaque)
{
    tegra_A9_powergated = 0;
    tegra_AVP_powergated = 0;

    tegra_cpu_hlt_clr();
}

static void tegra_do_cpu_reset(void *opaque)
{
    CPUState *cs = opaque;
    int cpu_id = cs->cpu_index;

    assert(cpu_id < TEGRA_NCPUS);

    tcpu_in_reset[cpu_id] = 1;
}

void tegra_cpu_reset_init(void)
{
    CPUState *cs;

    CPU_FOREACH(cs)
        qemu_register_reset(tegra_do_cpu_reset, cs);

    qemu_register_reset(tegra_cpu_pwrgate_reset, NULL);
}

int tegra_cpu_reset_asserted(int cpu_id)
{
    return tcpu_in_reset[cpu_id];
}

void tegra_cpu_reset_assert(int cpu_id)
{
    CPUState *cs = qemu_get_cpu(cpu_id);
    ARMCPU *cpu = ARM_CPU(cs);

    TPRINT("%s cpu %d tcpu_in_reset: %d\n",
           __func__, cpu_id, tcpu_in_reset[cpu_id]);

    if (!tcpu_in_reset[cpu_id]) {
        tcpu_in_reset[cpu_id] = 1;

        arm_set_cpu_off(cpu_id);

        /* Force poweron work queuing.  */
        cpu->power_state = PSCI_OFF;
    }
}

void tegra_cpu_reset_deassert(int cpu_id, int flow)
{
    CPUState *cs = qemu_get_cpu(cpu_id);
    ARMCPU *cpu = ARM_CPU(cs);

    TPRINT("%s cpu %d tcpu_in_reset: %d flow: %d powergated: %d halted: %d\n",
           __func__, cpu_id, tcpu_in_reset[cpu_id], flow,
           tegra_cpu_is_powergated(cpu_id), tegra_cpu_halted(cpu_id));

    if (!flow && tegra_cpu_is_powergated(cpu_id)) {
        return;
    }

    if (tcpu_in_reset[cpu_id]) {
        tcpu_in_reset[cpu_id] = 0;

        if (cpu_id == TEGRA_BPMP)
            arm_set_cpu_on(cpu_id, 0x0, 0, 1, 0);
        else
            arm_set_cpu_on(cpu_id, 0xf0010000, 0, 3, 1);

        /* Force poweroff work queuing.  */
        cpu->power_state = PSCI_ON;

        if (tegra_cpu_halted(cpu_id)) {
            cpu_interrupt(cs, CPU_INTERRUPT_HALT);
        }
    }
}

int tegra_cpu_is_powergated(int cpu_id)
{
    switch (cpu_id) {
    case TEGRA_CCPLEX_CORE0:
    case TEGRA_CCPLEX_CORE1:
    case TEGRA_CCPLEX_CORE2:
    case TEGRA_CCPLEX_CORE3:
        return tegra_A9_powergated;
    case TEGRA_BPMP:
        return tegra_AVP_powergated;
    default:
        g_assert_not_reached();
    }

    return 0;
}

static void tegra_cpu_powergateA9(void)
{
    TPRINT("%s\n", __func__);

    tegra_dump_cpus_pc();

    assert(!tegra_A9_powergated);

    tegra_cpu_reset_assert(TEGRA_CCPLEX_CORE0);
    tegra_cpu_reset_assert(TEGRA_CCPLEX_CORE1);
    tegra_cpu_reset_assert(TEGRA_CCPLEX_CORE2);
    tegra_cpu_reset_assert(TEGRA_CCPLEX_CORE3);
    tegra_a9mpcore_reset();
    tegra_A9_powergated = 1;
}

static void tegra_cpu_unpowergateA9(void)
{
    TPRINT("%s powergated=%d\n", __func__, tegra_A9_powergated);

    assert(tegra_A9_powergated);

    tegra_cpu_reset_deassert(TEGRA_CCPLEX_CORE0, 1);
    tegra_A9_powergated = 0;
}

static void tegra_cpu_powergateAVP(void)
{
    TPRINT("%s powergated=%d\n", __func__, tegra_AVP_powergated);

    assert(!tegra_AVP_powergated);

    tegra_cpu_reset_assert(TEGRA_BPMP);
    tegra_AVP_powergated = 1;
}

static void tegra_cpu_unpowergateAVP(void)
{
    TPRINT("%s\n", __func__);

    assert(tegra_AVP_powergated);

    tegra_cpu_reset_deassert(TEGRA_BPMP, 1);
    tegra_AVP_powergated = 0;
}

static void tegra_cpu_powergate_sanity_check(void)
{
    CPUState *cs = qemu_get_cpu(TEGRA_CCPLEX_CORE1);
    ARMCPU *cpu = ARM_CPU(cs);

    /* Core 1 should be stopped before CPU powergate.  */
    g_assert(cpu->power_state == PSCI_OFF);
}

void tegra_cpu_powergate(int cpu_id)
{
    switch (cpu_id) {
    case TEGRA_CCPLEX_CORE0:
    case TEGRA_CCPLEX_CORE1:
    case TEGRA_CCPLEX_CORE2:
    case TEGRA_CCPLEX_CORE3:
        tegra_cpu_powergate_sanity_check();
        tegra_cpu_powergateA9();
        break;
    case TEGRA_BPMP:
        tegra_cpu_powergateAVP();
        break;
    default:
        g_assert_not_reached();
    }
}

void tegra_cpu_unpowergate(int cpu_id)
{
    switch (cpu_id) {
    case TEGRA_CCPLEX_CORE0:
    case TEGRA_CCPLEX_CORE1:
    case TEGRA_CCPLEX_CORE2:
    case TEGRA_CCPLEX_CORE3:
        tegra_cpu_unpowergateA9();
        break;
    case TEGRA_BPMP:
        tegra_cpu_unpowergateAVP();
        break;
    default:
        g_assert_not_reached();
    }
}
