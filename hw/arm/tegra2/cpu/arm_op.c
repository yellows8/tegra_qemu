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

#include "hw/arm/boot.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "exec/exec-all.h"

#include "devices.h"
#include "iomap.h"
#include "tegra_cpu.h"
#include "tegra_trace.h"

#define HALT_WFI    0xfe
#define HALT_WFE    0xff

static int tegra_cpus[TEGRA_NCPUS];

void set_is_tegra_cpu(int cpu_id)
{
    tegra_cpus[cpu_id] = 1;
}

static int is_tegra_cpu(int cpu_id)
{
    if (cpu_id > TEGRA_NCPUS) {
        return 0;
    }

    return tegra_cpus[cpu_id];
}

/*int __attribute__((const)) tegra_sibling_cpu(int cpu_id)
{
    switch (cpu_id) {
    case TEGRA2_A9_CORE0:
        return TEGRA2_A9_CORE1;
        break;
    case TEGRA2_A9_CORE1:
        return TEGRA2_A9_CORE0;
        break;
    }

    return cpu_id;
}*/

uint32_t tegra_get_wfe_bitmap(int type)
{
    uint32_t wfe_bitmap = 0;
    int i;

    for (i = 0; i < TEGRA_CCPLEX_NCORES; i++) {
        CPUState *cs = CPU(qemu_get_cpu(i));
        wfe_bitmap |= (cs->halted == HALT_WFE-type) << i;
    }

    return wfe_bitmap;
}

void HELPER(wfi)(CPUARMState *env, uint32_t insn_len)
{
#ifdef CONFIG_USER_ONLY
    /*
     * WFI in the user-mode emulator is technically permitted but not
     * something any real-world code would do. AArch64 Linux kernels
     * trap it via SCTRL_EL1.nTWI and make it an (expensive) NOP;
     * AArch32 kernels don't trap it so it will delay a bit.
     * For QEMU, make it NOP here, because trying to raise EXCP_HLT
     * would trigger an abort.
     */
    return;
#else
    CPUState *cs = env_cpu(env);
    //int target_el = check_wfx_trap(env, false);

    if (cpu_has_work(cs)) {
        /* Don't bother to go into our "low power state" if
         * we would just wake up immediately.
         */
        return;
    }

    /*if (target_el) {
        if (env->aarch64) {
            env->pc -= insn_len;
        } else {
            env->regs[15] -= insn_len;
        }

        raise_exception(env, EXCP_UDEF, syn_wfx(1, 0xe, 0, insn_len == 2),
                        target_el);
    }*/

    cs->exception_index = EXCP_HLT;
    cs->halted = 1;

    int cpu_id = cs->cpu_index;
    if (is_tegra_cpu(cpu_id)) {
        cs->halted = HALT_WFI;

        tegra_flow_wfe_handle(cpu_id, 1);
    }
    else cpu_loop_exit(cs);
#endif
}

void HELPER(wfe)(CPUARMState *env)
{
    CPUState *cs = env_cpu(env);
    int cpu_id = cs->cpu_index;

    if (is_tegra_cpu(cpu_id)) {
//         TPRINT("WFE: cpu %d\n", cpu_id);

        cs->halted = HALT_WFE;

        tegra_flow_wfe_handle(cpu_id, 0);

        /* Won't return here if flow powergated CPU.  */
        cs->halted = 0;
    }

    HELPER(yield)(env);
}
