/*
 * ARM NVIDIA Tegra2/X1 emulation.
 *
 * Copyright (c) 2014-2015 Dmitry Osipenko <digetx@gmail.com>
 * Copyright (c) yellows8
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

// Modified version of tegra2.c.

#define CONFIG_ARCH_TEGRA_21x_SOC
#define CONFIG_ARM_ARCH_TIMER
#define CONFIG_ARCH_TEGRA_VIC

#include "tegra_common.h"

#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/config-file.h"
#include "qapi/qmp/qlist.h"
#include "cpu.h"
#include "hw/cpu/cluster.h"
#include "exec/address-spaces.h"
#include "hw/boards.h"
#include "hw/sysbus.h"
#include "hw/arm/boot.h"
#include "hw/loader.h"
#include "hw/char/serial.h"
#include "hw/sd/sdhci.h"
#include "hw/net/lan9118.h"
#include "sysemu/reset.h"
#include "sysemu/sysemu.h"
#include "sysemu/cpus.h"
#include "hw/intc/arm_gic.h"
#include "hw/intc/arm_gicv3_common.h"
#include "hw/intc/arm_gicv3_its_common.h"
#include "hw/arm/bsa.h"
#include "hw/arm/fdt.h"
#include "sysemu/device_tree.h"

#include "hw/hw.h"
#include "net/net.h"

#include "ahb/tsec/tsec.h"

#include "devices.h"
#include "iomap.h"
#include "irqs.h"
/*#include "remote_io.h"*/
#include "tegra_cpu.h"
//#include "tegra_trace.h"

//#include "bundle/boot_iram.bin.h"
//#include "bundle/u_boot_dtb_tegra.bin.h"

#define DIRQ(X) qdev_get_gpio_in(tegra_irq_dispatcher_dev, X)
#define DIRQ_INT(X) qdev_get_gpio_in(tegra_irq_dispatcher_dev, X + INT_MAIN_NR)

#define BOOTLOADER_BASE 0x40010000
#define BOOTMON_BASE    0xF0010000
#define BOOTROM_LOVEC_BASE 0x0

#define SYSTEM_TICK_FREQ 19200000

#define RW  0
#define RO  1

#define JMP_FIXUP   (sizeof(tegra_bootrom) / 4 - 2)

/* WARNING: HACK! FIXME: Copied from exec.c  */
struct CPUAddressSpace {
    CPUState *cpu;
    AddressSpace *as;
    struct AddressSpaceDispatch *memory_dispatch;
    MemoryListener tcg_as_listener;
};

#if 0
static uint32_t tegra_bootmon[] = {
    0xe3a00206, /* ldr r0, =TEGRA_PG */
    /*0xe5901000,*/ /* ldr r1, [r0]] */
    0xe3a01001, /* mov r1, #1*/
    0xe59f0054, /* ldr r0, =TEGRA_PG_A9 */
    0xe1500001, /* cmp r0, r1 */
    0x1a00000f, /* bne boot         @ on AVP */
    0xee100fb0, /* mrc 15, 0, r0, cr0, cr0, {5} */
    0xe200000f, /* and r0, r0, #0xF */
    0xe3500000, /* cmp r0, #0 */
    0x0a00000b, /* beq boot */
    0xe59f303c, /* ldr r3, =bootX */
    0xe5930000, /* ldr r0, [r3] */
    0xe3500001, /* cmp r0, #1 */
    0x1a000007, /* bne boot */
    0xe59f2030, /* ldr r2, =TEGRA_ARM_INT_DIST_BASE */
    0xe3a01001, /* mov r1, #1 */
    0xe5821000, /* str r1, [r2]         @ set GICC_CTLR.Enable */
    0xe3a010ff, /* mov r1, #0xff */
    0xe5821004, /* str r1, [r2, #4]     @ set GIC_PMR.Priority to 0xff */
    0xf57ff04f, /* dsb */
    0xe3a01001, /* mov r1, #1 */
    0xe5831000, /* str r1, [r3] */
/* boot: */
    0xe59f0014, /* ldr r0, =TEGRA_EXCEPTION_VECTORS_BASE */
    0xe5900000, /* ldr r0, [r0] */
    0xe12fff10, /* bx  r0 */
    0x00000000, /* bootX */
    0x55555555, /* TEGRA_PG_A9 */
    0x00000060, /* TEGRA_PG */
    0x50041000, /* TEGRA_ARM_INT_DIST_BASE */
    0x6000f000, /* TEGRA_EXCEPTION_VECTORS_BASE */
};
#endif

#if 0
static uint32_t tegra_bootmon[] = { // CCPLEX reset vector
    0xd29e201e, /*mov     x30, #0xf100*/
    0xf2ac001e, /*movk    x30, #0x6000, lsl #16*/
    0xb94003de, /*ldr     w30, [x30]*/
    0xeb1f03df, /*cmp     x30, xzr*/
    0x540000a1, /*b.ne    <jump>*/
    0xd298461e, /*mov     x30, #0xc230*/
    0xf2ac001e, /*movk    x30, #0x6000, lsl #16*/
    0xf94003de, /*ldr     x30, [x30]*/
    0x927ffbde, /*and     x30, x30, #0xfffffffffffffffe*/
/* jump:*/
    0xd61f03c0, /*br      x30*/
};
#endif

static uint32_t tegra_bootrom[] = {
    0xea000006, /* b reset_addr */
    0xea000007, /* b undefined_addr */
    0xea000008, /* b svc_addr */
    0xea000009, /* b prefetch_addr */
    0xea00000a, /* b abort_addr */
    0xea00000b, /* b reserved_vector */
    0xea00000c, /* b irq_addr */
    0xea00000d, /* b fiq_handler */
/* reset_addr: */
    0xe59f0044, /* ldr r0, =DUMMY */
    0xe12fff10, /* bx  r0 */
/* undefined_addr: */
    0xe3a01001, /* mov r1, #1 */
    0xea00000a, /* b reboot */
/* svc_addr: */
    0xe3a01002, /* mov r1, #2 */
    0xea000008, /* b reboot */
/* prefetch_addr: */
    0xe3a01003, /* mov r1, #3 */
    0xea000006, /* b reboot */
/* abort_addr: */
    0xe3a01004, /* mov r1, #4 */
    0xea000004, /* b reboot */
/* reserved_vector: */
    0xe3a01005, /* mov r1, #5 */
    0xea000002, /* b reboot */
/* irq_addr: */
    0xe3a01006, /* mov r1, #6 */
    0xea000000, /* b reboot */
/* fiq_handler: */
    0xe3a01007, /* mov r1, #7 */
/* reboot: */
    0xe3a02010, /* mov r2, #0x10 */
    0xe59f0008, /* ldr r0, =TEGRA_PMC_BASE */
    0xe5802000, /* str r2, [r0] */
    0xeafffffb, /* b reboot */
    0xffffffff, /* DUMMY */
    TEGRA_PMC_BASE,
};

static void memory_region_add_and_init_ram(MemoryRegion *mr, const char *name,
                                           hwaddr offset, uint64_t size, int ro)
{
    MemoryRegion *ram = g_new(MemoryRegion, 1);
    memory_region_init_ram(ram, NULL, name, size, &error_abort);
    memory_region_set_readonly(ram, ro);
    memory_region_add_subregion(mr, offset, ram);
}

static void cop_memory_region_add_alias(MemoryRegion *mr, const char *name,
                                        MemoryRegion *sysmem, hwaddr cop_offset,
                                         hwaddr sys_offset, uint64_t size)
{
    MemoryRegion *ram = g_new(MemoryRegion, 1);
    memory_region_init_alias(ram, NULL, name, sysmem, sys_offset, size);
    memory_region_add_subregion(mr, cop_offset, ram);
}

static void tegrax1_create_cpus(void)
{
    int i;

    Object *cluster = object_new(TYPE_CPU_CLUSTER);
    qdev_prop_set_uint32(DEVICE(cluster), "cluster-id", 0);

    for (i = 0; i < TEGRAX1_CCPLEX_NCORES; i++) {
        Object *cpuobj = object_new(ARM_CPU_TYPE_NAME("cortex-a57"));

        object_property_add_child(cluster, "cpu[*]", cpuobj);

        object_property_set_int(cpuobj, "reset-cbar", TEGRA_ARM_PERIF_BASE, &error_abort);
        object_property_set_bool(cpuobj, "has_el3", true, &error_abort);
        // rvbar is configured in reset.c.
        object_property_set_bool(cpuobj, "start-powered-off", true, &error_abort);
        object_property_set_uint(cpuobj, "cntfrq", SYSTEM_TICK_FREQ, &error_abort); // This must be configured manually since qemu doesn't update the timer frequency when the cntfrq reg is written.
        qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);

        // TODO: How to properly set oslsr_el1, without changing the default reset value?(Below doesn't work correctly)
        /*CPUState *cs = qemu_get_cpu(i);
        ARMCPU *cpu = ARM_CPU(cs);
        CPUARMState *env = &cpu->env;
        printf("env->cp15.oslsr_el1: 0x%lx\n", env->cp15.oslsr_el1);
        env->cp15.oslsr_el1 = 0;*/
    }

    qdev_realize(DEVICE(cluster), NULL, &error_fatal);
    cluster = object_new(TYPE_CPU_CLUSTER);
    qdev_prop_set_uint32(DEVICE(cluster), "cluster-id", 1);

    /* BPMP */
    Object *cpuobj = object_new(ARM_CPU_TYPE_NAME("arm7tdmi"));
    object_property_add_child(cluster, "cpu[*]", cpuobj);
    object_property_set_bool(cpuobj, "start-powered-off", true, &error_abort);
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);

    qdev_realize(DEVICE(cluster), NULL, &error_fatal);

    set_is_tegra_cpu(TEGRA_CCPLEX_CORE0);
    set_is_tegra_cpu(TEGRA_CCPLEX_CORE1);
    set_is_tegra_cpu(TEGRA_CCPLEX_CORE2);
    set_is_tegra_cpu(TEGRA_CCPLEX_CORE3);
    set_is_tegra_cpu(TEGRA_BPMP);
}

//static struct arm_boot_info tegra_board_binfo = {
//    .board_id = -1, /* device-tree-only board */
//};

static void load_memory_images(MachineState *machine)
{
    //const char *iram_path = machine->iram;
    int tmp;

    tegra_bootrom[JMP_FIXUP] = BOOTLOADER_BASE;

    for (tmp = 0; tmp < ARRAY_SIZE(tegra_bootrom); tmp++)
        tegra_bootrom[tmp] = tswap32(tegra_bootrom[tmp]);

    if (machine->bootloader != NULL) {
        /* Load bootloader */
        assert(load_image_targphys(machine->bootloader, BOOTLOADER_BASE,
                                   128*1024) > 0);
    }

    #if 0
    if (iram_path != NULL) {
        /* Load BIT */
        assert(load_image_targphys(iram_path, TEGRA_IRAM_BASE,
                                   TEGRA_IRAM_SIZE) > 0);
    } /*else {
        printf("-iram not specified, falling back to bundled\n");
        rom_add_blob_fixed_as("iram", iram_bin, iram_bin_len,
                              TEGRA_IRAM_BASE, &address_space_memory);
    }*/
    #endif

    /* Load IROM */
    if (machine->firmware == NULL) {
        rom_add_blob_fixed("bpmp.bootrom", tegra_bootrom, sizeof(tegra_bootrom),
                           TEGRA_IROM_BASE);
    }
    else {
        assert(load_image_targphys(machine->firmware, TEGRA_IROM_BASE,
                                   TEGRA_IROM_SIZE) > 0);
    }

    /*for (tmp = 0; tmp < ARRAY_SIZE(tegra_bootmon); tmp++)
        tegra_bootmon[tmp] = tswap32(tegra_bootmon[tmp]);*/

    /* Load boot monitor */
    /*rom_add_blob_fixed("bootmon", tegra_bootmon, sizeof(tegra_bootmon),
                       BOOTMON_BASE);*/

    /*if (machine->firmware != NULL) { // secmon
        tmp = load_image_targphys(machine->firmware, 0x040030000, 128*1024);
        assert(tmp > 0);
    }*/

    /*if (machine->kernel_filename != NULL) { // raw package2
        load_image_targphys(machine->kernel_filename, 0xA9800000,
                            machine->ram_size - 0x29800000);
    }*/
}

static void* tegra_init_sdmmc(int index, hwaddr base, qemu_irq irq, bool emmc, uint32_t bootpartsize, void** vendor_dev)
{
    DeviceState *carddev;
    BlockBackend *blk;
    DriveInfo *di;

    void* tmpdev = qdev_new(TYPE_SYSBUS_SDHCI);
    qdev_prop_set_uint32(tmpdev, "capareg", 0x376c0c8c); // Value from hardware is 0x376cd08c. Workaround qemu BASECLKFREQ validation.
    sysbus_realize_and_unref(SYS_BUS_DEVICE(tmpdev), &error_fatal);

    sysbus_mmio_map(SYS_BUS_DEVICE(tmpdev), 0, base);
    sysbus_connect_irq(SYS_BUS_DEVICE(tmpdev), 0, irq);
    *vendor_dev = sysbus_create_simple("tegra.sdmmcvendor", base+0x100, NULL);

    di = drive_get_by_index(IF_SD, index);
    blk = di ? blk_by_legacy_dinfo(di) : NULL;
    carddev = qdev_new(TYPE_SD_CARD);
    qdev_prop_set_drive(carddev, "drive", blk);
    qdev_prop_set_bit(carddev, "emmc", emmc);
    if (emmc) qdev_prop_set_uint32(carddev, "bootpartsize", bootpartsize);
    qdev_realize_and_unref(carddev, qdev_get_child_bus(tmpdev, "sd-bus"), &error_fatal);
    return tmpdev;
}

static void __tegrax1_init(MachineState *machine)
{
    MemoryRegion *cop_sysmem = g_new(MemoryRegion, 1);
    AddressSpace *cop_as = g_new(AddressSpace, 1);
    MemoryRegion *sysmem = get_system_memory();
    SysBusDevice *irq_dispatcher, *lic;
    DeviceState *cpudev;
    CPUState *cs;
    int i, j;

    /* Main RAM */
    assert(machine->ram_size <= TEGRA_DRAM_SIZE);
    memory_region_add_and_init_ram(sysmem, "tegra.dram",
                                   TEGRA_DRAM_BASE, machine->ram_size, RW);

    /*memory_region_add_and_init_ram(sysmem, "tegra.hi-vec",
                                   0xffff0000, SZ_64K, RW);*/

    /*memory_region_add_and_init_ram(sysmem, "tegra.bootmon",
                                   BOOTMON_BASE, TARGET_PAGE_SIZE, RO);*/

    /* Internal static RAM */
    memory_region_add_and_init_ram(sysmem, "tegra.iram",
                                   TEGRA_IRAM_BASE, TEGRA_IRAM_SIZE, RW);

    /* Map 0x400-0x40000 of IRAM to remote device.  */
//     memory_region_add_and_init_ram(sysmem, "tegra.iram",
//                                    TEGRA_IRAM_BASE, TEGRA_RESET_HANDLER_SIZE, RW);
//     sysbus_create_simple("tegra.remote_iram",
//                          TEGRA_IRAM_BASE + TEGRA_RESET_HANDLER_SIZE, NULL);

    /*memory_region_add_and_init_ram(sysmem, "tegra.irom_lovec",
                                   BOOTROM_LOVEC_BASE, 0x100000, RO);*/

    memory_region_add_and_init_ram(sysmem, "tegra.irom",
                                   TEGRA_IROM_BASE, TEGRA_IROM_SIZE, RO);

    memory_region_add_and_init_ram(sysmem, "tegra.ahb_a1",
                                   0x78000000, SZ_16M, RW);

    memory_region_add_and_init_ram(sysmem, "tegra.ppcs",
                                   0x7c000000, 0x10000, RW);

    memory_region_add_and_init_ram(sysmem, "tegra.tzram",
                                   0x7c010000, SZ_64K, RW);

    memory_region_add_and_init_ram(sysmem, "tegra.ahb_a2",
                                   0x7c020000, 0x7d000000-0x7c020000, RW);

    /* Create the actual CPUs */
    tegrax1_create_cpus();

    /* Generic Interrupt Controller */
    gic_dev = qdev_new(gic_class_name());
    SysBusDevice *gicbusdev = SYS_BUS_DEVICE(gic_dev);

    qdev_prop_set_uint32(DEVICE(gic_dev), "num-irq", INT_GIC_NR);
    qdev_prop_set_uint32(DEVICE(gic_dev), "revision", 2);
    qdev_prop_set_uint32(DEVICE(gic_dev), "num-cpu", TEGRAX1_CCPLEX_NCORES);
    qdev_prop_set_bit(DEVICE(gic_dev), "has-security-extensions", true);
    //qdev_prop_set_bit(DEVICE(gic_dev), "has-virtualization-extensions", true);

    /*QList *redist_region_count = qlist_new();
    qlist_append_int(redist_region_count, TEGRAX1_CCPLEX_NCORES);
    qdev_prop_set_array(gic_dev, "redist-region-count",
                        redist_region_count);*/

    sysbus_realize_and_unref(gicbusdev, &error_fatal);

    sysbus_mmio_map(SYS_BUS_DEVICE(gic_dev), 0, TEGRA_ARM_PERIF_BASE+0x1000);
    sysbus_mmio_map(SYS_BUS_DEVICE(gic_dev), 1, TEGRA_ARM_PERIF_BASE+0x2000);
    //sysbus_mmio_map(SYS_BUS_DEVICE(gic_dev), 2, TEGRA_ARM_PERIF_BASE+0x4000);
    //sysbus_mmio_map(SYS_BUS_DEVICE(gic_dev), 3, TEGRA_ARM_PERIF_BASE+0x5000);

    // Below block is based on virt.c.
    /* Wire the outputs from each CPU's generic timer and the GICv3
     * maintenance interrupt signal to the appropriate GIC PPI inputs,
     * and the GIC's IRQ/FIQ/VIRQ/VFIQ interrupt outputs to the CPU's inputs.
     */
    for (i = 0; i < TEGRAX1_CCPLEX_NCORES; i++) {
        cpudev = DEVICE(qemu_get_cpu(i));
        int intidbase = INT_MAIN_NR + i * GIC_INTERNAL;
        /* Mapping from the output timer irq lines from the CPU to the
         * GIC PPI inputs we use for the virt board.
         */
        const int timer_irq[] = {
            [GTIMER_PHYS] = ARCH_TIMER_NS_EL1_IRQ,
            [GTIMER_VIRT] = ARCH_TIMER_VIRT_IRQ,
            [GTIMER_HYP]  = ARCH_TIMER_NS_EL2_IRQ,
            [GTIMER_SEC]  = ARCH_TIMER_S_EL1_IRQ,
        };

        for (unsigned irq = 0; irq < ARRAY_SIZE(timer_irq); irq++) {
            qdev_connect_gpio_out(cpudev, irq,
                                  qdev_get_gpio_in(gic_dev,
                                                   intidbase + timer_irq[irq]));
        }

        /*qemu_irq irq = qdev_get_gpio_in(gic_dev,
                                        intidbase + ARCH_GIC_MAINT_IRQ);
        qdev_connect_gpio_out_named(cpudev, "gicv3-maintenance-interrupt",
                                    0, irq);*/

        /*qdev_connect_gpio_out_named(cpudev, "pmu-interrupt", 0,
                                    qdev_get_gpio_in(&gic, intidbase
                                                     + VIRTUAL_PMU_IRQ));*/

        sysbus_connect_irq(gicbusdev, i, qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));
        sysbus_connect_irq(gicbusdev, i + TEGRAX1_CCPLEX_NCORES,
                           qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));
        sysbus_connect_irq(gicbusdev, i + 2 * TEGRAX1_CCPLEX_NCORES,
                           qdev_get_gpio_in(cpudev, ARM_CPU_VIRQ));
        sysbus_connect_irq(gicbusdev, i + 3 * TEGRAX1_CCPLEX_NCORES,
                           qdev_get_gpio_in(cpudev, ARM_CPU_VFIQ));
    }

    /* Legacy interrupt controller */
    tegra_ictlr_dev = qdev_new("tegra.ictlr");
    lic = SYS_BUS_DEVICE(tegra_ictlr_dev);
    qdev_prop_set_uint32(DEVICE(tegra_ictlr_dev), "num-cpu", TEGRAX1_CCPLEX_NCORES);
    qdev_prop_set_uint32(DEVICE(tegra_ictlr_dev), "num-irq", INT_MAIN_NR);
    sysbus_realize_and_unref(lic, &error_fatal);
    sysbus_mmio_map(lic, 0, TEGRA_PRIMARY_ICTLR_BASE);

    tegra_irq_dispatcher_dev = qdev_new("tegra.irq_dispatcher");
    irq_dispatcher = SYS_BUS_DEVICE(tegra_irq_dispatcher_dev);
    qdev_prop_set_uint32(DEVICE(tegra_irq_dispatcher_dev), "num-cpu", TEGRAX1_CCPLEX_NCORES);
    sysbus_realize_and_unref(irq_dispatcher, &error_fatal);

    for (i = 0, j = 0; i < TEGRAX1_NCPUS; i++) {
        cpudev = DEVICE(qemu_get_cpu(i));
        sysbus_connect_irq(irq_dispatcher, j++,
                                        qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));
        sysbus_connect_irq(irq_dispatcher, j++,
                                        qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));
    }

    sysbus_connect_irq(gicbusdev, TEGRA_CCPLEX_CORE0, DIRQ_INT(ARM_CPU_IRQ));
    sysbus_connect_irq(gicbusdev, TEGRA_CCPLEX_CORE1, DIRQ_INT(ARM_CPU_IRQ + 1));
    sysbus_connect_irq(gicbusdev, TEGRA_CCPLEX_CORE2, DIRQ_INT(ARM_CPU_IRQ + 2));
    sysbus_connect_irq(gicbusdev, TEGRA_CCPLEX_CORE3, DIRQ_INT(ARM_CPU_IRQ + 3));

    /* CPU IRQ+FIQ */
    sysbus_connect_irq(lic, 0, DIRQ_INT(ARM_CPU_IRQ + 2));
    sysbus_connect_irq(lic, 1, DIRQ_INT(ARM_CPU_FIQ + 2));

    /* COP IRQ+FIQ */
    sysbus_connect_irq(lic, 2, DIRQ_INT(ARM_CPU_IRQ + 4));
    sysbus_connect_irq(lic, 3, DIRQ_INT(ARM_CPU_FIQ + 4));

    /* Cache controller */
    //sysbus_create_simple("l2x0", TEGRA_ARM_PL310_BASE, NULL);

    /* MSELECT */
    tegra_mselect_dev = sysbus_create_simple("tegra.mselect",
                                         TEGRA_MSELECT_BASE, NULL);

    /* Exception vectors */
    tegra_evp_dev = qdev_new("tegra.evp");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(tegra_evp_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(tegra_evp_dev), 0, TEGRA_EXCEPTION_VECTORS_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(tegra_evp_dev), 1, BOOTROM_LOVEC_BASE);

    /* Secure boot */
    tegra_sb_dev = qdev_new("tegra.sb");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(tegra_sb_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(tegra_sb_dev), 0, TEGRA_SB_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(tegra_sb_dev), 1, TEGRA_IPATCH_BASE);

    /* Embedded memory controller */
    tegra_emc_dev = sysbus_create_simple("tegra.emc", TEGRA_EMC_BASE, NULL);
    tegra_emc0_dev = sysbus_create_simple("tegra.emc", TEGRA_EMC0_BASE, NULL);
    tegra_emc1_dev = sysbus_create_simple("tegra.emc", TEGRA_EMC1_BASE, NULL);

    /* Memory controller */
    tegra_mc_dev = qdev_new("tegra.mc");
    qdev_prop_set_uint32(tegra_mc_dev, "ram_size_kb", machine->ram_size / SZ_1K);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(tegra_mc_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(tegra_mc_dev), 0, TEGRA_MC_BASE);

    tegra_mc0_dev = qdev_new("tegra.mc");
    qdev_prop_set_uint32(tegra_mc0_dev, "ram_size_kb", machine->ram_size / SZ_1K);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(tegra_mc0_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(tegra_mc0_dev), 0, TEGRA_MC0_BASE);

    tegra_mc1_dev = qdev_new("tegra.mc");
    qdev_prop_set_uint32(tegra_mc1_dev, "ram_size_kb", machine->ram_size / SZ_1K);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(tegra_mc1_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(tegra_mc1_dev), 0, TEGRA_MC1_BASE);

    /* AHB DMA controller */
    tegra_ahb_dma_dev = sysbus_create_simple("tegra.ahb_dma",
                                             TEGRA_AHB_DMA_BASE,
                                             DIRQ(INT_AHB_DMA));

    /* AHB Gizmo controller */
    tegra_ahb_gizmo_dev = sysbus_create_simple("tegra.ahb_gizmo",
                                               TEGRA_AHB_GIZMO_BASE, NULL);

    /* APB FUSE controller */
    tegra_fuse_dev = sysbus_create_simple("tegra.fuse", TEGRA_FUSE_BASE, NULL);

    /* SE */
    tegra_se_dev = sysbus_create_simple("tegra.se", TEGRA_SE_BASE, DIRQ(INT_SE));
    if (tegra_board == TEGRAX1PLUS_BOARD) {
        tegra_se2_dev = sysbus_create_simple("tegra.se", TEGRA_SE2_BASE, DIRQ(INT_SE)); // NOTE: This IRQ is likely wrong?
    }

    /* SYSCTR0 */
    tegra_sysctr0_dev = sysbus_create_simple("tegra.sysctr0", TEGRA_TSC_BASE, NULL);

    /* AVPCACHE */
    tegra_avpcache_dev = sysbus_create_simple("tegra.avpcache", TEGRA_ARM_PERIF_BASE, NULL);

    /* APB DMA controller */
    tegra_apb_dma_dev = sysbus_create_simple("tegra.apb_dma",
                                             TEGRA_APB_DMA_BASE, NULL);

    /* APB bus controller */
    tegra_apb_misc_dev = sysbus_create_simple("tegra.apb_misc",
                                              TEGRA_APB_MISC_BASE, NULL);

    /* PINMUX_AUX */
    tegra_pinmuxaux_dev = sysbus_create_simple("tegra.pinmuxaux",
                                              TEGRA_PINMUX_AUX_BASE, NULL);

    /* Clock and reset controller */
    tegra_car_dev = sysbus_create_simple("tegra.car",
                                         TEGRA_CLK_RESET_BASE, NULL);

    /* CPU Flow controller */
    tegra_flow_dev = qdev_new("tegra.flow");
    SysBusDevice *s = SYS_BUS_DEVICE(tegra_flow_dev);
    qdev_prop_set_uint32(DEVICE(tegra_flow_dev), "num-cpu", TEGRA_CCPLEX_NCORES);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_FLOW_CTRL_BASE);
    sysbus_connect_irq(s, 0, DIRQ(INT_FLOW_RSM_CPU));
    sysbus_connect_irq(s, 1, DIRQ(INT_FLOW_RSM_COP));

    /* GPIO controllers */
    tegra_gpios_dev = sysbus_create_varargs("tegra.gpio", TEGRA_GPIO_BASE,
                                            DIRQ(INT_GPIO1), DIRQ(INT_GPIO2),
                                            DIRQ(INT_GPIO3), DIRQ(INT_GPIO4),
                                            DIRQ(INT_GPIO5), DIRQ(INT_GPIO6),
                                            DIRQ(INT_GPIO7), DIRQ(INT_GPIO8),
                                            NULL);

    /* Power managment controller */
    tegra_pmc_dev = sysbus_create_simple("tegra.pmc", TEGRA_PMC_BASE, NULL);

    /* Real time clock */
    tegra_rtc_dev = sysbus_create_simple("tegra.rtc",
                                         TEGRA_RTC_BASE, DIRQ(INT_RTC));

    tegra_sdmmc1_dev = tegra_init_sdmmc(0, TEGRA_SDMMC1_BASE, DIRQ(INT_SDMMC1), false, 0, &tegra_sdmmc1_vendor_dev);
    tegra_sdmmc2_dev = tegra_init_sdmmc(1, TEGRA_SDMMC2_BASE, DIRQ(INT_SDMMC2), false, 0, &tegra_sdmmc2_vendor_dev);
    tegra_sdmmc3_dev = tegra_init_sdmmc(2, TEGRA_SDMMC3_BASE, DIRQ(INT_SDMMC3), false, 0, &tegra_sdmmc3_vendor_dev);
    tegra_sdmmc4_dev = tegra_init_sdmmc(3, TEGRA_SDMMC4_BASE, DIRQ(INT_SDMMC4), true, 0x400000, &tegra_sdmmc4_vendor_dev);

    /* Timer0 */
    tegra_timer10_dev = sysbus_create_simple("tegra.timer",
                                            TEGRA_TMR0_BASE, DIRQ(INT_TMR0));

    /* Timer1 */
    tegra_timer1_dev = sysbus_create_simple("tegra.timer",
                                            TEGRA_TMR1_BASE, DIRQ(INT_TMR1));

    /* Timer2 */
    tegra_timer2_dev = sysbus_create_simple("tegra.timer",
                                            TEGRA_TMR2_BASE, DIRQ(INT_TMR2));

    /* TimerUS */
    tegra_timer_us_dev = sysbus_create_simple("tegra.timer_us",
                                              TEGRA_TMRUS_BASE, NULL);

    /* Timer3 */
    tegra_timer3_dev = sysbus_create_simple("tegra.timer",
                                            TEGRA_TMR3_BASE, DIRQ(INT_TMR3));

    /* Timer4 */
    tegra_timer4_dev = sysbus_create_simple("tegra.timer",
                                            TEGRA_TMR4_BASE, DIRQ(INT_TMR4));

    /* Timer5 */
    tegra_timer5_dev = sysbus_create_simple("tegra.timer",
                                            TEGRA_TMR5_BASE, DIRQ(INT_TMR5));

    /* Timer6 */
    tegra_timer6_dev = sysbus_create_simple("tegra.timer",
                                            TEGRA_TMR6_BASE, DIRQ(INT_TMR6));

    /* Timer7 */
    tegra_timer7_dev = sysbus_create_simple("tegra.timer",
                                            TEGRA_TMR7_BASE, DIRQ(INT_TMR7));

    /* Timer8 */
    tegra_timer8_dev = sysbus_create_simple("tegra.timer",
                                            TEGRA_TMR8_BASE, DIRQ(INT_TMR8));

    /* Timer9 */
    tegra_timer9_dev = sysbus_create_simple("tegra.timer",
                                            TEGRA_TMR9_BASE, DIRQ(INT_TMR9));

    /* Timer10 */
    tegra_timer10_dev = sysbus_create_simple("tegra.timer",
                                            TEGRA_TMR10_BASE, DIRQ(INT_TMR10));

    /* Timer11 */
    tegra_timer11_dev = sysbus_create_simple("tegra.timer",
                                            TEGRA_TMR11_BASE, DIRQ(INT_TMR11));

    /* Timer12 */
    tegra_timer12_dev = sysbus_create_simple("tegra.timer",
                                            TEGRA_TMR12_BASE, DIRQ(INT_TMR12));

    /* Timer13 */
    tegra_timer13_dev = sysbus_create_simple("tegra.timer",
                                            TEGRA_TMR13_BASE, DIRQ(INT_TMR13));

    /* UART controllers */
    tegra_uarta_dev = serial_mm_init(sysmem, TEGRA_UARTA_BASE, 2,
                                     DIRQ(INT_UARTA), 115200,
                                     serial_hd(0),
                                     DEVICE_LITTLE_ENDIAN);

    tegra_uartb_dev = serial_mm_init(sysmem, TEGRA_UARTB_BASE, 2,
                                     DIRQ(INT_UARTB), 115200,
                                     serial_hd(1),
                                     DEVICE_LITTLE_ENDIAN);

    tegra_uartc_dev = serial_mm_init(sysmem, TEGRA_UARTC_BASE, 2,
                                     DIRQ(INT_UARTC), 115200,
                                     serial_hd(2),
                                     DEVICE_LITTLE_ENDIAN);

    tegra_uartd_dev = serial_mm_init(sysmem, TEGRA_UARTD_BASE, 2,
                                     DIRQ(INT_UARTD), 115200,
                                     serial_hd(3),
                                     DEVICE_LITTLE_ENDIAN);

    /* MIPI_CAL */
    tegra_mipical_dev = sysbus_create_simple("tegra.mipical",
                                             TEGRA_MIPI_CAL_BASE, NULL);

    /* USB2 controllers */
    tegra_ehci1_dev = sysbus_create_simple("tegra.usb",
                                           TEGRA_USB_BASE, DIRQ(INT_USB));
//     tegra_ehci2_dev = sysbus_create_simple("tegra.usb",
//                                            TEGRA_USB2_BASE, DIRQ(INT_USB2));
    /*tegra_ehci3_dev = sysbus_create_simple("tegra.usb",
                                           TEGRA_USB3_BASE, DIRQ(INT_USB3));*/

    /* Unified Command Queue */
    //tegra_ucq_dev = sysbus_create_simple("tegra.dummy256", 0x60010000, NULL);

    /* Bit Stream Engine Audio */
    /*tegra_bsea_dev = sysbus_create_simple("tegra.bsea", 0x60011000,
                                          DIRQ(INT_VDE_BSE_A));*/

    /* Syntax Engine */
    //tegra_sxe_dev = sysbus_create_simple("tegra.sxe", 0x6001A000, NULL);

    /* BSE Video */
    /*tegra_bsev_dev = sysbus_create_simple("tegra.bsev", 0x6001B000,
                                          DIRQ(INT_VDE_BSE_V));*/

    /* Macroblock Engine */
    //tegra_mbe_dev = sysbus_create_simple("tegra.mbe", 0x6001C000, NULL);

    /* Post-processing Engine */
    //tegra_ppe_dev = sysbus_create_simple("tegra.dummy256", 0x6001C200, NULL);

    /* Motion Compensation Engine */
    //tegra_mce_dev = sysbus_create_simple("tegra.mce", 0x6001C400, NULL);

    /* Transform Engine */
    //tegra_tfe_dev = sysbus_create_simple("tegra.tfe", 0x6001C600, NULL);

    /* Pixel Prediction Block?? */
    //tegra_ppb_dev = sysbus_create_simple("tegra.dummy256", 0x6001C800, NULL);

    /* Video DMA */
    //tegra_vdma_dev = sysbus_create_simple("tegra.vdma", 0x6001CA00, NULL);

    /* Unified Command Queue */
    //tegra_ucq2_dev = sysbus_create_simple("tegra.dummy256", 0x6001CC00, NULL);

    /* BSE Audio */
    //tegra_bsea2_dev = sysbus_create_simple("tegra.dummy_2k", 0x6001D000, NULL);

    /* FRAMEID */
    //tegra_frameid_dev = sysbus_create_simple("tegra.frameid", 0x6001D800, NULL);

//     sysbus_create_varargs("tegra.bse_remote", 0x60010000,
//                           DIRQ(INT_VDE_UCQ_ERROR),
//                           DIRQ(INT_VDE_SYNC_TOKEN),
//                           DIRQ(INT_VDE_BSE_V),
//                           DIRQ(INT_VDE_BSE_A),
//                           DIRQ(INT_VDE_SXE), NULL);

    /* I2C controllers */
    tegra_idc1_dev = sysbus_create_simple("tegra-i2c",
                                          TEGRA_I2C_BASE, DIRQ(INT_I2C));
    tegra_idc2_dev = sysbus_create_simple("tegra-i2c",
                                          TEGRA_I2C2_BASE, DIRQ(INT_I2C2));
    tegra_idc3_dev = sysbus_create_simple("tegra-i2c",
                                          TEGRA_I2C3_BASE, DIRQ(INT_I2C3));
    tegra_idc4_dev = sysbus_create_simple("tegra-i2c",
                                          TEGRA_I2C4_BASE, DIRQ(INT_I2C4));
    tegra_idc5_dev = sysbus_create_simple("tegra-i2c",
                                          TEGRA_I2C5_BASE, DIRQ(INT_I2C5));
    tegra_idc6_dev = sysbus_create_simple("tegra-i2c",
                                          TEGRA_I2C6_BASE, DIRQ(INT_I2C6));

    /* Host1x IO */
    tegra_grhost_dev = sysbus_create_varargs("tegra.grhost",
                                             TEGRA_GRHOST_BASE,
                                             DIRQ(INT_HOST1X_SYNCPT_COP),
                                             DIRQ(INT_HOST1X_SYNCPT_CPU),
                                             DIRQ(INT_HOST1X_GEN_COP),
                                             DIRQ(INT_HOST1X_GEN_CPU),
                                             NULL);

    /* Host1x */
    tegra_host1x_dev = sysbus_create_simple("tegra.host1x", TEGRA_HOST1X_BASE,
                                            NULL);

    /* GPU 2d */
    //tegra_gr2d_dev = sysbus_create_simple("tegra.gr2d", TEGRA_GR2D_BASE, NULL);

    /* Display1 controller */
    tegra_dc1_dev = sysbus_create_simple("tegra.dc", TEGRA_DISPLAY_BASE,
                                         DIRQ(INT_DISPLAY_GENERAL));

    /* DSI */
    tegra_dsi_dev = sysbus_create_simple("tegra.dsi", TEGRA_DSI_BASE,
                                         NULL);

    /* TSEC */
    tegra_tsec_dev = qdev_new("tegra.tsec");
    s = SYS_BUS_DEVICE(tegra_tsec_dev);
    qdev_prop_set_uint32(DEVICE(tegra_tsec_dev), "engine", TEGRA_TSEC_ENGINE_TSEC);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_TSEC_BASE);
    sysbus_connect_irq(s, 0, DIRQ(INT_TSEC));

    tegra_tsecb_dev = qdev_new("tegra.tsec");
    s = SYS_BUS_DEVICE(tegra_tsecb_dev);
    qdev_prop_set_uint32(DEVICE(tegra_tsecb_dev), "engine", TEGRA_TSEC_ENGINE_TSECB);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_TSECB_BASE);
    sysbus_connect_irq(s, 0, DIRQ(INT_TSECB));

    tegra_vic_dev = qdev_new("tegra.tsec");
    s = SYS_BUS_DEVICE(tegra_vic_dev);
    qdev_prop_set_uint32(DEVICE(tegra_vic_dev), "engine", TEGRA_TSEC_ENGINE_VIC);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_VIC_BASE);
    sysbus_connect_irq(s, 0, DIRQ(INT_VIC_GENERAL));

    tegra_nvenc_dev = qdev_new("tegra.tsec");
    s = SYS_BUS_DEVICE(tegra_nvenc_dev);
    qdev_prop_set_uint32(DEVICE(tegra_nvenc_dev), "engine", TEGRA_TSEC_ENGINE_NVENC);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_NVENC_BASE);
    sysbus_connect_irq(s, 0, DIRQ(INT_NVENC));

    tegra_nvdec_dev = qdev_new("tegra.tsec");
    s = SYS_BUS_DEVICE(tegra_nvdec_dev);
    qdev_prop_set_uint32(DEVICE(tegra_nvdec_dev), "engine", TEGRA_TSEC_ENGINE_NVDEC);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_NVDEC_BASE);
    sysbus_connect_irq(s, 0, DIRQ(INT_NVDEC));

    tegra_nvjpg_dev = qdev_new("tegra.tsec");
    s = SYS_BUS_DEVICE(tegra_nvjpg_dev);
    qdev_prop_set_uint32(DEVICE(tegra_nvjpg_dev), "engine", TEGRA_TSEC_ENGINE_NVJPG);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_NVJPG_BASE);
    sysbus_connect_irq(s, 0, DIRQ(INT_NVJPG));

    /* Process generator tag */
    //sysbus_create_simple("tegra.pg", 0x60000000, NULL);

    /* Multi-CPU shared resources access arbitration */
    tegra_arb_sema_dev = sysbus_create_varargs("tegra.arb_sema",
                                               TEGRA_ARB_SEMA_BASE,
                                               DIRQ(INT_ARB_SEM_GNT_COP),
                                               DIRQ(INT_ARB_SEM_GNT_CPU),
                                               NULL);

    tegra_res_sema_dev = sysbus_create_varargs("tegra.res_sema",
                                               TEGRA_RES_SEMA_BASE,
                                               DIRQ(INT_SHR_SEM_INBOX_IBF),
                                               DIRQ(INT_SHR_SEM_INBOX_IBE),
                                               DIRQ(INT_SHR_SEM_OUTBOX_IBF),
                                               DIRQ(INT_SHR_SEM_OUTBOX_IBE),
                                               NULL);

    /* AVP "MMU" TLB controls.  */
    //tegra_cop_mmu_dev = sysbus_create_simple("tegra.cop_mmu", 0xF0000000, NULL);

    /* COP's address map differs a bit from A9.  */
    memory_region_init(cop_sysmem, NULL, "tegra.cop-memory", UINT64_MAX);
    address_space_init(cop_as, cop_sysmem, "tegra.cop-address space");

    /*memory_region_add_and_init_ram(cop_sysmem, "tegra.cop-ivectors",
                                   0x00000000, 0x40, RW);

    memory_region_add_and_init_ram(cop_sysmem, "tegra.cop-hi-vec",
                                   0xffff0000, SZ_64K, RW);*/

    /*cop_memory_region_add_alias(cop_sysmem, "tegra.cop-DRAM", sysmem,
                                0x00000040,
                                0x00000040, TEGRA_DRAM_SIZE - 0x40);*/

    cop_memory_region_add_alias(cop_sysmem, "tegra.cop-IRAM", sysmem,
                                TEGRA_IRAM_BASE,
                                TEGRA_IRAM_BASE, TEGRA_IRAM_SIZE);

    /*cop_memory_region_add_alias(cop_sysmem, "tegra.cop-GRHOST", sysmem,
                                TEGRA_GRHOST_BASE,
                                TEGRA_GRHOST_BASE, TEGRA_GRHOST_SIZE);*/

    cop_memory_region_add_alias(cop_sysmem, "tegra.cop-iomirror", sysmem,
                                0x50040000,
                                0x50040000, 0x60000000-0x50040000);

    cop_memory_region_add_alias(cop_sysmem, "tegra.cop-HOST1X", sysmem,
                                TEGRA_HOST1X_BASE,
                                TEGRA_HOST1X_BASE, TEGRA_HOST1X_SIZE);

    /*cop_memory_region_add_alias(cop_sysmem, "tegra.cop-GART", sysmem,
                                TEGRA_GART_BASE,
                                TEGRA_GART_BASE, TEGRA_GART_SIZE);*/

    cop_memory_region_add_alias(cop_sysmem, "tegra.cop-PPSB", sysmem,
                                0x60000000,
                                0x60000000, SZ_256M);

    cop_memory_region_add_alias(cop_sysmem, "tegra.cop-APB", sysmem,
                                0x70000000,
                                0x70000000, SZ_256M);

    cop_memory_region_add_alias(cop_sysmem, "tegra.cop-DRAM UC", sysmem,
                                0x80000000,
                                0x80000000, machine->ram_size);

    /*cop_memory_region_add_alias(cop_sysmem, "tegra.cop-AHB", sysmem,
                                0xC0000000,
                                0xC0000000, SZ_128M + SZ_1K + SZ_512);*/

    cop_memory_region_add_alias(cop_sysmem, "tegra.bpmp-IROM_LOVEC", sysmem,
                                BOOTROM_LOVEC_BASE,
                                BOOTROM_LOVEC_BASE, 0x1000);

    cop_memory_region_add_alias(cop_sysmem, "tegra.cop-IROM", sysmem,
                                TEGRA_IROM_BASE,
                                TEGRA_IROM_BASE, TEGRA_IROM_SIZE);

    /*cop_memory_region_add_alias(cop_sysmem, "tegra.cop-bootmon", sysmem,
                                BOOTMON_BASE,
                                BOOTMON_BASE, TARGET_PAGE_SIZE);*/

    /*cop_memory_region_add_alias(cop_sysmem, "tegra.cop-mmu", sysmem,
                                0xF0000000,
                                0xF0000000, SZ_64K);*/

    /* Map 0x2F600000-0x1F600000 to remote device.  */
//     sysbus_create_simple("tegra.remote_mem", 0x2F600000, NULL);
//     cop_memory_region_add_alias(cop_sysmem, "tegra.cop-remote_mem", sysmem,
//                                 0x2F600000,
//                                 0x2F600000, 0x10000000);

    cs = qemu_get_cpu(TEGRA_BPMP);
    cs->as = cop_as;

    /* Override default AS.  */
    memory_listener_unregister(&cs->cpu_ases[0].tcg_as_listener);
    cs->cpu_ases[0].as = cop_as;
    memory_listener_register(&cs->cpu_ases[0].tcg_as_listener, cop_as);

    load_memory_images(machine);

    tegra_cpu_reset_init();
}

static void tegrax1_init(MachineState *machine)
{
    tegra_board = TEGRAX1_BOARD;
    __tegrax1_init(machine);
}

static void tegrax1plus_init(MachineState *machine)
{
    tegra_board = TEGRAX1PLUS_BOARD;
    __tegrax1_init(machine);
}

static void tegrax1_reset(MachineState *state, ShutdownCause cause)
{
//     remote_io_init("10.1.1.3:45312");
    //tegra_trace_init();
    qemu_devices_reset(cause);

    if (state->firmware != NULL || state->bootloader != NULL)
        tegra_cpu_reset_deassert(TEGRA_BPMP, 1);
    else
        tegra_cpu_reset_deassert(TEGRA_CCPLEX_CORE0, 1);
}

static void __tegrax1_machine_init(MachineClass *mc, const char *desc)
{
    mc->desc = desc;
    mc->init = tegrax1_init;
    mc->reset = tegrax1_reset;
    mc->default_cpus = TEGRAX1_NCPUS;
    mc->min_cpus = TEGRAX1_NCPUS;
    mc->max_cpus = TEGRAX1_NCPUS;
    mc->ignore_memory_transaction_failures = true;
}

static void tegrax1_machine_init(MachineClass *mc)
{
    __tegrax1_machine_init(mc, "ARM NVIDIA Tegra X1");
}

static void tegrax1plus_machine_init(MachineClass *mc)
{
    __tegrax1_machine_init(mc, "ARM NVIDIA Tegra X1+");
    mc->init = tegrax1plus_init;
}

DEFINE_MACHINE("tegrax1", tegrax1_machine_init)
DEFINE_MACHINE("tegrax1plus", tegrax1plus_machine_init)

