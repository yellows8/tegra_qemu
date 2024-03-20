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
#include "hw/cpu/a9mpcore.h"
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

#include "ahb/host1x/modules/tsec/tsec.h"
#include "apb/pmc/pmc.h"
#include "ahb/sb/sb.h"
#include "ppsb/evp/evp.h"
#include "apb/fuse/fuse.h"
#include "apb/i2c/i2c.h"
#include "dummyi2c/dummyi2c.h"
#include "dummyio/dummyio.h"

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
    MemoryRegion *ram = g_new0(MemoryRegion, 1);
    memory_region_init_ram(ram, NULL, name, size, &error_abort);
    memory_region_set_readonly(ram, ro);
    memory_region_add_subregion(mr, offset, ram);
}

static void tegra_memory_region_add_alias(MemoryRegion *mr, const char *name,
                                          MemoryRegion *sysmem, hwaddr alias_offset,
                                          hwaddr sys_offset, uint64_t size)
{
    MemoryRegion *ram = g_new0(MemoryRegion, 1);
    memory_region_init_alias(ram, NULL, name, sysmem, sys_offset, size);
    memory_region_add_subregion(mr, alias_offset, ram);
}

static void tegrax1_create_cpus(MemoryRegion *cop_sysmem, MemoryRegion *ape_sysmem)
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

    /* BPMP */
    cluster = object_new(TYPE_CPU_CLUSTER);
    qdev_prop_set_uint32(DEVICE(cluster), "cluster-id", 1);

    Object *cpuobj = object_new(ARM_CPU_TYPE_NAME("arm7tdmi"));
    object_property_add_child(cluster, "cpu[*]", cpuobj);
    object_property_set_bool(cpuobj, "start-powered-off", true, &error_abort);
    object_property_set_link(cpuobj, "memory", OBJECT(cop_sysmem), &error_abort);
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);

    qdev_realize(DEVICE(cluster), NULL, &error_fatal);

    /* ADSP */
    cluster = object_new(TYPE_CPU_CLUSTER);
    qdev_prop_set_uint32(DEVICE(cluster), "cluster-id", 2);

    cpuobj = object_new(ARM_CPU_TYPE_NAME("cortex-a9"));
    object_property_add_child(cluster, "cpu[*]", cpuobj);
    object_property_set_int(cpuobj, "reset-cbar", 0x00C00000, &error_abort);
    object_property_set_bool(cpuobj, "has_el3", /*true*/false, &error_abort); // NOTE: Disabled for AGIC to work properly CPU-side.
    object_property_set_bool(cpuobj, "reset-hivecs", false, &error_abort);
    object_property_set_bool(cpuobj, "start-powered-off", true, &error_abort);
    object_property_set_link(cpuobj, "memory", OBJECT(ape_sysmem), &error_abort);
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);

    qdev_realize(DEVICE(cluster), NULL, &error_fatal);

    set_is_tegra_cpu(TEGRA_CCPLEX_CORE0);
    set_is_tegra_cpu(TEGRA_CCPLEX_CORE1);
    set_is_tegra_cpu(TEGRA_CCPLEX_CORE2);
    set_is_tegra_cpu(TEGRA_CCPLEX_CORE3);
    set_is_tegra_cpu(TEGRA_BPMP);
    set_is_tegra_cpu(TEGRA_ADSP);
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
                                   TEGRA_IRAM_BASE+TEGRA_IRAM_SIZE-BOOTLOADER_BASE) > 0);
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
        /*rom_add_blob_fixed("bpmp.bootrom", tegra_bootrom, sizeof(tegra_bootrom),
                           TEGRA_IROM_BASE);*/
        tegra_sb_load_irom_fixed(tegra_sb_dev, tegra_bootrom, sizeof(tegra_bootrom));
    }
    else {
        /*assert(load_image_targphys(machine->firmware, TEGRA_IROM_BASE,
                                   TEGRA_IROM_SIZE) > 0);*/
        assert(tegra_sb_load_irom_file(tegra_sb_dev, machine->firmware) > 0);
    }
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

static void* tegra_init_dummyio(hwaddr base, uint32_t size, const char *name)
{
    void* tmpdev = qdev_new("tegra.dummyio");
    qdev_prop_set_uint32(tmpdev, "size", size);
    qdev_prop_set_string(tmpdev, "name", name);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(tmpdev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(tmpdev), 0, base);

    return tmpdev;
}

static void* tegra_init_obj(hwaddr base, qemu_irq irq, const char *name, const char *prop_name, uint32_t value)
{
    void* tmpdev = qdev_new(name);
    qdev_prop_set_uint32(tmpdev, prop_name, value);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(tmpdev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(tmpdev), 0, base);
    sysbus_connect_irq(SYS_BUS_DEVICE(tmpdev), 0, irq);

    return tmpdev;
}

static void* tegra_init_timer(hwaddr base, qemu_irq irq, uint32_t id)
{
    return tegra_init_obj(base, irq, "tegra.timer", "id", id);
}

// TODO: Doesn't work with CPU MMU, use memregions for now.
/*static hwaddr tegra_ape_translate(hwaddr addr, int access_type)
{
    if ((addr >= 0x01000000 && addr < 0x702C0000) || addr >= 0x70300000) {
        // TODO
        // addr = <translate>
        if (addr < 0x80000000ULL+0x6F2C0000ULL)
            addr = addr - 0x80000000ULL + 0x01000000ULL;
        else
            addr = addr - 0x80000000ULL - 0x6F2C0000ULL + 0x70300000;
    }

    return addr;
}*/

static void __tegrax1_init(MachineState *machine)
{
    MemoryRegion *cop_sysmem = g_new0(MemoryRegion, 1);
    MemoryRegion *ape_sysmem = g_new0(MemoryRegion, 1);
    //AddressSpace *cop_as = g_new0(AddressSpace, 1);
    //AddressSpace *ape_as = g_new0(AddressSpace, 1);
    MemoryRegion *sysmem = get_system_memory();
    SysBusDevice *irq_dispatcher, *lic;
    DeviceState *cpudev;
    //CPUState *cs;
    int i, j;

    memory_region_init(cop_sysmem, NULL, "tegra.cop-memory", UINT64_MAX);
    //address_space_init(cop_as, cop_sysmem, "tegra.cop-address space");

    memory_region_init(ape_sysmem, NULL, "tegra.ape-memory", UINT64_MAX);
    //address_space_init(ape_as, ape_sysmem, "tegra.ape-address space");

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

    /*memory_region_add_and_init_ram(sysmem, "tegra.irom",
                                   TEGRA_IROM_BASE, TEGRA_IROM_SIZE, RO);*/

    memory_region_add_and_init_ram(sysmem, "tegra.stm",
                                   0x71000000, SZ_16M, RW);

    memory_region_add_and_init_ram(sysmem, "tegra.csite",
                                   0x72000000, SZ_32M, RW);

    memory_region_add_and_init_ram(sysmem, "tegra.ahb_a1",
                                   0x78000000, SZ_16M, RW);

    memory_region_add_and_init_ram(sysmem, "tegra.ppcs",
                                   0x7c000000, 0x10000, RW);

    memory_region_add_and_init_ram(sysmem, "tegra.tzram",
                                   0x7c010000, SZ_64K, RW);

    memory_region_add_and_init_ram(sysmem, "tegra.ahb_a2",
                                   0x7c020000, 0x7d000000-0x7c020000, RW);

    memory_region_add_and_init_ram(sysmem, "tegra.ahb_a2_upper",
                                   0x7d005800, 0x7e000000-0x7d005800, RW);

    /* Create the actual CPUs */
    tegrax1_create_cpus(cop_sysmem, ape_sysmem);

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

        //sysbus_connect_irq(gicbusdev, i, qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));
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

    for (i = 0, j = 0; i < TEGRAX1_MAIN_NCPUS; i++) {
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
    sysbus_mmio_map(SYS_BUS_DEVICE(tegra_sb_dev), 2, TEGRA_IROM_BASE);

    /* Activity Monitor */
    tegra_actmon_dev = tegra_init_dummyio(TEGRA_ACTMON_BASE, TEGRA_ACTMON_SIZE, "tegra.actmon");

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

    /* SATA */
    tegra_sata_dev = tegra_init_dummyio(TEGRA_SATA_BASE, TEGRA_SATA_SIZE, "tegra.sata");

    /* Audio*/
    tegra_hda_dev = sysbus_create_simple("tegra.hda",
                                         TEGRA_HDA_BASE,
                                         DIRQ(INT_HDA));

    tegra_ape_dev = qdev_new("tegra.ape");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(tegra_ape_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(tegra_ape_dev), 0, TEGRA_APE_BASE);

    /* DDS */
    tegra_dds_dev = tegra_init_dummyio(TEGRA_DDS_BASE, TEGRA_DDS_SIZE, "tegra.dds");

    /* AHB DMA controller */
    tegra_ahb_dma_dev = sysbus_create_simple("tegra.ahb_dma",
                                             TEGRA_AHB_DMA_BASE,
                                             DIRQ(INT_AHB_DMA));

    /* AHB Gizmo controller */
    tegra_ahb_gizmo_dev = sysbus_create_simple("tegra.ahb_gizmo",
                                               TEGRA_AHB_GIZMO_BASE, NULL);

    /* AHB/APB Debug Bus */
    tegra_ahbapb_debugbus_dev = tegra_init_dummyio(TEGRA_AHBAPB_DEBUGBUS_BASE, TEGRA_AHBAPB_DEBUGBUS_SIZE, "tegra.ahbapb_debugbus");

    /* APB FUSE controller */
    tegra_fuse_dev = sysbus_create_simple("tegra.fuse", TEGRA_FUSE_BASE, NULL);

    /* SE */
    tegra_se_dev = tegra_init_obj(TEGRA_SE_BASE, DIRQ(INT_SE), "tegra.se", "engine", 1);
    if (tegra_board == TEGRAX1PLUS_BOARD) {
        tegra_se2_dev = tegra_init_obj(TEGRA_SE2_BASE, DIRQ(INT_SE), "tegra.se", "engine", 2); // NOTE: This IRQ is likely wrong?
    }

    /* TSENSOR */
    tegra_tsensor_dev = tegra_init_dummyio(TEGRA_TSENSOR_BASE, TEGRA_TSENSOR_SIZE, "tegra.tsensor");

    /* CEC */
    tegra_cec_dev = tegra_init_dummyio(TEGRA_CEC_BASE, TEGRA_CEC_SIZE, "tegra.cec");

    /* SYSCTR */
    tegra_sysctr0_dev = sysbus_create_simple("tegra.sysctr", TEGRA_TSC_BASE, NULL);
    tegra_sysctr1_dev = sysbus_create_simple("tegra.sysctr", TEGRA_TSC_BASE + TEGRA_TSC_SIZE, NULL);

    /* SOC_THERM */
    tegra_soctherm_dev = tegra_init_dummyio(TEGRA_SOCTHERM_BASE, TEGRA_SOCTHERM_SIZE, "tegra.soctherm");

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

    /* SDMMC */
    tegra_sdmmc1_dev = tegra_init_sdmmc(0, TEGRA_SDMMC1_BASE, DIRQ(INT_SDMMC1), false, 0, &tegra_sdmmc1_vendor_dev);
    tegra_sdmmc2_dev = tegra_init_sdmmc(1, TEGRA_SDMMC2_BASE, DIRQ(INT_SDMMC2), false, 0, &tegra_sdmmc2_vendor_dev);
    tegra_sdmmc3_dev = tegra_init_sdmmc(2, TEGRA_SDMMC3_BASE, DIRQ(INT_SDMMC3), false, 0, &tegra_sdmmc3_vendor_dev);
    tegra_sdmmc4_dev = tegra_init_sdmmc(3, TEGRA_SDMMC4_BASE, DIRQ(INT_SDMMC4), true, 0x400000, &tegra_sdmmc4_vendor_dev);

    /* SPEEDO */
    tegra_speedo_dev = tegra_init_dummyio(TEGRA_SPEEDO_BASE, TEGRA_SPEEDO_SIZE, "tegra.speedo");

    /* DP2 */
    tegra_dp2_dev = tegra_init_dummyio(TEGRA_DP2_BASE, TEGRA_DP2_SIZE, "tegra.dp2");

    /* APB2JTAG */
    tegra_apb2jtag_dev = tegra_init_dummyio(TEGRA_APB2JTAG_BASE, TEGRA_APB2JTAG_SIZE, "tegra.apb2jtag");

    /* Timer0 */
    tegra_timer_devs[0] = tegra_init_timer(TEGRA_TMR0_BASE, DIRQ(INT_TMR0), 0);

    /* Timer1 */
    tegra_timer_devs[1] = tegra_init_timer(TEGRA_TMR1_BASE, DIRQ(INT_TMR1), 1);

    /* Timer2 */
    tegra_timer_devs[2] = tegra_init_timer(TEGRA_TMR2_BASE, DIRQ(INT_TMR2), 2);

    /* TimerUS */
    tegra_timer_us_dev = sysbus_create_simple("tegra.timer_us",
                                              TEGRA_TMRUS_BASE, NULL);

    /* Timer3 */
    tegra_timer_devs[3] = tegra_init_timer(TEGRA_TMR3_BASE, DIRQ(INT_TMR3), 3);

    /* Timer4 */
    tegra_timer_devs[4] = tegra_init_timer(TEGRA_TMR4_BASE, DIRQ(INT_TMR4), 4);

    /* Timer5 */
    tegra_timer_devs[5] = tegra_init_timer(TEGRA_TMR5_BASE, DIRQ(INT_TMR5), 5);

    /* Timer6 */
    tegra_timer_devs[6] = tegra_init_timer(TEGRA_TMR6_BASE, DIRQ(INT_TMR6), 6);

    /* Timer7 */
    tegra_timer_devs[7] = tegra_init_timer(TEGRA_TMR7_BASE, DIRQ(INT_TMR7), 7);

    /* Timer8 */
    tegra_timer_devs[8] = tegra_init_timer(TEGRA_TMR8_BASE, DIRQ(INT_TMR8), 8);

    /* Timer9 */
    tegra_timer_devs[9] = tegra_init_timer(TEGRA_TMR9_BASE, DIRQ(INT_TMR9), 9);

    /* Timer10 */
    tegra_timer_devs[10] = tegra_init_timer(TEGRA_TMR10_BASE, DIRQ(INT_TMR10), 10);

    /* Timer11 */
    tegra_timer_devs[11] = tegra_init_timer(TEGRA_TMR11_BASE, DIRQ(INT_TMR11), 11);

    /* Timer12 */
    tegra_timer_devs[12] = tegra_init_timer(TEGRA_TMR12_BASE, DIRQ(INT_TMR12), 12);

    /* Timer13 */
    tegra_timer_devs[13] = tegra_init_timer(TEGRA_TMR13_BASE, DIRQ(INT_TMR13), 13);

    /* WDT */

    tegra_wdt_devs[0] = sysbus_create_simple("tegra.wdt",
                                             TEGRA_WDT0_BASE, DIRQ(INT_WDT_CPU));
    tegra_wdt_devs[1] = sysbus_create_simple("tegra.wdt",
                                             TEGRA_WDT1_BASE, DIRQ(INT_WDT_CPU));
    tegra_wdt_devs[2] = sysbus_create_simple("tegra.wdt",
                                             TEGRA_WDT2_BASE, DIRQ(INT_WDT_CPU));
    tegra_wdt_devs[3] = sysbus_create_simple("tegra.wdt",
                                             TEGRA_WDT3_BASE, DIRQ(INT_WDT_CPU));
    tegra_wdt_devs[4] = sysbus_create_simple("tegra.wdt",
                                             TEGRA_WDT4_BASE, DIRQ(INT_WDT_AVP));

    for (i = 0; i < TEGRAX1_MAIN_NCPUS; i++) {
        cpudev = DEVICE(qemu_get_cpu(i));
        sysbus_connect_irq(tegra_wdt_devs[i], 1,
                           qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));
    }

    /* Timer Shared */
    tegra_timer_shared_dev = sysbus_create_simple("tegra.timer_shared",
                                                  TEGRA_TMR_SHARED_BASE, NULL);

    /* UART controllers */
    tegra_uarta_dev = serial_mm_init(sysmem, TEGRA_UARTA_BASE, 2,
                                     DIRQ(INT_UARTA), 115200,
                                     serial_hd(0),
                                     DEVICE_LITTLE_ENDIAN);
    tegra_uarta_vendor_dev = sysbus_create_simple("tegra.uart", TEGRA_UARTA_BASE+0x20, NULL);

    tegra_uartb_dev = serial_mm_init(sysmem, TEGRA_UARTB_BASE, 2,
                                     DIRQ(INT_UARTB), 115200,
                                     serial_hd(1),
                                     DEVICE_LITTLE_ENDIAN);
    tegra_uartb_vendor_dev = sysbus_create_simple("tegra.uart", TEGRA_UARTB_BASE+0x20, NULL);

    tegra_uartc_dev = serial_mm_init(sysmem, TEGRA_UARTC_BASE, 2,
                                     DIRQ(INT_UARTC), 115200,
                                     serial_hd(2),
                                     DEVICE_LITTLE_ENDIAN);
    tegra_uartc_vendor_dev = sysbus_create_simple("tegra.uart", TEGRA_UARTC_BASE+0x20, NULL);

    tegra_uartd_dev = serial_mm_init(sysmem, TEGRA_UARTD_BASE, 2,
                                     DIRQ(INT_UARTD), 115200,
                                     serial_hd(3),
                                     DEVICE_LITTLE_ENDIAN);
    tegra_uartd_vendor_dev = sysbus_create_simple("tegra.uart", TEGRA_UARTD_BASE+0x20, NULL);

    /* IOBIST */
    tegra_iobist_dev = tegra_init_dummyio(TEGRA_IOBIST_BASE, TEGRA_IOBIST_SIZE, "tegra.iobist");

    /* PWM */
    tegra_pwm_dev = tegra_init_dummyio(TEGRA_PWFM_BASE, TEGRA_PWFM_SIZE, "tegra.pwm");

    /* MIPI_CAL */
    tegra_mipical_dev = sysbus_create_simple("tegra.mipical",
                                             TEGRA_MIPI_CAL_BASE, NULL);

    /* DVFS */
    tegra_dvfs_dev = sysbus_create_simple("tegra.dvfs",
                                           TEGRA_CL_DVFS_BASE, NULL);

    /* CLUSTER_CLOCK */
    tegra_cluster_clock_dev = tegra_init_dummyio(TEGRA_CLK13_RESET_BASE, SZ_256K, "tegra.cluster_clock");

    /* USB2 controllers */
    tegra_ehci1_dev = sysbus_create_simple("tegra.usb",
                                           TEGRA_USB_BASE, DIRQ(INT_USB));
    sysbus_mmio_map(SYS_BUS_DEVICE(tegra_ehci1_dev), 1, TEGRA_USB_BASE+0x1000);
    tegra_ehci2_dev = sysbus_create_simple("tegra.usb",
                                           TEGRA_USB2_BASE, DIRQ(INT_USB2));
    sysbus_mmio_map(SYS_BUS_DEVICE(tegra_ehci2_dev), 1, TEGRA_USB2_BASE+0x1000);

    /* XUSB controllers */
    tegra_xusb_dev = sysbus_create_simple("tegra.xusb",
                                           TEGRA_XUSB_PADCTL_BASE, DIRQ(INT_USB3_HOST_INT));
    sysbus_mmio_map(SYS_BUS_DEVICE(tegra_xusb_dev), 1, TEGRA_XUSB_DEV_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(tegra_xusb_dev), 2, TEGRA_XUSB_HOST_BASE);

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

    /* SPI controllers */
    tegra_spi_devs[0] = sysbus_create_simple("tegra.spi",
                                          TEGRA_SPI1_BASE, DIRQ(INT_SPI_1));
    tegra_spi_devs[1] = sysbus_create_simple("tegra.spi",
                                          TEGRA_SPI2_BASE, DIRQ(INT_SPI_2));
    tegra_spi_devs[2] = sysbus_create_simple("tegra.spi",
                                          TEGRA_SPI3_BASE, DIRQ(INT_SPI_3));
    tegra_spi_devs[3] = sysbus_create_simple("tegra.spi",
                                          TEGRA_SPI4_BASE, DIRQ(INT_SPI_4));

    /* QSPI */
    tegra_qspi_dev = tegra_init_dummyio(TEGRA_QSPI_BASE, TEGRA_QSPI_SIZE, "tegra.qspi");

    /* Tmp451 (Temperature Sensor) */
    tegra_i2c_tmpsensor_dev = i2c_slave_create_simple(tegra_i2c_get_bus(tegra_idc1_dev), "dummyi2c", 0x4C);

    uint8_t tmpsensor_regs[0xFF]={}; // Power-on reset values.

    tmpsensor_regs[0x04] = 0x08; // Conversion rate register
    tmpsensor_regs[0x05] = 0x55; // Local temperature high limit
    tmpsensor_regs[0x07] = 0x55; // Remote temperature high limit (high byte)
    tmpsensor_regs[0x19] = 0x6C; // Remote temperature THERM limit
    tmpsensor_regs[0x20] = 0x55; // Local temperature THERM limit
    tmpsensor_regs[0x21] = 0x0A; // THERM hysteresis
    tmpsensor_regs[0x22] = 0x01; // Consecutive ALERT
    tmpsensor_regs[0xFE] = 0x55; // Manufacturer ID

    // Set temp based on hardware, ignoring low values. local = 32c, remote = 33c.
    tmpsensor_regs[0x00] = 32; // Local Temperature High Byte Register
    tmpsensor_regs[0x01] = 33; // Remote Temperature High Byte Register
    tmpsensor_regs[0x10] = 0x00; // Remote Temperature Low Byte Register
    tmpsensor_regs[0x15] = 0x00; // Local Temperature Low Byte Register

    dummyi2c_set_regs(tegra_i2c_tmpsensor_dev, tmpsensor_regs, sizeof(tmpsensor_regs), sizeof(uint8_t), sizeof(tmpsensor_regs));

    /* Alc5639 (Audio Codec) */
    tegra_i2c_audio_codec_dev = i2c_slave_create_simple(tegra_i2c_get_bus(tegra_idc1_dev), "dummyi2c", 0x1C);

    /* Max77620Rtc */
    tegra_i2c_rtc_dev = i2c_slave_create_simple(tegra_i2c_get_bus(tegra_idc5_dev), "max77x", 0x68);

    /* Max77620Pmic */
    tegra_i2c_pmic_dev = i2c_slave_create_simple(tegra_i2c_get_bus(tegra_idc5_dev), "max77xpmic", 0x3C);

    /* Max77621Cpu */
    tegra_i2c_subpmic_devs[0] = i2c_slave_create_simple(tegra_i2c_get_bus(tegra_idc5_dev), "max77xpmic", 0x1B);

    /* Max77621Gpu */
    tegra_i2c_subpmic_devs[1] = i2c_slave_create_simple(tegra_i2c_get_bus(tegra_idc5_dev), "max77xpmic", 0x1C);

    /* Max77812Pmic */
    tegra_i2c_subpmic_devs[2] = i2c_slave_create_simple(tegra_i2c_get_bus(tegra_idc5_dev), "max77xpmic", 0x31);
    tegra_i2c_subpmic_devs[3] = i2c_slave_create_simple(tegra_i2c_get_bus(tegra_idc5_dev), "max77xpmic", 0x33);

    /* Bq24193 */
    tegra_i2c_charger_dev = i2c_slave_create_simple(tegra_i2c_get_bus(tegra_idc1_dev), "dummyi2c", 0x6B);

    uint8_t charger_regs[0xB]={}; // Power-on reset values.
    charger_regs[0x0] = 0x30; // Input Source Control Register
    charger_regs[0x1] = 0x1B; // Power-On Configuration Register
    charger_regs[0x2] = 0x60; // Charge Current Control Register
    charger_regs[0x3] = 0x3; // Pre-Charge/Termination Current Control Register
    charger_regs[0x4] = 0xB2; // Charge Voltage Control Register
    charger_regs[0x5] = 0x9A; // Charge Termination/Timer Control Register
    charger_regs[0x6] = 0x03; // IR Compensation / Thermal Regulation Control Register
    charger_regs[0x7] = 0x4B; // Misc Operation Control Register
    charger_regs[0x8] = 0x00; // System Status Register
    charger_regs[0x8] |= BIT(2) | (3<<4) | (2<<6); // PG_STAT = Power Good, CHRG_STAT = Charge Termination Done, VBUS_STAT = Adapter port.
    charger_regs[0x9] = 0x00; // Fault Register
    charger_regs[0xA] = 0x2F; // Vender / Part / Revision Status Register
    dummyi2c_set_regs(tegra_i2c_charger_dev, charger_regs, sizeof(charger_regs), sizeof(uint8_t), sizeof(charger_regs));

    /* Fuel Gauge */
    tegra_i2c_fuel_dev = i2c_slave_create_simple(tegra_i2c_get_bus(tegra_idc1_dev), "dummyi2c", 0x36);

    uint16_t fuel_regs[0x100]={}; // Power-on reset values.
    fuel_regs[0x00] = 0x0002; // Status
    fuel_regs[0x01] = 0xFF00; // VALRT Threshold
    fuel_regs[0x02] = 0x7F80; // TALRT Threshold
    fuel_regs[0x03] = 0xFF00; // SALRT Threshold
    fuel_regs[0x04] = 0x0000; // AtRate
    fuel_regs[0x05] = 0x03E8; // RemCapREP
    fuel_regs[0x06] = 0x3200; // SOCREP
    fuel_regs[0x07] = 0x6400; // Age
    fuel_regs[0x08] = 0x1600; // Temperature
    fuel_regs[0x09] = 0xB400; // VCELL
    fuel_regs[0x0A] = 0x0000; // Current
    fuel_regs[0x0B] = 0x0000; // AverageCurrent
    fuel_regs[0x0D] = 0x3200; // SOCMIX
    fuel_regs[0x0E] = 0x3200; // SOCAV
    fuel_regs[0x0F] = 0x03E8; // RemCapMIX
    fuel_regs[0x10] = 0x07D0; // FullCAP
    fuel_regs[0x11] = 0x0000; // TTE
    fuel_regs[0x12] = 0x1E2F; // QResidual 00
    fuel_regs[0x13] = 0x4600; // FullSOCThr
    fuel_regs[0x16] = 0x1600; // AverageTemperature
    fuel_regs[0x17] = 0x0000; // Cycles
    fuel_regs[0x18] = 0x07D0; // DesignCap
    fuel_regs[0x19] = 0xB400; // AverageVCELL
    fuel_regs[0x1A] = 0x807F; // MaxMinTemperature
    fuel_regs[0x1B] = 0x00FF; // MaxMinVCELL
    fuel_regs[0x1C] = 0x807F; // MaxMinCurrent
    fuel_regs[0x1D] = 0x2350; // CONFIG
    fuel_regs[0x1E] = 0x03C0; // ICHGTerm
    fuel_regs[0x1F] = 0x03E8; // RemCapAV
    fuel_regs[0x21] = 0x00AC; // Version
    fuel_regs[0x22] = 0x1E00; // QResidual
    fuel_regs[0x23] = 0x07D0; // FullCapNom
    fuel_regs[0x24] = 0x1400; // TempNom
    fuel_regs[0x25] = 0x2305; // TempLim
    fuel_regs[0x27] = 0x88D0; // AIN
    fuel_regs[0x28] = 0x2602; // LearnCFG
    fuel_regs[0x29] = 0x4EA4; // FilterCFG
    fuel_regs[0x2A] = 0x203B; // RelaxCFG
    fuel_regs[0x2B] = 0x0870; // MiscCFG
    fuel_regs[0x2C] = 0xE3E1; // TGAIN
    fuel_regs[0x2D] = 0x290E; // TOFF
    fuel_regs[0x2E] = 0x4000; // CGAIN
    fuel_regs[0x2F] = 0x0000; // COFF
    fuel_regs[0x32] = 0x1306; // QResidual 20
    fuel_regs[0x36] = 0x0780; // Iavg_empty
    fuel_regs[0x37] = 0x05E0; // FCTC
    fuel_regs[0x38] = 0x004B; // RCOMP0
    fuel_regs[0x39] = 0x262B; // TempCo
    fuel_regs[0x3A] = 0x9C5C; // V_empty
    fuel_regs[0x3D] = 0x0001; // FSTAT
    fuel_regs[0x3E] = 0x0000; // TIMER
    fuel_regs[0x3F] = 0xE000; // SHDNTIMER
    fuel_regs[0x42] = 0x0C00; // QResidual 30
    fuel_regs[0x45] = 0x007D; // dQacc
    fuel_regs[0x46] = 0x0C80; // dPacc
    fuel_regs[0x4D] = 0x0000; // QH
    fuel_regs[0xFB] = 0x0000; // VFOCV
    fuel_regs[0xFF] = 0x0000; // SOCVF

    // Values from hardware.
    fuel_regs[0x05] = 0x22F3; // RemCapREP
    fuel_regs[0x06] = 0x63AB; // SOCREP
    fuel_regs[0x07] = 0x6018; // Age
    fuel_regs[0x08] = 0x18FF; // Temperature
    fuel_regs[0x09] = 0xD274; // VCELL
    fuel_regs[0x0A] = 0x04FB; // Current
    fuel_regs[0x0B] = 0x0537; // AverageCurrent
    fuel_regs[0x0E] = 0x63EA; // SOCAV
    fuel_regs[0x0F] = 0x230A; // RemCapMIX
    fuel_regs[0x10] = 0x2311; // FullCAP
    fuel_regs[0x11] = 0xFFFF; // TTE
    fuel_regs[0x13] = 0x5F00; // FullSOCThr
    fuel_regs[0x16] = 0x1877; // AverageTemperature
    fuel_regs[0x17] = 0x00F9; // Cycles
    fuel_regs[0x18] = 0x2476; // DesignCap
    fuel_regs[0x19] = 0xD25E; // AverageVCELL
    fuel_regs[0x1B] = 0xD37F; // MaxMinVCELL
    fuel_regs[0x1E] = 0x0333; // ICHGTerm
    fuel_regs[0x1F] = 0x22F3; // RemCapAV
    fuel_regs[0x27] = 0x8600; // AIN
    fuel_regs[0x3A] = 0xA05F; // V_empty
    fuel_regs[0xFB] = 0xD031; // VFOCV
    fuel_regs[0xFF] = 0x6289; // SOCVF

    fuel_regs[0x3D] &= ~BIT(0); // FSTAT DNR (Data Not Ready) = 0

    dummyi2c_set_regs(tegra_i2c_fuel_dev, fuel_regs, sizeof(fuel_regs), sizeof(uint16_t), sizeof(fuel_regs)/sizeof(uint16_t));

    /* USB-PD controller */
    tegra_i2c_usbpd_dev = i2c_slave_create_simple(tegra_i2c_get_bus(tegra_idc1_dev), "dummyi2c", 0x18);

    uint16_t usbpd_regs[0x100]={};

    usbpd_regs[0x4E] = 0x0001;

    dummyi2c_set_regs(tegra_i2c_usbpd_dev, usbpd_regs, sizeof(usbpd_regs), sizeof(uint16_t), sizeof(usbpd_regs)/sizeof(uint16_t));

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

    /* VI */
    tegra_vi_dev = qdev_new("tegra.host1x_dummy_module");
    s = SYS_BUS_DEVICE(tegra_vi_dev);
    qdev_prop_set_uint8(DEVICE(tegra_vi_dev), "class_id", 0x30);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_VI_BASE);

    /* Display controllers */
    tegra_dca_dev = sysbus_create_simple("tegra.dc", TEGRA_DISPLAY_BASE,
                                         DIRQ(INT_DISPLAY_GENERAL));

    tegra_dcb_dev = qdev_new("tegra.dc");
    s = SYS_BUS_DEVICE(tegra_dcb_dev);
    qdev_prop_set_uint8(DEVICE(tegra_dcb_dev), "class_id", 0x71);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_DISPLAY2_BASE);
    sysbus_connect_irq(s, 0, DIRQ(INT_DISPLAY_B_GENERAL));

    /* SOR */
    tegra_sor_dev = qdev_new("tegra.host1x_dummy_module");
    s = SYS_BUS_DEVICE(tegra_sor_dev);
    qdev_prop_set_uint8(DEVICE(tegra_sor_dev), "class_id", 0x7B);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_SOR_BASE);

    tegra_sor1_dev = qdev_new("tegra.host1x_dummy_module");
    s = SYS_BUS_DEVICE(tegra_sor1_dev);
    qdev_prop_set_uint8(DEVICE(tegra_sor1_dev), "class_id", 0x7C); // TODO: Is this correct?
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_SOR1_BASE);

    /* DSI */
    tegra_dsi_dev = sysbus_create_simple("tegra.dsi", TEGRA_DSI_BASE,
                                         NULL);

    tegra_dsib_dev = qdev_new("tegra.dsi");
    s = SYS_BUS_DEVICE(tegra_dsib_dev);
    qdev_prop_set_uint8(DEVICE(tegra_dsib_dev), "class_id", 0x7A);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_DSIB_BASE);

    /* GPU 2d (VIC) */
    tegra_gr2d_dev = sysbus_create_simple("tegra.gr2d", TEGRA_VIC_BASE, NULL); // DIRQ(INT_VIC_GENERAL)

    /* CSI */
    tegra_csi_dev = qdev_new("tegra.host1x_dummy_module");
    s = SYS_BUS_DEVICE(tegra_csi_dev);
    qdev_prop_set_uint8(DEVICE(tegra_csi_dev), "class_id", 0x31); // TODO: Fix this.
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_CSI_BASE);

    /* ISP */
    tegra_isp_dev = qdev_new("tegra.host1x_dummy_module");
    s = SYS_BUS_DEVICE(tegra_isp_dev);
    qdev_prop_set_uint8(DEVICE(tegra_isp_dev), "class_id", 0x32);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_ISP_BASE);

    /* ISPB */
    tegra_ispb_dev = qdev_new("tegra.host1x_dummy_module");
    s = SYS_BUS_DEVICE(tegra_ispb_dev);
    qdev_prop_set_uint8(DEVICE(tegra_ispb_dev), "class_id", 0x34);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_ISPB_BASE);

    /* VII2C */
    tegra_vii2c_dev = qdev_new("tegra.host1x_dummy_module");
    s = SYS_BUS_DEVICE(tegra_vii2c_dev);
    qdev_prop_set_uint8(DEVICE(tegra_vii2c_dev), "class_id", 0x33); // TODO: Fix this.
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_VII2C_BASE);

    /* DPAUX */
    tegra_dpaux_dev = qdev_new("tegra.host1x_dummy_module");
    s = SYS_BUS_DEVICE(tegra_dpaux_dev);
    qdev_prop_set_uint8(DEVICE(tegra_dpaux_dev), "class_id", 0x7D);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_DPAUX_BASE);

    tegra_dpaux1_dev = qdev_new("tegra.host1x_dummy_module");
    s = SYS_BUS_DEVICE(tegra_dpaux1_dev);
    qdev_prop_set_uint8(DEVICE(tegra_dpaux1_dev), "class_id", 0x7E); // TODO: Is this correct?
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_DPAUX1_BASE);

    /* TSEC */
    tegra_tsec_dev = qdev_new("tegra.tsec");
    s = SYS_BUS_DEVICE(tegra_tsec_dev);
    qdev_prop_set_uint8(DEVICE(tegra_tsec_dev), "class_id", 0xE0);
    qdev_prop_set_uint32(DEVICE(tegra_tsec_dev), "engine", TEGRA_TSEC_ENGINE_TSEC);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_TSEC_BASE);
    sysbus_connect_irq(s, 0, DIRQ(INT_TSEC));

    tegra_tsecb_dev = qdev_new("tegra.tsec");
    s = SYS_BUS_DEVICE(tegra_tsecb_dev);
    qdev_prop_set_uint8(DEVICE(tegra_tsecb_dev), "class_id", 0xE1); // Is this correct?
    qdev_prop_set_uint32(DEVICE(tegra_tsecb_dev), "engine", TEGRA_TSEC_ENGINE_TSECB);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_TSECB_BASE);
    sysbus_connect_irq(s, 0, DIRQ(INT_TSECB));

    tegra_nvenc_dev = qdev_new("tegra.tsec");
    s = SYS_BUS_DEVICE(tegra_nvenc_dev);
    qdev_prop_set_uint8(DEVICE(tegra_nvenc_dev), "class_id", 0x21);
    qdev_prop_set_uint32(DEVICE(tegra_nvenc_dev), "engine", TEGRA_TSEC_ENGINE_NVENC);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_NVENC_BASE);
    sysbus_connect_irq(s, 0, DIRQ(INT_NVENC));

    tegra_nvdec_dev = qdev_new("tegra.tsec");
    s = SYS_BUS_DEVICE(tegra_nvdec_dev);
    qdev_prop_set_uint8(DEVICE(tegra_nvdec_dev), "class_id", 0xF0);
    qdev_prop_set_uint32(DEVICE(tegra_nvdec_dev), "engine", TEGRA_TSEC_ENGINE_NVDEC);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_NVDEC_BASE);
    sysbus_connect_irq(s, 0, DIRQ(INT_NVDEC));

    tegra_nvjpg_dev = qdev_new("tegra.tsec");
    s = SYS_BUS_DEVICE(tegra_nvjpg_dev);
    qdev_prop_set_uint8(DEVICE(tegra_nvjpg_dev), "class_id", 0xC0);
    qdev_prop_set_uint32(DEVICE(tegra_nvjpg_dev), "engine", TEGRA_TSEC_ENGINE_NVJPG);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, TEGRA_NVJPG_BASE);
    sysbus_connect_irq(s, 0, DIRQ(INT_NVJPG));

    /* GPU */
    tegra_gpu_dev = sysbus_create_simple("tegra.gpu", TEGRA_GK20A_BAR0_BASE, NULL);

    /* Process generator tag */
    sysbus_create_simple("tegra.pg", 0x60000000, NULL);

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

    /* BPMP address map differs a bit from CCPLEX.  */

    /*memory_region_add_and_init_ram(cop_sysmem, "tegra.cop-ivectors",
                                   0x00000000, 0x40, RW);

    memory_region_add_and_init_ram(cop_sysmem, "tegra.cop-hi-vec",
                                   0xffff0000, SZ_64K, RW);*/

    /*tegra_memory_region_add_alias(cop_sysmem, "tegra.cop-DRAM", sysmem,
                                0x00000040,
                                0x00000040, TEGRA_DRAM_SIZE - 0x40);*/

    tegra_memory_region_add_alias(cop_sysmem, "tegra.cop-IRAM", sysmem,
                                TEGRA_IRAM_BASE,
                                TEGRA_IRAM_BASE, TEGRA_IRAM_SIZE);

    /*tegra_memory_region_add_alias(cop_sysmem, "tegra.cop-GRHOST", sysmem,
                                TEGRA_GRHOST_BASE,
                                TEGRA_GRHOST_BASE, TEGRA_GRHOST_SIZE);*/

    tegra_memory_region_add_alias(cop_sysmem, "tegra.cop-iomirror", sysmem,
                                TEGRA_GRHOST_BASE,
                                TEGRA_GRHOST_BASE, 0x60000000-TEGRA_GRHOST_BASE);

    /*tegra_memory_region_add_alias(cop_sysmem, "tegra.cop-HOST1X", sysmem,
                                TEGRA_HOST1X_BASE,
                                TEGRA_HOST1X_BASE, TEGRA_HOST1X_SIZE);*/

    /*tegra_memory_region_add_alias(cop_sysmem, "tegra.cop-GART", sysmem,
                                TEGRA_GART_BASE,
                                TEGRA_GART_BASE, TEGRA_GART_SIZE);*/

    tegra_memory_region_add_alias(cop_sysmem, "tegra.cop-PPSB", sysmem,
                                0x60000000,
                                0x60000000, SZ_256M);

    tegra_memory_region_add_alias(cop_sysmem, "tegra.cop-APB", sysmem,
                                0x70000000,
                                0x70000000, SZ_256M);

    tegra_memory_region_add_alias(cop_sysmem, "tegra.cop-DRAM UC", sysmem,
                                0x80000000,
                                0x80000000, machine->ram_size);

    /*tegra_memory_region_add_alias(cop_sysmem, "tegra.cop-AHB", sysmem,
                                0xC0000000,
                                0xC0000000, SZ_128M + SZ_1K + SZ_512);*/

    tegra_memory_region_add_alias(cop_sysmem, "tegra.bpmp-IROM_LOVEC", sysmem,
                                BOOTROM_LOVEC_BASE,
                                BOOTROM_LOVEC_BASE, 0x1000);

    tegra_memory_region_add_alias(cop_sysmem, "tegra.cop-IROM", sysmem,
                                TEGRA_IROM_BASE,
                                TEGRA_IROM_BASE, TEGRA_IROM_SIZE);

    /*tegra_memory_region_add_alias(cop_sysmem, "tegra.cop-bootmon", sysmem,
                                BOOTMON_BASE,
                                BOOTMON_BASE, TARGET_PAGE_SIZE);*/

    /*tegra_memory_region_add_alias(cop_sysmem, "tegra.cop-mmu", sysmem,
                                0xF0000000,
                                0xF0000000, SZ_64K);*/

    /* Map 0x2F600000-0x1F600000 to remote device.  */
//     sysbus_create_simple("tegra.remote_mem", 0x2F600000, NULL);
//     tegra_memory_region_add_alias(cop_sysmem, "tegra.cop-remote_mem", sysmem,
//                                 0x2F600000,
//                                 0x2F600000, 0x10000000);

    //cs = qemu_get_cpu(TEGRA_BPMP);
    //cs->as = cop_as;

    /* Override default AS.  */
    /*memory_listener_unregister(&cs->cpu_ases[0].tcg_as_listener);
    cs->cpu_ases[0].as = cop_as;
    memory_listener_register(&cs->cpu_ases[0].tcg_as_listener, cop_as);*/

    //cpu_address_space_init(cs, 0, "tegra.cop-address-space", cop_sysmem);

    /* Setup APE addrspace. */

    /*memory_region_add_and_init_ram(ape_sysmem, "tegra.ape-arom",
                                   0x00000000, SZ_4M, RO);*/

    memory_region_add_and_init_ram(ape_sysmem, "tegra.ape-aram",
                                   0x00400000, SZ_8M, RW);

    /* Cache controller */
    void *cache_dev = qdev_new("l2x0");
    s = SYS_BUS_DEVICE(cache_dev);
    sysbus_realize_and_unref(s, &error_fatal);
    memory_region_add_subregion(ape_sysmem, 0x00C02000, sysbus_mmio_get_region(s, 0));

    /*tegra_memory_region_add_alias(ape_sysmem, "tegra.ape-dram1", sysmem,
                                0x01000000,
                                0x80000000, 0x6F2C0000);*/

    tegra_memory_region_add_alias(ape_sysmem, "tegra.ape-mirror", sysmem,
                                TEGRA_APE_BASE,
                                TEGRA_APE_BASE, (0x702F8000+0x1000)-TEGRA_APE_BASE);

    /*tegra_memory_region_add_alias(ape_sysmem, "tegra.ape-dram2", sysmem,
                                0x70300000,
                                0x80000000 + 0x6F2C0000, 0x8FD00000);*/

    /* A9 (SCU) private memory region */
    tegra_a9mpcore_dev = qdev_new("a9mpcore_priv");
    qdev_prop_set_uint32(tegra_a9mpcore_dev, "num-cpu", 6); // Must be 6 since APE is cpu5.
    qdev_prop_set_uint32(tegra_a9mpcore_dev, "num-irq", 96 + 32);

    A9MPPrivState *a9mpcore = A9MPCORE_PRIV(tegra_a9mpcore_dev);
    SysBusDevice *gicbusdev_ape = SYS_BUS_DEVICE(&a9mpcore->gic);
    //qdev_prop_set_uint32(DEVICE(gicbusdev_ape), "revision", 2);
    qdev_prop_set_uint32(DEVICE(gicbusdev_ape), "cpu-remap0", TEGRA_CCPLEX_CORE3);
    qdev_prop_set_uint32(DEVICE(gicbusdev_ape), "cpu-remap1", TEGRA_ADSP);

    s = SYS_BUS_DEVICE(tegra_a9mpcore_dev);
    sysbus_realize_and_unref(s, &error_fatal);
    memory_region_add_subregion(ape_sysmem, 0x00C00000, sysbus_mmio_get_region(s, 0));

    for (i = 0; i < 2; i++) {
        int cpu_id = TEGRA_ADSP;
        cpudev = DEVICE(qemu_get_cpu(cpu_id));

        if (i==0) {
            sysbus_connect_irq(gicbusdev_ape, i, DIRQ(INT_APE_1));
            continue;
        }
        else
            sysbus_connect_irq(gicbusdev_ape, i, qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));

        sysbus_connect_irq(gicbusdev_ape, i + 2,
                           qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));
        sysbus_connect_irq(gicbusdev_ape, i + 2 * 2,
                           qdev_get_gpio_in(cpudev, ARM_CPU_VIRQ));
        sysbus_connect_irq(gicbusdev_ape, i + 3 * 2,
                           qdev_get_gpio_in(cpudev, ARM_CPU_VFIQ));
    }

    // Mirror AGIC into the above device.
    tegra_memory_region_add_alias(ape_sysmem, "tegra.ape-agic-gicd", ape_sysmem,
                                0x702F8000+0x1000,
                                0x00C00000+0x1000, 0x1000);
    tegra_memory_region_add_alias(sysmem, "tegra.ape-agic-gicd", ape_sysmem,
                                0x702F8000+0x1000,
                                0x00C00000+0x1000, 0x1000);

    tegra_memory_region_add_alias(ape_sysmem, "tegra.ape-agic-gicc", ape_sysmem,
                                0x702F8000+0x2000,
                                0x00C00000+0x100, 0x100);
    tegra_memory_region_add_alias(sysmem, "tegra.ape-agic-gicc", ape_sysmem,
                                0x702F8000+0x2000,
                                0x00C00000+0x100, 0x100);

    // Mirror AMC EVP regs into the exception vectors. (Not sure if correct but whatever)
    tegra_memory_region_add_alias(ape_sysmem, "tegra.ape-evp", sysmem,
                                0x00000000,
                                0x702EF000+0x700, 0x40);

    // Setup APE IRQs.
    s = SYS_BUS_DEVICE(tegra_ape_dev);
    for (i = 0; i < 8; i++)
        sysbus_connect_irq(s, i, qdev_get_gpio_in(DEVICE(gicbusdev_ape), /*32+*/i));

    //cs = qemu_get_cpu(TEGRA_ADSP);
    //cs->as = ape_as;

    /* Override default AS.  */
    /*memory_listener_unregister(&cs->cpu_ases[0].tcg_as_listener);
    cs->cpu_ases[0].as = ape_as;
    memory_listener_register(&cs->cpu_ases[0].tcg_as_listener, ape_as);*/

    //cpu_address_space_init(cs, 0, "tegra.ape-address-space", ape_sysmem);

    //ARM_CPU(cs)->translate_addr = tegra_ape_translate;

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

    for (int i = 0; i < TEGRAX1_NCPUS; i++) {
        tegra_cpu_reset_assert(i);
    }

    qemu_devices_reset(cause);

    tegra_pmc_reset(tegra_pmc_dev, cause);

    // If IROM isn't being run during cold-boot, do the same PMC write which IROM would normally handle.
    if ((state->firmware == NULL || !tegra_evp_is_cold_bpmp_reset_vector_default()) && cause != SHUTDOWN_CAUSE_GUEST_RESET) {
        tegra_pmc_set_crypto_op(0);
    }

    tegra_evp_reset(tegra_evp_dev, cause);

    tegra_fuse_reset(tegra_fuse_dev, cause);

    int cpu_id = state->firmware != NULL || state->bootloader != NULL ? TEGRA_BPMP : TEGRA_CCPLEX_CORE0;
    tegra_cpu_unpowergate(cpu_id);
    tegra_cpu_reset_deassert(cpu_id, 1);
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

