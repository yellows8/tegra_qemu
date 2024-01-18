tegra2_inc = include_directories('include', 'ahb/host1x/include', '../../display')
tegra2_dep = declare_dependency(include_directories : tegra2_inc)
# tegra2_dep = declare_dependency(include_directories : tegra2_inc, compile_args: ['-DTEGRA_TRACE'])

arm_ss.add(files(
  'axi/emc/emc.c',
  'axi/mc/mc.c',

  'ppsb/arb_sema/arb_sema.c',
  'ppsb/res_sema/res_sema.c',
  'ppsb/ictlr/arb_gnt_ictlr.c',
  'ppsb/ictlr/ictlr.c',
  'ppsb/apb_misc/apb_misc.c',
  'ppsb/car/car.c',
  'ppsb/evp/evp.c',
  'ppsb/flow/flow.c',
  'ppsb/gpio/gpio.c',
  'ppsb/pg/pg.c',
  'ppsb/timer/timer.c',
  'ppsb/timer/timer_us.c',

  'ahb/avpcache/avpcache.c',
  'ahb/apb_dma/apb_dma.c',
  'ahb/bse/bsea.c',
  'ahb/bse/bsev.c',
  'ahb/bse/frameid.c',
  'ahb/bse/mbe.c',
  'ahb/bse/mce.c',
  'ahb/bse/sxe.c',
  'ahb/bse/tfe.c',
  'ahb/bse/vdma.c',
  'ahb/bse/dummy256.c',
  'ahb/bse/dummy768.c',
  'ahb/bse/dummy2k.c',
  'ahb/bse/dummy4k.c',
  'ahb/bse/bse_remote.c',
  'ahb/dma/ahb_dma.c',
  'ahb/gizmo/gizmo.c',
  'ahb/mselect/mselect.c',
  'ahb/dsi/dsi.c',
  'ahb/usb/usb.c',

  'apb/pinmuxaux/pinmuxaux.c',
  'apb/fuse/fuse.c',
  'apb/i2c/i2c.c',
  'apb/pmc/pmc.c',
  'apb/rtc/rtc.c',
  'apb/uart/uart.c',
  'apb/se/se.c',
  'apb/sdmmcvendor/sdmmcvendor.c',
  'apb/mipical/mipical.c',
  'apb/sysctr0/sysctr0.c',

  'cpu/arm_op.c',
  'cpu/cop_mmu.c',
  'cpu/halt.c',
  'cpu/reset.c',

  'remote/remote_io.c',
  'remote/remote_iram.c',
  'remote/remote_mem.c',

  'devices.c',
  'irq_dispatcher.c',
  'tegra2.c',
  'tegrax1.c',
  'trace.c',

  'ahb/host1x/core/cdma/cdma.c',
  'ahb/host1x/core/cdma/cmd_processor.c',

  'ahb/host1x/core/fifo.c',
  'ahb/host1x/core/hwlock.c',

  'ahb/host1x/core/module/irqs.c',
  'ahb/host1x/core/module/mlocks.c',
  'ahb/host1x/core/module/module.c',

  'ahb/host1x/core/syncpts/irqs.c',
  'ahb/host1x/core/syncpts/syncpts.c',
  'ahb/host1x/core/syncpts/waiters.c',

  'ahb/host1x/grhost.c',
  'ahb/host1x/host1x_channel.c',

  'ahb/host1x/modules/dc/registers/color_palette.c',
  'ahb/host1x/modules/dc/registers/color_space.c',
  'ahb/host1x/modules/dc/registers/dc.c',
  'ahb/host1x/modules/dc/registers/digital_vibrance.c',
  'ahb/host1x/modules/dc/registers/horizontal_filtering.c',
  'ahb/host1x/modules/dc/registers/vertical_filtering.c',
  'ahb/host1x/modules/dc/registers/win_common.c',
  'ahb/host1x/modules/dc/dc.c',
  'ahb/host1x/modules/dc/window.c',

  'ahb/host1x/modules/gr2d/gr2d.c',
  'ahb/host1x/modules/gr2d/gr2d_module.c',
  'ahb/host1x/modules/gr2d/engine.c',

  'ahb/host1x/modules/gr3d/gr3d_module.c',

  'ahb/host1x/modules/host1x/host1x.c',
  'ahb/host1x/modules/host1x/host1x_module.c',
), tegra2_dep)
