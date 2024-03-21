============
Tegra README
============

Build
=====

.. code-block:: shell

  cd tegra2_qemu
  mkdir build && cd build
  ../configure --target-list=aarch64-softmmu,arm-softmmu --enable-gcrypt
  make # Or ninja

gcrypt is required for using raw RSA messages.

==========
Run Tegra2
==========

.. code-block:: shell

  ./qemu-system-arm -M tegra2-qemu -m 1024 -kernel arch/arm/boot/zImage -dtb arch/arm/boot/dts/tegra20-qemu.dtb --append "earlyprintk=1 console=ttyS0" -serial stdio -net nic,model=lan9118 -net user -device usb-tablet -device usb-kbd

* Specify SD card image path with ``-drive if=sd,file=sd.img``. See QEMU help for the rest of standard cmdline arguments.

* Specify bootloader image path using ``-bootloader`` argument (optional).

* Specify IRAM image path using ``-iram`` argument (optional).

* Kernel image is loaded at fixed address 0x1000000. DTB is appended to the kernel.

Kernel and device-tree are available at `<https://github.com/grate-driver/linux>`_.

U-Boot is available at `<https://github.com/grate-driver/u-boot>`_, use ``qemu_tegra2_defconfig``.

============
Run Tegra X1
============

.. code-block:: shell

  ./qemu-system-aarch64 -machine tegrax1 -m 4G -bootloader {BPMP IRAM bootloader path}

* Use ``-machine tegrax1plus`` for TX1+.

* ``-bios {path}`` can also be used for running the BPMP IROM. The IROM must be unpatched (all IPATCH slots disabled), since IPATCH patching is emulated. If neither are specified, CCPLEX core0 is started in EL3. It's assumed the user loaded the required data, such as with (repeating as needed for each file): ``-device loader,addr={value},{force-raw=true},file={path}``. To set the reset vector for this: ``-global driver=tegra.evp,property=cpu-reset-vector,value={addr}``.

* To override the addr jumped to by the BPMP reset vector: ``-global driver=tegra.evp,property=bpmp-reset-vector,value={addr}`` ``cold-bpmp-reset-vector`` is used for cold (non-guest) reset, ``bpmp-reset-vector`` is used for guest-reset. This can used for booting the bootloader originally from emmc, for example for tegrax1plus: ``... -machine tegrax1plus ... -bootloader {bootloader path} ... -device loader,addr=0x40000464,force-raw=true,file={decrypted BCT} -device loader,addr=0x400002F8,data-len=4,data=0x1 -device loader,addr=0x04000000C,data-len=4,data=0x1 -global driver=tegra.evp,property=cold-bpmp-reset-vector,value=0x40010040`` Adjust reset vector as needed. Additionally, if the ipatch which overrides APB_MISC_PP_STRAPPING_OPT_A_0 is being used the following should be used (needed for warmboot): ``-global driver=tegra.pmc,property=cold-reset-regs,value=0xB14:0x1C00`` (sets the scratch reg used by the ipatch for APB_MISC_PP_STRAPPING_OPT_A_0, this is only needed since IROM wasn't ran for cold-boot)

* To configure the value of PMC registers during cold-boot (non-guest-reset), repeating as needed for each register: ``-global driver=tegra.pmc,property=cold-reset-regs,value={pmc_reg_offset}:{reg_value}``

* To load both IROM and bootloader with BPMP reset for bootloader, while supporting running IROM later for warmboot: `` -bios {path} -bootloader {path} global driver=tegra.evp,property=cold-bpmp-reset-vector,value=0x40010000`` Adjust reset vector as needed. Note that running IROM directly to load bootloader via PMC regs configured above may be preferred, so that various regs are configured properly by IROM.

* The sd -drive index 0-3 corresponds to the SDMMC1-SDMMC4 controllers. To (optionally) attach storage to each controller: ``-drive if=sd,index={0-3},format=raw,file={image path}``
* Use index=3 for emmc, and (for example) 0 for sd. The emmc image must start with the 0x400000-byte BOOT0 and BOOT1 partitions, with the main MMC partition following that at byte-offset 0x800000.

* Each input ``-serial`` argument corresponds to UART-A - UART-D. If UART-A output with stdio instead of the default is wanted, this can be used for example: ``-chardev stdio,id=char0 -serial chardev:char0``

* The fuse-cache can be specified via an input secret. End-of-file == end of fuse regs. Loading cache regs from iopage is also supported. To set fuse-cache: ``-object secret,id=tegra.fuse.cache,file={path}`` To set the fuse-array (size must be <=0x400-bytes): ``-object secret,id=tegra.fuse.array,file={path}`` The fuse-array is loaded after the fuse-cache, this is skipped for guest-reset. For guest-reset, the current fuse-array data for ODM is loaded into fuse-cache.

* SE AES keys can optionally be specified via input secrets if needed. Repeat as needed for each keyslot: ``-object secret,id=se.aeskeyslot{decimal keyslot 0-15},file={path to raw keydata}`` Note that this should also include any keys which would be setup internally by tsec if required.

* The TSEC outdata can optionally be configured with (size <=0x10-bytes): ``-object secret,id=tegra.tsec.outdata,file={path}`` The key used by tsec for PK11 decryption can be set with (should not be used if PK11-decryption isn't used): ``-object secret,id=tegra.tsec.package1_key,file={path}`` tegra.tsec.outdata is also used to set the tsec_key SE keyslot during a TSEC operation. To set the tsec_root_key SE keyslot during a TSEC operation (size <=0x10-bytes): ``-object secret,id=tegra.tsec.tsec_root_key,file={path}``

* To resume after sleep-mode is entered via the PMC reg, use ``system_wakeup`` via the QEMU monitor. This is only available when tegra.evp bpmp-reset-vector is left at the default, and when ``-bios`` is specified.

* The ``-rotate {value}`` option is also supported for rotating the display.

* To configure the value of register APB_MISC_PP_STRAPPING_OPT_A_0 (default is setup for booting from emmc): ``-global driver=tegra.apb_misc,property=pp-strapping-opt-a,value={value}``

* To configure the reset value of GPIO_IN for each bank/port (each bit corresponds to a GPIO pin): ``-global driver=tegra.gpio,property=reset-value-bank{0-7}-port{0-3},value={value}`` This defaults to 0, so make sure to set this as required for your machine (buttons etc).

See also QEMU docs regarding secrets input / cmdline arguments.

===========
QEMU README
===========

QEMU is a generic and open source machine & userspace emulator and
virtualizer.

QEMU is capable of emulating a complete machine in software without any
need for hardware virtualization support. By using dynamic translation,
it achieves very good performance. QEMU can also integrate with the Xen
and KVM hypervisors to provide emulated hardware while allowing the
hypervisor to manage the CPU. With hypervisor support, QEMU can achieve
near native performance for CPUs. When QEMU emulates CPUs directly it is
capable of running operating systems made for one machine (e.g. an ARMv7
board) on a different machine (e.g. an x86_64 PC board).

QEMU is also capable of providing userspace API virtualization for Linux
and BSD kernel interfaces. This allows binaries compiled against one
architecture ABI (e.g. the Linux PPC64 ABI) to be run on a host using a
different architecture ABI (e.g. the Linux x86_64 ABI). This does not
involve any hardware emulation, simply CPU and syscall emulation.

QEMU aims to fit into a variety of use cases. It can be invoked directly
by users wishing to have full control over its behaviour and settings.
It also aims to facilitate integration into higher level management
layers, by providing a stable command line interface and monitor API.
It is commonly invoked indirectly via the libvirt library when using
open source applications such as oVirt, OpenStack and virt-manager.

QEMU as a whole is released under the GNU General Public License,
version 2. For full licensing details, consult the LICENSE file.


Documentation
=============

Documentation can be found hosted online at
`<https://www.qemu.org/documentation/>`_. The documentation for the
current development version that is available at
`<https://www.qemu.org/docs/master/>`_ is generated from the ``docs/``
folder in the source tree, and is built by `Sphinx
<https://www.sphinx-doc.org/en/master/>`_.


Building
========

QEMU is multi-platform software intended to be buildable on all modern
Linux platforms, OS-X, Win32 (via the Mingw64 toolchain) and a variety
of other UNIX targets. The simple steps to build QEMU are:


.. code-block:: shell

  mkdir build
  cd build
  ../configure
  make

Additional information can also be found online via the QEMU website:

* `<https://wiki.qemu.org/Hosts/Linux>`_
* `<https://wiki.qemu.org/Hosts/Mac>`_
* `<https://wiki.qemu.org/Hosts/W32>`_


Submitting patches
==================

The QEMU source code is maintained under the GIT version control system.

.. code-block:: shell

   git clone https://gitlab.com/qemu-project/qemu.git

When submitting patches, one common approach is to use 'git
format-patch' and/or 'git send-email' to format & send the mail to the
qemu-devel@nongnu.org mailing list. All patches submitted must contain
a 'Signed-off-by' line from the author. Patches should follow the
guidelines set out in the `style section
<https://www.qemu.org/docs/master/devel/style.html>`_ of
the Developers Guide.

Additional information on submitting patches can be found online via
the QEMU website

* `<https://wiki.qemu.org/Contribute/SubmitAPatch>`_
* `<https://wiki.qemu.org/Contribute/TrivialPatches>`_

The QEMU website is also maintained under source control.

.. code-block:: shell

  git clone https://gitlab.com/qemu-project/qemu-web.git

* `<https://www.qemu.org/2017/02/04/the-new-qemu-website-is-up/>`_

A 'git-publish' utility was created to make above process less
cumbersome, and is highly recommended for making regular contributions,
or even just for sending consecutive patch series revisions. It also
requires a working 'git send-email' setup, and by default doesn't
automate everything, so you may want to go through the above steps
manually for once.

For installation instructions, please go to

*  `<https://github.com/stefanha/git-publish>`_

The workflow with 'git-publish' is:

.. code-block:: shell

  $ git checkout master -b my-feature
  $ # work on new commits, add your 'Signed-off-by' lines to each
  $ git publish

Your patch series will be sent and tagged as my-feature-v1 if you need to refer
back to it in the future.

Sending v2:

.. code-block:: shell

  $ git checkout my-feature # same topic branch
  $ # making changes to the commits (using 'git rebase', for example)
  $ git publish

Your patch series will be sent with 'v2' tag in the subject and the git tip
will be tagged as my-feature-v2.

Bug reporting
=============

The QEMU project uses GitLab issues to track bugs. Bugs
found when running code built from QEMU git or upstream released sources
should be reported via:

* `<https://gitlab.com/qemu-project/qemu/-/issues>`_

If using QEMU via an operating system vendor pre-built binary package, it
is preferable to report bugs to the vendor's own bug tracker first. If
the bug is also known to affect latest upstream code, it can also be
reported via GitLab.

For additional information on bug reporting consult:

* `<https://wiki.qemu.org/Contribute/ReportABug>`_


ChangeLog
=========

For version history and release notes, please visit
`<https://wiki.qemu.org/ChangeLog/>`_ or look at the git history for
more detailed information.


Contact
=======

The QEMU community can be contacted in a number of ways, with the two
main methods being email and IRC

* `<mailto:qemu-devel@nongnu.org>`_
* `<https://lists.nongnu.org/mailman/listinfo/qemu-devel>`_
* #qemu on irc.oftc.net

Information on additional methods of contacting the community can be
found online via the QEMU website:

* `<https://wiki.qemu.org/Contribute/StartHere>`_
