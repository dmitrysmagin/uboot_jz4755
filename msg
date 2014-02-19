CHANGELOG:    do_bootm_linux() may be uninitialized.  There is no possibility in the
CHANGELOG-before-U-Boot-1.1.5:  - Implement do_bootm_linux
CHANGELOG-before-U-Boot-1.1.5:* Correct header printing for multi-image files in do_bootm()
System.map:8010e220 T do_bootm
System.map:801271f4 T do_bootm_linux
board/esd/pci405/cmd_pci405.c:extern int do_bootm (cmd_tbl_t *, int, int, char *[]);
board/esd/pci405/cmd_pci405.c:			status = do_bootm (cmdtp, 0, 1, local_args);
board/esd/pci405/cmd_pci405.c:		status = do_bootm (cmdtp, 0, 1, local_args);
board/esd/common/cmd_loadpci.c:extern int do_bootm (cmd_tbl_t *, int, int, char *[]);
board/esd/common/cmd_loadpci.c:			do_bootm (cmdtp, 0, 0, NULL);
board/pn62/cmd_pn62.c:extern int do_bootm (cmd_tbl_t *, int, int, char *[]);
board/pn62/cmd_pn62.c:	rcode = do_bootm (cmdtp, 0, 1, local_args);
board/barco/barco.c:	extern int do_bootm (cmd_tbl_t *, int, int, char *[]);
board/barco/barco.c:	do_bootm (NULL,0,2,bootm_args);
common/cmd_net.c:extern int do_bootm (cmd_tbl_t *, int, int, char *[]);
common/cmd_net.c:		rcode = do_bootm (cmdtp, 0, 1, local_args);
common/cmd_bootm.c:static boot_os_Fcn do_bootm_linux;
common/cmd_bootm.c:extern boot_os_Fcn do_bootm_linux;
common/cmd_bootm.c:static boot_os_Fcn do_bootm_netbsd;
common/cmd_bootm.c:static boot_os_Fcn do_bootm_rtems;
common/cmd_bootm.c:static boot_os_Fcn do_bootm_vxworks;
common/cmd_bootm.c:static boot_os_Fcn do_bootm_qnxelf;
common/cmd_bootm.c:static boot_os_Fcn do_bootm_artos;
common/cmd_bootm.c:static boot_os_Fcn do_bootm_lynxkdi;
common/cmd_bootm.c:int do_bootm (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
common/cmd_bootm.c:			do_bootm_linux  (cmdtp, flag, argc, argv,
common/cmd_bootm.c:			do_bootm_netbsd (cmdtp, flag, argc, argv,
common/cmd_bootm.c:			do_bootm_lynxkdi (cmdtp, flag, argc, argv,
common/cmd_bootm.c:			do_bootm_rtems (cmdtp, flag, argc, argv,
common/cmd_bootm.c:			do_bootm_vxworks (cmdtp, flag, argc, argv,
common/cmd_bootm.c:			do_bootm_qnxelf (cmdtp, flag, argc, argv,
common/cmd_bootm.c:			do_bootm_artos  (cmdtp, flag, argc, argv,
common/cmd_bootm.c:		bootm,	CFG_MAXARGS,	1,	do_bootm,
common/cmd_bootm.c:do_bootm_linux (cmd_tbl_t *cmdtp, int flag,
common/cmd_bootm.c:do_bootm_netbsd (cmd_tbl_t *cmdtp, int flag,
common/cmd_bootm.c:do_bootm_artos (cmd_tbl_t *cmdtp, int flag,
common/cmd_bootm.c:do_bootm_rtems (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],
common/cmd_bootm.c:do_bootm_vxworks (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],
common/cmd_bootm.c:do_bootm_qnxelf (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],
common/cmd_bootm.c:do_bootm_lynxkdi (cmd_tbl_t *cmdtp, int flag,
common/cmd_ide.c:		extern int do_bootm (cmd_tbl_t *, int, int, char *[]);
common/cmd_ide.c:		do_bootm (cmdtp, 0, 1, local_args);
Binary file common/cmd_bootm.o matches
common/cmd_doc.c:		extern int do_bootm (cmd_tbl_t *, int, int, char *[]);
common/cmd_doc.c:		do_bootm (cmdtp, 0, 1, local_args);
Binary file common/libcommon.a matches
Binary file common/cmd_net.o matches
Binary file common/cmd_nand.o matches
common/cmd_fdos.c:	extern int do_bootm (cmd_tbl_t *, int, int, char *[]);
common/cmd_fdos.c:	rcode = do_bootm (cmdtp, 0, 1, local_args);
common/cmd_fdc.c:			extern int do_bootm (cmd_tbl_t *, int, int, char *[]);
common/cmd_fdc.c:			do_bootm (cmdtp, 0, 1, local_args);
common/cmd_usb.c:		extern int do_bootm (cmd_tbl_t *, int, int, char *[]);
common/cmd_usb.c:		rcode=do_bootm (cmdtp, 0, 1, local_args);
common/cmd_nand.c:		extern int do_bootm(cmd_tbl_t *, int, int, char *[]);
common/cmd_nand.c:		do_bootm(cmdtp, 0, 1, local_args);
common/cmd_nand.c:		extern int do_bootm (cmd_tbl_t *, int, int, char *[]);
common/cmd_nand.c:		do_bootm (cmdtp, 0, 1, local_args);
common/cmd_scsi.c:		extern int do_bootm (cmd_tbl_t *, int, int, char *[]);
common/cmd_scsi.c:		rcode = do_bootm (cmdtp, 0, 1, local_args);
doc/README.alaska8220:- common/cmd_bootm.c	    added clocks for MPC8220 in do_bootm_linux()
lib_arm/armlinux.c:void do_bootm_linux (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],
lib_avr32/avr32_linux.c:void do_bootm_linux(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],
lib_blackfin/bf533_linux.c:void do_bootm_linux(cmd_tbl_t * cmdtp, int flag, int argc, char *argv[],
lib_i386/i386_linux.c:void do_bootm_linux(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],
lib_m68k/m68k_linux.c:void do_bootm_linux (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[],
lib_microblaze/microblaze_linux.c:void do_bootm_linux(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],
Binary file lib_mips/libmips.a matches
lib_mips/mips_linux.c:void do_bootm_linux (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[],
Binary file lib_mips/mips_linux.o matches
lib_nios/nios_linux.c:void do_bootm_linux(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],
lib_nios2/nios_linux.c:void do_bootm_linux(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],
tags:do_bootm	common/cmd_bootm.c	/^int do_bootm (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])$/;"	f
tags:do_bootm_artos	common/cmd_bootm.c	/^do_bootm_artos (cmd_tbl_t *cmdtp, int flag,$/;"	f	file:
tags:do_bootm_artos	common/cmd_bootm.c	/^static boot_os_Fcn do_bootm_artos;$/;"	v	file:
tags:do_bootm_linux	common/cmd_bootm.c	/^do_bootm_linux (cmd_tbl_t *cmdtp, int flag,$/;"	f	file:
tags:do_bootm_linux	common/cmd_bootm.c	/^static boot_os_Fcn do_bootm_linux;$/;"	v	file:
tags:do_bootm_linux	lib_arm/armlinux.c	/^void do_bootm_linux (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],$/;"	f
tags:do_bootm_linux	lib_avr32/avr32_linux.c	/^void do_bootm_linux(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],$/;"	f
tags:do_bootm_linux	lib_blackfin/bf533_linux.c	/^void do_bootm_linux(cmd_tbl_t * cmdtp, int flag, int argc, char *argv[],$/;"	f
tags:do_bootm_linux	lib_i386/i386_linux.c	/^void do_bootm_linux(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],$/;"	f
tags:do_bootm_linux	lib_m68k/m68k_linux.c	/^void do_bootm_linux (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[],$/;"	f
tags:do_bootm_linux	lib_microblaze/microblaze_linux.c	/^void do_bootm_linux(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],$/;"	f
tags:do_bootm_linux	lib_mips/mips_linux.c	/^void do_bootm_linux (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[],$/;"	f
tags:do_bootm_linux	lib_nios/nios_linux.c	/^void do_bootm_linux(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],$/;"	f
tags:do_bootm_linux	lib_nios2/nios_linux.c	/^void do_bootm_linux(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],$/;"	f
tags:do_bootm_lynxkdi	common/cmd_bootm.c	/^do_bootm_lynxkdi (cmd_tbl_t *cmdtp, int flag,$/;"	f	file:
tags:do_bootm_lynxkdi	common/cmd_bootm.c	/^static boot_os_Fcn do_bootm_lynxkdi;$/;"	v	file:
tags:do_bootm_netbsd	common/cmd_bootm.c	/^do_bootm_netbsd (cmd_tbl_t *cmdtp, int flag,$/;"	f	file:
tags:do_bootm_netbsd	common/cmd_bootm.c	/^static boot_os_Fcn do_bootm_netbsd;$/;"	v	file:
tags:do_bootm_qnxelf	common/cmd_bootm.c	/^do_bootm_qnxelf (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],$/;"	f	file:
tags:do_bootm_qnxelf	common/cmd_bootm.c	/^static boot_os_Fcn do_bootm_qnxelf;$/;"	v	file:
tags:do_bootm_rtems	common/cmd_bootm.c	/^do_bootm_rtems (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],$/;"	f	file:
tags:do_bootm_rtems	common/cmd_bootm.c	/^static boot_os_Fcn do_bootm_rtems;$/;"	v	file:
tags:do_bootm_vxworks	common/cmd_bootm.c	/^do_bootm_vxworks (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[],$/;"	f	file:
tags:do_bootm_vxworks	common/cmd_bootm.c	/^static boot_os_Fcn do_bootm_vxworks;$/;"	v	file:
Binary file u-boot matches
Binary file u-boot-nand.bin matches
Binary file u-boot.bin matches
u-boot.map:                              common/libcommon.a(cmd_bootm.o) (do_bootm_linux)
u-boot.map:                0x000000008010e220                do_bootm
u-boot.map:                0x00000000801271f4                do_bootm_linux
