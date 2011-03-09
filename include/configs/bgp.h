/*
 * (C) Copyright 2000-2005
 * Stefan Roese, DENX Software Engineering, sr@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * board/config.h - configuration options, board specific
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * High Level Configuration Options
 * (easy to change)
 */

#define CONFIG_4xx		1	/* ...member of PPC4xx family	*/
#define CONFIG_440		1	/* This is a PPC440 CPU		*/
#define CONFIG_BGP		1	/* Blue Gene/P (which is a 450) */
#define CONFIG_BOOKE		1	/* BookE processor		*/

//#define CFG_BOARD_ASM_INIT	1
#define CONFIG_BOARD_EARLY_INIT_F	/* Call board_early_init_f	*/
#define CONFIG_BOARD_EARLY_INIT_R	/* Call board_early_init_r	*/
#define CONFIG_MISC_INIT_F		/* Call misc init */
#define CONFIG_MISC_INIT_R             /* Call relocated misc init */
#define CONFIG_LAST_STAGE_INIT  1       /* Call last_stage_init()       */

#define CONFIG_BOARD_ENV_INIT_R	/* Call board_early_init_r	*/
#define CONFIG_SYS_CLK_FREQ	33333333 /* external frequency to pll	*/

#define CONFIG_PREBOOT	"echo;" \
        "test -n $doannounce && run doannounce;" \
        "test -n $doinit && run doinit;" \
        "imi $argscriptaddr && autoscr $argscriptaddr;" \
        "test -n $doconfig && run doconfig;" \
        "test -n $dostartscript && run dostartscript;" \
        "test -n $doramdisk && imi $ramfsaddr && run doramdisk;" \
        "test -n $donoramdisk && imi $ramfsaddr || run donoramdisk;" \
        "test -n $doionode && itest $bgp_isio == 1 && run doionode;" \
        "test -n $doboot && imi $loadaddr && unsilence && run doboot;" \
        "echo dropping to console && unsilence && test -n $donoboot && run donoboot"


#undef	CONFIG_BOOTARGS

#define CFG_ENV_IS_NOWHERE	1
#define CONFIG_BOOTCOMMAND	"bootm $(loadaddr)"
#define CFG_AUTOLOAD		"no"

#define CONFIG_LWIP_TCP         1
#undef  CONFIG_LWIP_TCP

//#define CONFIG_BGP_SYNCTBASES   1
#undef  CONFIG_BGP_SYNCTBASES

#ifndef CONFIG_BGP_KHVMM

#define CONFIG_EXTRA_ENV_SETTINGS	\
          "argscriptaddr=0x100000\0" \
            "fdtbootaddr=0x1000000\0" \
        "startscriptaddr=0x1100000\0" \
   	       "loadaddr=0x10000000\0" \
              "ramfsaddr=0x12000000\0" \
             "initrdaddr=0x20000000\0" \
             "console=bgtty\0" \
        "dosetinitrd=setenv initrd_high $initrdaddr && echo dosetinitrd done\0" \
        "dobgttysettings=setenv bgtty_dest $bgp_ionoderank; setenv bgtty_sendid 0; setenv bgtty_rcvid 0\0" \
        "dofdtrelocate=fdt move $fdtdefaultaddr $fdtbootaddr $fdttotalsize && fdt addr $fdtbootaddr && echo dofdtrelocate done\0" \
        "bootfileload=nfs\0" \
        "nfsroot=172.24.1.1:/bgsys/kittyhawk/ro/aoe-vblade\0" \
	"bootfile=172.24.1.1:/bgsys/kittyhawk/rw/boot/images/default\0" \
        "donfsroot=setenv ramfsaddr - && setenv bootargs console=bgtty0,$bgtty_sendid,$bgtty_rcvid,$bgtty_dest ip=$ipaddr:$serverip:$gatewayip:$netmask::eth0:off ro root=/dev/nfs nfsroot=172.24.1.1:/bgsys/kittyhawk/ro/aoe-vblade,proto=tcp init=/init; echo donfsroot done\0" \
        "doramdisk=setenv doboot run boot; setenv ramfsarg $ramfsaddr && setenv bootargs console=$console,$bgtty_sendid,$bgtty_rcvid,$bgtty_dest ip=$ipaddr:$serverip:$gatewayip:$netmask::eth0:off;echo doramdisk done\0" \
        "donoramdisk=setenv doboot run boot; setenv ramfsarg - && setenv bootargs console=bgcons ip=$ipaddr:$serverip:$gatewayip:$netmask::eth0:off;echo donoramdisk done\0" \
        "boot=bootm ${loadaddr} ${ramfsarg} ${fdtbootaddr}\0" \
        "doinit=run dofdtrelocate && run dosetinitrd && run dobgttysettings && echo doinit done\0" \
        "doionode=setenv doboot run boot; setenv bootargs console=bgcons ip=$bgp_ioeth_ip::$bgp_ioeth_nfsserver:$bgp_ioeth_netmask::eth0:off init=/init khbr=off; echo doionode done\0" \
        "dobgttycons=echo $bgp_rank: switching to bgttyconsole: $bgtty_sendid $bgtty_rcvid $bgtty_dest; setenv stdout bgtty; setenv stdin bgtty\0" \
        "donoboot=sleep 30 && run dobgttycons\0" \
	""

#else
#define CONFIG_EXTRA_ENV_SETTINGS	\
          "argscriptaddr=0x100000\0" \
            "fdtbootaddr=0x1000000\0" \
        "startscriptaddr=0x1100000\0" \
   	       "loadaddr=0x11000000\0" \
              "ramfsaddr=0x12000000\0" \
             "initrdaddr=0x20000000\0" \
             "console=bgtty\0" \
        "dosetinitrd=setenv initrd_high $initrdaddr && echo dosetinitrd done\0" \
        "dobgttysettings=setenv bgtty_dest $bgp_ionoderank; setenv bgtty_sendid 0; setenv bgtty_rcvid 0\0" \
        "dofdtrelocate=fdt move $fdtdefaultaddr $fdtbootaddr $fdttotalsize && fdt addr $fdtbootaddr && echo dofdtrelocate done\0" \
        "bootfileload=nfs\0" \
        "nfsroot=172.24.1.1:/bgsys/kittyhawk/ro/aoe-vblade\0" \
        "bootfile=172.24.1.1:/bgsys/kittyhawk/rw/boot/images/default\0" \
        "donfsroot=setenv ramfsaddr - && setenv bootargs console=bgtty0,$bgtty_sendid,$bgtty_rcvid,$bgtty_dest ip=$ipaddr:$serverip:$gatewayip:$netmask::eth0:off ro root=/dev/nfs nfsroot=172.24.1.1:/bgsys/kittyhawk/ro/aoe-vblade,proto=tcp init=/init; echo donfsroot done\0" \
        "doramdisk=setenv doboot run boot; setenv ramfsarg $ramfsaddr && setenv bootargs console=bgcons ip=$ipaddr:$serverip:$gatewayip:$netmask::eth0:off;echo doramdisk done\0" \
          "donoramdisk=setenv doboot run boot; setenv ramfsarg - && setenv bootargs console=bgcons ip=$ipaddr:$serverip:$gatewayip:$netmask::eth0:off;echo donoramdisk done\0" \
        "boot=echo $bootargs && bootm ${loadaddr} ${ramfsarg} ${fdtbootaddr}\0" \
        "doinit=run dofdtrelocate && run dosetinitrd && run dobgttysettings && echo doinit done\0" \
        "doionode=setenv doboot run boot; setenv bootargs console=bgcons ip=$bgp_ioeth_ip::$bgp_ioeth_nfsserver:$bgp_ioeth_netmask::eth0:off init=/init khbr=off; echo doionode done\0" \
          "dobgttycons=echo $bgp_rank: switching to bgttyconsole: $bgtty_sendid $bgtty_rcvid $bgtty_dest; setenv stdout bgtty; setenv stdin bgtty\0" \
        "donoboot=\0" \
        "printfdt=echo some information about the fdt; echo initial addr: $fdtdefaultaddr, bootaddr: $fdtbootaddr; echo contents:; fdt print /; echo end of fdt\0" \
        "donoboot=\0"  \
	""

#endif

#define CONFIG_BOOTP_MASK        CONFIG_BOOTP_DEFAULT

#if 1
#define CONFIG_BOOTDELAY	-1	/* autoboot disabled		*/
#else
#define CONFIG_BOOTDELAY	10	/* autoboot after 5 seconds	*/
#endif

#define CONFIG_PHY_ADDR		1	/* PHY address			*/

#define CONFIG_NET_MULTI	1
#define CONFIG_NET_RETRY_COUNT	100
#define CFG_RX_ETH_BUFFER	16	/* use 16 rx buffer on 405 emac */

#define CFG_CMD_REMOVE		(CFG_CMD_IMLS | CFG_CMD_FLASH)
#define CONFIG_COMMANDS	       ( (CONFIG_CMD_DFL & ~CFG_CMD_REMOVE) | \
				CFG_CMD_ASKENV	| \
				CFG_CMD_DATE	| \
				CFG_CMD_DHCP	| \
				CFG_CMD_DIAG	| \
				CFG_CMD_ELF	| \
				CFG_CMD_IRQ	| \
				CFG_CMD_NET	| \
				CFG_CMD_NFS	| \
				CFG_CMD_PING	| \
				CFG_CMD_REGINFO | \
				CFG_CMD_SNTP	)

/* this must be included AFTER the definition of CONFIG_COMMANDS (if any) */
#include <cmd_confdefs.h>

#undef CONFIG_WATCHDOG			/* watchdog disabled		*/
#define CONFIG_PANIC_HANG	1
/*
 * Miscellaneous configurable options
 */
#define CFG_LONGHELP			/* undef to save memory		*/
#define CFG_PROMPT	"=>\n"		/* Monitor Command Prompt	*/
#if (CONFIG_COMMANDS & CFG_CMD_KGDB)
#define CFG_CBSIZE	1024		/* Console I/O Buffer Size	*/
#else
#define CFG_CBSIZE	256		/* Console I/O Buffer Size	*/
#endif
#define CFG_PBSIZE (CFG_CBSIZE+sizeof(CFG_PROMPT)+16) /* Print Buffer Size */
#define CFG_MAXARGS	16		/* max number of command args	*/
#define CFG_BARGSIZE	CFG_CBSIZE	/* Boot Argument Buffer Size	*/

#define CFG_MEMTEST_START	0x0400000	/* memtest works on	*/
#define CFG_MEMTEST_END		0x0C00000	/* 4 ... 12 MB in DRAM	*/

/*
 * If CFG_EXT_SERIAL_CLOCK, then the UART divisor is 1.
 * If CFG_405_UART_ERRATA_59, then UART divisor is 31.
 * Otherwise, UART divisor is determined by CPU Clock and CFG_BASE_BAUD value.
 * The Linux BASE_BAUD define should match this configuration.
 *    baseBaud = cpuClock/(uartDivisor*16)
 * If CFG_405_UART_ERRATA_59 and 200MHz CPU clock,
 * set Linux BASE_BAUD to 403200.
 */
#undef	CONFIG_SERIAL_SOFTWARE_FIFO
#undef	CFG_EXT_SERIAL_CLOCK	       /* external serial clock */
#undef	CFG_405_UART_ERRATA_59	       /* 405GP/CR Rev. D silicon */
#define CFG_BASE_BAUD	    691200

/* The following table includes the supported baudrates */
#define CFG_BAUDRATE_TABLE  \
    {300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400}
#define CONFIG_BAUDRATE 300

#define CFG_LOAD_ADDR		0x100000	/* default load address */
#undef CFG_EXTBDINFO			/* To use extended board_into (bd_t) */

#define CFG_HZ		1000		/* decrementer freq: 1 ms ticks */

#define CONFIG_CMDLINE_EDITING	1	/* add command line history	*/
#define CONFIG_LOOPW		1	/* enable loopw command		*/
#define CONFIG_ZERO_BOOTDELAY_CHECK	/* check for keypress on bootdelay==0 */
#define CONFIG_VERSION_VARIABLE 1	/* include version env variable */

/*-----------------------------------------------------------------------
 * I2C stuff
 *-----------------------------------------------------------------------
 */
#undef CONFIG_HARD_I2C			/* I2C with hardware support	*/
#undef	CONFIG_SOFT_I2C			/* I2C bit-banged		*/

/*-----------------------------------------------------------------------
 * PCI stuff
 *-----------------------------------------------------------------------
 */
#undef CONFIG_PCI			/* don't include pci support */

/*
 * Flash
 */
#define CFG_NO_FLASH	1


/*-----------------------------------------------------------------------
 * Start addresses for the final memory configuration
 * (Set up by the startup code)
 * Please note that CFG_SDRAM_BASE _must_ start at 0
 */

#define CFG_BGP_UA_BLIND        0x5             /* 64K Blind Device Normal Area */
#define CFG_BGP_PA_BLIND        0x00000000
#define CFG_BGP_VA_BLIND        0xFFF90000

#define CFG_BGP_UA_BLIND_TRANS  0x5             /* 64K Blind Device Transient Area */
#define CFG_BGP_PA_BLIND_TRANS  0x00010000
#define CFG_BGP_VA_BLIND_TRANS  0xFFFA0000

/* Netbus */
#define CFG_BGP_UA_DMA          0x6             /* 16K DMA - All Groups */
#define CFG_BGP_PA_DMA          0x00000000
#define CFG_BGP_VA_DMA          0xFFFD0000

#define CFG_BGP_UA_TREE0        0x6             /* 1K Tree VC0 FIFOs and Status */
#define CFG_BGP_PA_TREE0        0x10000000
#define CFG_BGP_VA_TREE0        0xFFFDC000

#define CFG_BGP_UA_TREE1        0x6             /* 1K Tree VC1 FIFOs and Status */
#define CFG_BGP_PA_TREE1        0x11000000
#define CFG_BGP_VA_TREE1        0xFFFDD000

#define CFG_BGP_UA_TORUS0       0x6             /* 32K Torus Group 0 FIFOs and Status */
#define CFG_BGP_PA_TORUS0       0x01140000
#define CFG_BGP_VA_TORUS0       0xFFFB0000

#define CFG_BGP_UA_TORUS1       0x6             /* 32K Torus Group 1 FIFOs and Status */
#define CFG_BGP_PA_TORUS1       0x01150000
#define CFG_BGP_VA_TORUS1       0xFFFC0000

/* Devbus */
#define CFG_BGP_UA_UPC          0x7             /* 4K Universal Performance Counters Data */
#define CFG_BGP_PA_UPC          0x10000000
#define CFG_BGP_VA_UPC          0xFFFDA000

#define CFG_BGP_UA_TOMAL        0x7             /* 16K TCP/IP Offload Memory Access */
#define CFG_BGP_PA_TOMAL        0x20000000
#define CFG_BGP_VA_TOMAL        0xFFFD4000

#define CFG_BGP_UA_XEMAC        0x7             /* 4K Ethernet Media Access Controller */
#define CFG_BGP_PA_XEMAC        0x20004000
#define CFG_BGP_VA_XEMAC        0xFFFD8000
  
#define CFG_BGP_UA_DEVBUS       0x7             /* 4K Ethernet devbus registers */
#define CFG_BGP_PA_DEVBUS       0x20005000
#define CFG_BGP_VA_DEVBUS       0xFFFD9000

#define CFG_BGP_UA_BIC          0x7             /* 4K Interrupt Controller */
#define CFG_BGP_PA_BIC          0x30000000
#define CFG_BGP_VA_BIC          0xFFFDE000

#define CFG_BGP_UA_SRAMERR      0x7             /* 1K SRAM Error Status/Counters */
#define CFG_BGP_PA_SRAMERR      0xFFFDFC00
#define CFG_BGP_VA_SRAMERR      0xFFFDFC00

#define CFG_BGP_UA_SRAMECC      0x7             /* 64K Back-door Access to SRAM ECC */
#define CFG_BGP_PA_SRAMECC      0xFFFE0000
#define CFG_BGP_VA_SRAMECC      0xFFFE0000

#define CFG_BGP_UA_LOCKBOX_SUP  0x7             /* 16K LockBox Supervisor */
#define CFG_BGP_PA_LOCKBOX_SUP  0xFFFF0000
#define CFG_BGP_VA_LOCKBOX_SUP  0xFFFF0000

#define CFG_BGP_UA_LOCKBOX_USR  0x7             /* 16K LockBox User */
#define CFG_BGP_PA_LOCKBOX_USR  0xFFFF4000
#define CFG_BGP_VA_LOCKBOX_USR  0xFFFF4000

#define CFG_BGP_UA_SRAM         0x7             /* 32K SRAM */
#define CFG_BGP_PA_SRAM0        0xFFFF8000
#define CFG_BGP_VA_SRAM0        0xFFFF8000
#define CFG_BGP_PA_SRAM1        0xFFFFC000
#define CFG_BGP_VA_SRAM1        0xFFFFC000

/*****/

#define CFG_BOOT_BASE_ADDR	0x00000000
#define CFG_SDRAM_BASE		0x00000000
#define CFG_FLASH_BASE		0xAFFF8000
#define CFG_MONITOR_LEN		(256 * 1024)	/* Reserve 256 kB for Monitor	*/
#define CFG_MALLOC_LEN		(128 * 1024)	/* Reserve 128 kB for malloc()	*/
#define CFG_MONITOR_BASE	TEXT_BASE
#define CFG_PERIPHERAL_BASE	(0xC0000000)	/* fake */
#define CFG_MAILBOX_BASE	0xFFFF8000

#if 0
#define CFG_TREE_CHN0          (0xD0000000)
#define CFG_TREE_CHN1          (0xD0002000)
#define CFG_IRQCTRL_BASE       (0xD0004000)
#else
#define CFG_TREE_CHN0          (0xD3000000)
#define CFG_TREE_CHN1          (0xD3001000)
#define CFG_IRQCTRL_BASE       (0xD0049000)
#endif
#define CONFIG_VERY_BIG_RAM 1

#if 0
#else
#define CFG_TREE_CHN0		(0xD3000000)
#define CFG_TREE_CHN1		(0xD3001000)
#define CFG_IRQCTRL_BASE 	(0xD0049000)
#endif
#define CONFIG_VERY_BIG_RAM 1

/*
 * Define here the location of the environment variables (FLASH or NVRAM).
 * Note: DENX encourages to use redundant environment in FLASH. NVRAM is only
 *	 supported for backward compatibility.
 */
#define CFG_ENV_SIZE		2*4096

/*
 * For booting Linux, the board info and command line data
 * have to be in the first 8 MB of memory, since this is
 * the maximum mapped by the Linux kernel during initialization.
 */
#define CFG_BOOTMAPSZ		(8 << 20)	/* Initial Memory map for Linux */

/*-----------------------------------------------------------------------
 * Cache Configuration
 */
#define CFG_DCACHE_SIZE		16384	/* For AMCC 405 CPUs, older 405 ppc's	*/
					/* have only 8kB, 16kB is save here	*/
#define CFG_CACHELINE_SIZE	32	/* ...			*/
#if (CONFIG_COMMANDS & CFG_CMD_KGDB)
#define CFG_CACHELINE_SHIFT	5	/* log base 2 of the above value	*/
#endif

/*-----------------------------------------------------------------------
 * External peripheral base address
 *-----------------------------------------------------------------------
 */

/*-----------------------------------------------------------------------
 * Definitions for initial stack pointer and data area
 */
//#define CFG_INIT_DCACHE_CS	4	/* use cs # 4 for data cache memory    */

#define CFG_INIT_RAM_ADDR	0x00000000  /* inside of SDRAM			   */
#define CFG_INIT_RAM_END	0x08000000//0x2000	/* End of used area in RAM	       */
#define CFG_GBL_DATA_SIZE      128  /* size in bytes reserved for initial data */
#define CFG_GBL_DATA_OFFSET    (CFG_INIT_RAM_END - CFG_GBL_DATA_SIZE)
#define CFG_INIT_SP_OFFSET	CFG_GBL_DATA_OFFSET

/*
 * Internal Definitions
 *
 * Boot Flags
 */
#define BOOTFLAG_COLD	0x01		/* Normal Power-On: Boot from FLASH	*/
#define BOOTFLAG_WARM	0x02		/* Software reboot			*/


/*
 * VU: helper to make things compile
 */
#define GPIO

/*
 * Miscellaneous configurable options
 */
#define CONFIG_CMDLINE_EDITING	1	/* add command line history	*/
#define	CFG_HUSH_PARSER		1	/* use "hush" command parser	*/
#define	CFG_PROMPT_HUSH_PS2	"==> "  

#define CFG_CONSOLE_INFO_QUIET 1
#define CONFIG_NETCONSOLE 1             /* turn on netconsole support   */


/* pass open firmware flat tree */
#define CONFIG_OF_LIBFDT	1
#undef  CONFIG_OF_FLAT_TREE
#define CONFIG_OF_BOARD_SETUP	1
#undef  CONFIG_OF_HAS_BD_T     
#define CONFIG_OF_HAS_UBOOT_ENV 1

/* maximum size of the flat tree (8K) */
#define OF_FLAT_TREE_MAX_SIZE	8192

/* XXX: needs fix for SMP; see fdt_support.c! */
#define OF_CPU			"PowerPC,450@0"
//#define OF_TBCLK		
//#define OF_STDOUT_PATH		"/soc8360@e0000000/serial@4500"

#define CONFIG_MAX_CPU			4
#define CONFIG_RELEASE_APP_PROCESSORS	1



#endif	/* __CONFIG_H */
