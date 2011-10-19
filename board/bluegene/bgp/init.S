/*
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

#include <ppc_asm.tmpl>
#include <config.h>

/* General */
#define TLB_VALID   0x00000200

/* Supported page sizes */

#define SZ_1K	    0x00000000
#define SZ_4K	    0x00000010
#define SZ_16K	    0x00000020
#define SZ_64K	    0x00000030
#define SZ_256K	    0x00000040
#define SZ_1M	    0x00000050
#define SZ_8M       0x00000060
#define SZ_16M	    0x00000070
#define SZ_256M	    0x00000090
#define SZ_1G       0x000000a0

/* Storage attributes */
#define SA_W	    0x00000800	    /* Write-through */
#define SA_I	    0x00000400	    /* Caching inhibited */
#define SA_M	    0x00000200	    /* Memory coherence */
#define SA_G	    0x00000100	    /* Guarded */
#define SA_E	    0x00000080	    /* Endian */

/* Access control */
#define AC_X	    0x00000024	    /* Execute */
#define AC_W	    0x00000012	    /* Write */
#define AC_R	    0x00000009	    /* Read */

/* Some handy macros */

#define EPN(e)		((e) & 0xfffffc00)
#define TLB0(epn,sz)	( (EPN((epn)) | (sz) | TLB_VALID ) )
#define TLB1(rpn,erpn)	( ((rpn)&0xfffffc00) | (erpn) )
#define TLB2(a)		( (a)&0x00000fbf )

#define tlbtab_start\
	mflr    r1  ;\
	bl 0f	    ;

#define tlbtab_end\
	.long 0, 0, 0	;   \
0:	mflr    r0	;   \
	mtlr    r1	;   \
	blr		;

#define tlbentry(epn,sz,rpn,erpn,attr)\
	.long TLB0(epn,sz),TLB1(rpn,erpn),TLB2(attr)


/**************************************************************************
 * TLB TABLE
 *
 * This table is used by the cpu boot code to setup the initial tlb
 * entries. Rather than make broad assumptions in the cpu source tree,
 * this table lets each board set things up however they like.
 *
 *  Pointer to the table is returned in r1
 *
 *************************************************************************/

    .section .bootpg,"ax"
    .globl tlbtab

tlbtab:
    tlbtab_start
#ifndef CONFIG_BGP_KHVMM
     /* VU: this is currently unused, the code runs on the microloader mappings */
    tlbentry( CFG_SDRAM_BASE, SZ_1G, CFG_SDRAM_BASE, 0, AC_R|AC_W|AC_X|SA_W)
    tlbentry( CFG_SDRAM_BASE + 0x40000000, SZ_1G, CFG_SDRAM_BASE + 0x40000000, 0, AC_R|AC_W|AC_X|SA_W)
    tlbentry( CFG_SDRAM_BASE + 0x40000000, SZ_1G, CFG_SDRAM_BASE + 0x40000000, 0, AC_R|AC_W|AC_X|SA_W)
    tlbentry( CFG_MAILBOX_BASE, SZ_16K, CFG_MAILBOX_BASE, 0, AC_R|AC_W )
    tlbentry( CFG_MAILBOX_BASE + 0x10000, SZ_16K, CFG_MAILBOX_BASE + 0x10000, 0, AC_R|AC_W )
#else /* CONFIG_BGP_KHVMM */
    tlbentry (CFG_BGP_VA_SRAM0, 	SZ_16K, CFG_BGP_PA_SRAM0, 	CFG_BGP_UA_SRAM, 	AC_R|AC_W|AC_X)
    tlbentry (CFG_BGP_VA_SRAM1, 	SZ_16K, CFG_BGP_PA_SRAM1, 	CFG_BGP_UA_SRAM, 	AC_R|AC_W|AC_X)
    tlbentry (CFG_BGP_VA_BIC,		SZ_4K,  CFG_BGP_PA_BIC,		CFG_BGP_UA_BIC,  	AC_R|AC_W|SA_G|SA_I)
    tlbentry (CFG_BGP_VA_LOCKBOX_SUP,	SZ_16K, CFG_BGP_PA_LOCKBOX_SUP,	CFG_BGP_UA_LOCKBOX_SUP,	AC_R|AC_W|SA_G|SA_I)

    tlbentry (CFG_BGP_VA_LOCKBOX_USR,	SZ_16K, CFG_BGP_PA_LOCKBOX_USR,	CFG_BGP_UA_LOCKBOX_USR,	AC_R|AC_W|SA_G|SA_I)
    tlbentry (CFG_BGP_VA_BLIND,		SZ_64K, CFG_BGP_PA_BLIND,	CFG_BGP_UA_BLIND,	AC_R|AC_W)
    tlbentry (CFG_BGP_VA_BLIND_TRANS,	SZ_64K, CFG_BGP_PA_BLIND_TRANS,	CFG_BGP_UA_BLIND_TRANS,	AC_R|AC_W)
    tlbentry (CFG_BGP_VA_UPC,		SZ_4K,	CFG_BGP_PA_UPC,		CFG_BGP_UA_UPC, 	AC_R|AC_W|SA_G|SA_I)

    tlbentry (CFG_BGP_VA_TREE0,		SZ_1K,	CFG_BGP_PA_TREE0,	CFG_BGP_UA_TREE0,	AC_R|AC_W|SA_G|SA_I)
    tlbentry (CFG_BGP_VA_TREE1,		SZ_1K,	CFG_BGP_PA_TREE1,	CFG_BGP_UA_TREE1,	AC_R|AC_W|SA_G|SA_I)
    tlbentry (CFG_BGP_VA_TORUS0,	SZ_64K,	CFG_BGP_PA_TORUS0,	CFG_BGP_UA_TORUS0,	AC_R|AC_W|SA_G|SA_I)
    tlbentry (CFG_BGP_VA_TORUS1,	SZ_64K,	CFG_BGP_PA_TORUS1,	CFG_BGP_UA_TORUS1,	AC_R|AC_W|SA_G|SA_I)

    tlbentry (CFG_BGP_VA_DMA,		SZ_16K,	CFG_BGP_PA_DMA,		CFG_BGP_UA_DMA,		AC_R|AC_W|SA_G|SA_I)
    tlbentry (CFG_BGP_VA_TOMAL,		SZ_16K, CFG_BGP_PA_TOMAL,	CFG_BGP_UA_TOMAL,	AC_R|AC_W|SA_G|SA_I)
    tlbentry (CFG_BGP_VA_XEMAC,		SZ_4K,	CFG_BGP_PA_XEMAC,	CFG_BGP_UA_XEMAC,	AC_R|AC_W|SA_G|SA_I)
    tlbentry (CFG_BGP_VA_DEVBUS,	SZ_4K,	CFG_BGP_PA_DEVBUS,	CFG_BGP_UA_DEVBUS,	AC_R|AC_W|SA_G|SA_I)

    tlbentry (CFG_BGP_VA_SRAMECC,	SZ_64K, CFG_BGP_PA_SRAMECC,	CFG_BGP_UA_SRAMECC,	AC_R|AC_W|SA_G|SA_I)
    tlbentry (CFG_BGP_VA_SRAMERR,	SZ_1K,	CFG_BGP_PA_SRAMERR,	CFG_BGP_UA_SRAMERR,	AC_R|SA_G|SA_I)

    tlbentry (CFG_SDRAM_BASE,		SZ_1G, 	CFG_SDRAM_BASE,		0,			AC_R|AC_W|AC_X|SA_W)
#endif
    tlbtab_end

/* SMP bring-up */
.section .text
	.global cpu_entry
	.global spin_app_processor
cpu_entry: .long -1
spin_app_processor:
	bl	1f
1:
	mflr	r1
	subi	r1,r1,8
2:
	dcbi	0,r1
	lwz	r2, 0(r1)
	addi	r3,r2,1
	cmpwi	r3,0
	beq	2b
	mtlr	r2
	blr

.section .cnstramp_io,"ax"
	.global cnstramp_io
cnstramp_io:
	b _start_bg
.end
