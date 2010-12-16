/*
 * (C) Copyright 2006
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

OUTPUT_ARCH(powerpc)
/*
SEARCH_DIR(/lib); SEARCH_DIR(/usr/lib); SEARCH_DIR(/usr/local/lib); SEARCH_DIR(/usr/local/powerpc-any-elf/lib);

PHDRS
{
	cnstramp_io PT_LOAD ;
	text PT_LOAD ;
	bootpg PT_LOAD ;
	resetvec PT_LOAD ;
	data PT_LOAD ;
}

start = 0xFFFFFFFC ;

*/

PHDRS
{
  sram2 PT_LOAD;
  sram1 PT_LOAD;
  dram  PT_LOAD;
}

SECTIONS
{
  .resetvec 0xFFFFFFFC :
  {
    *(.resetvec)
  } :resetvec =0xffff

  .bootpg 0xFFFFF000 :
  {
    cpu/ppc4xx/start.o	(.bootpg)
  } :bootpg =0xffff

  .resetvec 0xfffffffc :
  {
    *(.resetvec)
  } : sram2 = 0xffff

  .bootpg 0xfffff000 :
  {
    cpu/ppc4xx/start.o  (.bootpg)
  } : sram1 = 0xffff

  /* Read-only sections, merged into text segment: */
  . = + SIZEOF_HEADERS;
  .interp	: { *(.interp) 		} : dram
  .hash		: { *(.hash)		} : dram
  .dynsym	: { *(.dynsym)		} : dram
  .dynstr	: { *(.dynstr)		} : dram
  .rel.text	: { *(.rel.text)	} : dram
  .rela.text	: { *(.rela.text) 	} : dram
  .rel.data	: { *(.rel.data)	} : dram
  .rela.data	: { *(.rela.data) 	} : dram
  .rel.rodata	: { *(.rel.rodata) 	} : dram
  .rela.rodata	: { *(.rela.rodata) 	} : dram
  .rel.got	: { *(.rel.got)		} : dram
  .rela.got	: { *(.rela.got)	} : dram
  .rel.ctors	: { *(.rel.ctors)	} : dram
  .rela.ctors	: { *(.rela.ctors)	} : dram
  .rel.dtors	: { *(.rel.dtors)	} : dram
  .rela.dtors	: { *(.rela.dtors)	} : dram
  .rel.bss	: { *(.rel.bss)		} : dram
  .rela.bss	: { *(.rela.bss)	} : dram
  .rel.plt	: { *(.rel.plt)		} : dram
  .rela.plt	: { *(.rela.plt)	} : dram
  .init		: { *(.init)		} : dram
  .plt		: { *(.plt)		} : dram

  .text :
  {
    cpu/ppc4xx/start.o (.text)
    *(.text)
    *(.fixup)
    *(.got1)
  } : dram

  _etext = .;
  PROVIDE (etext = .);

  .rodata :
  {
    *(.rodata)
    *(.rodata1)
    *(.rodata.str1.4)
  } : dram

  .fini      : { *(.fini)    } : dram = 0
  .ctors     : { *(.ctors)   } : dram
  .dtors     : { *(.dtors)   } : dram

  /* Read-write section, merged into data segment: */
  . = (. + 0x00FF) & 0xFFFFFF00;
  _erotext = .;
  PROVIDE (erotext = .);

  .reloc :
  {
    *(.got)
    _GOT2_TABLE_ = .;
    *(.got2)
    _FIXUP_TABLE_ = .;
    *(.fixup)
  } : dram 
  __got2_entries = (_FIXUP_TABLE_ - _GOT2_TABLE_) >>2;
  __fixup_entries = (. - _FIXUP_TABLE_)>>2;

  .data :
  {
    *(.data)
    *(.data1)
    *(.sdata)
    *(.sdata2)
    *(.dynamic)
    CONSTRUCTORS
  } : dram

  _edata  =  .;
  PROVIDE (edata = .);

  . = .;
  __u_boot_cmd_start = .;
  .u_boot_cmd : { *(.u_boot_cmd) } : dram
  __u_boot_cmd_end = .;

  . = .;
  __start___ex_table = .;
  __ex_table : { *(__ex_table) } : dram
  __stop___ex_table = .;

  . = ALIGN(256);
  __init_begin = .;
  .text.init : { *(.text.init) } : dram
  .data.init : { *(.data.init) } : dram
  . = ALIGN(256);
  __init_end = .;

  __bss_start = .;
  .bss       :
  {
   *(.sbss) *(.scommon)
   *(.dynbss)
   *(.bss)
   *(COMMON)
   . = ALIGN(4); /* make BG's control system happy and size-align BSS */
  } : dram

  _end = . ;
  /*
   * This assertion is valid and should be restored.  For now, however, it
   * seems we need a large tree-packet buffer, and we seem to get away with
   * it because BSS is not zeroed as it should be.
   . = ASSERT (_end < 0x100000, "u-boot is too large to fit into 1MB");
   */
   PROVIDE (end = .);
   .cnstramp_io 0x800000: {
    board/bluegene/bgp/init.o (.cnstramp_io)
  } :cnstramp_io
}
