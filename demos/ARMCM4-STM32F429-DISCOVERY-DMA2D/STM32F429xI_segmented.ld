/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * ST32F429xI memory setup.
 */
__main_stack_size__     = 0x0400;
__process_stack_size__  = 0x0400;

MEMORY
{
    flash  : org = 0x08000000, len = 2M
    ccmram : org = 0x10000000, len = 64k
    ram    : org = 0x20000000, len = 112k
    ram2   : org = 0x2001C000, len = 16k
    ram3   : org = 0x20020000, len = 64k
    sdram  : org = 0xD0000000, len = 0x400000 
}

__ram_start__           = ORIGIN(ram);
__ram_size__            = LENGTH(ram);
__ram_end__             = __ram_start__ + __ram_size__;

__sdram_start__         = ORIGIN(sdram);
__sdram_size__          = LENGTH(sdram);
__sdram_end__           = __sdram_start__ + __sdram_size__;

ENTRY(ResetHandler)

SECTIONS
{
    . = 0;
    _text = .;

    startup : ALIGN(16) SUBALIGN(16)
    {
        KEEP(*(vectors))
    } > flash

    constructors : ALIGN(4) SUBALIGN(4)
    {
        PROVIDE(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE(__init_array_end = .);
    } > flash

    destructors : ALIGN(4) SUBALIGN(4)
    {
        PROVIDE(__fini_array_start = .);
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        PROVIDE(__fini_array_end = .);
    } > flash

    .text : ALIGN(16) SUBALIGN(16)
    {
        *(.text.startup.*)
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
    } > flash

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > flash

    .ARM.exidx : {
        PROVIDE(__exidx_start = .);
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        PROVIDE(__exidx_end = .);
     } > flash

    .eh_frame_hdr :
    {
        *(.eh_frame_hdr)
    } > flash

    .eh_frame : ONLY_IF_RO
    {
        *(.eh_frame)
    } > flash
    
    .resources : ALIGN(4) SUBALIGN(4)
    {
    	__resources_base__ = .;
    	KEEP(*(.resources))
    	KEEP(*(.resources.*))
    	__resources_end__ = .;
    } > flash
    
    .textalign : ONLY_IF_RO
    {
        . = ALIGN(8);
    } > flash

    . = ALIGN(4);
    _etext = .;
    _textdata = _etext;

    .stacks :
    {
        . = ALIGN(8);
        __main_stack_base__ = .;
        . += __main_stack_size__;
        . = ALIGN(8);
        __main_stack_end__ = .;
        __process_stack_base__ = .;
        __main_thread_stack_base__ = .;
        . += __process_stack_size__;
        . = ALIGN(8);
        __process_stack_end__ = .;
        __main_thread_stack_end__ = .;
    } > ccmram

    .ccm :
    {
        PROVIDE(_ccm_start = .);
        . = ALIGN(4);
        *(.bss.mainthread.*)
        . = ALIGN(4);
        *(.bss._idle_thread_wa)
        . = ALIGN(4);
        *(.bss.rlist)
        . = ALIGN(4);
        *(.bss.vtlist)
        . = ALIGN(4);
        *(.bss.endmem)
        . = ALIGN(4);
        *(.bss.nextmem)
        . = ALIGN(4);
        *(.bss.default_heap)
        . = ALIGN(4);
        *(.ccm);
        *(.ccm.*);
        . = ALIGN(4);
        PROVIDE(_ccm_end = .);
    } > ccmram

    .ram2 :
    {
        PROVIDE(_ram2_start = .);
        . = ALIGN(4);
        *(.ram2);
        *(.ram2.*);
        . = ALIGN(4);
        PROVIDE(_ram2_end = .);
    } > ram2

    .ram3 :
    {
        PROVIDE(_ram3_start = .);
        . = ALIGN(4);
        *(.ram3);
        *(.ram3.*);
        . = ALIGN(4);
        PROVIDE(_ram3_end = .);
    } > ram3

    .data :
    {
        . = ALIGN(4);
        PROVIDE(_data = .);
        *(.data)
        . = ALIGN(4);
        *(.data.*)
        . = ALIGN(4);
        *(.ramtext)
        . = ALIGN(4);
        PROVIDE(_edata = .);
    } > ram AT > flash

    .bss :
    {
        . = ALIGN(4);
        PROVIDE(_bss_start = .);
        *(.bss)
        . = ALIGN(4);
        *(.bss.*)
        . = ALIGN(4);
        *(COMMON)
        . = ALIGN(4);
        PROVIDE(_bss_end = .);
        PROVIDE(end = .);
        _end = .;
    } > ram
    
    .sdram (NOLOAD) : ALIGN(4) SUBALIGN(4)
    {
    	KEEP(*(.sdram))
    	KEEP(*(.sdram.*))
    } > sdram
}

__heap_base__   = _end;
__heap_end__    = __ram_end__;
