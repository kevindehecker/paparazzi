/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2016 Kevin van Hecke
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/* The STM32F427 has 2048Kb of FLASH beginning at address 0x0800:0000 and
 * 256Kb of SRAM. SRAM is split up into three blocks:
 *
 * 1) 112Kb of SRAM beginning at address 0x2000:0000
 * 2)  16Kb of SRAM beginning at address 0x2001:c000
 * 3)  64Kb of SRAM beginning at address 0x2002:0000
 * 4)  64Kb of TCM SRAM beginning at address 0x1000:0000
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x0000:0000
 * where the code expects to begin execution by jumping to the entry point in
 * the 0x0800:0000 address range.
 *
 * The first 0x4000 of flash is reserved for the bootloader.
 */

MEMORY
{
    /* disabled due to silicon errata flash (rx)   : ORIGIN = 0x08004000, LENGTH = 2032K */
    flash (rx)   : ORIGIN = 0x08004000, LENGTH = 1008K
    sram (rwx)   : ORIGIN = 0x20000000, LENGTH = 192K
    ccsram (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
}

/* OUTPUT_ARCH(arm) */

/* ENTRY(main) */		/* treat __start as the anchor for dead code stripping */
EXTERN(vector_table)	 /* force the vectors to be included in the output */

/*
 * Ensure that abort() is present in the final object.  The exception handling
 * code pulled in by libgcc.a requires it (and that code cannot be easily avoided).
 */
/* EXTERN(abort) */
/* EXTERN(_bootdelay_signature) */

SECTIONS
{
	.text : {
		_text = ABSOLUTE(.);
		*(.vectors)
                . = ALIGN(32);
                /*
                This signature provides the bootloader with a way to delay booting
                */
                _bootdelay_signature = ABSOLUTE(.);
                 FILL(0xffecc2925d7d05c5)
                . += 8;
		*(.text .text.*)
		*(.fixup)
		*(.gnu.warning)
		*(.rodata .rodata.*)
		*(.gnu.linkonce.t.*)
		*(.got)
		*(.gcc_except_table)
		*(.gnu.linkonce.r.*)
		_etext = ABSOLUTE(.);

		/*
		 * This is a hack to make the newlib libm __errno() call
		 * use the NuttX get_errno_ptr() function.
		 *
		 * __errno = get_errno_ptr;
		 */
	} > flash

	/*
	 * Init functions (static constructors and the like)
	 */
        .init_section : {
                _sinit = ABSOLUTE(.);
                KEEP(*(.init_array .init_array.*))
                _einit = ABSOLUTE(.);
        } > flash

	/* C++ Static constructors/destructors, also used for __attribute__
	 * ((constructor)) and the likes */
	.preinit_array : {
		. = ALIGN(4);
		__preinit_array_start = .;
		KEEP (*(.preinit_array))
		__preinit_array_end = .;
	} >flash
	.init_array : {
		. = ALIGN(4);
		__init_array_start = .;
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array))
		__init_array_end = .;
	} >flash
	.fini_array : {
		. = ALIGN(4);
		__fini_array_start = .;
		KEEP (*(.fini_array))
		KEEP (*(SORT(.fini_array.*)))
		__fini_array_end = .;
	} >flash



	/*
	 * Construction data for parameters.
	 */
	__param ALIGN(4): {
		__param_start = ABSOLUTE(.);
		KEEP(*(__param*))
		__param_end = ABSOLUTE(.);
	} > flash

	.ARM.extab : {
		*(.ARM.extab*)
	} > flash

	__exidx_start = ABSOLUTE(.);
	.ARM.exidx : {
		*(.ARM.exidx*)
	} > flash
	__exidx_end = ABSOLUTE(.);

	_eronly = ABSOLUTE(.);

	.data : {
		_data = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		_edata = ABSOLUTE(.);
	} > sram AT > flash
	_data_loadaddr = LOADADDR(.data);

	.bss : {
		_sbss = ABSOLUTE(.);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		_ebss = ABSOLUTE(.);
	} > sram

	/* Stabs debugging sections. */
	.stab 0 : { *(.stab) }
	.stabstr 0 : { *(.stabstr) }
	.stab.excl 0 : { *(.stab.excl) }
	.stab.exclstr 0 : { *(.stab.exclstr) }
	.stab.index 0 : { *(.stab.index) }
	.stab.indexstr 0 : { *(.stab.indexstr) }
	.comment 0 : { *(.comment) }
	.debug_abbrev 0 : { *(.debug_abbrev) }
	.debug_info 0 : { *(.debug_info) }
	.debug_line 0 : { *(.debug_line) }
	.debug_pubnames 0 : { *(.debug_pubnames) }
	.debug_aranges 0 : { *(.debug_aranges) }
}

PROVIDE(_stack = ORIGIN(sram) + LENGTH(sram));
