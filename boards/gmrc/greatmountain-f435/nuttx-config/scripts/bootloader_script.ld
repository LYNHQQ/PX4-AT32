/****************************************************************************
 * boards/gmrc/greatmountain-f435/nuttx-config/scripts/bootloader_script.ld
 *
 *   Copyright (C) 2016, 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* The GMRC GREATMOUNTAINF435 has an AT32F435VM with 4032 KiB internal flash
 * and 384 KiB SRAM.
 *
 * The bootloader owns the first 64 KiB of flash (0x08000000), which is the
 * BOOTLOADER_RESERVATION_SIZE used by the bootloader for the firmware size
 * report:
 *
 *   flash  0x08000000-0x0800ffff  64 KiB    bootloader (this image)
 *   flash  0x08010000-0x083edfff  3960 KiB  application
 *   flash  0x083ee000-0x083effff  8 KiB     application parameter pages
 *
 * The application is linked for APP_LOAD_ADDRESS (0x08010000) with its own
 * script.ld, which reserves the same parameter pages at the end of flash.
 */

MEMORY
{
    flash (rx) : ORIGIN = 0x08000000, LENGTH = 64K
    sram (rwx) : ORIGIN = 0x20000000, LENGTH = 384K
}

OUTPUT_ARCH(arm)

ENTRY(__start)		/* treat __start as the anchor for dead code stripping */
EXTERN(_vectors)	/* force the vectors to be included in the output */

/*
 * Ensure that abort() is present in the final object.  The exception handling
 * code pulled in by libgcc.a requires it (and that code cannot be easily avoided).
 */
EXTERN(abort)

SECTIONS
{
	.text : {
		_stext = ABSOLUTE(.);
		*(.vectors)
		. = ALIGN(32);
		*(.text .text.*)
		*(.fixup)
		*(.gnu.warning)
		*(.rodata .rodata.*)
		*(.gnu.linkonce.t.*)
		*(.glue_7)
		*(.glue_7t)
		*(.got)
		*(.gcc_except_table)
		*(.gnu.linkonce.r.*)
		_etext = ABSOLUTE(.);
	} > flash

	/*
	 * Init functions (static constructors and the like)
	 */
	.init_section : {
		_sinit = ABSOLUTE(.);
		KEEP(*(.init_array .init_array.*))
		_einit = ABSOLUTE(.);
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

	/* NuttX copies this group when CONFIG_ARCH_RAMFUNCS is enabled. */
	.ramfunc : ALIGN(8) {
		_sramfuncs = ABSOLUTE(.);
		*(.ramfunc .ramfunc.*)
		. = ALIGN(8);
		_eramfuncs = ABSOLUTE(.);
	} > sram AT > flash
	_framfuncs = LOADADDR(.ramfunc);

	.data : {
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		_edata = ABSOLUTE(.);
	} > sram AT > flash

	.bss : {
		_sbss = ABSOLUTE(.);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(8);
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
