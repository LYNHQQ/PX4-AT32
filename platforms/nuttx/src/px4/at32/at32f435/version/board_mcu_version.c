/****************************************************************************
 *
 *   Copyright (C) 2017-2026 PX4 Development Team. All rights reserved.
 *   Author: @author David Sidrane <david_s5@nscdg.com>
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

/**
 * @file board_mcu_version.c
 * AT32F435 silicon version identification.
 */

#include <px4_platform_common/px4_config.h>
#include <arm_internal.h>
#include <hardware/at32_dbgmcu.h>

int board_mcu_version(char *rev, const char **revstr, const char **errata)
{
	uint32_t series = getreg32(AT32_DEBUG_SER_ID) & (0xff << DEBUG_SER_ID_SHIFT);

	if (series != DEBUG_SER_ID_F435) {
		return -1;
	}

	*rev = '?'; // The numeric revision has no documented letter mapping.
	*revstr = "AT32F435";

	/* Not every caller wants the errata string (the logger passes NULL). */

	if (errata) {
		*errata = NULL;
	}

	return (getreg32(AT32_DEBUG_IDCODE) & DEBUG_IDCODE_REVID_MASK) >> DEBUG_IDCODE_REVID_SHIFT;
}
