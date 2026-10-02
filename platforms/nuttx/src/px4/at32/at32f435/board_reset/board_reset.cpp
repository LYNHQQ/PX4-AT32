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
 * @file board_reset.cpp
 * AT32F435 board reset interface.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/shutdown.h>
#include <systemlib/px4_macros.h>
#include <errno.h>
#include <at32_pwr.h>
#include <hardware/at32_rtcc.h>
#include <nuttx/board.h>
#include <arm_internal.h>

#ifdef CONFIG_BOARDCTL_RESET

/****************************************************************************
 * Name: board_configure_reset
 *
 * Description:
 *   Configures the device that maintains the state shared by the
 *   application and boot loader. On this chip that is a redundant
 *   parameter (RTC backup) register, matching the bootloader's
 *   BOOT_RTC_SIGNATURE.
 *
 * Input Parameters:
 *   mode  - The type of reset. See reset_mode_e
 *
 * Returned Value:
 *   0 for Success
 *   1 if invalid argument
 *
 ****************************************************************************/

static const uint32_t modes[] = {
	/*                                      to  tb   */
	/* BOARD_RESET_MODE_CLEAR                5   y   */  0,
	/* BOARD_RESET_MODE_BOOT_TO_BL           0   n   */  0xb007b007,
	/* BOARD_RESET_MODE_BOOT_TO_VALID_APP    0   y   */  0xb0070002,
	/* BOARD_RESET_MODE_CAN_BL               10  n   */  0xb0080000,
	/* BOARD_RESET_MODE_RTC_BOOT_FWOK        0   n   */  0xb0093a26
};

int board_configure_reset(reset_mode_e mode, uint32_t arg)
{
	int rv = -1;

	if (mode < arraySize(modes)) {

		at32_pwr_enablebkp(true);

		arg = mode == BOARD_RESET_MODE_CAN_BL ? arg & ~0xff : 0;
		putreg32(modes[mode] | arg, AT32_RTC_BK0R);

		at32_pwr_enablebkp(false);
		rv = OK;
	}

	return rv;
}

int board_reset(int status)
{
	if (status == REBOOT_TO_BOOTLOADER) {
		board_configure_reset(BOARD_RESET_MODE_BOOT_TO_BL, 0);
	}

#if defined(BOARD_HAS_ON_RESET)
	board_on_reset(status);
#endif

	up_systemreset();
	return 0;
}

#endif // CONFIG_BOARDCTL_RESET
