/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/init.h>

#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <at32.h>
#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>
#include <parameters/flashparams/flashfs.h>
#include <px4_arch/io_timer.h>
#include <systemlib/px4_macros.h>

#include "board_config.h"

__BEGIN_DECLS
void led_on(int led);
void led_off(int led);
__END_DECLS

__EXPORT void board_peripheral_reset(int ms)
{
	/* No sensor power switch is described by the board pin reference. */
	UNUSED(ms);
}

__EXPORT void board_on_reset(int status)
{
	for (unsigned i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
		px4_arch_configgpio(io_timer_channel_get_gpio_output(i));
	}

	if (status >= 0) {
		up_mdelay(400);
	}
}

__EXPORT void at32_boardinitialize(void)
{
	board_on_reset(-1);
	board_autoled_initialize();

	at32_configgpio(GPIO_ADC1_IN8);
	at32_configgpio(GPIO_ADC1_IN9);
	at32_configgpio(GPIO_BEEPER_IDLE);

	/* Initialize chip selects before any SPI device can be accessed. */
	at32_spiinitialize();

#ifdef CONFIG_AT32_OTGFS
	at32_usbinitialize();
#endif
}

__EXPORT int board_app_initialize(uintptr_t arg)
{
	UNUSED(arg);
	px4_platform_init();

	drv_led_start();
	led_off(LED_BLUE);
	led_off(LED_AMBER);

	/* Flash layout must match nuttx-config/scripts/script.ld. */
	static sector_descriptor_t params_sector_map[] = {
		{1006, 4 * 1024, 0x083ee000},
		{1007, 4 * 1024, 0x083ef000},
		{0, 0, 0},
	};

	int ret = parameter_flashfs_init(params_sector_map, NULL, 0);

	if (ret != OK) {
		syslog(LOG_ERR, "Flash parameter initialization failed: %d\n", ret);
		led_on(LED_AMBER);
		return ret;
	}

	px4_platform_configure();

	/* A missing device or an existing non-PX4 filesystem must not prevent boot. */
	if (board_flash_initialize() < 0) {
		led_on(LED_AMBER);
	}

	return OK;
}
