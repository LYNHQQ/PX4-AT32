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

#include <stdbool.h>
#include <stddef.h>
#include <nuttx/board.h>
#include <at32.h>

#include "board_config.h"

static const uint32_t g_ledmap[] = {
	GPIO_LED_BLUE,
	GPIO_LED_AMBER,
};

__EXPORT void led_init(void)
{
	for (unsigned i = 0; i < sizeof(g_ledmap) / sizeof(g_ledmap[0]); ++i) {
		at32_configgpio(g_ledmap[i]);
	}
}

__EXPORT void led_on(int led)
{
	if (led >= 0 && led < (int)(sizeof(g_ledmap) / sizeof(g_ledmap[0]))) {
		at32_gpiowrite(g_ledmap[led], false);
	}
}

__EXPORT void led_off(int led)
{
	if (led >= 0 && led < (int)(sizeof(g_ledmap) / sizeof(g_ledmap[0]))) {
		at32_gpiowrite(g_ledmap[led], true);
	}
}

__EXPORT void led_toggle(int led)
{
	if (led >= 0 && led < (int)(sizeof(g_ledmap) / sizeof(g_ledmap[0]))) {
		at32_gpiowrite(g_ledmap[led], !at32_gpioread(g_ledmap[led]));
	}
}

__EXPORT void board_autoled_initialize(void)
{
	led_init();
}

__EXPORT void board_autoled_on(int led)
{
	if (led == LED_STACKCREATED) {
		led_on(0);
	}
}

__EXPORT void board_autoled_off(int led)
{
	if (led == LED_STACKCREATED) {
		led_off(0);
	}
}
