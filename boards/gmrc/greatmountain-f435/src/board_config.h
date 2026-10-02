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

/**
 * @file board_config.h
 * GREATMOUNTAINF435 board wiring.
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/* LEDs are assumed active low; verify against the board schematic. */
#define GPIO_LED_BLUE  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_DRV_STRONG | GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN15)
#define GPIO_LED_AMBER (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_DRV_STRONG | GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN14)
#define BOARD_OVERLOAD_LED LED_AMBER

/* Parameters use the final two 4 KiB internal flash pages. */
#define FLASH_BASED_PARAMS

#define ADC_CHANNELS ((1 << 8) | (1 << 9))
#define ADC_BATTERY_VOLTAGE_CHANNEL 8
#define ADC_BATTERY_CURRENT_CHANNEL 9
#define BOARD_ADC_POS_REF_V 3.3f

#define DIRECT_PWM_OUTPUT_CHANNELS 8

/* TIM3 and TIM4 are reserved for motor outputs. */
#define HRT_TIMER 5
#define HRT_TIMER_CHANNEL 1

/* The beeper has an active-low input. Leave it inactive at startup. */
#define GPIO_BEEPER_IDLE (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_DRV_STRONG | GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN11)
#define GPIO_TONE_ALARM_IDLE GPIO_BEEPER_IDLE
#define GPIO_TONE_ALARM_GPIO GPIO_BEEPER_IDLE
#define BOARD_TONE_ALARM_ACTIVE_LOW 1

#define BOARD_HAS_ON_RESET 1
#define BOARD_CRASHDUMP_RESET_ONLY
#define BOARD_ENABLE_CONSOLE_BUFFER
#define BOARD_CONSOLE_BUFFER_SIZE (1024 * 3)

/* The generic chip pinmap exposes PA9 as VBUS, but this board uses it for UART1.
 * USB device mode must not read or configure PA9/PA10 as VBUS/OTG ID.
 */
#undef GPIO_OTGFS_VBUS

#if defined(CONFIG_AT32_OTGFS) && (defined(CONFIG_USBDEV_VBUSSENSING) || !defined(CONFIG_OTG_ID_GPIO_DISABLE))
#  error "USB VBUS sensing and OTG ID conflict with UART1"
#endif

#if defined(CONFIG_AT32_USBHOST)
#  error "This board supports USB device mode only"
#endif

__BEGIN_DECLS

#ifndef __ASSEMBLY__
void at32_spiinitialize(void);
void at32_usbinitialize(void);
int board_flash_initialize(void);
void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>
#endif

__END_DECLS
