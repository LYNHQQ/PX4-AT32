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

#pragma once

#include <px4_platform/micro_hal.h>

__BEGIN_DECLS

#include <at32_gpio.h>
#include <at32_i2c.h>
#include <at32_spi.h>
#include <at32_tim.h>

__EXPORT void at32_spiinitialize(void);

#define PX4_SOC_ARCH_ID                         PX4_SOC_ARCH_ID_AT32F435
#define PX4_FLASH_BASE                          AT32_FLASH_BASE
#define PX4_NUMBER_I2C_BUSES                     AT32_NI2C
#define PX4_ADC_INTERNAL_TEMP_SENSOR_CHANNEL     16

/* The 96-bit factory UID is exposed as words in order from MSW to LSW. */
#define PX4_CPU_UUID_BYTE_LENGTH                12
#define PX4_CPU_UUID_WORD32_LENGTH              (PX4_CPU_UUID_BYTE_LENGTH / sizeof(uint32_t))
#define PX4_CPU_MFGUID_BYTE_LENGTH              PX4_CPU_UUID_BYTE_LENGTH
/* The deprecated 64-bit MAVLink UID uses the low 64 bits of the factory UID. */
#define PX4_CPU_UUID_WORD32_UNIQUE_H            2
#define PX4_CPU_UUID_WORD32_UNIQUE_M            1
#define PX4_CPU_UUID_WORD32_UNIQUE_L            0
#define PX4_CPU_UUID_WORD32_FORMAT_SIZE         (PX4_CPU_UUID_WORD32_LENGTH - 1 + (2 * PX4_CPU_UUID_BYTE_LENGTH) + 1)
#define PX4_CPU_MFGUID_FORMAT_SIZE              ((2 * PX4_CPU_MFGUID_BYTE_LENGTH) + 1)

#define PX4_BUS_OFFSET                          0
#define px4_spibus_initialize(bus)              at32_spibus_initialize(bus)
#define px4_i2cbus_initialize(bus)               at32_i2cbus_initialize(bus)
#define px4_i2cbus_uninitialize(dev)             at32_i2cbus_uninitialize(dev)

#define px4_arch_configgpio(pinset)             at32_configgpio(pinset)
#define px4_arch_unconfiggpio(pinset)           at32_unconfiggpio(pinset)
#define px4_arch_gpioread(pinset)               at32_gpioread(pinset)
#define px4_arch_gpiowrite(pinset, value)        at32_gpiowrite(pinset, value)
#define px4_arch_gpiosetevent(pinset, r, f, e, fp, a) at32_gpiosetevent(pinset, r, f, e, fp, a)

#define PX4_MAKE_GPIO_INPUT(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT | GPIO_PULLUP))
#define PX4_MAKE_GPIO_EXTI(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_EXTI | GPIO_INPUT | GPIO_PULLUP))
#define PX4_MAKE_GPIO_OUTPUT_CLEAR(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_DRV_MODETATE | GPIO_OUTPUT_CLEAR))
#define PX4_MAKE_GPIO_OUTPUT_SET(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_DRV_MODETATE | GPIO_OUTPUT_SET))
#define PX4_GPIO_PIN_OFF(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT | GPIO_FLOAT))

#define TIMER_HRT_CYCLES_PER_US                 (AT32_HCLK_FREQUENCY / 1000000)
#define TIMER_HRT_CYCLES_PER_MS                 (AT32_HCLK_FREQUENCY / 1000)

#define px4_cache_aligned_data()
#define px4_cache_aligned_alloc                 malloc

__END_DECLS
