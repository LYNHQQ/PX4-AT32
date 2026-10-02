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

#include <board_config.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <px4_arch/adc.h>
#include <nuttx/mutex.h>
#include <errno.h>
#include <at32_adc.h>
#include <hardware/at32f43xxx_rcc.h>
#include "arm_internal.h"

// Use a conservative ADC clock across the supported supply range.
static constexpr uint32_t ADC_MAX_CLOCK = 36000000;
static constexpr uint32_t ADC_CLOCK_DIVIDER = (AT32_HCLK_FREQUENCY + ADC_MAX_CLOCK - 1) / ADC_MAX_CLOCK;
static constexpr hrt_abstime ADC_INIT_TIMEOUT_US = 1000;
static constexpr hrt_abstime ADC_SAMPLE_TIMEOUT_US = 50;
static constexpr unsigned ADC_MAX_CHANNEL = 18;
static constexpr unsigned ADC_RESOLUTION_BITS = 12;

static_assert(ADC_CLOCK_DIVIDER >= 2 && ADC_CLOCK_DIVIDER <= 17, "AT32 ADC HCLK divider out of range");

static mutex_t adc_mutex = NXMUTEX_INITIALIZER;
static uint32_t initialized[SYSTEM_ADC_COUNT] {};

static bool wait_register(uint32_t address, uint32_t mask, bool set)
{
	const hrt_abstime start = hrt_absolute_time();

	while (((getreg32(address) & mask) != 0) != set) {
		if (hrt_absolute_time() - start > ADC_INIT_TIMEOUT_US) {
			return false;
		}
	}

	return true;
}

int px4_arch_adc_init(uint32_t base_address)
{
	uint32_t clock_bit;

	switch (base_address) {
	case AT32_ADC1_BASE: clock_bit = CRM_APB2EN_ADC1EN; break;

	case AT32_ADC2_BASE: clock_bit = CRM_APB2EN_ADC2EN; break;

	case AT32_ADC3_BASE: clock_bit = CRM_APB2EN_ADC3EN; break;

	default: return -EINVAL;
	}

	int result = nxmutex_lock(&adc_mutex);

	if (result != 0) {
		return result;
	}

	uint32_t *slot = nullptr;

	for (unsigned i = 0; i < SYSTEM_ADC_COUNT; ++i) {
		if (initialized[i] == base_address) {
			nxmutex_unlock(&adc_mutex);
			return OK;
		}

		if (slot == nullptr && initialized[i] == 0) {
			slot = &initialized[i];
		}
	}

	if (slot == nullptr) {
		nxmutex_unlock(&adc_mutex);
		return -ENOMEM;
	}

	modifyreg32(AT32_CRM_APB2EN, 0, clock_bit);
	putreg32(0, base_address + AT32_ADC_CR2_OFFSET);
	putreg32(ADC_CR1_RES_12BIT, base_address + AT32_ADC_CR1_OFFSET);

	// AT32 divides HCLK, and ADCPRE occupies four bits (the NuttX mask has two).
	constexpr uint32_t divider_mask = 15u << ADC_CCR_ADCPRE_SHIFT;
	constexpr uint32_t divider = (ADC_CLOCK_DIVIDER - 2) << ADC_CCR_ADCPRE_SHIFT;
	modifyreg32(AT32_ADC_CCR, divider_mask | ADC_CCR_VBATEN, divider | ADC_CCR_TSVREFE);

	// 640.5 cycles allow the internal temperature sensor to settle.
	uint32_t sample_times = 0;

	for (unsigned channel = 0; channel < 10; ++channel) {
		sample_times |= ADC_SMPR_640p5 << (channel * 3);
	}

	putreg32(sample_times, base_address + AT32_ADC_SMPR2_OFFSET);
	sample_times &= (1u << 27) - 1;
	putreg32(sample_times, base_address + AT32_ADC_SMPR1_OFFSET);
	putreg32(0, base_address + AT32_ADC_SQR1_OFFSET);
	putreg32(0, base_address + AT32_ADC_SQR2_OFFSET);
	putreg32(0, base_address + AT32_ADC_SQR3_OFFSET);
	putreg32(0, base_address + AT32_ADC_SR_OFFSET);
	putreg32(ADC_CR2_ADON, base_address + AT32_ADC_CR2_OFFSET);

	result = -ETIMEDOUT;

	if (wait_register(base_address + AT32_ADC_SR_OFFSET, ADC_SR_RDY, true)) {
		modifyreg32(base_address + AT32_ADC_CR2_OFFSET, 0, ADC_CR2_RSTCAL);

		if (wait_register(base_address + AT32_ADC_CR2_OFFSET, ADC_CR2_RSTCAL, false)) {
			modifyreg32(base_address + AT32_ADC_CR2_OFFSET, 0, ADC_CR2_CAL);

			if (wait_register(base_address + AT32_ADC_CR2_OFFSET, ADC_CR2_CAL, false)) {
				modifyreg32(base_address + AT32_ADC_CR2_OFFSET, 0, ADC_CR2_SWSTART);

				if (wait_register(base_address + AT32_ADC_SR_OFFSET, ADC_SR_EOC, true)) {
					(void)getreg32(base_address + AT32_ADC_DR_OFFSET);
					*slot = base_address;
					result = OK;
				}
			}
		}
	}

	if (result != OK) {
		putreg32(0, base_address + AT32_ADC_CR2_OFFSET);
	}

	nxmutex_unlock(&adc_mutex);
	return result;
}

void px4_arch_adc_uninit(uint32_t base_address)
{
	// ADCs are shared with board hardware detection and remain initialized.
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	if ((base_address != AT32_ADC1_BASE && base_address != AT32_ADC2_BASE && base_address != AT32_ADC3_BASE)
	    || channel > ADC_MAX_CHANNEL) {
		return UINT32_MAX;
	}

	const irqstate_t flags = px4_enter_critical_section();
	(void)getreg32(base_address + AT32_ADC_DR_OFFSET);
	putreg32(channel, base_address + AT32_ADC_SQR3_OFFSET);
	modifyreg32(base_address + AT32_ADC_CR2_OFFSET, 0, ADC_CR2_SWSTART);
	const hrt_abstime start = hrt_absolute_time();

	while (!(getreg32(base_address + AT32_ADC_SR_OFFSET) & ADC_SR_EOC)) {
		if (hrt_absolute_time() - start > ADC_SAMPLE_TIMEOUT_US) {
			modifyreg32(base_address + AT32_ADC_CR2_OFFSET, 0, ADC_CR2_ADABRT);
			px4_leave_critical_section(flags);
			return UINT32_MAX;
		}
	}

	const uint32_t result = getreg32(base_address + AT32_ADC_DR_OFFSET);
	px4_leave_critical_section(flags);
	return result;
}

float px4_arch_adc_reference_v()
{
	return BOARD_ADC_POS_REF_V;
}

uint32_t px4_arch_adc_temp_sensor_mask()
{
	return 1u << PX4_ADC_INTERNAL_TEMP_SENSOR_CHANNEL;
}

uint32_t px4_arch_adc_dn_fullcount()
{
	return 1u << ADC_RESOLUTION_BITS;
}
