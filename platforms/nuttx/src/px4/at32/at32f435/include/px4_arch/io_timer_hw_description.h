/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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


#include <px4_arch/io_timer.h>
#include <px4_arch/hw_description.h>
#include <px4_platform_common/constexpr_util.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform/io_timer_init.h>
#include <at32_tim.h>
#include <at32_dma.h>
#include <hardware/at32f43xxx_rcc.h>

static inline constexpr timer_io_channels_t initIOTimerGPIOInOut(Timer::TimerChannel timer, GPIO::GPIOPin pin)
{
	timer_io_channels_t ret{};
	uint32_t gpio_af = GPIO_AF2;
	const uint32_t pin_port = getGPIOPort(pin.port) | getGPIOPin(pin.pin);
	bool valid = false;
	constexpr_assert((unsigned)timer.channel < 4, "Invalid AT32 timer channel");

	// Reference board motor routes, verified against the AT32F435 pinout.
	switch (timer.timer) {
	case Timer::Timer3:
		valid = pin.port == GPIO::PortC && (unsigned)pin.pin == (unsigned)timer.channel + 6;
		break;

	case Timer::Timer4:
		valid = pin.port == GPIO::PortB && (unsigned)pin.pin == (unsigned)timer.channel + 6;
		break;

	default: break;
	}

	constexpr_assert(valid, "Unsupported AT32 timer channel pin");
	ret.gpio_in = gpio_af | GPIO_ALT | GPIO_DRV_MODETATE | GPIO_FLOAT | pin_port;
	ret.gpio_out = gpio_af | GPIO_ALT | GPIO_DRV_MODETATE | GPIO_PUSHPULL | pin_port;
	return ret;
}
#define initIOTimerChannelCapture initIOTimerChannel // alias, used for param metadata generation

static inline constexpr timer_io_channels_t initIOTimerChannel(const io_timers_t io_timers_conf[MAX_IO_TIMERS],
		Timer::TimerChannel timer, GPIO::GPIOPin pin)
{
	timer_io_channels_t ret = initIOTimerGPIOInOut(timer, pin);

	ret.timer_channel = (uint8_t)timer.channel;

	switch (timer.channel) {
	case Timer::Channel1:
		ret.ccr_offset = AT32_GTIM_CCR1_OFFSET;
		ret.masks = GTIM_SR_CC1IF | GTIM_SR_CC1OF;
		break;

	case Timer::Channel2:
		ret.ccr_offset = AT32_GTIM_CCR2_OFFSET;
		ret.masks = GTIM_SR_CC2IF | GTIM_SR_CC2OF;
		break;

	case Timer::Channel3:
		ret.ccr_offset = AT32_GTIM_CCR3_OFFSET;
		ret.masks = GTIM_SR_CC3IF | GTIM_SR_CC3OF;
		break;

	case Timer::Channel4:
		ret.ccr_offset = AT32_GTIM_CCR4_OFFSET;
		ret.masks = GTIM_SR_CC4IF | GTIM_SR_CC4OF;
		break;
	}

	// find timer index
	ret.timer_index = 0xff;
	const uint32_t timer_base = timerBaseRegister(timer.timer);

	for (int i = 0; i < MAX_IO_TIMERS; ++i) {
		if (io_timers_conf[i].base == timer_base) {
			ret.timer_index = i;
			break;
		}
	}

	constexpr_assert(ret.timer_index != 0xff, "Timer not found");

	return ret;
}

static inline constexpr timer_io_channels_t initIOTimerChannelOutputClear(const io_timers_t
		io_timers_conf[MAX_IO_TIMERS], Timer::TimerChannel timer, GPIO::GPIOPin pin)
{
	timer_io_channels_t ret = initIOTimerChannel(io_timers_conf, timer, pin);
	ret.gpio_out |= GPIO_OUTPUT_CLEAR;
	return ret;
}


static inline constexpr io_timers_t initIOTimer(Timer::Timer timer, uint32_t dma_controller = UINT32_MAX)
{
	io_timers_t ret{};
	bool nuttx_config_timer_enabled = false;
	uint32_t dma_request_up = 0;
	uint32_t dma_request_ch1 = 0;

	switch (timer) {
	case Timer::Timer1:
		ret.base = AT32_TMR1_BASE;
		ret.clock_register = AT32_CRM_APB2EN;
		ret.clock_bit = CRM_APB2EN_TMR1EN;
		ret.clock_freq = AT32_APB2_TIM1_CLKIN;
		ret.vectorno = AT32_IRQ_TIM1CC;
		dma_request_up = DMAMUX_TIM1_UP;
		dma_request_ch1 = DMAMUX_TIM1_CH1;
#if defined(CONFIG_AT32_TIM1)
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer2:
		ret.base = AT32_TMR2_BASE;
		ret.clock_register = AT32_CRM_APB1EN;
		ret.clock_bit = CRM_APB1EN_TMR2EN;
		ret.clock_freq = AT32_APB1_TIM2_CLKIN;
		ret.vectorno = AT32_IRQ_TIM2;
		dma_request_up = DMAMUX_TIM2_UP;
		dma_request_ch1 = DMAMUX_TIM2_CH1;
#if defined(CONFIG_AT32_TIM2)
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer3:
		ret.base = AT32_TMR3_BASE;
		ret.clock_register = AT32_CRM_APB1EN;
		ret.clock_bit = CRM_APB1EN_TMR3EN;
		ret.clock_freq = AT32_APB1_TIM3_CLKIN;
		ret.vectorno = AT32_IRQ_TIM3;
		dma_request_up = DMAMUX_TIM3_UP;
		dma_request_ch1 = DMAMUX_TIM3_CH1;
#if defined(CONFIG_AT32_TIM3)
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer4:
		ret.base = AT32_TMR4_BASE;
		ret.clock_register = AT32_CRM_APB1EN;
		ret.clock_bit = CRM_APB1EN_TMR4EN;
		ret.clock_freq = AT32_APB1_TIM4_CLKIN;
		ret.vectorno = AT32_IRQ_TIM4;
		dma_request_up = DMAMUX_TIM4_UP;
		dma_request_ch1 = DMAMUX_TIM4_CH1;
#if defined(CONFIG_AT32_TIM4)
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer5:
		ret.base = AT32_TMR5_BASE;
		ret.clock_register = AT32_CRM_APB1EN;
		ret.clock_bit = CRM_APB1EN_TMR5EN;
		ret.clock_freq = AT32_APB1_TIM5_CLKIN;
		ret.vectorno = AT32_IRQ_TIM5;
		dma_request_up = DMAMUX_TIM5_UP;
		dma_request_ch1 = DMAMUX_TIM5_CH1;
#if defined(CONFIG_AT32_TIM5)
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer8:
		ret.base = AT32_TMR8_BASE;
		ret.clock_register = AT32_CRM_APB2EN;
		ret.clock_bit = CRM_APB2EN_TMR8EN;
		ret.clock_freq = AT32_APB2_TIM8_CLKIN;
		ret.vectorno = AT32_IRQ_TIM8CC;
		dma_request_up = DMAMUX_TIM8_UP;
		dma_request_ch1 = DMAMUX_TIM8_CH1;
#if defined(CONFIG_AT32_TIM8)
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer20:
		ret.base = AT32_TMR20_BASE;
		ret.clock_register = AT32_CRM_APB2EN;
		ret.clock_bit = CRM_APB2EN_TMR20EN;
		ret.clock_freq = AT32_APB2_TIM20_CLKIN;
		ret.vectorno = AT32_IRQ_TIM20CC;
		dma_request_up = DMAMUX_TIM20_UP;
		dma_request_ch1 = DMAMUX_TIM20_CH1;
#if defined(CONFIG_AT32_TIM20)
		nuttx_config_timer_enabled = true;
#endif
		break;

	default:
		constexpr_assert(false, "AT32 IO requires a four-channel timer");
		break;
	}

	constexpr_assert(!nuttx_config_timer_enabled, "IO requires CONFIG_AT32_TIMx disabled");
#if defined(HRT_TIMER)
	constexpr_assert((unsigned)timer != HRT_TIMER, "IO timer is reserved by HRT");
#endif
	constexpr_assert(ret.clock_freq >= 8000000 && ret.clock_freq % 8000000 == 0,
			 "IO timer clock must be a multiple of the OneShot frequency");

	if (dma_controller != UINT32_MAX) {
		constexpr_assert(dma_controller == DMA1 || dma_controller == DMA2, "Invalid AT32 DMA controller");
#if !defined(CONFIG_AT32_DMA1) || !defined(CONFIG_AT32_DMAMUX1)
		constexpr_assert(dma_controller != DMA1, "DShot requires DMA1 and DMAMUX1 enabled");
#endif
#if !defined(CONFIG_AT32_DMA2) || !defined(CONFIG_AT32_DMAMUX2)
		constexpr_assert(dma_controller != DMA2, "DShot requires DMA2 and DMAMUX2 enabled");
#endif
		ret.dshot.dma_map_up = DMAMAP_MAP(dma_controller, dma_request_up);

		// The four channel requests are consecutive in each AT32 timer's DMAMUX map.
		for (unsigned channel = 0; channel < 4; ++channel) {
			ret.dshot.dma_map_ch[channel] = DMAMAP_MAP(dma_controller, dma_request_ch1 + channel);
		}
	}

	return ret;
}
