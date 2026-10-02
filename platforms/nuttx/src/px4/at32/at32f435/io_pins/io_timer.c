/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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
 * @file io_timer.c
 *
 * Servo driver supporting PWM servos connected to AT32 timer blocks.
 */

#include <px4_platform_common/px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>

#include <px4_arch/io_timer.h>

#include <at32_gpio.h>
#include <at32_tim.h>
#include "arm_internal.h"

static int io_timer_handler0(int irq, void *context, void *arg);
static int io_timer_handler1(int irq, void *context, void *arg);
static int io_timer_handler2(int irq, void *context, void *arg);
static int io_timer_handler3(int irq, void *context, void *arg);
static int io_timer_handler4(int irq, void *context, void *arg);
static int io_timer_handler5(int irq, void *context, void *arg);
static int io_timer_handler6(int irq, void *context, void *arg);
static int io_timer_handler7(int irq, void *context, void *arg);

#define HW_GTIM_CCER_CC1NP GTIM_CCER_CC1NP

#define arraySize(a) (sizeof((a))/sizeof(((a)[0])))

/* If the timer clock source provided as clock_freq is the AT32_APBx_TIMx_CLKIN
 * then configure the timer to free-run at 1MHz.
 * Otherwise, other frequencies are attainable by adjusting .clock_freq accordingly.
 * For instance .clock_freq = 1000000 would set the prescaler to 1.
 * We also allow for overrides here but all timer register usage need to be
 * taken into account
 */
#ifdef CONFIG_BOARD_PWM_FREQ
#define BOARD_PWM_FREQ CONFIG_BOARD_PWM_FREQ
#endif

#if !defined(BOARD_PWM_FREQ)
#define BOARD_PWM_FREQ 1000000
#endif

#if !defined(BOARD_ONESHOT_FREQ)
#define BOARD_ONESHOT_FREQ 8000000
#endif

#define MAX_CHANNELS_PER_TIMER 4

#define _REG32(_base, _reg) (*(volatile uint32_t *)(_base + _reg))
#define REG(_tmr, _reg)     _REG32(io_timers[_tmr].base, _reg)

#define rCR1(_tmr)      REG(_tmr, AT32_GTIM_CR1_OFFSET)
#define rCR2(_tmr)      REG(_tmr, AT32_GTIM_CR2_OFFSET)
#define rSMCR(_tmr)     REG(_tmr, AT32_GTIM_SMCR_OFFSET)
#define rDIER(_tmr)     REG(_tmr, AT32_GTIM_DIER_OFFSET)
#define rSR(_tmr)       REG(_tmr, AT32_GTIM_SR_OFFSET)
#define rEGR(_tmr)      REG(_tmr, AT32_GTIM_EGR_OFFSET)
#define rCCMR1(_tmr)    REG(_tmr, AT32_GTIM_CCMR1_OFFSET)
#define rCCMR2(_tmr)    REG(_tmr, AT32_GTIM_CCMR2_OFFSET)
#define rCCER(_tmr)     REG(_tmr, AT32_GTIM_CCER_OFFSET)
#define rCNT(_tmr)      REG(_tmr, AT32_GTIM_CNT_OFFSET)
#define rPSC(_tmr)      REG(_tmr, AT32_GTIM_PSC_OFFSET)
#define rARR(_tmr)      REG(_tmr, AT32_GTIM_ARR_OFFSET)
#define rCCR1(_tmr)     REG(_tmr, AT32_GTIM_CCR1_OFFSET)
#define rCCR2(_tmr)     REG(_tmr, AT32_GTIM_CCR2_OFFSET)
#define rCCR3(_tmr)     REG(_tmr, AT32_GTIM_CCR3_OFFSET)
#define rCCR4(_tmr)     REG(_tmr, AT32_GTIM_CCR4_OFFSET)
#define rDCR(_tmr)      REG(_tmr, AT32_GTIM_DCR_OFFSET)
#define rBDTR(_tmr)     REG(_tmr, AT32_ATIM_BDTR_OFFSET)

#define GTIM_SR_CCIF (GTIM_SR_CC4IF|GTIM_SR_CC3IF|GTIM_SR_CC2IF|GTIM_SR_CC1IF)
#define GTIM_SR_CCOF (GTIM_SR_CC4OF|GTIM_SR_CC3OF|GTIM_SR_CC2OF|GTIM_SR_CC1OF)

#define CCMR_C1_RESET           0x00ff
#define CCMR_C1_NUM_BITS        8
#define CCER_C1_NUM_BITS        4

#define CCMR_C1_CAPTURE_INIT (GTIM_CCMR_CCS_CCIN1  << GTIM_CCMR1_CC1S_SHIFT) | \
	(GTIM_CCMR_ICPSC_NOPSC << GTIM_CCMR1_IC1PSC_SHIFT) | \
	(GTIM_CCMR_ICF_NOFILT << GTIM_CCMR1_IC1F_SHIFT)

#define CCMR_C1_PWMOUT_INIT (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC1M_SHIFT) | GTIM_CCMR1_OC1PE
#define CCMR_C1_PWMOUT_INVERTED_INIT (GTIM_CCMR_MODE_PWM2 << GTIM_CCMR1_OC1M_SHIFT) | GTIM_CCMR1_OC1PE

#if defined(BOARD_PWM_DRIVE_ACTIVE_LOW)
#define CCER_C1_INIT  (GTIM_CCER_CC1P | GTIM_CCER_CC1E)
#else
#define CCER_C1_INIT  GTIM_CCER_CC1E
#endif

static io_timer_channel_allocation_t channel_allocations[IOTimerChanModeSize] = { UINT16_MAX };

typedef uint8_t io_timer_allocation_t; /* big enough to hold MAX_IO_TIMERS */

static io_timer_channel_allocation_t timer_allocations[MAX_IO_TIMERS] = { };

#if !defined(BOARD_HAS_NO_CAPTURE)

/* Stats and handlers are only useful for Capture */

typedef struct channel_stat_t {
	uint32_t            isr_cout;
	uint32_t            overflows;
} channel_stat_t;

static channel_stat_t io_timer_channel_stats[MAX_TIMER_IO_CHANNELS];

static struct channel_handler_entry {
	channel_handler_t callback;
	void              *context;
} channel_handlers[MAX_TIMER_IO_CHANNELS];
#endif // !defined(BOARD_HAS_NO_CAPTURE)


static int io_timer_handler(uint16_t timer_index)
{
#if !defined(BOARD_HAS_NO_CAPTURE)
	/* Read the count at the time of the interrupt */

	uint16_t count = rCNT(timer_index);

	/* Read the HRT at the time of the interrupt */

	hrt_abstime now = hrt_absolute_time();

	const io_timers_t *tmr = &io_timers[timer_index];

	/* What is pending and enabled? */

	uint16_t statusr = rSR(timer_index);
	uint16_t enabled = rDIER(timer_index) & GTIM_SR_CCIF;

	/* Iterate over the timer_io_channels table */

	uint32_t first_channel_index = io_timers_channel_mapping.element[timer_index].first_channel_index;
	uint32_t last_channel_index = first_channel_index + io_timers_channel_mapping.element[timer_index].channel_count;

	for (unsigned chan_index = first_channel_index; chan_index < last_channel_index; chan_index++) {

		uint16_t masks = timer_io_channels[chan_index].masks;

		/* Do we have an enabled channel */

		if (enabled & masks) {


			if (statusr & masks & GTIM_SR_CCIF) {

				io_timer_channel_stats[chan_index].isr_cout++;

				/* Call the client to read the CCxR etc and clear the CCxIF */

				if (channel_handlers[chan_index].callback) {
					channel_handlers[chan_index].callback(channel_handlers[chan_index].context, tmr,
									      chan_index, &timer_io_channels[chan_index],
									      now, count);
				}
			}

			if (statusr & masks & GTIM_SR_CCOF) {

				/* Error we has a second edge before we cleared CCxR */

				io_timer_channel_stats[chan_index].overflows++;
			}
		}
	}

	/* Clear all the SR bits for interrupt enabled channels only */

	rSR(timer_index) = ~(statusr & (enabled | enabled << 8));
#endif // !defined(BOARD_HAS_NO_CAPTURE)
	return 0;
}

int io_timer_handler0(int irq, void *context, void *arg)
{
	return io_timer_handler(0);
}

int io_timer_handler1(int irq, void *context, void *arg)
{
	return io_timer_handler(1);
}

int io_timer_handler2(int irq, void *context, void *arg)
{
	return io_timer_handler(2);
}

int io_timer_handler3(int irq, void *context, void *arg)
{
	return io_timer_handler(3);
}

int io_timer_handler4(int irq, void *context, void *arg)
{
	return io_timer_handler(4);
}

int io_timer_handler5(int irq, void *context, void *arg)
{
	return io_timer_handler(5);
}

int io_timer_handler6(int irq, void *context, void *arg)
{
	return io_timer_handler(6);
}

int io_timer_handler7(int irq, void *context, void *arg)
{
	return io_timer_handler(7);
}

static inline int validate_timer_index(unsigned timer)
{
	return (timer < MAX_IO_TIMERS && io_timers[timer].base != 0) ? 0 : -EINVAL;
}

int io_timer_allocate_timer(unsigned timer, io_timer_channel_mode_t mode)
{
	if (mode <= IOTimerChanMode_NotUsed || mode >= IOTimerChanModeSize
	    || mode == IOTimerChanMode_PWMIn) {
		return -ENOTSUP;
	}

	int ret = -EINVAL;

	if (validate_timer_index(timer) == 0) {
		// check if timer is unused or already set to the mode we want
		if (timer_allocations[timer] == IOTimerChanMode_NotUsed || timer_allocations[timer] == mode) {
			timer_allocations[timer] = mode;
			ret = 0;

		} else {
			ret = -EBUSY;
		}
	}

	return ret;
}

int io_timer_unallocate_timer(unsigned timer)
{
	int ret = -EINVAL;

	if (validate_timer_index(timer) == 0) {
		timer_allocations[timer] = IOTimerChanMode_NotUsed;
		ret = 0;
	}

	return ret;
}

static inline int channels_timer(unsigned channel)
{
	return timer_io_channels[channel].timer_index;
}

static uint32_t get_timer_channels(unsigned timer)
{
	uint32_t channels = 0;
	static uint32_t channels_cache[MAX_IO_TIMERS] = {0};

	if (validate_timer_index(timer) < 0) {
		return channels;

	} else {
		if (channels_cache[timer] == 0) {
			/* Gather the channel bits that belong to the timer */

			uint32_t first_channel_index = io_timers_channel_mapping.element[timer].first_channel_index;
			uint32_t last_channel_index = first_channel_index + io_timers_channel_mapping.element[timer].channel_count;

			for (unsigned chan_index = first_channel_index; chan_index < last_channel_index; chan_index++) {
				channels |= 1 << chan_index;
			}

			/* cache them */

			channels_cache[timer] = channels;
		}
	}

	return channels_cache[timer];
}

int io_timer_validate_channel_index(unsigned channel)
{
	int rv = -EINVAL;

	if (channel < MAX_TIMER_IO_CHANNELS) {

		unsigned timer = timer_io_channels[channel].timer_index;

		/* test timer for validity */

		if ((validate_timer_index(timer) == 0) &&
		    (timer_io_channels[channel].timer_channel < MAX_CHANNELS_PER_TIMER) &&
		    (timer_io_channels[channel].gpio_out != 0) &&
		    (timer_io_channels[channel].gpio_in != 0)) {
			rv = 0;
		}
	}

	return rv;
}

uint32_t io_timer_channel_get_gpio_output(unsigned channel)
{
	if (io_timer_validate_channel_index(channel) != 0) {
		return 0;
	}

	return (timer_io_channels[channel].gpio_out & (GPIO_PORT_MASK | GPIO_PIN_MASK)) |
	       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_DRV_MODETATE | GPIO_OUTPUT_CLEAR);
}

uint32_t io_timer_channel_get_as_pwm_input(unsigned channel)
{
	if (io_timer_validate_channel_index(channel) != 0) {
		return 0;
	}

	return timer_io_channels[channel].gpio_in;
}

int io_timer_get_mode_channels(io_timer_channel_mode_t mode)
{
	if (mode < IOTimerChanModeSize) {
		return channel_allocations[mode];
	}

	return 0;
}

int io_timer_get_channel_mode(unsigned channel)
{
	if (channel >= MAX_TIMER_IO_CHANNELS) {
		return -EINVAL;
	}

	io_timer_channel_allocation_t bit = 1 << channel;

	for (int mode = IOTimerChanMode_NotUsed; mode < IOTimerChanModeSize; mode++) {
		if (bit & channel_allocations[mode]) {
			return mode;
		}
	}

	return -1;
}

static int reallocate_channel_resources(uint32_t channels, io_timer_channel_mode_t mode,
					io_timer_channel_mode_t new_mode)
{
	/* If caller mode is not based on current setting adjust it */

	if ((channels & channel_allocations[IOTimerChanMode_NotUsed]) == channels) {
		mode = IOTimerChanMode_NotUsed;
	}

	/* Remove old set of channels from original */

	channel_allocations[mode] &= ~channels;

	/* Will this change ?*/

	uint32_t before = channel_allocations[new_mode] & channels;

	/* add in the new set */

	channel_allocations[new_mode] |= channels;

	/* Indicate a mode change */

	return before ^ channels;
}

__EXPORT int io_timer_allocate_channel(unsigned channel, io_timer_channel_mode_t mode)
{
	if (io_timer_validate_channel_index(channel) != 0 || mode <= IOTimerChanMode_NotUsed
	    || mode >= IOTimerChanModeSize || mode == IOTimerChanMode_PWMIn) {
		return -EINVAL;
	}

	irqstate_t flags = px4_enter_critical_section();
	int existing_mode = io_timer_get_channel_mode(channel);
	int ret = -EBUSY;

	if (existing_mode <= IOTimerChanMode_NotUsed || existing_mode == mode) {
		io_timer_channel_allocation_t bit = 1 << channel;
		channel_allocations[IOTimerChanMode_NotUsed] &= ~bit;
		channel_allocations[mode] |= bit;
		ret = 0;
	}

	px4_leave_critical_section(flags);

	return ret;
}


int io_timer_unallocate_channel(unsigned channel)
{
	int mode = io_timer_get_channel_mode(channel);

	if (mode > IOTimerChanMode_NotUsed) {
		io_timer_channel_allocation_t bit = 1 << channel;
		channel_allocations[mode] &= ~bit;
		channel_allocations[IOTimerChanMode_NotUsed] |= bit;
	}

	return mode;
}

static int allocate_channel(unsigned channel, io_timer_channel_mode_t mode)
{
	int rv = -EINVAL;

	if (mode != IOTimerChanMode_NotUsed) {
		rv = io_timer_validate_channel_index(channel);

		if (rv == 0) {
			rv = io_timer_allocate_channel(channel, mode);
		}
	}

	return rv;
}

static int timer_set_rate(unsigned timer, unsigned rate)
{
	if (rate == 0 || rate > BOARD_PWM_FREQ || BOARD_PWM_FREQ / rate > UINT16_MAX + 1u) {
		return -EINVAL;
	}

	/* configure the timer to update at the desired rate */

	rARR(timer) = (BOARD_PWM_FREQ / rate) - 1;

	return 0;
}



static inline void io_timer_set_oneshot_mode(unsigned timer)
{
	/* Use the longest free-running period: 8.192 ms on 16-bit timers,
	 * 536.870912 s on TIM2/TIM5. One-pulse mode stops the whole timer,
	 * so outputs are retriggered together by an update event instead.
	 * Leave the reserved upper ARR bits clear on 16-bit timers.
	 */
	rARR(timer) = (io_timers[timer].base == AT32_TMR2_BASE
		       || io_timers[timer].base == AT32_TMR5_BASE) ? UINT32_MAX : UINT16_MAX;
	rPSC(timer) = (io_timers[timer].clock_freq / BOARD_ONESHOT_FREQ) - 1;
	rEGR(timer) = GTIM_EGR_UG;
}

static dshot_timing_t dshot_timings[MAX_IO_TIMERS];
static unsigned dshot_timing_rate[MAX_IO_TIMERS];

const dshot_timing_t *io_timer_get_dshot_timing(uint8_t timer, unsigned bitrate)
{
	if (validate_timer_index(timer) != 0 || io_timers[timer].clock_freq == 0
	    || (bitrate != 150000 && bitrate != 300000 && bitrate != 600000 && bitrate != 1200000)) {
		return NULL;
	}

	irqstate_t flags = px4_enter_critical_section();

	if (dshot_timing_rate[timer] != bitrate) {
		dshot_timings[timer] = dshot_timing(io_timers[timer].clock_freq, bitrate);
		dshot_timing_rate[timer] = bitrate;
	}

	px4_leave_critical_section(flags);
	return &dshot_timings[timer];
}

void io_timer_update_dma_req(uint8_t timer, bool enable)
{
	if (validate_timer_index(timer) != 0 || io_timers[timer].dshot.dma_map_up == 0) {
		return;
	}

	irqstate_t flags = px4_enter_critical_section();

	if (enable) {
		rDIER(timer) |= GTIM_DIER_UDE;

	} else {
		rDIER(timer) &= ~GTIM_DIER_UDE;
	}

	px4_leave_critical_section(flags);
}

int io_timer_set_dshot_burst_mode(uint8_t timer, unsigned bitrate, uint8_t channel_span)
{
	const dshot_timing_t *timing = io_timer_get_dshot_timing(timer, bitrate);

	if (timing == NULL || io_timers[timer].dshot.dma_map_up == 0
	    || (timer_allocations[timer] != IOTimerChanMode_Dshot
		&& timer_allocations[timer] != IOTimerChanMode_DshotInverted)
	    || channel_span == 0 || channel_span > MAX_CHANNELS_PER_TIMER
	    || io_timers_channel_mapping.element[timer].lowest_timer_channel + channel_span > MAX_CHANNELS_PER_TIMER
	    || timing->prescaler == 0 || timing->prescaler > UINT16_MAX + 1u) {
		return -EINVAL;
	}

	const uint32_t lowest_channel = io_timers_channel_mapping.element[timer].lowest_timer_channel;
	const uint32_t pwm_mode = timer_allocations[timer] == IOTimerChanMode_DshotInverted
				  ? CCMR_C1_PWMOUT_INVERTED_INIT : CCMR_C1_PWMOUT_INIT;
	irqstate_t flags = px4_enter_critical_section();

	// A capture channel may have borrowed another CCR. Restore all four channels
	// before clearing the preload registers, with every output disconnected.
	rDIER(timer) = 0;
	rCR1(timer) = GTIM_CR1_ARPE;
	rCR2(timer) &= ~GTIM_CR2_CCDS;
	rCCER(timer) = 0;
	rCCMR1(timer) = pwm_mode | (pwm_mode << CCMR_C1_NUM_BITS);
	rCCMR2(timer) = pwm_mode | (pwm_mode << CCMR_C1_NUM_BITS);
	rCCR1(timer) = 0;
	rCCR2(timer) = 0;
	rCCR3(timer) = 0;
	rCCR4(timer) = 0;
	rPSC(timer) = timing->prescaler - 1;
	rARR(timer) = timing->ticks - 1;
	rDCR(timer) = ((AT32_GTIM_CCR1_OFFSET / sizeof(uint32_t) + lowest_channel) << GTIM_DCR_DBA_SHIFT)
		      | ((channel_span - 1u) << GTIM_DCR_DBL_SHIFT);
	rEGR(timer) = GTIM_EGR_UG;
	rCNT(timer) = 0;
	rSR(timer) = 0;

	// The backend starts DMA before enabling UDE and CEN. Do not start here.
	px4_leave_critical_section(flags);
	return OK;
}

static int dshot_output_channel(uint8_t timer, uint8_t timer_channel_index)
{
	if (validate_timer_index(timer) != 0 || timer_channel_index >= MAX_CHANNELS_PER_TIMER) {
		return -EINVAL;
	}

	for (unsigned channel = 0; channel < MAX_TIMER_IO_CHANNELS; ++channel) {
		if (timer_io_channels[channel].timer_index == timer
		    && timer_io_channels[channel].timer_channel == timer_channel_index
		    && io_timer_validate_channel_index(channel) == 0) {
			return channel;
		}
	}

	return -EINVAL;
}

int io_timer_get_dshot_capture_channel(uint8_t timer, uint8_t timer_channel_index)
{
	if (dshot_output_channel(timer, timer_channel_index) < 0) {
		return -EINVAL;
	}

	// AT32 channel 4 has no complementary polarity bit. Channel 3's indirect
	// input selects the same physical TI4 signal and supports both-edge capture.
	const uint8_t capture_channel = timer_channel_index == 3 ? 2 : timer_channel_index;
	return io_timers[timer].dshot.dma_map_ch[capture_channel] != 0 ? capture_channel : -EINVAL;
}

int io_timer_set_dshot_capture_mode(uint8_t timer, uint8_t timer_channel_index, unsigned bitrate)
{
	const int capture_channel = io_timer_get_dshot_capture_channel(timer, timer_channel_index);
	const dshot_timing_t *timing = io_timer_get_dshot_timing(timer, bitrate);
	const int output_channel = dshot_output_channel(timer, timer_channel_index);

	if (capture_channel < 0 || timing == NULL || output_channel < 0
	    || (timer_allocations[timer] != IOTimerChanMode_DshotInverted
		&& timer_allocations[timer] != IOTimerChanMode_CaptureDMA)
	    || (io_timer_get_channel_mode(output_channel) != IOTimerChanMode_DshotInverted
		&& io_timer_get_channel_mode(output_channel) != IOTimerChanMode_CaptureDMA)
	    || timing->capture_prescaler == 0 || timing->capture_prescaler > UINT16_MAX + 1u) {
		return -EINVAL;
	}

	irqstate_t flags = px4_enter_critical_section();
	rDIER(timer) = 0;
	rCR1(timer) = 0;
	rCCER(timer) = 0;
	rCR2(timer) &= ~GTIM_CR2_CCDS;
	rCCMR1(timer) = 0;
	rCCMR2(timer) = 0;
	rPSC(timer) = timing->capture_prescaler - 1;
	rARR(timer) = UINT16_MAX;
	rDCR(timer) = 0;
	rEGR(timer) = GTIM_EGR_UG;
	rCNT(timer) = 0;
	rSR(timer) = 0;

	// Every ESC can reply to an inverted frame. Release all allocated outputs,
	// including the physical pin of the borrowed channel, before enabling input.
	for (unsigned channel = 0; channel < MAX_TIMER_IO_CHANNELS; ++channel) {
		const int mode = io_timer_get_channel_mode(channel);

		if (timer_io_channels[channel].timer_index == timer
		    && (mode == IOTimerChanMode_DshotInverted || mode == IOTimerChanMode_CaptureDMA)) {
			const uint32_t pin = timer_io_channels[channel].gpio_in & (GPIO_PORT_MASK | GPIO_PIN_MASK);
			px4_arch_configgpio(pin | GPIO_INPUT | GPIO_PULLUP);
		}
	}

	px4_arch_configgpio((timer_io_channels[output_channel].gpio_in & ~GPIO_PUPD_MASK) | GPIO_PULLUP);
	const uint32_t ccmr_offset = AT32_GTIM_CCMR1_OFFSET + (capture_channel / 2) * sizeof(uint32_t);
	const uint32_t capture_input = timer_channel_index == 3 ? GTIM_CCMR_CCS_CCIN2 : GTIM_CCMR_CCS_CCIN1;
	REG(timer, ccmr_offset) = capture_input << ((capture_channel & 1) * CCMR_C1_NUM_BITS);
	rCCER(timer) = (GTIM_CCER_CC1E | GTIM_CCER_CC1P | GTIM_CCER_CC1NP)
		       << (capture_channel * CCER_C1_NUM_BITS);

	// DMA setup and the counter start belong to the backend. No synthetic edge.
	px4_leave_critical_section(flags);
	return OK;
}

void io_timer_capture_dma_req(uint8_t timer, uint8_t timer_channel_index, bool enable)
{
	const int capture_channel = io_timer_get_dshot_capture_channel(timer, timer_channel_index);

	if (capture_channel < 0) {
		return;
	}

	irqstate_t flags = px4_enter_critical_section();
	const uint32_t request = GTIM_DIER_CC1DE << capture_channel;

	if (enable) {
		rDIER(timer) |= request;

	} else {
		rDIER(timer) &= ~request;
	}

	px4_leave_critical_section(flags);
}

static inline void io_timer_set_PWM_mode(unsigned timer)
{
	rPSC(timer) = (io_timers[timer].clock_freq / BOARD_PWM_FREQ) - 1;
}

void io_timer_trigger(unsigned channels_mask)
{
	int oneshots = io_timer_get_mode_channels(IOTimerChanMode_OneShot) & channels_mask;

	if (oneshots != 0) {
		uint32_t action_cache[MAX_IO_TIMERS] = {0};
		int actions = 0;

		/* Pre-calculate the list of timers to Trigger */

		for (int timer = 0; timer < MAX_IO_TIMERS; timer++) {
			if (validate_timer_index(timer) == 0) {
				int channels = get_timer_channels(timer);

				if (oneshots & channels) {
					action_cache[actions++] = io_timers[timer].base;
				}
			}
		}

		/* Now do them all with the shortest delay in between */

		irqstate_t flags = px4_enter_critical_section();

		for (actions = 0; actions < MAX_IO_TIMERS && action_cache[actions] != 0; actions++) {
			_REG32(action_cache[actions], AT32_GTIM_EGR_OFFSET) |= GTIM_EGR_UG;
		}

		px4_leave_critical_section(flags);
	}
}

int io_timer_init_timer(unsigned timer, io_timer_channel_mode_t mode)
{
	if (validate_timer_index(timer) != 0) {
		return -EINVAL;
	}

	io_timer_channel_mode_t previous_mode = timer_allocations[timer];
	int rv = io_timer_allocate_timer(timer, mode);

	/* Do this only once per timer */
	if (rv == 0 && previous_mode == IOTimerChanMode_NotUsed) {

		irqstate_t flags = px4_enter_critical_section();

		/* enable the timer clock before we try to talk to it */

		modifyreg32(io_timers[timer].clock_register, 0, io_timers[timer].clock_bit);

		/* disable and configure the timer */
		rCR1(timer) = 0;
		rCR2(timer) = 0;
		rSMCR(timer) = 0;
		rDIER(timer) = 0;
		rCCER(timer) = 0;
		rCCMR1(timer) = 0;
		rCCMR2(timer) = 0;
		rCCR1(timer) = 0;
		rCCR2(timer) = 0;
		rCCR3(timer) = 0;
		rCCR4(timer) = 0;
		rCCER(timer) = 0;
		rDCR(timer) = 0;

		if ((io_timers[timer].base == AT32_TMR1_BASE)
		    || (io_timers[timer].base == AT32_TMR8_BASE)
		    || (io_timers[timer].base == AT32_TMR20_BASE)
		   ) {

			/* master output enable = on */

			rBDTR(timer) = ATIM_BDTR_MOE;
		}

		io_timer_set_PWM_mode(timer);

		/*
		 * Note we do the Standard PWM Out init here
		 * default to updating at 50Hz
		 */

		if (mode == IOTimerChanMode_Capture) {
			// Capture timestamps use 16-bit modulo arithmetic at 1 MHz.
			rARR(timer) = UINT16_MAX;

		} else {
			timer_set_rate(timer, 50);
		}

		/*
		 * Note that the timer is left disabled with IRQ subs installed
		 * and active but DEIR bits are not set.
		 */
		xcpt_t handler;

		switch (timer) {
		case 0: handler = io_timer_handler0; break;

		case 1: handler = io_timer_handler1; break;

		case 2: handler = io_timer_handler2; break;

		case 3: handler = io_timer_handler3; break;

		case 4: handler = io_timer_handler4; break;

		case 5: handler = io_timer_handler5; break;

		case 6: handler = io_timer_handler6; break;

		case 7: handler = io_timer_handler7; break;

		default:
			handler = NULL;
			rv = -EINVAL;
			break;
		}

		if (handler) {
			irq_attach(io_timers[timer].vectorno, handler, NULL);
			up_enable_irq(io_timers[timer].vectorno);
		}

		px4_leave_critical_section(flags);
	}

	return rv;
}


int io_timer_set_pwm_rate(unsigned timer, unsigned rate)
{
	if (validate_timer_index(timer) != 0 || (rate != PWM_RATE_ONESHOT
			&& (rate == 0 || rate > BOARD_PWM_FREQ || BOARD_PWM_FREQ / rate > UINT16_MAX + 1u))) {
		return -EINVAL;
	}

	/* Change only a timer that is owned by pwm or one shot */
	if (timer_allocations[timer] != IOTimerChanMode_PWMOut && timer_allocations[timer] != IOTimerChanMode_OneShot) {
		return -EINVAL;
	}

	/* Get the channel bits that belong to the timer and are in PWM or OneShot mode */

	uint32_t channels = get_timer_channels(timer) & (io_timer_get_mode_channels(IOTimerChanMode_OneShot) |
			    io_timer_get_mode_channels(IOTimerChanMode_PWMOut));

	/* Request to use OneShot ?*/

	if (PWM_RATE_ONESHOT == rate) {

		/* Request to use OneShot
		 */
		int changed_channels = reallocate_channel_resources(channels, IOTimerChanMode_PWMOut, IOTimerChanMode_OneShot);

		/* Did the allocation change */
		if (changed_channels) {
			io_timer_set_oneshot_mode(timer);
		}

	} else {

		/* Request to use PWM
		 */
		int changed_channels = reallocate_channel_resources(channels, IOTimerChanMode_OneShot, IOTimerChanMode_PWMOut);

		if (changed_channels) {
			io_timer_set_PWM_mode(timer);
		}

		timer_set_rate(timer, rate);
	}

	return OK;
}

int io_timer_channel_init(unsigned channel, io_timer_channel_mode_t mode,
			  channel_handler_t channel_handler, void *context)
{
	if (io_timer_validate_channel_index(channel) != 0) {
		return -EINVAL;
	}

	if (mode == IOTimerChanMode_NotUsed) {
		const int previous_mode = io_timer_get_channel_mode(channel);

		if (previous_mode > IOTimerChanMode_NotUsed) {
			io_timer_set_enable(false, previous_mode, 1 << channel);
			io_timer_unallocate_channel(channel);
		}

		return OK;
	}

	uint32_t gpio = 0;
	uint32_t clearbits = CCMR_C1_RESET;
	uint32_t setbits = CCMR_C1_CAPTURE_INIT;
	uint32_t ccer_setbits = CCER_C1_INIT;
	uint32_t dier_setbits = GTIM_DIER_CC1IE;

	/* figure out the GPIO config first */

	switch (mode) {

	case IOTimerChanMode_OneShot:
	case IOTimerChanMode_PWMOut:
	case IOTimerChanMode_Trigger:
	case IOTimerChanMode_Dshot:
		ccer_setbits = 0;
		dier_setbits = 0;
		setbits = CCMR_C1_PWMOUT_INIT;
		break;

	case IOTimerChanMode_DshotInverted:
		ccer_setbits = 0;
		dier_setbits = 0;
		setbits = CCMR_C1_PWMOUT_INVERTED_INIT;
		break;

	case IOTimerChanMode_CaptureDMA:
		ccer_setbits = 0;
		dier_setbits = 0;
		setbits = CCMR_C1_CAPTURE_INIT;
		gpio = (timer_io_channels[channel].gpio_in & ~GPIO_PUPD_MASK) | GPIO_PULLUP;
		break;


#if !defined(BOARD_HAS_NO_CAPTURE)

	case IOTimerChanMode_Capture:
		setbits = CCMR_C1_CAPTURE_INIT;
		gpio = timer_io_channels[channel].gpio_in | GPIO_PULLUP;
		break;

#endif

	case IOTimerChanMode_NotUsed:
		setbits = 0;
		break;

	default:
		return -EINVAL;
	}

	irqstate_t flags = px4_enter_critical_section(); // atomic channel allocation and hw config

	int previous_mode = io_timer_get_channel_mode(channel);
	int rv = allocate_channel(channel, mode);
	unsigned timer = channels_timer(channel);

	if (rv == 0) {
		/* Try to reserve & initialize the timer - it will only do it once */

		rv = io_timer_init_timer(timer, mode);

		if (rv != 0 && previous_mode == IOTimerChanMode_NotUsed) {
			/* free the channel if it was not used before */
			io_timer_unallocate_channel(channel);
		}
	}

	/* Valid channel should now be reserved in new mode */

	if (rv == 0) {

		/* Set up IO */
		if (gpio) {
			px4_arch_configgpio(gpio);
		}

		/* configure the channel */

		uint32_t shifts = timer_io_channels[channel].timer_channel;

		/* Map shifts timer channel 0-3 */

		uint32_t ccmr_offset = AT32_GTIM_CCMR1_OFFSET + ((shifts >> 1)  * sizeof(uint32_t));
		uint32_t ccr_offset = AT32_GTIM_CCR1_OFFSET + (shifts * sizeof(uint32_t));
		// CCxS can only change while the channel is disabled.
		rCCER(timer) &= ~(GTIM_CCER_CC1E << (shifts * CCER_C1_NUM_BITS));

		clearbits <<= (shifts & 1) * CCMR_C1_NUM_BITS;
		setbits <<= (shifts & 1) * CCMR_C1_NUM_BITS;

		uint16_t rvalue = REG(timer, ccmr_offset);
		rvalue &= ~clearbits;
		rvalue |=  setbits;
		REG(timer, ccmr_offset) = rvalue;

		// Input capture makes CCR read-only; outputs start with zero duty.

		REG(timer, ccr_offset) = 0;

		/* on PWM Out ccer_setbits is 0 */

		clearbits = (GTIM_CCER_CC1E | GTIM_CCER_CC1P) << (shifts * CCER_C1_NUM_BITS);

		// Channel 4 has no complementary polarity bit on AT32F435.
		if (shifts < 3) {
			clearbits |= HW_GTIM_CCER_CC1NP << (shifts * CCER_C1_NUM_BITS);
		}

		setbits  = ccer_setbits << (shifts * CCER_C1_NUM_BITS);
		rvalue = rCCER(timer);
		rvalue &= ~clearbits;
		rvalue |=  setbits;
		rCCER(timer) = rvalue;

#if defined(BOARD_HAS_NO_CAPTURE)
		UNUSED(dier_setbits);
#else
		channel_handlers[channel].callback = channel_handler;
		channel_handlers[channel].context = context;
		rDIER(timer) |= dier_setbits << shifts;
#endif
	}

	px4_leave_critical_section(flags);

	return rv;
}

int io_timer_set_enable(bool state, io_timer_channel_mode_t mode, io_timer_channel_allocation_t masks)
{
	struct action_cache_t {
		uint32_t ccer_clearbits;
		uint32_t ccer_setbits;
		uint32_t dier_setbits;
		uint32_t dier_clearbits;
		uint32_t base;
		uint32_t gpio[MAX_CHANNELS_PER_TIMER];
	} action_cache[MAX_IO_TIMERS];
	memset(action_cache, 0, sizeof(action_cache));

	uint32_t dier_bit = state ? GTIM_DIER_CC1IE : 0;
	uint32_t ccer_bit =  state ? CCER_C1_INIT : 0;
	uint32_t cr1_bit  = 0;

	switch (mode) {
	case IOTimerChanMode_NotUsed:
	case IOTimerChanMode_OneShot:
	case IOTimerChanMode_PWMOut:
	case IOTimerChanMode_Trigger:
		dier_bit = 0;
		cr1_bit  = GTIM_CR1_CEN | GTIM_CR1_ARPE;
		break;

	case IOTimerChanMode_Capture:
		cr1_bit  = state ? GTIM_CR1_CEN : 0;
		break;

	case IOTimerChanMode_Dshot:
	case IOTimerChanMode_DshotInverted:
	case IOTimerChanMode_CaptureDMA:
		dier_bit = 0;
		// The DShot backend owns the DMA/counter startup sequence.
		cr1_bit = 0;
		break;

	default:
		return -EINVAL;
	}

	/* Was the request for all channels in this mode ?*/

	if (masks == IO_TIMER_ALL_MODES_CHANNELS) {

		/* Yes - we provide them */

		masks = channel_allocations[mode];

	} else {

		/* No - caller provided mask */

		/* Only allow the channels in that mode to be affected */

		masks &= channel_allocations[mode];

	}

	/* Pre calculate all the changes */

	for (int chan_index = 0; masks != 0 && chan_index < MAX_TIMER_IO_CHANNELS; chan_index++) {
		if (masks & (1 << chan_index)) {
			masks &= ~(1 << chan_index);
			uint32_t shifts = timer_io_channels[chan_index].timer_channel;
			uint32_t timer = channels_timer(chan_index);

			if (mode == IOTimerChanMode_CaptureDMA) {
				const int capture_channel = io_timer_get_dshot_capture_channel(timer, shifts);

				if (capture_channel < 0) {
					return -EINVAL;
				}

				shifts = capture_channel;
			}

			action_cache[timer].base  = io_timers[timer].base;
			action_cache[timer].ccer_clearbits |= CCER_C1_INIT << (shifts * CCER_C1_NUM_BITS);
			action_cache[timer].ccer_setbits   |= ccer_bit  << (shifts * CCER_C1_NUM_BITS);
			action_cache[timer].dier_clearbits |= GTIM_DIER_CC1IE  << shifts;
			action_cache[timer].dier_setbits   |= dier_bit << shifts;

			if ((state &&
			     (mode == IOTimerChanMode_PWMOut ||
			      mode == IOTimerChanMode_OneShot ||
			      mode == IOTimerChanMode_Dshot ||
			      mode == IOTimerChanMode_DshotInverted ||
			      mode == IOTimerChanMode_Trigger))) {
				action_cache[timer].gpio[shifts] = timer_io_channels[chan_index].gpio_out;

			} else if (state && mode == IOTimerChanMode_CaptureDMA) {
				action_cache[timer].gpio[shifts] = (timer_io_channels[chan_index].gpio_in & ~GPIO_PUPD_MASK) | GPIO_PULLUP;
			}
		}
	}

	irqstate_t flags = px4_enter_critical_section();

	for (unsigned actions = 0; actions < arraySize(action_cache); actions++) {
		if (action_cache[actions].base != 0) {
			uint32_t rvalue = _REG32(action_cache[actions].base, AT32_GTIM_CCER_OFFSET);
			rvalue &= ~action_cache[actions].ccer_clearbits;
			rvalue |= action_cache[actions].ccer_setbits;
			_REG32(action_cache[actions].base, AT32_GTIM_CCER_OFFSET) = rvalue;
			uint32_t after = rvalue & (GTIM_CCER_CC1E | GTIM_CCER_CC2E | GTIM_CCER_CC3E | GTIM_CCER_CC4E);

			rvalue = _REG32(action_cache[actions].base, AT32_GTIM_DIER_OFFSET);
			rvalue &= ~action_cache[actions].dier_clearbits;
			rvalue |= action_cache[actions].dier_setbits;
			_REG32(action_cache[actions].base, AT32_GTIM_DIER_OFFSET) = rvalue;

			/* Any On ?*/

			if (after != 0) {

				if (mode != IOTimerChanMode_Dshot && mode != IOTimerChanMode_DshotInverted
				    && mode != IOTimerChanMode_CaptureDMA) {
					/* force an update to preload all registers */
					rEGR(actions) = GTIM_EGR_UG;
				}

				for (unsigned chan = 0; chan < arraySize(action_cache[actions].gpio); chan++) {
					if (action_cache[actions].gpio[chan]) {
						px4_arch_configgpio(action_cache[actions].gpio[chan]);
						action_cache[actions].gpio[chan] = 0;
					}
				}

				/* arm requires the timer be enabled */
				rCR1(actions) |= cr1_bit;

			} else  {

				rCR1(actions) = 0;
			}
		}
	}

	px4_leave_critical_section(flags);

	return 0;
}

int io_timer_set_ccr(unsigned channel, uint16_t value)
{
	int rv = io_timer_validate_channel_index(channel);
	int mode = io_timer_get_channel_mode(channel);

	if (rv == 0) {
		if ((mode != IOTimerChanMode_PWMOut) &&
		    (mode != IOTimerChanMode_OneShot) &&
		    (mode != IOTimerChanMode_Dshot) &&
		    (mode != IOTimerChanMode_DshotInverted) &&
		    (mode != IOTimerChanMode_Trigger)) {

			rv = -EIO;

		} else {

			/* configure the channel */

			REG(channels_timer(channel), timer_io_channels[channel].ccr_offset) = value;
		}
	}

	return rv;
}

uint16_t io_channel_get_ccr(unsigned channel)
{
	uint16_t value = 0;

	if (io_timer_validate_channel_index(channel) == 0) {
		int mode = io_timer_get_channel_mode(channel);

		if ((mode == IOTimerChanMode_PWMOut) ||
		    (mode == IOTimerChanMode_OneShot) ||
		    (mode == IOTimerChanMode_Trigger)) {
			value = REG(channels_timer(channel), timer_io_channels[channel].ccr_offset);
		}
	}

	return value;
}

uint32_t io_timer_get_group(unsigned timer)
{
	return get_timer_channels(timer);

}
