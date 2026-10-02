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

#include <px4_arch/dshot.h>
#include <px4_arch/io_timer.h>
#include <drivers/drv_dshot.h>
#include <px4_platform_common/log.h>
#include <at32_dma.h>
#include <at32_tim.h>
#include <errno.h>
#include <string.h>
#include "arm_internal.h"

#define DSHOT_FRAME_BITS 16u
#define DSHOT_TIMER_CHANNELS 4u
// Two zero rows let the final data bit finish before the DMA completion IRQ.
#define DSHOT_FRAME_ROWS (DSHOT_FRAME_BITS + 2u)
#define BDSHOT_CAPTURE_EDGES 32u
#define BDSHOT_RESPONSE_BITS 21u
#define BDSHOT_ONLINE_COUNT 200u
#define BDSHOT_BOOT_DELAY_US 3000000u
#define DSHOT_FRAME_GAP_US 30u
#define BDSHOT_TURNAROUND_US 30u
#define BDSHOT_CAPTURE_MARGIN_US 20u
#define DSHOT_DMA_CONFIG (DMA_CCR_PRIHI | DMA_CCR_MSIZE_16BITS | DMA_CCR_PSIZE_16BITS | DMA_CCR_MINC)

enum dshot_phase { DSHOT_IDLE, DSHOT_TRANSMIT, DSHOT_CAPTURE };

typedef struct {
	DMA_HANDLE output_dma;
	DMA_HANDLE capture_dma;
	struct hrt_call capture_call;
	hrt_abstime next_frame_time;
	uint32_t channel_mask;
	uint32_t dma_errors;
	uint8_t timer_index;
	uint8_t span;
	uint8_t capture_channel;
	enum dshot_phase phase;
	bool bidirectional;
	bool capture_error;
} dshot_timer_t;

typedef struct {
	int erpm;
	uint32_t packets;
	uint32_t errors;
	uint32_t base_interval_x8;
	uint16_t successes;
	uint16_t failures;
	uint8_t edt_value[4];
	uint8_t edt_ready;
	bool erpm_ready;
	bool online;
} bdshot_sample_t;

static dshot_timer_t timers[MAX_IO_TIMERS];
static bdshot_sample_t samples[MAX_TIMER_IO_CHANNELS];
// Task-side writes must never modify a buffer being read by DMA.
static uint16_t pending[MAX_IO_TIMERS][DSHOT_FRAME_ROWS * DSHOT_TIMER_CHANNELS];
static uint16_t output[MAX_IO_TIMERS][DSHOT_FRAME_ROWS * DSHOT_TIMER_CHANNELS];
static uint16_t capture[MAX_IO_TIMERS][BDSHOT_CAPTURE_EDGES];
static uint32_t initialized_mask;
static uint32_t bidirectional_mask;
static uint16_t ready_mask;
static unsigned bitrate;
static hrt_abstime capture_start_time;
static bool outputs_armed;
static bool edt_enabled;

// GCR and period conversion follow the existing PX4 STM32 DShot backend.
// Invalid GCR symbols must not alias the valid zero symbol.
static const int8_t gcr_decode[32] = {
	-1, -1, -1, -1, -1, -1, -1, -1,
		-1, 9, 10, 11, -1, 13, 14, 15,
		-1, -1, 2, 3, -1, 5, 6, 7,
		-1, 0, 8, 1, -1, 4, 12, -1
	};

static void capture_complete(void *arg);
static void output_complete(DMA_HANDLE handle, uint8_t status, void *arg);

static uint8_t output_channel(uint8_t timer, uint8_t physical_channel)
{
	for (uint8_t channel = 0; channel < MAX_TIMER_IO_CHANNELS; ++channel) {
		if (timer_io_channels[channel].timer_index == timer
		    && timer_io_channels[channel].timer_channel == physical_channel) {
			return channel;
		}
	}

	return MAX_TIMER_IO_CHANNELS;
}

static io_timer_channel_mode_t output_mode(const dshot_timer_t *timer)
{
	return timer->bidirectional ? IOTimerChanMode_DshotInverted : IOTimerChanMode_Dshot;
}

static void stop_counter(uint8_t timer)
{
	modifyreg32(io_timers[timer].base + AT32_GTIM_CR1_OFFSET, GTIM_CR1_CEN, 0);
}

static void start_counter(uint8_t timer)
{
	modifyreg32(io_timers[timer].base + AT32_GTIM_CR1_OFFSET, 0, GTIM_CR1_CEN);
}

static void capture_failed(uint8_t channel)
{
	bdshot_sample_t *sample = &samples[channel];
	++sample->errors;
	sample->successes = 0;
	sample->erpm_ready = false;

	if (sample->failures < BDSHOT_ONLINE_COUNT) {
		++sample->failures;
	}

	if (sample->failures == BDSHOT_ONLINE_COUNT) {
		sample->online = false;
	}

	ready_mask |= 1u << channel;
}

static bool decode_capture(const uint16_t *edges, unsigned count, uint32_t *base_x8, uint16_t *payload)
{
	if (count < 4 || count > BDSHOT_CAPTURE_EDGES) {
		return false;
	}

	uint16_t intervals[BDSHOT_CAPTURE_EDGES];
	unsigned interval_count = 0;
	uint16_t shortest = UINT16_MAX;

	// The inverted output and input pull-up both idle high. The first captured
	// edge is the ESC's falling start edge, so retain its first data interval.
	for (unsigned i = 1; i < count; ++i) {
		const uint16_t interval = (uint16_t)(edges[i] - edges[i - 1]);

		if (interval < DSHOT_CAPTURE_TICKS_PER_BIT / 2 || interval > DSHOT_CAPTURE_TICKS_PER_BIT * 5) {
			return false;
		}

		intervals[interval_count++] = interval;

		if (interval < shortest) {
			shortest = interval;
		}
	}

	// Update only plausible single-bit intervals; a frame can start with a longer run.
	if (shortest * 8u >= *base_x8 * 3u / 4u && shortest * 8u <= *base_x8 * 5u / 4u) {
		*base_x8 = *base_x8 - (*base_x8 >> 3) + shortest;
	}

	uint32_t value = 0;
	unsigned shifted = 0;
	unsigned high = 1;

	for (unsigned i = 0; i < interval_count; ++i) {
		const unsigned bits = (intervals[i] * 8u + *base_x8 / 2u) / *base_x8;

		if (bits == 0 || bits > 4 || shifted + bits > BDSHOT_RESPONSE_BITS) {
			return false;
		}

		for (unsigned bit = 0; bit < bits; ++bit) {
			value = (value << 1) | high;
		}

		shifted += bits;
		high ^= 1;
	}

	if (shifted < BDSHOT_RESPONSE_BITS - 3u) {
		return false;
	}

	value <<= BDSHOT_RESPONSE_BITS - shifted;
	value ^= value >> 1;
	uint16_t data = 0;

	for (unsigned nibble = 0; nibble < 4; ++nibble) {
		const int8_t decoded = gcr_decode[(value >> (nibble * 5u)) & 0x1f];

		if (decoded < 0) {
			return false;
		}

		data |= (uint16_t)decoded << (nibble * 4u);
	}

	if (((data ^ (data >> 4) ^ (data >> 8) ^ (data >> 12)) & 0xf) != 0xf) {
		return false;
	}

	*payload = data >> 4;
	return true;
}

static void save_telemetry(uint8_t channel, uint16_t payload)
{
	bdshot_sample_t *sample = &samples[channel];
	++sample->packets;
	sample->failures = 0;

	if (sample->successes < BDSHOT_ONLINE_COUNT) {
		++sample->successes;
	}

	if (sample->successes == BDSHOT_ONLINE_COUNT) {
		sample->online = true;
	}

	if (edt_enabled && !(payload & 0x100)) {
		const unsigned type = payload >> 8;
		unsigned index;

		switch (type) {
		case DSHOT_EDT_TEMPERATURE: index = 0; break;

		case DSHOT_EDT_VOLTAGE: index = 1; break;

		case DSHOT_EDT_CURRENT: index = 2; break;

		case DSHOT_EDT_STATE_EVENT: index = 3; break;

		default: ready_mask |= 1u << channel; return;
		}

		sample->edt_value[index] = payload & 0xff;
		sample->edt_ready |= 1u << index;

	} else {
		const unsigned period = (payload & 0x1ff) << (payload >> 9);
		// PX4's DShot interface reports electrical RPM in units of 100 RPM.
		sample->erpm = period == 0 || period == 65408 ? 0 : (600000u + period / 2) / period;
		sample->erpm_ready = true;
	}

	ready_mask |= 1u << channel;
}

static void restore_output(dshot_timer_t *timer)
{
	io_timer_set_dshot_burst_mode(timer->timer_index, bitrate, timer->span);
	io_timer_set_enable(outputs_armed, output_mode(timer), timer->channel_mask);
	timer->phase = DSHOT_IDLE;
}

static void capture_complete(void *arg)
{
	dshot_timer_t *timer = arg;
	const irqstate_t flags = px4_enter_critical_section();

	if (!outputs_armed || timer->phase != DSHOT_CAPTURE) {
		px4_leave_critical_section(flags);
		return;
	}

	stop_counter(timer->timer_index);
	io_timer_capture_dma_req(timer->timer_index, timer->capture_channel, false);
	at32_dmastop(timer->capture_dma);
	const unsigned count = BDSHOT_CAPTURE_EDGES - at32_dmaresidual(timer->capture_dma);
	at32_dmafree(timer->capture_dma);
	timer->capture_dma = NULL;
	const uint8_t channel = output_channel(timer->timer_index, timer->capture_channel);
	uint16_t payload;

	if (!timer->capture_error && decode_capture(capture[timer->timer_index], count,
			&samples[channel].base_interval_x8, &payload)) {
		save_telemetry(channel, payload);

	} else {
		capture_failed(channel);
	}

	// Keep idle pins released with pull-ups. The next trigger restores output
	// once before DMA starts, rather than repeating that work in this ISR.
	timer->phase = DSHOT_IDLE;
	px4_leave_critical_section(flags);
}

static void capture_dma_complete(DMA_HANDLE handle, uint8_t status, void *arg)
{
	dshot_timer_t *timer = arg;

	if (timer->phase == DSHOT_CAPTURE && (status & DMA_STATUS_ERROR)) {
		timer->capture_error = true;
		stop_counter(timer->timer_index);
		io_timer_capture_dma_req(timer->timer_index, timer->capture_channel, false);
		at32_dmastop(handle);
	}
}

static void begin_capture(dshot_timer_t *timer)
{
	for (unsigned next = 1; next <= DSHOT_TIMER_CHANNELS; ++next) {
		const uint8_t physical = (timer->capture_channel + next) % DSHOT_TIMER_CHANNELS;
		const uint8_t channel = output_channel(timer->timer_index, physical);

		if (channel < MAX_TIMER_IO_CHANNELS && (bidirectional_mask & (1u << channel))) {
			timer->capture_channel = physical;
			break;
		}
	}

	const uint8_t channel = output_channel(timer->timer_index, timer->capture_channel);
	const int hardware_channel = io_timer_get_dshot_capture_channel(timer->timer_index, timer->capture_channel);

	if (channel >= MAX_TIMER_IO_CHANNELS || hardware_channel < 0) {
		restore_output(timer);
		return;
	}

	timer->capture_dma = at32_dmachannel(io_timers[timer->timer_index].dshot.dma_map_ch[hardware_channel]);

	if (timer->capture_dma == NULL
	    || io_timer_set_dshot_capture_mode(timer->timer_index, timer->capture_channel, bitrate) != OK) {
		if (timer->capture_dma) {
			at32_dmafree(timer->capture_dma);
			timer->capture_dma = NULL;
		}

		capture_failed(channel);
		restore_output(timer);
		return;
	}

	timer->capture_error = false;
	memset(capture[timer->timer_index], 0, sizeof(capture[timer->timer_index]));
	at32_dmasetup(timer->capture_dma,
		      io_timers[timer->timer_index].base + AT32_GTIM_CCR1_OFFSET + sizeof(uint32_t) * hardware_channel,
		      (uintptr_t)capture[timer->timer_index], BDSHOT_CAPTURE_EDGES, DSHOT_DMA_CONFIG);
	timer->phase = DSHOT_CAPTURE;
	at32_dmastart(timer->capture_dma, capture_dma_complete, timer, false);
	io_timer_capture_dma_req(timer->timer_index, timer->capture_channel, true);
	start_counter(timer->timer_index);
	const unsigned response_rate = bitrate * 5u / 4u;
	const hrt_abstime window = BDSHOT_TURNAROUND_US + BDSHOT_CAPTURE_MARGIN_US
				   + (BDSHOT_RESPONSE_BITS * 1000000u + response_rate - 1) / response_rate;
	hrt_call_after(&timer->capture_call, window, capture_complete, timer);
}

static void output_complete(DMA_HANDLE handle, uint8_t status, void *arg)
{
	dshot_timer_t *timer = arg;
	const irqstate_t flags = px4_enter_critical_section();

	if (timer->phase != DSHOT_TRANSMIT) {
		px4_leave_critical_section(flags);
		return;
	}

	io_timer_update_dma_req(timer->timer_index, false);
	stop_counter(timer->timer_index);
	at32_dmastop(handle);
	timer->next_frame_time = hrt_absolute_time() + DSHOT_FRAME_GAP_US;

	if (status & DMA_STATUS_ERROR) {
		++timer->dma_errors;

		for (uint8_t channel = 0; channel < MAX_TIMER_IO_CHANNELS; ++channel) {
			if (timer->channel_mask & bidirectional_mask & (1u << channel)) {
				capture_failed(channel);
			}
		}

		restore_output(timer);

	} else if ((status & DMA_STATUS_TCIF) && outputs_armed && timer->bidirectional
		   && hrt_absolute_time() >= capture_start_time) {
		begin_capture(timer);

	} else {
		timer->phase = DSHOT_IDLE;
	}

	px4_leave_critical_section(flags);
}

void dshot_motor_data_set(uint8_t channel, uint16_t data, bool telemetry)
{
	if (channel >= MAX_TIMER_IO_CHANNELS || data > DSHOT_CMD_MAX_THROTTLE) {
		return;
	}

	const irqstate_t flags = px4_enter_critical_section();

	if (initialized_mask & (1u << channel)) {
		const uint8_t timer = timer_io_channels[channel].timer_index;
		const uint8_t slot = timer_io_channels[channel].timer_channel
				     - io_timers_channel_mapping.element[timer].lowest_timer_channel;
		uint16_t packet = (data << 1) | (telemetry ? 1u : 0u);
		uint16_t checksum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0xf;

		if (bidirectional_mask & (1u << channel)) {
			checksum ^= 0xf;
		}

		packet = (packet << 4) | checksum;
		const dshot_timing_t *timing = io_timer_get_dshot_timing(timer, bitrate);

		for (unsigned bit = 0; bit < DSHOT_FRAME_BITS; ++bit) {
			pending[timer][bit * timers[timer].span + slot] = (packet & 0x8000) ? timing->bit_1 : timing->bit_0;
			packet <<= 1;
		}
	}

	px4_leave_critical_section(flags);
}

int up_dshot_arm(bool armed)
{
	const irqstate_t flags = px4_enter_critical_section();
	outputs_armed = armed;

	for (unsigned timer = 0; timer < MAX_IO_TIMERS; ++timer) {
		dshot_timer_t *state = &timers[timer];

		if (!state->channel_mask) {
			continue;
		}

		hrt_cancel(&state->capture_call);
		io_timer_update_dma_req(timer, false);
		stop_counter(timer);
		at32_dmastop(state->output_dma);

		if (state->capture_dma) {
			io_timer_capture_dma_req(timer, state->capture_channel, false);
			at32_dmastop(state->capture_dma);
			at32_dmafree(state->capture_dma);
			state->capture_dma = NULL;
		}

		restore_output(state);
	}

	if (!armed) {
		for (uint8_t channel = 0; channel < MAX_TIMER_IO_CHANNELS; ++channel) {
			if (initialized_mask & (1u << channel)) {
				px4_arch_configgpio(io_timer_channel_get_gpio_output(channel));
				dshot_motor_data_set(channel, DSHOT_CMD_MOTOR_STOP, false);
			}
		}
	}

	px4_leave_critical_section(flags);
	return OK;
}

int up_dshot_init(uint32_t channel_mask, uint32_t bdshot_channel_mask, unsigned dshot_pwm_freq, bool edt_enable)
{
	if ((channel_mask >> MAX_TIMER_IO_CHANNELS) != 0 || (bdshot_channel_mask & ~channel_mask) != 0
	    || (dshot_pwm_freq != 150000 && dshot_pwm_freq != 300000
		&& dshot_pwm_freq != 600000 && dshot_pwm_freq != 1200000)) {
		return -EINVAL;
	}

	for (unsigned timer = 0; timer < MAX_IO_TIMERS; ++timer) {
		const uint32_t channels = channel_mask & io_timer_get_group(timer);
		const uint32_t bidir = bdshot_channel_mask & channels;

		if (bidir != 0 && bidir != channels) {
			// A timer group shares its output/capture phases.
			return -EINVAL;
		}
	}

	up_dshot_arm(false);

	for (unsigned timer = 0; timer < MAX_IO_TIMERS; ++timer) {
		for (uint8_t channel = 0; channel < MAX_TIMER_IO_CHANNELS; ++channel) {
			if (timers[timer].channel_mask & (1u << channel)) {
				io_timer_channel_init(channel, IOTimerChanMode_NotUsed, NULL, NULL);
			}
		}

		if (timers[timer].channel_mask) {
			io_timer_unallocate_timer(timer);
		}

		if (timers[timer].output_dma) {
			at32_dmafree(timers[timer].output_dma);
		}
	}

	memset(timers, 0, sizeof(timers));
	memset(samples, 0, sizeof(samples));
	memset(pending, 0, sizeof(pending));
	initialized_mask = 0;
	bidirectional_mask = 0;
	ready_mask = 0;
	bitrate = dshot_pwm_freq;
	edt_enabled = edt_enable;
	capture_start_time = hrt_absolute_time() + BDSHOT_BOOT_DELAY_US;

	for (uint8_t timer = 0; timer < MAX_IO_TIMERS; ++timer) {
		dshot_timer_t *state = &timers[timer];
		const uint32_t requested = channel_mask & io_timer_get_group(timer);
		const io_timers_channel_mapping_element_t *mapping = &io_timers_channel_mapping.element[timer];

		if (!requested || !io_timers[timer].dshot.dma_map_up || mapping->channel_count_including_gaps == 0
		    || mapping->lowest_timer_channel + mapping->channel_count_including_gaps > DSHOT_TIMER_CHANNELS) {
			continue;
		}

		state->output_dma = at32_dmachannel(io_timers[timer].dshot.dma_map_up);

		if (state->output_dma == NULL) {
			PX4_ERR("No DShot DMA channel for timer %u", timer);
			continue;
		}

		state->timer_index = timer;
		state->span = mapping->channel_count_including_gaps;
		state->capture_channel = DSHOT_TIMER_CHANNELS - 1;
		state->bidirectional = (requested & bdshot_channel_mask) != 0;

		for (uint8_t channel = 0; channel < MAX_TIMER_IO_CHANNELS; ++channel) {
			if ((requested & (1u << channel)) && io_timer_validate_channel_index(channel) == OK) {
				if (state->bidirectional) {
					const int hardware_channel = io_timer_get_dshot_capture_channel(timer, timer_io_channels[channel].timer_channel);

					if (hardware_channel < 0 || !io_timers[timer].dshot.dma_map_ch[hardware_channel]) {
						continue;
					}
				}

				if (io_timer_channel_init(channel, output_mode(state), NULL, NULL) == OK) {
					state->channel_mask |= 1u << channel;
					initialized_mask |= 1u << channel;
					bidirectional_mask |= bdshot_channel_mask & (1u << channel);
					samples[channel].base_interval_x8 = DSHOT_CAPTURE_TICKS_PER_BIT * 8;
				}
			}
		}

		if (state->channel_mask == 0) {
			at32_dmafree(state->output_dma);
			state->output_dma = NULL;
		}
	}

	for (uint8_t channel = 0; channel < MAX_TIMER_IO_CHANNELS; ++channel) {
		dshot_motor_data_set(channel, DSHOT_CMD_MOTOR_STOP, false);
	}

	return initialized_mask;
}

void up_dshot_trigger(void)
{
	const irqstate_t flags = px4_enter_critical_section();
	const hrt_abstime now = hrt_absolute_time();

	if (!outputs_armed) {
		px4_leave_critical_section(flags);
		return;
	}

	// Keep timer groups on the same control update; never interrupt a response or DMA frame.
	for (unsigned timer = 0; timer < MAX_IO_TIMERS; ++timer) {
		if (timers[timer].channel_mask
		    && (timers[timer].phase != DSHOT_IDLE || now < timers[timer].next_frame_time)) {
			px4_leave_critical_section(flags);
			return;
		}
	}

	for (unsigned timer = 0; timer < MAX_IO_TIMERS; ++timer) {
		dshot_timer_t *state = &timers[timer];

		if (!state->channel_mask) {
			continue;
		}

		restore_output(state);
		const size_t words = DSHOT_FRAME_ROWS * state->span;
		memcpy(output[timer], pending[timer], words * sizeof(uint16_t));
		at32_dmasetup(state->output_dma, io_timers[timer].base + AT32_GTIM_DMAR_OFFSET,
			      (uintptr_t)output[timer], words, DSHOT_DMA_CONFIG | DMA_CCR_DIR);
		state->phase = DSHOT_TRANSMIT;
		at32_dmastart(state->output_dma, output_complete, state, false);
		io_timer_update_dma_req(timer, true);
		start_counter(timer);
	}

	px4_leave_critical_section(flags);
}

uint16_t up_bdshot_get_ready_mask(void)
{
	const irqstate_t flags = px4_enter_critical_section();
	const uint16_t result = ready_mask;
	px4_leave_critical_section(flags);
	return result;
}

int up_bdshot_get_erpm(uint8_t channel, int *erpm)
{
	if (channel >= MAX_TIMER_IO_CHANNELS || erpm == NULL) {
		return -EINVAL;
	}

	const irqstate_t flags = px4_enter_critical_section();
	const int result = samples[channel].erpm_ready ? OK : -EAGAIN;

	if (result == OK) {
		*erpm = samples[channel].erpm;
		samples[channel].erpm_ready = false;
	}

	ready_mask &= ~(1u << channel);
	px4_leave_critical_section(flags);
	return result;
}

int up_bdshot_get_extended_telemetry(uint8_t channel, int type, uint8_t *value)
{
	if (channel >= MAX_TIMER_IO_CHANNELS || value == NULL) {
		return -EINVAL;
	}

	unsigned index;

	switch (type) {
	case DSHOT_EDT_TEMPERATURE: index = 0; break;

	case DSHOT_EDT_VOLTAGE: index = 1; break;

	case DSHOT_EDT_CURRENT: index = 2; break;

	case DSHOT_EDT_STATE_EVENT: index = 3; break;

	default: return -EINVAL;
	}

	const irqstate_t flags = px4_enter_critical_section();
	const int result = (samples[channel].edt_ready & (1u << index)) ? OK : -EAGAIN;

	if (result == OK) {
		*value = samples[channel].edt_value[index];
		samples[channel].edt_ready &= ~(1u << index);
	}

	px4_leave_critical_section(flags);
	return result;
}

int up_bdshot_num_errors(uint8_t channel)
{
	return channel < MAX_TIMER_IO_CHANNELS ? samples[channel].errors : 0;
}

int up_bdshot_channel_online(uint8_t channel)
{
	return channel < MAX_TIMER_IO_CHANNELS && samples[channel].online;
}

int up_bdshot_channel_capture_supported(uint8_t channel)
{
	return channel < MAX_TIMER_IO_CHANNELS && (bidirectional_mask & (1u << channel)) != 0;
}

void up_bdshot_status(void)
{
	for (unsigned channel = 0; channel < MAX_TIMER_IO_CHANNELS; ++channel) {
		if (bidirectional_mask & (1u << channel)) {
			PX4_INFO("Ch%u: read_ok %lu, errors %lu, %s", channel,
				 (unsigned long)samples[channel].packets, (unsigned long)samples[channel].errors,
				 samples[channel].online ? "online" : "offline");
		}
	}

	for (unsigned timer = 0; timer < MAX_IO_TIMERS; ++timer) {
		if (timers[timer].channel_mask) {
			PX4_INFO("Timer %u: DMA errors %lu", timer, (unsigned long)timers[timer].dma_errors);
		}
	}
}
