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
#include "host.h"
#include "actual_headers.h"

const io_timers_t io_timers[MAX_IO_TIMERS] = {
	{0x40000400u, 0x40021000u, 1u, 288000000u, 1u, {65u, {61u, 62u, 63u, 64u}}},
	{0x40000800u, 0x40021000u, 2u, 288000000u, 2u, {71u, {67u, 68u, 69u, 70u}}}
};
const io_timers_channel_mapping_t io_timers_channel_mapping = {{{0, 4, 0, 4}, {4, 4, 0, 4}}};
#define CHANNEL(t,c,p) {GPIO_AF2|GPIO_ALT|GPIO_DRV_MODETATE|(p), GPIO_AF2|GPIO_ALT|GPIO_DRV_MODETATE|(p), (t), (c), 0, AT32_GTIM_CCR1_OFFSET+4*(c)}
const timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	CHANNEL(0, 0, GPIO_PORTC | GPIO_PIN6), CHANNEL(0, 1, GPIO_PORTC | GPIO_PIN7),
	CHANNEL(0, 2, GPIO_PORTC | GPIO_PIN8), CHANNEL(0, 3, GPIO_PORTC | GPIO_PIN9),
	CHANNEL(1, 0, GPIO_PORTB | GPIO_PIN6), CHANNEL(1, 1, GPIO_PORTB | GPIO_PIN7),
	CHANNEL(1, 2, GPIO_PORTB | GPIO_PIN8), CHANNEL(1, 3, GPIO_PORTB | GPIO_PIN9)
};
#include "actual_io_timer.c"
#include "actual_dshot.c"

static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr, "FAIL line %u: %s\n", __LINE__, #x); exit(1); } } while (0)
static const uint8_t gcr_encode[16] = {0x19, 0x1b, 0x12, 0x13, 0x1d, 0x15, 0x16, 0x17, 0x1a, 9, 10, 11, 0x1e, 13, 14, 15};

static unsigned encode_gcr(uint32_t gcr, uint16_t start, uint16_t edges[32], unsigned ticks)
{
	// The ESC starts its response with a falling edge from the pulled-up idle
	// line. Releasing the FC output does not add an edge before that response.
	uint32_t nrzi = 0;
	unsigned high = 0;

	for (int bit = 19; bit >= 0; --bit) { high ^= (gcr >> bit) & 1u; nrzi = (nrzi << 1) | high; }

	memset(edges, 0, 32 * sizeof(*edges));
	edges[0] = start;
	unsigned count = 1, run = 1;
	high = 0;
	uint16_t time = start;

	for (int bit = 19; bit >= 0; --bit) {
		const unsigned value = (nrzi >> bit) & 1u;

		if (value == high) { ++run; }

		else { time += run * ticks; edges[count++] = time; high = value; run = 1; }
	}

	// The ESC releases the line back to its pulled-up high idle level.
	if (!high) { time += run * ticks; edges[count++] = time; }

	CHECK(count <= 32);
	return count;
}

static uint32_t telemetry_gcr(uint16_t payload, bool corrupt_crc)
{
	// Independent ESC-side encoder: inverted nibble checksum and standard GCR.
	uint16_t crc = ((payload ^ (payload >> 4) ^ (payload >> 8)) & 15u) ^ 15u;

	if (corrupt_crc) { crc ^= 1; }

	const uint16_t packet = (payload << 4) | crc;
	uint32_t gcr = 0;

	for (int n = 3; n >= 0; --n) { gcr = (gcr << 5) | gcr_encode[(packet >> (n * 4)) & 15]; }

	return gcr;
}
static unsigned encode_capture(uint16_t payload, uint16_t start, uint16_t edges[32], bool corrupt_crc)
{
	return encode_gcr(telemetry_gcr(payload, corrupt_crc), start, edges, 20);
}

static void test_decode(void)
{
	for (unsigned p = 0; p < 4096; ++p) {
		for (unsigned wrap = 0; wrap < 2; ++wrap) {
			uint16_t edges[32], decoded = 0xffff;
			uint32_t base = 160;
			const unsigned count = encode_capture(p, wrap ? 65490 : 100, edges, false);

			if (!decode_capture(edges, count, &base, &decoded) || decoded != p) {
				fprintf(stderr, "decode p=%u wrap=%u count=%u base=%u\n", p, wrap, count, base);

				for (unsigned e = 0; e < count; ++e) { fprintf(stderr, " %u", edges[e]); }

				fprintf(stderr, "\n"); exit(1);
			}

			++checks;
			CHECK(base == 160);
			// Dropping the ESC start edge used to be hidden by a synthetic
			// release edge in this fixture. It must not decode as valid data.
			CHECK(!decode_capture(edges + 1, count - 1, &base, &decoded));
			const unsigned bad_count = encode_capture(p, wrap ? 65490 : 100, edges, true);
			CHECK(!decode_capture(edges, bad_count, &base, &decoded));
		}
	}

	uint16_t edges[32], decoded;
	uint32_t base = 160;
	CHECK(!decode_capture(edges, 0, &base, &decoded));
	CHECK(!decode_capture(edges, 3, &base, &decoded));
	CHECK(!decode_capture(edges, 33, &base, &decoded));
	unsigned count = encode_capture(0x345, 100, edges, false);
	CHECK(edges[0] == 100 && edges[1] == 120);
	CHECK(!decode_capture(edges, 4, &base, &decoded));
	// Reject a noise edge inside the first data interval and a leading edge
	// separated from the ESC response by a non-data idle interval.
	edges[1] = edges[0] + 1;
	CHECK(!decode_capture(edges, count, &base, &decoded));
	count = encode_capture(0x345, 100, edges, false);

	for (unsigned edge = count; edge > 0; --edge) { edges[edge] = edges[edge - 1]; }

	edges[0] = edges[1] - 101u;
	CHECK(!decode_capture(edges, count + 1, &base, &decoded));
	count = encode_capture(0x345, 100, edges, false);
	edges[2] = edges[1] + 1;
	CHECK(!decode_capture(edges, count, &base, &decoded));
	count = encode_capture(0x345, 100, edges, false); edges[2] = edges[1] + 101;
	CHECK(!decode_capture(edges, count, &base, &decoded));

	// Invalid GCR symbols are rejected even with a checksum-preserving zero
	// substitution; this distinguishes symbol validation from CRC validation.
	for (unsigned nibble = 1; nibble < 4; ++nibble) {
		const uint32_t raw = telemetry_gcr(0, false);
		const uint32_t invalid = (raw & ~(31u << (nibble * 5))) | (0x18u << (nibble * 5));
		count = encode_gcr(invalid, 100, edges, 20); base = 160;
		CHECK(!decode_capture(edges, count, &base, &decoded));
	}

	for (unsigned ticks = 18; ticks <= 22; ++ticks) for (unsigned p = 0; p < 4096; ++p) {
			count = encode_gcr(telemetry_gcr(p, false), 65490, edges, ticks); base = 160;
			CHECK(decode_capture(edges, count, &base, &decoded) && decoded == p);
			CHECK(base == 160 || base == 140 + ticks);

			if (p == 0x345) { CHECK(base == 140 + ticks); }
		}

	for (unsigned p = 0; p < 4096; ++p) {
		count = encode_capture(p, 100, edges, false); base = 160;

		// Alternating +/- one tick of edge jitter preserves the period.
		for (unsigned edge = 2; edge < count; ++edge) { edges[edge] += (edge & 1) ? 1 : (uint16_t) -1; }

		CHECK(decode_capture(edges, count, &base, &decoded) && decoded == p);
		CHECK(base >= 158 && base <= 162);
	}
}

static void test_telemetry(void)
{
	edt_enabled = false;

	for (unsigned p = 0; p < 4096; ++p) {
		save_telemetry(0, p);
		int erpm = -1;
		unsigned period = (p & 511u) << (p >> 9);
		int expected = period == 0 || period == 65408 ? 0 : (600000u + period / 2u) / period;
		CHECK(up_bdshot_get_erpm(0, &erpm) == 0 && erpm == expected);
		CHECK(up_bdshot_get_erpm(0, &erpm) == -EAGAIN);
	}

	memset(samples, 0, sizeof(samples));

	for (unsigned n = 0; n < 200; ++n) { save_telemetry(0, 0x345); }

	CHECK(up_bdshot_channel_online(0)); capture_failed(0); CHECK(up_bdshot_channel_online(0));
	save_telemetry(0, 0x345); CHECK(up_bdshot_channel_online(0));

	for (unsigned n = 0; n < 199; ++n) { capture_failed(0); }

	CHECK(up_bdshot_channel_online(0)); capture_failed(0); CHECK(!up_bdshot_channel_online(0));
	edt_enabled = true;
	static const unsigned type[4] = {2, 4, 6, 14};

	for (unsigned t = 0; t < 4; ++t) for (unsigned value = 0; value < 256; ++value) {
			save_telemetry(0, (type[t] << 8) | value); uint8_t result = 0;
			CHECK(up_bdshot_get_extended_telemetry(0, type[t], &result) == 0 && result == value);
			CHECK(up_bdshot_get_extended_telemetry(0, type[t], &result) == -EAGAIN);
		}
}

static void test_packets(void)
{
	static const unsigned rates[] = {150000, 300000, 600000, 1200000};

	for (unsigned rate = 0; rate < 4; ++rate) for (unsigned bidir = 0; bidir < 2; ++bidir) {
			CHECK(up_dshot_init(255, bidir ? 255 : 0, rates[rate], false) == 255);

			for (unsigned data = 0; data <= 2047; ++data) for (unsigned request = 0; request < 2; ++request) {
					unsigned channel = data % 8, timer = channel / 4, slot = channel % 4;
					dshot_motor_data_set(channel, data, request);
					uint16_t payload = (data << 1) | request;
					uint16_t crc = (payload ^ (payload >> 4) ^ (payload >> 8)) & 15;

					if (bidir) { crc ^= 15; }

					uint16_t packet = (payload << 4) | crc;
					const dshot_timing_t *timing = io_timer_get_dshot_timing(timer, rates[rate]);
					CHECK(timing->ticks * timing->prescaler * rates[rate] == 288000000u);
					CHECK(timing->bit_0 * 8 == timing->ticks * 3 && timing->bit_1 * 4 == timing->ticks * 3);

					for (unsigned bit = 0; bit < 16; ++bit) {
						if (pending[timer][bit * 4 + slot] != ((packet & (0x8000u >> bit)) ? timing->bit_1 : timing->bit_0)) {
							fprintf(stderr, "packet rate=%u bidir=%u data=%u request=%u channel=%u bit=%u got=%u packet=%04x\n", rates[rate], bidir, data, request,
								channel, bit, pending[timer][bit * 4 + slot], packet);
						}

						CHECK(pending[timer][bit * 4 + slot] == ((packet & (0x8000u >> bit)) ? timing->bit_1 : timing->bit_0));
					}

					CHECK(pending[timer][64 + slot] == 0 && pending[timer][68 + slot] == 0);
				}
		}
}

static void dma_irq(DMA_HANDLE d, uint8_t status) { CHECK(d && d->active && d->callback); d->callback(d, status, d->arg); }
static void finish_transmit(unsigned timer)
{
	DMA_HANDLE dma = timers[timer].output_dma;

	for (unsigned ch = 0; ch < 4; ++ch) {
		const unsigned pin = timer_io_channels[timer * 4 + ch].gpio_in & 255;
		CHECK(gpio_state[pin] == timer_io_channels[timer * 4 + ch].gpio_out);
	}

	CHECK(dma->active && dma->remaining == 72 && dma->peripheral == io_timers[timer].base + 0x4c);
	CHECK((dma->config & DSHOT_DMA_CONFIG) == DSHOT_DMA_CONFIG && (dma->config & DMA_CCR_DIR));
	uint16_t active[4] = {0}, preload[4] = {0};
	unsigned emitted[4] = {0};
	const uint16_t *words = (const uint16_t *)dma->memory;

	for (unsigned update = 0; update < 18; ++update) {
		// A complete active bit elapses before update. Shadow then latches;
		// the update DMA writes the following preload row.
		for (unsigned ch = 0; ch < 4; ++ch) {
			if (active[ch]) {
				CHECK(emitted[ch] < 16);
				CHECK(active[ch] == output[timer][emitted[ch] * 4 + ch]);
				++emitted[ch];
			}

			active[ch] = preload[ch]; preload[ch] = words[update * 4 + ch];
		}

		dma->remaining -= 4;
	}

	for (unsigned ch = 0; ch < 4; ++ch) { CHECK(emitted[ch] == 16 && active[ch] == 0 && preload[ch] == 0); }

	dma_irq(dma, DMA_STATUS_TCIF);
}
static void complete_capture(unsigned timer, uint16_t payload)
{
	dshot_timer_t *state = &timers[timer];
	CHECK(state->phase == DSHOT_CAPTURE && state->capture_call.pending);
	const unsigned physical = state->capture_channel, hw = physical == 3 ? 2 : physical;
	CHECK(state->capture_dma->map == io_timers[timer].dshot.dma_map_ch[hw]);
	CHECK(state->capture_dma->peripheral == io_timers[timer].base + 0x34 + hw * 4);
	CHECK(!(state->capture_dma->config & DMA_CCR_DIR));
	CHECK(rCCER(timer) == ((uint32_t)(GTIM_CCER_CC1E | GTIM_CCER_CC1P | GTIM_CCER_CC1NP) << (hw * 4)));
	CHECK((rCCER(timer) & (1u << 15)) == 0);
	CHECK(rDIER(timer) == ((uint32_t)GTIM_DIER_CC1DE << hw));

	for (unsigned ch = 0; ch < 4; ++ch) {
		const unsigned pin = timer_io_channels[timer * 4 + ch].gpio_in & 255;
		CHECK((gpio_state[pin]&GPIO_MODE_MASK) == (ch == physical ? GPIO_ALT : GPIO_INPUT));
		CHECK((gpio_state[pin]&GPIO_PUPD_MASK) == GPIO_PULLUP);
	}

	const unsigned count = encode_capture(payload, 65490, capture[timer], false);
	state->capture_dma->remaining = 32 - count;
	struct hrt_call *call = &state->capture_call;
	now_us = call->time; call->pending = false; call->cb(call->arg);
	CHECK(state->phase == DSHOT_IDLE && state->capture_dma == NULL);
	CHECK(rDIER(timer) == 0 && !(rCR1(timer) & GTIM_CR1_CEN));

	// The completed response remains released until the following transmit.
	for (unsigned ch = 0; ch < 4; ++ch) {
		const unsigned pin = timer_io_channels[timer * 4 + ch].gpio_in & 255;
		CHECK((gpio_state[pin] & GPIO_PUPD_MASK) == GPIO_PULLUP);
	}

	const unsigned channel = timer * 4 + physical;
	int erpm;
	CHECK(up_bdshot_get_erpm(channel, &erpm) == 0);
	const unsigned period = (payload & 511u) << (payload >> 9);
	CHECK(erpm == (period == 0 || period == 65408 ? 0 : (600000u + period / 2) / period));
}
static unsigned used_dma(void) { unsigned used = 0; for (unsigned i = 0; i < 7; ++i) used += dma_channels[i].used; return used; }

static void finish_empty_capture(unsigned timer)
{
	dshot_timer_t *state = &timers[timer];
	CHECK(state->phase == DSHOT_CAPTURE && state->capture_call.pending);
	CHECK(state->capture_dma->remaining == 32);
	const unsigned channel = timer * 4 + state->capture_channel;
	const uint32_t errors = samples[channel].errors;
	struct hrt_call *call = &state->capture_call;
	now_us = call->time; call->pending = false; call->cb(call->arg);
	CHECK(state->phase == DSHOT_IDLE && state->capture_dma == NULL);
	CHECK(rDIER(timer) == 0 && !(rCR1(timer) & GTIM_CR1_CEN));
	CHECK(samples[channel].errors == errors + 1 && !samples[channel].erpm_ready);
	CHECK(up_bdshot_get_ready_mask() & (1u << channel));
}

static void test_capture_gpio_recovery(void)
{
	CHECK(up_dshot_init(255, 255, 600000, false) == 255);
	CHECK(up_dshot_arm(true) == 0); now_us = capture_start_time + 1;
	const unsigned gpio_before = gpio_configurations;
	const unsigned allocations_before = dma_allocations;
	const unsigned frees_before = dma_frees;

	for (unsigned frame = 0; frame < 1000; ++frame) {
		up_dshot_trigger();
		CHECK(timers[0].phase == DSHOT_TRANSMIT && timers[1].phase == DSHOT_TRANSMIT);
		finish_transmit(0); finish_transmit(1);

		if (frame < 8) {
			complete_capture(0, 0x345); complete_capture(1, 0x456);

		} else {
			finish_empty_capture(0); finish_empty_capture(1);
		}

		CHECK(used_dma() == 2 && !timers[0].capture_call.pending && !timers[1].capture_call.pending);
	}

	// Each frame prepares four output pins and five capture pins per timer.
	// Restoring output again at capture completion would add eight calls/frame.
	CHECK(gpio_configurations - gpio_before == 1000 * 2 * (4 + 5));
	CHECK(dma_allocations - allocations_before == 2000 && dma_frees - frees_before == 2000);

	// Reinitialisation during an active response cancels both deadlines and DMA.
	up_dshot_trigger(); finish_transmit(0); finish_transmit(1);
	CHECK(used_dma() == 4);
	CHECK(up_dshot_init(255, 255, 600000, false) == 255);
	CHECK(used_dma() == 2 && !timers[0].capture_call.pending && !timers[1].capture_call.pending);
	CHECK(!outputs_armed && timers[0].phase == DSHOT_IDLE && timers[1].phase == DSHOT_IDLE);
	CHECK(up_dshot_init(0, 0, 600000, false) == 0 && used_dma() == 0);
}

static void test_state_machine(void)
{
	CHECK(up_dshot_init(255, 255, 600000, false) == 255); CHECK(used_dma() == 2);
	CHECK(up_dshot_arm(true) == 0); now_us = capture_start_time + 1;

	for (unsigned round = 0; round < 8; ++round) {
		dshot_motor_data_set(round % 8, 500 + round, false);
		up_dshot_trigger(); CHECK(timers[0].phase == DSHOT_TRANSMIT && timers[1].phase == DSHOT_TRANSMIT);
		uint16_t snapshot[72]; memcpy(snapshot, output[0], sizeof(snapshot));
		dshot_motor_data_set(0, 2047, true); CHECK(memcmp(snapshot, output[0], sizeof(snapshot)) == 0);
		up_dshot_trigger(); CHECK(memcmp(snapshot, output[0], sizeof(snapshot)) == 0);
		finish_transmit(0); finish_transmit(1); CHECK(used_dma() == 4);
		complete_capture(0, 0x345); complete_capture(1, 0x456); CHECK(used_dma() == 2);
		now_us += 30;
	}

	up_dshot_trigger();
	dma_irq(timers[0].output_dma, DMA_STATUS_ERROR); CHECK(timers[0].phase == DSHOT_IDLE && timers[0].dma_errors == 1);
	finish_transmit(1); dma_irq(timers[1].capture_dma, DMA_STATUS_ERROR); CHECK(timers[1].capture_error);
	struct hrt_call *call = &timers[1].capture_call; now_us = call->time; call->pending = false; call->cb(call->arg);
	CHECK(timers[1].phase == DSHOT_IDLE && used_dma() == 2);
	now_us += 30; up_dshot_trigger(); allocation_fail_map = 61;
	timers[0].capture_channel = 3; finish_transmit(0);
	CHECK(timers[0].phase == DSHOT_IDLE && timers[0].capture_dma == NULL);
	allocation_fail_map = -1; finish_transmit(1); complete_capture(1, 0x345);
	now_us += 30; up_dshot_trigger(); finish_transmit(0); finish_transmit(1);
	CHECK(used_dma() == 4); CHECK(up_dshot_arm(false) == 0); CHECK(used_dma() == 2);
	CHECK(!timers[0].capture_call.pending && !timers[1].capture_call.pending);
	CHECK(timers[0].phase == DSHOT_IDLE && timers[1].phase == DSHOT_IDLE);
	CHECK((rCCER(0) & 0x1111u) == 0 && (rCCER(1) & 0x1111u) == 0);
	up_dshot_trigger(); CHECK(!timers[0].output_dma->active);

	for (unsigned n = 0; n < 50; ++n) { CHECK(up_dshot_init(255, n % 2 ? 255 : 0, 600000, false) == 255); CHECK(used_dma() == 2); }

	allocation_fail_map = 65; CHECK(up_dshot_init(255, 255, 600000, false) == 240); CHECK(used_dma() == 1);
	allocation_fail_map = -1; CHECK(up_dshot_init(255, 255, 600000, false) == 255); CHECK(used_dma() == 2);
	CHECK(up_dshot_init(0, 0, 600000, false) == 0); CHECK(used_dma() == 0);
	CHECK(up_dshot_init(256, 0, 600000, false) == -EINVAL);
	CHECK(up_dshot_init(15, 1, 600000, false) == -EINVAL);
	CHECK(up_dshot_init(15, 16, 600000, false) == -EINVAL);
	CHECK(up_dshot_init(255, 255, 100000, false) == -EINVAL);
	CHECK(up_bdshot_get_erpm(8, NULL) == -EINVAL);
	CHECK(up_bdshot_get_extended_telemetry(0, 3, NULL) == -EINVAL);
	CHECK(io_timer_get_dshot_capture_channel(0, 3) == 2);
	CHECK(io_timer_get_dshot_capture_channel(1, 3) == 2);
	CHECK(io_timer_get_dshot_capture_channel(2, 0) == -EINVAL);
	CHECK(io_timer_get_dshot_capture_channel(0, 4) == -EINVAL);

	// Normal output and bidirectional output may share separate timer groups.
	CHECK(up_dshot_init(0xf3, 0xf0, 300000, false) == 0xf3);
	CHECK(!up_bdshot_channel_capture_supported(0) && up_bdshot_channel_capture_supported(7));
	uint16_t saved[72]; memcpy(saved, pending[0], sizeof(saved));
	dshot_motor_data_set(2, 123, true); dshot_motor_data_set(0, 2048, true); dshot_motor_data_set(8, 123, true);
	CHECK(memcmp(saved, pending[0], sizeof(saved)) == 0);
	CHECK(up_dshot_arm(true) == 0); now_us = capture_start_time - 1000;
	up_dshot_trigger();
	// Both timers stop after TX until the ESC boot delay has expired.
	DMA_HANDLE first = timers[0].output_dma, second = timers[1].output_dma;
	dma_irq(first, DMA_STATUS_TCIF); dma_irq(second, DMA_STATUS_TCIF);
	CHECK(timers[0].phase == DSHOT_IDLE && timers[1].phase == DSHOT_IDLE && used_dma() == 2);
	up_dshot_trigger(); CHECK(!first->active && !second->active);
	now_us += 29; up_dshot_trigger(); CHECK(!first->active && !second->active);
	now_us += 1; up_dshot_trigger(); CHECK(first->active && second->active);
	CHECK(up_dshot_arm(false) == 0);
	// Delayed completion callbacks after disarm cannot restart a timer or DMA.
	output_complete(first, DMA_STATUS_TCIF, &timers[0]); capture_complete(&timers[1]);
	CHECK(!first->active && !second->active && used_dma() == 2);
	CHECK(up_dshot_init(0, 0, 600000, false) == 0 && used_dma() == 0);

	// A preexisting PWM owner retains its timer/channel when DShot is requested.
	CHECK(io_timer_channel_init(0, IOTimerChanMode_PWMOut, NULL, NULL) == 0);
	CHECK(up_dshot_init(255, 255, 600000, false) == 240 && used_dma() == 1);
	CHECK(io_timer_get_channel_mode(0) == IOTimerChanMode_PWMOut);
	CHECK(io_timer_channel_init(0, IOTimerChanMode_NotUsed, NULL, NULL) == 0);
	CHECK(io_timer_unallocate_timer(0) == 0);
	CHECK(up_dshot_init(0, 0, 600000, false) == 0 && used_dma() == 0);
}

int main(void)
{
	void *registers = map_registers();
	CHECK(registers == (void *)0x40000000u);

	for (unsigned timer = 0; timer < 2; ++timer) {
		io_timer_set_oneshot_mode(timer);
		CHECK(rARR(timer) == UINT16_MAX);
		CHECK(rPSC(timer) == io_timers[timer].clock_freq / BOARD_ONESHOT_FREQ - 1);
		CHECK(rEGR(timer) == GTIM_EGR_UG);
	}

	test_decode(); test_packets(); test_state_machine(); test_capture_gpio_recovery(); test_telemetry();
	CHECK(critical_depth == 0); CHECK(dma_allocations == dma_frees);
	printf("PASS %u checks: whole production DShot+IO timer backend, 32768 valid wire frames with actual ESC start edges (rollover, rate drift, jitter), 8192 corrupted CRC frames, 8192 missing start edges, noise/truncation/GCR/timing bounds, 32768 transmit frames, eight channel DMA/capture state recovery.\n",
	       checks);
	unmap_registers(registers); return 0;
}
