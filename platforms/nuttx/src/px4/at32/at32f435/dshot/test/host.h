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
#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <sys/mman.h>
#endif
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#define __EXPORT
#define __BEGIN_DECLS
#define __END_DECLS
#define BOARD_HAS_NO_CAPTURE 1
#define PWM_RATE_ONESHOT 0
#define OK 0
#ifndef ENOTSUP
#define ENOTSUP 95
#endif
#define UNUSED(v) ((void)(v))
#define AT32_TMR1_BASE 0x40010000u
#define AT32_TMR2_BASE 0x40000000u
#define AT32_TMR5_BASE 0x40000c00u
#define AT32_TMR8_BASE 0x40010400u
#define AT32_TMR20_BASE 0x40014c00u
#define DMA_CCR_DIR (1u<<4)
#define DMA_CCR_MINC (1u<<7)
#define DMA_CCR_PSIZE_16BITS (1u<<8)
#define DMA_CCR_MSIZE_16BITS (1u<<10)
#define DMA_CCR_PRIHI (2u<<12)
#define DMA_STATUS_ERROR 8u
#define DMA_STATUS_TCIF 2u
typedef unsigned irqstate_t;
typedef uint64_t hrt_abstime;
typedef int (*xcpt_t)(int, void *, void *);
struct hrt_call { bool pending; hrt_abstime time; void (*cb)(void *); void *arg; };
static hrt_abstime now_us;
static uint32_t gpio_state[256];
static unsigned gpio_configurations;
static unsigned critical_depth;
static irqstate_t px4_enter_critical_section(void) { return critical_depth++; }
static void px4_leave_critical_section(irqstate_t saved) { assert(critical_depth == saved + 1); --critical_depth; }
static hrt_abstime hrt_absolute_time(void) { return now_us; }
static void hrt_cancel(struct hrt_call *c) { c->pending = false; }
static void hrt_call_after(struct hrt_call *c, hrt_abstime delay, void (*cb)(void *), void *arg)
{ c->pending = true; c->time = now_us + delay; c->cb = cb; c->arg = arg; }
static int irq_attach(int irq, xcpt_t cb, void *arg) { return 0; }
static void up_enable_irq(int irq) {}
static void px4_arch_configgpio(uint32_t cfg) { ++gpio_configurations; gpio_state[cfg & 255u] = cfg; }
static uint32_t getreg32(uintptr_t a) { return *(volatile uint32_t *)a; }
static void putreg32(uint32_t v, uintptr_t a) { *(volatile uint32_t *)a = v; }
static void modifyreg32(uintptr_t a, uint32_t clear, uint32_t set) { putreg32((getreg32(a) & ~clear) | set, a); }
#define PX4_ERR(...) ((void)0)
#define PX4_INFO(...) ((void)0)
struct mock_dma;
typedef struct mock_dma *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE, uint8_t, void *);
struct mock_dma {
	bool used, active;
	unsigned map, setup_count, start_count, remaining, total;
	uintptr_t peripheral, memory;
	uint32_t config;
	dma_callback_t callback;
	void *arg;
};
static struct mock_dma dma_channels[7];
static int allocation_fail_map = -1;
static unsigned dma_allocations, dma_frees;
static DMA_HANDLE at32_dmachannel(unsigned map)
{
	if ((int)map == allocation_fail_map) { return NULL; }

	for (unsigned i = 0; i < 7; ++i) {
		if (!dma_channels[i].used) {
			memset(&dma_channels[i], 0, sizeof(dma_channels[i]));
			dma_channels[i].used = true; dma_channels[i].map = map; ++dma_allocations;
			return &dma_channels[i];
		}
	}

	return NULL;
}
static void at32_dmafree(DMA_HANDLE d) { assert(d && d->used && !d->active); d->used = false; ++dma_frees; }
static void at32_dmastop(DMA_HANDLE d) { assert(d && d->used); d->active = false; }
static unsigned at32_dmaresidual(DMA_HANDLE d) { assert(d && d->used && !d->active); return d->remaining; }
static void at32_dmasetup(DMA_HANDLE d, uintptr_t peripheral, uintptr_t memory, unsigned count, uint32_t config)
{
	assert(d && d->used && !d->active && count > 0 && count < 65536);
	// Production must stop the timer and DMA request before reconfiguring DMA.
	const uintptr_t timer_base = peripheral & ~(uintptr_t)255u;
	assert((getreg32(timer_base) & 1u) == 0 && getreg32(timer_base + 0x0cu) == 0);
	d->peripheral = peripheral; d->memory = memory; d->remaining = d->total = count; d->config = config;
	++d->setup_count;
}
static void at32_dmastart(DMA_HANDLE d, dma_callback_t cb, void *arg, bool half)
{
	assert(d && d->used && !d->active && d->remaining > 0 && !half);
	const uintptr_t timer_base = d->peripheral & ~(uintptr_t)255u;
	assert((getreg32(timer_base) & 1u) == 0 && getreg32(timer_base + 0x0cu) == 0);
	d->active = true; d->callback = cb; d->arg = arg; ++d->start_count;
}

static void *map_registers(void)
{
#if defined(_WIN32)
	return VirtualAlloc((void *)0x40000000u, 0x40000, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
#else
	// A hint must never overwrite an existing host mapping.
	void *memory = mmap((void *)0x40000000u, 0x40000, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);

	if (memory != MAP_FAILED && memory != (void *)0x40000000u) { munmap(memory, 0x40000); return NULL; }

	return memory == MAP_FAILED ? NULL : memory;
#endif
}
static void unmap_registers(void *memory)
{
#if defined(_WIN32)
	VirtualFree(memory, 0, MEM_RELEASE);
#else
	munmap(memory, 0x40000);
#endif
}
