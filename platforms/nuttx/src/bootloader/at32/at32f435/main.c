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

/*
 * AT32F435 board support for the bootloader.
 *
 * Same structure as the STM32 bootloader: the chip independent protocol lives
 * in platforms/nuttx/src/bootloader/common, this file provides the flash,
 * identification, LED, clock-reset and jump primitives for the AT32F435.
 */

#include <px4_defines.h>

#include "hw_config.h"

#include <arch/board/board.h>
#include <arch/barriers.h>
#include <arch/irq.h>
#include <arm_internal.h>
#include <at32_pwr.h>
#include <hardware/at32_dbgmcu.h>
#include <hardware/at32_flash.h>
#include <hardware/at32_rtcc.h>
#include <hardware/at32f43xxx_memorymap.h>
#include <hardware/at32f43xxx_rcc.h>
#include <nvic.h>
#include <nuttx/arch.h>
#include <nuttx/progmem.h>
#include <px4_arch/micro_hal.h>
#include <lib/systick.h>
#include <lib/flash_cache.h>

#include "bl.h"
#include "uart.h"

#define MK_GPIO_INPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT))

#define BOOTLOADER_RESERVATION_SIZE	(64 * 1024)

/* AT32F435VM flash is organised in uniform 4 KiB pages, so a protocol sector
 * is one page.
 */
#define FLASH_SECTOR_SIZE		AT32_FLASH_PAGESIZE

/*
 * If APP_RESERVATION_SIZE is greater than 0 and
 * FLASH_BASED_PARAMS is defined, throw a compile error
 */
#if defined(FLASH_BASED_PARAMS) && (APP_RESERVATION_SIZE <= 0)
# error "APP_RESERVATION_SIZE must be greater than 0 if FLASH_BASED_PARAMS is defined"
#endif

#define APP_SIZE_MAX			(BOARD_FLASH_SIZE - (BOOTLOADER_RESERVATION_SIZE + APP_RESERVATION_SIZE))

/* The application region implied by the linker scripts (bootloader_script.ld
 * and the application's script.ld) must be the region this bootloader erases
 * and reports to the uploader, otherwise a valid upload can be rejected or an
 * uploaded image can be erased. The AT32F435VM has a fixed 4032 KiB of flash,
 * which is also what hw_config.h reads back from the chip at run time; the
 * check below uses the fixed geometry because _FLASH_KBYTES is not a
 * preprocessor constant.
 */
#define AT32F435VM_FLASH_BASE		0x08000000
#define AT32F435VM_FLASH_SIZE		(4032 * 1024)
#define APP_REGION_SIZE			((AT32F435VM_FLASH_BASE + AT32F435VM_FLASH_SIZE - APP_RESERVATION_SIZE) - APP_LOAD_ADDRESS)

#if ((AT32F435VM_FLASH_SIZE - (BOOTLOADER_RESERVATION_SIZE + APP_RESERVATION_SIZE)) != APP_REGION_SIZE)
# error "APP_SIZE_MAX does not match the application linker region"
#endif

/* context passed to cinit */
#if INTERFACE_USART
# define BOARD_INTERFACE_CONFIG_USART	INTERFACE_USART_CONFIG
#endif
#if INTERFACE_USB
# define BOARD_INTERFACE_CONFIG_USB  INTERFACE_USB_CONFIG
#endif

/* board definition */
struct boardinfo board_info = {
	.board_type	= BOARD_TYPE,
	.board_rev	= 0,
	.fw_size	= 0,
	.systick_mhz	= AT32_SYSCLK_FREQUENCY / 1000000,
};

static void board_init(void);

#define BOOT_RTC_SIGNATURE          0xb007b007

/* State of an inserted USB cable (always false without a VBUS sense pin) */
static bool usb_connected = false;

static uint32_t board_get_rtc_signature(void)
{
	/* enable the backup domain */
	at32_pwr_enablebkp(true);

	uint32_t result = getreg32(AT32_RTC_BK0R);

	/* disable the backup domain */
	at32_pwr_enablebkp(false);

	return result;
}

static void
board_set_rtc_signature(uint32_t sig)
{
	/* enable the backup domain */
	at32_pwr_enablebkp(true);

	putreg32(sig, AT32_RTC_BK0R);

	/* disable the backup domain */
	at32_pwr_enablebkp(false);
}

static bool board_test_force_pin(void)
{
#if defined(BOARD_FORCE_BL_PIN_IN) && defined(BOARD_FORCE_BL_PIN_OUT)
	/* two pins strapped together */
	volatile unsigned samples = 0;
	volatile unsigned vote = 0;

	for (volatile unsigned cycles = 0; cycles < 10; cycles++) {
		px4_arch_gpiowrite(BOARD_FORCE_BL_PIN_OUT, 1);

		for (unsigned count = 0; count < 20; count++) {
			if (px4_arch_gpioread(BOARD_FORCE_BL_PIN_IN) != 0) {
				vote++;
			}

			samples++;
		}

		px4_arch_gpiowrite(BOARD_FORCE_BL_PIN_OUT, 0);

		for (unsigned count = 0; count < 20; count++) {
			if (px4_arch_gpioread(BOARD_FORCE_BL_PIN_IN) == 0) {
				vote++;
			}

			samples++;
		}
	}

	/* the idea here is to reject wire-to-wire coupling, so require > 90% agreement */
	if ((vote * 100) > (samples * 90)) {
		return true;
	}

#endif
#if defined(BOARD_FORCE_BL_PIN)
	/* single pin pulled up or down */
	volatile unsigned samples = 0;
	volatile unsigned vote = 0;

	for (samples = 0; samples < 200; samples++) {
		if ((px4_arch_gpioread(BOARD_FORCE_BL_PIN) ? 1 : 0) == BOARD_FORCE_BL_STATE) {
			vote++;
		}
	}

	/* reject a little noise */
	if ((vote * 100) > (samples * 90)) {
		return true;
	}

#endif
	return false;
}

uint32_t
board_get_devices(void)
{
	uint32_t devices = BOOT_DEVICES_SELECTION;

	if (usb_connected) {
		devices &= BOOT_DEVICES_FILTER_ONUSB;
	}

	return devices;
}

static void
board_init(void)
{
	/* fix up the max firmware size, we have to read memory to get this */
	board_info.fw_size = APP_SIZE_MAX;

#if defined(BOARD_POWER_PIN_OUT)
	/* Configure the Power pins */
	px4_arch_configgpio(BOARD_POWER_PIN_OUT);
	px4_arch_gpiowrite(BOARD_POWER_PIN_OUT, BOARD_POWER_ON);
#endif

#if defined(BOARD_FORCE_BL_PIN_IN) && defined(BOARD_FORCE_BL_PIN_OUT)
	/* configure the force BL pins */
	px4_arch_configgpio(BOARD_FORCE_BL_PIN_IN);
	px4_arch_configgpio(BOARD_FORCE_BL_PIN_OUT);
#endif

#if defined(BOARD_FORCE_BL_PIN)
	/* configure the force BL pins */
	px4_arch_configgpio(BOARD_FORCE_BL_PIN);
#endif

#if defined(BOARD_PIN_LED_ACTIVITY)
	/* Initialize LEDs */
	px4_arch_configgpio(BOARD_PIN_LED_ACTIVITY);
#endif
#if defined(BOARD_PIN_LED_BOOTLOADER)
	/* Initialize LEDs */
	px4_arch_configgpio(BOARD_PIN_LED_BOOTLOADER);
#endif
}

void
board_deinit(void)
{
	/* Reset the USB controller so the application starts from a known state */
#ifdef CONFIG_AT32_OTGFS
	up_disable_irq(AT32_IRQ_OTGFS);
	modifyreg32(AT32_CRM_AHBRST2, 0, CRM_AHBRST2_OTGFS1RST);
	modifyreg32(AT32_CRM_AHBRST2, CRM_AHBRST2_OTGFS1RST, 0);
	putreg32(1u << ((AT32_IRQ_OTGFS - AT32_IRQ_FIRST) & 31),
		 NVIC_IRQ_CLRPEND(AT32_IRQ_OTGFS - AT32_IRQ_FIRST));
#endif
	putreg32(NVIC_INTCTRL_PENDSTCLR | NVIC_INTCTRL_PENDSVCLR, NVIC_INTCTRL);

#if defined(BOARD_FORCE_BL_PIN_IN) && defined(BOARD_FORCE_BL_PIN_OUT)
	/* deinitialise the force BL pins */
	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_FORCE_BL_PIN_IN));
	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_FORCE_BL_PIN_OUT));
#endif

#if defined(BOARD_FORCE_BL_PIN)
	/* deinitialise the force BL pin */
	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_FORCE_BL_PIN));
#endif

#if defined(BOARD_POWER_PIN_OUT) && defined(BOARD_POWER_PIN_RELEASE)
	/* deinitialize the POWER pin - with the assumption the hold up time of
	 * the voltage being bleed off by an inupt pin impedance will allow
	 * enough time to boot the app
	 */
	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_POWER_PIN_OUT));
#endif

#if defined(BOARD_PIN_LED_ACTIVITY)
	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_PIN_LED_ACTIVITY));
#endif
#if defined(BOARD_PIN_LED_BOOTLOADER)
	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_PIN_LED_BOOTLOADER));
#endif
}

inline void arch_systic_init(void)
{
	/* (re)start the timer system */
	systick_set_clocksource(CLKSOURCE_PROCESOR);
	systick_set_reload(board_info.systick_mhz * 1000);  /* 1ms tick, magic number */
	systick_interrupt_enable();
	systick_counter_enable();
}

inline void arch_systic_deinit(void)
{
	/* kill the systick interrupt */
	systick_interrupt_disable();
	systick_counter_disable();
	systick_set_clocksource(CLKSOURCE_EXTERNAL);
	systick_set_reload(0);  /* 1ms tick, magic number */
}

static inline void
clock_init(void)
{
	// Done by Nuttx
}

void
clock_deinit(void)
{
	/* Switch away from PLL before the application's startup reconfigures it. */
	modifyreg32(AT32_CRM_CTRL, 0, CRM_CTRL_HICKEN);

	while ((getreg32(AT32_CRM_CTRL) & CRM_CTRL_HICKSTBL) == 0) {}

	modifyreg32(AT32_CRM_CFG, CRM_CFG_SCLKSEL_MASK, CRM_CFG_SEL_HICK);

	while ((getreg32(AT32_CRM_CFG) & CRM_CFG_SCLKSTSL_MASK) != CRM_CFG_STS_HICK) {}

	modifyreg32(AT32_CRM_CTRL, CRM_CTRL_PLLEN, 0);

	while ((getreg32(AT32_CRM_CTRL) & CRM_CTRL_PLLSTBL) != 0) {}

	/* Keep HEXT running: the RTC uses it across bootloader/application jumps. */
}

void arch_flash_lock(void)
{
	at32_flash_lock();
}

void arch_flash_unlock(void)
{
	fc_reset();
	at32_flash_unlock();
}

ssize_t arch_flash_write(uintptr_t address, const void *buffer, size_t buflen)
{
	return up_progmem_write(address, buffer, buflen);
}

inline void arch_setvtor(const uint32_t *address)
{
	putreg32((uint32_t)address, NVIC_VECTAB);
	UP_DSB();
	UP_ISB();
}

uint32_t
flash_func_sector_size(unsigned sector)
{
	if (sector <= BOARD_FLASH_SECTORS) {
		return FLASH_SECTOR_SIZE;
	}

	return 0;
}

void
flash_func_erase_sector(unsigned sector, bool force)
{
	if (sector > BOARD_FLASH_SECTORS || (int)sector < BOARD_FIRST_FLASH_SECTOR_TO_ERASE) {
		return;
	}

	if (force || (up_progmem_ispageerased(sector) != 0)) {
		up_progmem_eraseblock(sector);
	}
}

void
flash_func_write_word(uintptr_t address, uint32_t word)
{
	address += APP_LOAD_ADDRESS;
	fc_write(address, word);
}

uint32_t flash_func_read_word(uintptr_t address)
{
	if (address & 3) {
		return 0;
	}

	return fc_read(address + APP_LOAD_ADDRESS);
}

uint32_t
flash_func_read_otp(uintptr_t address)
{
	return 0;
}

uint32_t get_mcu_id(void)
{
	return getreg32(AT32_DEBUG_IDCODE);
}

int get_mcu_desc(int max, uint8_t *revstr)
{
	uint32_t series = getreg32(AT32_DEBUG_SER_ID) & (0xff << DEBUG_SER_ID_SHIFT);
	uint32_t revid = (getreg32(AT32_DEBUG_IDCODE) & DEBUG_IDCODE_REVID_MASK) >> DEBUG_IDCODE_REVID_SHIFT;
	const char *desc = (series == DEBUG_SER_ID_F435) ? "AT32F435" : "AT32???????";
	static const char hex[] = "0123456789abcdef";

	uint8_t *endp = &revstr[max - 1];
	uint8_t *strp = revstr;

	while (strp < endp && *desc) {
		*strp++ = *desc++;
	}

	if (strp < endp) {
		*strp++ = ',';
	}

	/* There is no documented letter mapping for the AT32 revision, so report
	 * the raw revision identifier the debug unit returns.
	 */
	if (strp < endp) {
		*strp++ = 'r';
	}

	for (int shift = 12; shift >= 0 && strp < endp; shift -= 4) {
		*strp++ = hex[(revid >> shift) & 0xf];
	}

	return strp - revstr;
}

int check_silicon(void)
{
	return 0;
}

uint32_t
flash_func_read_sn(uintptr_t address)
{
	// read a word out of the unique chip ID area
	// it's 12 bytes, or 3 words.
	return *(uint32_t *)(address + AT32_SYSMEM_UID);
}

void
led_on(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		px4_arch_gpiowrite(BOARD_PIN_LED_ACTIVITY, BOARD_LED_ON);
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		px4_arch_gpiowrite(BOARD_PIN_LED_BOOTLOADER, BOARD_LED_ON);
#endif
		break;
	}
}

void
led_off(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		px4_arch_gpiowrite(BOARD_PIN_LED_ACTIVITY, BOARD_LED_OFF);
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		px4_arch_gpiowrite(BOARD_PIN_LED_BOOTLOADER, BOARD_LED_OFF);
#endif
		break;
	}
}

void
led_toggle(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		px4_arch_gpiowrite(BOARD_PIN_LED_ACTIVITY, px4_arch_gpioread(BOARD_PIN_LED_ACTIVITY) ^ 1);
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		px4_arch_gpiowrite(BOARD_PIN_LED_BOOTLOADER, px4_arch_gpioread(BOARD_PIN_LED_BOOTLOADER) ^ 1);
#endif
		break;
	}
}

/* we should know this, but we don't */
#ifndef SCB_CPACR
# define SCB_CPACR (*((volatile uint32_t *) (((0xE000E000UL) + 0x0D00UL) + 0x088)))
#endif

/* Make the actual jump to app */
void
arch_do_jump(const uint32_t *app_base)
{
	/* extract the stack and entrypoint from the app vector table and go */
	uint32_t stacktop = app_base[0];
	uint32_t entrypoint = app_base[1];

	uint32_t scratch;

	/* NuttX 12.12 runs thread mode on the PSP (arm_initialize_stack(), enabled
	 * whenever CONFIG_ARCH_INTERRUPTSTACK > 7). The application expects to start
	 * on the MSP, so select it again before handing over -- otherwise the app
	 * keeps running on the bootloader's stack and faults once it is clobbered.
	 */
	asm volatile(
		"mrs %0, control  \n"
		"bic %0, %0, #2   \n"
		"msr control, %0  \n"
		"isb sy           \n"
		"msr msp, %1      \n"
		"bx %2  \n"
		: "=&r"(scratch) : "r"(stacktop), "r"(entrypoint) :);

	// just to keep noreturn happy
	for (;;) ;
}

int
bootloader_main(int argc, char *argv[])
{
	bool try_boot = true;			/* try booting before we drop to the bootloader */
	unsigned timeout = BOOTLOADER_DELAY;	/* if nonzero, drop out of the bootloader after this time */

	/* Enable the FPU before we hit any FP instructions */
	SCB_CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 Full Access and set CP11 Full Access */

#if defined(BOARD_POWER_PIN_OUT)

	/* Here we check for the app setting the POWER_DOWN_RTC_SIGNATURE
	 * in this case, we reset the signature and wait to die
	 */
	if (board_get_rtc_signature() == POWER_DOWN_RTC_SIGNATURE) {
		board_set_rtc_signature(0);

		while (1);
	}

#endif

	/* do board-specific initialisation */
	board_init();

	/* configure the clock for bootloader activity */
	clock_init();

	/*
	 * Check the force-bootloader register; if we find the signature there, don't
	 * try booting.
	 */
	if (board_get_rtc_signature() == BOOT_RTC_SIGNATURE) {

		/*
		 * Don't even try to boot before dropping to the bootloader.
		 */
		try_boot = false;

		/*
		 * Don't drop out of the bootloader until something has been uploaded.
		 */
		timeout = 0;

		/*
		 * Clear the signature so that if someone resets us while we're
		 * in the bootloader we'll try to boot next time.
		 */
		board_set_rtc_signature(0);
	}

	/*
	 * Check if the force-bootloader pins are strapped; if strapped,
	 * don't try booting.
	 */
	if (board_test_force_pin()) {
		try_boot = false;
	}

#if INTERFACE_USB

	/*
	 * Check for USB connection - if present, don't try to boot, but set a timeout after
	 * which we will fall out of the bootloader.
	 *
	 * If the force-bootloader pins are tied, we will stay here until they are removed and
	 * we then time out.
	 */
#if defined(BOARD_VBUS)

	if (px4_arch_gpioread(BOARD_VBUS) != 0) {
		usb_connected = true;
		/* don't try booting before we set up the bootloader */
		try_boot = false;
	}

#elif defined(BOARD_USB_VBUS_SENSE_DISABLED)

	/* Without a VBUS sense input the controller cannot report a cable before the
	 * host enumerates, so USB presence must not hold us in the bootloader: a
	 * board with a valid application boots immediately and only stays here when
	 * the application or the ground station asked for it (redundant parameter
	 * register signature), or when there is nothing to boot.
	 */
#else
	try_boot = false;

#endif
#endif

	/* Try to boot the app if we think we should just go straight there */
	if (try_boot) {

		/* set the boot-to-bootloader flag so that if boot fails on reset we will stop here */
#ifdef BOARD_BOOT_FAIL_DETECT
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);
#endif

		/* try to boot immediately */
		jump_to_app();

		// If it failed to boot, reset the boot signature and stay in bootloader
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);

		/* booting failed, stay in the bootloader forever */
		timeout = 0;
	}


	/* start the interface */
#if INTERFACE_USART
	cinit(BOARD_INTERFACE_CONFIG_USART, USART);
#endif
#if INTERFACE_USB
	cinit(BOARD_INTERFACE_CONFIG_USB, USB);
#endif

	while (1) {
		/* run the bootloader, come back after an app is uploaded or we time out */
		bootloader(timeout);

		/* if the force-bootloader pins are strapped, just loop back */
		if (board_test_force_pin()) {
			continue;
		}

		/* set the boot-to-bootloader flag so that if boot fails on reset we will stop here */
#ifdef BOARD_BOOT_FAIL_DETECT
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);
#endif

		/* look to see if we can boot the app */
		jump_to_app();

		/* launching the app failed - stay in the bootloader forever */
		timeout = 0;
	}
}
