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

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <sys/mount.h>
#include <syslog.h>

#include <nuttx/mtd/mtd.h>
#include <nuttx/spi/spi.h>
#include <px4_arch/micro_hal.h>

#include "board_config.h"

/* The pin reference permits these three parts. Probe their IDs before calling
 * a driver: NAND uses an extra dummy byte, and its reset opcode must not be
 * sent to an unidentified NOR device.
 */
#define FLASH_BUS 3
#define FLASH_READ_ID 0x9f
#define FLASH_DUMMY 0xff
#define FLASH_PROBE_FREQUENCY 1000000
#define M25P16_ID 0x202015
#define W25Q128FV_ID 0xef4018
#define W25N01GV_ID 0xefaa21
#define FLASH_DEVICE_PATH "/dev/mtd0"

int board_flash_initialize(void)
{
	struct spi_dev_s *spi = px4_spibus_initialize(FLASH_BUS);

	if (spi == NULL) {
		syslog(LOG_ERR, "External flash SPI bus unavailable\n");
		return -ENODEV;
	}

	uint8_t id[4];
	SPI_LOCK(spi, true);
	SPI_SETBITS(spi, 8);
	SPI_SETMODE(spi, SPIDEV_MODE0);
	SPI_SETFREQUENCY(spi, FLASH_PROBE_FREQUENCY);
	SPI_SELECT(spi, SPIDEV_FLASH(0), true);
	SPI_SEND(spi, FLASH_READ_ID);

	for (unsigned i = 0; i < sizeof(id); ++i) {
		id[i] = SPI_SEND(spi, FLASH_DUMMY);
	}

	SPI_SELECT(spi, SPIDEV_FLASH(0), false);
	SPI_LOCK(spi, false);

	const uint32_t nor_id = ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
	const uint32_t nand_id = ((uint32_t)id[1] << 16) | ((uint32_t)id[2] << 8) | id[3];
	struct mtd_dev_s *mtd = NULL;

	if (nor_id == M25P16_ID) {
		mtd = m25p_initialize(spi);

	} else if (nor_id == W25Q128FV_ID) {
		mtd = w25_initialize(spi);

	} else if (nand_id == W25N01GV_ID) {
		mtd = w25n_initialize(spi, 0);

	} else {
		syslog(LOG_WARNING, "Unsupported external flash ID %02x:%02x:%02x:%02x\n",
		       id[0], id[1], id[2], id[3]);
		return -ENODEV;
	}

	if (mtd == NULL) {
		syslog(LOG_ERR, "External flash driver initialisation failed\n");
		return -EIO;
	}

	struct mtd_geometry_s geometry;

	int ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)(uintptr_t)&geometry);

	if (ret < 0) {
		syslog(LOG_ERR, "External flash geometry unavailable: %d\n", ret);
		return ret;
	}

	syslog(LOG_INFO, "External flash: %" PRIu32 " blocks of %" PRIu32 " bytes\n",
	       geometry.neraseblocks, geometry.erasesize);
	ret = register_mtddriver(FLASH_DEVICE_PATH, mtd, 0755, NULL);

	if (ret < 0) {
		syslog(LOG_ERR, "External flash registration failed: %d\n", ret);
		return ret;
	}

	/* Never format on boot: the flash can contain Betaflight logs or another
	 * filesystem. Formatting is an explicit user operation via mklittlefs.
	 */
	ret = nx_mount(FLASH_DEVICE_PATH, CONFIG_BOARD_ROOT_PATH, "littlefs", 0, NULL);

	if (ret < 0) {
		syslog(LOG_WARNING, "External flash mount failed: %d. "
		       "Back up the flash before using mklittlefs %s %s\n",
		       ret, FLASH_DEVICE_PATH, CONFIG_BOARD_ROOT_PATH);
	}

	return ret;
}
