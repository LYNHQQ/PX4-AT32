/****************************************************************************
 *
 *   Copyright (C) 2017-2026 PX4 Development Team. All rights reserved.
 *   Author: @author David Sidrane <david_s5@nscdg.com>
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
 * @file board_identity.c
 * AT32F435 factory UID and PX4 GUID interfaces.
 */

#include <px4_platform_common/px4_config.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <arm_internal.h>

void board_get_uuid32(uuid_uint32_t uuid_words)
{
	for (unsigned i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; ++i) {
		// New PX4 targets expose the most significant word first.
		uuid_words[i] = getreg32(AT32_SYSMEM_UID + (PX4_CPU_UUID_WORD32_LENGTH - 1 - i) * sizeof(uint32_t));
	}
}

void board_get_uuid(uuid_byte_t uuid_bytes)
{
	uuid_uint32_t words;
	board_get_uuid32(words);

	for (unsigned i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; ++i) {
		for (unsigned j = 0; j < sizeof(uint32_t); ++j) {
			uuid_bytes[i * sizeof(uint32_t) + j] = words[i] >> ((sizeof(uint32_t) - 1 - j) * 8);
		}
	}
}

int board_get_uuid32_formated(char *format_buffer, int size, const char *format, const char *separator)
{
	if (size <= 0) {
		return -EINVAL;
	}

	uuid_uint32_t words;
	board_get_uuid32(words);
	int offset = 0;
	format_buffer[0] = '\0';

	for (unsigned i = 0; i < PX4_CPU_UUID_WORD32_LENGTH && offset < size - 1; ++i) {
		int written = snprintf(format_buffer + offset, size - offset, format, words[i]);

		if (written < 0 || written >= size - offset) {
			return written < 0 ? -EINVAL : 0;
		}

		offset += written;

		if (separator && i + 1 < PX4_CPU_UUID_WORD32_LENGTH) {
			written = snprintf(format_buffer + offset, size - offset, "%s", separator);

			if (written < 0 || written >= size - offset) {
				return written < 0 ? -EINVAL : 0;
			}

			offset += written;
		}
	}

	return 0;
}

int board_get_mfguid(mfguid_t mfguid)
{
	board_get_uuid(mfguid);
	return PX4_CPU_MFGUID_BYTE_LENGTH;
}

int board_get_mfguid_formated(char *format_buffer, int size)
{
	if (size <= 0) {
		return -EINVAL;
	}

	mfguid_t mfguid;
	board_get_mfguid(mfguid);
	int offset = 0;
	format_buffer[0] = '\0';

	for (unsigned i = 0; i < PX4_CPU_MFGUID_BYTE_LENGTH && offset + 2 < size; ++i) {
		offset += snprintf(format_buffer + offset, size - offset, "%02x", mfguid[i]);
	}

	return offset;
}

int board_get_px4_guid(px4_guid_t px4_guid)
{
	const uint16_t soc_arch_id = PX4_SOC_ARCH_ID;
	memset(px4_guid, 0, PX4_GUID_BYTE_LENGTH);
	px4_guid[0] = soc_arch_id >> 8;
	px4_guid[1] = soc_arch_id & 0xff;
	board_get_uuid(px4_guid + PX4_GUID_BYTE_LENGTH - PX4_CPU_UUID_BYTE_LENGTH);
	return PX4_GUID_BYTE_LENGTH;
}

int board_get_px4_guid_formated(char *format_buffer, int size)
{
	if (size <= 0) {
		return -EINVAL;
	}

	px4_guid_t guid;
	board_get_px4_guid(guid);
	unsigned bytes = (unsigned)(size - 1) / 2;

	if (bytes > PX4_GUID_BYTE_LENGTH) {
		bytes = PX4_GUID_BYTE_LENGTH;
	}

	int offset = 0;
	format_buffer[0] = '\0';

	// Short output retains the least significant bytes, as required by the API.
	for (unsigned i = PX4_GUID_BYTE_LENGTH - bytes; i < PX4_GUID_BYTE_LENGTH; ++i) {
		offset += snprintf(format_buffer + offset, size - offset, "%02x", guid[i]);
	}

	return offset;
}
