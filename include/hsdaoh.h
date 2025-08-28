/*
 * hsdaoh - High Speed Data Acquisition over MS213x USB3 HDMI capture sticks
 *
 * Copyright (C) 2024 by Steve Markgraf <steve@steve-m.de>
 *
 * based on librtlsdr:
 * Copyright (C) 2012-2024 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Dimitri Stolnikov <horiz0n@gmx.net>
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __HSDAOH_H
#define __HSDAOH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <hsdaoh_export.h>

#define HSDAOH_MAX_BUF_SIZE	(1920 * 1080 * 2)

typedef struct hsdaoh_data_info {
	void *ctx;
	unsigned char *buf;
	size_t len;			/* buffer length */
	uint16_t stream_id;
	uint32_t srate;
	uint8_t nchans;
	uint8_t bits_per_samp;
	bool is_signed;
	bool is_float;
	bool device_error;		/* device error happened, terminate application */
} hsdaoh_data_info_t;

typedef struct hsdaoh_dev hsdaoh_dev_t;

typedef enum
{
	OUT_FMT_RAW,
	OUT_FMT_UNPACKED,
	OUT_FMT_FLOAT
} hsdaoh_output_format_t;

HSDAOH_API uint32_t hsdaoh_get_device_count(void);

HSDAOH_API const char* hsdaoh_get_device_name(uint32_t index);

/*!
 * Get USB device strings.
 *
 * NOTE: The string arguments must provide space for up to 256 bytes.
 *
 * \param index the device index
 * \param manufact manufacturer name, may be NULL
 * \param product product name, may be NULL
 * \param serial serial number, may be NULL
 * \return 0 on success
 */
HSDAOH_API int hsdaoh_get_device_usb_strings(uint32_t index,
					     char *manufact,
					     char *product,
					     char *serial);

/*!
 * Get device index by USB serial string descriptor.
 *
 * \param serial serial string of the device
 * \return device index of first device where the name matched
 * \return -1 if name is NULL
 * \return -2 if no devices were found at all
 * \return -3 if devices were found, but none with matching name
 */
HSDAOH_API int hsdaoh_get_index_by_serial(const char *serial);

HSDAOH_API int hsdaoh_open(hsdaoh_dev_t **dev, uint32_t index);

HSDAOH_API int hsdaoh_close(hsdaoh_dev_t *dev);

/* configuration functions */

/*!
 * Get USB device strings.
 *
 * NOTE: The string arguments must provide space for up to 256 bytes.
 *
 * \param dev the device handle given by hsdaoh_open()
 * \param manufact manufacturer name, may be NULL
 * \param product product name, may be NULL
 * \param serial serial number, may be NULL
 * \return 0 on success
 */
HSDAOH_API int hsdaoh_get_usb_strings(hsdaoh_dev_t *dev, char *manufact,
				      char *product, char *serial);

/* streaming functions */

typedef void(*hsdaoh_read_cb_t)(hsdaoh_data_info_t *data_info);

/*!
 * Start streaming data from the device.
 *
 * \param dev the device handle given by hsdaoh_open()
 * \param cb callback function to return received data
 * \param ctx user specific context to pass via the callback function
 * \param buf_num optional buffer count
 *		  set to 0 for default buffer count (16)
 * \return 0 on success
 */
HSDAOH_API int hsdaoh_start_stream(hsdaoh_dev_t *dev,
				   hsdaoh_read_cb_t cb,
				   void *ctx,
				   unsigned int buf_num);

/*!
 * Stop streaming data from the device.
 *
 * \param dev the device handle given by hsdaoh_open()
 * \return 0 on success
 */
HSDAOH_API int hsdaoh_stop_stream(hsdaoh_dev_t *dev);

/*!
 * Write a datagram to the EDID RAM to control a downstream data source.
 * A header with a sequence number and length is prepended to the mesage
 * by the library.
 *
 * \param dev the device handle given by hsdaoh_open()
 * \param data pointer to the datagram
 * \param len length of the datagram, must not exceed 256-2 = 254 bytes
 *            due to the prepended header.
 * \return 0 on success
 */
HSDAOH_API int hsdaoh_write_edid_cmd_data(hsdaoh_dev_t *dev, uint8_t *data, uint8_t len);
#ifdef __cplusplus
}
#endif

#endif /* __HSDAOH_H */
