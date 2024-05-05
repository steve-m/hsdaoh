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

typedef struct hsdaoh_dev hsdaoh_dev_t;

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

/*!
 * Set the sample rate for the device
 *
 * \param dev the device handle given by hsdaoh_open()
 * \param samp_rate the sample rate to be set
 * \param ext_clock if true, use the IFCLK input insteafd of internal clock source
 *		    if a Si5351 is connected, it will be configured
 * \return 0 on success, -EINVAL on invalid rate
 */
HSDAOH_API int hsdaoh_set_sample_rate(hsdaoh_dev_t *dev, uint32_t rate, bool ext_clock);

/*!
 * Get actual sample rate the device is configured to.
 *
 * \param dev the device handle given by hsdaoh_open()
 * \return 0 on error, sample rate in Hz otherwise
 */
HSDAOH_API uint32_t hsdaoh_get_sample_rate(hsdaoh_dev_t *dev);

/* streaming functions */

typedef void(*hsdaoh_read_cb_t)(unsigned char *buf, uint32_t len, void *ctx);

/*!
 * Start streaming data from the device.
 *
 * \param dev the device handle given by hsdaoh_open()
 * \param cb callback function to return received data
 * \param ctx user specific context to pass via the callback function
 * \return 0 on success
 */
HSDAOH_API int hsdaoh_start_stream(hsdaoh_dev_t *dev,
				   hsdaoh_read_cb_t cb,
				   void *ctx);

/*!
 * Stop streaming data from the device.
 *
 * \param dev the device handle given by hsdaoh_open()
 * \return 0 on success
 */
HSDAOH_API int hsdaoh_stop_stream(hsdaoh_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __HSDAOH_H */
