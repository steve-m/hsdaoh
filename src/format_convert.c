/*
 * hsdaoh - High Speed Data Acquisition over MS213x USB3 HDMI capture sticks
 * Format conversion routines
 *
 * Copyright (C) 2024-2025 by Steve Markgraf <steve@steve-m.de>
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

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <libusb.h>
#include <libuvc/libuvc.h>
#include <iqconverter_float.h>
#include <pthread.h>
#include <hsdaoh.h>
#include <hsdaoh_private.h>
#include <format_convert.h>

static inline void hsdaoh_16bit_to_float(hsdaoh_dev_t *dev, hsdaoh_data_info_t *data_info, uint16_t *buf, size_t length, float scale, bool duplicate)
{
	unsigned int i, j = 0;

	float *floats = malloc(sizeof(float) * dev->width * dev->height * 2 * 2);

	if (!floats)
		return;

	for (unsigned int i = 0; i < length; i++) {
		float sample_i = buf[i];
		floats[j++] = (sample_i - scale) * (1/scale);

		if (duplicate)
			floats[j++] = (sample_i - scale) * (1/scale);
	}

//	iqconverter_float_process(dev->cnv_f, (float *) floats, j);

	data_info->buf = (uint8_t *)floats;
	data_info->len = j * sizeof(float);
	dev->cb(data_info);

	free(floats);
}

// We receive three 16-bit words containing four 12-bit samples (sample A - D)
// First word:  A03 A02 A01 A00 B11 B10 B09 B08 B07 B06 B05 B04 B03 B02 B01 B00
// Second word: A07 A06 A05 A04 C11 C10 C09 C08 C07 C06 C05 C04 C03 C02 C01 C00
// Third word:  A11 A10 A09 A08 D11 D10 D09 D08 D07 D06 D05 D04 D03 D02 D01 D00
void hsdaoh_unpack_pio_12bit(hsdaoh_dev_t *dev, hsdaoh_data_info_t *data_info)
{
	uint16_t *in = (uint16_t *)data_info->buf;
	size_t inlen = data_info->len / sizeof(uint16_t);
	uint16_t *out = malloc(sizeof(uint16_t) * dev->width * dev->height  * 2);
	unsigned int j = 0;

	for (unsigned int i = 0; i < inlen; i += 3) {
		out[j++] = (in[i+2] & 0xf000) >> 4 | (in[i+1] & 0xf000) >> 8 | (in[i] >> 12);
		out[j++] = in[i  ] & 0x0fff;
		out[j++] = in[i+1] & 0x0fff;
		out[j++] = in[i+2] & 0x0fff;
	}

	if (dev->output_float) {
		hsdaoh_16bit_to_float(dev, data_info, out, j, 2047.5, true);
	} else {
		data_info->buf = (uint8_t *)out;
		data_info->len = j * sizeof(uint16_t);
		dev->cb(data_info);
	}

	free(out);
}

// We receive three 32-bit words containing four 24-bit samples (sample A - D)
// First word:  A07 A06 A05 A04 A03 A02 A01 A00 B23 B22 B21 B20 B19 B18 B17 B16 B15 B14 B13 B12 B11 B10 B09 B08 B07 B06 B05 B04 B03 B02 B01 B00
// Second word: A15 A14 A13 A12 A11 A10 A09 A08 C23 C22 C21 C20 C19 C18 C17 C16 C15 C14 C13 C12 C11 C10 C09 C08 C07 C06 C05 C04 C03 C02 C01 C00
// Third word:  A23 A22 A21 A20 A19 A18 A17 A16 D23 D22 D21 D20 D19 D18 D17 D16 D15 D14 D13 D12 D11 D10 D09 D08 D07 D06 D05 D04 D03 D02 D01 D00
void hsdaoh_unpack_pio_12bit_dual(hsdaoh_dev_t *dev, hsdaoh_data_info_t *data_info)
{
	uint32_t *in = (uint32_t *)data_info->buf;
	size_t inlen = data_info->len / sizeof(uint32_t);
	uint32_t *out = malloc(sizeof(uint32_t) * dev->width * dev->height  * 2);
	uint16_t *out16_1 = malloc(sizeof(uint16_t) * dev->width * dev->height  * 2);
	uint16_t *out16_2 = malloc(sizeof(uint16_t) * dev->width * dev->height  * 2);
	unsigned int i = 0, j = 0;

	for (i = 0; i < inlen; i += 3) {
		out[j++] = (in[i+2] & 0xff000000) >> 8 | (in[i+1] & 0xff000000) >> 16 | (in[i] >> 24);
		out[j++] = in[i  ] & 0x00ffffff;
		out[j++] = in[i+1] & 0x00ffffff;
		out[j++] = in[i+2] & 0x00ffffff;
	}

	for (i = 0; i < j; i++) {
		out16_1[i] = (out[i] >> 12) & 0x0fff;
		out16_2[i] = out[i] & 0x0fff;
	}

	if (dev->output_float) {
		hsdaoh_16bit_to_float(dev, data_info, out16_1, i, 2047.5, true);
	} else {
		data_info->buf = (uint8_t *)out16_1;
		data_info->len = i * sizeof(uint16_t);

		dev->cb(data_info);
	}

	data_info->stream_id += 1;

	if (dev->output_float) {
		hsdaoh_16bit_to_float(dev, data_info, out16_2, i, 2047.5, true);
	} else {
		data_info->buf = (uint8_t *)out16_2;
		data_info->len = i * sizeof(uint16_t);

		dev->cb(data_info);
	}

	free(out);
	free(out16_1);
	free(out16_2);
}

// We receive the samples as 16 bit words containing 8 bit I, 8 bit Q
void hsdaoh_unpack_pio_8bit_iq(hsdaoh_dev_t *dev, hsdaoh_data_info_t *data_info)
{
	uint16_t *in = (uint16_t *)data_info->buf;
	size_t inlen = data_info->len / sizeof(uint16_t);

	uint16_t *iq_samps = malloc(sizeof(uint16_t) * dev->width * dev->height * 2  * 2);
	unsigned int i = 0;
	unsigned int out_samps = 0;

	for (i = 0; i < inlen; i++) {
		iq_samps[out_samps++] = in[i] & 0xff;
		iq_samps[out_samps++] = in[i] >> 8;
	}

	if (dev->output_float) {
		hsdaoh_16bit_to_float(dev, data_info, iq_samps, out_samps, 127.5, false);
	} else {
		data_info->buf = (uint8_t *)in;
		data_info->len = inlen;

		dev->cb(data_info);
	}

	free(iq_samps);
}

// We receive five 16-bit words containing four 20-bit samples (sample A - D)
// First word:  A15 A14 A13 A12 A11 A10 A09 A08 A07 A06 A05 A04 A03 A02 A01 A00
// Second word: A19 A18 A17 A16 B11 B10 B09 B08 B07 B06 B05 B04 B03 B02 B01 B00
// Third word:  B19 B18 B17 B16 B15 B14 B13 B12 C07 C06 C05 C04 C03 C02 C01 C00
// Fourth word: C19 C18 C17 C16 C15 C14 C13 C12 C11 C10 C09 C08 D03 D02 D01 D00
// Fifth word:  D19 D18 D17 D16 D15 D14 D13 D12 D11 D10 D09 D08 D07 D06 D05 D04
void hsdaoh_unpack_pio_10bit_iq(hsdaoh_dev_t *dev, hsdaoh_data_info_t *data_info)
{
	uint16_t *in = (uint16_t *)data_info->buf;
	size_t inlen = data_info->len / sizeof(uint16_t);

	uint32_t *out = malloc(sizeof(uint32_t) * dev->width * dev->height  * 2);
	uint16_t *iq_samps = malloc(sizeof(uint16_t) * dev->width * dev->height * 2  * 2);
	unsigned int i = 0;
	unsigned int j = 0;

	for (i = 0; i < inlen; i += 5) {
		out[j++] = ((in[i+1] & 0xf000) << 4) | in[i];			// Sample A
		out[j++] = ((in[i+2] & 0xff00) << 4) | (in[i+1] & 0x0fff);	// Sample B
		out[j++] = ((in[i+3] & 0xfff0) << 4) | (in[i+2] & 0x00ff);	// Sample C
		out[j++] =  (in[i+4] << 4)           | (in[i+3] & 0x000f);	// Sample D
	}

	unsigned int out_samps = 0;

	// convert 20 bit words to interleaved 10 bit IQ samples
	// the LSBs are stored at the top 4 bits of the 20 bit word, to easily allow
	// switching to an 8 bit IQ mode with another PIO program
	for (i = 0; i < j; i++) {
		iq_samps[out_samps++] = ((out[i] & 0x000000ff) << 2) | ((out[i] >> 16) & 3);
		iq_samps[out_samps++] = ((out[i] & 0x0000ff00) >> 6) | ((out[i] >> 18) & 3);
	}

	if (dev->output_float) {
		hsdaoh_16bit_to_float(dev, data_info, iq_samps, out_samps, 511.5, false);
	} else {
		data_info->buf = (uint8_t *)iq_samps;
		data_info->len = out_samps * sizeof(uint16_t);

		dev->cb(data_info);
	}

	free(out);
	free(iq_samps);
}

void hsdaoh_unpack_pio_pcm1802_audio(hsdaoh_dev_t *dev, hsdaoh_data_info_t *data_info)
{
	uint32_t *in = (uint32_t *)data_info->buf;
	size_t inlen = data_info->len / sizeof(uint32_t);

	/* convert from S24LE to S32LE */
	for (unsigned int i = 0; i < inlen; i++)
		in[i] <<= 8;

	data_info->buf = (uint8_t *)in;
	data_info->len = inlen * sizeof(uint32_t);

	dev->cb(data_info);
}

void hsdaoh_unpack_fpga_12bit_dual(hsdaoh_dev_t *dev, hsdaoh_data_info_t *data_info)
{
	uint16_t *in = (uint16_t *)data_info->buf;
	size_t inlen = data_info->len / sizeof(uint16_t);
	unsigned int j = 0;
	uint16_t *out16_1 = malloc(sizeof(uint16_t) * inlen * 2);
	uint16_t *out16_2 = malloc(sizeof(uint16_t) * inlen * 2);

	if (!out16_1 || !out16_2)
		return;

	/* extract packed 2x12 bit samples */
	for (unsigned int i = 0; i < inlen; i += 3) {
		uint16_t lsbs = in[i+2];
		out16_1[j]   = ((in[i+0] & 0xff00) >> 4) | ((lsbs >>  0) & 0xf);
		out16_2[j++] = ((in[i+0] & 0x00ff) << 4) | ((lsbs >>  4) & 0xf);
		out16_1[j]   = ((in[i+1] & 0xff00) >> 4) | ((lsbs >>  8) & 0xf);
		out16_2[j++] = ((in[i+1] & 0x00ff) << 4) | ((lsbs >> 12) & 0xf);
	}

	if (dev->output_float) {
		hsdaoh_16bit_to_float(dev, data_info, out16_1, j, 2047.5, true);
	} else {
		data_info->buf = (uint8_t *)out16_1;
		data_info->len = j * sizeof(uint16_t);

		dev->cb(data_info);
	}

	data_info->stream_id += 1;

	if (dev->output_float) {
		hsdaoh_16bit_to_float(dev, data_info, out16_2, j, 2047.5, true);
	} else {
		data_info->buf = (uint8_t *)out16_2;
		data_info->len = j * sizeof(uint16_t);

		dev->cb(data_info);
	}

	free(out16_1);
	free(out16_2);
}
