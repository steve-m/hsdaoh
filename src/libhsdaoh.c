/*
 * hsdaoh - High Speed Data Acquisition over MS213x USB3 HDMI capture sticks
 *
 * Copyright (C) 2024 by Steve Markgraf <steve@steve-m.de>
 *
 * portions based on librtlsdr:
 * Copyright (C) 2012-2014 by Steve Markgraf <steve@steve-m.de>
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

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#ifndef _WIN32
#include <unistd.h>
#include <endian.h>
#else
#include <windows.h>
#define usleep(t) Sleep((t)/1000)
#define le32toh(x) (x)
#define le16toh(x) (x)
#endif

#include <inttypes.h>
#include <pthread.h>
#include <libusb.h>
#include <libuvc/libuvc.h>

#include <iqconverter_float.h>
#include <filters.h>

#include <hsdaoh.h>
#include <hsdaoh_private.h>
#include <format_convert.h>
#include <crc.h>

#define DEFAULT_BUFFERS 96

typedef struct hsdaoh_adapter {
	uint16_t vid;
	uint16_t pid;
	const char *name;
} hsdaoh_adapter_t;

static hsdaoh_adapter_t known_devices[] = {
	{ 0x345f, 0x2130, "MS2130" },
	{ 0x534d, 0x2130, "MS2130 OEM" },
	{ 0x345f, 0x2131, "MS2131" },
};

enum crc_config {
	CRC_NONE,		/* No CRC, just 16 bit idle counter */
	CRC16_1_LINE,		/* Line contains CRC of the last line */
	CRC16_2_LINE		/* Line contains CRC of the line before the last line */
};

#define DEFAULT_MAX_STREAMS		8

typedef struct
{
	uint64_t data_cnt;
	uint32_t srate;
	uint32_t reserved1;
	char reserved2[16];
} __attribute__((packed, aligned(1))) stream_info_t;

typedef struct
{
	uint32_t magic;
	uint16_t framecounter;
	uint8_t  reserved1;
	uint8_t  crc_config;
	uint16_t version;
	uint32_t flags;
	uint32_t reserved2[8];
	uint16_t stream0_format;
	uint16_t max_streamid;
	stream_info_t stream_info[DEFAULT_MAX_STREAMS];
} __attribute__((packed, aligned(1))) metadata_t;

#define FLAG_STREAM_ID_PRESENT	(1 << 0)
#define FLAG_FORMAT_ID_PRESENT	(1 << 1)

#define CTRL_TIMEOUT	300

int hsdaoh_get_hid_feature_report(hsdaoh_dev_t *dev, unsigned char *data, size_t length)
{
	int report_number = data[0];

	return libusb_control_transfer(dev->devh,
		LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_IN,
		0x01, (3 << 8) | report_number,
		dev->hid_interface,
		(unsigned char *)data, length,
		CTRL_TIMEOUT);
}

int hsdaoh_send_hid_feature_report(hsdaoh_dev_t *dev, const unsigned char *data, size_t length)
{
	int report_number = data[0];

	return libusb_control_transfer(dev->devh,
		LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT,
		0x09, (3 << 8) | report_number,
		dev->hid_interface,
		(unsigned char *)data, length,
		CTRL_TIMEOUT);
}

int hsdaoh_ms_write_register(hsdaoh_dev_t *dev, uint16_t addr, uint8_t val)
{
	uint8_t cmd[8] = { 0xb6, (addr >> 8), (addr & 0xff), val,
			   0x00, 0x00, 0x00, 0x00 };

	return hsdaoh_send_hid_feature_report(dev, cmd, sizeof(cmd));
}

int hsdaoh_read_register(hsdaoh_dev_t *dev, uint16_t addr, uint8_t *val)
{
	int r = 0;
	uint8_t cmd[8] = { 0xb5, (addr >> 8), (addr & 0xff), 0x00,
			   0x00, 0x00, 0x00, 0x00 };
	uint8_t resp[8];

	r = hsdaoh_send_hid_feature_report(dev, cmd, sizeof(cmd));
	if (r < 0)
		return r;

	r = hsdaoh_get_hid_feature_report(dev, resp, sizeof(resp));

	if (val)
		*val = resp[3];

	return r;
}

/* Write a datagram to the EDID RAM to control a downstream data source */
int hsdaoh_write_edid_cmd_data(hsdaoh_dev_t *dev, uint8_t *data, uint8_t len)
{
	if (!dev || !data || (len > 254))
		return -1;

	/* increment sequence counter and avoid values 0x00 and 0xff */
	dev->edid_seq_cnt++;
	if (!dev->edid_seq_cnt || (dev->edid_seq_cnt == 0xff))
		dev->edid_seq_cnt = 1;

	/* disable I2C access to EDID RAM, reading via I2C will result in a NAK */
	hsdaoh_ms_write_register(dev, 0xf063, 0x00);

	/* switch EDID RAM to 8051 */
	hsdaoh_ms_write_register(dev, 0xf062, 0x80);

	/* store header with sequence counter and length of data */
	hsdaoh_ms_write_register(dev, 0xf900, dev->edid_seq_cnt);
	hsdaoh_ms_write_register(dev, 0xf901, len);

	/* store actual data to the EDID RAM */
	for (uint8_t i = 0; i < len; i++)
		hsdaoh_ms_write_register(dev, 0xf902 + i, data[i]);

	/* switch EDID RAM to DDC I2C slave */
	hsdaoh_ms_write_register(dev, 0xf062, 0x00);

	/* re-enable I2C access to EDID RAM */
	hsdaoh_ms_write_register(dev, 0xf063, 0x08);

	return 0;
}

/* Switch the MS2130 to a transparent mode, YUYV data received via HDMI
 * will be passed through unmodified */
void hsdaoh_ms_enable_transparent_mode(hsdaoh_dev_t *dev)
{
	/* Note: those registers and settings have been
	 * found by try and error and observing changes to the output:
	 * no warranty! */

	/* force YCbCr 4:2:2/YUV input, default is 0x04 (RGB) */
	hsdaoh_ms_write_register(dev, 0xf039, 0x00);
	hsdaoh_ms_write_register(dev, 0xf030, 0x02);

	/* disable sharpening */
	hsdaoh_ms_write_register(dev, 0xf6b0, 0x00);

	/* disable luma processing -> UVC brightness/contrast control has no effect anymore */
	hsdaoh_ms_write_register(dev, 0xf6be, 0x11);

	/* disable chroma processing -> UVC hue/saturation control has no effect anymore */
	hsdaoh_ms_write_register(dev, 0xf6bf, 0x11);

	/* disable luma horizontal scaling/subpixel interpolation */
	hsdaoh_ms_write_register(dev, 0xf65c, 0x10);

	/* disable luma vertical scaling/subpixel interpolation */
	hsdaoh_ms_write_register(dev, 0xf65e, 0x10);

	/* disable chroma interpolation */
	hsdaoh_ms_write_register(dev, 0xf600, 0x80);
}

int hsdaoh_get_usb_strings(hsdaoh_dev_t *dev, char *manufact, char *product,
			    char *serial)
{
	struct libusb_device_descriptor dd;
	libusb_device *device = NULL;
	const int buf_max = 256;
	int r = 0;

	if (!dev || !dev->devh)
		return -1;

	device = libusb_get_device(dev->devh);

	r = libusb_get_device_descriptor(device, &dd);
	if (r < 0)
		return -1;

	if (manufact) {
		memset(manufact, 0, buf_max);
		libusb_get_string_descriptor_ascii(dev->devh, dd.iManufacturer,
						   (unsigned char *)manufact,
						   buf_max);
	}

	if (product) {
		memset(product, 0, buf_max);
		libusb_get_string_descriptor_ascii(dev->devh, dd.iProduct,
						   (unsigned char *)product,
						   buf_max);
	}

	if (serial) {
		memset(serial, 0, buf_max);
		libusb_get_string_descriptor_ascii(dev->devh, dd.iSerialNumber,
						   (unsigned char *)serial,
						   buf_max);
	}

	return 0;
}

static hsdaoh_adapter_t *find_known_device(uint16_t vid, uint16_t pid)
{
	unsigned int i;
	hsdaoh_adapter_t *device = NULL;

	for (i = 0; i < sizeof(known_devices)/sizeof(hsdaoh_adapter_t); i++ ) {
		if (known_devices[i].vid == vid && known_devices[i].pid == pid) {
			device = &known_devices[i];
			break;
		}
	}

	return device;
}

uint32_t hsdaoh_get_device_count(void)
{
	int i,r;
	libusb_context *ctx;
	libusb_device **list;
	uint32_t device_count = 0;
	struct libusb_device_descriptor dd;
	ssize_t cnt;

	r = libusb_init(&ctx);
	if (r < 0)
		return 0;

	cnt = libusb_get_device_list(ctx, &list);

	for (i = 0; i < cnt; i++) {
		libusb_get_device_descriptor(list[i], &dd);

		if (find_known_device(dd.idVendor, dd.idProduct))
			device_count++;
	}

	libusb_free_device_list(list, 1);

	libusb_exit(ctx);

	return device_count;
}

const char *hsdaoh_get_device_name(uint32_t index)
{
	int i,r;
	libusb_context *ctx;
	libusb_device **list;
	struct libusb_device_descriptor dd;
	hsdaoh_adapter_t *device = NULL;
	uint32_t device_count = 0;
	ssize_t cnt;

	r = libusb_init(&ctx);
	if (r < 0)
		return "";

	cnt = libusb_get_device_list(ctx, &list);

	for (i = 0; i < cnt; i++) {
		libusb_get_device_descriptor(list[i], &dd);

		device = find_known_device(dd.idVendor, dd.idProduct);

		if (device) {
			device_count++;

			if (index == device_count - 1)
				break;
		}
	}

	libusb_free_device_list(list, 1);

	libusb_exit(ctx);

	if (device)
		return device->name;
	else
		return "";
}

/* This function is a workaround for the fact that libuvc does not clear
 * the halted UVC endpoint. After we properly close the UVC device,
 * the endpoint becomes stalled, so after a restart of our library,
 * no data is received.
 * Thus, we need to manually claim the interface and clear the halted EP
 * before we open the device with libuvc...
 */
int hsdaoh_clear_endpoint_halt(hsdaoh_dev_t *dev)
{
	int r;

	if (libusb_kernel_driver_active(dev->devh, 1) == 1) {
		dev->driver_active = true;
		r = libusb_detach_kernel_driver(dev->devh, 1);
		if (r < 0) {
			fprintf(stderr, "Failed to detach UVC Kernel driver: %d\n", r);
			return r;
		}
	}

	r = libusb_claim_interface(dev->devh, 1);
	if (r < 0) {
		fprintf(stderr, "usb_claim_interface 1 error %d\n", r);
		return r;
	}

	r = libusb_clear_halt(dev->devh, LIBUSB_ENDPOINT_IN + 3);
	if (r < 0) {
		fprintf(stderr, "error clearing endpoint halt %d\n", r);
		return r;
	}

	return libusb_release_interface(dev->devh, 1);
}

int _hsdaoh_open_uvc_device(hsdaoh_dev_t *dev)
{
	uvc_error_t r;

	/* Initialize UVC context */
	r = uvc_init(&dev->uvc_ctx, NULL);

	if (r < 0) {
		uvc_perror(r, "uvc_init");
		return r;
	}

	/* Locates the first attached UVC device, stores in uvc_dev */
	r = uvc_find_device(dev->uvc_ctx, &dev->uvc_dev, dev->vid, dev->pid, NULL);

	if (r < 0) {
		uvc_perror(r, "uvc_find_device"); /* no devices found */
	} else {

		/* Try to open the device: requires exclusive access */
		r = uvc_open(dev->uvc_dev, &dev->uvc_devh);

		if (r < 0)
			uvc_perror(r, "uvc_open"); /* unable to open device */
	}

	return (int)r;
}

int hsdaoh_open(hsdaoh_dev_t **out_dev, uint32_t index)
{
	int r;
	int i;
	libusb_device **list;
	hsdaoh_dev_t *dev = NULL;
	libusb_device *device = NULL;
	uint32_t device_count = 0;
	struct libusb_device_descriptor dd;
	uint8_t reg;
	ssize_t cnt;

	dev = malloc(sizeof(hsdaoh_dev_t));
	if (NULL == dev)
		return -ENOMEM;

	memset(dev, 0, sizeof(hsdaoh_dev_t));

	r = libusb_init(&dev->ctx);
	if(r < 0){
		free(dev);
		return -1;
	}

	dev->dev_lost = 1;

	cnt = libusb_get_device_list(dev->ctx, &list);

	for (i = 0; i < cnt; i++) {
		device = list[i];

		libusb_get_device_descriptor(list[i], &dd);

		if (find_known_device(dd.idVendor, dd.idProduct)) {
			device_count++;
		}

		if (index == device_count - 1)
			break;

		device = NULL;
	}

	dev->vid = dd.idVendor;
	dev->pid = dd.idProduct;

	libusb_free_device_list(list, 1);
	libusb_exit(dev->ctx);

	if (!device) {
		r = -1;
		goto err;
	}

	r = _hsdaoh_open_uvc_device(dev);
	if (r < 0) {
		if (r == LIBUSB_ERROR_ACCESS)
			fprintf(stderr, "Please fix the device permissions, e.g. "
			"by installing the udev rules file\n");
		goto err;
	}

	dev->devh = uvc_get_libusb_handle(dev->uvc_devh);
	if (!dev->devh) {
		fprintf(stderr, "Failed to get libusb device handle\n");
		goto err;
	}

	dev->hid_interface = 4;
	if (libusb_kernel_driver_active(dev->devh, dev->hid_interface) == 1) {
		dev->driver_active = true;
		r = libusb_detach_kernel_driver(dev->devh, dev->hid_interface);
		if (r < 0) {
			fprintf(stderr, "Failed to detach HID Kernel driver: %d\n", r);
			goto err;
		}
	}

	r = libusb_claim_interface(dev->devh, dev->hid_interface);
	if (r < 0) {
		fprintf(stderr, "usb_claim_interface hid error %d\n", r);
		return r;
	}

	if (hsdaoh_clear_endpoint_halt(dev) < 0)
		goto err;

	dev->dev_lost = 0;
	dev->cnv_f = iqconverter_float_create(HB_KERNEL_FLOAT, HB_KERNEL_FLOAT_LEN);

found:
	*out_dev = dev;

	return 0;
err:
	if (dev)
		free(dev);

	return r;
}

int hsdaoh_close(hsdaoh_dev_t *dev)
{
	if (!dev)
		return -1;

	if (HSDAOH_INACTIVE != dev->async_status)
		uvc_stop_streaming(dev->uvc_devh);

	libusb_release_interface(dev->devh, dev->hid_interface);

	if (dev->driver_active) {
		if (libusb_attach_kernel_driver(dev->devh, 1) && libusb_attach_kernel_driver(dev->devh, 4))
			fprintf(stderr, "Reattaching kernel driver failed!\n");
	}

	uvc_close(dev->uvc_devh);
	uvc_unref_device(dev->uvc_dev);
	uvc_exit(dev->uvc_ctx);

	iqconverter_float_free(dev->cnv_f);
	free(dev);

	return 0;
}

int hsdaoh_set_output_format(hsdaoh_dev_t *dev, hsdaoh_output_format_t format)
{
	if (!dev)
		return -1;


	return 0;
}

void hsdaoh_output(hsdaoh_dev_t *dev, uint16_t sid, int format, uint32_t srate, uint8_t *data, size_t len)
{
	hsdaoh_data_info_t data_info;
	data_info.ctx = dev->cb_ctx;
	data_info.stream_id = sid;
	data_info.buf = data;
	data_info.len = len;
	data_info.srate = srate;

	switch (format) {
		case PIO_8BIT_IQ:
			hsdaoh_unpack_pio_8bit_iq(dev, &data_info);
			break;
		case PIO_10BIT_IQ:
			hsdaoh_unpack_pio_10bit_iq(dev, &data_info);
			break;
		case PIO_12BIT:
			hsdaoh_unpack_pio_12bit(dev, &data_info);
			break;
		case PIO_12BIT_DUAL:
			hsdaoh_unpack_pio_12bit_dual(dev, &data_info);
			break;
		case PIO_PCM1802_AUDIO:
			hsdaoh_unpack_pio_pcm1802_audio(dev, &data_info);
			break;
		case FPGA_12BIT_DUAL:
			hsdaoh_unpack_fpga_12bit_dual(dev, &data_info);
			break;
		default:
			dev->cb(&data_info);
			break;
	}
}

static void *hsdaoh_output_worker(void *arg)
{
	struct llist *curelem, *prev;
	struct timespec ts;
	struct timeval tp;
	fd_set writefds;
	int r = 0;
	hsdaoh_dev_t *dev = (hsdaoh_dev_t *)arg;

	while(1) {
		if (dev->async_status != HSDAOH_RUNNING)
			pthread_exit(NULL);

		pthread_mutex_lock(&dev->ll_mutex);
		gettimeofday(&tp, NULL);
		ts.tv_sec  = tp.tv_sec+1;
		ts.tv_nsec = tp.tv_usec * 1000;
		r = pthread_cond_timedwait(&dev->cond, &dev->ll_mutex, &ts);

		if (r == ETIMEDOUT) {
			pthread_mutex_unlock(&dev->ll_mutex);
			continue;
		}

		curelem = dev->ll_buffers;
		dev->ll_buffers = NULL;
		pthread_mutex_unlock(&dev->ll_mutex);

		while (curelem != NULL) {
			hsdaoh_output(dev, curelem->sid, curelem->format, curelem->srate, curelem->data, curelem->len);

			prev = curelem;
			curelem = curelem->next;
			free(prev->data);
			free(prev);
		}
	}
}

void hsdaoh_enqueue_data(hsdaoh_dev_t *dev, uint16_t sid, int format, uint32_t srate, uint8_t *data, size_t len)
{
	if (dev->async_status != HSDAOH_RUNNING) {
		free(data);
		return;
	}

	struct llist *rpt = (struct llist*)malloc(sizeof(struct llist));
	rpt->data = data;
	rpt->len = len;
	rpt->sid = sid;
	rpt->format = format;
	rpt->srate = srate;
	rpt->next = NULL;

	pthread_mutex_lock(&dev->ll_mutex);

	if (dev->ll_buffers == NULL) {
		dev->ll_buffers = rpt;
	} else {
		struct llist *cur = dev->ll_buffers;
		unsigned int num_queued = 0;

		while (cur->next != NULL) {
			cur = cur->next;
			num_queued++;
		}

		if (dev->llbuf_num && dev->llbuf_num == num_queued-2) {
			struct llist *curelem;
			fprintf(stderr, "Buffer dropped due to overrun!\n");
			free(dev->ll_buffers->data);
			curelem = dev->ll_buffers->next;
			free(dev->ll_buffers);
			dev->ll_buffers = curelem;
		}

		cur->next = rpt;

		if (num_queued > dev->highest_numq) {
			fprintf(stderr, "Maximum buffer queue length: %d\n", num_queued);
			dev->highest_numq = num_queued;
		}

		dev->global_numq = num_queued;
	}
	pthread_cond_signal(&dev->cond);
	pthread_mutex_unlock(&dev->ll_mutex);
}

/* callback for idle/filler data */
inline int hsdaoh_check_idle_cnt(hsdaoh_dev_t *dev, uint16_t *buf, size_t length)
{
	int idle_counter_errors = 0;

	if (length == 0)
		return 0;

	for (unsigned int i = 0; i < length; i++) {
		if (buf[i] != ((dev->idle_cnt+1) & 0xffff))
			idle_counter_errors++;

		dev->idle_cnt = buf[i];
	}

	return idle_counter_errors;
}

/* Extract the metadata stored in the upper 4 bits of the last word of each line */
inline void hsdaoh_extract_metadata(uint8_t *data, metadata_t *metadata, unsigned int width)
{
	uint8_t *meta = (uint8_t *)metadata;

	for (unsigned i = 0; i < sizeof(metadata_t)*2; i += 2)
		meta[i/2] = (data[((i+1)*width*2) - 1] >> 4) | (data[((i+2)*width*2) - 1] & 0xf0);
}

void hsdaoh_process_frame(hsdaoh_dev_t *dev, uint8_t *data, int size)
{
	metadata_t meta;
	hsdaoh_extract_metadata(data, &meta, dev->width);

	if (le32toh(meta.magic) != 0xda7acab1) {
		if (dev->stream_synced)
			fprintf(stderr, "Lost sync to HDMI input stream\n");

		dev->stream_synced = false;
		return;
	}

	/* drop duplicated frames */
	if (meta.framecounter == dev->last_frame_cnt)
		return;

	if (meta.framecounter != ((dev->last_frame_cnt + 1) & 0xffff)) {
		dev->in_order_cnt = 0;
		if (dev->stream_synced)
			fprintf(stderr, "Missed at least one frame, fcnt %d, expected %d!\n",
				meta.framecounter, ((dev->last_frame_cnt + 1) & 0xffff));
	} else
		dev->in_order_cnt++;

	dev->last_frame_cnt = meta.framecounter;
	int frame_errors = 0;
	unsigned int stream0_payload_bytes = 0;
	uint16_t stream0_format = 0;
	uint8_t *stream0_data = malloc(dev->width-1 * dev->height  * 2);

	if (!stream0_data) {
		fprintf(stderr, "Out of memory, frame skipped!\n");
		return;
	}

	for (unsigned int i = 0; i < dev->height; i++) {
		uint8_t *line_dat = data + (dev->width * sizeof(uint16_t) * i);

		/* extract number of payload words from reserved field at end of line */
		uint16_t payload_len = le16toh(((uint16_t *)line_dat)[dev->width - 1]);
		uint16_t crc = le16toh(((uint16_t *)line_dat)[dev->width - 2]);
		uint16_t stream_id = le16toh(((uint16_t *)line_dat)[dev->width - 3]);
		uint16_t format = (meta.flags & FLAG_FORMAT_ID_PRESENT) ? stream_id >> 6 : RAW_8BIT;

		if (meta.flags & FLAG_STREAM_ID_PRESENT)
			stream_id &= 0x3f;
		else {
			stream_id = 0;
			format =  meta.stream0_format;
		}

		/* we only use 12 bits, the upper 4 bits are reserved for the metadata */
		payload_len &= 0x0fff;

		if (payload_len > dev->width-1) {
			if (dev->stream_synced)
				fprintf(stderr, "Invalid payload length: %d\n", payload_len);

			/* discard frame */
			free(stream0_data);
			return;
		}

		if (meta.crc_config == CRC_NONE) {
			uint16_t idle_len = (dev->width-1) - payload_len;
			frame_errors += hsdaoh_check_idle_cnt(dev, (uint16_t *)line_dat + payload_len, idle_len);
		} else if ((meta.crc_config == CRC16_1_LINE) || (meta.crc_config == CRC16_2_LINE)) {
			uint16_t expected_crc = (meta.crc_config == CRC16_1_LINE) ? dev->last_crc[0] : dev->last_crc[1];

			if ((crc != expected_crc) && dev->stream_synced)
				frame_errors++;

			dev->last_crc[1] = dev->last_crc[0];
			dev->last_crc[0] = crc16_ccitt(line_dat, dev->width * sizeof(uint16_t));
		}

		if ((payload_len > 0) && dev->stream_synced) {
			unsigned int out_len = payload_len * sizeof(uint16_t);

			if (!(meta.flags & FLAG_STREAM_ID_PRESENT) || stream_id == 0) {
				memcpy(stream0_data + stream0_payload_bytes, line_dat, out_len);
				stream0_payload_bytes += out_len;
				stream0_format = format;
			} else {
				uint8_t *out_data = malloc(out_len);
				uint32_t srate = stream_id < DEFAULT_MAX_STREAMS ? meta.stream_info[stream_id].srate : 0;
				memcpy(out_data, line_dat, out_len);
				hsdaoh_enqueue_data(dev, stream_id, format, srate, out_data, out_len);
			}
		}
	}

	if (dev->stream_synced && stream0_payload_bytes)
		hsdaoh_enqueue_data(dev, 0, stream0_format, meta.stream_info[0].srate, stream0_data, stream0_payload_bytes);
	else
		free(stream0_data);

	if (frame_errors && dev->stream_synced) {
		fprintf(stderr,"%d frame errors, %d frames since last error\n", frame_errors, dev->frames_since_error);
		dev->frames_since_error = 0;
	} else
		dev->frames_since_error++;

	if (!dev->stream_synced && !frame_errors && (dev->in_order_cnt > 4)) {
		fprintf(stderr, "Synchronized to HDMI input stream\n");
		dev->stream_synced = true;
	}
}

void _uvc_callback(uvc_frame_t *frame, void *ptr)
{
	hsdaoh_dev_t *dev = (hsdaoh_dev_t *)ptr;

	if (frame->frame_format != UVC_COLOR_FORMAT_YUYV) {
		fprintf(stderr, "Error: incorrect frame format!\n");
		return;
	}

	/* discard frame if it is incomplete (can happen after streaming was started) */
	if (frame->data_bytes != (dev->width * dev->height * 2))
		return;

	if (dev->discard_start_frames) {
		dev->discard_start_frames--;

		if (dev->discard_start_frames == 5)
			hsdaoh_ms_enable_transparent_mode(dev);

		return;
	}

	hsdaoh_process_frame(dev, (uint8_t *)frame->data, frame->data_bytes);
}

int hsdaoh_start_stream(hsdaoh_dev_t *dev, hsdaoh_read_cb_t cb, void *ctx, unsigned int buf_num)
{
	int r = 0;

	if (!dev)
		return -1;

	if (HSDAOH_INACTIVE != dev->async_status)
		return -2;

	iqconverter_float_reset(dev->cnv_f);

	dev->async_status = HSDAOH_RUNNING;
	dev->async_cancel = 0;

	dev->cb = cb;
	dev->cb_ctx = ctx;

//	dev->output_float = true;

	/* initialize with a threshold */
	dev->highest_numq = DEFAULT_BUFFERS/2;
	dev->llbuf_num = (buf_num == 0) ? DEFAULT_BUFFERS : buf_num;

	pthread_mutex_init(&dev->ll_mutex, NULL);
	pthread_cond_init(&dev->cond, NULL);

	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	r = pthread_create(&dev->hsdaoh_output_worker_thread, &attr, hsdaoh_output_worker, (void *)dev);
	pthread_attr_destroy(&attr);

	uvc_error_t res;
	uvc_stream_ctrl_t ctrl;

	const uvc_format_desc_t *format_desc = uvc_get_format_descs(dev->uvc_devh);
	const uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
	enum uvc_frame_format frame_format = UVC_FRAME_FORMAT_YUYV;

	dev->width = 1920;
	dev->height = 1080;
	dev->fps = 60;

	res = uvc_get_stream_ctrl_format_size(dev->uvc_devh, &ctrl, frame_format, dev->width, dev->height, dev->fps);

	if (res < 0) {
		uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
	} else {
		/* start the UVC stream */
		dev->discard_start_frames = 30;
		res = uvc_start_streaming(dev->uvc_devh, &ctrl, _uvc_callback, (void *)dev, 0);

		if (res < 0)
			uvc_perror(res, "start_streaming"); /* unable to start stream */
	}

	return r;
}

int hsdaoh_stop_stream(hsdaoh_dev_t *dev)
{
	if (!dev)
		return -1;

	/* if streaming, try to cancel gracefully */
	if (HSDAOH_RUNNING == dev->async_status) {
		dev->async_status = HSDAOH_CANCELING;
		dev->async_cancel = 1;
		pthread_cond_signal(&dev->cond);

		/* End the stream. Blocks until last callback is serviced */
		uvc_stop_streaming(dev->uvc_devh);

		void *status;
		struct llist *curelem, *prev;
		pthread_join(dev->hsdaoh_output_worker_thread, &status);

		curelem = dev->ll_buffers;
		dev->ll_buffers = NULL;

		while (curelem != 0) {
			prev = curelem;
			curelem = curelem->next;
			free(prev->data);
			free(prev);
		}

		dev->global_numq = 0;

		return 0;
	}

	return -2;
}
