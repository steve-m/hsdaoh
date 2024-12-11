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
#endif

#ifdef _MSC_VER
#include <windows.h>
#define usleep(t) Sleep((t)/1000)
#else
#include <unistd.h>
#endif

#include <inttypes.h>
#include <endian.h>
#include <libusb.h>
#include <libuvc/libuvc.h>
//#include <math.h>
#include <hsdaoh_i2c.h>
#include <hsdaoh.h>
#include <crc.h>

/*
 * All libusb callback functions should be marked with the LIBUSB_CALL macro
 * to ensure that they are compiled with the same calling convention as libusb.
 *
 * If the macro isn't available in older libusb versions, we simply define it.
 */
#ifndef LIBUSB_CALL
#define LIBUSB_CALL
#endif

enum hsdaoh_async_status {
	HSDAOH_INACTIVE = 0,
	HSDAOH_CANCELING,
	HSDAOH_RUNNING
};

struct hsdaoh_dev {
	libusb_context *ctx;
	struct libusb_device_handle *devh;
	hsdaoh_read_cb_t cb;
	void *cb_ctx;
	enum hsdaoh_async_status async_status;
	int async_cancel;
	uint16_t vid;
	uint16_t pid;

	/* UVC related */
	uvc_context_t *uvc_ctx;
	uvc_device_t *uvc_dev;
	uvc_device_handle_t *uvc_devh;

	uint32_t rate; /* Hz */

	int hid_interface;

	uint8_t edid_seq_cnt;
	int frames_since_error;
	int discard_start_frames;
	uint16_t last_frame_cnt;
	uint16_t last_crc;
	uint16_t idle_cnt;
	bool stream_synced;

	unsigned int width, height, fps;

	/* status */
	int dev_lost;
	int driver_active;
	unsigned int xfer_errors;
	char manufact[256];
	char product[256];
};

typedef struct hsdaoh_adapter {
	uint16_t vid;
	uint16_t pid;
	const char *name;
} hsdaoh_adapter_t;

static hsdaoh_adapter_t known_devices[] = {
	{ 0x345f, 0x2130, "MS2130" },
	{ 0x534d, 0x2130, "MS2130 OEM?" },
	{ 0x345f, 0x2131, "MS2131" },
};

enum crc_config {
	CRC_NONE,
	CRC16_PREV_LINE,
};

typedef struct
{
	uint32_t magic;
	uint16_t framecounter;
	uint8_t  pack_state;
	uint8_t  crc_config;
	uint8_t  data_width;
	uint8_t  data_signedness;
} __attribute__((packed, aligned(1))) metadata_t;

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

int hsdaoh_i2c_write_fn(void *dev, uint8_t addr, uint8_t *buf, uint16_t len)
{

	return -1;
}

int hsdaoh_i2c_read_fn(void *dev, uint8_t addr, uint8_t *buf, uint16_t len)
{

	return -1;
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

int hsdaoh_set_sample_rate(hsdaoh_dev_t *dev, uint32_t samp_rate, bool ext_clock)
{
	return 0;
}

uint32_t hsdaoh_get_sample_rate(hsdaoh_dev_t *dev)
{
	if (!dev)
		return 0;

	return dev->rate;
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
		r = libusb_detach_kernel_driver(dev->devh, 1);
		if (r < 0) {
			fprintf(stderr, "Failed to detach UVC Kernel driver: %d\n", r);
			return r;
		}
	}

	r = libusb_claim_interface(dev->devh, dev->hid_interface);
	if (r < 0) {
		fprintf(stderr, "usb_claim_interface hid error %d\n", r);
		return r;
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

	/* Initialize UVC context. In theory it should be possible to re-use our
	 * libusb context, but for whatever reason that does not work */
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

		if (r < 0) {
			uvc_perror(r, "uvc_open"); /* unable to open device */
		}
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

#if LIBUSB_API_VERSION >= 0x01000106
	libusb_set_option(dev->ctx, LIBUSB_OPTION_LOG_LEVEL, 3);
#else
	libusb_set_debug(dev->ctx, 3);
#endif

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

	if (!device) {
		r = -1;
		goto err;
	}

	dev->vid = dd.idVendor;
	dev->pid = dd.idProduct;

	r = libusb_open(device, &dev->devh);
	libusb_free_device_list(list, 1);
	if (r < 0) {
		fprintf(stderr, "usb_open error %d\n", r);
		if(r == LIBUSB_ERROR_ACCESS)
			fprintf(stderr, "Please fix the device permissions, e.g. "
			"by installing the udev rules file\n");
		goto err;
	}

	dev->hid_interface = 4;
	if (libusb_kernel_driver_active(dev->devh, dev->hid_interface) == 1) {

		r = libusb_detach_kernel_driver(dev->devh, dev->hid_interface);
		if (r < 0) {
			fprintf(stderr, "Failed to detach HID Kernel driver: %d\n", r);
			goto err;
		}
	}

	if (hsdaoh_clear_endpoint_halt(dev) < 0)
		goto err;

	_hsdaoh_open_uvc_device(dev);


	//dev->rate = DEFAULT_SAMPLERATE;
	dev->dev_lost = 0;

found:
	*out_dev = dev;

	return 0;
err:
	if (dev) {
		if (dev->ctx)
			libusb_exit(dev->ctx);

		free(dev);
	}

	return r;
}

int hsdaoh_close(hsdaoh_dev_t *dev)
{
	if (!dev)
		return -1;

	if (HSDAOH_INACTIVE != dev->async_status)
		uvc_stop_streaming(dev->uvc_devh);


	uvc_close(dev->uvc_devh);
	uvc_unref_device(dev->uvc_dev);
	uvc_exit(dev->uvc_ctx);

	libusb_release_interface(dev->devh, dev->hid_interface);

	//TODO: only re-attach kernel driver if it was attached previously
	if (!libusb_attach_kernel_driver(dev->devh, 4))
			fprintf(stderr, "Reattached kernel driver\n");
	else
			fprintf(stderr, "Reattaching kernel driver failed!\n");

	libusb_close(dev->devh);
	libusb_exit(dev->ctx);
	free(dev);

	return 0;
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
	int j = 0;
	uint8_t *meta = (uint8_t *)metadata;

	for (unsigned i = 0; i < sizeof(metadata_t)*2; i += 2)
		meta[j++] = (data[((i+1)*width*2) - 1] >> 4) | (data[((i+2)*width*2) - 1] & 0xf0);
}

void hsdaoh_process_frame(hsdaoh_dev_t *dev, uint8_t *data, int size)
{
	uint32_t frame_payload_bytes = 0;

	metadata_t meta;
	hsdaoh_extract_metadata(data, &meta, dev->width);

	if (le32toh(meta.magic) != 0xda7acab1)
		return;

	/* drop duplicated frames */
	if (meta.framecounter == dev->last_frame_cnt)
		return;

	if (dev->stream_synced) {
		if (meta.framecounter != ((dev->last_frame_cnt + 1) & 0xffff))
			printf("Missed at least one frame, fcnt %d, expected %d!\n",
			       meta.framecounter, ((dev->last_frame_cnt + 1) & 0xffff));
	}

	dev->last_frame_cnt = meta.framecounter;

	int frame_errors = 0;

	for (unsigned int i = 0; i < dev->height; i++) {
		uint8_t *line_dat = data + (dev->width * 2 * i);

		/* extract number of payload words from reserved field at end of line */
		uint16_t payload_len = le16toh(((uint16_t *)line_dat)[dev->width - 1]);
		uint16_t crc = le16toh(((uint16_t *)line_dat)[dev->width - 2]);

		/* we only use 12 bits, the upper 4 bits are reserved for the metadata */
		payload_len &= 0x0fff;

		if (payload_len > dev->width-1) {
			if (dev->stream_synced)
				fprintf(stderr, "Invalid payload length: %d\n", payload_len);

			/* discard frame */
			return;
		}

		if (meta.crc_config == CRC_NONE) {
			uint16_t idle_len = (dev->width-1) - payload_len;
			frame_errors += hsdaoh_check_idle_cnt(dev, (uint16_t *)line_dat + payload_len, idle_len);
		} else if (meta.crc_config == CRC16_PREV_LINE) {
			if ((crc != dev->last_crc) && dev->stream_synced)
				frame_errors++;

			dev->last_crc = crc16_ccitt(line_dat, dev->width * 2);
		}

		if (payload_len > 0)
			memmove(data + frame_payload_bytes, line_dat, payload_len * sizeof(uint16_t));

		frame_payload_bytes += payload_len*2;
	}

	if (dev->cb)
		dev->cb(data, frame_payload_bytes, dev->cb_ctx);

	if (frame_errors && dev->stream_synced) {
		fprintf(stderr,"%d frame errors, %d frames since last error\n", frame_errors, dev->frames_since_error);
		dev->frames_since_error = 0;
	} else
		dev->frames_since_error++;

	if (!dev->stream_synced) {
		fprintf(stderr, "Syncronized to HDMI input stream\n");
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

int hsdaoh_start_stream(hsdaoh_dev_t *dev, hsdaoh_read_cb_t cb, void *ctx)
{
	int r = 0;

	if (!dev)
		return -1;

	if (HSDAOH_INACTIVE != dev->async_status)
		return -2;

	dev->async_status = HSDAOH_RUNNING;
	dev->async_cancel = 0;

	dev->cb = cb;
	dev->cb_ctx = ctx;

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
		dev->discard_start_frames = 60;
		res = uvc_start_streaming(dev->uvc_devh, &ctrl, _uvc_callback, (void *)dev, 0);

		if (res < 0) {
			uvc_perror(res, "start_streaming"); /* unable to start stream */
		} else {
			puts("Streaming...");
		}
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

		/* End the stream. Blocks until last callback is serviced */
		uvc_stop_streaming(dev->uvc_devh);
		puts("Done streaming.");

		return 0;
	}

	/* if called while in pending state, change the state forcefully */
#if 0
	if (HSDAOH_INACTIVE != dev->async_status) {
		dev->async_status = HSDAOH_INACTIVE;
		return 0;
	}
#endif
	return -2;
}
