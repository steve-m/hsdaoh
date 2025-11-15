#ifndef __HSDAOH_PRIVATE_H
#define __HSDAOH_PRIVATE_H

enum hsdaoh_async_status {
	HSDAOH_INACTIVE = 0,
	HSDAOH_CANCELING,
	HSDAOH_RUNNING
};

struct llist {
	uint8_t *data;
	size_t len;
	uint16_t sid;
	uint16_t format;
	uint32_t srate;
	struct llist *next;
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

	int hid_interface;

	uint8_t edid_seq_cnt;
	int frames_since_error;
	int discard_start_frames;
	unsigned int in_order_cnt;
	uint16_t last_frame_cnt;
	uint16_t last_crc[2];
	uint16_t idle_cnt;
	bool stream_synced;

	unsigned int width, height, fps;

	bool output_float;
	iqconverter_float_t *cnv_f1, *cnv_f2;

	/* status */
	int dev_lost;
	bool driver_active;
	unsigned int xfer_errors;
	char manufact[256];
	char product[256];

	/* buffering */
	pthread_t hsdaoh_output_worker_thread;

	pthread_mutex_t ll_mutex;
	pthread_cond_t cond;
	unsigned int highest_numq;
	unsigned int global_numq;
	struct llist *ll_buffers;
	unsigned int llbuf_num;
};

enum
{
	RAW_8BIT,
	RAW_16BIT,
	RAW_24BIT,
	RAW_32BIT,
	RAW_64BIT,
	PIO_1BIT,
	PIO_2BIT,
	PIO_3BIT,
	PIO_4BIT,
	PIO_5BIT,
	PIO_6BIT,
	PIO_7BIT,
	PIO_8BIT,
	PIO_8BIT_DUAL,
	PIO_8BIT_IQ,
	PIO_9BIT,
	PIO_10BIT,
	PIO_10BIT_DUAL,
	PIO_10BIT_IQ,
	PIO_11BIT,
	PIO_12BIT,
	PIO_12BIT_DUAL,
	PIO_12BIT_IQ,
	PIO_13BIT,
	PIO_14BIT,
	PIO_14BIT_DUAL,
	PIO_14BIT_IQ,
	PIO_15BIT,
	PIO_16BIT,
	PIO_16BIT_DUAL,
	PIO_16BIT_IQ,
	PIO_17BIT,
	PIO_18BIT,
	PIO_19BIT,
	PIO_20BIT,
	PIO_24BIT,
	PIO_24BIT_IQ,
	PIO_28BIT,
	PIO_32BIT,
	PIO_32BIT_IQ,
	PIO_PCM1802_AUDIO,
	PIO_AUDIO_PLACEHOLDER1,
	PIO_AUDIO_PLACEHOLDER2,
	PIO_AUDIO_PLACEHOLDER3,
	PIO_DUALCHAN_1BIT,
	PIO_DUALCHAN_1BIT_IQ,
	PIO_DUALCHAN_2BIT,
	PIO_DUALCHAN_2BIT_IQ,
	PIO_DUALCHAN_4BIT,
	PIO_DUALCHAN_4BIT_IQ,
	PIO_DUALCHAN_8BIT,
	PIO_DUALCHAN_8BIT_IQ,
	PIO_DUALCHAN_10BIT,
	PIO_DUALCHAN_10BIT_IQ,
	PIO_DUALCHAN_12BIT,
	PIO_DUALCHAN_12BIT_IQ,
	PIO_DUALCHAN_14BIT,
	PIO_DUALCHAN_14BIT_IQ,
	PIO_DUALCHAN_16BIT,
	PIO_DUALCHAN_16BIT_IQ,
	PIO_DUALCHAN_24BIT,
	PIO_DUALCHAN_24BIT_IQ,
	PIO_DUALCHAN_32BIT,
	PIO_DUALCHAN_32BIT_IQ,
	// Placeholder for internal ADC data from pico
	FPGA_1BIT	= 256,
	FPGA_2BIT,
	FPGA_3BIT,
	FPGA_4BIT,
	FPGA_5BIT,
	FPGA_6BIT,
	FPGA_7BIT,
	FPGA_8BIT,
	FPGA_8BIT_DUAL,
	FPGA_8BIT_DDR,
	FPGA_8BIT_IQ,
	FPGA_9BIT,
	FPGA_10BIT,
	FPGA_10BIT_DUAL,
	FPGA_10BIT_DDR,
	FPGA_10BIT_IQ,
	FPGA_11BIT,
	FPGA_12BIT,
	FPGA_12BIT_DUAL,
	FPGA_12BIT_DDR,
	FPGA_12BIT_IQ,
};

#endif
