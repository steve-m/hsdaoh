/*
 * hsdaoh - High Speed Data Acquisition over MS213x USB3 HDMI capture sticks
 *
 * Copyright (C) 2024 by Steve Markgraf <steve@steve-m.de>
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

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#include "getopt/getopt.h"
#define usleep(t) Sleep((t)/1000)
#endif

#include "FLAC/metadata.h"
#include "FLAC/stream_encoder.h"

#include "hsdaoh.h"

#define FD_NUMS				4

static bool do_exit = false;
static hsdaoh_dev_t *dev = NULL;
static uint32_t flac_level = 5;
static uint32_t flac_nthreads = 4;

typedef struct file_ctx {
	bool use_signed[FD_NUMS];
	FILE *files[FD_NUMS];
	bool use_flac[FD_NUMS];
	FLAC__StreamEncoder *encoder[FD_NUMS];
	FLAC__StreamMetadata *seektable[FD_NUMS];
} file_ctx_t;

void usage(void)
{
	fprintf(stderr,
		"hsdaoh_file, HDMI data acquisition tool\n\n"
		"Usage:\n"
		"\t[-d device_index (default: 0)]\n"
		"\t[-b maximum number of buffers (default: 16)]\n"
		"\t[-l FLAC compression level (default: 5)]\n"
		"\t[-t number of threads for FLAC encoding (default: 4)]\n"
		"\t[-0 to -3 filename of stream 0 to stream 3 (a '-' dumps samples to stdout)]\n"
		"\tfilename (of stream 0) (a '-' dumps samples to stdout)\n\n"
		"\tFilenames with the extension .flac will enable FLAC encoding\n\n");
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = true;
		hsdaoh_stop_stream(dev);
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	signal(SIGPIPE, SIG_IGN);
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = true;
	hsdaoh_stop_stream(dev);
}
#endif

static void hsdaoh_callback(hsdaoh_data_info_t *data_info)
{
	size_t nbytes = 0;
	uint32_t len = data_info->len;

	if (!data_info->ctx || do_exit)
		return;

	if (data_info->stream_id >= FD_NUMS)
		return;

	file_ctx_t *f = (file_ctx_t *)data_info->ctx;
	FILE *file = f->files[data_info->stream_id];

	if (!file)
		return;

	FLAC__StreamEncoder *encoder = f->encoder[data_info->stream_id];
	/* allocate FLAC encoder if required */
	if (f->use_flac[data_info->stream_id] && !encoder) {
		FLAC__bool ret = true;
		FLAC__StreamEncoder *encoder = NULL;
		FLAC__StreamEncoderInitStatus init_status;

		if ((encoder = FLAC__stream_encoder_new()) == NULL) {
			fprintf(stderr, "ERROR: failed allocating FLAC encoder\n");
			do_exit = true;
			return;
		}

		ret &= FLAC__stream_encoder_set_verify(encoder, false);
		ret &= FLAC__stream_encoder_set_compression_level(encoder, flac_level);
		ret &= FLAC__stream_encoder_set_sample_rate(encoder,
							   data_info->srate > FLAC__MAX_SAMPLE_RATE ?
							   data_info->srate/1000 : data_info->srate);

		ret &= FLAC__stream_encoder_set_channels(encoder, data_info->nchans);
		ret &= FLAC__stream_encoder_set_bits_per_sample(encoder, data_info->bits_per_samp);
		ret &= FLAC__stream_encoder_set_total_samples_estimate(encoder, 0);
		ret &= FLAC__stream_encoder_set_streamable_subset(encoder, false);
		if (FLAC__stream_encoder_set_num_threads(encoder, flac_nthreads) != FLAC__STREAM_ENCODER_SET_NUM_THREADS_OK)
			ret = false;

		if (!ret) {
			fprintf(stderr, "ERROR: failed initializing FLAC encoder\n");
			do_exit = true;
			return;
		}
#if 0
		if ((f->seektable[data_info->stream_id] = FLAC__metadata_object_new(FLAC__METADATA_TYPE_SEEKTABLE)) == NULL
			|| FLAC__metadata_object_seektable_template_append_spaced_points(f->seektable[data_info->stream_id], 1<<18, (uint64_t)1<<41) != true
			|| FLAC__stream_encoder_set_metadata(encoder, &f->seektable[data_info->stream_id], 1) != true) {
			fprintf(stderr, "ERROR: could not create FLAC seektable\n");
			do_exit = true;
			return;
		}
#endif
		init_status = FLAC__stream_encoder_init_FILE(encoder, f->files[data_info->stream_id], NULL, NULL);
		if (init_status != FLAC__STREAM_ENCODER_INIT_STATUS_OK) {
			fprintf(stderr, "ERROR: failed initializing FLAC encoder: %s\n", FLAC__StreamEncoderInitStatusString[init_status]);
			do_exit = true;
			return;
		}

		f->encoder[data_info->stream_id] = encoder;
	}

	if (f->use_flac[data_info->stream_id]) {
		/* write FLAC output */
		FLAC__bool ok = false;

		//FIXME use real metadata
		if (data_info->stream_id < 2) {

			FLAC__int32 test[len/sizeof(uint16_t)];
			uint16_t *dat = (uint16_t *)data_info->buf;
			for (int i = 0; i < len/sizeof(uint16_t); i++) {
				test[i] = dat[i] - 2048;
			}

			ok = FLAC__stream_encoder_process_interleaved(f->encoder[data_info->stream_id], test, len/sizeof(uint16_t));
		} else {
			FLAC__int32 test[len/4];
			int32_t *dat = (int32_t *)data_info->buf;
			for (int i = 0; i < len/4; i++) {
				test[i] = dat[i]/256;
			}

			ok = FLAC__stream_encoder_process_interleaved(f->encoder[data_info->stream_id], test, len/8);
		}

		if (!ok) {
			fprintf(stderr, "ERROR: () FLAC encoder could not process data: %s\n", FLAC__StreamEncoderStateString[FLAC__stream_encoder_get_state(encoder)]);

		}
	} else {
		/* write raw file output */
		while (nbytes < len) {
			nbytes += fwrite(data_info->buf + nbytes, 1, len - nbytes, file);

			if (ferror(file)) {
				fprintf(stderr, "Error writing file, samples lost, exiting!\n");
				hsdaoh_stop_stream(dev);
				break;
			}
		}
	}
}

int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	char *filenames[FD_NUMS] = { NULL, };
	int n_read;
	int r, opt;
	file_ctx_t f;
	int dev_index = 0;
	unsigned int num_bufs = 0;
	bool fname0_used = false;
	bool have_file = false;

	while ((opt = getopt(argc, argv, "0:1:2:3:d:b:l:t:")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = (uint32_t)atoi(optarg);
			break;
		case 'b':
			num_bufs = (unsigned int)atoi(optarg);
			break;
		case 'l':
			flac_level = atoi(optarg);
			break;
		case 't':
			flac_nthreads = atoi(optarg);
			break;
		case '0':
			fname0_used = true;
			have_file = true;
			filenames[0] = optarg;
			break;
		case '1':
			have_file = true;
			filenames[1] = optarg;
			break;
		case '2':
			have_file = true;
			filenames[2] = optarg;
			break;
		case '3':
			have_file = true;
			filenames[3] = optarg;
			break;
		default:
			usage();
			break;
		}
	}

	if (!fname0_used) {
		if (argc <= optind) {
			if (!have_file)
				usage();
		} else {
			filenames[0] = argv[optind];
		}
	}

	if (dev_index < 0)
		exit(1);

	r = hsdaoh_open(&dev, (uint32_t)dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open hsdaoh device #%d.\n", dev_index);
		exit(1);
	}
#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

	for (int i = 0; i < FD_NUMS; i++) {
		f.files[i] = NULL;
		f.encoder[i] = NULL;

		if (!filenames[i])
			continue;

		if (strcmp(filenames[i], "-") == 0) { /* Write samples to stdout */
			f.files[i] = stdout;
#ifdef _WIN32
			_setmode(_fileno(stdin), _O_BINARY);
#endif
		} else {
			f.files[i] = fopen(filenames[i], "wb");
			if (!f.files[i]) {
				fprintf(stderr, "Failed to open %s\n", filenames[i]);
				goto out;
			}

			char *dot = strrchr(filenames[i], '.');
			if (dot && !strcmp(dot, ".flac")) {
				f.use_flac[i] = true;
				f.use_signed[i] = false;
			} else if (dot && !strcmp(dot, ".s16") && i < 2) {
				f.use_flac[i] = false;
				f.use_signed[i] = true;
			} else {
				f.use_flac[i] = false;
				f.use_signed[i] = false;
			}

			if (f.use_flac[i])
				printf("File %d is flac!\n", i);
			if (f.use_signed[i])
				printf("File %d is signed!\n", i);
		}
	}

	r = hsdaoh_start_stream(dev, hsdaoh_callback, (void *)&f, num_bufs);

	while (!do_exit)
		usleep(50000);

	fprintf(stderr, "\nUser cancel, exiting...\n");
	hsdaoh_close(dev);

	for (int i = 0; i < FD_NUMS; i++) {
		if (!f.files[i])
			continue;

		if (f.use_flac[i] && f.encoder[i]) {
#if 0
			FLAC__metadata_object_seektable_template_sort(f.seektable[i], false);
			/* bug in libflac, fix seektable manually */
			for (int j = f.seektable[i]->data.seek_table.num_points - 1; j >= 0; j--) {
				if (f.seektable[i]->data.seek_table.points[j].stream_offset != 0)
					break;

				f.seektable[i]->data.seek_table.points[j].sample_number = 0xFFFFFFFFFFFFFFFF;
			}
#endif
			FLAC__bool ret = FLAC__stream_encoder_finish(f.encoder[i]);

			if (!ret)
				fprintf(stderr, "ERROR: FLAC encoder did not finish correctly: %s\n", FLAC__StreamEncoderStateString[FLAC__stream_encoder_get_state(f.encoder[i])]);

//			FLAC__metadata_object_delete(f.seektable[i]);
			FLAC__stream_encoder_delete(f.encoder[i]);
		} else if (f.files[i] != stdout)
			fclose(f.files[i]);
	}

out:
	return r >= 0 ? r : -r;
}
