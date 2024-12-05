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
#endif

#include "hsdaoh.h"

#define DEFAULT_SAMPLE_RATE		30000000
#define FD_NUMS				2

static int do_exit = 0;
static uint32_t bytes_to_read = 0;
static hsdaoh_dev_t *dev = NULL;

typedef struct file_ctx {
	FILE *sample_file;
	FILE *audio_file;
} file_ctx_t;

void usage(void)
{
	fprintf(stderr,
		"hsdaoh_file, HDMI data acquisition tool\n\n"
		"Usage:\n"
		"\t[-s samplerate (default: 30 MHz)]\n"
		"\t[-d device_index (default: 0)]\n"
		"\t[-p ppm_error (default: 0)]\n"
		"\t[-n number of samples to read (default: 0, infinite)]\n"
		"\tfilename (a '-' dumps samples to stdout)\n\n");
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
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
	do_exit = 1;
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

	file_ctx_t *files = (file_ctx_t *)data_info->ctx;
	FILE *file;

	if (data_info->stream_id == 0)
		file = files->sample_file;
	else
		file = files->audio_file;

	if ((bytes_to_read > 0) && (bytes_to_read < len)) {
		len = bytes_to_read;
		do_exit = 1;
		hsdaoh_stop_stream(dev);
	}

	while (nbytes < len) {
		nbytes += fwrite(data_info->buf + nbytes, 1, len - nbytes, file);

		if (ferror(file)) {
			fprintf(stderr, "Error writing file, samples lost, exiting!\n");
			hsdaoh_stop_stream(dev);
			break;
		}
	}

	if (bytes_to_read > 0)
		bytes_to_read -= len;
}

int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	char *filename = NULL;
	char *audio_filename = NULL;
	int n_read;
	int r, opt;
	int ppm_error = 0;
	file_ctx_t files;
	int dev_index = 0;
	uint32_t samp_rate = DEFAULT_SAMPLE_RATE;

	while ((opt = getopt(argc, argv, "d:s:n:p:d:a:")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = (uint32_t)atoi(optarg);
			break;
		case 's':
			samp_rate = (uint32_t)atof(optarg);
			break;
		case 'p':
			ppm_error = atoi(optarg);
			break;
		case 'n':
			bytes_to_read = (uint32_t)atof(optarg) * 2;
			break;
		case 'a':
			audio_filename = optarg;
			break;
		default:
			usage();
			break;
		}
	}

	if (argc <= optind) {
		usage();
	} else {
		filename = argv[optind];
	}

	if (dev_index < 0) {
		exit(1);
	}

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

	/* Set the sample rate */
	r = hsdaoh_set_sample_rate(dev, samp_rate, 0);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to set sample rate.\n");

	if (strcmp(filename, "-") == 0) { /* Write samples to stdout */
		files.sample_file = stdout;
#ifdef _WIN32
		_setmode(_fileno(stdin), _O_BINARY);
#endif
	} else {
		files.sample_file = fopen(filename, "wb");
		if (!files.sample_file) {
			fprintf(stderr, "Failed to open %s\n", filename);
			goto out;
		}
	}

	if (audio_filename) {
		if (strcmp(audio_filename, "-") == 0) { /* Write samples to stdout */
			files.audio_file = stdout;
#ifdef _WIN32
			_setmode(_fileno(stdin), _O_BINARY);
#endif
		} else {
			files.audio_file = fopen(audio_filename, "wb");
			if (!files.audio_file) {
				fprintf(stderr, "Failed to open %s\n", audio_filename);
				goto out;
			}
		}
	}

	fprintf(stderr, "Reading samples...\n");
	r = hsdaoh_start_stream(dev, hsdaoh_callback, (void *)&files);

	while (!do_exit) {
		usleep(50000);
	}

	if (do_exit)
		fprintf(stderr, "\nUser cancel, exiting...\n");
	else
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);

	if (files.sample_file != stdout)
		fclose(files.sample_file);

	hsdaoh_close(dev);
out:
	return r >= 0 ? r : -r;
}
