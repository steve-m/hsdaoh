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
#include <getopt.h>
#define usleep(t) Sleep((t)/1000)
#endif

#include "hsdaoh.h"

#define FD_NUMS				4

static int do_exit = 0;
static hsdaoh_dev_t *dev = NULL;

typedef struct file_ctx {
	FILE *files[FD_NUMS];
} file_ctx_t;

void usage(void)
{
	fprintf(stderr,
		"hsdaoh_file, HDMI data acquisition tool\n\n"
		"Usage:\n"
		"\t[-d device_index (default: 0)]\n"
		"\t[-b maximum number of buffers (default: 16)]\n"
		"\t[-0 to -3 filename of steam 0 to stream 3 (a '-' dumps samples to stdout)]\n"
		"\tfilename (of stream 0) (a '-' dumps samples to stdout)\n\n");
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

	file_ctx_t *f = (file_ctx_t *)data_info->ctx;
	FILE *file = f->files[data_info->stream_id];

	if (!file)
		return;

	while (nbytes < len) {
		nbytes += fwrite(data_info->buf + nbytes, 1, len - nbytes, file);

		if (ferror(file)) {
			fprintf(stderr, "Error writing file, samples lost, exiting!\n");
			hsdaoh_stop_stream(dev);
			break;
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

	while ((opt = getopt(argc, argv, "0:1:2:3:d:b:")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = (uint32_t)atoi(optarg);
			break;
		case 'b':
			num_bufs = (unsigned int)atoi(optarg);
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
		}
	}

	r = hsdaoh_start_stream(dev, hsdaoh_callback, (void *)&f, num_bufs);

	while (!do_exit) {
		usleep(50000);
	}

	if (do_exit)
		fprintf(stderr, "\nUser cancel, exiting...\n");
	else
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);

	hsdaoh_close(dev);

	for (int i = 0; i < FD_NUMS; i++) {
		if (f.files[i] && (f.files[i] != stdout))
			fclose(f.files[i]);
	}

out:
	return r >= 0 ? r : -r;
}
