/*
 * VISCA Camera Info Program
 * Copyright (C) 2020 Grant Likely <grant.likely@secretlab.ca>
 * Copyright (C) 2002 Damien Douxchamps <douxchamps@ieee.org>
 *
 * SPDX-License-Identifier: GPLv2+
 */

#include "../visca/libvisca.h"
#include <stdlib.h>
#include <stdio.h>

#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */

int main(int argc, char **argv)
{
	VISCAInterface_t iface;
	VISCACamera_t camera;
	int count;
	uint8_t value;
	uint16_t zoom;

	if (argc < 2) {
		printf("%s usage: %s <serial port device>\n", argv[0], argv[0]);
		exit(1);
	}

	if (VISCA_open_serial(&iface, argv[1]) != VISCA_SUCCESS) {
		printf("%s: unable to open serial device %s\n", argv[0], argv[1]);
		exit(1);
	}

	iface.broadcast = 0;
	VISCA_set_address(&iface, &count);
	printf("Found %i VISCA cameras\n", count);

	iface.broadcast = 1;
	camera.address = 0;
	VISCA_clear(&iface, &camera);
	iface.broadcast = 0;


	for (camera.address = 1; camera.address < count + 1; camera.address++) {
		printf("Camera %i:", camera.address);
		VISCA_clear(&iface, &camera);

		if (VISCA_get_camera_info(&iface, &camera) != VISCA_SUCCESS) {
			printf("error getting camera %i info\n", camera.address);
			continue;
		}
		printf("  vendor: 0x%04x\n", camera.vendor);
		printf("  model: 0x%04x\n", camera.model);
		printf("  ROM version: 0x%04x\n", camera.rom_version);
		printf("  socket number: 0x%02x\n", camera.socket_num);

		if (VISCA_get_zoom_value(&iface, &camera, &zoom) != VISCA_SUCCESS) {
			printf("error getting zoom value\n");
			continue;
		}
		printf("  Zoom value: 0x%04x\n", zoom);

		if (VISCA_get_power(&iface, &camera, &value) != VISCA_SUCCESS) {
			printf("error getting power status\n");
			continue;
		}
		printf("  power status: 0x%02x\n", value);
	}

	VISCA_close_serial(&iface);
	exit(0);
}
