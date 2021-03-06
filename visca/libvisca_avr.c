/*
 * VISCA(tm) Camera Control Library
 * Copyright (C) 2002 Damien Douxchamps
 *
 * Written by Damien Douxchamps <ddouxchamps@users.sf.net>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include <string.h>
#include <errno.h>

#include "config.h"
#include "system.h"
#include "v24.h"
#include "debugging.h"
#include "libvisca.h"

uint32_t
_VISCA_write_packet_data(VISCAInterface_t *iface, VISCAPacket_t *packet)
{
	int i;

	for (i = 0; i < packet->length; ++i)
		v24Putc(iface->port_fd, packet->bytes[i]);
	return VISCA_SUCCESS;
}

uint32_t
_VISCA_get_byte(VISCAInterface_t *iface, unsigned char *buffer)
{
	int curr;

	// get one octet
	curr = v24Getc(iface->port_fd);
	if (curr < 0) {
#ifdef DEBUG
		dbg_ReportStrP(PSTR("_VISCA_get_packet: timeout\n"));
#endif
		return VISCA_FAILURE;
	}
	*buffer = (BYTE)curr;
	return VISCA_SUCCESS;
}

/***********************************/
/*       SYSTEM  FUNCTIONS         */
/***********************************/

uint32_t
VISCA_open_serial(VISCAInterface_t *iface, const char *device_name)
{
	iface->reply_packet = NULL;
	iface->ipacket.length = 0;
	iface->busy = 0;

	/* Hey, this is a microcontroller. We don't have UART device names. ;-)
	 *
	 * The used already opened
	 */
	if (!iface || !device_name) {
#ifdef DEBUG
		dbg_ReportStrP(PSTR("VISCA_open_serial: bad parms\n"));
#endif
		return VISCA_FAILURE;
	}

	iface->port_fd = UART_VISCA;

	return VISCA_SUCCESS;
}

uint32_t
VISCA_unread_bytes(VISCAInterface_t *iface, unsigned char *buffer, uint32_t *buffer_size)
{
	// TODO
	*buffer_size = 0;
	return VISCA_SUCCESS;
}

uint32_t
VISCA_close_serial(VISCAInterface_t *iface)
{
	if (!iface) {
#ifdef DEBUG
		dbg_ReportStrP(PSTR("_VISCA_close_serial: bad header parms\n"));
#endif
		return VISCA_FAILURE;
	}

	if (iface->port_fd != 0xFF) {
		/* Hey, this is a microcontroller. The port must be closed outside this
		 * function call.
		 */
		iface->port_fd = 0xFF;
		return VISCA_SUCCESS;
	}
	return VISCA_FAILURE;
}

uint32_t
VISCA_usleep(uint32_t useconds)
{
	return (uint32_t) usleep(useconds);
}

