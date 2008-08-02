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
 */

#include "config.h"
#include "libvisca.h"

/********************************/
/*      PRIVATE FUNCTIONS       */
/********************************/

void
_VISCA_append_byte(VISCAPacket_t *packet, unsigned char byte)
{
  packet->bytes[packet->length]=byte;
  (packet->length)++;
}


void
_VISCA_init_packet(VISCAPacket_t *packet)
{
  // we start writing at byte 1, the first byte will be filled by the
  // packet sending function. This function will also append a terminator.
  packet->length=1;
}


unsigned int
_VISCA_send_packet(VISCAInterface_t *iface, VISCACamera_t *camera, VISCAPacket_t *packet)
{
#ifdef WIN
  DWORD iBytesWritten;
  static long bytesTogether = 0;
  BOOL rVal = 0;
  DWORD errors;
  COMSTAT stat;
  int nTrials;
#else
  int err;
#endif

  // check data:
  if ((iface->address>7)||(camera->address>7)||(iface->broadcast>1))
    {
      fprintf(stderr,"(%s): Invalid header parameters\n",__FILE__);
      fprintf(stderr," %d %d %d   \n",iface->address,camera->address,iface->broadcast);
      return VISCA_FAILURE;
    }

  // build header:
  packet->bytes[0]=0x80;
  packet->bytes[0]|=(iface->address << 4);
  if (iface->broadcast>0)
    {
      packet->bytes[0]|=(iface->broadcast << 3);
      packet->bytes[0]&=0xF8;
    }
  else
    packet->bytes[0]|=camera->address;
    
  // append footer
  _VISCA_append_byte(packet,VISCA_TERMINATOR);

#ifdef WIN
  for (nTrials = 0; nTrials < 3 && rVal == 0; nTrials++) {
    if (nTrials > 0)
      ClearCommError(iface->port_fd, &errors, &stat);
	rVal = WriteFile(iface->port_fd, &packet->bytes, packet->length, &iBytesWritten, NULL);
  }

  if ( iBytesWritten < packet->length )
    {
      DWORD lastError = GetLastError();
      return VISCA_FAILURE;
    }
  else {
    bytesTogether += iBytesWritten;
    return VISCA_SUCCESS;
  }
#else
  err=write(iface->port_fd, &packet->bytes, packet->length);

  if ( err < packet->length )
    return VISCA_FAILURE;
  else
    return VISCA_SUCCESS;
#endif

}


unsigned int
_VISCA_get_packet(VISCAInterface_t *iface)
{
  int bytes_read;
  int pos=0;
#ifdef WIN
  DWORD iBytesRead;
#endif

  // wait for message
#ifdef WIN
  ReadFile(iface->port_fd, iface->ibuf, 1, &iBytesRead, NULL);
  while (iface->ibuf[pos]!=VISCA_TERMINATOR) {
    pos++;
    ReadFile(iface->port_fd, iface->ibuf + pos, 1, &iBytesRead, NULL);
  }
#else
  ioctl(iface->port_fd, FIONREAD, &(iface->bytes));
  while (iface->bytes==0) {
    usleep(0);
    ioctl(iface->port_fd, FIONREAD, &(iface->bytes));
  }

  // get octets one by one
  bytes_read=read(iface->port_fd, iface->ibuf, 1);
  while (iface->ibuf[pos]!=VISCA_TERMINATOR) {
    pos++;
    bytes_read=read(iface->port_fd, &iface->ibuf[pos], 1);
    usleep(0);
  }
#endif
  iface->bytes=pos+1;

  return VISCA_SUCCESS;
}


unsigned int
_VISCA_get_reply(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  // first message: -------------------
  _VISCA_get_packet(iface);
  iface->type=iface->ibuf[1]&0xF0;

  // skip ack messages
  while (iface->type==VISCA_RESPONSE_ACK)
    {
      _VISCA_get_packet(iface);
      iface->type=iface->ibuf[1]&0xF0;
    }
 
  switch (iface->type)
    {
    case VISCA_RESPONSE_CLEAR:
      return VISCA_SUCCESS;
      break;
    case VISCA_RESPONSE_ADDRESS:
      return VISCA_SUCCESS;
      break;
    case VISCA_RESPONSE_COMPLETED:
      return VISCA_SUCCESS;
      break;
    case VISCA_RESPONSE_ERROR:
      return VISCA_SUCCESS;
      break;
    default:
      return VISCA_FAILURE;
      break;
    }

}


unsigned int
_VISCA_send_packet_with_reply(VISCAInterface_t *iface, VISCACamera_t *camera, VISCAPacket_t *packet)
{
  if (_VISCA_send_packet(iface,camera,packet)!=VISCA_SUCCESS)
    return VISCA_FAILURE;

  if (_VISCA_get_reply(iface,camera)!=VISCA_SUCCESS)
    return VISCA_FAILURE;
  else
    return VISCA_SUCCESS;
    
}


/****************************************************************************/
/*                           PUBLIC FUNCTIONS                               */
/****************************************************************************/


/***********************************/
/*       SYSTEM  FUNCTIONS         */
/***********************************/

unsigned int
VISCA_open_serial(VISCAInterface_t *iface, char *device_name)
{
#ifdef WIN
  BOOL     m_bPortReady;
  HANDLE   m_hCom;
  DCB      m_dcb;
  
  m_hCom = CreateFile(device_name, 
		      GENERIC_READ | GENERIC_WRITE,
		      0, // exclusive access
		      NULL, // no security
		      OPEN_EXISTING,
		      0, // no overlapped I/O
		      NULL); // null template
  
  // Check the returned handle for INVALID_HANDLE_VALUE and then set the buffer sizes.
  if (m_hCom == INVALID_HANDLE_VALUE) {
    fprintf(stderr, "(%s): cannot open serial device %s\n", __FILE__, device_name);
    iface->port_fd = NULL;
    return VISCA_FAILURE;
  }
  
  m_bPortReady = SetupComm(m_hCom, 4, 4); // set buffer sizes
  
  // Port settings are specified in a Data Communication Block (DCB). The easiest way to initialize a DCB is to call GetCommState to fill in its default values, override the values that you want to change and then call SetCommState to set the values.
  m_bPortReady = GetCommState(m_hCom, &m_dcb);
  m_dcb.BaudRate = 9600;
  m_dcb.ByteSize = 8;
  m_dcb.Parity = NOPARITY;
  m_dcb.StopBits = ONESTOPBIT;
  m_dcb.fAbortOnError = TRUE;
  m_bPortReady = SetCommState(m_hCom, &m_dcb);
  
  // If all of these API's were successful then the port is ready for use.
  iface->port_fd = m_hCom;
  iface->address = 0;

#else

  int fd;
  fd = open(device_name, O_RDWR | O_NDELAY | O_NOCTTY);

  if (fd == -1)
    {
      fprintf(stderr,"(%s): cannot open serial device %s\n",__FILE__,device_name);
      iface->port_fd=-1;
      return VISCA_FAILURE;
    }	
  else
    {
      fcntl(fd, F_SETFL,0);
      /* Setting port parameters */
      tcgetattr(fd, &iface->options);

      /* control flags */
      cfsetispeed(&iface->options,B9600);    /* 9600 Bds   */
      iface->options.c_cflag &= ~PARENB;     /* No parity  */
      iface->options.c_cflag &= ~CSTOPB;     /*            */
      iface->options.c_cflag &= ~CSIZE;      /* 8bit       */
      iface->options.c_cflag |= CS8;         /*            */
      iface->options.c_cflag &= ~CRTSCTS;    /* No hdw ctl */

      /* local flags */
      iface->options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* raw input */

      /* input flags */
      /*
      iface->options.c_iflag &= ~(INPCK | ISTRIP); // no parity
      iface->options.c_iflag &= ~(IXON | IXOFF | IXANY); // no soft ctl
      */
      /* patch: bpflegin: set to 0 in order to avoid invalid pan/tilt return values */
      interface->options.c_iflag = 0;

      /* output flags */
      iface->options.c_oflag &= ~OPOST; /* raw output */

      tcsetattr(fd, TCSANOW, &iface->options);

    }
  iface->port_fd = fd;
  iface->address=0;

#endif

  return VISCA_SUCCESS;
}


unsigned int
VISCA_close_serial(VISCAInterface_t *iface)
{
#ifdef WIN

  if (iface->port_fd != NULL)
    {
      CloseHandle(iface->port_fd);
      iface->port_fd = NULL;
      return VISCA_SUCCESS;
    }
  else
    return VISCA_FAILURE;

#else

  if (iface->port_fd!=-1)
    {
      close(iface->port_fd);
      iface->port_fd = -1;
      return VISCA_SUCCESS;
    }
  else
    return VISCA_FAILURE;

#endif
}


unsigned int
VISCA_set_address(VISCAInterface_t *iface, int *camera_num)
{
  VISCAPacket_t packet;
  int backup;
  VISCACamera_t camera; /* dummy camera struct */

  camera.address=0;
  backup=iface->broadcast;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet,0x30);
  _VISCA_append_byte(&packet,0x01);

  iface->broadcast=1;
  if (_VISCA_send_packet(iface, &camera, &packet)!=VISCA_SUCCESS)
    {
      iface->broadcast=backup;
      return VISCA_FAILURE;
    }
  else
    iface->broadcast=backup;
  
  if (_VISCA_get_reply(iface, &camera)!=VISCA_SUCCESS)
    return VISCA_FAILURE;
  else
    {
      /* We parse the message from the camera here  */
      /* We expect to receive 4*camera_num bytes,
         every packet should be 88 30 0x FF, x being
         the camera id+1. The number of cams will thus be
         ibuf[bytes-2]-1  */
      if ((iface->bytes & 0x3)!=0) /* check multiple of 4 */
	return VISCA_FAILURE;
      else
	{
	  *camera_num=iface->ibuf[iface->bytes-2]-1;
	  if ((*camera_num==0)||(*camera_num>7))
	    return VISCA_FAILURE;
	  else
	    return VISCA_SUCCESS;
	}
    }
  
}


unsigned int
VISCA_clear(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet,0x01);
  _VISCA_append_byte(&packet,0x00);
  _VISCA_append_byte(&packet,0x01);

  if (_VISCA_send_packet(iface, camera, &packet)!=VISCA_SUCCESS)
    return VISCA_FAILURE;
  else
    if (_VISCA_get_reply(iface, camera)!=VISCA_SUCCESS)
      return VISCA_FAILURE;
    else
      return VISCA_SUCCESS;
}

unsigned int
VISCA_get_camera_info(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;
#ifdef WIN
  DWORD iBytesWritten;
#endif
  packet.bytes[0]=0x80 | camera->address;
  packet.bytes[1]=0x09;
  packet.bytes[2]=0x00;
  packet.bytes[3]=0x02;
  packet.bytes[4]=VISCA_TERMINATOR;
  packet.length=5;

#ifdef WIN
  WriteFile(iface->port_fd, packet.bytes, packet.length, &iBytesWritten, NULL);
#else
  write(iface->port_fd, packet.bytes, packet.length);
#endif

  if (_VISCA_get_reply(iface, camera)!=VISCA_SUCCESS)
    return VISCA_FAILURE;

  if (iface->bytes!= 10) /* we expect 10 bytes as answer */
    return VISCA_FAILURE;
  else
    {
      camera->vendor=(iface->ibuf[2]<<8) + iface->ibuf[3];
      camera->model=(iface->ibuf[4]<<8) + iface->ibuf[5];
      camera->rom_version=(iface->ibuf[6]<<8) + iface->ibuf[7];
      camera->socket_num=iface->ibuf[8];
      return VISCA_SUCCESS;
    }
}

/***********************************/
/*       COMMAND FUNCTIONS         */
/***********************************/

unsigned int
VISCA_set_power(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_POWER);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_keylock(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_KEYLOCK);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_camera_id(VISCAInterface_t *iface, VISCACamera_t *camera, UInt16_t id)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_ID);
  _VISCA_append_byte(&packet, (id & 0xF000) >> 12);
  _VISCA_append_byte(&packet, (id & 0x0F00) >>  8);
  _VISCA_append_byte(&packet, (id & 0x00F0) >>  4);
  _VISCA_append_byte(&packet, (id & 0x000F));

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_zoom_tele(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_ZOOM);
  _VISCA_append_byte(&packet, VISCA_ZOOM_TELE);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_zoom_wide(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_ZOOM);
  _VISCA_append_byte(&packet, VISCA_ZOOM_WIDE);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_zoom_stop(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_ZOOM);
  _VISCA_append_byte(&packet, VISCA_ZOOM_STOP);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_zoom_tele_speed(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int speed)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_ZOOM);
  _VISCA_append_byte(&packet, VISCA_ZOOM_TELE_SPEED | (speed & 0x7));

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_zoom_wide_speed(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int speed)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_ZOOM);
  _VISCA_append_byte(&packet, VISCA_ZOOM_WIDE_SPEED | (speed & 0x7));

  return _VISCA_send_packet_with_reply(iface, camera, &packet);

}


unsigned int
VISCA_set_zoom_value(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int zoom)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_ZOOM_VALUE);
  _VISCA_append_byte(&packet, (zoom & 0xF000) >> 12);
  _VISCA_append_byte(&packet, (zoom & 0x0F00) >>  8);
  _VISCA_append_byte(&packet, (zoom & 0x00F0) >>  4);
  _VISCA_append_byte(&packet, (zoom & 0x000F));

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_zoom_and_focus_value(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int zoom, unsigned int focus)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_ZOOM_FOCUS_VALUE);
  _VISCA_append_byte(&packet, (zoom & 0xF000) >> 12);
  _VISCA_append_byte(&packet, (zoom & 0x0F00) >>  8);
  _VISCA_append_byte(&packet, (zoom & 0x00F0) >>  4);
  _VISCA_append_byte(&packet, (zoom & 0x000F));
  _VISCA_append_byte(&packet, (focus & 0xF000) >> 12);
  _VISCA_append_byte(&packet, (focus & 0x0F00) >>  8);
  _VISCA_append_byte(&packet, (focus & 0x00F0) >>  4);
  _VISCA_append_byte(&packet, (focus & 0x000F));
 
  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_dzoom(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_DZOOM);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_focus_far(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FOCUS);
  _VISCA_append_byte(&packet, VISCA_FOCUS_FAR);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_focus_near(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FOCUS);
  _VISCA_append_byte(&packet, VISCA_FOCUS_NEAR);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_focus_stop(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FOCUS);
  _VISCA_append_byte(&packet, VISCA_FOCUS_STOP);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_focus_far_speed(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int speed)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FOCUS);
  _VISCA_append_byte(&packet, VISCA_FOCUS_FAR_SPEED | (speed & 0x7));

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_focus_near_speed(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int speed)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FOCUS);
  _VISCA_append_byte(&packet, VISCA_FOCUS_NEAR_SPEED | (speed & 0x7));

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_focus_value(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int focus)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FOCUS_VALUE);
  _VISCA_append_byte(&packet, (focus & 0xF000) >> 12);
  _VISCA_append_byte(&packet, (focus & 0x0F00) >>  8);
  _VISCA_append_byte(&packet, (focus & 0x00F0) >>  4);
  _VISCA_append_byte(&packet, (focus & 0x000F));

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_focus_auto(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FOCUS_AUTO);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_focus_one_push(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FOCUS_ONE_PUSH);
  _VISCA_append_byte(&packet, VISCA_FOCUS_ONE_PUSH_TRIG);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_focus_infinity(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FOCUS_ONE_PUSH);
  _VISCA_append_byte(&packet, VISCA_FOCUS_ONE_PUSH_INF);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_focus_autosense_high(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FOCUS_AUTO_SENSE);
  _VISCA_append_byte(&packet, VISCA_FOCUS_AUTO_SENSE_HIGH);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_focus_autosense_low(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FOCUS_AUTO_SENSE);
  _VISCA_append_byte(&packet, VISCA_FOCUS_AUTO_SENSE_LOW);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_focus_near_limit(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int limit)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FOCUS_NEAR_LIMIT);
  _VISCA_append_byte(&packet, (limit & 0xF000) >> 12);
  _VISCA_append_byte(&packet, (limit & 0x0F00) >>  8);
  _VISCA_append_byte(&packet, (limit & 0x00F0) >>  4);
  _VISCA_append_byte(&packet, (limit & 0x000F));

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_whitebal_mode(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int mode)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_WB);
  _VISCA_append_byte(&packet, mode);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_whitebal_one_push(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_WB_ONE_PUSH);
  _VISCA_append_byte(&packet, VISCA_WB_ONE_PUSH_TRIG);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_rgain_up(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_RGAIN);
  _VISCA_append_byte(&packet, VISCA_UP);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_rgain_down(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_RGAIN);
  _VISCA_append_byte(&packet, VISCA_DOWN);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_rgain_reset(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_RGAIN);
  _VISCA_append_byte(&packet, VISCA_RESET);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);

}


unsigned int
VISCA_set_rgain_value(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int value)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_RGAIN_VALUE);
  _VISCA_append_byte(&packet, (value & 0xF000) >> 12);
  _VISCA_append_byte(&packet, (value & 0x0F00) >>  8);
  _VISCA_append_byte(&packet, (value & 0x00F0) >>  4);
  _VISCA_append_byte(&packet, (value & 0x000F));

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_bgain_up(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_BGAIN);
  _VISCA_append_byte(&packet, VISCA_UP);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_bgain_down(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_BGAIN);
  _VISCA_append_byte(&packet, VISCA_DOWN);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_bgain_reset(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_BGAIN);
  _VISCA_append_byte(&packet, VISCA_RESET);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_bgain_value(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int value)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_BGAIN_VALUE);
  _VISCA_append_byte(&packet, (value & 0xF000) >> 12);
  _VISCA_append_byte(&packet, (value & 0x0F00) >>  8);
  _VISCA_append_byte(&packet, (value & 0x00F0) >>  4);
 _VISCA_append_byte(&packet, (value & 0x000F));

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_shutter_up(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_SHUTTER);
  _VISCA_append_byte(&packet, VISCA_UP);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_shutter_down(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_SHUTTER);
  _VISCA_append_byte(&packet, VISCA_DOWN);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_shutter_reset(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_SHUTTER);
  _VISCA_append_byte(&packet, VISCA_RESET);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_shutter_value(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int value)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_SHUTTER_VALUE);
  _VISCA_append_byte(&packet, (value & 0xF000) >> 12);
  _VISCA_append_byte(&packet, (value & 0x0F00) >>  8);
  _VISCA_append_byte(&packet, (value & 0x00F0) >>  4);
 _VISCA_append_byte(&packet, (value & 0x000F));

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_iris_up(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_IRIS);
  _VISCA_append_byte(&packet, VISCA_UP);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_iris_down(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_IRIS);
  _VISCA_append_byte(&packet, VISCA_DOWN);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_iris_reset(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_IRIS);
  _VISCA_append_byte(&packet, VISCA_RESET);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_iris_value(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int value)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_IRIS_VALUE);
  _VISCA_append_byte(&packet, (value & 0xF000) >> 12);
  _VISCA_append_byte(&packet, (value & 0x0F00) >>  8);
  _VISCA_append_byte(&packet, (value & 0x00F0) >>  4);
 _VISCA_append_byte(&packet, (value & 0x000F));

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_gain_up(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_GAIN);
  _VISCA_append_byte(&packet, VISCA_UP);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_gain_down(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_GAIN);
  _VISCA_append_byte(&packet, VISCA_DOWN);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_gain_reset(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_GAIN);
  _VISCA_append_byte(&packet, VISCA_RESET);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_gain_value(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int value)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_GAIN_VALUE);
  _VISCA_append_byte(&packet, (value & 0xF000) >> 12);
  _VISCA_append_byte(&packet, (value & 0x0F00) >>  8);
  _VISCA_append_byte(&packet, (value & 0x00F0) >>  4);
  _VISCA_append_byte(&packet, (value & 0x000F));

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_bright_up(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_BRIGHT);
  _VISCA_append_byte(&packet, VISCA_UP);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_bright_down(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_BRIGHT);
  _VISCA_append_byte(&packet, VISCA_DOWN);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_bright_reset(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_BRIGHT);
  _VISCA_append_byte(&packet, VISCA_RESET);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_bright_value(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int value)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_BRIGHT_VALUE);
  _VISCA_append_byte(&packet, (value & 0xF000) >> 12);
  _VISCA_append_byte(&packet, (value & 0x0F00) >>  8);
  _VISCA_append_byte(&packet, (value & 0x00F0) >>  4);
  _VISCA_append_byte(&packet, (value & 0x000F));

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_aperture_up(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_APERTURE);
  _VISCA_append_byte(&packet, VISCA_UP);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_aperture_down(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_APERTURE);
  _VISCA_append_byte(&packet, VISCA_DOWN);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_aperture_reset(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_APERTURE);
  _VISCA_append_byte(&packet, VISCA_RESET);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_aperture_value(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int value)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_APERTURE_VALUE);
  _VISCA_append_byte(&packet, (value & 0xF000) >> 12);
  _VISCA_append_byte(&packet, (value & 0x0F00) >>  8);
  _VISCA_append_byte(&packet, (value & 0x00F0) >>  4);
  _VISCA_append_byte(&packet, (value & 0x000F));

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_exp_comp_up(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_EXP_COMP);
  _VISCA_append_byte(&packet, VISCA_UP);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_exp_comp_down(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_EXP_COMP);
  _VISCA_append_byte(&packet, VISCA_DOWN);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_exp_comp_reset(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_EXP_COMP);
  _VISCA_append_byte(&packet, VISCA_RESET);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);

  return VISCA_SUCCESS;
}


unsigned int
VISCA_set_exp_comp_value(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int value)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_EXP_COMP_VALUE);
  _VISCA_append_byte(&packet, (value & 0xF000) >> 12);
  _VISCA_append_byte(&packet, (value & 0x0F00) >>  8);
  _VISCA_append_byte(&packet, (value & 0x00F0) >>  4);
  _VISCA_append_byte(&packet, (value & 0x000F));

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_exp_comp_power(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_EXP_COMP_POWER);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_auto_exp_mode(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t mode)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_AUTO_EXP);
  _VISCA_append_byte(&packet, mode);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_slow_shutter_auto(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_SLOW_SHUTTER);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_backlight_comp(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_BACKLIGHT_COMP);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_zero_lux_shot(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_ZERO_LUX);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_ir_led(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_IR_LED);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_wide_mode(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t mode)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_WIDE_MODE);
  _VISCA_append_byte(&packet, mode);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_mirror(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_MIRROR);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_freeze(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FREEZE);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_picture_effect(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t mode)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_PICTURE_EFFECT);
  _VISCA_append_byte(&packet, mode);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_digital_effect(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t mode)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_DIGITAL_EFFECT);
  _VISCA_append_byte(&packet, mode);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_digital_effect_level(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t level)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_DIGITAL_EFFECT_LEVEL);
  _VISCA_append_byte(&packet, level);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_memory_set(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t channel)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_MEMORY);
  _VISCA_append_byte(&packet, VISCA_MEMORY_SET);
  _VISCA_append_byte(&packet, channel);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_memory_recall(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t channel)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_MEMORY);
  _VISCA_append_byte(&packet, VISCA_MEMORY_RECALL);
  _VISCA_append_byte(&packet, channel);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_memory_reset(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t channel)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_MEMORY);
  _VISCA_append_byte(&packet, VISCA_MEMORY_RESET);
  _VISCA_append_byte(&packet, channel);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_display(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_DISPLAY);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_date_time(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int year, unsigned int month, unsigned int day, unsigned int hour, unsigned int minute)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_DATE_TIME_SET);
  _VISCA_append_byte(&packet, year/10);
  _VISCA_append_byte(&packet, year-10*(year/10));
  _VISCA_append_byte(&packet, month/10);
  _VISCA_append_byte(&packet, month-10*(month/10));
  _VISCA_append_byte(&packet, day/10);
  _VISCA_append_byte(&packet, day-10*(day/10));
  _VISCA_append_byte(&packet, hour/10);
  _VISCA_append_byte(&packet, hour-10*(hour/10));
  _VISCA_append_byte(&packet, minute/10);
  _VISCA_append_byte(&packet, minute-10*(minute/10));

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_date_display(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_DATE_DISPLAY);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_time_display(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_TIME_DISPLAY);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_title_display(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_TITLE_DISPLAY);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_title_clear(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_TITLE_DISPLAY);
  _VISCA_append_byte(&packet, VISCA_TITLE_DISPLAY_CLEAR);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_title_params(VISCAInterface_t *iface, VISCACamera_t *camera, VISCATitleData_t *title)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_TITLE_SET);
  _VISCA_append_byte(&packet, VISCA_TITLE_SET_PARAMS);
  _VISCA_append_byte(&packet, title->vposition);
  _VISCA_append_byte(&packet, title->hposition);
  _VISCA_append_byte(&packet, title->color);
  _VISCA_append_byte(&packet, title->blink);
  _VISCA_append_byte(&packet, 0);
  _VISCA_append_byte(&packet, 0);
  _VISCA_append_byte(&packet, 0);
  _VISCA_append_byte(&packet, 0);
  _VISCA_append_byte(&packet, 0);
  _VISCA_append_byte(&packet, 0);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_title(VISCAInterface_t *iface, VISCACamera_t *camera, VISCATitleData_t *title)
{
  VISCAPacket_t packet;
  int i, err=0;
  
  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_TITLE_SET);
  _VISCA_append_byte(&packet, VISCA_TITLE_SET_PART1);

  for (i=0;i<10;i++)
    _VISCA_append_byte(&packet, title->title[i]);

  err+=_VISCA_send_packet_with_reply(iface, camera, &packet);

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_TITLE_SET);
  _VISCA_append_byte(&packet, VISCA_TITLE_SET_PART2);

  for (i=0;i<10;i++)
    _VISCA_append_byte(&packet, title->title[i+10]);

  err+=_VISCA_send_packet_with_reply(iface, camera, &packet);

  return err;
}

/***********************************/
/*       INQUIRY FUNCTIONS         */
/***********************************/

unsigned int
VISCA_get_power(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_POWER);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_dzoom(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_DZOOM);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_zoom_value(VISCAInterface_t *iface, VISCACamera_t *camera, UInt16_t *value)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_ZOOM_VALUE);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *value=(iface->ibuf[2]<<12)+(iface->ibuf[3]<<8)+(iface->ibuf[4]<<4)+iface->ibuf[5];
      return VISCA_SUCCESS;
    }

}


unsigned int
VISCA_get_focus_auto(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FOCUS_AUTO);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_focus_value(VISCAInterface_t *iface, VISCACamera_t *camera, UInt16_t *value)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FOCUS_VALUE);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *value=(iface->ibuf[2]<<12)+(iface->ibuf[3]<<8)+(iface->ibuf[4]<<4)+iface->ibuf[5];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_focus_auto_sense(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *mode)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FOCUS_AUTO_SENSE );
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *mode=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_focus_near_limit(VISCAInterface_t *iface, VISCACamera_t *camera, UInt16_t *value)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FOCUS_NEAR_LIMIT);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *value=(iface->ibuf[2]<<12)+(iface->ibuf[3]<<8)+(iface->ibuf[4]<<4)+iface->ibuf[5];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_whitebal_mode(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *mode)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_WB);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *mode=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_rgain_value(VISCAInterface_t *iface, VISCACamera_t *camera, UInt16_t *value)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_RGAIN_VALUE);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *value=(iface->ibuf[2]<<12)+(iface->ibuf[3]<<8)+(iface->ibuf[4]<<4)+iface->ibuf[5];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_bgain_value(VISCAInterface_t *iface, VISCACamera_t *camera, UInt16_t *value)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_BGAIN_VALUE);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *value=(iface->ibuf[2]<<12)+(iface->ibuf[3]<<8)+(iface->ibuf[4]<<4)+iface->ibuf[5];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_auto_exp_mode(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *mode)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_AUTO_EXP);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *mode=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_slow_shutter_auto(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *mode)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_SLOW_SHUTTER);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *mode=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_shutter_value(VISCAInterface_t *iface, VISCACamera_t *camera, UInt16_t *value)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_SHUTTER_VALUE);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *value=(iface->ibuf[2]<<12)+(iface->ibuf[3]<<8)+(iface->ibuf[4]<<4)+iface->ibuf[5];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_iris_value(VISCAInterface_t *iface, VISCACamera_t *camera, UInt16_t *value)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_IRIS_VALUE);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *value=(iface->ibuf[2]<<12)+(iface->ibuf[3]<<8)+(iface->ibuf[4]<<4)+iface->ibuf[5];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_gain_value(VISCAInterface_t *iface, VISCACamera_t *camera, UInt16_t *value)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_GAIN_VALUE);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *value=(iface->ibuf[2]<<12)+(iface->ibuf[3]<<8)+(iface->ibuf[4]<<4)+iface->ibuf[5];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_bright_value(VISCAInterface_t *iface, VISCACamera_t *camera, UInt16_t *value)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_BRIGHT_VALUE);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *value=(iface->ibuf[2]<<12)+(iface->ibuf[3]<<8)+(iface->ibuf[4]<<4)+iface->ibuf[5];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_exp_comp_power(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_EXP_COMP_POWER);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_exp_comp_value(VISCAInterface_t *iface, VISCACamera_t *camera, UInt16_t *value)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_EXP_COMP_VALUE);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *value=(iface->ibuf[2]<<12)+(iface->ibuf[3]<<8)+(iface->ibuf[4]<<4)+iface->ibuf[5];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_backlight_comp(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_BACKLIGHT_COMP);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_aperture_value(VISCAInterface_t *iface, VISCACamera_t *camera, UInt16_t *value)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_APERTURE_VALUE);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *value=(iface->ibuf[2]<<12)+(iface->ibuf[3]<<8)+(iface->ibuf[4]<<4)+iface->ibuf[5];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_zero_lux_shot(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_ZERO_LUX);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_ir_led(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_IR_LED);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_wide_mode(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *mode)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_WIDE_MODE);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *mode=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_mirror(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_MIRROR);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_freeze(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_FREEZE);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_picture_effect(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *mode)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_PICTURE_EFFECT);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *mode=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_digital_effect(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *mode)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_DIGITAL_EFFECT);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *mode=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_digital_effect_level(VISCAInterface_t *iface, VISCACamera_t *camera, UInt16_t *value)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_DIGITAL_EFFECT_LEVEL);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *value=(iface->ibuf[2]<<12)+(iface->ibuf[3]<<8)+(iface->ibuf[4]<<4)+iface->ibuf[5];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_memory(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *channel)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_MEMORY);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *channel=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_display(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_DISPLAY);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_id(VISCAInterface_t *iface, VISCACamera_t *camera, UInt16_t *id)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_ID);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *id=(iface->ibuf[2]<<12)+(iface->ibuf[3]<<8)+(iface->ibuf[4]<<4)+iface->ibuf[5];
      return VISCA_SUCCESS;
    }
}
unsigned int
VISCA_set_irreceive_on(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_IRRECEIVE);
  _VISCA_append_byte(&packet, VISCA_IRRECEIVE_ON);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}

unsigned int
VISCA_set_irreceive_off(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_IRRECEIVE);
  _VISCA_append_byte(&packet, VISCA_IRRECEIVE_OFF);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}

unsigned int
VISCA_set_irreceive_onoff(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_IRRECEIVE);
  _VISCA_append_byte(&packet, VISCA_IRRECEIVE_ONOFF);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_pantilt_up(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int pan_speed, unsigned int tilt_speed)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE);
  _VISCA_append_byte(&packet, pan_speed);
  _VISCA_append_byte(&packet, tilt_speed);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_HORIZ_STOP);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_VERT_UP);
  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}

unsigned int
VISCA_set_pantilt_down(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int pan_speed, unsigned int tilt_speed)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE);
  _VISCA_append_byte(&packet, pan_speed);
  _VISCA_append_byte(&packet, tilt_speed);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_HORIZ_STOP);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_VERT_DOWN);
  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}

unsigned int
VISCA_set_pantilt_left(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int pan_speed, unsigned int tilt_speed)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE);
  _VISCA_append_byte(&packet, pan_speed);
  _VISCA_append_byte(&packet, tilt_speed);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_HORIZ_LEFT);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_VERT_STOP);
  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}

unsigned int
VISCA_set_pantilt_right(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int pan_speed, unsigned int tilt_speed)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE);
  _VISCA_append_byte(&packet, pan_speed);
  _VISCA_append_byte(&packet, tilt_speed);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_HORIZ_RIGHT);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_VERT_STOP);
  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}

unsigned int
VISCA_set_pantilt_upleft(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int pan_speed, unsigned int tilt_speed)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE);
  _VISCA_append_byte(&packet, pan_speed);
  _VISCA_append_byte(&packet, tilt_speed);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_HORIZ_LEFT);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_VERT_UP);
  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}

unsigned int
VISCA_set_pantilt_upright(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int pan_speed, unsigned int tilt_speed)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE);
  _VISCA_append_byte(&packet, pan_speed);
  _VISCA_append_byte(&packet, tilt_speed);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_HORIZ_RIGHT);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_VERT_UP);
  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}

unsigned int
VISCA_set_pantilt_downleft(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int pan_speed, unsigned int tilt_speed)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE);
  _VISCA_append_byte(&packet, pan_speed);
  _VISCA_append_byte(&packet, tilt_speed);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_HORIZ_LEFT);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_VERT_DOWN);
  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}

unsigned int
VISCA_set_pantilt_downright(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int pan_speed, unsigned int tilt_speed)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE);
  _VISCA_append_byte(&packet, pan_speed);
  _VISCA_append_byte(&packet, tilt_speed);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_HORIZ_RIGHT);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_VERT_DOWN);
  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}

unsigned int
VISCA_set_pantilt_stop(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int pan_speed, unsigned int tilt_speed)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE);
  _VISCA_append_byte(&packet, pan_speed);
  _VISCA_append_byte(&packet, tilt_speed);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_HORIZ_STOP);
  _VISCA_append_byte(&packet, VISCA_PT_DRIVE_VERT_STOP);
  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}

unsigned int
VISCA_set_pantilt_absolute_position(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int pan_speed, unsigned int tilt_speed, int pan_position, int tilt_position)
{
  VISCAPacket_t packet;

  unsigned int pan_pos=(unsigned int) pan_position;
  unsigned int tilt_pos=(unsigned int) tilt_position;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_ABSOLUTE_POSITION);
  _VISCA_append_byte(&packet, pan_speed);
  _VISCA_append_byte(&packet, tilt_speed);
  _VISCA_append_byte(&packet, (pan_pos & 0xf0000) >> 16);
  _VISCA_append_byte(&packet, (pan_pos & 0x0f000) >> 12);
  _VISCA_append_byte(&packet, (pan_pos & 0x00f00) >>  8);
  _VISCA_append_byte(&packet, (pan_pos & 0x000f0) >>  4);
  _VISCA_append_byte(&packet,  pan_pos & 0x0000f       );

  _VISCA_append_byte(&packet, (tilt_pos & 0xf000) >> 12);
  _VISCA_append_byte(&packet, (tilt_pos & 0x0f00) >> 8);
  _VISCA_append_byte(&packet, (tilt_pos & 0x00f0) >> 4);
  _VISCA_append_byte(&packet, tilt_pos & 0x000f);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_pantilt_relative_position(VISCAInterface_t *iface, VISCACamera_t *camera, unsigned int pan_speed, unsigned int tilt_speed, int pan_position, int tilt_position)
{
  VISCAPacket_t packet;

  unsigned int pan_pos=(unsigned int) pan_position;
  unsigned int tilt_pos=(unsigned int) tilt_position;
  

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_RELATIVE_POSITION);
  _VISCA_append_byte(&packet, pan_speed);
  _VISCA_append_byte(&packet, tilt_speed);
  _VISCA_append_byte(&packet, (pan_pos & 0xf0000) >> 16);
  _VISCA_append_byte(&packet, (pan_pos & 0x0f000) >> 12);
  _VISCA_append_byte(&packet, (pan_pos & 0x00f00) >>  8);
  _VISCA_append_byte(&packet, (pan_pos & 0x000f0) >>  4);
  _VISCA_append_byte(&packet,  pan_pos & 0x0000f       );

  _VISCA_append_byte(&packet, (tilt_pos & 0xf000) >> 12);
  _VISCA_append_byte(&packet, (tilt_pos & 0x0f00) >> 8);
  _VISCA_append_byte(&packet, (tilt_pos & 0x00f0) >> 4);
  _VISCA_append_byte(&packet, tilt_pos & 0x000f);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}

unsigned int
VISCA_set_pantilt_home(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_HOME);
  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}

unsigned int
VISCA_set_pantilt_reset(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_RESET);
  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}

unsigned int
VISCA_set_pantilt_limit_upright(VISCAInterface_t *iface, VISCACamera_t *camera, int pan_position, int tilt_position)
{
  VISCAPacket_t packet;

  unsigned int pan_pos=(unsigned int) pan_position;
  unsigned int tilt_pos=(unsigned int) tilt_position;
  

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_LIMITSET);
  _VISCA_append_byte(&packet, VISCA_PT_LIMITSET_SET);
  _VISCA_append_byte(&packet, VISCA_PT_LIMITSET_SET_UR);
  _VISCA_append_byte(&packet, (pan_pos & 0xf000) >> 12);
  _VISCA_append_byte(&packet, (pan_pos & 0x0f00) >> 8);
  _VISCA_append_byte(&packet, (pan_pos & 0x00f0) >> 4);
  _VISCA_append_byte(&packet, pan_pos & 0x000f);

  _VISCA_append_byte(&packet, (tilt_pos & 0xf000) >> 12);
  _VISCA_append_byte(&packet, (tilt_pos & 0x0f00) >> 8);
  _VISCA_append_byte(&packet, (tilt_pos & 0x00f0) >> 4);
  _VISCA_append_byte(&packet, tilt_pos & 0x000f);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_pantilt_limit_downleft(VISCAInterface_t *iface, VISCACamera_t *camera, int pan_position, int tilt_position)
{
  VISCAPacket_t packet;

  unsigned int pan_pos=(unsigned int) pan_position;
  unsigned int tilt_pos=(unsigned int) tilt_position;
  

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_LIMITSET);
  _VISCA_append_byte(&packet, VISCA_PT_LIMITSET_SET);
  _VISCA_append_byte(&packet, VISCA_PT_LIMITSET_SET_DL);
  _VISCA_append_byte(&packet, (pan_pos & 0xf000) >> 12);
  _VISCA_append_byte(&packet, (pan_pos & 0x0f00) >> 8);
  _VISCA_append_byte(&packet, (pan_pos & 0x00f0) >> 4);
  _VISCA_append_byte(&packet, pan_pos & 0x000f);

  _VISCA_append_byte(&packet, (tilt_pos & 0xf000) >> 12);
  _VISCA_append_byte(&packet, (tilt_pos & 0x0f00) >> 8);
  _VISCA_append_byte(&packet, (tilt_pos & 0x00f0) >> 4);
  _VISCA_append_byte(&packet, tilt_pos & 0x000f);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_pantilt_limit_downleft_clear(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  unsigned int pan_pos=0x7fff;
  unsigned int tilt_pos=0x7fff;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_LIMITSET);
  _VISCA_append_byte(&packet, VISCA_PT_LIMITSET_CLEAR);
  _VISCA_append_byte(&packet, VISCA_PT_LIMITSET_SET_DL);
  _VISCA_append_byte(&packet, (pan_pos & 0xf000) >> 12);
  _VISCA_append_byte(&packet, (pan_pos & 0x0f00) >> 8);
  _VISCA_append_byte(&packet, (pan_pos & 0x00f0) >> 4);
  _VISCA_append_byte(&packet, pan_pos & 0x000f);

  _VISCA_append_byte(&packet, (tilt_pos & 0xf000) >> 12);
  _VISCA_append_byte(&packet, (tilt_pos & 0x0f00) >> 8);
  _VISCA_append_byte(&packet, (tilt_pos & 0x00f0) >> 4);
  _VISCA_append_byte(&packet, tilt_pos & 0x000f);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_pantilt_limit_upright_clear(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  unsigned int pan_pos=0x7fff;
  unsigned int tilt_pos=0x7fff;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_LIMITSET);
  _VISCA_append_byte(&packet, VISCA_PT_LIMITSET_CLEAR);
  _VISCA_append_byte(&packet, VISCA_PT_LIMITSET_SET_UR);
  _VISCA_append_byte(&packet, (pan_pos & 0xf000) >> 12);
  _VISCA_append_byte(&packet, (pan_pos & 0x0f00) >> 8);
  _VISCA_append_byte(&packet, (pan_pos & 0x00f0) >> 4);
  _VISCA_append_byte(&packet, pan_pos & 0x000f);

  _VISCA_append_byte(&packet, (tilt_pos & 0xf000) >> 12);
  _VISCA_append_byte(&packet, (tilt_pos & 0x0f00) >> 8);
  _VISCA_append_byte(&packet, (tilt_pos & 0x00f0) >> 4);
  _VISCA_append_byte(&packet, tilt_pos & 0x000f);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_datascreen_on(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_DATASCREEN);
  _VISCA_append_byte(&packet, VISCA_PT_DATASCREEN_ON);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}

unsigned int
VISCA_set_datascreen_off(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_DATASCREEN);
  _VISCA_append_byte(&packet, VISCA_PT_DATASCREEN_OFF);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}

unsigned int
VISCA_set_datascreen_onoff(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_DATASCREEN);
  _VISCA_append_byte(&packet, VISCA_PT_DATASCREEN_ONOFF);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}



/***********************************/
/*       INQUIRY FUNCTIONS         */
/***********************************/

unsigned int
VISCA_get_videosystem(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *system)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_VIDEOSYSTEM_INQ);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *system=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}

unsigned int
VISCA_get_pantilt_mode(VISCAInterface_t *iface, VISCACamera_t *camera, UInt16_t *status)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_MODE_INQ);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *status = ((iface->ibuf[2] & 0xff) << 8) + (iface->ibuf[3] & 0xff);
      return VISCA_SUCCESS;
    }
}

unsigned int
VISCA_get_pantilt_maxspeed(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *max_pan_speed, UInt8_t *max_tilt_speed)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_MAXSPEED_INQ);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *max_pan_speed = (iface->ibuf[2] & 0xff);
      *max_tilt_speed = (iface->ibuf[3] & 0xff);
      return VISCA_SUCCESS;
    }
}

unsigned int
VISCA_get_pantilt_position(VISCAInterface_t *iface, VISCACamera_t *camera, int *pan_position, int *tilt_position)
{
  VISCAPacket_t packet;
  unsigned int err;
  UInt16_t pan_pos, tilt_pos;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_POSITION_INQ);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      pan_pos  = ((iface->ibuf[3] & 0xf) << 12) + ((iface->ibuf[4] & 0xf) << 8) + ((iface->ibuf[5] & 0xf) << 4) + (iface->ibuf[6] & 0xf); 
      tilt_pos = ((iface->ibuf[7] & 0xf) << 12) + ((iface->ibuf[8] & 0xf) << 8) + ((iface->ibuf[9] & 0xf) << 4) + (iface->ibuf[10] & 0xf); 

      if (!iface->ibuf[2]) *pan_position=pan_pos;
      else *pan_position=((int)pan_pos) - 65536;
      if (tilt_pos<0x8000) *tilt_position=tilt_pos;
      else *tilt_position=((int)tilt_pos) - 65536;

      return VISCA_SUCCESS;
    }
}

unsigned int
VISCA_get_datascreen(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *status)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_PT_DATASCREEN_INQ);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *status=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


/********************************/
/* SPECIAL FUNCTIONS FOR D30/31 */
/********************************/

unsigned int
VISCA_set_wide_con_lens(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_WIDE_CON_LENS);
  _VISCA_append_byte(&packet, VISCA_WIDE_CON_LENS_SET);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_at_mode_onoff(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_AT_MODE);
  _VISCA_append_byte(&packet, VISCA_AT_ONOFF);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_at_mode(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_AT_MODE);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_at_ae_onoff(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_AT_AE);
  _VISCA_append_byte(&packet, VISCA_AT_ONOFF);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_at_ae(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_AT_AE);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_at_autozoom_onoff(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_AT_AUTOZOOM);
  _VISCA_append_byte(&packet, VISCA_AT_ONOFF);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_at_autozoom(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_AT_AUTOZOOM);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_atmd_framedisplay_onoff(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_ATMD_FRAMEDISPLAY);
  _VISCA_append_byte(&packet, VISCA_AT_ONOFF);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_atmd_framedisplay(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_ATMD_FRAMEDISPLAY);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_at_frameoffset_onoff(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_AT_FRAMEOFFSET);
  _VISCA_append_byte(&packet, VISCA_AT_ONOFF);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_at_frameoffset(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_AT_FRAMEOFFSET);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_atmd_startstop(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_ATMD_STARTSTOP);
  _VISCA_append_byte(&packet, VISCA_AT_ONOFF);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_at_chase(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_AT_CHASE);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}

unsigned int
VISCA_set_at_chase_next(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_AT_CHASE);
  _VISCA_append_byte(&packet, VISCA_AT_CHASE_NEXT);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_md_mode_onoff(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_MODE);
  _VISCA_append_byte(&packet, VISCA_MD_ONOFF);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_md_mode(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_MODE);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_md_frame(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_FRAME);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_md_detect(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_DETECT);
  _VISCA_append_byte(&packet, VISCA_MD_ONOFF);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_at_entry(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_AT_ENTRY);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_at_lostinfo(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_ATMD_LOSTINFO1);
  _VISCA_append_byte(&packet, VISCA_ATMD_LOSTINFO2);
  _VISCA_append_byte(&packet, VISCA_AT_LOSTINFO);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_md_lostinfo(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
  _VISCA_append_byte(&packet, VISCA_ATMD_LOSTINFO1);
  _VISCA_append_byte(&packet, VISCA_ATMD_LOSTINFO2);
  _VISCA_append_byte(&packet, VISCA_MD_LOSTINFO);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_md_adjust_ylevel(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_ADJUST_YLEVEL);
  _VISCA_append_byte(&packet, VISCA_MD_ADJUST);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_md_adjust_huelevel(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_ADJUST_HUELEVEL);
  _VISCA_append_byte(&packet, VISCA_MD_ADJUST);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_md_adjust_size(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_ADJUST_SIZE);
  _VISCA_append_byte(&packet, VISCA_MD_ADJUST);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_md_adjust_disptime(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_ADJUST_DISPTIME);
  _VISCA_append_byte(&packet, VISCA_MD_ADJUST);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_md_adjust_refmode(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_ADJUST_REFMODE);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_md_adjust_reftime(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_ADJUST_REFTIME);
  _VISCA_append_byte(&packet, VISCA_MD_ADJUST);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_md_measure_mode1_onoff(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_MEASURE_MODE_1);
  _VISCA_append_byte(&packet, VISCA_MD_ONOFF);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_md_measure_mode1(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_MEASURE_MODE_1);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_md_measure_mode2_onoff(VISCAInterface_t *iface, VISCACamera_t *camera)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_MEASURE_MODE_2);
  _VISCA_append_byte(&packet, VISCA_MD_ONOFF);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_set_md_measure_mode2(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t power)
{
  VISCAPacket_t packet;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_COMMAND);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_MEASURE_MODE_2);
  _VISCA_append_byte(&packet, power);

  return _VISCA_send_packet_with_reply(iface, camera, &packet);
}


unsigned int
VISCA_get_keylock(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_KEYLOCK);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_wide_con_lens(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
  _VISCA_append_byte(&packet, VISCA_WIDE_CON_LENS);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_atmd_mode(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_ATMD_MODE);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_at_mode(VISCAInterface_t *iface, VISCACamera_t *camera, UInt16_t *value)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_AT_MODE_QUERY);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *value=((iface->ibuf[2] & 0xff) << 8) + (iface->ibuf[3] & 0xff);
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_at_entry(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_AT_ENTRY);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_md_mode(VISCAInterface_t *iface, VISCACamera_t *camera, UInt16_t *value)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_MODE_QUERY);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *value=((iface->ibuf[2] & 0xff) << 8) + (iface->ibuf[3] & 0xff);
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_md_ylevel(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_ADJUST_YLEVEL);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=(iface->ibuf[3] & 0x0f);
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_md_huelevel(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_ADJUST_HUELEVEL);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=(iface->ibuf[3] & 0x0f);
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_md_size(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_ADJUST_SIZE);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=(iface->ibuf[3] & 0x0f);
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_md_disptime(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_ADJUST_DISPTIME);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=(iface->ibuf[3] & 0x0f);
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_md_refmode(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_ADJUST_REFMODE);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=iface->ibuf[2];
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_md_reftime(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *power)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_REFTIME_QUERY);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *power=(iface->ibuf[3] & 0x0f);
      return VISCA_SUCCESS;
    }
}



unsigned int
VISCA_get_at_obj_pos(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *xpos, UInt8_t *ypos, UInt8_t *status)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_AT_POSITION);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *xpos=iface->ibuf[2];
      *ypos=iface->ibuf[3];
      *status=(iface->ibuf[4] & 0x0f);
      return VISCA_SUCCESS;
    }
}


unsigned int
VISCA_get_md_obj_pos(VISCAInterface_t *iface, VISCACamera_t *camera, UInt8_t *xpos, UInt8_t *ypos, UInt8_t *status)
{
  VISCAPacket_t packet;
  unsigned int err;

  _VISCA_init_packet(&packet);
  _VISCA_append_byte(&packet, VISCA_INQUIRY);
  _VISCA_append_byte(&packet, VISCA_CATEGORY_CAMERA2);
  _VISCA_append_byte(&packet, VISCA_MD_POSITION);
  err=_VISCA_send_packet_with_reply(iface, camera, &packet);
  if (err!=VISCA_SUCCESS)
    return err;
  else
    {
      *xpos=iface->ibuf[2];
      *ypos=iface->ibuf[3];
      *status=(iface->ibuf[4] & 0x0f);
      return VISCA_SUCCESS;
    }
}
