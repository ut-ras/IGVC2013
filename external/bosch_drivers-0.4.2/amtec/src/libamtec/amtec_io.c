/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
 
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/serial.h>
#include "amtec_io.h"

//#define IO_DEBUG

int iParity(enum PARITY_TYPE par)
{
  if (par == N)
    return (IGNPAR);
  else
    return (INPCK);
}

int iSoftControl(int flowcontrol)
{
  if (flowcontrol)
    return (IXON);
  else
    return (IXOFF);
}

int cDataSize(int numbits)
{
  switch (numbits) {
  case 5:
    return (CS5);
    break;
  case 6:
    return (CS6);
    break;
  case 7:
    return (CS7);
    break;
  case 8:
    return (CS8);
    break;
  default:
    return (CS8);
    break;
  }
}

int cStopSize(int numbits)
{
  if (numbits == 2)
    return (CSTOPB);
  else
    return (0);
}

int cFlowControl(int flowcontrol)
{
  if (flowcontrol)
    return (CRTSCTS);
  else
    return (CLOCAL);
}

int cParity(enum PARITY_TYPE par)
{
  if (par != N) {
    if (par == O)
      return (PARENB | PARODD);
    else
      return (PARENB);
  } else
    return (0);
}

int cBaudrate(int baudrate)
{
  switch (baudrate) {
  case 0:
    return (B0);
    break;
  case 300:
    return (B300);
    break;
  case 600:
    return (B600);
    break;
  case 1200:
    return (B1200);
    break;
  case 2400:
    return (B2400);
    break;
  case 4800:
    return (B4800);
    break;
  case 9600:
    return (B9600);
    break;
  case 19200:
    return (B19200);
    break;
  case 38400:
    return (B38400);
    break;
  case 57600:
    return (B57600);
    break;
  case 115200:
    return (B115200);
    break;
  case 500000:
    /* to use 500k you have to change the entry of B460800 in you kernel:
     /usr/src/linux/drivers/usb/serial/ftdi_sio.h:
     ftdi_8U232AM_48MHz_b460800 = 0x0006    */
    return (B460800);
    break;
  default:
    return (B9600);
    break;
  }
}

long bytesWaiting(int sd)
{
  long available = 0;
  if (ioctl(sd, FIONREAD, &available) == 0)
    return available;
  else
    return -1;
}

void amtecDeviceSetParams(amtec_powercube_device_p dev)
{
  struct termios ctio;

  tcgetattr(dev->fd, &ctio); /* save current port settings */

  ctio.c_iflag = iSoftControl(dev->swf) | iParity(dev->parity);
  ctio.c_oflag = 0;
  ctio.c_cflag = CREAD | cFlowControl(dev->hwf || dev->swf)
                       | cParity(dev->parity)
                       | cDataSize(dev->databits)
                       | cStopSize(dev->stopbits);
  ctio.c_lflag = 0;
  ctio.c_cc[VTIME] = 0; /* inter-character timer unused */
  ctio.c_cc[VMIN] = 0; /* blocking read until 0 chars received */

  cfsetispeed(&ctio, (speed_t) cBaudrate(dev->baud));
  cfsetospeed(&ctio, (speed_t) cBaudrate(dev->baud));

  tcflush(dev->fd, TCIFLUSH);
  tcsetattr(dev->fd, TCSANOW, &ctio);
}

void amtecDeviceSetBaudrate(amtec_powercube_device_p dev, int brate)
{
  struct termios ctio;

  tcgetattr(dev->fd, &ctio); /* save current port settings */

  cfsetispeed(&ctio, (speed_t) cBaudrate(brate));
  cfsetospeed(&ctio, (speed_t) cBaudrate(brate));

  tcflush(dev->fd, TCIFLUSH);
  tcsetattr(dev->fd, TCSANOW, &ctio);
}

int amtecDeviceConnectPort(amtec_powercube_device_p dev)
{
  fprintf(stderr, "\nset device:\n");
  fprintf(stderr, "   port   = %s\n", dev->ttyport);
  fprintf(stderr, "   baud   = %d\n", dev->baud);
  fprintf(stderr, "   params = %d%s%d\n", dev->databits,
                                          dev->parity == N ? "N": dev->parity == O ? "O" : "E",
                                          dev->stopbits);
  if ((dev->fd = open((dev->ttyport), (O_RDWR | O_NOCTTY), 0)) < 0)
    return (-1);
  amtecDeviceSetParams(dev);
  return (dev->fd);
}

int waitForETX(int fd, unsigned char *buf, int *len)
{
  static int pos, loop, val;
#ifdef IO_DEBUG
  int i;
#endif
  pos = 0;
  loop = 0;
  while (loop < MAX_NUM_LOOPS) {
    val = bytesWaiting(fd);
    if (val > 0) {
      if(pos + val >= MAX_ACMD_SIZE) return (0);
      read(fd, &(buf[pos]), val);
#ifdef IO_DEBUG
      for(i=0;i<val;i++)
      fprintf(stderr, "[0x%s%x]", buf[pos+i]<16?"0":"", buf[pos+i]);
#endif
      if (buf[pos + val - 1] == B_ETX) {
        *len = pos + val - 1;
#ifdef IO_DEBUG
        fprintf(stderr, "\n");
#endif
        return (1);
      }
      pos += val;
    } else {
      usleep(1000);
      loop++;
    }
  }
#ifdef IO_DEBUG
  fprintf(stderr, "\n");
#endif
  return (0);
}

int waitForAnswer(int fd, unsigned char *buf, int *len)
{
  int loop = 0;
  *len = 0;
#ifdef IO_DEBUG
  fprintf(stderr, "<--- ");
#endif
  while (loop < MAX_NUM_LOOPS) {
    if (bytesWaiting(fd)) {
      read(fd, &(buf[0]), 1);
#ifdef IO_DEBUG
      fprintf(stderr, "(0x%s%x)", buf[0]<16?"0":"", buf[0]);
#endif
      if (buf[0] == B_STX) {
        return (waitForETX(fd, buf, len));
      }
    } else {
      usleep(1000);
      loop++;
    }
  }
#ifdef IO_DEBUG
  fprintf(stderr, "\n");
#endif
  return (0);
}

int writeData(int fd, unsigned char *buf, int nChars)
{
  int written = 0;
  while (nChars > 0) {
    written = write(fd, buf, nChars);
    if (written < 0) {
      return 0;
    } else {
      nChars -= written;
      buf += written;
    }
    usleep(1000);
  }
  return 1;
}

int amtecSendCommand(amtec_powercube_device_p dev, int id, unsigned char *cmd, int len)
{
  static int i, ctr, add;
  static unsigned char rcmd[MAX_ACMD_SIZE];
  static unsigned char bcc;
  static unsigned char umnr;
  static unsigned char lmnr;

#ifdef IO_DEBUG
  fprintf(stderr, "\n---> ");
  for(i=0;i<len;i++) {
    fprintf(stderr, "(0x%s%x)", cmd[i]<16?"0":"", cmd[i]);
  }
  fprintf(stderr, "\n");
#endif

  add = 0;
  lmnr = id & 7;
  lmnr = lmnr << 5;
  umnr = id >> 3;
  umnr = umnr | 4;
  for (i = 0; i < len; i++) {
    if ((cmd[i] == 0x02) || (cmd[i] == 0x03) || (cmd[i] == 0x10)) {
      add++;
    }
  }
  lmnr = lmnr + len;
  rcmd[0] = B_STX;
  rcmd[1] = umnr;
  rcmd[2] = lmnr;
  ctr = 3;
  for (i = 0; i < len; i++) {
    switch (cmd[i]) {
    case 0x02:
      rcmd[ctr] = 0x10;
      rcmd[++ctr] = 0x82;
      break;
    case 0x03:
      rcmd[ctr] = 0x10;
      rcmd[++ctr] = 0x83;
      break;
    case 0x10:
      rcmd[ctr] = 0x10;
      rcmd[++ctr] = 0x90;
      break;
    default:
      rcmd[ctr] = cmd[i];
    }
    ctr++;
  }
  bcc = id;
  for (i = 0; i < len; i++) {
    bcc += cmd[i];
  }
  bcc = bcc + (bcc >> 8);
  switch (bcc) {
  case 0x02:
    rcmd[ctr++] = 0x10;
    rcmd[ctr++] = 0x82;
    break;
  case 0x03:
    rcmd[ctr++] = 0x10;
    rcmd[ctr++] = 0x83;
    break;
  case 0x10:
    rcmd[ctr++] = 0x10;
    rcmd[ctr++] = 0x90;
    break;
  default:
    rcmd[ctr++] = bcc;
  }
  rcmd[ctr++] = B_ETX;

#ifdef IO_DEBUG
  fprintf(stderr, "-*-> ");
  for(i=0;i<ctr;i++) {
    fprintf(stderr, "(0x%s%x)", rcmd[i]<16?"0":"", rcmd[i]);
  }
  fprintf(stderr, "\n");
#endif

  if (writeData(dev->fd, rcmd, ctr)) {
    return (1);
  } else {
    return (0);
  }
}

void convertBuffer(unsigned char *cmd, int *len)
{
  int i, j;
  for (i = 0; i < *len; i++) {
    if (cmd[i] == B_DLE) {
      switch (cmd[i + 1]) {
      case 0x82:
        cmd[i] = 0x02;
        for (j = i + 2; j < *len; j++)
          cmd[j - 1] = cmd[j];
        (*len)--;
        break;
      case 0x83:
        cmd[i] = 0x03;
        for (j = i + 2; j < *len; j++)
          cmd[j - 1] = cmd[j];
        (*len)--;
        break;
      case 0x90:
        cmd[i] = 0x10;
        for (j = i + 2; j < *len; j++)
          cmd[j - 1] = cmd[j];
        (*len)--;
        break;
      }
    }
  }
}

int amtecGetAnswer(amtec_powercube_device_p dev, unsigned char *cmd, int *len)
{
#ifdef IO_DEBUG
  int i;
#endif
  if (waitForAnswer(dev->fd, cmd, len)) {
#ifdef IO_DEBUG
    fprintf(stderr, "<=== ");
    for(i=0;i<*len;i++)
    fprintf(stderr, "[0x%s%x]", cmd[i]<16?"0":"", cmd[i]);
    fprintf(stderr, "\n");
#endif
    convertBuffer(cmd, len);
#ifdef IO_DEBUG
    fprintf(stderr, "<=p= ");
    for(i=0;i<*len;i++)
    fprintf(stderr, "[0x%s%x]", cmd[i]<16?"0":"", cmd[i]);
    fprintf(stderr, "\n");
#endif
    return (1);
  } else {
    return (0);
  }
}
