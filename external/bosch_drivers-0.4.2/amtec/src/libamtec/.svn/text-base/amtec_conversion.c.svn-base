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

#include "amtec_conversion.h"

int ParamBytes(int type)
{
  switch(type) {
  case INT8:
    return(1);
    break;
  case UINT8:
    return(1);
    break;
  case INT16:
    return(2);
    break;
  case UINT16:
    return(2);
    break;
  case INT32:
    return(4);
    break;
  case UINT32:
    return(4);
    break;
  case FLOAT:
    return(4);
    break;
  }
  return(0);
}

/* CONVERSION BYTES -> NUM */

char convertBytes2Int8(unsigned char *bytes)
{
  char i;
  memcpy(&i, bytes, sizeof(char));
  return(i);
}

unsigned char convertBytes2UInt8(unsigned char *bytes)
{
  unsigned char i;
  memcpy(&i, bytes, sizeof(unsigned char));
  return(i);
}

short convertBytes2Int16(unsigned char *bytes)
{
  short i;
  memcpy(&i, bytes, sizeof(short));
  return(i);
}

unsigned short convertBytes2UInt16(unsigned char *bytes)
{
  unsigned short i;
  memcpy(&i, bytes, sizeof(unsigned short));
  return(i);
}

int convertBytes2Int32(unsigned char *bytes)
{
  int i;
  memcpy(&i, bytes, sizeof(int));
  return(i);
}

unsigned int convertBytes2UInt32(unsigned char *bytes)
{
  unsigned int i;
  memcpy(&i, bytes, sizeof(unsigned int));
  return(i);
}

float convertBytes2Float(unsigned char *bytes)
{
  float f;
  memcpy(&f, bytes, sizeof(float));
  return(f);
}

/* CONVERSION NUM -> BYTES */

void convertInt8(char i, unsigned char *bytes)
{
  memcpy(bytes, &i, sizeof(char));
}

void convertUInt8(unsigned char i, unsigned char *bytes)
{
  memcpy(bytes, &i, sizeof(unsigned char));
}

void convertInt16(short i, unsigned char *bytes)
{
  memcpy(bytes, &i, sizeof(short));
}

void convertUInt16(unsigned short i, unsigned char *bytes)
{
  memcpy(bytes, &i, sizeof(unsigned short));
}

void convertInt32(int i, unsigned char *bytes)
{
  memcpy(bytes, &i, sizeof(int));
}

void convertUInt32(unsigned int i, unsigned char *bytes)
{
  memcpy(bytes, &i, sizeof(unsigned int));
}

void convertFloat(float f, unsigned char *bytes)
{
  memcpy(bytes, &f, sizeof(float));
}
