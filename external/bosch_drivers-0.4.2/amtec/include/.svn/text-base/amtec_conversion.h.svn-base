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

#ifndef AMTEC_CONVERSION_H
#define AMTEC_CONVERSION_H

#include "amtec_base.h"

#ifdef __cplusplus
extern "C" {
#endif

int            ParamBytes(int type);

/* CONVERSION BYTES -> NUM */
float          convertBytes2Float(unsigned char *bytes);
char           convertBytes2Int8(unsigned char *bytes);
short          convertBytes2Int16(unsigned char *bytes);
int            convertBytes2Int32(unsigned char *bytes);
unsigned char  convertBytes2UInt8(unsigned char *bytes);
unsigned short convertBytes2UInt16(unsigned char *bytes);
unsigned int   convertBytes2UInt32(unsigned char *bytes);

/* CONVERSION NUM -> BYTES */
void           convertFloat(float f, unsigned char *bytes);
void           convertInt8(char c, unsigned char *bytes);
void           convertInt16(short s, unsigned char *bytes);
void           convertInt32(int i, unsigned char *bytes);
void           convertUInt8(unsigned char c, unsigned char *bytes);
void           convertUInt16(unsigned short s, unsigned char *bytes);
void           convertUInt32(unsigned int i, unsigned char *bytes);

#ifdef __cplusplus
}
#endif

#endif
