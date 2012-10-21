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
 
#include "amtec_io.h"
#include "amtec_conversion.h"
#include "amtec_commands.h"

/**********************************************************/
/*               POWERCUBE COMMANDS */
/**********************************************************/

void cmdReset(amtec_powercube_device_p dev, int id)
{
  unsigned char cmd[1];
  cmd[0] = CodeReset;
  amtecSendCommand(dev, id, cmd, 1);
}

void cmdHome(amtec_powercube_device_p dev, int id)
{
  unsigned char cmd[1];
  cmd[0] = CodeHome;
  amtecSendCommand(dev, id, cmd, 1);
}

void cmdHalt(amtec_powercube_device_p dev, int id)
{
  unsigned char cmd[1];
  cmd[0] = CodeHalt;
  amtecSendCommand(dev, id, cmd, 1);
}

void cmdRecalcPIDParam(amtec_powercube_device_p dev, int id)
{
  unsigned char cmd[1];
  cmd[0] = CodeRecalcPIDParam;
  amtecSendCommand(dev, id, cmd, 1);
}

void cmdResetTime(amtec_powercube_device_p dev, int id)
{
  unsigned char cmd[1];
  cmd[0] = CodeResetTime;
  amtecSendCommand(dev, id, cmd, 1);
}

void cmdGetExtended(amtec_powercube_device_p dev, int id, unsigned char code)
{
  unsigned char cmd[2];
  cmd[0] = CodeGetExtended;
  cmd[1] = code;
  amtecSendCommand(dev, id, cmd, 2);
}

void cmdSetExtendedInt8(amtec_powercube_device_p dev, int id, unsigned char code, char val)
{
  unsigned char cmd[3];
  cmd[0] = CodeSetExtended;
  cmd[1] = code;
  convertInt8(val, &(cmd[2]));
  amtecSendCommand(dev, id, cmd, 3);
}

void cmdSetExtendedUInt8(amtec_powercube_device_p dev, int id, unsigned char code, unsigned char val)
{
  unsigned char cmd[3];
  cmd[0] = CodeSetExtended;
  cmd[1] = code;
  convertUInt8(val, &(cmd[2]));
  amtecSendCommand(dev, id, cmd, 3);
}

void cmdSetExtendedInt16(amtec_powercube_device_p dev, int id, unsigned char code, short val)
{
  unsigned char cmd[4];
  cmd[0] = CodeSetExtended;
  cmd[1] = code;
  convertInt16(val, &(cmd[2]));
  amtecSendCommand(dev, id, cmd, 4);
}

void cmdSetExtendedUInt16(amtec_powercube_device_p dev, int id, unsigned char code, unsigned short val)
{
  unsigned char cmd[4];
  cmd[0] = CodeSetExtended;
  cmd[1] = code;
  convertUInt16(val, &(cmd[2]));
  amtecSendCommand(dev, id, cmd, 3);
}

void cmdSetExtendedInt32(amtec_powercube_device_p dev, int id, unsigned char code, int val)
{
  unsigned char cmd[6];
  cmd[0] = CodeSetExtended;
  cmd[1] = code;
  convertInt32(val, &(cmd[2]));
  amtecSendCommand(dev, id, cmd, 3);
}

void cmdSetExtendedUInt32(amtec_powercube_device_p dev, int id, unsigned char code, unsigned int val)
{
  unsigned char cmd[6];
  cmd[0] = CodeSetExtended;
  cmd[1] = code;
  convertUInt32(val, &(cmd[2]));
  amtecSendCommand(dev, id, cmd, 6);
}

void cmdSetExtendedFloat(amtec_powercube_device_p dev, int id, unsigned char code, float val)
{
  unsigned char cmd[6];
  cmd[0] = CodeSetExtended;
  cmd[1] = code;
  convertFloat(val, &(cmd[2]));
  amtecSendCommand(dev, id, cmd, 6);
}

void cmdSetMotionInt16(amtec_powercube_device_p dev, int id, unsigned char code, short val)
{
  unsigned char cmd[4];
  cmd[0] = CodeSetMotion;
  cmd[1] = code;
  convertInt16(val, &(cmd[2]));
  amtecSendCommand(dev, id, cmd, 4);
}

void cmdSetMotionInt32(amtec_powercube_device_p dev, int id, unsigned char code, int val)
{
  unsigned char cmd[6];
  cmd[0] = CodeSetMotion;
  cmd[1] = code;
  convertInt32(val, &(cmd[2]));
  amtecSendCommand(dev, id, cmd, 6);
}

void cmdSetMotionFloat(amtec_powercube_device_p dev, int id, unsigned char code, float val)
{
  unsigned char cmd[6];
  cmd[0] = CodeSetMotion;
  cmd[1] = code;
  convertFloat(val, &(cmd[2]));
  amtecSendCommand(dev, id, cmd, 6);
}

void cmdSetFStepMotion(amtec_powercube_device_p dev, int id, unsigned char code, float val1, unsigned short val2)
{
  unsigned char cmd[8];
  cmd[0] = CodeSetMotion;
  cmd[1] = code;
  convertFloat(val1, &(cmd[2]));
  convertUInt16(val2, &(cmd[6]));
  amtecSendCommand(dev, id, cmd, 8);
}

void cmdSetIStepMotion(amtec_powercube_device_p dev, int id, unsigned char code, int val1, unsigned short val2)
{
  unsigned char cmd[8];
  cmd[0] = CodeSetMotion;
  cmd[1] = code;
  convertInt32(val1, &(cmd[2]));
  convertUInt16(val2, &(cmd[6]));
  amtecSendCommand(dev, id, cmd, 8);
}

void cmdSetMotionLoop(amtec_powercube_device_p dev, int id, unsigned char code, float pos, unsigned short period)
{
  unsigned char cmd[8];
  cmd[0] = CodeSetMotion;
  cmd[1] = code;
  convertFloat(pos, &(cmd[2]));
  convertUInt16(period, &(cmd[6]));
  amtecSendCommand(dev, id, cmd, 8);
}

/**********************************************************/
/*      POWERCUBE SPECIAL SET EXTENDED COMMANDS */
/**********************************************************/

void cmdSetMinPos(amtec_powercube_device_p dev, int id, float pos)
{
  cmdSetExtendedFloat(dev, id, CodeMinPos, pos);
}

void cmdSetMaxPos(amtec_powercube_device_p dev, int id, float pos)
{
  cmdSetExtendedFloat(dev, id, CodeMaxPos, pos);
}

void cmdSetMaxVel(amtec_powercube_device_p dev, int id, float vel)
{
  cmdSetExtendedFloat(dev, id, CodeMaxVel, vel);
}

void cmdSetMaxAcc(amtec_powercube_device_p dev, int id, float acc)
{
  cmdSetExtendedFloat(dev, id, CodeMaxAcc, acc);
}

void cmdSetMaxCur(amtec_powercube_device_p dev, int id, float cur)
{
  cmdSetExtendedFloat(dev, id, CodeMaxCur, cur);
}

void cmdSetTargetVel(amtec_powercube_device_p dev, int id, float vel)
{
  cmdSetExtendedFloat(dev, id, CodeTargetVel, vel);
}

void cmdSetTargetAcc(amtec_powercube_device_p dev, int id, float acc)
{
  cmdSetExtendedFloat(dev, id, CodeTargetAcc, acc);
}

void cmdSetConfig(amtec_powercube_device_p dev, int id, unsigned int conf)
{
  cmdSetExtendedUInt32(dev, id, CodeConfig, conf);
}

void cmdSetActC0(amtec_powercube_device_p dev, int id, short c0)
{
  cmdSetExtendedInt16(dev, id, CodeActC0, c0);
}

void cmdSetActDamp(amtec_powercube_device_p dev, int id, short damp)
{
  cmdSetExtendedInt16(dev, id, CodeActDamp, damp);
}

void cmdSetActA0(amtec_powercube_device_p dev, int id, short a0)
{
  cmdSetExtendedInt16(dev, id, CodeActA0, a0);
}

/**********************************************************/
/*                    CODE CHECK */
/**********************************************************/
int getCorrectResetCode(unsigned char *buf, int len)
{
  if(len<3 || buf[2]!=CodeReset)
    return(0);
  return(1);
}

int getCorrectHomeCode(unsigned char *buf, int len)
{
  if(len<3 || buf[2]!=CodeHome)
    return(0);
  return(1);
}

int getCorrectHaltCode(unsigned char *buf, int len)
{
  if(len<3 || buf[2]!=CodeHalt)
    return(0);
  return(1);
}

int getCorrectRecalcPIDParam(unsigned char *buf, int len)
{
  if(len<3 || buf[2]!=CodeRecalcPIDParam)
    return(0);
  return(1);
}

int getCorrectResetTime(unsigned char *buf, int len)
{
  if(len<3 || buf[2]!=CodeResetTime)
    return(0);
  return(1);
}

int getCorrectSetExtCode(unsigned char *buf, int len, unsigned char code)
{
  if(len<4 || buf[2]!=CodeSetExtended || buf[3]!=code)
    return(0);
  return(1);
}

int getCorrectSetMotionCode(unsigned char *buf, int len, unsigned char code)
{
  if(len<4 || buf[2]!=CodeSetMotion || buf[3]!=code)
    return(0);
  return(1);
}

int getCorrectGetExtInt8(unsigned char *buf, int len, unsigned char code, char *val)
{
  if(len<4 || buf[2]!=CodeGetExtended || buf[3]!=code)
    return(0);
  *val = convertBytes2Int8(&(buf[4]));
  return(1);
}

int getCorrectGetExtUInt8(unsigned char *buf, int len, unsigned char code, unsigned char *val)
{
  if(len<4 || buf[2]!=CodeGetExtended || buf[3]!=code)
    return(0);
  *val = convertBytes2UInt8(&(buf[4]));
  return(1);
}

int getCorrectGetExtInt16(unsigned char *buf, int len, unsigned char code, short *val)
{
  if(len<4 || buf[2]!=CodeGetExtended || buf[3]!=code)
    return(0);
  *val = convertBytes2Int16(&(buf[4]));
  return(1);
}

int getCorrectGetExtUInt16(unsigned char *buf, int len, unsigned char code, unsigned short *val)
{
  if(len<4 || buf[2]!=CodeGetExtended || buf[3]!=code)
    return(0);
  *val = convertBytes2UInt16(&(buf[4]));
  return(1);
}

int getCorrectGetExtInt32(unsigned char *buf, int len, unsigned char code, int *val)
{
  if(len<4 || buf[2]!=CodeGetExtended || buf[3]!=code)
    return(0);
  *val = convertBytes2Int32(&(buf[4]));
  return(1);
}

int getCorrectGetExtUInt32(unsigned char *buf, int len, unsigned char code, unsigned int *val)
{
  if(len<4 || buf[2]!=CodeGetExtended || buf[3]!=code)
    return(0);
  *val = convertBytes2UInt32(&(buf[4]));
  return(1);
}

int getCorrectGetExtFloat(unsigned char *buf, int len, unsigned char code, float *val)
{
  if(len<4 || buf[2]!=CodeGetExtended || buf[3]!=code)
    return(0);
  *val = convertBytes2Float(&(buf[4]));
  return(1);
}

/**********************************************************/
/*               POWERCUBE GET INFORMATION */
/**********************************************************/

char amtecGetInt8(amtec_powercube_device_p dev, int id, unsigned char code)
{
  static int len;
  static unsigned char buf[MAX_ACMD_SIZE];
  static char val;
  do {
    cmdGetExtended(dev, id, code);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectGetExtInt8(buf, len, code, &val));
  return(val);
}

unsigned char amtecGetUInt8(amtec_powercube_device_p dev, int id, unsigned char code)
{
  static int len;
  static unsigned char buf[MAX_ACMD_SIZE];
  static unsigned char val;
  do {
    cmdGetExtended(dev, id, code);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectGetExtUInt8(buf, len, code, &val));
  return(val);
}

short amtecGetInt16(amtec_powercube_device_p dev, int id, unsigned char code)
{
  static int len;
  static unsigned char buf[MAX_ACMD_SIZE];
  static short val;
  do {
    cmdGetExtended(dev, id, code);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectGetExtInt16(buf, len, code, &val));
  return(val);
}

unsigned short amtecGetUInt16(amtec_powercube_device_p dev, int id, unsigned char code)
{
  static int len;
  static unsigned char buf[MAX_ACMD_SIZE];
  static unsigned short val;
  do {
    cmdGetExtended(dev, id, code);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectGetExtUInt16(buf, len, code, &val));
  return(val);
}

int amtecGetInt32(amtec_powercube_device_p dev, int id, unsigned char code)
{
  static int len;
  static unsigned char buf[MAX_ACMD_SIZE];
  static int val;
  do {
    cmdGetExtended(dev, id, code);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectGetExtInt32(buf, len, code,  &val));
  return(val);
}

unsigned int amtecGetUInt32(amtec_powercube_device_p dev, int id, unsigned char code)
{
  static int len;
  static unsigned char buf[MAX_ACMD_SIZE];
  static unsigned int val;
  do {
    cmdGetExtended(dev, id, code);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectGetExtUInt32(buf, len, code,  &val));
  return(val);
}

float amtecGetFloat(amtec_powercube_device_p dev, int id, unsigned char code)
{
  int len;
  float val;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdGetExtended(dev, id, code);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectGetExtFloat(buf, len, code,  &val));
  return(val);
}

/**********************************************************/
/*        POWERCUBE GENERAL COMMANDS                      */
/**********************************************************/
int amtecReset(amtec_powercube_device_p dev, int id)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdReset(dev, id);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectResetCode(buf, len));
  return 1;
}

int amtecHome(amtec_powercube_device_p dev, int id)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdHome(dev, id);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectHomeCode(buf, len));
  return 1;
}

int amtecHalt(amtec_powercube_device_p dev, int id)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdHalt(dev, id);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectHaltCode(buf, len));
  return 1;
}

int amtecRecalcPIDParam(amtec_powercube_device_p dev, int id)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdRecalcPIDParam(dev, id);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectRecalcPIDParam(buf, len));
  return 1;
}

int amtecResetTime(amtec_powercube_device_p dev, int id)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdResetTime(dev, id);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectResetTime(buf, len));
  return 1;
}

/**********************************************************/
/*        POWERCUBE MOTION COMMANDS                       */
/**********************************************************/
int amtecMotionFRamp(amtec_powercube_device_p dev, int id, float position)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetMotionFloat(dev, id, FRAMP_MODE, position);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetMotionCode(buf, len, FRAMP_MODE));
  return 1;
}

int amtecMotionFStep(amtec_powercube_device_p dev, int id, float position, unsigned short time)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetFStepMotion(dev, id, FSTEP_MODE, position, time);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetMotionCode(buf, len, FSTEP_MODE));
  return 1;
}

int amtecMotionFVel(amtec_powercube_device_p dev, int id, float velocity)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetMotionFloat(dev, id, FVEL_MODE, velocity);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetMotionCode(buf, len, FVEL_MODE));
  return 1;
}

int amtecMotionFCur(amtec_powercube_device_p dev, int id, float current)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetMotionFloat(dev, id, FCUR_MODE, current);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetMotionCode(buf, len, FCUR_MODE));
  return 1;
}

int amtecMotionIRamp(amtec_powercube_device_p dev, int id, int ticks)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetMotionInt32(dev, id, IRAMP_MODE, ticks);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetMotionCode(buf, len, IRAMP_MODE));
  return 1;
}

int amtecMotionIStep(amtec_powercube_device_p dev, int id, int ticks, unsigned short time)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetIStepMotion(dev, id, ISTEP_MODE, ticks, time);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetMotionCode(buf, len, ISTEP_MODE));
  return 1;
}

int amtecMotionIVel(amtec_powercube_device_p dev, int id, int ticks)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetMotionInt32(dev, id, IVEL_MODE, ticks);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetMotionCode(buf, len, IVEL_MODE));
  return 1;
}

int amtecMotionICur(amtec_powercube_device_p dev, int id, short digits)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetMotionInt16(dev, id, ICUR_MODE, digits);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetMotionCode(buf, len, ICUR_MODE));
  return 1;
}

int amtecMotionFCosLoop(amtec_powercube_device_p dev, int id, float val, unsigned short period)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetMotionLoop(dev, id, FCOSLOOP, val, period);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetMotionCode(buf, len, FCOSLOOP));
  return 1;
}

int amtecMotionFRampLoop(amtec_powercube_device_p dev, int id, float val)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetMotionFloat(dev, id, FCUR_MODE, val);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetMotionCode(buf, len, FCUR_MODE));
}

/**********************************************************/
/*        POWERCUBE SPECIAL GET INFORMATION               */
/**********************************************************/

float amtecGetDefHomeOffset(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeDefHomeOffset));
}

float amtecGetDefGearRatio(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeDefGearRatio));
}

float amtecGetDefLinRatio(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeDefLinRatio));
}

float amtecGetDefMinPos(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeDefMinPos));
}

float amtecGetDefMaxPos(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeDefMaxPos));
}

float amtecGetDefMaxDeltaPos(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeDefMaxDeltaPos));
}

float amtecGetDefMaxDeltaVel(amtec_powercube_device_p dev, int id)
{
  return 0;//(amtecGetFloat(dev, id, CodeDefMaxDeltaVel));
}

float amtecGetDefTorqueRatio(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeDefTorqueRatio));
}

float amtecGetDefCurRatio(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeDefCurRatio));
}

float amtecGetDefMaxVel(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeDefMaxVel));
}

float amtecGetDefMaxAcc(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeDefMaxAcc));
}

float amtecGetDefMaxCur(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeDefMaxCur));
}

float amtecGetDefHomeVel(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeDefHomeVel));
}

float amtecGetDefHomeAcc(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeDefHomeAcc));
}

unsigned int amtecGetDefCubeSerial(amtec_powercube_device_p dev, int id)
{
  return(amtecGetUInt32(dev, id, CodeDefCubeSerial));
}

unsigned int amtecGetDefPulsesPerTurn(amtec_powercube_device_p dev, int id)
{
  return(amtecGetUInt32(dev, id, CodeDefPulsesPerTurn));
}

unsigned int amtecGetDefConfig(amtec_powercube_device_p dev, int id)
{
  return(amtecGetUInt32(dev, id, CodeDefConfig));
}

unsigned short amtecGetDefCubeVersion(amtec_powercube_device_p dev, int id)
{
  return(amtecGetUInt16(dev, id, CodeDefCubeVersion));
}

unsigned short amtecGetDefServiceInterval(amtec_powercube_device_p dev, int id)
{
  return 0;//(amtecGetUInt16(dev, id, CodeDefServiceInterval));
}

unsigned short amtecGetDefBrakeTimeOut(amtec_powercube_device_p dev, int id)
{
  return(amtecGetUInt16(dev, id, CodeDefBrakeTimeOut));
}

unsigned char amtecGetDefAddress(amtec_powercube_device_p dev, int id)
{
  return(amtecGetUInt8(dev, id, CodeDefAddress));
}

unsigned char amtecGetDefPrimBaud(amtec_powercube_device_p dev, int id)
{
  return(amtecGetUInt8(dev, id, CodeDefPrimBaud));
}

unsigned char amtecGetDefScndBaud(amtec_powercube_device_p dev, int id)
{
  return(amtecGetUInt8(dev, id, CodeDefScndBaud));
}

int amtecGetPosCount(amtec_powercube_device_p dev, int id)
{
  return(amtecGetUInt8(dev, id, CodePosCount));
}

int amtecGetRefPosCount(amtec_powercube_device_p dev, int id)
{
  return(amtecGetUInt8(dev, id, CodeRefPosCount));
}

unsigned int amtecGetDioSetup(amtec_powercube_device_p dev, int id)
{
  return(amtecGetUInt32(dev, id, CodeDioSetup));
}

unsigned int amtecGetCubeState(amtec_powercube_device_p dev, int id)
{
  return(amtecGetUInt32(dev, id, CodeCubeState));
}

unsigned int amtecGetTargetPosInc(amtec_powercube_device_p dev, int id)
{
  return(amtecGetUInt32(dev, id, CodeTargetPosInc));
}

unsigned int amtecGetTargetVelInc(amtec_powercube_device_p dev, int id)
{
  return(amtecGetUInt32(dev, id, CodeTargetVelInc));
}

unsigned int amtecGetTargetAccInc(amtec_powercube_device_p dev, int id)
{
  return(amtecGetUInt32(dev, id, CodeTargetAccInc));
}

unsigned int amtecGetStepInc(amtec_powercube_device_p dev, int id)
{
  return(amtecGetUInt32(dev, id, CodeStepInc));
}


float amtecGetMinPos(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeMinPos));
}

float amtecGetMaxPos(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeMaxPos));
}

float amtecGetMaxVel(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeMaxVel));
}

float amtecGetMaxAcc(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeMaxAcc));
}

float amtecGetTargetVel(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeTargetVel));
}

float amtecGetTargetAcc(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeTargetAcc));
}

float amtecGetMaxCur(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeMaxCur));
}

float amtecGetActPos(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeActPos));
}

unsigned int amtecGetConfig(amtec_powercube_device_p dev __attribute__ ((unused)),
			     int id __attribute__ ((unused)))
{
  return(0);
}

float amtecGetActVel(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeActVel));
}

float amtecGetCur(amtec_powercube_device_p dev, int id)
{
  return(amtecGetFloat(dev, id, CodeCur));
}

short amtecGetActC0(amtec_powercube_device_p dev, int id)
{
  return amtecGetInt16(dev, id, CodeActC0);
}

short amtecGetActDamp(amtec_powercube_device_p dev, int id)
{
  return amtecGetInt16(dev, id, CodeActDamp);
}

short amtecGetActA0(amtec_powercube_device_p dev, int id)
{
  return amtecGetInt16(dev, id, CodeActA0);
}

/**********************************************************/
/*              POWERCUBE SET VALUES */
/**********************************************************/

void amtecSetMinPos(amtec_powercube_device_p dev, int id, float val)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetMinPos(dev, id, val);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetExtCode(buf, len, CodeMinPos));
}

void amtecSetMaxPos(amtec_powercube_device_p dev, int id, float val)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetMaxPos(dev, id, val);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetExtCode(buf, len, CodeMaxPos));
}

void amtecSetMaxAcc(amtec_powercube_device_p dev, int id, float val)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetMaxAcc(dev, id, val);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetExtCode(buf, len, CodeMaxAcc));
}

void amtecSetMaxVel(amtec_powercube_device_p dev, int id, float val)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetMaxVel(dev, id, val);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetExtCode(buf, len, CodeMaxVel));
}

void amtecSetTargetAcc(amtec_powercube_device_p dev, int id, float val)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetTargetAcc(dev, id, val);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetExtCode(buf, len, CodeTargetAcc));
}

void amtecSetTargetVel(amtec_powercube_device_p dev, int id, float val)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetTargetVel(dev, id, val);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetExtCode(buf, len, CodeTargetVel));
}

void amtecSetMaxCur(amtec_powercube_device_p dev, int id, float val)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetMaxCur(dev, id, val);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetExtCode(buf, len, CodeMaxCur));
}

void amtecSetConfig(amtec_powercube_device_p dev, int id, unsigned int conf)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetConfig(dev, id, conf);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetExtCode(buf, len, CodeConfig));
}


void amtecSetActC0(amtec_powercube_device_p dev, int id, short c0)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetActC0(dev, id, c0);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetExtCode(buf, len, CodeActC0));
}

void amtecSetActDamp(amtec_powercube_device_p dev, int id, short damp)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetActDamp(dev, id, damp);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetExtCode(buf, len, CodeActDamp));
}

void amtecSetActA0(amtec_powercube_device_p dev, int id, short a0)
{
  int len;
  unsigned char buf[MAX_ACMD_SIZE];
  do {
    cmdSetActA0(dev, id, a0);
    amtecGetAnswer(dev, buf, &len);
  } while(!getCorrectSetExtCode(buf, len, CodeActA0));
}
