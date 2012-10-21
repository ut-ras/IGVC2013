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

#include "amtec_base.h"

///Conversion factor for degrees to radians
#define D2R       0.0174532925199432957692
///Conversion factor for radians to degrees
#define R2D       57.295779513082320876

#define AMTEC_DEFAULT_PAN_ID          14
#define AMTEC_DEFAULT_PAN_MIN_POS   -200.0
#define AMTEC_DEFAULT_PAN_MAX_POS    200.0
#define AMTEC_DEFAULT_PAN_MAX_VEL    0.5
#define AMTEC_DEFAULT_PAN_MAX_ACC    0.4
#define AMTEC_DEFAULT_PAN_MAX_CUR    4.0
#define AMTEC_DEFAULT_PAN_TARGET_VEL 0.5
#define AMTEC_DEFAULT_PAN_TARGET_ACC 0.4

#define AMTEC_DEFAULT_TILT_ID         13
#define AMTEC_DEFAULT_TILT_MIN_POS   -90.0
#define AMTEC_DEFAULT_TILT_MAX_POS    90.0
#define AMTEC_DEFAULT_TILT_MAX_VEL    0.3
#define AMTEC_DEFAULT_TILT_MAX_ACC    0.2
#define AMTEC_DEFAULT_TILT_MAX_CUR    4.0
#define AMTEC_DEFAULT_TILT_TARGET_VEL 0.3
#define AMTEC_DEFAULT_TILT_TARGET_ACC 0.2


#define AMTEC_DEFAULT_PAN_ACC          0.4
#define AMTEC_DEFAULT_PAN_VEL          0.5

#define AMTEC_DEFAULT_TILT_ACC         0.2
#define AMTEC_DEFAULT_TILT_VEL         0.3

void amtecInitializeDevice(amtec_powercube_device_p p)
{
  strncpy(p->ttyport, "/dev/ttyUSB3", MAX_NAME_LENGTH);
  p->baud = 38400;
  p->parity = N;
  p->fd = -1;
  p->databits = 8;
  p->stopbits = 1;
  p->hwf = 0;
  p->swf = 0;
}

void amtecInitializePanSettings(amtec_powercube_setting_p p)
{
  p->MinPos = AMTEC_DEFAULT_PAN_MIN_POS;
  p->MaxPos = AMTEC_DEFAULT_PAN_MAX_POS;
  p->MaxVel = AMTEC_DEFAULT_PAN_MAX_VEL;
  p->MaxAcc = AMTEC_DEFAULT_PAN_MAX_ACC;
  p->MaxCur = AMTEC_DEFAULT_PAN_MAX_CUR;
  p->TargetVel = AMTEC_DEFAULT_PAN_TARGET_VEL;
  p->TargetAcc = AMTEC_DEFAULT_PAN_TARGET_ACC;
}

void amtecInitializeTiltSettings(amtec_powercube_setting_p p)
{
  p->MinPos = AMTEC_DEFAULT_TILT_MIN_POS;
  p->MaxPos = AMTEC_DEFAULT_TILT_MAX_POS;
  p->MaxVel = AMTEC_DEFAULT_TILT_MAX_VEL;
  p->MaxAcc = AMTEC_DEFAULT_TILT_MAX_ACC;
  p->MaxCur = AMTEC_DEFAULT_TILT_MAX_CUR;
  p->TargetVel = AMTEC_DEFAULT_TILT_TARGET_VEL;
  p->TargetAcc = AMTEC_DEFAULT_TILT_TARGET_ACC;
}

void amtecInitializeParams(amtec_powercube_params_p p)
{
  p->id = -1;
  p->DefHomeOffset = -1;
  p->DefMinPos = -1;
  p->DefMaxPos = -1;
  p->DefMaxVel = -1;
  p->DefMaxAcc = -1;
  p->DefMaxCur = -1;
  p->DefCubeSerial = -1;
  p->DefConfig = -1;
  p->DefCubeVersion = -1;
  p->DefScndBaud = -1;
  p->ActPos = -1;
  p->ActVel = -1;
  p->MinPos = -1;
  p->MaxPos = -1;
  p->MaxVel = -1;
  p->MaxAcc = -1;
  p->MaxCur = -1;
  p->Config = -1;
  p->UseBreak = -1;

  p->ActC0 = -1;
  p->ActDamp = -1;
  p->ActA0 = -1;
}

void amtecInitializeSettings(amtec_powercube_p powercube)
{
  amtecInitializeParams(&(powercube->pan));
  amtecInitializePanSettings(&(powercube->panset));
  powercube->pan.id = AMTEC_DEFAULT_PAN_ID;

  amtecInitializeParams(&(powercube->tilt));
  amtecInitializeTiltSettings(&(powercube->tiltset));
  powercube->tilt.id = AMTEC_DEFAULT_TILT_ID;

  amtecInitializeDevice(&(powercube->dev));
}

amtec_powercube_p amtecInitialize()
{
  amtec_powercube_p powercube;
  powercube = (amtec_powercube_p)calloc(1, sizeof(amtec_powercube_t));
  amtecInitializeSettings(powercube);
  return powercube;
}

void amtecClear(amtec_powercube_p powercube)
{
  free(powercube);
}

void amtecPrintParams(amtec_powercube_params_p params)
{
  fprintf(stderr, "DefHomeOffset  = %4.3f\n", R2D*(params->DefHomeOffset));
  fprintf(stderr, "DefGearRatio   = %4.3f\n", params->DefGearRatio);
  fprintf(stderr, "DefLinRatio    = %4.3f\n", params->DefLinRatio);
  fprintf(stderr, "DefMinPos      = %4.3f\n", R2D*(params->DefMinPos));
  fprintf(stderr, "DefMaxPos      = %4.3f\n", R2D*(params->DefMaxPos));
  fprintf(stderr, "DefMaxDeltaPos = %4.3f\n", R2D*(params->DefMaxDeltaPos));
//  fprintf(stderr, "DefMaxDeltaVel = %4.3f\n", params->DefMaxDeltaVel);
  fprintf(stderr, "DefTorqueRatio = %4.3f\n", params->DefTorqueRatio);
  fprintf(stderr, "DefCurRatio    = %4.3f\n", params->DefCurRatio);
  fprintf(stderr, "DefMaxVel      = %4.3f\n", params->DefMaxVel);
  fprintf(stderr, "DefMaxAcc      = %4.3f\n", params->DefMaxAcc);
  fprintf(stderr, "DefMaxCur      = %4.3f\n", params->DefMaxCur);
  fprintf(stderr, "DefHomeVel     = %4.3f\n", params->DefHomeVel);
  fprintf(stderr, "DefHomeAcc     = %4.3f\n", params->DefHomeAcc);

  fprintf(stderr, "DefCubeSerial  = %u\n", params->DefCubeSerial);
  fprintf(stderr, "DefConfig      = %u\n", params->DefConfig);
  fprintf(stderr, "DefPulsesPerTurn = %u\n", params->DefPulsesPerTurn);
  fprintf(stderr, "DefCubeVersion = %u\n", params->DefCubeVersion);
  fprintf(stderr, "DefServiceInterval = %u\n", params->DefServiceInterval);
  fprintf(stderr, "DefBrakeTimeOut= %u\n", params->DefBrakeTimeOut);
  fprintf(stderr, "DefAddress     = %u\n", params->DefAddress);
  fprintf(stderr, "DefPrimBaud    = %u\n", params->DefPrimBaud);
  fprintf(stderr, "DefScndBaud    = %u\n", params->DefScndBaud);
  fprintf(stderr, "PosCount       = %d\n", params->PosCount);
  fprintf(stderr, "RefPosCount    = %d\n", params->RefPosCount);
  fprintf(stderr, "DioSetup       = %u\n", params->DioSetup);
  fprintf(stderr, "CubeState      = %u\n", params->CubeState);
  fprintf(stderr, "TargetPosInc   = %u\n", params->TargetPosInc);
  fprintf(stderr, "TargetVelInc   = %u\n", params->TargetVelInc);
  fprintf(stderr, "TargetAccInc   = %u\n", params->TargetAccInc);
  fprintf(stderr, "StepInc        = %u\n", params->StepInc);


  // fprintf(stderr, "  DefSendBaud    = %u\n", params->DefSendBaud);
  fprintf(stderr, "MinPos         = %4.3f\n", R2D*(params->MinPos));
  fprintf(stderr, "MaxPos         = %4.3f\n", R2D*(params->MaxPos));
  fprintf(stderr, "MaxVel         = %4.3f\n", params->MaxVel);
  fprintf(stderr, "MaxAcc         = %4.3f\n", params->MaxAcc);
  fprintf(stderr, "MaxCur         = %4.3f\n", params->MaxCur);
  // fprintf(stderr, "  TargetAcc      = %4.3f\n", params->TargetAcc);
  // fprintf(stderr, "  TargetVel      = %4.3f\n", params->TargetVel);
  fprintf(stderr, "ActPos         = %4.3f\n", R2D*(params->ActPos));
  fprintf(stderr, "ActVel         = %4.3f\n", params->ActVel);
  fprintf(stderr, "Break          = %d\n", (int) ((params->Config >> 3) & 1));

  fprintf(stderr, "ActC0          = %d\n", params->ActC0);
  fprintf(stderr, "ActDamp        = %d\n", params->ActDamp);
  fprintf(stderr, "ActA0          = %d\n", params->ActA0);
}

void amtecPrintInformation(amtec_powercube_p powercube) {
  fprintf(stderr, "\n------ PAN -----\n");
  amtecPrintParams(&powercube->pan);
  fprintf(stderr, "\n");
  fprintf(stderr, "\n------ TILT -----\n");
  amtecPrintParams(&powercube->tilt);
}

