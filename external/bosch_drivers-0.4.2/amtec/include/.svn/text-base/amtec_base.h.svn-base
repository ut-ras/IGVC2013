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
 
#ifndef AMTEC_BASE_H
#define AMTEC_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#define MAX_NAME_LENGTH                256
#define MAX_COMMAND_LENGTH             256
#define MAX_ACMD_SIZE                   48
#define MAX_NUM_LOOPS                   10
#define MAX_TIME_DELAY                 0.2
#define MAX_STATUS_TIME                3.0

#define EPSILON                     0.0001

#define TIMEOUT                         -1
#define WRONG                            0
#define OK                               1


#define INT8                             0
#define UINT8                            1
#define INT16                            2
#define UINT16                           3
#define INT32                            4
#define UINT32                           5
#define FLOAT                            6

/*
 * Short state in Acknowledge
 */
#define SHORT_NOT_OK                  0x01
#define SHORT_SWR                     0x02
#define SHORT_SW1                     0x04
#define SHORT_SW2                     0x08
#define SHORT_MOTION                  0x10
#define SHORT_RAMP_END                0x20
#define SHORT_INPROGRESS              0x40
#define SHORT_FULLBUFFER              0x80

/*
 * Motion modes
 */
#define FRAMP_MODE                    0x04
#define FSTEP_MODE                    0x06
#define FVEL_MODE                     0x07
#define FCUR_MODE                     0x08
#define IRAMP_MODE                    0x09
#define ISTEP_MODE                    0x0b
#define IVEL_MODE                     0x0c
#define ICUR_MODE                     0xd
#define FCOSLOOP                      0x18
#define FRAMPLOOP                     0x19

/*
 * Module State
 */
#define STATE_HOME_OK                 0x00000002
#define STATE_HALTED                  0x00000004
#define STATE_SWR                     0x00000040
#define STATE_SW1                     0x00000080
#define STATE_SW2                     0x00000100
#define STATE_BRAKEACTIVE             0x00000200
#define STATE_CURLIMIT                0x00000400
#define STATE_MOTION                  0x00000800
#define STATE_RAMP_ACC                0x00001000
#define STATE_RAMP_STEADY             0x00002000
#define STATE_RAMP_DEC                0x00004000
#define STATE_RAMP_END                0x00008000
#define STATE_INPROGRESS              0x00010000
#define STATE_FULLBUFFER              0x00020000
#define STATE_ERROR                   0x00000001
#define STATE_POWERFAULT              0x00000008
#define STATE_TOW_ERROR               0x00000010
#define STATE_COMM_ERROR              0x00000020
#define STATE_POW_VOLT_ERR            0x00040000
#define STATE_POW_FET_TEMP            0x00080000
#define STATE_POW_INTEGRALERR         0x00800000
#define STATE_BEYOND_HARD             0x02000000
#define STATE_BEYOND_SOFT             0x04000000
#define STATE_LOGIC_VOLT              0x08000000

/*
 * A frame is always starting with the character STX (02h) and finishes
 * with the character ETX (03h).
 */
#define B_STX                         0x02
#define B_ETX                         0x03
/*
 * If these characters occur within the data stream they will be replaced using a
 * combination of two characters: DLE (10h) and (80h + character to replace). The character DLE
 * (10h) used in this case will thus be replaced in the same manner
 */
#define B_DLE                         0x10
#define B_ACK                         0x64

/*
 * The data transfered first of all states the command to be processed by the module. These
 * commands are available:
 */
#define CodeReset                     0x00 /* Clear error state */
#define CodeHome                      0x01 /* Start Homing procedure */
#define CodeHalt                      0x02 /* Stop immediately */
#define CodeRecalcPIDParam            0x09 /* Recalculate the PID loop parameters */
#define CodeSetExtended               0x08 /* Set parameter */
#define CodeGetExtended               0x0a /* Fetch parameter */
#define CodeSetMotion                 0x0b /* Set Motion command */
#define CodeSetIStep                  0x0d /* Motion command in Step mode */
#define CodeResetTime                 0x12 /* Reset internal clock to zero */

/*
 * Parameter IDs
 */
#define CodeDefHomeOffset             0x00
#define CodeDefGearRatio              0x01
#define CodeDefLinRatio               0x02
#define CodeDefMinPos                 0x03
#define CodeDefMaxPos                 0x04
#define CodeDefMaxDeltaPos            0x05
#define CodeDefMaxDeltaVel            0x06
#define CodeDefTorqueRatio            0x07
#define CodeDefCurRatio               0x08
#define CodeDefMaxVel                 0x0a
#define CodeDefMaxAcc                 0x0c
#define CodeDefMaxCur                 0x0e
#define CodeDefHomeVel                0x0f
#define CodeDefHomeAcc                0x10
#define CodeDefCubeSerial             0x1a
#define CodeDefConfig                 0x1b
#define CodeDefPulsesPerTurn          0x1c
#define CodeDefCubeVersion            0x1d
#define CodeDefServiceInterval        0x1e
#define CodeDefBrakeTimeOut           0x1f
#define CodeDefAddress                0x20
#define CodeDefPrimBaud               0x22
#define CodeDefScndBaud               0x23
#define CodePosCount                  0x24
#define CodeRefPosCount               0x25
#define CodeDioSetup                  0x26
#define CodeCubeState                 0x27
#define CodeTargetPosInc              0x28
#define CodeTargetVelInc              0x29
#define CodeTargetAccInc              0x2a
#define CodeStepInc                   0x2b

#define CodeConfig                    0x39
#define CodeActPos                    0x3c
#define CodeActVel                    0x41
#define CodeMinPos                    0x45
#define CodeMaxPos                    0x46
#define CodeMaxVel                    0x48
#define CodeMaxAcc                    0x4a
#define CodeMaxCur                    0x4c
#define CodeCur                       0x4d
#define CodeTargetVel                 0x4f
#define CodeTargetAcc                 0x50
#define CodeDefC0                     0x51
#define CodeDefDamp                   0x52
#define CodeDefA0                     0x53
#define CodeActC0                     0x54
#define CodeActDamp                   0x55
#define CodeActA0                     0x56
#define CodeDefBurnCount              0x57
#define CodeSetup                     0x58
#define CodeHomeOffset                0x59

enum PARITY_TYPE   { N, E, O };

typedef struct {
  char                       ttyport[MAX_NAME_LENGTH];
  int                        baud;
  enum PARITY_TYPE           parity;
  int                        fd;
  int                        databits;
  int                        stopbits;
  int                        hwf;
  int                        swf;
} amtec_powercube_device_t, *amtec_powercube_device_p;

typedef struct {
  double                     MinPos;
  double                     MaxPos;
  double                     MaxVel;
  double                     MaxAcc;
  double                     MaxCur;
  double                     TargetVel;
  double                     TargetAcc;
  int                        UseBreak;

  int                        C0;
  int                        Damp;
  int                        A0;
} amtec_powercube_setting_t, *amtec_powercube_setting_p;

typedef struct {
  int                        id;
  float                      DefHomeOffset;
  float                      DefGearRatio;
  float                      DefLinRatio;
  float                      DefMinPos;
  float                      DefMaxPos;
  float                      DefMaxDeltaPos;
  float                      DefMaxDeltaVel;
  float                      DefTorqueRatio;
  float                      DefCurRatio ;
  float                      DefMaxVel;
  float                      DefMaxAcc;
  float                      DefMaxCur;
  float                      DefHomeVel;
  float                      DefHomeAcc;

  unsigned int               DefCubeSerial;
  unsigned int               DefConfig;
  unsigned int               DefPulsesPerTurn;
  unsigned short             DefCubeVersion;
  unsigned short             DefServiceInterval;
  unsigned short             DefBrakeTimeOut;
  unsigned char              DefAddress;
  unsigned char              DefPrimBaud;
  unsigned char              DefScndBaud;
  int                        PosCount;
  int                        RefPosCount;
  unsigned int               DioSetup;
  unsigned int               CubeState;
  unsigned int               TargetPosInc;
  unsigned int               TargetVelInc;
  unsigned int               TargetAccInc;
  unsigned int               StepInc;

  double                     ActPos;
  double                     ActVel;
  double                     MinPos;
  double                     MaxPos;
  double                     MaxVel;
  double                     MaxAcc;
  double                     TargetVel;
  double                     TargetAcc;
  double                     MaxCur;
  unsigned long              Config;
  int                        UseBreak;

  int                        ActC0;
  int                        ActDamp;
  int                        ActA0;
} amtec_powercube_params_t, *amtec_powercube_params_p;

typedef struct {
  float                      pan;
  float                      tilt;
  struct timeval             time;
} amtec_powercube_pos_t, *amtec_powercube_pos_p;

typedef struct {
  amtec_powercube_device_t   dev;
  amtec_powercube_params_t   pan;
  amtec_powercube_setting_t  panset;
  amtec_powercube_params_t   tilt;
  amtec_powercube_setting_t  tiltset;
} amtec_powercube_t, *amtec_powercube_p;

amtec_powercube_p amtecInitialize();

void amtecClear(amtec_powercube_p powercube);

void amtecPrintInformation(amtec_powercube_p powercube);

void amtecPrintParams(amtec_powercube_params_p params);

#ifdef __cplusplus
}
#endif

#endif



