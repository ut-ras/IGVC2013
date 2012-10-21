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
 
#ifndef AMTEC_COMMANDS
#define AMTEC_COMMANDS

#include "amtec_base.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * General Commands
 */
int amtecReset(amtec_powercube_device_p dev, int id);                        /* Clear error state */
int amtecHome(amtec_powercube_device_p dev, int id);                         /* Start Homing procedure */
int amtecHalt(amtec_powercube_device_p dev, int id);                         /* Stop immediately */
int amtecRecalcPIDParam(amtec_powercube_device_p dev, int id);               /* Recalculate the PID loop parameters */
int amtecResetTime(amtec_powercube_device_p dev, int id);                    /* Reset internal clock to zero */

/*
 * Motion Commands
 */
int amtecMotionFRamp(amtec_powercube_device_p dev, int id, float position);   /* Target position in rad */
int amtecMotionFStep(amtec_powercube_device_p dev, int id, float position,    /* Target position in rad */
                                                    unsigned short time);      /* Target time in ms */
int amtecMotionFVel(amtec_powercube_device_p dev, int id, float velocity);    /* Target velocity in rad/s */
int amtecMotionFCur(amtec_powercube_device_p dev, int id, float current);     /* Target current in amp */
int amtecMotionIRamp(amtec_powercube_device_p dev, int id, int ticks);        /* Target position in encoder ticks */
int amtecMotionIStep(amtec_powercube_device_p dev, int id, int ticks,         /* Target position in encoder ticks */
                                                    unsigned short time);      /* Target time in ms */
int amtecMotionIVel(amtec_powercube_device_p dev, int id, int val);           /* Target velocity in encoder ticks/s */
int amtecMotionICur(amtec_powercube_device_p dev, int id, short val);         /* Target current in digits */
int amtecMotionFCosLoop(amtec_powercube_device_p dev, int id, float pos,      /* Target position in rad */
                                                    unsigned short period);    /* Target period in ms */
int amtecMotionFRampLoop(amtec_powercube_device_p dev, int id, float val);    /* Target position in rad/s */

/*
 * Retrieve Module Parameters
 */
float          amtecGetDefHomeOffset(amtec_powercube_device_p dev, int id);
float          amtecGetDefGearRatio(amtec_powercube_device_p dev, int id);
float          amtecGetDefLinRatio(amtec_powercube_device_p dev, int id);
float          amtecGetDefMinPos(amtec_powercube_device_p dev, int id);
float          amtecGetDefMaxPos(amtec_powercube_device_p dev, int id);
float          amtecGetDefMaxDeltaPos(amtec_powercube_device_p dev, int id);
float          amtecGetDefMaxDeltaVel(amtec_powercube_device_p dev, int id);
float          amtecGetDefTorqueRatio(amtec_powercube_device_p dev, int id);
float          amtecGetDefCurRatio(amtec_powercube_device_p dev, int id);
float          amtecGetDefMaxVel(amtec_powercube_device_p dev, int id);
float          amtecGetDefMaxAcc(amtec_powercube_device_p dev, int id);
float          amtecGetDefMaxCur(amtec_powercube_device_p dev, int id);
float          amtecGetDefHomeVel(amtec_powercube_device_p dev, int id);
float          amtecGetDefHomeAcc(amtec_powercube_device_p dev, int id);
unsigned int   amtecGetDefCubeSerial(amtec_powercube_device_p dev, int id);
unsigned int   amtecGetDefPulsesPerTurn(amtec_powercube_device_p dev, int id);
unsigned int   amtecGetDefConfig(amtec_powercube_device_p dev, int id);
unsigned short amtecGetDefCubeVersion(amtec_powercube_device_p dev, int id);
unsigned short amtecGetDefServiceInterval(amtec_powercube_device_p dev, int id);
unsigned short amtecGetDefBrakeTimeOut (amtec_powercube_device_p dev, int id);
unsigned char  amtecGetDefAddress(amtec_powercube_device_p dev, int id);
unsigned char  amtecGetDefPrimBaud(amtec_powercube_device_p dev, int id);
unsigned char  amtecGetDefScndBaud(amtec_powercube_device_p dev, int id);
int            amtecGetPosCount(amtec_powercube_device_p dev, int id);
int            amtecGetRefPosCount(amtec_powercube_device_p dev, int id);
unsigned int   amtecGetDioSetup(amtec_powercube_device_p dev, int id);
unsigned int   amtecGetCubeState(amtec_powercube_device_p dev, int id);
unsigned int   amtecGetTargetPosInc(amtec_powercube_device_p dev, int id);
unsigned int   amtecGetTargetVelInc(amtec_powercube_device_p dev, int id);
unsigned int   amtecGetTargetAccInc(amtec_powercube_device_p dev, int id);
unsigned int   amtecGetStepInc(amtec_powercube_device_p dev, int id);

float          amtecGetMinPos(amtec_powercube_device_p dev, int id);
float          amtecGetMaxPos(amtec_powercube_device_p dev, int id);
float          amtecGetMaxVel(amtec_powercube_device_p dev, int id);
float          amtecGetMaxAcc(amtec_powercube_device_p dev, int id);
float          amtecGetMaxCur(amtec_powercube_device_p dev, int id);
float          amtecGetActPos(amtec_powercube_device_p dev, int id);
unsigned int   amtecGetConfig(amtec_powercube_device_p dev, int id);
float          amtecGetActVel(amtec_powercube_device_p dev, int id);
float          amtecGetTargetVel(amtec_powercube_device_p dev, int id);
float          amtecGetTargetAcc(amtec_powercube_device_p dev, int id);
float          amtecGetCur(amtec_powercube_device_p dev, int id);
short          amtecGetActC0(amtec_powercube_device_p dev, int id);
short          amtecGetActDamp(amtec_powercube_device_p dev, int id);
short          amtecGetActA0(amtec_powercube_device_p dev, int id);

/*
 * Set Powercube Module Parameters
 */
void           amtecSetMinPos(amtec_powercube_device_p dev, int id, float val);
void           amtecSetMaxPos(amtec_powercube_device_p dev, int id, float val);
void           amtecSetMaxAcc(amtec_powercube_device_p dev, int id, float val);
void           amtecSetMaxVel(amtec_powercube_device_p dev, int id, float val);
void           amtecSetTargetAcc(amtec_powercube_device_p dev, int id, float val);
void           amtecSetTargetVel(amtec_powercube_device_p dev, int id, float val);
void           amtecSetMaxCur(amtec_powercube_device_p dev, int id, float val);
void           amtecSetConfig(amtec_powercube_device_p dev, int id, unsigned int val);
void           amtecSetActC0(amtec_powercube_device_p dev, int id, short val);
void           amtecSetActDamp(amtec_powercube_device_p dev, int id, short val);
void           amtecSetActA0(amtec_powercube_device_p dev, int id, short val);

///*
// * Special commands
// */
//void           AMTEC_vel_cmd(amtec_powercube_device_p dev, int id, float val);
//void           AMTEC_exec_move(amtec_powercube_device_p dev, int id, float val);
//void           AMTEC_exec_reset(amtec_powercube_device_p dev, int id);
//void           AMTEC_exec_home(amtec_powercube_device_p dev, int id);
//void           AMTEC_exec_halt(amtec_powercube_device_p dev, int id);
//void           AMTEC_recalc_pid(amtec_powercube_device_p dev, int id);
//void           AMTEC_set_velocity(amtec_powercube_device_p dev, int id, float val);
//void           AMTEC_set_acceleration(amtec_powercube_device_p dev, int id, float val);

#ifdef __cplusplus
}
#endif

#endif
