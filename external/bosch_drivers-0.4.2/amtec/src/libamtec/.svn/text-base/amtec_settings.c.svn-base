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
#include "amtec_io.h"
#include "amtec_commands.h"
#include "amtec_conversion.h"
#include "amtec_settings.h"

void AMTEC_set_settings(amtec_powercube_device_p dev, amtec_powercube_setting_p settings, amtec_powercube_params_p params)
{
  if(params->MinPos != settings->MinPos) {
    amtecSetMinPos(dev, params->id, settings->MinPos);
    params->MinPos = settings->MinPos;
  }
  if(params->MaxPos != settings->MaxPos) {
    amtecSetMaxPos(dev, params->id, settings->MaxPos);
    params->MaxPos = settings->MaxPos;
  }
  if(params->MaxAcc != settings->MaxAcc) {
    amtecSetMaxAcc(dev, params->id, settings->MaxAcc);
    params->MaxAcc = settings->MaxAcc;
  }
  if(params->MaxVel != settings->MaxVel) {
    amtecSetMaxVel(dev, params->id, settings->MaxVel);
    params->MaxVel = settings->MaxVel;
  }
  if(params->TargetAcc != settings->TargetAcc) {
    amtecSetTargetAcc(dev, params->id, settings->TargetAcc);
    params->TargetAcc = settings->TargetAcc;
  }
  if(params->TargetVel != settings->TargetVel) {
    amtecSetTargetVel(dev, params->id, settings->TargetVel);
    params->TargetVel = settings->TargetVel;
  }
  if(params->MaxCur != settings->MaxCur) {
    amtecSetMaxCur(dev, params->id, settings->MaxCur);
    params->MaxCur = settings->MaxCur;
  }

  if (settings->C0 != 0 && params->ActC0 != settings->C0) {
    amtecSetActC0(dev, params->id, settings->C0);
  }
  if (settings->Damp != 0 && params->ActDamp != settings->Damp) {
    amtecSetActDamp(dev, params->id, settings->Damp);
  }
  if (settings->A0 != 0 && params->ActA0 != settings->A0) {
    amtecSetActA0(dev, params->id, settings->A0);
  }

  amtecRecalcPIDParam(dev, params->id);
  /*
  if(settings->UseBreak) {
    params->Config = amtecGetConfig(dev, params->id);
    params->Config = params->Config | 8;
    amtecSetConfig(dev, params->id, params->Config);
  }*/
}

void amtecSetSettings(amtec_powercube_p powercube)
{
  AMTEC_set_settings(&powercube->dev, &powercube->panset, &powercube->pan);
  AMTEC_set_settings(&powercube->dev, &powercube->tiltset, &powercube->tilt);
}

void AMTEC_get_params(amtec_powercube_device_p dev, amtec_powercube_params_p params)
{
  params->DefHomeOffset  = amtecGetDefHomeOffset(dev, params->id);
  params->DefGearRatio   = amtecGetDefGearRatio(dev, params->id);
  params->DefLinRatio    = amtecGetDefLinRatio(dev, params->id);
  params->DefMinPos      = amtecGetDefMinPos(dev, params->id);
  params->DefMaxPos      = amtecGetDefMaxPos(dev, params->id);
  params->DefMaxDeltaVel = amtecGetDefMaxDeltaVel(dev, params->id);
  params->DefTorqueRatio = amtecGetDefTorqueRatio(dev, params->id);
  params->DefCurRatio    = amtecGetDefCurRatio(dev, params->id);
  params->DefMaxVel      = amtecGetDefMaxVel(dev, params->id);
  params->DefMaxAcc      = amtecGetDefMaxAcc(dev, params->id);
  params->DefMaxCur      = amtecGetDefMaxCur(dev, params->id);
  params->DefHomeVel     = amtecGetDefHomeVel(dev, params->id);
  params->DefHomeAcc     = amtecGetDefHomeAcc(dev, params->id);

  params->DefCubeSerial  = amtecGetDefCubeSerial(dev, params->id);
  params->DefConfig      = amtecGetDefConfig(dev, params->id);
  params->DefPulsesPerTurn = amtecGetDefPulsesPerTurn(dev, params->id);
  params->DefCubeVersion  = amtecGetDefCubeVersion(dev, params->id);
  params->DefServiceInterval = amtecGetDefServiceInterval(dev, params->id);
  params->DefBrakeTimeOut = amtecGetDefBrakeTimeOut(dev, params->id);
  params->DefAddress      = amtecGetDefAddress(dev, params->id);
  params->DefPrimBaud     = amtecGetDefPrimBaud(dev, params->id);
  //params->DefScndBaud     = amtecGetDefScndBaud(dev, params->id);
  params->PosCount        = amtecGetPosCount(dev, params->id);
  params->RefPosCount     = amtecGetRefPosCount(dev, params->id);
  params->DioSetup        = amtecGetDioSetup(dev, params->id);
  params->CubeState       = amtecGetCubeState(dev, params->id);
  params->TargetPosInc    = amtecGetTargetPosInc(dev, params->id);
  params->TargetVelInc    = amtecGetTargetVelInc(dev, params->id);
  params->TargetAccInc    = amtecGetTargetAccInc(dev, params->id);
  params->StepInc         = amtecGetStepInc(dev, params->id);


  params->ActPos         = amtecGetActPos(dev, params->id);
  params->ActVel         = amtecGetActVel(dev, params->id);
  params->MinPos         = amtecGetMinPos(dev, params->id);
  params->MaxPos         = amtecGetMaxPos(dev, params->id);
  params->MaxVel         = amtecGetMaxVel(dev, params->id);
  params->MaxAcc         = amtecGetMaxAcc(dev, params->id);
  // params->TargetVel      = amtecGetTargetVel(dev, params->id);
  // params->TargetAcc      = amtecGetTargetAcc(dev, params->id);
  params->MaxCur         = amtecGetMaxCur(dev, params->id);
  params->Config         = amtecGetConfig(dev, params->id);
  params->UseBreak       = (int)(((params->Config)>>3)&1);

  params->ActC0          = amtecGetActC0(dev, params->id);
  params->ActDamp        = amtecGetActDamp(dev, params->id);
  params->ActA0          = amtecGetActA0(dev, params->id);
}

void amtecGetParams(amtec_powercube_p powercube)
{
  AMTEC_get_params(&powercube->dev, &powercube->pan);
  AMTEC_get_params(&powercube->dev, &powercube->tilt);
}

void PRINT_CubeState(unsigned long state)
{
  int i;
  unsigned long v = state;

  for(i = 0; i < 32; i++)
    fprintf(stderr, "  %s", (v>>i & 1)==1?"+":"-");
  fprintf(stderr, "\r");
}





