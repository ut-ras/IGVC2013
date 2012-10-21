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
 
#include <iostream>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#include <amtec_base.h>
#include <amtec_commands.h>
#include <amtec_settings.h>
#include <amtec_io.h>

using namespace std;

void printModuleState(unsigned int state)
{
  if(state&STATE_HOME_OK) std::cout << "STATE_HOME_OK" << std::endl;
  if(state&STATE_HALTED) std::cout << "STATE_HALTED" << std::endl;
  if(state&STATE_SWR) std::cout << "STATE_SWR" << std::endl;
  if(state&STATE_SW1) std::cout << "STATE_SW1" << std::endl;
  if(state&STATE_SW2) std::cout << "STATE_SW2" << std::endl;
  if(state&STATE_BRAKEACTIVE) std::cout << "STATE_BRAKEACTIVE" << std::endl;
  if(state&STATE_CURLIMIT) std::cout << "STATE_CURLIMIT" << std::endl;
  if(state&STATE_MOTION) std::cout << "STATE_MOTION" << std::endl;
  if(state&STATE_RAMP_ACC) std::cout << "STATE_RAMP_ACC" << std::endl;
  if(state&STATE_RAMP_STEADY) std::cout << "STATE_RAMP_STEADY" << std::endl;
  if(state&STATE_RAMP_DEC) std::cout << "STATE_RAMP_DEC" << std::endl;
  if(state&STATE_RAMP_END) std::cout << "STATE_RAMP_END" << std::endl;
  if(state&STATE_INPROGRESS) std::cout << "STATE_INPROGRESS" << std::endl;
  if(state&STATE_FULLBUFFER) std::cout << "STATE_FULLBUFFER" << std::endl;
  if(state&STATE_ERROR) std::cout << "STATE_ERROR" << std::endl;
  if(state&STATE_POWERFAULT) std::cout << "STATE_POWERFAULT" << std::endl;
  if(state&STATE_TOW_ERROR) std::cout << "STATE_TOW_ERROR" << std::endl;
  if(state&STATE_COMM_ERROR) std::cout << "STATE_COMM_ERROR" << std::endl;
  if(state&STATE_POW_VOLT_ERR) std::cout << "STATE_POW_VOLT_ERR" << std::endl;
  if(state&STATE_POW_FET_TEMP) std::cout << "STATE_POW_FET_TEMP" << std::endl;
  if(state&STATE_POW_INTEGRALERR) std::cout << "STATE_POW_INTEGRALERR" << std::endl;
  if(state&STATE_BEYOND_HARD) std::cout << "STATE_BEYOND_HARD" << std::endl;
  if(state&STATE_BEYOND_SOFT) std::cout << "STATE_BEYOND_SOFT" << std::endl;
  if(state&STATE_LOGIC_VOLT) std::cout << "STATE_LOGIC_VOLT" << std::endl;
}

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    printf("Usage: test_amtec DEVICE BAUD_RATE\n");
    return 1;
  }
  std::string amtec_dev = argv[1];
  int amtec_baudrate = atoi(argv[2]);

  amtec_powercube_p amtec;
  amtec = amtecInitialize();
  strcpy(amtec->dev.ttyport, amtec_dev.c_str());
  amtec->dev.baud = amtec_baudrate;

  if (amtecDeviceConnectPort(&amtec->dev) < 0) {
    printf("Unable to connect amtec at %s\n", amtec->dev.ttyport);
    return -1;
  }

  printf("Resetting device\n");
  if(!amtecReset(&amtec->dev, amtec->pan.id)) {
  printf("Unable to connect to pan module\n");
  return -1;
  }
  if(!amtecReset(&amtec->dev, amtec->tilt.id)) {
  printf("Unable to connect tilt module\n");
  return -1;
  }

  printf("Retrieving module state\n");
  unsigned int pan_serial = amtecGetDefCubeSerial(&amtec->dev, amtec->pan.id);
  unsigned short pan_version = amtecGetDefCubeVersion(&amtec->dev, amtec->pan.id);
  unsigned int pan_state = amtecGetCubeState(&amtec->dev, amtec->pan.id);

  std::cout << "pan serial " << pan_serial << std::endl;
  std::cout << "pan version " << pan_version << std::endl;
  std::cout << "pan state: " << std::endl;
  printModuleState(pan_state);

  unsigned int tilt_serial = amtecGetDefCubeSerial(&amtec->dev, amtec->tilt.id);
  unsigned short tilt_version = amtecGetDefCubeVersion(&amtec->dev, amtec->tilt.id);
  unsigned int tilt_state = amtecGetCubeState(&amtec->dev, amtec->pan.id);

  std::cout << "tilt serial " << tilt_serial << std::endl;
  std::cout << "tilt version " << tilt_version << std::endl;
  std::cout << "tilt state: " << std::endl;
  printModuleState(tilt_state);

  float pan = amtecGetActPos(&amtec->dev, amtec->pan.id);
  float tilt = amtecGetActPos(&amtec->dev, amtec->tilt.id);
  printf("pan %f tilt %f\n",pan,tilt);
  
  printf("Homing device\n");
  amtecHome(&amtec->dev, amtec->pan.id);
  amtecHome(&amtec->dev, amtec->tilt.id);
  printf("Done\n");

  return 0;
}

