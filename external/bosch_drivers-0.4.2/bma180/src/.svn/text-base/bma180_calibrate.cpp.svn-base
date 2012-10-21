/*
 * Copyright (c) 2010, Bosch LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Bosch LLC nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <bma180/bma180meas.h>
#include <ros/time.h>
#include <bma180/bma180.h>
#include <bma180/bma180_calibrate.h>

bma180_calibrate::bma180_calibrate() {
  for (iActiveDataIndex=0; iActiveDataIndex < 2; iActiveDataIndex++) {
    CalibSensor[iActiveDataIndex].bCalib_complete		= false;
    CalibSensor[iActiveDataIndex].bCalibsensID_set		= false;
    CalibSensor[iActiveDataIndex].iNumOfCalibMeas		= 0;
  }
  iActiveDataIndex 		= 0;	//primary is always index 0
  bVerification_activated = false;
};


bma180_calibrate::~bma180_calibrate() {
};

bool bma180_calibrate::setdata_bma180(OneBma180Meas OneMeas) {
  double 	std_estbias_acc;
  double 	measdev_accX;
  double 	measdev_accY;
  double 	measdev_accZ;
  bool 	bSetdata_success = false;

  if (CalibSensor[iActiveDataIndex].bCalibsensID_set == true) {
    if  ( ( CalibSensor[iActiveDataIndex].Sensor_ID.Sub20ID == OneMeas.strSerial ) && (CalibSensor[iActiveDataIndex].Sensor_ID.iChipSelect == OneMeas.iChipSelect[0]) ) {
      if ( CalibSensor[iActiveDataIndex].bCalib_complete == false ) {
        if (CalibSensor[iActiveDataIndex].iNumOfCalibMeas == 0) {
          //since first data set - current measurement equals bias
          CalibSensor[iActiveDataIndex].dBias_AccX 			= OneMeas.dAccX[0];
          CalibSensor[iActiveDataIndex].dBias_AccY 			= OneMeas.dAccY[0];
          CalibSensor[iActiveDataIndex].dBias_AccZ 			= OneMeas.dAccZ[0];
          CalibSensor[iActiveDataIndex].AgeOfBias 			= OneMeas.dtomeas;
          CalibSensor[iActiveDataIndex].iNumOfCalibMeas++;
          bSetdata_success = true;
        }
        else {
          //calculate variance of estimated bias
          std_estbias_acc		= bma180_spec::std_acc_g/sqrt((double)CalibSensor[iActiveDataIndex].iNumOfCalibMeas);

          //calculate measurement deviation
          measdev_accX = fabs(OneMeas.dAccX[0] - CalibSensor[iActiveDataIndex].dBias_AccX);
          measdev_accY = fabs(OneMeas.dAccY[0] - CalibSensor[iActiveDataIndex].dBias_AccY);
          measdev_accZ = fabs(OneMeas.dAccZ[0] - CalibSensor[iActiveDataIndex].dBias_AccZ);

//					std::cout << "Num of Meas " << CalibSensor.iNumOfCalibMeas  << "std est bias " << std_estbias_acc << " \r";

          //verify if an outlier is detected
          if  ( (measdev_accX > bma180_spec::outlier_sigma*(std_estbias_acc  + bma180_spec::std_acc_g)) ||
                (measdev_accY > bma180_spec::outlier_sigma*(std_estbias_acc  + bma180_spec::std_acc_g)) ||
                (measdev_accZ > bma180_spec::outlier_sigma*(std_estbias_acc  + bma180_spec::std_acc_g)) ) {

            // movement during calibration
            ROS_INFO("***DUDE - CALIBRATION - DO NOT TOUCH***");
            //Reset validation counter to zero
            CalibSensor[iActiveDataIndex].iNumOfCalibMeas = 0;
          }
          else {
            // Filter bias based on filterconstant tau
            // bias_k1 = bias_k0 + 1/Tau*(meas_k1-bias_k0), where TAU needs to be adjusted to
            // to the filter period as defined by the Allan Variance
            CalibSensor[iActiveDataIndex].dBias_AccX 	= CalibSensor[iActiveDataIndex].dBias_AccX	+ 1/((double)bma180_spec::calib_varreduct)*(OneMeas.dAccX[0]-CalibSensor[iActiveDataIndex].dBias_AccX);
            CalibSensor[iActiveDataIndex].dBias_AccY 	= CalibSensor[iActiveDataIndex].dBias_AccY  + 1/((double)bma180_spec::calib_varreduct)*(OneMeas.dAccY[0]-CalibSensor[iActiveDataIndex].dBias_AccY);
            CalibSensor[iActiveDataIndex].dBias_AccZ 	= CalibSensor[iActiveDataIndex].dBias_AccZ  + 1/((double)bma180_spec::calib_varreduct)*(OneMeas.dAccZ[0]-CalibSensor[iActiveDataIndex].dBias_AccZ);

            CalibSensor[iActiveDataIndex].iNumOfCalibMeas++;
            //set age of bias
            CalibSensor[iActiveDataIndex].AgeOfBias 		= OneMeas.dtomeas;
            //verify if calibration accuracy is satisfied
//						std::cout << " Meas Count: " << CalibSensor.iNumOfCalibMeas << " calibe reduct " <<  bma180_spec::calib_varreduct << std::endl;
            if ( CalibSensor[iActiveDataIndex].iNumOfCalibMeas >= bma180_spec::calib_varreduct ) {
              //indicate that system has been calibrated
              CalibSensor[iActiveDataIndex].bCalib_complete 	= true;
              //measurement successful
              bSetdata_success = true;
              //std::cout << "Calibration completed BiasX " << CalibSensor[iActiveDataIndex].dBias_AccX << " BiasY " << CalibSensor[iActiveDataIndex].dBias_AccY << " BiasZ " << CalibSensor[iActiveDataIndex].dBias_AccZ << std::endl;
            }
          }
        }
      }
      else {
//			std::cout << " Calibration completed " << std::endl;
      }
    }
    else {
      //message does contain data from other sensor
    }
  }
  else {
    //Sensor is not set
  }
  return (bSetdata_success);
};

void bma180_calibrate::setcalibsens(OneBma180Meas OneMeas) {
  int iCount;
  for (iCount=0; iCount < 2; iCount++) {
    CalibSensor[iCount].Sensor_ID.iChipSelect 	= OneMeas.iChipSelect[0];
    CalibSensor[iCount].Sensor_ID.Sub20ID		= OneMeas.strSerial;
    CalibSensor[iCount].bCalibsensID_set 		= true;
    CalibSensor[iCount].bCalib_complete			= false;
    CalibSensor[iCount].iNumOfCalibMeas			= 0;
  }
}

void bma180_calibrate::clearcalibsens() {
  int iCount;
  for (iCount=0; iCount < 2; iCount++) {
    CalibSensor[iCount].bCalibsensID_set 		= false;
    CalibSensor[iCount].bCalib_complete			= false;
    CalibSensor[iCount].iNumOfCalibMeas			= 0;
  };
  iActiveDataIndex = 0;
};

bool bma180_calibrate::calibsens_completed() {
  return(CalibSensor[0].bCalib_complete);
};

bool bma180_calibrate::verification_completed() {
  return(CalibSensor[1].bCalib_complete);
};

bool bma180_calibrate::calibsens_set() {
  return(CalibSensor[0].bCalibsensID_set);
};

bool bma180_calibrate::verification_active() {
  return(bVerification_activated);
};

bool bma180_calibrate::get_estbiases(double* dAccX, double* dAccY, double* dAccZ ) {
  bool bSuccess;
  //always provides the primary bias
  if ( (CalibSensor[0].bCalibsensID_set==true)&&(CalibSensor[0].bCalib_complete==true) ) {
    *dAccX = CalibSensor[0].dBias_AccX;
    *dAccY = CalibSensor[0].dBias_AccY;
    *dAccZ = CalibSensor[0].dBias_AccZ;
    bSuccess = true;
  }
  else {
    *dAccX = 0;
    *dAccY = 0;
    *dAccZ = 0;
    bSuccess = false;
  }
  return(bSuccess);
};

bool bma180_calibrate::biasverify(void) {
  bool bSuccess;
  if ( (CalibSensor[0].bCalibsensID_set==true)&&(CalibSensor[0].bCalib_complete==true) ) {
    iActiveDataIndex = 1;
    bVerification_activated = true;
    bSuccess = true;
  }
  else {
    bSuccess = false;
  }
  return (bSuccess);
};

bool bma180_calibrate::get_verifiedbiases(double* dAccX, double* dAccY, double* dAccZ ) {
  bool bSuccess;
  if ( (CalibSensor[1].bCalibsensID_set==true)&&(CalibSensor[1].bCalib_complete==true) ) {
    *dAccX = CalibSensor[1].dBias_AccX;
    *dAccY = CalibSensor[1].dBias_AccY;
    *dAccZ = CalibSensor[1].dBias_AccZ;
    bSuccess = true;
  }
  else {
    *dAccX = 0;
    *dAccY = 0;
    *dAccZ = 0;
    bSuccess = false;
  }
  return(bSuccess);
};

