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

//\Author Lukas Marti, Bosch LLC

#ifndef __BMA180_CALIBRATE__
#define __BMA180_CALIBRATE__

#include <bma180/bma180meas.h>
#include <bma180/bma180.h>

namespace bma180_spec {
  const static double std_acc_g	            = 0.02;  		//[g] measured on the actual sensor
  const static double outlier_sigma         = 2; 			//consdiering 3 sigma rule for outlier
  const static int    calib_varreduct       = 100;			//calibartion accuracy -> variance red = num of calib samples
};

//! \brief A Class to estimate the sensor biases
/*!
 * The class handles the calibration of a sensor. It estimates the biases and
 * subsequently performs a verification of the estimated biases
 */
class bma180_calibrate
{
public:
  //! Constructor resets calibration status
  /*!
   * Resets sensor calibration ID to undefined and
   * verification mode to inactive
   */
  bma180_calibrate				();

  //! Destructor
  ~bma180_calibrate				();

  //! Sets data to be used for calibrating the sensor.
  /*!
   * Verifies if the received data set is from the sensor to be calibrated. If this is
   * given, the data is run through the calibration filter.
   *
   * \param OneMeas contains one measurement from a BMA180
   * \return success when data was used for calibration
   */
  bool setdata_bma180			(struct OneBma180Meas);

  //! Sets the sensor ID to be calibrated
  /*!
   * Sets the sensor ID to be calibrated. Whenever calling this method
   * any previously defined sensor will be cleared and the calibration
   * status will be reset
   *
   * \param OneMeas contains the ID of the sensor to be calibrated
   */
  void setcalibsens				(struct OneBma180Meas);

  //! Clears any previously gathered calibration data
  /*!
   * Resets any previously defined calibration data to neutral
   */
  void clearcalibsens			();

  //! Verifies if calibration has been completed
  /*!
   * \return Calibration complete true or false
   */
  bool calibsens_completed		();

  //! Verifies if calibration verification has been completed
  /*!
   * \return Verification calibration complete true or false
   */
  bool verification_completed	();

  //! Verifies if the calibration verification has been activated
  /*!
   *
   * Verification can only be activated upon successful completion of
   * calibration
   *
   * \return Verification active true or false
   */
  bool verification_active 		();

  //! Verifies if a calibration sensor ID has been set
  /*!
   * \return sensor ID set true or false
   */
  bool calibsens_set			();

  //! Retrieves estimated sensor biases
  /*!
   * \return sensor ID set true or false
   */
  bool get_estbiases			(double*, double*, double*);
  bool biasverify				();
  bool get_verifiedbiases		(double*, double*, double*);

private:
  struct SensID {
	std::string					Sub20ID;
	int							iChipSelect;
  };
  struct OneCalibDataSet {
    SensID	      Sensor_ID;
    ros::Time     AgeOfBias; //contains the time stamp of the last calibration measurement
    double        dBias_AccX;
    double        dBias_AccY;
    double        dBias_AccZ;
    int           iNumOfCalibMeas;
    bool          bCalib_complete;
    bool          bCalibsensID_set;
  };
  OneCalibDataSet CalibSensor[2];  //2 array of 0: primary calibration, 1: calibration verification
  short int       iActiveDataIndex;
  bool            bVerification_activated;
}; // class

#endif
