/*
 * Copyright (c) 2011, Robert Bosch LLC
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
 *     * Neither the name of Robert Bosch LLC nor the names of its
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

/*
 * bma180.h
 *
 *  Created on: Jul 26, 2011
 *      Author: Lucas Marti, Robert Bosch LLC |
 *      Editor: Nikhil Deshpande, Robert Bosch LLC |
 */

#ifndef BMA180_H_
#define BMA180_H_

#include "bma180/bma180meas.h"
#include "bma180/bma180err.h"
#include "bma180/bma180_calibrate.h"
#include <list>
#include <libsub.h> // sub20 device

//Define BMA180 specifics
namespace bma180_cmd {
  const char chFLAG_R             = {0x80};
  //Define BMA180 commands
  const char    chADR_ACCLXYZ        = {0x02};  //read accel data
  const char    chADR_VER            = {0x00};  //read version data
  const char    chADR_STATREG1       = {0x09};
  const char    chADR_STATREG2       = {0x0A};
  const char    chADR_STATREG3       = {0x0B};
  const char    chADR_STATREG4       = {0x0C};
  const char    chADR_CTRLREG0       = {0x0D};
  const char    chADR_CTRLREG1       = {0x0E};
  const char    chADR_CTRLREG2       = {0x0F};
  const char    chADR_SOFTRESET      = {0x10};
  const char    chADR_EE_OFFSET_Z    = {0x5A};  // |
  const char    chADR_EE_OFFSET_Y    = {0x59};  // |
  const char    chADR_EE_OFFSET_X    = {0x58};  // |-> THIS IS THE EEPROM
  const char    chADR_EE_OFFSET_T    = {0x57};  // |
  const char    chADR_EE_OFFSET_LSB2 = {0x56};  // |
  const char    chADR_EE_OFFSET_LSB1 = {0x55};  // |
  const char    chADR_OFFSET_Z       = {0x3A};
  const char    chADR_OFFSET_Y       = {0x39};
  const char    chADR_OFFSET_X       = {0x38};
  const char    chADR_OFFSET_T       = {0x37};
  const char    chADR_OFFSET_LSB2    = {0x36};
  const char    chADR_OFFSET_LSB1    = {0x35};
  const char    chADR_EE_GAIN_Z      = {0x54};
  //-----------------------------------
  const char    chADR_BWTCS          = {0x20};
  const char    chADR_RANGE          = {0x35};
  //-----------------------------------
  const double  dFULLRANGELOOKUP[] = {1.0, 1.5, 2.0, 3.0, 4.0, 8.0, 16.0};
  const char    chCMD_FULLSCALE_G[]= {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
  //-----------------------------------
  const double  dBWLOOKUP[]       = {10, 20, 40, 75, 150, 300, 600, 1200};
  const char    chCMD_BANDWIDTH_HZ[]= {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
  //-----------------------------------
  const char    chCMD_SOFTRESET         = {0xB6};
  const char    chCMD_SET_EEW           = {0x01};
  //-----------------------------------
  enum eSensorType {eACCEL, eTEMP, eBIAS_X, eBIAS_Y, eBIAS_Z};
  //-----------------------------------
  const unsigned short iMAXNUM_OF_SENSORS = 5;          //Defined by the Xdimax Unit (which supports 5 sensors per box)
  //-----------------------------------
  const double  dDEFAULT_MAXACC_g               = 16;
  const std::string     sSUB20SERIAL("0651");
  const double  dDEFAULT_BANDWIDTH_Hz           = 300;
  const double  dDEFAULT_RATE_Hz                = 500;
  const bool    bDEFAULT_CALIBRATE              = false;
  const double  dCALIB_ACCURACY                 = 0.05;
  //-----------------------------------
  const unsigned short uSERIALNUMLENGTH = 20;
  //-----------------------------------
  const short   iDEFAULT_CALIBCHIPSELECT= 0;            //this is out of range and ensures that no calibration is executed in case of error
}

//Define measurement output
struct OneBma180Meas {
  bool          bMeasAvailable; //indicates true if a valid measurement is available
  int           iNumAccels;
  double        dAccX[bma180_cmd::iMAXNUM_OF_SENSORS];
  double        dAccY[bma180_cmd::iMAXNUM_OF_SENSORS];
  double        dAccZ[bma180_cmd::iMAXNUM_OF_SENSORS];
  double        dTemp;
  ros::Time     dtomeas; //time tag measurement immediately in case of other delays
  std::string   strSerial;
  int           iChipSelect[bma180_cmd::iMAXNUM_OF_SENSORS];
};


//! \brief A Class to interface to the BMA180 over SPI
/*!
 * This class interfaces with Bosch BMA180 over SPI using an
 * XDIMAX Sub20 interface device. The class allows multiple Sub20
 * devices to be connected. Each Sub20 device can handle up to 5
 * BMA 180ies. The class independently detects a BMA180 connected.
 */
class Bma180 {
  public:
        //! Constructor initializes Sub20 device and BMA180ies
        /*!
     * Detects connected Sub20 devices and BMA180ies. Initializes
     * the Sub20 device and loads configures the BMA180 with the
     * specified bandwidth and max acceleration range. On request it
     * performs sensor calibration
     *
     * \param dMaxAcc_g The maximum acceleration range of the sensor
     * \param dBandwidth_Hz The bandwidth of the sensor front-end
     * \param bCalibrate Requesting sensor calibration
     * \param dRate_Hz Sensor reading rate
     * \param sSub20Serial Requesting desired sub20 to be opened
     */
        Bma180(double, double, bool, double, std::string);

        //! Destructor closing all open connections
        /*!
     * Closes open SPI connections and disables the Sub20 device. Disposes
     * any dynamic structure in the heap.
     */
        ~Bma180();

        //! Polls one measurement from all BMA180 defined in std::list<OneMultBmaMeas>&
        /*!
     * Iterates through all detected BMA180ies on all connected Sub20 devices and sends
     * a measurement request to the respective sensor. In case of calibration request,
     * any received measurement is used to calibrate the sensor.
     * NOTE: As the EEPROM can only handle a limited amount of write cycles, the
     *       calibration routine only allows one Sub20 device to be connected for
     *       the calibration mode to be executed successfully.
     *
     * \param &list_meas points onto the std::list containing the connected devices
     */
        void GetMeasurements( std::list<OneBma180Meas>& );

  private:
        // structure for one bma180 configuration
    struct OneBma180Config {
      double          dFullScaleRange;
      double          dSensorBandwidth;
      bool            bConfigured;
      unsigned short  uiBiasImageX;
      unsigned short  uiBiasImageY;
      unsigned short  uiBiasImageZ;
      double          iNumOfCalibMeas;
    };
        // structure for one Sub20 device configuration
    struct OneSub20Config {
      std::string     strSub20Serial;
      sub_handle      handle_subdev;
      sub_device      subdev;
      bool            bSubSPIConfigured;
      OneBma180Config Bma180Cluster[bma180_cmd::iMAXNUM_OF_SENSORS];
    };
    std::list<OneSub20Config>   Sub20Device_list;

    // internal calibration flags
    bool              bExecuteCalibration;
    bool              bCalibrationCompleted;
    bma180_calibrate  calib_bma180;

    // internal selected accelerometer range and bandwidth
    char              chMaxAccRange_selected;
    char              chBW_selected;
    int               iCalib_CS;

    // internal Sub20 device data
    sub_handle        subhndl;                                               //handle for subdevice
    bool              bSubDeviceOpen;                   //initialization flag
    bool              bSubDeviceConfigured;     //verify if subdevice is configured
    std::string       strSerial;                                //Serial number of SUB20 device

    /// Converts BMA180 formatted data into double
    double            bma180data_to_double(char, char, bma180_cmd::eSensorType, int *,  double);

    /// Converts BMA180 formatted data into an unsigned int
    unsigned short    bma180data_to_uint(char, char, bma180_cmd::eSensorType );

    /// Reads the stored bias settings of the BMA180
    bool              read_biassettings(sub_handle, int, unsigned short*, unsigned short*, unsigned short*);

    /// Stores estimated biases into the BMA180 image
    bool              set_biassettings(sub_handle, int, unsigned short, unsigned short, unsigned short, bool);

    /// Reads a byte from the EEPROM
    bool              read_byte_eeprom_sub20(char, char*, unsigned short, sub_handle);

    /// Writes specified bits into the eeprom image
    bool              write_bit_eeprom_sub20(char, unsigned short, unsigned short, char, unsigned short, sub_handle);

    /// Configures specified BMA180 according to the default or user settings
    void              confsens_on_sub20(OneSub20Config*, char, char);

    /// Sets the calibration status of the object
    void              set_calibstatus(bool bCalibrate);
};

#endif /* MULT_BMA180_H_ */

