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
 * smi540.h
 *
 *  Created on: Jul 27, 2011
 *      Author: Lucas Marti, Robert Bosch LLC |
 *      Author: Nikhil Deshpande, Robert Bosch LLC |
 */

#ifndef SMI540_H_
#define SMI540_H_

#include "smi540/smi540meas.h"
#include <list>
#include <libsub.h> // sub20 device

//Define SMI540 specifics
namespace smi540_cmd
{
  //Define MM5 commands
  static char chRD_ACT_DATA_64[4] = { 0x41, 0x99, 0x00, 0x5C };
  static char chTRIGGER_RESET[4]  = { 0x61, 0x03, 0x00, 0xE9 };
  static char chRD_STATUS_A[4]  = { 0x40, 0x35, 0x00, 0x4F };

  //Specify MM5 Scale Factor - note: Pos is slightly different from negative
  const double fSFACC_inv = 6667;
  const double fSFGYRO_inv = 175;

  //Define enumerated sensor type
  enum eSensorType { eACCEL, eGYRO };

  //Define default sampling rate
  const double DEFAULT_RATE_Hz = 20;
  const std::string sSUB20SERIAL("064E");
  const unsigned short MAX_SENSORS = 5;          //Defined by the Xdimax Unit (which supports 5 sensors per box)
  const unsigned short uSERIALNUMLENGTH   = 20;
}

//Define measurement output
struct OneSmi540Meas
{
  bool        bMeasAvailable; //indicates true if a valid measurement is available
  int         iNumAccels;
  double      dRateZ[smi540_cmd::MAX_SENSORS];
  double      dAccX[smi540_cmd::MAX_SENSORS];
  double      dAccY[smi540_cmd::MAX_SENSORS];
  ros::Time   dtomeas; //time tag measurement immediately in case of other delays
  std::string strSerial;
  int         iChipSelect[smi540_cmd::MAX_SENSORS];
};


//! \brief A Class to interface to the SMI540 over SPI
/*!
 * This class interfaces with the BOSCH SMI 530/540 over SPI using an
 * XDIMAX Sub20 interface device.
 */

class Smi540
{
public:
  //! Constructor initializes Sub20 device and SMI540ies
      /*!
   * Detects connected Sub20 devices and SMI540ies. Initializes
   * the Sub20 device and loads configures the SMI540 with the
   * specified bandwidth and max acceleration range. On request it
   * performs sensor calibration
   *
   * \param sSub20Serial Requesting desired sub20 device to be opened
   */
  Smi540(std::string);

  //! Destructor closing all open connections
    /*!
  * Closes open SPI connections and disables the Sub20 device. Disposes
  * any dynamic structure in the heap.
  */
  virtual ~Smi540();

  //! Polls one measurement from all SMI540 defined in std::list<OneMulSmiMeas>&
     /*!
  * Iterates through all detected SMI540ies on all connected Sub20 devices and sends
  * a measurement request to the respective sensor. In case of calibration request,
  * any received measurement is used to calibrate the sensor.
  * NOTE: As the EEPROM can only handle a limited amount of write cycles, the
  *       calibration routine only allows one Sub20 device to be connected for
  *       the calibration mode to be executed successfully.
  *
  * \param &list_meas points onto the std::list containing the connected devices
  */
  void GetMeasurements( std::list<OneSmi540Meas>& );

private:
  // structure for one smi540 configuration
  struct OneSmi540Config
  {
    bool bConfigured;
  };
  // structure for one Sub20 device configuration
  struct OneSub20Config
  {
    std::string     strSub20Serial;
    sub_handle      handle_subdev;
    sub_device      subdev;
    bool            bSubSPIConfigured;
    OneSmi540Config Smi540Cluster[smi540_cmd::MAX_SENSORS];
  };
  std::list<OneSub20Config>   Sub20Device_list;

  //subdevice handle for closing the device in destructor
  sub_handle subhndl;
  //status flag indicating if subdevice has been opened
  bool bSubDeviceOpen;
  //status flag indicating if SPI configuration for SMI530/540 has been stored
  bool bSubDeviceConfigured;
  //Serial number of subdevice
  std::string strSerial;
  /// Converts SMI540 formatted data into int
  double mm5data_to_double( char, char, smi540_cmd::eSensorType );
  /// Configures specified SMI540ies according to the default or user settings
  void confsens_on_sub20( OneSub20Config* );
};


#endif /* SMI540_H_ */

