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
 * bma180.cpp
 *
 *  Created on: Jul 26, 2011
 *      Author: Lucas Marti, Nikhil Deshpande, Philip Roan, Robert Bosch LLC
 */
#include <iostream>
#include <string>
#include <ros/ros.h>
#include "bma180/bma180s.h"
#include <sstream>
#include <ros/time.h>
#include <math.h>
#include <vector>

#define ROS_INFO printf
#define ROS_WARN printf
#define ROS_ERROR printf

Bma180::Bma180( double max_acceleration_g, double dBandwidth_Hz, bool bCalibrate, double dRate_Hz, std::string sSub20Serial ):
  bSubDeviceOpen       (false),
  bSubDeviceConfigured (false)
{
  int               error_code; // Sub20 error code
  int               iSPI_cfg_set; //Set SPI config
  int               iSPI_cfg_get; //Get SPI config
  int               iNumOfRanges;
  int               iElCount;
  std::stringstream ss_errmsg;
  std::string       ss_config, sub20Serial;
  sub_device        sub20dev;
  sub_handle        sub20handle;
  OneSub20Config    OneSub20Configuration;

  printf("Max acceleration entered: %f [g].\n", max_acceleration_g);
  printf("Sensor bandwidth entered: %f [Hz].\n", dBandwidth_Hz);
  printf("Sensor Reading Rate entered: %f [Hz].\n", dRate_Hz);

  //Retrieve calibration data from user
  set_calibstatus( bCalibrate );

  // Evaluate the accelerometer range  to be selected
  iNumOfRanges = sizeof(bma180_cmd::COMMAND_FULLSCALE_G) / sizeof(char);
  iElCount = 0;
  while( (max_acceleration_g > bma180_cmd::dFULLRANGELOOKUP[iElCount]) && (iElCount < iNumOfRanges) )
  {
    iElCount++;
  };
  printf("Range chosen: %f [g].\n", bma180_cmd::dFULLRANGELOOKUP[iElCount]);

  chMaxAccRange_selected = bma180_cmd::COMMAND_FULLSCALE_G[iElCount];
  
  // Evaluate the bandwidth to be selected
  iNumOfRanges = sizeof(bma180_cmd::COMMAND_BANDWIDTH_HZ) / sizeof(char);
  iElCount = 0;
  while( (dBandwidth_Hz > bma180_cmd::dBWLOOKUP[iElCount]) && (iElCount < iNumOfRanges) )
  {
    iElCount++;
  };
  printf("Range chosen BW: %f.\n", bma180_cmd::dBWLOOKUP[iElCount]);

  chBW_selected = bma180_cmd::COMMAND_BANDWIDTH_HZ[iElCount];

  // Open connected Sub20 Devices
  sub20dev = 0;
  sub20dev = sub_find_devices(sub20dev);
  while( sub20dev != 0 )
  {
    sub20handle = sub_open( sub20dev );
    // on success, sub_open returns non-zero handle
    if( sub20handle > 0 )
    {
      sub20Serial.clear();
      sub20Serial.resize( bma180_cmd::uSERIALNUMLENGTH );
      sub_get_serial_number( sub20handle, const_cast<char*>(sub20Serial.c_str()), sub20Serial.size() );
      printf( "Serial Number: %s.\n", sub20Serial.c_str() );
      if( strcmp(sub20Serial.c_str(), sSub20Serial.c_str()) == 0 )
      {
        subhndl = sub20handle;

        // Configure Sub20 device
        std::cout << "---Initializing SPI Interface---" << std::endl;;
        // Read current SPI configuration
        error_code = sub_spi_config( sub20handle, 0, &iSPI_cfg_get );
        std::cout << "Sub SPI config  : " << iSPI_cfg_get << std::endl;
        
        // Important: The SPI interface does not work properly at higher frequencies, i.e. > 2MHz
        // SET: Polarity Fall, SetupSmpl,  MSB first, 2MHZ
        iSPI_cfg_set = SPI_ENABLE | SPI_CPOL_FALL | SPI_SETUP_SMPL | SPI_MSB_FIRST | SPI_CLK_2MHZ;
        // Configure SPI
        error_code = sub_spi_config( sub20handle, iSPI_cfg_set, 0 );
        // Read current SPI configuration
        error_code = sub_spi_config( sub20handle, 0, &iSPI_cfg_get );
        // verify if sub20 device has accepted the configuration
        if( iSPI_cfg_get == iSPI_cfg_set )
	{
          std::cout<< "SPI Configuration: " << iSPI_cfg_set << " successfully stored." << std::endl;

          // Subdevice has been configured successfully
          OneSub20Configuration.bSubSPIConfigured = true;
          OneSub20Configuration.handle_subdev     = sub20handle;

          // Configure Sensors on Sub20
          confsens_on_sub20( &OneSub20Configuration, chMaxAccRange_selected, chBW_selected);
        }
        else
	{
          ROS_INFO("ERROR! SPI Configuration %d not accepted by device.\n", iSPI_cfg_set);

          // Subdevice could not be configured
          OneSub20Configuration.bSubSPIConfigured = false;
        }
        if( OneSub20Configuration.bSubSPIConfigured )
	{
          // string needs to be cleared otherwise conversion is going wrong
          serial_number.clear();
          serial_number.resize( bma180_cmd::uSERIALNUMLENGTH );
          sub_get_serial_number( sub20handle, const_cast<char*>(serial_number.c_str()), serial_number.size() );
          OneSub20Configuration.strSub20Serial = serial_number;
          OneSub20Configuration.subdev = sub20dev;
          std::cout << "Device Handle: " << OneSub20Configuration.handle_subdev << std::endl;
          std::cout << "Serial Number: " << OneSub20Configuration.strSub20Serial << std::endl;

          // Push element onto list of subdevices
          Sub20Device_list.push_back( OneSub20Configuration );
          std::cout << "Publishing to topic /bma180." << std::endl;
        }
        break;
      }
      else
      {
        sub_close( sub20handle );
      }
    }
    // find next device, sub_find_devices using current provides next
    sub20dev = sub_find_devices( sub20dev );
  }
};

Bma180::~Bma180()
{
  int error_code;
  OneSub20Config OneSub20Configuration;

  // close opened subdevices

  // Disable SPI
  error_code = sub_spi_config( subhndl, 0, 0 );

  // Close USB device
  sub_close( subhndl );
  while( !Sub20Device_list.empty() )
  {
    OneSub20Configuration = Sub20Device_list.back();
    std::cout << "Sub device removed " << OneSub20Configuration.strSub20Serial << std::endl;
    Sub20Device_list.pop_back();
  }
};

void Bma180::GetMeasurements( std::list<OneBma180Meas> &list_meas )
{
  char              chACC_XYZ[7]; //Containing
  double            dAccX, dAccY, dAccZ;
  double            dEstBiasX, dEstBiasY, dEstBiasZ;
  int               uiBcorrX, uiBcorrY, uiBcorrZ, uiRawX, uiRawY, uiRawZ;
  double            dTemp;
  int               error_code, dummy, j = 0;
  bool              SPIMeas = false;
  OneBma180Meas     sMeas;
  //ros::Time         dtomeas;
  char              chCMD;
  std::stringstream ss_errmsg;
  int               chip_select;
  std::list<OneSub20Config>::iterator iterat;
  
  // verify that a subdevice is connected
  if( 1 > (int)Sub20Device_list.size() )
  {
    throw std::string( "No SubDevice connected OR access rights to USB not given" );
  }

  // Trace sub20 list
  for( iterat = Sub20Device_list.begin(); iterat != Sub20Device_list.end(); iterat++ )
  {
    // verify if the sub20 device is configured (which should be the case as only configured sub20ies are pushed on list)
    if( iterat->bSubSPIConfigured == true)
    {
      // Trace through cluster
      for( chip_select = 0; chip_select < bma180_cmd::MAX_NUM_SENSORS; chip_select++ )
      {
      // verify if sensor is available on respective chipselect
        if( iterat->Bma180Cluster[chip_select].bConfigured == true )
	{
          error_code = 0;
          // NOTE: Not executed in order to save one SPI transfer
          // //ensure CS is high
          // //error_code = sub_spi_transfer( iterat->handle_subdev, 0, &chCMD, 1, SS_CONF(chip_select,SS_H) );
          // //send read command
          chCMD = bma180_cmd::ADDRESS_ACCLXYZ | bma180_cmd::FLAG_R;
          error_code += sub_spi_transfer( iterat->handle_subdev, &chCMD, 0, 1, SS_CONF(chip_select,SS_L) );
          error_code += sub_spi_transfer( iterat->handle_subdev, 0, chACC_XYZ, 7, SS_CONF(chip_select,SS_L) );
          // ensure CS is high
          error_code += sub_spi_transfer( iterat->handle_subdev, 0, &chCMD, 1, SS_CONF(chip_select,SS_H) );

          if( error_code == 0 )
	  {
            SPIMeas = true;
            dAccX = bma180data_to_double(chACC_XYZ[1], chACC_XYZ[0], bma180_cmd::eACCEL, &uiRawX, iterat->Bma180Cluster[chip_select].dFullScaleRange );
            dAccY = bma180data_to_double(chACC_XYZ[3], chACC_XYZ[2], bma180_cmd::eACCEL, &uiRawY, iterat->Bma180Cluster[chip_select].dFullScaleRange );
            dAccZ = bma180data_to_double(chACC_XYZ[5], chACC_XYZ[4], bma180_cmd::eACCEL, &uiRawZ, iterat->Bma180Cluster[chip_select].dFullScaleRange );
            dTemp = bma180data_to_double(chACC_XYZ[6], chACC_XYZ[6], bma180_cmd::eTEMP,  &dummy, iterat->Bma180Cluster[chip_select].dFullScaleRange );

            // Collect data for all chipSelects
            sMeas.dAccX[j] = dAccX;
            sMeas.dAccY[j] = dAccY;
            sMeas.dAccZ[j] = dAccZ;
            sMeas.dTemp    = dTemp;
            sMeas.chip_select[j] = chip_select;
            j++;
            sMeas.iNumAccels = j;

            // Execute calibration if desired
            // Note: For calibration procedure, only one XDIMAX box is allowed to be connected
            if( bExecuteCalibration == true && 2 > (int)Sub20Device_list.size() )  
	    {
              if( chip_select == iCalib_CS ) 
	      {
                if( calib_bma180.calibsens_completed() == false && calib_bma180.calibsens_set() == false )
		{
                  std::cout << " Bias read from image: " << iterat->Bma180Cluster[chip_select].uiBiasImageX << " " << iterat->Bma180Cluster[chip_select].uiBiasImageY << " " << iterat->Bma180Cluster[chip_select].uiBiasImageZ << std::endl;
                  // set the sensor to be calibrated
                  calib_bma180.setcalibsens( sMeas );
                }
                else
		{
                  calib_bma180.setdata_bma180( sMeas );
                }
                if( calib_bma180.calibsens_completed() && !calib_bma180.verification_active() )
		{
                  // Get estimated sensor biases

                  calib_bma180.get_estbiases(&dEstBiasX, &dEstBiasY, &dEstBiasZ);
                  std::cout << "estimated biases : " << dEstBiasX << " " << dEstBiasY << " " << dEstBiasZ << std::endl;

                  // calculate adjustment
                  // +0.5 for typecast to get propper rounding
                  // 2048: Bias has 12 bit, thus halfrange is 2^11 where as bias corr scales by range
                  int iBcorrX, iBcorrY, iBcorrZ;
                  iBcorrX = (short)( dEstBiasX / iterat->Bma180Cluster[chip_select].dFullScaleRange * 2048 + 0.5 );
                  iBcorrY = (short)( dEstBiasY / iterat->Bma180Cluster[chip_select].dFullScaleRange * 2048 + 0.5 ) ;
                  iBcorrZ = (short)( (dEstBiasZ - 1) / iterat->Bma180Cluster[chip_select].dFullScaleRange * 2048 + 0.5 ) ;
                  
		  std::cout << "Bias adjustments [X Y Z] " << iBcorrX << " " << iBcorrY << " " << iBcorrZ << std::endl;
                  std::cout << "Image values before adjustments " << iterat->Bma180Cluster[chip_select].uiBiasImageX << " " << iterat->Bma180Cluster[chip_select].uiBiasImageY << " " << iterat->Bma180Cluster[chip_select].uiBiasImageZ << std::endl;
                  uiBcorrX = (unsigned short)( iterat->Bma180Cluster[chip_select].uiBiasImageX - ((short)( dEstBiasX / bma180_cmd::dFULLRANGELOOKUP[6] * 2048 )) );
                  uiBcorrY = (unsigned short)( iterat->Bma180Cluster[chip_select].uiBiasImageY - ((short)( dEstBiasY / bma180_cmd::dFULLRANGELOOKUP[6] * 2048 )) );
                  uiBcorrZ = (unsigned short)( iterat->Bma180Cluster[chip_select].uiBiasImageZ - ((short)( (dEstBiasZ - 1) / bma180_cmd::dFULLRANGELOOKUP[6] * 2048 )) );

                  std::cout << "corrected biases " << uiBcorrX << " " << uiBcorrY << " " << uiBcorrZ << std::endl;

                  // Write to image
                  if( !set_biassettings( iterat->handle_subdev, chip_select, uiBcorrX, uiBcorrY, uiBcorrZ, false) )
		  {
                    bExecuteCalibration = false;
		    throw std::string( "Bias corrections cannot be written to image. Calibration procedure aborted." );
                  }

                  // Activate verification
                  if( !calib_bma180.biasverify() )
		  {
                    bExecuteCalibration = false;
                    throw std::string( "Bias verification cannot be executed. Calibration procedure aborted." );
                  }
                }

                if( calib_bma180.calibsens_completed() && calib_bma180.verification_active() && calib_bma180.verification_completed() )
		{
                  std::cout << "Verifying biases..." << std::endl;

                  // Get verified biases after image write
                  if( !calib_bma180.get_verifiedbiases( &dEstBiasX, &dEstBiasY, &dEstBiasZ ) )
		  {
                    bExecuteCalibration = false;
                    throw std::string( "Verified biases cannot be retrieved. Calibration procedure aborted." );
                  };

                  if( bma180_cmd::dCALIB_ACCURACY > fabs(dEstBiasX) && bma180_cmd::dCALIB_ACCURACY > fabs(dEstBiasY) && bma180_cmd::dCALIB_ACCURACY > fabs( dEstBiasZ - 1 ) )
		  {
                    // Writing EEPROM
                    uiBcorrX = 0;
                    uiBcorrY = 0;
                    uiBcorrZ = 0;
                    std::string myAnswer;
                    std::cin.ignore();
                    std::cout << "Really write EEPROM <Y/N>? ";
                    std::getline( std::cin, myAnswer );
                    if( 0 == myAnswer.find('Y') || 0 == myAnswer.find('y') )
		    {
                      if( !set_biassettings( iterat->handle_subdev, chip_select, uiBcorrX, uiBcorrY, uiBcorrZ, true ) )
		      {
                        bExecuteCalibration = false;
                        throw std::string( "Biases cannot be written into EEPROM. Calibration procedure aborted.");
                      };
                      ROS_INFO("EEPROM has been written.\n");

                    }
                    else
		    {
                      ROS_INFO("EEPROM write aborted.\n");
                    }
                  }
                  else
		  {
                    std::cout << "Error values " << fabs(dEstBiasX) << " " << fabs(dEstBiasY) << " " << fabs(dEstBiasZ-1) << std::endl;
                    bExecuteCalibration = false;
                    throw std::string( "Bias verification failed because the calibration values are too large. Calibration procedure aborted.");
                  }
                  // We are done with calibration
                  bExecuteCalibration = false;
                  std::cout << "Calibration completed successfully." << std::endl;
                }
              }
            }
            else
	    {
              if( bExecuteCalibration == true && 1 < (int)Sub20Device_list.size() )
	      {
                throw std::string( "As a pre-caution, please connect only one Sub20 device during calibration.");
              }
            }
          }
          else
	  {
            throw std::string( "There has been an SPI communication error. Continuing attempts to communicate with the device will be made." );
          }
        } // config verify
      } // scanning through all the chipselects
    }

    // Only publish data after all data for all chipSelects is collected
    if( SPIMeas )
    {
      //sMeas.dtomeas = ros::Time::now();
      sMeas.bMeasAvailable = true;
      sMeas.serial_number = iterat->strSub20Serial;
      
      // Push measurement onto heap
      list_meas.push_back( sMeas );
    }
    else
    {
      throw std::string( "SUB20 connected but not configured. A restart of the ROS node is suggested.");
    }
  } // sub20 for loop
};

double Bma180::bma180data_to_double( char chMSB, char chLSB, bma180_cmd::eSensorType eSensor, int *raw, double dAccScale )
{
  short	s16int;	      // 2 byte int to build message
  double dSensorMeas; // Scaled sensor measurement

  switch( eSensor )
  {
    // Retrieve Accel Data
    case bma180_cmd::eACCEL:
      // verify if pos or neg
      if( (chMSB & 0x80) == 0 )
      {
        // positive number
        s16int = (short)( (((unsigned short)chMSB) & 0xFF) << 6) + ((((unsigned short)chLSB) & 0xFC) >> 2 );
      }
      else
      {
        // negative number
        // set MSB (sign) to zero and build 2 complement unsigned; offset 8192 for 2^13
        s16int = (short)( ((((unsigned short)chMSB) & 0x7F ) << 6 ) + ((((unsigned short)chLSB) & 0xFC) >> 2 ) - 8192 );
      }
      dSensorMeas = ( ((double)s16int) / 8192 ) * dAccScale;
      break;

      // Convert Temperature Data
  case bma180_cmd::eTEMP:
    // verify if pos or neg
    if( (chLSB & 0x80) == 0 )
    {
      // positive number
      s16int = (short)( ((unsigned short)chLSB) & 0xFF );
    }
    else
    {
      // negative number
      // set MSB (sign) to zero and build 2 complement unsigned; offset 128 for 2^7
      s16int = (short)( (((unsigned short)chLSB) & 0x7F ) - 128 );
    };
    dSensorMeas = ( ( (double)s16int) / 8192 ) * 2;
    break;

  default :
    dSensorMeas = 0;
  }
  *raw = s16int;
  return (dSensorMeas);
};

unsigned short Bma180::bma180data_to_uint( char chMSB, char chLSB, bma180_cmd::eSensorType eSensor )
{
  unsigned short u16int_bias; // 2 byte int to build message
  switch( eSensor )
  {
  // Bias X axis data
  case bma180_cmd::eBIAS_X:
    u16int_bias = (short) ((((unsigned short)chMSB)&0xFF)<<4)+((((unsigned short)chLSB)&0xF0)>>4);
    break;

  // Bias Y axis data
  case bma180_cmd::eBIAS_Y:
    u16int_bias = (short) ((((unsigned short)chMSB)&0xFF)<<4)+((((unsigned short)chLSB)&0x0F));
    break;

  // Bias Z axis data
  case bma180_cmd::eBIAS_Z:
    u16int_bias = (short) ((((unsigned short)chMSB)&0xFF)<<4)+((((unsigned short)chLSB)&0xF0)>>4);
    break;
  default :
    u16int_bias = 0;
  }
  return (u16int_bias);
};

bool Bma180::read_byte_eeprom_sub20( char chADR, char* pchREGISTER, unsigned short chip_select, sub_handle sub_h )
{
  int     error_code = 0;
  bool    bSuccess;
  char    chCMD;

  // Read register at specified adderss chADR

  // ensure CS is high
  error_code += sub_spi_transfer( sub_h, 0, &chCMD, 1, SS_CONF(chip_select,SS_H) );
  // send read command
  chCMD  	= chADR | bma180_cmd::FLAG_R;
  error_code += sub_spi_transfer( sub_h, &chCMD, 0, 1, SS_CONF(chip_select,SS_L) );
  error_code += sub_spi_transfer( sub_h, 0, pchREGISTER, 1, SS_CONF(chip_select,SS_L) );
  // ensure CS is high
  error_code += sub_spi_transfer( sub_h, 0, &chCMD, 1, SS_CONF(chip_select,SS_H) );

  if (error_code == 0)
  {
    // successful operation
    bSuccess = true;
  }
  else
  {
    // SPI error
    bSuccess = false;
  }
  return (bSuccess);
};

bool Bma180::write_bit_eeprom_sub20( char chADR, unsigned short iLSBOfByte, unsigned short iNumOfBits, char chBitSeq, unsigned short chip_select, sub_handle sub_h )
{
  char chCMD;      // temporary character for building commands
  char chREGISTER; // temporary read status of register
  char chREGISTER_MODIFIED;
  int  error_code;
  char chMask;
  bool bSuccess;
  bool bEEReadSuccess;

  // only one byte will be handled, thus iLSBOfByte + iNumOfBits > 8 means out of range
  if ( iLSBOfByte + iNumOfBits <= 8 )
  {
    error_code = 0;
    bEEReadSuccess = true;
 
    // Read register at specified address chADR
    chCMD = chADR | bma180_cmd::FLAG_R;
    bEEReadSuccess &= read_byte_eeprom_sub20(chCMD, &chREGISTER, chip_select, sub_h );

    // Write the specified bit into register
    // Build mask, i.e. zero pad the respective points where we need to have
    if ( iNumOfBits < 8 )
    {
      chMask = ~( ((1 << iNumOfBits) - 1) << iLSBOfByte );
    }
    else
    {
      // as function of the range check above, it can only be 8
      chMask = 0x00;
    }
    // write specific register
    chREGISTER_MODIFIED = ( chREGISTER & chMask ) | ( chBitSeq << iLSBOfByte );
    // build up command to be written to register (note read flag is not set
    chCMD  = chADR;
    error_code += sub_spi_transfer( sub_h, &chCMD, 0, 1, SS_CONF(chip_select,SS_L) );
    error_code += sub_spi_transfer( sub_h, &chREGISTER_MODIFIED, 0, 1, SS_CONF(chip_select,SS_L) );
    // ensure CS is high
    error_code += sub_spi_transfer( sub_h, 0, &chCMD, 1, SS_CONF(chip_select,SS_H) );

    // Check if register has been set (only verification)

    // send read command
    chCMD = chADR | bma180_cmd::FLAG_R;
    bEEReadSuccess &= read_byte_eeprom_sub20(chCMD, &chREGISTER, chip_select, sub_h);

    if( (error_code == 0) && (bEEReadSuccess) )
    {
      bSuccess = true;
    }
    else
    {
      // SPI communication error
      bSuccess = false;
    }
  }
  else
  {
    // out of range
    bSuccess = false;
  }
  return bSuccess;
};

void Bma180::confsens_on_sub20( OneSub20Config *pOneSub20Conf, char chFullscale, char chBandwidth )
 {
  int               error_code;
  int               chip_select;
  bool              bSuccess;
  char              chCHIP_ID;
  char              chALML_VER;
  char              chCMD;
  char              chREGSTATUS;
  unsigned short    uiBiasX;
  unsigned short    uiBiasY;
  unsigned short    uiBiasZ;
  std::stringstream ss_errmsg;
  sub_handle        handle_sub20;
  bool              bEE_RWsuccess;

  // retrieve handle for sub20 device
  handle_sub20 = pOneSub20Conf->handle_subdev;
  for( chip_select = 0; chip_select < bma180_cmd::MAX_NUM_SENSORS; chip_select++ )
  {
    pOneSub20Conf->Bma180Cluster[chip_select].iNumOfCalibMeas = 0;
    error_code = 0;
    bEE_RWsuccess = true;
    // ensure CS is high
    error_code += sub_spi_transfer( handle_sub20, 0, &chCMD, 1, SS_CONF(chip_select,SS_H) );
    // send read command
    chCMD = bma180_cmd::ADDRESS_VER | bma180_cmd::FLAG_R;
    error_code += sub_spi_transfer( handle_sub20, &chCMD, 0, 1, SS_CONF(chip_select,SS_L) );
    error_code += sub_spi_transfer( handle_sub20, 0, &chCHIP_ID, 1, SS_CONF(chip_select,SS_L) );
    error_code += sub_spi_transfer( handle_sub20, 0, &chALML_VER, 1, SS_CONF(chip_select,SS_L) );
    // ensure CS is high
    error_code += sub_spi_transfer( handle_sub20, 0, &chCMD, 1, SS_CONF(chip_select,SS_H) );

    // Verify if a BMA180 is connected on the respective ChipSelect
    if( error_code == 0 && 0 < (unsigned short)chCHIP_ID && 0 < (unsigned short)chALML_VER )
    {
      ROS_INFO("BMA 180 - Chip Select %d\n", chip_select);
      ROS_INFO("CHIP ID: %u\n", (unsigned short)chCHIP_ID);
      ROS_INFO("ALML Ver: %u\n", (unsigned short)chALML_VER);

      // call method to set ee_write flag
      bEE_RWsuccess &= write_bit_eeprom_sub20( bma180_cmd::ADDRESS_CTRLREG0, 4, 1, bma180_cmd::COMMAND_SET_EEW, chip_select, handle_sub20 );
      // issue soft reset
      bEE_RWsuccess &= write_bit_eeprom_sub20( bma180_cmd::ADDRESS_SOFTRESET, 0, 8, bma180_cmd::COMMAND_SOFTRESET, chip_select, handle_sub20 );
      // call method to set ee_write flag
      bEE_RWsuccess &= write_bit_eeprom_sub20( bma180_cmd::ADDRESS_CTRLREG0, 4, 1, bma180_cmd::COMMAND_SET_EEW, chip_select, handle_sub20 );
      // change maxrange of sensor
      bEE_RWsuccess &= write_bit_eeprom_sub20( bma180_cmd::ADDRESS_RANGE, 1, 3, chFullscale, chip_select, handle_sub20 );
      bEE_RWsuccess &= read_byte_eeprom_sub20( bma180_cmd::ADDRESS_RANGE, &chREGSTATUS, chip_select, handle_sub20 );
      // Extract range
      pOneSub20Conf->Bma180Cluster[chip_select].dFullScaleRange = bma180_cmd::dFULLRANGELOOKUP[((unsigned short)((chREGSTATUS & 0x0E)>>1))];
      // change sensor bandwidth
      bEE_RWsuccess &= write_bit_eeprom_sub20( bma180_cmd::ADDRESS_BWTCS, 4, 4, chBandwidth, chip_select, handle_sub20 );
      bEE_RWsuccess &= read_byte_eeprom_sub20( bma180_cmd::ADDRESS_BWTCS, &chREGSTATUS, chip_select, handle_sub20 );
      // Extract range
      pOneSub20Conf->Bma180Cluster[chip_select].dSensorBandwidth = bma180_cmd::dBWLOOKUP[((unsigned short)((chREGSTATUS & 0xF0)>>4))];
      ROS_INFO("EEPROM stored Fullrange: %f [g]\n", pOneSub20Conf->Bma180Cluster[chip_select].dFullScaleRange);
      ROS_INFO("EEPROM stored Bandwidth: %f [Hz]\n", pOneSub20Conf->Bma180Cluster[chip_select].dSensorBandwidth);

      // Read bias that are currently stored in image
      bEE_RWsuccess &= read_biassettings( handle_sub20, chip_select, &uiBiasX, &uiBiasY, &uiBiasZ );
      pOneSub20Conf->Bma180Cluster[chip_select].uiBiasImageX = uiBiasX;
      pOneSub20Conf->Bma180Cluster[chip_select].uiBiasImageY = uiBiasY;
      pOneSub20Conf->Bma180Cluster[chip_select].uiBiasImageZ = uiBiasZ;

      if( bEE_RWsuccess )
      {
        // Set flag that sensor has been initialized
        pOneSub20Conf->Bma180Cluster[chip_select].bConfigured = true;
        // successful operation
        bSuccess = true;
      }
      else
      {
        // Set flag that sensor has been initialized
        pOneSub20Conf->Bma180Cluster[chip_select].bConfigured = false;
        // unsuccessful operation
        bSuccess = false;
      }
    }
    else
    {
      ROS_INFO("BMA 180 %d: No sensor detected\n", chip_select);

      // Set flag that sensor hasn't been initialized
      pOneSub20Conf->Bma180Cluster[chip_select].bConfigured = false;
      pOneSub20Conf->Bma180Cluster[chip_select].dFullScaleRange = 0;
      pOneSub20Conf->Bma180Cluster[chip_select].dSensorBandwidth = 0;
      pOneSub20Conf->Bma180Cluster[chip_select].uiBiasImageX = 0;
      pOneSub20Conf->Bma180Cluster[chip_select].uiBiasImageY = 0;
      pOneSub20Conf->Bma180Cluster[chip_select].uiBiasImageZ = 0;
      // unsuccessful operation
      bSuccess = false;
    }
  }
};

bool Bma180::read_biassettings( sub_handle handle_sub20, int chip_select, unsigned short* uiBiasX, unsigned short* uiBiasY, unsigned short* uiBiasZ )
{
  int  error_code;
  char chBiasXYZT[6];
  bool bSuccess;
  char chCMD;
  char chCHIP_ID;
  char chALML_VER;

  // Ensure BMA180 is connected

  // ensure CS is high
  error_code = 0;
  error_code += sub_spi_transfer( handle_sub20, 0, &chCMD, 1, SS_CONF(chip_select,SS_H) );
  // send read command
  chCMD   = bma180_cmd::ADDRESS_VER|bma180_cmd::FLAG_R;
  error_code += sub_spi_transfer( handle_sub20, &chCMD, 0, 1, SS_CONF(chip_select,SS_L) );
  error_code += sub_spi_transfer( handle_sub20, 0, &chCHIP_ID, 1, SS_CONF(chip_select,SS_L) );
  error_code += sub_spi_transfer( handle_sub20, 0, &chALML_VER, 1, SS_CONF(chip_select,SS_L) );
  // ensure CS is high
  error_code += sub_spi_transfer( handle_sub20, 0, &chCMD, 1, SS_CONF(chip_select,SS_H) );

  // Verify if a BMA180 is connected on the respective chip select line
  if( (error_code == 0) && ( 0 < (unsigned short)chCHIP_ID) && ( 0 < (unsigned short)chALML_VER) ) 
  {
    // ensure CS is high
    // NOTE: Not executed in order to save one SPI transfer
    //error_code = sub_spi_transfer( iterat->handle_subdev, 0, &chCMD, 1, SS_CONF(chip_select,SS_H) );
    // send read command
    chCMD = bma180_cmd::ADDRESS_OFFSET_LSB1 | bma180_cmd::FLAG_R;
    error_code += sub_spi_transfer( handle_sub20, &chCMD, 0, 1, SS_CONF(chip_select,SS_L) );
    error_code += sub_spi_transfer( handle_sub20, 0, chBiasXYZT, 6, SS_CONF(chip_select,SS_L) );
    // ensure CS is high
    error_code += sub_spi_transfer( handle_sub20, 0, &chCMD, 1, SS_CONF(chip_select,SS_H) );

    if( error_code == 0 )
    {
      (*uiBiasX) = bma180data_to_uint( chBiasXYZT[3], chBiasXYZT[0], bma180_cmd::eBIAS_X );
      (*uiBiasY) = bma180data_to_uint( chBiasXYZT[4], chBiasXYZT[1], bma180_cmd::eBIAS_Y );
      (*uiBiasZ) = bma180data_to_uint( chBiasXYZT[5], chBiasXYZT[1], bma180_cmd::eBIAS_Z );
      bSuccess = true;
    }
    else
    {
      (*uiBiasX) = 0;
      (*uiBiasY) = 0;
      (*uiBiasZ) = 0;
      bSuccess = false;
    }
  }
  else
  {
    (*uiBiasX) = 0;
    (*uiBiasY) = 0;
    (*uiBiasZ) = 0;
    bSuccess = false;
  }
  return bSuccess;
};

bool Bma180::set_biassettings( sub_handle handle_sub20, int chip_select, unsigned short uiBiasX, unsigned short uiBiasY, unsigned short uiBiasZ, bool bWriteEEPROM )
{
  int  error_code; 		//Error code for Sub20 API
  bool bSuccess;
  char chCMD;
  char chCHIP_ID;
  char chALML_VER;
  char chMSB;
  char chLSB;
  bool bEE_RWsuccess;

  // Ensure BMA180 is connected

  // ensure CS is high
  error_code = 0;
  bEE_RWsuccess = true;
  error_code += sub_spi_transfer( handle_sub20, 0, &chCMD, 1, SS_CONF(chip_select,SS_H) );
  
  // send read command
  chCMD = bma180_cmd::ADDRESS_VER | bma180_cmd::FLAG_R;
  error_code += sub_spi_transfer( handle_sub20, &chCMD, 0, 1, SS_CONF(chip_select,SS_L) );
  error_code += sub_spi_transfer( handle_sub20, 0, &chCHIP_ID, 1, SS_CONF(chip_select,SS_L) );
  error_code += sub_spi_transfer( handle_sub20, 0, &chALML_VER, 1, SS_CONF(chip_select,SS_L) );

  // ensure CS is high
  error_code += sub_spi_transfer( handle_sub20, 0, &chCMD, 1, SS_CONF(chip_select,SS_H) );

  // Verify if a BMA180 is connected on the respective ChipSelect
  if( error_code == 0 && ( 0 < (unsigned short) chCHIP_ID) && ( 0 < (unsigned short)chALML_VER) )
  {
    if( !bWriteEEPROM == true)
    {
      // Write X axis offset
      chMSB = (char)( (uiBiasX&0xFF0) >> 4 );
      chLSB = (char)( (uiBiasX&0x0F) );
      bEE_RWsuccess &= write_bit_eeprom_sub20( bma180_cmd::ADDRESS_OFFSET_X, 0, 8, chMSB, chip_select, handle_sub20 );
      bEE_RWsuccess &= write_bit_eeprom_sub20( bma180_cmd::ADDRESS_OFFSET_LSB1, 4, 4, chLSB, chip_select, handle_sub20 );

      // Write Y axis offset
      chMSB = (char)( (uiBiasY&0xFF0) >> 4 );
      chLSB = (char)( (uiBiasY&0x0F));
      bEE_RWsuccess &= write_bit_eeprom_sub20( bma180_cmd::ADDRESS_OFFSET_Y, 0, 8, chMSB, chip_select, handle_sub20 );
      bEE_RWsuccess &= write_bit_eeprom_sub20( bma180_cmd::ADDRESS_OFFSET_LSB2, 0, 4, chLSB, chip_select, handle_sub20 );

      // Write Z axis offset
      chMSB = (char)( (uiBiasZ&0xFF0) >> 4 );
      chLSB = (char)( (uiBiasZ&0x0F) );
      bEE_RWsuccess &= write_bit_eeprom_sub20( bma180_cmd::ADDRESS_OFFSET_Z, 0, 8, chMSB, chip_select, handle_sub20 );
      bEE_RWsuccess &= write_bit_eeprom_sub20( bma180_cmd::ADDRESS_OFFSET_LSB2, 4, 4, chLSB, chip_select, handle_sub20 );

      if( bEE_RWsuccess )
      {
        std::cout << "Writing to image " << std::endl;
        bSuccess = true;
      }
      else
      {
        bSuccess = false;
        ROS_WARN("Cannot write BMA180 image.\n");
      }
    }
    else
    {
      // Writing to the EEPROM is done by indirect write, i.e. setting values to image register
      // and then writing to the eeprom, which in fact reads the image register
      // Per write 16 bit from image to eeprom are written and only even addresses can used (see docu)
      bEE_RWsuccess &= write_bit_eeprom_sub20(bma180_cmd::ADDRESS_EE_GAIN_Z, 0, 8, 0x00, chip_select, handle_sub20 );
      bEE_RWsuccess &= write_bit_eeprom_sub20(bma180_cmd::ADDRESS_EE_OFFSET_LSB2, 0, 8, 0x00, chip_select, handle_sub20 );
      bEE_RWsuccess &= write_bit_eeprom_sub20(bma180_cmd::ADDRESS_EE_OFFSET_X, 0, 8, 0x00, chip_select, handle_sub20 );
      bEE_RWsuccess &= write_bit_eeprom_sub20(bma180_cmd::ADDRESS_EE_OFFSET_Z, 0, 8, 0x00, chip_select, handle_sub20 );

      if( bEE_RWsuccess )
      {
        ROS_INFO("BMA180 EEPROM written.\n");
        bSuccess = true;
      }
      else
      {
        bSuccess = false;
      }
    }
  }
  else 
  {
    // unsuccessful operation
    bSuccess = false;
  }
  return bSuccess;
};


void Bma180::set_calibstatus( bool bCalibrate)
{
  if( bCalibrate == true )
  {
    std::string myAnswer;
    std::cout << "Sensors shall be calibrated <Y/N>? ";
    std::getline( std::cin, myAnswer );
    if( (0 == myAnswer.find('Y')) || (0 == myAnswer.find('y')) )
    {
      unsigned short myCS;
      std::cout << "Specify ChipSelect <0..4>? ";
      if( std::cin >> myCS )
      {
        if( myCS < bma180_cmd::MAX_NUM_SENSORS )
	{
          bExecuteCalibration = true;
          bCalibrationCompleted = false;
          iCalib_CS = myCS;
        }
        else
	{
          bExecuteCalibration = false;
          bCalibrationCompleted = true;
        }
      }
      else
      {
        bExecuteCalibration = false;
        bCalibrationCompleted = true;
        std::cin.clear();
      }
    }
    else
    {
      bExecuteCalibration = false;
      bCalibrationCompleted = true;
    }
  }
  else
  {
    bExecuteCalibration = false;
    bCalibrationCompleted = true;
  }
};




// main loop

int main(int argc, char **argv)
{
  int count = 1;
  int i = 0;


  // Parameter Server values
  double 	max_acceleration_g, dRate_Hz, dBandwidth_Hz;
  bool 		bCalibrate;
  std::string   sSub20Serial;

  // Set initialization values
  max_acceleration_g = 2.0;
  dBandwidth_Hz = 100; 
  dRate_Hz = 100;
  bCalibrate = false;
  sSub20Serial = "0651";

  Bma180 bma180_attached( max_acceleration_g, dBandwidth_Hz, bCalibrate, dRate_Hz, sSub20Serial );
  OneBma180Meas sOneMeas;
  bma180::bma180meas msg;
  std::list<OneBma180Meas> measurements_list;


  // Run sensor node
  // some sort of rate setup here
  while( true )
  {
    // initiate a measurement
    try
    {
      bma180_attached.GetMeasurements(measurements_list);
    }

    catch( std::string e )
    {
      ROS_ERROR( "An exception occurred: %s\n", e.c_str() );
      std::exit(1);
    }

    while( !measurements_list.empty() ) 
    {
      sOneMeas = measurements_list.back();
      measurements_list.pop_back();

      // only publish if a successful measurement was received
      if( sOneMeas.bMeasAvailable == true )
      {
        msg.strIdSubDev = sOneMeas.serial_number;

        // Print results to screen
        for( i = 0; i < sOneMeas.iNumAccels; i++ )
	{
	  printf("BMA180 %d\tx: %1.4f\ty: %1.4f\tz: %1.4f\n", sOneMeas.chip_select[i], sOneMeas.dAccX[i], sOneMeas.dAccY[i], sOneMeas.dAccZ[i] );
        }
      }
    }

    //loop_rate.sleep();
    usleep( 1e6 / dRate_Hz );

    ++count;
  }
}


