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
 * Smi540.cpp
 *
 *  Created on: Jul 27, 2011
 *      Author: Lucas Marti, Robert Bosch LLC |
 *      Author: Nikhil Deshpande, Robert Bosch LLC |
 */

#include <iostream>
#include <string>
#include <ros/ros.h>
#include "smi540/smi540.h"
#include <sstream>
#include <ros/time.h>
#include <math.h>
#include <vector>

Smi540::Smi540(std::string sSub20Serial):
    bSubDeviceOpen (false),
    bSubDeviceConfigured (false),
    strSerial ("         ")
{
  int spi_error_code; //Error code for Sub20 API
  int iSPI_cfg_set; //Set SPI config
  int iSPI_cfg_get; //Get SPI config
  sub_device sub20dev;
  sub_handle sub20handle;
  std::cout << "---opening SUB device---" << " \n";
  std::string sub20serial;
  OneSub20Config    OneSub20Configuration;

  ///////////////////////////////////////////////////////////
  // Open connected Sub20 Devices
  ///////////////////////////////////////////////////////////
  sub20dev = 0;
  sub20dev = sub_find_devices(sub20dev);
  std::cout << "sub 20 Device:  " << sub20dev << std::endl;
  while( sub20dev != 0 )
  {
    sub20handle = sub_open(sub20dev);
    // on success, sub_open returns non-zero handle
    if( sub20handle == 0 )
    {
      //sub_open was not successful
      ROS_INFO("ERROR - Sub20 could not be opened: %s ", sub_strerror(sub_errno) );
    }
    else
    {
      //Subdevice successfully opened
      sub20serial.clear();
      sub20serial.resize( smi540_cmd::uSERIALNUMLENGTH );
      sub_get_serial_number(sub20handle, const_cast<char*>(sub20serial.c_str()), sub20serial.size());
      std::cout << "Serial Number   : " << sub20serial << std::endl;
      // Compare if the opened sub20 device is the one desired!
      if( strcmp(sub20serial.c_str(), sSub20Serial.c_str()) == 0 )
      {
        //////////////////////////////////////////////////////////
        // Configure Sub20 device
        //////////////////////////////////////////////////////////
        subhndl = sub20handle;
        bSubDeviceOpen = true;
        std::cout << "---Initializing SPI Interface---" << std::endl;
        /* Read current SPI configuration */
        spi_error_code = sub_spi_config( sub20handle, 0, &iSPI_cfg_get );
        std::cout << "Dev Sub config  : " << iSPI_cfg_get << " \n";
        //Important: The SPI interface does not work properly at higher frequencies
        iSPI_cfg_set = SPI_ENABLE|SPI_CPOL_RISE|SPI_SMPL_SETUP|SPI_MSB_FIRST|SPI_CLK_1MHZ;
        /* Configure SPI */
        spi_error_code = sub_spi_config( sub20handle, iSPI_cfg_set, 0 );
        /* Read current SPI configuration */
        spi_error_code = sub_spi_config( sub20handle, 0, &iSPI_cfg_get );

        //verify if sub20 device has accepted the configuration
        if( iSPI_cfg_get == iSPI_cfg_set )
	{
          bSubDeviceConfigured = true;
          std::cout<< "Configuration   : " << iSPI_cfg_set << " successfully stored \n";
          //Subdevice has been configured successfully
          OneSub20Configuration.bSubSPIConfigured = true;
          OneSub20Configuration.handle_subdev = sub20handle;
          /////////////////////////////////////////////////
          // Configure Sensors on Sub20
          /////////////////////////////////////////////////
          std::cout<< "Configuring SMI540 device" << std::endl;
          confsens_on_sub20( &OneSub20Configuration);
        }
	else
	{
          std::cout<< "ERROR - Configuration :" << iSPI_cfg_set << " not accept by device \n";
          //Subdevice could not be configured
          bSubDeviceConfigured = false;
          OneSub20Configuration.bSubSPIConfigured = false;
        }

        //only execute if SubDevice has accepted configuration
        if( OneSub20Configuration.bSubSPIConfigured )
	{
          // SS_CONF(0,SS_LO):
          // SS_N Chipselect 0;
          // SS_LO SS goes low and stays low during entire transfer, after that it goes high
          spi_error_code = sub_spi_transfer( sub20handle, smi540_cmd::chTRIGGER_RESET, 0, 4, SS_CONF(0,SS_LO) );
          strSerial.clear();
          strSerial.resize( smi540_cmd::uSERIALNUMLENGTH );
          sub_get_serial_number( sub20handle, const_cast<char*>(strSerial.c_str()), strSerial.size() );
          OneSub20Configuration.strSub20Serial = strSerial;
          OneSub20Configuration.subdev = sub20dev;
          std::cout << "Device Handle : " << OneSub20Configuration.handle_subdev << std::endl;
          std::cout << "Serial Number : " << OneSub20Configuration.strSub20Serial << std::endl;
          /////////////////////////////////////////////////
          // Push element onto list of subdevices
          /////////////////////////////////////////////////
          Sub20Device_list.push_back (OneSub20Configuration);
          std::cout << "... Publishing to topic /smi540 ... " << std::endl;
        }
        break;
      }
      else
      {
        sub_close( sub20handle );
      }
    }
    //find next device, sub_find_devices using current provides next
    sub20dev = sub_find_devices( sub20dev );
  }
};

Smi540::~Smi540()
{
  int spi_error_code;
  OneSub20Config OneSub20Configuration;
  // Disable SPI
  spi_error_code = sub_spi_config( subhndl, 0, 0 );
  // Close USB device
  sub_close( subhndl );
  //Set status
  bSubDeviceOpen = false;
  bSubDeviceConfigured = false;
  while( !Sub20Device_list.empty() )
  {
    OneSub20Configuration = Sub20Device_list.back();
    std::cout << "Sub device removed " << OneSub20Configuration.strSub20Serial << "\n";
    Sub20Device_list.pop_back();
  }
};


void Smi540::GetMeasurements( std::list<OneSmi540Meas> &list_meas )
{
  char           chMM5_rx[8], chCMD; //Containing
  int            spi_error_code, iChipSelect, j = 0;
  OneSmi540Meas  sMeas;
  unsigned short smi540health;
  double         dAccX, dAccY, dRateZ;
  bool           SPIMeas = false;
  std::list<OneSub20Config>::iterator iterat;

  //verify that a subdevice is connected
  if( 1 > (int)Sub20Device_list.size() )
  {
    throw std::string ("No SubDevice connected OR access rights to USB not given");
  }
  //Trace sub20 list
  for( iterat = Sub20Device_list.begin(); iterat != Sub20Device_list.end(); iterat++ )
  {
    //verify if the sub20 device is configured (which should be the case as only configured sub20ies are pushed on list)
    if( iterat->bSubSPIConfigured == true )
    {
      //Trace through cluster
      for( iChipSelect = 0; iChipSelect < smi540_cmd::MAX_SENSORS; iChipSelect++ )
      {
        //verify if sensor is available on respective chipselect
        if( iterat->Smi540Cluster[iChipSelect].bConfigured == true )
	{
          spi_error_code = 0;
          // SS_N iChipSelect;
          // SS_LO SS goes low and stays low during entire transfer, after that it goes high
          spi_error_code += sub_spi_transfer( iterat->handle_subdev, smi540_cmd::chRD_ACT_DATA_64, 0, 4, SS_CONF(iChipSelect,SS_LO) );
	  if( spi_error_code != 0 )
	  {
	    ROS_WARN("SPI transfer result: %d\tCS: %d, ", spi_error_code, iChipSelect);
	  }
          spi_error_code += sub_spi_transfer( iterat->handle_subdev, 0, chMM5_rx, 8, SS_CONF(iChipSelect,SS_LO) );
          //ensure CS is high
          //spi_error_code += sub_spi_transfer( iterat->handle_subdev, 0, &chCMD, 1, SS_CONF(iChipSelect,SS_H) );
          if( spi_error_code == 0 )
	  {
            SPIMeas = true;
            //Convert the response into scaled sensor measurements
            dRateZ = mm5data_to_double(chMM5_rx[5], chMM5_rx[6], smi540_cmd::eGYRO);
            dAccX  = mm5data_to_double(chMM5_rx[3], chMM5_rx[4], smi540_cmd::eACCEL);
            dAccY  = mm5data_to_double(chMM5_rx[1], chMM5_rx[2], smi540_cmd::eACCEL);

            // collect the data for all available chipselects
            sMeas.dAccX[j]       = dAccX;
            sMeas.dAccY[j]       = dAccY;
            sMeas.dRateZ[j]      = dRateZ;
            sMeas.iChipSelect[j] = iChipSelect;
            j++;
            sMeas.iNumAccels = j;
            //Extract status of channels
            smi540health = (unsigned short)( chMM5_rx[0] & 0x23 );
          }
          else
	  {
            throw std::string ("SPI transfer error");
          }
        }
      }
    }
    // only publish data once data for all available chipselects is collected
    if( SPIMeas )
    {
      sMeas.dtomeas        = ros::Time::now();
      sMeas.bMeasAvailable = true;
      sMeas.strSerial      = iterat->strSub20Serial;
      //Push measurement onto heap
      list_meas.push_back( sMeas );
    }
    else
    {
      throw std::string ("SUB20 connected but no access rights given!");
    }
  }
}

double Smi540::mm5data_to_double( char chMSB, char chLSB, smi540_cmd::eSensorType eSensor )
{
  short  s16int;      //2 byte int to build message
  double dSensorMeas; //Scaled sensor measurement
  //verify if pos or neg
  if( (chMSB & 0x80) == 0 )
  {
    //positive number
    s16int = (short) ((((unsigned short)chMSB)&0xFF)<<8) + (((unsigned short)chLSB)&0xFF);
  }
  else {
    //negative number
    //set MSB (sign) to zero and build 2 complement unsigned
    s16int = (short)(((((unsigned short)chMSB)&0x7F)<<8) + (((unsigned short)chLSB)&0xFF)-32768);
  }

  //Convert data with the respective Scale Factor
  if( eSensor == smi540_cmd::eACCEL )
  {
    dSensorMeas = ((double)s16int) / smi540_cmd::fSFACC_inv;
  }
  else
  {
    dSensorMeas = ((double)s16int) / smi540_cmd::fSFGYRO_inv;
  }
  return (dSensorMeas);
}

void Smi540::confsens_on_sub20( OneSub20Config *pOneSub20Conf )
{
  int        spi_error_code; //Error code for Sub20 API
  int        iChipSelect;
  sub_handle handle_sub20;
  char       chCMD[8];

  //retrieve handle for sub20 device
  handle_sub20 = pOneSub20Conf->handle_subdev;
  for( iChipSelect = 0; iChipSelect < smi540_cmd::MAX_SENSORS; iChipSelect++ )
  {
    spi_error_code = 0;
    // Ensure CS is high
    spi_error_code += sub_spi_transfer( handle_sub20, 0, &chCMD[0], 1, SS_CONF(iChipSelect,SS_H) );
    // SS_CONF(iChipSelect,SS_LO):
    // SS_N iChipSelect;
    // SS_LO SS goes low and stays low during entire transfer, after that it goes high
    spi_error_code += sub_spi_transfer( handle_sub20, smi540_cmd::chTRIGGER_RESET, 0, 4, SS_CONF(iChipSelect,SS_LO) );
    spi_error_code += sub_spi_transfer( handle_sub20, 0, &chCMD[0], 1, SS_CONF(iChipSelect,SS_H) );
    spi_error_code += sub_spi_transfer( handle_sub20, smi540_cmd::chRD_STATUS_A, 0, 4, SS_CONF(iChipSelect,SS_LO) );
    spi_error_code += sub_spi_transfer( handle_sub20, 0, chCMD, 8, SS_CONF(iChipSelect,SS_LO) );

    // Check if a sensor is connected on the specified iChipSelect
    if( (spi_error_code == 0) && (strlen(chCMD)<=1) )
    {
      ROS_INFO("-----------------------------------------");
      ROS_INFO("SMI 540 - Chip Select %d", iChipSelect);
      ROS_INFO("CONFIGURED");
      pOneSub20Conf->Smi540Cluster[iChipSelect].bConfigured = true;
    } else {
      ROS_INFO("-----------------------------------------");
      ROS_INFO("SMI 540 - Chip Select %d", iChipSelect);
      ROS_INFO("NO SENSOR DETECTED");
      pOneSub20Conf->Smi540Cluster[iChipSelect].bConfigured = false;
    }

  }
};

// ----------
// -- MAIN --
// ----------
int main( int argc, char **argv )
{
  double               dRate_Hz;
  std::string          sSub20Serial;
  ros::init(argc, argv, "smi540");
  ros::NodeHandle nh;
  ros::Publisher smi540_pub = nh.advertise<smi540::smi540meas>("smi540", 100);

  if( nh.getParam("/smi540/rate_Hz", dRate_Hz ) == false )
  {
    dRate_Hz = smi540_cmd::DEFAULT_RATE_Hz;
  };
  if(nh.getParam("/smi540/sub20serial", sSub20Serial) == false )
  {
    sSub20Serial = smi540_cmd::sSUB20SERIAL;
  };

  Smi540 smi540_attached(sSub20Serial);
  OneSmi540Meas sOneMeas;
  smi540::smi540meas msg;
  std::list<OneSmi540Meas>measurements_list;

  ros::Rate loop_rate(dRate_Hz);

  while( nh.ok() )
  {
    //try to poll one measurement from sensor
    try
    {
      smi540_attached.GetMeasurements( measurements_list );
    }
    catch ( std::string strType )
    {
      ROS_WARN_STREAM("An exception occurred: " << strType);
      //std::exit(1);
      continue;
    }

    while( !measurements_list.empty() )
    {
      sOneMeas = measurements_list.back();
      measurements_list.pop_back();
      //only publish if a successful measurement was received
      if( sOneMeas.bMeasAvailable == true )
      {
        msg.strIdSubDev = sOneMeas.strSerial;

        // collect all available chipselect values into one message
        for( int i = 0; i < sOneMeas.iNumAccels; i++ )
	{
          msg.iChipSelect.push_back( sOneMeas.iChipSelect[i] );
          msg.fAcclX.push_back( sOneMeas.dAccX[i] );
          msg.fAcclY.push_back( sOneMeas.dAccY[i] );
          msg.fRateZ.push_back( sOneMeas.dRateZ[i] );
        }
        msg.header.stamp = sOneMeas.dtomeas;
        smi540_pub.publish( msg );   // publish data in message

        // clear the message values after publishing to avoid accumulation of further values on subsequent messages.
        msg.iChipSelect.clear();
        msg.fAcclX.clear();
        msg.fAcclY.clear();
        msg.fRateZ.clear();
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

