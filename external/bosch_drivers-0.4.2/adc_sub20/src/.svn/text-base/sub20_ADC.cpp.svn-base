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
 * adc_sub20.cpp
 *
 *  Created on: Jun 12, 2011
 *      Author: Nikhil Deshpande, Robert Bosch LLC|
 */

#include <iostream>
#include <string>
#include "ros/ros.h"
#include <sstream>
#include "ros/time.h"
#include <math.h>
#include "adc_sub20/sub20_ADC.h"
#include <vector>
#include <map>

Sub20_ADC::Sub20_ADC(std::string ADC_MUXCONFIG, double dRate_Hz, double dVRef):
	bSubDeviceOpen        (false),
	bSubDeviceConfigured  (false)
{
	// TODO Auto-generated constructor stub
	int               iADCErr; 		//Error code for std Sub20 API
	int               iADC_cfg_set; 	//Set ADC config
	std::stringstream ss_errmsg;
        std::string       ss_config;
	sub_device        sub20dev;
	sub_handle        sub20handle;
	OneSub20Config    OneSub20Configuration;

	ROS_INFO("--------------------------------------");
	initADC_MUX_MAP();				// Initialize the ADC_MUX mapping for String Values of enum entries
	///////////////////////////////////////////////////////////
	// Open connected Sub20 Devices
	///////////////////////////////////////////////////////////
	sub20dev = 0;
	sub20dev = sub_find_devices(sub20dev);
	while( sub20dev != 0 ) {
            sub20handle = sub_open( sub20dev );
	    // on success, sub_open returns non-zero handle
	    if (sub20handle > 0) {
                subhndl = sub20handle;
	    	//////////////////////////////////////////////////////////
	    	// Configure Sub20 device
	    	//////////////////////////////////////////////////////////
	    	std::cout << "---Initializing ADC Interface---" << std::endl;
	    	// Read and set current ADC configuration
	    	iADC_cfg_set = ADC_ENABLE;
	    	ss_config = "ADC_ENABLE | ";
                if((dVRef==VCC_5V)||(dVRef==VCC_3V3)){
                        iADC_cfg_set |= ADC_REF_VCC;
                        ss_config += "ADC_REF_VCC";
                } else {
                        iADC_cfg_set |= ADC_REF_2_56;
                        ss_config += "ADC_REF_2_56";
                }
                //Configure ADC
	    	iADCErr = sub_adc_config( sub20handle, iADC_cfg_set);
	    	if(iADCErr == 0) {
	    		std::cout << "ADC Configuration   : " << iADC_cfg_set << " successfully stored \n";
	    		std::cout << "Configuration Details   : " << std::endl;
	    		std::cout << "		Register Settings   		: " << ss_config << std::endl;
	    		std::cout << "		ADC Readings Rate (/sec)	: " << dRate_Hz << std::endl;

                        //ADC Read Configuration
                        get_ADC_MuxCode(ADC_MUXCONFIG, OneSub20Configuration.sub20_ADC_Config.sub20_ADC_MUX, OneSub20Configuration.sub20_ADC_Config.sub20_ADC_TYPE, &OneSub20Configuration.sub20_ADC_Config.Num_Chans);
                        std::cout << "		ADC Channels Selected  		: " << ADC_MUXCONFIG << std::endl;
                        OneSub20Configuration.bSubDevConfigured = true;

                        //string needs to be cleared otherwise conversion is going wrong
                        strSerial.clear();
                        strSerial.resize(sub20_ADC_cmd::uSERIALNUMLENGTH);
                        sub_get_serial_number(sub20handle, const_cast<char*>(strSerial.c_str()), strSerial.size());
                        OneSub20Configuration.strSub20Serial 	= strSerial;
                        OneSub20Configuration.handle_subdev 	= sub20handle;
                        OneSub20Configuration.subdev 		= sub20dev;
                        std::cout << "Device Handle   : " << OneSub20Configuration.handle_subdev << std::endl;
                        std::cout << "Serial Number   : " << OneSub20Configuration.strSub20Serial << std::endl;

                        /////////////////////////////////////////////////
                        // Push element onto list of subdevices
                        /////////////////////////////////////////////////
                        Sub20Device_list.push_back (OneSub20Configuration);
                        std::cout << "Serial : " << OneSub20Configuration.strSub20Serial << "\n";
	    	} else {
	    		ROS_INFO("ERROR - Configuration : %d not accepted by device", iADC_cfg_set);
	    		//Subdevice could not be configured
	    		OneSub20Configuration.bSubDevConfigured = false;
	    	}
            }
            //find next device, sub_find_devices using current provides next
            sub20dev = sub_find_devices(sub20dev);
	}
	ROS_INFO("--------------------------------------");
	std::cout << "  ************* Publishing to rostopic - /sub20_ADC *************  " << std::endl;
}

Sub20_ADC::~Sub20_ADC() {
	// TODO Auto-generated destructor stub
	int iSpiErr;
	OneSub20Config OneSub20Configuration;
	//close opened subdevices
        // Disable SPI
        iSpiErr = sub_spi_config( subhndl, 0, 0 );
        // Close USB device
        sub_close( subhndl );
	while (!Sub20Device_list.empty()) {
		OneSub20Configuration = Sub20Device_list.back ();
		std::cout << "Sub device removed " << OneSub20Configuration.strSub20Serial << "\n";
		Sub20Device_list.pop_back ();
	}
}

// Map the string input to enum ADC channel types
void Sub20_ADC::initADC_MUX_MAP(){
	sub20_ADC_cmd::adcMuxMap["ADC_S0"] = ADC_S0;
	sub20_ADC_cmd::adcMuxMap["ADC_S1"] = ADC_S1;
	sub20_ADC_cmd::adcMuxMap["ADC_S2"] = ADC_S2;
	sub20_ADC_cmd::adcMuxMap["ADC_S3"] = ADC_S3;
	sub20_ADC_cmd::adcMuxMap["ADC_S4"] = ADC_S4;
	sub20_ADC_cmd::adcMuxMap["ADC_S5"] = ADC_S5;
	sub20_ADC_cmd::adcMuxMap["ADC_S6"] = ADC_S6;
	sub20_ADC_cmd::adcMuxMap["ADC_S7"] = ADC_S7;

	sub20_ADC_cmd::adcMuxMap["ADC_D10_10X"] = ADC_D10_10X;
	sub20_ADC_cmd::adcMuxMap["ADC_D10_200X"] = ADC_D10_200X;
	sub20_ADC_cmd::adcMuxMap["ADC_D32_10X"] = ADC_D32_10X;
	sub20_ADC_cmd::adcMuxMap["ADC_D32_200X"] = ADC_D32_200X;

	sub20_ADC_cmd::adcMuxMap["ADC_D01"] = ADC_D01;
	sub20_ADC_cmd::adcMuxMap["ADC_D21"] = ADC_D21;
	sub20_ADC_cmd::adcMuxMap["ADC_D31"] = ADC_D31;
	sub20_ADC_cmd::adcMuxMap["ADC_D41"] = ADC_D41;
	sub20_ADC_cmd::adcMuxMap["ADC_D51"] = ADC_D51;
	sub20_ADC_cmd::adcMuxMap["ADC_D61"] = ADC_D61;
	sub20_ADC_cmd::adcMuxMap["ADC_D71"] = ADC_D71;
	sub20_ADC_cmd::adcMuxMap["ADC_D02"] = ADC_D02;
	sub20_ADC_cmd::adcMuxMap["ADC_D12"] = ADC_D12;
	sub20_ADC_cmd::adcMuxMap["ADC_D32"] = ADC_D32;
	sub20_ADC_cmd::adcMuxMap["ADC_D42"] = ADC_D42;
	sub20_ADC_cmd::adcMuxMap["ADC_D52"] = ADC_D52;

	sub20_ADC_cmd::adcMuxMap["ADC_1_1V"] = ADC_1_1V;
	sub20_ADC_cmd::adcMuxMap["ADC_GND"] = ADC_GND;
}

// Get the Mux Code to output the channels being used for ADC
void Sub20_ADC::get_ADC_MuxCode(std::string ADC_MUXCONFIG, int *adc_mux, int *adc_type, int *channels) {

	unsigned int i=0, j=0, numVal=0;
	size_t charLocn[sub20_ADC_cmd::iMAXNUM_OF_CHANNELS], found;
	std::string ADCMux_Cfg[sub20_ADC_cmd::iMAXNUM_OF_CHANNELS], tmpStr;

	size_t strlen = ADC_MUXCONFIG.length();

	charLocn[0] = ADC_MUXCONFIG.find("|");
	if(charLocn[0] != std::string::npos){
		ADCMux_Cfg[0] = ADC_MUXCONFIG.substr(0,charLocn[0]);
		adc_mux[j] = sub20_ADC_cmd::adcMuxMap[ADCMux_Cfg[0]];

		tmpStr = ADCMux_Cfg[0];
		found = tmpStr.find("S");
		if(found != std::string::npos) {
			adc_type[j] = 0;
		} else {
			found = tmpStr.find("D");
			if(found != std::string::npos){
				adc_type[j] = 1;
			} else {
				adc_type[j] = 2;
			}
		}
		j++;	numVal++;
	}
	for(i=1; i<strlen; i++){
		charLocn[i] = ADC_MUXCONFIG.find("|",(charLocn[i-1]+1));
		if(charLocn[i] == std::string::npos){
			break;
		}
		ADCMux_Cfg[i] = ADC_MUXCONFIG.substr((charLocn[i-1]+1),((charLocn[i]-charLocn[i-1])-1));
		adc_mux[j] = sub20_ADC_cmd::adcMuxMap[ADCMux_Cfg[i]];

		tmpStr = ADCMux_Cfg[i];
		found = tmpStr.find("S");
		if(found != std::string::npos) {
			adc_type[j] = 0;
		} else {
			found = tmpStr.find("D");
			if(found != std::string::npos){
				adc_type[j] = 1;
			} else {
				adc_type[j] = 2;
			}
		}
		j++;	numVal++;
	}
	*channels = numVal;
}

double Sub20_ADC::adc_To_Volts(int rawData, char convType, int muxCode, double dVRef){
	double value = 0, gain = 0;
	// convert adc to volts using 10-bit ADC and desired configuration
        // 'gains' are from the sub20-manual
	switch(convType){
		case 0:
			value = (rawData*dVRef)/1023;
			break;
		case 1:
			if((muxCode==ADC_D10_10X)||(muxCode==ADC_D32_10X)){
				gain = 10;
			} else if((muxCode==ADC_D10_200X)||(muxCode==ADC_D32_200X)){
				gain = 200;
			} else {
				gain = 1;
			}
			value = (rawData*dVRef)/(512*gain);
			break;
		case 2:
			value = rawData;
			break;
		default:
			break;
	}
	return value;
}

void Sub20_ADC::GetMeasurements(std::list<OneSub20_ADC_Meas> &list_meas, double dVRef, int *channels) {

	int                                    iADCErr;
	OneSub20_ADC_Meas                      sMeas;
	ros::Time                              dtomeas;
        std::stringstream	               ss_errmsg;
	std::list<OneSub20Config>::iterator    iterat;
	int 				       sub20_ADC_BUF[16];	// Buffer for ADC Channels
	int                                    adc_buf, adc_mux;
	int				       Num_Chans;

	//verify that a subdevice is connected
	if (1 > (int)Sub20Device_list.size()) {
		throw std::string ("No SubDevice connected OR access rights to USB not given");
	}


	//Trace sub20 list
	for (iterat = Sub20Device_list.begin(); iterat != Sub20Device_list.end(); iterat++ ) {
		//verify if the sub20 device is configured (which should be the case as only configured sub20ies are pushed on list)
		if (iterat->bSubDevConfigured == true) {
			Num_Chans = iterat->sub20_ADC_Config.Num_Chans;
//			iADCErr = sub_adc_read(iterat->handle_subdev, sub20_ADC_BUF, iterat->sub20_ADC_Config.sub20_ADC_MUX, Num_Chans);
//			if(iADCErr == 0){
				sMeas.bMeasAvailable = true;
				for (int i=0; i<Num_Chans; i++){
                                        adc_mux = iterat->sub20_ADC_Config.sub20_ADC_MUX[i];
                                        iADCErr = sub_adc_single(iterat->handle_subdev, &adc_buf, adc_mux);
                                        sub20_ADC_BUF[i] = adc_buf;
                                        if(iADCErr == 0){
                                              sMeas.uiADCRaw[i] = sub20_ADC_BUF[i];
                                              sMeas.fADCVolt[i] = adc_To_Volts(sub20_ADC_BUF[i], iterat->sub20_ADC_Config.sub20_ADC_TYPE[i], iterat->sub20_ADC_Config.sub20_ADC_MUX[i], dVRef);
                                        } else {
                                              sMeas.bMeasAvailable = true;
                                              throw std::string ("Error on ADC conversion - Unplug or reset Sub20 Device");
                                        }
				}
				sMeas.dtomeas 	= ros::Time::now();
				sMeas.strSerial	= iterat->strSub20Serial;
				// Push measurement onto heap once all available channels are read!
				list_meas.push_back (sMeas);
//			} else {
//				throw std::string ("Error on ADC conversion - Unplug or reset Sub20 Device");
//			}
		} else {
			throw std::string ("SUB20 connected but not configured - Restart of node suggested");
		}
	}
	*channels = Num_Chans;
}

// ----------
// -- MAIN --
// ----------
int main(int argc, char **argv) {
	//---------------------------------------------
	// Declarations
	//---------------------------------------------
	//General definitions
	int 				count = 1;
	//Publisher for sub20_ADC data
	ros::init(argc, argv, "adc_sub20");
	ros::NodeHandle n;
	ros::Publisher sub20_ADC_pub = n.advertise<adc_sub20::sub20_ADC_meas>("adc_sub20", 100);
	//Parameter Server values
	double 			dRate_Hz;
	double 			dVRef;
	int 			Num_Chans;

	std::string		ADC_MUXCONFIG;

	//---------------------------------------------
	// Read initialization parameters from server
	//---------------------------------------------
	//Read parameters from server - if parameters are not available, the node
	//is initialized with default
	if (n.getParam("/adc_sub20/ADC_MUX_CONFIG", ADC_MUXCONFIG)         == false ) {
		ADC_MUXCONFIG = sub20_ADC_cmd::sDEFAULT_ADC_MUXCONFIG;
	};
	if (n.getParam("/adc_sub20/rate_Hz", dRate_Hz)                   == false ) {
		dRate_Hz = sub20_ADC_cmd::dDEFAULT_RATE_Hz;
	};
	if (n.getParam("/adc_sub20/VoltRef", dVRef)               == false ) {
		dVRef = sub20_ADC_cmd::dDEFAULT_VOLTREF;
	};

        Sub20_ADC 			sub20_ADC_attached(ADC_MUXCONFIG, dRate_Hz, dVRef);
	OneSub20_ADC_Meas		sOneMeas;
	adc_sub20::sub20_ADC_meas 	msg;
	std::list<OneSub20_ADC_Meas> 	measurements_list;

	///////////////////////////////////////////////////////////
	// Run sensor node
	///////////////////////////////////////////////////////////
	//set loop rate for measurement polling
	ros::Rate loop_rate(dRate_Hz);

	while (n.ok()) {
		//initiate a measurement on ADC
		try {
		  sub20_ADC_attached.GetMeasurements(measurements_list, dVRef, &Num_Chans);
		}
		catch ( std::string strType ) {
			std::cout << " An exception occurred: " << strType << std::endl;
			std::exit(1);
		}

		while (!measurements_list.empty()) {
			sOneMeas = measurements_list.back ();
			measurements_list.pop_back ();
			//only publish if a successful measurement was received
			if ( sOneMeas.bMeasAvailable == true ) {
				msg.strIdSubDev   	= sOneMeas.strSerial;

				// collect all available channel values into one message
				for(int i=0; i<Num_Chans; i++){
					msg.uiRaw.push_back(sOneMeas.uiADCRaw[i]);
					msg.fVolts.push_back(sOneMeas.fADCVolt[i]);
				}
				msg.header.stamp  	= sOneMeas.dtomeas;
				sub20_ADC_pub.publish(msg);         // publish to topic!

				// clear the message values after publishing to avoid accumulation of further values on subsequent messages.
				msg.uiRaw.clear();
				msg.fVolts.clear();
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
}
