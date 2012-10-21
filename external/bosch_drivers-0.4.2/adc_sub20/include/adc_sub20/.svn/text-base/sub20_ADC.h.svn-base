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

//\Author Nikhil Deshpande, Robert Bosch LLC

#ifndef SUB20_ADC_H_
#define SUB20_ADC_H_

#include "adc_sub20/sub20_ADC_meas.h"
#include "adc_sub20/sub20_ADC_err.h"
#include <list>
#include <libsub.h> // sub20 device
#include <map>


//Define Sub20 ADC specifics
namespace sub20_ADC_cmd {
	//-----------------------------------
	const unsigned short uSERIALNUMLENGTH	= 20;
	//-----------------------------------
	const double   		dDEFAULT_VOLTREF				= 3.3;
	const double  		dDEFAULT_RATE_Hz				= 10;
	const std::string 	sDEFAULT_ADC_MUXCONFIG				= "ADC_S0|";
	//-----------------------------------
	const unsigned short iMAXNUM_OF_CHANNELS = 16; 		//Defined by the XDimax Unit's possible ADC Configurations
	//-----------------------------------
	std::map<std::string, int> adcMuxMap;

}

#define VCC_5V          5.0
#define VCC_3V3         3.3

//Define measurement output
struct OneSub20_ADC_Meas {
  bool          bMeasAvailable; //indicates true if a valid measurement is available
  unsigned int  uiADCRaw[sub20_ADC_cmd::iMAXNUM_OF_CHANNELS];
  double  	fADCVolt[sub20_ADC_cmd::iMAXNUM_OF_CHANNELS];
  ros::Time     dtomeas; //time tag measurement immediately in case of other delays
  std::string   strSerial;
};

class Sub20_ADC {
public:
	//! Constructor initializes Sub20 device
	/*!
	 * Detects connected Sub20 devices. Initializes
	 * the Sub20 device and configures its ADC with the
	 * specified VOltage Ref and ADC Channel Mux Codes.
	 *
	 * \param bVRef - The Voltage Reference setting - Vcc (false) or internal 2.5V (true)
	 */
	Sub20_ADC(std::string, double, double);

	//! Destructor closing all open connections
	/*!
	 * Disables the Sub20 device. Disposes any dynamic
	 * structure in the heap.
	 */
	~Sub20_ADC();

	//! Reads one measurement from all ADC Channels defined in ucADCSingleChan and uiADCDiffrChan
	/*!
	 * Iterates through all connected Sub20 devices and sends
	 * a measurement request to the respective ADCs.
	 *
	 * \param &list_meas points onto the std::list containing the connected devices
	 * \param voltRef - The setting for the voltage reference for the ADC channels
	 * \param &channels - number of channels connected
	 */
	void GetMeasurements( std::list<OneSub20_ADC_Meas>&, double, int *);

private:
	// structure for Sub20 ADC configuration
	struct Sub20_ADC_Config {
		int         sub20_ADC_TYPE[sub20_ADC_cmd::iMAXNUM_OF_CHANNELS];	// Type of Channel - 0=Single, 1=Differential, 2=1.1V/GND
		int         sub20_ADC_MUX[sub20_ADC_cmd::iMAXNUM_OF_CHANNELS];	// Mux Codes for ADC Channels
                int         Num_Chans;			// Number of ADC Channels used
	};
	// structure for one Sub20 device configuration
	struct OneSub20Config {
		std::string     	strSub20Serial;
		sub_handle      	handle_subdev;
		sub_device      	subdev;
		bool            	bSubDevConfigured;
		Sub20_ADC_Config	sub20_ADC_Config;
	};
	std::list<OneSub20Config> 	Sub20Device_list;



	// internal Sub20 device data
	sub_handle 	  subhndl;						//handle for subdevice
	bool              bSubDeviceOpen;			//initialization flag
	bool 		  bSubDeviceConfigured;  	//verify if subdevice is configured
	std::string       strSerial; 				//Serial number of SUB20 device

	/// Sets the Mux Codes for the ADC Signals
	void            get_ADC_MuxCode(std::string, int *, int *, int *);
	/// Convert raw ADC to Volts
	double 		adc_To_Volts(int, char, int, double);
	/// Map enum to String
	void 		initADC_MUX_MAP();
};

#endif /* SUB20_ADC_H_ */
