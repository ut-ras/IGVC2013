/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include <stdlib.h>
#include <device.h>
#include <uartComSlave.h>
#include <joystick.h>
#include <time.h>
#include <uart.h>
#include <velctrl.h>
#include <encoder.h>
#include <sphero.h>
#include <servo.h>


#define CODE_START					1
#define CODE_LENGTH					4
#define DATA_START					6
#define SPLM 'S'^'P'^'L'^'M' //Set Power Left Motor
#define SPRM 'S'^'P'^'R'^'M' //Set Power Right Motor
#define SAHS 'S'^'A'^'H'^'S' //Set Angle Hokuyo Servo
#define SVLX 'S'^'V'^'L'^'X' //Set Velocity Linear X
#define SVAZ 'S'^'V'^'A'^'Z' //Set Velocity Angular Z
#define ETFM 'E'^'T'^'F'^'M' //Enable Telemetry Feedback Messages
#define DTFM 'D'^'T'^'F'^'M' //Disable Telemetry Feedback Messages
#define RSTE 'R'^'S'^'T'^'E' //Reset Encoder Count
#define STMR 'S'^'T'^'M'^'R' //Set Message Rate
#define SETT 'S'^'E'^'T'^'T' //Set Time
#define SATURATE(in,min,max) (min > in) ? min : ( (max < in) ? max : in)

extern uint8 VelCtrlRunning;
uint8 EnableSensorFeedbackMessages;

uint8 isValidMessage(const char * ch){
	if(ch[0] == '>') return 1;
	else return 0;
}

#define COMM_BUFFER_SIZE 80
void handleCommMessage(void){
	char buffer[COMM_BUFFER_SIZE];
	unsigned char function_code = 0;
	long i;
	buffer[64]=0;
	buffer[79]=0;
	UARTgets((uint8*)buffer, COMM_BUFFER_SIZE);
	//UARTgetMessage(buffer);
	
	//UARTprintf("MESSAGE GET!!%s\r\n", buffer);

	if(isValidMessage(buffer))
	{
		for(i = 0; i < CODE_LENGTH; i++) //generate hash of function code
		{
			function_code ^= buffer[CODE_START + i];
		}

		switch(function_code)
		{
			case SPLM: 	//JoystickXOut(SATURATE(atoi(&buffer[DATA_START]),0,255));
					   	//UARTprintf("GOOD MESSAGE- SVXA: %s\r\n", buffer);
						SetLeftMotor(SATURATE(atoi(&buffer[DATA_START]),-128,127));
					   	VelCtrlRunning = 0;
					  	ResetWatchdog(); //we got a valid message, so they are still talking to us
					   	break;
			case SPRM: 	//JoystickYOut(SATURATE(atoi(&buffer[DATA_START]),0,255));
					   	//UARTprintf("GOOD MESSAGE- SVYA: %s\r\n", buffer);
					   	SetRightMotor(SATURATE(atoi(&buffer[DATA_START]),-128,127));
					   	VelCtrlRunning = 0;
					   	ResetWatchdog(); //we got a valid message, so they are still talking to us
					   	break;
			case SVLX: 	UpdateLinearX(atoi(&buffer[DATA_START]));
					   	ResetWatchdog();
					   	VelCtrlRunning = 1;
					   	break;
			case SVAZ: 	UpdateAngularZ(atoi(&buffer[DATA_START]));
					   	VelCtrlRunning = 1;
					   	ResetWatchdog();
					   	break;
			case ETFM: 	EnableSensorFeedbackMessages = 1;
					  	break;
			case DTFM: 	EnableSensorFeedbackMessages = 0;
					   	break;
			case RSTE: 	ResetEncoders();
					   	break;
			case STMR: 	if(1000 % atoi(&buffer[DATA_START]) == 0)
						    SetMessageRate(atoi(&buffer[DATA_START]));
						else
							UARTprintf("INVALID MESSAGE RATE (1000%rate must = 0): %s", buffer);
						break;
			case SETT:	SetTime(atoi(&buffer[DATA_START]));
						break;
			default:   	UARTprintf("UNRECOGNIZED MESSAGE: %s\r\n", buffer);
					   	break;
		}
	}
	else
	{
		UARTprintf("INVALID START CHARACTER: %s\r\n", buffer);
	}
}

void sendCommMessage(void){
	if(EnableSensorFeedbackMessages){
		UARTprintf("(: %d %d %d %d %d%.3d %d :)\r\n",
		GetLeftEncoder(), 
		GetRightEncoder(), 
		GetV(), 
		GetW(), 
		GetTime(),GetMS(),
		GetMessageRate() );
		//UARTprintf("^.- ROLL %d PTCH %d YAWW %d -.^\r\n", roll, pitch, yaw );
		/*UARTprintf("(: %d %d %d %d %d %d %d %d%.3d %d :)\r\n", 
		GetLeftEncoder(), 
		GetRightEncoder(), 
		GetV(), 
		GetW(), 
		GetRoll(),
		GetPitch(),
		GetYaw(),
		GetTime(), 
		GetMS(),
		GetMessageRate() );*/
	}
}

void InitializeUCSlave(void){
	InitializeUART();
	InitializeTime();
	InitializeServo();
	InitializeWatchdog();
	InitializeVelocityControl();
	//InitializeIMU();
	EnableSensorFeedbackMessages = 0;
}

void RunUCSlave(void){
	InitializeUCSlave();
	while(1) handleCommMessage();
	
}

/* [] END OF FILE */
