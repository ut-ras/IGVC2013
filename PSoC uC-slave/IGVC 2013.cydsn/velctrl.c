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
#include <device.h>
#include <velctrl.h>
#include <time.h>
#include <encoder.h>
#include <servo.h>
#include <uart.h>
#define SATURATE(in,min,max) (min > in) ? min : ( (max < in) ? max : in)


int linearX;  //percieved linear X velocity in encoder ticks per 20ms (50hz) (+ being the front of the robot, 0 being stationary)
int angularZ; //percieved angular Z velocity in encoder ticks per 20ms (50hz) (+ being clockwise, 0 being 12 o'clock)
int leftMotor;
int rightMotor;
int dLeft;
int dRight;
int curLinearXCommand;
int curAngularZCommand;
int curLeftMotorCommand;
int curRightMotorCommand;

//called to change the velocity command
void UpdateVelCommands(int lx, int az){
	curLeftMotorCommand = lx+az;
	curRightMotorCommand = lx-az;
}
void UpdateLinearX(int lx){
	curLinearXCommand = lx;
	curLeftMotorCommand = lx+curAngularZCommand;
	curRightMotorCommand = lx-curAngularZCommand;
}
void UpdateAngularZ(int az){
	curAngularZCommand = az;
	curLeftMotorCommand = curLinearXCommand+az;
	curRightMotorCommand = curLinearXCommand-az;
}


int GetV(void){ return linearX; }
int GetW(void){	return angularZ; }


#define rP 1
#define rPdiv 20
#define rD 1
#define rDdiv 10
#define rI 1
#define rIdiv 1000
int8 RightMotorPID(){
	static int16 prevError = 0;
	static int8 prevOutput = 0;
	static long accError = 0;
	static uint8 count = 0;
	int16 curError;
	int16 output = 0;
	curError = curRightMotorCommand - dRight;
	output += rP * curError / rPdiv;
	output += rD * (curError-prevError) / rDdiv;
	output += rI * accError / rIdiv;
	output += prevOutput;
	if (output > 127) output = 127;
	else if (output < -128) output = -128;
	prevOutput = output;
	prevError = curError;
	if(count++ == 0) accError = 0;
	accError += curError;
	return output;
}

#define lP 1
#define lPdiv 20
#define lD 1
#define lDdiv 10
#define lI 1
#define lIdiv 1000
int8 LeftMotorPID(){
	static int16 prevError = 0;
	static int8 prevOutput = 0;
	static long accError = 0;
	static uint8 count;
	int16 curError;
	int16 output = 0;
	curError = curLeftMotorCommand - dLeft;
	output += lP * curError / lPdiv;
	output += lD * (curError-prevError) / lDdiv;
	output += lI * accError / lIdiv;
	output += prevOutput;
	if (output > 127) output = 127;
	else if (output < -128) output = -128;
	prevOutput = output;
	prevError = curError;
	if(count++ == 0) accError = 0;
	accError += curError;
	return output;
}

//called to update the motor outputs
void RunVelocityControl(void){
	int8 leftOut = LeftMotorPID();
	int8 rightOut = RightMotorPID();
	SetLeftMotor(leftOut);
	SetRightMotor(rightOut);
	//UARTprintf("PID Debug %d %d %d %d %d %d\r\n", curLeftMotorCommand, curRightMotorCommand, dLeft, dRight, leftOut, rightOut);
}

void UpdateVelocity(void){
	static long prevLeft = 0;
	static long prevRight = 0;
	
	dLeft = GetLeftEncoder()-prevLeft;
	dRight = GetRightEncoder()-prevRight;
	
	linearX = dLeft+dRight;
	angularZ = dLeft-dRight;
	
	prevLeft = GetLeftEncoder();
	prevRight = GetRightEncoder();
}

void InitializeVelocityControl(void){
	//These two need to happen if they haven't already
	/*InitializeWatchdog(); 
	InitializeTime();*/
	InitializeEncodersInvert(-1,1);
	linearX = 0;
	angularZ = 0;
	rightMotor = 0;
	leftMotor = 0;
	dLeft = 0;
	dRight = 0;
	curLeftMotorCommand = 0;
	curRightMotorCommand = 0;
	curLinearXCommand = 0;
	curAngularZCommand = 0;
}

/* [] END OF FILE */
