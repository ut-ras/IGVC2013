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
#define rPdiv 1
#define rD 1
#define rDdiv 1
#define rI 0
#define rIdiv 1
int8 RightMotorPID(){
	static int prevError = 0;
	static long accError = 0;
	int16 curError;
	int8 output = 0;
	curError = curRightMotorCommand - rightMotor;
	output += rP * curError / rPdiv;
	output += rD * (curError-prevError) / rDdiv;
	output += rI * accError / rIdiv;
	prevError = curError;
	accError += curError;
	return output;
}

#define lP 1
#define lPdiv 1
#define lD 1
#define lDdiv 1
#define lI 0
#define lIdiv 1
int8 LeftMotorPID(){
	static int prevError = 0;
	static long accError = 0;
	int16 curError;
	int16 output = 0;
	curError = curLeftMotorCommand - leftMotor;
	output += lP * curError / lPdiv;
	output += lD * (curError-prevError) / lDdiv;
	output += lI * accError / lIdiv;
	prevError = curError;
	accError += curError;
	return output;
}

//called to update the motor outputs
void RunVelocityControl(void){
	SetLeftMotor(SATURATE(LeftMotorPID(),-128,127));
	SetRightMotor(SATURATE(RightMotorPID(),-128,127));
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
	InitializeEncodersInvert(1,-1);
	linearX = 0;
	angularZ = 0;
	rightMotor = 0;
	leftMotor = 0;
	curLeftMotorCommand = 0;
	curRightMotorCommand = 0;
	curLinearXCommand = 0;
	curAngularZCommand = 0;
}

/* [] END OF FILE */
