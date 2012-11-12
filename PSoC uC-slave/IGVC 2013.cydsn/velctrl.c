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
#include <joystick.h>
#define SATURATE(in,min,max) (min > in) ? min : ( (max < in) ? max : in)


int linearX;  //percieved linear X velocity in encoder ticks per 20ms (50hz) (+ being the front of the robot, 0 being stationary)
int angularZ; //percieved angular Z velocity in encoder ticks per 20ms (50hz) (+ being clockwise, 0 being 12 o'clock)
int dLeft;
int dRight;
int curLinearXCommand;
int curAngularZCommand;

//called to change the velocity command
void UpdateVelCommands(int lx, int az){
	curLinearXCommand = lx;
	curAngularZCommand = az;
}
void UpdateAngularZ(int az){
	curAngularZCommand = az;
}
void UpdateLinearX(int lx){
	curLinearXCommand = lx;
}
int GetV(void){ return linearX; }
int GetW(void){	return angularZ; }


#define xP 1
#define xPdiv 1
#define xD 1
#define xDdiv 1
#define xI 0
#define xIdiv 1
int XPID(){
	static int prevError = 0;
	static long accError = 0;
	int curError;
	int output = 0;
	curError = curLinearXCommand - linearX;
	output += xP * curError / xPdiv;
	output += xD * (curError-prevError) / xDdiv;
	output += xI * accError / xIdiv;
	prevError = curError;
	accError += curError;
	return output;
}

#define zP 1
#define zPdiv 1
#define zD 1
#define zDdiv 1
#define zI 0
#define zIdiv 1
int ZPID(){
	static int prevError = 0;
	static long accError = 0;
	int curError;
	int output = 0;
	curError = curAngularZCommand - angularZ;
	output += zP * curError / zPdiv;
	output += zD * (curError-prevError) / zDdiv;
	output += zI * accError / zIdiv;
	prevError = curError;
	accError += curError;
	return output;
}

//called to update the joystick outputs based on the current
void RunVelocityControl(void){
	JoystickYOut(SATURATE(XPID(),0,255));
	JoystickXOut(SATURATE(ZPID(),0,255));
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
	InitializeEncodersInvert(1,1);
	linearX = 0;
	angularZ = 0;
	curLinearXCommand = 0;
	curAngularZCommand = 0;
}

/* [] END OF FILE */
