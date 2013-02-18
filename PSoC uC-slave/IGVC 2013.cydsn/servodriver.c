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
#include <servo.h>
#define SERVO_COMPARE_CENTER 3095
#define SERVO_COMPARE_DEVIATION 1031

//Experimental calibration values for the specific Victor at the time
#define LEFT_MOTOR_UP_LOWER 3231
#define LEFT_MOTOR_UP_DEVIATION 932
#define LEFT_MOTOR_DOWN_LOWER 2998
#define LEFT_MOTOR_DOWN_DEVIATION 870
void SetLeftMotor(int8 out){
	if(out == 0)
		Servo_0_WriteCompare1(SERVO_COMPARE_CENTER);
	if(out > 0)
		Servo_0_WriteCompare1(LEFT_MOTOR_UP_LOWER + ((LEFT_MOTOR_UP_DEVIATION * (int16)out) >> 7));
	if(out < 0)
		Servo_0_WriteCompare1(LEFT_MOTOR_DOWN_LOWER + ((LEFT_MOTOR_DOWN_DEVIATION * (int16)out) >> 7));
}

//Experimental calibration values for the specific Victor at the time
#define RIGHT_MOTOR_UP_LOWER 3184
#define RIGHT_MOTOR_UP_DEVIATION 821
#define RIGHT_MOTOR_DOWN_LOWER 3031
#define RIGHT_MOTOR_DOWN_DEVIATION 741
void SetRightMotor(int8 out){
	if(out == 0)
		Servo_0_WriteCompare2(SERVO_COMPARE_CENTER);
	if(out > 0)
		Servo_0_WriteCompare2(RIGHT_MOTOR_UP_LOWER + ((RIGHT_MOTOR_UP_DEVIATION * (int16)out) >> 7));
	if(out < 0)
		Servo_0_WriteCompare2(RIGHT_MOTOR_DOWN_LOWER + ((RIGHT_MOTOR_DOWN_DEVIATION * (int16)out) >> 7));
}

#define HOKUYO_TILT_TOP_BOUND 50
#define HOKUYO_TILT_BOTTOM_BOUND -50
void HokuyoTiltStep(void){
	static int8 step = 0;
	static int8 direction = 1;
	step += direction;
	if(step == HOKUYO_TILT_TOP_BOUND) direction = -1;
	else if(step == HOKUYO_TILT_BOTTOM_BOUND) direction = 1;
	SetHokuyoServo(step);
}
uint16 GetHokuyoServo(void){
	return Servo_1_ReadCompare();
}
void SetHokuyoServo(int8 out){
	Servo_1_WriteCompare(SERVO_COMPARE_CENTER + ((SERVO_COMPARE_DEVIATION * (int16)out) >> 7));
}

void InitializeServo(){
	Servo_0_Start();
	Servo_1_Start();
}



/* [] END OF FILE */
