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
#include <servodriver.h>

void ServosOut(uint8 x, uint8 y){
	Servo_WriteCompare1(x?1500:4500);
	Servo_WriteCompare2(y?1500:4500);
}

void ServoInit(void){
	Servo_Start();
}



/* [] END OF FILE */
