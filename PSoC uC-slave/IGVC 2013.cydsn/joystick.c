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
#include <joystick.h>
#define DAC_CENTER_OUT 156
#define DAC_MIN_OUT 78
#define DAC_MAX_OUT 234
#define SOFT_KILL_DISABLE 0

// Input 0-255
// DAC out range 78 (1.25V) to 234 (3.75V)
void JoystickXOut(uint8 x){
	unsigned short xOut = ((((unsigned short) x) * DAC_CENTER_OUT) / 256) + DAC_MIN_OUT;
	if(SOFT_KILL_DISABLE || Soft_Kill_Read()) VDAC8_1_SetValue(xOut);
	else VDAC8_1_SetValue(DAC_CENTER_OUT);
}
void JoystickYOut(uint8 y){
	unsigned short yOut = ((((unsigned short) y) * DAC_CENTER_OUT) / 256) + DAC_MIN_OUT;
	if(SOFT_KILL_DISABLE || Soft_Kill_Read()) VDAC8_2_SetValue(yOut);
	else VDAC8_2_SetValue(DAC_CENTER_OUT);
}
void JoystickOut(uint8 x, uint8 y){ 
	unsigned short xOut = ((((unsigned short) x) * DAC_CENTER_OUT) / 256) + DAC_MIN_OUT;
	unsigned short yOut = ((((unsigned short) y) * DAC_CENTER_OUT) / 256) + DAC_MIN_OUT;
	if(SOFT_KILL_DISABLE || Soft_Kill_Read()) VDAC8_1_SetValue(xOut);
	else VDAC8_1_SetValue(DAC_CENTER_OUT);
	if(SOFT_KILL_DISABLE || Soft_Kill_Read()) VDAC8_2_SetValue(yOut);
	else VDAC8_2_SetValue(DAC_CENTER_OUT);
}

void JoystickInit(void){
	VDAC8_1_Start();
    VDAC8_2_Start();
	VDAC8_3_Start();
	JoystickOut(128,128);
	VDAC8_3_SetValue(DAC_CENTER_OUT);
}


/* [] END OF FILE */
