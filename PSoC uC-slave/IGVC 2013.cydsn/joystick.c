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


// Input 0-255
// DAC out range 78 (1.25V) to 234 (3.75V)
void JoystickXOut(uint8 x){
	unsigned short xOut = ((((unsigned short) x) * 156) / 256) + 78;
	VDAC8_1_SetValue(xOut);
}
void JoystickYOut(uint8 y){
	unsigned short yOut = ((((unsigned short) y) * 156) / 256) + 78;
	VDAC8_2_SetValue(yOut);
}
void JoystickOut(uint8 x, uint8 y){ 
	unsigned short xOut = ((((unsigned short) x) * 156) / 256) + 78;
	unsigned short yOut = ((((unsigned short) y) * 156) / 256) + 78;
	VDAC8_1_SetValue(xOut);
	VDAC8_2_SetValue(yOut);
}

void JoystickInit(void){
	VDAC8_1_Start();
    VDAC8_2_Start();
	VDAC8_3_Start();
	JoystickOut(128,128);
	VDAC8_3_SetValue(156);
}


/* [] END OF FILE */
