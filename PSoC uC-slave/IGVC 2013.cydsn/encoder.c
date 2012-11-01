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
#include <encoder.h>

int invertLeft;
int invertRight;

int32 GetLeftEncoder(void){
	return invertLeft * Left_Encoder_GetCounter();
}
int32 GetRightEncoder(void){
	return invertRight * Right_Encoder_GetCounter();
}
void LeftEncoderInterruptHandler(void){
	Left_Encoder_Interrupt_ClearPending() ;
	//doubt this is gonna overflow
}
void RightEncoderInterruptHandler(void){
	Right_Encoder_Interrupt_ClearPending() ;
}

void InitializeEncoders(void){
	invertLeft = 1;
	invertRight = 1;
	Left_Encoder_Start();
	Right_Encoder_Start();
	Left_Encoder_SetCounter(0);
	Right_Encoder_SetCounter(0);
	Left_Encoder_SetInterruptMask(Left_Encoder_COUNTER_OVERFLOW|Left_Encoder_COUNTER_UNDERFLOW);
	Right_Encoder_SetInterruptMask(Right_Encoder_COUNTER_OVERFLOW|Right_Encoder_COUNTER_UNDERFLOW);
	Left_Encoder_Interrupt_StartEx(LeftEncoderInterruptHandler) ;
	Right_Encoder_Interrupt_StartEx(RightEncoderInterruptHandler) ;
}
void InitializeEncodersInvert(int leftInvert, int rightInvert){
	invertLeft = leftInvert;
	invertRight = rightInvert;
	Left_Encoder_Start();
	Right_Encoder_Start();
	Left_Encoder_SetCounter(0);
	Right_Encoder_SetCounter(0);
	Left_Encoder_SetInterruptMask(Left_Encoder_COUNTER_OVERFLOW|Left_Encoder_COUNTER_UNDERFLOW);
	Right_Encoder_SetInterruptMask(Right_Encoder_COUNTER_OVERFLOW|Right_Encoder_COUNTER_UNDERFLOW);
	Left_Encoder_Interrupt_StartEx(LeftEncoderInterruptHandler) ;
	Right_Encoder_Interrupt_StartEx(RightEncoderInterruptHandler) ;
}

/* [] END OF FILE */
