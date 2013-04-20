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
#include <adc.h>
#include <uart.h>

int16 GetADC(void){
	return ADC_1_GetResult16();
}

int16 GetMV(void){
	return ADC_1_CountsTo_mVolts(ADC_1_GetResult16());
}
int32 GetUV(void){
	return ADC_1_CountsTo_uVolts(ADC_1_GetResult16());
}
int32 GetUVTriggered(void){
	ADC_1_StartConvert();
	ADC_1_IsEndConversion(ADC_1_WAIT_FOR_RESULT);
	return ADC_1_CountsTo_uVolts(ADC_1_GetResult16());
}
void InitializeADC(void){
	ADC_1_Start();
	ADC_1_StartConvert();
}
void InitializeADCTriggered(void){
	ADC_1_Start();
}
/* [] END OF FILE */
