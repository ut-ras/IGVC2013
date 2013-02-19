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

int8 GetADC(void){
	return ADC_1_GetResult8();
}

void InitializeADC(void){
	ADC_1_Start();
	ADC_1_StartConvert();
}
/* [] END OF FILE */
