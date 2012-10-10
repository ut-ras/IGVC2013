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
#include <uart.h>

int fgetc (FILE * f){
  return USBUART_GetChar();
}
int ferror(FILE * f){
  return EOF;
}
int UARTprintf(const char * ch, ... ){
	char8 lineStr[64];
	sprintf(lineStr,ch);
	USBUART_PutString(lineStr);
	return 0;
}


void InitializeUART(void){
	 /* Start USBFS Operation with 3V operation */
    USBUART_Start(0, USBUART_3V_OPERATION);
    while(!USBUART_GetConfiguration());
	USBUART_CDC_Init();
}


/* [] END OF FILE */
