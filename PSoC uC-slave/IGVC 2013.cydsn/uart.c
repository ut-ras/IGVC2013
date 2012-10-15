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
#define CR 13;
#define LF 10;


int fgetc (FILE * f){
  return USBUART_GetChar();
}


int ferror(FILE * f){
  return EOF;
}

uint8 UARTgetchar(void){
	while(USBUART_DataIsReady() == 0);
	return USBUART_GetChar();
}



//"gets" function through USBUART instead of stdio
//echos the input
//terminates on CR or LF
//returns size of string
uint16 UARTgets(uint8 * str){
	uint16 i = 0;
	uint8 buffer;
	do{
		while(USBUART_DataIsReady() == 0);
		USBUART_GetData(&buffer , 1);
		str[i++]= buffer;
		UARTputc(buffer); //echo
	}while((buffer != '\n') && (buffer != '\r'));
	str[--i] = 0; //null terminate, remove the CR/LF
	while(USBUART_CDCIsReady()==0);
	USBUART_PutCRLF(); //print CRLF
	return i;
}


//"printf" function through USBUART instead of stdio
//limited buffer size
#define PRINTF_BUFFER_SIZE 64
int UARTprintf(const char * ch, ... ){
	char8 lineStr[PRINTF_BUFFER_SIZE];
	va_list ap;
	va_start(ap,ch);
	vsnprintf(lineStr,PRINTF_BUFFER_SIZE,ch,ap);
	va_end(ap);
	while(USBUART_CDCIsReady()==0);
	USBUART_PutString(lineStr);
	return 0;
}
void UARTputc(const char ch){
	while(USBUART_CDCIsReady()==0);
	USBUART_PutChar(ch);
}

void InitializeUART(void){
	 /* Start USBFS Operation with 3V operation */
    USBUART_Start(0, USBUART_3V_OPERATION);
    while(!USBUART_GetConfiguration());
	USBUART_CDC_Init();
}


/* [] END OF FILE */
