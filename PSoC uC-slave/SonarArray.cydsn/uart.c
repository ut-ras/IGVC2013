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

/*
int fgetc (FILE * f){
  return USBUART_GetChar();
}


int ferror(FILE * f){
  return EOF;
}*/

uint8 UARTgetchar(void){
	while(USBUART_DataIsReady() == 0);
	return USBUART_GetChar();
}



//"gets" function through USBUART instead of stdio
//echos the input
//terminates on CR or LF
//returns size of string
uint16 UARTgets(uint8 * str, int bufferSize){
	uint16 i = 0;
	uint16 size;
	do{
		while(USBUART_DataIsReady() == 0);
		size = USBUART_GetCount();
		if(size+i > bufferSize){
			str[i]=0;
			UARTprintf("gets: Buffer not big enough. Size: %d Input: %s\r\n", size, str);
			return i;
		}
		USBUART_GetData(&(str[i]),size);
		i+=size;
		if(size == 1)UARTputc(str[i-1]); //echo if single char
	}while((str[i-1] != '\n') && (str[i-1] != '\r'));
	str[--i] = 0; //null terminate, remove the CR/LF
	//UARTprintf("gets: LF or CR found at end of message. Size: %d Input: %s\r\n", size, str);
	//while(USBUART_CDCIsReady()==0);
	//USBUART_PutCRLF(); //print CRLF
	return i;
}

void UARTgetMessage(uint8 * str){
	while(USBUART_DataIsReady() == 0);
	USBUART_GetAll(str);
	//UARTprintf("%c",6);//return ACK
}


//"printf" function through USBUART instead of stdio
//limited buffer size
#define PRINTF_BUFFER_SIZE 64
int UARTprintf(const char * ch, ... ){
	char8 lineStr[PRINTF_BUFFER_SIZE];
	va_list ap;
	va_start(ap,ch);
	//vsnprintf(lineStr,PRINTF_BUFFER_SIZE,ch,ap);
	sprintf(lineStr, ch, ap);
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
