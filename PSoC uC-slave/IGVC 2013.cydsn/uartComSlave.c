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

#include <stdlib.h>
#include <device.h>
#include <uartComSlave.h>
#include <joystick.h>
#include <time.h>
#include <uart.h>

#define CODE_START					1
#define CODE_LENGTH					4
#define DATA_START					6
#define SVXA 'S'^'V'^'X'^'A'
#define SVYA 'S'^'V'^'Y'^'A'
#define SATURATE(in,min,max) (min > in) ? min : ( (max < in) ? max : in)

uint8 isValidMessage(const char * ch){
	if(ch[0] == '>') return 1;
	else return 0;
}
#define COMM_BUFFER_SIZE 80
void handleCommMessage(void)
{
	char buffer[COMM_BUFFER_SIZE];
	unsigned char function_code = 0;
	long i;
	buffer[64]=0;
	buffer[79]=0;
	UARTgets(buffer, COMM_BUFFER_SIZE);
	//UARTgetMessage(buffer);
	
	//UARTprintf("MESSAGE GET!!%s\r\n", buffer);

	if(isValidMessage(buffer))
	{
		for(i = 0; i < CODE_LENGTH; i++) //generate hash of function code
		{
			function_code ^= buffer[CODE_START + i];
		}

		switch(function_code)
		{
			case SVXA: JoystickXOut(SATURATE(atoi(&buffer[DATA_START]),0,255));
					   UARTprintf("GOOD MESSAGE- SVXA: %s\r\n", buffer);
					   ResetWatchdog(); //we got a valid message, so they are still talking to us
					   break;
			case SVYA: JoystickYOut(SATURATE(atoi(&buffer[DATA_START]),0,255));
					   UARTprintf("GOOD MESSAGE- SVYA: %s\r\n", buffer);
					   ResetWatchdog(); //we got a valid message, so they are still talking to us
					   break;
			default :  UARTprintf("UNRECOGNIZED MESSAGE: %s\r\n", buffer);
					   break;
		}
	}
	else
	{
		UARTprintf("INVALID START CHARACTER: %s\r\n", buffer);
	}
}

void InitializeUCSlave(void){
	JoystickInit();
	InitializeTime();
	InitializeWatchdog();
	InitializeUART();
}

void RunUCSlave(void){
	InitializeUCSlave();
	while(1) handleCommMessage();
	
}

/* [] END OF FILE */
