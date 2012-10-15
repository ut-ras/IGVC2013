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
#include <nunchuck.h>
#include <joystick.h>
#include <servo.h>
#include <uart.h>
#include <time.h>

void main()
{
	int i = 0;
	uint16 j = 0;
	CyGlobalIntEnable;
	InitializeUART();
	uint8 str[100];
	
	for(;;){
		UARTprintf("\r\nTest: %d %d %d\r\n", i , i+1, i+2);
		i++;
		UARTprintf("Input>");
		j = UARTgets(str);
		UARTprintf("Length: %d\r\n", j);
		UARTprintf("Your input: %s \r\n", str);
		
		//CyDelay(1000);
	}
	
	
}

/* [] END OF FILE */
