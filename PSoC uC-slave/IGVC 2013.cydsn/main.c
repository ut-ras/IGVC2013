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
#include <uartComSlave.h>
#include <encoder.h>
#include <sphero.h>

void main()
{
	CyGlobalIntEnable;
	//RunNunchuck();
	RunUCSlave();
	for(;;);
	
	
}

/* [] END OF FILE */
