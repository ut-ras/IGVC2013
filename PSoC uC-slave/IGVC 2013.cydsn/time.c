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
#include <time.h>
#include <joystick.h>
#include <velctrl.h>
#include <uartComSlave.h>

#define SEC_PER_DAY 86400
#define SEC_PER_HOUR 3600
#define	SEC_PER_MIN 60
#define MS_PER_SEC 1000
#define WATCHDOG_TIMEOUT 200
#define VELOCITY_UPDATE 20 //needs to be a factor of 1000 (MS_PER_SEC) in current implementation

uint32 time; //current time in sec past 0:00
uint16 ms; //millisecond counter
uint32 WatchdogTime; //watchdog timeout
uint8 WatchdogRunning; //boolean representing if WD is running
uint8 WatchdogOverflow;
uint8 VelCtrlRunning;

void SetClock(int hr, int min, int sec, int milli){
	time = (((hr * SEC_PER_HOUR) + (min * SEC_PER_MIN) + sec ) % SEC_PER_DAY );
	ms = milli;
}
void SetTime(uint32 t){ time = t; }
void SetMS(uint16 m){ ms = m; }
uint16 GetMS(void){ return ms; }
uint32 GetTime(void){ return time; }
int GetHour(void){ return time/SEC_PER_HOUR; }
int GetMin(void){ return time%SEC_PER_HOUR/SEC_PER_MIN; }
int GetSec(void){ return time%SEC_PER_MIN; }
 
void WatchdogTimeout(void){
	JoystickOut(128,128); //stop the bot
	VelCtrlRunning = 0;
	Err_LED_1_Write(0);
}

void MainTimeISRHandler(void){
	MainTimeISR_ClearPending() ;
	ms = ( ms + 1 ) % MS_PER_SEC ;
	if(ms == 0){ 
		time++;
		WatchdogOverflow = 0;
	}
	if(time >= SEC_PER_DAY) time = 0;
	if(WatchdogRunning && !WatchdogOverflow && (WatchdogTime <= ms)) WatchdogTimeout();
	if((ms % VELOCITY_UPDATE) == 0){
		UpdateVelocity();
		sendCommMessage();
		if(VelCtrlRunning) RunVelocityControl();
	}
}

void ResetWatchdog(void){
	if( ms + WATCHDOG_TIMEOUT >= MS_PER_SEC) WatchdogOverflow = 1;
	WatchdogTime = (( ms + WATCHDOG_TIMEOUT ) % MS_PER_SEC);
	Err_LED_1_Write(1);
}

void InitializeWatchdog(void){
	WatchdogRunning = 1;
	VelCtrlRunning = 0;
	ResetWatchdog();
}

void InitializeTime(void){
	ms = 0 ;
	time = 0 ;
	MainTimer_Start() ;
	MainTimeISR_StartEx(MainTimeISRHandler) ;
}

/* [] END OF FILE */
