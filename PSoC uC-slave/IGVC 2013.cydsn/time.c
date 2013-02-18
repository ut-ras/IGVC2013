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
#include <servo.h>

#define SEC_PER_DAY 86400
#define SEC_PER_HOUR 3600
#define	SEC_PER_MIN 60
#define MS_PER_SEC 1000
#define WATCHDOG_TIMEOUT 200

uint32 time; //current time in sec past 0:00
uint16 ms; //millisecond counter
uint32 WatchdogTime; //watchdog timeout
uint8 WatchdogRunning; //boolean representing if WD is running
uint8 WatchdogOverflow;
uint8 VelCtrlRunning;
uint16 VelCtrlRate;
uint8 HokuyoTiltRunning;
uint16 HokuyoTiltRate;

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
void SetMessageRate(uint16 in) { VelCtrlRate = in; }
void SetHokuyoTiltRate(uint16 in) { HokuyoTiltRate = in; }
void EnableHokuyoTilting(void){ HokuyoTiltRunning = 1; }
void DisableHokuyoTilting(void){ HokuyoTiltRunning = 0; }
uint16 GetMessageRate(void) { return VelCtrlRate; }
void WatchdogTimeout(void){
	SetLeftMotor(0); //stop the bot
	SetRightMotor(0);
	VelCtrlRunning = 0;
	ClearVelocityControl();
	Err_LED_1_Write(1);
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
	if((ms % VelCtrlRate) == 0){
		UpdateVelocity();
		sendCommMessage();
		if(VelCtrlRunning) RunVelocityControl();
	}
	if((ms % HokuyoTiltRate) == 0){
		if(HokuyoTiltRunning) HokuyoTiltStep();
	}
}

void ResetWatchdog(void){
	if( ms + WATCHDOG_TIMEOUT >= MS_PER_SEC) WatchdogOverflow = 1;
	WatchdogTime = (( ms + WATCHDOG_TIMEOUT ) % MS_PER_SEC);
	Err_LED_1_Write(0);
}

#define DEFAULT_VEL_CTRL_RATE 20
void InitializeWatchdog(void){
	WatchdogRunning = 1;
	VelCtrlRunning = 0;
	VelCtrlRate = DEFAULT_VEL_CTRL_RATE;
	ResetWatchdog();
}
#define DEFAULT_HOKUYO_TILT_RATE 20
void InitializeTime(void){
	ms = 0 ;
	time = 0 ;
	HokuyoTiltRunning = 1;
	HokuyoTiltRate = DEFAULT_HOKUYO_TILT_RATE;
	MainTimer_Start() ;
	MainTimeISR_StartEx(MainTimeISRHandler) ;
}

/* [] END OF FILE */
