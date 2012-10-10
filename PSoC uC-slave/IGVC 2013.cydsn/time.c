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

#define SEC_PER_DAY 86400
#define SEC_PER_HOUR 3600
#define	SEC_PER_MIN 60
#define MS_PER_SEC 1000

uint32 time;
uint16 ms;

void SetClock(int hr, int min, int sec, int milli){
	time = (((hr * SEC_PER_HOUR) + (min * SEC_PER_MIN) + sec ) % SEC_PER_DAY );
	ms = milli;
}

void SetTime(uint32 t){ time = t; }
void SetMS(uint16 m){ ms = m; }
uint32 GetTime(void){ return time; }
int GetHour(void){ return time/SEC_PER_HOUR; }
int GetMin(void){ return time%SEC_PER_HOUR/SEC_PER_MIN; }
int GetSec(void){ return time%SEC_PER_MIN; }


void MainTimeISRHandler(void){
	MainTimeISR_ClearPending() ;
	ms = ( ms + 1 ) % MS_PER_SEC ;
	if(ms == 0) time++;
	if(time >= SEC_PER_DAY) time = 0;
}

void InitializeTime(void){
	ms = 0 ;
	time = 0 ;
	MainTimer_Start() ;
	MainTimeISR_StartEx(MainTimeISRHandler) ;
}

/* [] END OF FILE */
