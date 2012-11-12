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

void SetClock(int hr, int min, int sec, int milli);
void SetTime(uint32 t);
void SetMS(uint16 m);
uint16 GetMS(void);
uint32 GetTime(void);
int GetHour(void);
int GetMin(void);
int GetSec(void);
void SetMessageRate(uint16 in);
uint16 GetMessageRate(void);
void MainTimeISRHandler(void);
void ResetWatchdog(void);
void InitializeWatchdog(void);
void InitializeTime(void);

//[] END OF FILE
