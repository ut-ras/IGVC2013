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
#include <imudriver.h>
#include <uart.h>

#define IMU_MESSAGE_SIZE 23
#define OFFSET_RAWMAG_0 1
#define OFFSET_RAWMAG_1 3
#define OFFSET_RAWMAG_2 5
#define OFFSET_RAWACCEL_0 7
#define OFFSET_RAWACCEL_1 9
#define OFFSET_RAWACCEL_2 11
#define OFFSET_RAWANGLE_0 13
#define OFFSET_RAWANGLE_1 15
#define OFFSET_RAWANGLE_2 17
#define OFFSET_TIME 19

uint8 buffer[23];
uint8 ptr;
int16 RawValues[10]; // RawMag(0,1,2), RawAccel(0,1,2), RawAngRate(0,1,2), Timestamp
uint8 EnableIMUFeedbackMessages;
//messages back are little endian
void UpdateIMUValues(void){
	RawValues[0] = (((uint16)buffer[OFFSET_RAWMAG_0]) << 8) + buffer[OFFSET_RAWMAG_0+1];
	RawValues[1] = (((uint16)buffer[OFFSET_RAWMAG_1]) << 8) + buffer[OFFSET_RAWMAG_1+1];
	RawValues[2] = (((uint16)buffer[OFFSET_RAWMAG_2]) << 8) + buffer[OFFSET_RAWMAG_2+1];
	RawValues[3] = (((uint16)buffer[OFFSET_RAWACCEL_0]) << 8) + buffer[OFFSET_RAWACCEL_0+1];
	RawValues[4] = (((uint16)buffer[OFFSET_RAWACCEL_1]) << 8) + buffer[OFFSET_RAWACCEL_1+1];
	RawValues[5] = (((uint16)buffer[OFFSET_RAWACCEL_2]) << 8) + buffer[OFFSET_RAWACCEL_2+1];
	RawValues[6] = (((uint16)buffer[OFFSET_RAWANGLE_0]) << 8) + buffer[OFFSET_RAWANGLE_0+1];
	RawValues[7] = (((uint16)buffer[OFFSET_RAWANGLE_1]) << 8) + buffer[OFFSET_RAWANGLE_1+1];
	RawValues[8] = (((uint16)buffer[OFFSET_RAWANGLE_2]) << 8) + buffer[OFFSET_RAWANGLE_2+1];
	RawValues[9] = (((uint16)buffer[OFFSET_TIME]) << 8) + buffer[OFFSET_TIME+1];
	if(EnableIMUFeedbackMessages) 
		UARTprintf("^.- MAGX %d MAGY %d MAGZ %d ACCX %d ACCY %d ACCZ %d ANGX %d ANGY %d ANGZ %d TIME %d -.^\r\n", 
		RawValues[0],RawValues[1],RawValues[2],RawValues[3],RawValues[4],RawValues[5],RawValues[6],RawValues[7],RawValues[8],RawValues[9]);
}

void IMUInterruptHandler(void){
	IMU_Interrupt_ClearPending();
	buffer[ptr] = IMU_GetChar();
	ptr++;
	if(ptr == IMU_MESSAGE_SIZE){// recieved a full message
		UpdateIMUValues();
		ptr = 0;
	}
}

void InitializeIMU(void){
	ptr = 0;
	IMU_Start();
	IMU_WriteControlRegister(IMU_ReadControlRegister()|IMU_CTRL_MARK);
	IMU_Interrupt_StartEx(IMUInterruptHandler);
	IMU_PutChar(0x10); // Continuous mode
	IMU_PutChar(0x00); // Null
	IMU_PutChar(0x01); // Raw Values
	EnableIMUFeedbackMessages = 1;
}

/* [] END OF FILE */
