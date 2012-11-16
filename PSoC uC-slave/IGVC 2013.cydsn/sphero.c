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
#include <sphero.h>
#include <uart.h>
#include <stdarg.h>
#include <time.h>

#define SPHERO_IMU_YAW      0x00010000
#define SPHERO_IMU_ROLL     0x00020000
#define SPHERO_IMU_PITCH    0x00040000
#define DATA_START 5
#define DOLLAR '$'

uint8 EnableIMUFeedbackMessages;
uint8 buffer[32];
int ptr;
uint8 sum;
int16 leng,roll,pitch,yaw;

int16 GetRoll(void){ return roll; }
int16 GetPitch(void){ return pitch; }
int16 GetYaw(void){ return yaw; }

void UpdateIMUValues(void){
	int16 tp,ty,tr;
	tp = (((int16)buffer[DATA_START])<<8) | (int16)buffer[DATA_START+1];
	ty = (((int16)buffer[DATA_START+2])<<8) | (int16)buffer[DATA_START+3];
	tr = (((int16)buffer[DATA_START+4])<<8) | (int16)buffer[DATA_START+5];
	if(tp >= -180 && tp <= 180 && ty >= -180 && ty <= 180 && tr >= -180 && tr <= 180 ){ //check if the numbers are actually valid
		pitch = tp;
		yaw = ty;
		roll = tr;
		if(EnableIMUFeedbackMessages) 
			UARTprintf( "^.- ROLL %d PTCH %d YAWW %d -.^\r\n", roll, pitch, yaw );
	}
}
void handleUARTRx(void){
	//while((IMU_ReadRxStatus()|IMU_RX_STS_FIFO_NOTEMPTY) == IMU_RX_STS_FIFO_NOTEMPTY){
		buffer[ptr] = IMU_GetChar();
		ptr++;
		if(ptr == 1){
			if(buffer[ptr-1] != 0xFF)
				sum = ptr = 0;
		}else if(ptr == 2){
			if(buffer[ptr-1] != 0xFE){
				sum = ptr = 0;
			}
		}
		else if(ptr == 4)
			leng = ((int16)buffer[ptr-1]) << 8;
		else if(ptr == 5){
			leng |= (int16)buffer[ptr-1];
		}else if(ptr >= 6){
			if(ptr >= (leng + 6)){ // handle checksum
				sum--;
				if(sum == buffer[ptr-1]){
					UpdateIMUValues();
					Err_LED_2_Write(~Err_LED_2_Read());
				}
				sum = ptr = 0;
			}
		}
		if(ptr == 32){ 
			sum = ptr = 0 ;
		}
	//}
}

void IMUInterruptHandler(void){
	IMU_Interrupt_ClearPending();
	//Err_LED_2_Write(~Err_LED_2_Read());
	handleUARTRx();
	//UARTprintf("At least I'm still interrupting!!\r\n");
}

char readSimplePacket(){
    int i;
    uint8 mrsp, seq, len, chksum;
	uint8 dat[32];
    // Wait for new data
    //while(Serial1.available() < 5);
    
    IMU_GetChar();         // 0xFF
    IMU_GetChar();         // 0xFF
    mrsp = IMU_GetChar();  // MRSP
    seq = IMU_GetChar();   // SEQ
    len = IMU_GetChar();   // DLEN
    
    if (len > 32)   // Error
        return -1;
    
    for(i=0; i<len-1; i++){
        // Wait for new data
        //while(!Serial1.available())
            //delay(1);
        
        // Add data to fifo
        dat[i] = IMU_GetChar();
    }
    
    // Wait for new data
    //while(!Serial1.available()); 
    chksum = IMU_GetChar();     // CHK
    
    return mrsp;
}

char sendCommand(char DID, char CID, char SEQ, char DLEN, ...){
    short i=0;
    char dat;
    va_list args;
    sum = 0;
    // Flush pipe (just in case)
    //Serial1.flush();
    
    // Write data
    IMU_PutChar(0xFF);
    IMU_PutChar(0xFF);
    IMU_PutChar(DID);
    IMU_PutChar(CID);
    IMU_PutChar(SEQ);
    IMU_PutChar(DLEN);
    
    // Calculate checksum
    sum += DID + CID + SEQ + DLEN;
    
    va_start(args, DLEN);
    for(; i<DLEN-1; i++){
        dat = va_arg(args, int);
        IMU_PutChar(dat);
        
        sum += dat;
    }
    va_end(args);
    IMU_PutChar(~sum);
    // Wait for Simple Response
    return readSimplePacket();
}

 typedef struct {
    char count;
    char numOfPackets;
    
    short freq;
    short frames_per_sample;
    long mask;
  } StreamingParams;
    StreamingParams streamingParams;
  
void InitializeSphero(void){
	uint8 stuff=0;
	uint32 t;
	CyDelay(500);
	Err_LED_2_Write(1);
	IMU_PutChar('$');
	IMU_PutChar('$');
	IMU_PutChar('$');
	t = GetTime();
	do{stuff = IMU_GetChar();}while(!stuff && (( t + 10) > GetTime()));
	if(( t + 10) <= GetTime()) return; // timeout on trying to send to UART
	do{stuff = IMU_GetChar();}while(!stuff);
	do{stuff = IMU_GetChar();}while(!stuff);
	IMU_PutChar('C');
	IMU_PutChar('\n');
	CyDelay(10000);
	IMU_PutChar('-');
	IMU_PutChar('-');
	IMU_PutChar('-');
	IMU_PutChar('\r');
	IMU_ClearRxBuffer();
	Err_LED_2_Write(0);
	
	// Indicator LED
    sendCommand(0x02, 0x21, 0x04, 0x02, 0xFF);
  
    // Lock motors
    sendCommand(0x02, 0x33, 0x0A, 0x05, 0x04, 0x00, 0x04, 0x00);
    sendCommand(0x02, 0x02, 0x09, 0x02, 0x00);
  
    // Get Data @ 10Hz
    streamingParams.count = 0;
    streamingParams.freq = 100;
    streamingParams.frames_per_sample = 1;
    streamingParams.mask = SPHERO_IMU_PITCH | SPHERO_IMU_ROLL | SPHERO_IMU_YAW;
    sendCommand(0x02, 0x11, 0x07, 0x0A, 
				(400/streamingParams.freq) >> 8, 
				(400/streamingParams.freq), 
				streamingParams.frames_per_sample >> 8, 
				streamingParams.frames_per_sample, 
				streamingParams.mask >> 24, 
				streamingParams.mask >> 16, 
				streamingParams.mask >> 8, 
				streamingParams.mask, 
				streamingParams.numOfPackets);
				
	CyDelay(50);
	IMU_ClearRxBuffer();
}

void InitializeIMU(void){
	pitch = 0;
	roll = 0;
	yaw = 0;
	ptr = 0;
	IMU_Start();	
	InitializeSphero();
	IMU_Interrupt_StartEx(IMUInterruptHandler);
	EnableIMUFeedbackMessages = 0;
}

/* [] END OF FILE */
