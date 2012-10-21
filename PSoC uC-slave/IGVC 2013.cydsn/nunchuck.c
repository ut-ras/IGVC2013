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
#include <joystick.h>
#include <nunchuck.h>

void InitNunchuck(){
	uint8 init1[2] = {0xF0, 0x55};
	uint8 init2[2] = {0xFB, 0x00};
	uint8 finish = 0;
	Err_LED_1_Write(1);
	JoystickOut(128,128);
    while(finish==0){
		I2C_MasterClearStatus();
		I2C_MasterWriteBuf(0x52, init1, 2, I2C_MODE_COMPLETE_XFER);
        CyDelay(1);
		if (0u == (I2C_MasterStatus() & I2C_MSTAT_ERR_XFER)){
            I2C_MasterClearStatus();
			I2C_MasterWriteBuf(0x52, init2, 2, I2C_MODE_COMPLETE_XFER);
			CyDelay(1);
			if (0u == (I2C_MasterStatus() & I2C_MSTAT_ERR_XFER)){
				finish = 1;
			}	
        }
		CyDelay(100);
    }
	Err_LED_1_Write(0);
}

void RunNunchuck(void){
	uint8 clear[] = {0x00}; 
	uint8 buffer[16];
	unsigned short wiichuck[7];	
	Err_LED_1_Write(0);
	Err_LED_2_Write(0);
	JoystickInit();
	I2C_Start();
	InitNunchuck();
    for(;;)
    {
		I2C_MasterClearStatus();
		CyDelay(1);
        I2C_MasterWriteBuf(0x52, clear, 1, I2C_MODE_COMPLETE_XFER);
		CyDelay(1);
		I2C_MasterWriteBuf(0x52, clear, 1, I2C_MODE_COMPLETE_XFER);
		CyDelay(1);
		I2C_MasterWriteBuf(0x52, clear, 1, I2C_MODE_COMPLETE_XFER);
		CyDelay(1);
		if (0u != (I2C_MasterStatus() & I2C_MSTAT_ERR_XFER)){
			InitNunchuck();
        }else{
			I2C_MasterClearStatus();
			CyDelay(1);
            I2C_MasterReadBuf(0x52, buffer, 6, I2C_MODE_COMPLETE_XFER);   // Nunchuck data is 6 bytes, but for whatever reason, MEMOREX Wireless Nunchuck wants to send 8...
            CyDelay(1);
			if (0u != (I2C_MasterStatus() & I2C_MSTAT_ERR_XFER)){
                InitNunchuck();
            }else{
                wiichuck[0] = buffer[1];// X Axis Joystick
                wiichuck[1] = buffer[0];// Y Axis Joystick
                wiichuck[2] = (((unsigned short) buffer[2]) << 2) + (((unsigned short) buffer[5]) & (3<<2));    // X Axis Accel
                wiichuck[3] = (((unsigned short) buffer[3]) << 2) + (((unsigned short) buffer[5]) & (3<<4));    // Y Axis Accel
                wiichuck[4] = (((unsigned short) buffer[4]) << 2) + (((unsigned short) buffer[5]) & (3<<6));    // Z Axis Accel
                wiichuck[5] = buffer[5] & (1 << 1) ? 0 : 1;  //'C' Button 
                wiichuck[6] = buffer[5] & (1 << 0) ? 0 : 1;  //'Z' Button
				if(wiichuck[5]) JoystickOut(128,128);
				else JoystickOut(wiichuck[0],wiichuck[1]);
            }
		}
		Err_LED_2_Write(~Err_LED_2_Read());
		CyDelay(100);
    }
}

/* [] END OF FILE */

