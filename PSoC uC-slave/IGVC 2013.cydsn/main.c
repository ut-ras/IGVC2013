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

void Servos_Out(uint8 x, uint8 y){
	Servo_WriteCompare1(x?1500:4500);
	Servo_WriteCompare2(y?1500:4500);
}

// Input 0-255
// DAC out range 78 (1.25V) to 234 (3.75V)
void Joystick_Out(uint8 x, uint8 y){ 
	unsigned short xOut = ((((unsigned short) x) * 156) / 256) + 78;
	unsigned short yOut = ((((unsigned short) y) * 156) / 256) + 78;
	VDAC8_1_SetValue(yOut);
	VDAC8_2_SetValue(xOut);
}

void Button_1_InterruptHandler(){
	Button_1_ClearPending();
	//Err_LED_1_Write(1);
	Joystick_Out(128,255);
}
void Button_2_InterruptHandler(){
	Button_2_ClearPending();
	//Err_LED_1_Write(0);
	Joystick_Out(128,128);
}

void JoystickTest(){
	Joystick_Out(128,128);
	VDAC8_3_SetValue(156);
	for(;;){
		
		Err_LED_1_Write(CyPins_ReadPin(Test_Button_1_0)) ;
		Err_LED_2_Write(Test_Button_1_Read()!=0) ;
		if(Test_Button_1_Read()!=0) Joystick_Out(255,128);
		//else if(Test_Button_2_Read()!=0) Joystick_Out(128,255);
		else Joystick_Out(128,128);
		CyDelay(10);
	}
}


void init_nunchuck(){
	uint8 init1[2] = {0xF0, 0x55};
	uint8 init2[2] = {0xFB, 0x00};
	uint8 finish = 0;
	Err_LED_1_Write(1);
	Joystick_Out(128,128);
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

void main()
{
	uint8 clear[] = {0x00}; 
	uint8 buffer[16];
	unsigned short wiichuck[7];
	VDAC8_1_Start();
    VDAC8_2_Start();
	VDAC8_3_Start();
	Servo_Start();
	Err_LED_1_Write(0);
	Err_LED_2_Write(0);
	CyGlobalIntEnable;
	I2C_Start();
	Joystick_Out(128,128);
	VDAC8_3_SetValue(156);
	init_nunchuck();
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
			init_nunchuck();
        }else{
			I2C_MasterClearStatus();
			CyDelay(1);
            I2C_MasterReadBuf(0x52, buffer, 6, I2C_MODE_COMPLETE_XFER);   // Nunchuck data is 6 bytes, but for whatever reason, MEMOREX Wireless Nunchuck wants to send 8...
            CyDelay(1);
			if (0u != (I2C_MasterStatus() & I2C_MSTAT_ERR_XFER)){
                init_nunchuck();
            }else{
                wiichuck[0] = buffer[1];// X Axis Joystick
                wiichuck[1] = buffer[0];// Y Axis Joystick
                wiichuck[2] = (((unsigned short) buffer[2]) << 2) + (((unsigned short) buffer[5]) & (3<<2));    // X Axis Accel
                wiichuck[3] = (((unsigned short) buffer[3]) << 2) + (((unsigned short) buffer[5]) & (3<<4));    // Y Axis Accel
                wiichuck[4] = (((unsigned short) buffer[4]) << 2) + (((unsigned short) buffer[5]) & (3<<6));    // Z Axis Accel
                wiichuck[5] = buffer[5] & (1 << 1) ? 0 : 1;  //'C' Button 
                wiichuck[6] = buffer[5] & (1 << 0) ? 0 : 1;  //'Z' Button
				if(wiichuck[5]) Joystick_Out(128,128);
				else Joystick_Out(wiichuck[0],wiichuck[1]);
				Servos_Out(wiichuck[5],wiichuck[6]);
            }
		}
		Err_LED_2_Write(~Err_LED_2_Read());
		CyDelay(100);
    }
}

/* [] END OF FILE */
