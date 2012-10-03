/*******************************************************************************
* File Name: Servo.c  
* Version 2.10
*
* Description:
*  The PWM User Module consist of an 8 or 16-bit counter with two 8 or 16-bit
*  comparitors. Each instance of this user module is capable of generating
*  two PWM outputs with the same period. The pulse width is selectable between
*  1 and 255/65535. The period is selectable between 2 and 255/65536 clocks. 
*  The compare value output may be configured to be active when the present 
*  counter is less than or less than/equal to the compare value.
*  A terminal count output is also provided. It generates a pulse one clock
*  width wide when the counter is equal to zero.
*
* Note:
*
*******************************************************************************
* Copyright 2008-2011, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

#include "cytypes.h"
#include "Servo.h"

uint8 Servo_initVar = 0u;

/*******************************************************************************
* Function Name: Servo_Start
********************************************************************************
*
* Summary:
*  The start function initializes the pwm with the default values, the 
*  enables the counter to begin counting.  It does not enable interrupts,
*  the EnableInt command should be called if interrupt generation is required.
*
* Parameters:  
*  void  
*
* Return: 
*  void
*
* Global variables:
*  Servo_initVar: Is modified when this function is called for the first 
*   time. Is used to ensure that initialization happens only once.
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(Servo_initVar == 0u)
    {
        Servo_Init();
        Servo_initVar = 1u;
    }
    Servo_Enable();

}


/*******************************************************************************
* Function Name: Servo_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the 
*  customizer of the component placed onto schematic. Usually called in 
*  Servo_Start().
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_Init(void) 
{
    #if (Servo_UsingFixedFunction || Servo_UseControl)
        uint8 ctrl;
    #endif
    
    #if(!Servo_UsingFixedFunction) 
        #if(Servo_UseStatus)
            /* Interrupt State Backup for Critical Region*/
            uint8 Servo_interruptState;
        #endif
    #endif
    
   #if (Servo_UsingFixedFunction)
        /* You are allowed to write the compare value (FF only) */
        Servo_CONTROL |= Servo_CFG0_MODE;
        #if (Servo_DeadBand2_4)
            Servo_CONTROL |= Servo_CFG0_DB;
        #endif
                
        /* Set the default Compare Mode */
        #if(Servo_PSOC3_ES2 || Servo_PSOC5_ES1)
                ctrl = Servo_CONTROL2 & ~Servo_CTRL_CMPMODE1_MASK;
                Servo_CONTROL2 = ctrl | Servo_DEFAULT_COMPARE1_MODE;
        #endif
        #if(Servo_PSOC3_ES3 || Servo_PSOC5_ES2)
                ctrl = Servo_CONTROL3 & ~Servo_CTRL_CMPMODE1_MASK;
                Servo_CONTROL3 = ctrl | Servo_DEFAULT_COMPARE1_MODE;
        #endif
        
         /* Clear and Set SYNCTC and SYNCCMP bits of RT1 register */
        Servo_RT1 &= ~Servo_RT1_MASK;
        Servo_RT1 |= Servo_SYNC;     
                
        /*Enable DSI Sync all all inputs of the PWM*/
        Servo_RT1 &= ~(Servo_SYNCDSI_MASK);
        Servo_RT1 |= Servo_SYNCDSI_EN;
       
    #elif (Servo_UseControl)
        /* Set the default compare mode defined in the parameter */
        ctrl = Servo_CONTROL & ~Servo_CTRL_CMPMODE2_MASK & ~Servo_CTRL_CMPMODE1_MASK;
        Servo_CONTROL = ctrl | Servo_DEFAULT_COMPARE2_MODE | Servo_DEFAULT_COMPARE1_MODE;
    #endif 
        
    #if (!Servo_UsingFixedFunction)
        #if (Servo_Resolution == 8)
            /* Set FIFO 0 to 1 byte register for period*/
            Servo_AUX_CONTROLDP0 |= (Servo_AUX_CTRL_FIFO0_CLR);
        #else /* (Servo_Resolution == 16)*/
            /* Set FIFO 0 to 1 byte register for period */
            Servo_AUX_CONTROLDP0 |= (Servo_AUX_CTRL_FIFO0_CLR);
            Servo_AUX_CONTROLDP1 |= (Servo_AUX_CTRL_FIFO0_CLR);
        #endif
    #endif
        
    Servo_WritePeriod(Servo_INIT_PERIOD_VALUE);
    Servo_WriteCounter(Servo_INIT_PERIOD_VALUE);
        
        #if (Servo_UseOneCompareMode)
            Servo_WriteCompare(Servo_INIT_COMPARE_VALUE1);
        #else
            Servo_WriteCompare1(Servo_INIT_COMPARE_VALUE1);
            Servo_WriteCompare2(Servo_INIT_COMPARE_VALUE2);
        #endif
        
        #if (Servo_KillModeMinTime)
            Servo_WriteKillTime(Servo_MinimumKillTime);
        #endif
        
        #if (Servo_DeadBandUsed)
            Servo_WriteDeadTime(Servo_INIT_DEAD_TIME);
        #endif

    #if (Servo_UseStatus || Servo_UsingFixedFunction)
        Servo_SetInterruptMode(Servo_INIT_INTERRUPTS_MODE);
    #endif
        
    #if (Servo_UsingFixedFunction)
        /* Globally Enable the Fixed Function Block chosen */
        Servo_GLOBAL_ENABLE |= Servo_BLOCK_EN_MASK;
        /* Set the Interrupt source to come from the status register */
        Servo_CONTROL2 |= Servo_CTRL2_IRQ_SEL;
    #else
        #if(Servo_UseStatus)
            
            /* CyEnterCriticalRegion and CyExitCriticalRegion are used to mark following region critical*/
            /* Enter Critical Region*/
            Servo_interruptState = CyEnterCriticalSection();
            /* Use the interrupt output of the status register for IRQ output */
            Servo_STATUS_AUX_CTRL |= Servo_STATUS_ACTL_INT_EN_MASK;
            
             /* Exit Critical Region*/
            CyExitCriticalSection(Servo_interruptState);
            
            /* Clear the FIFO to enable the Servo_STATUS_FIFOFULL
                   bit to be set on FIFO full. */
            Servo_ClearFIFO();
        #endif
    #endif
}


/*******************************************************************************
* Function Name: Servo_Enable
********************************************************************************
*
* Summary: 
*  Enables the PWM block operation
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Side Effects: 
*  This works only if software enable mode is chosen
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_Enable(void) 
{
    /* Globally Enable the Fixed Function Block chosen */
    #if (Servo_UsingFixedFunction)
        Servo_GLOBAL_ENABLE |= Servo_BLOCK_EN_MASK;
        Servo_GLOBAL_STBY_ENABLE |= Servo_BLOCK_STBY_EN_MASK;
    #endif 
    
    /* Enable the PWM from the control register  */
    #if (Servo_UseControl || Servo_UsingFixedFunction)
        Servo_CONTROL |= Servo_CTRL_ENABLE;
    #endif
}


/*******************************************************************************
* Function Name: Servo_Stop
********************************************************************************
*
* Summary:
*  The stop function halts the PWM, but does not change any modes or disable
*  interrupts.
*
* Parameters:  
*  void  
*
* Return: 
*  void
*
* Side Effects:
*  If the Enable mode is set to Hardware only then this function
*  has no effect on the operation of the PWM
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_Stop(void) 
{
    #if (Servo_UseControl || Servo_UsingFixedFunction)
        Servo_CONTROL &= ~Servo_CTRL_ENABLE;
    #endif
    
    /* Globally disable the Fixed Function Block chosen */
    #if (Servo_UsingFixedFunction)
        Servo_GLOBAL_ENABLE &= ~Servo_BLOCK_EN_MASK;
        Servo_GLOBAL_STBY_ENABLE &= ~Servo_BLOCK_STBY_EN_MASK;
    #endif
}


#if (Servo_UseOneCompareMode)
#if (Servo_CompareMode1SW)
/*******************************************************************************
* Function Name: Servo_SetCompareMode
********************************************************************************
* 
* Summary:
*  This function writes the Compare Mode for the pwm output when in Dither mode,
*  Center Align Mode or One Output Mode.
*
* Parameters:
*  comparemode:  The new compare mode for the PWM output. Use the compare types
*                defined in the H file as input arguments.
*
* Return:
*  void
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_SetCompareMode(uint8 comparemode) 
{
    #if(Servo_UsingFixedFunction)
            #if(Servo_PSOC3_ES2 || Servo_PSOC5_ES1)
                        uint8 comparemodemasked = (comparemode << Servo_CTRL_CMPMODE1_SHIFT);
                        Servo_CONTROL2 &= ~Servo_CTRL_CMPMODE1_MASK; /*Clear Existing Data */
                        Servo_CONTROL2 |= comparemodemasked;  
                #endif
                
            #if(Servo_PSOC3_ES3 || Servo_PSOC5_ES2)
                    uint8 comparemodemasked = (comparemode << Servo_CTRL_CMPMODE1_SHIFT);
            Servo_CONTROL3 &= ~Servo_CTRL_CMPMODE1_MASK; /*Clear Existing Data */
            Servo_CONTROL3 |= comparemodemasked;     
                #endif
                
    #elif (Servo_UseControl)
        uint8 comparemode1masked = (comparemode << Servo_CTRL_CMPMODE1_SHIFT) & Servo_CTRL_CMPMODE1_MASK;
        uint8 comparemode2masked = (comparemode << Servo_CTRL_CMPMODE2_SHIFT) & Servo_CTRL_CMPMODE2_MASK;
        Servo_CONTROL &= ~(Servo_CTRL_CMPMODE1_MASK | Servo_CTRL_CMPMODE2_MASK); /*Clear existing mode */
        Servo_CONTROL |= (comparemode1masked | comparemode2masked);
        
    #else
        uint8 temp = comparemode;
    #endif
}
#endif /* Servo_CompareMode1SW */

#else /* UseOneCompareMode */


#if (Servo_CompareMode1SW)
/*******************************************************************************
* Function Name: Servo_SetCompareMode1
********************************************************************************
* 
* Summary:
*  This function writes the Compare Mode for the pwm or pwm1 output
*
* Parameters:  
*  comparemode:  The new compare mode for the PWM output. Use the compare types
*                defined in the H file as input arguments.
*
* Return: 
*  void
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_SetCompareMode1(uint8 comparemode) 
{
    uint8 comparemodemasked = (comparemode << Servo_CTRL_CMPMODE1_SHIFT) & Servo_CTRL_CMPMODE1_MASK;
    #if(Servo_UsingFixedFunction)
            #if(Servo_PSOC3_ES2 || Servo_PSOC5_ES1)
                        Servo_CONTROL2 &= Servo_CTRL_CMPMODE1_MASK; /*Clear existing mode */
                        Servo_CONTROL2 |= comparemodemasked; 
            #endif
                
                #if(Servo_PSOC3_ES3 || Servo_PSOC5_ES2)
                    Servo_CONTROL3 &= Servo_CTRL_CMPMODE1_MASK; /*Clear existing mode */
                        Servo_CONTROL3 |= comparemodemasked; 
            #endif
                
    #elif (Servo_UseControl)
        Servo_CONTROL &= Servo_CTRL_CMPMODE1_MASK; /*Clear existing mode */
        Servo_CONTROL |= comparemodemasked;
    #endif    
}
#endif /* Servo_CompareMode1SW */


#if (Servo_CompareMode2SW)
/*******************************************************************************
* Function Name: Servo_SetCompareMode2
********************************************************************************
* 
* Summary:
*  This function writes the Compare Mode for the pwm or pwm2 output
*
* Parameters:  
*  comparemode:  The new compare mode for the PWM output. Use the compare types
*                defined in the H file as input arguments.
*
* Return: 
*  void
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_SetCompareMode2(uint8 comparemode) 
{
    #if(Servo_UsingFixedFunction)
        /* Do Nothing because there is no second Compare Mode Register in FF block*/ 
    #elif (Servo_UseControl)
    uint8 comparemodemasked = (comparemode << Servo_CTRL_CMPMODE2_SHIFT) & Servo_CTRL_CMPMODE2_MASK;
    Servo_CONTROL &= Servo_CTRL_CMPMODE2_MASK; /*Clear existing mode */
    Servo_CONTROL |= comparemodemasked;
    #endif    
}
#endif /*Servo_CompareMode2SW */
#endif /* UseOneCompareMode */


/*******************************************************************************
* Function Name: Servo_WriteCounter
********************************************************************************
* 
* Summary:
*  This function is used to change the counter value.
*
* Parameters:  
*  counter:  This value may be between 1 and (2^Resolution)-1.   
*
* Return: 
*  void
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_WriteCounter(uint16 counter) 
{
    #if(Servo_UsingFixedFunction)
        CY_SET_REG16(Servo_COUNTER_LSB_PTR, (uint16)counter);
    #else
        CY_SET_REG16(Servo_COUNTER_LSB_PTR, counter);
    #endif
}


#if (!Servo_UsingFixedFunction)
/*******************************************************************************
* Function Name: Servo_ReadCounter
********************************************************************************
* 
* Summary:
*  This function returns the current value of the counter.  It doesn't matter
*  if the counter is enabled or running.
*
* Parameters:  
*  void  
*
* Return: 
*  The current value of the counter.
*
* Reentrant:
*  Yes
*
*******************************************************************************/
uint16 Servo_ReadCounter(void) 
{
    /* Force capture by reading Accumulator */
    /* Must first do a software capture to be able to read the counter */
    /* It is up to the user code to make sure there isn't already captured data in the FIFO */
    CY_GET_REG8(Servo_COUNTERCAP_LSB_PTR);
    
    /* Read the data from the FIFO (or capture register for Fixed Function)*/
    return (CY_GET_REG16(Servo_CAPTURE_LSB_PTR));
}


#if (Servo_UseStatus)
/*******************************************************************************
* Function Name: Servo_ClearFIFO
********************************************************************************
* 
* Summary:
*  This function clears all capture data from the capture FIFO
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_ClearFIFO(void) 
{
    while(Servo_ReadStatusRegister() & Servo_STATUS_FIFONEMPTY)
        Servo_ReadCapture();
}
#endif /* Servo_UseStatus */
#endif /* !Servo_UsingFixedFunction */


/*******************************************************************************
* Function Name: Servo_WritePeriod
********************************************************************************
* 
* Summary:
*  This function is used to change the period of the counter.  The new period 
*  will be loaded the next time terminal count is detected.
*
* Parameters:  
*  period:  Period value. May be between 1 and (2^Resolution)-1.  A value of 0 
*           will result in the counter remaining at zero.
*
* Return: 
*  void
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_WritePeriod(uint16 period) 
{
    #if(Servo_UsingFixedFunction)
        CY_SET_REG16(Servo_PERIOD_LSB_PTR, (uint16)period);
    #else
        CY_SET_REG16(Servo_PERIOD_LSB_PTR, period);
    #endif
}


#if (Servo_UseOneCompareMode)
/*******************************************************************************
* Function Name: Servo_WriteCompare
********************************************************************************
* 
* Summary:
*  This funtion is used to change the compare1 value when the PWM is in Dither
*  mode. The compare output will reflect the new value on the next UDB clock. 
*  The compare output will be driven high when the present counter value is 
*  compared to the compare value based on the compare mode defined in 
*  Dither Mode.
*
* Parameters:  
*  compare:  New compare value.  
*
* Return: 
*  void
*
* Side Effects:
*  This function is only available if the PWM mode parameter is set to
*  Dither Mode, Center Aligned Mode or One Output Mode
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_WriteCompare(uint16 compare) 
{
   CY_SET_REG16(Servo_COMPARE1_LSB_PTR, compare);
   #if (Servo_PWMMode == Servo__B_PWM__DITHER)
        CY_SET_REG16(Servo_COMPARE2_LSB_PTR, compare+1);
   #endif
}


#else


/*******************************************************************************
* Function Name: Servo_WriteCompare1
********************************************************************************
* 
* Summary:
*  This funtion is used to change the compare1 value.  The compare output will 
*  reflect the new value on the next UDB clock.  The compare output will be 
*  driven high when the present counter value is less than or less than or 
*  equal to the compare register, depending on the mode.
*
* Parameters:  
*  compare:  New compare value.  
*
* Return: 
*  void
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_WriteCompare1(uint16 compare) 
{
    #if(Servo_UsingFixedFunction)
        CY_SET_REG16(Servo_COMPARE1_LSB_PTR, (uint16)compare);
    #else
        CY_SET_REG16(Servo_COMPARE1_LSB_PTR, compare);
    #endif
}


/*******************************************************************************
* Function Name: Servo_WriteCompare2
********************************************************************************
* 
* Summary:
*  This funtion is used to change the compare value, for compare1 output.  
*  The compare output will reflect the new value on the next UDB clock.  
*  The compare output will be driven high when the present counter value is 
*  less than or less than or equal to the compare register, depending on the 
*  mode.
*
* Parameters:  
*  compare:  New compare value.  
*
* Return: 
*  void
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_WriteCompare2(uint16 compare) 
{
    #if(Servo_UsingFixedFunction)
        CY_SET_REG16(Servo_COMPARE2_LSB_PTR, compare);
    #else
        CY_SET_REG16(Servo_COMPARE2_LSB_PTR, compare);
    #endif
}
#endif /* UseOneCompareMode */


#if (Servo_DeadBandUsed)
/*******************************************************************************
* Function Name: Servo_WriteDeadTime
********************************************************************************
* 
* Summary:
*  This function writes the dead-band counts to the corresponding register
*
* Parameters:  
*  deadtime:  Number of counts for dead time 
*
* Return: 
*  void
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_WriteDeadTime(uint8 deadtime) 
{
    /* If using the Dead Band 1-255 mode then just write the register */
    #if(!Servo_DeadBand2_4)
        CY_SET_REG8(Servo_DEADBAND_COUNT_PTR, deadtime);
    #else
        /* Otherwise the data has to be masked and offset */        
        /* Clear existing data */
        Servo_DEADBAND_COUNT &= ~Servo_DEADBAND_COUNT_MASK; 
            /* Set new dead time */
        Servo_DEADBAND_COUNT |= (deadtime << Servo_DEADBAND_COUNT_SHIFT) & Servo_DEADBAND_COUNT_MASK; 
    #endif
}


/*******************************************************************************
* Function Name: Servo_ReadDeadTime
********************************************************************************
* 
* Summary:
*  This function reads the dead-band counts from the corresponding register
*
* Parameters:  
*  void
*
* Return: 
*  Dead Band Counts
*
* Reentrant:
*  Yes
*
*******************************************************************************/
uint8 Servo_ReadDeadTime(void) 
{
    /* If using the Dead Band 1-255 mode then just read the register */
    #if(!Servo_DeadBand2_4)
        return (CY_GET_REG8(Servo_DEADBAND_COUNT_PTR));
    #else
        /* Otherwise the data has to be masked and offset */
        return ((Servo_DEADBAND_COUNT & Servo_DEADBAND_COUNT_MASK) >> Servo_DEADBAND_COUNT_SHIFT);
    #endif
}
#endif /* DeadBandUsed */

