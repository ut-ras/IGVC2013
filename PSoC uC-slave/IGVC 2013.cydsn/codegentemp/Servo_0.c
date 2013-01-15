/*******************************************************************************
* File Name: Servo_0.c  
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
#include "Servo_0.h"

uint8 Servo_0_initVar = 0u;

/*******************************************************************************
* Function Name: Servo_0_Start
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
*  Servo_0_initVar: Is modified when this function is called for the first 
*   time. Is used to ensure that initialization happens only once.
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_0_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(Servo_0_initVar == 0u)
    {
        Servo_0_Init();
        Servo_0_initVar = 1u;
    }
    Servo_0_Enable();

}


/*******************************************************************************
* Function Name: Servo_0_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the 
*  customizer of the component placed onto schematic. Usually called in 
*  Servo_0_Start().
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
void Servo_0_Init(void) 
{
    #if (Servo_0_UsingFixedFunction || Servo_0_UseControl)
        uint8 ctrl;
    #endif
    
    #if(!Servo_0_UsingFixedFunction) 
        #if(Servo_0_UseStatus)
            /* Interrupt State Backup for Critical Region*/
            uint8 Servo_0_interruptState;
        #endif
    #endif
    
   #if (Servo_0_UsingFixedFunction)
        /* You are allowed to write the compare value (FF only) */
        Servo_0_CONTROL |= Servo_0_CFG0_MODE;
        #if (Servo_0_DeadBand2_4)
            Servo_0_CONTROL |= Servo_0_CFG0_DB;
        #endif
                
        /* Set the default Compare Mode */
        #if(Servo_0_PSOC3_ES2 || Servo_0_PSOC5_ES1)
                ctrl = Servo_0_CONTROL2 & ~Servo_0_CTRL_CMPMODE1_MASK;
                Servo_0_CONTROL2 = ctrl | Servo_0_DEFAULT_COMPARE1_MODE;
        #endif
        #if(Servo_0_PSOC3_ES3 || Servo_0_PSOC5_ES2)
                ctrl = Servo_0_CONTROL3 & ~Servo_0_CTRL_CMPMODE1_MASK;
                Servo_0_CONTROL3 = ctrl | Servo_0_DEFAULT_COMPARE1_MODE;
        #endif
        
         /* Clear and Set SYNCTC and SYNCCMP bits of RT1 register */
        Servo_0_RT1 &= ~Servo_0_RT1_MASK;
        Servo_0_RT1 |= Servo_0_SYNC;     
                
        /*Enable DSI Sync all all inputs of the PWM*/
        Servo_0_RT1 &= ~(Servo_0_SYNCDSI_MASK);
        Servo_0_RT1 |= Servo_0_SYNCDSI_EN;
       
    #elif (Servo_0_UseControl)
        /* Set the default compare mode defined in the parameter */
        ctrl = Servo_0_CONTROL & ~Servo_0_CTRL_CMPMODE2_MASK & ~Servo_0_CTRL_CMPMODE1_MASK;
        Servo_0_CONTROL = ctrl | Servo_0_DEFAULT_COMPARE2_MODE | Servo_0_DEFAULT_COMPARE1_MODE;
    #endif 
        
    #if (!Servo_0_UsingFixedFunction)
        #if (Servo_0_Resolution == 8)
            /* Set FIFO 0 to 1 byte register for period*/
            Servo_0_AUX_CONTROLDP0 |= (Servo_0_AUX_CTRL_FIFO0_CLR);
        #else /* (Servo_0_Resolution == 16)*/
            /* Set FIFO 0 to 1 byte register for period */
            Servo_0_AUX_CONTROLDP0 |= (Servo_0_AUX_CTRL_FIFO0_CLR);
            Servo_0_AUX_CONTROLDP1 |= (Servo_0_AUX_CTRL_FIFO0_CLR);
        #endif
    #endif
        
    Servo_0_WritePeriod(Servo_0_INIT_PERIOD_VALUE);
    Servo_0_WriteCounter(Servo_0_INIT_PERIOD_VALUE);
        
        #if (Servo_0_UseOneCompareMode)
            Servo_0_WriteCompare(Servo_0_INIT_COMPARE_VALUE1);
        #else
            Servo_0_WriteCompare1(Servo_0_INIT_COMPARE_VALUE1);
            Servo_0_WriteCompare2(Servo_0_INIT_COMPARE_VALUE2);
        #endif
        
        #if (Servo_0_KillModeMinTime)
            Servo_0_WriteKillTime(Servo_0_MinimumKillTime);
        #endif
        
        #if (Servo_0_DeadBandUsed)
            Servo_0_WriteDeadTime(Servo_0_INIT_DEAD_TIME);
        #endif

    #if (Servo_0_UseStatus || Servo_0_UsingFixedFunction)
        Servo_0_SetInterruptMode(Servo_0_INIT_INTERRUPTS_MODE);
    #endif
        
    #if (Servo_0_UsingFixedFunction)
        /* Globally Enable the Fixed Function Block chosen */
        Servo_0_GLOBAL_ENABLE |= Servo_0_BLOCK_EN_MASK;
        /* Set the Interrupt source to come from the status register */
        Servo_0_CONTROL2 |= Servo_0_CTRL2_IRQ_SEL;
    #else
        #if(Servo_0_UseStatus)
            
            /* CyEnterCriticalRegion and CyExitCriticalRegion are used to mark following region critical*/
            /* Enter Critical Region*/
            Servo_0_interruptState = CyEnterCriticalSection();
            /* Use the interrupt output of the status register for IRQ output */
            Servo_0_STATUS_AUX_CTRL |= Servo_0_STATUS_ACTL_INT_EN_MASK;
            
             /* Exit Critical Region*/
            CyExitCriticalSection(Servo_0_interruptState);
            
            /* Clear the FIFO to enable the Servo_0_STATUS_FIFOFULL
                   bit to be set on FIFO full. */
            Servo_0_ClearFIFO();
        #endif
    #endif
}


/*******************************************************************************
* Function Name: Servo_0_Enable
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
void Servo_0_Enable(void) 
{
    /* Globally Enable the Fixed Function Block chosen */
    #if (Servo_0_UsingFixedFunction)
        Servo_0_GLOBAL_ENABLE |= Servo_0_BLOCK_EN_MASK;
        Servo_0_GLOBAL_STBY_ENABLE |= Servo_0_BLOCK_STBY_EN_MASK;
    #endif 
    
    /* Enable the PWM from the control register  */
    #if (Servo_0_UseControl || Servo_0_UsingFixedFunction)
        Servo_0_CONTROL |= Servo_0_CTRL_ENABLE;
    #endif
}


/*******************************************************************************
* Function Name: Servo_0_Stop
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
void Servo_0_Stop(void) 
{
    #if (Servo_0_UseControl || Servo_0_UsingFixedFunction)
        Servo_0_CONTROL &= ~Servo_0_CTRL_ENABLE;
    #endif
    
    /* Globally disable the Fixed Function Block chosen */
    #if (Servo_0_UsingFixedFunction)
        Servo_0_GLOBAL_ENABLE &= ~Servo_0_BLOCK_EN_MASK;
        Servo_0_GLOBAL_STBY_ENABLE &= ~Servo_0_BLOCK_STBY_EN_MASK;
    #endif
}


#if (Servo_0_UseOneCompareMode)
#if (Servo_0_CompareMode1SW)
/*******************************************************************************
* Function Name: Servo_0_SetCompareMode
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
void Servo_0_SetCompareMode(uint8 comparemode) 
{
    #if(Servo_0_UsingFixedFunction)
            #if(Servo_0_PSOC3_ES2 || Servo_0_PSOC5_ES1)
                        uint8 comparemodemasked = (comparemode << Servo_0_CTRL_CMPMODE1_SHIFT);
                        Servo_0_CONTROL2 &= ~Servo_0_CTRL_CMPMODE1_MASK; /*Clear Existing Data */
                        Servo_0_CONTROL2 |= comparemodemasked;  
                #endif
                
            #if(Servo_0_PSOC3_ES3 || Servo_0_PSOC5_ES2)
                    uint8 comparemodemasked = (comparemode << Servo_0_CTRL_CMPMODE1_SHIFT);
            Servo_0_CONTROL3 &= ~Servo_0_CTRL_CMPMODE1_MASK; /*Clear Existing Data */
            Servo_0_CONTROL3 |= comparemodemasked;     
                #endif
                
    #elif (Servo_0_UseControl)
        uint8 comparemode1masked = (comparemode << Servo_0_CTRL_CMPMODE1_SHIFT) & Servo_0_CTRL_CMPMODE1_MASK;
        uint8 comparemode2masked = (comparemode << Servo_0_CTRL_CMPMODE2_SHIFT) & Servo_0_CTRL_CMPMODE2_MASK;
        Servo_0_CONTROL &= ~(Servo_0_CTRL_CMPMODE1_MASK | Servo_0_CTRL_CMPMODE2_MASK); /*Clear existing mode */
        Servo_0_CONTROL |= (comparemode1masked | comparemode2masked);
        
    #else
        uint8 temp = comparemode;
    #endif
}
#endif /* Servo_0_CompareMode1SW */

#else /* UseOneCompareMode */


#if (Servo_0_CompareMode1SW)
/*******************************************************************************
* Function Name: Servo_0_SetCompareMode1
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
void Servo_0_SetCompareMode1(uint8 comparemode) 
{
    uint8 comparemodemasked = (comparemode << Servo_0_CTRL_CMPMODE1_SHIFT) & Servo_0_CTRL_CMPMODE1_MASK;
    #if(Servo_0_UsingFixedFunction)
            #if(Servo_0_PSOC3_ES2 || Servo_0_PSOC5_ES1)
                        Servo_0_CONTROL2 &= Servo_0_CTRL_CMPMODE1_MASK; /*Clear existing mode */
                        Servo_0_CONTROL2 |= comparemodemasked; 
            #endif
                
                #if(Servo_0_PSOC3_ES3 || Servo_0_PSOC5_ES2)
                    Servo_0_CONTROL3 &= Servo_0_CTRL_CMPMODE1_MASK; /*Clear existing mode */
                        Servo_0_CONTROL3 |= comparemodemasked; 
            #endif
                
    #elif (Servo_0_UseControl)
        Servo_0_CONTROL &= Servo_0_CTRL_CMPMODE1_MASK; /*Clear existing mode */
        Servo_0_CONTROL |= comparemodemasked;
    #endif    
}
#endif /* Servo_0_CompareMode1SW */


#if (Servo_0_CompareMode2SW)
/*******************************************************************************
* Function Name: Servo_0_SetCompareMode2
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
void Servo_0_SetCompareMode2(uint8 comparemode) 
{
    #if(Servo_0_UsingFixedFunction)
        /* Do Nothing because there is no second Compare Mode Register in FF block*/ 
    #elif (Servo_0_UseControl)
    uint8 comparemodemasked = (comparemode << Servo_0_CTRL_CMPMODE2_SHIFT) & Servo_0_CTRL_CMPMODE2_MASK;
    Servo_0_CONTROL &= Servo_0_CTRL_CMPMODE2_MASK; /*Clear existing mode */
    Servo_0_CONTROL |= comparemodemasked;
    #endif    
}
#endif /*Servo_0_CompareMode2SW */
#endif /* UseOneCompareMode */


/*******************************************************************************
* Function Name: Servo_0_WriteCounter
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
void Servo_0_WriteCounter(uint16 counter) 
{
    #if(Servo_0_UsingFixedFunction)
        CY_SET_REG16(Servo_0_COUNTER_LSB_PTR, (uint16)counter);
    #else
        CY_SET_REG16(Servo_0_COUNTER_LSB_PTR, counter);
    #endif
}


#if (!Servo_0_UsingFixedFunction)
/*******************************************************************************
* Function Name: Servo_0_ReadCounter
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
uint16 Servo_0_ReadCounter(void) 
{
    /* Force capture by reading Accumulator */
    /* Must first do a software capture to be able to read the counter */
    /* It is up to the user code to make sure there isn't already captured data in the FIFO */
    CY_GET_REG8(Servo_0_COUNTERCAP_LSB_PTR);
    
    /* Read the data from the FIFO (or capture register for Fixed Function)*/
    return (CY_GET_REG16(Servo_0_CAPTURE_LSB_PTR));
}


#if (Servo_0_UseStatus)
/*******************************************************************************
* Function Name: Servo_0_ClearFIFO
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
void Servo_0_ClearFIFO(void) 
{
    while(Servo_0_ReadStatusRegister() & Servo_0_STATUS_FIFONEMPTY)
        Servo_0_ReadCapture();
}
#endif /* Servo_0_UseStatus */
#endif /* !Servo_0_UsingFixedFunction */


/*******************************************************************************
* Function Name: Servo_0_WritePeriod
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
void Servo_0_WritePeriod(uint16 period) 
{
    #if(Servo_0_UsingFixedFunction)
        CY_SET_REG16(Servo_0_PERIOD_LSB_PTR, (uint16)period);
    #else
        CY_SET_REG16(Servo_0_PERIOD_LSB_PTR, period);
    #endif
}


#if (Servo_0_UseOneCompareMode)
/*******************************************************************************
* Function Name: Servo_0_WriteCompare
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
void Servo_0_WriteCompare(uint16 compare) 
{
   CY_SET_REG16(Servo_0_COMPARE1_LSB_PTR, compare);
   #if (Servo_0_PWMMode == Servo_0__B_PWM__DITHER)
        CY_SET_REG16(Servo_0_COMPARE2_LSB_PTR, compare+1);
   #endif
}


#else


/*******************************************************************************
* Function Name: Servo_0_WriteCompare1
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
void Servo_0_WriteCompare1(uint16 compare) 
{
    #if(Servo_0_UsingFixedFunction)
        CY_SET_REG16(Servo_0_COMPARE1_LSB_PTR, (uint16)compare);
    #else
        CY_SET_REG16(Servo_0_COMPARE1_LSB_PTR, compare);
    #endif
}


/*******************************************************************************
* Function Name: Servo_0_WriteCompare2
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
void Servo_0_WriteCompare2(uint16 compare) 
{
    #if(Servo_0_UsingFixedFunction)
        CY_SET_REG16(Servo_0_COMPARE2_LSB_PTR, compare);
    #else
        CY_SET_REG16(Servo_0_COMPARE2_LSB_PTR, compare);
    #endif
}
#endif /* UseOneCompareMode */


#if (Servo_0_DeadBandUsed)
/*******************************************************************************
* Function Name: Servo_0_WriteDeadTime
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
void Servo_0_WriteDeadTime(uint8 deadtime) 
{
    /* If using the Dead Band 1-255 mode then just write the register */
    #if(!Servo_0_DeadBand2_4)
        CY_SET_REG8(Servo_0_DEADBAND_COUNT_PTR, deadtime);
    #else
        /* Otherwise the data has to be masked and offset */        
        /* Clear existing data */
        Servo_0_DEADBAND_COUNT &= ~Servo_0_DEADBAND_COUNT_MASK; 
            /* Set new dead time */
        Servo_0_DEADBAND_COUNT |= (deadtime << Servo_0_DEADBAND_COUNT_SHIFT) & Servo_0_DEADBAND_COUNT_MASK; 
    #endif
}


/*******************************************************************************
* Function Name: Servo_0_ReadDeadTime
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
uint8 Servo_0_ReadDeadTime(void) 
{
    /* If using the Dead Band 1-255 mode then just read the register */
    #if(!Servo_0_DeadBand2_4)
        return (CY_GET_REG8(Servo_0_DEADBAND_COUNT_PTR));
    #else
        /* Otherwise the data has to be masked and offset */
        return ((Servo_0_DEADBAND_COUNT & Servo_0_DEADBAND_COUNT_MASK) >> Servo_0_DEADBAND_COUNT_SHIFT);
    #endif
}
#endif /* DeadBandUsed */

