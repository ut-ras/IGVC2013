/*******************************************************************************
* File Name: Servo_1.c  
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
#include "Servo_1.h"

uint8 Servo_1_initVar = 0u;

/*******************************************************************************
* Function Name: Servo_1_Start
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
*  Servo_1_initVar: Is modified when this function is called for the first 
*   time. Is used to ensure that initialization happens only once.
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_1_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(Servo_1_initVar == 0u)
    {
        Servo_1_Init();
        Servo_1_initVar = 1u;
    }
    Servo_1_Enable();

}


/*******************************************************************************
* Function Name: Servo_1_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the 
*  customizer of the component placed onto schematic. Usually called in 
*  Servo_1_Start().
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
void Servo_1_Init(void) 
{
    #if (Servo_1_UsingFixedFunction || Servo_1_UseControl)
        uint8 ctrl;
    #endif
    
    #if(!Servo_1_UsingFixedFunction) 
        #if(Servo_1_UseStatus)
            /* Interrupt State Backup for Critical Region*/
            uint8 Servo_1_interruptState;
        #endif
    #endif
    
   #if (Servo_1_UsingFixedFunction)
        /* You are allowed to write the compare value (FF only) */
        Servo_1_CONTROL |= Servo_1_CFG0_MODE;
        #if (Servo_1_DeadBand2_4)
            Servo_1_CONTROL |= Servo_1_CFG0_DB;
        #endif
                
        /* Set the default Compare Mode */
        #if(Servo_1_PSOC3_ES2 || Servo_1_PSOC5_ES1)
                ctrl = Servo_1_CONTROL2 & ~Servo_1_CTRL_CMPMODE1_MASK;
                Servo_1_CONTROL2 = ctrl | Servo_1_DEFAULT_COMPARE1_MODE;
        #endif
        #if(Servo_1_PSOC3_ES3 || Servo_1_PSOC5_ES2)
                ctrl = Servo_1_CONTROL3 & ~Servo_1_CTRL_CMPMODE1_MASK;
                Servo_1_CONTROL3 = ctrl | Servo_1_DEFAULT_COMPARE1_MODE;
        #endif
        
         /* Clear and Set SYNCTC and SYNCCMP bits of RT1 register */
        Servo_1_RT1 &= ~Servo_1_RT1_MASK;
        Servo_1_RT1 |= Servo_1_SYNC;     
                
        /*Enable DSI Sync all all inputs of the PWM*/
        Servo_1_RT1 &= ~(Servo_1_SYNCDSI_MASK);
        Servo_1_RT1 |= Servo_1_SYNCDSI_EN;
       
    #elif (Servo_1_UseControl)
        /* Set the default compare mode defined in the parameter */
        ctrl = Servo_1_CONTROL & ~Servo_1_CTRL_CMPMODE2_MASK & ~Servo_1_CTRL_CMPMODE1_MASK;
        Servo_1_CONTROL = ctrl | Servo_1_DEFAULT_COMPARE2_MODE | Servo_1_DEFAULT_COMPARE1_MODE;
    #endif 
        
    #if (!Servo_1_UsingFixedFunction)
        #if (Servo_1_Resolution == 8)
            /* Set FIFO 0 to 1 byte register for period*/
            Servo_1_AUX_CONTROLDP0 |= (Servo_1_AUX_CTRL_FIFO0_CLR);
        #else /* (Servo_1_Resolution == 16)*/
            /* Set FIFO 0 to 1 byte register for period */
            Servo_1_AUX_CONTROLDP0 |= (Servo_1_AUX_CTRL_FIFO0_CLR);
            Servo_1_AUX_CONTROLDP1 |= (Servo_1_AUX_CTRL_FIFO0_CLR);
        #endif
    #endif
        
    Servo_1_WritePeriod(Servo_1_INIT_PERIOD_VALUE);
    Servo_1_WriteCounter(Servo_1_INIT_PERIOD_VALUE);
        
        #if (Servo_1_UseOneCompareMode)
            Servo_1_WriteCompare(Servo_1_INIT_COMPARE_VALUE1);
        #else
            Servo_1_WriteCompare1(Servo_1_INIT_COMPARE_VALUE1);
            Servo_1_WriteCompare2(Servo_1_INIT_COMPARE_VALUE2);
        #endif
        
        #if (Servo_1_KillModeMinTime)
            Servo_1_WriteKillTime(Servo_1_MinimumKillTime);
        #endif
        
        #if (Servo_1_DeadBandUsed)
            Servo_1_WriteDeadTime(Servo_1_INIT_DEAD_TIME);
        #endif

    #if (Servo_1_UseStatus || Servo_1_UsingFixedFunction)
        Servo_1_SetInterruptMode(Servo_1_INIT_INTERRUPTS_MODE);
    #endif
        
    #if (Servo_1_UsingFixedFunction)
        /* Globally Enable the Fixed Function Block chosen */
        Servo_1_GLOBAL_ENABLE |= Servo_1_BLOCK_EN_MASK;
        /* Set the Interrupt source to come from the status register */
        Servo_1_CONTROL2 |= Servo_1_CTRL2_IRQ_SEL;
    #else
        #if(Servo_1_UseStatus)
            
            /* CyEnterCriticalRegion and CyExitCriticalRegion are used to mark following region critical*/
            /* Enter Critical Region*/
            Servo_1_interruptState = CyEnterCriticalSection();
            /* Use the interrupt output of the status register for IRQ output */
            Servo_1_STATUS_AUX_CTRL |= Servo_1_STATUS_ACTL_INT_EN_MASK;
            
             /* Exit Critical Region*/
            CyExitCriticalSection(Servo_1_interruptState);
            
            /* Clear the FIFO to enable the Servo_1_STATUS_FIFOFULL
                   bit to be set on FIFO full. */
            Servo_1_ClearFIFO();
        #endif
    #endif
}


/*******************************************************************************
* Function Name: Servo_1_Enable
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
void Servo_1_Enable(void) 
{
    /* Globally Enable the Fixed Function Block chosen */
    #if (Servo_1_UsingFixedFunction)
        Servo_1_GLOBAL_ENABLE |= Servo_1_BLOCK_EN_MASK;
        Servo_1_GLOBAL_STBY_ENABLE |= Servo_1_BLOCK_STBY_EN_MASK;
    #endif 
    
    /* Enable the PWM from the control register  */
    #if (Servo_1_UseControl || Servo_1_UsingFixedFunction)
        Servo_1_CONTROL |= Servo_1_CTRL_ENABLE;
    #endif
}


/*******************************************************************************
* Function Name: Servo_1_Stop
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
void Servo_1_Stop(void) 
{
    #if (Servo_1_UseControl || Servo_1_UsingFixedFunction)
        Servo_1_CONTROL &= ~Servo_1_CTRL_ENABLE;
    #endif
    
    /* Globally disable the Fixed Function Block chosen */
    #if (Servo_1_UsingFixedFunction)
        Servo_1_GLOBAL_ENABLE &= ~Servo_1_BLOCK_EN_MASK;
        Servo_1_GLOBAL_STBY_ENABLE &= ~Servo_1_BLOCK_STBY_EN_MASK;
    #endif
}


#if (Servo_1_UseOneCompareMode)
#if (Servo_1_CompareMode1SW)
/*******************************************************************************
* Function Name: Servo_1_SetCompareMode
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
void Servo_1_SetCompareMode(uint8 comparemode) 
{
    #if(Servo_1_UsingFixedFunction)
            #if(Servo_1_PSOC3_ES2 || Servo_1_PSOC5_ES1)
                        uint8 comparemodemasked = (comparemode << Servo_1_CTRL_CMPMODE1_SHIFT);
                        Servo_1_CONTROL2 &= ~Servo_1_CTRL_CMPMODE1_MASK; /*Clear Existing Data */
                        Servo_1_CONTROL2 |= comparemodemasked;  
                #endif
                
            #if(Servo_1_PSOC3_ES3 || Servo_1_PSOC5_ES2)
                    uint8 comparemodemasked = (comparemode << Servo_1_CTRL_CMPMODE1_SHIFT);
            Servo_1_CONTROL3 &= ~Servo_1_CTRL_CMPMODE1_MASK; /*Clear Existing Data */
            Servo_1_CONTROL3 |= comparemodemasked;     
                #endif
                
    #elif (Servo_1_UseControl)
        uint8 comparemode1masked = (comparemode << Servo_1_CTRL_CMPMODE1_SHIFT) & Servo_1_CTRL_CMPMODE1_MASK;
        uint8 comparemode2masked = (comparemode << Servo_1_CTRL_CMPMODE2_SHIFT) & Servo_1_CTRL_CMPMODE2_MASK;
        Servo_1_CONTROL &= ~(Servo_1_CTRL_CMPMODE1_MASK | Servo_1_CTRL_CMPMODE2_MASK); /*Clear existing mode */
        Servo_1_CONTROL |= (comparemode1masked | comparemode2masked);
        
    #else
        uint8 temp = comparemode;
    #endif
}
#endif /* Servo_1_CompareMode1SW */

#else /* UseOneCompareMode */


#if (Servo_1_CompareMode1SW)
/*******************************************************************************
* Function Name: Servo_1_SetCompareMode1
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
void Servo_1_SetCompareMode1(uint8 comparemode) 
{
    uint8 comparemodemasked = (comparemode << Servo_1_CTRL_CMPMODE1_SHIFT) & Servo_1_CTRL_CMPMODE1_MASK;
    #if(Servo_1_UsingFixedFunction)
            #if(Servo_1_PSOC3_ES2 || Servo_1_PSOC5_ES1)
                        Servo_1_CONTROL2 &= Servo_1_CTRL_CMPMODE1_MASK; /*Clear existing mode */
                        Servo_1_CONTROL2 |= comparemodemasked; 
            #endif
                
                #if(Servo_1_PSOC3_ES3 || Servo_1_PSOC5_ES2)
                    Servo_1_CONTROL3 &= Servo_1_CTRL_CMPMODE1_MASK; /*Clear existing mode */
                        Servo_1_CONTROL3 |= comparemodemasked; 
            #endif
                
    #elif (Servo_1_UseControl)
        Servo_1_CONTROL &= Servo_1_CTRL_CMPMODE1_MASK; /*Clear existing mode */
        Servo_1_CONTROL |= comparemodemasked;
    #endif    
}
#endif /* Servo_1_CompareMode1SW */


#if (Servo_1_CompareMode2SW)
/*******************************************************************************
* Function Name: Servo_1_SetCompareMode2
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
void Servo_1_SetCompareMode2(uint8 comparemode) 
{
    #if(Servo_1_UsingFixedFunction)
        /* Do Nothing because there is no second Compare Mode Register in FF block*/ 
    #elif (Servo_1_UseControl)
    uint8 comparemodemasked = (comparemode << Servo_1_CTRL_CMPMODE2_SHIFT) & Servo_1_CTRL_CMPMODE2_MASK;
    Servo_1_CONTROL &= Servo_1_CTRL_CMPMODE2_MASK; /*Clear existing mode */
    Servo_1_CONTROL |= comparemodemasked;
    #endif    
}
#endif /*Servo_1_CompareMode2SW */
#endif /* UseOneCompareMode */


/*******************************************************************************
* Function Name: Servo_1_WriteCounter
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
void Servo_1_WriteCounter(uint16 counter) 
{
    #if(Servo_1_UsingFixedFunction)
        CY_SET_REG16(Servo_1_COUNTER_LSB_PTR, (uint16)counter);
    #else
        CY_SET_REG16(Servo_1_COUNTER_LSB_PTR, counter);
    #endif
}


#if (!Servo_1_UsingFixedFunction)
/*******************************************************************************
* Function Name: Servo_1_ReadCounter
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
uint16 Servo_1_ReadCounter(void) 
{
    /* Force capture by reading Accumulator */
    /* Must first do a software capture to be able to read the counter */
    /* It is up to the user code to make sure there isn't already captured data in the FIFO */
    CY_GET_REG8(Servo_1_COUNTERCAP_LSB_PTR);
    
    /* Read the data from the FIFO (or capture register for Fixed Function)*/
    return (CY_GET_REG16(Servo_1_CAPTURE_LSB_PTR));
}


#if (Servo_1_UseStatus)
/*******************************************************************************
* Function Name: Servo_1_ClearFIFO
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
void Servo_1_ClearFIFO(void) 
{
    while(Servo_1_ReadStatusRegister() & Servo_1_STATUS_FIFONEMPTY)
        Servo_1_ReadCapture();
}
#endif /* Servo_1_UseStatus */
#endif /* !Servo_1_UsingFixedFunction */


/*******************************************************************************
* Function Name: Servo_1_WritePeriod
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
void Servo_1_WritePeriod(uint16 period) 
{
    #if(Servo_1_UsingFixedFunction)
        CY_SET_REG16(Servo_1_PERIOD_LSB_PTR, (uint16)period);
    #else
        CY_SET_REG16(Servo_1_PERIOD_LSB_PTR, period);
    #endif
}


#if (Servo_1_UseOneCompareMode)
/*******************************************************************************
* Function Name: Servo_1_WriteCompare
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
void Servo_1_WriteCompare(uint16 compare) 
{
   CY_SET_REG16(Servo_1_COMPARE1_LSB_PTR, compare);
   #if (Servo_1_PWMMode == Servo_1__B_PWM__DITHER)
        CY_SET_REG16(Servo_1_COMPARE2_LSB_PTR, compare+1);
   #endif
}


#else


/*******************************************************************************
* Function Name: Servo_1_WriteCompare1
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
void Servo_1_WriteCompare1(uint16 compare) 
{
    #if(Servo_1_UsingFixedFunction)
        CY_SET_REG16(Servo_1_COMPARE1_LSB_PTR, (uint16)compare);
    #else
        CY_SET_REG16(Servo_1_COMPARE1_LSB_PTR, compare);
    #endif
}


/*******************************************************************************
* Function Name: Servo_1_WriteCompare2
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
void Servo_1_WriteCompare2(uint16 compare) 
{
    #if(Servo_1_UsingFixedFunction)
        CY_SET_REG16(Servo_1_COMPARE2_LSB_PTR, compare);
    #else
        CY_SET_REG16(Servo_1_COMPARE2_LSB_PTR, compare);
    #endif
}
#endif /* UseOneCompareMode */


#if (Servo_1_DeadBandUsed)
/*******************************************************************************
* Function Name: Servo_1_WriteDeadTime
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
void Servo_1_WriteDeadTime(uint8 deadtime) 
{
    /* If using the Dead Band 1-255 mode then just write the register */
    #if(!Servo_1_DeadBand2_4)
        CY_SET_REG8(Servo_1_DEADBAND_COUNT_PTR, deadtime);
    #else
        /* Otherwise the data has to be masked and offset */        
        /* Clear existing data */
        Servo_1_DEADBAND_COUNT &= ~Servo_1_DEADBAND_COUNT_MASK; 
            /* Set new dead time */
        Servo_1_DEADBAND_COUNT |= (deadtime << Servo_1_DEADBAND_COUNT_SHIFT) & Servo_1_DEADBAND_COUNT_MASK; 
    #endif
}


/*******************************************************************************
* Function Name: Servo_1_ReadDeadTime
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
uint8 Servo_1_ReadDeadTime(void) 
{
    /* If using the Dead Band 1-255 mode then just read the register */
    #if(!Servo_1_DeadBand2_4)
        return (CY_GET_REG8(Servo_1_DEADBAND_COUNT_PTR));
    #else
        /* Otherwise the data has to be masked and offset */
        return ((Servo_1_DEADBAND_COUNT & Servo_1_DEADBAND_COUNT_MASK) >> Servo_1_DEADBAND_COUNT_SHIFT);
    #endif
}
#endif /* DeadBandUsed */

