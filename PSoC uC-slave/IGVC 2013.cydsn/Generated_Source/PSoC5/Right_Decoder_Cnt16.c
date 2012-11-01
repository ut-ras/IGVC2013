/*******************************************************************************
* File Name: Right_Decoder_Cnt16.c  
* Version 2.0
*
*  Description:
*     The Counter User Module consists of a 8, 16, 24 or 32-bit counter with
*     a selectable period between 2 and 2^Width - 1.  
*
*   Note:
*     None
*
*******************************************************************************
* Copyright 2008-2010, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

#include "Right_Decoder_Cnt16.h"

uint8 Right_Decoder_Cnt16_initVar = 0u;


/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_Init
********************************************************************************
* Summary:
*     Initialize to the schematic state
* 
* Parameters:  
*  void  
*
* Return: 
*  void
*
* Reentrant
*  Yes
*
*******************************************************************************/
void Right_Decoder_Cnt16_Init(void) 
{
        #if (!Right_Decoder_Cnt16_UsingFixedFunction && !Right_Decoder_Cnt16_ControlRegRemoved)
            uint8 ctrl;
        #endif
        
        #if(!Right_Decoder_Cnt16_UsingFixedFunction) 
            /* Interrupt State Backup for Critical Region*/
            uint8 Right_Decoder_Cnt16_interruptState;
        #endif
        
        #if (Right_Decoder_Cnt16_UsingFixedFunction)
            /* Clear all bits but the enable bit (if it's already set for Timer operation */
            Right_Decoder_Cnt16_CONTROL &= Right_Decoder_Cnt16_CTRL_ENABLE;
            
            /* Clear the mode bits for continuous run mode */
            #if (Right_Decoder_Cnt16_PSOC3_ES2 || Right_Decoder_Cnt16_PSOC5_ES1)
                Right_Decoder_Cnt16_CONTROL2 &= ~Right_Decoder_Cnt16_CTRL_MODE_MASK;
            #endif
            #if (Right_Decoder_Cnt16_PSOC3_ES3 || Right_Decoder_Cnt16_PSOC5_ES2)
                Right_Decoder_Cnt16_CONTROL3 &= ~Right_Decoder_Cnt16_CTRL_MODE_MASK;                
            #endif
            /* Check if One Shot mode is enabled i.e. RunMode !=0*/
            #if (Right_Decoder_Cnt16_RunModeUsed != 0x0u)
                /* Set 3rd bit of Control register to enable one shot mode */
                Right_Decoder_Cnt16_CONTROL |= Right_Decoder_Cnt16_ONESHOT;
            #endif
            
            /* Set the IRQ to use the status register interrupts */
            Right_Decoder_Cnt16_CONTROL2 |= Right_Decoder_Cnt16_CTRL2_IRQ_SEL;
            
            /* Clear and Set SYNCTC and SYNCCMP bits of RT1 register */
            Right_Decoder_Cnt16_RT1 &= ~Right_Decoder_Cnt16_RT1_MASK;
            Right_Decoder_Cnt16_RT1 |= Right_Decoder_Cnt16_SYNC;     
                    
            /*Enable DSI Sync all all inputs of the Timer*/
            Right_Decoder_Cnt16_RT1 &= ~(Right_Decoder_Cnt16_SYNCDSI_MASK);
            Right_Decoder_Cnt16_RT1 |= Right_Decoder_Cnt16_SYNCDSI_EN;

        #else
            #if(!Right_Decoder_Cnt16_ControlRegRemoved)
            /* Set the default compare mode defined in the parameter */
            ctrl = Right_Decoder_Cnt16_CONTROL & ~Right_Decoder_Cnt16_CTRL_CMPMODE_MASK;
            Right_Decoder_Cnt16_CONTROL = ctrl | Right_Decoder_Cnt16_DEFAULT_COMPARE_MODE;
            
            /* Set the default capture mode defined in the parameter */
            ctrl = Right_Decoder_Cnt16_CONTROL & ~Right_Decoder_Cnt16_CTRL_CAPMODE_MASK;
            Right_Decoder_Cnt16_CONTROL = ctrl | Right_Decoder_Cnt16_DEFAULT_CAPTURE_MODE;
            #endif
        #endif 
        
        /* Clear all data in the FIFO's */
        #if (!Right_Decoder_Cnt16_UsingFixedFunction)
            Right_Decoder_Cnt16_ClearFIFO();
        #endif
        
        /* Set Initial values from Configuration */
        Right_Decoder_Cnt16_WritePeriod(Right_Decoder_Cnt16_INIT_PERIOD_VALUE);
        Right_Decoder_Cnt16_WriteCounter(Right_Decoder_Cnt16_INIT_COUNTER_VALUE);
        Right_Decoder_Cnt16_SetInterruptMode(Right_Decoder_Cnt16_INIT_INTERRUPTS_MASK);
        
        #if (!Right_Decoder_Cnt16_UsingFixedFunction)
            /* Read the status register to clear the unwanted interrupts */
            Right_Decoder_Cnt16_ReadStatusRegister();
            /* Set the compare value (only available to non-fixed function implementation */
            Right_Decoder_Cnt16_WriteCompare(Right_Decoder_Cnt16_INIT_COMPARE_VALUE);
            /* Use the interrupt output of the status register for IRQ output */
            
            /* CyEnterCriticalRegion and CyExitCriticalRegion are used to mark following region critical*/
            /* Enter Critical Region*/
            Right_Decoder_Cnt16_interruptState = CyEnterCriticalSection();
            
            Right_Decoder_Cnt16_STATUS_AUX_CTRL |= Right_Decoder_Cnt16_STATUS_ACTL_INT_EN_MASK;
            
            /* Exit Critical Region*/
            CyExitCriticalSection(Right_Decoder_Cnt16_interruptState);
            
        #endif
}


/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_Enable
********************************************************************************
* Summary:
*     Enable the Counter
* 
* Parameters:  
*  void  
*
* Return: 
*  void
*
* Side Effects: 
*   If the Enable mode is set to Hardware only then this function has no effect 
*   on the operation of the counter.
*
* Reentrant
*  Yes
*
*******************************************************************************/
void Right_Decoder_Cnt16_Enable(void) 
{
    /* Globally Enable the Fixed Function Block chosen */
    #if (Right_Decoder_Cnt16_UsingFixedFunction)
        Right_Decoder_Cnt16_GLOBAL_ENABLE |= Right_Decoder_Cnt16_BLOCK_EN_MASK;
        Right_Decoder_Cnt16_GLOBAL_STBY_ENABLE |= Right_Decoder_Cnt16_BLOCK_STBY_EN_MASK;
    #endif   
        
    /* Enable the counter from the control register  */
    /* If Fixed Function then make sure Mode is set correctly */
    /* else make sure reset is clear */
    #if(!Right_Decoder_Cnt16_ControlRegRemoved || Right_Decoder_Cnt16_UsingFixedFunction)
        Right_Decoder_Cnt16_CONTROL |= Right_Decoder_Cnt16_CTRL_ENABLE;                
    #endif
    
}


/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_Start
********************************************************************************
* Summary:
*  Enables the counter for operation 
*
* Parameters:  
*  void  
*
* Return: 
*  void
*
* Global variables:
*  Right_Decoder_Cnt16_initVar: Is modified when this function is called for the first 
*   time. Is used to ensure that initialization happens only once.
*
* Reentrant
*  No
*
*******************************************************************************/
void Right_Decoder_Cnt16_Start(void) 
{
    if(Right_Decoder_Cnt16_initVar == 0u)
    {
        Right_Decoder_Cnt16_Init();
        
        Right_Decoder_Cnt16_initVar = 1u; /* Clear this bit for Initialization */        
    }
    
    /* Enable the Counter */
    Right_Decoder_Cnt16_Enable();        
}


/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_Stop
********************************************************************************
* Summary:
* Halts the counter, but does not change any modes or disable interrupts.
*
* Parameters:  
*  void  
*
* Return: 
*  void
*
* Side Effects: If the Enable mode is set to Hardware only then this function
*               has no effect on the operation of the counter.
*
* Reentrant
*  Yes
*
*******************************************************************************/
void Right_Decoder_Cnt16_Stop(void) 
{
    /* Disable Counter */
    #if(!Right_Decoder_Cnt16_ControlRegRemoved || Right_Decoder_Cnt16_UsingFixedFunction)
        Right_Decoder_Cnt16_CONTROL &= ~Right_Decoder_Cnt16_CTRL_ENABLE;        
    #endif
    
    /* Globally disable the Fixed Function Block chosen */
    #if (Right_Decoder_Cnt16_UsingFixedFunction)
        Right_Decoder_Cnt16_GLOBAL_ENABLE &= ~Right_Decoder_Cnt16_BLOCK_EN_MASK;
        Right_Decoder_Cnt16_GLOBAL_STBY_ENABLE &= ~Right_Decoder_Cnt16_BLOCK_STBY_EN_MASK;
    #endif
}


/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_SetInterruptMode
********************************************************************************
* Summary:
* Configures which interrupt sources are enabled to generate the final interrupt
*
* Parameters:  
*  InterruptsMask: This parameter is an or'd collection of the status bits
*                   which will be allowed to generate the counters interrupt.   
*
* Return: 
*  void
*
* Reentrant
*  Yes
*
*******************************************************************************/
void Right_Decoder_Cnt16_SetInterruptMode(uint8 interruptsMask) 
{
    Right_Decoder_Cnt16_STATUS_MASK = interruptsMask;
}


/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_ReadStatusRegister
********************************************************************************
* Summary:
*   Reads the status register and returns it's state. This function should use
*       defined types for the bit-field information as the bits in this
*       register may be permuteable.
*
* Parameters:  
*  void
*
* Return: 
*  (uint8) The contents of the status register
*
* Side Effects:
*   Status register bits may be clear on read. 
*
* Reentrant
*  Yes
*
*******************************************************************************/
uint8   Right_Decoder_Cnt16_ReadStatusRegister(void) 
{
    return Right_Decoder_Cnt16_STATUS;
}


#if(!Right_Decoder_Cnt16_ControlRegRemoved)
/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_ReadControlRegister
********************************************************************************
* Summary:
*   Reads the control register and returns it's state. This function should use
*       defined types for the bit-field information as the bits in this
*       register may be permuteable.
*
* Parameters:  
*  void
*
* Return: 
*  (uint8) The contents of the control register
*
* Reentrant
*  Yes
*
*******************************************************************************/
uint8   Right_Decoder_Cnt16_ReadControlRegister(void) 
{
    return Right_Decoder_Cnt16_CONTROL;
}


/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_WriteControlRegister
********************************************************************************
* Summary:
*   Sets the bit-field of the control register.  This function should use
*       defined types for the bit-field information as the bits in this
*       register may be permuteable.
*
* Parameters:  
*  void
*
* Return: 
*  (uint8) The contents of the control register
*   
* Reentrant
*  Yes
*
*******************************************************************************/
void    Right_Decoder_Cnt16_WriteControlRegister(uint8 control) 
{
    Right_Decoder_Cnt16_CONTROL = control;
}

#endif  /* (!Right_Decoder_Cnt16_ControlRegRemoved) */


/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_WriteCounter
********************************************************************************
* Summary:
*   This funtion is used to set the counter to a specific value
*
* Parameters:  
*  counter:  New counter value. 
*
* Return: 
*  void 
*
* Reentrant
*  Yes
*
*******************************************************************************/
void Right_Decoder_Cnt16_WriteCounter(uint16 counter) 
{
    #if(Right_Decoder_Cnt16_UsingFixedFunction)
        CY_SET_REG16(Right_Decoder_Cnt16_COUNTER_LSB_PTR, (uint16)counter);
    #else
        CY_SET_REG16(Right_Decoder_Cnt16_COUNTER_LSB_PTR, counter);
    #endif
}


/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_ReadCounter
********************************************************************************
* Summary:
* Returns the current value of the counter.  It doesn't matter
* if the counter is enabled or running.
*
* Parameters:  
*  void:  
*
* Return: 
*  (uint16) The present value of the counter.
*
* Reentrant
*  Yes
*
*******************************************************************************/
uint16 Right_Decoder_Cnt16_ReadCounter(void) 
{
    /* Force capture by reading Accumulator */
    /* Must first do a software capture to be able to read the counter */
    /* It is up to the user code to make sure there isn't already captured data in the FIFO */
    CY_GET_REG8(Right_Decoder_Cnt16_COUNTER_LSB_PTR);
    
    /* Read the data from the FIFO (or capture register for Fixed Function)*/
    return (CY_GET_REG16(Right_Decoder_Cnt16_STATICCOUNT_LSB_PTR));
}


/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_ReadCapture
********************************************************************************
* Summary:
*   This function returns the last value captured.
*
* Parameters:  
*  void
*
* Return: 
*  (uint16) Present Capture value.
*
* Reentrant
*  Yes
*
*******************************************************************************/
uint16 Right_Decoder_Cnt16_ReadCapture(void) 
{
   return ( CY_GET_REG16(Right_Decoder_Cnt16_STATICCOUNT_LSB_PTR) );  
}


/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_WritePeriod
********************************************************************************
* Summary:
* Changes the period of the counter.  The new period 
* will be loaded the next time terminal count is detected.
*
* Parameters:  
*  period: (uint16) A value of 0 will result in
*         the counter remaining at zero.  
*
* Return: 
*  void
*
* Reentrant
*  Yes
*
*******************************************************************************/
void Right_Decoder_Cnt16_WritePeriod(uint16 period) 
{
    #if(Right_Decoder_Cnt16_UsingFixedFunction)
        CY_SET_REG16(Right_Decoder_Cnt16_PERIOD_LSB_PTR,(uint16)period);
    #else
        CY_SET_REG16(Right_Decoder_Cnt16_PERIOD_LSB_PTR,period);
    #endif
}


/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_ReadPeriod
********************************************************************************
* Summary:
* Reads the current period value without affecting counter operation.
*
* Parameters:  
*  void:  
*
* Return: 
*  (uint16) Present period value.
*
* Reentrant
*  Yes
*
*******************************************************************************/
uint16 Right_Decoder_Cnt16_ReadPeriod(void) 
{
   return ( CY_GET_REG16(Right_Decoder_Cnt16_PERIOD_LSB_PTR));
}


#if (!Right_Decoder_Cnt16_UsingFixedFunction)
/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_WriteCompare
********************************************************************************
* Summary:
* Changes the compare value.  The compare output will 
* reflect the new value on the next UDB clock.  The compare output will be 
* driven high when the present counter value compares true based on the 
* configured compare mode setting. 
*
* Parameters:  
*  Compare:  New compare value. 
*
* Return: 
*  void
*
* Reentrant
*  Yes
*
*******************************************************************************/
void Right_Decoder_Cnt16_WriteCompare(uint16 compare) 
{
    #if(Right_Decoder_Cnt16_UsingFixedFunction)
        CY_SET_REG16(Right_Decoder_Cnt16_COMPARE_LSB_PTR,(uint16)compare);
    #else
        CY_SET_REG16(Right_Decoder_Cnt16_COMPARE_LSB_PTR,compare);
    #endif
}


/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_ReadCompare
********************************************************************************
* Summary:
* Returns the compare value.
*
* Parameters:  
*  void:
*
* Return: 
*  (uint16) Present compare value.
*
* Reentrant
*  Yes
*
*******************************************************************************/
uint16 Right_Decoder_Cnt16_ReadCompare(void) 
{
   return ( CY_GET_REG16(Right_Decoder_Cnt16_COMPARE_LSB_PTR));
}


#if (Right_Decoder_Cnt16_COMPARE_MODE_SOFTWARE)
/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_SetCompareMode
********************************************************************************
* Summary:
*  Sets the software controlled Compare Mode.
*
* Parameters:
*  compareMode:  Compare Mode Enumerated Type.
*
* Return:
*  void
*
* Reentrant
*  Yes
*
*******************************************************************************/
void Right_Decoder_Cnt16_SetCompareMode(uint8 compareMode) 
{
    /* Clear the compare mode bits in the control register */
    Right_Decoder_Cnt16_CONTROL &= ~Right_Decoder_Cnt16_CTRL_CMPMODE_MASK;
    
    /* Write the new setting */
    Right_Decoder_Cnt16_CONTROL |= (compareMode << Right_Decoder_Cnt16_CTRL_CMPMODE0_SHIFT);
}
#endif  /* (Right_Decoder_Cnt16_COMPARE_MODE_SOFTWARE) */


#if (Right_Decoder_Cnt16_CAPTURE_MODE_SOFTWARE)
/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_SetCaptureMode
********************************************************************************
* Summary:
*  Sets the software controlled Capture Mode.
*
* Parameters:
*  captureMode:  Capture Mode Enumerated Type.
*
* Return:
*  void
*
* Reentrant
*  Yes
*
*******************************************************************************/
void Right_Decoder_Cnt16_SetCaptureMode(uint8 captureMode) 
{
    /* Clear the capture mode bits in the control register */
    Right_Decoder_Cnt16_CONTROL &= ~Right_Decoder_Cnt16_CTRL_CAPMODE_MASK;
    
    /* Write the new setting */
    Right_Decoder_Cnt16_CONTROL |= (captureMode << Right_Decoder_Cnt16_CTRL_CAPMODE0_SHIFT);
}
#endif  /* (Right_Decoder_Cnt16_CAPTURE_MODE_SOFTWARE) */


/*******************************************************************************
* Function Name: Right_Decoder_Cnt16_ClearFIFO
********************************************************************************
* Summary:
*   This function clears all capture data from the capture FIFO
*
* Parameters:  
*  void:
*
* Return: 
*  None
*
* Reentrant
*  Yes
*
*******************************************************************************/
void Right_Decoder_Cnt16_ClearFIFO(void) 
{

    while(Right_Decoder_Cnt16_ReadStatusRegister() & Right_Decoder_Cnt16_STATUS_FIFONEMP)
    {
        Right_Decoder_Cnt16_ReadCapture();
    }

}
#endif  /* (!Right_Decoder_Cnt16_UsingFixedFunction) */


/* [] END OF FILE */

