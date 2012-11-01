/*******************************************************************************
* File Name: Left_Encoder.c  
* Version 2.0
*
* Description:
*  This file provides the source code to the API for the Quadrature Decoder
*  component.
*
* Note:
*  None.
*   
********************************************************************************
* Copyright 2008-2011, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "CyLib.h"
#include "Left_Encoder.h"

#if(Left_Encoder_COUNTER_SIZE == 32u)
    extern volatile int32 Left_Encoder_count32SoftPart;    
#endif /*Left_Encoder_COUNTER_SIZE == 32u*/

uint8 Left_Encoder_initVar = 0u;


/*******************************************************************************
* Function Name: Left_Encoder_Init
********************************************************************************
*
* Summary:   
*  Inits/Restores default QuadDec configuration provided with customizer.
*
* Parameters:  
*  None.
*
* Return: 
*  None.
*
*******************************************************************************/
void Left_Encoder_Init(void) 
{      
    #if (Left_Encoder_COUNTER_SIZE == 32u)
      
        /* Disable Interrupt. */
        CyIntDisable(Left_Encoder_ISR_NUMBER);
        
        /* Set the ISR to point to the Left_Encoder_isr Interrupt. */
        CyIntSetVector(Left_Encoder_ISR_NUMBER, Left_Encoder_ISR);
        
        /* Set the priority. */
        CyIntSetPriority(Left_Encoder_ISR_NUMBER, Left_Encoder_ISR_PRIORITY);       
        
    #endif /* Left_Encoder_COUNTER_SIZE == 32u */    
}


/*******************************************************************************
* Function Name: Left_Encoder_Enable
********************************************************************************
*
* Summary:   
*  This function enable interrupts from Component and also enable Component's 
*  isr for 32-bit counter.
*
* Parameters:  
*  None.
*
* Return: 
*  None.
*
*******************************************************************************/
void Left_Encoder_Enable(void) 
{
    uint8 enableInterrupts = 0u;
    
    Left_Encoder_SetInterruptMask(Left_Encoder_INIT_INT_MASK);
    
    enableInterrupts = CyEnterCriticalSection();
    
    /* Enable interrupts from Statusi register */
    Left_Encoder_SR_AUX_CONTROL |= Left_Encoder_INTERRUPTS_ENABLE;
    
    CyExitCriticalSection(enableInterrupts);
    
    #if (Left_Encoder_COUNTER_SIZE == 32u)
        /* Enable Component interrupts */
        CyIntEnable(Left_Encoder_ISR_NUMBER);
    #endif /* Left_Encoder_COUNTER_SIZE == 32u */
}


/*******************************************************************************
* Function Name: Left_Encoder_Start
********************************************************************************
*
* Summary:
*  Initializes UDBs and other relevant hardware. 
*  Resets counter to 0, enables or disables all relevant interrupts.
*  Starts monitoring the inputs and counting.
*
* Parameters:  
*  None.
*
* Return: 
*  None.
*
* Global variables:
*  Left_Encoder_initVar - used to check initial configuration, modified on 
*  first function call.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Left_Encoder_Start(void)
{  
    #if (Left_Encoder_COUNTER_SIZE == 8u)
    
        Left_Encoder_Cnt8_Start();
        Left_Encoder_Cnt8_WriteCounter(Left_Encoder_COUNTER_INIT_VALUE);
        
    #else /* (Left_Encoder_COUNTER_SIZE == 16u) || (Left_Encoder_COUNTER_SIZE == 32u) */
    
        Left_Encoder_Cnt16_Start();
        Left_Encoder_Cnt16_WriteCounter(Left_Encoder_COUNTER_INIT_VALUE);
        
    #endif /* Left_Encoder_COUNTER_SIZE == 8u */        
    
    if(Left_Encoder_initVar == 0u)
    {
        Left_Encoder_Init();
        Left_Encoder_initVar = 1u;
    }        

    Left_Encoder_Enable();
}


/*******************************************************************************
* Function Name: Left_Encoder_Stop
********************************************************************************
*
* Summary:
*  Turns off UDBs and other relevant hardware.
*
* Parameters:  
*  None.  
*
* Return: 
*  None.
*
*******************************************************************************/
void Left_Encoder_Stop(void) 
{                        
    uint8 enableInterrupts = 0u;
    
    #if (Left_Encoder_COUNTER_SIZE == 8u)
        Left_Encoder_Cnt8_Stop();
    #else /* (Left_Encoder_COUNTER_SIZE == 16u) || (Left_Encoder_COUNTER_SIZE == 32u) */
        Left_Encoder_Cnt16_Stop();                                          /* counter disable */
    #endif /* (Left_Encoder_COUNTER_SIZE == 8u) */
    
    enableInterrupts = CyEnterCriticalSection();
    
    /* Disable interrupts interrupts from Statusi register */
    Left_Encoder_SR_AUX_CONTROL &= ~Left_Encoder_INTERRUPTS_ENABLE;
    
    CyExitCriticalSection(enableInterrupts);
    
    #if (Left_Encoder_COUNTER_SIZE == 32u)
        CyIntDisable(Left_Encoder_ISR_NUMBER);                              /* interrupt disable */
    #endif /* Left_Encoder_COUNTER_SIZE == 32u */
}


/*******************************************************************************
* Function Name: Left_Encoder_GetCounter
********************************************************************************
*
* Summary:
*  Reports the current value of the counter.
*
* Parameters:  
*  None.  
*
* Return: 
*  The counter value. Return type is signed and per 
*  the counter size setting. A positive value indicates 
*  clockwise movement (B before A).
*
* Global variables:
*  Left_Encoder_count32SoftPart - used to get hi 16 bit for current value 
*  of the 32-bit counter, when Counter size equal 32-bit.
*
*******************************************************************************/
int32 Left_Encoder_GetCounter(void) 
{
    int32 count;
    uint16 tmpCnt;   
    
    #if (Left_Encoder_COUNTER_SIZE == 32u)  
    
        int16 hwCount;   
        
    #endif /* Left_Encoder_COUNTER_SIZE == 32u */
    
    #if (Left_Encoder_COUNTER_SIZE == 8u)
    
        tmpCnt = Left_Encoder_Cnt8_ReadCounter();
        count = tmpCnt ^ 0x80u;
        
    #endif /* Left_Encoder_COUNTER_SIZE == 8u */
    
    #if (Left_Encoder_COUNTER_SIZE == 16u)
    
        tmpCnt = Left_Encoder_Cnt16_ReadCounter();
        count = tmpCnt ^ 0x8000u;    
        
    #endif /* Left_Encoder_COUNTER_SIZE == 16u */ 
    
    #if (Left_Encoder_COUNTER_SIZE == 32u)
    
        CyIntDisable(Left_Encoder_ISR_NUMBER);
        
        tmpCnt = Left_Encoder_Cnt16_ReadCounter();
        hwCount = tmpCnt ^ 0x8000u;
        count = Left_Encoder_count32SoftPart + hwCount;
        
        CyIntEnable(Left_Encoder_ISR_NUMBER);
        
    #endif /* Left_Encoder_COUNTER_SIZE == 32u */
        
    return(count);    
}


/*******************************************************************************
* Function Name: Left_Encoder_SetCounter
********************************************************************************
*
* Summary:
*  Sets the current value of the counter.
*
* Parameters:  
*  value:  The new value. Parameter type is signed and per the counter size  
*  setting.  
*
* Return: 
*  None.
*
* Global variables:
*  Left_Encoder_count32SoftPart - modified to set hi 16 bit for current 
*  value of the 32-bit counter, when Counter size equal 32-bit.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Left_Encoder_SetCounter(int32 value)
{    
    #if ((Left_Encoder_COUNTER_SIZE == 8u) || (Left_Encoder_COUNTER_SIZE == 16u))        
        uint16 count;         
    #endif  /* (Left_Encoder_COUNTER_SIZE == 8u) || (Left_Encoder_COUNTER_SIZE == 16u) */   
    
    #if (Left_Encoder_COUNTER_SIZE == 8u)     
    
        count = (value ^ 0x80u);
        Left_Encoder_Cnt8_WriteCounter(count);
        
    #endif  /* Left_Encoder_COUNTER_SIZE == 8u */
    
    #if (Left_Encoder_COUNTER_SIZE == 16u) 
    
        count = (value ^ 0x8000u);
        Left_Encoder_Cnt16_WriteCounter(count);
        
    #endif  /* Left_Encoder_COUNTER_SIZE == 16u */
    
    #if (Left_Encoder_COUNTER_SIZE == 32u)
    
        CyIntDisable(Left_Encoder_ISR_NUMBER);
        
        Left_Encoder_Cnt16_WriteCounter(0x8000u);
        Left_Encoder_count32SoftPart = value;
        
        CyIntEnable(Left_Encoder_ISR_NUMBER);
        
    #endif  /* Left_Encoder_COUNTER_SIZE == 32u */
}


/*******************************************************************************
* Function Name: Left_Encoder_GetEvents
********************************************************************************
* 
* Summary:
*   Reports the current status of events.
*
* Parameters:  
*  None.  
*
* Return: 
*  The events, as bits in an unsigned 8-bit value:
*        Bit      Description
*
*        0        Counter overflow.
*        1        Counter underflow.
*        2        Counter reset due to index, if index input is used.
*        3        Invalid A, B inputs state transition.
*
*******************************************************************************/
uint8 Left_Encoder_GetEvents(void) 
{   
    return(Left_Encoder_STATUS_REG & Left_Encoder_INIT_INT_MASK);
}


/*******************************************************************************
* Function Name: Left_Encoder_SetInterruptMask
********************************************************************************
*
* Summary:
*  Enables / disables interrupts due to the events. 
*  For the 32-bit counter, the overflow, underflow and reset interrupts cannot 
*  be disabled, these bits are ignored.
*
* Parameters:  
*  mask:  Enable / disable bits in an 8-bit value,where 1 enables the interrupt. 
*
* Return: 
*  None.
*
*******************************************************************************/
void Left_Encoder_SetInterruptMask(uint8 mask) 
{
    #if (Left_Encoder_COUNTER_SIZE == 32u)
    
        /* Underflow, Overflow and Reset interrupts for 32-bit Counter are always enable */
        mask |= (Left_Encoder_COUNTER_OVERFLOW | Left_Encoder_COUNTER_UNDERFLOW |
                 Left_Encoder_COUNTER_RESET);
                 
    #endif /* Left_Encoder_COUNTER_SIZE == 32u */
    
    Left_Encoder_STATUS_MASK = mask;
}


/*******************************************************************************
* Function Name: Left_Encoder_GetInterruptMask
********************************************************************************
* 
* Summary:
*  Reports the current interrupt mask settings.
*
* Parameters:  
*  None.
*
* Return: 
*  Enable / disable bits in an 8-bit value, where 1 enables the interrupt.
*  For the 32-bit counter, the overflow, underflow and reset enable bits are 
*  always set.
*
*******************************************************************************/
uint8 Left_Encoder_GetInterruptMask(void) 
{
    return(Left_Encoder_STATUS_MASK & Left_Encoder_INIT_INT_MASK);
}


/* [] END OF FILE */
