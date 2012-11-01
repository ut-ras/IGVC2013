/*******************************************************************************
* File Name: Right_Decoder.c  
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
#include "Right_Decoder.h"

#if(Right_Decoder_COUNTER_SIZE == 32u)
    extern volatile int32 Right_Decoder_count32SoftPart;    
#endif /*Right_Decoder_COUNTER_SIZE == 32u*/

uint8 Right_Decoder_initVar = 0u;


/*******************************************************************************
* Function Name: Right_Decoder_Init
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
void Right_Decoder_Init(void) 
{      
    #if (Right_Decoder_COUNTER_SIZE == 32u)
      
        /* Disable Interrupt. */
        CyIntDisable(Right_Decoder_ISR_NUMBER);
        
        /* Set the ISR to point to the Right_Decoder_isr Interrupt. */
        CyIntSetVector(Right_Decoder_ISR_NUMBER, Right_Decoder_ISR);
        
        /* Set the priority. */
        CyIntSetPriority(Right_Decoder_ISR_NUMBER, Right_Decoder_ISR_PRIORITY);       
        
    #endif /* Right_Decoder_COUNTER_SIZE == 32u */    
}


/*******************************************************************************
* Function Name: Right_Decoder_Enable
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
void Right_Decoder_Enable(void) 
{
    uint8 enableInterrupts = 0u;
    
    Right_Decoder_SetInterruptMask(Right_Decoder_INIT_INT_MASK);
    
    enableInterrupts = CyEnterCriticalSection();
    
    /* Enable interrupts from Statusi register */
    Right_Decoder_SR_AUX_CONTROL |= Right_Decoder_INTERRUPTS_ENABLE;
    
    CyExitCriticalSection(enableInterrupts);
    
    #if (Right_Decoder_COUNTER_SIZE == 32u)
        /* Enable Component interrupts */
        CyIntEnable(Right_Decoder_ISR_NUMBER);
    #endif /* Right_Decoder_COUNTER_SIZE == 32u */
}


/*******************************************************************************
* Function Name: Right_Decoder_Start
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
*  Right_Decoder_initVar - used to check initial configuration, modified on 
*  first function call.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Right_Decoder_Start(void)
{  
    #if (Right_Decoder_COUNTER_SIZE == 8u)
    
        Right_Decoder_Cnt8_Start();
        Right_Decoder_Cnt8_WriteCounter(Right_Decoder_COUNTER_INIT_VALUE);
        
    #else /* (Right_Decoder_COUNTER_SIZE == 16u) || (Right_Decoder_COUNTER_SIZE == 32u) */
    
        Right_Decoder_Cnt16_Start();
        Right_Decoder_Cnt16_WriteCounter(Right_Decoder_COUNTER_INIT_VALUE);
        
    #endif /* Right_Decoder_COUNTER_SIZE == 8u */        
    
    if(Right_Decoder_initVar == 0u)
    {
        Right_Decoder_Init();
        Right_Decoder_initVar = 1u;
    }        

    Right_Decoder_Enable();
}


/*******************************************************************************
* Function Name: Right_Decoder_Stop
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
void Right_Decoder_Stop(void) 
{                        
    uint8 enableInterrupts = 0u;
    
    #if (Right_Decoder_COUNTER_SIZE == 8u)
        Right_Decoder_Cnt8_Stop();
    #else /* (Right_Decoder_COUNTER_SIZE == 16u) || (Right_Decoder_COUNTER_SIZE == 32u) */
        Right_Decoder_Cnt16_Stop();                                          /* counter disable */
    #endif /* (Right_Decoder_COUNTER_SIZE == 8u) */
    
    enableInterrupts = CyEnterCriticalSection();
    
    /* Disable interrupts interrupts from Statusi register */
    Right_Decoder_SR_AUX_CONTROL &= ~Right_Decoder_INTERRUPTS_ENABLE;
    
    CyExitCriticalSection(enableInterrupts);
    
    #if (Right_Decoder_COUNTER_SIZE == 32u)
        CyIntDisable(Right_Decoder_ISR_NUMBER);                              /* interrupt disable */
    #endif /* Right_Decoder_COUNTER_SIZE == 32u */
}


/*******************************************************************************
* Function Name: Right_Decoder_GetCounter
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
*  Right_Decoder_count32SoftPart - used to get hi 16 bit for current value 
*  of the 32-bit counter, when Counter size equal 32-bit.
*
*******************************************************************************/
int32 Right_Decoder_GetCounter(void) 
{
    int32 count;
    uint16 tmpCnt;   
    
    #if (Right_Decoder_COUNTER_SIZE == 32u)  
    
        int16 hwCount;   
        
    #endif /* Right_Decoder_COUNTER_SIZE == 32u */
    
    #if (Right_Decoder_COUNTER_SIZE == 8u)
    
        tmpCnt = Right_Decoder_Cnt8_ReadCounter();
        count = tmpCnt ^ 0x80u;
        
    #endif /* Right_Decoder_COUNTER_SIZE == 8u */
    
    #if (Right_Decoder_COUNTER_SIZE == 16u)
    
        tmpCnt = Right_Decoder_Cnt16_ReadCounter();
        count = tmpCnt ^ 0x8000u;    
        
    #endif /* Right_Decoder_COUNTER_SIZE == 16u */ 
    
    #if (Right_Decoder_COUNTER_SIZE == 32u)
    
        CyIntDisable(Right_Decoder_ISR_NUMBER);
        
        tmpCnt = Right_Decoder_Cnt16_ReadCounter();
        hwCount = tmpCnt ^ 0x8000u;
        count = Right_Decoder_count32SoftPart + hwCount;
        
        CyIntEnable(Right_Decoder_ISR_NUMBER);
        
    #endif /* Right_Decoder_COUNTER_SIZE == 32u */
        
    return(count);    
}


/*******************************************************************************
* Function Name: Right_Decoder_SetCounter
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
*  Right_Decoder_count32SoftPart - modified to set hi 16 bit for current 
*  value of the 32-bit counter, when Counter size equal 32-bit.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Right_Decoder_SetCounter(int32 value)
{    
    #if ((Right_Decoder_COUNTER_SIZE == 8u) || (Right_Decoder_COUNTER_SIZE == 16u))        
        uint16 count;         
    #endif  /* (Right_Decoder_COUNTER_SIZE == 8u) || (Right_Decoder_COUNTER_SIZE == 16u) */   
    
    #if (Right_Decoder_COUNTER_SIZE == 8u)     
    
        count = (value ^ 0x80u);
        Right_Decoder_Cnt8_WriteCounter(count);
        
    #endif  /* Right_Decoder_COUNTER_SIZE == 8u */
    
    #if (Right_Decoder_COUNTER_SIZE == 16u) 
    
        count = (value ^ 0x8000u);
        Right_Decoder_Cnt16_WriteCounter(count);
        
    #endif  /* Right_Decoder_COUNTER_SIZE == 16u */
    
    #if (Right_Decoder_COUNTER_SIZE == 32u)
    
        CyIntDisable(Right_Decoder_ISR_NUMBER);
        
        Right_Decoder_Cnt16_WriteCounter(0x8000u);
        Right_Decoder_count32SoftPart = value;
        
        CyIntEnable(Right_Decoder_ISR_NUMBER);
        
    #endif  /* Right_Decoder_COUNTER_SIZE == 32u */
}


/*******************************************************************************
* Function Name: Right_Decoder_GetEvents
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
uint8 Right_Decoder_GetEvents(void) 
{   
    return(Right_Decoder_STATUS_REG & Right_Decoder_INIT_INT_MASK);
}


/*******************************************************************************
* Function Name: Right_Decoder_SetInterruptMask
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
void Right_Decoder_SetInterruptMask(uint8 mask) 
{
    #if (Right_Decoder_COUNTER_SIZE == 32u)
    
        /* Underflow, Overflow and Reset interrupts for 32-bit Counter are always enable */
        mask |= (Right_Decoder_COUNTER_OVERFLOW | Right_Decoder_COUNTER_UNDERFLOW |
                 Right_Decoder_COUNTER_RESET);
                 
    #endif /* Right_Decoder_COUNTER_SIZE == 32u */
    
    Right_Decoder_STATUS_MASK = mask;
}


/*******************************************************************************
* Function Name: Right_Decoder_GetInterruptMask
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
uint8 Right_Decoder_GetInterruptMask(void) 
{
    return(Right_Decoder_STATUS_MASK & Right_Decoder_INIT_INT_MASK);
}


/* [] END OF FILE */
