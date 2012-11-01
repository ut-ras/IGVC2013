/*******************************************************************************
* File Name: Right_Decoder_INT.c  
* Version 2.0
*
* Description:
*  This file contains the Interrupt Service Routine (ISR) for the Quadrature
*  Decoder component.
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

#include "Right_Decoder.h"

volatile int32 Right_Decoder_count32SoftPart = 0u;
static uint8 Right_Decoder_swStatus;


/*******************************************************************************
* FUNCTION NAME: void Right_Decoder_ISR
********************************************************************************
*
* Summary:
*  Interrupt Service Routine for Quadrature Decoder Component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  Right_Decoder_count32SoftPart - modified to update hi 16 bit for current
*  value of the 32-bit counter, when Counter size equal 32-bit.
*  Right_Decoder_swStatus - modified with the updated values of STATUS 
*  register.
*
*******************************************************************************/
CY_ISR( Right_Decoder_ISR )
{
   Right_Decoder_swStatus = Right_Decoder_STATUS_REG;
   
    /* User code required at start of ISR */
    /* `#START Right_Decoder_ISR_START` */

    /* `#END` */
    
    if(Right_Decoder_swStatus & Right_Decoder_COUNTER_OVERFLOW)
    {
        Right_Decoder_count32SoftPart += 0x7FFFu;
    }
    else if(Right_Decoder_swStatus & Right_Decoder_COUNTER_UNDERFLOW)
    {
        Right_Decoder_count32SoftPart -= 0x8000u;
    }
    else
    {
        /* Nothing to do here */
    }
    
    if(Right_Decoder_swStatus & Right_Decoder_COUNTER_RESET)
    {
        Right_Decoder_count32SoftPart = 0u;
    }
    
    /* User code required at end of ISR */
    /* `#START Right_Decoder_ISR_END` */

    /* `#END` */
    
    /* PSoC3 ES1, ES2 Right_Decoder ISR PATCH  */     
    #if(CY_PSOC3_ES2 && (Right_Decoder_isr__ES2_PATCH))
        Right_Decoder_ISR_PATCH();
    #endif /* End CY_PSOC3_ES2 */   
}


/* [] END OF FILE */
