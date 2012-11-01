/*******************************************************************************
* File Name: Left_Encoder_INT.c  
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

#include "Left_Encoder.h"

volatile int32 Left_Encoder_count32SoftPart = 0u;
static uint8 Left_Encoder_swStatus;


/*******************************************************************************
* FUNCTION NAME: void Left_Encoder_ISR
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
*  Left_Encoder_count32SoftPart - modified to update hi 16 bit for current
*  value of the 32-bit counter, when Counter size equal 32-bit.
*  Left_Encoder_swStatus - modified with the updated values of STATUS 
*  register.
*
*******************************************************************************/
CY_ISR( Left_Encoder_ISR )
{
   Left_Encoder_swStatus = Left_Encoder_STATUS_REG;
   
    /* User code required at start of ISR */
    /* `#START Left_Encoder_ISR_START` */

    /* `#END` */
    
    if(Left_Encoder_swStatus & Left_Encoder_COUNTER_OVERFLOW)
    {
        Left_Encoder_count32SoftPart += 0x7FFFu;
    }
    else if(Left_Encoder_swStatus & Left_Encoder_COUNTER_UNDERFLOW)
    {
        Left_Encoder_count32SoftPart -= 0x8000u;
    }
    else
    {
        /* Nothing to do here */
    }
    
    if(Left_Encoder_swStatus & Left_Encoder_COUNTER_RESET)
    {
        Left_Encoder_count32SoftPart = 0u;
    }
    
    /* User code required at end of ISR */
    /* `#START Left_Encoder_ISR_END` */

    /* `#END` */
    
    /* PSoC3 ES1, ES2 Left_Encoder ISR PATCH  */     
    #if(CY_PSOC3_ES2 && (Left_Encoder_isr__ES2_PATCH))
        Left_Encoder_ISR_PATCH();
    #endif /* End CY_PSOC3_ES2 */   
}


/* [] END OF FILE */
