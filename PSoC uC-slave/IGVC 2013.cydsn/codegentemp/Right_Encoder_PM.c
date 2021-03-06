/*******************************************************************************
* File Name: Right_Encoder_PM.c
* Version 2.0
*
* Description:
*  This file contains the setup, control and status commands to support 
*  component operations in low power mode.  
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

#include "Right_Encoder.h"

static Right_Encoder_BACKUP_STRUCT Right_Encoder_backup = {0u};


/*******************************************************************************
* Function Name: Right_Encoder_SaveConfig
********************************************************************************
* Summary:
*  Saves the current user configuration of the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void Right_Encoder_SaveConfig(void) 
{
    #if (Right_Encoder_COUNTER_SIZE == 8u)
        Right_Encoder_Cnt8_SaveConfig();
    #else /* (Right_Encoder_COUNTER_SIZE == 16u) || (Right_Encoder_COUNTER_SIZE == 32u) */
        Right_Encoder_Cnt16_SaveConfig();                                          
    #endif /* (Right_Encoder_COUNTER_SIZE == 8u) */
}


/*******************************************************************************
* Function Name: Right_Encoder_RestoreConfig
********************************************************************************
* Summary:
*  Restores the current user configuration of the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void Right_Encoder_RestoreConfig(void) 
{
    #if (Right_Encoder_COUNTER_SIZE == 8u)
        Right_Encoder_Cnt8_RestoreConfig();
    #else /* (Right_Encoder_COUNTER_SIZE == 16u) || (Right_Encoder_COUNTER_SIZE == 32u) */
        Right_Encoder_Cnt16_RestoreConfig();                                          
    #endif /* (Right_Encoder_COUNTER_SIZE == 8u) */
}


/*******************************************************************************
* Function Name: Right_Encoder_Sleep
********************************************************************************
* 
* Summary:
*  Prepare Quadrature Decoder Component goes to sleep.
*
* Parameters:  
*  None.  
*
* Return: 
*  None.
*
* Global Variables:
*  Right_Encoder_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Right_Encoder_Sleep(void)
{   
    if((Right_Encoder_SR_AUX_CONTROL & Right_Encoder_INTERRUPTS_ENABLE) == Right_Encoder_INTERRUPTS_ENABLE)
    {
        Right_Encoder_backup.enableState = 1u;
    }
    else /* The Quadrature Decoder Component is disabled */
    {
        Right_Encoder_backup.enableState = 0u;
    }
    
    Right_Encoder_Stop();
    
    Right_Encoder_SaveConfig();
}


/*******************************************************************************
* Function Name: Right_Encoder_Wakeup
********************************************************************************
* 
* Summary:
*  Prepare Quadrature Decoder Component to wake up.
*
* Parameters:  
*  None.
*
* Return: 
*  None.
*
* Global Variables:
*  Right_Encoder_backup - used when non-retention registers are restored.
*
*******************************************************************************/
void Right_Encoder_Wakeup(void) 
{             
    Right_Encoder_RestoreConfig();
    
    if(Right_Encoder_backup.enableState != 0u)
    {       
        #if (Right_Encoder_COUNTER_SIZE == 8u)
            Right_Encoder_Cnt8_Enable();
        #else /* (Right_Encoder_COUNTER_SIZE == 16u) || (Right_Encoder_COUNTER_SIZE == 32u) */
            Right_Encoder_Cnt16_Enable();                                          
        #endif /* (Right_Encoder_COUNTER_SIZE == 8u) */
        
        /* Enable component's operation */
        Right_Encoder_Enable();
    } /* Do nothing if component's block was disabled before */
}


/* [] END OF FILE */

