/*******************************************************************************
* File Name: Left_Encoder_PM.c
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

#include "Left_Encoder.h"

static Left_Encoder_BACKUP_STRUCT Left_Encoder_backup = {0u};


/*******************************************************************************
* Function Name: Left_Encoder_SaveConfig
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
void Left_Encoder_SaveConfig(void) 
{
    #if (Left_Encoder_COUNTER_SIZE == 8u)
        Left_Encoder_Cnt8_SaveConfig();
    #else /* (Left_Encoder_COUNTER_SIZE == 16u) || (Left_Encoder_COUNTER_SIZE == 32u) */
        Left_Encoder_Cnt16_SaveConfig();                                          
    #endif /* (Left_Encoder_COUNTER_SIZE == 8u) */
}


/*******************************************************************************
* Function Name: Left_Encoder_RestoreConfig
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
void Left_Encoder_RestoreConfig(void) 
{
    #if (Left_Encoder_COUNTER_SIZE == 8u)
        Left_Encoder_Cnt8_RestoreConfig();
    #else /* (Left_Encoder_COUNTER_SIZE == 16u) || (Left_Encoder_COUNTER_SIZE == 32u) */
        Left_Encoder_Cnt16_RestoreConfig();                                          
    #endif /* (Left_Encoder_COUNTER_SIZE == 8u) */
}


/*******************************************************************************
* Function Name: Left_Encoder_Sleep
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
*  Left_Encoder_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Left_Encoder_Sleep(void)
{   
    if((Left_Encoder_SR_AUX_CONTROL & Left_Encoder_INTERRUPTS_ENABLE) == Left_Encoder_INTERRUPTS_ENABLE)
    {
        Left_Encoder_backup.enableState = 1u;
    }
    else /* The Quadrature Decoder Component is disabled */
    {
        Left_Encoder_backup.enableState = 0u;
    }
    
    Left_Encoder_Stop();
    
    Left_Encoder_SaveConfig();
}


/*******************************************************************************
* Function Name: Left_Encoder_Wakeup
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
*  Left_Encoder_backup - used when non-retention registers are restored.
*
*******************************************************************************/
void Left_Encoder_Wakeup(void) 
{             
    Left_Encoder_RestoreConfig();
    
    if(Left_Encoder_backup.enableState != 0u)
    {       
        #if (Left_Encoder_COUNTER_SIZE == 8u)
            Left_Encoder_Cnt8_Enable();
        #else /* (Left_Encoder_COUNTER_SIZE == 16u) || (Left_Encoder_COUNTER_SIZE == 32u) */
            Left_Encoder_Cnt16_Enable();                                          
        #endif /* (Left_Encoder_COUNTER_SIZE == 8u) */
        
        /* Enable component's operation */
        Left_Encoder_Enable();
    } /* Do nothing if component's block was disabled before */
}


/* [] END OF FILE */

