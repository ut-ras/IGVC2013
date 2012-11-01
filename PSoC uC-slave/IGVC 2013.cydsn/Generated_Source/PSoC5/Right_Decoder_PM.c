/*******************************************************************************
* File Name: Right_Decoder_PM.c
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

#include "Right_Decoder.h"

static Right_Decoder_BACKUP_STRUCT Right_Decoder_backup = {0u};


/*******************************************************************************
* Function Name: Right_Decoder_SaveConfig
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
void Right_Decoder_SaveConfig(void) 
{
    #if (Right_Decoder_COUNTER_SIZE == 8u)
        Right_Decoder_Cnt8_SaveConfig();
    #else /* (Right_Decoder_COUNTER_SIZE == 16u) || (Right_Decoder_COUNTER_SIZE == 32u) */
        Right_Decoder_Cnt16_SaveConfig();                                          
    #endif /* (Right_Decoder_COUNTER_SIZE == 8u) */
}


/*******************************************************************************
* Function Name: Right_Decoder_RestoreConfig
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
void Right_Decoder_RestoreConfig(void) 
{
    #if (Right_Decoder_COUNTER_SIZE == 8u)
        Right_Decoder_Cnt8_RestoreConfig();
    #else /* (Right_Decoder_COUNTER_SIZE == 16u) || (Right_Decoder_COUNTER_SIZE == 32u) */
        Right_Decoder_Cnt16_RestoreConfig();                                          
    #endif /* (Right_Decoder_COUNTER_SIZE == 8u) */
}


/*******************************************************************************
* Function Name: Right_Decoder_Sleep
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
*  Right_Decoder_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Right_Decoder_Sleep(void)
{   
    if((Right_Decoder_SR_AUX_CONTROL & Right_Decoder_INTERRUPTS_ENABLE) == Right_Decoder_INTERRUPTS_ENABLE)
    {
        Right_Decoder_backup.enableState = 1u;
    }
    else /* The Quadrature Decoder Component is disabled */
    {
        Right_Decoder_backup.enableState = 0u;
    }
    
    Right_Decoder_Stop();
    
    Right_Decoder_SaveConfig();
}


/*******************************************************************************
* Function Name: Right_Decoder_Wakeup
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
*  Right_Decoder_backup - used when non-retention registers are restored.
*
*******************************************************************************/
void Right_Decoder_Wakeup(void) 
{             
    Right_Decoder_RestoreConfig();
    
    if(Right_Decoder_backup.enableState != 0u)
    {       
        #if (Right_Decoder_COUNTER_SIZE == 8u)
            Right_Decoder_Cnt8_Enable();
        #else /* (Right_Decoder_COUNTER_SIZE == 16u) || (Right_Decoder_COUNTER_SIZE == 32u) */
            Right_Decoder_Cnt16_Enable();                                          
        #endif /* (Right_Decoder_COUNTER_SIZE == 8u) */
        
        /* Enable component's operation */
        Right_Decoder_Enable();
    } /* Do nothing if component's block was disabled before */
}


/* [] END OF FILE */

