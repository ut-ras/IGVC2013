/*******************************************************************************
* File Name: Left_Encoder_Cnt16_PM.c  
* Version 2.0
*
*  Description:
*     This file provides the power management source code to API for the
*     Counter.  
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

#include "Left_Encoder_Cnt16.h"

static Left_Encoder_Cnt16_backupStruct Left_Encoder_Cnt16_backup;

/*******************************************************************************
* Function Name: Left_Encoder_Cnt16_SaveConfig
********************************************************************************
* Summary:
*     Save the current user configuration
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  Left_Encoder_Cnt16_backup:  Variables of this global structure are modified to 
*  store the values of non retention configuration registers when Sleep() API is 
*  called.
*
* Reentrant:
*    No
*
*******************************************************************************/
void Left_Encoder_Cnt16_SaveConfig(void)
{
    #if (!Left_Encoder_Cnt16_UsingFixedFunction)
        /* Backup the UDB non-rentention registers for PSoC3 ES2 and PSoC5 ES1*/
        #if (Left_Encoder_Cnt16_PSOC3_ES2 || Left_Encoder_Cnt16_PSOC5_ES1)
            Left_Encoder_Cnt16_backup.CounterUdb = Left_Encoder_Cnt16_ReadCounter();
            Left_Encoder_Cnt16_backup.CounterPeriod = Left_Encoder_Cnt16_ReadPeriod();
            Left_Encoder_Cnt16_backup.CompareValue = Left_Encoder_Cnt16_ReadCompare();
            Left_Encoder_Cnt16_backup.InterruptMaskValue = Left_Encoder_Cnt16_STATUS_MASK;
        #endif
		
		#if (Left_Encoder_Cnt16_PSOC3_ES3 || Left_Encoder_Cnt16_PSOC5_ES2)
			Left_Encoder_Cnt16_backup.CounterUdb = Left_Encoder_Cnt16_ReadCounter();
			Left_Encoder_Cnt16_backup.InterruptMaskValue = Left_Encoder_Cnt16_STATUS_MASK;
		#endif
		
        #if(!Left_Encoder_Cnt16_ControlRegRemoved)
            Left_Encoder_Cnt16_backup.CounterControlRegister = Left_Encoder_Cnt16_ReadControlRegister();
		#endif
    #endif
}

/*******************************************************************************
* Function Name: Left_Encoder_Cnt16_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  Left_Encoder_Cnt16_backup:  Variables of this global structure are used to restore 
*  the values of non retention registers on wakeup from sleep mode.
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Left_Encoder_Cnt16_RestoreConfig(void) 
{      
    #if (!Left_Encoder_Cnt16_UsingFixedFunction)     
        /* Restore the UDB non-rentention registers for PSoC3 ES2 and PSoC5 ES1*/
        #if (Left_Encoder_Cnt16_PSOC3_ES2 || Left_Encoder_Cnt16_PSOC5_ES1)
            Left_Encoder_Cnt16_WriteCounter(Left_Encoder_Cnt16_backup.CounterUdb);
            Left_Encoder_Cnt16_WritePeriod(Left_Encoder_Cnt16_backup.CounterPeriod);
            Left_Encoder_Cnt16_WriteCompare(Left_Encoder_Cnt16_backup.CompareValue);
            Left_Encoder_Cnt16_STATUS_MASK = Left_Encoder_Cnt16_backup.InterruptMaskValue;
        #endif
		
		#if (Left_Encoder_Cnt16_PSOC3_ES3 || Left_Encoder_Cnt16_PSOC5_ES2)
			Left_Encoder_Cnt16_WriteCounter(Left_Encoder_Cnt16_backup.CounterUdb);
			Left_Encoder_Cnt16_STATUS_MASK = Left_Encoder_Cnt16_backup.InterruptMaskValue;
		#endif
		
		
        #if(!Left_Encoder_Cnt16_ControlRegRemoved)
            Left_Encoder_Cnt16_WriteControlRegister(Left_Encoder_Cnt16_backup.CounterControlRegister);
        #endif
    #endif
}


/*******************************************************************************
* Function Name: Left_Encoder_Cnt16_Sleep
********************************************************************************
* Summary:
*     Stop and Save the user configuration
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  Left_Encoder_Cnt16_backup.enableState:  Is modified depending on the enable state
*  of the block before entering sleep mode.
*
* Reentrant:
*  No
*
*******************************************************************************/
void Left_Encoder_Cnt16_Sleep(void)
{
    #if(!Left_Encoder_Cnt16_ControlRegRemoved)
        /* Save Counter's enable state */
        if(Left_Encoder_Cnt16_CTRL_ENABLE == (Left_Encoder_Cnt16_CONTROL & Left_Encoder_Cnt16_CTRL_ENABLE))
        {
            /* Counter is enabled */
            Left_Encoder_Cnt16_backup.CounterEnableState = 1u;
        }
        else
        {
            /* Counter is disabled */
            Left_Encoder_Cnt16_backup.CounterEnableState = 0u;
        }
    #endif
    Left_Encoder_Cnt16_Stop();
    Left_Encoder_Cnt16_SaveConfig();
}


/*******************************************************************************
* Function Name: Left_Encoder_Cnt16_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration
*  
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  Left_Encoder_Cnt16_backup.enableState:  Is used to restore the enable state of 
*  block on wakeup from sleep mode.
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Left_Encoder_Cnt16_Wakeup(void) 
{
    Left_Encoder_Cnt16_RestoreConfig();
    #if(!Left_Encoder_Cnt16_ControlRegRemoved)
        if(Left_Encoder_Cnt16_backup.CounterEnableState == 1u)
        {
            /* Enable Counter's operation */
            Left_Encoder_Cnt16_Enable();
        } /* Do nothing if Counter was disabled before */    
    #endif
    
}


/* [] END OF FILE */
