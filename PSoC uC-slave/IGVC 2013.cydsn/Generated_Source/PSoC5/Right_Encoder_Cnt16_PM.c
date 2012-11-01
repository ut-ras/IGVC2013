/*******************************************************************************
* File Name: Right_Encoder_Cnt16_PM.c  
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

#include "Right_Encoder_Cnt16.h"

static Right_Encoder_Cnt16_backupStruct Right_Encoder_Cnt16_backup;

/*******************************************************************************
* Function Name: Right_Encoder_Cnt16_SaveConfig
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
*  Right_Encoder_Cnt16_backup:  Variables of this global structure are modified to 
*  store the values of non retention configuration registers when Sleep() API is 
*  called.
*
* Reentrant:
*    No
*
*******************************************************************************/
void Right_Encoder_Cnt16_SaveConfig(void)
{
    #if (!Right_Encoder_Cnt16_UsingFixedFunction)
        /* Backup the UDB non-rentention registers for PSoC3 ES2 and PSoC5 ES1*/
        #if (Right_Encoder_Cnt16_PSOC3_ES2 || Right_Encoder_Cnt16_PSOC5_ES1)
            Right_Encoder_Cnt16_backup.CounterUdb = Right_Encoder_Cnt16_ReadCounter();
            Right_Encoder_Cnt16_backup.CounterPeriod = Right_Encoder_Cnt16_ReadPeriod();
            Right_Encoder_Cnt16_backup.CompareValue = Right_Encoder_Cnt16_ReadCompare();
            Right_Encoder_Cnt16_backup.InterruptMaskValue = Right_Encoder_Cnt16_STATUS_MASK;
        #endif
		
		#if (Right_Encoder_Cnt16_PSOC3_ES3 || Right_Encoder_Cnt16_PSOC5_ES2)
			Right_Encoder_Cnt16_backup.CounterUdb = Right_Encoder_Cnt16_ReadCounter();
			Right_Encoder_Cnt16_backup.InterruptMaskValue = Right_Encoder_Cnt16_STATUS_MASK;
		#endif
		
        #if(!Right_Encoder_Cnt16_ControlRegRemoved)
            Right_Encoder_Cnt16_backup.CounterControlRegister = Right_Encoder_Cnt16_ReadControlRegister();
		#endif
    #endif
}

/*******************************************************************************
* Function Name: Right_Encoder_Cnt16_RestoreConfig
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
*  Right_Encoder_Cnt16_backup:  Variables of this global structure are used to restore 
*  the values of non retention registers on wakeup from sleep mode.
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Right_Encoder_Cnt16_RestoreConfig(void) 
{      
    #if (!Right_Encoder_Cnt16_UsingFixedFunction)     
        /* Restore the UDB non-rentention registers for PSoC3 ES2 and PSoC5 ES1*/
        #if (Right_Encoder_Cnt16_PSOC3_ES2 || Right_Encoder_Cnt16_PSOC5_ES1)
            Right_Encoder_Cnt16_WriteCounter(Right_Encoder_Cnt16_backup.CounterUdb);
            Right_Encoder_Cnt16_WritePeriod(Right_Encoder_Cnt16_backup.CounterPeriod);
            Right_Encoder_Cnt16_WriteCompare(Right_Encoder_Cnt16_backup.CompareValue);
            Right_Encoder_Cnt16_STATUS_MASK = Right_Encoder_Cnt16_backup.InterruptMaskValue;
        #endif
		
		#if (Right_Encoder_Cnt16_PSOC3_ES3 || Right_Encoder_Cnt16_PSOC5_ES2)
			Right_Encoder_Cnt16_WriteCounter(Right_Encoder_Cnt16_backup.CounterUdb);
			Right_Encoder_Cnt16_STATUS_MASK = Right_Encoder_Cnt16_backup.InterruptMaskValue;
		#endif
		
		
        #if(!Right_Encoder_Cnt16_ControlRegRemoved)
            Right_Encoder_Cnt16_WriteControlRegister(Right_Encoder_Cnt16_backup.CounterControlRegister);
        #endif
    #endif
}


/*******************************************************************************
* Function Name: Right_Encoder_Cnt16_Sleep
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
*  Right_Encoder_Cnt16_backup.enableState:  Is modified depending on the enable state
*  of the block before entering sleep mode.
*
* Reentrant:
*  No
*
*******************************************************************************/
void Right_Encoder_Cnt16_Sleep(void)
{
    #if(!Right_Encoder_Cnt16_ControlRegRemoved)
        /* Save Counter's enable state */
        if(Right_Encoder_Cnt16_CTRL_ENABLE == (Right_Encoder_Cnt16_CONTROL & Right_Encoder_Cnt16_CTRL_ENABLE))
        {
            /* Counter is enabled */
            Right_Encoder_Cnt16_backup.CounterEnableState = 1u;
        }
        else
        {
            /* Counter is disabled */
            Right_Encoder_Cnt16_backup.CounterEnableState = 0u;
        }
    #endif
    Right_Encoder_Cnt16_Stop();
    Right_Encoder_Cnt16_SaveConfig();
}


/*******************************************************************************
* Function Name: Right_Encoder_Cnt16_Wakeup
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
*  Right_Encoder_Cnt16_backup.enableState:  Is used to restore the enable state of 
*  block on wakeup from sleep mode.
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Right_Encoder_Cnt16_Wakeup(void) 
{
    Right_Encoder_Cnt16_RestoreConfig();
    #if(!Right_Encoder_Cnt16_ControlRegRemoved)
        if(Right_Encoder_Cnt16_backup.CounterEnableState == 1u)
        {
            /* Enable Counter's operation */
            Right_Encoder_Cnt16_Enable();
        } /* Do nothing if Counter was disabled before */    
    #endif
    
}


/* [] END OF FILE */
