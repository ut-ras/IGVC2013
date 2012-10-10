/*******************************************************************************
* File Name: MainTimer_PM.c
* Version 2.20
*
*  Description:
*     This file provides the power management source code to API for the
*     Timer.
*
*   Note:
*     None
*
*******************************************************************************
* Copyright 2008-2011, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

#include "MainTimer.h"
static MainTimer_backupStruct MainTimer_backup;


/*******************************************************************************
* Function Name: MainTimer_SaveConfig
********************************************************************************
*
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
*  MainTimer_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
* Reentrant:
*    No
*
*******************************************************************************/
void MainTimer_SaveConfig(void)
{
    #if (!MainTimer_UsingFixedFunction)
        /* Backup the UDB non-rentention registers for PSoC3 ES2 and PSoC5 ES1*/
        #if (MainTimer_PSOC3_ES2 || MainTimer_PSOC5_ES1)
            MainTimer_backup.TimerUdb = MainTimer_ReadCounter();
            MainTimer_backup.TimerPeriod = MainTimer_ReadPeriod();
            MainTimer_backup.InterruptMaskValue = MainTimer_STATUS_MASK;
            #if (MainTimer_UsingHWCaptureCounter)
                MainTimer_backup.TimerCaptureCounter = MainTimer_ReadCaptureCount();
            #endif /* Backup the UDB non-rentention register capture counter for PSoC3 ES2 and PSoC5 ES1 */
        #endif /* Backup the UDB non-rentention registers for PSoC3 ES2 and PSoC5 ES1*/

        #if (MainTimer_PSOC3_ES3 || MainTimer_PSOC5_ES2)
            MainTimer_backup.TimerUdb = MainTimer_ReadCounter();
            MainTimer_backup.InterruptMaskValue = MainTimer_STATUS_MASK;
            #if (MainTimer_UsingHWCaptureCounter)
                MainTimer_backup.TimerCaptureCounter = MainTimer_ReadCaptureCount();
            #endif /* Back Up capture counter register  */
        #endif /* Backup non retention registers, interrupt mask and capture counter for PSoC3ES3 or PSoC5ES2 */

        #if(!MainTimer_ControlRegRemoved)
            MainTimer_backup.TimerControlRegister = MainTimer_ReadControlRegister();
        #endif /* Backup the enable state of the Timer component */
    #endif /* Backup non retention registers in UDB implementation. All fixed function registers are retention */
}


/*******************************************************************************
* Function Name: MainTimer_RestoreConfig
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
*  MainTimer_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
* Reentrant:
*    Yes
*
*******************************************************************************/
void MainTimer_RestoreConfig(void) 
{   
    #if (!MainTimer_UsingFixedFunction)
        /* Restore the UDB non-rentention registers for PSoC3 ES2 and PSoC5 ES1*/
        #if (MainTimer_PSOC3_ES2 || MainTimer_PSOC5_ES1)
            /* Interrupt State Backup for Critical Region*/
            uint8 MainTimer_interruptState;

            MainTimer_WriteCounter(MainTimer_backup.TimerUdb);
            MainTimer_WritePeriod(MainTimer_backup.TimerPeriod);
            /* CyEnterCriticalRegion and CyExitCriticalRegion are used to mark following region critical*/
            /* Enter Critical Region*/
            MainTimer_interruptState = CyEnterCriticalSection();
            /* Use the interrupt output of the status register for IRQ output */
            MainTimer_STATUS_AUX_CTRL |= MainTimer_STATUS_ACTL_INT_EN_MASK;
            /* Exit Critical Region*/
            CyExitCriticalSection(MainTimer_interruptState);
            MainTimer_STATUS_MASK =MainTimer_backup.InterruptMaskValue;
            #if (MainTimer_UsingHWCaptureCounter)
                MainTimer_SetCaptureCount(MainTimer_backup.TimerCaptureCounter);
            #endif /* Restore the UDB non-rentention register capture counter for PSoC3 ES2 and PSoC5 ES1 */
        #endif /* Restore the UDB non-rentention registers for PSoC3 ES2 and PSoC5 ES1*/

        #if (MainTimer_PSOC3_ES3 || MainTimer_PSOC5_ES2)
            MainTimer_WriteCounter(MainTimer_backup.TimerUdb);
            MainTimer_STATUS_MASK =MainTimer_backup.InterruptMaskValue;
            #if (MainTimer_UsingHWCaptureCounter)
                MainTimer_SetCaptureCount(MainTimer_backup.TimerCaptureCounter);
            #endif /* Restore Capture counter register*/
        #endif /* Restore up non retention registers, interrupt mask and capture counter for PSoC3ES3 or PSoC5ES2 */

        #if(!MainTimer_ControlRegRemoved)
            MainTimer_WriteControlRegister(MainTimer_backup.TimerControlRegister);
        #endif /* Restore the enable state of the Timer component */
    #endif /* Restore non retention registers in the UDB implementation only */
}


/*******************************************************************************
* Function Name: MainTimer_Sleep
********************************************************************************
*
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
*  MainTimer_backup.TimerEnableState:  Is modified depending on the
*  enable state of the block before entering sleep mode.
*
* Reentrant:
*    No
*
*******************************************************************************/
void MainTimer_Sleep(void)
{
    #if(!MainTimer_ControlRegRemoved)
        /* Save Counter's enable state */
        if(MainTimer_CTRL_ENABLE == (MainTimer_CONTROL & MainTimer_CTRL_ENABLE))
        {
            /* Timer is enabled */
            MainTimer_backup.TimerEnableState = 1u;
        }
        else
        {
            /* Timer is disabled */
            MainTimer_backup.TimerEnableState = 0u;
        }
    #endif /* Back up enable state from the Timer control register */
    MainTimer_Stop();
    MainTimer_SaveConfig();
}


/*******************************************************************************
* Function Name: MainTimer_Wakeup
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
*  MainTimer_backup.enableState:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
* Reentrant:
*    Yes
*
*******************************************************************************/
void MainTimer_Wakeup(void) 
{
    MainTimer_RestoreConfig();
    #if(!MainTimer_ControlRegRemoved)
        if(MainTimer_backup.TimerEnableState == 1u)
        {     /* Enable Timer's operation */
                MainTimer_Enable();
        } /* Do nothing if Timer was disabled before */
    #endif /* Remove this code section if Control register is removed */
}


/* [] END OF FILE */
