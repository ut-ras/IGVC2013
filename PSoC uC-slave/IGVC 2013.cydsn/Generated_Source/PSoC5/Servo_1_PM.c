/*******************************************************************************
* File Name: Servo_1_PM.c
* Version 2.10
*
* Description:
*  This file provides the power management source code to API for the
*  PWM.
*
* Note:
*
********************************************************************************
* Copyright 2008-2011, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/
#include "cytypes.h"
#include "Servo_1.h"

static Servo_1_backupStruct Servo_1_backup;


/*******************************************************************************
* Function Name: Servo_1_SaveConfig
********************************************************************************
*
* Summary:
*  Saves the current user configuration of the component.
*  
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  Servo_1_backup:  Variables of this global structure are modified to 
*  store the values of non retention configuration registers when Sleep() API is 
*  called.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Servo_1_SaveConfig(void)
{
    
    #if(!Servo_1_UsingFixedFunction)
        #if (Servo_1_PSOC3_ES2 || Servo_1_PSOC5_ES1)
            Servo_1_backup.PWMUdb = Servo_1_ReadCounter();
            Servo_1_backup.PWMPeriod = Servo_1_ReadPeriod();
            #if (Servo_1_UseStatus)
                Servo_1_backup.InterruptMaskValue = Servo_1_STATUS_MASK;
            #endif
            
            #if(Servo_1_UseOneCompareMode)
                Servo_1_backup.PWMCompareValue = Servo_1_ReadCompare();
            #else
                Servo_1_backup.PWMCompareValue1 = Servo_1_ReadCompare1();
                Servo_1_backup.PWMCompareValue2 = Servo_1_ReadCompare2();
            #endif
            
           #if(Servo_1_DeadBandUsed)
                Servo_1_backup.PWMdeadBandValue = Servo_1_ReadDeadTime();
            #endif
          
            #if ( Servo_1_KillModeMinTime)
                Servo_1_backup.PWMKillCounterPeriod = Servo_1_ReadKillTime();
            #endif
        #endif
        
        #if (Servo_1_PSOC3_ES3 || Servo_1_PSOC5_ES2)
            #if(!Servo_1_PWMModeIsCenterAligned)
                Servo_1_backup.PWMPeriod = Servo_1_ReadPeriod();
            #endif
            Servo_1_backup.PWMUdb = Servo_1_ReadCounter();
            #if (Servo_1_UseStatus)
                Servo_1_backup.InterruptMaskValue = Servo_1_STATUS_MASK;
            #endif
            
            #if(Servo_1_DeadBandMode == Servo_1__B_PWM__DBM_256_CLOCKS || Servo_1_DeadBandMode == Servo_1__B_PWM__DBM_2_4_CLOCKS)
                Servo_1_backup.PWMdeadBandValue = Servo_1_ReadDeadTime();
            #endif
            
            #if(Servo_1_KillModeMinTime)
                 Servo_1_backup.PWMKillCounterPeriod = Servo_1_ReadKillTime();
            #endif
        #endif
        
        #if(Servo_1_UseControl)
            Servo_1_backup.PWMControlRegister = Servo_1_ReadControlRegister();
        #endif
    #endif  
}


/*******************************************************************************
* Function Name: Servo_1_RestoreConfig
********************************************************************************
* 
* Summary:
*  Restores the current user configuration of the component.
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  Servo_1_backup:  Variables of this global structure are used to restore 
*  the values of non retention registers on wakeup from sleep mode.
*
* Reentrant:
*  Yes.
*
*******************************************************************************/
void Servo_1_RestoreConfig(void) 
{
        #if(!Servo_1_UsingFixedFunction)
            #if (Servo_1_PSOC3_ES2 || Servo_1_PSOC5_ES1)
                /* Interrupt State Backup for Critical Region*/
                uint8 Servo_1_interruptState;
                /* Enter Critical Region*/
                Servo_1_interruptState = CyEnterCriticalSection();
                #if (Servo_1_UseStatus)
                    /* Use the interrupt output of the status register for IRQ output */
                    Servo_1_STATUS_AUX_CTRL |= Servo_1_STATUS_ACTL_INT_EN_MASK;
                    
                    Servo_1_STATUS_MASK = Servo_1_backup.InterruptMaskValue;
                #endif
                
                #if (Servo_1_Resolution == 8)
                    /* Set FIFO 0 to 1 byte register for period*/
                    Servo_1_AUX_CONTROLDP0 |= (Servo_1_AUX_CTRL_FIFO0_CLR);
                #else /* (Servo_1_Resolution == 16)*/
                    /* Set FIFO 0 to 1 byte register for period */
                    Servo_1_AUX_CONTROLDP0 |= (Servo_1_AUX_CTRL_FIFO0_CLR);
                    Servo_1_AUX_CONTROLDP1 |= (Servo_1_AUX_CTRL_FIFO0_CLR);
                #endif
                /* Exit Critical Region*/
                CyExitCriticalSection(Servo_1_interruptState);
                
                Servo_1_WriteCounter(Servo_1_backup.PWMUdb);
                Servo_1_WritePeriod(Servo_1_backup.PWMPeriod);
                
                #if(Servo_1_UseOneCompareMode)
                    Servo_1_WriteCompare(Servo_1_backup.PWMCompareValue);
                #else
                    Servo_1_WriteCompare1(Servo_1_backup.PWMCompareValue1);
                    Servo_1_WriteCompare2(Servo_1_backup.PWMCompareValue2);
                #endif
                
               #if(Servo_1_DeadBandMode == Servo_1__B_PWM__DBM_256_CLOCKS || Servo_1_DeadBandMode == Servo_1__B_PWM__DBM_2_4_CLOCKS)
                    Servo_1_WriteDeadTime(Servo_1_backup.PWMdeadBandValue);
                #endif
            
                #if ( Servo_1_KillModeMinTime)
                    Servo_1_WriteKillTime(Servo_1_backup.PWMKillCounterPeriod);
                #endif
            #endif
            
            #if (Servo_1_PSOC3_ES3 || Servo_1_PSOC5_ES2)
                #if(!Servo_1_PWMModeIsCenterAligned)
                    Servo_1_WritePeriod(Servo_1_backup.PWMPeriod);
                #endif
                Servo_1_WriteCounter(Servo_1_backup.PWMUdb);
                #if (Servo_1_UseStatus)
                    Servo_1_STATUS_MASK = Servo_1_backup.InterruptMaskValue;
                #endif
                
                #if(Servo_1_DeadBandMode == Servo_1__B_PWM__DBM_256_CLOCKS || Servo_1_DeadBandMode == Servo_1__B_PWM__DBM_2_4_CLOCKS)
                    Servo_1_WriteDeadTime(Servo_1_backup.PWMdeadBandValue);
                #endif
                
                #if(Servo_1_KillModeMinTime)
                    Servo_1_WriteKillTime(Servo_1_backup.PWMKillCounterPeriod);
                #endif
            #endif
            
            #if(Servo_1_UseControl)
                Servo_1_WriteControlRegister(Servo_1_backup.PWMControlRegister); 
            #endif
        #endif  
    }


/*******************************************************************************
* Function Name: Servo_1_Sleep
********************************************************************************
* 
* Summary:
*  Disables block's operation and saves the user configuration. Should be called 
*  just prior to entering sleep.
*  
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  Servo_1_backup.PWMEnableState:  Is modified depending on the enable state
*  of the block before entering sleep mode.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Servo_1_Sleep(void)
{
    #if(Servo_1_UseControl)
        if(Servo_1_CTRL_ENABLE == (Servo_1_CONTROL & Servo_1_CTRL_ENABLE))
        {
            /*Component is enabled */
            Servo_1_backup.PWMEnableState = 1u;
        }
        else
        {
            /* Component is disabled */
            Servo_1_backup.PWMEnableState = 0u;
        }
    #endif
    /* Stop component */
    Servo_1_Stop();
    
    /* Save registers configuration */
    Servo_1_SaveConfig();
}


/*******************************************************************************
* Function Name: Servo_1_Wakeup
********************************************************************************
* 
* Summary:
*  Restores and enables the user configuration. Should be called just after 
*  awaking from sleep.
*  
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  Servo_1_backup.pwmEnable:  Is used to restore the enable state of 
*  block on wakeup from sleep mode.
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_1_Wakeup(void) 
{
     /* Restore registers values */
    Servo_1_RestoreConfig();
    
    if(Servo_1_backup.PWMEnableState != 0u)
    {
        /* Enable component's operation */
        Servo_1_Enable();
    } /* Do nothing if component's block was disabled before */
    
}

/* [] END OF FILE */
