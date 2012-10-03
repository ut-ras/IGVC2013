/*******************************************************************************
* File Name: Servo_PM.c
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
#include "Servo.h"

static Servo_backupStruct Servo_backup;


/*******************************************************************************
* Function Name: Servo_SaveConfig
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
*  Servo_backup:  Variables of this global structure are modified to 
*  store the values of non retention configuration registers when Sleep() API is 
*  called.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Servo_SaveConfig(void)
{
    
    #if(!Servo_UsingFixedFunction)
        #if (Servo_PSOC3_ES2 || Servo_PSOC5_ES1)
            Servo_backup.PWMUdb = Servo_ReadCounter();
            Servo_backup.PWMPeriod = Servo_ReadPeriod();
            #if (Servo_UseStatus)
                Servo_backup.InterruptMaskValue = Servo_STATUS_MASK;
            #endif
            
            #if(Servo_UseOneCompareMode)
                Servo_backup.PWMCompareValue = Servo_ReadCompare();
            #else
                Servo_backup.PWMCompareValue1 = Servo_ReadCompare1();
                Servo_backup.PWMCompareValue2 = Servo_ReadCompare2();
            #endif
            
           #if(Servo_DeadBandUsed)
                Servo_backup.PWMdeadBandValue = Servo_ReadDeadTime();
            #endif
          
            #if ( Servo_KillModeMinTime)
                Servo_backup.PWMKillCounterPeriod = Servo_ReadKillTime();
            #endif
        #endif
        
        #if (Servo_PSOC3_ES3 || Servo_PSOC5_ES2)
            #if(!Servo_PWMModeIsCenterAligned)
                Servo_backup.PWMPeriod = Servo_ReadPeriod();
            #endif
            Servo_backup.PWMUdb = Servo_ReadCounter();
            #if (Servo_UseStatus)
                Servo_backup.InterruptMaskValue = Servo_STATUS_MASK;
            #endif
            
            #if(Servo_DeadBandMode == Servo__B_PWM__DBM_256_CLOCKS || Servo_DeadBandMode == Servo__B_PWM__DBM_2_4_CLOCKS)
                Servo_backup.PWMdeadBandValue = Servo_ReadDeadTime();
            #endif
            
            #if(Servo_KillModeMinTime)
                 Servo_backup.PWMKillCounterPeriod = Servo_ReadKillTime();
            #endif
        #endif
        
        #if(Servo_UseControl)
            Servo_backup.PWMControlRegister = Servo_ReadControlRegister();
        #endif
    #endif  
}


/*******************************************************************************
* Function Name: Servo_RestoreConfig
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
*  Servo_backup:  Variables of this global structure are used to restore 
*  the values of non retention registers on wakeup from sleep mode.
*
* Reentrant:
*  Yes.
*
*******************************************************************************/
void Servo_RestoreConfig(void) 
{
        #if(!Servo_UsingFixedFunction)
            #if (Servo_PSOC3_ES2 || Servo_PSOC5_ES1)
                /* Interrupt State Backup for Critical Region*/
                uint8 Servo_interruptState;
                /* Enter Critical Region*/
                Servo_interruptState = CyEnterCriticalSection();
                #if (Servo_UseStatus)
                    /* Use the interrupt output of the status register for IRQ output */
                    Servo_STATUS_AUX_CTRL |= Servo_STATUS_ACTL_INT_EN_MASK;
                    
                    Servo_STATUS_MASK = Servo_backup.InterruptMaskValue;
                #endif
                
                #if (Servo_Resolution == 8)
                    /* Set FIFO 0 to 1 byte register for period*/
                    Servo_AUX_CONTROLDP0 |= (Servo_AUX_CTRL_FIFO0_CLR);
                #else /* (Servo_Resolution == 16)*/
                    /* Set FIFO 0 to 1 byte register for period */
                    Servo_AUX_CONTROLDP0 |= (Servo_AUX_CTRL_FIFO0_CLR);
                    Servo_AUX_CONTROLDP1 |= (Servo_AUX_CTRL_FIFO0_CLR);
                #endif
                /* Exit Critical Region*/
                CyExitCriticalSection(Servo_interruptState);
                
                Servo_WriteCounter(Servo_backup.PWMUdb);
                Servo_WritePeriod(Servo_backup.PWMPeriod);
                
                #if(Servo_UseOneCompareMode)
                    Servo_WriteCompare(Servo_backup.PWMCompareValue);
                #else
                    Servo_WriteCompare1(Servo_backup.PWMCompareValue1);
                    Servo_WriteCompare2(Servo_backup.PWMCompareValue2);
                #endif
                
               #if(Servo_DeadBandMode == Servo__B_PWM__DBM_256_CLOCKS || Servo_DeadBandMode == Servo__B_PWM__DBM_2_4_CLOCKS)
                    Servo_WriteDeadTime(Servo_backup.PWMdeadBandValue);
                #endif
            
                #if ( Servo_KillModeMinTime)
                    Servo_WriteKillTime(Servo_backup.PWMKillCounterPeriod);
                #endif
            #endif
            
            #if (Servo_PSOC3_ES3 || Servo_PSOC5_ES2)
                #if(!Servo_PWMModeIsCenterAligned)
                    Servo_WritePeriod(Servo_backup.PWMPeriod);
                #endif
                Servo_WriteCounter(Servo_backup.PWMUdb);
                #if (Servo_UseStatus)
                    Servo_STATUS_MASK = Servo_backup.InterruptMaskValue;
                #endif
                
                #if(Servo_DeadBandMode == Servo__B_PWM__DBM_256_CLOCKS || Servo_DeadBandMode == Servo__B_PWM__DBM_2_4_CLOCKS)
                    Servo_WriteDeadTime(Servo_backup.PWMdeadBandValue);
                #endif
                
                #if(Servo_KillModeMinTime)
                    Servo_WriteKillTime(Servo_backup.PWMKillCounterPeriod);
                #endif
            #endif
            
            #if(Servo_UseControl)
                Servo_WriteControlRegister(Servo_backup.PWMControlRegister); 
            #endif
        #endif  
    }


/*******************************************************************************
* Function Name: Servo_Sleep
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
*  Servo_backup.PWMEnableState:  Is modified depending on the enable state
*  of the block before entering sleep mode.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Servo_Sleep(void)
{
    #if(Servo_UseControl)
        if(Servo_CTRL_ENABLE == (Servo_CONTROL & Servo_CTRL_ENABLE))
        {
            /*Component is enabled */
            Servo_backup.PWMEnableState = 1u;
        }
        else
        {
            /* Component is disabled */
            Servo_backup.PWMEnableState = 0u;
        }
    #endif
    /* Stop component */
    Servo_Stop();
    
    /* Save registers configuration */
    Servo_SaveConfig();
}


/*******************************************************************************
* Function Name: Servo_Wakeup
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
*  Servo_backup.pwmEnable:  Is used to restore the enable state of 
*  block on wakeup from sleep mode.
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_Wakeup(void) 
{
     /* Restore registers values */
    Servo_RestoreConfig();
    
    if(Servo_backup.PWMEnableState != 0u)
    {
        /* Enable component's operation */
        Servo_Enable();
    } /* Do nothing if component's block was disabled before */
    
}

/* [] END OF FILE */
