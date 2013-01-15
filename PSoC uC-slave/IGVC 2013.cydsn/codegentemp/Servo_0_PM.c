/*******************************************************************************
* File Name: Servo_0_PM.c
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
#include "Servo_0.h"

static Servo_0_backupStruct Servo_0_backup;


/*******************************************************************************
* Function Name: Servo_0_SaveConfig
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
*  Servo_0_backup:  Variables of this global structure are modified to 
*  store the values of non retention configuration registers when Sleep() API is 
*  called.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Servo_0_SaveConfig(void)
{
    
    #if(!Servo_0_UsingFixedFunction)
        #if (Servo_0_PSOC3_ES2 || Servo_0_PSOC5_ES1)
            Servo_0_backup.PWMUdb = Servo_0_ReadCounter();
            Servo_0_backup.PWMPeriod = Servo_0_ReadPeriod();
            #if (Servo_0_UseStatus)
                Servo_0_backup.InterruptMaskValue = Servo_0_STATUS_MASK;
            #endif
            
            #if(Servo_0_UseOneCompareMode)
                Servo_0_backup.PWMCompareValue = Servo_0_ReadCompare();
            #else
                Servo_0_backup.PWMCompareValue1 = Servo_0_ReadCompare1();
                Servo_0_backup.PWMCompareValue2 = Servo_0_ReadCompare2();
            #endif
            
           #if(Servo_0_DeadBandUsed)
                Servo_0_backup.PWMdeadBandValue = Servo_0_ReadDeadTime();
            #endif
          
            #if ( Servo_0_KillModeMinTime)
                Servo_0_backup.PWMKillCounterPeriod = Servo_0_ReadKillTime();
            #endif
        #endif
        
        #if (Servo_0_PSOC3_ES3 || Servo_0_PSOC5_ES2)
            #if(!Servo_0_PWMModeIsCenterAligned)
                Servo_0_backup.PWMPeriod = Servo_0_ReadPeriod();
            #endif
            Servo_0_backup.PWMUdb = Servo_0_ReadCounter();
            #if (Servo_0_UseStatus)
                Servo_0_backup.InterruptMaskValue = Servo_0_STATUS_MASK;
            #endif
            
            #if(Servo_0_DeadBandMode == Servo_0__B_PWM__DBM_256_CLOCKS || Servo_0_DeadBandMode == Servo_0__B_PWM__DBM_2_4_CLOCKS)
                Servo_0_backup.PWMdeadBandValue = Servo_0_ReadDeadTime();
            #endif
            
            #if(Servo_0_KillModeMinTime)
                 Servo_0_backup.PWMKillCounterPeriod = Servo_0_ReadKillTime();
            #endif
        #endif
        
        #if(Servo_0_UseControl)
            Servo_0_backup.PWMControlRegister = Servo_0_ReadControlRegister();
        #endif
    #endif  
}


/*******************************************************************************
* Function Name: Servo_0_RestoreConfig
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
*  Servo_0_backup:  Variables of this global structure are used to restore 
*  the values of non retention registers on wakeup from sleep mode.
*
* Reentrant:
*  Yes.
*
*******************************************************************************/
void Servo_0_RestoreConfig(void) 
{
        #if(!Servo_0_UsingFixedFunction)
            #if (Servo_0_PSOC3_ES2 || Servo_0_PSOC5_ES1)
                /* Interrupt State Backup for Critical Region*/
                uint8 Servo_0_interruptState;
                /* Enter Critical Region*/
                Servo_0_interruptState = CyEnterCriticalSection();
                #if (Servo_0_UseStatus)
                    /* Use the interrupt output of the status register for IRQ output */
                    Servo_0_STATUS_AUX_CTRL |= Servo_0_STATUS_ACTL_INT_EN_MASK;
                    
                    Servo_0_STATUS_MASK = Servo_0_backup.InterruptMaskValue;
                #endif
                
                #if (Servo_0_Resolution == 8)
                    /* Set FIFO 0 to 1 byte register for period*/
                    Servo_0_AUX_CONTROLDP0 |= (Servo_0_AUX_CTRL_FIFO0_CLR);
                #else /* (Servo_0_Resolution == 16)*/
                    /* Set FIFO 0 to 1 byte register for period */
                    Servo_0_AUX_CONTROLDP0 |= (Servo_0_AUX_CTRL_FIFO0_CLR);
                    Servo_0_AUX_CONTROLDP1 |= (Servo_0_AUX_CTRL_FIFO0_CLR);
                #endif
                /* Exit Critical Region*/
                CyExitCriticalSection(Servo_0_interruptState);
                
                Servo_0_WriteCounter(Servo_0_backup.PWMUdb);
                Servo_0_WritePeriod(Servo_0_backup.PWMPeriod);
                
                #if(Servo_0_UseOneCompareMode)
                    Servo_0_WriteCompare(Servo_0_backup.PWMCompareValue);
                #else
                    Servo_0_WriteCompare1(Servo_0_backup.PWMCompareValue1);
                    Servo_0_WriteCompare2(Servo_0_backup.PWMCompareValue2);
                #endif
                
               #if(Servo_0_DeadBandMode == Servo_0__B_PWM__DBM_256_CLOCKS || Servo_0_DeadBandMode == Servo_0__B_PWM__DBM_2_4_CLOCKS)
                    Servo_0_WriteDeadTime(Servo_0_backup.PWMdeadBandValue);
                #endif
            
                #if ( Servo_0_KillModeMinTime)
                    Servo_0_WriteKillTime(Servo_0_backup.PWMKillCounterPeriod);
                #endif
            #endif
            
            #if (Servo_0_PSOC3_ES3 || Servo_0_PSOC5_ES2)
                #if(!Servo_0_PWMModeIsCenterAligned)
                    Servo_0_WritePeriod(Servo_0_backup.PWMPeriod);
                #endif
                Servo_0_WriteCounter(Servo_0_backup.PWMUdb);
                #if (Servo_0_UseStatus)
                    Servo_0_STATUS_MASK = Servo_0_backup.InterruptMaskValue;
                #endif
                
                #if(Servo_0_DeadBandMode == Servo_0__B_PWM__DBM_256_CLOCKS || Servo_0_DeadBandMode == Servo_0__B_PWM__DBM_2_4_CLOCKS)
                    Servo_0_WriteDeadTime(Servo_0_backup.PWMdeadBandValue);
                #endif
                
                #if(Servo_0_KillModeMinTime)
                    Servo_0_WriteKillTime(Servo_0_backup.PWMKillCounterPeriod);
                #endif
            #endif
            
            #if(Servo_0_UseControl)
                Servo_0_WriteControlRegister(Servo_0_backup.PWMControlRegister); 
            #endif
        #endif  
    }


/*******************************************************************************
* Function Name: Servo_0_Sleep
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
*  Servo_0_backup.PWMEnableState:  Is modified depending on the enable state
*  of the block before entering sleep mode.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Servo_0_Sleep(void)
{
    #if(Servo_0_UseControl)
        if(Servo_0_CTRL_ENABLE == (Servo_0_CONTROL & Servo_0_CTRL_ENABLE))
        {
            /*Component is enabled */
            Servo_0_backup.PWMEnableState = 1u;
        }
        else
        {
            /* Component is disabled */
            Servo_0_backup.PWMEnableState = 0u;
        }
    #endif
    /* Stop component */
    Servo_0_Stop();
    
    /* Save registers configuration */
    Servo_0_SaveConfig();
}


/*******************************************************************************
* Function Name: Servo_0_Wakeup
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
*  Servo_0_backup.pwmEnable:  Is used to restore the enable state of 
*  block on wakeup from sleep mode.
*
* Reentrant:
*  Yes
*
*******************************************************************************/
void Servo_0_Wakeup(void) 
{
     /* Restore registers values */
    Servo_0_RestoreConfig();
    
    if(Servo_0_backup.PWMEnableState != 0u)
    {
        /* Enable component's operation */
        Servo_0_Enable();
    } /* Do nothing if component's block was disabled before */
    
}

/* [] END OF FILE */
