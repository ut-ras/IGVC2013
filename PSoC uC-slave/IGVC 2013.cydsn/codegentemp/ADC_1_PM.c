/*******************************************************************************
* File Name: ADC_1_PM.c
* Version 1.71
*
* Description:
*  This file provides Sleep/WakeUp APIs functionality.
*
* Note:
*
*******************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

#include "ADC_1.h"


/***************************************
* Local data allocation
***************************************/

static ADC_1_BACKUP_STRUCT  ADC_1_backup =
{
    ADC_1_DISABLED
};
    

/*******************************************************************************
* Function Name: ADC_1_SaveConfig
********************************************************************************
*
* Summary:
*  Saves the current user configuration.
*  
* Parameters:  
*  None.
*
* Return: 
*  None.
*
*******************************************************************************/
void ADC_1_SaveConfig(void)
{
    /* All configuration registers are marked as [reset_all_retention] */
}


/*******************************************************************************
* Function Name: ADC_1_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*  
* Parameters:  
*  None.
*
* Return: 
*  None.
*
*******************************************************************************/
void ADC_1_RestoreConfig(void)
{
    /* All congiguration registers are marked as [reset_all_retention] */
}


/*******************************************************************************
* Function Name: ADC_1_Sleep
********************************************************************************
*
* Summary:
*  Stops and saves the user configuration
*  
* Parameters:  
*  None.
*
* Return: 
*  None.
*
* Global Variables:
*  ADC_1_backup - modified.
*
*******************************************************************************/
void ADC_1_Sleep(void)
{
    if((ADC_1_PWRMGR_SAR_REG  & ADC_1_ACT_PWR_SAR_EN) != 0u) 
    {
        ADC_1_backup.enableState = ADC_1_ENABLED;
        if((ADC_1_SAR_CSR0_REG & ADC_1_SAR_SOF_START_CONV) != 0u)
        {
            ADC_1_backup.enableState |= ADC_1_STARTED;
        }
        ADC_1_Stop();
    }
    else
    {
        ADC_1_backup.enableState = ADC_1_DISABLED;
    }
    /*ADC_1_SaveConfig();*/
}


/*******************************************************************************
* Function Name: ADC_1_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration
*  
* Parameters:  
*  None.
*
* Return: 
*  None.
*
* Global Variables:
*  ADC_1_backup - used. 
*
*******************************************************************************/
void ADC_1_Wakeup(void)
{
    /*ADC_1_RestoreConfig();*/
    if(ADC_1_backup.enableState != ADC_1_DISABLED)
    {
        ADC_1_Enable();
        if((ADC_1_backup.enableState & ADC_1_STARTED) != 0u)
        {
            ADC_1_StartConvert();
        }
    } 
}


/* [] END OF FILE */
