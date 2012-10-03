/*******************************************************************************
* File Name: Opamp_3.c
* Version 1.70
*
* Description:
*  This file provides the source code to the API for OpAmp (Analog Buffer) 
*  Component.
*
* Note:
*
*******************************************************************************
* Copyright 2008-2011, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "Opamp_3.h"
#include <CyLib.h>

uint8 Opamp_3_initVar = 0u;

/* Check to see if required defines such as CY_PSOC3 and CY_PSOC5 are available */
/* They are defined starting with cy_boot v2.30 */
#ifndef CY_PSOC3
#error Component OpAmp_v1_70 requires cy_boot v2.30 or later
#endif


/*******************************************************************************   
* Function Name: Opamp_3_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the 
*  customizer of the component placed onto schematic. Usually called in 
*  Opamp_3_Start().
*
* Parameters:
*  void
*
* Return:
*  void
*
* Reentrant:
*  Yes
* 
*******************************************************************************/
void Opamp_3_Init(void) 
{
    Opamp_3_SetPower(Opamp_3_DEFAULT_POWER);
}


/*******************************************************************************   
* Function Name: Opamp_3_Enable
********************************************************************************
*
* Summary:
*  Enables the OpAmp block operation
*
* Parameters:
*  void
*
* Return:
*  void
*
* Reentrant:
*  Yes
* 
*******************************************************************************/
void Opamp_3_Enable(void) 
{
    /* Enable negative charge pumps in ANIF */
    Opamp_3_PUMP_CR1_REG  |= (Opamp_3_PUMP_CR1_CLKSEL | Opamp_3_PUMP_CR1_FORCE);
    
    /* Enable power to buffer in active mode */
    Opamp_3_PM_ACT_CFG_REG |= Opamp_3_ACT_PWR_EN;
    
    /* Enable power to buffer in alternative active mode */
    Opamp_3_PM_STBY_CFG_REG |= Opamp_3_STBY_PWR_EN;
}


/*******************************************************************************
* Function Name:   Opamp_3_Start
********************************************************************************
*
* Summary:
*  The start function initializes the Analog Buffer with the default values, and 
*  sets the power to the given level.  A power level of 0, is the same as 
*  executing the stop function.
*
* Parameters:
*  void
*
* Return:  
*  void
*
* Global variables:
*  Opamp_3_initVar:  Used to check the initial configuration, modified 
*  when this function is called for the first time.
*
* Reentrant:
*  No
*
*******************************************************************************/
void Opamp_3_Start(void) 
{
    if(Opamp_3_initVar == 0u)
    {
        Opamp_3_initVar = 1u;
        Opamp_3_Init();
    }
    
    Opamp_3_Enable();
}


/*******************************************************************************
* Function Name: Opamp_3_Stop
********************************************************************************
*
* Summary:
*  Powers down amplifier to lowest power state.
*
* Parameters:  
*  void
*
* Return:  
*  void
*
* Reentrant: 
*  Yes
*
*******************************************************************************/
void Opamp_3_Stop(void) 
{
    /* Disable negative charge pumps for ANIF only if the one ABuf is turned ON */
    if(Opamp_3_PM_ACT_CFG_REG == Opamp_3_ACT_PWR_EN)
    {
        Opamp_3_PUMP_CR1_REG &= ~(Opamp_3_PUMP_CR1_CLKSEL | Opamp_3_PUMP_CR1_FORCE);
    }
    
    /* Disable power to buffer in active mode template */
    Opamp_3_PM_ACT_CFG_REG &= ~Opamp_3_ACT_PWR_EN;

    /* Disable power to buffer in alternative active mode template */
    Opamp_3_PM_STBY_CFG_REG &= ~Opamp_3_STBY_PWR_EN;
}


/*******************************************************************************
* Funciton Name:   Opamp_3_SetPower
********************************************************************************
*
* Summary:
*  Sets power level of Analog buffer.
*
* Parameters: 
*  power:  PSoC3: Sets power level between low (1) and high power (3).
*          PSoC5: Sets power level High (0)
*
* Return:  
*  void
*
* Reentrant:
*  Yes
*
**********************************************************************************/
void Opamp_3_SetPower(uint8 power) 
{
    /* Only High power can be used in PSoC5 */
    #if CY_PSOC5
        CYASSERT(power == Opamp_3_HIGHPOWER);
    #endif
    
    Opamp_3_CR_REG = ((Opamp_3_CR_REG & ~Opamp_3_PWR_MASK) | 
                               ( power & Opamp_3_PWR_MASK));   /* Set device power */
}


/* [] END OF FILE */
