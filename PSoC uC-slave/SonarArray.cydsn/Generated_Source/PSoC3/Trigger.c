/*******************************************************************************
* File Name: Trigger.c  
* Version 1.60
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2010, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

#include "cytypes.h"
#include "Trigger.h"


/*******************************************************************************
* Function Name: Trigger_Write
********************************************************************************
* Summary:
*  Assign a new value to the digital port's data output register.  
*
* Parameters:  
*  prtValue:  The value to be assigned to the Digital Port. 
*
* Return: 
*  void 
*  
*******************************************************************************/
void Trigger_Write(uint8 value) 
{
    uint8 staticBits = Trigger_DR & ~Trigger_MASK;
    Trigger_DR = staticBits | ((value << Trigger_SHIFT) & Trigger_MASK);
}


/*******************************************************************************
* Function Name: Trigger_SetDriveMode
********************************************************************************
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to this drive mode.
*
* Return: 
*  void
*
*******************************************************************************/
void Trigger_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(Trigger_0, mode);
}


/*******************************************************************************
* Function Name: Trigger_Read
********************************************************************************
* Summary:
*  Read the current value on the pins of the Digital Port in right justified 
*  form.
*
* Parameters:  
*  void 
*
* Return: 
*  Returns the current value of the Digital Port as a right justified number
*  
* Note:
*  Macro Trigger_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Trigger_Read(void) 
{
    return (Trigger_PS & Trigger_MASK) >> Trigger_SHIFT;
}


/*******************************************************************************
* Function Name: Trigger_ReadDataReg
********************************************************************************
* Summary:
*  Read the current value assigned to a Digital Port's data output register
*
* Parameters:  
*  void 
*
* Return: 
*  Returns the current value assigned to the Digital Port's data output register
*  
*******************************************************************************/
uint8 Trigger_ReadDataReg(void) 
{
    return (Trigger_DR & Trigger_MASK) >> Trigger_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Trigger_INTSTAT) 

    /*******************************************************************************
    * Function Name: Trigger_ClearInterrupt
    ********************************************************************************
    * Summary:
    *  Clears any active interrupts attached to port and returns the value of the 
    *  interrupt status register.
    *
    * Parameters:  
    *  void 
    *
    * Return: 
    *  Returns the value of the interrupt status register
    *  
    *******************************************************************************/
    uint8 Trigger_ClearInterrupt(void) 
    {
        return (Trigger_INTSTAT & Trigger_MASK) >> Trigger_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */ 
