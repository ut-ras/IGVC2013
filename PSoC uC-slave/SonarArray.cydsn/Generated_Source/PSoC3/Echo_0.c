/*******************************************************************************
* File Name: Echo_0.c  
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
#include "Echo_0.h"


/*******************************************************************************
* Function Name: Echo_0_Write
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
void Echo_0_Write(uint8 value) 
{
    uint8 staticBits = Echo_0_DR & ~Echo_0_MASK;
    Echo_0_DR = staticBits | ((value << Echo_0_SHIFT) & Echo_0_MASK);
}


/*******************************************************************************
* Function Name: Echo_0_SetDriveMode
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
void Echo_0_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(Echo_0_0, mode);
	CyPins_SetPinDriveMode(Echo_0_1, mode);
	CyPins_SetPinDriveMode(Echo_0_2, mode);
	CyPins_SetPinDriveMode(Echo_0_3, mode);
	CyPins_SetPinDriveMode(Echo_0_4, mode);
	CyPins_SetPinDriveMode(Echo_0_5, mode);
}


/*******************************************************************************
* Function Name: Echo_0_Read
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
*  Macro Echo_0_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Echo_0_Read(void) 
{
    return (Echo_0_PS & Echo_0_MASK) >> Echo_0_SHIFT;
}


/*******************************************************************************
* Function Name: Echo_0_ReadDataReg
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
uint8 Echo_0_ReadDataReg(void) 
{
    return (Echo_0_DR & Echo_0_MASK) >> Echo_0_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Echo_0_INTSTAT) 

    /*******************************************************************************
    * Function Name: Echo_0_ClearInterrupt
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
    uint8 Echo_0_ClearInterrupt(void) 
    {
        return (Echo_0_INTSTAT & Echo_0_MASK) >> Echo_0_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */ 
