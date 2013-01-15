/*******************************************************************************
* File Name: Soft_Kill.c  
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
#include "Soft_Kill.h"

/* APIs are not generated for P15[7:6] */
#if !(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5 &&\
     CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_5A_PRODUCTION &&\
	 Soft_Kill__PORT == 15 && (Soft_Kill__MASK & 0xC0))

/*******************************************************************************
* Function Name: Soft_Kill_Write
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
void Soft_Kill_Write(uint8 value) 
{
    uint8 staticBits = Soft_Kill_DR & ~Soft_Kill_MASK;
    Soft_Kill_DR = staticBits | ((value << Soft_Kill_SHIFT) & Soft_Kill_MASK);
}


/*******************************************************************************
* Function Name: Soft_Kill_SetDriveMode
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
void Soft_Kill_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(Soft_Kill_0, mode);
}


/*******************************************************************************
* Function Name: Soft_Kill_Read
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
*  Macro Soft_Kill_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Soft_Kill_Read(void) 
{
    return (Soft_Kill_PS & Soft_Kill_MASK) >> Soft_Kill_SHIFT;
}


/*******************************************************************************
* Function Name: Soft_Kill_ReadDataReg
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
uint8 Soft_Kill_ReadDataReg(void) 
{
    return (Soft_Kill_DR & Soft_Kill_MASK) >> Soft_Kill_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Soft_Kill_INTSTAT) 

    /*******************************************************************************
    * Function Name: Soft_Kill_ClearInterrupt
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
    uint8 Soft_Kill_ClearInterrupt(void) 
    {
        return (Soft_Kill_INTSTAT & Soft_Kill_MASK) >> Soft_Kill_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif
/* [] END OF FILE */ 
