/*******************************************************************************
* File Name: Servo0.c  
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
#include "Servo0.h"

/* APIs are not generated for P15[7:6] */
#if !(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5 &&\
     CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_5A_PRODUCTION &&\
	 Servo0__PORT == 15 && (Servo0__MASK & 0xC0))

/*******************************************************************************
* Function Name: Servo0_Write
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
void Servo0_Write(uint8 value) 
{
    uint8 staticBits = Servo0_DR & ~Servo0_MASK;
    Servo0_DR = staticBits | ((value << Servo0_SHIFT) & Servo0_MASK);
}


/*******************************************************************************
* Function Name: Servo0_SetDriveMode
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
void Servo0_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(Servo0_0, mode);
}


/*******************************************************************************
* Function Name: Servo0_Read
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
*  Macro Servo0_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Servo0_Read(void) 
{
    return (Servo0_PS & Servo0_MASK) >> Servo0_SHIFT;
}


/*******************************************************************************
* Function Name: Servo0_ReadDataReg
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
uint8 Servo0_ReadDataReg(void) 
{
    return (Servo0_DR & Servo0_MASK) >> Servo0_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Servo0_INTSTAT) 

    /*******************************************************************************
    * Function Name: Servo0_ClearInterrupt
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
    uint8 Servo0_ClearInterrupt(void) 
    {
        return (Servo0_INTSTAT & Servo0_MASK) >> Servo0_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif
/* [] END OF FILE */ 
