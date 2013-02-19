/*******************************************************************************
* File Name: Servo_Out.c  
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
#include "Servo_Out.h"

/* APIs are not generated for P15[7:6] */
#if !(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5 &&\
     CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_5A_PRODUCTION &&\
	 Servo_Out__PORT == 15 && (Servo_Out__MASK & 0xC0))

/*******************************************************************************
* Function Name: Servo_Out_Write
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
void Servo_Out_Write(uint8 value) 
{
    uint8 staticBits = Servo_Out_DR & ~Servo_Out_MASK;
    Servo_Out_DR = staticBits | ((value << Servo_Out_SHIFT) & Servo_Out_MASK);
}


/*******************************************************************************
* Function Name: Servo_Out_SetDriveMode
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
void Servo_Out_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(Servo_Out_0, mode);
}


/*******************************************************************************
* Function Name: Servo_Out_Read
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
*  Macro Servo_Out_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Servo_Out_Read(void) 
{
    return (Servo_Out_PS & Servo_Out_MASK) >> Servo_Out_SHIFT;
}


/*******************************************************************************
* Function Name: Servo_Out_ReadDataReg
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
uint8 Servo_Out_ReadDataReg(void) 
{
    return (Servo_Out_DR & Servo_Out_MASK) >> Servo_Out_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Servo_Out_INTSTAT) 

    /*******************************************************************************
    * Function Name: Servo_Out_ClearInterrupt
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
    uint8 Servo_Out_ClearInterrupt(void) 
    {
        return (Servo_Out_INTSTAT & Servo_Out_MASK) >> Servo_Out_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif
/* [] END OF FILE */ 
