/*******************************************************************************
* File Name: Left_Motor.c  
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
#include "Left_Motor.h"

/* APIs are not generated for P15[7:6] */
#if !(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5 &&\
     CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_5A_PRODUCTION &&\
	 Left_Motor__PORT == 15 && (Left_Motor__MASK & 0xC0))

/*******************************************************************************
* Function Name: Left_Motor_Write
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
void Left_Motor_Write(uint8 value) 
{
    uint8 staticBits = Left_Motor_DR & ~Left_Motor_MASK;
    Left_Motor_DR = staticBits | ((value << Left_Motor_SHIFT) & Left_Motor_MASK);
}


/*******************************************************************************
* Function Name: Left_Motor_SetDriveMode
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
void Left_Motor_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(Left_Motor_0, mode);
}


/*******************************************************************************
* Function Name: Left_Motor_Read
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
*  Macro Left_Motor_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Left_Motor_Read(void) 
{
    return (Left_Motor_PS & Left_Motor_MASK) >> Left_Motor_SHIFT;
}


/*******************************************************************************
* Function Name: Left_Motor_ReadDataReg
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
uint8 Left_Motor_ReadDataReg(void) 
{
    return (Left_Motor_DR & Left_Motor_MASK) >> Left_Motor_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Left_Motor_INTSTAT) 

    /*******************************************************************************
    * Function Name: Left_Motor_ClearInterrupt
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
    uint8 Left_Motor_ClearInterrupt(void) 
    {
        return (Left_Motor_INTSTAT & Left_Motor_MASK) >> Left_Motor_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif
/* [] END OF FILE */ 
