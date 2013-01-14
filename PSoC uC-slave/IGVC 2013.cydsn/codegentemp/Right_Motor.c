/*******************************************************************************
* File Name: Right_Motor.c  
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
#include "Right_Motor.h"

/* APIs are not generated for P15[7:6] */
#if !(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5 &&\
     CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_5A_PRODUCTION &&\
	 Right_Motor__PORT == 15 && (Right_Motor__MASK & 0xC0))

/*******************************************************************************
* Function Name: Right_Motor_Write
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
void Right_Motor_Write(uint8 value) 
{
    uint8 staticBits = Right_Motor_DR & ~Right_Motor_MASK;
    Right_Motor_DR = staticBits | ((value << Right_Motor_SHIFT) & Right_Motor_MASK);
}


/*******************************************************************************
* Function Name: Right_Motor_SetDriveMode
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
void Right_Motor_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(Right_Motor_0, mode);
}


/*******************************************************************************
* Function Name: Right_Motor_Read
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
*  Macro Right_Motor_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Right_Motor_Read(void) 
{
    return (Right_Motor_PS & Right_Motor_MASK) >> Right_Motor_SHIFT;
}


/*******************************************************************************
* Function Name: Right_Motor_ReadDataReg
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
uint8 Right_Motor_ReadDataReg(void) 
{
    return (Right_Motor_DR & Right_Motor_MASK) >> Right_Motor_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Right_Motor_INTSTAT) 

    /*******************************************************************************
    * Function Name: Right_Motor_ClearInterrupt
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
    uint8 Right_Motor_ClearInterrupt(void) 
    {
        return (Right_Motor_INTSTAT & Right_Motor_MASK) >> Right_Motor_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif
/* [] END OF FILE */ 
