/*******************************************************************************
* File Name: Right_Encoder_Pins.c  
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
#include "Right_Encoder_Pins.h"

/* APIs are not generated for P15[7:6] */
#if !(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5 &&\
     CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_5A_PRODUCTION &&\
	 Right_Encoder_Pins__PORT == 15 && (Right_Encoder_Pins__MASK & 0xC0))

/*******************************************************************************
* Function Name: Right_Encoder_Pins_Write
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
void Right_Encoder_Pins_Write(uint8 value) 
{
    uint8 staticBits = Right_Encoder_Pins_DR & ~Right_Encoder_Pins_MASK;
    Right_Encoder_Pins_DR = staticBits | ((value << Right_Encoder_Pins_SHIFT) & Right_Encoder_Pins_MASK);
}


/*******************************************************************************
* Function Name: Right_Encoder_Pins_SetDriveMode
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
void Right_Encoder_Pins_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(Right_Encoder_Pins_0, mode);
	CyPins_SetPinDriveMode(Right_Encoder_Pins_1, mode);
}


/*******************************************************************************
* Function Name: Right_Encoder_Pins_Read
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
*  Macro Right_Encoder_Pins_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Right_Encoder_Pins_Read(void) 
{
    return (Right_Encoder_Pins_PS & Right_Encoder_Pins_MASK) >> Right_Encoder_Pins_SHIFT;
}


/*******************************************************************************
* Function Name: Right_Encoder_Pins_ReadDataReg
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
uint8 Right_Encoder_Pins_ReadDataReg(void) 
{
    return (Right_Encoder_Pins_DR & Right_Encoder_Pins_MASK) >> Right_Encoder_Pins_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Right_Encoder_Pins_INTSTAT) 

    /*******************************************************************************
    * Function Name: Right_Encoder_Pins_ClearInterrupt
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
    uint8 Right_Encoder_Pins_ClearInterrupt(void) 
    {
        return (Right_Encoder_Pins_INTSTAT & Right_Encoder_Pins_MASK) >> Right_Encoder_Pins_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif
/* [] END OF FILE */ 
