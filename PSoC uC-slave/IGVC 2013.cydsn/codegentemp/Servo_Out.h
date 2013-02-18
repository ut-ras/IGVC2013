/*******************************************************************************
* File Name: Servo_Out.h  
* Version 1.60
*
* Description:
*  This file containts Control Register function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2010, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

#if !defined(CY_PINS_Servo_Out_H) /* Pins Servo_Out_H */
#define CY_PINS_Servo_Out_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Servo_Out_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5 &&\
     CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_5A_PRODUCTION &&\
	 Servo_Out__PORT == 15 && (Servo_Out__MASK & 0xC0))

/***************************************
*        Function Prototypes             
***************************************/    

void    Servo_Out_Write(uint8 value) ;
void    Servo_Out_SetDriveMode(uint8 mode) ;
uint8   Servo_Out_ReadDataReg(void) ;
uint8   Servo_Out_Read(void) ;
uint8   Servo_Out_ClearInterrupt(void) ;

/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Servo_Out_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Servo_Out_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Servo_Out_DM_RES_UP          PIN_DM_RES_UP
#define Servo_Out_DM_RES_DWN         PIN_DM_RES_DWN
#define Servo_Out_DM_OD_LO           PIN_DM_OD_LO
#define Servo_Out_DM_OD_HI           PIN_DM_OD_HI
#define Servo_Out_DM_STRONG          PIN_DM_STRONG
#define Servo_Out_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Servo_Out_MASK               Servo_Out__MASK
#define Servo_Out_SHIFT              Servo_Out__SHIFT
#define Servo_Out_WIDTH              1u

/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Servo_Out_PS                     (* (reg8 *) Servo_Out__PS)
/* Data Register */
#define Servo_Out_DR                     (* (reg8 *) Servo_Out__DR)
/* Port Number */
#define Servo_Out_PRT_NUM                (* (reg8 *) Servo_Out__PRT) 
/* Connect to Analog Globals */                                                  
#define Servo_Out_AG                     (* (reg8 *) Servo_Out__AG)                       
/* Analog MUX bux enable */
#define Servo_Out_AMUX                   (* (reg8 *) Servo_Out__AMUX) 
/* Bidirectional Enable */                                                        
#define Servo_Out_BIE                    (* (reg8 *) Servo_Out__BIE)
/* Bit-mask for Aliased Register Access */
#define Servo_Out_BIT_MASK               (* (reg8 *) Servo_Out__BIT_MASK)
/* Bypass Enable */
#define Servo_Out_BYP                    (* (reg8 *) Servo_Out__BYP)
/* Port wide control signals */                                                   
#define Servo_Out_CTL                    (* (reg8 *) Servo_Out__CTL)
/* Drive Modes */
#define Servo_Out_DM0                    (* (reg8 *) Servo_Out__DM0) 
#define Servo_Out_DM1                    (* (reg8 *) Servo_Out__DM1)
#define Servo_Out_DM2                    (* (reg8 *) Servo_Out__DM2) 
/* Input Buffer Disable Override */
#define Servo_Out_INP_DIS                (* (reg8 *) Servo_Out__INP_DIS)
/* LCD Common or Segment Drive */
#define Servo_Out_LCD_COM_SEG            (* (reg8 *) Servo_Out__LCD_COM_SEG)
/* Enable Segment LCD */
#define Servo_Out_LCD_EN                 (* (reg8 *) Servo_Out__LCD_EN)
/* Slew Rate Control */
#define Servo_Out_SLW                    (* (reg8 *) Servo_Out__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Servo_Out_PRTDSI__CAPS_SEL       (* (reg8 *) Servo_Out__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Servo_Out_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Servo_Out__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Servo_Out_PRTDSI__OE_SEL0        (* (reg8 *) Servo_Out__PRTDSI__OE_SEL0) 
#define Servo_Out_PRTDSI__OE_SEL1        (* (reg8 *) Servo_Out__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Servo_Out_PRTDSI__OUT_SEL0       (* (reg8 *) Servo_Out__PRTDSI__OUT_SEL0) 
#define Servo_Out_PRTDSI__OUT_SEL1       (* (reg8 *) Servo_Out__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Servo_Out_PRTDSI__SYNC_OUT       (* (reg8 *) Servo_Out__PRTDSI__SYNC_OUT) 


#if defined(Servo_Out__INTSTAT)  /* Interrupt Registers */

    #define Servo_Out_INTSTAT                (* (reg8 *) Servo_Out__INTSTAT)
    #define Servo_Out_SNAP                   (* (reg8 *) Servo_Out__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins Servo_Out_H */

#endif
/* [] END OF FILE */
