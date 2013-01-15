/*******************************************************************************
* File Name: Left_Motor.h  
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

#if !defined(CY_PINS_Left_Motor_H) /* Pins Left_Motor_H */
#define CY_PINS_Left_Motor_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Left_Motor_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5 &&\
     CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_5A_PRODUCTION &&\
	 Left_Motor__PORT == 15 && (Left_Motor__MASK & 0xC0))

/***************************************
*        Function Prototypes             
***************************************/    

void    Left_Motor_Write(uint8 value) ;
void    Left_Motor_SetDriveMode(uint8 mode) ;
uint8   Left_Motor_ReadDataReg(void) ;
uint8   Left_Motor_Read(void) ;
uint8   Left_Motor_ClearInterrupt(void) ;

/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Left_Motor_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Left_Motor_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Left_Motor_DM_RES_UP          PIN_DM_RES_UP
#define Left_Motor_DM_RES_DWN         PIN_DM_RES_DWN
#define Left_Motor_DM_OD_LO           PIN_DM_OD_LO
#define Left_Motor_DM_OD_HI           PIN_DM_OD_HI
#define Left_Motor_DM_STRONG          PIN_DM_STRONG
#define Left_Motor_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Left_Motor_MASK               Left_Motor__MASK
#define Left_Motor_SHIFT              Left_Motor__SHIFT
#define Left_Motor_WIDTH              1u

/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Left_Motor_PS                     (* (reg8 *) Left_Motor__PS)
/* Data Register */
#define Left_Motor_DR                     (* (reg8 *) Left_Motor__DR)
/* Port Number */
#define Left_Motor_PRT_NUM                (* (reg8 *) Left_Motor__PRT) 
/* Connect to Analog Globals */                                                  
#define Left_Motor_AG                     (* (reg8 *) Left_Motor__AG)                       
/* Analog MUX bux enable */
#define Left_Motor_AMUX                   (* (reg8 *) Left_Motor__AMUX) 
/* Bidirectional Enable */                                                        
#define Left_Motor_BIE                    (* (reg8 *) Left_Motor__BIE)
/* Bit-mask for Aliased Register Access */
#define Left_Motor_BIT_MASK               (* (reg8 *) Left_Motor__BIT_MASK)
/* Bypass Enable */
#define Left_Motor_BYP                    (* (reg8 *) Left_Motor__BYP)
/* Port wide control signals */                                                   
#define Left_Motor_CTL                    (* (reg8 *) Left_Motor__CTL)
/* Drive Modes */
#define Left_Motor_DM0                    (* (reg8 *) Left_Motor__DM0) 
#define Left_Motor_DM1                    (* (reg8 *) Left_Motor__DM1)
#define Left_Motor_DM2                    (* (reg8 *) Left_Motor__DM2) 
/* Input Buffer Disable Override */
#define Left_Motor_INP_DIS                (* (reg8 *) Left_Motor__INP_DIS)
/* LCD Common or Segment Drive */
#define Left_Motor_LCD_COM_SEG            (* (reg8 *) Left_Motor__LCD_COM_SEG)
/* Enable Segment LCD */
#define Left_Motor_LCD_EN                 (* (reg8 *) Left_Motor__LCD_EN)
/* Slew Rate Control */
#define Left_Motor_SLW                    (* (reg8 *) Left_Motor__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Left_Motor_PRTDSI__CAPS_SEL       (* (reg8 *) Left_Motor__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Left_Motor_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Left_Motor__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Left_Motor_PRTDSI__OE_SEL0        (* (reg8 *) Left_Motor__PRTDSI__OE_SEL0) 
#define Left_Motor_PRTDSI__OE_SEL1        (* (reg8 *) Left_Motor__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Left_Motor_PRTDSI__OUT_SEL0       (* (reg8 *) Left_Motor__PRTDSI__OUT_SEL0) 
#define Left_Motor_PRTDSI__OUT_SEL1       (* (reg8 *) Left_Motor__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Left_Motor_PRTDSI__SYNC_OUT       (* (reg8 *) Left_Motor__PRTDSI__SYNC_OUT) 


#if defined(Left_Motor__INTSTAT)  /* Interrupt Registers */

    #define Left_Motor_INTSTAT                (* (reg8 *) Left_Motor__INTSTAT)
    #define Left_Motor_SNAP                   (* (reg8 *) Left_Motor__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins Left_Motor_H */

#endif
/* [] END OF FILE */
