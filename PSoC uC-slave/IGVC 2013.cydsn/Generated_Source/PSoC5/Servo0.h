/*******************************************************************************
* File Name: Servo0.h  
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

#if !defined(CY_PINS_Servo0_H) /* Pins Servo0_H */
#define CY_PINS_Servo0_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Servo0_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5 &&\
     CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_5A_PRODUCTION &&\
	 Servo0__PORT == 15 && (Servo0__MASK & 0xC0))

/***************************************
*        Function Prototypes             
***************************************/    

void    Servo0_Write(uint8 value) ;
void    Servo0_SetDriveMode(uint8 mode) ;
uint8   Servo0_ReadDataReg(void) ;
uint8   Servo0_Read(void) ;
uint8   Servo0_ClearInterrupt(void) ;

/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Servo0_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Servo0_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Servo0_DM_RES_UP          PIN_DM_RES_UP
#define Servo0_DM_RES_DWN         PIN_DM_RES_DWN
#define Servo0_DM_OD_LO           PIN_DM_OD_LO
#define Servo0_DM_OD_HI           PIN_DM_OD_HI
#define Servo0_DM_STRONG          PIN_DM_STRONG
#define Servo0_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Servo0_MASK               Servo0__MASK
#define Servo0_SHIFT              Servo0__SHIFT
#define Servo0_WIDTH              1u

/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Servo0_PS                     (* (reg8 *) Servo0__PS)
/* Data Register */
#define Servo0_DR                     (* (reg8 *) Servo0__DR)
/* Port Number */
#define Servo0_PRT_NUM                (* (reg8 *) Servo0__PRT) 
/* Connect to Analog Globals */                                                  
#define Servo0_AG                     (* (reg8 *) Servo0__AG)                       
/* Analog MUX bux enable */
#define Servo0_AMUX                   (* (reg8 *) Servo0__AMUX) 
/* Bidirectional Enable */                                                        
#define Servo0_BIE                    (* (reg8 *) Servo0__BIE)
/* Bit-mask for Aliased Register Access */
#define Servo0_BIT_MASK               (* (reg8 *) Servo0__BIT_MASK)
/* Bypass Enable */
#define Servo0_BYP                    (* (reg8 *) Servo0__BYP)
/* Port wide control signals */                                                   
#define Servo0_CTL                    (* (reg8 *) Servo0__CTL)
/* Drive Modes */
#define Servo0_DM0                    (* (reg8 *) Servo0__DM0) 
#define Servo0_DM1                    (* (reg8 *) Servo0__DM1)
#define Servo0_DM2                    (* (reg8 *) Servo0__DM2) 
/* Input Buffer Disable Override */
#define Servo0_INP_DIS                (* (reg8 *) Servo0__INP_DIS)
/* LCD Common or Segment Drive */
#define Servo0_LCD_COM_SEG            (* (reg8 *) Servo0__LCD_COM_SEG)
/* Enable Segment LCD */
#define Servo0_LCD_EN                 (* (reg8 *) Servo0__LCD_EN)
/* Slew Rate Control */
#define Servo0_SLW                    (* (reg8 *) Servo0__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Servo0_PRTDSI__CAPS_SEL       (* (reg8 *) Servo0__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Servo0_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Servo0__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Servo0_PRTDSI__OE_SEL0        (* (reg8 *) Servo0__PRTDSI__OE_SEL0) 
#define Servo0_PRTDSI__OE_SEL1        (* (reg8 *) Servo0__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Servo0_PRTDSI__OUT_SEL0       (* (reg8 *) Servo0__PRTDSI__OUT_SEL0) 
#define Servo0_PRTDSI__OUT_SEL1       (* (reg8 *) Servo0__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Servo0_PRTDSI__SYNC_OUT       (* (reg8 *) Servo0__PRTDSI__SYNC_OUT) 


#if defined(Servo0__INTSTAT)  /* Interrupt Registers */

    #define Servo0_INTSTAT                (* (reg8 *) Servo0__INTSTAT)
    #define Servo0_SNAP                   (* (reg8 *) Servo0__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins Servo0_H */

#endif
/* [] END OF FILE */
