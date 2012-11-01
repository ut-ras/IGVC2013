/*******************************************************************************
* File Name: Right_Encoder_Pins.h  
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

#if !defined(CY_PINS_Right_Encoder_Pins_H) /* Pins Right_Encoder_Pins_H */
#define CY_PINS_Right_Encoder_Pins_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Right_Encoder_Pins_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5 &&\
     CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_5A_PRODUCTION &&\
	 Right_Encoder_Pins__PORT == 15 && (Right_Encoder_Pins__MASK & 0xC0))

/***************************************
*        Function Prototypes             
***************************************/    

void    Right_Encoder_Pins_Write(uint8 value) ;
void    Right_Encoder_Pins_SetDriveMode(uint8 mode) ;
uint8   Right_Encoder_Pins_ReadDataReg(void) ;
uint8   Right_Encoder_Pins_Read(void) ;
uint8   Right_Encoder_Pins_ClearInterrupt(void) ;

/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Right_Encoder_Pins_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Right_Encoder_Pins_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Right_Encoder_Pins_DM_RES_UP          PIN_DM_RES_UP
#define Right_Encoder_Pins_DM_RES_DWN         PIN_DM_RES_DWN
#define Right_Encoder_Pins_DM_OD_LO           PIN_DM_OD_LO
#define Right_Encoder_Pins_DM_OD_HI           PIN_DM_OD_HI
#define Right_Encoder_Pins_DM_STRONG          PIN_DM_STRONG
#define Right_Encoder_Pins_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Right_Encoder_Pins_MASK               Right_Encoder_Pins__MASK
#define Right_Encoder_Pins_SHIFT              Right_Encoder_Pins__SHIFT
#define Right_Encoder_Pins_WIDTH              2u

/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Right_Encoder_Pins_PS                     (* (reg8 *) Right_Encoder_Pins__PS)
/* Data Register */
#define Right_Encoder_Pins_DR                     (* (reg8 *) Right_Encoder_Pins__DR)
/* Port Number */
#define Right_Encoder_Pins_PRT_NUM                (* (reg8 *) Right_Encoder_Pins__PRT) 
/* Connect to Analog Globals */                                                  
#define Right_Encoder_Pins_AG                     (* (reg8 *) Right_Encoder_Pins__AG)                       
/* Analog MUX bux enable */
#define Right_Encoder_Pins_AMUX                   (* (reg8 *) Right_Encoder_Pins__AMUX) 
/* Bidirectional Enable */                                                        
#define Right_Encoder_Pins_BIE                    (* (reg8 *) Right_Encoder_Pins__BIE)
/* Bit-mask for Aliased Register Access */
#define Right_Encoder_Pins_BIT_MASK               (* (reg8 *) Right_Encoder_Pins__BIT_MASK)
/* Bypass Enable */
#define Right_Encoder_Pins_BYP                    (* (reg8 *) Right_Encoder_Pins__BYP)
/* Port wide control signals */                                                   
#define Right_Encoder_Pins_CTL                    (* (reg8 *) Right_Encoder_Pins__CTL)
/* Drive Modes */
#define Right_Encoder_Pins_DM0                    (* (reg8 *) Right_Encoder_Pins__DM0) 
#define Right_Encoder_Pins_DM1                    (* (reg8 *) Right_Encoder_Pins__DM1)
#define Right_Encoder_Pins_DM2                    (* (reg8 *) Right_Encoder_Pins__DM2) 
/* Input Buffer Disable Override */
#define Right_Encoder_Pins_INP_DIS                (* (reg8 *) Right_Encoder_Pins__INP_DIS)
/* LCD Common or Segment Drive */
#define Right_Encoder_Pins_LCD_COM_SEG            (* (reg8 *) Right_Encoder_Pins__LCD_COM_SEG)
/* Enable Segment LCD */
#define Right_Encoder_Pins_LCD_EN                 (* (reg8 *) Right_Encoder_Pins__LCD_EN)
/* Slew Rate Control */
#define Right_Encoder_Pins_SLW                    (* (reg8 *) Right_Encoder_Pins__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Right_Encoder_Pins_PRTDSI__CAPS_SEL       (* (reg8 *) Right_Encoder_Pins__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Right_Encoder_Pins_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Right_Encoder_Pins__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Right_Encoder_Pins_PRTDSI__OE_SEL0        (* (reg8 *) Right_Encoder_Pins__PRTDSI__OE_SEL0) 
#define Right_Encoder_Pins_PRTDSI__OE_SEL1        (* (reg8 *) Right_Encoder_Pins__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Right_Encoder_Pins_PRTDSI__OUT_SEL0       (* (reg8 *) Right_Encoder_Pins__PRTDSI__OUT_SEL0) 
#define Right_Encoder_Pins_PRTDSI__OUT_SEL1       (* (reg8 *) Right_Encoder_Pins__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Right_Encoder_Pins_PRTDSI__SYNC_OUT       (* (reg8 *) Right_Encoder_Pins__PRTDSI__SYNC_OUT) 


#if defined(Right_Encoder_Pins__INTSTAT)  /* Interrupt Registers */

    #define Right_Encoder_Pins_INTSTAT                (* (reg8 *) Right_Encoder_Pins__INTSTAT)
    #define Right_Encoder_Pins_SNAP                   (* (reg8 *) Right_Encoder_Pins__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins Right_Encoder_Pins_H */

#endif
/* [] END OF FILE */
