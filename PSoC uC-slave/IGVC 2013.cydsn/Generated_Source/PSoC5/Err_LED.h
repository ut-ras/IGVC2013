/*******************************************************************************
* File Name: Err_LED.h  
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

#if !defined(CY_PINS_Err_LED_H) /* Pins Err_LED_H */
#define CY_PINS_Err_LED_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Err_LED_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5 &&\
     CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_5A_PRODUCTION &&\
	 Err_LED__PORT == 15 && (Err_LED__MASK & 0xC0))

/***************************************
*        Function Prototypes             
***************************************/    

void    Err_LED_Write(uint8 value) ;
void    Err_LED_SetDriveMode(uint8 mode) ;
uint8   Err_LED_ReadDataReg(void) ;
uint8   Err_LED_Read(void) ;
uint8   Err_LED_ClearInterrupt(void) ;

/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Err_LED_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Err_LED_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Err_LED_DM_RES_UP          PIN_DM_RES_UP
#define Err_LED_DM_RES_DWN         PIN_DM_RES_DWN
#define Err_LED_DM_OD_LO           PIN_DM_OD_LO
#define Err_LED_DM_OD_HI           PIN_DM_OD_HI
#define Err_LED_DM_STRONG          PIN_DM_STRONG
#define Err_LED_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Err_LED_MASK               Err_LED__MASK
#define Err_LED_SHIFT              Err_LED__SHIFT
#define Err_LED_WIDTH              1u

/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Err_LED_PS                     (* (reg8 *) Err_LED__PS)
/* Data Register */
#define Err_LED_DR                     (* (reg8 *) Err_LED__DR)
/* Port Number */
#define Err_LED_PRT_NUM                (* (reg8 *) Err_LED__PRT) 
/* Connect to Analog Globals */                                                  
#define Err_LED_AG                     (* (reg8 *) Err_LED__AG)                       
/* Analog MUX bux enable */
#define Err_LED_AMUX                   (* (reg8 *) Err_LED__AMUX) 
/* Bidirectional Enable */                                                        
#define Err_LED_BIE                    (* (reg8 *) Err_LED__BIE)
/* Bit-mask for Aliased Register Access */
#define Err_LED_BIT_MASK               (* (reg8 *) Err_LED__BIT_MASK)
/* Bypass Enable */
#define Err_LED_BYP                    (* (reg8 *) Err_LED__BYP)
/* Port wide control signals */                                                   
#define Err_LED_CTL                    (* (reg8 *) Err_LED__CTL)
/* Drive Modes */
#define Err_LED_DM0                    (* (reg8 *) Err_LED__DM0) 
#define Err_LED_DM1                    (* (reg8 *) Err_LED__DM1)
#define Err_LED_DM2                    (* (reg8 *) Err_LED__DM2) 
/* Input Buffer Disable Override */
#define Err_LED_INP_DIS                (* (reg8 *) Err_LED__INP_DIS)
/* LCD Common or Segment Drive */
#define Err_LED_LCD_COM_SEG            (* (reg8 *) Err_LED__LCD_COM_SEG)
/* Enable Segment LCD */
#define Err_LED_LCD_EN                 (* (reg8 *) Err_LED__LCD_EN)
/* Slew Rate Control */
#define Err_LED_SLW                    (* (reg8 *) Err_LED__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Err_LED_PRTDSI__CAPS_SEL       (* (reg8 *) Err_LED__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Err_LED_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Err_LED__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Err_LED_PRTDSI__OE_SEL0        (* (reg8 *) Err_LED__PRTDSI__OE_SEL0) 
#define Err_LED_PRTDSI__OE_SEL1        (* (reg8 *) Err_LED__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Err_LED_PRTDSI__OUT_SEL0       (* (reg8 *) Err_LED__PRTDSI__OUT_SEL0) 
#define Err_LED_PRTDSI__OUT_SEL1       (* (reg8 *) Err_LED__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Err_LED_PRTDSI__SYNC_OUT       (* (reg8 *) Err_LED__PRTDSI__SYNC_OUT) 


#if defined(Err_LED__INTSTAT)  /* Interrupt Registers */

    #define Err_LED_INTSTAT                (* (reg8 *) Err_LED__INTSTAT)
    #define Err_LED_SNAP                   (* (reg8 *) Err_LED__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins Err_LED_H */

#endif
/* [] END OF FILE */
