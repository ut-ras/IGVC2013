/*******************************************************************************
* File Name: Soft_Kill.h  
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

#if !defined(CY_PINS_Soft_Kill_H) /* Pins Soft_Kill_H */
#define CY_PINS_Soft_Kill_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Soft_Kill_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5 &&\
     CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_5A_PRODUCTION &&\
	 Soft_Kill__PORT == 15 && (Soft_Kill__MASK & 0xC0))

/***************************************
*        Function Prototypes             
***************************************/    

void    Soft_Kill_Write(uint8 value) ;
void    Soft_Kill_SetDriveMode(uint8 mode) ;
uint8   Soft_Kill_ReadDataReg(void) ;
uint8   Soft_Kill_Read(void) ;
uint8   Soft_Kill_ClearInterrupt(void) ;

/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Soft_Kill_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Soft_Kill_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Soft_Kill_DM_RES_UP          PIN_DM_RES_UP
#define Soft_Kill_DM_RES_DWN         PIN_DM_RES_DWN
#define Soft_Kill_DM_OD_LO           PIN_DM_OD_LO
#define Soft_Kill_DM_OD_HI           PIN_DM_OD_HI
#define Soft_Kill_DM_STRONG          PIN_DM_STRONG
#define Soft_Kill_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Soft_Kill_MASK               Soft_Kill__MASK
#define Soft_Kill_SHIFT              Soft_Kill__SHIFT
#define Soft_Kill_WIDTH              1u

/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Soft_Kill_PS                     (* (reg8 *) Soft_Kill__PS)
/* Data Register */
#define Soft_Kill_DR                     (* (reg8 *) Soft_Kill__DR)
/* Port Number */
#define Soft_Kill_PRT_NUM                (* (reg8 *) Soft_Kill__PRT) 
/* Connect to Analog Globals */                                                  
#define Soft_Kill_AG                     (* (reg8 *) Soft_Kill__AG)                       
/* Analog MUX bux enable */
#define Soft_Kill_AMUX                   (* (reg8 *) Soft_Kill__AMUX) 
/* Bidirectional Enable */                                                        
#define Soft_Kill_BIE                    (* (reg8 *) Soft_Kill__BIE)
/* Bit-mask for Aliased Register Access */
#define Soft_Kill_BIT_MASK               (* (reg8 *) Soft_Kill__BIT_MASK)
/* Bypass Enable */
#define Soft_Kill_BYP                    (* (reg8 *) Soft_Kill__BYP)
/* Port wide control signals */                                                   
#define Soft_Kill_CTL                    (* (reg8 *) Soft_Kill__CTL)
/* Drive Modes */
#define Soft_Kill_DM0                    (* (reg8 *) Soft_Kill__DM0) 
#define Soft_Kill_DM1                    (* (reg8 *) Soft_Kill__DM1)
#define Soft_Kill_DM2                    (* (reg8 *) Soft_Kill__DM2) 
/* Input Buffer Disable Override */
#define Soft_Kill_INP_DIS                (* (reg8 *) Soft_Kill__INP_DIS)
/* LCD Common or Segment Drive */
#define Soft_Kill_LCD_COM_SEG            (* (reg8 *) Soft_Kill__LCD_COM_SEG)
/* Enable Segment LCD */
#define Soft_Kill_LCD_EN                 (* (reg8 *) Soft_Kill__LCD_EN)
/* Slew Rate Control */
#define Soft_Kill_SLW                    (* (reg8 *) Soft_Kill__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Soft_Kill_PRTDSI__CAPS_SEL       (* (reg8 *) Soft_Kill__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Soft_Kill_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Soft_Kill__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Soft_Kill_PRTDSI__OE_SEL0        (* (reg8 *) Soft_Kill__PRTDSI__OE_SEL0) 
#define Soft_Kill_PRTDSI__OE_SEL1        (* (reg8 *) Soft_Kill__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Soft_Kill_PRTDSI__OUT_SEL0       (* (reg8 *) Soft_Kill__PRTDSI__OUT_SEL0) 
#define Soft_Kill_PRTDSI__OUT_SEL1       (* (reg8 *) Soft_Kill__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Soft_Kill_PRTDSI__SYNC_OUT       (* (reg8 *) Soft_Kill__PRTDSI__SYNC_OUT) 


#if defined(Soft_Kill__INTSTAT)  /* Interrupt Registers */

    #define Soft_Kill_INTSTAT                (* (reg8 *) Soft_Kill__INTSTAT)
    #define Soft_Kill_SNAP                   (* (reg8 *) Soft_Kill__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins Soft_Kill_H */

#endif
/* [] END OF FILE */
