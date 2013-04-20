/*******************************************************************************
* File Name: Echo_1.h  
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

#if !defined(CY_PINS_Echo_1_H) /* Pins Echo_1_H */
#define CY_PINS_Echo_1_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Echo_1_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    Echo_1_Write(uint8 value) ;
void    Echo_1_SetDriveMode(uint8 mode) ;
uint8   Echo_1_ReadDataReg(void) ;
uint8   Echo_1_Read(void) ;
uint8   Echo_1_ClearInterrupt(void) ;

/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Echo_1_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Echo_1_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Echo_1_DM_RES_UP          PIN_DM_RES_UP
#define Echo_1_DM_RES_DWN         PIN_DM_RES_DWN
#define Echo_1_DM_OD_LO           PIN_DM_OD_LO
#define Echo_1_DM_OD_HI           PIN_DM_OD_HI
#define Echo_1_DM_STRONG          PIN_DM_STRONG
#define Echo_1_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Echo_1_MASK               Echo_1__MASK
#define Echo_1_SHIFT              Echo_1__SHIFT
#define Echo_1_WIDTH              6u

/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Echo_1_PS                     (* (reg8 *) Echo_1__PS)
/* Data Register */
#define Echo_1_DR                     (* (reg8 *) Echo_1__DR)
/* Port Number */
#define Echo_1_PRT_NUM                (* (reg8 *) Echo_1__PRT) 
/* Connect to Analog Globals */                                                  
#define Echo_1_AG                     (* (reg8 *) Echo_1__AG)                       
/* Analog MUX bux enable */
#define Echo_1_AMUX                   (* (reg8 *) Echo_1__AMUX) 
/* Bidirectional Enable */                                                        
#define Echo_1_BIE                    (* (reg8 *) Echo_1__BIE)
/* Bit-mask for Aliased Register Access */
#define Echo_1_BIT_MASK               (* (reg8 *) Echo_1__BIT_MASK)
/* Bypass Enable */
#define Echo_1_BYP                    (* (reg8 *) Echo_1__BYP)
/* Port wide control signals */                                                   
#define Echo_1_CTL                    (* (reg8 *) Echo_1__CTL)
/* Drive Modes */
#define Echo_1_DM0                    (* (reg8 *) Echo_1__DM0) 
#define Echo_1_DM1                    (* (reg8 *) Echo_1__DM1)
#define Echo_1_DM2                    (* (reg8 *) Echo_1__DM2) 
/* Input Buffer Disable Override */
#define Echo_1_INP_DIS                (* (reg8 *) Echo_1__INP_DIS)
/* LCD Common or Segment Drive */
#define Echo_1_LCD_COM_SEG            (* (reg8 *) Echo_1__LCD_COM_SEG)
/* Enable Segment LCD */
#define Echo_1_LCD_EN                 (* (reg8 *) Echo_1__LCD_EN)
/* Slew Rate Control */
#define Echo_1_SLW                    (* (reg8 *) Echo_1__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Echo_1_PRTDSI__CAPS_SEL       (* (reg8 *) Echo_1__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Echo_1_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Echo_1__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Echo_1_PRTDSI__OE_SEL0        (* (reg8 *) Echo_1__PRTDSI__OE_SEL0) 
#define Echo_1_PRTDSI__OE_SEL1        (* (reg8 *) Echo_1__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Echo_1_PRTDSI__OUT_SEL0       (* (reg8 *) Echo_1__PRTDSI__OUT_SEL0) 
#define Echo_1_PRTDSI__OUT_SEL1       (* (reg8 *) Echo_1__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Echo_1_PRTDSI__SYNC_OUT       (* (reg8 *) Echo_1__PRTDSI__SYNC_OUT) 


#if defined(Echo_1__INTSTAT)  /* Interrupt Registers */

    #define Echo_1_INTSTAT                (* (reg8 *) Echo_1__INTSTAT)
    #define Echo_1_SNAP                   (* (reg8 *) Echo_1__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins Echo_1_H */


/* [] END OF FILE */
