/*******************************************************************************
* File Name: Echo_0.h  
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

#if !defined(CY_PINS_Echo_0_H) /* Pins Echo_0_H */
#define CY_PINS_Echo_0_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Echo_0_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    Echo_0_Write(uint8 value) ;
void    Echo_0_SetDriveMode(uint8 mode) ;
uint8   Echo_0_ReadDataReg(void) ;
uint8   Echo_0_Read(void) ;
uint8   Echo_0_ClearInterrupt(void) ;

/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Echo_0_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Echo_0_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Echo_0_DM_RES_UP          PIN_DM_RES_UP
#define Echo_0_DM_RES_DWN         PIN_DM_RES_DWN
#define Echo_0_DM_OD_LO           PIN_DM_OD_LO
#define Echo_0_DM_OD_HI           PIN_DM_OD_HI
#define Echo_0_DM_STRONG          PIN_DM_STRONG
#define Echo_0_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Echo_0_MASK               Echo_0__MASK
#define Echo_0_SHIFT              Echo_0__SHIFT
#define Echo_0_WIDTH              6u

/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Echo_0_PS                     (* (reg8 *) Echo_0__PS)
/* Data Register */
#define Echo_0_DR                     (* (reg8 *) Echo_0__DR)
/* Port Number */
#define Echo_0_PRT_NUM                (* (reg8 *) Echo_0__PRT) 
/* Connect to Analog Globals */                                                  
#define Echo_0_AG                     (* (reg8 *) Echo_0__AG)                       
/* Analog MUX bux enable */
#define Echo_0_AMUX                   (* (reg8 *) Echo_0__AMUX) 
/* Bidirectional Enable */                                                        
#define Echo_0_BIE                    (* (reg8 *) Echo_0__BIE)
/* Bit-mask for Aliased Register Access */
#define Echo_0_BIT_MASK               (* (reg8 *) Echo_0__BIT_MASK)
/* Bypass Enable */
#define Echo_0_BYP                    (* (reg8 *) Echo_0__BYP)
/* Port wide control signals */                                                   
#define Echo_0_CTL                    (* (reg8 *) Echo_0__CTL)
/* Drive Modes */
#define Echo_0_DM0                    (* (reg8 *) Echo_0__DM0) 
#define Echo_0_DM1                    (* (reg8 *) Echo_0__DM1)
#define Echo_0_DM2                    (* (reg8 *) Echo_0__DM2) 
/* Input Buffer Disable Override */
#define Echo_0_INP_DIS                (* (reg8 *) Echo_0__INP_DIS)
/* LCD Common or Segment Drive */
#define Echo_0_LCD_COM_SEG            (* (reg8 *) Echo_0__LCD_COM_SEG)
/* Enable Segment LCD */
#define Echo_0_LCD_EN                 (* (reg8 *) Echo_0__LCD_EN)
/* Slew Rate Control */
#define Echo_0_SLW                    (* (reg8 *) Echo_0__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Echo_0_PRTDSI__CAPS_SEL       (* (reg8 *) Echo_0__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Echo_0_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Echo_0__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Echo_0_PRTDSI__OE_SEL0        (* (reg8 *) Echo_0__PRTDSI__OE_SEL0) 
#define Echo_0_PRTDSI__OE_SEL1        (* (reg8 *) Echo_0__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Echo_0_PRTDSI__OUT_SEL0       (* (reg8 *) Echo_0__PRTDSI__OUT_SEL0) 
#define Echo_0_PRTDSI__OUT_SEL1       (* (reg8 *) Echo_0__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Echo_0_PRTDSI__SYNC_OUT       (* (reg8 *) Echo_0__PRTDSI__SYNC_OUT) 


#if defined(Echo_0__INTSTAT)  /* Interrupt Registers */

    #define Echo_0_INTSTAT                (* (reg8 *) Echo_0__INTSTAT)
    #define Echo_0_SNAP                   (* (reg8 *) Echo_0__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins Echo_0_H */


/* [] END OF FILE */
