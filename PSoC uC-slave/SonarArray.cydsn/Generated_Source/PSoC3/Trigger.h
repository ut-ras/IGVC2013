/*******************************************************************************
* File Name: Trigger.h  
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

#if !defined(CY_PINS_Trigger_H) /* Pins Trigger_H */
#define CY_PINS_Trigger_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Trigger_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    Trigger_Write(uint8 value) ;
void    Trigger_SetDriveMode(uint8 mode) ;
uint8   Trigger_ReadDataReg(void) ;
uint8   Trigger_Read(void) ;
uint8   Trigger_ClearInterrupt(void) ;

/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Trigger_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Trigger_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Trigger_DM_RES_UP          PIN_DM_RES_UP
#define Trigger_DM_RES_DWN         PIN_DM_RES_DWN
#define Trigger_DM_OD_LO           PIN_DM_OD_LO
#define Trigger_DM_OD_HI           PIN_DM_OD_HI
#define Trigger_DM_STRONG          PIN_DM_STRONG
#define Trigger_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Trigger_MASK               Trigger__MASK
#define Trigger_SHIFT              Trigger__SHIFT
#define Trigger_WIDTH              1u

/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Trigger_PS                     (* (reg8 *) Trigger__PS)
/* Data Register */
#define Trigger_DR                     (* (reg8 *) Trigger__DR)
/* Port Number */
#define Trigger_PRT_NUM                (* (reg8 *) Trigger__PRT) 
/* Connect to Analog Globals */                                                  
#define Trigger_AG                     (* (reg8 *) Trigger__AG)                       
/* Analog MUX bux enable */
#define Trigger_AMUX                   (* (reg8 *) Trigger__AMUX) 
/* Bidirectional Enable */                                                        
#define Trigger_BIE                    (* (reg8 *) Trigger__BIE)
/* Bit-mask for Aliased Register Access */
#define Trigger_BIT_MASK               (* (reg8 *) Trigger__BIT_MASK)
/* Bypass Enable */
#define Trigger_BYP                    (* (reg8 *) Trigger__BYP)
/* Port wide control signals */                                                   
#define Trigger_CTL                    (* (reg8 *) Trigger__CTL)
/* Drive Modes */
#define Trigger_DM0                    (* (reg8 *) Trigger__DM0) 
#define Trigger_DM1                    (* (reg8 *) Trigger__DM1)
#define Trigger_DM2                    (* (reg8 *) Trigger__DM2) 
/* Input Buffer Disable Override */
#define Trigger_INP_DIS                (* (reg8 *) Trigger__INP_DIS)
/* LCD Common or Segment Drive */
#define Trigger_LCD_COM_SEG            (* (reg8 *) Trigger__LCD_COM_SEG)
/* Enable Segment LCD */
#define Trigger_LCD_EN                 (* (reg8 *) Trigger__LCD_EN)
/* Slew Rate Control */
#define Trigger_SLW                    (* (reg8 *) Trigger__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Trigger_PRTDSI__CAPS_SEL       (* (reg8 *) Trigger__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Trigger_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Trigger__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Trigger_PRTDSI__OE_SEL0        (* (reg8 *) Trigger__PRTDSI__OE_SEL0) 
#define Trigger_PRTDSI__OE_SEL1        (* (reg8 *) Trigger__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Trigger_PRTDSI__OUT_SEL0       (* (reg8 *) Trigger__PRTDSI__OUT_SEL0) 
#define Trigger_PRTDSI__OUT_SEL1       (* (reg8 *) Trigger__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Trigger_PRTDSI__SYNC_OUT       (* (reg8 *) Trigger__PRTDSI__SYNC_OUT) 


#if defined(Trigger__INTSTAT)  /* Interrupt Registers */

    #define Trigger_INTSTAT                (* (reg8 *) Trigger__INTSTAT)
    #define Trigger_SNAP                   (* (reg8 *) Trigger__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins Trigger_H */


/* [] END OF FILE */
