/*******************************************************************************
* File Name: Right_Motor.h  
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

#if !defined(CY_PINS_Right_Motor_H) /* Pins Right_Motor_H */
#define CY_PINS_Right_Motor_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Right_Motor_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5 &&\
     CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_5A_PRODUCTION &&\
	 Right_Motor__PORT == 15 && (Right_Motor__MASK & 0xC0))

/***************************************
*        Function Prototypes             
***************************************/    

void    Right_Motor_Write(uint8 value) ;
void    Right_Motor_SetDriveMode(uint8 mode) ;
uint8   Right_Motor_ReadDataReg(void) ;
uint8   Right_Motor_Read(void) ;
uint8   Right_Motor_ClearInterrupt(void) ;

/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Right_Motor_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Right_Motor_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Right_Motor_DM_RES_UP          PIN_DM_RES_UP
#define Right_Motor_DM_RES_DWN         PIN_DM_RES_DWN
#define Right_Motor_DM_OD_LO           PIN_DM_OD_LO
#define Right_Motor_DM_OD_HI           PIN_DM_OD_HI
#define Right_Motor_DM_STRONG          PIN_DM_STRONG
#define Right_Motor_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Right_Motor_MASK               Right_Motor__MASK
#define Right_Motor_SHIFT              Right_Motor__SHIFT
#define Right_Motor_WIDTH              1u

/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Right_Motor_PS                     (* (reg8 *) Right_Motor__PS)
/* Data Register */
#define Right_Motor_DR                     (* (reg8 *) Right_Motor__DR)
/* Port Number */
#define Right_Motor_PRT_NUM                (* (reg8 *) Right_Motor__PRT) 
/* Connect to Analog Globals */                                                  
#define Right_Motor_AG                     (* (reg8 *) Right_Motor__AG)                       
/* Analog MUX bux enable */
#define Right_Motor_AMUX                   (* (reg8 *) Right_Motor__AMUX) 
/* Bidirectional Enable */                                                        
#define Right_Motor_BIE                    (* (reg8 *) Right_Motor__BIE)
/* Bit-mask for Aliased Register Access */
#define Right_Motor_BIT_MASK               (* (reg8 *) Right_Motor__BIT_MASK)
/* Bypass Enable */
#define Right_Motor_BYP                    (* (reg8 *) Right_Motor__BYP)
/* Port wide control signals */                                                   
#define Right_Motor_CTL                    (* (reg8 *) Right_Motor__CTL)
/* Drive Modes */
#define Right_Motor_DM0                    (* (reg8 *) Right_Motor__DM0) 
#define Right_Motor_DM1                    (* (reg8 *) Right_Motor__DM1)
#define Right_Motor_DM2                    (* (reg8 *) Right_Motor__DM2) 
/* Input Buffer Disable Override */
#define Right_Motor_INP_DIS                (* (reg8 *) Right_Motor__INP_DIS)
/* LCD Common or Segment Drive */
#define Right_Motor_LCD_COM_SEG            (* (reg8 *) Right_Motor__LCD_COM_SEG)
/* Enable Segment LCD */
#define Right_Motor_LCD_EN                 (* (reg8 *) Right_Motor__LCD_EN)
/* Slew Rate Control */
#define Right_Motor_SLW                    (* (reg8 *) Right_Motor__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Right_Motor_PRTDSI__CAPS_SEL       (* (reg8 *) Right_Motor__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Right_Motor_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Right_Motor__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Right_Motor_PRTDSI__OE_SEL0        (* (reg8 *) Right_Motor__PRTDSI__OE_SEL0) 
#define Right_Motor_PRTDSI__OE_SEL1        (* (reg8 *) Right_Motor__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Right_Motor_PRTDSI__OUT_SEL0       (* (reg8 *) Right_Motor__PRTDSI__OUT_SEL0) 
#define Right_Motor_PRTDSI__OUT_SEL1       (* (reg8 *) Right_Motor__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Right_Motor_PRTDSI__SYNC_OUT       (* (reg8 *) Right_Motor__PRTDSI__SYNC_OUT) 


#if defined(Right_Motor__INTSTAT)  /* Interrupt Registers */

    #define Right_Motor_INTSTAT                (* (reg8 *) Right_Motor__INTSTAT)
    #define Right_Motor_SNAP                   (* (reg8 *) Right_Motor__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins Right_Motor_H */

#endif
/* [] END OF FILE */
