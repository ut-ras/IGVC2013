/*******************************************************************************
* File Name: Test_Button_1.h  
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

#if !defined(CY_PINS_Test_Button_1_H) /* Pins Test_Button_1_H */
#define CY_PINS_Test_Button_1_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Test_Button_1_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5 &&\
     CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_5A_PRODUCTION &&\
	 Test_Button_1__PORT == 15 && (Test_Button_1__MASK & 0xC0))

/***************************************
*        Function Prototypes             
***************************************/    

void    Test_Button_1_Write(uint8 value) ;
void    Test_Button_1_SetDriveMode(uint8 mode) ;
uint8   Test_Button_1_ReadDataReg(void) ;
uint8   Test_Button_1_Read(void) ;
uint8   Test_Button_1_ClearInterrupt(void) ;

/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Test_Button_1_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Test_Button_1_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Test_Button_1_DM_RES_UP          PIN_DM_RES_UP
#define Test_Button_1_DM_RES_DWN         PIN_DM_RES_DWN
#define Test_Button_1_DM_OD_LO           PIN_DM_OD_LO
#define Test_Button_1_DM_OD_HI           PIN_DM_OD_HI
#define Test_Button_1_DM_STRONG          PIN_DM_STRONG
#define Test_Button_1_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Test_Button_1_MASK               Test_Button_1__MASK
#define Test_Button_1_SHIFT              Test_Button_1__SHIFT
#define Test_Button_1_WIDTH              1u

/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Test_Button_1_PS                     (* (reg8 *) Test_Button_1__PS)
/* Data Register */
#define Test_Button_1_DR                     (* (reg8 *) Test_Button_1__DR)
/* Port Number */
#define Test_Button_1_PRT_NUM                (* (reg8 *) Test_Button_1__PRT) 
/* Connect to Analog Globals */                                                  
#define Test_Button_1_AG                     (* (reg8 *) Test_Button_1__AG)                       
/* Analog MUX bux enable */
#define Test_Button_1_AMUX                   (* (reg8 *) Test_Button_1__AMUX) 
/* Bidirectional Enable */                                                        
#define Test_Button_1_BIE                    (* (reg8 *) Test_Button_1__BIE)
/* Bit-mask for Aliased Register Access */
#define Test_Button_1_BIT_MASK               (* (reg8 *) Test_Button_1__BIT_MASK)
/* Bypass Enable */
#define Test_Button_1_BYP                    (* (reg8 *) Test_Button_1__BYP)
/* Port wide control signals */                                                   
#define Test_Button_1_CTL                    (* (reg8 *) Test_Button_1__CTL)
/* Drive Modes */
#define Test_Button_1_DM0                    (* (reg8 *) Test_Button_1__DM0) 
#define Test_Button_1_DM1                    (* (reg8 *) Test_Button_1__DM1)
#define Test_Button_1_DM2                    (* (reg8 *) Test_Button_1__DM2) 
/* Input Buffer Disable Override */
#define Test_Button_1_INP_DIS                (* (reg8 *) Test_Button_1__INP_DIS)
/* LCD Common or Segment Drive */
#define Test_Button_1_LCD_COM_SEG            (* (reg8 *) Test_Button_1__LCD_COM_SEG)
/* Enable Segment LCD */
#define Test_Button_1_LCD_EN                 (* (reg8 *) Test_Button_1__LCD_EN)
/* Slew Rate Control */
#define Test_Button_1_SLW                    (* (reg8 *) Test_Button_1__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Test_Button_1_PRTDSI__CAPS_SEL       (* (reg8 *) Test_Button_1__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Test_Button_1_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Test_Button_1__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Test_Button_1_PRTDSI__OE_SEL0        (* (reg8 *) Test_Button_1__PRTDSI__OE_SEL0) 
#define Test_Button_1_PRTDSI__OE_SEL1        (* (reg8 *) Test_Button_1__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Test_Button_1_PRTDSI__OUT_SEL0       (* (reg8 *) Test_Button_1__PRTDSI__OUT_SEL0) 
#define Test_Button_1_PRTDSI__OUT_SEL1       (* (reg8 *) Test_Button_1__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Test_Button_1_PRTDSI__SYNC_OUT       (* (reg8 *) Test_Button_1__PRTDSI__SYNC_OUT) 


#if defined(Test_Button_1__INTSTAT)  /* Interrupt Registers */

    #define Test_Button_1_INTSTAT                (* (reg8 *) Test_Button_1__INTSTAT)
    #define Test_Button_1_SNAP                   (* (reg8 *) Test_Button_1__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins Test_Button_1_H */

#endif
/* [] END OF FILE */
