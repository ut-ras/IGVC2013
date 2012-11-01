/*******************************************************************************
* File Name: Encoder_Clock_2.h
* Version 1.60
*
*  Description:
*   Provides the function and constant definitions for the clock component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2010, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

#if !defined(CY_CLOCK_Encoder_Clock_2_H)
#define CY_CLOCK_Encoder_Clock_2_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
*        Function Prototypes
***************************************/

void Encoder_Clock_2_Start(void) ;
void Encoder_Clock_2_Stop(void) ;

#if(!(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3 && \
    CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_3A_ES2) && \
	!(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5 && \
	CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_5A_ES1))
void Encoder_Clock_2_StopBlock(void) ;
#endif

void Encoder_Clock_2_StandbyPower(uint8 state) ;
void Encoder_Clock_2_SetDividerRegister(uint16 clkDivider, uint8 reset) ;
uint16 Encoder_Clock_2_GetDividerRegister(void) ;
void Encoder_Clock_2_SetModeRegister(uint8 modeBitMask) ;
void Encoder_Clock_2_ClearModeRegister(uint8 modeBitMask) ;
uint8 Encoder_Clock_2_GetModeRegister(void) ;
void Encoder_Clock_2_SetSourceRegister(uint8 clkSource) ;
uint8 Encoder_Clock_2_GetSourceRegister(void) ;
#if defined(Encoder_Clock_2__CFG3)
void Encoder_Clock_2_SetPhaseRegister(uint8 clkPhase) ;
uint8 Encoder_Clock_2_GetPhaseRegister(void) ;
#endif

#define Encoder_Clock_2_Enable()                       Encoder_Clock_2_Start()
#define Encoder_Clock_2_Disable()                      Encoder_Clock_2_Stop()
#define Encoder_Clock_2_SetDivider(clkDivider)         Encoder_Clock_2_SetDividerRegister(clkDivider, 1)
#define Encoder_Clock_2_SetDividerValue(clkDivider)    Encoder_Clock_2_SetDividerRegister((clkDivider) - 1, 1)
#define Encoder_Clock_2_SetMode(clkMode)               Encoder_Clock_2_SetModeRegister(clkMode)
#define Encoder_Clock_2_SetSource(clkSource)           Encoder_Clock_2_SetSourceRegister(clkSource)
#if defined(Encoder_Clock_2__CFG3)
#define Encoder_Clock_2_SetPhase(clkPhase)             Encoder_Clock_2_SetPhaseRegister(clkPhase)
#define Encoder_Clock_2_SetPhaseValue(clkPhase)        Encoder_Clock_2_SetPhaseRegister((clkPhase) + 1)
#endif


/***************************************
*           API Constants
***************************************/

/* Constants SetPhase(), clkPhase parameter. Only valid for PSoC 3 ES2 and earlier. See datasheet for details. */
#if CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3 && \
   (CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_3A_ES1 || \
    CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_3A_ES2)
#define CYCLK_2_5NS             0x01 /* 2.5 ns delay. */
#define CYCLK_3_5NS             0x02 /* 3.5 ns delay. */
#define CYCLK_4_5NS             0x03 /* 4.5 ns delay. */
#define CYCLK_5_5NS             0x04 /* 5.5 ns delay. */
#define CYCLK_6_5NS             0x05 /* 6.5 ns delay. */
#define CYCLK_7_5NS             0x06 /* 7.5 ns delay. */
#define CYCLK_8_5NS             0x07 /* 8.5 ns delay. */
#define CYCLK_9_5NS             0x08 /* 9.5 ns delay. */
#define CYCLK_10_5NS            0x09 /* 10.5 ns delay. */
#define CYCLK_11_5NS            0x0A /* 11.5 ns delay. */
#define CYCLK_12_5NS            0x0B /* 12.5 ns delay. */
#endif


/***************************************
*             Registers
***************************************/

/* Register to enable or disable the clock */
#define Encoder_Clock_2_CLKEN              (* (reg8 *) Encoder_Clock_2__PM_ACT_CFG)
#define Encoder_Clock_2_CLKEN_PTR          ((reg8 *) Encoder_Clock_2__PM_ACT_CFG)

/* Register to enable or disable the clock */
#define Encoder_Clock_2_CLKSTBY            (* (reg8 *) Encoder_Clock_2__PM_STBY_CFG)
#define Encoder_Clock_2_CLKSTBY_PTR        ((reg8 *) Encoder_Clock_2__PM_STBY_CFG)

/* Clock LSB divider configuration register. */
#define Encoder_Clock_2_DIV_LSB            (* (reg8 *) Encoder_Clock_2__CFG0)
#define Encoder_Clock_2_DIV_LSB_PTR        ((reg8 *) Encoder_Clock_2__CFG0)
#define Encoder_Clock_2_DIV_PTR            ((reg16 *) Encoder_Clock_2__CFG0)

/* Clock MSB divider configuration register. */
#define Encoder_Clock_2_DIV_MSB            (* (reg8 *) Encoder_Clock_2__CFG1)
#define Encoder_Clock_2_DIV_MSB_PTR        ((reg8 *) Encoder_Clock_2__CFG1)

/* Mode and source configuration register */
#define Encoder_Clock_2_MOD_SRC            (* (reg8 *) Encoder_Clock_2__CFG2)
#define Encoder_Clock_2_MOD_SRC_PTR        ((reg8 *) Encoder_Clock_2__CFG2)

#if defined(Encoder_Clock_2__CFG3)
/* Analog clock phase configuration register */
#define Encoder_Clock_2_PHASE              (* (reg8 *) Encoder_Clock_2__CFG3)
#define Encoder_Clock_2_PHASE_PTR          ((reg8 *) Encoder_Clock_2__CFG3)
#endif


/**************************************
*       Register Constants
**************************************/

/* Power manager register masks */
#define Encoder_Clock_2_CLKEN_MASK         Encoder_Clock_2__PM_ACT_MSK
#define Encoder_Clock_2_CLKSTBY_MASK       Encoder_Clock_2__PM_STBY_MSK

/* CFG2 field masks */
#define Encoder_Clock_2_SRC_SEL_MSK        Encoder_Clock_2__CFG2_SRC_SEL_MASK
#define Encoder_Clock_2_MODE_MASK          (~(Encoder_Clock_2_SRC_SEL_MSK))

#if defined(Encoder_Clock_2__CFG3)
/* CFG3 phase mask */
#define Encoder_Clock_2_PHASE_MASK         Encoder_Clock_2__CFG3_PHASE_DLY_MASK
#endif

#endif /* CY_CLOCK_Encoder_Clock_2_H */


/* [] END OF FILE */
