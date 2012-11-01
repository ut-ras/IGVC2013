/*******************************************************************************
* File Name: PWM_Clock.h
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

#if !defined(CY_CLOCK_PWM_Clock_H)
#define CY_CLOCK_PWM_Clock_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
*        Function Prototypes
***************************************/

void PWM_Clock_Start(void) ;
void PWM_Clock_Stop(void) ;

#if(!(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3 && \
    CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_3A_ES2) && \
	!(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5 && \
	CYDEV_CHIP_REVISION_USED == CYDEV_CHIP_REVISION_5A_ES1))
void PWM_Clock_StopBlock(void) ;
#endif

void PWM_Clock_StandbyPower(uint8 state) ;
void PWM_Clock_SetDividerRegister(uint16 clkDivider, uint8 reset) ;
uint16 PWM_Clock_GetDividerRegister(void) ;
void PWM_Clock_SetModeRegister(uint8 modeBitMask) ;
void PWM_Clock_ClearModeRegister(uint8 modeBitMask) ;
uint8 PWM_Clock_GetModeRegister(void) ;
void PWM_Clock_SetSourceRegister(uint8 clkSource) ;
uint8 PWM_Clock_GetSourceRegister(void) ;
#if defined(PWM_Clock__CFG3)
void PWM_Clock_SetPhaseRegister(uint8 clkPhase) ;
uint8 PWM_Clock_GetPhaseRegister(void) ;
#endif

#define PWM_Clock_Enable()                       PWM_Clock_Start()
#define PWM_Clock_Disable()                      PWM_Clock_Stop()
#define PWM_Clock_SetDivider(clkDivider)         PWM_Clock_SetDividerRegister(clkDivider, 1)
#define PWM_Clock_SetDividerValue(clkDivider)    PWM_Clock_SetDividerRegister((clkDivider) - 1, 1)
#define PWM_Clock_SetMode(clkMode)               PWM_Clock_SetModeRegister(clkMode)
#define PWM_Clock_SetSource(clkSource)           PWM_Clock_SetSourceRegister(clkSource)
#if defined(PWM_Clock__CFG3)
#define PWM_Clock_SetPhase(clkPhase)             PWM_Clock_SetPhaseRegister(clkPhase)
#define PWM_Clock_SetPhaseValue(clkPhase)        PWM_Clock_SetPhaseRegister((clkPhase) + 1)
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
#define PWM_Clock_CLKEN              (* (reg8 *) PWM_Clock__PM_ACT_CFG)
#define PWM_Clock_CLKEN_PTR          ((reg8 *) PWM_Clock__PM_ACT_CFG)

/* Register to enable or disable the clock */
#define PWM_Clock_CLKSTBY            (* (reg8 *) PWM_Clock__PM_STBY_CFG)
#define PWM_Clock_CLKSTBY_PTR        ((reg8 *) PWM_Clock__PM_STBY_CFG)

/* Clock LSB divider configuration register. */
#define PWM_Clock_DIV_LSB            (* (reg8 *) PWM_Clock__CFG0)
#define PWM_Clock_DIV_LSB_PTR        ((reg8 *) PWM_Clock__CFG0)
#define PWM_Clock_DIV_PTR            ((reg16 *) PWM_Clock__CFG0)

/* Clock MSB divider configuration register. */
#define PWM_Clock_DIV_MSB            (* (reg8 *) PWM_Clock__CFG1)
#define PWM_Clock_DIV_MSB_PTR        ((reg8 *) PWM_Clock__CFG1)

/* Mode and source configuration register */
#define PWM_Clock_MOD_SRC            (* (reg8 *) PWM_Clock__CFG2)
#define PWM_Clock_MOD_SRC_PTR        ((reg8 *) PWM_Clock__CFG2)

#if defined(PWM_Clock__CFG3)
/* Analog clock phase configuration register */
#define PWM_Clock_PHASE              (* (reg8 *) PWM_Clock__CFG3)
#define PWM_Clock_PHASE_PTR          ((reg8 *) PWM_Clock__CFG3)
#endif


/**************************************
*       Register Constants
**************************************/

/* Power manager register masks */
#define PWM_Clock_CLKEN_MASK         PWM_Clock__PM_ACT_MSK
#define PWM_Clock_CLKSTBY_MASK       PWM_Clock__PM_STBY_MSK

/* CFG2 field masks */
#define PWM_Clock_SRC_SEL_MSK        PWM_Clock__CFG2_SRC_SEL_MASK
#define PWM_Clock_MODE_MASK          (~(PWM_Clock_SRC_SEL_MSK))

#if defined(PWM_Clock__CFG3)
/* CFG3 phase mask */
#define PWM_Clock_PHASE_MASK         PWM_Clock__CFG3_PHASE_DLY_MASK
#endif

#endif /* CY_CLOCK_PWM_Clock_H */


/* [] END OF FILE */
