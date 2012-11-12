/*******************************************************************************
* File Name: IMU_Interrupt.h
* Version 1.50
*
*  Description:
*   Provides the function definitions for the Interrupt Controller.
*
*
********************************************************************************
* Copyright 2008-2010, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/
#if !defined(__IMU_Interrupt_INTC_H__)
#define __IMU_Interrupt_INTC_H__


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void IMU_Interrupt_Start(void);
void IMU_Interrupt_StartEx(cyisraddress address);
void IMU_Interrupt_Stop(void) ;

CY_ISR_PROTO(IMU_Interrupt_Interrupt);

void IMU_Interrupt_SetVector(cyisraddress address) ;
cyisraddress IMU_Interrupt_GetVector(void) ;

void IMU_Interrupt_SetPriority(uint8 priority) ;
uint8 IMU_Interrupt_GetPriority(void) ;

void IMU_Interrupt_Enable(void) ;
uint8 IMU_Interrupt_GetState(void) ;
void IMU_Interrupt_Disable(void) ;

void IMU_Interrupt_SetPending(void) ;
void IMU_Interrupt_ClearPending(void) ;


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the IMU_Interrupt ISR. */
#define IMU_Interrupt_INTC_VECTOR            ((reg32 *) IMU_Interrupt__INTC_VECT)

/* Address of the IMU_Interrupt ISR priority. */
#define IMU_Interrupt_INTC_PRIOR             ((reg8 *) IMU_Interrupt__INTC_PRIOR_REG)

/* Priority of the IMU_Interrupt interrupt. */
#define IMU_Interrupt_INTC_PRIOR_NUMBER      IMU_Interrupt__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable IMU_Interrupt interrupt. */
#define IMU_Interrupt_INTC_SET_EN            ((reg32 *) IMU_Interrupt__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the IMU_Interrupt interrupt. */
#define IMU_Interrupt_INTC_CLR_EN            ((reg32 *) IMU_Interrupt__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the IMU_Interrupt interrupt state to pending. */
#define IMU_Interrupt_INTC_SET_PD            ((reg32 *) IMU_Interrupt__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the IMU_Interrupt interrupt. */
#define IMU_Interrupt_INTC_CLR_PD            ((reg32 *) IMU_Interrupt__INTC_CLR_PD_REG)



/* __IMU_Interrupt_INTC_H__ */
#endif
