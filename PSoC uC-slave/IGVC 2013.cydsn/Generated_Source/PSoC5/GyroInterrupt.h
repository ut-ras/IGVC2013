/*******************************************************************************
* File Name: GyroInterrupt.h
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
#if !defined(__GyroInterrupt_INTC_H__)
#define __GyroInterrupt_INTC_H__


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void GyroInterrupt_Start(void);
void GyroInterrupt_StartEx(cyisraddress address);
void GyroInterrupt_Stop(void) ;

CY_ISR_PROTO(GyroInterrupt_Interrupt);

void GyroInterrupt_SetVector(cyisraddress address) ;
cyisraddress GyroInterrupt_GetVector(void) ;

void GyroInterrupt_SetPriority(uint8 priority) ;
uint8 GyroInterrupt_GetPriority(void) ;

void GyroInterrupt_Enable(void) ;
uint8 GyroInterrupt_GetState(void) ;
void GyroInterrupt_Disable(void) ;

void GyroInterrupt_SetPending(void) ;
void GyroInterrupt_ClearPending(void) ;


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the GyroInterrupt ISR. */
#define GyroInterrupt_INTC_VECTOR            ((reg32 *) GyroInterrupt__INTC_VECT)

/* Address of the GyroInterrupt ISR priority. */
#define GyroInterrupt_INTC_PRIOR             ((reg8 *) GyroInterrupt__INTC_PRIOR_REG)

/* Priority of the GyroInterrupt interrupt. */
#define GyroInterrupt_INTC_PRIOR_NUMBER      GyroInterrupt__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable GyroInterrupt interrupt. */
#define GyroInterrupt_INTC_SET_EN            ((reg32 *) GyroInterrupt__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the GyroInterrupt interrupt. */
#define GyroInterrupt_INTC_CLR_EN            ((reg32 *) GyroInterrupt__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the GyroInterrupt interrupt state to pending. */
#define GyroInterrupt_INTC_SET_PD            ((reg32 *) GyroInterrupt__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the GyroInterrupt interrupt. */
#define GyroInterrupt_INTC_CLR_PD            ((reg32 *) GyroInterrupt__INTC_CLR_PD_REG)



/* __GyroInterrupt_INTC_H__ */
#endif
