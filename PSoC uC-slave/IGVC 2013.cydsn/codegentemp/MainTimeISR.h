/*******************************************************************************
* File Name: MainTimeISR.h
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
#if !defined(__MainTimeISR_INTC_H__)
#define __MainTimeISR_INTC_H__


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void MainTimeISR_Start(void);
void MainTimeISR_StartEx(cyisraddress address);
void MainTimeISR_Stop(void) ;

CY_ISR_PROTO(MainTimeISR_Interrupt);

void MainTimeISR_SetVector(cyisraddress address) ;
cyisraddress MainTimeISR_GetVector(void) ;

void MainTimeISR_SetPriority(uint8 priority) ;
uint8 MainTimeISR_GetPriority(void) ;

void MainTimeISR_Enable(void) ;
uint8 MainTimeISR_GetState(void) ;
void MainTimeISR_Disable(void) ;

void MainTimeISR_SetPending(void) ;
void MainTimeISR_ClearPending(void) ;


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the MainTimeISR ISR. */
#define MainTimeISR_INTC_VECTOR            ((reg32 *) MainTimeISR__INTC_VECT)

/* Address of the MainTimeISR ISR priority. */
#define MainTimeISR_INTC_PRIOR             ((reg8 *) MainTimeISR__INTC_PRIOR_REG)

/* Priority of the MainTimeISR interrupt. */
#define MainTimeISR_INTC_PRIOR_NUMBER      MainTimeISR__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable MainTimeISR interrupt. */
#define MainTimeISR_INTC_SET_EN            ((reg32 *) MainTimeISR__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the MainTimeISR interrupt. */
#define MainTimeISR_INTC_CLR_EN            ((reg32 *) MainTimeISR__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the MainTimeISR interrupt state to pending. */
#define MainTimeISR_INTC_SET_PD            ((reg32 *) MainTimeISR__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the MainTimeISR interrupt. */
#define MainTimeISR_INTC_CLR_PD            ((reg32 *) MainTimeISR__INTC_CLR_PD_REG)



/* __MainTimeISR_INTC_H__ */
#endif
