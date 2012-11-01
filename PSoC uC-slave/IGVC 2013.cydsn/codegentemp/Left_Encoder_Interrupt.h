/*******************************************************************************
* File Name: Left_Encoder_Interrupt.h
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
#if !defined(__Left_Encoder_Interrupt_INTC_H__)
#define __Left_Encoder_Interrupt_INTC_H__


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void Left_Encoder_Interrupt_Start(void);
void Left_Encoder_Interrupt_StartEx(cyisraddress address);
void Left_Encoder_Interrupt_Stop(void) ;

CY_ISR_PROTO(Left_Encoder_Interrupt_Interrupt);

void Left_Encoder_Interrupt_SetVector(cyisraddress address) ;
cyisraddress Left_Encoder_Interrupt_GetVector(void) ;

void Left_Encoder_Interrupt_SetPriority(uint8 priority) ;
uint8 Left_Encoder_Interrupt_GetPriority(void) ;

void Left_Encoder_Interrupt_Enable(void) ;
uint8 Left_Encoder_Interrupt_GetState(void) ;
void Left_Encoder_Interrupt_Disable(void) ;

void Left_Encoder_Interrupt_SetPending(void) ;
void Left_Encoder_Interrupt_ClearPending(void) ;


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the Left_Encoder_Interrupt ISR. */
#define Left_Encoder_Interrupt_INTC_VECTOR            ((reg32 *) Left_Encoder_Interrupt__INTC_VECT)

/* Address of the Left_Encoder_Interrupt ISR priority. */
#define Left_Encoder_Interrupt_INTC_PRIOR             ((reg8 *) Left_Encoder_Interrupt__INTC_PRIOR_REG)

/* Priority of the Left_Encoder_Interrupt interrupt. */
#define Left_Encoder_Interrupt_INTC_PRIOR_NUMBER      Left_Encoder_Interrupt__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable Left_Encoder_Interrupt interrupt. */
#define Left_Encoder_Interrupt_INTC_SET_EN            ((reg32 *) Left_Encoder_Interrupt__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the Left_Encoder_Interrupt interrupt. */
#define Left_Encoder_Interrupt_INTC_CLR_EN            ((reg32 *) Left_Encoder_Interrupt__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the Left_Encoder_Interrupt interrupt state to pending. */
#define Left_Encoder_Interrupt_INTC_SET_PD            ((reg32 *) Left_Encoder_Interrupt__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the Left_Encoder_Interrupt interrupt. */
#define Left_Encoder_Interrupt_INTC_CLR_PD            ((reg32 *) Left_Encoder_Interrupt__INTC_CLR_PD_REG)



/* __Left_Encoder_Interrupt_INTC_H__ */
#endif
