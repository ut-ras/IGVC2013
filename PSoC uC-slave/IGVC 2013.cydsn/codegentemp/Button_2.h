/*******************************************************************************
* File Name: Button_2.h
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
#if !defined(__Button_2_INTC_H__)
#define __Button_2_INTC_H__


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void Button_2_Start(void);
void Button_2_StartEx(cyisraddress address);
void Button_2_Stop(void) ;

CY_ISR_PROTO(Button_2_Interrupt);

void Button_2_SetVector(cyisraddress address) ;
cyisraddress Button_2_GetVector(void) ;

void Button_2_SetPriority(uint8 priority) ;
uint8 Button_2_GetPriority(void) ;

void Button_2_Enable(void) ;
uint8 Button_2_GetState(void) ;
void Button_2_Disable(void) ;

void Button_2_SetPending(void) ;
void Button_2_ClearPending(void) ;


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the Button_2 ISR. */
#define Button_2_INTC_VECTOR            ((reg32 *) Button_2__INTC_VECT)

/* Address of the Button_2 ISR priority. */
#define Button_2_INTC_PRIOR             ((reg8 *) Button_2__INTC_PRIOR_REG)

/* Priority of the Button_2 interrupt. */
#define Button_2_INTC_PRIOR_NUMBER      Button_2__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable Button_2 interrupt. */
#define Button_2_INTC_SET_EN            ((reg32 *) Button_2__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the Button_2 interrupt. */
#define Button_2_INTC_CLR_EN            ((reg32 *) Button_2__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the Button_2 interrupt state to pending. */
#define Button_2_INTC_SET_PD            ((reg32 *) Button_2__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the Button_2 interrupt. */
#define Button_2_INTC_CLR_PD            ((reg32 *) Button_2__INTC_CLR_PD_REG)



/* __Button_2_INTC_H__ */
#endif
