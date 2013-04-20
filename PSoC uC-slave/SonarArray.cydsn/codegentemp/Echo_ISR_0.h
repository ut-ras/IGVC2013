/*******************************************************************************
* File Name: Echo_ISR_0.h
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
#if !defined(__Echo_ISR_0_INTC_H__)
#define __Echo_ISR_0_INTC_H__


#include <cytypes.h>
#include <cyfitter.h>

#if(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3)     
    #if(CYDEV_CHIP_REVISION_USED <= CYDEV_CHIP_REVISION_3A_ES2)      
        #include <intrins.h>
        #define Echo_ISR_0_ISR_PATCH() _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
    #endif
#endif


/* Interrupt Controller API. */
void Echo_ISR_0_Start(void);
void Echo_ISR_0_StartEx(cyisraddress address);
void Echo_ISR_0_Stop(void) ;

CY_ISR_PROTO(Echo_ISR_0_Interrupt);

void Echo_ISR_0_SetVector(cyisraddress address) ;
cyisraddress Echo_ISR_0_GetVector(void) ;

void Echo_ISR_0_SetPriority(uint8 priority) ;
uint8 Echo_ISR_0_GetPriority(void) ;

void Echo_ISR_0_Enable(void) ;
uint8 Echo_ISR_0_GetState(void) ;
void Echo_ISR_0_Disable(void) ;

void Echo_ISR_0_SetPending(void) ;
void Echo_ISR_0_ClearPending(void) ;


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the Echo_ISR_0 ISR. */
#define Echo_ISR_0_INTC_VECTOR            ((reg16 *) Echo_ISR_0__INTC_VECT)

/* Address of the Echo_ISR_0 ISR priority. */
#define Echo_ISR_0_INTC_PRIOR             ((reg8 *) Echo_ISR_0__INTC_PRIOR_REG)

/* Priority of the Echo_ISR_0 interrupt. */
#define Echo_ISR_0_INTC_PRIOR_NUMBER      Echo_ISR_0__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable Echo_ISR_0 interrupt. */
#define Echo_ISR_0_INTC_SET_EN            ((reg8 *) Echo_ISR_0__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the Echo_ISR_0 interrupt. */
#define Echo_ISR_0_INTC_CLR_EN            ((reg8 *) Echo_ISR_0__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the Echo_ISR_0 interrupt state to pending. */
#define Echo_ISR_0_INTC_SET_PD            ((reg8 *) Echo_ISR_0__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the Echo_ISR_0 interrupt. */
#define Echo_ISR_0_INTC_CLR_PD            ((reg8 *) Echo_ISR_0__INTC_CLR_PD_REG)



/* __Echo_ISR_0_INTC_H__ */
#endif
