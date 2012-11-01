/*******************************************************************************
* File Name: Left_Encoder.h  
* Version 2.0
*
* Description:
*  This file provides constants and parameter values for the Quadrature
*  Decoder component.
*
* Note:
*  None.
*
********************************************************************************
* Copyright 2008-2011, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_QUADRATURE_DECODER_Left_Encoder_H)
#define CY_QUADRATURE_DECODER_Left_Encoder_H

#include "cyfitter.h"

#define Left_Encoder_COUNTER_SIZE               (32u)

#if (Left_Encoder_COUNTER_SIZE == 8u)
    #include "Left_Encoder_Cnt8.h"
#else
    #include "Left_Encoder_Cnt16.h"
#endif /* Left_Encoder_COUNTER_SIZE == 8u */

#if(Left_Encoder_COUNTER_SIZE == 32u)

	/* Check to see if required defines such as CY_PSOC3 and CY_PSOC5 are available */
	/* They are defined starting with cy_boot v2.30 */
	#ifndef CY_PSOC3
	    #error Component QuadDec_v2_0 requires cy_boot v2.30 or later
	#endif /* CY_PSOC3 */    
                                        
    #if(CY_PSOC3_ES2 && (Left_Encoder_isr__ES2_PATCH))
    
        #include <intrins.h>
        #define Left_Encoder_ISR_PATCH() _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
        
    #endif /* End PSOC3_ES2 */

#endif /* Left_Encoder_COUNTER_SIZE == 32u */


/***************************************
*   Conditional Compilation Parameters
***************************************/

#define Left_Encoder_COUNTER_RESOLUTION         (4u)


/***************************************
*       Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct _Left_Encoder_backupStruct
{
    uint8 enableState;
} Left_Encoder_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

void    Left_Encoder_Init(void) ;
void    Left_Encoder_Start(void);
void    Left_Encoder_Stop(void) ;
void    Left_Encoder_Enable(void) ;
uint8   Left_Encoder_GetEvents(void) ;
void    Left_Encoder_SetInterruptMask(uint8 mask) ;
uint8   Left_Encoder_GetInterruptMask(void) ;
int32    Left_Encoder_GetCounter(void) ;
void    Left_Encoder_SetCounter(int32 value);
void    Left_Encoder_Sleep(void);
void    Left_Encoder_Wakeup(void) ;
void    Left_Encoder_SaveConfig(void) ;
void    Left_Encoder_RestoreConfig(void) ;

CY_ISR_PROTO(Left_Encoder_ISR);


/***************************************
*           API Constants
***************************************/

#define Left_Encoder_ISR_NUMBER                 (Left_Encoder_isr__INTC_NUMBER)
#define Left_Encoder_ISR_PRIORITY               (Left_Encoder_isr__INTC_PRIOR_NUM)


/***************************************
*    Enumerated Types and Parameters
***************************************/

#define Left_Encoder_GLITCH_FILTERING           (1u)
#define Left_Encoder_INDEX_INPUT                (0u)


/***************************************
*    Initial Parameter Constants
***************************************/

#if (Left_Encoder_COUNTER_SIZE == 8u)    
    #define Left_Encoder_COUNTER_INIT_VALUE    (0x80u)
#else /* (Left_Encoder_COUNTER_SIZE == 16u) || (Left_Encoder_COUNTER_SIZE == 32u) */
    #define Left_Encoder_COUNTER_INIT_VALUE    (0x8000u)
#endif /* Left_Encoder_COUNTER_SIZE == 8u */   


/***************************************
*             Registers
***************************************/

#define Left_Encoder_STATUS_REG                 (* (reg8 *) Left_Encoder_bQuadDec_Stsreg__STATUS_REG)
#define Left_Encoder_STATUS_PTR                 (  (reg8 *) Left_Encoder_bQuadDec_Stsreg__STATUS_REG)
#define Left_Encoder_STATUS_MASK                (* (reg8 *) Left_Encoder_bQuadDec_Stsreg__MASK_REG)
#define Left_Encoder_STATUS_MASK_PTR            (  (reg8 *) Left_Encoder_bQuadDec_Stsreg__MASK_REG)
#define Left_Encoder_SR_AUX_CONTROL             (* (reg8 *) Left_Encoder_bQuadDec_Stsreg__STATUS_AUX_CTL_REG)
#define Left_Encoder_SR_AUX_CONTROL_PTR         (  (reg8 *) Left_Encoder_bQuadDec_Stsreg__STATUS_AUX_CTL_REG)


/***************************************
*        Register Constants
***************************************/

#define Left_Encoder_COUNTER_OVERFLOW_SHIFT     (0x00u)
#define Left_Encoder_COUNTER_UNDERFLOW_SHIFT    (0x01u)
#define Left_Encoder_COUNTER_RESET_SHIFT        (0x02u)
#define Left_Encoder_INVALID_IN_SHIFT           (0x03u)
#define Left_Encoder_COUNTER_OVERFLOW           (0x01u << Left_Encoder_COUNTER_OVERFLOW_SHIFT)
#define Left_Encoder_COUNTER_UNDERFLOW          (0x01u << Left_Encoder_COUNTER_UNDERFLOW_SHIFT)
#define Left_Encoder_COUNTER_RESET              (0x01u << Left_Encoder_COUNTER_RESET_SHIFT)
#define Left_Encoder_INVALID_IN                 (0x01u << Left_Encoder_INVALID_IN_SHIFT)

#define Left_Encoder_INTERRUPTS_ENABLE_SHIFT    (0x04u)
#define Left_Encoder_INTERRUPTS_ENABLE          (0x01u << Left_Encoder_INTERRUPTS_ENABLE_SHIFT)
#define Left_Encoder_INIT_INT_MASK              (0x0Fu)
#define Left_Encoder_DISABLE                    (0x00u)     

#endif /* CY_QUADRATURE_DECODER_Left_Encoder_H */


/* [] END OF FILE */
