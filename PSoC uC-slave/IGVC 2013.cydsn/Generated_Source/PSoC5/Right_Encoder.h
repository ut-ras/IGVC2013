/*******************************************************************************
* File Name: Right_Encoder.h  
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

#if !defined(CY_QUADRATURE_DECODER_Right_Encoder_H)
#define CY_QUADRATURE_DECODER_Right_Encoder_H

#include "cyfitter.h"

#define Right_Encoder_COUNTER_SIZE               (32u)

#if (Right_Encoder_COUNTER_SIZE == 8u)
    #include "Right_Encoder_Cnt8.h"
#else
    #include "Right_Encoder_Cnt16.h"
#endif /* Right_Encoder_COUNTER_SIZE == 8u */

#if(Right_Encoder_COUNTER_SIZE == 32u)

	/* Check to see if required defines such as CY_PSOC3 and CY_PSOC5 are available */
	/* They are defined starting with cy_boot v2.30 */
	#ifndef CY_PSOC3
	    #error Component QuadDec_v2_0 requires cy_boot v2.30 or later
	#endif /* CY_PSOC3 */    
                                        
    #if(CY_PSOC3_ES2 && (Right_Encoder_isr__ES2_PATCH))
    
        #include <intrins.h>
        #define Right_Encoder_ISR_PATCH() _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
        
    #endif /* End PSOC3_ES2 */

#endif /* Right_Encoder_COUNTER_SIZE == 32u */


/***************************************
*   Conditional Compilation Parameters
***************************************/

#define Right_Encoder_COUNTER_RESOLUTION         (4u)


/***************************************
*       Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct _Right_Encoder_backupStruct
{
    uint8 enableState;
} Right_Encoder_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

void    Right_Encoder_Init(void) ;
void    Right_Encoder_Start(void);
void    Right_Encoder_Stop(void) ;
void    Right_Encoder_Enable(void) ;
uint8   Right_Encoder_GetEvents(void) ;
void    Right_Encoder_SetInterruptMask(uint8 mask) ;
uint8   Right_Encoder_GetInterruptMask(void) ;
int32    Right_Encoder_GetCounter(void) ;
void    Right_Encoder_SetCounter(int32 value);
void    Right_Encoder_Sleep(void);
void    Right_Encoder_Wakeup(void) ;
void    Right_Encoder_SaveConfig(void) ;
void    Right_Encoder_RestoreConfig(void) ;

CY_ISR_PROTO(Right_Encoder_ISR);


/***************************************
*           API Constants
***************************************/

#define Right_Encoder_ISR_NUMBER                 (Right_Encoder_isr__INTC_NUMBER)
#define Right_Encoder_ISR_PRIORITY               (Right_Encoder_isr__INTC_PRIOR_NUM)


/***************************************
*    Enumerated Types and Parameters
***************************************/

#define Right_Encoder_GLITCH_FILTERING           (1u)
#define Right_Encoder_INDEX_INPUT                (0u)


/***************************************
*    Initial Parameter Constants
***************************************/

#if (Right_Encoder_COUNTER_SIZE == 8u)    
    #define Right_Encoder_COUNTER_INIT_VALUE    (0x80u)
#else /* (Right_Encoder_COUNTER_SIZE == 16u) || (Right_Encoder_COUNTER_SIZE == 32u) */
    #define Right_Encoder_COUNTER_INIT_VALUE    (0x8000u)
#endif /* Right_Encoder_COUNTER_SIZE == 8u */   


/***************************************
*             Registers
***************************************/

#define Right_Encoder_STATUS_REG                 (* (reg8 *) Right_Encoder_bQuadDec_Stsreg__STATUS_REG)
#define Right_Encoder_STATUS_PTR                 (  (reg8 *) Right_Encoder_bQuadDec_Stsreg__STATUS_REG)
#define Right_Encoder_STATUS_MASK                (* (reg8 *) Right_Encoder_bQuadDec_Stsreg__MASK_REG)
#define Right_Encoder_STATUS_MASK_PTR            (  (reg8 *) Right_Encoder_bQuadDec_Stsreg__MASK_REG)
#define Right_Encoder_SR_AUX_CONTROL             (* (reg8 *) Right_Encoder_bQuadDec_Stsreg__STATUS_AUX_CTL_REG)
#define Right_Encoder_SR_AUX_CONTROL_PTR         (  (reg8 *) Right_Encoder_bQuadDec_Stsreg__STATUS_AUX_CTL_REG)


/***************************************
*        Register Constants
***************************************/

#define Right_Encoder_COUNTER_OVERFLOW_SHIFT     (0x00u)
#define Right_Encoder_COUNTER_UNDERFLOW_SHIFT    (0x01u)
#define Right_Encoder_COUNTER_RESET_SHIFT        (0x02u)
#define Right_Encoder_INVALID_IN_SHIFT           (0x03u)
#define Right_Encoder_COUNTER_OVERFLOW           (0x01u << Right_Encoder_COUNTER_OVERFLOW_SHIFT)
#define Right_Encoder_COUNTER_UNDERFLOW          (0x01u << Right_Encoder_COUNTER_UNDERFLOW_SHIFT)
#define Right_Encoder_COUNTER_RESET              (0x01u << Right_Encoder_COUNTER_RESET_SHIFT)
#define Right_Encoder_INVALID_IN                 (0x01u << Right_Encoder_INVALID_IN_SHIFT)

#define Right_Encoder_INTERRUPTS_ENABLE_SHIFT    (0x04u)
#define Right_Encoder_INTERRUPTS_ENABLE          (0x01u << Right_Encoder_INTERRUPTS_ENABLE_SHIFT)
#define Right_Encoder_INIT_INT_MASK              (0x0Fu)
#define Right_Encoder_DISABLE                    (0x00u)     

#endif /* CY_QUADRATURE_DECODER_Right_Encoder_H */


/* [] END OF FILE */
