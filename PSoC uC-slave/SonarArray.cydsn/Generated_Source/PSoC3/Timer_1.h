/*******************************************************************************
* File Name: Timer_1.h
* Version 2.20
*
*  Description:
*     Contains the function prototypes and constants available to the timer
*     user module.
*
*   Note:
*     None
*
********************************************************************************
* Copyright 2008-2011, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

#if !defined(CY_Timer_v2_20_Timer_1_H)
#define CY_Timer_v2_20_Timer_1_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h" /* For CyEnterCriticalSection() and CyExitCriticalSection() functions */

/***************************************
*   Conditional Compilation Parameters
***************************************/

/* PSoC3 ES2 or early */
#define Timer_1_PSOC3_ES2  ((CYDEV_CHIP_MEMBER_USED == CYDEV_CHIP_MEMBER_3A)    && \
                                    (CYDEV_CHIP_REVISION_USED <= CYDEV_CHIP_REVISION_3A_ES2))
/* PSoC5 ES1 or early */
#define Timer_1_PSOC5_ES1  ((CYDEV_CHIP_MEMBER_USED == CYDEV_CHIP_MEMBER_5A)    && \
                                    (CYDEV_CHIP_REVISION_USED <= CYDEV_CHIP_REVISION_5A_ES1))
/* PSoC3 ES3 or later */
#define Timer_1_PSOC3_ES3  ((CYDEV_CHIP_MEMBER_USED == CYDEV_CHIP_MEMBER_3A)    && \
                                    (CYDEV_CHIP_REVISION_USED > CYDEV_CHIP_REVISION_3A_ES2))
/* PSoC5 ES2 or later */
#define Timer_1_PSOC5_ES2  ((CYDEV_CHIP_MEMBER_USED == CYDEV_CHIP_MEMBER_5A)    && \
                                    (CYDEV_CHIP_REVISION_USED > CYDEV_CHIP_REVISION_5A_ES1))


/**************************************
*           Parameter Defaults
**************************************/

#define Timer_1_Resolution                 32u
#define Timer_1_UsingFixedFunction         0u
#define Timer_1_UsingHWCaptureCounter      0u
#define Timer_1_SoftwareCaptureMode        0u
#define Timer_1_SoftwareTriggerMode        0u
#define Timer_1_UsingHWEnable              0u
#define Timer_1_EnableTriggerMode          0u
#define Timer_1_InterruptOnCaptureCount    0u
#define Timer_1_RunModeUsed                0u
#define Timer_1_ControlRegRemoved          0u


/***************************************
*       Type defines
***************************************/
/**************************************************************************
 * Sleep Wakeup Backup structure for Timer Component
 *************************************************************************/
typedef struct Timer_1_backupStruct
{
    uint8 TimerEnableState;
    #if(!Timer_1_UsingFixedFunction)
        #if (Timer_1_PSOC3_ES2 || Timer_1_PSOC5_ES1)
            uint32 TimerUdb;                 /* Timer internal counter value      */
            uint32 TimerPeriod;              /* Timer Period value       */
            uint8 InterruptMaskValue;       /* Timer Compare Value */
            #if (Timer_1_UsingHWCaptureCounter)
                uint32 TimerCaptureCounter;  /* Timer Capture Counter Value */
            #endif /* variable declaration for backing up Capture Counter value*/
        #endif /* variables for non retention registers in PSoC3 ES2, PSoC5 ES1 */

        #if (Timer_1_PSOC3_ES3 || Timer_1_PSOC5_ES2)
            uint32 TimerUdb;
            uint8 InterruptMaskValue;
            #if (Timer_1_UsingHWCaptureCounter)
                uint32 TimerCaptureCounter;
            #endif /* variable declarations for backing up non retention registers in PSoC3 ES3 or PSoC5 ES2 */
        #endif 

        #if (!Timer_1_ControlRegRemoved)
        uint8 TimerControlRegister;
        #endif /* variable declaration for backing up enable state of the Timer */
    #endif /* define backup variables only for UDB implementation. Fixed function registers are all retention */
}Timer_1_backupStruct;


/***************************************
*       Function Prototypes
***************************************/

void    Timer_1_Start(void);
void    Timer_1_Stop(void) ;

void    Timer_1_SetInterruptMode(uint8 interruptMode) ;
uint8   Timer_1_ReadStatusRegister(void) ;
/* Deprecated function. Do not use this in future. Retained for backward compatibility */
#define Timer_1_GetInterruptSource() Timer_1_ReadStatusRegister()

#if(!Timer_1_ControlRegRemoved)
    uint8   Timer_1_ReadControlRegister(void) ;
    void    Timer_1_WriteControlRegister(uint8 control) \
        ;
#endif

uint32  Timer_1_ReadPeriod(void) ;
void    Timer_1_WritePeriod(uint32 period) \
    ;
uint32  Timer_1_ReadCounter(void) ;
void    Timer_1_WriteCounter(uint32 counter) \
    ;
uint32  Timer_1_ReadCapture(void) ;
void    Timer_1_SoftwareCapture(void) ;


#if(!Timer_1_UsingFixedFunction) /* UDB Prototypes */
    #if (Timer_1_SoftwareCaptureMode)
        void    Timer_1_SetCaptureMode(uint8 captureMode) ;
    #endif

    #if (Timer_1_SoftwareTriggerMode)
        void    Timer_1_SetTriggerMode(uint8 triggerMode) ;
    #endif
    #if (Timer_1_EnableTriggerMode)
        void    Timer_1_EnableTrigger(void) ;
        void    Timer_1_DisableTrigger(void) ;
    #endif

    #if(Timer_1_InterruptOnCaptureCount)
        #if(!Timer_1_ControlRegRemoved)
            void    Timer_1_SetInterruptCount(uint8 interruptCount) \
                ;
        #endif
    #endif

    #if (Timer_1_UsingHWCaptureCounter)
        void    Timer_1_SetCaptureCount(uint8 captureCount) \
            ;
        uint8   Timer_1_ReadCaptureCount(void) ;
    #endif

    void Timer_1_ClearFIFO(void) ;
#endif /* UDB Prototypes */

/* Sleep Retention APIs */
void Timer_1_Init(void)          ;
void Timer_1_Enable(void)        ;
void Timer_1_SaveConfig(void);
void Timer_1_RestoreConfig(void) ;
void Timer_1_Sleep(void);
void Timer_1_Wakeup(void)        ;


/***************************************
*   Enumerated Types and Parameters
***************************************/

/* Enumerated Type B_Timer__CaptureModes, Used in Capture Mode */
#define Timer_1__B_TIMER__CM_NONE 0
#define Timer_1__B_TIMER__CM_RISINGEDGE 1
#define Timer_1__B_TIMER__CM_FALLINGEDGE 2
#define Timer_1__B_TIMER__CM_EITHEREDGE 3
#define Timer_1__B_TIMER__CM_SOFTWARE 4



/* Enumerated Type B_Timer__TriggerModes, Used in Trigger Mode */
#define Timer_1__B_TIMER__TM_NONE 0x00u
#define Timer_1__B_TIMER__TM_RISINGEDGE 0x04u
#define Timer_1__B_TIMER__TM_FALLINGEDGE 0x08u
#define Timer_1__B_TIMER__TM_EITHEREDGE 0x0Cu
#define Timer_1__B_TIMER__TM_SOFTWARE 0x10u


/***************************************
*    Initialial Parameter Constants
***************************************/

#define Timer_1_INIT_PERIOD             1199999u
#define Timer_1_INIT_CAPTURE_MODE       (0u << Timer_1_CTRL_CAP_MODE_SHIFT)
#define Timer_1_INIT_TRIGGER_MODE       (0u << Timer_1_CTRL_TRIG_MODE_SHIFT)
#if (Timer_1_UsingFixedFunction)
    #define Timer_1_INIT_INTERRUPT_MODE ((1u << Timer_1_STATUS_TC_INT_MASK_SHIFT) | \
                                                  (0 << Timer_1_STATUS_CAPTURE_INT_MASK_SHIFT))
#else
    #define Timer_1_INIT_INTERRUPT_MODE ((1u << Timer_1_STATUS_TC_INT_MASK_SHIFT) | \
                                                  (0 << Timer_1_STATUS_CAPTURE_INT_MASK_SHIFT) | \
                                                  (0 << Timer_1_STATUS_FIFOFULL_INT_MASK_SHIFT))
#endif
#define Timer_1_INIT_CAPTURE_COUNT      (2u)
#define Timer_1_INIT_INT_CAPTURE_COUNT  ((1u - 1) << Timer_1_CTRL_INTCNT_SHIFT)


/***************************************
*           Registers
***************************************/

#if (Timer_1_UsingFixedFunction) /* Implementation Specific Registers and Register Constants */


    /***************************************
    *    Fixed Function Registers
    ***************************************/

    #define Timer_1_STATUS         (*(reg8 *) Timer_1_TimerHW__SR0 )
    /* In Fixed Function Block Status and Mask are the same register */
    #define Timer_1_STATUS_MASK    (*(reg8 *) Timer_1_TimerHW__SR0 )
    #define Timer_1_CONTROL        (*(reg8 *) Timer_1_TimerHW__CFG0)
    #define Timer_1_CONTROL2       (*(reg8 *) Timer_1_TimerHW__CFG1)
    #define Timer_1_CONTROL2_PTR   ( (reg8 *) Timer_1_TimerHW__CFG1)
    #define Timer_1_RT1            (*(reg8 *) Timer_1_TimerHW__RT1)
    #define Timer_1_RT1_PTR        ( (reg8 *) Timer_1_TimerHW__RT1)

    #if (Timer_1_PSOC3_ES3 || Timer_1_PSOC5_ES2)
        #define Timer_1_CONTROL3       (*(reg8 *) Timer_1_TimerHW__CFG2)
        #define Timer_1_CONTROL3_PTR   ( (reg8 *) Timer_1_TimerHW__CFG2)
    #endif
    #define Timer_1_GLOBAL_ENABLE  (*(reg8 *) Timer_1_TimerHW__PM_ACT_CFG)
    #define Timer_1_GLOBAL_STBY_ENABLE  (*(reg8 *) Timer_1_TimerHW__PM_STBY_CFG)

    #define Timer_1_CAPTURE_LSB         (* (reg16 *) Timer_1_TimerHW__CAP0 )
    #define Timer_1_CAPTURE_LSB_PTR       ((reg16 *) Timer_1_TimerHW__CAP0 )
    #define Timer_1_PERIOD_LSB          (* (reg16 *) Timer_1_TimerHW__PER0 )
    #define Timer_1_PERIOD_LSB_PTR        ((reg16 *) Timer_1_TimerHW__PER0 )
    #define Timer_1_COUNTER_LSB         (* (reg16 *) Timer_1_TimerHW__CNT_CMP0 )
    #define Timer_1_COUNTER_LSB_PTR       ((reg16 *) Timer_1_TimerHW__CNT_CMP0 )

    /***************************************
    *    Register Constants
    ***************************************/

    /* Fixed Function Block Chosen */
    #define Timer_1_BLOCK_EN_MASK                     Timer_1_TimerHW__PM_ACT_MSK
    #define Timer_1_BLOCK_STBY_EN_MASK                Timer_1_TimerHW__PM_STBY_MSK

    /* Control Register Bit Locations */
    /* Interrupt Count - Not valid for Fixed Function Block */
    #define Timer_1_CTRL_INTCNT_SHIFT                  0x00u
    /* Trigger Polarity - Not valid for Fixed Function Block */
    #define Timer_1_CTRL_TRIG_MODE_SHIFT               0x00u
    /* Trigger Enable - Not valid for Fixed Function Block */
    #define Timer_1_CTRL_TRIG_EN_SHIFT                 0x00u
    /* Capture Polarity - Not valid for Fixed Function Block */
    #define Timer_1_CTRL_CAP_MODE_SHIFT                0x00u
    /* Timer Enable - As defined in Register Map, part of TMRX_CFG0 register */
    #define Timer_1_CTRL_ENABLE_SHIFT                  0x00u

    /* Control Register Bit Masks */
    #define Timer_1_CTRL_ENABLE                        (0x01u << Timer_1_CTRL_ENABLE_SHIFT)

    /* Control2 Register Bit Masks */
    /* As defined in Register Map, Part of the TMRX_CFG1 register */
    #define Timer_1_CTRL2_IRQ_SEL_SHIFT                 0x00u
    #define Timer_1_CTRL2_IRQ_SEL                      (0x01u << Timer_1_CTRL2_IRQ_SEL_SHIFT)

    #if (Timer_1_PSOC3_ES2 || Timer_1_PSOC5_ES1)
        /* Use CFG1 Mode bits to set run mode */
        /* As defined by Verilog Implementation */
        #define Timer_1_CTRL_MODE_SHIFT                     0x01u
        #define Timer_1_CTRL_MODE_MASK                     (0x07u << Timer_1_CTRL_MODE_SHIFT)
    #endif
    #if (Timer_1_PSOC3_ES3 || Timer_1_PSOC5_ES2)
        /* Control3 Register Bit Locations */
        #define Timer_1_CTRL_RCOD_SHIFT        0x02u
        #define Timer_1_CTRL_ENBL_SHIFT        0x00u
        #define Timer_1_CTRL_MODE_SHIFT        0x00u

        /* Control3 Register Bit Masks */
        #define Timer_1_CTRL_RCOD_MASK  (0x03u << Timer_1_CTRL_RCOD_SHIFT) /* ROD and COD bit masks */
        #define Timer_1_CTRL_ENBL_MASK  (0x80u << Timer_1_CTRL_ENBL_SHIFT) /* HW_EN bit mask */
        #define Timer_1_CTRL_MODE_MASK  (0x03u << Timer_1_CTRL_MODE_SHIFT) /* Run mode bit mask */

        #define Timer_1_CTRL_RCOD       (0x03u << Timer_1_CTRL_RCOD_SHIFT)
        #define Timer_1_CTRL_ENBL       (0x80u << Timer_1_CTRL_ENBL_SHIFT)
    #endif

    /*RT1 Synch Constants: Applicable for PSoC3 ES2/PSoC3 ES3 PSoC5 ES2*/
    #define Timer_1_RT1_SHIFT                       0x04u
    /* Sync TC and CMP bit masks */
    #define Timer_1_RT1_MASK                        (0x03u << Timer_1_RT1_SHIFT)
    #define Timer_1_SYNC                            (0x03u << Timer_1_RT1_SHIFT)
    #define Timer_1_SYNCDSI_SHIFT                   0x00u
    /* Sync all DSI inputs with Mask  */
    #define Timer_1_SYNCDSI_MASK                    (0x0Fu << Timer_1_SYNCDSI_SHIFT)
    /* Sync all DSI inputs */
    #define Timer_1_SYNCDSI_EN                      (0x0Fu << Timer_1_SYNCDSI_SHIFT)

    #define Timer_1_CTRL_MODE_PULSEWIDTH            (0x01u << Timer_1_CTRL_MODE_SHIFT)
    #define Timer_1_CTRL_MODE_PERIOD                (0x02u << Timer_1_CTRL_MODE_SHIFT)
    #define Timer_1_CTRL_MODE_CONTINUOUS            (0x00u << Timer_1_CTRL_MODE_SHIFT)

    /* Status Register Bit Locations */
    /* As defined in Register Map, part of TMRX_SR0 register */
    #define Timer_1_STATUS_TC_SHIFT                 0x07u
    /* As defined in Register Map, part of TMRX_SR0 register, Shared with Compare Status */
    #define Timer_1_STATUS_CAPTURE_SHIFT            0x06u
    /* As defined in Register Map, part of TMRX_SR0 register */
    #define Timer_1_STATUS_TC_INT_MASK_SHIFT        (Timer_1_STATUS_TC_SHIFT - 4)
    /* As defined in Register Map, part of TMRX_SR0 register, Shared with Compare Status */
    #define Timer_1_STATUS_CAPTURE_INT_MASK_SHIFT   (Timer_1_STATUS_CAPTURE_SHIFT - 4)

    /* Status Register Bit Masks */
    #define Timer_1_STATUS_TC                       (0x01u << Timer_1_STATUS_TC_SHIFT)
    #define Timer_1_STATUS_CAPTURE                  (0x01u << Timer_1_STATUS_CAPTURE_SHIFT)
    /* Interrupt Enable Bit-Mask for interrupt on TC */
    #define Timer_1_STATUS_TC_INT_MASK              (0x01u << Timer_1_STATUS_TC_INT_MASK_SHIFT)
    /* Interrupt Enable Bit-Mask for interrupt on Capture */
    #define Timer_1_STATUS_CAPTURE_INT_MASK         (0x01u << Timer_1_STATUS_CAPTURE_INT_MASK_SHIFT)

#else   /* UDB Registers and Register Constants */


    /***************************************
    *           UDB Registers
    ***************************************/

    #define Timer_1_STATUS              (* (reg8 *) Timer_1_TimerUDB_rstSts_stsreg__STATUS_REG )
    #define Timer_1_STATUS_MASK         (* (reg8 *) Timer_1_TimerUDB_rstSts_stsreg__MASK_REG)
    #define Timer_1_STATUS_AUX_CTRL     (* (reg8 *) Timer_1_TimerUDB_rstSts_stsreg__STATUS_AUX_CTL_REG)
    #define Timer_1_CONTROL             (* (reg8 *) Timer_1_TimerUDB_sCTRLReg_SyncCtl_ctrlreg__CONTROL_REG )

    #define Timer_1_CAPTURE_LSB         (* (reg32 *) Timer_1_TimerUDB_sT32_timerdp_u0__F0_REG )
    #define Timer_1_CAPTURE_LSB_PTR       ((reg32 *) Timer_1_TimerUDB_sT32_timerdp_u0__F0_REG )
    #define Timer_1_PERIOD_LSB          (* (reg32 *) Timer_1_TimerUDB_sT32_timerdp_u0__D0_REG )
    #define Timer_1_PERIOD_LSB_PTR        ((reg32 *) Timer_1_TimerUDB_sT32_timerdp_u0__D0_REG )
    #define Timer_1_COUNTER_LSB         (* (reg32 *) Timer_1_TimerUDB_sT32_timerdp_u0__A0_REG )
    #define Timer_1_COUNTER_LSB_PTR       ((reg32 *) Timer_1_TimerUDB_sT32_timerdp_u0__A0_REG )

    #if (Timer_1_UsingHWCaptureCounter)
        #define Timer_1_CAP_COUNT              (*(reg8 *) Timer_1_TimerUDB_sCapCount_counter__PERIOD_REG )
        #define Timer_1_CAP_COUNT_PTR          ( (reg8 *) Timer_1_TimerUDB_sCapCount_counter__PERIOD_REG )
        #define Timer_1_CAPTURE_COUNT_CTRL     (*(reg8 *) Timer_1_TimerUDB_sCapCount_counter__CONTROL_AUX_CTL_REG )
        #define Timer_1_CAPTURE_COUNT_CTRL_PTR ( (reg8 *) Timer_1_TimerUDB_sCapCount_counter__CONTROL_AUX_CTL_REG )
    #endif


    /***************************************
    *       Register Constants
    ***************************************/

    /* Control Register Bit Locations */
    #define Timer_1_CTRL_INTCNT_SHIFT              0x00u       /* As defined by Verilog Implementation */
    #define Timer_1_CTRL_TRIG_MODE_SHIFT           0x02u       /* As defined by Verilog Implementation */
    #define Timer_1_CTRL_TRIG_EN_SHIFT             0x04u       /* As defined by Verilog Implementation */
    #define Timer_1_CTRL_CAP_MODE_SHIFT            0x05u       /* As defined by Verilog Implementation */
    #define Timer_1_CTRL_ENABLE_SHIFT              0x07u       /* As defined by Verilog Implementation */

    /* Control Register Bit Masks */
    #define Timer_1_CTRL_INTCNT_MASK               (0x03u << Timer_1_CTRL_INTCNT_SHIFT)
    #define Timer_1_CTRL_TRIG_MODE_MASK            (0x03u << Timer_1_CTRL_TRIG_MODE_SHIFT)
    #define Timer_1_CTRL_TRIG_EN                   (0x01u << Timer_1_CTRL_TRIG_EN_SHIFT)
    #define Timer_1_CTRL_CAP_MODE_MASK             (0x03u << Timer_1_CTRL_CAP_MODE_SHIFT)
    #define Timer_1_CTRL_ENABLE                    (0x01u << Timer_1_CTRL_ENABLE_SHIFT)

    /* Bit Counter (7-bit) Control Register Bit Definitions */
    /* As defined by the Register map for the AUX Control Register */
    #define Timer_1_CNTR_ENABLE                    0x20u

    /* Status Register Bit Locations */
    #define Timer_1_STATUS_TC_SHIFT                0x00u  /* As defined by Verilog Implementation */
    #define Timer_1_STATUS_CAPTURE_SHIFT           0x01u  /* As defined by Verilog Implementation */
    #define Timer_1_STATUS_TC_INT_MASK_SHIFT       Timer_1_STATUS_TC_SHIFT
    #define Timer_1_STATUS_CAPTURE_INT_MASK_SHIFT  Timer_1_STATUS_CAPTURE_SHIFT
    #define Timer_1_STATUS_FIFOFULL_SHIFT          0x02u  /* As defined by Verilog Implementation */
    #define Timer_1_STATUS_FIFONEMP_SHIFT          0x03u  /* As defined by Verilog Implementation */
    #define Timer_1_STATUS_FIFOFULL_INT_MASK_SHIFT Timer_1_STATUS_FIFOFULL_SHIFT

    /* Status Register Bit Masks */
    /* Sticky TC Event Bit-Mask */
    #define Timer_1_STATUS_TC                      (0x01u << Timer_1_STATUS_TC_SHIFT)
    /* Sticky Capture Event Bit-Mask */
    #define Timer_1_STATUS_CAPTURE                 (0x01u << Timer_1_STATUS_CAPTURE_SHIFT)
    /* Interrupt Enable Bit-Mask */
    #define Timer_1_STATUS_TC_INT_MASK             (0x01u << Timer_1_STATUS_TC_SHIFT)
    /* Interrupt Enable Bit-Mask */
    #define Timer_1_STATUS_CAPTURE_INT_MASK        (0x01u << Timer_1_STATUS_CAPTURE_SHIFT)
    /* NOT-Sticky FIFO Full Bit-Mask */
    #define Timer_1_STATUS_FIFOFULL                (0x01u << Timer_1_STATUS_FIFOFULL_SHIFT)
    /* NOT-Sticky FIFO Not Empty Bit-Mask */
    #define Timer_1_STATUS_FIFONEMP                (0x01u << Timer_1_STATUS_FIFONEMP_SHIFT)
    /* Interrupt Enable Bit-Mask */
    #define Timer_1_STATUS_FIFOFULL_INT_MASK       (0x01u << Timer_1_STATUS_FIFOFULL_SHIFT)

    #define Timer_1_STATUS_ACTL_INT_EN             0x10u   /* As defined for the ACTL Register */

    /* Datapath Auxillary Control Register definitions */
    #define Timer_1_AUX_CTRL_FIFO0_CLR             0x01u   /* As defined by Register map */
    #define Timer_1_AUX_CTRL_FIFO1_CLR             0x02u   /* As defined by Register map */
    #define Timer_1_AUX_CTRL_FIFO0_LVL             0x04u   /* As defined by Register map */
    #define Timer_1_AUX_CTRL_FIFO1_LVL             0x08u   /* As defined by Register map */
    #define Timer_1_STATUS_ACTL_INT_EN_MASK        0x10u   /* As defined for the ACTL Register */

#endif /* Implementation Specific Registers and Register Constants */

#endif  /* CY_Timer_v2_20_Timer_1_H */


/* [] END OF FILE */
