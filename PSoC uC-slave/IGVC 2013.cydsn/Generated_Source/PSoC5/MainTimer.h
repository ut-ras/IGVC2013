/*******************************************************************************
* File Name: MainTimer.h
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

#if !defined(CY_Timer_v2_20_MainTimer_H)
#define CY_Timer_v2_20_MainTimer_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h" /* For CyEnterCriticalSection() and CyExitCriticalSection() functions */

/***************************************
*   Conditional Compilation Parameters
***************************************/

/* PSoC3 ES2 or early */
#define MainTimer_PSOC3_ES2  ((CYDEV_CHIP_MEMBER_USED == CYDEV_CHIP_MEMBER_3A)    && \
                                    (CYDEV_CHIP_REVISION_USED <= CYDEV_CHIP_REVISION_3A_ES2))
/* PSoC5 ES1 or early */
#define MainTimer_PSOC5_ES1  ((CYDEV_CHIP_MEMBER_USED == CYDEV_CHIP_MEMBER_5A)    && \
                                    (CYDEV_CHIP_REVISION_USED <= CYDEV_CHIP_REVISION_5A_ES1))
/* PSoC3 ES3 or later */
#define MainTimer_PSOC3_ES3  ((CYDEV_CHIP_MEMBER_USED == CYDEV_CHIP_MEMBER_3A)    && \
                                    (CYDEV_CHIP_REVISION_USED > CYDEV_CHIP_REVISION_3A_ES2))
/* PSoC5 ES2 or later */
#define MainTimer_PSOC5_ES2  ((CYDEV_CHIP_MEMBER_USED == CYDEV_CHIP_MEMBER_5A)    && \
                                    (CYDEV_CHIP_REVISION_USED > CYDEV_CHIP_REVISION_5A_ES1))


/**************************************
*           Parameter Defaults
**************************************/

#define MainTimer_Resolution                 16u
#define MainTimer_UsingFixedFunction         1u
#define MainTimer_UsingHWCaptureCounter      0u
#define MainTimer_SoftwareCaptureMode        0u
#define MainTimer_SoftwareTriggerMode        0u
#define MainTimer_UsingHWEnable              0u
#define MainTimer_EnableTriggerMode          0u
#define MainTimer_InterruptOnCaptureCount    0u
#define MainTimer_RunModeUsed                0u
#define MainTimer_ControlRegRemoved          0u


/***************************************
*       Type defines
***************************************/
/**************************************************************************
 * Sleep Wakeup Backup structure for Timer Component
 *************************************************************************/
typedef struct MainTimer_backupStruct
{
    uint8 TimerEnableState;
    #if(!MainTimer_UsingFixedFunction)
        #if (MainTimer_PSOC3_ES2 || MainTimer_PSOC5_ES1)
            uint16 TimerUdb;                 /* Timer internal counter value      */
            uint16 TimerPeriod;              /* Timer Period value       */
            uint8 InterruptMaskValue;       /* Timer Compare Value */
            #if (MainTimer_UsingHWCaptureCounter)
                uint16 TimerCaptureCounter;  /* Timer Capture Counter Value */
            #endif /* variable declaration for backing up Capture Counter value*/
        #endif /* variables for non retention registers in PSoC3 ES2, PSoC5 ES1 */

        #if (MainTimer_PSOC3_ES3 || MainTimer_PSOC5_ES2)
            uint16 TimerUdb;
            uint8 InterruptMaskValue;
            #if (MainTimer_UsingHWCaptureCounter)
                uint16 TimerCaptureCounter;
            #endif /* variable declarations for backing up non retention registers in PSoC3 ES3 or PSoC5 ES2 */
        #endif 

        #if (!MainTimer_ControlRegRemoved)
        uint8 TimerControlRegister;
        #endif /* variable declaration for backing up enable state of the Timer */
    #endif /* define backup variables only for UDB implementation. Fixed function registers are all retention */
}MainTimer_backupStruct;


/***************************************
*       Function Prototypes
***************************************/

void    MainTimer_Start(void);
void    MainTimer_Stop(void) ;

void    MainTimer_SetInterruptMode(uint8 interruptMode) ;
uint8   MainTimer_ReadStatusRegister(void) ;
/* Deprecated function. Do not use this in future. Retained for backward compatibility */
#define MainTimer_GetInterruptSource() MainTimer_ReadStatusRegister()

#if(!MainTimer_ControlRegRemoved)
    uint8   MainTimer_ReadControlRegister(void) ;
    void    MainTimer_WriteControlRegister(uint8 control) \
        ;
#endif

uint16  MainTimer_ReadPeriod(void) ;
void    MainTimer_WritePeriod(uint16 period) \
    ;
uint16  MainTimer_ReadCounter(void) ;
void    MainTimer_WriteCounter(uint16 counter) \
    ;
uint16  MainTimer_ReadCapture(void) ;
void    MainTimer_SoftwareCapture(void) ;


#if(!MainTimer_UsingFixedFunction) /* UDB Prototypes */
    #if (MainTimer_SoftwareCaptureMode)
        void    MainTimer_SetCaptureMode(uint8 captureMode) ;
    #endif

    #if (MainTimer_SoftwareTriggerMode)
        void    MainTimer_SetTriggerMode(uint8 triggerMode) ;
    #endif
    #if (MainTimer_EnableTriggerMode)
        void    MainTimer_EnableTrigger(void) ;
        void    MainTimer_DisableTrigger(void) ;
    #endif

    #if(MainTimer_InterruptOnCaptureCount)
        #if(!MainTimer_ControlRegRemoved)
            void    MainTimer_SetInterruptCount(uint8 interruptCount) \
                ;
        #endif
    #endif

    #if (MainTimer_UsingHWCaptureCounter)
        void    MainTimer_SetCaptureCount(uint8 captureCount) \
            ;
        uint8   MainTimer_ReadCaptureCount(void) ;
    #endif

    void MainTimer_ClearFIFO(void) ;
#endif /* UDB Prototypes */

/* Sleep Retention APIs */
void MainTimer_Init(void)          ;
void MainTimer_Enable(void)        ;
void MainTimer_SaveConfig(void);
void MainTimer_RestoreConfig(void) ;
void MainTimer_Sleep(void);
void MainTimer_Wakeup(void)        ;


/***************************************
*   Enumerated Types and Parameters
***************************************/

/* Enumerated Type B_Timer__CaptureModes, Used in Capture Mode */
#define MainTimer__B_TIMER__CM_NONE 0
#define MainTimer__B_TIMER__CM_RISINGEDGE 1
#define MainTimer__B_TIMER__CM_FALLINGEDGE 2
#define MainTimer__B_TIMER__CM_EITHEREDGE 3
#define MainTimer__B_TIMER__CM_SOFTWARE 4



/* Enumerated Type B_Timer__TriggerModes, Used in Trigger Mode */
#define MainTimer__B_TIMER__TM_NONE 0x00u
#define MainTimer__B_TIMER__TM_RISINGEDGE 0x04u
#define MainTimer__B_TIMER__TM_FALLINGEDGE 0x08u
#define MainTimer__B_TIMER__TM_EITHEREDGE 0x0Cu
#define MainTimer__B_TIMER__TM_SOFTWARE 0x10u


/***************************************
*    Initialial Parameter Constants
***************************************/

#define MainTimer_INIT_PERIOD             32999u
#define MainTimer_INIT_CAPTURE_MODE       (1u << MainTimer_CTRL_CAP_MODE_SHIFT)
#define MainTimer_INIT_TRIGGER_MODE       (0u << MainTimer_CTRL_TRIG_MODE_SHIFT)
#if (MainTimer_UsingFixedFunction)
    #define MainTimer_INIT_INTERRUPT_MODE ((1u << MainTimer_STATUS_TC_INT_MASK_SHIFT) | \
                                                  (0 << MainTimer_STATUS_CAPTURE_INT_MASK_SHIFT))
#else
    #define MainTimer_INIT_INTERRUPT_MODE ((1u << MainTimer_STATUS_TC_INT_MASK_SHIFT) | \
                                                  (0 << MainTimer_STATUS_CAPTURE_INT_MASK_SHIFT) | \
                                                  (0 << MainTimer_STATUS_FIFOFULL_INT_MASK_SHIFT))
#endif
#define MainTimer_INIT_CAPTURE_COUNT      (2u)
#define MainTimer_INIT_INT_CAPTURE_COUNT  ((1u - 1) << MainTimer_CTRL_INTCNT_SHIFT)


/***************************************
*           Registers
***************************************/

#if (MainTimer_UsingFixedFunction) /* Implementation Specific Registers and Register Constants */


    /***************************************
    *    Fixed Function Registers
    ***************************************/

    #define MainTimer_STATUS         (*(reg8 *) MainTimer_TimerHW__SR0 )
    /* In Fixed Function Block Status and Mask are the same register */
    #define MainTimer_STATUS_MASK    (*(reg8 *) MainTimer_TimerHW__SR0 )
    #define MainTimer_CONTROL        (*(reg8 *) MainTimer_TimerHW__CFG0)
    #define MainTimer_CONTROL2       (*(reg8 *) MainTimer_TimerHW__CFG1)
    #define MainTimer_CONTROL2_PTR   ( (reg8 *) MainTimer_TimerHW__CFG1)
    #define MainTimer_RT1            (*(reg8 *) MainTimer_TimerHW__RT1)
    #define MainTimer_RT1_PTR        ( (reg8 *) MainTimer_TimerHW__RT1)

    #if (MainTimer_PSOC3_ES3 || MainTimer_PSOC5_ES2)
        #define MainTimer_CONTROL3       (*(reg8 *) MainTimer_TimerHW__CFG2)
        #define MainTimer_CONTROL3_PTR   ( (reg8 *) MainTimer_TimerHW__CFG2)
    #endif
    #define MainTimer_GLOBAL_ENABLE  (*(reg8 *) MainTimer_TimerHW__PM_ACT_CFG)
    #define MainTimer_GLOBAL_STBY_ENABLE  (*(reg8 *) MainTimer_TimerHW__PM_STBY_CFG)

    #define MainTimer_CAPTURE_LSB         (* (reg16 *) MainTimer_TimerHW__CAP0 )
    #define MainTimer_CAPTURE_LSB_PTR       ((reg16 *) MainTimer_TimerHW__CAP0 )
    #define MainTimer_PERIOD_LSB          (* (reg16 *) MainTimer_TimerHW__PER0 )
    #define MainTimer_PERIOD_LSB_PTR        ((reg16 *) MainTimer_TimerHW__PER0 )
    #define MainTimer_COUNTER_LSB         (* (reg16 *) MainTimer_TimerHW__CNT_CMP0 )
    #define MainTimer_COUNTER_LSB_PTR       ((reg16 *) MainTimer_TimerHW__CNT_CMP0 )

    /***************************************
    *    Register Constants
    ***************************************/

    /* Fixed Function Block Chosen */
    #define MainTimer_BLOCK_EN_MASK                     MainTimer_TimerHW__PM_ACT_MSK
    #define MainTimer_BLOCK_STBY_EN_MASK                MainTimer_TimerHW__PM_STBY_MSK

    /* Control Register Bit Locations */
    /* Interrupt Count - Not valid for Fixed Function Block */
    #define MainTimer_CTRL_INTCNT_SHIFT                  0x00u
    /* Trigger Polarity - Not valid for Fixed Function Block */
    #define MainTimer_CTRL_TRIG_MODE_SHIFT               0x00u
    /* Trigger Enable - Not valid for Fixed Function Block */
    #define MainTimer_CTRL_TRIG_EN_SHIFT                 0x00u
    /* Capture Polarity - Not valid for Fixed Function Block */
    #define MainTimer_CTRL_CAP_MODE_SHIFT                0x00u
    /* Timer Enable - As defined in Register Map, part of TMRX_CFG0 register */
    #define MainTimer_CTRL_ENABLE_SHIFT                  0x00u

    /* Control Register Bit Masks */
    #define MainTimer_CTRL_ENABLE                        (0x01u << MainTimer_CTRL_ENABLE_SHIFT)

    /* Control2 Register Bit Masks */
    /* As defined in Register Map, Part of the TMRX_CFG1 register */
    #define MainTimer_CTRL2_IRQ_SEL_SHIFT                 0x00u
    #define MainTimer_CTRL2_IRQ_SEL                      (0x01u << MainTimer_CTRL2_IRQ_SEL_SHIFT)

    #if (MainTimer_PSOC3_ES2 || MainTimer_PSOC5_ES1)
        /* Use CFG1 Mode bits to set run mode */
        /* As defined by Verilog Implementation */
        #define MainTimer_CTRL_MODE_SHIFT                     0x01u
        #define MainTimer_CTRL_MODE_MASK                     (0x07u << MainTimer_CTRL_MODE_SHIFT)
    #endif
    #if (MainTimer_PSOC3_ES3 || MainTimer_PSOC5_ES2)
        /* Control3 Register Bit Locations */
        #define MainTimer_CTRL_RCOD_SHIFT        0x02u
        #define MainTimer_CTRL_ENBL_SHIFT        0x00u
        #define MainTimer_CTRL_MODE_SHIFT        0x00u

        /* Control3 Register Bit Masks */
        #define MainTimer_CTRL_RCOD_MASK  (0x03u << MainTimer_CTRL_RCOD_SHIFT) /* ROD and COD bit masks */
        #define MainTimer_CTRL_ENBL_MASK  (0x80u << MainTimer_CTRL_ENBL_SHIFT) /* HW_EN bit mask */
        #define MainTimer_CTRL_MODE_MASK  (0x03u << MainTimer_CTRL_MODE_SHIFT) /* Run mode bit mask */

        #define MainTimer_CTRL_RCOD       (0x03u << MainTimer_CTRL_RCOD_SHIFT)
        #define MainTimer_CTRL_ENBL       (0x80u << MainTimer_CTRL_ENBL_SHIFT)
    #endif

    /*RT1 Synch Constants: Applicable for PSoC3 ES2/PSoC3 ES3 PSoC5 ES2*/
    #define MainTimer_RT1_SHIFT                       0x04u
    /* Sync TC and CMP bit masks */
    #define MainTimer_RT1_MASK                        (0x03u << MainTimer_RT1_SHIFT)
    #define MainTimer_SYNC                            (0x03u << MainTimer_RT1_SHIFT)
    #define MainTimer_SYNCDSI_SHIFT                   0x00u
    /* Sync all DSI inputs with Mask  */
    #define MainTimer_SYNCDSI_MASK                    (0x0Fu << MainTimer_SYNCDSI_SHIFT)
    /* Sync all DSI inputs */
    #define MainTimer_SYNCDSI_EN                      (0x0Fu << MainTimer_SYNCDSI_SHIFT)

    #define MainTimer_CTRL_MODE_PULSEWIDTH            (0x01u << MainTimer_CTRL_MODE_SHIFT)
    #define MainTimer_CTRL_MODE_PERIOD                (0x02u << MainTimer_CTRL_MODE_SHIFT)
    #define MainTimer_CTRL_MODE_CONTINUOUS            (0x00u << MainTimer_CTRL_MODE_SHIFT)

    /* Status Register Bit Locations */
    /* As defined in Register Map, part of TMRX_SR0 register */
    #define MainTimer_STATUS_TC_SHIFT                 0x07u
    /* As defined in Register Map, part of TMRX_SR0 register, Shared with Compare Status */
    #define MainTimer_STATUS_CAPTURE_SHIFT            0x06u
    /* As defined in Register Map, part of TMRX_SR0 register */
    #define MainTimer_STATUS_TC_INT_MASK_SHIFT        (MainTimer_STATUS_TC_SHIFT - 4)
    /* As defined in Register Map, part of TMRX_SR0 register, Shared with Compare Status */
    #define MainTimer_STATUS_CAPTURE_INT_MASK_SHIFT   (MainTimer_STATUS_CAPTURE_SHIFT - 4)

    /* Status Register Bit Masks */
    #define MainTimer_STATUS_TC                       (0x01u << MainTimer_STATUS_TC_SHIFT)
    #define MainTimer_STATUS_CAPTURE                  (0x01u << MainTimer_STATUS_CAPTURE_SHIFT)
    /* Interrupt Enable Bit-Mask for interrupt on TC */
    #define MainTimer_STATUS_TC_INT_MASK              (0x01u << MainTimer_STATUS_TC_INT_MASK_SHIFT)
    /* Interrupt Enable Bit-Mask for interrupt on Capture */
    #define MainTimer_STATUS_CAPTURE_INT_MASK         (0x01u << MainTimer_STATUS_CAPTURE_INT_MASK_SHIFT)

#else   /* UDB Registers and Register Constants */


    /***************************************
    *           UDB Registers
    ***************************************/

    #define MainTimer_STATUS              (* (reg8 *) MainTimer_TimerUDB_nrstSts_stsreg__STATUS_REG )
    #define MainTimer_STATUS_MASK         (* (reg8 *) MainTimer_TimerUDB_nrstSts_stsreg__MASK_REG)
    #define MainTimer_STATUS_AUX_CTRL     (* (reg8 *) MainTimer_TimerUDB_nrstSts_stsreg__STATUS_AUX_CTL_REG)
    #define MainTimer_CONTROL             (* (reg8 *) MainTimer_TimerUDB_sCTRLReg_AsyncCtl_ctrlreg__CONTROL_REG )

    #define MainTimer_CAPTURE_LSB         (* (reg16 *) MainTimer_TimerUDB_sT16_timerdp_u0__F0_REG )
    #define MainTimer_CAPTURE_LSB_PTR       ((reg16 *) MainTimer_TimerUDB_sT16_timerdp_u0__F0_REG )
    #define MainTimer_PERIOD_LSB          (* (reg16 *) MainTimer_TimerUDB_sT16_timerdp_u0__D0_REG )
    #define MainTimer_PERIOD_LSB_PTR        ((reg16 *) MainTimer_TimerUDB_sT16_timerdp_u0__D0_REG )
    #define MainTimer_COUNTER_LSB         (* (reg16 *) MainTimer_TimerUDB_sT16_timerdp_u0__A0_REG )
    #define MainTimer_COUNTER_LSB_PTR       ((reg16 *) MainTimer_TimerUDB_sT16_timerdp_u0__A0_REG )

    #if (MainTimer_UsingHWCaptureCounter)
        #define MainTimer_CAP_COUNT              (*(reg8 *) MainTimer_TimerUDB_sCapCount_counter__PERIOD_REG )
        #define MainTimer_CAP_COUNT_PTR          ( (reg8 *) MainTimer_TimerUDB_sCapCount_counter__PERIOD_REG )
        #define MainTimer_CAPTURE_COUNT_CTRL     (*(reg8 *) MainTimer_TimerUDB_sCapCount_counter__CONTROL_AUX_CTL_REG )
        #define MainTimer_CAPTURE_COUNT_CTRL_PTR ( (reg8 *) MainTimer_TimerUDB_sCapCount_counter__CONTROL_AUX_CTL_REG )
    #endif


    /***************************************
    *       Register Constants
    ***************************************/

    /* Control Register Bit Locations */
    #define MainTimer_CTRL_INTCNT_SHIFT              0x00u       /* As defined by Verilog Implementation */
    #define MainTimer_CTRL_TRIG_MODE_SHIFT           0x02u       /* As defined by Verilog Implementation */
    #define MainTimer_CTRL_TRIG_EN_SHIFT             0x04u       /* As defined by Verilog Implementation */
    #define MainTimer_CTRL_CAP_MODE_SHIFT            0x05u       /* As defined by Verilog Implementation */
    #define MainTimer_CTRL_ENABLE_SHIFT              0x07u       /* As defined by Verilog Implementation */

    /* Control Register Bit Masks */
    #define MainTimer_CTRL_INTCNT_MASK               (0x03u << MainTimer_CTRL_INTCNT_SHIFT)
    #define MainTimer_CTRL_TRIG_MODE_MASK            (0x03u << MainTimer_CTRL_TRIG_MODE_SHIFT)
    #define MainTimer_CTRL_TRIG_EN                   (0x01u << MainTimer_CTRL_TRIG_EN_SHIFT)
    #define MainTimer_CTRL_CAP_MODE_MASK             (0x03u << MainTimer_CTRL_CAP_MODE_SHIFT)
    #define MainTimer_CTRL_ENABLE                    (0x01u << MainTimer_CTRL_ENABLE_SHIFT)

    /* Bit Counter (7-bit) Control Register Bit Definitions */
    /* As defined by the Register map for the AUX Control Register */
    #define MainTimer_CNTR_ENABLE                    0x20u

    /* Status Register Bit Locations */
    #define MainTimer_STATUS_TC_SHIFT                0x00u  /* As defined by Verilog Implementation */
    #define MainTimer_STATUS_CAPTURE_SHIFT           0x01u  /* As defined by Verilog Implementation */
    #define MainTimer_STATUS_TC_INT_MASK_SHIFT       MainTimer_STATUS_TC_SHIFT
    #define MainTimer_STATUS_CAPTURE_INT_MASK_SHIFT  MainTimer_STATUS_CAPTURE_SHIFT
    #define MainTimer_STATUS_FIFOFULL_SHIFT          0x02u  /* As defined by Verilog Implementation */
    #define MainTimer_STATUS_FIFONEMP_SHIFT          0x03u  /* As defined by Verilog Implementation */
    #define MainTimer_STATUS_FIFOFULL_INT_MASK_SHIFT MainTimer_STATUS_FIFOFULL_SHIFT

    /* Status Register Bit Masks */
    /* Sticky TC Event Bit-Mask */
    #define MainTimer_STATUS_TC                      (0x01u << MainTimer_STATUS_TC_SHIFT)
    /* Sticky Capture Event Bit-Mask */
    #define MainTimer_STATUS_CAPTURE                 (0x01u << MainTimer_STATUS_CAPTURE_SHIFT)
    /* Interrupt Enable Bit-Mask */
    #define MainTimer_STATUS_TC_INT_MASK             (0x01u << MainTimer_STATUS_TC_SHIFT)
    /* Interrupt Enable Bit-Mask */
    #define MainTimer_STATUS_CAPTURE_INT_MASK        (0x01u << MainTimer_STATUS_CAPTURE_SHIFT)
    /* NOT-Sticky FIFO Full Bit-Mask */
    #define MainTimer_STATUS_FIFOFULL                (0x01u << MainTimer_STATUS_FIFOFULL_SHIFT)
    /* NOT-Sticky FIFO Not Empty Bit-Mask */
    #define MainTimer_STATUS_FIFONEMP                (0x01u << MainTimer_STATUS_FIFONEMP_SHIFT)
    /* Interrupt Enable Bit-Mask */
    #define MainTimer_STATUS_FIFOFULL_INT_MASK       (0x01u << MainTimer_STATUS_FIFOFULL_SHIFT)

    #define MainTimer_STATUS_ACTL_INT_EN             0x10u   /* As defined for the ACTL Register */

    /* Datapath Auxillary Control Register definitions */
    #define MainTimer_AUX_CTRL_FIFO0_CLR             0x01u   /* As defined by Register map */
    #define MainTimer_AUX_CTRL_FIFO1_CLR             0x02u   /* As defined by Register map */
    #define MainTimer_AUX_CTRL_FIFO0_LVL             0x04u   /* As defined by Register map */
    #define MainTimer_AUX_CTRL_FIFO1_LVL             0x08u   /* As defined by Register map */
    #define MainTimer_STATUS_ACTL_INT_EN_MASK        0x10u   /* As defined for the ACTL Register */

#endif /* Implementation Specific Registers and Register Constants */

#endif  /* CY_Timer_v2_20_MainTimer_H */


/* [] END OF FILE */
