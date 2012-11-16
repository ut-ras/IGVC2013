/*******************************************************************************
* File Name: IMU.h
* Version 2.10
*
* Description:
*  Contains the function prototypes and constants available to the UART
*  user module.
*
* Note:
*
********************************************************************************
* Copyright 2008-2011, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/


#include "cytypes.h"
#include "cyfitter.h"

#if !defined(CY_UART_IMU_H)
#define CY_UART_IMU_H


/***************************************
* Conditional Compilation Parameters
***************************************/

#define IMU_RX_ENABLED                     (1u)
#define IMU_TX_ENABLED                     (1u)
#define IMU_HD_ENABLED                     (0u)
#define IMU_RX_INTERRUPT_ENABLED           (1u)
#define IMU_TX_INTERRUPT_ENABLED           (1u)
#define IMU_INTERNAL_CLOCK_USED            (1u)
#define IMU_RXHW_ADDRESS_ENABLED           (0u)
#define IMU_OVER_SAMPLE_COUNT              (8u)
#define IMU_PARITY_TYPE                    (0u)
#define IMU_PARITY_TYPE_SW                 (0u)
#define IMU_BREAK_DETECT                   (0u)
#define IMU_BREAK_BITS_TX                  (13u)
#define IMU_BREAK_BITS_RX                  (13u)
#define IMU_TXCLKGEN_DP                    (1u)
#define IMU_USE23POLLING                   (1u)
#define IMU_FLOW_CONTROL                   (0u)

/* Check to see if required defines such as CY_PSOC3 and CY_PSOC5 are available */
/* They are defined starting with cy_boot v2.30 */
#ifndef CY_PSOC3
    #error Component UART_v2_10 requires cy_boot v2.30 or later
#endif /* End CY_PSOC3 */

#if(CY_PSOC3_ES2 && (IMU_RX_INTERRUPT_ENABLED || IMU_TX_INTERRUPT_ENABLED))
    #include <intrins.h>
    #define IMU_ISR_PATCH() _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
#endif /* End CY_PSOC3_ES2 */

#ifdef IMU_BUART_sCR_AsyncCtl_CtrlReg__CONTROL_REG
    #define IMU_CONTROL_REG_REMOVED            (0u)
#else
    #define IMU_CONTROL_REG_REMOVED            (1u)
#endif /* End IMU_BUART_sCR_AsyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct _IMU_backupStruct
{
    uint8 enableState;

    #if(IMU_CONTROL_REG_REMOVED == 0u)
        uint8 cr;            
    #endif /* End IMU_CONTROL_REG_REMOVED */    
    #if( (IMU_RX_ENABLED) || (IMU_HD_ENABLED) )
        uint8 rx_period;
        #if (CY_PSOC3_ES2 || CY_PSOC5_ES1) /* PSoC3 ES2 or early, PSoC5 ES1*/
            uint8 rx_mask;
            #if (IMU_RXHW_ADDRESS_ENABLED)
                uint8 rx_addr1;
                uint8 rx_addr2;
            #endif /* End IMU_RXHW_ADDRESS_ENABLED */
        #endif /* End PSOC3_ES2 || PSOC5_ES1 */
    #endif  /* End (IMU_RX_ENABLED) || (IMU_HD_ENABLED)*/

    #if(IMU_TX_ENABLED)
        #if(IMU_TXCLKGEN_DP)
            uint8 tx_clk_ctr;
            #if (CY_PSOC3_ES2 || CY_PSOC5_ES1) /* PSoC3 ES2 or early, PSoC5 ES1*/
                uint8 tx_clk_compl;
            #endif  /* End PSOC3_ES2 || PSOC5_ES1 */
        #else
            uint8 tx_period;
        #endif /*End IMU_TXCLKGEN_DP */
        #if (CY_PSOC3_ES2 || CY_PSOC5_ES1) /* PSoC3 ES2 or early, PSoC5 ES1*/
            uint8 tx_mask;
        #endif  /* End PSOC3_ES2 || PSOC5_ES1 */
    #endif /*End IMU_TX_ENABLED */
} IMU_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void IMU_Start(void) ;
void IMU_Stop(void) ;
uint8 IMU_ReadControlRegister(void) ;
void IMU_WriteControlRegister(uint8 control) ;

void IMU_Init(void) ;
void IMU_Enable(void) ;
void IMU_SaveConfig(void) ;
void IMU_RestoreConfig(void) ;
void IMU_Sleep(void) ;
void IMU_Wakeup(void) ;

/* Only if RX is enabled */
#if( (IMU_RX_ENABLED) || (IMU_HD_ENABLED) )

    #if(IMU_RX_INTERRUPT_ENABLED)
        void  IMU_EnableRxInt(void) ;
        void  IMU_DisableRxInt(void) ;
        CY_ISR_PROTO(IMU_RXISR);
    #endif /* IMU_RX_INTERRUPT_ENABLED */

    void IMU_SetRxAddressMode(uint8 addressMode) 
                                                           ;
    void IMU_SetRxAddress1(uint8 address) ;
    void IMU_SetRxAddress2(uint8 address) ;

    void  IMU_SetRxInterruptMode(uint8 intSrc) ;
    uint8 IMU_ReadRxData(void) ;
    uint8 IMU_ReadRxStatus(void) ;
    uint8 IMU_GetChar(void) ;
    uint16 IMU_GetByte(void) ;
    uint8 IMU_GetRxBufferSize(void)  
                                                            ;
    void IMU_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define IMU_GetRxInterruptSource   IMU_ReadRxStatus

#endif /* End (IMU_RX_ENABLED) || (IMU_HD_ENABLED) */

/* Only if TX is enabled */
#if(IMU_TX_ENABLED || IMU_HD_ENABLED)

    #if(IMU_TX_INTERRUPT_ENABLED)
        void IMU_EnableTxInt(void) ;
        void IMU_DisableTxInt(void) ;
        CY_ISR_PROTO(IMU_TXISR);
    #endif /* IMU_TX_INTERRUPT_ENABLED */

    void IMU_SetTxInterruptMode(uint8 intSrc) ;
    void IMU_WriteTxData(uint8 txDataByte) ;
    uint8 IMU_ReadTxStatus(void) ;
    void IMU_PutChar(uint8 txDataByte) ;
    void IMU_PutString(char* string) ;
    void IMU_PutArray(uint8* string, uint8 byteCount)
                                                            ;
    void IMU_PutCRLF(uint8 txDataByte) ;
    void IMU_ClearTxBuffer(void) ;
    void IMU_SetTxAddressMode(uint8 addressMode) ;
    void IMU_SendBreak(uint8 retMode) ;
    uint8 IMU_GetTxBufferSize(void) 
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define IMU_PutStringConst         IMU_PutString
    #define IMU_PutArrayConst          IMU_PutArray
    #define IMU_GetTxInterruptSource   IMU_ReadTxStatus
    
#endif /* End IMU_TX_ENABLED || IMU_HD_ENABLED */

#if(IMU_HD_ENABLED)
    void IMU_LoadRxConfig(void) ;
    void IMU_LoadTxConfig(void) ;
#endif /* End IMU_HD_ENABLED */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define IMU_SET_SPACE                              (0x00u)
#define IMU_SET_MARK                               (0x01u)

/* Status Register definitions */
#if( (IMU_TX_ENABLED) || (IMU_HD_ENABLED) )
    #if(IMU_TX_INTERRUPT_ENABLED)
        #define IMU_TX_VECT_NUM                IMU_TXInternalInterrupt__INTC_NUMBER
        #define IMU_TX_PRIOR_NUM               IMU_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* IMU_TX_INTERRUPT_ENABLED */
    #if(IMU_TX_ENABLED) 
        #define IMU_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define IMU_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define IMU_TX_STS_FIFO_FULL_SHIFT         (0x02u)
        #define IMU_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* IMU_TX_ENABLED */
    #if(IMU_HD_ENABLED) 
        #define IMU_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define IMU_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define IMU_TX_STS_FIFO_FULL_SHIFT         (0x05u)  /*needs MD=0*/
        #define IMU_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* IMU_HD_ENABLED */
    #define IMU_TX_STS_COMPLETE                (0x01u << IMU_TX_STS_COMPLETE_SHIFT)
    #define IMU_TX_STS_FIFO_EMPTY              (0x01u << IMU_TX_STS_FIFO_EMPTY_SHIFT)
    #define IMU_TX_STS_FIFO_FULL               (0x01u << IMU_TX_STS_FIFO_FULL_SHIFT)
    #define IMU_TX_STS_FIFO_NOT_FULL           (0x01u << IMU_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (IMU_TX_ENABLED) || (IMU_HD_ENABLED)*/

#if( (IMU_RX_ENABLED) || (IMU_HD_ENABLED) )
    #if(IMU_RX_INTERRUPT_ENABLED)
        #define IMU_RX_VECT_NUM                IMU_RXInternalInterrupt__INTC_NUMBER
        #define IMU_RX_PRIOR_NUM               IMU_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* IMU_RX_INTERRUPT_ENABLED */
    #define IMU_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define IMU_RX_STS_BREAK_SHIFT             (0x01u)
    #define IMU_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define IMU_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define IMU_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define IMU_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define IMU_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define IMU_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define IMU_RX_STS_MRKSPC                  (0x01u << IMU_RX_STS_MRKSPC_SHIFT)
    #define IMU_RX_STS_BREAK                   (0x01u << IMU_RX_STS_BREAK_SHIFT)
    #define IMU_RX_STS_PAR_ERROR               (0x01u << IMU_RX_STS_PAR_ERROR_SHIFT)
    #define IMU_RX_STS_STOP_ERROR              (0x01u << IMU_RX_STS_STOP_ERROR_SHIFT)
    #define IMU_RX_STS_OVERRUN                 (0x01u << IMU_RX_STS_OVERRUN_SHIFT)
    #define IMU_RX_STS_FIFO_NOTEMPTY           (0x01u << IMU_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define IMU_RX_STS_ADDR_MATCH              (0x01u << IMU_RX_STS_ADDR_MATCH_SHIFT)
    #define IMU_RX_STS_SOFT_BUFF_OVER          (0x01u << IMU_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define IMU_RX_HW_MASK                     (0x7Fu)
#endif /* End (IMU_RX_ENABLED) || (IMU_HD_ENABLED) */

/* Control Register definitions */
#define IMU_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define IMU_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define IMU_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define IMU_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define IMU_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define IMU_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define IMU_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define IMU_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define IMU_CTRL_HD_SEND                       (0x01u << IMU_CTRL_HD_SEND_SHIFT)
#define IMU_CTRL_HD_SEND_BREAK                 (0x01u << IMU_CTRL_HD_SEND_BREAK_SHIFT)
#define IMU_CTRL_MARK                          (0x01u << IMU_CTRL_MARK_SHIFT)
#define IMU_CTRL_PARITY_TYPE_MASK              (0x03u << IMU_CTRL_PARITY_TYPE0_SHIFT)
#define IMU_CTRL_RXADDR_MODE_MASK              (0x07u << IMU_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define IMU_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define IMU_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define IMU_SEND_BREAK                         (0x00u)
#define IMU_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define IMU_REINIT                             (0x02u)
#define IMU_SEND_WAIT_REINIT                   (0x03u)

#define IMU_OVER_SAMPLE_8                      (8u)
#define IMU_OVER_SAMPLE_16                     (16u)

#define IMU_BIT_CENTER                         (IMU_OVER_SAMPLE_COUNT - 1u)

#define IMU_FIFO_LENGTH                        (4u)
#define IMU_NUMBER_OF_START_BIT                (1u)

/* 8X always for count7 implementation*/
#define IMU_TXBITCTR_BREAKBITS8X   (IMU_BREAK_BITS_TX * IMU_OVER_SAMPLE_8 - 1u)
/* 8X or 16X for DP implementation*/
#define IMU_TXBITCTR_BREAKBITS   (IMU_BREAK_BITS_TX * IMU_OVER_SAMPLE_COUNT - 1u)

#if (IMU_OVER_SAMPLE_COUNT == IMU_OVER_SAMPLE_8)
    #define IMU_HD_TXBITCTR_INIT   ((IMU_BREAK_BITS_TX + \
        IMU_NUMBER_OF_START_BIT) * IMU_OVER_SAMPLE_COUNT - 1u)
    /* This parameter is increased on the 1 in 2 out of 3 mode to sample voting in the middle */
    #define IMU_HD_RXBITCTR_INIT   ((IMU_BREAK_BITS_RX + \
    IMU_NUMBER_OF_START_BIT) * IMU_OVER_SAMPLE_COUNT - 1u + \
    (IMU_OVER_SAMPLE_COUNT / 2u) + (IMU_USE23POLLING * 2u) - 1u)
#else /* IMU_OVER_SAMPLE_COUNT == IMU_OVER_SAMPLE_16 */
    #define IMU_HD_TXBITCTR_INIT   (8u * IMU_OVER_SAMPLE_COUNT - 1u)
    #define IMU_HD_RXBITCTR_INIT   (7u * IMU_OVER_SAMPLE_COUNT - 1u  +  \
       (IMU_OVER_SAMPLE_COUNT / 2u) + (IMU_USE23POLLING * 2u) - 1u)
#endif /* End IMU_OVER_SAMPLE_COUNT */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define IMU__B_UART__AM_SW_BYTE_BYTE 1
#define IMU__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define IMU__B_UART__AM_HW_BYTE_BY_BYTE 3
#define IMU__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define IMU__B_UART__AM_NONE 0

#define IMU__B_UART__NONE_REVB 0
#define IMU__B_UART__EVEN_REVB 1
#define IMU__B_UART__ODD_REVB 2
#define IMU__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

#define IMU_TXBUFFERSIZE           (64u)
#define IMU_RXBUFFERSIZE           (64u)
/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define IMU_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define IMU_NUMBER_OF_STOP_BITS    (1u)

#if (IMU_RXHW_ADDRESS_ENABLED)
    #define IMU_RXADDRESSMODE      (0u)
    #define IMU_RXHWADDRESS1       (0u)
    #define IMU_RXHWADDRESS2       (0u)
    /* Backward compatible define */
    #define IMU_RXAddressMode      IMU_RXADDRESSMODE
#endif /* End EnableHWAddress */

#define IMU_INIT_RX_INTERRUPTS_MASK \
                                          (1 << IMU_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << IMU_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << IMU_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << IMU_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << IMU_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << IMU_RX_STS_BREAK_SHIFT) \
                                        | (0 << IMU_RX_STS_OVERRUN_SHIFT)

#define IMU_INIT_TX_INTERRUPTS_MASK \
                                          (0 << IMU_TX_STS_COMPLETE_SHIFT) \
                                        | (1 << IMU_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << IMU_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << IMU_TX_STS_FIFO_NOT_FULL_SHIFT)


/***************************************
*              Registers
***************************************/

#ifdef IMU_BUART_sCR_AsyncCtl_CtrlReg__CONTROL_REG
    #define IMU_CONTROL_REG \
                            (* (reg8 *) IMU_BUART_sCR_AsyncCtl_CtrlReg__CONTROL_REG )
    #define IMU_CONTROL_PTR \
                            (  (reg8 *) IMU_BUART_sCR_AsyncCtl_CtrlReg__CONTROL_REG )
#endif /* End IMU_BUART_sCR_AsyncCtl_CtrlReg__CONTROL_REG */

#if(IMU_TX_ENABLED)
    #define IMU_TXDATA_REG          (* (reg8 *) IMU_BUART_sTX_TxShifter_u0__F0_REG)
    #define IMU_TXDATA_PTR          (  (reg8 *) IMU_BUART_sTX_TxShifter_u0__F0_REG)
    #define IMU_TXDATA_AUX_CTL_REG  (* (reg8 *) IMU_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define IMU_TXDATA_AUX_CTL_PTR  (  (reg8 *) IMU_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define IMU_TXSTATUS_REG        (* (reg8 *) IMU_BUART_sTX_TxSts__STATUS_REG)
    #define IMU_TXSTATUS_PTR        (  (reg8 *) IMU_BUART_sTX_TxSts__STATUS_REG)
    #define IMU_TXSTATUS_MASK_REG   (* (reg8 *) IMU_BUART_sTX_TxSts__MASK_REG)
    #define IMU_TXSTATUS_MASK_PTR   (  (reg8 *) IMU_BUART_sTX_TxSts__MASK_REG)
    #define IMU_TXSTATUS_ACTL_REG   (* (reg8 *) IMU_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define IMU_TXSTATUS_ACTL_PTR   (  (reg8 *) IMU_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(IMU_TXCLKGEN_DP)
        #define IMU_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) IMU_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define IMU_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) IMU_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define IMU_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) IMU_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define IMU_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) IMU_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define IMU_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) IMU_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define IMU_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) IMU_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define IMU_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) IMU_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define IMU_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) IMU_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define IMU_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) IMU_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define IMU_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) IMU_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* IMU_TXCLKGEN_DP */

#endif /* End IMU_TX_ENABLED */

#if(IMU_HD_ENABLED)

    #define IMU_TXDATA_REG             (* (reg8 *) IMU_BUART_sRX_RxShifter_u0__F1_REG )
    #define IMU_TXDATA_PTR             (  (reg8 *) IMU_BUART_sRX_RxShifter_u0__F1_REG )
    #define IMU_TXDATA_AUX_CTL_REG     (* (reg8 *) IMU_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define IMU_TXDATA_AUX_CTL_PTR     (  (reg8 *) IMU_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define IMU_TXSTATUS_REG           (* (reg8 *) IMU_BUART_sRX_RxSts__STATUS_REG )
    #define IMU_TXSTATUS_PTR           (  (reg8 *) IMU_BUART_sRX_RxSts__STATUS_REG )
    #define IMU_TXSTATUS_MASK_REG      (* (reg8 *) IMU_BUART_sRX_RxSts__MASK_REG )
    #define IMU_TXSTATUS_MASK_PTR      (  (reg8 *) IMU_BUART_sRX_RxSts__MASK_REG )
    #define IMU_TXSTATUS_ACTL_REG      (* (reg8 *) IMU_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define IMU_TXSTATUS_ACTL_PTR      (  (reg8 *) IMU_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End IMU_HD_ENABLED */

#if( (IMU_RX_ENABLED) || (IMU_HD_ENABLED) )
    #define IMU_RXDATA_REG             (* (reg8 *) IMU_BUART_sRX_RxShifter_u0__F0_REG )
    #define IMU_RXDATA_PTR             (  (reg8 *) IMU_BUART_sRX_RxShifter_u0__F0_REG )
    #define IMU_RXADDRESS1_REG         (* (reg8 *) IMU_BUART_sRX_RxShifter_u0__D0_REG )
    #define IMU_RXADDRESS1_PTR         (  (reg8 *) IMU_BUART_sRX_RxShifter_u0__D0_REG )
    #define IMU_RXADDRESS2_REG         (* (reg8 *) IMU_BUART_sRX_RxShifter_u0__D1_REG )
    #define IMU_RXADDRESS2_PTR         (  (reg8 *) IMU_BUART_sRX_RxShifter_u0__D1_REG )
    #define IMU_RXDATA_AUX_CTL_REG     (* (reg8 *) IMU_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define IMU_RXBITCTR_PERIOD_REG    (* (reg8 *) IMU_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define IMU_RXBITCTR_PERIOD_PTR    (  (reg8 *) IMU_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define IMU_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) IMU_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define IMU_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) IMU_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define IMU_RXBITCTR_COUNTER_REG   (* (reg8 *) IMU_BUART_sRX_RxBitCounter__COUNT_REG )
    #define IMU_RXBITCTR_COUNTER_PTR   (  (reg8 *) IMU_BUART_sRX_RxBitCounter__COUNT_REG )

    #define IMU_RXSTATUS_REG           (* (reg8 *) IMU_BUART_sRX_RxSts__STATUS_REG )
    #define IMU_RXSTATUS_PTR           (  (reg8 *) IMU_BUART_sRX_RxSts__STATUS_REG )
    #define IMU_RXSTATUS_MASK_REG      (* (reg8 *) IMU_BUART_sRX_RxSts__MASK_REG )
    #define IMU_RXSTATUS_MASK_PTR      (  (reg8 *) IMU_BUART_sRX_RxSts__MASK_REG )
    #define IMU_RXSTATUS_ACTL_REG      (* (reg8 *) IMU_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define IMU_RXSTATUS_ACTL_PTR      (  (reg8 *) IMU_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (IMU_RX_ENABLED) || (IMU_HD_ENABLED) */

#if(IMU_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define IMU_INTCLOCK_CLKEN_REG     (* (reg8 *) IMU_IntClock__PM_ACT_CFG)
    #define IMU_INTCLOCK_CLKEN_PTR     (  (reg8 *) IMU_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define IMU_INTCLOCK_CLKEN_MASK    IMU_IntClock__PM_ACT_MSK
#endif /* End IMU_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants              
***************************************/

#if(IMU_TX_ENABLED)
    #define IMU_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End IMU_TX_ENABLED */

#if(IMU_HD_ENABLED)
    #define IMU_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End IMU_HD_ENABLED */

#if( (IMU_RX_ENABLED) || (IMU_HD_ENABLED) )
    #define IMU_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (IMU_RX_ENABLED) || (IMU_HD_ENABLED) */


/***************************************
* Renamed global variables or defines 
* for backward compatible
***************************************/

#define IMU_initvar                    IMU_initVar

#define IMU_RX_Enabled                 IMU_RX_ENABLED
#define IMU_TX_Enabled                 IMU_TX_ENABLED
#define IMU_HD_Enabled                 IMU_HD_ENABLED
#define IMU_RX_IntInterruptEnabled     IMU_RX_INTERRUPT_ENABLED
#define IMU_TX_IntInterruptEnabled     IMU_TX_INTERRUPT_ENABLED
#define IMU_InternalClockUsed          IMU_INTERNAL_CLOCK_USED
#define IMU_RXHW_Address_Enabled       IMU_RXHW_ADDRESS_ENABLED
#define IMU_OverSampleCount            IMU_OVER_SAMPLE_COUNT
#define IMU_ParityType                 IMU_PARITY_TYPE

#if( IMU_TX_ENABLED && (IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH))
    #define IMU_TXBUFFER               IMU_txBuffer
    #define IMU_TXBUFFERREAD           IMU_txBufferRead 
    #define IMU_TXBUFFERWRITE          IMU_txBufferWrite 
#endif /* End IMU_TX_ENABLED */
#if( ( IMU_RX_ENABLED || IMU_HD_ENABLED ) && \
     (IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH) )
    #define IMU_RXBUFFER               IMU_rxBuffer
    #define IMU_RXBUFFERREAD           IMU_rxBufferRead 
    #define IMU_RXBUFFERWRITE          IMU_rxBufferWrite 
    #define IMU_RXBUFFERLOOPDETECT     IMU_rxBufferLoopDetect
    #define IMU_RXBUFFER_OVERFLOW      IMU_rxBufferOverflow
#endif /* End IMU_RX_ENABLED */

#ifdef IMU_BUART_sCR_AsyncCtl_CtrlReg__CONTROL_REG
    #define IMU_CONTROL                IMU_CONTROL_REG 
#endif /* End IMU_BUART_sCR_AsyncCtl_CtrlReg__CONTROL_REG */

#if(IMU_TX_ENABLED)
    #define IMU_TXDATA                 IMU_TXDATA_REG
    #define IMU_TXSTATUS               IMU_TXSTATUS_REG
    #define IMU_TXSTATUS_MASK          IMU_TXSTATUS_MASK_REG   
    #define IMU_TXSTATUS_ACTL          IMU_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(IMU_TXCLKGEN_DP)
        #define IMU_TXBITCLKGEN_CTR        IMU_TXBITCLKGEN_CTR_REG
        #define IMU_TXBITCLKTX_COMPLETE    IMU_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define IMU_TXBITCTR_PERIOD        IMU_TXBITCTR_PERIOD_REG
        #define IMU_TXBITCTR_CONTROL       IMU_TXBITCTR_CONTROL_REG
        #define IMU_TXBITCTR_COUNTER       IMU_TXBITCTR_COUNTER_REG
    #endif /* IMU_TXCLKGEN_DP */
#endif /* End IMU_TX_ENABLED */

#if(IMU_HD_ENABLED)
    #define IMU_TXDATA                 IMU_TXDATA_REG
    #define IMU_TXSTATUS               IMU_TXSTATUS_REG
    #define IMU_TXSTATUS_MASK          IMU_TXSTATUS_MASK_REG   
    #define IMU_TXSTATUS_ACTL          IMU_TXSTATUS_ACTL_REG
#endif /* End IMU_HD_ENABLED */

#if( (IMU_RX_ENABLED) || (IMU_HD_ENABLED) )
    #define IMU_RXDATA                 IMU_RXDATA_REG             
    #define IMU_RXADDRESS1             IMU_RXADDRESS1_REG
    #define IMU_RXADDRESS2             IMU_RXADDRESS2_REG
    #define IMU_RXBITCTR_PERIOD        IMU_RXBITCTR_PERIOD_REG
    #define IMU_RXBITCTR_CONTROL       IMU_RXBITCTR_CONTROL_REG
    #define IMU_RXBITCTR_COUNTER       IMU_RXBITCTR_COUNTER_REG
    #define IMU_RXSTATUS               IMU_RXSTATUS_REG
    #define IMU_RXSTATUS_MASK          IMU_RXSTATUS_MASK_REG
    #define IMU_RXSTATUS_ACTL          IMU_RXSTATUS_ACTL_REG
#endif /* End  (IMU_RX_ENABLED) || (IMU_HD_ENABLED) */

#if(IMU_INTERNAL_CLOCK_USED)
    #define IMU_INTCLOCK_CLKEN         IMU_INTCLOCK_CLKEN_REG
#endif /* End IMU_INTERNAL_CLOCK_USED */

#define IMU_WAIT_FOR_COMLETE_REINIT    IMU_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_IMU_H */


/* [] END OF FILE */
