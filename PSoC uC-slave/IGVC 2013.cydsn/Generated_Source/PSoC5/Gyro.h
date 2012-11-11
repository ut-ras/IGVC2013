/*******************************************************************************
* File Name: Gyro.h
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

#if !defined(CY_UART_Gyro_H)
#define CY_UART_Gyro_H


/***************************************
* Conditional Compilation Parameters
***************************************/

#define Gyro_RX_ENABLED                     (1u)
#define Gyro_TX_ENABLED                     (1u)
#define Gyro_HD_ENABLED                     (0u)
#define Gyro_RX_INTERRUPT_ENABLED           (0u)
#define Gyro_TX_INTERRUPT_ENABLED           (0u)
#define Gyro_INTERNAL_CLOCK_USED            (1u)
#define Gyro_RXHW_ADDRESS_ENABLED           (0u)
#define Gyro_OVER_SAMPLE_COUNT              (8u)
#define Gyro_PARITY_TYPE                    (0u)
#define Gyro_PARITY_TYPE_SW                 (0u)
#define Gyro_BREAK_DETECT                   (0u)
#define Gyro_BREAK_BITS_TX                  (13u)
#define Gyro_BREAK_BITS_RX                  (13u)
#define Gyro_TXCLKGEN_DP                    (1u)
#define Gyro_USE23POLLING                   (1u)
#define Gyro_FLOW_CONTROL                   (0u)

/* Check to see if required defines such as CY_PSOC3 and CY_PSOC5 are available */
/* They are defined starting with cy_boot v2.30 */
#ifndef CY_PSOC3
    #error Component UART_v2_10 requires cy_boot v2.30 or later
#endif /* End CY_PSOC3 */

#if(CY_PSOC3_ES2 && (Gyro_RX_INTERRUPT_ENABLED || Gyro_TX_INTERRUPT_ENABLED))
    #include <intrins.h>
    #define Gyro_ISR_PATCH() _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
#endif /* End CY_PSOC3_ES2 */

#ifdef Gyro_BUART_sCR_AsyncCtl_CtrlReg__CONTROL_REG
    #define Gyro_CONTROL_REG_REMOVED            (0u)
#else
    #define Gyro_CONTROL_REG_REMOVED            (1u)
#endif /* End Gyro_BUART_sCR_AsyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct _Gyro_backupStruct
{
    uint8 enableState;

    #if(Gyro_CONTROL_REG_REMOVED == 0u)
        uint8 cr;            
    #endif /* End Gyro_CONTROL_REG_REMOVED */    
    #if( (Gyro_RX_ENABLED) || (Gyro_HD_ENABLED) )
        uint8 rx_period;
        #if (CY_PSOC3_ES2 || CY_PSOC5_ES1) /* PSoC3 ES2 or early, PSoC5 ES1*/
            uint8 rx_mask;
            #if (Gyro_RXHW_ADDRESS_ENABLED)
                uint8 rx_addr1;
                uint8 rx_addr2;
            #endif /* End Gyro_RXHW_ADDRESS_ENABLED */
        #endif /* End PSOC3_ES2 || PSOC5_ES1 */
    #endif  /* End (Gyro_RX_ENABLED) || (Gyro_HD_ENABLED)*/

    #if(Gyro_TX_ENABLED)
        #if(Gyro_TXCLKGEN_DP)
            uint8 tx_clk_ctr;
            #if (CY_PSOC3_ES2 || CY_PSOC5_ES1) /* PSoC3 ES2 or early, PSoC5 ES1*/
                uint8 tx_clk_compl;
            #endif  /* End PSOC3_ES2 || PSOC5_ES1 */
        #else
            uint8 tx_period;
        #endif /*End Gyro_TXCLKGEN_DP */
        #if (CY_PSOC3_ES2 || CY_PSOC5_ES1) /* PSoC3 ES2 or early, PSoC5 ES1*/
            uint8 tx_mask;
        #endif  /* End PSOC3_ES2 || PSOC5_ES1 */
    #endif /*End Gyro_TX_ENABLED */
} Gyro_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void Gyro_Start(void) ;
void Gyro_Stop(void) ;
uint8 Gyro_ReadControlRegister(void) ;
void Gyro_WriteControlRegister(uint8 control) ;

void Gyro_Init(void) ;
void Gyro_Enable(void) ;
void Gyro_SaveConfig(void) ;
void Gyro_RestoreConfig(void) ;
void Gyro_Sleep(void) ;
void Gyro_Wakeup(void) ;

/* Only if RX is enabled */
#if( (Gyro_RX_ENABLED) || (Gyro_HD_ENABLED) )

    #if(Gyro_RX_INTERRUPT_ENABLED)
        void  Gyro_EnableRxInt(void) ;
        void  Gyro_DisableRxInt(void) ;
        CY_ISR_PROTO(Gyro_RXISR);
    #endif /* Gyro_RX_INTERRUPT_ENABLED */

    void Gyro_SetRxAddressMode(uint8 addressMode) 
                                                           ;
    void Gyro_SetRxAddress1(uint8 address) ;
    void Gyro_SetRxAddress2(uint8 address) ;

    void  Gyro_SetRxInterruptMode(uint8 intSrc) ;
    uint8 Gyro_ReadRxData(void) ;
    uint8 Gyro_ReadRxStatus(void) ;
    uint8 Gyro_GetChar(void) ;
    uint16 Gyro_GetByte(void) ;
    uint8 Gyro_GetRxBufferSize(void)  
                                                            ;
    void Gyro_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define Gyro_GetRxInterruptSource   Gyro_ReadRxStatus

#endif /* End (Gyro_RX_ENABLED) || (Gyro_HD_ENABLED) */

/* Only if TX is enabled */
#if(Gyro_TX_ENABLED || Gyro_HD_ENABLED)

    #if(Gyro_TX_INTERRUPT_ENABLED)
        void Gyro_EnableTxInt(void) ;
        void Gyro_DisableTxInt(void) ;
        CY_ISR_PROTO(Gyro_TXISR);
    #endif /* Gyro_TX_INTERRUPT_ENABLED */

    void Gyro_SetTxInterruptMode(uint8 intSrc) ;
    void Gyro_WriteTxData(uint8 txDataByte) ;
    uint8 Gyro_ReadTxStatus(void) ;
    void Gyro_PutChar(uint8 txDataByte) ;
    void Gyro_PutString(char* string) ;
    void Gyro_PutArray(uint8* string, uint8 byteCount)
                                                            ;
    void Gyro_PutCRLF(uint8 txDataByte) ;
    void Gyro_ClearTxBuffer(void) ;
    void Gyro_SetTxAddressMode(uint8 addressMode) ;
    void Gyro_SendBreak(uint8 retMode) ;
    uint8 Gyro_GetTxBufferSize(void) 
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define Gyro_PutStringConst         Gyro_PutString
    #define Gyro_PutArrayConst          Gyro_PutArray
    #define Gyro_GetTxInterruptSource   Gyro_ReadTxStatus
    
#endif /* End Gyro_TX_ENABLED || Gyro_HD_ENABLED */

#if(Gyro_HD_ENABLED)
    void Gyro_LoadRxConfig(void) ;
    void Gyro_LoadTxConfig(void) ;
#endif /* End Gyro_HD_ENABLED */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define Gyro_SET_SPACE                              (0x00u)
#define Gyro_SET_MARK                               (0x01u)

/* Status Register definitions */
#if( (Gyro_TX_ENABLED) || (Gyro_HD_ENABLED) )
    #if(Gyro_TX_INTERRUPT_ENABLED)
        #define Gyro_TX_VECT_NUM                Gyro_TXInternalInterrupt__INTC_NUMBER
        #define Gyro_TX_PRIOR_NUM               Gyro_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Gyro_TX_INTERRUPT_ENABLED */
    #if(Gyro_TX_ENABLED) 
        #define Gyro_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Gyro_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Gyro_TX_STS_FIFO_FULL_SHIFT         (0x02u)
        #define Gyro_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Gyro_TX_ENABLED */
    #if(Gyro_HD_ENABLED) 
        #define Gyro_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Gyro_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Gyro_TX_STS_FIFO_FULL_SHIFT         (0x05u)  /*needs MD=0*/
        #define Gyro_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Gyro_HD_ENABLED */
    #define Gyro_TX_STS_COMPLETE                (0x01u << Gyro_TX_STS_COMPLETE_SHIFT)
    #define Gyro_TX_STS_FIFO_EMPTY              (0x01u << Gyro_TX_STS_FIFO_EMPTY_SHIFT)
    #define Gyro_TX_STS_FIFO_FULL               (0x01u << Gyro_TX_STS_FIFO_FULL_SHIFT)
    #define Gyro_TX_STS_FIFO_NOT_FULL           (0x01u << Gyro_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (Gyro_TX_ENABLED) || (Gyro_HD_ENABLED)*/

#if( (Gyro_RX_ENABLED) || (Gyro_HD_ENABLED) )
    #if(Gyro_RX_INTERRUPT_ENABLED)
        #define Gyro_RX_VECT_NUM                Gyro_RXInternalInterrupt__INTC_NUMBER
        #define Gyro_RX_PRIOR_NUM               Gyro_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Gyro_RX_INTERRUPT_ENABLED */
    #define Gyro_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define Gyro_RX_STS_BREAK_SHIFT             (0x01u)
    #define Gyro_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define Gyro_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define Gyro_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define Gyro_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define Gyro_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define Gyro_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define Gyro_RX_STS_MRKSPC                  (0x01u << Gyro_RX_STS_MRKSPC_SHIFT)
    #define Gyro_RX_STS_BREAK                   (0x01u << Gyro_RX_STS_BREAK_SHIFT)
    #define Gyro_RX_STS_PAR_ERROR               (0x01u << Gyro_RX_STS_PAR_ERROR_SHIFT)
    #define Gyro_RX_STS_STOP_ERROR              (0x01u << Gyro_RX_STS_STOP_ERROR_SHIFT)
    #define Gyro_RX_STS_OVERRUN                 (0x01u << Gyro_RX_STS_OVERRUN_SHIFT)
    #define Gyro_RX_STS_FIFO_NOTEMPTY           (0x01u << Gyro_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define Gyro_RX_STS_ADDR_MATCH              (0x01u << Gyro_RX_STS_ADDR_MATCH_SHIFT)
    #define Gyro_RX_STS_SOFT_BUFF_OVER          (0x01u << Gyro_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define Gyro_RX_HW_MASK                     (0x7Fu)
#endif /* End (Gyro_RX_ENABLED) || (Gyro_HD_ENABLED) */

/* Control Register definitions */
#define Gyro_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define Gyro_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define Gyro_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define Gyro_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define Gyro_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define Gyro_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define Gyro_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define Gyro_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define Gyro_CTRL_HD_SEND                       (0x01u << Gyro_CTRL_HD_SEND_SHIFT)
#define Gyro_CTRL_HD_SEND_BREAK                 (0x01u << Gyro_CTRL_HD_SEND_BREAK_SHIFT)
#define Gyro_CTRL_MARK                          (0x01u << Gyro_CTRL_MARK_SHIFT)
#define Gyro_CTRL_PARITY_TYPE_MASK              (0x03u << Gyro_CTRL_PARITY_TYPE0_SHIFT)
#define Gyro_CTRL_RXADDR_MODE_MASK              (0x07u << Gyro_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define Gyro_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define Gyro_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define Gyro_SEND_BREAK                         (0x00u)
#define Gyro_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define Gyro_REINIT                             (0x02u)
#define Gyro_SEND_WAIT_REINIT                   (0x03u)

#define Gyro_OVER_SAMPLE_8                      (8u)
#define Gyro_OVER_SAMPLE_16                     (16u)

#define Gyro_BIT_CENTER                         (Gyro_OVER_SAMPLE_COUNT - 1u)

#define Gyro_FIFO_LENGTH                        (4u)
#define Gyro_NUMBER_OF_START_BIT                (1u)

/* 8X always for count7 implementation*/
#define Gyro_TXBITCTR_BREAKBITS8X   (Gyro_BREAK_BITS_TX * Gyro_OVER_SAMPLE_8 - 1u)
/* 8X or 16X for DP implementation*/
#define Gyro_TXBITCTR_BREAKBITS   (Gyro_BREAK_BITS_TX * Gyro_OVER_SAMPLE_COUNT - 1u)

#if (Gyro_OVER_SAMPLE_COUNT == Gyro_OVER_SAMPLE_8)
    #define Gyro_HD_TXBITCTR_INIT   ((Gyro_BREAK_BITS_TX + \
        Gyro_NUMBER_OF_START_BIT) * Gyro_OVER_SAMPLE_COUNT - 1u)
    /* This parameter is increased on the 1 in 2 out of 3 mode to sample voting in the middle */
    #define Gyro_HD_RXBITCTR_INIT   ((Gyro_BREAK_BITS_RX + \
    Gyro_NUMBER_OF_START_BIT) * Gyro_OVER_SAMPLE_COUNT - 1u + \
    (Gyro_OVER_SAMPLE_COUNT / 2u) + (Gyro_USE23POLLING * 2u) - 1u)
#else /* Gyro_OVER_SAMPLE_COUNT == Gyro_OVER_SAMPLE_16 */
    #define Gyro_HD_TXBITCTR_INIT   (8u * Gyro_OVER_SAMPLE_COUNT - 1u)
    #define Gyro_HD_RXBITCTR_INIT   (7u * Gyro_OVER_SAMPLE_COUNT - 1u  +  \
       (Gyro_OVER_SAMPLE_COUNT / 2u) + (Gyro_USE23POLLING * 2u) - 1u)
#endif /* End Gyro_OVER_SAMPLE_COUNT */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define Gyro__B_UART__AM_SW_BYTE_BYTE 1
#define Gyro__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define Gyro__B_UART__AM_HW_BYTE_BY_BYTE 3
#define Gyro__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define Gyro__B_UART__AM_NONE 0

#define Gyro__B_UART__NONE_REVB 0
#define Gyro__B_UART__EVEN_REVB 1
#define Gyro__B_UART__ODD_REVB 2
#define Gyro__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

#define Gyro_TXBUFFERSIZE           (4u)
#define Gyro_RXBUFFERSIZE           (4u)
/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define Gyro_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define Gyro_NUMBER_OF_STOP_BITS    (1u)

#if (Gyro_RXHW_ADDRESS_ENABLED)
    #define Gyro_RXADDRESSMODE      (0u)
    #define Gyro_RXHWADDRESS1       (0u)
    #define Gyro_RXHWADDRESS2       (0u)
    /* Backward compatible define */
    #define Gyro_RXAddressMode      Gyro_RXADDRESSMODE
#endif /* End EnableHWAddress */

#define Gyro_INIT_RX_INTERRUPTS_MASK \
                                          (1 << Gyro_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << Gyro_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << Gyro_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << Gyro_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << Gyro_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << Gyro_RX_STS_BREAK_SHIFT) \
                                        | (0 << Gyro_RX_STS_OVERRUN_SHIFT)

#define Gyro_INIT_TX_INTERRUPTS_MASK \
                                          (0 << Gyro_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << Gyro_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << Gyro_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << Gyro_TX_STS_FIFO_NOT_FULL_SHIFT)


/***************************************
*              Registers
***************************************/

#ifdef Gyro_BUART_sCR_AsyncCtl_CtrlReg__CONTROL_REG
    #define Gyro_CONTROL_REG \
                            (* (reg8 *) Gyro_BUART_sCR_AsyncCtl_CtrlReg__CONTROL_REG )
    #define Gyro_CONTROL_PTR \
                            (  (reg8 *) Gyro_BUART_sCR_AsyncCtl_CtrlReg__CONTROL_REG )
#endif /* End Gyro_BUART_sCR_AsyncCtl_CtrlReg__CONTROL_REG */

#if(Gyro_TX_ENABLED)
    #define Gyro_TXDATA_REG          (* (reg8 *) Gyro_BUART_sTX_TxShifter_u0__F0_REG)
    #define Gyro_TXDATA_PTR          (  (reg8 *) Gyro_BUART_sTX_TxShifter_u0__F0_REG)
    #define Gyro_TXDATA_AUX_CTL_REG  (* (reg8 *) Gyro_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Gyro_TXDATA_AUX_CTL_PTR  (  (reg8 *) Gyro_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Gyro_TXSTATUS_REG        (* (reg8 *) Gyro_BUART_sTX_TxSts__STATUS_REG)
    #define Gyro_TXSTATUS_PTR        (  (reg8 *) Gyro_BUART_sTX_TxSts__STATUS_REG)
    #define Gyro_TXSTATUS_MASK_REG   (* (reg8 *) Gyro_BUART_sTX_TxSts__MASK_REG)
    #define Gyro_TXSTATUS_MASK_PTR   (  (reg8 *) Gyro_BUART_sTX_TxSts__MASK_REG)
    #define Gyro_TXSTATUS_ACTL_REG   (* (reg8 *) Gyro_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define Gyro_TXSTATUS_ACTL_PTR   (  (reg8 *) Gyro_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(Gyro_TXCLKGEN_DP)
        #define Gyro_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) Gyro_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Gyro_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) Gyro_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Gyro_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) Gyro_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define Gyro_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) Gyro_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define Gyro_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) Gyro_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Gyro_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) Gyro_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Gyro_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Gyro_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Gyro_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Gyro_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Gyro_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) Gyro_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define Gyro_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) Gyro_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* Gyro_TXCLKGEN_DP */

#endif /* End Gyro_TX_ENABLED */

#if(Gyro_HD_ENABLED)

    #define Gyro_TXDATA_REG             (* (reg8 *) Gyro_BUART_sRX_RxShifter_u0__F1_REG )
    #define Gyro_TXDATA_PTR             (  (reg8 *) Gyro_BUART_sRX_RxShifter_u0__F1_REG )
    #define Gyro_TXDATA_AUX_CTL_REG     (* (reg8 *) Gyro_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define Gyro_TXDATA_AUX_CTL_PTR     (  (reg8 *) Gyro_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Gyro_TXSTATUS_REG           (* (reg8 *) Gyro_BUART_sRX_RxSts__STATUS_REG )
    #define Gyro_TXSTATUS_PTR           (  (reg8 *) Gyro_BUART_sRX_RxSts__STATUS_REG )
    #define Gyro_TXSTATUS_MASK_REG      (* (reg8 *) Gyro_BUART_sRX_RxSts__MASK_REG )
    #define Gyro_TXSTATUS_MASK_PTR      (  (reg8 *) Gyro_BUART_sRX_RxSts__MASK_REG )
    #define Gyro_TXSTATUS_ACTL_REG      (* (reg8 *) Gyro_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Gyro_TXSTATUS_ACTL_PTR      (  (reg8 *) Gyro_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End Gyro_HD_ENABLED */

#if( (Gyro_RX_ENABLED) || (Gyro_HD_ENABLED) )
    #define Gyro_RXDATA_REG             (* (reg8 *) Gyro_BUART_sRX_RxShifter_u0__F0_REG )
    #define Gyro_RXDATA_PTR             (  (reg8 *) Gyro_BUART_sRX_RxShifter_u0__F0_REG )
    #define Gyro_RXADDRESS1_REG         (* (reg8 *) Gyro_BUART_sRX_RxShifter_u0__D0_REG )
    #define Gyro_RXADDRESS1_PTR         (  (reg8 *) Gyro_BUART_sRX_RxShifter_u0__D0_REG )
    #define Gyro_RXADDRESS2_REG         (* (reg8 *) Gyro_BUART_sRX_RxShifter_u0__D1_REG )
    #define Gyro_RXADDRESS2_PTR         (  (reg8 *) Gyro_BUART_sRX_RxShifter_u0__D1_REG )
    #define Gyro_RXDATA_AUX_CTL_REG     (* (reg8 *) Gyro_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Gyro_RXBITCTR_PERIOD_REG    (* (reg8 *) Gyro_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Gyro_RXBITCTR_PERIOD_PTR    (  (reg8 *) Gyro_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Gyro_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Gyro_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Gyro_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Gyro_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Gyro_RXBITCTR_COUNTER_REG   (* (reg8 *) Gyro_BUART_sRX_RxBitCounter__COUNT_REG )
    #define Gyro_RXBITCTR_COUNTER_PTR   (  (reg8 *) Gyro_BUART_sRX_RxBitCounter__COUNT_REG )

    #define Gyro_RXSTATUS_REG           (* (reg8 *) Gyro_BUART_sRX_RxSts__STATUS_REG )
    #define Gyro_RXSTATUS_PTR           (  (reg8 *) Gyro_BUART_sRX_RxSts__STATUS_REG )
    #define Gyro_RXSTATUS_MASK_REG      (* (reg8 *) Gyro_BUART_sRX_RxSts__MASK_REG )
    #define Gyro_RXSTATUS_MASK_PTR      (  (reg8 *) Gyro_BUART_sRX_RxSts__MASK_REG )
    #define Gyro_RXSTATUS_ACTL_REG      (* (reg8 *) Gyro_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Gyro_RXSTATUS_ACTL_PTR      (  (reg8 *) Gyro_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (Gyro_RX_ENABLED) || (Gyro_HD_ENABLED) */

#if(Gyro_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define Gyro_INTCLOCK_CLKEN_REG     (* (reg8 *) Gyro_IntClock__PM_ACT_CFG)
    #define Gyro_INTCLOCK_CLKEN_PTR     (  (reg8 *) Gyro_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define Gyro_INTCLOCK_CLKEN_MASK    Gyro_IntClock__PM_ACT_MSK
#endif /* End Gyro_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants              
***************************************/

#if(Gyro_TX_ENABLED)
    #define Gyro_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End Gyro_TX_ENABLED */

#if(Gyro_HD_ENABLED)
    #define Gyro_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End Gyro_HD_ENABLED */

#if( (Gyro_RX_ENABLED) || (Gyro_HD_ENABLED) )
    #define Gyro_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (Gyro_RX_ENABLED) || (Gyro_HD_ENABLED) */


/***************************************
* Renamed global variables or defines 
* for backward compatible
***************************************/

#define Gyro_initvar                    Gyro_initVar

#define Gyro_RX_Enabled                 Gyro_RX_ENABLED
#define Gyro_TX_Enabled                 Gyro_TX_ENABLED
#define Gyro_HD_Enabled                 Gyro_HD_ENABLED
#define Gyro_RX_IntInterruptEnabled     Gyro_RX_INTERRUPT_ENABLED
#define Gyro_TX_IntInterruptEnabled     Gyro_TX_INTERRUPT_ENABLED
#define Gyro_InternalClockUsed          Gyro_INTERNAL_CLOCK_USED
#define Gyro_RXHW_Address_Enabled       Gyro_RXHW_ADDRESS_ENABLED
#define Gyro_OverSampleCount            Gyro_OVER_SAMPLE_COUNT
#define Gyro_ParityType                 Gyro_PARITY_TYPE

#if( Gyro_TX_ENABLED && (Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH))
    #define Gyro_TXBUFFER               Gyro_txBuffer
    #define Gyro_TXBUFFERREAD           Gyro_txBufferRead 
    #define Gyro_TXBUFFERWRITE          Gyro_txBufferWrite 
#endif /* End Gyro_TX_ENABLED */
#if( ( Gyro_RX_ENABLED || Gyro_HD_ENABLED ) && \
     (Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH) )
    #define Gyro_RXBUFFER               Gyro_rxBuffer
    #define Gyro_RXBUFFERREAD           Gyro_rxBufferRead 
    #define Gyro_RXBUFFERWRITE          Gyro_rxBufferWrite 
    #define Gyro_RXBUFFERLOOPDETECT     Gyro_rxBufferLoopDetect
    #define Gyro_RXBUFFER_OVERFLOW      Gyro_rxBufferOverflow
#endif /* End Gyro_RX_ENABLED */

#ifdef Gyro_BUART_sCR_AsyncCtl_CtrlReg__CONTROL_REG
    #define Gyro_CONTROL                Gyro_CONTROL_REG 
#endif /* End Gyro_BUART_sCR_AsyncCtl_CtrlReg__CONTROL_REG */

#if(Gyro_TX_ENABLED)
    #define Gyro_TXDATA                 Gyro_TXDATA_REG
    #define Gyro_TXSTATUS               Gyro_TXSTATUS_REG
    #define Gyro_TXSTATUS_MASK          Gyro_TXSTATUS_MASK_REG   
    #define Gyro_TXSTATUS_ACTL          Gyro_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(Gyro_TXCLKGEN_DP)
        #define Gyro_TXBITCLKGEN_CTR        Gyro_TXBITCLKGEN_CTR_REG
        #define Gyro_TXBITCLKTX_COMPLETE    Gyro_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define Gyro_TXBITCTR_PERIOD        Gyro_TXBITCTR_PERIOD_REG
        #define Gyro_TXBITCTR_CONTROL       Gyro_TXBITCTR_CONTROL_REG
        #define Gyro_TXBITCTR_COUNTER       Gyro_TXBITCTR_COUNTER_REG
    #endif /* Gyro_TXCLKGEN_DP */
#endif /* End Gyro_TX_ENABLED */

#if(Gyro_HD_ENABLED)
    #define Gyro_TXDATA                 Gyro_TXDATA_REG
    #define Gyro_TXSTATUS               Gyro_TXSTATUS_REG
    #define Gyro_TXSTATUS_MASK          Gyro_TXSTATUS_MASK_REG   
    #define Gyro_TXSTATUS_ACTL          Gyro_TXSTATUS_ACTL_REG
#endif /* End Gyro_HD_ENABLED */

#if( (Gyro_RX_ENABLED) || (Gyro_HD_ENABLED) )
    #define Gyro_RXDATA                 Gyro_RXDATA_REG             
    #define Gyro_RXADDRESS1             Gyro_RXADDRESS1_REG
    #define Gyro_RXADDRESS2             Gyro_RXADDRESS2_REG
    #define Gyro_RXBITCTR_PERIOD        Gyro_RXBITCTR_PERIOD_REG
    #define Gyro_RXBITCTR_CONTROL       Gyro_RXBITCTR_CONTROL_REG
    #define Gyro_RXBITCTR_COUNTER       Gyro_RXBITCTR_COUNTER_REG
    #define Gyro_RXSTATUS               Gyro_RXSTATUS_REG
    #define Gyro_RXSTATUS_MASK          Gyro_RXSTATUS_MASK_REG
    #define Gyro_RXSTATUS_ACTL          Gyro_RXSTATUS_ACTL_REG
#endif /* End  (Gyro_RX_ENABLED) || (Gyro_HD_ENABLED) */

#if(Gyro_INTERNAL_CLOCK_USED)
    #define Gyro_INTCLOCK_CLKEN         Gyro_INTCLOCK_CLKEN_REG
#endif /* End Gyro_INTERNAL_CLOCK_USED */

#define Gyro_WAIT_FOR_COMLETE_REINIT    Gyro_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_Gyro_H */


/* [] END OF FILE */
