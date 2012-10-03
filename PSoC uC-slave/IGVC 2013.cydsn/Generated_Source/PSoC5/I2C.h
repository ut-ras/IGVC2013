/*******************************************************************************
* File Name: I2C.h
* Version 3.1
*
* Description:
*  This file provides constants and parameter values for the I2C component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_I2C_I2C_H)
#define CY_I2C_I2C_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
*   Conditional Compilation Parameters
****************************************/

#define I2C_IMPLEMENTATION             (0u)
#define I2C_MODE                       (2u)
#define I2C_ADDR_DECODE                (1u)
#define I2C_ENABLE_WAKEUP              (0u)
#define I2C_I2C_PAIR_SELECTED          (0u)

/* I2C implementation types */
#define I2C_UDB                        (0x00u)
#define I2C_FF                         (0x01u)

/* I2C modes */
#define I2C_MODE_SLAVE                 (0x01u) /* I2C Slave Mode */
#define I2C_MODE_MASTER                (0x02u) /* I2C Master Mode */
#define I2C_MODE_MULTI_MASTER          (0x06u) /* I2C Multi-Master Mode */
#define I2C_MODE_MULTI_MASTER_SLAVE    (0x07u) /* I2C Multi-Master Slave Mode */
#define I2C_MODE_MULTI_MASTER_ENABLE   (0x04u) /* I2C Multi-Master Mode enable */

/* Address detection */
#define I2C_SW_DECODE                  (0x00u) /* Software address decode type */
#define I2C_HDWR_DECODE                (0x01u) /* Hardware address decode type */

#define I2C_I2C_PAIR0                  (0x01u) /* SIO pair 0 - P12[0] & P12[1] */
#define I2C_I2C_PAIR1                  (0x02u) /* SIO pair 1 - P12[4] & P12[5] */

/* Silicon defines validation */
#ifndef CY_PSOC3
    #error Component I2C_v3_1 requires cy_boot v2.30 or later
#endif


/***************************************
*       Type defines
***************************************/

/* Structure to save registers before go to sleep */
typedef struct _I2C_BACKUP_STRUCT
{
    uint8 enableState;
        
    #if (I2C_IMPLEMENTATION == I2C_FF)
        uint8 xcfg;
        uint8 cfg;
        
        #if (0u != (I2C_MODE & I2C_MODE_SLAVE))
            uint8 addr;
        #endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */
        
        #if (CY_PSOC3_ES2 || CY_PSOC5_ES1)
            uint8   clk_div;        /* only for TO3 */
        #else
            uint8   clk_div1;       /* only for TO4 */
            uint8   clk_div2;       /* only for TO4 */
        #endif  /* End  (CY_PSOC3_ES2 || CY_PSOC5_ES1) */
        
    #else /* (I2C_IMPLEMENTATION == I2C_UDB) */
        uint8 control;
        
        #if (CY_PSOC3_ES2 || CY_PSOC5_ES1)
            uint8 int_mask;         /* Status interrupt mask register */
            
            #if (0u != (I2C_MODE & I2C_MODE_SLAVE))
                uint8 addr;         /* D0 register */
            #endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */
        #else
            /* Retention registers for ES3:
                - Status Int mask: int_mask;
                - D0 register: addr;
                - Auxiliary Control: aux_ctl;
                - Period Register: always 7;
                - D0 and D1: clock generator 7, 15;
            */
        #endif  /* End (CY_PSOC3_ES2 || CY_PSOC5_ES1)*/
    #endif  /* End (I2C_IMPLEMENTATION == I2C_FF)*/
    
} I2C_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

void I2C_Init(void) ;
void I2C_Enable(void) ;

void I2C_Start(void) ;
void I2C_Stop(void) ;
#define I2C_EnableInt()        CyIntEnable(I2C_ISR_NUMBER)
#define I2C_DisableInt()       CyIntDisable(I2C_ISR_NUMBER)

void I2C_SaveConfig(void) ;
void I2C_Sleep(void) ;
void I2C_RestoreConfig(void) ;
void I2C_Wakeup(void) ;

/* I2C Master functions prototypes */
#if (0u != (I2C_MODE & I2C_MODE_MASTER))
    /* Read and Clear status functions */
    uint8  I2C_MasterStatus(void) ;
    uint8  I2C_MasterClearStatus(void) ;
    
    /* Interrupt based operation functions */
    uint8  I2C_MasterWriteBuf(uint8 slaveAddress, uint8 * wrData, uint8 cnt, uint8 mode) 
           ;
    uint8  I2C_MasterReadBuf(uint8 slaveAddress, uint8 * rdData, uint8 cnt, uint8 mode) 
            ;
    uint16 I2C_MasterGetReadBufSize(void) ;
    uint16 I2C_MasterGetWriteBufSize(void) ;
    void   I2C_MasterClearReadBuf(void) ;
    void   I2C_MasterClearWriteBuf(void) ;
    
    /* Manual operation functions */
    uint8  I2C_MasterSendStart(uint8 slaveAddress, uint8 R_nW)
           ;
    uint8  I2C_MasterSendRestart(uint8 slaveAddress, uint8 R_nW) 
           ;
    uint8  I2C_MasterSendStop(void) ;
    uint8  I2C_MasterWriteByte(uint8 theByte) ;
    uint8  I2C_MasterReadByte(uint8 acknNak) ;
    
    /* This fake function use as workaround for CDT 78083 */
    void   I2C_Workaround(void) ;
    
#endif  /* End (0u != (I2C_MODE & I2C_MODE_MASTER)) */

/* I2C Slave functions prototypes */
#if (0u != (I2C_MODE & I2C_MODE_SLAVE))
    /* Read and Clear status functions */
    uint8 I2C_SlaveStatus(void) ;
    uint8 I2C_SlaveClearReadStatus(void) ;
    uint8 I2C_SlaveClearWriteStatus(void) ;
    
    void  I2C_SlaveSetAddress(uint8 address) ;
    
    /* Interrupt based operation functions */
    void  I2C_SlaveInitReadBuf(uint8 * rdBuf, uint8 bufSize)
          ;
    void  I2C_SlaveInitWriteBuf(uint8 * wrBuf, uint8 bufSize)
          ;
    uint8 I2C_SlaveGetReadBufSize(void) ;
    uint8 I2C_SlaveGetWriteBufSize(void) ;
    void  I2C_SlaveClearReadBuf(void) ;
    void  I2C_SlaveClearWriteBuf(void) ;
    
    /* Communication bootloader I2C Slave APIs */
    #if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_I2C) || \
                                              (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
        /* Physical layer functions */
        void I2C_CyBtldrCommStart(void) CYSMALL ;
        void I2C_CyBtldrCommStop(void) CYSMALL ;
        void I2C_CyBtldrCommReset(void) CYSMALL ;
        cystatus I2C_CyBtldrCommWrite(uint8 * Data, uint16 size, uint16 * count, uint8 timeOut) CYSMALL 
                 ;
        cystatus I2C_CyBtldrCommRead(uint8 * Data, uint16 size, uint16 * count, uint8 timeOut) CYSMALL 
                 ;
        
        #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_I2C)
            #define CyBtldrCommStart    I2C_CyBtldrCommStart
            #define CyBtldrCommStop     I2C_CyBtldrCommStop
            #define CyBtldrCommReset    I2C_CyBtldrCommReset
            #define CyBtldrCommWrite    I2C_CyBtldrCommWrite
            #define CyBtldrCommRead     I2C_CyBtldrCommRead
        #endif  /* End (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_I2C)*/
        
        /* Size of Read/Write buffers for I2C bootloader  */
        #define I2C_BTLDR_SIZEOF_READ_BUFFER   (0x80u)
        #define I2C_BTLDR_SIZEOF_WRITE_BUFFER  (0x80u)
        #define I2C_MIN_UINT16(a, b)           ( ((uint16)(a) < (b)) ? ((uint16) (a)) : ((uint16) (b)) )
        
    #endif /* End (CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_I2C) || \
                                                 (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface)) */

#endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */

/* Interrupt handler */
CY_ISR_PROTO(I2C_ISR);

#if (CY_PSOC3_ES2 && (I2C_I2C_IRQ__ES2_PATCH))
    #include <intrins.h>
    #define I2C_ISR_PATCH() _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
#endif  /* End (CY_PSOC3_ES2 && (I2C_I2C_IRQ__ES2_PATCH)) */


/***************************************
*   Initial Parameter Constants
***************************************/

#define I2C_DEFAULT_ADDR               (8u) 
#define I2C_DATA_RATE                  (100u)

#define I2C_DATA_RATE_50               (50u)
#define I2C_DATA_RATE_100              (100u)

/*
    CLK_DIV = DataRate(kbps) / (DataRate * OversampleRate);
    For Slave picks up the grater unsigned integer.
    For Master/MultiMaster/MultiMaster-Slave picks up the smallest unsigned integer(round).
    The OversampleRate equal 16 for DataRate >= 100, for others 32 (truncate).
    The real BusSpeed could be differ from desired due division with round/truncate.
*/
#if (CY_PSOC3_ES2 || CY_PSOC5_ES1)
    /* Define CLK_DIV */
    #define I2C_DEFAULT_DIVIDE_FACTOR      (0u)
    
    /* Define proper clock rate according to Data Rate */
    #if (I2C_DATA_RATE <= I2C_DATA_RATE_50)
        #define I2C_DEFAULT_CLK_RATE       I2C_CFG_CLK_RATE_050
    #elif (I2C_DATA_RATE <= I2C_DATA_RATE_100)
        #define I2C_DEFAULT_CLK_RATE       I2C_CFG_CLK_RATE_100
    #else
        #define I2C_DEFAULT_CLK_RATE       I2C_CFG_CLK_RATE_400
    #endif  /* End (I2C_DATA_RATE <= I2C_DATA_RATE_50KHZ) */
    
#else
    /* Define CLK_DIV1 and CLK_DIV2 */
    #define I2C_DEFAULT_DIVIDE_FACTOR      ((uint16) 1u )
    
    /* Define proper clock rate according to Data Rate */
    #if (I2C_DATA_RATE <= I2C_DATA_RATE_50)
        #define I2C_DEFAULT_CLK_RATE       I2C_CFG_CLK_RATE_LESS_EQUAL_50
    #else
        #define I2C_DEFAULT_CLK_RATE       I2C_CFG_CLK_RATE_GRATER_50
    #endif  /* End (I2C_DATA_RATE <= I2C_DATA_RATE_50KHZ) */
    
#endif /* End (I2C_PSOC3_ES2 || I2C_PSOC5_ES1) */


/***************************************
* I2C state machine constants
***************************************/

/* Default slave address states */
#define  I2C_DEV_MASK                  (0xF0u)    /* Wait for sub-address */
#define  I2C_SM_IDLE                   (0x10u)    /* Idle I2C state */
#define  I2C_DEV_MASTER_XFER           (0x40u)    /* Wait for sub-address */

/* Default slave address states */
#define  I2C_SM_SL_WR_DATA             (0x11u)    /* Slave waiting for master to write data */
#define  I2C_SM_SL_RD_DATA             (0x12u)    /* Slave waiting for master to read data */
#define  I2C_SM_SL_STOP                (0x14u)    /* Slave waiting for stop */

/* Master mode states */
#define  I2C_SM_MASTER                 (0x40u)    /* Master or Multi-Master mode is set */
#define  I2C_SM_MASTER_IDLE            (0x40u)    /* Hardware in Master mode and sitting idle */

#define  I2C_SM_MSTR_RD                (0x08u)    /* Mask for Read states */
#define  I2C_SM_MSTR_RD_ADDR           (0x49u)    /* Master has sent a Start/Address/RD */
#define  I2C_SM_MSTR_RD_DATA           (0x4Au)    /* Master is receiving data from external slave */

#define  I2C_SM_MSTR_WR                (0x04u)    /* Mask for Write states */
#define  I2C_SM_MSTR_WR_ADDR           (0x45u)    /* Master has sent a Start/Address/WR */
#define  I2C_SM_MSTR_WR_DATA           (0x46u)    /* Master is writing data to external slave */

#define  I2C_SM_MSTR_HALT              (0x60u)    /* Master Halt state */


/***************************************
*            API Constants
***************************************/

/* Master/Slave control constants */
#define I2C_READ_XFER_MODE             (0x01u)    /* Read */
#define I2C_WRITE_XFER_MODE            (0x00u)    /* Write */
#define I2C_ACK_DATA                   (0x01u)    /* Send ACK */
#define I2C_NAK_DATA                   (0x00u)    /* Send NAK */

#if (0u != (I2C_MODE & I2C_MODE_MASTER))
    /* "Mode" constants for MasterWriteBuf() or MasterReadBuf() function */
    #define I2C_MODE_COMPLETE_XFER     (0x00u)    /* Full transfer with Start and Stop */
    #define I2C_MODE_REPEAT_START      (0x01u)    /* Begin with a ReStart instead of a Start */
    #define I2C_MODE_NO_STOP           (0x02u)    /* Complete the transfer without a Stop */

    /* Master status */
    #define I2C_MSTAT_CLEAR            (0x00u)    /* Clear (init) status value */
    
    #define I2C_MSTAT_RD_CMPLT         (0x01u)    /* Read complete */
    #define I2C_MSTAT_WR_CMPLT         (0x02u)    /* Write complete */
    #define I2C_MSTAT_XFER_INP         (0x04u)    /* Master transfer in progress */
    #define I2C_MSTAT_XFER_HALT        (0x08u)    /* Transfer is halted */
    
    #define I2C_MSTAT_ERR_MASK         (0xF0u)    /* Mask for all errors */
    #define I2C_MSTAT_ERR_SHORT_XFER   (0x10u)    /* Master NAKed before end of packet */
    #define I2C_MSTAT_ERR_ADDR_NAK     (0x20u)    /* Slave did not ACK */
    #define I2C_MSTAT_ERR_ARB_LOST     (0x40u)    /* Master lost arbitration during communication */
    #define I2C_MSTAT_ERR_XFER         (0x80u)    /* Error during transfer */
    
    /* Master API returns */
    #define I2C_MSTR_NO_ERROR          (0x00u)    /* Function complete without error */
    #define I2C_MSTR_BUS_BUSY          (0x01u)    /* Bus is busy, process not started */
    #define I2C_MSTR_NOT_READY         (0x02u)    /* Master not Master on the bus or 
                                                                  Slave operation in progress */
    #define I2C_MSTR_ERR_LB_NAK        (0x03u)    /* Last Byte Naked */
    #define I2C_MSTR_ERR_ARB_LOST      (0x04u)    /* Master lost arbitration during communication */
    #define I2C_MSTR_ERR_ABORT_START_GEN  (0x05u) /* Master did not generate Start, the Slave was addressed before */
    
    /* mstrControl bit definitions */
    #define  I2C_MSTR_GEN_STOP         (0x01u)    /* Generate a stop after a data transfer */
    #define  I2C_MSTR_NO_STOP          (0x01u)    /* Do not generate a stop after a data transfer */
    
    #define I2C_READ_FLAG              (0x01u)     /* Read flag of the Address */
    
#endif  /* End (0u != (I2C_MODE & I2C_MODE_MASTER)) */

#if (0u != (I2C_MODE & I2C_MODE_SLAVE))
    /* Slave Status Constants */
    #define I2C_SSTAT_RD_CMPLT         (0x01u)    /* Read transfer complete */
    #define I2C_SSTAT_RD_BUSY          (0x02u)    /* Read transfer in progress */
    #define I2C_SSTAT_RD_ERR_OVFL      (0x04u)    /* Read overflow Error */
    #define I2C_SSTAT_RD_MASK          (0x0Fu)    /* Read Status Mask */
    #define I2C_SSTAT_RD_NO_ERR        (0x00u)    /* Read no Error */
    
    #define I2C_SSTAT_WR_CMPLT         (0x10u)    /* Write transfer complete */
    #define I2C_SSTAT_WR_BUSY          (0x20u)    /* Write transfer in progress */
    #define I2C_SSTAT_WR_ERR_OVFL      (0x40u)    /* Write overflow Error */
    #define I2C_SSTAT_WR_MASK          (0xF0u)    /* Write Status Mask  */
    #define I2C_SSTAT_WR_NO_ERR        (0x00u)    /* Write no Error */
    
    #define I2C_SSTAT_RD_CLEAR         (0x0Du)    /* Read Status clear */
    #define I2C_SSTAT_WR_CLEAR         (0xD0u)    /* Write Status Clear */
    
#endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */

/* Deprecated constants */
#define I2C_SSTAT_RD_ERR               (0x08u)
#define I2C_SSTAT_WR_ERR               (0x80u)
#define I2C_MSTR_SLAVE_BUSY            I2C_MSTR_NOT_READY
#define I2C_MSTAT_ERR_BUF_OVFL         (0x80u)
#define I2C_SSTAT_RD_CMPT              I2C_SSTAT_RD_CMPLT
#define I2C_SSTAT_WR_CMPT              I2C_SSTAT_WR_CMPLT
    
    
/***************************************
*              Registers
***************************************/

#if (I2C_IMPLEMENTATION == I2C_FF)
    /* Fixed Function registers */
    #define I2C_XCFG_REG               (*(reg8 *) I2C_I2C_FF__XCFG )
    #define I2C_XCFG_PTR               ( (reg8 *) I2C_I2C_FF__XCFG )
    
    #define I2C_ADDR_REG               (*(reg8 *) I2C_I2C_FF__ADR )
    #define I2C_ADDR_PTR               ( (reg8 *) I2C_I2C_FF__ADR )
    
    #define I2C_CFG_REG                (*(reg8 *) I2C_I2C_FF__CFG )
    #define I2C_CFG_PTR                ( (reg8 *) I2C_I2C_FF__CFG )
    
    #define I2C_CSR_REG                (*(reg8 *) I2C_I2C_FF__CSR )
    #define I2C_CSR_PTR                ( (reg8 *) I2C_I2C_FF__CSR )
    
    #define I2C_DATA_REG               (*(reg8 *) I2C_I2C_FF__D )
    #define I2C_DATA_PTR               ( (reg8 *) I2C_I2C_FF__D )
    
    #define I2C_MCSR_REG               (*(reg8 *) I2C_I2C_FF__MCSR )
    #define I2C_MCSR_PTR               ( (reg8 *) I2C_I2C_FF__MCSR )
    
    #define I2C_ACT_PWRMGR_REG         (*(reg8 *) I2C_I2C_FF__PM_ACT_CFG )
    #define I2C_ACT_PWRMGR_PTR         ( (reg8 *) I2C_I2C_FF__PM_ACT_CFG )
    #define I2C_ACT_PWR_EN                        I2C_I2C_FF__PM_ACT_MSK
    
    #define I2C_STBY_PWRMGR_REG        (*(reg8 *) I2C_I2C_FF__PM_STBY_CFG )
    #define I2C_STBY_PWRMGR_PTR        ( (reg8 *) I2C_I2C_FF__PM_STBY_CFG ) 
    #define I2C_STBY_PWR_EN                       I2C_I2C_FF__PM_STBY_MSK
    
    #define I2C_PWRSYS_CR1_REG         (*(reg8 *) CYREG_PWRSYS_CR1 )
    #define I2C_PWRSYS_CR1_PTR         ( (reg8 *) CYREG_PWRSYS_CR1 )
    
    /* Clock divider register depends on silicon */
    #if (CY_PSOC3_ES2 || CY_PSOC5_ES1)
        #define I2C_CLKDIV_REG         (*(reg8 *) I2C_I2C_FF__CLK_DIV )
        #define I2C_CLKDIV_PTR         ( (reg8 *) I2C_I2C_FF__CLK_DIV )
        
    #else
        #define I2C_CLKDIV1_REG        (*(reg8 *) I2C_I2C_FF__CLK_DIV1 )
        #define I2C_CLKDIV1_PTR        ( (reg8 *) I2C_I2C_FF__CLK_DIV1 )
        #define I2C_CLKDIV2_REG        (*(reg8 *) I2C_I2C_FF__CLK_DIV2 )
        #define I2C_CLKDIV2_PTR        ( (reg8 *) I2C_I2C_FF__CLK_DIV2 )
        
    #endif  /* End (CY_PSOC3_ES2 || CY_PSOC5_ES1)*/
    
#else

    /* UDB implementation registers */
    #define I2C_CFG_REG    (*(reg8 *) \
                                           I2C_bI2C_UDB_AsyncCtl_CtrlReg__CONTROL_REG )
    #define I2C_CFG_PTR    ( (reg8 *) \
                                           I2C_bI2C_UDB_AsyncCtl_CtrlReg__CONTROL_REG )
    
    #define I2C_CSR_REG                (*(reg8 *) I2C_bI2C_UDB_StsReg__STATUS_REG )
    #define I2C_CSR_PTR                ( (reg8 *) I2C_bI2C_UDB_StsReg__STATUS_REG )
    
    #define I2C_INT_MASK_REG           (*(reg8 *) I2C_bI2C_UDB_StsReg__MASK_REG )
    #define I2C_INT_MASK_PTR           ( (reg8 *) I2C_bI2C_UDB_StsReg__MASK_REG )
    
    #define I2C_INT_ENABLE_REG         (*(reg8 *) I2C_bI2C_UDB_StsReg__STATUS_AUX_CTL_REG )
    #define I2C_INT_ENABLE_PTR         ( (reg8 *) I2C_bI2C_UDB_StsReg__STATUS_AUX_CTL_REG )
    
    #define I2C_DATA_REG               (*(reg8 *) I2C_bI2C_UDB_Shifter_u0__A0_REG )
    #define I2C_DATA_PTR               ( (reg8 *) I2C_bI2C_UDB_Shifter_u0__A0_REG )
    
    #define I2C_GO_REG                 (*(reg8 *) I2C_bI2C_UDB_Shifter_u0__F1_REG )
    #define I2C_GO_PTR                 ( (reg8 *) I2C_bI2C_UDB_Shifter_u0__F1_REG )
    
    #define I2C_MCLK_PRD_REG           (*(reg8 *) I2C_bI2C_UDB_Master_ClkGen_u0__D0_REG )
    #define I2C_MCLK_PRD_PTR           ( (reg8 *) I2C_bI2C_UDB_Master_ClkGen_u0__D0_REG )
    
    #define I2C_MCLK_CMP_REG           (*(reg8 *) I2C_bI2C_UDB_Master_ClkGen_u0__D1_REG )
    #define I2C_MCLK_CMP_PTR           ( (reg8 *) I2C_bI2C_UDB_Master_ClkGen_u0__D1_REG )
    
    #if (0u != (I2C_MODE & I2C_MODE_SLAVE))
        /* UDB implementation registers --- Slave only */
        #define I2C_ADDR_REG       (*(reg8 *) I2C_bI2C_UDB_Shifter_u0__D0_REG )
        #define I2C_ADDR_PTR       ( (reg8 *) I2C_bI2C_UDB_Shifter_u0__D0_REG )
        
        #define I2C_PERIOD_REG     (*(reg8 *) I2C_bI2C_UDB_Slave_BitCounter__PERIOD_REG )
        #define I2C_PERIOD_PTR     ( (reg8 *) I2C_bI2C_UDB_Slave_BitCounter__PERIOD_REG )
        
        #define I2C_COUNTER_REG    (*(reg8 *) I2C_bI2C_UDB_Slave_BitCounter__COUNT_REG )
        #define I2C_COUNTER_PTR    ( (reg8 *) I2C_bI2C_UDB_Slave_BitCounter__COUNT_REG )
        
        #define I2C_COUNTER_AUX_CTL_REG  (*(reg8 *) \
                                                       I2C_bI2C_UDB_Slave_BitCounter__CONTROL_AUX_CTL_REG )
        #define I2C_COUNTER_AUX_CTL_PTR  ( (reg8 *) \
                                                       I2C_bI2C_UDB_Slave_BitCounter__CONTROL_AUX_CTL_REG )
        
    #endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */
    
#endif  /* End (I2C_IMPLEMENTATION == I2C_FF) */


/***************************************
*        Registers Constants       
***************************************/ 

/* Number and priority of the I2C interrupt */
#define I2C_ISR_NUMBER                 I2C_I2C_IRQ__INTC_NUMBER
#define I2C_ISR_PRIORITY               I2C_I2C_IRQ__INTC_PRIOR_NUM

/* I2C Slave Data Register */
#define I2C_SLAVE_ADDR_MASK            (0x7Fu)
#define I2C_SLAVE_ADDR_SHIFT           (0x01u)
#define I2C_DATA_MASK                  (0xFFu)
#define I2C_READ_FLAG                  (0x01u)

#if (I2C_IMPLEMENTATION == I2C_FF)
    /* XCFG I2C Extended Configuration Register */
    #define I2C_XCFG_CLK_EN            (0x80u)    /* Enable gated clock to block */
    #define I2C_XCFG_I2C_ON            (0x40u)    /* Enable I2C as wake up source*/
    #define I2C_XCFG_RDY_TO_SLEEP      (0x20u)    /* I2C ready go to sleep */
    #define I2C_XCFG_FORCE_NACK        (0x10u)    /* Force NACK all incomming transactions */
    #define I2C_XCFG_NO_BC_INT         (0x08u)    /* No interrupt on byte complete */
    #define I2C_XCFG_BUF_MODE          (0x02u)    /* Enable buffer mode */
    #define I2C_XCFG_HDWR_ADDR_EN      (0x01u)    /* Enable Hardware address match */
    
    /* CFG I2C Configuration Register */
    #define I2C_CFG_SIO_SELECT         (0x80u)    /* Pin Select for SCL/SDA lines */
    #define I2C_CFG_PSELECT            (0x40u)    /* Pin Select */
    #define I2C_CFG_BUS_ERR_IE         (0x20u)    /* Bus Error Interrupt Enable */
    #define I2C_CFG_STOP_IE            (0x10u)    /* Enable Interrupt on STOP condition */
    #define I2C_CFG_STOP_ERR_IE        (0x10u)    /* Enable Interrupt on STOP condition */
    #define I2C_CFG_CLK_RATE_MSK       (0x0Cu)    /* Clock rate select  **CHECK**  */
    #define I2C_CFG_CLK_RATE_100       (0x00u)    /* Clock rate select 100K */
    #define I2C_CFG_CLK_RATE_400       (0x04u)    /* Clock rate select 400K */
    #define I2C_CFG_CLK_RATE_050       (0x08u)    /* Clock rate select 50K  */
    #define I2C_CFG_CLK_RATE_RSVD      (0x0Cu)    /* Clock rate select Invalid */
    #define I2C_CFG_EN_MSTR            (0x02u)    /* Enable Master operation */
    #define I2C_CFG_EN_SLAVE           (0x01u)    /* Enable Slave operation */
    
    #define I2C_CFG_CLK_RATE_LESS_EQUAL_50 (0x04u) /* Clock rate select <= 50kHz */
    #define I2C_CFG_CLK_RATE_GRATER_50     (0x00u) /* Clock rate select > 50kHz */
   
    /* CSR I2C Control and Status Register */
    #define I2C_CSR_BUS_ERROR          (0x80u)    /* Active high when bus error has occured */
    #define I2C_CSR_LOST_ARB           (0x40u)    /* Set to 1 if lost arbitration in host mode */
    #define I2C_CSR_STOP_STATUS        (0x20u)    /* Set if Stop has been detected */
    #define I2C_CSR_ACK                (0x10u)    /* ACK response */
    #define I2C_CSR_NAK                (0x00u)    /* NAK response */
    #define I2C_CSR_ADDRESS            (0x08u)    /* Set in firmware 0 = status bit, 1 Address is slave */
    #define I2C_CSR_TRANSMIT           (0x04u)    /* Set in firmware 1 = transmit, 0 = receive */
    #define I2C_CSR_LRB                (0x02u)    /* Last received bit */
    #define I2C_CSR_LRB_ACK            (0x00u)    /* Last received bit was an ACK */
    #define I2C_CSR_LRB_NAK            (0x02u)    /* Last received bit was an NAK */
    #define I2C_CSR_BYTE_COMPLETE      (0x01u)    /* Informs that last byte has been sent */
    #define I2C_CSR_STOP_GEN           (0x00u)    /* Generate a stop condition */
    #define I2C_CSR_RDY_TO_RD          (0x00u)    /* Set to recieve mode */
    
    /* MCSR I2C Master Control and Status Register */
    #define I2C_MCSR_STOP_GEN          (0x10u)    /* Firmware sets this bit to initiate a Stop condition */
    #define I2C_MCSR_BUS_BUSY          (0x08u)    /* Status bit, Set at Start and cleared at Stop condition */
    #define I2C_MCSR_MSTR_MODE         (0x04u)    /* Status bit, Set at Start and cleared at Stop condition */
    #define I2C_MCSR_RESTART_GEN       (0x02u)    /* Firmware sets this bit to initiate a ReStart condition */
    #define I2C_MCSR_START_GEN         (0x01u)    /* Firmware sets this bit to initiate a Start condition */
    
    /* CLK_DIV I2C Clock Divide Factor Register */
    #define I2C_CLK_DIV_MSK            (0x07u)    /* Status bit, Set at Start and cleared at Stop condition */
    #define I2C_CLK_DIV_1              (0x00u)    /* Divide input clock by  1 */
    #define I2C_CLK_DIV_2              (0x01u)    /* Divide input clock by  2 */
    #define I2C_CLK_DIV_4              (0x02u)    /* Divide input clock by  4 */
    #define I2C_CLK_DIV_8              (0x03u)    /* Divide input clock by  8 */
    #define I2C_CLK_DIV_16             (0x04u)    /* Divide input clock by 16 */
    #define I2C_CLK_DIV_32             (0x05u)    /* Divide input clock by 32 */
    #define I2C_CLK_DIV_64             (0x06u)    /* Divide input clock by 64 */
    
    /* PWRSYS_CR1 to handle Sleep */
    #define I2C_PWRSYS_CR1_I2C_REG_BACKUP      (0x04u)    /* Enables, power to I2C regs while sleep */
    
    /* UDB compatible defines */
    #define I2C_DISABLE_INT_ON_STOP        { I2C_CFG_REG &= ~I2C_CFG_STOP_IE; }
    #define I2C_ENABLE_INT_ON_STOP         { I2C_CFG_REG |= I2C_CFG_STOP_IE; }
    
    #define I2C_TRANSMIT_DATA              { I2C_CSR_REG = I2C_CSR_TRANSMIT; }
    #define I2C_ACK_AND_TRANSMIT           { I2C_CSR_REG = (I2C_CSR_ACK | \
                                                                                      I2C_CSR_TRANSMIT); \
                                                        }
                                                        
    #define I2C_NAK_AND_TRANSMIT           { I2C_CSR_REG = I2C_CSR_NAK; }
    
    /* Special case: udb needs to ack, ff needs to nak. */
    #define I2C_ACKNAK_AND_TRANSMIT        { I2C_CSR_REG  = (I2C_CSR_NAK | \
                                                                                       I2C_CSR_TRANSMIT); \
                                                        }
    
    #define I2C_ACK_AND_RECEIVE            { I2C_CSR_REG = I2C_CSR_ACK; }
    #define I2C_NAK_AND_RECEIVE            { I2C_CSR_REG = I2C_CSR_NAK; }
    #define I2C_READY_TO_READ              { I2C_CSR_REG = I2C_CSR_RDY_TO_RD; }
        
    #define I2C_GENERATE_START             { I2C_MCSR_REG = I2C_MCSR_START_GEN; }
    
    #if (CY_PSOC3_ES2 || CY_PSOC5_ES1)
        #define I2C_GENERATE_RESTART   { I2C_MCSR_REG = I2C_MCSR_RESTART_GEN; \
                                                      I2C_NAK_AND_RECEIVE; \
                                                    }
                                                    
        #define I2C_GENERATE_STOP      { I2C_CSR_REG = I2C_CSR_STOP_GEN; }
    
    #else   /* PSoC3 ES3 handlees zero lenght packets */
        #define I2C_GENERATE_RESTART   { I2C_MCSR_REG = (I2C_MCSR_RESTART_GEN | \
                                                                                   I2C_MCSR_STOP_GEN); \
                                                      I2C_TRANSMIT_DATA; \
                                                    }
                                                    
        #define I2C_GENERATE_STOP      { I2C_MCSR_REG = I2C_MCSR_STOP_GEN; \
                                                      I2C_TRANSMIT_DATA; }
    #endif  /* End (CY_PSOC3_ES2 || CY_PSOC5_ES1) */
    

#else
    /* Control Register Bit Locations */
    #define I2C_CTRL_START_SHIFT           (7u)
    #define I2C_CTRL_START_MASK            ( 0x01u << I2C_CTRL_START_SHIFT )
    
    #define I2C_CTRL_STOP_SHIFT            (6u)
    #define I2C_CTRL_STOP_MASK             ( 0x01u << I2C_CTRL_STOP_SHIFT )
    
    #define I2C_CTRL_RESTART_SHIFT         (5u)
    #define I2C_CTRL_RESTART_MASK          ( 0x01u << I2C_CTRL_RESTART_SHIFT)
               
    #define I2C_CTRL_NACK_SHIFT            (4u)
    #define I2C_CTRL_NACK_MASK             ( 0x01u << I2C_CTRL_NACK_SHIFT )
    
    #define I2C_CTRL_ANY_ADDRESS_SHIFT     (3u)
    #define I2C_CTRL_ANY_ADDRESS_MASK      ( 0x01u << I2C_CTRL_ANY_ADDRESS_SHIFT ) 
        
    #define I2C_CTRL_TRANSMIT_SHIFT        (2u)
    #define I2C_CTRL_TRANSMIT_MASK         ( 0x01u << I2C_CTRL_TRANSMIT_SHIFT )
    
    #define I2C_CTRL_ENABLE_MASTER_SHIFT   (1u)
    #define I2C_CTRL_ENABLE_MASTER_MASK    ( 0x01u << I2C_CTRL_ENABLE_MASTER_SHIFT )
    
    #define I2C_CTRL_ENABLE_SLAVE_SHIFT    (0u)
    #define I2C_CTRL_ENABLE_SLAVE_MASK     ( 0x01u << I2C_CTRL_ENABLE_SLAVE_SHIFT )


    /* Status Register Bit Locations */
    #define I2C_STS_LOST_ARB_SHIFT         (6u)
    #define I2C_STS_LOST_ARB_MASK          ( 0x01u << I2C_STS_LOST_ARB_SHIFT )
    
    /* NOT available in Master mode */
    #define I2C_STS_STOP_SHIFT             (5u)
    #define I2C_STS_STOP_MASK              ( 0x01u << I2C_STS_STOP_SHIFT )

    #define I2C_STS_BUSY_SHIFT             (4u)
    #define I2C_STS_BUSY_MASK              ( 0x01u << I2C_STS_BUSY_SHIFT )
    
    #define I2C_STS_ADDR_SHIFT             (3u)
    #define I2C_STS_ADDR_MASK              ( 0x01u << I2C_STS_ADDR_SHIFT )
    
    #define I2C_STS_MASTER_MODE_SHIFT      (2u)
    #define I2C_STS_MASTER_MODE_MASK       ( 0x01u << I2C_STS_MASTER_MODE_SHIFT )

    #define I2C_STS_LRB_SHIFT              (1u)
    #define I2C_STS_LRB_MASK               ( 0x01u << I2C_STS_LRB_SHIFT )

    #define I2C_STS_BYTE_COMPLETE_SHIFT    (0u)
    #define I2C_STS_BYTE_COMPLETE_MASK     ( 0x01u << I2C_STS_BYTE_COMPLETE_SHIFT )
    
    /* Master clock devider values */
    #define I2C_MCLK_PERIOD_VALUE      (0x0Fu)
    #define I2C_MCLK_COMPARE_VALUE     (0x08u)
    
    /* Enable counter and interrupts mask */
    #define I2C_COUNTER_ENABLE_MASK    (0x20u)     /* Enable counter7 */
    #define I2C_INT_ENABLE_MASK        (0x10u)     /* Enable interrupts */
    
    /* Conter period register value */ 
    #define I2C_PERIOD_VALUE           (0x07u)
     
    /* Masks to enalbe interrupts from Status register */
    #define I2C_STOP_IE_MASK               I2C_STS_STOP_MASK
    #define I2C_BYTE_COMPLETE_IE_MASK      I2C_STS_BYTE_COMPLETE_MASK
    
    /* Defines to make UDB match Fixed Function */
    #define I2C_CSR_LOST_ARB           I2C_STS_LOST_ARB_MASK          /* Set if Master Lost Arbitrage while send Start */
    #define I2C_CSR_STOP_STATUS        I2C_STS_STOP_MASK              /* Set if Stop has been detected */
    #define I2C_CSR_BUS_ERROR          (0x00u)                                     /* NOT used - Active high when bus error has occured */
    #define I2C_CSR_ADDRESS            I2C_STS_ADDR_MASK              /* Set in firmware 0 = status bit, 1 Address is slave */
    #define I2C_CSR_TRANSMIT           I2C_CTRL_TRANSMIT_MASK         /* Set in firmware 1 = transmit, 0 = receive. */
    #define I2C_CSR_LRB                I2C_STS_LRB_MASK               /* Last received bit */
    #define I2C_CSR_LRB_NAK            I2C_STS_LRB_MASK               /* Last received bit was an NAK */
    #define I2C_CSR_LRB_ACK            (0x00u)                                     /* Last received bit was an ACK */
    #define I2C_CSR_BYTE_COMPLETE      I2C_STS_BYTE_COMPLETE_MASK     /* Informs that last byte has been sent */
                
    /* MCSR Registers definitions */
    #define I2C_MCSR_REG           I2C_CSR_REG
    #define I2C_MCSR_BUS_BUSY      I2C_STS_BUSY_MASK
    #define I2C_MCSR_START_GEN     I2C_CTRL_START_MASK        /* Generate Sart condition */
    #define I2C_MCSR_RESTART_GEN   I2C_CTRL_RESTART_MASK      /* Generates RESTART condition */
    #define I2C_MCSR_MSTR_MODE     I2C_STS_MASTER_MODE_MASK   /* Define if Master drives the bus */

      
    /* FF compatible defines */
    #define I2C_DISABLE_INT_ON_STOP    { I2C_INT_MASK_REG &= ~I2C_STOP_IE_MASK; }
    #define I2C_ENABLE_INT_ON_STOP     { I2C_INT_MASK_REG |= I2C_STOP_IE_MASK; }
    
    /* Transmits data used by Slave and Master */
    #define I2C_TRANSMIT_DATA      { I2C_CFG_REG |= I2C_CTRL_TRANSMIT_MASK; \
                                                  I2C_CFG_REG &= ~(I2C_CTRL_NACK_MASK | \
                                                                                I2C_CTRL_RESTART_MASK | \
                                                                                I2C_CTRL_STOP_MASK | \
                                                                                I2C_CTRL_START_MASK); \
                                                  I2C_GO_REG = 0x00u; \
                                                }
    
    /* Slave only macro, used in address phase of read transaction */
    #define I2C_ACK_AND_TRANSMIT   { I2C_CFG_REG |= I2C_CTRL_TRANSMIT_MASK; \
                                                  I2C_CFG_REG &= ~(I2C_CTRL_NACK_MASK | \
                                                                                I2C_CTRL_RESTART_MASK | \
                                                                                I2C_CTRL_STOP_MASK | \
                                                                                I2C_CTRL_START_MASK); \
                                                  I2C_GO_REG = 0x00u; \
                                                }
    
    /* Slave only macro, used to terminate read transaction 
        - temporary add clear of: _CTRL_RESTART_MASK and _CTRL_STOP_MASK
    */
    #define I2C_NAK_AND_TRANSMIT   { I2C_CFG_REG |= (I2C_CTRL_NACK_MASK | \
                                                                               I2C_CTRL_TRANSMIT_MASK); \
                                                  I2C_CFG_REG &= ~(I2C_CTRL_RESTART_MASK | \
                                                                                I2C_CTRL_STOP_MASK | \
                                                                                I2C_CTRL_START_MASK); \
                                                  I2C_GO_REG = 0x00u; \
                                                }
    /* ACK and receives one byte - used by master and slave to receive data
       - temporary add clear of: _CTRL_RESTART_MASK and _CTRL_STOP_MASK
    */
    #define I2C_ACK_AND_RECEIVE    { I2C_CFG_REG &= ~(I2C_CTRL_TRANSMIT_MASK | \
                                                                                I2C_CTRL_NACK_MASK | \
                                                                                I2C_CTRL_RESTART_MASK | \
                                                                                I2C_CTRL_STOP_MASK | \
                                                                                I2C_CTRL_START_MASK ); \
                                                  I2C_GO_REG = 0x00u; \
                                                }
    
    /* NACK and recieves one byte - used by master and slave to end receive data
        - temporary add clear of: _CTRL_RESTART_MASK and _CTRL_STOP_MASK
    */
    #define I2C_NAK_AND_RECEIVE    { I2C_CFG_REG |= I2C_CTRL_NACK_MASK; \
                                                  I2C_CFG_REG &= ~(I2C_CTRL_TRANSMIT_MASK | \
                                                                                I2C_CTRL_RESTART_MASK | \
                                                                                I2C_CTRL_STOP_MASK | \
                                                                                I2C_CTRL_START_MASK); \
                                                  I2C_GO_REG = 0x00u; \
                                                }
    
    /* Generates Start - used only by the Master */
    #define I2C_GENERATE_START     { I2C_CFG_REG |= I2C_CTRL_START_MASK; \
                                                  I2C_CFG_REG &= ~(I2C_CTRL_TRANSMIT_MASK | \
                                                                                I2C_CTRL_NACK_MASK | \
                                                                                I2C_CTRL_RESTART_MASK | \
                                                                                I2C_CTRL_STOP_MASK); \
                                                  I2C_GO_REG = 0x00u; \
                                                }
                                                
    /* Generates ReStart - used only by the Master */              
    #define I2C_GENERATE_RESTART   { I2C_CFG_REG |=  (I2C_CTRL_RESTART_MASK | \
                                                                                I2C_CTRL_NACK_MASK); \
                                                  I2C_CFG_REG &= ~(I2C_CTRL_TRANSMIT_MASK | \
                                                                                I2C_CTRL_STOP_MASK | \
                                                                                I2C_CTRL_START_MASK); \
                                                  I2C_GO_REG = 0x00u; \
                                                }
    
    /* Generates Stop - used only by the Master */
    #define I2C_GENERATE_STOP      { I2C_CFG_REG |=  (I2C_CTRL_NACK_MASK | \
                                                                                I2C_CTRL_STOP_MASK); \
                                                  I2C_CFG_REG &= ~(I2C_CTRL_TRANSMIT_MASK | \
                                                                                I2C_CTRL_RESTART_MASK | \
                                                                                I2C_CTRL_START_MASK); \
                                                  I2C_GO_REG = 0x00u; \
                                                }
    
    /* Release the bus - used to receive 1st byte of Master read transaction */
    #define I2C_READY_TO_READ      { I2C_CFG_REG &= ~(I2C_CTRL_TRANSMIT_MASK | \
                                                                                I2C_CTRL_NACK_MASK | \
                                                                                I2C_CTRL_RESTART_MASK | \
                                                                                I2C_CTRL_STOP_MASK | \
                                                                                I2C_CTRL_START_MASK); \
                                                  I2C_GO_REG = 0x00u; \
                                                }
#endif  /* End (I2C_IMPLEMENTATION == I2C_FF)*/


/* Conditions definitions common for FF and UDB */
#define I2C_CHECK_ADDR_ACK(csr)    ( (I2C_CSR_LRB_ACK | I2C_CSR_ADDRESS) == \
                                                  ((I2C_CSR_LRB | I2C_CSR_ADDRESS) & \
                                                   (csr)) \
                                                )
                                                
                                                
#define I2C_CHECK_ADDR_NAK(csr)    ( (I2C_CSR_LRB_NAK | I2C_CSR_ADDRESS) == \
                                                  ((I2C_CSR_LRB | I2C_CSR_ADDRESS) & \
                                                   (csr)) \
                                                )
                                                

#define I2C_CHECK_DATA_ACK(csr)    ( I2C_CSR_LRB_ACK  == \
                                                  ((csr) & I2C_CSR_LRB) \
                                                )

/* Conditional checks */
#define I2C_CHECK_BUS_FREE(mcsr)       ( 0u == ((mcsr) & I2C_MCSR_BUS_BUSY))
#define I2C_CHECK_MASTER_MODE(mcsr)    ( 0u != ((mcsr) & I2C_MCSR_MSTR_MODE))

#define I2C_WAIT_BYTE_COMPLETE(csr)    ( 0u == ((csr) & I2C_CSR_BYTE_COMPLETE))
#define I2C_CHECK_BYTE_COMPLETE(csr)   ( 0u != ((csr) & I2C_CSR_BYTE_COMPLETE))
#define I2C_CHECK_STOP_STS(csr)        ( 0u != ((csr) & I2C_CSR_STOP_STATUS))
#define I2C_CHECK_LOST_ARB(csr)        ( 0u != ((csr) & I2C_CSR_LOST_ARB))

#define I2C_CHECK_NO_STOP(mstrCntl)    ( 0u != ((mstrCntl) & I2C_MSTR_NO_STOP))


#if (I2C_IMPLEMENTATION == I2C_FF)
    #define I2C_CHECK_START_GEN(mcsr)  ((0u != (mcsr & I2C_MCSR_START_GEN)) && \
                                                     (0u == (mcsr & I2C_MCSR_MSTR_MODE)))
                                                     
    #define I2C_CLEAR_START_GEN        { I2C_MCSR_REG &= ~I2C_MCSR_START_GEN; }
#else
    #define I2C_CHECK_START_GEN(mcsr)  ((0u != (I2C_CFG_REG & I2C_MCSR_START_GEN)) && \
                                                     (0u == (mcsr & I2C_MCSR_MSTR_MODE)))
                                                     
    #define I2C_CLEAR_START_GEN        { I2C_CFG_REG &= ~I2C_MCSR_START_GEN; }
#endif

/* Create constansts to enable slave */
#if (0u != (I2C_MODE & I2C_MODE_SLAVE))
    #if (I2C_IMPLEMENTATION == I2C_FF)
        #define I2C_ENABLE_SLAVE  I2C_CFG_EN_SLAVE
    #else
        #define I2C_ENABLE_SLAVE  I2C_CTRL_ENABLE_SLAVE_MASK
    #endif  /* End (I2C_IMPLEMENTATION == I2C_FF)*/
#else
    #define  I2C_ENABLE_SLAVE    (0u)
#endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */

/* Create constansts to enable master */
#if (0u != (I2C_MODE & I2C_MODE_MASTER))
    #if (I2C_IMPLEMENTATION == I2C_FF)
        #define I2C_ENABLE_MASTER  I2C_CFG_EN_MSTR
    #else
        #define I2C_ENABLE_MASTER  I2C_CTRL_ENABLE_MASTER_MASK
    #endif  /* End (I2C_IMPLEMENTATION == I2C_FF) */
#else
    #define I2C_ENABLE_MASTER    (0u)  
#endif  /* End (0u != (I2C_MODE & I2C_MODE_MASTER)) */
 
#endif  /* End CY_I2C_I2C_H */

/* Default XCFG register value */
#if (I2C_ADDR_DECODE == I2C_HDWR_DECODE)
    #if (I2C_ENABLE_WAKEUP)
        #define  I2C_DEFAULT_XCFG  (I2C_XCFG_CLK_EN | \
                                                 I2C_XCFG_I2C_ON | \
                                                 I2C_XCFG_HDWR_ADDR_EN)
    #else
        #define  I2C_DEFAULT_XCFG  (I2C_XCFG_CLK_EN | \
                                                 I2C_XCFG_HDWR_ADDR_EN)
    #endif  /* End (I2C_ENABLE_WAKEUP) */
#else
    #define  I2C_DEFAULT_XCFG      I2C_XCFG_CLK_EN
#endif  /* End (I2C_ADDR_DECODE == I2C_HDWR_DECODE) */

/* Default CFG register value */
#if (I2C_IMPLEMENTATION == I2C_FF)
    #if (I2C_ENABLE_WAKEUP)
        #if (I2C_I2C_PAIR_SELECTED == I2C_I2C_PAIR1)
            #define I2C_DEFAULT_CFG    (I2C_CFG_SIO_SELECT | \
                                                     I2C_CFG_PSELECT | \
                                                     I2C_DEFAULT_CLK_RATE | \
                                                     I2C_ENABLE_MASTER | \
                                                     I2C_ENABLE_SLAVE)
        #else
            #define I2C_DEFAULT_CFG    (I2C_CFG_PSELECT | \
                                                     I2C_DEFAULT_CLK_RATE | \
                                                     I2C_ENABLE_MASTER | \
                                                     I2C_ENABLE_SLAVE)
        #endif  /*  End (I2C_I2C_PAIR_SELECTED == I2C_I2C_PAIR0)*/
    #else
        #define I2C_DEFAULT_CFG    (I2C_DEFAULT_CLK_RATE | \
                                                 I2C_ENABLE_MASTER | \
                                                 I2C_ENABLE_SLAVE)
    #endif  /* End (I2C_ENABLE_WAKEUP) */
#else
    #if (I2C_ADDR_DECODE == I2C_HDWR_DECODE)
        #define I2C_DEFAULT_CFG    (0u)
    #else
        #define I2C_DEFAULT_CFG    I2C_CTRL_ANY_ADDRESS_MASK
    #endif  /* End (I2C_ADDR_DECODE == I2C_HDWR_DECODE) */
#endif  /* End (I2C_IMPLEMENTATION == I2C_FF)*/

/* Define active state */
#if (I2C_IMPLEMENTATION == I2C_FF)
    #define I2C_I2C_ENABLE_REG     I2C_ACT_PWRMGR_REG
    #define I2C_IS_I2C_ENABLE(reg) (0u != ((reg) & I2C_ACT_PWR_EN))
#else
    #define I2C_I2C_ENABLE_REG     I2C_CFG_REG
    #define I2C_IS_I2C_ENABLE(reg) ((0u != ((reg) & I2C_ENABLE_MASTER)) || \
                                                 (0u != ((reg) & I2C_ENABLE_SLAVE)))
#endif  /* End (I2C_IMPLEMENTATION == I2C_FF)*/


/* [] END OF FILE */
