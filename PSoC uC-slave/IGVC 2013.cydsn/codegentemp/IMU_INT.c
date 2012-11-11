/*******************************************************************************
* File Name: IMU_INT.c
* Version 2.10
*
* Description:
*  This file provides all Interrupt Service functionality of the UART component
*
* Note:
*  Any unusual or non-standard behavior should be noted here. Other-
*  wise, this section should remain blank.
*
*******************************************************************************
* Copyright 2008-2011, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

#include "IMU.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (IMU_RX_ENABLED || IMU_HD_ENABLED) && \
     (IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH))

    extern volatile uint8 IMU_rxBuffer[];
    extern volatile uint8 IMU_rxBufferRead;
    extern volatile uint8 IMU_rxBufferWrite;
    extern volatile uint8 IMU_rxBufferLoopDetect;
    extern volatile uint8 IMU_rxBufferOverflow;
    #if (IMU_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 IMU_rxAddressMode;
        extern volatile uint8 IMU_rxAddressDetected;
    #endif /* End EnableHWAddress */    

    /*******************************************************************************
    * Function Name: IMU_RXISR
    ********************************************************************************
    *
    * Summary:
    *  Interrupt Service Routine for RX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  IMU_rxBuffer - RAM buffer pointer for save received data.
    *  IMU_rxBufferWrite - cyclic index for write to rxBuffer, 
    *     increments after each byte saved to buffer.
    *  IMU_rxBufferRead - cyclic index for read from rxBuffer, 
    *     checked to detect overflow condition.
    *  IMU_rxBufferOverflow - software overflow flag. Set to one
    *     when IMU_rxBufferWrite index overtakes 
    *     IMU_rxBufferRead index.
    *  IMU_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when IMU_rxBufferWrite is equal to 
    *    IMU_rxBufferRead
    *  IMU_rxAddressMode - this variable contains the Address mode, 
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  IMU_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(IMU_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0;

        /* User code required at start of ISR */
        /* `#START IMU_RXISR_START` */

        /* `#END` */

        readData = IMU_RXSTATUS_REG;

        if((readData & (IMU_RX_STS_BREAK | IMU_RX_STS_PAR_ERROR |
                        IMU_RX_STS_STOP_ERROR | IMU_RX_STS_OVERRUN)) != 0)
        {
            /* ERROR handling. */
            /* `#START IMU_RXISR_ERROR` */

            /* `#END` */
        }

        while(readData & IMU_RX_STS_FIFO_NOTEMPTY)
        {
            
            #if (IMU_RXHW_ADDRESS_ENABLED)
                if(IMU_rxAddressMode == IMU__B_UART__AM_SW_DETECT_TO_BUFFER) 
                {
                    if((readData & IMU_RX_STS_MRKSPC) != 0u )
                    {  
                        if ((readData & IMU_RX_STS_ADDR_MATCH) != 0)
                        {
                            IMU_rxAddressDetected = 1u;
                        }
                        else
                        {
                            IMU_rxAddressDetected = 0u;
                        }
                    }

                    readData = IMU_RXDATA_REG;
                    if(IMU_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        IMU_rxBuffer[IMU_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    IMU_rxBuffer[IMU_rxBufferWrite] = IMU_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                IMU_rxBuffer[IMU_rxBufferWrite] = IMU_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */
            
            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(IMU_rxBufferLoopDetect)
                {   /* Set Software Buffer status Overflow */
                    IMU_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                IMU_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(IMU_rxBufferWrite >= IMU_RXBUFFERSIZE)
                {
                    IMU_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(IMU_rxBufferWrite == IMU_rxBufferRead)
                {
                    IMU_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(IMU_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        IMU_RXSTATUS_MASK_REG  &= ~IMU_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(IMU_RX_VECT_NUM); 
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End IMU_FLOW_CONTROL != 0 */    
                }
            }

            /* Check again if there is data. */
            readData = IMU_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START IMU_RXISR_END` */

        /* `#END` */

        /* PSoC3 ES1, ES2 RTC ISR PATCH  */
        #if(CY_PSOC3_ES2 && (IMU_RXInternalInterrupt__ES2_PATCH))
            IMU_ISR_PATCH();
        #endif /* End CY_PSOC3_ES2*/
    }

#endif /* End IMU_RX_ENABLED && (IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH) */


#if(IMU_TX_ENABLED && (IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH))

    extern volatile uint8 IMU_txBuffer[];
    extern volatile uint8 IMU_txBufferRead;
    extern uint8 IMU_txBufferWrite;


    /*******************************************************************************
    * Function Name: IMU_TXISR
    ********************************************************************************
    *
    * Summary:
    * Interrupt Service Routine for the TX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  IMU_txBuffer - RAM buffer pointer for transmit data from.
    *  IMU_txBufferRead - cyclic index for read and transmit data 
    *     from txBuffer, increments after each transmited byte.
    *  IMU_rxBufferWrite - cyclic index for write to txBuffer, 
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(IMU_TXISR)
    {

        /* User code required at start of ISR */
        /* `#START IMU_TXISR_START` */

        /* `#END` */

        while((IMU_txBufferRead != IMU_txBufferWrite) && \
             !(IMU_TXSTATUS_REG & IMU_TX_STS_FIFO_FULL))
        {
            /* Check pointer. */
            if(IMU_txBufferRead >= IMU_TXBUFFERSIZE)
            {
                IMU_txBufferRead = 0u;
            }

            IMU_TXDATA_REG = IMU_txBuffer[IMU_txBufferRead];

            /* Set next pointer. */
            IMU_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START IMU_TXISR_END` */

        /* `#END` */
        
        /* PSoC3 ES1, ES2 RTC ISR PATCH  */
        #if(CY_PSOC3_ES2 && (IMU_TXInternalInterrupt__ES2_PATCH))
            IMU_ISR_PATCH();
        #endif /* End CY_PSOC3_ES2*/
    }

#endif /* End IMU_TX_ENABLED && (IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH) */


/* [] END OF FILE */
