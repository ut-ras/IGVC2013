/*******************************************************************************
* File Name: Gyro_INT.c
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

#include "Gyro.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (Gyro_RX_ENABLED || Gyro_HD_ENABLED) && \
     (Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH))

    extern volatile uint8 Gyro_rxBuffer[];
    extern volatile uint8 Gyro_rxBufferRead;
    extern volatile uint8 Gyro_rxBufferWrite;
    extern volatile uint8 Gyro_rxBufferLoopDetect;
    extern volatile uint8 Gyro_rxBufferOverflow;
    #if (Gyro_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 Gyro_rxAddressMode;
        extern volatile uint8 Gyro_rxAddressDetected;
    #endif /* End EnableHWAddress */    

    /*******************************************************************************
    * Function Name: Gyro_RXISR
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
    *  Gyro_rxBuffer - RAM buffer pointer for save received data.
    *  Gyro_rxBufferWrite - cyclic index for write to rxBuffer, 
    *     increments after each byte saved to buffer.
    *  Gyro_rxBufferRead - cyclic index for read from rxBuffer, 
    *     checked to detect overflow condition.
    *  Gyro_rxBufferOverflow - software overflow flag. Set to one
    *     when Gyro_rxBufferWrite index overtakes 
    *     Gyro_rxBufferRead index.
    *  Gyro_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when Gyro_rxBufferWrite is equal to 
    *    Gyro_rxBufferRead
    *  Gyro_rxAddressMode - this variable contains the Address mode, 
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  Gyro_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(Gyro_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0;

        /* User code required at start of ISR */
        /* `#START Gyro_RXISR_START` */

        /* `#END` */

        readData = Gyro_RXSTATUS_REG;

        if((readData & (Gyro_RX_STS_BREAK | Gyro_RX_STS_PAR_ERROR |
                        Gyro_RX_STS_STOP_ERROR | Gyro_RX_STS_OVERRUN)) != 0)
        {
            /* ERROR handling. */
            /* `#START Gyro_RXISR_ERROR` */

            /* `#END` */
        }

        while(readData & Gyro_RX_STS_FIFO_NOTEMPTY)
        {
            
            #if (Gyro_RXHW_ADDRESS_ENABLED)
                if(Gyro_rxAddressMode == Gyro__B_UART__AM_SW_DETECT_TO_BUFFER) 
                {
                    if((readData & Gyro_RX_STS_MRKSPC) != 0u )
                    {  
                        if ((readData & Gyro_RX_STS_ADDR_MATCH) != 0)
                        {
                            Gyro_rxAddressDetected = 1u;
                        }
                        else
                        {
                            Gyro_rxAddressDetected = 0u;
                        }
                    }

                    readData = Gyro_RXDATA_REG;
                    if(Gyro_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        Gyro_rxBuffer[Gyro_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    Gyro_rxBuffer[Gyro_rxBufferWrite] = Gyro_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                Gyro_rxBuffer[Gyro_rxBufferWrite] = Gyro_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */
            
            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(Gyro_rxBufferLoopDetect)
                {   /* Set Software Buffer status Overflow */
                    Gyro_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                Gyro_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(Gyro_rxBufferWrite >= Gyro_RXBUFFERSIZE)
                {
                    Gyro_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(Gyro_rxBufferWrite == Gyro_rxBufferRead)
                {
                    Gyro_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(Gyro_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        Gyro_RXSTATUS_MASK_REG  &= ~Gyro_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(Gyro_RX_VECT_NUM); 
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End Gyro_FLOW_CONTROL != 0 */    
                }
            }

            /* Check again if there is data. */
            readData = Gyro_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Gyro_RXISR_END` */

        /* `#END` */

        /* PSoC3 ES1, ES2 RTC ISR PATCH  */
        #if(CY_PSOC3_ES2 && (Gyro_RXInternalInterrupt__ES2_PATCH))
            Gyro_ISR_PATCH();
        #endif /* End CY_PSOC3_ES2*/
    }

#endif /* End Gyro_RX_ENABLED && (Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH) */


#if(Gyro_TX_ENABLED && (Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH))

    extern volatile uint8 Gyro_txBuffer[];
    extern volatile uint8 Gyro_txBufferRead;
    extern uint8 Gyro_txBufferWrite;


    /*******************************************************************************
    * Function Name: Gyro_TXISR
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
    *  Gyro_txBuffer - RAM buffer pointer for transmit data from.
    *  Gyro_txBufferRead - cyclic index for read and transmit data 
    *     from txBuffer, increments after each transmited byte.
    *  Gyro_rxBufferWrite - cyclic index for write to txBuffer, 
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(Gyro_TXISR)
    {

        /* User code required at start of ISR */
        /* `#START Gyro_TXISR_START` */

        /* `#END` */

        while((Gyro_txBufferRead != Gyro_txBufferWrite) && \
             !(Gyro_TXSTATUS_REG & Gyro_TX_STS_FIFO_FULL))
        {
            /* Check pointer. */
            if(Gyro_txBufferRead >= Gyro_TXBUFFERSIZE)
            {
                Gyro_txBufferRead = 0u;
            }

            Gyro_TXDATA_REG = Gyro_txBuffer[Gyro_txBufferRead];

            /* Set next pointer. */
            Gyro_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Gyro_TXISR_END` */

        /* `#END` */
        
        /* PSoC3 ES1, ES2 RTC ISR PATCH  */
        #if(CY_PSOC3_ES2 && (Gyro_TXInternalInterrupt__ES2_PATCH))
            Gyro_ISR_PATCH();
        #endif /* End CY_PSOC3_ES2*/
    }

#endif /* End Gyro_TX_ENABLED && (Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH) */


/* [] END OF FILE */
