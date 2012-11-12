/*******************************************************************************
* File Name: IMU.c
* Version 2.10
*
* Description:
*  This file provides all API functionality of the UART component
*
* Note:
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
* Global data allocation
***************************************/

#if( IMU_TX_ENABLED && (IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH))
    volatile uint8 IMU_txBuffer[IMU_TXBUFFERSIZE];
    volatile uint8 IMU_txBufferRead = 0u;
    uint8 IMU_txBufferWrite = 0u;
#endif /* End IMU_TX_ENABLED */
#if( ( IMU_RX_ENABLED || IMU_HD_ENABLED ) && \
     (IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH) )
    volatile uint8 IMU_rxBuffer[IMU_RXBUFFERSIZE];
    volatile uint8 IMU_rxBufferRead = 0u;
    volatile uint8 IMU_rxBufferWrite = 0u;
    volatile uint8 IMU_rxBufferLoopDetect = 0u;
    volatile uint8 IMU_rxBufferOverflow = 0u;
    #if (IMU_RXHW_ADDRESS_ENABLED)
        volatile uint8 IMU_rxAddressMode = IMU_RXADDRESSMODE;
        volatile uint8 IMU_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */    
#endif /* End IMU_RX_ENABLED */


/***************************************
* Local data allocation
***************************************/

uint8 IMU_initVar = 0u;


/*******************************************************************************
* Function Name: IMU_Start
********************************************************************************
*
* Summary:
*  Initialize and Enable the UART component.
*  Enable the clock input to enable operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The IMU_intiVar variable is used to indicate initial 
*  configuration of this component. The variable is initialized to zero (0u) 
*  and set to one (1u) the first time UART_Start() is called. This allows for 
*  component initialization without re-initialization in all subsequent calls 
*  to the IMU_Start() routine. 
*
* Reentrant:
*  No.
*
*******************************************************************************/
void IMU_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(IMU_initVar == 0u)
    {
        IMU_Init();
        IMU_initVar = 1u;
    }
    IMU_Enable();
}


/*******************************************************************************
* Function Name: IMU_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  IMU_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void IMU_Init(void) 
{
    #if(IMU_RX_ENABLED || IMU_HD_ENABLED)

        #if(IMU_RX_INTERRUPT_ENABLED && (IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            CyIntSetVector(IMU_RX_VECT_NUM,   IMU_RXISR);
            CyIntSetPriority(IMU_RX_VECT_NUM, IMU_RX_PRIOR_NUM);
        #endif /* End IMU_RX_INTERRUPT_ENABLED */

        #if (IMU_RXHW_ADDRESS_ENABLED)
            IMU_SetRxAddressMode(IMU_RXAddressMode);
            IMU_SetRxAddress1(IMU_RXHWADDRESS1);
            IMU_SetRxAddress2(IMU_RXHWADDRESS2);
        #endif /* End IMU_RXHW_ADDRESS_ENABLED */

        /* Configure the Initial RX interrupt mask */
        IMU_RXSTATUS_MASK_REG  = IMU_INIT_RX_INTERRUPTS_MASK;
    #endif /* End IMU_RX_ENABLED || IMU_HD_ENABLED*/

    #if(IMU_TX_ENABLED)
        #if(IMU_TX_INTERRUPT_ENABLED && (IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            CyIntSetVector(IMU_TX_VECT_NUM,   IMU_TXISR);
            CyIntSetPriority(IMU_TX_VECT_NUM, IMU_TX_PRIOR_NUM);
        #endif /* End IMU_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(IMU_TXCLKGEN_DP)
            IMU_TXBITCLKGEN_CTR_REG = IMU_BIT_CENTER;
            IMU_TXBITCLKTX_COMPLETE_REG = (IMU_NUMBER_OF_DATA_BITS + \
                                                    IMU_NUMBER_OF_START_BIT) * \
                                                    IMU_OVER_SAMPLE_COUNT;
        #else
            IMU_TXBITCTR_COUNTER_REG = (IMU_NUMBER_OF_DATA_BITS + \
                                                    IMU_NUMBER_OF_START_BIT) * \
                                                    IMU_OVER_SAMPLE_8;
        #endif /* End IMU_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(IMU_TX_INTERRUPT_ENABLED && (IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH))
            IMU_TXSTATUS_MASK_REG = IMU_TX_STS_FIFO_EMPTY;
        #else
            IMU_TXSTATUS_MASK_REG = IMU_INIT_TX_INTERRUPTS_MASK;
        #endif /*End IMU_TX_INTERRUPT_ENABLED*/
        
    #endif /* End IMU_TX_ENABLED */

    #if(IMU_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        IMU_WriteControlRegister( \
            (IMU_ReadControlRegister() & ~IMU_CTRL_PARITY_TYPE_MASK) | \
            (IMU_PARITY_TYPE << IMU_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End IMU_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: IMU_Enable
********************************************************************************
*
* Summary:
*  Enables the UART block operation
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  IMU_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void IMU_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();
    
    #if(IMU_RX_ENABLED || IMU_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        IMU_RXBITCTR_CONTROL_REG |= IMU_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        IMU_RXSTATUS_ACTL_REG  |= IMU_INT_ENABLE;
        #if(IMU_RX_INTERRUPT_ENABLED && (IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH))
            CyIntEnable(IMU_RX_VECT_NUM);
            #if (IMU_RXHW_ADDRESS_ENABLED)
                IMU_rxAddressDetected = 0u;
            #endif /* End IMU_RXHW_ADDRESS_ENABLED */
        #endif /* End IMU_RX_INTERRUPT_ENABLED */
    #endif /* End IMU_RX_ENABLED || IMU_HD_ENABLED*/

    #if(IMU_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!IMU_TXCLKGEN_DP)
            IMU_TXBITCTR_CONTROL_REG |= IMU_CNTR_ENABLE;
        #endif /* End IMU_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        IMU_TXSTATUS_ACTL_REG |= IMU_INT_ENABLE;
        #if(IMU_TX_INTERRUPT_ENABLED && (IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH))
            CyIntEnable(IMU_TX_VECT_NUM);
        #endif /* End IMU_TX_INTERRUPT_ENABLED*/
     #endif /* End IMU_TX_ENABLED */

    #if(IMU_INTERNAL_CLOCK_USED)
        /* Set the bit to enable the clock. */
        IMU_INTCLOCK_CLKEN_REG |= IMU_INTCLOCK_CLKEN_MASK;
    #endif /* End IMU_INTERNAL_CLOCK_USED */
    
    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: IMU_Stop
********************************************************************************
*
* Summary:
*  Disable the UART component
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void IMU_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /*Write Bit Counter Disable */
    #if(IMU_RX_ENABLED || IMU_HD_ENABLED)
        IMU_RXBITCTR_CONTROL_REG &= ~IMU_CNTR_ENABLE;
    #endif /* End IMU_RX_ENABLED */

    #if(IMU_TX_ENABLED)
        #if(!IMU_TXCLKGEN_DP)
            IMU_TXBITCTR_CONTROL_REG &= ~IMU_CNTR_ENABLE;
        #endif /* End IMU_TXCLKGEN_DP */
    #endif /* IMU_TX_ENABLED */

    #if(IMU_INTERNAL_CLOCK_USED)
        /* Clear the bit to enable the clock. */
        IMU_INTCLOCK_CLKEN_REG &= ~IMU_INTCLOCK_CLKEN_MASK;
    #endif /* End IMU_INTERNAL_CLOCK_USED */
    
    /*Disable internal interrupt component*/
    #if(IMU_RX_ENABLED || IMU_HD_ENABLED)
        IMU_RXSTATUS_ACTL_REG  &= ~IMU_INT_ENABLE;
        #if(IMU_RX_INTERRUPT_ENABLED && (IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH))
            IMU_DisableRxInt();
        #endif /* End IMU_RX_INTERRUPT_ENABLED */
    #endif /* End IMU_RX_ENABLED */
    
    #if(IMU_TX_ENABLED)
        IMU_TXSTATUS_ACTL_REG &= ~IMU_INT_ENABLE;
        #if(IMU_TX_INTERRUPT_ENABLED && (IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH))
            IMU_DisableTxInt();
        #endif /* End IMU_TX_INTERRUPT_ENABLED */
    #endif /* End IMU_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: IMU_ReadControlRegister
********************************************************************************
*
* Summary:
*  Read the current state of the control register
*
* Parameters:
*  None.
*
* Return:
*  Current state of the control register.
*
*******************************************************************************/
uint8 IMU_ReadControlRegister(void) 
{
    #if( IMU_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(IMU_CONTROL_REG);
    #endif /* End IMU_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: IMU_WriteControlRegister
********************************************************************************
*
* Summary:
*  Writes an 8-bit value into the control register
*
* Parameters:
*  control:  control register value
*
* Return:
*  None.
*
*******************************************************************************/
void  IMU_WriteControlRegister(uint8 control) 
{
    #if( IMU_CONTROL_REG_REMOVED )
        control = control;      /* Reassigning to release compiler warning */ 
    #else
       IMU_CONTROL_REG = control;
    #endif /* End IMU_CONTROL_REG_REMOVED */
}


#if(IMU_RX_ENABLED || IMU_HD_ENABLED)

    #if(IMU_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: IMU_EnableRxInt
        ********************************************************************************
        *
        * Summary:
        *  Enable RX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Enable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void IMU_EnableRxInt(void) 
        {
            CyIntEnable(IMU_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: IMU_DisableRxInt
        ********************************************************************************
        *
        * Summary:
        *  Disable RX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Disable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void IMU_DisableRxInt(void) 
        {
            CyIntDisable(IMU_RX_VECT_NUM);
        }

    #endif /* IMU_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: IMU_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configure which status bits trigger an interrupt event
    *
    * Parameters:
    *  IntSrc:  An or'd combination of the desired status bit masks (defined in
    *           the header file)
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void IMU_SetRxInterruptMode(uint8 intSrc) 
    {
        IMU_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: IMU_ReadRxData
    ********************************************************************************
    *
    * Summary:
    *  Returns data in RX Data register without checking status register to 
    *  determine if data is valid
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Received data from RX register
    *
    * Global Variables:
    *  IMU_rxBuffer - RAM buffer pointer for save received data.
    *  IMU_rxBufferWrite - cyclic index for write to rxBuffer, 
    *     checked to identify new data. 
    *  IMU_rxBufferRead - cyclic index for read from rxBuffer, 
    *     incremented after each byte has been read from buffer.
    *  IMU_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR. 
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 IMU_ReadRxData(void) 
    {
        uint8 rxData;

        #if(IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(IMU_RX_INTERRUPT_ENABLED)
                IMU_DisableRxInt();
            #endif /* End IMU_RX_INTERRUPT_ENABLED */

            if( (IMU_rxBufferRead != IMU_rxBufferWrite) ||
                (IMU_rxBufferLoopDetect > 0u) )
            {

                rxData = IMU_rxBuffer[IMU_rxBufferRead];

                IMU_rxBufferRead++;

                if(IMU_rxBufferRead >= IMU_RXBUFFERSIZE)
                {
                    IMU_rxBufferRead = 0u;
                }

                if(IMU_rxBufferLoopDetect > 0u )
                {
                    IMU_rxBufferLoopDetect = 0u;
                    #if( (IMU_RX_INTERRUPT_ENABLED) && (IMU_FLOW_CONTROL != 0u) && \
                         (IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( IMU_HD_ENABLED )
                            if((IMU_CONTROL_REG & IMU_CTRL_HD_SEND) == 0)
                            {   /* In Half duplex mode return RX mask only in RX configuration set, otherwise 
                                *  mask will be returned in LoadRxConfig() API. 
                                */
                                IMU_RXSTATUS_MASK_REG  |= IMU_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            IMU_RXSTATUS_MASK_REG  |= IMU_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end IMU_HD_ENABLED */
                    #endif /* IMU_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = IMU_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(IMU_RX_INTERRUPT_ENABLED)
                IMU_EnableRxInt();
            #endif /* End IMU_RX_INTERRUPT_ENABLED */

        #else /* IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = IMU_RXDATA_REG;

        #endif /* IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: IMU_ReadRxStatus
    ********************************************************************************
    *
    * Summary:
    *  Read the current state of the status register
    *  And detect software buffer overflow.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Current state of the status register.
    *
    * Global Variables:
    *  IMU_rxBufferOverflow - used to indicate overload condition. 
    *   It set to one in RX interrupt when there isn?t free space in 
    *   IMU_rxBufferRead to write new data. This condition returned 
    *   and cleared to zero by this API as an 
    *   IMU_RX_STS_SOFT_BUFF_OVER bit along with RX Status register 
    *   bits.
    *
    *******************************************************************************/
    uint8 IMU_ReadRxStatus(void) 
    {
        uint8 status;

        status = IMU_RXSTATUS_REG;
        status &= IMU_RX_HW_MASK;

        #if(IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH)
            if( IMU_rxBufferOverflow )
            {
                status |= IMU_RX_STS_SOFT_BUFF_OVER;
                IMU_rxBufferOverflow = 0u;
            }
        #endif /* IMU_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: IMU_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Reads UART RX buffer immediately, if data is not available or an error 
    *  condition exists, zero is returned; otherwise, character is read and 
    *  returned.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Character read from UART RX buffer. ASCII characters from 1 to 255 are valid.
    *  A returned zero signifies an error condition or no data available.
    *
    * Global Variables:
    *  IMU_rxBuffer - RAM buffer pointer for save received data.
    *  IMU_rxBufferWrite - cyclic index for write to rxBuffer, 
    *     checked to identify new data. 
    *  IMU_rxBufferRead - cyclic index for read from rxBuffer, 
    *     incremented after each byte has been read from buffer.
    *  IMU_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR. 
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 IMU_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(IMU_RX_INTERRUPT_ENABLED)
                IMU_DisableRxInt();
            #endif /* IMU_RX_INTERRUPT_ENABLED */

            if( (IMU_rxBufferRead != IMU_rxBufferWrite) ||
                (IMU_rxBufferLoopDetect > 0u) )
            {
                rxData = IMU_rxBuffer[IMU_rxBufferRead];

                IMU_rxBufferRead++;

                if(IMU_rxBufferRead >= IMU_RXBUFFERSIZE)
                {
                    IMU_rxBufferRead = 0u;
                }

                if(IMU_rxBufferLoopDetect > 0u ) 
                {
                    IMU_rxBufferLoopDetect = 0u;
                    #if( (IMU_RX_INTERRUPT_ENABLED) && (IMU_FLOW_CONTROL != 0u) && \
                         (IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( IMU_HD_ENABLED )
                            if((IMU_CONTROL_REG & IMU_CTRL_HD_SEND) == 0)
                            {   /* In Half duplex mode return RX mask only in RX configuration set, otherwise 
                                *  mask will be returned in LoadRxConfig() API. 
                                */
                                IMU_RXSTATUS_MASK_REG  |= IMU_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            IMU_RXSTATUS_MASK_REG  |= IMU_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end IMU_HD_ENABLED */
                    #endif /* IMU_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus =IMU_RXSTATUS_REG;
                if(rxStatus & IMU_RX_STS_FIFO_NOTEMPTY)
                {   /* Read received data from FIFO*/
                    rxData = IMU_RXDATA_REG;
                    /*Check status on error*/
                    if(rxStatus & (IMU_RX_STS_BREAK | IMU_RX_STS_PAR_ERROR |
                                   IMU_RX_STS_STOP_ERROR | IMU_RX_STS_OVERRUN))
                    {
                        rxData = 0u;
                    }    
                }
            }

            /* Enable Rx interrupt. */
            #if(IMU_RX_INTERRUPT_ENABLED)
                IMU_EnableRxInt();
            #endif /* IMU_RX_INTERRUPT_ENABLED */

        #else /* IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH */

            rxStatus =IMU_RXSTATUS_REG;
            if(rxStatus & IMU_RX_STS_FIFO_NOTEMPTY)
            {   /* Read received data from FIFO*/
                rxData = IMU_RXDATA_REG;
                /*Check status on error*/
                if(rxStatus & (IMU_RX_STS_BREAK | IMU_RX_STS_PAR_ERROR |
                               IMU_RX_STS_STOP_ERROR | IMU_RX_STS_OVERRUN))
                {
                    rxData = 0u;
                }
            }
        #endif /* IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: IMU_GetByte
    ********************************************************************************
    *
    * Summary:
    *  Grab the next available byte of data from the recieve FIFO
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  MSB contains Status Register and LSB contains UART RX data
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint16 IMU_GetByte(void) 
    {
        return ( ((uint16)IMU_ReadRxStatus() << 8u) | IMU_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: IMU_GetRxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Determine the amount of bytes left in the RX buffer and return the count in
    *  bytes
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  uint8: Integer count of the number of bytes left 
    *  in the RX buffer
    *
    * Global Variables:
    *  IMU_rxBufferWrite - used to calculate left bytes. 
    *  IMU_rxBufferRead - used to calculate left bytes.
    *  IMU_rxBufferLoopDetect - checked to decide left bytes amount. 
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 IMU_GetRxBufferSize(void) 
                                                            
    {
        uint8 size;

        #if(IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(IMU_RX_INTERRUPT_ENABLED)
                IMU_DisableRxInt();
            #endif /* IMU_RX_INTERRUPT_ENABLED */

            if(IMU_rxBufferRead == IMU_rxBufferWrite)
            {
                if(IMU_rxBufferLoopDetect > 0u)
                {
                    size = IMU_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(IMU_rxBufferRead < IMU_rxBufferWrite)
            {
                size = (IMU_rxBufferWrite - IMU_rxBufferRead);
            }
            else
            {
                size = (IMU_RXBUFFERSIZE - IMU_rxBufferRead) + IMU_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(IMU_RX_INTERRUPT_ENABLED)
                IMU_EnableRxInt();
            #endif /* End IMU_RX_INTERRUPT_ENABLED */

        #else /* IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = (IMU_RXSTATUS_REG & IMU_RX_STS_FIFO_NOTEMPTY) ? 1u : 0u;

        #endif /* End IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: IMU_ClearRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the RX RAM buffer by setting the read and write pointers both to zero.
    *  Clears hardware RX FIFO.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  IMU_rxBufferWrite - cleared to zero. 
    *  IMU_rxBufferRead - cleared to zero.
    *  IMU_rxBufferLoopDetect - cleared to zero. 
    *  IMU_rxBufferOverflow - cleared to zero. 
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to 
    *  read and writing will resume at address 0 overwriting any data that may 
    *  have remained in the RAM.
    *
    * Side Effects:
    *  Any received data not read from the RAM or FIFO buffer will be lost.
    *******************************************************************************/
    void IMU_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;
        
        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();        
        IMU_RXDATA_AUX_CTL_REG |=  IMU_RX_FIFO_CLR;
        IMU_RXDATA_AUX_CTL_REG &= ~IMU_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);
        
        #if(IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(IMU_RX_INTERRUPT_ENABLED)
                IMU_DisableRxInt();
            #endif /* End IMU_RX_INTERRUPT_ENABLED */

            IMU_rxBufferRead = 0u;
            IMU_rxBufferWrite = 0u;
            IMU_rxBufferLoopDetect = 0u;
            IMU_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(IMU_RX_INTERRUPT_ENABLED)
                IMU_EnableRxInt();
            #endif /* End IMU_RX_INTERRUPT_ENABLED */
        #endif /* End IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH */
        
    }


    /*******************************************************************************
    * Function Name: IMU_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  IMU__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address 
    *                                               detection
    *  IMU__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer 
    *                                               address detection
    *  IMU__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address 
    *                                               detection
    *  IMU__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer 
    *                                               address detection
    *  IMU__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  IMU_rxAddressMode - the parameter stored in this variable for 
    *   the farther usage in RX ISR.
    *  IMU_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void IMU_SetRxAddressMode(uint8 addressMode)  
                                                        
    {
        #if(IMU_RXHW_ADDRESS_ENABLED)
            #if(IMU_CONTROL_REG_REMOVED)
                addressMode = addressMode;
            #else /* IMU_CONTROL_REG_REMOVED */
                uint8 tmpCtrl = 0u;
                tmpCtrl = IMU_CONTROL_REG & ~IMU_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= ((addressMode << IMU_CTRL_RXADDR_MODE0_SHIFT) & 
                           IMU_CTRL_RXADDR_MODE_MASK);
                IMU_CONTROL_REG = tmpCtrl;
                #if(IMU_RX_INTERRUPT_ENABLED && \
                   (IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH) )
                    IMU_rxAddressMode = addressMode;
                    IMU_rxAddressDetected = 0u;
                #endif /* End IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH*/   
            #endif /* End IMU_CONTROL_REG_REMOVED */
        #else /* IMU_RXHW_ADDRESS_ENABLED */
            addressMode = addressMode;
        #endif /* End IMU_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: IMU_SetRxAddress1
    ********************************************************************************
    *
    * Summary:
    *  Set the first hardware address compare value
    *
    * Parameters:
    *  address
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void IMU_SetRxAddress1(uint8 address) 

    {
        IMU_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: IMU_SetRxAddress2
    ********************************************************************************
    *
    * Summary:
    *  Set the second hardware address compare value
    *
    * Parameters:
    *  address
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void IMU_SetRxAddress2(uint8 address) 
    {
        IMU_RXADDRESS2_REG = address;
    }
        
#endif  /* IMU_RX_ENABLED || IMU_HD_ENABLED*/


#if( (IMU_TX_ENABLED) || (IMU_HD_ENABLED) )

    #if(IMU_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: IMU_EnableTxInt
        ********************************************************************************
        *
        * Summary:
        *  Enable TX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Enable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void IMU_EnableTxInt(void) 
        {
            CyIntEnable(IMU_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: IMU_DisableTxInt
        ********************************************************************************
        *
        * Summary:
        *  Disable TX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Disable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void IMU_DisableTxInt(void) 
        {
            CyIntDisable(IMU_TX_VECT_NUM);
        }

    #endif /* IMU_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: IMU_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configure which status bits trigger an interrupt event
    *
    * Parameters:
    *  intSrc: An or'd combination of the desired status bit masks (defined in
    *          the header file)
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void IMU_SetTxInterruptMode(uint8 intSrc) 
    {
        IMU_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: IMU_WriteTxData
    ********************************************************************************
    *
    * Summary:
    *  Write a byte of data to the Transmit FIFO or TX buffer to be sent when the 
    *  bus is available. WriteTxData sends a byte without checking for buffer room 
    *  or status. It is up to the user to separately check status.    
    *
    * Parameters:
    *  TXDataByte: byte of data to place in the transmit FIFO
    *
    * Return:
    * void
    *
    * Global Variables:
    *  IMU_txBuffer - RAM buffer pointer for save data for transmission
    *  IMU_txBufferWrite - cyclic index for write to txBuffer, 
    *    incremented after each byte saved to buffer.
    *  IMU_txBufferRead - cyclic index for read from txBuffer, 
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  IMU_initVar - checked to identify that the component has been  
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void IMU_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(IMU_initVar != 0u)
        {
            #if(IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(IMU_TX_INTERRUPT_ENABLED)
                    IMU_DisableTxInt();
                #endif /* End IMU_TX_INTERRUPT_ENABLED */

                if( (IMU_txBufferRead == IMU_txBufferWrite) &&
                   !(IMU_TXSTATUS_REG & IMU_TX_STS_FIFO_FULL) )
                {
                    /* Add directly to the FIFO. */
                    IMU_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(IMU_txBufferWrite >= IMU_TXBUFFERSIZE)
                    {
                        IMU_txBufferWrite = 0;
                    }

                    IMU_txBuffer[IMU_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    IMU_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(IMU_TX_INTERRUPT_ENABLED)
                    IMU_EnableTxInt();
                #endif /* End IMU_TX_INTERRUPT_ENABLED */

            #else /* IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                IMU_TXDATA_REG = txDataByte;

            #endif /* End IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: IMU_ReadTxStatus
    ********************************************************************************
    *
    * Summary:
    *  Read the status register for the component
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Contents of the status register
    *
    * Theory:
    *  This function reads the status register which is clear on read. It is up to 
    *  the user to handle all bits in this return value accordingly, even if the bit 
    *  was not enabled as an interrupt source the event happened and must be handled
    *  accordingly.    
    *
    *******************************************************************************/
    uint8 IMU_ReadTxStatus(void) 
    {
        return(IMU_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: IMU_PutChar
    ********************************************************************************
    *
    * Summary:
    *  Wait to send byte until TX register or buffer has room.
    *
    * Parameters:
    *  txDataByte: The 8-bit data value to send across the UART.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  IMU_txBuffer - RAM buffer pointer for save data for transmission
    *  IMU_txBufferWrite - cyclic index for write to txBuffer, 
    *     checked to identify free space in txBuffer and incremented after each byte 
    *     saved to buffer.
    *  IMU_txBufferRead - cyclic index for read from txBuffer, 
    *     checked to identify free space in txBuffer.
    *  IMU_initVar - checked to identify that the component has been  
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void IMU_PutChar(uint8 txDataByte) 
    {
            #if(IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH)

                /* Block if buffer is full, so we dont overwrite. */
                while( IMU_txBufferWrite == (IMU_txBufferRead - 1u) ||
                    (uint8)(IMU_txBufferWrite - IMU_txBufferRead) ==
                    (uint8)(IMU_TXBUFFERSIZE - 1u) )
                {
                    /* Software buffer is full. */
                }
                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(IMU_TX_INTERRUPT_ENABLED)
                    IMU_DisableTxInt();
                #endif /* End IMU_TX_INTERRUPT_ENABLED */

                if( (IMU_txBufferRead == IMU_txBufferWrite) &&
                   !(IMU_TXSTATUS_REG & IMU_TX_STS_FIFO_FULL) )
                {
                    /* Add directly to the FIFO. */
                    IMU_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(IMU_txBufferWrite >= IMU_TXBUFFERSIZE)
                    {
                        IMU_txBufferWrite = 0;
                    }

                    IMU_txBuffer[IMU_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    IMU_txBufferWrite++;

                }

                /* Enable Rx interrupt. */
                #if(IMU_TX_INTERRUPT_ENABLED)
                    IMU_EnableTxInt();
                #endif /* End IMU_TX_INTERRUPT_ENABLED */

            #else /* IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH */

                /* Block if there isnt room. */
                while(IMU_TXSTATUS_REG & IMU_TX_STS_FIFO_FULL);

                /* Add directly to the FIFO. */
                IMU_TXDATA_REG = txDataByte;

            #endif /* End IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: IMU_PutString
    ********************************************************************************
    *
    * Summary:
    *  Write a Sequence of bytes on the Transmit line. Data comes from RAM or ROM.
    *
    * Parameters:
    *  string: char pointer to character string of Data to Send.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  IMU_initVar - checked to identify that the component has been  
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  This function will block if there is not enough memory to place the whole 
    *  string, it will block until the entire string has been written to the 
    *  transmit buffer.
    *
    *******************************************************************************/
    void IMU_PutString(char* string) 
    {
        /* If not Initialized then skip this function*/
        if(IMU_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(*string != 0u)
            {
                IMU_PutChar(*string++);
            }
        }
    }


    /*******************************************************************************
    * Function Name: IMU_PutArray
    ********************************************************************************
    *
    * Summary:
    *  Write a Sequence of bytes on the Transmit line. Data comes from RAM or ROM.
    *
    * Parameters:
    *  string: Address of the memory array residing in RAM or ROM.
    *  byteCount: Number of Bytes to be transmitted.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  IMU_initVar - checked to identify that the component has been  
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void IMU_PutArray(uint8* string, uint8 byteCount) 
                                                                    
    {
        /* If not Initialized then skip this function*/
        if(IMU_initVar != 0u)
        {
            while(byteCount > 0u)
            {
                IMU_PutChar(*string++);
                byteCount--;
            }
        }
    }


    /*******************************************************************************
    * Function Name: IMU_PutCRLF
    ********************************************************************************
    *
    * Summary:
    *  Write a character and then carriage return and line feed.
    *
    * Parameters:
    *  txDataByte: uint8 Character to send.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  IMU_initVar - checked to identify that the component has been  
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void IMU_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(IMU_initVar != 0u)
        {
            IMU_PutChar(txDataByte);
            IMU_PutChar(0x0Du);
            IMU_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: IMU_GetTxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Determine the amount of space left in the TX buffer and return the count in
    *  bytes
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Integer count of the number of bytes left in the TX buffer
    *
    * Global Variables:
    *  IMU_txBufferWrite - used to calculate left space. 
    *  IMU_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 IMU_GetTxBufferSize(void) 
                                                            
    {
        uint8 size;

        #if(IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(IMU_TX_INTERRUPT_ENABLED)
                IMU_DisableTxInt();
            #endif /* End IMU_TX_INTERRUPT_ENABLED */

            if(IMU_txBufferRead == IMU_txBufferWrite)
            {
                size = 0u;
            }
            else if(IMU_txBufferRead < IMU_txBufferWrite)
            {
                size = (IMU_txBufferWrite - IMU_txBufferRead);
            }
            else
            {
                size = (IMU_TXBUFFERSIZE - IMU_txBufferRead) + IMU_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(IMU_TX_INTERRUPT_ENABLED)
                IMU_EnableTxInt();
            #endif /* End IMU_TX_INTERRUPT_ENABLED */

        #else /* IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH */

            size = IMU_TXSTATUS_REG;

            /* Is the fifo is full. */
            if(size & IMU_TX_STS_FIFO_FULL)
            {
                size = IMU_FIFO_LENGTH;
            }
            else if(size & IMU_TX_STS_FIFO_EMPTY)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: IMU_ClearTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the TX RAM buffer by setting the read and write pointers both to zero.
    *  Clears the hardware TX FIFO.  Any data present in the FIFO will not be sent.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  IMU_txBufferWrite - cleared to zero. 
    *  IMU_txBufferRead - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to 
    *  read and writing will resume at address 0 overwriting any data that may have
    *  remained in the RAM.
    *
    * Side Effects:
    *  Any received data not read from the RAM buffer will be lost when overwritten.
    *
    *******************************************************************************/
    void IMU_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;
        
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();        
        /* clear the HW FIFO */
        IMU_TXDATA_AUX_CTL_REG |=  IMU_TX_FIFO_CLR;
        IMU_TXDATA_AUX_CTL_REG &= ~IMU_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(IMU_TX_INTERRUPT_ENABLED)
                IMU_DisableTxInt();
            #endif /* End IMU_TX_INTERRUPT_ENABLED */

            IMU_txBufferRead = 0u;
            IMU_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(IMU_TX_INTERRUPT_ENABLED)
                IMU_EnableTxInt();
            #endif /* End IMU_TX_INTERRUPT_ENABLED */

        #endif /* End IMU_TXBUFFERSIZE > IMU_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: IMU_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Write a Break command to the UART
    *
    * Parameters:
    *  uint8 retMode:  Wait mode,
    *   0 - Initialize registers for Break, sends the Break signal and return 
    *       imediately.
    *   1 - Wait until Break sending is complete, reinitialize registers to normal
    *       transmission mode then return.
    *   2 - Reinitialize registers to normal transmission mode then return.
    *   3 - both steps: 0 and 1
    *       init registers for Break, send Break signal
    *       wait until Break sending is complete, reinit registers to normal
    *       transmission mode then return.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  IMU_initVar - checked to identify that the component has been  
    *     initialized.
    *  tx_period - static variable, used for keeping TX period configuration.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  SendBreak function initializes registers to send 13-bit break signal. It is
    *  important to return the registers configuration to normal for continue 8-bit
    *  operation.
    *  Trere are 3 variants for this API usage:
    *  1) SendBreak(3) - function will send the Break signal and take care on the
    *     configuration returning. Funcition will block CPU untill transmition 
    *     complete.
    *  2) User may want to use bloking time if UART configured to the low speed 
    *     operation
    *     Emample for this case:
    *     SendBreak(0);     - init Break signal transmition
    *         Add your code here to use CPU time
    *     SendBreak(1);     - complete Break operation
    *  3) Same to 2) but user may want to init and use the interrupt for complete 
    *     break operation.
    *     Example for this case:
    *     Init TX interrupt whith "TX - On TX Complete" parameter
    *     SendBreak(0);     - init Break signal transmition
    *         Add your code here to use CPU time
    *     When interrupt appear with UART_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *   Uses static variable to keep registers configuration.
    *
    *******************************************************************************/
    void IMU_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(IMU_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(IMU_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == IMU_SEND_BREAK) ||
                    (retMode == IMU_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    IMU_WriteControlRegister(IMU_ReadControlRegister() |
                                                          IMU_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    IMU_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = IMU_TXSTATUS_REG;
                    }while(tmpStat & IMU_TX_STS_FIFO_EMPTY);
                }

                if( (retMode == IMU_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == IMU_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = IMU_TXSTATUS_REG;
                    }while(~tmpStat & IMU_TX_STS_COMPLETE);
                }

                if( (retMode == IMU_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == IMU_REINIT) ||
                    (retMode == IMU_SEND_WAIT_REINIT) )
                {
                    IMU_WriteControlRegister(IMU_ReadControlRegister() &
                                                         ~IMU_CTRL_HD_SEND_BREAK);
                }

            #else /* IMU_HD_ENABLED Full Duplex mode */

                static uint8 tx_period; 
                
                if( (retMode == IMU_SEND_BREAK) ||
                    (retMode == IMU_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit @ Break signal in Full Duplex mode*/
                    if( (IMU_PARITY_TYPE != IMU__B_UART__NONE_REVB) ||
                         IMU_PARITY_TYPE_SW )
                    {
                        IMU_WriteControlRegister(IMU_ReadControlRegister() |
                                                              IMU_CTRL_HD_SEND_BREAK);
                    }                                                          

                    #if(IMU_TXCLKGEN_DP)
                        tx_period = IMU_TXBITCLKTX_COMPLETE_REG;
                        IMU_TXBITCLKTX_COMPLETE_REG = IMU_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = IMU_TXBITCTR_PERIOD_REG;
                        IMU_TXBITCTR_PERIOD_REG = IMU_TXBITCTR_BREAKBITS8X;
                    #endif /* End IMU_TXCLKGEN_DP */

                    /* Send zeros*/
                    IMU_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = IMU_TXSTATUS_REG;
                    }while(tmpStat & IMU_TX_STS_FIFO_EMPTY);
                }

                if( (retMode == IMU_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == IMU_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = IMU_TXSTATUS_REG;
                    }while(~tmpStat & IMU_TX_STS_COMPLETE);
                }

                if( (retMode == IMU_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == IMU_REINIT) ||
                    (retMode == IMU_SEND_WAIT_REINIT) )
                {

                    #if(IMU_TXCLKGEN_DP)
                        IMU_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        IMU_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End IMU_TXCLKGEN_DP */

                    if( (IMU_PARITY_TYPE != IMU__B_UART__NONE_REVB) || 
                         IMU_PARITY_TYPE_SW )
                    {
                        IMU_WriteControlRegister(IMU_ReadControlRegister() &
                                                             ~IMU_CTRL_HD_SEND_BREAK);
                    }                                     
                }
            #endif    /* End IMU_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: IMU_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the transmit addressing mode
    *
    * Parameters:
    *  addressMode: 0 -> Space
    *               1 -> Mark
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void IMU_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0)
        {
            IMU_WriteControlRegister(IMU_ReadControlRegister() | IMU_CTRL_MARK);
        }
        else
        {
            IMU_WriteControlRegister(IMU_ReadControlRegister() & ~IMU_CTRL_MARK);
        }
    }

#endif  /* EndIMU_TX_ENABLED */

#if(IMU_HD_ENABLED)


    /*******************************************************************************
    * Function Name: IMU_LoadTxConfig
    ********************************************************************************
    *
    * Summary:
    *  Unloads the Rx configuration if required and loads the
    *  Tx configuration. It is the users responsibility to ensure that any
    *  transaction is complete and it is safe to unload the Tx
    *  configuration.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Valid only for half duplex UART. 
    *
    * Side Effects:
    *  Disable RX interrupt mask, when software buffer has been used.
    *
    *******************************************************************************/
    void IMU_LoadTxConfig(void) 
    {
        #if((IMU_RX_INTERRUPT_ENABLED) && (IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            IMU_SetRxInterruptMode(0);
        #endif /* IMU_RX_INTERRUPT_ENABLED */

        IMU_WriteControlRegister(IMU_ReadControlRegister() | IMU_CTRL_HD_SEND);
        IMU_RXBITCTR_PERIOD_REG = IMU_HD_TXBITCTR_INIT;
    }


    /*******************************************************************************
    * Function Name: IMU_LoadRxConfig
    ********************************************************************************
    *
    * Summary:
    *  Unloads the Tx configuration if required and loads the
    *  Rx configuration. It is the users responsibility to ensure that any
    *  transaction is complete and it is safe to unload the Rx
    *  configuration.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Valid only for half duplex UART
    *
    * Side Effects:
    *  Set RX interrupt mask based on customizer settings, when software buffer 
    *  has been used.
    *
    *******************************************************************************/
    void IMU_LoadRxConfig(void) 
    {
        #if((IMU_RX_INTERRUPT_ENABLED) && (IMU_RXBUFFERSIZE > IMU_FIFO_LENGTH))
            /* Enable RX interrupt mask before set RX configuration */
            IMU_SetRxInterruptMode(IMU_INIT_RX_INTERRUPTS_MASK);    
        #endif /* IMU_RX_INTERRUPT_ENABLED */
        
        IMU_WriteControlRegister(IMU_ReadControlRegister() & ~IMU_CTRL_HD_SEND);
        IMU_RXBITCTR_PERIOD_REG = IMU_HD_RXBITCTR_INIT;
    }

#endif  /* IMU_HD_ENABLED */


/* [] END OF FILE */
