/*******************************************************************************
* File Name: Gyro.c
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


#include "Gyro.h"
#include "CyLib.h"


/***************************************
* Global data allocation
***************************************/

#if( Gyro_TX_ENABLED && (Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH))
    volatile uint8 Gyro_txBuffer[Gyro_TXBUFFERSIZE];
    volatile uint8 Gyro_txBufferRead = 0u;
    uint8 Gyro_txBufferWrite = 0u;
#endif /* End Gyro_TX_ENABLED */
#if( ( Gyro_RX_ENABLED || Gyro_HD_ENABLED ) && \
     (Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH) )
    volatile uint8 Gyro_rxBuffer[Gyro_RXBUFFERSIZE];
    volatile uint8 Gyro_rxBufferRead = 0u;
    volatile uint8 Gyro_rxBufferWrite = 0u;
    volatile uint8 Gyro_rxBufferLoopDetect = 0u;
    volatile uint8 Gyro_rxBufferOverflow = 0u;
    #if (Gyro_RXHW_ADDRESS_ENABLED)
        volatile uint8 Gyro_rxAddressMode = Gyro_RXADDRESSMODE;
        volatile uint8 Gyro_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */    
#endif /* End Gyro_RX_ENABLED */


/***************************************
* Local data allocation
***************************************/

uint8 Gyro_initVar = 0u;


/*******************************************************************************
* Function Name: Gyro_Start
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
*  The Gyro_intiVar variable is used to indicate initial 
*  configuration of this component. The variable is initialized to zero (0u) 
*  and set to one (1u) the first time UART_Start() is called. This allows for 
*  component initialization without re-initialization in all subsequent calls 
*  to the Gyro_Start() routine. 
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Gyro_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(Gyro_initVar == 0u)
    {
        Gyro_Init();
        Gyro_initVar = 1u;
    }
    Gyro_Enable();
}


/*******************************************************************************
* Function Name: Gyro_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  Gyro_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void Gyro_Init(void) 
{
    #if(Gyro_RX_ENABLED || Gyro_HD_ENABLED)

        #if(Gyro_RX_INTERRUPT_ENABLED && (Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            CyIntSetVector(Gyro_RX_VECT_NUM,   Gyro_RXISR);
            CyIntSetPriority(Gyro_RX_VECT_NUM, Gyro_RX_PRIOR_NUM);
        #endif /* End Gyro_RX_INTERRUPT_ENABLED */

        #if (Gyro_RXHW_ADDRESS_ENABLED)
            Gyro_SetRxAddressMode(Gyro_RXAddressMode);
            Gyro_SetRxAddress1(Gyro_RXHWADDRESS1);
            Gyro_SetRxAddress2(Gyro_RXHWADDRESS2);
        #endif /* End Gyro_RXHW_ADDRESS_ENABLED */

        /* Configure the Initial RX interrupt mask */
        Gyro_RXSTATUS_MASK_REG  = Gyro_INIT_RX_INTERRUPTS_MASK;
    #endif /* End Gyro_RX_ENABLED || Gyro_HD_ENABLED*/

    #if(Gyro_TX_ENABLED)
        #if(Gyro_TX_INTERRUPT_ENABLED && (Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            CyIntSetVector(Gyro_TX_VECT_NUM,   Gyro_TXISR);
            CyIntSetPriority(Gyro_TX_VECT_NUM, Gyro_TX_PRIOR_NUM);
        #endif /* End Gyro_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(Gyro_TXCLKGEN_DP)
            Gyro_TXBITCLKGEN_CTR_REG = Gyro_BIT_CENTER;
            Gyro_TXBITCLKTX_COMPLETE_REG = (Gyro_NUMBER_OF_DATA_BITS + \
                                                    Gyro_NUMBER_OF_START_BIT) * \
                                                    Gyro_OVER_SAMPLE_COUNT;
        #else
            Gyro_TXBITCTR_COUNTER_REG = (Gyro_NUMBER_OF_DATA_BITS + \
                                                    Gyro_NUMBER_OF_START_BIT) * \
                                                    Gyro_OVER_SAMPLE_8;
        #endif /* End Gyro_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(Gyro_TX_INTERRUPT_ENABLED && (Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH))
            Gyro_TXSTATUS_MASK_REG = Gyro_TX_STS_FIFO_EMPTY;
        #else
            Gyro_TXSTATUS_MASK_REG = Gyro_INIT_TX_INTERRUPTS_MASK;
        #endif /*End Gyro_TX_INTERRUPT_ENABLED*/
        
    #endif /* End Gyro_TX_ENABLED */

    #if(Gyro_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        Gyro_WriteControlRegister( \
            (Gyro_ReadControlRegister() & ~Gyro_CTRL_PARITY_TYPE_MASK) | \
            (Gyro_PARITY_TYPE << Gyro_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End Gyro_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: Gyro_Enable
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
*  Gyro_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void Gyro_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();
    
    #if(Gyro_RX_ENABLED || Gyro_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        Gyro_RXBITCTR_CONTROL_REG |= Gyro_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        Gyro_RXSTATUS_ACTL_REG  |= Gyro_INT_ENABLE;
        #if(Gyro_RX_INTERRUPT_ENABLED && (Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH))
            CyIntEnable(Gyro_RX_VECT_NUM);
            #if (Gyro_RXHW_ADDRESS_ENABLED)
                Gyro_rxAddressDetected = 0u;
            #endif /* End Gyro_RXHW_ADDRESS_ENABLED */
        #endif /* End Gyro_RX_INTERRUPT_ENABLED */
    #endif /* End Gyro_RX_ENABLED || Gyro_HD_ENABLED*/

    #if(Gyro_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!Gyro_TXCLKGEN_DP)
            Gyro_TXBITCTR_CONTROL_REG |= Gyro_CNTR_ENABLE;
        #endif /* End Gyro_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        Gyro_TXSTATUS_ACTL_REG |= Gyro_INT_ENABLE;
        #if(Gyro_TX_INTERRUPT_ENABLED && (Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH))
            CyIntEnable(Gyro_TX_VECT_NUM);
        #endif /* End Gyro_TX_INTERRUPT_ENABLED*/
     #endif /* End Gyro_TX_ENABLED */

    #if(Gyro_INTERNAL_CLOCK_USED)
        /* Set the bit to enable the clock. */
        Gyro_INTCLOCK_CLKEN_REG |= Gyro_INTCLOCK_CLKEN_MASK;
    #endif /* End Gyro_INTERNAL_CLOCK_USED */
    
    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Gyro_Stop
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
void Gyro_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /*Write Bit Counter Disable */
    #if(Gyro_RX_ENABLED || Gyro_HD_ENABLED)
        Gyro_RXBITCTR_CONTROL_REG &= ~Gyro_CNTR_ENABLE;
    #endif /* End Gyro_RX_ENABLED */

    #if(Gyro_TX_ENABLED)
        #if(!Gyro_TXCLKGEN_DP)
            Gyro_TXBITCTR_CONTROL_REG &= ~Gyro_CNTR_ENABLE;
        #endif /* End Gyro_TXCLKGEN_DP */
    #endif /* Gyro_TX_ENABLED */

    #if(Gyro_INTERNAL_CLOCK_USED)
        /* Clear the bit to enable the clock. */
        Gyro_INTCLOCK_CLKEN_REG &= ~Gyro_INTCLOCK_CLKEN_MASK;
    #endif /* End Gyro_INTERNAL_CLOCK_USED */
    
    /*Disable internal interrupt component*/
    #if(Gyro_RX_ENABLED || Gyro_HD_ENABLED)
        Gyro_RXSTATUS_ACTL_REG  &= ~Gyro_INT_ENABLE;
        #if(Gyro_RX_INTERRUPT_ENABLED && (Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH))
            Gyro_DisableRxInt();
        #endif /* End Gyro_RX_INTERRUPT_ENABLED */
    #endif /* End Gyro_RX_ENABLED */
    
    #if(Gyro_TX_ENABLED)
        Gyro_TXSTATUS_ACTL_REG &= ~Gyro_INT_ENABLE;
        #if(Gyro_TX_INTERRUPT_ENABLED && (Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH))
            Gyro_DisableTxInt();
        #endif /* End Gyro_TX_INTERRUPT_ENABLED */
    #endif /* End Gyro_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Gyro_ReadControlRegister
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
uint8 Gyro_ReadControlRegister(void) 
{
    #if( Gyro_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(Gyro_CONTROL_REG);
    #endif /* End Gyro_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: Gyro_WriteControlRegister
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
void  Gyro_WriteControlRegister(uint8 control) 
{
    #if( Gyro_CONTROL_REG_REMOVED )
        control = control;      /* Reassigning to release compiler warning */ 
    #else
       Gyro_CONTROL_REG = control;
    #endif /* End Gyro_CONTROL_REG_REMOVED */
}


#if(Gyro_RX_ENABLED || Gyro_HD_ENABLED)

    #if(Gyro_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Gyro_EnableRxInt
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
        void Gyro_EnableRxInt(void) 
        {
            CyIntEnable(Gyro_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Gyro_DisableRxInt
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
        void Gyro_DisableRxInt(void) 
        {
            CyIntDisable(Gyro_RX_VECT_NUM);
        }

    #endif /* Gyro_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Gyro_SetRxInterruptMode
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
    void Gyro_SetRxInterruptMode(uint8 intSrc) 
    {
        Gyro_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: Gyro_ReadRxData
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
    *  Gyro_rxBuffer - RAM buffer pointer for save received data.
    *  Gyro_rxBufferWrite - cyclic index for write to rxBuffer, 
    *     checked to identify new data. 
    *  Gyro_rxBufferRead - cyclic index for read from rxBuffer, 
    *     incremented after each byte has been read from buffer.
    *  Gyro_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR. 
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Gyro_ReadRxData(void) 
    {
        uint8 rxData;

        #if(Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Gyro_RX_INTERRUPT_ENABLED)
                Gyro_DisableRxInt();
            #endif /* End Gyro_RX_INTERRUPT_ENABLED */

            if( (Gyro_rxBufferRead != Gyro_rxBufferWrite) ||
                (Gyro_rxBufferLoopDetect > 0u) )
            {

                rxData = Gyro_rxBuffer[Gyro_rxBufferRead];

                Gyro_rxBufferRead++;

                if(Gyro_rxBufferRead >= Gyro_RXBUFFERSIZE)
                {
                    Gyro_rxBufferRead = 0u;
                }

                if(Gyro_rxBufferLoopDetect > 0u )
                {
                    Gyro_rxBufferLoopDetect = 0u;
                    #if( (Gyro_RX_INTERRUPT_ENABLED) && (Gyro_FLOW_CONTROL != 0u) && \
                         (Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Gyro_HD_ENABLED )
                            if((Gyro_CONTROL_REG & Gyro_CTRL_HD_SEND) == 0)
                            {   /* In Half duplex mode return RX mask only in RX configuration set, otherwise 
                                *  mask will be returned in LoadRxConfig() API. 
                                */
                                Gyro_RXSTATUS_MASK_REG  |= Gyro_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Gyro_RXSTATUS_MASK_REG  |= Gyro_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Gyro_HD_ENABLED */
                    #endif /* Gyro_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = Gyro_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(Gyro_RX_INTERRUPT_ENABLED)
                Gyro_EnableRxInt();
            #endif /* End Gyro_RX_INTERRUPT_ENABLED */

        #else /* Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = Gyro_RXDATA_REG;

        #endif /* Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Gyro_ReadRxStatus
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
    *  Gyro_rxBufferOverflow - used to indicate overload condition. 
    *   It set to one in RX interrupt when there isn?t free space in 
    *   Gyro_rxBufferRead to write new data. This condition returned 
    *   and cleared to zero by this API as an 
    *   Gyro_RX_STS_SOFT_BUFF_OVER bit along with RX Status register 
    *   bits.
    *
    *******************************************************************************/
    uint8 Gyro_ReadRxStatus(void) 
    {
        uint8 status;

        status = Gyro_RXSTATUS_REG;
        status &= Gyro_RX_HW_MASK;

        #if(Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH)
            if( Gyro_rxBufferOverflow )
            {
                status |= Gyro_RX_STS_SOFT_BUFF_OVER;
                Gyro_rxBufferOverflow = 0u;
            }
        #endif /* Gyro_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: Gyro_GetChar
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
    *  Gyro_rxBuffer - RAM buffer pointer for save received data.
    *  Gyro_rxBufferWrite - cyclic index for write to rxBuffer, 
    *     checked to identify new data. 
    *  Gyro_rxBufferRead - cyclic index for read from rxBuffer, 
    *     incremented after each byte has been read from buffer.
    *  Gyro_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR. 
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Gyro_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Gyro_RX_INTERRUPT_ENABLED)
                Gyro_DisableRxInt();
            #endif /* Gyro_RX_INTERRUPT_ENABLED */

            if( (Gyro_rxBufferRead != Gyro_rxBufferWrite) ||
                (Gyro_rxBufferLoopDetect > 0u) )
            {
                rxData = Gyro_rxBuffer[Gyro_rxBufferRead];

                Gyro_rxBufferRead++;

                if(Gyro_rxBufferRead >= Gyro_RXBUFFERSIZE)
                {
                    Gyro_rxBufferRead = 0u;
                }

                if(Gyro_rxBufferLoopDetect > 0u ) 
                {
                    Gyro_rxBufferLoopDetect = 0u;
                    #if( (Gyro_RX_INTERRUPT_ENABLED) && (Gyro_FLOW_CONTROL != 0u) && \
                         (Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Gyro_HD_ENABLED )
                            if((Gyro_CONTROL_REG & Gyro_CTRL_HD_SEND) == 0)
                            {   /* In Half duplex mode return RX mask only in RX configuration set, otherwise 
                                *  mask will be returned in LoadRxConfig() API. 
                                */
                                Gyro_RXSTATUS_MASK_REG  |= Gyro_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Gyro_RXSTATUS_MASK_REG  |= Gyro_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Gyro_HD_ENABLED */
                    #endif /* Gyro_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus =Gyro_RXSTATUS_REG;
                if(rxStatus & Gyro_RX_STS_FIFO_NOTEMPTY)
                {   /* Read received data from FIFO*/
                    rxData = Gyro_RXDATA_REG;
                    /*Check status on error*/
                    if(rxStatus & (Gyro_RX_STS_BREAK | Gyro_RX_STS_PAR_ERROR |
                                   Gyro_RX_STS_STOP_ERROR | Gyro_RX_STS_OVERRUN))
                    {
                        rxData = 0u;
                    }    
                }
            }

            /* Enable Rx interrupt. */
            #if(Gyro_RX_INTERRUPT_ENABLED)
                Gyro_EnableRxInt();
            #endif /* Gyro_RX_INTERRUPT_ENABLED */

        #else /* Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH */

            rxStatus =Gyro_RXSTATUS_REG;
            if(rxStatus & Gyro_RX_STS_FIFO_NOTEMPTY)
            {   /* Read received data from FIFO*/
                rxData = Gyro_RXDATA_REG;
                /*Check status on error*/
                if(rxStatus & (Gyro_RX_STS_BREAK | Gyro_RX_STS_PAR_ERROR |
                               Gyro_RX_STS_STOP_ERROR | Gyro_RX_STS_OVERRUN))
                {
                    rxData = 0u;
                }
            }
        #endif /* Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Gyro_GetByte
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
    uint16 Gyro_GetByte(void) 
    {
        return ( ((uint16)Gyro_ReadRxStatus() << 8u) | Gyro_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: Gyro_GetRxBufferSize
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
    *  Gyro_rxBufferWrite - used to calculate left bytes. 
    *  Gyro_rxBufferRead - used to calculate left bytes.
    *  Gyro_rxBufferLoopDetect - checked to decide left bytes amount. 
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 Gyro_GetRxBufferSize(void) 
                                                            
    {
        uint8 size;

        #if(Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Gyro_RX_INTERRUPT_ENABLED)
                Gyro_DisableRxInt();
            #endif /* Gyro_RX_INTERRUPT_ENABLED */

            if(Gyro_rxBufferRead == Gyro_rxBufferWrite)
            {
                if(Gyro_rxBufferLoopDetect > 0u)
                {
                    size = Gyro_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(Gyro_rxBufferRead < Gyro_rxBufferWrite)
            {
                size = (Gyro_rxBufferWrite - Gyro_rxBufferRead);
            }
            else
            {
                size = (Gyro_RXBUFFERSIZE - Gyro_rxBufferRead) + Gyro_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(Gyro_RX_INTERRUPT_ENABLED)
                Gyro_EnableRxInt();
            #endif /* End Gyro_RX_INTERRUPT_ENABLED */

        #else /* Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = (Gyro_RXSTATUS_REG & Gyro_RX_STS_FIFO_NOTEMPTY) ? 1u : 0u;

        #endif /* End Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Gyro_ClearRxBuffer
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
    *  Gyro_rxBufferWrite - cleared to zero. 
    *  Gyro_rxBufferRead - cleared to zero.
    *  Gyro_rxBufferLoopDetect - cleared to zero. 
    *  Gyro_rxBufferOverflow - cleared to zero. 
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
    void Gyro_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;
        
        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();        
        Gyro_RXDATA_AUX_CTL_REG |=  Gyro_RX_FIFO_CLR;
        Gyro_RXDATA_AUX_CTL_REG &= ~Gyro_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);
        
        #if(Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Gyro_RX_INTERRUPT_ENABLED)
                Gyro_DisableRxInt();
            #endif /* End Gyro_RX_INTERRUPT_ENABLED */

            Gyro_rxBufferRead = 0u;
            Gyro_rxBufferWrite = 0u;
            Gyro_rxBufferLoopDetect = 0u;
            Gyro_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(Gyro_RX_INTERRUPT_ENABLED)
                Gyro_EnableRxInt();
            #endif /* End Gyro_RX_INTERRUPT_ENABLED */
        #endif /* End Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH */
        
    }


    /*******************************************************************************
    * Function Name: Gyro_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  Gyro__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address 
    *                                               detection
    *  Gyro__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer 
    *                                               address detection
    *  Gyro__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address 
    *                                               detection
    *  Gyro__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer 
    *                                               address detection
    *  Gyro__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Gyro_rxAddressMode - the parameter stored in this variable for 
    *   the farther usage in RX ISR.
    *  Gyro_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void Gyro_SetRxAddressMode(uint8 addressMode)  
                                                        
    {
        #if(Gyro_RXHW_ADDRESS_ENABLED)
            #if(Gyro_CONTROL_REG_REMOVED)
                addressMode = addressMode;
            #else /* Gyro_CONTROL_REG_REMOVED */
                uint8 tmpCtrl = 0u;
                tmpCtrl = Gyro_CONTROL_REG & ~Gyro_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= ((addressMode << Gyro_CTRL_RXADDR_MODE0_SHIFT) & 
                           Gyro_CTRL_RXADDR_MODE_MASK);
                Gyro_CONTROL_REG = tmpCtrl;
                #if(Gyro_RX_INTERRUPT_ENABLED && \
                   (Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH) )
                    Gyro_rxAddressMode = addressMode;
                    Gyro_rxAddressDetected = 0u;
                #endif /* End Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH*/   
            #endif /* End Gyro_CONTROL_REG_REMOVED */
        #else /* Gyro_RXHW_ADDRESS_ENABLED */
            addressMode = addressMode;
        #endif /* End Gyro_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: Gyro_SetRxAddress1
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
    void Gyro_SetRxAddress1(uint8 address) 

    {
        Gyro_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: Gyro_SetRxAddress2
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
    void Gyro_SetRxAddress2(uint8 address) 
    {
        Gyro_RXADDRESS2_REG = address;
    }
        
#endif  /* Gyro_RX_ENABLED || Gyro_HD_ENABLED*/


#if( (Gyro_TX_ENABLED) || (Gyro_HD_ENABLED) )

    #if(Gyro_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Gyro_EnableTxInt
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
        void Gyro_EnableTxInt(void) 
        {
            CyIntEnable(Gyro_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Gyro_DisableTxInt
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
        void Gyro_DisableTxInt(void) 
        {
            CyIntDisable(Gyro_TX_VECT_NUM);
        }

    #endif /* Gyro_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Gyro_SetTxInterruptMode
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
    void Gyro_SetTxInterruptMode(uint8 intSrc) 
    {
        Gyro_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: Gyro_WriteTxData
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
    *  Gyro_txBuffer - RAM buffer pointer for save data for transmission
    *  Gyro_txBufferWrite - cyclic index for write to txBuffer, 
    *    incremented after each byte saved to buffer.
    *  Gyro_txBufferRead - cyclic index for read from txBuffer, 
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  Gyro_initVar - checked to identify that the component has been  
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Gyro_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Gyro_initVar != 0u)
        {
            #if(Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(Gyro_TX_INTERRUPT_ENABLED)
                    Gyro_DisableTxInt();
                #endif /* End Gyro_TX_INTERRUPT_ENABLED */

                if( (Gyro_txBufferRead == Gyro_txBufferWrite) &&
                   !(Gyro_TXSTATUS_REG & Gyro_TX_STS_FIFO_FULL) )
                {
                    /* Add directly to the FIFO. */
                    Gyro_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(Gyro_txBufferWrite >= Gyro_TXBUFFERSIZE)
                    {
                        Gyro_txBufferWrite = 0;
                    }

                    Gyro_txBuffer[Gyro_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    Gyro_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(Gyro_TX_INTERRUPT_ENABLED)
                    Gyro_EnableTxInt();
                #endif /* End Gyro_TX_INTERRUPT_ENABLED */

            #else /* Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                Gyro_TXDATA_REG = txDataByte;

            #endif /* End Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: Gyro_ReadTxStatus
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
    uint8 Gyro_ReadTxStatus(void) 
    {
        return(Gyro_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: Gyro_PutChar
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
    *  Gyro_txBuffer - RAM buffer pointer for save data for transmission
    *  Gyro_txBufferWrite - cyclic index for write to txBuffer, 
    *     checked to identify free space in txBuffer and incremented after each byte 
    *     saved to buffer.
    *  Gyro_txBufferRead - cyclic index for read from txBuffer, 
    *     checked to identify free space in txBuffer.
    *  Gyro_initVar - checked to identify that the component has been  
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void Gyro_PutChar(uint8 txDataByte) 
    {
            #if(Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH)

                /* Block if buffer is full, so we dont overwrite. */
                while( Gyro_txBufferWrite == (Gyro_txBufferRead - 1u) ||
                    (uint8)(Gyro_txBufferWrite - Gyro_txBufferRead) ==
                    (uint8)(Gyro_TXBUFFERSIZE - 1u) )
                {
                    /* Software buffer is full. */
                }
                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(Gyro_TX_INTERRUPT_ENABLED)
                    Gyro_DisableTxInt();
                #endif /* End Gyro_TX_INTERRUPT_ENABLED */

                if( (Gyro_txBufferRead == Gyro_txBufferWrite) &&
                   !(Gyro_TXSTATUS_REG & Gyro_TX_STS_FIFO_FULL) )
                {
                    /* Add directly to the FIFO. */
                    Gyro_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(Gyro_txBufferWrite >= Gyro_TXBUFFERSIZE)
                    {
                        Gyro_txBufferWrite = 0;
                    }

                    Gyro_txBuffer[Gyro_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    Gyro_txBufferWrite++;

                }

                /* Enable Rx interrupt. */
                #if(Gyro_TX_INTERRUPT_ENABLED)
                    Gyro_EnableTxInt();
                #endif /* End Gyro_TX_INTERRUPT_ENABLED */

            #else /* Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH */

                /* Block if there isnt room. */
                while(Gyro_TXSTATUS_REG & Gyro_TX_STS_FIFO_FULL);

                /* Add directly to the FIFO. */
                Gyro_TXDATA_REG = txDataByte;

            #endif /* End Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Gyro_PutString
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
    *  Gyro_initVar - checked to identify that the component has been  
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
    void Gyro_PutString(char* string) 
    {
        /* If not Initialized then skip this function*/
        if(Gyro_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(*string != 0u)
            {
                Gyro_PutChar(*string++);
            }
        }
    }


    /*******************************************************************************
    * Function Name: Gyro_PutArray
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
    *  Gyro_initVar - checked to identify that the component has been  
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Gyro_PutArray(uint8* string, uint8 byteCount) 
                                                                    
    {
        /* If not Initialized then skip this function*/
        if(Gyro_initVar != 0u)
        {
            while(byteCount > 0u)
            {
                Gyro_PutChar(*string++);
                byteCount--;
            }
        }
    }


    /*******************************************************************************
    * Function Name: Gyro_PutCRLF
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
    *  Gyro_initVar - checked to identify that the component has been  
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Gyro_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Gyro_initVar != 0u)
        {
            Gyro_PutChar(txDataByte);
            Gyro_PutChar(0x0Du);
            Gyro_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: Gyro_GetTxBufferSize
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
    *  Gyro_txBufferWrite - used to calculate left space. 
    *  Gyro_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 Gyro_GetTxBufferSize(void) 
                                                            
    {
        uint8 size;

        #if(Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Gyro_TX_INTERRUPT_ENABLED)
                Gyro_DisableTxInt();
            #endif /* End Gyro_TX_INTERRUPT_ENABLED */

            if(Gyro_txBufferRead == Gyro_txBufferWrite)
            {
                size = 0u;
            }
            else if(Gyro_txBufferRead < Gyro_txBufferWrite)
            {
                size = (Gyro_txBufferWrite - Gyro_txBufferRead);
            }
            else
            {
                size = (Gyro_TXBUFFERSIZE - Gyro_txBufferRead) + Gyro_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(Gyro_TX_INTERRUPT_ENABLED)
                Gyro_EnableTxInt();
            #endif /* End Gyro_TX_INTERRUPT_ENABLED */

        #else /* Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH */

            size = Gyro_TXSTATUS_REG;

            /* Is the fifo is full. */
            if(size & Gyro_TX_STS_FIFO_FULL)
            {
                size = Gyro_FIFO_LENGTH;
            }
            else if(size & Gyro_TX_STS_FIFO_EMPTY)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Gyro_ClearTxBuffer
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
    *  Gyro_txBufferWrite - cleared to zero. 
    *  Gyro_txBufferRead - cleared to zero.
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
    void Gyro_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;
        
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();        
        /* clear the HW FIFO */
        Gyro_TXDATA_AUX_CTL_REG |=  Gyro_TX_FIFO_CLR;
        Gyro_TXDATA_AUX_CTL_REG &= ~Gyro_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Gyro_TX_INTERRUPT_ENABLED)
                Gyro_DisableTxInt();
            #endif /* End Gyro_TX_INTERRUPT_ENABLED */

            Gyro_txBufferRead = 0u;
            Gyro_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(Gyro_TX_INTERRUPT_ENABLED)
                Gyro_EnableTxInt();
            #endif /* End Gyro_TX_INTERRUPT_ENABLED */

        #endif /* End Gyro_TXBUFFERSIZE > Gyro_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Gyro_SendBreak
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
    *  Gyro_initVar - checked to identify that the component has been  
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
    void Gyro_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(Gyro_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(Gyro_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == Gyro_SEND_BREAK) ||
                    (retMode == Gyro_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    Gyro_WriteControlRegister(Gyro_ReadControlRegister() |
                                                          Gyro_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    Gyro_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = Gyro_TXSTATUS_REG;
                    }while(tmpStat & Gyro_TX_STS_FIFO_EMPTY);
                }

                if( (retMode == Gyro_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Gyro_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Gyro_TXSTATUS_REG;
                    }while(~tmpStat & Gyro_TX_STS_COMPLETE);
                }

                if( (retMode == Gyro_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Gyro_REINIT) ||
                    (retMode == Gyro_SEND_WAIT_REINIT) )
                {
                    Gyro_WriteControlRegister(Gyro_ReadControlRegister() &
                                                         ~Gyro_CTRL_HD_SEND_BREAK);
                }

            #else /* Gyro_HD_ENABLED Full Duplex mode */

                static uint8 tx_period; 
                
                if( (retMode == Gyro_SEND_BREAK) ||
                    (retMode == Gyro_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit @ Break signal in Full Duplex mode*/
                    if( (Gyro_PARITY_TYPE != Gyro__B_UART__NONE_REVB) ||
                         Gyro_PARITY_TYPE_SW )
                    {
                        Gyro_WriteControlRegister(Gyro_ReadControlRegister() |
                                                              Gyro_CTRL_HD_SEND_BREAK);
                    }                                                          

                    #if(Gyro_TXCLKGEN_DP)
                        tx_period = Gyro_TXBITCLKTX_COMPLETE_REG;
                        Gyro_TXBITCLKTX_COMPLETE_REG = Gyro_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = Gyro_TXBITCTR_PERIOD_REG;
                        Gyro_TXBITCTR_PERIOD_REG = Gyro_TXBITCTR_BREAKBITS8X;
                    #endif /* End Gyro_TXCLKGEN_DP */

                    /* Send zeros*/
                    Gyro_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = Gyro_TXSTATUS_REG;
                    }while(tmpStat & Gyro_TX_STS_FIFO_EMPTY);
                }

                if( (retMode == Gyro_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Gyro_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Gyro_TXSTATUS_REG;
                    }while(~tmpStat & Gyro_TX_STS_COMPLETE);
                }

                if( (retMode == Gyro_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Gyro_REINIT) ||
                    (retMode == Gyro_SEND_WAIT_REINIT) )
                {

                    #if(Gyro_TXCLKGEN_DP)
                        Gyro_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        Gyro_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End Gyro_TXCLKGEN_DP */

                    if( (Gyro_PARITY_TYPE != Gyro__B_UART__NONE_REVB) || 
                         Gyro_PARITY_TYPE_SW )
                    {
                        Gyro_WriteControlRegister(Gyro_ReadControlRegister() &
                                                             ~Gyro_CTRL_HD_SEND_BREAK);
                    }                                     
                }
            #endif    /* End Gyro_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: Gyro_SetTxAddressMode
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
    void Gyro_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0)
        {
            Gyro_WriteControlRegister(Gyro_ReadControlRegister() | Gyro_CTRL_MARK);
        }
        else
        {
            Gyro_WriteControlRegister(Gyro_ReadControlRegister() & ~Gyro_CTRL_MARK);
        }
    }

#endif  /* EndGyro_TX_ENABLED */

#if(Gyro_HD_ENABLED)


    /*******************************************************************************
    * Function Name: Gyro_LoadTxConfig
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
    void Gyro_LoadTxConfig(void) 
    {
        #if((Gyro_RX_INTERRUPT_ENABLED) && (Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            Gyro_SetRxInterruptMode(0);
        #endif /* Gyro_RX_INTERRUPT_ENABLED */

        Gyro_WriteControlRegister(Gyro_ReadControlRegister() | Gyro_CTRL_HD_SEND);
        Gyro_RXBITCTR_PERIOD_REG = Gyro_HD_TXBITCTR_INIT;
    }


    /*******************************************************************************
    * Function Name: Gyro_LoadRxConfig
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
    void Gyro_LoadRxConfig(void) 
    {
        #if((Gyro_RX_INTERRUPT_ENABLED) && (Gyro_RXBUFFERSIZE > Gyro_FIFO_LENGTH))
            /* Enable RX interrupt mask before set RX configuration */
            Gyro_SetRxInterruptMode(Gyro_INIT_RX_INTERRUPTS_MASK);    
        #endif /* Gyro_RX_INTERRUPT_ENABLED */
        
        Gyro_WriteControlRegister(Gyro_ReadControlRegister() & ~Gyro_CTRL_HD_SEND);
        Gyro_RXBITCTR_PERIOD_REG = Gyro_HD_RXBITCTR_INIT;
    }

#endif  /* Gyro_HD_ENABLED */


/* [] END OF FILE */
