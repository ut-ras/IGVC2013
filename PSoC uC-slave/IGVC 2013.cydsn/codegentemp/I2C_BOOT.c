/*******************************************************************************
* File Name: I2C_BOOT.c  
* Version 3.1
*
* Description:
*  This file provides the source code of bootloader communication APIs for the 
*  I2C component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "I2C.h"

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (0u != (I2C_MODE & I2C_MODE_SLAVE))  && \
                                                ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_I2C) || \
                                                 (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))

/***************************************
*    Bootloader Variables
***************************************/

/* Writes to this buffer */
uint8 XDATA I2C_slReadBuf[I2C_BTLDR_SIZEOF_READ_BUFFER];

/* Reads from this buffer */
uint8 XDATA I2C_slWriteBuf[I2C_BTLDR_SIZEOF_WRITE_BUFFER];


/***************************************
*    Extern Bootloader Variables
***************************************/

extern volatile uint8 I2C_slStatus;            /* Slave Status  */

/* Transmit buffer variables */
extern volatile uint8   I2C_slRdBufSize;       /* Slave Transmit buffer size */
extern volatile uint8   I2C_slRdBufIndex;      /* Slave Transmit buffer Index */

/* Receive buffer variables */
extern volatile uint8   I2C_slWrBufSize;       /* Slave Receive buffer size */
extern volatile uint8   I2C_slWrBufIndex;      /* Slave Receive buffer Index */


/*******************************************************************************
* Function Name: I2C_CyBtldrCommStart
********************************************************************************
*
* Summary:
*  Starts the communication component and enables the interrupt.
*  The read buffer initial state is full and the read always is 0xFFu.
*  The write buffer is clear and ready to receive a commmand.
*
* Parameters:
*  None
*
* Return:
*  None 
*
* Side Effects:
*  This fucntion enables component interrupt. If I2C is enabled
*  without the interrupt enabled, it could lock up the I2C bus.
* 
* Global variables:
*  I2C_slWriteBuf - used to store received command.
*  I2C_slReadBuf - used to store response.
*  I2C_slRdBufIndex - used to store current index within slave
*  read buffer.
*
*******************************************************************************/
void I2C_CyBtldrCommStart(void) CYSMALL 
{
    /* Set Write buffer */
    I2C_SlaveInitWriteBuf(I2C_slWriteBuf, I2C_BTLDR_SIZEOF_WRITE_BUFFER);
    
    /* Set Read buffer and make it full */
    I2C_SlaveInitReadBuf(I2C_slReadBuf, 0u);
    
    /* Enable power to I2C Module */
    I2C_Start();
}


/*******************************************************************************
* Function Name: I2C_CyBtldrCommStop
********************************************************************************
*
* Summary:
*  Disables the communication component and disables the interrupt.
*
* Parameters:
*  None 
*
* Return:
*  None 
*
*******************************************************************************/
void I2C_CyBtldrCommStop(void) CYSMALL 
{
    /* Stop I2C component */
    I2C_Stop();
}


/*******************************************************************************
* Function Name: I2C_CyBtldrCommReset
********************************************************************************
*
* Summary:
*  Set buffers to the initial state and reset the statuses.
*  The read buffer initial state is full and the read always is 0xFFu.
*  The write buffer is clear and ready to receive a commmand.
*
* Parameters:
*  None 
*
* Return:
*  None
*  
* Global variables:
*  I2C_slRdBufIndex - used to store current index within slave
*  read buffer.
*
*******************************************************************************/
void I2C_CyBtldrCommReset(void) CYSMALL 
{
    /* Make the Read buffer full */
    I2C_slRdBufSize = 0u;
    
    /* Reset Write buffer and Read buffer */
    I2C_slRdBufIndex = 0u;
    I2C_slWrBufIndex = 0u;
       
    /* Clear read and write status */
    I2C_slStatus = 0u;
}


/*******************************************************************************
* Function Name: I2C_CyBtldrCommWrite
********************************************************************************
*
* Summary:
*  Transmits the status of executed command to the Host. 
*  The function updates the I2C read buffer with response and realeases it to
*  the host. All reads return 0xFF till the buffer will be released. All bytes
*  are transfered by the I2C ISR.
*  The function waits with timeout till all bytes will be read.
*  After exist this function the reads return 0xFF.
*
* Parameters:
*  Data:     pointer to data buffer with response command.
*  Size:     number of bytes required to be transmitted.
*  Count:    actual size of data was transmitted.
*  TimeOut:  timeout value in tries of 10uS.
*
* Return:
*  Status of transmit operation.
* 
* Global variables:
*  I2C_slReadBuf - used to store response.
*  I2C_slRdBufIndex - used to store current index within slave
*  read buffer.
*
*******************************************************************************/
cystatus I2C_CyBtldrCommWrite(uint8 * Data, uint16 size, uint16 * count, uint8 timeOut) CYSMALL 
         
{
    cystatus status = CYRET_BAD_PARAM;  /* Initialize as bad parameters */
    uint16 timeoutMs;                   /* Timeout in mS */
    
    /* Check that correct buffer is provided by bootloader */
    if ((NULL != Data) && (size > 0u))
    {
        timeoutMs = 10u * timeOut;  /* To be in 10mS units, really check 1mS * 10 */
        status = CYRET_TIMEOUT;     /* Fail due timeout */
        *count = size;              /* The size only be transmitted, all other will be 0xFFu */
        
        /* Copy response to the buffer */
        memcpy(I2C_slReadBuf, Data, size);
        
        /* The buffer is free now */
        I2C_slRdBufSize = (uint8) size;
        
        /* Wait till response will be read */
        while (0u != timeoutMs--)
        {
            /* Check if host complete a reading */
            if (I2C_slRdBufIndex == size)
            {
                I2C_slRdBufSize = 0u;
                I2C_slRdBufIndex = 0u;
                
                status = CYRET_SUCCESS;
                break;
            }
            
            CyDelay(1u); /* Wait 1mS for data to become available */
        }
    }
    
    return (status);
}


/*******************************************************************************
* Function Name: I2C_CyBtldrCommRead
********************************************************************************
*
* Summary:
*  Receives the command from the Host.
*  All bytes are received by the I2C ISR and stored in internal I2C buffer. The
*  function checks status with timeout to detemine the end of transfer and
*  then copy data to bootloader buffer.
*  After exist this function the I2C ISR is able to receive more data.
*
* Parameters:
*  Data:     pointer to data buffer to store command.
*  Size:     maximum number of bytes which could to be passed back.
*  Count:    actual size of data was received.
*  TimeOut:  timeout value in tries of 10uS.
*
* Return:
*  Status of receive operation.
*
* Global variables:
*  I2C_slWriteBuf - used to store received command.
*
*******************************************************************************/
cystatus I2C_CyBtldrCommRead(uint8 * Data, uint16 size, uint16 * count, uint8 timeOut) CYSMALL 
         
{
    cystatus status = CYRET_BAD_PARAM;  /* Initialize as bad parameters */
    uint16 timeoutMs;                   /* Timeout in mS */
    uint8 byteCount;                    /* Number of bytes that the host has been written */
    
    /* Check that correct buffer is provided by bootloader */
    if ((NULL != Data) && (size > 0u))
    {
        timeoutMs = 10u * timeOut;  /* To be in 10mS units, really check 1mS * 10 */
        status = CYRET_TIMEOUT;     /* Fail due timeout */
    
        /* Wait for Command from the host */
        while (0u != timeoutMs--)
        {
            /* Check if the host complete write */
            if (0u != (I2C_slStatus & I2C_SSTAT_WR_CMPLT))
            {
                /* How many bytes the host has been written */
                byteCount = I2C_slWrBufIndex;
                *count = (uint16) byteCount;
                
                /* Copy to command for the host to bootloader buffer */
                memcpy(Data, I2C_slWriteBuf,  I2C_MIN_UINT16(byteCount, size));
                
                /* Clear I2C write buffer and status */
                I2C_slStatus = 0u;
                I2C_slWrBufIndex = 0u;
                
                status = CYRET_SUCCESS;
                break;
            }
            
            CyDelay(1u); /* Wait 1mS for data to become available */
        }
    }
    
    return (status);
}

#endif /* End if (CYDEV_BOOTLOADER_IO_COMP) && (I2C_MODE == I2C_MODE_SLAVE) && \
                                               ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_I2C) || \
                                                (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface)) */


/* [] END OF FILE */
