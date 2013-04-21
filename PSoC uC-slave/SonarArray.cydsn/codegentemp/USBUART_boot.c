/*******************************************************************************
* File Name: USBUART_boot.c
* Version 2.12
*
* Description:
*  Boot loader API for USBFS Component.
*
*  Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "USBUART.h"

#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_USBUART) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))


/***************************************
*    Bootloader defines
***************************************/

#define USBUART_CyBtLdrStarttimer(X, T)         {USBUART_universalTime = T * 10; X = 0u;}
#define USBUART_CyBtLdrChecktimer(X)            ((X++ < USBUART_universalTime) ? 1u : 0u)

#define USBUART_BTLDR_OUT_EP      (0x01u)
#define USBUART_BTLDR_IN_EP       (0x02u)


/***************************************
*    Bootloader Variables
***************************************/

uint16 USBUART_universalTime = 0u;
uint8 USBUART_started = 0u;


/*******************************************************************************
* Function Name: USBUART_CyBtldrCommStart
********************************************************************************
*
* Summary:
*  Starts the component and enables the interupt.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Side Effects:
*  This function starts the USB with 3V or 5V operation.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void USBUART_CyBtldrCommStart(void) 
{
    CyGlobalIntEnable;      /* Enable Global Interrupts */

    /*Start USBFS Operation/device 0 and with 5V or 3V operation depend on Voltage Congiguration in DWR */
    USBUART_Start(0u, USBUART_DWR_VDDD_OPERATION);

    /* USB componet started, the correct enumeration will be checked in first Read operation */
    USBUART_started = 1u;

}


/*******************************************************************************
* Function Name: USBUART_CyBtldrCommStop.
********************************************************************************
*
* Summary:
*  Disable the component and disable the interrupt.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void USBUART_CyBtldrCommStop(void) 
{
    USBUART_Stop();
}


/*******************************************************************************
* Function Name: USBUART_CyBtldrCommReset.
********************************************************************************
*
* Summary:
*  Resets the receive and transmit communication Buffers.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void USBUART_CyBtldrCommReset(void) 
{
    USBUART_EnableOutEP(USBUART_BTLDR_OUT_EP);  /* Enable the OUT endpoint */
}


/*******************************************************************************
* Function Name: USBUART_CyBtldrCommWrite.
********************************************************************************
*
* Summary:
*  Allows the caller to write data to the boot loader host. The function will
*  handle polling to allow a block of data to be completely sent to the host
*  device.
*
* Parameters:
*  pData:    A pointer to the block of data to send to the device
*  size:     The number of bytes to write.
*  count:    Pointer to an unsigned short variable to write the number of
*             bytes actually written.
*  timeOut:  Number of units to wait before returning because of a timeout.
*
* Return:
*  Returns the value that best describes the problem.
*
* Reentrant:
*  No.
*
*******************************************************************************/
cystatus USBUART_CyBtldrCommWrite(uint8 *pData, uint16 size, uint16 *count, uint8 timeOut) CYSMALL
                                                            
{
    uint16 time;
    cystatus status;

    /* Enable IN transfer */
    USBUART_LoadInEP(USBUART_BTLDR_IN_EP, pData, USBUART_BTLDR_SIZEOF_READ_BUFFER);

    /* Start a timer to wait on. */
    USBUART_CyBtLdrStarttimer(time, timeOut);

    /* Wait for the master to read it. */
    while((USBUART_GetEPState(USBUART_BTLDR_IN_EP) == USBUART_IN_BUFFER_FULL) && \
           USBUART_CyBtLdrChecktimer(time))
    {
        CyDelay(1); /* 1ms delay */
    }

    if (USBUART_GetEPState(USBUART_BTLDR_IN_EP) == USBUART_IN_BUFFER_FULL)
    {
        status = CYRET_TIMEOUT;
    }
    else
    {
        *count = size;
        status = CYRET_SUCCESS;
    }

    return(status);
}


/*******************************************************************************
* Function Name: USBUART_CyBtldrCommRead.
********************************************************************************
*
* Summary:
*  Allows the caller to read data from the boot loader host. The function will
*  handle polling to allow a block of data to be completely received from the
*  host device.
*
* Parameters:
*  pData:    A pointer to the area to store the block of data received
*             from the device.
*  size:     The number of bytes to read.
*  count:    Pointer to an unsigned short variable to write the number
*             of bytes actually read.
*  timeOut:  Number of units to wait before returning because of a timeOut.
*            Timeout is measured in 10s of ms.
*
* Return:
*  Returns the value that best describes the problem.
*
* Reentrant:
*  No.
*
*******************************************************************************/
cystatus USBUART_CyBtldrCommRead(uint8 *pData, uint16 size, uint16 *count, uint8 timeOut) CYSMALL
                                                            
{
    cystatus status;
    uint16 time;

    if(size > USBUART_BTLDR_SIZEOF_WRITE_BUFFER)
    {
        size = USBUART_BTLDR_SIZEOF_WRITE_BUFFER;
    }
    /* Start a timer to wait on. */
    USBUART_CyBtLdrStarttimer(time, timeOut);

    /* Wait on enumeration in first time */
    if(USBUART_started)
    {
        /* Wait for Device to enumerate */
        while(!USBUART_GetConfiguration() && USBUART_CyBtLdrChecktimer(time))
        {
            CyDelay(1); /* 1ms delay */
        }
        /* Enable first OUT, if enumeration complete */
        if(USBUART_GetConfiguration())
        {
            USBUART_CyBtldrCommReset();
            USBUART_started = 0u;
        }
    }

    /* Wait on next packet */
    while((USBUART_GetEPState(USBUART_BTLDR_OUT_EP) != USBUART_OUT_BUFFER_FULL) && \
           USBUART_CyBtLdrChecktimer(time))
    {
        CyDelay(1); /* 1ms delay */
    }

    /* OUT EP has completed */
    if (USBUART_GetEPState(USBUART_BTLDR_OUT_EP) == USBUART_OUT_BUFFER_FULL)
    {
        *count = USBUART_ReadOutEP(USBUART_BTLDR_OUT_EP, pData, size);
        status = CYRET_SUCCESS;
    }
    else
    {
        *count = 0u;
        status = CYRET_TIMEOUT;
    }
    return(status);
}

#endif /* End CYDEV_BOOTLOADER_IO_COMP == CyBtldr_USBUART */


/* [] END OF FILE */
