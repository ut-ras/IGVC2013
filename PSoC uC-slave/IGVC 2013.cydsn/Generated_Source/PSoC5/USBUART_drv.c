/*******************************************************************************
* File Name: USBUART_drv.c
* Version 2.12
*
* Description:
*  Endpoint 0 Driver for the USBFS Component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "USBUART.h"


/***************************************
* Forward references for the EP0 ISR
***************************************/

void  USBUART_HandleSetup(void) ;
void  USBUART_HandleIN(void) ;
void  USBUART_HandleOUT(void) ;
uint8 USBUART_InitControlRead(void) ;
void  USBUART_ControlReadDataStage(void) ;
void  USBUART_ControlReadStatusStage(void) ;
void  USBUART_ControlReadPrematureStatus(void)
                                                ;
uint8 USBUART_InitControlWrite(void) ;
uint8 USBUART_InitZeroLengthControlTransfer(void)
                                                ;
void  USBUART_ControlWriteDataStage(void) ;
void  USBUART_ControlWriteStatusStage(void) ;
void  USBUART_ControlWritePrematureStatus(void)
                                                ;
uint8 USBUART_InitNoDataControlTransfer(void) ;
void  USBUART_NoDataControlStatusStage(void) ;
void  USBUART_InitializeStatusBlock(void) ;
void  USBUART_UpdateStatusBlock(uint8 completionCode) ;


/***************************************
* Request Handlers
***************************************/

uint8 USBUART_HandleStandardRqst(void) ;
uint8 USBUART_DispatchClassRqst(void) ;
uint8 USBUART_HandleVendorRqst(void) ;


/***************************************
* External data references
***************************************/


/***************************************
* Global data allocation
***************************************/

volatile T_USBUART_EP_CTL_BLOCK USBUART_EP[USBUART_MAX_EP];
volatile uint8 USBUART_configuration;
volatile uint8 USBUART_configurationChanged;
volatile uint8 USBUART_deviceAddress;
volatile uint8 USBUART_deviceStatus;
volatile uint8 USBUART_interfaceSetting[USBUART_MAX_INTERFACES_NUMBER];
volatile uint8 USBUART_interfaceSetting_last[USBUART_MAX_INTERFACES_NUMBER];
volatile uint8 USBUART_interfaceStatus[USBUART_MAX_INTERFACES_NUMBER];
volatile uint8 USBUART_device;
const uint8 CYCODE *USBUART_interfaceClass;


/***************************************
* Local data allocation
***************************************/

volatile uint8 USBUART_ep0Toggle;
volatile uint8 USBUART_lastPacketSize;
volatile uint8 USBUART_transferState;
volatile T_USBUART_TD USBUART_currentTD;
volatile uint8 USBUART_ep0Mode;
volatile uint8 USBUART_ep0Count;
volatile uint16 USBUART_transferByteCount;


/*******************************************************************************
* Function Name: USBUART_ep_0_Interrupt
********************************************************************************
*
* Summary:
*  This Interrupt Service Routine handles Endpoint 0 (Control Pipe) traffic.
*  It dispactches setup requests and handles the data and status stages.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
CY_ISR(USBUART_EP_0_ISR)
{
    uint8 bRegTemp;
    uint8 modifyReg;


    bRegTemp = CY_GET_REG8(USBUART_EP0_CR_PTR);
    if (bRegTemp & USBUART_MODE_ACKD)
    {
        modifyReg = 1u;
        if (bRegTemp & USBUART_MODE_SETUP_RCVD)
        {
            if((bRegTemp & USBUART_MODE_MASK) != USBUART_MODE_NAK_IN_OUT)
            {
                modifyReg = 0u;                                     /* When mode not NAK_IN_OUT => invalid setup */
            }
            else
            {
                USBUART_HandleSetup();
                if(USBUART_ep0Mode & USBUART_MODE_SETUP_RCVD)
                {
                    modifyReg = 0u;                         /* if SETUP bit set -> exit without modifying the mode */
                }

            }
        }
        else if (bRegTemp & USBUART_MODE_IN_RCVD)
        {
            USBUART_HandleIN();
        }
        else if (bRegTemp & USBUART_MODE_OUT_RCVD)
        {
            USBUART_HandleOUT();
        }
        else
        {
            modifyReg = 0u;
        }
        if(modifyReg != 0u)
        {
            bRegTemp = CY_GET_REG8(USBUART_EP0_CR_PTR);    /* unlock registers */
            if((bRegTemp & USBUART_MODE_SETUP_RCVD) == 0)  /* Check if SETUP bit is not set, otherwise exit */
            {
                /* Update the count register */
                bRegTemp = USBUART_ep0Toggle | USBUART_ep0Count;
                CY_SET_REG8(USBUART_EP0_CNT_PTR, bRegTemp);
                if(bRegTemp == CY_GET_REG8(USBUART_EP0_CNT_PTR))   /* continue if writing was succesful */
                {
                    do
                    {
                        modifyReg = USBUART_ep0Mode;       /* init temporary variable */
                        /* unlock registers */
                        bRegTemp = CY_GET_REG8(USBUART_EP0_CR_PTR) & USBUART_MODE_SETUP_RCVD;
                        if(bRegTemp == 0)                           /* Check if SETUP bit is not set */
                        {
                            /* Set the Mode Register  */
                            CY_SET_REG8(USBUART_EP0_CR_PTR, USBUART_ep0Mode);
                            /* writing check */
                            modifyReg = CY_GET_REG8(USBUART_EP0_CR_PTR) & USBUART_MODE_MASK;
                        }
                    }while(modifyReg != USBUART_ep0Mode);  /* repeat if writing was not successful */
                }
            }
        }
    }
    /* PSoC3 ES1, ES2 RTC ISR PATCH  */
    #if(CY_PSOC3_ES2 && (USBUART_ep_0__ES2_PATCH))
        USBUART_ISR_PATCH();
    #endif /* End CY_PSOC3_ES2*/
}


/*******************************************************************************
* Function Name: USBUART_HandleSetup
********************************************************************************
*
* Summary:
*  This Routine dispatches requests for the four USB request types
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
void USBUART_HandleSetup(void) 
{
    uint8 requestHandled;

    requestHandled = CY_GET_REG8(USBUART_EP0_CR_PTR);      /* unlock registers */
    CY_SET_REG8(USBUART_EP0_CR_PTR, requestHandled);       /* clear setup bit */
    requestHandled = CY_GET_REG8(USBUART_EP0_CR_PTR);      /* reread register */
    if(requestHandled & USBUART_MODE_SETUP_RCVD)
    {
        USBUART_ep0Mode = requestHandled;        /* if SETUP bit set -> exit without modifying the mode */
    }
    else
    {
        /* In case the previous transfer did not complete, close it out */
        USBUART_UpdateStatusBlock(USBUART_XFER_PREMATURE);

        switch (CY_GET_REG8(USBUART_bmRequestType) & USBUART_RQST_TYPE_MASK)
        {
            case USBUART_RQST_TYPE_STD:
                requestHandled = USBUART_HandleStandardRqst();
                break;
            case USBUART_RQST_TYPE_CLS:
                requestHandled = USBUART_DispatchClassRqst();
                break;
            case USBUART_RQST_TYPE_VND:
                requestHandled = USBUART_HandleVendorRqst();
                break;
            default:
                requestHandled = USBUART_FALSE;
                break;
        }
        if (requestHandled == USBUART_FALSE)
        {
            USBUART_ep0Mode = USBUART_MODE_STALL_IN_OUT;
        }
    }
}


/*******************************************************************************
* Function Name: USBUART_HandleIN
********************************************************************************
*
* Summary:
*  This routine handles EP0 IN transfers.
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
void USBUART_HandleIN(void) 
{
    switch (USBUART_transferState)
    {
        case USBUART_TRANS_STATE_IDLE:
            break;
        case USBUART_TRANS_STATE_CONTROL_READ:
            USBUART_ControlReadDataStage();
            break;
        case USBUART_TRANS_STATE_CONTROL_WRITE:
            USBUART_ControlWriteStatusStage();
            break;
        case USBUART_TRANS_STATE_NO_DATA_CONTROL:
            USBUART_NoDataControlStatusStage();
            break;
        default:    /* there are no more states */
            break;
    }
}


/*******************************************************************************
* Function Name: USBUART_HandleOUT
********************************************************************************
*
* Summary:
*  This routine handles EP0 OUT transfers.
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
void USBUART_HandleOUT(void) 
{
    switch (USBUART_transferState)
    {
        case USBUART_TRANS_STATE_IDLE:
            break;
        case USBUART_TRANS_STATE_CONTROL_READ:
            USBUART_ControlReadStatusStage();
            break;
        case USBUART_TRANS_STATE_CONTROL_WRITE:
            USBUART_ControlWriteDataStage();
            break;
        case USBUART_TRANS_STATE_NO_DATA_CONTROL:
            /* Update the completion block */
            USBUART_UpdateStatusBlock(USBUART_XFER_ERROR);
            /* We expect no more data, so stall INs and OUTs */
            USBUART_ep0Mode = USBUART_MODE_STALL_IN_OUT;
            break;
        default:    /* there are no more states */
            break;
    }
}


/*******************************************************************************
* Function Name: USBUART_LoadEP0
********************************************************************************
*
* Summary:
*  This routine loads the EP0 data registers for OUT transfers.  It uses the
*  currentTD (previously initialized by the _InitControlWrite function and
*  updated for each OUT transfer, and the bLastPacketSize) to determine how
*  many uint8s to transfer on the current OUT.
*
*  If the number of uint8s remaining is zero and the last transfer was full,
*  we need to send a zero length packet.  Otherwise we send the minimum
*  of the control endpoint size (8) or remaining number of uint8s for the
*  transaction.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  USBUART_transferByteCount - Update the transfer byte count from the
*     last transaction.
*  USBUART_ep0Count - counts the data loaded to the SIE memory in
*     current packet.
*  USBUART_lastPacketSize - remebers the USBFS_ep0Count value for the
*     next packet.
*  USBUART_transferByteCount - suma of the previos bytes transferred
*     on previous packets(suma of USBFS_lastPacketSize)
*  USBUART_ep0Toggle - inverted
*  USBUART_ep0Mode  - prepare for mode register content.
*  USBUART_transferState - set to TRANS_STATE_CONTROL_READ
*
* Reentrant:
*  No.
*
*******************************************************************************/
void USBUART_LoadEP0(void) 
{
    /* Update the transfer byte count from the last transaction */
    USBUART_transferByteCount += USBUART_lastPacketSize;
    /* Now load the next transaction */
    USBUART_ep0Count = 0u;
    while ((USBUART_currentTD.count > 0u) && (USBUART_ep0Count < 8u))
    {
        USBUART_EP0_DR0_PTR[USBUART_ep0Count] = *USBUART_currentTD.pData++;
        USBUART_ep0Count++;
        USBUART_currentTD.count--;
    }
    /* Support zero-length packet*/
    if( (USBUART_ep0Count > 0u) || (USBUART_lastPacketSize == 8u) )
    {
        /* Update the data toggle */
        USBUART_ep0Toggle ^= USBUART_EP0_CNT_DATA_TOGGLE;
        /* Set the Mode Register  */
        USBUART_ep0Mode = USBUART_MODE_ACK_IN_STATUS_OUT;
        /* Update the state (or stay the same) */
        USBUART_transferState = USBUART_TRANS_STATE_CONTROL_READ;
    }
    else
    {
        /* Expect Status Stage Out */
        USBUART_ep0Mode = USBUART_MODE_STATUS_OUT_ONLY;
        /* Update the state (or stay the same) */
        USBUART_transferState = USBUART_TRANS_STATE_CONTROL_READ;
    }

    /* Save the packet size for next time */
    USBUART_lastPacketSize = USBUART_ep0Count;
}


/*******************************************************************************
* Function Name: USBUART_InitControlRead
********************************************************************************
*
* Summary:
*  Initialize a control read transaction, usable to send data to the host.
*  The following global variables shold be initialized before this function
*  called. To send zero length packet use InitZeroLengthControlTransfer
*  function.
*
* Parameters:
*  None.
*
* Return:
*  requestHandled state.
*
* Global variables:
*  USBUART_currentTD.count - counts of data to be sent.
*  USBUART_currentTD.pData - data pointer.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 USBUART_InitControlRead(void) 
{
    uint16 xferCount;
    if(USBUART_currentTD.count == 0)
    {
        USBUART_InitZeroLengthControlTransfer();
    }
    else
    {
        /* Set up the state machine */
        USBUART_transferState = USBUART_TRANS_STATE_CONTROL_READ;
        /* Set the toggle, it gets updated in LoadEP */
        USBUART_ep0Toggle = 0u;
        /* Initialize the Status Block */
        USBUART_InitializeStatusBlock();
        xferCount = (((uint16)CY_GET_REG8(USBUART_lengthHi) << 8u) | (CY_GET_REG8(USBUART_lengthLo)));

        if (USBUART_currentTD.count > xferCount)
        {
            USBUART_currentTD.count = xferCount;
        }
        USBUART_LoadEP0();
    }

    return(USBUART_TRUE);
}


/*******************************************************************************
* Function Name: USBUART_InitZeroLengthControlTransfer
********************************************************************************
*
* Summary:
*  Initialize a zero length data IN transfer.
*
* Parameters:
*  None.
*
* Return:
*  requestHandled state.
*
* Global variables:
*  USBUART_ep0Toggle - set to EP0_CNT_DATA_TOGGLE
*  USBUART_ep0Mode  - prepare for mode register content.
*  USBUART_transferState - set to TRANS_STATE_CONTROL_READ
*  USBUART_ep0Count - cleared, means the zero-length packet.
*  USBUART_lastPacketSize - cleared.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 USBUART_InitZeroLengthControlTransfer(void)
                                                
{
    /* Update the state */
    USBUART_transferState = USBUART_TRANS_STATE_CONTROL_READ;
    /* Set the data toggle */
    USBUART_ep0Toggle = USBUART_EP0_CNT_DATA_TOGGLE;
    /* Set the Mode Register  */
    USBUART_ep0Mode = USBUART_MODE_ACK_IN_STATUS_OUT;
    /* Save the packet size for next time */
    USBUART_lastPacketSize = 0u;
    USBUART_ep0Count = 0u;

    return(USBUART_TRUE);
}


/*******************************************************************************
* Function Name: USBUART_ControlReadDataStage
********************************************************************************
*
* Summary:
*  Handle the Data Stage of a control read transfer.
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
void USBUART_ControlReadDataStage(void) 

{
    USBUART_LoadEP0();
}


/*******************************************************************************
* Function Name: USBUART_ControlReadStatusStage
********************************************************************************
*
* Summary:
*  Handle the Status Stage of a control read transfer.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  USBUART_USBFS_transferByteCount - updated with last packet size.
*  USBUART_transferState - set to TRANS_STATE_IDLE.
*  USBUART_ep0Mode  - set to MODE_STALL_IN_OUT.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void USBUART_ControlReadStatusStage(void) 
{
    /* Update the transfer byte count */
    USBUART_transferByteCount += USBUART_lastPacketSize;
    /* Go Idle */
    USBUART_transferState = USBUART_TRANS_STATE_IDLE;
    /* Update the completion block */
    USBUART_UpdateStatusBlock(USBUART_XFER_STATUS_ACK);
     /* We expect no more data, so stall INs and OUTs */
    USBUART_ep0Mode =  USBUART_MODE_STALL_IN_OUT;
}


/*******************************************************************************
* Function Name: USBUART_InitControlWrite
********************************************************************************
*
* Summary:
*  Initialize a control write transaction
*
* Parameters:
*  None.
*
* Return:
*  requestHandled state.
*
* Global variables:
*  USBUART_USBFS_transferState - set to TRANS_STATE_CONTROL_WRITE
*  USBUART_ep0Toggle - set to EP0_CNT_DATA_TOGGLE
*  USBUART_ep0Mode  - set to MODE_ACK_OUT_STATUS_IN
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 USBUART_InitControlWrite(void) 
{
    uint16 xferCount;

    /* Set up the state machine */
    USBUART_transferState = USBUART_TRANS_STATE_CONTROL_WRITE;
    /* This migh not be necessary */
    USBUART_ep0Toggle = USBUART_EP0_CNT_DATA_TOGGLE;
    /* Initialize the Status Block */
    USBUART_InitializeStatusBlock();

    xferCount = (((uint16)CY_GET_REG8(USBUART_lengthHi) << 8u) | (CY_GET_REG8(USBUART_lengthLo)));

    if (USBUART_currentTD.count > xferCount)
    {
        USBUART_currentTD.count = xferCount;
    }

    /* Expect Data or Status Stage */
    USBUART_ep0Mode = USBUART_MODE_ACK_OUT_STATUS_IN;

    return(USBUART_TRUE);
}


/*******************************************************************************
* Function Name: USBUART_ControlWriteDataStage
********************************************************************************
*
* Summary:
*  Handle the Data Stage of a control write transfer
*       1. Get the data (We assume the destination was validated previously)
*       2. Update the count and data toggle
*       3. Update the mode register for the next transaction
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  USBUART_transferByteCount - Update the transfer byte count from the
*    last transaction.
*  USBUART_ep0Count - counts the data loaded from the SIE memory
*    in current packet.
*  USBUART_transferByteCount - suma of the previos bytes transferred
*    on previous packets(suma of USBFS_lastPacketSize)
*  USBUART_ep0Toggle - inverted
*  USBUART_ep0Mode  - set to MODE_ACK_OUT_STATUS_IN.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void USBUART_ControlWriteDataStage(void) 
{
    uint8 *pReg = (uint8 *)USBUART_EP0_DR0_PTR;

    USBUART_ep0Count = (CY_GET_REG8(USBUART_EP0_CNT_PTR ) & 0x0Fu) - 2u;

    USBUART_transferByteCount += USBUART_ep0Count;

    while ((USBUART_currentTD.count > 0u) && (USBUART_ep0Count > 0u))
    {
        *USBUART_currentTD.pData++ = CY_GET_REG8(pReg++);
        USBUART_ep0Count--;
        USBUART_currentTD.count--;
    }
    /* Update the data toggle */
    USBUART_ep0Toggle ^= USBUART_EP0_CNT_DATA_TOGGLE;
    /* Expect Data or Status Stage */
    USBUART_ep0Mode = USBUART_MODE_ACK_OUT_STATUS_IN;
}


/*******************************************************************************
* Function Name: USBUART_ControlWriteStatusStage
********************************************************************************
*
* Summary:
*  Handle the Status Stage of a control write transfer
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  USBUART_transferState - set to TRANS_STATE_IDLE.
*  USBUART_USBFS_ep0Mode  - set to MODE_STALL_IN_OUT.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void USBUART_ControlWriteStatusStage(void) 
{
    /* Go Idle */
    USBUART_transferState = USBUART_TRANS_STATE_IDLE;
    /* Update the completion block */
    USBUART_UpdateStatusBlock(USBUART_XFER_STATUS_ACK);
     /* We expect no more data, so stall INs and OUTs */
    USBUART_ep0Mode = USBUART_MODE_STALL_IN_OUT;
}


/*******************************************************************************
* Function Name: USBUART_InitNoDataControlTransfer
********************************************************************************
*
* Summary:
*  Initialize a no data control transfer
*
* Parameters:
*  None.
*
* Return:
*  requestHandled state.
*
* Global variables:
*  USBUART_transferState - set to TRANS_STATE_NO_DATA_CONTROL.
*  USBUART_ep0Mode  - set to MODE_STATUS_IN_ONLY.
*  USBUART_ep0Count - cleared.
*  USBUART_ep0Toggle - set to EP0_CNT_DATA_TOGGLE
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 USBUART_InitNoDataControlTransfer(void) 
{
    USBUART_transferState = USBUART_TRANS_STATE_NO_DATA_CONTROL;
    USBUART_ep0Mode = USBUART_MODE_STATUS_IN_ONLY;
    USBUART_ep0Toggle = USBUART_EP0_CNT_DATA_TOGGLE;
    USBUART_ep0Count = 0u;

    return(USBUART_TRUE);
}


/*******************************************************************************
* Function Name: USBUART_NoDataControlStatusStage
********************************************************************************
* Summary:
*  Handle the Status Stage of a no data control transfer.
*
*  SET_ADDRESS is special, since we need to receive the status stage with
*  the old address.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  USBUART_transferState - set to TRANS_STATE_IDLE.
*  USBUART_ep0Mode  - set to MODE_STALL_IN_OUT.
*  USBUART_ep0Toggle - set to EP0_CNT_DATA_TOGGLE
*  USBUART_deviceAddress - used to set new address and cleared
*
* Reentrant:
*  No.
*
*******************************************************************************/
void USBUART_NoDataControlStatusStage(void) 
{
    /* Change the USB address register if we got a SET_ADDRESS. */
    if (USBUART_deviceAddress != 0u)
    {
        CY_SET_REG8(USBUART_CR0_PTR, USBUART_deviceAddress | USBUART_CR0_ENABLE);
        USBUART_deviceAddress = 0u;
    }
    /* Go Idle */
    USBUART_transferState = USBUART_TRANS_STATE_IDLE;
    /* Update the completion block */
    USBUART_UpdateStatusBlock(USBUART_XFER_STATUS_ACK);
     /* We expect no more data, so stall INs and OUTs */
    USBUART_ep0Mode = USBUART_MODE_STALL_IN_OUT;
}


/*******************************************************************************
* Function Name: USBUART_UpdateStatusBlock
********************************************************************************
*
* Summary:
*  Update the Completion Status Block for a Request.  The block is updated
*  with the completion code the USBUART_transferByteCount.  The
*  StatusBlock Pointer is set to NULL.
*
* Parameters:
*  completionCode - status.
*
* Return:
*  None.
*
* Global variables:
*  USBUART_currentTD.pStatusBlock->status - updated by the 
*    completionCode parameter.
*  USBUART_currentTD.pStatusBlock->length - updated.
*  USBUART_currentTD.pStatusBlock - cleared.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void USBUART_UpdateStatusBlock(uint8 completionCode) 
{
    if (USBUART_currentTD.pStatusBlock != NULL)
    {
        USBUART_currentTD.pStatusBlock->status = completionCode;
        USBUART_currentTD.pStatusBlock->length = USBUART_transferByteCount;
        USBUART_currentTD.pStatusBlock = NULL;
    }
}


/*******************************************************************************
* Function Name: USBUART_InitializeStatusBlock
********************************************************************************
*
* Summary:
*  Initialize the Completion Status Block for a Request.  The completion
*  code is set to USB_XFER_IDLE.
*
*  Also, initializes USBUART_transferByteCount.  Save some space,
*  this is the only consumer.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  USBUART_currentTD.pStatusBlock->status - set to XFER_IDLE.
*  USBUART_currentTD.pStatusBlock->length - cleared.
*  USBUART_transferByteCount - cleared.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void USBUART_InitializeStatusBlock(void) 
{
    USBUART_transferByteCount = 0u;
    if(USBUART_currentTD.pStatusBlock != NULL)
    {
        USBUART_currentTD.pStatusBlock->status = USBUART_XFER_IDLE;
        USBUART_currentTD.pStatusBlock->length = 0u;
    }
}


/* [] END OF FILE */
