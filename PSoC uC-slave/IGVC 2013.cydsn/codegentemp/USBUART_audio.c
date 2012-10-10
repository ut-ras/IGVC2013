/*******************************************************************************
* File Name: USBUART_audio.c
* Version 2.12
*
* Description:
*  USB AUDIO Class request handler.
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

#if defined(USBUART_ENABLE_AUDIO_CLASS)

#include "USBUART_audio.h"
#include "USBUART_midi.h"


/***************************************
*    AUDIO Variables
***************************************/

#if defined(USBUART_ENABLE_AUDIO_STREAMING)
    volatile uint8 USBUART_currentSampleFrequency[USBUART_MAX_EP][USBUART_SAMPLE_FREQ_LEN];
    volatile uint8 USBUART_frequencyChanged;
    volatile uint8 USBUART_currentMute;
    volatile uint8 USBUART_currentVolume[USBUART_VOLUME_LEN];
    volatile uint8 USBUART_minimumVolume[] = {0x01, 0x80};
    volatile uint8 USBUART_maximumVolume[] = {0xFF, 0x7F};
    volatile uint8 USBUART_resolutionVolume[] = {0x01, 0x00};
#endif /* End USBUART_ENABLE_AUDIO_STREAMING */


/***************************************
* Custom Declarations
***************************************/

/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */


/***************************************
*    External references
***************************************/

uint8 USBUART_InitControlRead(void) ;
uint8 USBUART_InitControlWrite(void) ;
uint8 USBUART_InitNoDataControlTransfer(void) ;
uint8 USBUART_InitZeroLengthControlTransfer(void)
                                                ;

extern volatile T_USBUART_EP_CTL_BLOCK USBUART_EP[];
extern volatile T_USBUART_TD USBUART_currentTD;


/***************************************
*    Private function prototypes
***************************************/


/*******************************************************************************
* Function Name: USBUART_DispatchAUDIOClassRqst
********************************************************************************
*
* Summary:
*  This routine dispatches class requests
*
* Parameters:
*  None.
*
* Return:
*  requestHandled
*
* Global variables:
*   USBUART_currentSampleFrequency: Contains the current audio Sample
*       Frequency. It is set by the Host using SET_CUR request to the endpoint.
*   USBUART_frequencyChanged: This variable is used as a flag for the
*       user code, to be aware that Host has been sent request for changing
*       Sample Frequency. Sample frequency will be sent on the next OUT
*       transaction. It is contains endpoint address when set. The following
*       code is recommended for detecting new Sample Frequency in main code:
*       if((USBUART_frequencyChanged != 0) &&
*       (USBUART_transferState == USBUART_TRANS_STATE_IDLE))
*       {
*          USBUART_frequencyChanged = 0;
*       }
*       USBUART_transferState variable is checked to be sure that
*             transfer completes.
*   USBUART_currentMute: Contains mute configuration set by Host.
*   USBUART_currentVolume: Contains volume level set by Host.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 USBUART_DispatchAUDIOClassRqst() 
{
    uint8 requestHandled = USBUART_FALSE;
    
    #if defined(USBUART_ENABLE_AUDIO_STREAMING)
        uint8 epNumber;
        epNumber = CY_GET_REG8(USBUART_wIndexLo) & USBUART_DIR_UNUSED;
    #endif /* End USBUART_ENABLE_AUDIO_STREAMING */

    if ((CY_GET_REG8(USBUART_bmRequestType) & USBUART_RQST_DIR_MASK) == USBUART_RQST_DIR_D2H)
    {
        /* Control Read */
        if((CY_GET_REG8(USBUART_bmRequestType) & USBUART_RQST_RCPT_MASK) == \
                                                                                    USBUART_RQST_RCPT_EP)
        {
            /* Endpoint */
            switch (CY_GET_REG8(USBUART_bRequest))
            {
                case USBUART_GET_CUR:
                #if defined(USBUART_ENABLE_AUDIO_STREAMING)
                    if(CY_GET_REG8(USBUART_wValueHi) == USBUART_SAMPLING_FREQ_CONTROL)
                    {
                         /* Endpoint Control Selector is Sampling Frequency */
                        USBUART_currentTD.wCount = USBUART_SAMPLE_FREQ_LEN;
                        USBUART_currentTD.pData  = USBUART_currentSampleFrequency[epNumber];
                        requestHandled   = USBUART_InitControlRead();
                    }
                #endif /* End USBUART_ENABLE_AUDIO_STREAMING */

                /* `#START AUDIO_READ_REQUESTS` Place other request handler here */

                /* `#END` */
                    break;
                default:
                    break;
            }
        }
        else if((CY_GET_REG8(USBUART_bmRequestType) & USBUART_RQST_RCPT_MASK) == \
                                                                                    USBUART_RQST_RCPT_IFC)
        {
            /* Interface or Entity ID */
            switch (CY_GET_REG8(USBUART_bRequest))
            {
                case USBUART_GET_CUR:
                #if defined(USBUART_ENABLE_AUDIO_STREAMING)
                    if(CY_GET_REG8(USBUART_wValueHi) == USBUART_MUTE_CONTROL)
                    {
                         /* Entity ID Control Selector is MUTE */
                        USBUART_currentTD.wCount = 1;
                        USBUART_currentTD.pData  = &USBUART_currentMute;
                        requestHandled   = USBUART_InitControlRead();
                    }
                    else if(CY_GET_REG8(USBUART_wValueHi) == USBUART_VOLUME_CONTROL)
                    {
                         /* Entity ID Control Selector is VOLUME, */
                        USBUART_currentTD.wCount = USBUART_VOLUME_LEN;
                        USBUART_currentTD.pData  = USBUART_currentVolume;
                        requestHandled   = USBUART_InitControlRead();
                    }
                    else
                    {
                    }
                    break;
                case USBUART_GET_MIN:    /* GET_MIN */
                    if(CY_GET_REG8(USBUART_wValueHi) == USBUART_VOLUME_CONTROL)
                    {
                         /* Entity ID Control Selector is VOLUME, */
                        USBUART_currentTD.wCount = USBUART_VOLUME_LEN;
                        USBUART_currentTD.pData  = &USBUART_minimumVolume[0];
                        requestHandled   = USBUART_InitControlRead();
                    }
                    break;
                case USBUART_GET_MAX:    /* GET_MAX */
                    if(CY_GET_REG8(USBUART_wValueHi) == USBUART_VOLUME_CONTROL)
                    {
                             /* Entity ID Control Selector is VOLUME, */
                        USBUART_currentTD.wCount = USBUART_VOLUME_LEN;
                        USBUART_currentTD.pData  = &USBUART_maximumVolume[0];
                        requestHandled   = USBUART_InitControlRead();
                    }
                    break;
                case USBUART_GET_RES:    /* GET_RES */
                    if(CY_GET_REG8(USBUART_wValueHi) == USBUART_VOLUME_CONTROL)
                    {
                         /* Entity ID Control Selector is VOLUME, */
                        USBUART_currentTD.wCount = USBUART_VOLUME_LEN;
                        USBUART_currentTD.pData  = &USBUART_resolutionVolume[0];
                        requestHandled   = USBUART_InitControlRead();
                    }
                    break;
                /* The contents of the status message is reserved for future use.
                *  For the time being, a null packet should be returned in the data stage of the 
                *  control transfer, and the received null packet should be ACKed.
                */
                case USBUART_GET_STAT:
                        USBUART_currentTD.wCount = 0;
                        requestHandled   = USBUART_InitControlWrite();

                #endif /* End USBUART_ENABLE_AUDIO_STREAMING */

                /* `#START AUDIO_WRITE_REQUESTS` Place other request handler here */

                /* `#END` */
                    break;
                default:
                    break;
            }
        }
        else
        {   /* USBUART_RQST_RCPT_OTHER */
        }
    }
    else if ((CY_GET_REG8(USBUART_bmRequestType) & USBUART_RQST_DIR_MASK) == \
                                                                                    USBUART_RQST_DIR_H2D)
    {
        /* Control Write */
        if((CY_GET_REG8(USBUART_bmRequestType) & USBUART_RQST_RCPT_MASK) == \
                                                                                    USBUART_RQST_RCPT_EP)
        {
            /* Endpoint */
            switch (CY_GET_REG8(USBUART_bRequest))
            {
                case USBUART_SET_CUR:
                #if defined(USBUART_ENABLE_AUDIO_STREAMING)
                    if(CY_GET_REG8(USBUART_wValueHi) == USBUART_SAMPLING_FREQ_CONTROL)
                    {
                         /* Endpoint Control Selector is Sampling Frequency */
                        USBUART_currentTD.wCount = USBUART_SAMPLE_FREQ_LEN;
                        USBUART_currentTD.pData  = USBUART_currentSampleFrequency[epNumber];
                        requestHandled   = USBUART_InitControlWrite();
                        USBUART_frequencyChanged = epNumber;
                    }
                #endif /* End USBUART_ENABLE_AUDIO_STREAMING */

                /* `#START AUDIO_SAMPLING_FREQ_REQUESTS` Place other request handler here */

                /* `#END` */
                    break;
                default:
                    break;
            }
        }
        else if((CY_GET_REG8(USBUART_bmRequestType) & USBUART_RQST_RCPT_MASK) == \
                                                                                    USBUART_RQST_RCPT_IFC)
        {
            /* Interface or Entity ID */
            switch (CY_GET_REG8(USBUART_bRequest))
            {
                case USBUART_SET_CUR:
                #if defined(USBUART_ENABLE_AUDIO_STREAMING)
                    if(CY_GET_REG8(USBUART_wValueHi) == USBUART_MUTE_CONTROL)
                    {
                         /* Entity ID Control Selector is MUTE */
                        USBUART_currentTD.wCount = 1;
                        USBUART_currentTD.pData  = &USBUART_currentMute;
                        requestHandled   = USBUART_InitControlWrite();
                    }
                    else if(CY_GET_REG8(USBUART_wValueHi) == USBUART_VOLUME_CONTROL)
                    {
                         /* Entity ID Control Selector is VOLUME */
                        USBUART_currentTD.wCount = USBUART_VOLUME_LEN;
                        USBUART_currentTD.pData  = USBUART_currentVolume;
                        requestHandled   = USBUART_InitControlWrite();
                    }
                #endif /* End USBUART_ENABLE_AUDIO_STREAMING */

                /* `#START AUDIO_CONTROL_SEL_REQUESTS` Place other request handler here */

                /* `#END` */
                    break;
                default:
                    break;
            }
        }
        else
        {   /* USBUART_RQST_RCPT_OTHER */
        }
    }
    else
    {   /* requestHandled is initialezed as FALSE by default */
    }

    return(requestHandled);
}


/*******************************************************************************
* Additional user functions supporting AUDIO Requests
********************************************************************************/

/* `#START AUDIO_FUNCTIONS` Place any additional functions here */

/* `#END` */

#endif  /* End USBUART_ENABLE_AUDIO_CLASS*/


/* [] END OF FILE */
