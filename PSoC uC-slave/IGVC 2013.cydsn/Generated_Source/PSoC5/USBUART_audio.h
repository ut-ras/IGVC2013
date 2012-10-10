/*******************************************************************************
* File Name: USBUART_audio.h
* Version 2.12
*
* Description:
*  Header File for the USFS component. Contains prototypes and constant values.
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(USBUART_audio_H)
#define USBUART_audio_H

#include "cytypes.h"


/***************************************
*  Constants for $INSTANCE_NAME` API.
***************************************/

/* Audio Class-Specific Request Codes (AUDIO Table A-9) */
#define USBUART_REQUEST_CODE_UNDEFINED     (0x00u)
#define USBUART_SET_CUR                    (0x01u)
#define USBUART_GET_CUR                    (0x81u)
#define USBUART_SET_MIN                    (0x02u)
#define USBUART_GET_MIN                    (0x82u)
#define USBUART_SET_MAX                    (0x03u)
#define USBUART_GET_MAX                    (0x83u)
#define USBUART_SET_RES                    (0x04u)
#define USBUART_GET_RES                    (0x84u)
#define USBUART_SET_MEM                    (0x05u)
#define USBUART_GET_MEM                    (0x85u)
#define USBUART_GET_STAT                   (0xFFu)

/* Endpoint Control Selectors (AUDIO Table A-19) */
#define USBUART_EP_CONTROL_UNDEFINED       (0x00u)
#define USBUART_SAMPLING_FREQ_CONTROL      (0x01u)
#define USBUART_PITCH_CONTROL              (0x02u)

/* Feature Unit Control Selectors (AUDIO Table A-11) */
#define USBUART_FU_CONTROL_UNDEFINED       (0x00u)
#define USBUART_MUTE_CONTROL               (0x01u)
#define USBUART_VOLUME_CONTROL             (0x02u)
#define USBUART_BASS_CONTROL               (0x03u)
#define USBUART_MID_CONTROL                (0x04u)
#define USBUART_TREBLE_CONTROL             (0x05u)
#define USBUART_GRAPHIC_EQUALIZER_CONTROL  (0x06u)
#define USBUART_AUTOMATIC_GAIN_CONTROL     (0x07u)
#define USBUART_DELAY_CONTROL              (0x08u)
#define USBUART_BASS_BOOST_CONTROL         (0x09u)
#define USBUART_LOUDNESS_CONTROL           (0x0Au)

#define USBUART_SAMPLE_FREQ_LEN            (3u)
#define USBUART_VOLUME_LEN                 (2u)

#endif /* End USBUART_audio_H */


/* [] END OF FILE */
