/*******************************************************************************
* File Name: USBUART_cls.c
* Version 2.12
*
* Description:
*  USB Class request handler.
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

#if(USBUART_EXTERN_CLS == USBUART_FALSE)


/***************************************
* External references
***************************************/

#if defined(USBUART_ENABLE_HID_CLASS)
    uint8 USBUART_DispatchHIDClassRqst(void);
#endif /* End USBUART_ENABLE_HID_CLASS */
#if defined(USBUART_ENABLE_AUDIO_CLASS)
    uint8 USBUART_DispatchAUDIOClassRqst(void);
#endif /* End USBUART_ENABLE_HID_CLASS */
#if defined(USBUART_ENABLE_CDC_CLASS)
    uint8 USBUART_DispatchCDCClassRqst(void);
#endif /* End USBUART_ENABLE_CDC_CLASS */

extern uint8 CYCODE *USBUART_interfaceClass;
extern volatile T_USBUART_EP_CTL_BLOCK USBUART_EP[];


/***************************************
* User Implemented Class Driver Declarations.
***************************************/
/* `#START USER_DEFINED_CLASS_DECLARATIONS` Place your declaration here */

/* `#END` */


/*******************************************************************************
* Function Name: USBUART_DispatchClassRqst
********************************************************************************
* Summary:
*  This routine dispatches class specific requests depend on inteface class.
*
* Parameters:
*  None.
*
* Return:
*  requestHandled.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 USBUART_DispatchClassRqst() 
{
    uint8 requestHandled = USBUART_FALSE;
    uint8 interfaceNumber = 0u;

    switch(CY_GET_REG8(USBUART_bmRequestType) & USBUART_RQST_RCPT_MASK)
    {
        case USBUART_RQST_RCPT_IFC:        /* class-specific request directed to an interface */
            interfaceNumber = CY_GET_REG8(USBUART_wIndexLo); /* wIndexLo contain Interface number */
            break;
        case USBUART_RQST_RCPT_EP:         /* class-specific request directed to the endpoint */
            /* find related intenface to the endpoint, wIndexLo contain EP number */
            interfaceNumber =
                USBUART_EP[CY_GET_REG8(USBUART_wIndexLo) & USBUART_DIR_UNUSED].interface;
            break;
        default:    /* requestHandled is initialized as FALSE by default */
            break;
    }
    /* Handle Class request depend on interface type */
    switch(USBUART_interfaceClass[interfaceNumber])
    {
        case USBUART_CLASS_HID:
            #if defined(USBUART_ENABLE_HID_CLASS)
                requestHandled = USBUART_DispatchHIDClassRqst();
            #endif /* USBUART_ENABLE_HID_CLASS */
            break;
        case USBUART_CLASS_AUDIO:
            #if defined(USBUART_ENABLE_AUDIO_CLASS)
                requestHandled = USBUART_DispatchAUDIOClassRqst();
            #endif /* USBUART_ENABLE_HID_CLASS */
            break;
        case USBUART_CLASS_CDC:
            #if defined(USBUART_ENABLE_CDC_CLASS)
                requestHandled = USBUART_DispatchCDCClassRqst();
            #endif /* USBUART_ENABLE_CDC_CLASS */
            break;
        default:    /* requestHandled is initialezed as FALSE by default */
            break;
    }

    /* `#START USER_DEFINED_CLASS_CODE` Place your Class request here */

    /* `#END` */

    return(requestHandled);
}


/*******************************************************************************
* Additional user functions supporting Class Specific Requests
********************************************************************************/

/* `#START CLASS_SPECIFIC_FUNCTIONS` Place any additional functions here */

/* `#END` */

#endif /* USBUART_EXTERN_CLS */


/* [] END OF FILE */
