/*******************************************************************************
* File Name: USBUART_hid.c
* Version 2.12
*
* Description:
*  USB HID Class request handler.
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

#if defined(USBUART_ENABLE_HID_CLASS)

#include "USBUART_hid.h"


/***************************************
*    HID Variables
***************************************/

volatile uint8 USBUART_hidProtocol[USBUART_MAX_INTERFACES_NUMBER];  /* HID device protocol status */
volatile uint8 USBUART_hidIdleRate[USBUART_MAX_INTERFACES_NUMBER];  /* HID device idle reload value */
volatile uint8 USBUART_hidIdleTimer[USBUART_MAX_INTERFACES_NUMBER]; /* HID device idle rate value */


/***************************************
* Custom Declarations
***************************************/

/* `#START HID_CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */


/***************************************
*    External references
***************************************/

uint8 USBUART_InitControlRead(void) ;
uint8 USBUART_InitControlWrite(void) ;
uint8 USBUART_InitNoDataControlTransfer(void) ;
T_USBUART_LUT *USBUART_GetConfigTablePtr(uint8 c)
                                                            ;
T_USBUART_LUT *USBUART_GetDeviceTablePtr(void)
                                                            ;

extern volatile T_USBUART_TD USBUART_currentTD;
extern uint8 CYCODE USBUART_HIDREPORT_DESCRIPTORS[];
extern volatile uint8 USBUART_configuration;
extern volatile uint8 USBUART_interfaceSetting[];


/***************************************
*    Internal references
***************************************/

void USBUART_FindReport(void) ;
void USBUART_FindReportDescriptor(void) ;
void USBUART_FindHidClassDecriptor(void) ;


/*******************************************************************************
* Function Name: USBUART_UpdateHIDTimer
********************************************************************************
*
* Summary:
*  Updates the HID report timer and reloads it if expired
*
* Parameters:
*  interface:  Interface Number.
*
* Return:
*  status.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 USBUART_UpdateHIDTimer(uint8 interface) 
{
    uint8 status = USBUART_IDLE_TIMER_INDEFINITE;

    if(USBUART_hidIdleRate[interface] != 0u)
    {
        if(USBUART_hidIdleTimer[interface] > 0u)
        {
            USBUART_hidIdleTimer[interface]--;
            status = USBUART_IDLE_TIMER_RUNNING;
        }
        else
        {
            USBUART_hidIdleTimer[interface] = USBUART_hidIdleRate[interface];
            status = USBUART_IDLE_TIMER_EXPIRED;
        }
    }

    return(status);
}


/*******************************************************************************
* Function Name: USBUART_GetProtocol
********************************************************************************
*
* Summary:
*  Returns the selected protocol value to the application
*
* Parameters:
*  interface:  Interface Number.
*
* Return:
*  Interface protocol.
*
*******************************************************************************/
uint8 USBUART_GetProtocol(uint8 interface) 
{
    return(USBUART_hidProtocol[interface]);
}


/*******************************************************************************
* Function Name: USBUART_DispatchHIDClassRqst
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
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 USBUART_DispatchHIDClassRqst() 
{
    uint8 requestHandled = USBUART_FALSE;
    uint8 interfaceNumber;

    interfaceNumber = CY_GET_REG8(USBUART_wIndexLo);
    if ((CY_GET_REG8(USBUART_bmRequestType) & USBUART_RQST_DIR_MASK) == USBUART_RQST_DIR_D2H)
    {   /* Control Read */
        switch (CY_GET_REG8(USBUART_bRequest))
        {
            case USBUART_GET_DESCRIPTOR:
                if (CY_GET_REG8(USBUART_wValueHi) == USBUART_DESCR_HID_CLASS)
                {
                    USBUART_FindHidClassDecriptor();
                    if (USBUART_currentTD.count != 0u)
                    {
                        requestHandled = USBUART_InitControlRead();
                    }
                }
                else if (CY_GET_REG8(USBUART_wValueHi) == USBUART_DESCR_HID_REPORT)
                {
                    USBUART_FindReportDescriptor();
                    if (USBUART_currentTD.count != 0u)
                    {
                        requestHandled = USBUART_InitControlRead();
                    }
                }
                else
                {   /* requestHandled is initialezed as FALSE by default */
                }
                break;
            case USBUART_HID_GET_REPORT:
                USBUART_FindReport();
                if (USBUART_currentTD.count != 0u)
                {
                    requestHandled = USBUART_InitControlRead();
                }
                break;

            case USBUART_HID_GET_IDLE:
                /* This function does not support multiple reports per interface*/
                /* Validate interfaceNumber and Report ID (should be 0) */
                if( (interfaceNumber < USBUART_MAX_INTERFACES_NUMBER) &&
                    (CY_GET_REG8(USBUART_wValueLo) == 0u ) ) /* do not support Idle per Report ID */
                {
                    USBUART_currentTD.count = 1u;
                    USBUART_currentTD.pData = &USBUART_hidIdleRate[interfaceNumber];
                    requestHandled  = USBUART_InitControlRead();
                }
                break;
            case USBUART_HID_GET_PROTOCOL:
                /* Validate interfaceNumber */
                if( interfaceNumber < USBUART_MAX_INTERFACES_NUMBER)
                {
                    USBUART_currentTD.count = 1u;
                    USBUART_currentTD.pData = &USBUART_hidProtocol[interfaceNumber];
                    requestHandled  = USBUART_InitControlRead();
                }
                break;
            default:    /* requestHandled is initialezed as FALSE by default */
                break;
        }
    }
    else if ((CY_GET_REG8(USBUART_bmRequestType) & USBUART_RQST_DIR_MASK) ==
                                                                            USBUART_RQST_DIR_H2D)
    {   /* Control Write */
        switch (CY_GET_REG8(USBUART_bRequest))
        {
            case USBUART_HID_SET_REPORT:
                USBUART_FindReport();
                if (USBUART_currentTD.count != 0u)
                {
                    requestHandled = USBUART_InitControlWrite();
                }
                break;
            case USBUART_HID_SET_IDLE:
                /* This function does not support multiple reports per interface */
                /* Validate interfaceNumber and Report ID (should be 0) */
                if( (interfaceNumber < USBUART_MAX_INTERFACES_NUMBER) &&
                    (CY_GET_REG8(USBUART_wValueLo) == 0u ) ) /* do not support Idle per Report ID */
                {
                    USBUART_hidIdleRate[interfaceNumber] = CY_GET_REG8(USBUART_wValueHi);
                    /* With regards to HID spec: "7.2.4 Set_Idle Request"
                    *  Latency. If the current period has gone past the
                    *  newly proscribed time duration, then a report
                    *  will be generated immediately.
                    */
                    if(USBUART_hidIdleRate[interfaceNumber] <
                       USBUART_hidIdleTimer[interfaceNumber])
                    {
                        /* Set the timer to zero and let the UpdateHIDTimer() API return IDLE_TIMER_EXPIRED status*/
                        USBUART_hidIdleTimer[interfaceNumber] = 0u;
                    }
                    /* If the new request is received within 4 milliseconds
                    *  (1 count) of the end of the current period, then the
                    *  new request will have no effect until after the report.
                    */
                    else if(USBUART_hidIdleTimer[interfaceNumber] <= 1u)
                    {
                        /* Do nothing.
                        * Let the UpdateHIDTimer() API continue to work and
                        * return IDLE_TIMER_EXPIRED status
                        */
                    }
                    else
                    {   /* Reload the timer*/
                        USBUART_hidIdleTimer[interfaceNumber] =
                        USBUART_hidIdleRate[interfaceNumber];
                    }
                    requestHandled = USBUART_InitNoDataControlTransfer();
                }
                break;

            case USBUART_HID_SET_PROTOCOL:
                /* Validate interfaceNumber and protocol (must be 0 or 1) */
                if( (interfaceNumber < USBUART_MAX_INTERFACES_NUMBER) &&
                    (CY_GET_REG8(USBUART_wValueLo) <= 1u) )
                {
                    USBUART_hidProtocol[interfaceNumber] = CY_GET_REG8(USBUART_wValueLo);
                    requestHandled = USBUART_InitNoDataControlTransfer();
                }
                break;
            default:    /* requestHandled is initialezed as FALSE by default */
                break;
        }
    }
    else
    {   /* requestHandled is initialezed as FALSE by default */
    }

    return(requestHandled);
}


/*******************************************************************************
* Function Name: USB_FindHidClassDescriptor
********************************************************************************
*
* Summary:
*  This routine find Hid Class Descriptor pointer based on the Interface number
*  and Alternate setting then loads the currentTD structure with the address of
*  the buffer and the size.
*  The HID Class Descriptor resides inside the config descriptor.
*
* Parameters:
*  None.
*
* Return:
*  currentTD
*
* Reentrant:
*  No.
*
*******************************************************************************/
void USBUART_FindHidClassDecriptor() 
{
    T_USBUART_LUT *pTmp;
    volatile uint8 *pDescr;
    uint8 interfaceN;

    pTmp = USBUART_GetConfigTablePtr(USBUART_configuration - 1u);
    interfaceN = CY_GET_REG8(USBUART_wIndexLo);
    /* Third entry in the LUT starts the Interface Table pointers */
    pTmp += 2;
    /* Now use the request interface number*/
    pTmp = &pTmp[interfaceN];
    /*USB_DEVICEx_CONFIGURATIONy_INTERFACEz_TABLE*/
    pTmp = (T_USBUART_LUT *) pTmp->p_list;
    /* Now use Alternate setting number */
    pTmp = &pTmp[USBUART_interfaceSetting[interfaceN]];
    /*USB_DEVICEx_CONFIGURATIONy_INTERFACEz_ALTERNATEi_HID_TABLE*/
    pTmp = (T_USBUART_LUT *) pTmp->p_list;
    /* Fifth entry in the LUT points to Hid Class Descriptor in Configuration Descriptor*/
    pTmp += 4;
    pDescr = (volatile uint8 *)pTmp->p_list;
    USBUART_currentTD.count = pDescr[0u];
    USBUART_currentTD.pData = pDescr;
}


/*******************************************************************************
* Function Name: USB_FindReportDescriptor
********************************************************************************
*
* Summary:
*  This routine find Hid Report Descriptor pointer based on the Interface
*  number, then loads the currentTD structure with the address of the buffer
*  and the size.
*  Hid Report Descriptor is located after IN/OUT/FEATURE reports.
*
* Parameters:
*   void
*
* Return:
*  currentTD
*
* Reentrant:
*  No.
*
*******************************************************************************/
void USBUART_FindReportDescriptor() 
{
    T_USBUART_LUT *pTmp;
    volatile uint8 *pDescr;
    uint8 interfaceN;

    pTmp = USBUART_GetConfigTablePtr(USBUART_configuration - 1u);
    interfaceN = CY_GET_REG8(USBUART_wIndexLo);
    /* Third entry in the LUT starts the Interface Table pointers */
    pTmp += 2u;
    /* Now use the request interface number*/
    pTmp = &pTmp[interfaceN];
    /*USB_DEVICEx_CONFIGURATIONy_INTERFACEz_TABLE*/
    pTmp = (T_USBUART_LUT *) pTmp->p_list;
    /* Now use Alternate setting number */
    pTmp = &pTmp[USBUART_interfaceSetting[interfaceN]];
    /*USB_DEVICEx_CONFIGURATIONy_INTERFACEz_ALTERNATEi_HID_TABLE*/
    pTmp = (T_USBUART_LUT *) pTmp->p_list;
    /* Fourth entry in the LUT starts the Hid Report Descriptor */
    pTmp += 3;
    pDescr = (volatile uint8 *)pTmp->p_list;
    USBUART_currentTD.count =  (((uint16)pDescr[1] << 8u) | pDescr[0u]);
    USBUART_currentTD.pData = &pDescr[2u];
}


/*******************************************************************************
* Function Name: USBUART_FindReport
********************************************************************************
*
* Summary:
*  This routine sets up a transfer based on the Interface number, Report Type
*  and Report ID, then loads the currentTD structure with the address of the
*  buffer and the size.  The caller has to decide if it is a control read or
*  control write.
*
* Parameters:
*  None.
*
* Return:
*  currentTD
*
* Reentrant:
*  No.
*
*******************************************************************************/
void USBUART_FindReport() 
{
    T_USBUART_LUT *pTmp;
    T_USBUART_TD *pTD;
    uint8 interfaceN;
    uint8 reportType;

    /* `#START HID_FINDREPORT` Place custom handling here */

    /* `#END` */
    USBUART_currentTD.count = 0u;   /* Init not supported condition */
    pTmp = USBUART_GetConfigTablePtr(USBUART_configuration - 1u);
    reportType = CY_GET_REG8(USBUART_wValueHi);
    interfaceN = CY_GET_REG8(USBUART_wIndexLo);
    /* Third entry in the LUT COnfiguration Table starts the Interface Table pointers */
    pTmp += 2;
    /* Now use the request interface number */
    pTmp = &pTmp[interfaceN];
    /*USB_DEVICEx_CONFIGURATIONy_INTERFACEz_TABLE*/
    pTmp = (T_USBUART_LUT *) pTmp->p_list;
    if(interfaceN < USBUART_MAX_INTERFACES_NUMBER)
    {
        /* Now use Alternate setting number */
        pTmp = &pTmp[USBUART_interfaceSetting[interfaceN]];
        /*USB_DEVICEx_CONFIGURATIONy_INTERFACEz_ALTERNATEi_HID_TABLE*/
        pTmp = (T_USBUART_LUT *) pTmp->p_list;
        /* Validate reportType to comply with "7.2.1 Get_Report Request" */
        if((reportType >= USBUART_HID_GET_REPORT_INPUT) &&
           (reportType <= USBUART_HID_GET_REPORT_FEATURE))
        {
            /* Get the entry proper TD (IN, OUT or Feature Report Table)*/
            pTmp = &pTmp[reportType - 1u];
            reportType = CY_GET_REG8(USBUART_wValueLo);    /* Get reportID */
            /* Validate table support by the HID descriptor, compare table count with reportID */
            if(pTmp->c > reportType)
            {
                pTD = (T_USBUART_TD *) pTmp->p_list;
                pTD = &pTD[reportType];                          /* select entry depend on report ID*/
                USBUART_currentTD.pData = pTD->pData;   /* Bufer pointer */
                USBUART_currentTD.count = pTD->count;   /* Bufer Size */
                USBUART_currentTD.pStatusBlock = pTD->pStatusBlock;
            }
        }
    }
}


/*******************************************************************************
* Additional user functions supporting HID Requests
********************************************************************************/

/* `#START HID_FUNCTIONS` Place any additional functions here */

/* `#END` */

#endif  /* End USBUART_ENABLE_HID_CLASS*/


/* [] END OF FILE */
