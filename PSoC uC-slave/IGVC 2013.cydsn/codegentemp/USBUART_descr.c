/*******************************************************************************
* File Name: USBUART_descr.c
* Version 2.12
*
* Description:
*  USB descriptors and storage.
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


/*****************************************************************************
*  User supplied descriptors.  If you want to specify your own descriptors,
*  remove the comments around the define USER_SUPPLIED_DESCRIPTORS below and
*  add your descriptors.
*****************************************************************************/
/* `#START USER_DESCRIPTORS_DECLARATIONS` Place your declaration here */

/* `#END` */


/***************************************
*  USB Customizer Generated Descriptors
***************************************/

#if !defined(USER_SUPPLIED_DESCRIPTORS)
/*********************************************************************
* Device Descriptors
*********************************************************************/
const uint8 CYCODE USBUART_DEVICE0_DESCR[] = {
/* Descriptor Length                       */ 0x12u,
/* DescriptorType: DEVICE                  */ 0x01u,
/* bcdUSB (ver 2.0)                        */ 0x00u, 0x02u,
/* bDeviceClass                            */ 0x02u,
/* bDeviceSubClass                         */ 0x00u,
/* bDeviceProtocol                         */ 0x00u,
/* bMaxPacketSize0                         */ 0x08u,
/* idVendor                                */ 0xB4u, 0x04u,
/* idProduct                               */ 0x32u, 0xF2u,
/* bcdDevice                               */ 0x01u, 0x00u,
/* iManufacturer                           */ 0x01u,
/* iProduct                                */ 0x05u,
/* iSerialNumber                           */ 0x80u,
/* bNumConfigurations                      */ 0x01u
};
/*********************************************************************
* Config Descriptor  
*********************************************************************/
const uint8 CYCODE USBUART_DEVICE0_CONFIGURATION0_DESCR[] = {
/*  Config Descriptor Length               */ 0x09u,
/*  DescriptorType: CONFIG                 */ 0x02u,
/*  wTotalLength                           */ 0x43u, 0x00u,
/*  bNumInterfaces                         */ 0x02u,
/*  bConfigurationValue                    */ 0x01u,
/*  iConfiguration                         */ 0x01u,
/*  bmAttributes                           */ 0xC0u,
/*  bMaxPower                              */ 0x32u,
/*********************************************************************
* CDC Interface Descriptor
*********************************************************************/
/*  Interface Descriptor Length            */ 0x09u,
/*  DescriptorType: INTERFACE              */ 0x04u,
/*  bInterfaceNumber                       */ 0x00u,
/*  bAlternateSetting                      */ 0x00u,
/*  bNumEndpoints                          */ 0x01u,
/*  bInterfaceClass                        */ 0x02u,
/*  bInterfaceSubClass                     */ 0x02u,
/*  bInterfaceProtocol                     */ 0x01u,
/*  iInterface                             */ 0x03u,
/*********************************************************************
* Header Descriptor
*********************************************************************/
/*  Header Descriptor Length               */ 0x05u,
/*  DescriptorType: CS_INTERFACE           */ 0x24u,
/*  bDescriptorSubtype                     */ 0x00u,
/*  bcdADC                                 */ 0x10u, 0x01u,
/*********************************************************************
* Abstract Control Management Descriptor
*********************************************************************/
/*  Abstract Control Management Descriptor Length*/ 0x04u,
/*  DescriptorType: CS_INTERFACE           */ 0x24u,
/*  bDescriptorSubtype                     */ 0x02u,
/*  bmCapabilities                         */ 0x02u,
/*********************************************************************
* Union Descriptor
*********************************************************************/
/*  Union Descriptor Length                */ 0x05u,
/*  DescriptorType: CS_INTERFACE           */ 0x24u,
/*  bDescriptorSubtype                     */ 0x06u,
/*  bControlInterface                      */ 0x00u,
/*  bSubordinateInterface                  */ 0x01u,
/*********************************************************************
* Call Management Descriptor
*********************************************************************/
/*  Call Management Descriptor Length      */ 0x05u,
/*  DescriptorType: CS_INTERFACE           */ 0x24u,
/*  bDescriptorSubtype                     */ 0x01u,
/*  bmCapabilities                         */ 0x00u,
/*  bDataInterface                         */ 0x01u,
/*********************************************************************
* Endpoint Descriptor
*********************************************************************/
/*  Endpoint Descriptor Length             */ 0x07u,
/*  DescriptorType: ENDPOINT               */ 0x05u,
/*  bEndpointAddress                       */ 0x81u,
/*  bmAttributes                           */ 0x03u,
/*  wMaxPacketSize                         */ 0x08u, 0x00u,
/*  bInterval                              */ 0x0Au,
/*********************************************************************
* Data Interface Descriptor
*********************************************************************/
/*  Interface Descriptor Length            */ 0x09u,
/*  DescriptorType: INTERFACE              */ 0x04u,
/*  bInterfaceNumber                       */ 0x01u,
/*  bAlternateSetting                      */ 0x00u,
/*  bNumEndpoints                          */ 0x02u,
/*  bInterfaceClass                        */ 0x0Au,
/*  bInterfaceSubClass                     */ 0x00u,
/*  bInterfaceProtocol                     */ 0x00u,
/*  iInterface                             */ 0x04u,
/*********************************************************************
* Endpoint Descriptor
*********************************************************************/
/*  Endpoint Descriptor Length             */ 0x07u,
/*  DescriptorType: ENDPOINT               */ 0x05u,
/*  bEndpointAddress                       */ 0x82u,
/*  bmAttributes                           */ 0x02u,
/*  wMaxPacketSize                         */ 0x40u, 0x00u,
/*  bInterval                              */ 0x0Au,
/*********************************************************************
* Endpoint Descriptor
*********************************************************************/
/*  Endpoint Descriptor Length             */ 0x07u,
/*  DescriptorType: ENDPOINT               */ 0x05u,
/*  bEndpointAddress                       */ 0x03u,
/*  bmAttributes                           */ 0x02u,
/*  wMaxPacketSize                         */ 0x40u, 0x00u,
/*  bInterval                              */ 0x0Au
};

/*********************************************************************
* String Descriptor Table
*********************************************************************/
const uint8 CYCODE USBUART_STRING_DESCRIPTORS[] = {
/*********************************************************************
* Language ID Descriptor
*********************************************************************/
/* Descriptor Length                       */ 0x04u,
/* DescriptorType: STRING                  */ 0x03u,
/* Language Id                             */ 0x09u, 0x04u,
/*********************************************************************
* String Descriptor: "Cypress Semiconductor"
*********************************************************************/
/* Descriptor Length                       */ 0x2Cu,
/* DescriptorType: STRING                  */ 0x03u,
 'C', 0,'y', 0,'p', 0,'r', 0,'e', 0,'s', 0,'s', 0,' ', 0,'S', 0,'e', 0
,'m', 0,'i', 0,'c', 0,'o', 0,'n', 0,'d', 0,'u', 0,'c', 0,'t', 0,'o', 0
,'r', 0,
/*********************************************************************
* String Descriptor: "USBUART"
*********************************************************************/
/* Descriptor Length                       */ 0x10u,
/* DescriptorType: STRING                  */ 0x03u,
 'U', 0,'S', 0,'B', 0,'U', 0,'A', 0,'R', 0,'T', 0,
/*********************************************************************
* String Descriptor: "CDC Communication Interface"
*********************************************************************/
/* Descriptor Length                       */ 0x38u,
/* DescriptorType: STRING                  */ 0x03u,
 'C', 0,'D', 0,'C', 0,' ', 0,'C', 0,'o', 0,'m', 0,'m', 0,'u', 0,'n', 0
,'i', 0,'c', 0,'a', 0,'t', 0,'i', 0,'o', 0,'n', 0,' ', 0,'I', 0,'n', 0
,'t', 0,'e', 0,'r', 0,'f', 0,'a', 0,'c', 0,'e', 0,
/*********************************************************************
* String Descriptor: "CDC Data Interface"
*********************************************************************/
/* Descriptor Length                       */ 0x26u,
/* DescriptorType: STRING                  */ 0x03u,
 'C', 0,'D', 0,'C', 0,' ', 0,'D', 0,'a', 0,'t', 0,'a', 0,' ', 0,'I', 0
,'n', 0,'t', 0,'e', 0,'r', 0,'f', 0,'a', 0,'c', 0,'e', 0,
/*********************************************************************
* String Descriptor: "Doloras IGVC 2013 UT IEEE-RAS"
*********************************************************************/
/* Descriptor Length                       */ 0x3Cu,
/* DescriptorType: STRING                  */ 0x03u,
 'D', 0,'o', 0,'l', 0,'o', 0,'r', 0,'a', 0,'s', 0,' ', 0,'I', 0,'G', 0
,'V', 0,'C', 0,' ', 0,'2', 0,'0', 0,'1', 0,'3', 0,' ', 0,'U', 0,'T', 0
,' ', 0,'I', 0,'E', 0,'E', 0,'E', 0,'-', 0,'R', 0,'A', 0,'S', 0,
/*********************************************************************/
/* Marks the end of the list.              */ 0x00u};
/*********************************************************************/

/*********************************************************************
* Serial Number String Descriptor
*********************************************************************/
const uint8 CYCODE USBUART_SN_STRING_DESCRIPTOR[] = {
/* Descriptor Length                       */ 0x10u,
/* DescriptorType: STRING                  */ 0x03u,
'D', 0,'o', 0,'l', 0,'o', 0,'r', 0,'a', 0,'s', 0
};



/*********************************************************************
* Endpoint Setting Table -- This table contain the endpoint setting
*                           for each endpoint in the configuration. It
*                           contains the necessary information to
*                           configure the endpoint hardware for each
*                           interface and alternate setting.
*********************************************************************/
const T_USBUART_EP_SETTINGS_BLOCK CYCODE USBUART_DEVICE0_CONFIGURATION0_EP_SETTINGS_TABLE[] = {
/* IFC  ALT    EPAddr bmAttr MaxPktSize Class ********************/
{0x00u, 0x00u, 0x81u, 0x03u, 0x0008u,   0x02u},
{0x01u, 0x00u, 0x82u, 0x02u, 0x0040u,   0x0Au},
{0x01u, 0x00u, 0x03u, 0x02u, 0x0040u,   0x0Au}
};
const uint8 CYCODE USBUART_DEVICE0_CONFIGURATION0_INTERFACE_CLASS[] = {
0x02u, 0x0Au
};
/*********************************************************************
* Config Dispatch Table -- Points to the Config Descriptor and each of
*                          and endpoint setup table and to each
*                          interface table if it specifies a USB Class
*********************************************************************/
const T_USBUART_LUT CYCODE USBUART_DEVICE0_CONFIGURATION0_TABLE[] = {
    {0x01u,     &USBUART_DEVICE0_CONFIGURATION0_DESCR},
    {0x03u,     &USBUART_DEVICE0_CONFIGURATION0_EP_SETTINGS_TABLE},
    {0x00u,    NULL},
    {0x00u,    NULL},
    {0x00u,     &USBUART_DEVICE0_CONFIGURATION0_INTERFACE_CLASS}
};
/*********************************************************************
* Device Dispatch Table -- Points to the Device Descriptor and each of
*                          and Configuration Tables for this Device 
*********************************************************************/
const T_USBUART_LUT CYCODE USBUART_DEVICE0_TABLE[] = {
    {0x01u,     &USBUART_DEVICE0_DESCR},
    {0x01u,     &USBUART_DEVICE0_CONFIGURATION0_TABLE}
};
/*********************************************************************
* Device Table -- Indexed by the device number.
*********************************************************************/
const T_USBUART_LUT CYCODE USBUART_TABLE[] = {
    {0x01u,     &USBUART_DEVICE0_TABLE}
};

#endif /* USER_SUPPLIED_DESCRIPTORS */

#if defined(USBUART_ENABLE_MSOS_STRING)

    /******************************************************************************
    *  USB Microsoft OS String Descriptor
    *  "MSFT" identifies a Microsoft host
    *  "100" specifies version 1.00
    *  USBUART_GET_EXTENDED_CONFIG_DESCRIPTOR becomes the bRequest value
    *  in a host vendor device/class request
    ******************************************************************************/

    const uint8 CYCODE USBUART_MSOS_DESCRIPTOR[] = {
    /* Descriptor Length                       */   0x12u,
    /* DescriptorType: STRING                  */   0x03u,
    /* qwSignature */                               'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0,
    /* bMS_VendorCode:                         */   USBUART_GET_EXTENDED_CONFIG_DESCRIPTOR,
    /* bPad                                    */   0x00u
    };

    /* Extended Configuration Descriptor */

    const uint8 CYCODE USBUART_MSOS_CONFIGURATION_DESCR[] = {
    /*  Length of the descriptor 4 bytes       */   0x28u, 0x00u, 0x00u, 0x00u,
    /*  Version of the descriptor 2 bytes      */   0x00u, 0x01u,
    /*  wIndex - Fixed:INDEX_CONFIG_DESCRIPTOR */   0x04u, 0x00u,
    /*  bCount - Count of device functions.    */   0x01u,
    /*  Reserved : 7 bytes                     */   0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    /*  bFirstInterfaceNumber                  */   0x00u,
    /*  Reserved                               */   0x01u,
    /*  compatibleID    - "CYUSB\0\0"          */   'C', 'Y', 'U', 'S', 'B', 0x00, 0x00u, 0x00u,
    /*  subcompatibleID - "00001\0\0"          */   '0', '0', '0', '0', '1', 0x00, 0x00u, 0x00u,
    /*  Reserved : 6 bytes                     */   0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u
    };

#endif /* USBUART_ENABLE_MSOS_STRING */


/* [] END OF FILE */
