 /*
  * @file    usbh_class_hid_custom.c
  * @author  Joe Gorse
  * @version V0.0.0
  * @date    August 11th, 2016
  * @brief   This file is the application layer for USB Host HID custom handling
  * @license 2-clause BSD. https://opensource.org/licenses/BSD-2-Clause
  */ 

#include "usbh_class_hid_custom.h"
#include "usbh_hid_parser.h"

//HID_Custom_Info_TypeDef   custom_info;
uint8_t                   custom_report_data[61];

// Statically defined report descriptor
#define HID_REPORT_DESCRIPTOR_SIZE 0x0044
#define HID_REPORT_DESCRIPTOR_SIZE_LE 0x4400
typedef unsigned char hid_report_descriptor[HID_REPORT_DESCRIPTOR_SIZE];

const hid_report_descriptor HIDREPORTDESC = {
    0x06, 0x00, 0xff,              // USAGE_PAGE (Vendor Defined Page 1)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)

#define IN_CONTROL 0xFE
#define IN_CONTROL_SIZE 8

    0x85, IN_CONTROL,              // Report ID
    0x95, IN_CONTROL_SIZE,         //   REPORT_COUNT ()
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)

#define OUT_CONTROL 0xFD
#define OUT_CONTROL_SIZE 8

    0x85, OUT_CONTROL,             // Report ID
    0x95, OUT_CONTROL_SIZE,        //   REPORT_COUNT ()
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)

#define IN_DATA  0x01
#define IN_DATA_SIZE 60


    0x85, IN_DATA,                 // Report ID
    0x95, IN_DATA_SIZE,            //   REPORT_COUNT ()
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)

#define OUT_DATA 0x02
#define OUT_DATA_SIZE 60

    0x85, OUT_DATA,                // Report ID
    0x95, OUT_DATA_SIZE,           //   REPORT_COUNT ()
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)

    0xC0                           //   end Application Collection
};


/**
  * @brief  USBH_HID_CustomInit
  *         The function init the HID custom device.
  * @param  phost: Host handle
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_HID_CustomInit(USBH_HandleTypeDef *phost)
{
  HID_HandleTypeDef *HID_Handle =  (HID_HandleTypeDef *) phost->pActiveClass->pData;  
    
  // init report data
  memset (custom_report_data, 0, sizeof(custom_report_data));
  
  // select ep 1 (index 0) from here on out for
  //   interrupt IN
  HID_Handle->ep_addr   = phost->device.CfgDesc.Itf_Desc[phost->device.current_interface].Ep_Desc[0].bEndpointAddress;
  HID_Handle->length    = phost->device.CfgDesc.Itf_Desc[phost->device.current_interface].Ep_Desc[0].wMaxPacketSize;
  HID_Handle->poll      = phost->device.CfgDesc.Itf_Desc[phost->device.current_interface].Ep_Desc[0].bInterval ;

  // manually adjust HID_Handle settings as necessary
  // set min polling time in ms
  if (HID_Handle->poll < 110)
    HID_Handle->poll = 110;
  
  // set length in bytes
  if(HID_Handle->length >= (sizeof(custom_report_data)))
    HID_Handle->length = (sizeof(custom_report_data));

  // set HID->pData to report data
  HID_Handle->pData = (uint8_t*)custom_report_data;

  // attach fifo to phost->device.Data
  fifo_init(&HID_Handle->fifo, phost->device.Data, HID_QUEUE_SIZE * sizeof(custom_report_data));
  
  return USBH_OK;    
}

/**
  * @brief  USBH_HID_CustomData
  *         The function to get custom data.
  * @param  phost: Host handle
  * @retval pointer to data buffer
  */
uint8_t *USBH_HID_CustomData(USBH_HandleTypeDef *phost)
{
  HID_HandleTypeDef *HID_Handle =  (HID_HandleTypeDef *) phost->pActiveClass->pData;
  if(HID_Handle->length == 0)
  {
    return (uint8_t *)NULL;
  }
  /* Fill report */
  if(fifo_read(&HID_Handle->fifo, &custom_report_data, HID_Handle->length) ==  HID_Handle->length)
  {
//    keybd_info.rgui=(uint8_t)HID_ReadItem((HID_Report_ItemTypedef *) &imp_0_rgui, 0); // e.g. HID report parsing
    return (uint8_t *)custom_report_data;
  }
  return (uint8_t *)NULL;
}



