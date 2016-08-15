 /*
  * @file    usbh_class_hid_custom.c
  * @author  Joe Gorse
  * @version V0.0.0
  * @date    August 11th, 2016
  * @brief   This file is the application layer for USB Host HID custom handling
  * @license 2-clause BSD. https://opensource.org/licenses/BSD-2-Clause
  */

#ifndef __USBH_CLASS_HID_CUSTOM_H__
#define __USBH_CLASS_HID_CUSTOM_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "usbh_hid.h"

USBH_StatusTypeDef USBH_HID_CustomInit(USBH_HandleTypeDef *phost);
uint8_t *USBH_HID_CustomData(USBH_HandleTypeDef *phost);

#ifdef __cplusplus
}
#endif

#endif /* __USBH_CLASS_HID_CUSTOM_H__ */
