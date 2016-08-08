/**
 ******************************************************************************
  * @file            : USB_HOST
  * @version         : v1.0_Cube
  * @brief           :  This file implements the USB Host 
  ******************************************************************************
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_hid.h"

/* USB Host Core handle declaration */
USBH_HandleTypeDef hUsbHostHS;
ApplicationTypeDef Appli_state = APPLICATION_IDLE;

/**
* -- Insert your variables declaration here --
*/ 
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
* user callbak declaration
*/ 
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id);

/**
* -- Insert your external function declaration here --
*/ 
/* USER CODE BEGIN 1 */
void USBH_HID_EventCallback(USBH_HandleTypeDef *phost);

void USBH_HID_EventCallback(USBH_HandleTypeDef *phost) {
//  HID_KEYBD_Info_TypeDef *kbdInfo;
//  kbdInfo = USBH_HID_GetKeybdInfo (phost);

//  printf ("Keyboard key: %c\n", USBH_HID_GetASCIICode(kbdInfo));
  HID_HandleTypeDef *HID_Handle =  (HID_HandleTypeDef *) phost->pActiveClass->pData;
  printf("%s:%d %s() should have data if we get here\n", (uint8_t *)__FILE__, __LINE__, __FUNCTION__);
  for (int i=0; i < HID_Handle->length; i++)
    printf ("%x ", HID_Handle->pData[i] ); // keybd_report_data[i]

  printf("\n");
}
/* USER CODE END 1 */

/* init function */				        
void MX_USB_HOST_Init(void)
{
  printf("%s:%d %s()\n", (uint8_t *)__FILE__, __LINE__, __FUNCTION__);
  /* Init Host Library,Add Supported Class and Start the library*/
  USBH_Init(&hUsbHostHS, USBH_UserProcess, HOST_HS);

  USBH_RegisterClass(&hUsbHostHS, USBH_HID_CLASS);

  USBH_Start(&hUsbHostHS);
}

/*
 * Background task
*/ 
void MX_USB_HOST_Process(void) 
{
//  printf("%s:%d %s()\n", (uint8_t *)__FILE__, __LINE__, __FUNCTION__);
  /* USB Host Background task */
  USBH_HandleTypeDef *phost = &hUsbHostHS;
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
  USBH_StatusTypeDef _status;
  USBH_URBStateTypeDef _urbstatus;
  uint8_t _buf[61];

//  phost->Control.setup.b.bRequest = USB_HID_SET_REPORT
  if (Appli_state == APPLICATION_READY && HID_Handle->state == HID_IDLE ) { //
//    HID_Handle->state = HID_SEND_DATA;
    memset (_buf, 0, sizeof(_buf));
    _buf[0] = 2;
//    _buf[1] = 1;

    _status = USBH_FAIL;
    while (_status != USBH_OK) { // 7 polls
      _status = USBH_InterruptSendData(phost,
                                      _buf,
                                      sizeof(_buf),
                                      HID_Handle->OutPipe);
//      _status = USBH_HID_SetReport (phost,
//                                    0, // Report type
//                                    2,
//                                    _buf,
//                                    sizeof(_buf));
      _urbstatus = USBH_LL_GetURBState( phost, HID_Handle->OutPipe);
      if ( _urbstatus == USBH_URB_DONE )
        break;
    }
    //printf ("Did we send it?\n"); // watch receiver

    _status = USBH_FAIL;
//    HID_Handle->state = HID_POLL;
//    HID_Handle->timer = phost->Timer;
//    HID_Handle->DataReady = 0;
    USBH_LL_SetToggle (phost, HID_Handle->InPipe, 0);
    while (_status != USBH_OK) { // 7 polls
    _status = USBH_InterruptReceiveData( phost,
                                      HID_Handle->pData,
                                      HID_Handle->length,
                                      HID_Handle->InPipe);
//    _status = USBH_HID_GetReport (&hUsbHostHS,
//                                  0, // Report type
//                                  1,
//                                  HID_Handle->pData,
//                                  HID_Handle->length);
    _urbstatus = USBH_LL_GetURBState( phost, HID_Handle->InPipe);
    if ( _urbstatus == USBH_URB_DONE )
            break;
    }
    HID_Handle->DataReady = 1;
    printf ("Did we get it?\n"); // buffer

  }

  USBH_Process(&hUsbHostHS);
}
/*
 * user callbak definition
*/ 
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id)
{
  printf("%s:%d %s()\n", (uint8_t *)__FILE__, __LINE__, __FUNCTION__);

  /* USER CODE BEGIN 2 */
  printf("id: %d ", id);
  switch(id)
  { 
  case HOST_USER_SELECT_CONFIGURATION:
    printf("HOST_USER_SELECT_CONFIGURATION\n");
  break;
    
  case HOST_USER_DISCONNECTION:
    Appli_state = APPLICATION_DISCONNECT;
    printf("HOST_USER_DISCONNECTION -> APPLICATION_DISCONNECT\n");
  break;
    
  case HOST_USER_CLASS_ACTIVE:
    Appli_state = APPLICATION_READY;
    printf("HOST_USER_CLASS_ACTIVE -> APPLICATION_READY\n");
  break;

  case HOST_USER_CLASS_SELECTED:
    //Appli_state = APPLICATION_READY;
    printf("HOST_USER_CLASS_SELECTED ...\n");
  break;

  case HOST_USER_CONNECTION:
    Appli_state = APPLICATION_START;
    printf("HOST_USER_CONNECTION -> APPLICATION_START\n");
  break;

  default:
    printf("Unhandled id in USBH_UserProcess() usb_host.c\n");
  break;
  }
  /* USER CODE END 2 */
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
