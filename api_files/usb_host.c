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

// Called by USBH_HID_Process()
void USBH_HID_EventCallback(USBH_HandleTypeDef *phost) {
  HID_HandleTypeDef *HID_Handle =  (HID_HandleTypeDef *) phost->pActiveClass->pData;
  static uint8_t _ecbuf[64];
  uint16_t nbytes = 0;

  HID_Handle->DataReady = 0;
  if(HID_Handle->length == 0)
  {
    printf("%s:%d %s() HID_Handle->length == 0\n", (uint8_t *)__FILE__, __LINE__, __FUNCTION__);
    return; // Failed
  }

  nbytes = fifo_read(&HID_Handle->fifo, &_ecbuf, HID_Handle->length);
  while (nbytes !=  0) {
    printf("%s:%d %s() dump %d bytes fifo data\n", (uint8_t *)__FILE__, __LINE__, __FUNCTION__,nbytes);
    for (int i=0; i < nbytes; i++)
      printf ("%x ", _ecbuf[i] );
    printf("\n");
    nbytes = fifo_read(&HID_Handle->fifo, _ecbuf, HID_Handle->length);
  }

  // or use unint8_t USBH_HID_GetASCIICode(USBH_HID_GetKeybdInfo(phost))
//  printf("%s() keycode %c\n",__FUNCTION__,
//      USBH_HID_GetASCIICode(USBH_HID_GetKeybdInfo(phost)));
}
/* USER CODE END 1 */

/* init function */				        
void MX_USB_HOST_Init(void)
{
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
  /* USB Host Background task */
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