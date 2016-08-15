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
#include "usbh_conf.h"
#include "usbh_hid.h"
#include "mxconstants.h"

/* USB Host Core handle declaration */
USBH_HandleTypeDef hUsbHostHS;
struct USBHAppData usbh_data;

/**
* -- Insert your variables declaration here --
*/ 
/* USER CODE BEGIN 0 */
TIM_HandleTypeDef usbh_timer_handle;

/* USER CODE END 0 */

/*
* user callback declaration
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
  char DevID[10];
  DevID[9] = '\0';

  HID_Handle->DataReady = 0;
  if(HID_Handle->length == 0)
  {
    USBH_DbgLog("%s:%d %s() HID_Handle->length == 0", (uint8_t *)__FILE__, __LINE__, __FUNCTION__);
    return; // Failed
  }

  nbytes = fifo_read(&HID_Handle->fifo, &_ecbuf, HID_Handle->length);
  while (nbytes !=  0) {
//    USBH_DbgLog("%s:%d %s() dump %d bytes fifo data", (uint8_t *)__FILE__, __LINE__, __FUNCTION__,nbytes);
    usbh_data.ReportType = _ecbuf[1];
    usbh_data.SensorType = _ecbuf[2];
		if (usbh_data.ReportType == 1) { // Page 1 of 3 of report data
			usbh_data.RawTemp 			 = _ecbuf[3] * 0x100 + _ecbuf[4];
			usbh_data.RawIrradianceA = _ecbuf[5] * 0x100 + _ecbuf[6];
			usbh_data.RawIrradianceB = _ecbuf[7] * 0x100 + _ecbuf[8];
			usbh_data.RawADC				 = _ecbuf[9] * 0x100 + _ecbuf[10];
			// 11 12 FWVersion
			usbh_data.FWVersion = _ecbuf[11] * 0x100 + _ecbuf[12];
			// 13-21 DevID
			for (int i=13; i < 22; i++) {
			  usbh_data.DevID[i-13] = _ecbuf[i];
//				printf ("%c", _ecbuf[i] );
			}
			DevID[9] = '\0';
			printf (" RawTemp %03x RawIrradianceA %04x  RawIrradianceB %04x  RawADC %04x  DevID %s \n",
					(int)usbh_data.RawTemp, (int)usbh_data.RawIrradianceA, (int)usbh_data.RawIrradianceB, (int)usbh_data.RawADC, (char *)usbh_data.DevID);
			for (int i=0; i < 13; i++) {
				printf ("%x ", _ecbuf[i] );
			}
			printf ("\n");
    }
    nbytes = fifo_read(&HID_Handle->fifo, _ecbuf, HID_Handle->length);
  }
}

static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_FS_ID_GPIO_Port, USB_FS_ID_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_HOST_DEV_GPIO_Port, USB_HOST_DEV_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : USB_FS_ID_Pin USB_HOST_DEV_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin|USB_HOST_DEV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

static void MX_Timer_Init(void)
{
  __TIM4_CLK_ENABLE();

  /* Set the TIMx priority */
  HAL_NVIC_SetPriority(TIM4_IRQn, 10, 1); // 0-15 priority, lower number is higher priority

  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIM4_IRQn);

  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  uint32_t uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;

  /* Set TIMx instance */
  usbh_timer_handle.Instance = TIM4;

  /* Initialize TIMx peripheral as follows:
       + Period = 5 - 1 @ 10 kHz => 0.5 ms
       + Prescaler = ((SystemCoreClock/2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Down
  */
  usbh_timer_handle.Init.Period = 4;
  usbh_timer_handle.Init.Prescaler = uwPrescalerValue;
  usbh_timer_handle.Init.ClockDivision = 0;
  usbh_timer_handle.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  if(HAL_TIM_Base_Init(&usbh_timer_handle) != HAL_OK) { assert_param(0); }
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&usbh_timer_handle) != HAL_OK) { assert_param(0); }

}

/* USER CODE END 1 */

/* init function */				        
void MX_USB_HOST_Init(void)
{
	MX_GPIO_Init();
//	MX_Timer_Init();
  USBH_DbgLog("CubeMX Host USB for HID\n");

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
  USBH_DbgLog("%s:%d %s()\n", (uint8_t *)__FILE__, __LINE__, __FUNCTION__);

  /* USER CODE BEGIN 2 */
  USBH_DbgLog("id: %d ", id);
  switch(id)
  { 
  case HOST_USER_SELECT_CONFIGURATION:
    USBH_DbgLog("HOST_USER_SELECT_CONFIGURATION\n");
  break;
    
  case HOST_USER_DISCONNECTION:
    usbh_data.state = USBH_DISCONNECT;
    USBH_DbgLog("HOST_USER_DISCONNECTION -> USBH_DISCONNECT\n");
  break;
    
  case HOST_USER_CLASS_ACTIVE:
    usbh_data.state = USBH_READY;
    USBH_DbgLog("HOST_USER_CLASS_ACTIVE -> USBH_READY\n");
  break;

  case HOST_USER_CLASS_SELECTED:
    USBH_DbgLog("HOST_USER_CLASS_SELECTED ...\n");
  break;

  case HOST_USER_CONNECTION:
    usbh_data.state = USBH_START;
    USBH_DbgLog("HOST_USER_CONNECTION -> USBH_START\n");
  break;

  default:
    USBH_DbgLog("Unhandled id in USBH_UserProcess() usb_host.c\n");
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
