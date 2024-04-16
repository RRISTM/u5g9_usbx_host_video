/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_usbx_host.c
  * @author  MCD Application Team
  * @brief   USBX host applicative file
  ******************************************************************************
    * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_usbx_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN UX_Host_Memory_Buffer */

/* USER CODE END UX_Host_Memory_Buffer */
#if defined ( __ICCARM__ )
#pragma data_alignment=4
#endif
__ALIGN_BEGIN static UCHAR ux_host_byte_pool_buffer[UX_HOST_APP_MEM_POOL_SIZE] __ALIGN_END;
/* USER CODE BEGIN PV */
extern HCD_HandleTypeDef hhcd_USB_OTG_HS;
UX_HOST_CLASS_VIDEO *video=NULL;
UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST video_transfer_request;
uint32_t videoReadDone=0;
uint32_t videoConnected=0;
uint32_t videoLine=0;
uint32_t frameTimestamp=0;
uint8_t buffer[2048];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static UINT ux_host_event_callback(ULONG event, UX_HOST_CLASS *current_class, VOID *current_instance);
static VOID ux_host_error_callback(UINT system_level, UINT system_context, UINT error_code);
/* USER CODE BEGIN PFP */
void video_transfer_complete(UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST* video_transfer);
uint32_t yuvToRgb(uint8_t* in,uint8_t* out,uint32_t length);
void oneYuvToRgbConversion(int8_t y,int8_t u,int8_t v,uint8_t* rOut,uint8_t* gOut,uint8_t* bOut);
/* USER CODE END PFP */

/**
  * @brief  Application USBX Host Initialization.
  * @param  none
  * @retval status
  */
UINT MX_USBX_Host_Init(VOID)
{
  UINT ret = UX_SUCCESS;
  UCHAR *pointer;

  /* USER CODE BEGIN MX_USBX_Host_Init0 */

  /* USER CODE END MX_USBX_Host_Init0 */

  pointer = ux_host_byte_pool_buffer;

  /* Initialize USBX Memory */
  if (ux_system_initialize(pointer, USBX_HOST_MEMORY_STACK_SIZE, UX_NULL, 0) != UX_SUCCESS)
  {
    /* USER CODE BEGIN USBX_SYSTEM_INITIALIZE_ERROR */
    return UX_ERROR;
    /* USER CODE END USBX_SYSTEM_INITIALIZE_ERROR */
  }

  /* Install the host portion of USBX */
  if (ux_host_stack_initialize(ux_host_event_callback) != UX_SUCCESS)
  {
    /* USER CODE BEGIN USBX_HOST_INITIALIZE_ERROR */
    return UX_ERROR;
    /* USER CODE END USBX_HOST_INITIALIZE_ERROR */
  }

  /* Register a callback error function */
  ux_utility_error_callback_register(&ux_host_error_callback);

  /* USER CODE BEGIN MX_USBX_Host_Init1 */
  /* Initialize the host cdc acm class */
  if ((ux_host_stack_class_register(_ux_system_host_class_video_name, ux_host_class_video_entry)) != UX_SUCCESS)
  {
    /* USER CODE BEGIN USBX_HOST_CDC_ACM_REGISTER_ERROR */
    return UX_ERROR;
    /* USER CODE END USBX_HOST_CDC_ACM_REGISTER_ERROR */
  }
  /* Initialize the host controller driver */
  ux_host_stack_hcd_register(_ux_system_host_hcd_stm32_name,  _ux_hcd_stm32_initialize, (ULONG)USB_OTG_HS,(ULONG)&hhcd_USB_OTG_HS);

  HAL_HCD_Start(&hhcd_USB_OTG_HS);
  /* USER CODE END MX_USBX_Host_Init1 */

  return ret;
}

/**
  * @brief  _ux_utility_interrupt_disable
  *         USB utility interrupt disable.
  * @param  none
  * @retval none
  */
ALIGN_TYPE _ux_utility_interrupt_disable(VOID)
{
  /* USER CODE BEGIN _ux_utility_interrupt_disable */
  __set_PRIMASK(1);
  return(0);
  /* USER CODE END _ux_utility_interrupt_disable */
}

/**
  * @brief  _ux_utility_interrupt_restore
  *         USB utility interrupt restore.
  * @param  flags
  * @retval none
  */
VOID _ux_utility_interrupt_restore(ALIGN_TYPE flags)
{
  /* USER CODE BEGIN _ux_utility_interrupt_restore */
  __set_PRIMASK(0);
  UX_PARAMETER_NOT_USED(flags);
  /* USER CODE END _ux_utility_interrupt_restore */
}

/**
  * @brief  _ux_utility_time_get
  *         Get Time Tick for host timing.
  * @param  none
  * @retval time tick
  */
ULONG _ux_utility_time_get(VOID)
{
  ULONG time_tick = 0U;

  /* USER CODE BEGIN _ux_utility_time_get */
  time_tick = HAL_GetTick();
  /* USER CODE END _ux_utility_time_get */

  return time_tick;
}

/**
  * @brief  ux_host_event_callback
  *         This callback is invoked to notify application of instance changes.
  * @param  event: event code.
  * @param  current_class: Pointer to class.
  * @param  current_instance: Pointer to class instance.
  * @retval status
  */
UINT ux_host_event_callback(ULONG event, UX_HOST_CLASS *current_class, VOID *current_instance)
{
  UINT status = UX_SUCCESS;

  /* USER CODE BEGIN ux_host_event_callback0 */
  UX_PARAMETER_NOT_USED(current_class);
  UX_PARAMETER_NOT_USED(current_instance);

  // video_transfer_request.ux_host_class_video_transfer_request_data_pointer=0x300D0000;//data buffer
  // video_transfer_request.ux_host_class_video_transfer_request_requested_length= 320*240*2;//640*480*2
  // video_transfer_request.ux_host_class_video_transfer_request_completion_function=&video_transfer_complete;

  /* USER CODE END ux_host_event_callback0 */

  switch (event)
  {
    case UX_DEVICE_INSERTION:

      /* USER CODE BEGIN UX_DEVICE_INSERTION */
      if (current_class -> ux_host_class_entry_function == ux_host_class_video_entry)
      {
        if (video == UX_NULL)
        {
          /* Get current video Instance */
          video = (UX_HOST_CLASS_VIDEO *)current_instance;
          ux_host_class_video_frame_parameters_set(video,0x4,320,240,2000000);// 0x4 --VS_UNCOMPRESSED
          ux_host_class_video_start(video);
          videoLine=0;
          videoConnected=1;
          videoReadDone=1;
        }
      }
      /* USER CODE END UX_DEVICE_INSERTION */

      break;

    case UX_DEVICE_REMOVAL:

      /* USER CODE BEGIN UX_DEVICE_REMOVAL */
      if (current_class -> ux_host_class_entry_function == ux_host_class_video_entry)
      {
          videoLine=0;
          videoConnected=0;
          videoReadDone=0;
          /* Get current video Instance */
          video = NULL;
        
      }
      /* USER CODE END UX_DEVICE_REMOVAL */

      break;

    case UX_DEVICE_CONNECTION:

      /* USER CODE BEGIN UX_DEVICE_CONNECTION */
      __NOP();
      /* USER CODE END UX_DEVICE_CONNECTION */

      break;

    case UX_DEVICE_DISCONNECTION:

      /* USER CODE BEGIN UX_DEVICE_DISCONNECTION */
      __NOP();
      /* USER CODE END UX_DEVICE_DISCONNECTION */

      break;

    default:

      /* USER CODE BEGIN EVENT_DEFAULT */

      /* USER CODE END EVENT_DEFAULT */

      break;
  }

  /* USER CODE BEGIN ux_host_event_callback1 */

  /* USER CODE END ux_host_event_callback1 */

  return status;
}

/**
  * @brief ux_host_error_callback
  *         This callback is invoked to notify application of error changes.
  * @param  system_level: system level parameter.
  * @param  system_context: system context code.
  * @param  error_code: error event code.
  * @retval Status
  */
VOID ux_host_error_callback(UINT system_level, UINT system_context, UINT error_code)
{
  /* USER CODE BEGIN ux_host_error_callback0 */
  UX_PARAMETER_NOT_USED(system_level);
  UX_PARAMETER_NOT_USED(system_context);
  /* USER CODE END ux_host_error_callback0 */

  switch (error_code)
  {
    case UX_DEVICE_ENUMERATION_FAILURE:

      /* USER CODE BEGIN UX_DEVICE_ENUMERATION_FAILURE */

      /* USER CODE END UX_DEVICE_ENUMERATION_FAILURE */

      break;

    case  UX_NO_DEVICE_CONNECTED:

      /* USER CODE BEGIN UX_NO_DEVICE_CONNECTED */

      /* USER CODE END UX_NO_DEVICE_CONNECTED */

      break;

    default:

      /* USER CODE BEGIN ERROR_DEFAULT */

      /* USER CODE END ERROR_DEFAULT */

      break;
  }

  /* USER CODE BEGIN ux_host_error_callback1 */

  /* USER CODE END ux_host_error_callback1 */
}

/* USER CODE BEGIN 1 */
// uint32_t videoReadDone=0;
// uint32_t videoConnected=0;

void video_transfer_complete(UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST* video_transfer){
	videoReadDone=1;
  uint8_t currentVideoLine =video_transfer->ux_host_class_video_transfer_request_actual_length;
  if(currentVideoLine>12){
    currentVideoLine-=12;
    uint32_t newTimestamp=*(uint32_t*)&buffer[2];
    if(frameTimestamp!=newTimestamp){
      frameTimestamp=newTimestamp;
      videoLine=0;
    }
    // memcpy((uint8_t*)(0x300D0000+videoLine),&buffer[12],currentVideoLine);
    videoLine+= yuvToRgb(&buffer[12],(uint8_t*)(0x300D0000+videoLine),currentVideoLine);
    // videoLine+=currentVideoLine;
    if(videoLine>=(320*240*2)){
      videoLine=0;
    }
  }
}


void video_read(){
	  if(videoReadDone==1 && videoConnected==1){

      video_transfer_request.ux_host_class_video_transfer_request_data_pointer=buffer;//data buffer
      video_transfer_request.ux_host_class_video_transfer_request_requested_length= 320*240*2;//640*480*2
      video_transfer_request.ux_host_class_video_transfer_request_completion_function=&video_transfer_complete;
      ux_host_class_video_read(video,&video_transfer_request );
    }
}


uint32_t yuvToRgb(uint8_t* in,uint8_t* out,uint32_t length){
  // Get the bytes
  uint32_t i=0;
  uint32_t outCounter=0;
  for(i=0;i<length;i=i+4){
    uint8_t r,g,b;
    uint16_t pixel;

    // uint8_t y = in[i];
    // uint8_t u = in[i+1];
    // uint8_t v = in[i+2];


    // oneYuvToRgbConversion(y,u,v,&r,&g,&b);
    // out[outCounter++]=r;
    // out[outCounter++]=g;
    // out[outCounter++]=b;

    uint8_t y1 = in[i];
    uint8_t u = in[i+1];
    uint8_t y2 = in[i+2];
    uint8_t v = in[i+3];
    oneYuvToRgbConversion(y1,u,v,&r,&g,&b);
    pixel=(r>>3) | ((g>>2)<<5) | ((b>>3)<<11); 
    out[outCounter++]=pixel&0xFF;
    out[outCounter++]=pixel>>8;

    oneYuvToRgbConversion(y2,u,v,&r,&g,&b);
    pixel=(r>>3) | ((g>>2)<<5) | ((b>>3)<<11); 
    out[outCounter++]=pixel&0xFF;
    out[outCounter++]=pixel>>8;
  }
  return outCounter;
}

void oneYuvToRgbConversion(int8_t y, int8_t u, int8_t v, uint8_t *rOut, uint8_t *gOut, uint8_t *bOut)
{
  // Convert, cast to signed byte is important!
  // uint8_t r = y + (1.403 * (int8_t)v);
  // uint8_t g = y - (0.344 * (int8_t)u) - (0.714 * (int8_t)v);
  // uint8_t b = y + (1.770 * (int8_t)u);

  // float r = y + (1.403 * (float)v);
  // float g = y - (0.344 * (float)u) - (0.714 * (float)v);
  // float b = y + (1.770 * (float)u);

//bm709
  float r = y + (1.28033 * (float)v);
  float g = y - (0.21482 * (float)u) - (0.38059 * (float)v);
  float b = y + (2.12798 * (float)u);

	// float r = 1.164*((float)y - 16) + 2.018*((float)u - 128);

	// float g = 1.164*((float)y - 16) - 0.813*((float)v - 128) - 0.391*((float)u - 128);

	// float b = 1.164*((float)y - 16) + 1.596*((float)v - 128);

  if (r < 0)
    r = 0;
  else if (r > 255)
    r = 255;

  if (g < 0)
  {
    g = 0;
  }
  else if (g > 255)
  {
    g = 255;
  }

  if (b < 0)
  {
    b = 0;
  }
  else if (b > 255)
  {
    b = 255;
  }

  *rOut = r;
  *gOut = g;
  *bOut = b;
}
/* USER CODE END 1 */
