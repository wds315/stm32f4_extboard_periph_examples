/**
  ******************************************************************************
  * @file    DCMI/Camera/camera_api.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-September-2011
  * @brief   This file contains the routinue needed to configure OV9655 
  *          Camera modules.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "camera_api.h"
#include "dcmi_ov9655.h"
#include "stm324xg_eval.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup DCMI_Camera
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Image Formats */
const uint8_t *ImageForematArray[] =
{
  "BMP QQVGA Format    ",
  "BMP QVGA Format     ",
};
static __IO uint32_t TimingDelay;
__IO uint32_t PressedKey = 0;
uint8_t ValueMin = 0, ValueMax = 0;
Camera_TypeDef Camera;
ImageFormat_TypeDef ImageFormat;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Display the navigation menu.
  * @param  None
  * @retval None
  */
void Display_Menu(uint8_t ForematIndex, uint8_t MaxForematIndex)
{
  uint32_t index;

  for(index=0; index<MaxForematIndex; index++)
  {
    if(index == ForematIndex)
    {
      LCD_SetTextColor(LCD_COLOR_RED);
      LCD_DisplayStringLine(LINE(12 + index), (uint8_t*)ImageForematArray[index]);
      LCD_SetTextColor(LCD_COLOR_WHITE);
    }
    else
    {
      LCD_DisplayStringLine(LINE(12 + index), (uint8_t*)ImageForematArray[index]);
    }
  }
}

/**
  * @brief  Configures OV9655 Camera module mounted on STM324xG-EVAL board.
  * @param  ImageBuffer: Pointer to the camera configuration structure
  * @retval None
  */
void Camera_Config(void)
{
  if(Camera == OV9655_CAMERA)
  {
    switch (ImageFormat)
    {
      case BMP_QQVGA:
      {
        /* Configure the OV9655 camera and set the QQVGA mode */
        OV9655_HW_Init();
        OV9655_Init(BMP_QQVGA);
        OV9655_QQVGAConfig();
        break;
      }
      case BMP_QVGA:
      {
        /* Configure the OV9655 camera and set set the QVGA mode */
        OV9655_HW_Init();
        OV9655_Init(BMP_QVGA);
        OV9655_QVGAConfig();
        break;
      }
      default:
      {
        /* Configure the OV9655 camera and set the QQVGA mode */
        OV9655_HW_Init();
        OV9655_Init(BMP_QQVGA);
        OV9655_QQVGAConfig();
        break;
      } 
    }
  }
}

/**
  * @brief  Clear LCD screen.
  * @param  None
  * @retval None
  */
void Demo_LCD_Clear(void)
{
  uint32_t j;
  for( j= 5; j < 19; j++ ) 
  {
    LCD_ClearLine(LINE(j));
  }
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds
  * @retval None
  */
void Delay(uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);

}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

/**
  * @brief  Empty function.
  * @param  None
  * @retval None
  */
void NullFunc(void)
{
}

/**
  * @}
  */ 

/**
  * @}
  */ 
  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
