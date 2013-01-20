/**
  ******************************************************************************
  * @file    DCMI/Camera/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-September-2011
  * @brief   Main program body
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
#include "main.h"
#include "dcmi_ov9655.h"
#include "dcmi_ov2640.h"
#include  "lcd_log.h"
#include "stm32f4_discovery_lis302dl.h"
#include "stm32f4xx_flash.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup DCMI_Camera
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_2   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_5   /* End @ of user Flash area */

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

#define DATA_32                 ((uint32_t)0x12345678)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t StartSector = 0, EndSector = 0, Address = 0, SectorCounter = 0;
RCC_ClocksTypeDef RCC_Clocks;
OV9655_IDTypeDef OV9655_Camera_ID;
OV2640_IDTypeDef OV2640_Camera_ID;
__IO uint16_t ADCVal = 0;
uint8_t buffer[40];
extern Camera_TypeDef Camera;
extern ImageFormat_TypeDef ImageFormat;
extern __IO uint8_t ValueMax;
extern const uint8_t *ImageForematArray[];

/* Private function prototypes -----------------------------------------------*/
void ADC_Config(void);

/* Private functions ---------------------------------------------------------*/
uint32_t GetSector(uint32_t Address);

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	uint8_t tmpreg;
	LIS302DL_InitTypeDef  LIS302DL_InitStruct;
  /*!< At this stage the microcontroller clock setting is already configured, 
  this is done through SystemInit() function which is called from startup
  file (startup_stm32f4xx.s) before to branch to application main.
  To reconfigure the default setting of SystemInit() function, refer to
  system_stm32f4xx.c file
  */

  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

  //NVIC_SetPriority(SysTick_IRQn, -1);

    /* MEMS configuration and set int1/int2 pins*/
    LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
    LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
    LIS302DL_InitStruct.Axes_Enable = LIS302DL_XYZ_ENABLE;
    LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
    LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
    LIS302DL_Init(&LIS302DL_InitStruct);
	//Set INT1/INT2 pins
	tmpreg = 0x40;
	LIS302DL_Write(&tmpreg, LIS302DL_CTRL_REG3_ADDR, 1);

  /* Initialize LEDs and push-buttons mounted on STM324xG-EVAL board */
  STM_EVAL_LEDInit(LED1);
  STM_EVAL_LEDInit(LED2);
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);

  //STM_EVAL_LEDOn(LED1);

 /* Initialize the LCD */
  STM324xG_LCD_Init();
  //STM_EVAL_LEDOn(LED1);
  LCD_Clear(Black);
  LCD_SetTextColor(White);

  LCD_LOG_SetHeader("STM32 Camera Demo");
  LCD_LOG_SetFooter ("   Copyright (c) STMicroelectronics" );

  /* ADC configuration */
  ADC_Config();

  /* Initializes the DCMI interface (I2C and GPIO) used to configure the camera */
  OV9655_HW_Init();

  /* Read the OV9655/OV2640 Manufacturer identifier */
  OV9655_ReadID(&OV9655_Camera_ID);
  OV2640_ReadID(&OV2640_Camera_ID);

  if(OV9655_Camera_ID.PID  == 0x96)
  {
    Camera = OV9655_CAMERA;
    sprintf((char*)buffer, "OV9655 Camera ID 0x%x", OV9655_Camera_ID.PID);
    ValueMax = 2;
  }
  else if(OV2640_Camera_ID.PIDH  == 0x26)
  {
    Camera = OV2640_CAMERA;
    sprintf((char*)buffer, "OV2640 Camera ID 0x%x", OV2640_Camera_ID.PIDH);
    ValueMax = 2;
  }
  else
  {
    LCD_SetTextColor(LCD_COLOR_RED);
    sprintf((char*)buffer, "OV Camera ID 0x%02x%02x", OV9655_Camera_ID.Version, OV9655_Camera_ID.PID);
    LCD_DisplayStringLine(LINE(4), buffer);
    while(1);  
  }

  LCD_SetTextColor(LCD_COLOR_YELLOW);
  LCD_DisplayStringLine(LINE(4), (uint8_t*)buffer);
  LCD_SetTextColor(LCD_COLOR_WHITE);

  Delay(200);

  /* Initialize demo */
  ImageFormat = (ImageFormat_TypeDef)BMP_QVGA;//Demo_Init();

  /* Configure the Camera module mounted on STM324xG-EVAL board */
  Demo_LCD_Clear();
  LCD_DisplayStringLine(LINE(4), "Camera Init..               ");
  Camera_Config();

  sprintf((char*)buffer, " Image selected: %s", ImageForematArray[ImageFormat]);
  LCD_DisplayStringLine(LINE(4),(uint8_t*)buffer);

  LCD_ClearLine(LINE(4));
  Demo_LCD_Clear();


  if(ImageFormat == BMP_QQVGA)
  {
    /* LCD Display window */
    LCD_SetDisplayWindow(60, 80, 160, 120);
    ReverseLCD();
    LCD_WriteRAM_Prepare(); 
  }
  else if(ImageFormat == BMP_QVGA)
  {
    /* LCD Display window */
    LCD_SetDisplayWindow(0, 0, 320, 240);
    ReverseLCD();
    LCD_WriteRAM_Prepare(); 
  }

  /* Enable DMA2 stream 1 and DCMI interface then start image capture */
  DMA_Cmd(DMA2_Stream1, ENABLE); 
  DCMI_Cmd(ENABLE); 

  /* Insert 100ms delay: wait 100ms */
  Delay(200); 

  DCMI_CaptureCmd(ENABLE); 

  while(1)
  {
  	static uint16_t oldADCVal[4] = {0}; //smooth the data
	static uint16_t lastavgADC = 0;
    /* Blink LD1, LED2 and LED4 */
    //STM_EVAL_LEDToggle(LED1);
    //STM_EVAL_LEDToggle(LED2);
    //STM_EVAL_LEDToggle(LED3);
    //STM_EVAL_LEDToggle(LED4);

    /* Insert 100ms delay */
    Delay(10);

    /* Get the last ADC3 conversion result data */
	oldADCVal[0] = oldADCVal[1];
	oldADCVal[1] = oldADCVal[2];
	oldADCVal[2] = oldADCVal[3];
	oldADCVal[3] = ADC_GetConversionValue(ADC3);
	ADCVal = (oldADCVal[0] + oldADCVal[1] + oldADCVal[2] + oldADCVal[3]) >> 2; //average 4 times value;

    
    /* Change the Brightness of camera using "Brightness Adjustment" register:
       For OV9655 camera Brightness can be positively (0x01 ~ 0x7F) and negatively (0x80 ~ 0xFF) adjusted
       For OV2640 camera Brightness can be positively (0x20 ~ 0x40) and negatively (0 ~ 0x20) adjusted */
	if (!((lastavgADC - ADCVal < 5 && lastavgADC - ADCVal >= 0) || (ADCVal - lastavgADC < 5 && ADCVal - lastavgADC >= 0)))
	{
#if 0
		DCMI_CaptureCmd(DISABLE);

  		DMA_Cmd(DMA2_Stream1, DISABLE); 
  		FLASH_Unlock();

		/* Clear pending flags (if any) */  
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
		FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 
		
		/* Get the number of the start and end sectors */
		StartSector = GetSector(FLASH_USER_START_ADDR);
		EndSector = GetSector(FLASH_USER_END_ADDR);
		
		for (SectorCounter = StartSector; SectorCounter < EndSector; SectorCounter += 8)
		{
			/* Device voltage range supposed to be [2.7V to 3.6V], the operation will
			be done by word */ 
			if (FLASH_EraseSector(SectorCounter, VoltageRange_3) != FLASH_COMPLETE)
			{ 
				/* Error occurred while sector erase. 
				User can add here some code to deal with this error  */
				while (1)
				{
				}
			}
		}
		
		/* Program the user Flash area word by word
		(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
		
		DCMI_CaptureCmd(ENABLE);
		Address = FLASH_USER_START_ADDR;
		
		while (Address < FLASH_USER_START_ADDR + 240 * 320)
		{
			if (FLASH_ProgramWord(Address, *(volatile unsigned long *)DCMI_DR_ADDRESS) == FLASH_COMPLETE)
			{
				Address = Address + 4;
			}
			else
			{ 
				/* Error occurred while writing data in Flash memory. 
				User can add here some code to deal with this error */
				while (1)
				{
				}
			}
		}
		FLASH_Lock();

#endif


	    if(Camera == OV9655_CAMERA)
	    {
	      OV9655_BrightnessConfig(ADCVal);
	    }
	    if(Camera == OV2640_CAMERA)
	    {
	      OV2640_BrightnessConfig(ADCVal/2);
	    }
		lastavgADC = ADCVal;
	}
  }
}

/**
  * @brief  Configures the ADC.
  * @param  None
  * @retval None
  */
void ADC_Config(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable ADC3 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  /* GPIOF clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 

  /* Configure ADC Channel3 as analog */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* ADC Common Init */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; 
  ADC_CommonInit(&ADC_CommonInitStructure); 

  /* ADC3 Configuration ------------------------------------------------------*/
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_8b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; 
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* ADC3 Regular Channel Config */
  ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 1, ADC_SampleTime_56Cycles);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);

  /* ADC3 regular Software Start Conv */ 
  ADC_SoftwareStartConv(ADC3);
}

/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_Sector_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_Sector_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_Sector_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_Sector_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_Sector_4;  
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_Sector_5;  
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_Sector_6;  
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_Sector_7;  
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_Sector_8;  
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_Sector_9;  
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_Sector_10;  
  }
  else/*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11))*/
  {
    sector = FLASH_Sector_11;  
  }

  return sector;
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
