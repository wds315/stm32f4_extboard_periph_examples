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
#include  "lcd_log.h"
#include "stm32f4_discovery_lis302dl.h"
#include "stm324xg_eval_tsc.h"
#include "stm32f4xx_flash.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup DCMI_Camera
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_5   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_8   /* End @ of user Flash area */

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

/* Private variables ---------------------------------------------------------*/
RCC_ClocksTypeDef RCC_Clocks;
OV9655_IDTypeDef OV9655_Camera_ID;
__IO uint16_t ADCVal = 0;
uint8_t buffer[40];
extern Camera_TypeDef Camera;
extern ImageFormat_TypeDef ImageFormat;
extern __IO uint8_t ValueMax;
extern const uint8_t *ImageForematArray[];

volatile int datablock = 0;				//DMA当前数据块的位置
volatile int bShot = 0;					//是否处于快门模式

/* Private function prototypes -----------------------------------------------*/
void ADC_Config(void);
uint32_t GetSector(uint32_t Address);

/* Private functions ---------------------------------------------------------*/

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

	//Initialize the touchscreen
	TSC_Init();

  	/* Initialize LEDs and push-buttons mounted on STM324xG-EVAL board */
	STM_EVAL_LEDInit(LED1);
	STM_EVAL_LEDInit(LED2);
	//STM_EVAL_LEDInit(LED3);
	//STM_EVAL_LEDInit(LED4);

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

  	if(OV9655_Camera_ID.PID  == 0x96)
  	{
    	Camera = OV9655_CAMERA;
    	sprintf((char*)buffer, "OV9655 Camera ID 0x%x", OV9655_Camera_ID.PID);
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
  	ImageFormat = (ImageFormat_TypeDef)BMP_QVGA;

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

	{
		int i, j;
		for (j = 0; j < 16; j++)
		{
			LCD_SetDisplayWindow(0, (240 / 16) * j, 320, (240 / 16));
			LCD_WriteRAM_Prepare();
			for (i = 0; i <	320 * 240 / 16; i++)
			{
				LCD_WriteRAM(0x0003 << j);
			}
		}
	}

  	/* Enable DMA2 stream 1 and DCMI interface then start image capture */
  	DMA_Cmd(DMA2_Stream1, ENABLE); 
  	DCMI_Cmd(ENABLE); 

  	/* Insert 100ms delay: wait 100ms */
  	//Delay(200); 

  	DCMI_CaptureCmd(ENABLE); 

  	while(1)
  	{
		//static int step = 0;
		int i, block, begin;
		unsigned short *p;

		if (!bShot)
		{
			begin = (datablock - 11 + 48) % 48;
	
			for (block = begin; block < begin + 11; block++)
			{
				p = (unsigned short *)(0x20000000 + (320 * 240 * 2 / 16) * ((block) % 12));
				LCD_SetDisplayWindow(0, (block % 16) * (240 / 16), 320, (240 / 16));
				//LCD_SetCursor(0, (block % 16) * (240 / 16));
				LCD_WriteRAM_Prepare();
				for (i = 0; i <	320 * 240 / 16; i++)
				{
					LCD_WriteRAM(*p++);
				}
			}
		}
		if (TSC_TouchDet())
		{
			int x, y;
			TP_GetAdXY(&x, &y);
			if (x > 300 && x < 2800 && y > 300 && y < 2800)
			{
				if (x < 1550 && y < 1550)
				{
					uint32_t StartSector = 0, EndSector = 0, Address = 0, SectorCounter = 0;
					int m;
					//拍照bShot
					bShot = 1;
					while(datablock % 16);
					DMA_Cmd(DMA2_Stream1, DISABLE); 
					DCMI_Cmd(DISABLE);
					//STM_EVAL_LEDOn(LED2); 
					DMA_ClearFlag(DMA2_Stream1, 0x2F7D0F7D);
					DMA_ClearFlag(DMA2_Stream1, 0x0F7D0F7D);
					DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
					datablock = 0;
					DMA2_Stream1->M0AR = 0x20000000 + 9600 * (datablock % 12);
					DMA_SetCurrDataCounter(DMA2_Stream1, (320 * 240 * 2 / 4) / 16);

					DCMI_Cmd(ENABLE);
					DMA_Cmd(DMA2_Stream1, ENABLE);
					
					{
						unsigned long *p, *q;
						while(datablock < 4);	//等待准备好前4块数据，就将这4块数据导入到0x10000000+0x50000之后。
						q = (unsigned long *)(0x20000000);
						p = (unsigned long *)(0x10000000 + 0x5000);
						while (q < (unsigned long *)(0x20000000 + 4 * (320 * 240 * 2 / 16)))
						{
							*p = *q;
							p++;
							q++;
						}
					}


					while(datablock < 16);		//等待全部采集完成。

					STM_EVAL_LEDOn(LED2);		//LED2亮表示采集完成。
					LCD_SetDisplayWindow(0, 0, 320, 240);
					LCD_WriteRAM_Prepare();
					LCD_Clear(Black);
					//RAM位置
					/*
						序号  地址                大小
						1:    0x10005000+9600*0     9600
						2:    0x10005000+9600*1     9600
						3:    0x10005000+9600*2     9600
						4:    0x10005000+9600*3     9600
						5:    0x20000000+9600*5     9600
						6:    0x20000000+9600*6     9600
						7:    0x20000000+9600*7     9600
						8:    0x20000000+9600*8     9600
						9:    0x20000000+9600*9     9600
						10:   0x20000000+9600*10    9600
						11:   0x20000000+9600*11    9600
						12:   0x20000000+9600*0     9600
						13:   0x20000000+9600*1     9600
						14:   0x20000000+9600*2     9600
						15:   0x20000000+9600*3     9600					
					*/
				 	for (m = 0; m < 16; m++)	//显示保存的图片
					{
						unsigned short *q;
						if (m < 4)
						{
							q = (unsigned short *)(0x10000000 + 0x5000 + m * (320 * 240 * 2 / 16));
						}
						else
						{
							q = (unsigned short *)(0x20000000 + (m % 12) * (320 * 240 * 2 / 16));
						}
						LCD_SetDisplayWindow(0, m * (240 / 16), 320, (240 / 16));
						LCD_WriteRAM_Prepare();
						for (i = 0; i <	320 * 240 / 16; i++)
						{
							LCD_WriteRAM(*q++);
						}
					}

					/* Erase the user Flash area ***********/
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
										
				 	for (m = 0; m < 16; m++)	//保存图片到flash
					{
						unsigned long *q;
						Address = FLASH_USER_START_ADDR + m * (320 * 240 * 2 / 16);
						if (m < 4)
						{
							q = (unsigned long *)(0x10000000 + 0x5000 + m * (320 * 240 * 2 / 16));
						}
						else
						{
							q = (unsigned long *)(0x20000000 + (m % 12) * (320 * 240 * 2 / 16));
						}
						while (Address < FLASH_USER_START_ADDR + (m + 1) *(320 * 240 * 2 / 16))
						{
							if (FLASH_ProgramWord(Address, *q) == FLASH_COMPLETE)
							{
								Address = Address + 4;
								q++;
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
					}

					STM_EVAL_LEDOff(LED2);
					LCD_SetDisplayWindow(0, 0, 320, 240);
					LCD_WriteRAM_Prepare();
					LCD_Clear(Black);
				 	for (m = 0; m < 16; m++)	//显示flash中的图片
					{
						unsigned short *q;
						q = (unsigned short *)(FLASH_USER_START_ADDR + m * (320 * 240 * 2 / 16));
						LCD_SetDisplayWindow(0, m * (240 / 16), 320, (240 / 16));
						LCD_WriteRAM_Prepare();
						for (i = 0; i <	320 * 240 / 16; i++)
						{
							LCD_WriteRAM(*q++);
						}
					}
					/* Lock the Flash to disable the flash control register access (recommended
					 to protect the FLASH memory against possible unwanted operation) *********/
					FLASH_Lock(); 
				}
				else if (x >= 1550 && y < 1550)
				{
					//righttop
					STM_EVAL_LEDOff(LED1);
					STM_EVAL_LEDOff(LED2);
					bShot = 0;
					datablock = 0;
					DMA_Cmd(DMA2_Stream1, ENABLE);
					DCMI_Cmd(ENABLE); 
				}
				else if (x < 1550 && y >= 1550)
				{
					//righttop
					//DMA_Cmd(DMA2_Stream1, ENABLE);
					//DCMI_Cmd(ENABLE); 
				}
				else if (x >= 1550 && y >= 1550)
				{
					//righttop
					//DMA_Cmd(DMA2_Stream1, ENABLE);
					//DCMI_Cmd(ENABLE); 
				}
			}
		}
		//Delay(10);
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
/*
void InitForCapture(void)
{
	DMA_DeInit(DMA2_Stream1);
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_1;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = DCMI_DR_ADDRESS;	
	DMA_InitStructure.DMA_Memory0BaseAddr = 0x20000000;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);   
}
*/

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
