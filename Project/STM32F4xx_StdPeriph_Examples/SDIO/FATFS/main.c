/**
  ******************************************************************************
  * @file    SDIO/uSDCard/main.c 
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
#include "stm32f4xx.h"
#include "stm324xg_eval_sdio_sd.h"
#include "stm324xg_eval_lcd.h"
#include "ff.h"
#include "string.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup SDIO_uSDCard
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define BLOCK_SIZE            512 /* Block Size in Bytes */


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
SD_Error Status = SD_OK;

/* Private function prototypes -----------------------------------------------*/
void NVIC_Configuration(void);
void ShowBMP(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	FATFS fs;           

	FRESULT res;                 

	DIR dirs;

  	FILINFO finfo;

	/*!< At this stage the microcontroller clock setting is already configured, 
	   this is done through SystemInit() function which is called from startup
	   file (startup_stm32f4xx.s) before to branch to application main.
	   To reconfigure the default setting of SystemInit() function, refer to
	   system_stm32f4xx.c file
	 */

    /* Initialize LEDs available on STM324xG-EVAL board *************************/
    STM_EVAL_LEDInit(LED1);
    STM_EVAL_LEDInit(LED2);
    STM_EVAL_LEDInit(LED3);
    STM_EVAL_LEDInit(LED4);  

    STM324xG_LCD_Init();
    
    LCD_Clear(Black);
    LCD_SetBackColor(Black);
    LCD_SetTextColor(White);

    /* Interrupt Config */
    NVIC_Configuration();

    /*------------------------------ SD Init ---------------------------------- */
    if((Status = SD_Init()) != SD_OK)
    {
        LCD_SetTextColor(Red);
        LCD_DisplayStringLine(LCD_LINE_0, "SD_Init failed");
    }
    else
    {
        LCD_DisplayStringLine(LCD_LINE_0, "SD_Init ok");
    }

    if((Status == SD_OK) && (SD_Detect()== SD_PRESENT))
    {
        /* Infinite loop */
        res=f_mount(0, &fs);
        res=f_opendir(&dirs, "0:/");//打开根目录
        if (res!= FR_OK)
        {
            LCD_SetTextColor(Red);
            LCD_DisplayStringLine(LCD_LINE_1, "opendir failed");
            while (1);
        }
        while (FR_OK == f_readdir(&dirs, &finfo))
        {
            if (finfo.fname[0] == 'T' && 
                finfo.fname[1] == 'E' && 
                finfo.fname[2] == 'S' && 
                finfo.fname[3] == 'T' && 
                finfo.fname[4] == '.' && 
                finfo.fname[5] == 'B' && 
                finfo.fname[6] == 'M' && 
                finfo.fname[7] == 'P')
            {
                ShowBMP();
                break;
            }
        }
/*
        res=f_readdir(&dirs, &finfo);
        LCD_DisplayStringLine(LCD_LINE_1, (uint8_t *)finfo.fname);
        f_readdir(&dirs, &finfo);
        LCD_DisplayStringLine(LCD_LINE_2, (uint8_t *)finfo.fname);
        f_readdir(&dirs, &finfo);
        LCD_DisplayStringLine(LCD_LINE_3, (uint8_t *)finfo.fname);
        f_readdir(&dirs, &finfo);
        LCD_DisplayStringLine(LCD_LINE_4, (uint8_t *)finfo.fname);
        STM_EVAL_LEDOn(LED2); 
*/
    }

    //LCD_DisplayStringLine(LCD_LINE_5, "End");

    while (1)
    {
        int i;
        for (i = 0; i < 10000000; i++);
        STM_EVAL_LEDToggle(LED1);
    }
}

/**
  * @brief  Configures SDIO IRQ channel.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = SD_SDIO_DMA_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure);  
}

typedef struct tagBITMAPFILEHEADER { 
  unsigned short    bfType; 
  unsigned long     bfSize; 
  unsigned short    bfReserved1; 
  unsigned short    bfReserved2; 
  unsigned long     bfOffBits; 
} BITMAPFILEHEADER, *PBITMAPFILEHEADER;

BITMAPFILEHEADER LoadBmpHeader(u8 *data)
{
	BITMAPFILEHEADER bmpheader;
	bmpheader.bfType = (data[1]<<8)|data[0];
	bmpheader.bfSize = (data[5]<<24)|(data[4]<<16)|(data[3]<<8)|data[2];
	bmpheader.bfOffBits = (data[13]<<24)|(data[12]<<16)|(data[11]<<8)|data[10];
	return bmpheader ;
} 

u8 data[sizeof(BITMAPFILEHEADER) + 10];
u8 linebuffer[4*320];

void ShowBMP(void)
{
    FIL file;
    FRESULT res;
    UINT br;
    BITMAPFILEHEADER bmpheader;
    uint32_t index = 0,i = 0;
    res = f_open(&file, "test.bmp", FA_OPEN_EXISTING | FA_READ);
    if(res != FR_OK) 
    {
        LCD_Clear(Black);
        LCD_SetTextColor(Red);
        LCD_DisplayStringLine(LCD_LINE_0, _T("Open bmp failed!"));
    }
    f_read(&file, data, sizeof(bmpheader), &br);   //
    bmpheader = LoadBmpHeader(data);
    if(bmpheader.bfType == 0x4d42) /* 文件格式标记为BM */
    {
        LCD_SetDisplayWindow(0,0,320,240);
        LCD_WriteRAM_Prepare();
        f_lseek(&file, bmpheader.bfOffBits);
        for(i=0;i<240;i++)
        {
            f_read(&file, linebuffer, 320 * 3, &br);
            //LCD_SetCursor(0, i);
            //LCD_WriteRAM_Prepare();
            for(index=0;index<320;index++)
            {
                LCD_WriteRAM(((linebuffer[(index)*3+2]>>3)<<11)|((linebuffer[(index)*3+1]>>2)<<5)|(linebuffer[(index)*3]>>3));
            }
        }
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
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
  {}
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
