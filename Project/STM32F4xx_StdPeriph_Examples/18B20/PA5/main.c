/**
  ******************************************************************************
  * @file    ADC/VBAT_Measurement/main.c 
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
#include "stm324xg_eval.h"
#include "stm324xg_eval_lcd.h"
#include <stdio.h>


/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup ADC_VBAT_Measurement
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* used to display the ADC converted value on LCD */
#define PRINT_ON_LCD

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t Temperature = 0;

/* Private function prototypes -----------------------------------------------*/

#ifdef PRINT_ON_LCD
void Display_Init(void);
void Display(void);
static void delay(__IO uint32_t nCount);
#endif /* PRINT_ON_LCD */

/* Private functions ---------------------------------------------------------*/
void GPIO1820out_Configuration(void)	 //	18b20输出配置
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_5;	
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void GPIO1820reset(void)	 //	18b20复位
{
    u8 pd;
    GPIO1820out_Configuration();
    GPIO_SetBits(GPIOA, GPIO_Pin_5); //高等1us
    delay(50); 
    GPIO_ResetBits(GPIOA, GPIO_Pin_5);	 //拉低总线600us
    delay(500); 
    GPIO_SetBits(GPIOA, GPIO_Pin_5); //高等1us
    delay(40); 
    pd= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);
    while(pd!=0)
    {
        pd= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);
    }
    delay(500); //等240us
    GPIO_SetBits(GPIOA, GPIO_Pin_5); //高等1us
} 

void Writetemper(u8 data)	 // 写数据	 低位先写
{
    u8 i=8; //次数
    u8 datamw;
    while(i>0)
    {
        datamw=data&0X01;
        data=data>>1;
        GPIO_ResetBits(GPIOA, GPIO_Pin_5);	// 总线拉低2us

        delay(5); 

        if(datamw==1)	 // 写1
        {
            GPIO_SetBits(GPIOA, GPIO_Pin_5);	
        }
        else
        {
            GPIO_ResetBits(GPIOA, GPIO_Pin_5);	// 总线拉低2us
        }
        //写0
        delay(65); //等60us
        GPIO_SetBits(GPIOA, GPIO_Pin_5);	//总线先拉高 1us
        delay(5); //等60us
        i--;
    }	
} 

u8 Readtemper(void)	 //读温度数据8位
{
    u8 i=8; //次数
    u8 datamw;
    u8 data=0;
    GPIO_SetBits(GPIOA, GPIO_Pin_5);	 //总线先拉高1us
    delay(5); 
    while(i>0)
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_5);	 // 总线拉低2us
        delay(5); 
        GPIO_SetBits(GPIOA, GPIO_Pin_5);	 //总线先拉高1us
        delay(5); 
        datamw=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5);
        data=data+(datamw<<(8-i));	 // 先读最低位
        delay(65); //等60us
        GPIO_SetBits(GPIOA, GPIO_Pin_5);	 //总线先拉高1us
        i--;
    }
    return data;	
}

u32 Gettemper(void)	 //读温度数据	
{
    u32 datazhuan; //经过转换
    u16 data;
    u16 datag;
    GPIO1820out_Configuration();
    GPIO_SetBits(GPIOA, GPIO_Pin_5);
    GPIO1820reset();	 // 复位
    Writetemper(0XCC);	 //跳过ROM	CCH
    Writetemper(0X44);	 //温度转换 44H
    GPIO1820reset();	 // 复位
    Writetemper(0XCC);	 //跳过ROM	CCH

    Writetemper(0XBE);	 //读取RAM的命令 BEH
    delay(10); //等1us
    data=Readtemper();
    datag=Readtemper();
    data=(datag<<8)+data;
    datag=datag&0XF8;
    if(datag!=0)
    {
        data=~data+1;
    }
    datazhuan=data*625/1000;
    return datazhuan;	 //实际温度乘10
} 

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */
#ifdef PRINT_ON_LCD
  /* LCD Display init  */
  Display_Init();
#endif /* PRINT_ON_LCD */

  while (1)
  {
#ifdef PRINT_ON_LCD
  /* Display ADC converted value on LCD */
    Display();
#endif /* PRINT_ON_LCD */
  }
}

#ifdef PRINT_ON_LCD
/**
  * @brief  Display ADC converted value on LCD
  * @param  None
  * @retval None
  */
void Display(void)
{
   uint8_t text[50];
   Temperature = Gettemper();
  sprintf((char*)text,"%d",Temperature);
  LCD_DisplayStringLine(LINE(6),text);

  delay(100);
}

/**
  * @brief  Display Init (LCD)
  * @param  None
  * @retval None
  */
void Display_Init(void)
{

  /* Initialize the LCD */
  STM324xG_LCD_Init();

  /* Clear the LCD */
  LCD_Clear(White);

  /* Set the LCD Text size */
  LCD_SetFont(&Font8x12);

  /* Set the LCD Back Color and Text Color*/
  LCD_SetBackColor(Blue);
    LCD_SetTextColor(Red);
    LCD_DrawCircle(50, 200, 20);
    LCD_DrawFullCircle(250, 200, 20);
  LCD_SetTextColor(White);

  LCD_DisplayStringLine(LINE(0x13), " STM32F4xx 18B20@PA5 Measurement example");

  /* Set the LCD Text size */
  LCD_SetFont(&Font16x24);

  //LCD_DisplayStringLine(LINE(0), "**VBAT Measurement**");
  LCD_DisplayHanzi(0, 0, "汉");
  LCD_DisplayHanzi(16, 0, "字");

  /* Set the LCD Back Color and Text Color*/
  LCD_SetBackColor(White);
  LCD_SetTextColor(Blue);

  LCD_DisplayStringLine(LINE(2)," Eval Board Instant ");
  LCD_DisplayStringLine(LINE(4),"  Battery Voltage   ");
}

/**
  * @brief  Inserts a delay time
  * @param  nCount: specifies the delay time length
  * @retval None
  */
static void delay(__IO uint32_t nCount)
{
  __IO uint32_t index = 0; 
  for(index = (100000 * nCount); index != 0; index--)
  {
  }
}
#endif /* PRINT_ON_LCD */


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
