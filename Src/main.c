/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "oled.h"
#include "pid.h"
#include "mpu6050.h"
#include "control.h"
#include "st7533.h"
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

/* USER CODE BEGIN PV */
uint16_t time_count=0;
//获取的转速
int Get_Count_L_F=0;	
int Get_Count_L_B=0;
int Get_Count_R_F=0;
int Get_Count_R_B=0;

//当前的转速
int Count_L_F=0;
int Count_L_B=0;
int Count_R_F=0;
int Count_R_B=0;

char LF_str[50]="";
char LB_str[50]="";
char RF_str[50]="";
char RB_str[50]="";

//输出速度
float PWM_Value_L_F=0;
float PWM_Value_L_B=0;
float PWM_Value_R_F=0;
float PWM_Value_R_B=0;

//接收数据
uint8_t get_buff[1]="";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	Motor_Init();
	
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);	//左前
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);  //左后
	
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);	//右前
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);	//右后
	
	HAL_TIM_Base_Start_IT(&htim6);
	
	__HAL_TIM_SET_COUNTER(&htim1,32767);
	__HAL_TIM_SET_COUNTER(&htim2,32767);
	__HAL_TIM_SET_COUNTER(&htim3,32767);
	__HAL_TIM_SET_COUNTER(&htim4,32767);
	
	
	Speed_PID_Init(100);
	
	lcd7735_ini();
	lcd7735_fillrect(0,0,128,160,BLACK);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//Move(PWM_Value_L_F,PWM_Value_L_B,PWM_Value_R_F,PWM_Value_R_B,FRONT);
		Move(PWM_Value_L_F,PWM_Value_L_B,PWM_Value_R_F,PWM_Value_R_B,BACK);
		sprintf(LF_str,"LF:%06d",Count_L_F);
		lcd7735_putstr(0,0,LF_str,WHITE,BLACK);
		
		sprintf(LB_str,"LB:%06d",Count_L_B);
		lcd7735_putstr(12,0,LB_str,WHITE,BLACK);
		
		sprintf(RF_str,"RF:%06d",Count_R_F);
		lcd7735_putstr(24,0,RF_str,WHITE,BLACK);
		
		sprintf(RB_str,"RB:%06d",Count_R_B);
		lcd7735_putstr(36,0,RB_str,WHITE,BLACK);
		
		sprintf(LF_str,"LF_DUTY:%.2f",PWM_Value_L_F);
		lcd7735_putstr(48,0,LF_str,WHITE,BLACK);
		
		sprintf(LB_str,"LB_DUTY:%.2f",PWM_Value_L_B);
		lcd7735_putstr(60,0,LB_str,WHITE,BLACK);
		
		sprintf(RF_str,"RF_DUTY:%.2f",PWM_Value_R_F);
		lcd7735_putstr(72,0,RF_str,WHITE,BLACK);
		
		sprintf(RB_str,"RB_DUTY:%.2f",PWM_Value_R_B);
		lcd7735_putstr(84,0,RB_str,WHITE,BLACK);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	if(htim==(&htim6))
	{
		time_count++;
	  if(time_count>100)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			
			Get_Count_L_F=__HAL_TIM_GET_COUNTER(&htim1);
			Get_Count_L_B=__HAL_TIM_GET_COUNTER(&htim2);
			Get_Count_R_F=__HAL_TIM_GET_COUNTER(&htim3);
			Get_Count_R_B=__HAL_TIM_GET_COUNTER(&htim4);
				
			Count_L_F=32767-Get_Count_L_F;
			Count_L_B=32767-Get_Count_L_B;
			Count_R_F=32767-Get_Count_R_F;
			Count_R_B=32767-Get_Count_R_B;
			
			if(Count_L_F<0)
				Count_L_F=-Count_L_F;
			if(Count_L_B<0)
				Count_L_B=-Count_L_B;
			if(Count_R_F<0)
				Count_R_F=-Count_R_F;
			if(Count_R_B<0)
				Count_R_B=-Count_R_B;
			
			PWM_Value_L_F+=Speed_Adjust(Count_L_F);
			if(PWM_Value_L_F>90)
				PWM_Value_L_F=90;
			if(PWM_Value_L_F<0)
				PWM_Value_L_F=0;
			PWM_Value_L_B+=Speed_Adjust(Count_L_B);
			if(PWM_Value_L_B>90)
				PWM_Value_L_B=90;
			if(PWM_Value_L_B<0)
				PWM_Value_L_B=0;
			PWM_Value_R_F+=Speed_Adjust(Count_R_F);
			if(PWM_Value_R_F>90)
				PWM_Value_R_F=90;
			if(PWM_Value_R_F<0)
				PWM_Value_R_F=0;
			PWM_Value_R_B+=Speed_Adjust(Count_R_B);
			if(PWM_Value_R_B>90)
				PWM_Value_R_B=90;
			if(PWM_Value_R_B<0)
				PWM_Value_R_B=0;
			
			__HAL_TIM_SET_COUNTER(&htim1,32767);
			__HAL_TIM_SET_COUNTER(&htim2,32767);
			__HAL_TIM_SET_COUNTER(&htim3,32767);
			__HAL_TIM_SET_COUNTER(&htim4,32767);
			
			time_count=0;
		}
		
	}

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
