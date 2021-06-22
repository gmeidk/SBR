/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  * 		Authors: Quatela Alessandro, Roberto Giuseppe
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>		// Used to obtain serial communication
#include "MPU6050.h"
#include "Stepper.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROL_TYPE 0				// 0->PID 1->LQR
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char uart_buf[50];
int uart_buf_len;
//uint16_t start_t = 0, end_t;
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* START ROBOT CONFIGURATION */
  uart_buf_len = sprintf(uart_buf, "STARTING SERIAL COMMUNICATION\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);


  /* MPU6050 Initialization */
  if (MPU6050_Init(hi2c1, 500, 1000)) {	/* if MPU6050_Init() = 1 -> HAL_Busy error */
	  uart_buf_len = sprintf(uart_buf, "CONFIGURATION ERROR: MPU6050 is not reachable\r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
	  while (1) {
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  HAL_Delay(500);
	  }
  }

  uart_buf_len = sprintf(uart_buf, "CONFIGURATION COMPLETED\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

  /* TIM Initialization */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);

  HAL_TIM_Base_Start_IT(&htim10);
  //HAL_TIM_Base_Start(&htim11);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim==&htim10)
	{

//		end_t = __HAL_TIM_GET_COUNTER(&htim11);
//		float dt = ABS((float) (end_t - start_t) / 10000);
//		start_t = __HAL_TIM_GET_COUNTER(&htim11);
//
//		uart_buf_len = sprintf(uart_buf, "Time= %.4f s \r\n", dt);
//		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

		/* Sampling time */
		float dt = (float) 20 / 10000;


		/* Sensor reading */
		Control_Data_t Control_Data = MPU6050_Data(hi2c1, dt);


		/* Control selection */
		float velocity = 0;
		if (CONTROL_TYPE == 0) {
			velocity = PID_Control(Control_Data, dt);
		} else if (CONTROL_TYPE == 1) {
			velocity = LQR_Control(Control_Data, dt);
		}


		/* Speed actuation */
		Speed_Actuation(htim2, htim5, velocity);

//		uart_buf_len = sprintf(uart_buf, "Y_angle= %.2f" "Y_out= %.2f" "Gy= %.2f\r\n", Control_Data.angleY, Control_Data.angleY_out, Control_Data.Gy);
//		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
//		uart_buf_len = sprintf(uart_buf, "Time= %.5f s, Vel= %.2f, ARR= %d\r\n", dt, velocity, htim2.Instance->ARR);
//		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
//		uart_buf_len = sprintf(uart_buf, "Time= %.4f s \r\n", dt);
//		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
