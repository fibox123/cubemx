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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "math.h"
#include "stdlib.h"
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
//fp32 PID[3]={45,18,0};//速度
//fp32 PID[3]={0,0,0};//角度 注:0 = 8000多当手动调到0的时候会直接突变到8000//5  31.5 8.2 0
int T_Angel = 1000, Target = 500;
PidTypeDef angle_pid,speed_pid;

//float A_kp = 5, A_ki = 0, A_kd = 0;
//float S_kp = 2, S_ki = 0, S_kd = 0;
//float A_Error_Last = 0,A_Error_Prev = 0,A_SumError = 0;
//float S_Error_Last = 0,S_Error_Prev = 0,S_SumError = 0;
fp32 angle_pid_parameters[] = {100,35,0};
fp32 speed_pid_parameters[] = {35,13,0};
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
  MX_CAN1_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	PID_Init(&angle_pid, PID_POSITION, angle_pid_parameters, 2000, 2000);
  PID_Init(&speed_pid, PID_POSITION, speed_pid_parameters, 20000, 20000);
	USER_CAN_Start();
	//PID_Init(&pid,0,PID,20000,20000);//电压限幅（给的电压）10000
	//PID_Init(&pid,PID_POSITION,PID,20000,20000);//电压限幅（给的电压）
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//int Angle = PID_Calc(&pid,motor_1.MechDegree,Target);//Output 电压 Target 角度
		//int16_t Output = C_PID(T_Angel,Target);//Output 电压 Target 速度
		PID_Calc(&angle_pid, motor_1.MechDegree, T_Angel);
		if(T_Angel - motor_1.MechDegree > 4096)
		 {
			 angle_pid.out = T_Angel - motor_1.MechDegree - 8192;
		 }
		 else if (T_Angel - motor_1.MechDegree < -4096)
		 {
				 angle_pid.out = T_Angel - motor_1.MechDegree + 8192;
		 }
		 else
		 {
				 angle_pid.out = T_Angel - motor_1.MechDegree ;
		 }
    PID_Calc(&speed_pid, motor_1.Speed, angle_pid.out);
		printf("%f,%f,%f,%f,%f,%f\n",(double)motor_1.MechDegree,(double)motor_1.Speed,(double)angle_pid.out,(double)motor_1.Temperature,(double)Target,(double)speed_pid.out);
		GM6020_SetCurrent(speed_pid.out);
    HAL_Delay(10);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart6,(uint8_t *)&ch,1,HAL_MAX_DELAY);
	return ch;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim ->Instance == TIM3)
	{
		//T_Angel = (T_Angel + 1000) % 8192;
		T_Angel = (T_Angel + 2000) % 8192;
	}
}
//位置式串级PID
/*int C_PID(int T_Angel, int Target)
{
	int Error;
	float A_Output,S_Output,Ero;
	//error >  4096 对error 减8192
	//error < -4096 对error 加8192
 if(T_Angel - motor_1.MechDegree > 4096)
 {
	 Error = T_Angel - motor_1.MechDegree - 8192;
 }
 else if (T_Angel - motor_1.MechDegree < -4096)
 {
	 	 Error = T_Angel - motor_1.MechDegree + 8192;
 }
 else
 {
	 	 Error = T_Angel - motor_1.MechDegree ;
 }
	A_SumError += Error;
	A_SumError = Limit (A_SumError,6000);
	
	A_Error_Prev = A_Error_Last;
	A_Error_Last = Error;
	
	A_Output = A_kp * Error + A_ki * A_SumError + A_kd * (A_Error_Last - A_Error_Prev);

	Ero = A_Output - Target;
	S_SumError += Ero;
	S_SumError = Limit (S_SumError,200);
	S_Error_Prev = S_Error_Last;
	S_Error_Last = Ero;
	
	S_Output = S_kp * Ero + S_ki * S_SumError + S_kd * (S_Error_Last - S_Error_Prev);
	S_Output = Limit(S_Output, 20000);
	return S_Output;
}
int Limit(float x,int p)
{
	int Out;
	if( x > p )
	{
		return p;
	}
	if( x < -p)
	{
		return -p;
	}
	return x;
}
*/
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
