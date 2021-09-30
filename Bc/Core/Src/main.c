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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include "bsp_imu.h"
	#include "stdio.h"
	#include "pid.h"
	#include "math.h"
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
fp32 MotorSpeed1 = 0, MotorSpeed2 = 0;//寄存器CNT的值是16位在后面的printf中强转为int
fp32 PWM = 0;
fp32 speed_pid_parameters[] = {10,0.00001,0};// 0.0001
fp32 angle_pid_parameters[] = {0,0,0};//5.8 0 3.41
fp32 site_pid_parameters[] = {0,0,0};//800 0 400
//int16_t Number = 0;
int Target = 10;
fp32 Angle = 0,Dn = 0;
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
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  MPU6050Init();
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	    // TIM3_CH1(pwm)
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);	    // TIM3_CH2(pwm)
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);	    // TIM3_CH3(pwm)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);	    // TIM3_CH4(pwm)
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1); // 电机1开启编码器AB
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2); // 电机1开启编码器AB
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1); // 电机1开启编码器AB
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2); // 电机2开启编码器AB
	PID_Init(&angle, PID_POSITION, angle_pid_parameters, 100, 100);//调？？速度max
  PID_Init(&speed, PID_POSITION, speed_pid_parameters,	100,100);//调？？PWM max
	PID_Init(&site, PID_POSITION, site_pid_parameters, 60000, 60000);//调？？速度max
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		MPU6050ReturnData();
//		if(imu.rol>Angle)
//		{

//		}
//		else if(imu.rol<Angle)
//		{
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);//
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (int)abs(PWM));
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (int)abs(PWM));
//		}
		printf("%d,%f,%f,%f\n",mpu_data.gx,MotorSpeed2,imu.rol,PWM);
		//int16_t PWM = angle.out;
		//int16_t PWM = A_PWM + S_PWM;
			//if(abs(PWM) > 80) PWM = 80;
		}
	HAL_Delay(5);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,HAL_MAX_DELAY);
	return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{//10ms
	if(htim ->Instance == TIM1)
	{
				MotorSpeed1 = __HAL_TIM_GET_COUNTER(&htim2);
				printf("%d,%f,%f,%f\n",mpu_data.gx,MotorSpeed1,imu.rol,PWM);
//			MotorSpeed1 = __HAL_TIM_GET_COUNTER(&htim2);//获得10ms采样脉冲cm/s
//			//Number += __HAL_TIM_GET_COUNTER(&htim2);
//			MotorSpeed1 = MotorSpeed1/48960*2*3.14*65/10*100;
//			MotorSpeed2 = __HAL_TIM_GET_COUNTER(&htim4);//只要测到脉冲就行，不用在定时器xS里获取值相除获取速度
//			MotorSpeed2 = MotorSpeed2/48960*2*3.14*65/10*100;
		  __HAL_TIM_SET_COUNTER(&htim2,0);  // 计数器清零
//		  __HAL_TIM_SET_COUNTER(&htim4,0);  // 计数器清零
//			PID_Calc(&speed, (MotorSpeed1 + MotorSpeed2)/10/2,0);
//			PID_Calc(&angle, imu.rol/2, Angle);
//			PWM = speed.out+ angle.out;
//		  if(abs(PWM)>=100) PWM = 100;
		PWM = 100;
			
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, abs(PWM));
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, abs(PWM));
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
//		if(imu.rol > Angle+Dn)
//		{
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, abs(PWM));
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, abs(PWM));
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
//		}
//		else if(imu.rol < Angle-Dn)
//		{
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, abs(PWM));
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, abs(PWM));
//		}
//			//PID_Calc(&site, Number, 48960/2);//5:角度中值
//		//if(Number>(48960/2))Number = 0;
//			//PID_Calc(&angle, imu.rol, 2);//2:角度中值
//			//PID_Calc(&speed, (MotorSpeed1 + MotorSpeed2)/2,angle.out);//40基本满PWM
//	}
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
