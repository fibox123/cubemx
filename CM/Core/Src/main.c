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
uint16_t MotorSpeed1 = 0, MotorSpeed2 = 0;//寄存器CNT的值是16位在后面的printf中强转为int
fp32 PWM1 = 0,PWM2 = 0;
fp32 speed_pid_parameters1[] = {8,0.001,0};//8 0.001 0
fp32 speed_pid_parameters2[] = {8,0.001,0};//左轮调i1 0.1 0 
fp32 angle1_pid_parameters[] = {8,0,50};//50
fp32 angle2_pid_parameters[] = {8,0,50};//50
fp32 site1_pid_parameters[] = {1,0,0};//800 0 400
fp32 site2_pid_parameters[] = {0,0,0};//800 0 400
fp32 Angle = -9;// 0.405 -2.785
uint8_t Front = 2;
uint8_t Flag = 0;
fp32 kp = 1;
fp32 PWM = 0;
uint8_t Num1 = 0, Num2 = 0;
//fp32 N = 10;//圈数
//uint16_t circle = 280;//一圈的编码器计数值
//前进给3度ok
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
  MX_I2C3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	MPU6050Init();
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);	    // TIM3_CH1(pwm)
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);	    // TIM3_CH2(pwm)
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);	    // TIM3_CH3(pwm)
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);	    // TIM3_CH4(pwm)
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1); // 电机1开启编码器AB
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2); // 电机1开启编码器AB
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1); // 电机1开启编码器AB
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2); // 电机2开启编码器AB	
	__HAL_TIM_SET_COUNTER(&htim3,100);  
	__HAL_TIM_SET_COUNTER(&htim4,100);  
  PID_Init(&site1, PID_POSITION, site1_pid_parameters, 1000, 1000);//调？？速度max
	PID_Init(&site2, PID_POSITION, site2_pid_parameters, 1000, 1000);//调？？速度m
	PID_Init(&angle1, PID_POSITION, angle1_pid_parameters, 1000, 1000);//调？？速度max
  PID_Init(&angle2, PID_POSITION, angle2_pid_parameters, 1000, 1000);//调？？速度max
  PID_Init(&speed1, PID_POSITION, speed_pid_parameters1,	1000,1000);//调？？PWM max
  PID_Init(&speed2, PID_POSITION, speed_pid_parameters2,	1000,1000);//调？？PWM max

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		MPU6050ReturnData();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/*定时器*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{//10ms
	if(htim ->Instance == TIM1)
	{
		MotorSpeed1 = __HAL_TIM_GET_COUNTER(&htim4);//获得10ms采样脉冲cm/s
		MotorSpeed2 = __HAL_TIM_GET_COUNTER(&htim3);//获得10ms采样脉冲cm/s
		PWM = PID_Calc(&site1,MotorSpeed2-MotorSpeed1,10);
		x();
		__HAL_TIM_SET_COUNTER(&htim4,100);  // 计数器清零
  	__HAL_TIM_SET_COUNTER(&htim3,100);  // 计数器清零
		printf("%d,%d,%f,%f\r\n",MotorSpeed1,MotorSpeed2,angle1.out,imu.pit);//右偏正 右轮是1
	}
}
void x(void)
{
  PID_Calc(&angle1, imu.rol,Front+Angle);// 给偏差+3前进
	PID_Calc(&angle2, imu.rol,Front+Angle);// 
  if(angle1.out<0)
		{
			PWM1 = PID_Calc(&speed1, MotorSpeed1-100,-angle1.out-(uint8_t)PWM);//右轮 = 1
			PWM2 = PID_Calc(&speed2, MotorSpeed2-100,-angle2.out+(uint8_t)PWM);	//左轮 = 2
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, (uint16_t)PWM2);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, (uint16_t)PWM1);
		}
		else
		{
			PWM1 = PID_Calc(&speed1, MotorSpeed1-100,angle1.out-(uint8_t)PWM);	//右轮 = 1
			PWM2 = PID_Calc(&speed2, MotorSpeed2-100,angle2.out+(uint8_t)PWM);	//左轮 = 2
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, (uint16_t)PWM2);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, (uint16_t)PWM1);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
		
		}
}
void Go(void)
{
	PID_Calc(&angle1, imu.rol,Front+Angle);// 给偏差+3前进
	PID_Calc(&angle2, imu.rol,Front+Angle);// 
  if(angle1.out<0)
		{
			PWM1 = PID_Calc(&speed1, MotorSpeed1-100,-angle1.out);//右轮 = 1
			PWM2 = PID_Calc(&speed2, MotorSpeed2-100,-angle2.out);	//左轮 = 2
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, (uint16_t)PWM2);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, (uint16_t)PWM1);
		}
		else
		{
			PWM1 = PID_Calc(&speed1, MotorSpeed1-100,angle1.out);	//右轮 = 1
			PWM2 = PID_Calc(&speed2, MotorSpeed2-100,angle2.out);	//左轮 = 2
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, (uint16_t)PWM2);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, (uint16_t)PWM1);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
		
		}
}
void Left(void)
{
  	PID_Calc(&angle1, imu.rol,Angle);// 给偏差+3前进
		PID_Calc(&angle2, imu.rol,Angle);// 
    if(angle1.out<0)
		{
			PWM1 = PID_Calc(&speed1, MotorSpeed1-100,-angle1.out);//右轮 = 1
			PWM2 = PID_Calc(&speed2, MotorSpeed2-100,-angle2.out);	//左轮 = 2
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, (uint16_t)PWM2);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, (uint16_t)PWM1);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
		}
		else
		{
			PWM1 = PID_Calc(&speed1, MotorSpeed1-100,angle1.out);	//右轮 = 1
			PWM2 = PID_Calc(&speed2, MotorSpeed2-100,angle2.out);	//左轮 = 2
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, (uint16_t)PWM2);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, (uint16_t)PWM1);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
		
		}
}
void Right(void)
{
    PID_Calc(&angle1, imu.rol,Angle);// 给偏差+3前进
		PID_Calc(&angle2, imu.rol,Angle);// 
		PWM1 = PID_Calc(&speed1, MotorSpeed1-100,angle1.out-(uint8_t)PWM);//右轮 = 1
		PWM2 = PID_Calc(&speed2, MotorSpeed2-100,angle2.out+(uint8_t)PWM);	//左轮 = 2
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, (uint16_t)PWM2);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, (uint16_t)PWM1);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
}
void Judge(void)
{	
		if(Flag == 2)
		{
			Right();
		}
		if((HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==RESET))
		{
			Flag = 2;
			Right();
			HAL_Delay(5);
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
