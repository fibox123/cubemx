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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"

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

int Angle_P = 5630, T_Speed = 0;

RC_Ctl_t RC_CtrlData = {
	.rc = {
		.ch1 = 1024,
		.ch0 = 1024,
		.ch2 = 1024,
	}
};

fp32 speed_pid_parameters1[] = {13,1,0};
fp32 speed_pid_parameters2[] = {13,1,0};
fp32 speed_pid_parameters3[] = {13,1,0};
fp32 speed_pid_parameters4[] = {13,1,0};
fp32 angle_pid_parametersx[] = {0.5,0,0.05};//ң������6020����PID
fp32 speed_pid_parametersx[] = {190,10,0};//ң������6020����PID
fp32 angle_pid_parameters_a[] = {10,0,0};//6020�͵��̽ǶȻ� 
																				//���ܸ�����������һֱ��ƫ��Ȼ��һֱת

uint8_t receive_buff[255];
uint32_t uw_Tick;
RC rc_move;
int16_t Output1,Output2,Output3,Output4,s1,s2,s3,s4;

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
	s1 = (RC_CtrlData.rc.ch0-1024)*2 * 3 +(RC_CtrlData.rc.ch1-1024)*2 * 3;
	s2 = (RC_CtrlData.rc.ch0-1024)*2 * -3 +(RC_CtrlData.rc.ch1-1024)*2 * 3;
	s3 = (RC_CtrlData.rc.ch0-1024)*2 * -3 +(RC_CtrlData.rc.ch1-1024)*2 * -3;
	s4 = (RC_CtrlData.rc.ch0-1024)*2 * 3 +(RC_CtrlData.rc.ch1-1024)*2 * -3;
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
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	uw_Tick = HAL_GetTick();
	Pid_Init();
	USER_CAN_Start();
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)receive_buff, 255); 
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		slope();
		Motor_Begin();
		pid();
		PID_Calc(&angle_pid, motor_x.MechDegree, (1024 - RC_CtrlData.rc.ch2) + 5630);
		//PID_Calc(&Angle_pid, motor_y.MechDegree, Angle_Y);
		//criticality();//�Ƕ��ٽ�
	  PID_Calc(&speed_pid, motor_x.Speed, angle_pid.out);	 
		//PID_Calc(&Speed_pid, motor_y.Speed, Angle_pid.out);
		if(HAL_GetTick() - uw_Tick > 50)//���ң��������80ms��ֵ��ʼֵ
		{
			RC_CtrlData.rc.ch0 = 1024;
			RC_CtrlData.rc.ch1 = 1024;	
			RC_CtrlData.rc.ch2 = 1024;
		}
		GM_SetCurrent(0x1FF,speed_pid.out,0,0,0);
		PID_Calc(&a_pid, motor_x.MechDegree, Angle_P);
		GM_SetCurrent(0x200,Output1,Output2,Output3,Output4);
		//PID_Calc(&s_pid, motor_x.Speed, 0);	
		//GM_SetCurrent(0x1FF,s_pid.out,0,0,0);
		printf("%f,%f,%f,%f,%f,%f\n",(double)motor_1.Speed,(double)motor_2.Speed,(double)motor_3.Speed,
						(double)motor_4.Speed,(double)T_Speed,(double)motor_x.MechDegree);
		HAL_Delay(1);
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
void Motor_Begin()
{
	rc_move.motor1 = s1 + a_pid.out;
	rc_move.motor2 = s2 + a_pid.out;
	rc_move.motor3 = s3 + a_pid.out;
	rc_move.motor4 = s4 + a_pid.out;
  //rc_move.motor1 = s1 + (RC_CtrlData.rc.ch2-1024) * 3;
	//rc_move.motor2 = s1 + (RC_CtrlData.rc.ch2-1024) * 3;
	//rc_move.motor3 = s3 + (RC_CtrlData.rc.ch2-1024) * 3;
	//rc_move.motor4 = s4 + (RC_CtrlData.rc.ch2-1024) * 3;
}
void criticality()
{
	if(Angle_P - motor_x.MechDegree > 4096)
		 {
			 angle_pid.out = Angle_P - motor_x.MechDegree - 8192;
		 }
		 else if (Angle_P - motor_x.MechDegree < -4096)
		 {
				 angle_pid.out = Angle_P - motor_x.MechDegree + 8192;
		 }
		 else
		 {
				 angle_pid.out = Angle_P - motor_x.MechDegree ;
		 }
}
void slope()
{
	//����
	if(s1 - ((RC_CtrlData.rc.ch0-1024)*2 * 3 +(RC_CtrlData.rc.ch1-1024)*2 * 3) < 0)
	{
		if(s1 - ((RC_CtrlData.rc.ch0-1024)*2 * 3 +(RC_CtrlData.rc.ch1-1024)*2 * 3) >= -80)
		{
			s1 = (RC_CtrlData.rc.ch0-1024)*2 * 3 +(RC_CtrlData.rc.ch1-1024)*2 * 3;
		}
		else
		{
			s1 += 80;
		}
	}
	if(s1 - ((RC_CtrlData.rc.ch0-1024)*2 * 3 +(RC_CtrlData.rc.ch1-1024)*2 * 3) > 0)
	{
		if(s1 - ((RC_CtrlData.rc.ch0-1024)*2 * 3 +(RC_CtrlData.rc.ch1-1024)*2 * 3) <= 80)
		{
			s1 = (RC_CtrlData.rc.ch0-1024)*2 * 3 +(RC_CtrlData.rc.ch1-1024)*2 * 3;
		}
		else
		{
			s1 -= 80;
		}
	}
	if(s2 - ((RC_CtrlData.rc.ch0-1024)*2 * -3 +(RC_CtrlData.rc.ch1-1024)*2 * 3) < 0)
	{
		if(s2 - ((RC_CtrlData.rc.ch0-1024)*2 * -3 +(RC_CtrlData.rc.ch1-1024)*2 * 3) >= -80)
		{
			 s2 = (RC_CtrlData.rc.ch0-1024)*2 * -3 +(RC_CtrlData.rc.ch1-1024)*2 * 3;
		}
		else
		{
			s2 += 80;
		}
	}
	if(s2 - ((RC_CtrlData.rc.ch0-1024)*2 * -3 +(RC_CtrlData.rc.ch1-1024)*2 * 3) > 0)
	{
		if(s2 - ((RC_CtrlData.rc.ch0-1024)*2 * -3 +(RC_CtrlData.rc.ch1-1024)*2 * 3) <= 80)
		{
			s2 = (RC_CtrlData.rc.ch0-1024)*2 * -3 +(RC_CtrlData.rc.ch1-1024)*2 * 3;
		}
		else
		{
			s2 -= 80;
		}
	}
	if(s3 - ((RC_CtrlData.rc.ch0-1024)*2 * -3 +(RC_CtrlData.rc.ch1-1024)*2 * -3) < 0)
	{
		if(s3 - ((RC_CtrlData.rc.ch0-1024)*2 * -3 +(RC_CtrlData.rc.ch1-1024)*2 * -3) >= -80)
		{
			s3 = (RC_CtrlData.rc.ch0-1024)*2 * -3 +(RC_CtrlData.rc.ch1-1024)*2 * -3;
		}
		else
		{
			s3 += 80;
		}
	}
	if(s3 - ((RC_CtrlData.rc.ch0-1024)*2 * -3 +(RC_CtrlData.rc.ch1-1024)*2 * -3) > 0)
	{
		if(s3 - ((RC_CtrlData.rc.ch0-1024)*2 * -3 +(RC_CtrlData.rc.ch1-1024)*2 * -3) <= 80)
		{
			s3 = (RC_CtrlData.rc.ch0-1024)*2 * -3 +(RC_CtrlData.rc.ch1-1024)*2 * -3;
		}
		else
		{
			s3 -= 80;
		}
	}
	if(s4 - ((RC_CtrlData.rc.ch0-1024)*2 * 3 +(RC_CtrlData.rc.ch1-1024)*2 * -3) < 0)
	{
		if(s4 - ((RC_CtrlData.rc.ch0-1024)*2 * 3 +(RC_CtrlData.rc.ch1-1024)*2 * -3) >= -80)
		{
			s4 = (RC_CtrlData.rc.ch0-1024)*2 * 3 +(RC_CtrlData.rc.ch1-1024)*2 * -3;
		}
		else
		{
			s4 += 80;
		}
	}
	if(s4 - ((RC_CtrlData.rc.ch0-1024)*2 * 3 +(RC_CtrlData.rc.ch1-1024)*2 * -3) > 0)
	{
		if(s4 - ((RC_CtrlData.rc.ch0-1024)*2 * 3 +(RC_CtrlData.rc.ch1-1024)*2 * -3) <= 80)
		{
			s4 = (RC_CtrlData.rc.ch0-1024)*2 * 3 +(RC_CtrlData.rc.ch1-1024)*2 * -3;
		}
		else
		{
			s4 -= 80;
		}
	}
}

void RemoteDataProcess(uint8_t *pData)
{
 if(pData == NULL)
 {
		return;
 }

 RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
 RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5))
& 0x07FF;
 RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
 ((int16_t)pData[4] << 10)) & 0x07FF;
 RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) &
0x07FF;

 RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
 RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);
 RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
 RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
 RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
 RC_CtrlData.mouse.press_l = pData[12];
 RC_CtrlData.mouse.press_r = pData[13];
 RC_CtrlData.key.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8);
 //your control code ��.
}
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart6,(uint8_t *)&ch,1,HAL_MAX_DELAY);
	return ch;
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
