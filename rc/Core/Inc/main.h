/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
		int16_t motor1;
		int16_t motor2;
		int16_t motor3;
		int16_t motor4;
}RC;
		typedef  struct
{
 struct
 {
 uint16_t ch0;
 uint16_t ch1;
 uint16_t ch2;
 uint16_t ch3;
 uint8_t s1;
 uint8_t s2;
 }rc;
 struct
 {
 int16_t x;
 int16_t y;
 int16_t z;
 uint8_t press_l;
 uint8_t press_r;
 }mouse;
 struct
 {
 uint16_t v;
 }key;
}RC_Ctl_t;
void criticality();
extern uint32_t uw_Tick;
extern uint8_t receive_buff[255];
extern RC rc_move;
extern RC_Ctl_t RC_CtrlData;
extern int16_t Output1,Output2,Output3,Output4,s1,s2,s3,s4;
extern void	Motor_Begin();
extern void	slope();



/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void RemoteDataProcess(uint8_t *pData);
//void Motor_Begin();
void slope();
extern fp32 speed_pid_parameters1[];
extern fp32 speed_pid_parameters2[];
extern fp32 speed_pid_parameters3[];
extern fp32 speed_pid_parameters4[];
extern fp32 angle_pid_parametersx[];//Ò£¿ØÆ÷ºÍ6020´®¼¶PID
extern fp32 speed_pid_parametersx[];
extern fp32 angle_pid_parameters_a[];
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
