#include "move.h"
#include "main.h"
#include "tim.h"
#include "pid.h"
fp32 PWM1, PWM2;

void Go(void)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 240);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 200);
}
void Stop(void)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);
}
/*б�º���*/

void Turn_left(fp32 angle)
{
	if(Flag == 1)
		{
			Flag = 0;
		  	fp32 ref = MotorSpeed1 - MotorSpeed2;
	fp32 set = angle * (2000 / 120);
	//cha = 2000 ---> 120 degree
	fp32 PWMLf = 0;
	while(ref<set)
	{
	MotorSpeed1 = __HAL_TIM_GET_COUNTER(&htim4) ;//获得10ms采样脉冲cm/s
	MotorSpeed2 = __HAL_TIM_GET_COUNTER(&htim5) ;//获得10ms采样脉冲cm/s
	fp32 step = 10;
	ref = MotorSpeed1 - MotorSpeed2;
	set = angle * (2000 / 120);
	fp32 temp = PID_Calc(&turnleft, ref, set);
	if (PWMLf < temp) {
			PWMLf += (temp - PWMLf) < step
																	 ? (temp - PWMLf)
																	 : step;
	} else {
			PWMLf -= (PWMLf - temp) < step
																	 ? (PWMLf - temp)
																	 : step;
	}
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)PWMLf);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint16_t)PWMLf);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
			    HAL_Delay(20);

}
	if(ref>set)
	{
		 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);	//右轮
	 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);//左轮
	 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	}
		}

  }
	/*fp32 ref = MotorSpeed2 - MotorSpeed1;
	fp32 set = angle * (2500 / 90);
	//cha = 2500 ---> 90 degree
	fp32 PWMLf = 0;
	while(ref<set)
	{
	fp32 step = 10;
	ref = MotorSpeed1 - MotorSpeed2;
	set = angle * (2500 / 90);
	fp32 temp = PID_Calc(&turnleft, ref, set);
	if (PWMLf < temp) {
			PWMLf += (temp - PWMLf) < step
																	 ? (temp - PWMLf)
																	 : step;
	} else {
			PWMLf -= (PWMLf - temp) < step
																	 ? (PWMLf - temp)
																	 : step;
	}
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)PWMLf);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint16_t)PWMLf);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}
	if(ref>set)
	{
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);	    // TIM3_CH1(pwm)
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);	    // TIM3_CH2(pwm)
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);	    // TIM3_CH3(pwm)
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);	    // TIM3_CH4(pwm)
	}
	*/
void Turn_right(fp32 angle)
{
	if(Flag == 1)
		{
			Flag = 0;
		  	fp32 ref = MotorSpeed2 - MotorSpeed1;
	fp32 set = angle * (2000 / 120);
	//cha = 2000 ---> 120 degree
	fp32 PWMRt = 0;
	while(ref<set)
	{
	MotorSpeed1 = __HAL_TIM_GET_COUNTER(&htim4) ;//获得10ms采样脉冲cm/s
	MotorSpeed2 = __HAL_TIM_GET_COUNTER(&htim5) ;//获得10ms采样脉冲cm/s
	fp32 step = 10;
	ref = MotorSpeed2 - MotorSpeed1;
	set = angle * (2000 / 120);
	fp32 temp = PID_Calc(&turnleft, ref, set);
	if (PWMRt < temp) {
			PWMRt += (temp - PWMRt) < step
																	 ? (temp - PWMRt)
																	 : step;
	} else {
			PWMRt -= (PWMRt - temp) < step
																	 ? (PWMRt - temp)
																	 : step;
	}
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint16_t)PWMRt);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint16_t)PWMRt);
			    HAL_Delay(20);

}
	if(ref>set)
	{
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);	    // TIM3_CH1(pwm)
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);	    // TIM3_CH2(pwm)
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);	    // TIM3_CH3(pwm)
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);	    // TIM3_CH4(pwm)

	}
		}

}
void Turn_back(fp32 angle)
{
	if(Flag == 1)
		{
			Flag = 0;
		  	fp32 ref = MotorSpeed2 - MotorSpeed1;
	fp32 set = angle * (2000 / 120);
	//cha = 2000 ---> 120 degree
	fp32 PWMRt = 0;
	while(ref<set)
	{
	MotorSpeed1 = __HAL_TIM_GET_COUNTER(&htim4) ;//获得10ms采样脉冲cm/s
	MotorSpeed2 = __HAL_TIM_GET_COUNTER(&htim5) ;//获得10ms采样脉冲cm/s
	fp32 step = 10;
	ref = MotorSpeed2 - MotorSpeed1;
	set = angle * (2000 / 120);
	fp32 temp = PID_Calc(&turnleft, ref, set);
	if (PWMRt < temp) {
			PWMRt += (temp - PWMRt) < step
																	 ? (temp - PWMRt)
																	 : step;
	} else {
			PWMRt -= (PWMRt - temp) < step
																	 ? (PWMRt - temp)
																	 : step;
	}
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint16_t)PWMRt);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint16_t)PWMRt);
			    HAL_Delay(20);

}
	if(ref>set)
	{
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);	    // TIM3_CH1(pwm)
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);	    // TIM3_CH2(pwm)
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);	    // TIM3_CH3(pwm)
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);	    // TIM3_CH4(pwm)

	}
		}

}

