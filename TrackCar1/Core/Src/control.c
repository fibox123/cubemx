#include "control.h"
#include "main.h"
#include "tim.h"
#include "math.h"

//全局变量
unsigned int MotorSpeed;
int SpeedTarget = 5000;
int MotorOutput;

//通过TIM4读取电机脉冲并计数
void GetMotorPulse(void)
{
	MotorSpeed = (short)(__HAL_TIM_GET_COUNTER(&htim4)/18*7200/100);//????
	__HAL_TIM_SET_COUNTER(&htim4,0);
}

//增量式PID
int Error_Last,Error_Prev;//上一次误差值，上上次误差
int Pwm_add,Pwm;//PWM增量，PWM占空比
float Kp = 2,Ki = 3,Kd = 2;//PID系数

int SpeedInnerControl(int Speed,int Target)
{
	int Error=Speed-Target;
	
	Pwm_add = Kp * (Error - Error_Last) + Ki * Error 
						+ Kd * (Error - 2.0f * Error_Last + Error_Prev) + 1;
	//加一的目的是如果输入信号为0，系统将失控
	
	Pwm += Pwm_add;//原始量 + 增量 = 输出量
	
	Error_Prev = Error_Last;
	Error_Last = Error;
	
	if(Pwm > 7100) Pwm = 7100;//限幅
	if(Pwm < -7100) Pwm =-7100; //电机输出有最大转速，要调????
	
	return Pwm;
}

//电机电压和方向控制函数
void SetMotorVoltageAndDirection(int Pwm)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Pwm);
}
