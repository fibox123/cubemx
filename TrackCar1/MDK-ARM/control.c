#include "control.h"
#include "main.h"
#include "tim.h"
#include "math.h"

//ȫ�ֱ���
float MotorSpeed1,MotorSpeed2;
float SpeedTarget =5;//Ҫ��????
float MotorOutput1,MotorOutput2;
int SumError1 = 0, SumError2 = 0;

/*//ͨ��TIM4��ȡ������岢����
void GetMotorPulse(void)
{
	MotorSpeed = (short)(__HAL_TIM_GET_COUNTER(&htim4)*100/20*2*5);//1s���ߵ�·�̼��ٶ�
	__HAL_TIM_SET_COUNTER(&htim4,0);
}*/

//����ʽPID
int Error_Last1 = 0,Error_Last2 = 0,Error_Prev1 = 0,Error_Prev2 = 0;//��һ�����ֵ�����ϴ����
float Pwm_add1=0,Pwm_add2=0,Pwm1 = 0,Pwm2 = 0;//PWM������PWMռ�ձ�10 10
float Kp = 2.5,Ki = 0.7,Kd = 0;//PIDϵ��

int SpeedInnerControl1(int Speed,int Target)
{
	int Error=Target - Speed;
	if(SumError1 > 100)
	{
		SumError1 = 100;
	}
	Pwm1 = Kp * Error + Ki * SumError1 + Kd * (Error_Last1 - Error_Prev1);
	Error_Prev1 = Error_Last1;
	Error_Last1 = Error;
	SumError1 += Error;
	if(Pwm1 > 100) Pwm1 = 100;//�޷�
	if(Pwm1 < -100) Pwm1 =-100; //�����������ת�٣�Ҫ��????
	
	return Pwm1;
}
int SpeedInnerControl2(int Speed,int Target)
{
	int Error=Target - Speed;
	
	if(SumError2 > 100)
	{
		SumError2 = 100;
	}
	Pwm2 = Kp * Error + Ki * SumError2 + Kd * (Error_Last2 - Error_Prev2)+1;
	Error_Prev2 = Error_Last2;
	Error_Last2 = Error;
	SumError2 += Error;
	if(Pwm2 > 100) Pwm2 = 100;//�޷�
	if(Pwm2 < -100) Pwm2 =-100; //�����������ת�٣�Ҫ��????
	
	return Pwm2;
}
void Go(void)
{
	//Servo_Control(89);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MotorOutput1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MotorOutput2);
}
void Left(void)
{
	Servo_Control(124);//��ת�ĽǶ�
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MotorOutput1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MotorOutput2);
	//HAL_Delay(200);
}
void Right(void)
{
	Servo_Control(54);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MotorOutput1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MotorOutput2);
	//HAL_Delay(200);
}
void Stop(void)
{
	Servo_Control(89);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
}
//angle:�Ƕ�ֵ:0~180
void Servo_Control(unsigned short int angle)
{
	float temp;
	temp = (1.0 / 9.0)*angle + 5.0;//ռ�ձ�ֵ = 1/9 * �Ƕ� + 5
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (unsigned short int)temp);
}
//�����ѹ�ͷ�����ƺ���
/*void SetMotorVoltageAndDirection(int Pwm)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Pwm);
}
*/
