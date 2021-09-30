#include "control.h"
#include "main.h"
#include "tim.h"
#include "math.h"

//ȫ�ֱ���
unsigned int MotorSpeed;
int SpeedTarget = 5000;
int MotorOutput;

//ͨ��TIM4��ȡ������岢����
void GetMotorPulse(void)
{
	MotorSpeed = (short)(__HAL_TIM_GET_COUNTER(&htim4)/18*7200/100);//????
	__HAL_TIM_SET_COUNTER(&htim4,0);
}

//����ʽPID
int Error_Last,Error_Prev;//��һ�����ֵ�����ϴ����
int Pwm_add,Pwm;//PWM������PWMռ�ձ�
float Kp = 2,Ki = 3,Kd = 2;//PIDϵ��

int SpeedInnerControl(int Speed,int Target)
{
	int Error=Speed-Target;
	
	Pwm_add = Kp * (Error - Error_Last) + Ki * Error 
						+ Kd * (Error - 2.0f * Error_Last + Error_Prev) + 1;
	//��һ��Ŀ������������ź�Ϊ0��ϵͳ��ʧ��
	
	Pwm += Pwm_add;//ԭʼ�� + ���� = �����
	
	Error_Prev = Error_Last;
	Error_Last = Error;
	
	if(Pwm > 7100) Pwm = 7100;//�޷�
	if(Pwm < -7100) Pwm =-7100; //�����������ת�٣�Ҫ��????
	
	return Pwm;
}

//�����ѹ�ͷ�����ƺ���
void SetMotorVoltageAndDirection(int Pwm)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Pwm);
}
