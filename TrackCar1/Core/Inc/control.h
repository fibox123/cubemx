#ifndef __CONTROL_H
#define __CONTROL_H


//全局变量
extern float MotorSpeed1,MotorSpeed2;
extern float SpeedTarget;
extern float MotorOutput1,MotorOutput2;
extern float Pwm1,Pwm2;
//函数声明
void GetMotorPulse(void);
int SpeedInnerControl1(int Speed,int Target);
int SpeedInnerControl2(int Speed,int Target);
void SetMotorVoltageAndDirection(int Pwm);
void Go(void);
void Left(void);
void Right(void);
void Stop(void);
void Servo_Control(unsigned short int angle);
	
#endif