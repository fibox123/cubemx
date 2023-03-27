/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "pid.h"
//PidTypeDef pid;
PidTypeDef speed_pid1,speed_pid2,speed_pid3,speed_pid4,angle_pid,speed_pid,a_pid,s_pid;//P,Y

#define LimitMax(input,max) 	\
 {                        	 	\
        if (input > max)      \
        {                     \
            input = max;      \
        }                     \
        else if (input < -max)\
        {                     \
            input = -max;     \
        }                     \
    }

void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
		
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
void 	Pid_Init()
{
	PID_Init(&speed_pid1, PID_POSITION, speed_pid_parameters1, 20000, 20000);
	PID_Init(&speed_pid2, PID_POSITION, speed_pid_parameters2, 20000, 20000);
  PID_Init(&speed_pid3, PID_POSITION, speed_pid_parameters3, 20000, 20000);
  PID_Init(&speed_pid4, PID_POSITION, speed_pid_parameters4, 20000, 20000);
	PID_Init(&angle_pid, PID_POSITION, angle_pid_parametersx, 200, 200);//角度环输出速度限幅300转
  PID_Init(&speed_pid, PID_POSITION, speed_pid_parametersx, 20000, 20000);
	PID_Init(&a_pid, PID_POSITION, angle_pid_parameters_a, 3000, 3000);
}
void 	pid()
{
		Output1 = PID_Calc(&speed_pid1, motor_1.Speed, rc_move.motor1);
		Output2 = PID_Calc(&speed_pid2, motor_2.Speed, rc_move.motor2);
		Output3 = PID_Calc(&speed_pid3, motor_3.Speed, rc_move.motor3);
		Output4 = PID_Calc(&speed_pid4, motor_4.Speed, rc_move.motor4);
}

