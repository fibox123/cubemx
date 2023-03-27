#include "main.h"
#include "pid.h"
#include "tim.h"
#include "bno055.h"
#include "Bno.h"

  /*斜坡函数*/
void ramp_cale(fp32 next, fp32 last, fp32 step)
{
    if (next < last) {
        next += (last - next) < step
                                     ? (last - next)
                                     : step;
    } else {
        next -= (next - last) < step
                                     ? (next - last)
                                     : step;
    }
}
/*Y0:Yaw轴初始值*//*Y:当前Yaw值*/
void turnleft(fp32 angle)
{
	 fp32 PWMLf;
	 if(Y0 - angle > 200) 	{	Y -= 400;	}
	 else if(Y0 - angle < -200) { angle -= 400; };
   fp32 temp = PID_Calc(&speed1, Y,angle);
	 ramp_cale(PWMLf,temp, 10);
   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t)PWMLf);
   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint16_t)PWMLf);
   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

}
void turnright(fp32 angle)
{
	 fp32 PWMRt; 
	 if(Y0 - angle > 200) 	{	Y -= 400;	}
	 else if(Y0 - angle < -200) { angle -= 400; };
	 fp32 temp = PID_Calc(&speed1, Y,angle);
	 ramp_cale(PWMRt,temp, 10);
   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)PWMRt);
   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint16_t)PWMRt);
   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}
