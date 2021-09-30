/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-08-09     RT-Thread    first version
 */

#include <rtthread.h>
#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_config.h"

//ros
#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include "msg_generate/SIRB_Catch.h"
#include "msg_generate/Visual_msg.h"
#include "stm32f4xx.h"

static ros::NodeHandle  nh;
static msg_generate::SIRB_Catch test_catch;

//test_catch.Number = 0;
//test_catch.MotorState = 0, ButtonState = 0;
//test_catch.control = 0, height = 0;
//test_catch.NowHigh = 0;
//test_catch.Flag = 0;

static msg_generate::Visual_msg test_throw;

static ros::Publisher test_pub("test1", &test_catch);


#define LED_PIN GET_PIN(A, 7)
rt_uint32_t period, pulse;
#define PWM_DEV_NAME        "pwm2"  /* PWM设备名称 */
#define PWM_DEV_CHANNEL     2      /* PWM通道 */
struct rt_device_pwm *pwm_dev;      /* PWM设备句柄 */
#define PWM_PIN    GET_PIN(C, 5)   /*PWM脉冲计数、PC5下降沿触发外部中断计数*/


static int Number = 0;
static int MotorState = 0, ButtonState = 0;
static float control = 50, height = 0;
static float NowHigh = 0;
static int Flag = 0;

static void messageCb( const msg_generate::Visual_msg& test_throw){
  //rt_pin_write(LED0_PIN, PIN_HIGH - rt_pin_read(LED0_PIN));   // blink the led
    Flag = test_throw.flag;
    control = test_throw.control;
    rt_kprintf("recv");
}

static ros::Subscriber<msg_generate::Visual_msg> test_sub("Visual_msg", messageCb);


void HAL_TIM_PWM_PulseFinishedCallback(void *args)
{
    rt_interrupt_enter();
     Number++;

/*这里发送Number和NowHigh的数值*/
     NowHigh = (float)Number/800*2*3.14*4.8;//cm
     rt_kprintf("Number = %d\n", Number);
     rt_kprintf("NowHigh = %d\n", (int)NowHigh);
     if(Flag == 0)
     {
         if(NowHigh < control)
         {
            rt_pin_write(LED_PIN, PIN_LOW);//上升
            MotorState = 1;
/*这里发送MotorState的数值*/
         }
         if(NowHigh >= control)
         {
             rt_pwm_disable(pwm_dev, PWM_DEV_CHANNEL);
//             rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 0, 0);
             MotorState = 0;
/*这里发送MotorState的数值*/
         }
     }

     rt_interrupt_leave();
}



static void rosserial_thread_entry(void *parameter)
{

    //Init node>
    nh.initNode();

    // 发布了一个话题
    nh.advertise(test_pub);
    nh.subscribe(test_sub);

    while(1)
    {
        /*这里接收control和Flag的数值*/

//        rt_kprintf("[rosserial] send o`ne\n");
        test_catch.control = control;
        test_catch.Flag = Flag;
        test_catch.Number = Number;
        test_pub.publish( &test_catch );

        if(Flag == 1)
            {
                rt_kprintf("START PWM\n");

                rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);
                rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 1000000, 300000);

       /*这里发送ButtonState的数值*/
                if(Number > height)//加碰撞开关后在下降这里改demo
                 {
                    rt_pin_write(LED_PIN, PIN_HIGH);//下降
                    MotorState = 2;
                 }
                 if(Number <= height)
                 {
                     rt_pwm_disable(pwm_dev, PWM_DEV_CHANNEL);
       //              rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 0, 0);
                     MotorState = 0;
                 }
            }

        nh.spinOnce();
        rt_thread_delay(100);

    }
}



int main(void)
{
        rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);
        rt_pin_mode(PWM_PIN, PIN_MODE_INPUT_PULLUP);
        rt_pin_write(LED_PIN, PIN_HIGH);
       /* 查找PWM设备 */
        pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
       if (pwm_dev == RT_NULL)
       {
           rt_kprintf("pwm dev %s not found!\n", PWM_DEV_NAME);
           return RT_ERROR;
       }
       /* 设置PWM周期和脉冲宽度默认值 */
       rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 1000000, 1000000);//1000000
       /* 使能设备 */
       rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);


       // 启动一个线程用来和 ROS 通信
       rt_thread_t thread = rt_thread_create("rosserial",     rosserial_thread_entry, RT_NULL, 2048, 8, 10);
       if(thread != RT_NULL)
       {
           rt_thread_startup(thread);
           rt_kprintf("[rosserial] New thread rosserial\n");
       }
       else
       {
           rt_kprintf("[rosserial] Failed to create thread rosserial\n");
       }
       /* 绑定中断，上升沿模式，回调函数名为IQR_HANDALE_PWM */
//       rt_pin_attach_irq(PWM_PIN, PIN_IRQ_MODE_FALLING, IQR_HANDALE_PWM, RT_NULL);
       /* 使能中断 */
//       rt_pin_irq_enable(PWM_PIN, PIN_IRQ_ENABLE);

       return RT_EOK;
}

