//
// Created by pc on 2021/7/19.
//

#ifndef UART_FREECOMMUNICATION_H
#define UART_FREECOMMUNICATION_H

#include "stm32f4xx_hal.h"
#include "usart.h"

#define UART_DMA_RECV_UNKNOW_LEN_EN 1

#define MAX_UART_NUM 1
#define MAX_UART_BUFFER_LEN 32

#define UART_DEVICE_LIST {&huart2}

extern uint8_t uart_recv[MAX_UART_NUM][MAX_UART_BUFFER_LEN],
        *uart_recv_p[MAX_UART_NUM],
        uart_buf[MAX_UART_NUM][MAX_UART_BUFFER_LEN];

extern uint16_t recv_count[MAX_UART_NUM];

extern UART_HandleTypeDef *huart_temp;

void UART_FreeCommunication_Init(void);

void UART_DMA_FreeCommunication_Handler(void);

extern void (*User_UART_DMA_RecvCallBack_point)(UART_HandleTypeDef *huart);

void User_UART_DMA_RecvCallBack(UART_HandleTypeDef *huart);

#if UART_DMA_RECV_UNKNOW_LEN_EN

void UART_DMA_RECV_RxCpltCallback(UART_HandleTypeDef *huart);

void UART_DMA_RECV_IDLE(UART_HandleTypeDef *huart, uint16_t UART_Num);

#endif

#endif //GY_56_DEMO_DRIVER_H
