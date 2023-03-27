//
// Created by pc on 2021/7/19.
//

#include "UART_FreeCommunication.h"

#include "usart.h"

#include "stdio.h"

#include "string.h"

#include "stdlib.h"

uint8_t uart_recv[MAX_UART_NUM][MAX_UART_BUFFER_LEN],
        *uart_recv_p[MAX_UART_NUM],
        uart_buf[MAX_UART_NUM][MAX_UART_BUFFER_LEN];

uint16_t recv_count[MAX_UART_NUM];

void (*User_UART_DMA_RecvCallBack_point)(UART_HandleTypeDef *huart) = NULL;

UART_HandleTypeDef *huart_temp = NULL;
UART_HandleTypeDef *huart_list[MAX_UART_NUM] = UART_DEVICE_LIST;

uint16_t move_message = 410;

void UART_FreeCommunication_Init() {
//    Use this function to Initializes UART FreeCommunication.
    for (int i = 0; i < MAX_UART_NUM; ++i) {
        __HAL_UART_ENABLE_IT(huart_list[i], UART_IT_IDLE);
        HAL_UART_Receive_DMA(huart_list[i], uart_recv[i], 1);
    }
}

void UART_DMA_FreeCommunication_Handler()
//    Use this function to ACT UART Message Getting.
{
    if (User_UART_DMA_RecvCallBack_point) {
        User_UART_DMA_RecvCallBack_point(huart_temp);
    }
}
//在Maxipy接收字符串str a[4]
//a[4]相当于uart_buf[0]
void User_UART_DMA_RecvCallBack(UART_HandleTypeDef *huart)
//    Change and use this function for your user to realize getting UART Message.
{
    User_UART_DMA_RecvCallBack_point = NULL;
    huart_temp = NULL;
    if (huart == &huart2) {
			printf("%s",uart_buf[0]);
			
    }

}


#if UART_DMA_RECV_UNKNOW_LEN_EN

void UART_DMA_RECV_RxCpltCallback(UART_HandleTypeDef *huart)
//    Please add this function into CallBack: void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    for (int i = 0; i < MAX_UART_NUM; ++i) {
        if (huart_list[i] == huart) {
            HAL_UART_DMAStop(huart);
            uart_recv_p[i]++;
            recv_count[i]++;
            HAL_UART_DMAResume(huart);
            HAL_UART_Receive_DMA(huart, uart_recv_p[i], 1);
            break;
        }
    }
}

void UART_DMA_RECV_IDLE(UART_HandleTypeDef *huart, uint16_t UART_Num)
//    Please add this function into IRQHandler: void USARTx_IRQHandler(void)
{

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        HAL_UART_DMAStop(huart);

        for (int i = 0; i < recv_count[UART_Num]; ++i) {
            uart_buf[UART_Num][i] = uart_recv[UART_Num][i];
        }
        for (int i = recv_count[UART_Num]; i < MAX_UART_BUFFER_LEN; ++i) {
            uart_buf[UART_Num][i] = 0;
        }

        huart_temp = huart;
        User_UART_DMA_RecvCallBack_point = User_UART_DMA_RecvCallBack;

        for (int i = 0; i < recv_count[UART_Num]; ++i) {
            uart_recv[UART_Num][i] = 0;
        }

        uart_recv_p[UART_Num] = uart_recv[UART_Num];
        recv_count[UART_Num] = 0;
        HAL_UART_Receive_DMA(huart, uart_recv[UART_Num], 1);
    }
}

#endif
