#include "Vl.h"
#include "usart.h"
#include "main.h"

uint16_t distance;
uint8_t temperature;
uint8_t RxFlag;
/* 接收数据 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        RxFlag = 1;
        HAL_UART_Receive_IT(&huart1,(uint8_t *)recv, 9);
    }
}
void DataHandle()
{ 
    distance = (recv[4]<<8) | recv[5];
    temperature = recv[7];
}