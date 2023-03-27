#ifndef __User_I2C_H
#define __User_I2C_H


#define USER_I2C_SCL_PIN  GPIO_PIN_1
#define USER_I2C_SDA_PIN  GPIO_PIN_0
#define USER_I2C_GPIO     GPIOF
#define USER_I2C          I2C2

#define I2C_SCL_H() 		HAL_GPIO_WritePin(USER_I2C_GPIO, USER_I2C_SCL_PIN, GPIO_PIN_SET);
#define I2C_SCL_L()			HAL_GPIO_WritePin(USER_I2C_GPIO, USER_I2C_SCL_PIN, GPIO_PIN_RESET);

#define I2C_SDA_H()			HAL_GPIO_WritePin(USER_I2C_GPIO, USER_I2C_SDA_PIN, GPIO_PIN_SET);
#define I2C_SDA_L()			HAL_GPIO_WritePin(USER_I2C_GPIO, USER_I2C_SDA_PIN, GPIO_PIN_RESET);

#define I2C_SCL_IN()		HAL_GPIO_ReadPin(USER_I2C_GPIO, USER_I2C_SCL_PIN)
#define I2C_SDA_IN()		HAL_GPIO_ReadPin(USER_I2C_GPIO, USER_I2C_SDA_PIN)

#define GPIO_Mode_IPU 0x00000001U

typedef enum
{
	I2C_OK = 0U,
	I2C_BUSY = 1U,
	I2C_TIMEOUT = 3U,
	I2C_ERROR = 4U,
}__I2C_Status_TypeDef;

typedef enum
{
	I2C_ACK = 0U,
	I2C_NACK = 1U,
}__I2C_ACK_TypeDef;

typedef enum
{
	I2C_Transmitter = 0U,
	I2C_Receiver = 1U,
}__I2C_Oper_TypeDef;

typedef enum
{
	I2C_SDA_IN = 0U,
	I2C_SDA_OUT = 1U,
}__I2C_SDAMode_TypeDef;
/*************************************************
Function: User_I2C_Init
Description: config I2C 
*************************************************/
void User_I2C_Init(void);

/*************************************************
Function: delay
Description: 
Input:  Delaycnt - delay cnt
*************************************************/
void delay(uint32_t Delaycnt);

/*************************************************
Function: I2C_SendBytes
Description: I2C SendBytes
*************************************************/
__I2C_Status_TypeDef I2C_SendBytes(uint8_t SlaveAddr, uint16_t RegAddr, uint8_t *TxBuf, uint8_t OperLen, uint32_t Timeout);

/*************************************************
Function: I2C_RecvBytes
Description: I2C SendBytes
*************************************************/
__I2C_Status_TypeDef I2C_RecvBytes(uint8_t SlaveAddr, uint16_t RegAddr, uint8_t *Rxbuf, uint8_t OperLen, uint32_t Timeout);
#endif /* __User_I2C_H */



