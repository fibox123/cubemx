#include "Hc.h"
#include "i2c.h"

uint8_t add = 0x00;
void HcInit()
{
    HAL_I2C_Mem_Write(&hi2c1, Write_Address, Config_RegA,I2C_MEMADD_SIZE_8BIT,&add,1,0xffff);
    HAL_Delay(50);
    HAL_I2C_Mem_Write(&hi2c1, Write_Address, Config_RegB, I2C_MEMADD_SIZE_8BIT,&add,1,0xffff);
    HAL_Delay(50);
    HAL_I2C_Mem_Write(&hi2c1, Write_Address, Mode_Reg, I2C_MEMADD_SIZE_8BIT,&add,1,0xffff);
    HAL_Delay(50);
}
uint8_t ReadReg_HMC5883(uint8_t Hc_Address)
{
    uint8_t temp;
    HAL_I2C_Mem_Read(&hi2c1, Read_Address , Hc_Address, I2C_MEMADD_SIZE_8BIT, &temp, 1, 0xffff);
    return temp;
}
