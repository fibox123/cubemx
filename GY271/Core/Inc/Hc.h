#ifndef __HC_H
#define __HC_H

#define    Write_Address     0x3C      //定义器件5883在IIC总线中的从地址
#define    Read_Address    0x3D      //地址读
#define    Config_RegA       0x00
#define    Config_RegB      0x01
#define    Mode_Reg          0x02
#define X_MSB                 0x03      //数据寄存器地址
#define X_LSB            0x04
#define Y_MSB            0x07
#define Y_LSB            0x08
#define Z_MSB            0x05
#define Z_LSB            0x06
typedef unsigned char uint8_t;

void HcInit();
uint8_t ReadReg_HMC5883(uint8_t Hc_Address);

#endif