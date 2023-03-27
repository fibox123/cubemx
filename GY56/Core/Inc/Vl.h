#ifndef _VL_H
#define _VL_H

#define Byte0 0x5A //帧头标志 
#define Byte1 0x5A //帧头标志 
#define Byte2 0x15 //本帧数据类型
#define Byte3 0x03// 数据量
void DataHandle();
typedef unsigned short     int uint16_t;
typedef unsigned          char uint8_t;

extern uint16_t distance;
extern uint8_t temperature;
extern uint8_t RxFlag;

#endif
