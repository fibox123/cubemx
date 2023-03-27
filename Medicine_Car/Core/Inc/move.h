#ifndef _MOVE_H
#define _MOVE_H

void Go(void);
void Stop(void);
typedef float fp32;
extern fp32 PWM1, PWM2;
extern void Turn_right(fp32 angle);
extern void Turn_left(fp32 angle);
extern void Turn_back(fp32 angle);
#endif
