#ifndef __USER_HRTIM_H
#define __USER_HRTIM_H

#include "main.h"

void Charge_Power(uint16_t P_use);
void Charge_mode(uint16_t P_u);
void Discharge_mode();
extern void PWM_Compare();
extern void PWM_Output(uint16_t *Duty_Cycle);

#endif