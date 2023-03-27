#include "user_ad.h"

uint32_t AD1_DMA[2] = {0};
uint32_t AD2_DMA[3] = {0};
double V_IN,C_IN,C_OUT,V_CAP,C_CAP,V_OUT;

void ADC_Read()
{
		V_IN = (double)(AD1_DMA[0])*3.3/4096*20;
		C_IN= (double)(AD1_DMA[1])*3.3/4096*5;
		C_OUT = (double)(AD2_DMA[0])*3.3/4096*5;
		V_CAP = (double)(AD2_DMA[1])*3.3/4096*20;
		C_CAP = (double)(AD2_DMA[2])*3.3/4096*2.5;
		//printf("V_IN=%f,C_IN=%f,C_OUT=%f,V_CAP=%f,C_CAP=%f\n",V_IN,C_IN,C_OUT,V_CAP,C_CAP);
}