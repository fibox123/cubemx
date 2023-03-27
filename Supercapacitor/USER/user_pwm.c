#include "user_pwm.h"

uint16_t Duty_Cycle[4] = {0}; // 0:BATH,1:BATL,2:CAPH,3.CAPL
/*额定、实际、可用、底盘消耗功率、额定电流、电容端经过BB电路后的输出电压、增益电压*/
uint16_t P_rated, P_real, P_usable, P_OUT, I_rated, V_COUT, V_gain;

void PWM_Compare()
{
    P_real = V_IN * C_OUT;
    P_usable = P_rated - P_real;
    if (P_rated < P_real)
    {
        Charge_Power(P_usable);
    }
    else if (P_rated >= P_real)
    {
        Discharge_mode();
    }
}
void Charge_Power(uint16_t P_use)
{
    uint16_t Px;
    if (P_use > 20 && P_use < 50)
        Px = 20;
    else if (P_use < 75)
        Px = 50;
    else
        Px = 80;
    Charge_mode(Px);
}
void Charge_mode(uint16_t P_u)
{
    if (V_OUT < 24)
    {
        Duty_Cycle[0] = V_OUT / V_IN;
        Duty_Cycle[1] = V_OUT / V_IN;
        Duty_Cycle[2] = HRTIM_TIMD_PERIOD * 0.95;
        Duty_Cycle[3] = 0.00;
    }
    else if (V_OUT >= 24) //占空比D保持不变
    {
        if (C_CAP == 0)
        {
            Duty_Cycle[0] = 0.00;
            Duty_Cycle[1] = 0.00;
            Duty_Cycle[2] = 0.00;
            Duty_Cycle[3] = 0.00;
        }
    }
    PWM_Output(Duty_Cycle);
    ADC_Read();
    V_OUT = P_u / C_CAP;
}
void Discharge_mode()
{
    Duty_Cycle[0] = 0.00;
    Duty_Cycle[1] = 0.00;
    Duty_Cycle[2] = 0.00;
    Duty_Cycle[3] = 0.00;
    ADC_Read();
    I_rated = P_rated / V_IN;
    V_COUT = V_IN + V_gain;
    Duty_Cycle[0] = HRTIM_TIMD_PERIOD * 0.95;
    Duty_Cycle[1] = (V_COUT - V_CAP) / V_COUT;
    Duty_Cycle[2] = HRTIM_TIMD_PERIOD * 0.95;
    Duty_Cycle[3] = 0.00;
    P_OUT = V_COUT * C_OUT;
    if(C_IN > I_rated)
    {
        V_COUT += V_gain;
    }
    else if(C_IN < 0)
    {
        V_OUT = V_COUT - V_gain;
    }
}