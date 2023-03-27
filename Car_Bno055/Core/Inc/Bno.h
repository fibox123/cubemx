#ifndef _BNO_H
#define _BNO_H

typedef unsigned          char uint8_t;
typedef double fp32;

void turnleft(fp32 angle);
void turnright(fp32 angle);
void ramp_cale(fp32 next, fp32 last, fp32 step);
#endif
