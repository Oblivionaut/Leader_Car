#ifndef __MOTOR_H
#define __MOTOR_H
#include "gpio.h"

extern int Encoder_Count;
extern int Speed_Now;
extern GPIO_PinState motor_dir;
void MotorA_Duty(int duty);
void MotorB_Duty(int duty);




#endif
