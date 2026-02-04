#ifndef __MOTOR_H
#define __MOTOR_H
#include "gpio.h"

extern int Encoder_CountA;
extern int Encoder_CountB;
extern int Speed_Now;
extern GPIO_PinState motorA_dir;
extern GPIO_PinState motorB_dir;
void MotorA_Duty(int duty);
void MotorB_Duty(int duty);




#endif
