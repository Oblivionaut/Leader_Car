#ifndef __MOTOR_H
#define __MOTOR_H
#include "gpio.h"
#define MAX_DUTY 50000


extern int Encoder_CountA;
extern int Encoder_CountB;
extern int SpeedA_Now;
extern int SpeedB_Now;
extern GPIO_PinState motorA_dir;
extern GPIO_PinState motorB_dir;
void MotorA_Duty(int duty);
void MotorB_Duty(int duty);
void pwm_update_hal(TIM_HandleTypeDef *htim,uint32_t channel,uint16_t duty);




#endif
