#include "tim.h"
#include "gpio.h"
#include "PID.h"
volatile int Encoder_CountA = 0;
volatile int Encoder_CountB = 0;
int Speed_Now = 0;
GPIO_PinState motorA_dir = GPIO_PIN_SET;//高电平正转，低电平反转
GPIO_PinState motorB_dir = GPIO_PIN_SET;//高电平正转，低电平反转

void MotorA_Duty(int duty)
{
	int pwm = duty * 200;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);//最大值20000
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (GPIO_PinState)motorA_dir);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (GPIO_PinState)!motorA_dir);
	
}

void MotorB_Duty(int duty)
{
	int pwm = duty * 200;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, (GPIO_PinState)motorB_dir);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, (GPIO_PinState)!motorB_dir);
}



