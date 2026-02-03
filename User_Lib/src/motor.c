#include "tim.h"
#include "gpio.h"
#include "PID.h"
int Encoder_Count = 0;
int Speed_Now = 0;
GPIO_PinState motor_dir = GPIO_PIN_SET;//高电平正转，低电平反转

void MotorA_Duty(int duty)
{
	int pwm = duty*500;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (GPIO_PinState)motor_dir);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (GPIO_PinState)!motor_dir);
	
}

void MotorB_Duty(int duty)
{
	int pwm = duty*500;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, (GPIO_PinState)motor_dir);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, (GPIO_PinState)!motor_dir);
}



