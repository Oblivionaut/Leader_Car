#include "tim.h"
#include "gpio.h"

GPIO_PinState motor_dir = GPIO_PIN_SET;//高电平正转，低电平反转

void Motor_Duty(int duty)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (GPIO_PinState)motor_dir);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (GPIO_PinState)!motor_dir);
	
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, (GPIO_PinState)motor_dir);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, (GPIO_PinState)!motor_dir);
}


