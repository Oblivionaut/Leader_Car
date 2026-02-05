#include "tim.h"
#include "gpio.h"
#include "PID.h"

#define MAX_DUTY 50000
volatile int Encoder_CountA = 0;
volatile int Encoder_CountB = 0;
int SpeedA_Now = 0;
int SpeedB_Now = 0;
GPIO_PinState motorA_dir = 1;//高电平正转，低电平反转
GPIO_PinState motorB_dir = 1;//高电平正转，低电平反转

void pwm_update_hal(TIM_HandleTypeDef *htim,uint32_t channel,uint16_t duty)
{
    uint32_t arr;

    if (duty >= MAX_DUTY)
        duty = MAX_DUTY;

    /* 读取 ARR（自动适配不同 PWM 频率） */
    arr = __HAL_TIM_GET_AUTORELOAD(htim);

    /* duty → CCR 映射 */
    uint32_t ccr = (uint32_t)duty * (arr + 1) / MAX_DUTY;

    /* 更新 PWM 占空比 */
    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}

void MotorA_Duty(int duty)
{
    pwm_update_hal(&htim2, TIM_CHANNEL_1,duty);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, motorA_dir);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (GPIO_PinState)!motorA_dir);
}

void MotorB_Duty(int duty)
{
	pwm_update_hal(&htim2, TIM_CHANNEL_2,duty);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, motorB_dir);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, (GPIO_PinState)!motorB_dir);
}


