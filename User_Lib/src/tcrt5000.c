#include "stm32f1xx_hal.h"                  // Device header
#include "PID.h"
void TCRT_Init(void)
{
	
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0)
	{
		Motor_Target_Set(0, 30); //一二三号识别到黑线，直角左转
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0)
	{
		Motor_Target_Set(30, 0); //三四五号识别到黑线，直角右转
	}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0)
	{
		Motor_Target_Set(30, 30); //三号识别到黑线，直行
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0)
	{
		Motor_Target_Set(25, 30); //二号识别到黑线，左转
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 0)
	{
		Motor_Target_Set(30, 25); //四号识别到黑线，右转
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0)
	{
		Motor_Target_Set(20, 30); //一号识别到黑线，大角度左转
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0)
	{
		Motor_Target_Set(25, 30); //五号识别到黑线，大角度右转
	}
}









