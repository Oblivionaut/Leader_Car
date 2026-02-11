#include "stm32f1xx_hal.h"                  // Device header
#include "PID.h"

#define High_Speed 30
#define Mid_Speed 20
#define Low_Speed 10

void TCRT_Init(void)
{	
	static uint8_t status = 0;
	
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0)
	{
		 
		status = 1;	//一二三号识别到黑线，直角左转
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0)
	{

		status = 2;	//三四五号识别到黑线，直角右转
	}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0)
	{
		Motor_Target_Set(30, 30); 
		status = 3;	//三号识别到黑线，直行
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0)
	{
		Motor_Target_Set(25, 30); 
		status = 4;	//二号识别到黑线，左转
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 0)
	{
		Motor_Target_Set(30, 25);
		status = 5;		//四号识别到黑线，右转
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0)
	{
		Motor_Target_Set(20, 30);
		status = 6; //一号识别到黑线，大角度左转
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0)
	{
		status = 7;		//五号识别到黑线，大角度右转
	}
	else
	{
		status = status;
	}
	switch(status)
	{
		case 1:
			Motor_Target_Set(Low_Speed, High_Speed);
		break;
		
		case 2:
			Motor_Target_Set(High_Speed, Low_Speed); 
		break;
		
		case 3:
			Motor_Target_Set(High_Speed, High_Speed);
		break;
		
		case 4:
			Motor_Target_Set(Mid_Speed, High_Speed);
		break;
		
		case 5:
			Motor_Target_Set(High_Speed, Mid_Speed);
		break;
		
		case 6:
			Motor_Target_Set(Low_Speed, High_Speed);
		break;
		
		case 7:
			Motor_Target_Set(High_Speed, Low_Speed);
		break;
	}
}









