#include "stm32f1xx_hal.h"                  // Device header
#include "PID.h"

#define Speed1 30
#define Speed2 25
#define Speed3 20
#define Speed4 15
#define Speed5 10
#define Speed6 5

void TCRT_Init(void)
{	
	static uint8_t status = 0;
	
	if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0) || (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0) || (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8 ) == 0))
	{ 
		status = 1;	//一二三号识别到黑线，直角左转
	}
	else if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0) || (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0) || (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0))
	{
		status = 2;	//三四五号识别到黑线，直角右转
	}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0)
	{
		status = 3;	//三号识别到黑线，直行
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0)
	{
		status = 4;	//二号识别到黑线，左转
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0)
	{
		status = 5;		//四号识别到黑线，右转
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0)
	{
		status = 6; //一号识别到黑线，大角度左转
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0)
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
			Motor_Target_Set(-Speed3, Speed1);
		break;
		
		case 2:
			Motor_Target_Set(Speed1, -Speed3); 
		break;
		
		case 3:
			Motor_Target_Set(Speed1, Speed1);
		break;
		
		case 4:
			Motor_Target_Set(Speed3, Speed1);
		break;
		
		case 5:
			Motor_Target_Set(Speed1, Speed3);
		break;
		
		case 6:
			Motor_Target_Set(Speed6, Speed1);
		break;
		
		case 7:
			Motor_Target_Set(Speed1, Speed6);
		break;
	}
}









