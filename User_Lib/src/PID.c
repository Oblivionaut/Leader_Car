#include "PID.h"
#include "math.h"
#include "motor.h"
#include "gpio.h"
#include "OLED.h"
#include "stdint.h"
#include "usart.h"
#include "usart3_debug.h"

#define Max_Duty 50000
PID_t MotorA_PID;
PID_t MotorB_PID;

int A = 0;
int B = 0;

PID_Debug_t pid_debug;

void datavision_send(void)
{
    uint8_t head[2] = {0x03, 0xFC};
    uint8_t tail[2] = {0xFC, 0x03};

    int16_t tar = MotorA_PID.Target * 10;
    int16_t act = MotorA_PID.Actual * 10;

    HAL_UART_Transmit(&huart3, head, 2, 100);
    HAL_UART_Transmit(&huart3, (uint8_t*)&tar, 2, 100);
    HAL_UART_Transmit(&huart3, (uint8_t*)&act, 2, 100);
    HAL_UART_Transmit(&huart3, tail, 2, 100);
}

void Motor_PID_Init(void)
{
    MotorA_PID.Kp = 1.14f;
    MotorA_PID.Ki = 0.0f;
    MotorA_PID.Kd = 0.1f;
    MotorA_PID.OutMax = Max_Duty;
    MotorA_PID.OutMin = -Max_Duty;
    PID_Clear(&MotorA_PID);

    MotorB_PID.Kp = 1.14f;
    MotorB_PID.Ki = 0.0f;
    MotorB_PID.Kd = 0.0f;
    MotorB_PID.OutMax = Max_Duty;
    MotorB_PID.OutMin = -Max_Duty;
    PID_Clear(&MotorB_PID);
}

void Motor_PID_Control(void)
{
    static uint8_t div = 0;
    div++;
    if(div < 5) return;   // 10ms × 5 = 50ms
    div = 0;

    int speedA = Encoder_CountA;
	int speedB = Encoder_CountB;

    MotorA_PID.Actual = speedA;
    MotorB_PID.Actual = speedB;

    PID_Update(&MotorA_PID);
    PID_Update(&MotorB_PID);

    MotorA_Duty((int)MotorA_PID.Out);
    MotorB_Duty((int)MotorB_PID.Out);

//	datavision_send();
	
    Encoder_CountA = 0;
    Encoder_CountB = 0;
}


void Motor_Target_Set(int speA, int speB)//电机目标值
{
	if(speA >= 0)
	{
		motorA_dir = GPIO_PIN_SET;
		MotorA_PID.Target = speA;	
	}else{
		motorA_dir = GPIO_PIN_RESET;
		MotorA_PID.Target = -speA;
	}
	
	if(speB >= 0)
	{
		motorB_dir = GPIO_PIN_SET;
		MotorB_PID.Target = speB;	
	}else{
		motorB_dir = GPIO_PIN_RESET;
		MotorB_PID.Target = -speB;
	}
}

void PID_Clear(PID_t *p)
{
	p->Err0 = 0;
	p->Err1 = 0;
	p->ErrInt = 0;
}

void PID_Update(PID_t *p)
{
	p->Err1 = p->Err0;
	p->Err0 = p->Target - p->Actual;
	
	if (p->Ki != 0)
	{
		if (p->ErrIntThreshold == 0)
		{
			p->ErrInt += p->Err0;
		}
		else if (fabs(p->Err0) < p->ErrIntThreshold)
		{
			p->ErrInt += p->Err0;
		}
		else
		{
			p->ErrInt = 0;
		}
	}
	else
	{
		p->ErrInt = 0;
	}
	
	p->Out = p->Kp * p->Err0
		   + p->Ki * p->ErrInt
		   + p->Kd * (p->Err0 - p->Err1);
	
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
}

