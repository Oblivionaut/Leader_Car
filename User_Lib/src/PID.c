#include "PID.h"
#include "math.h"
#include "motor.h"
#include "gpio.h"
#include "OLED.h"

#define Max_Duty 50000
PID_t MotorA_PID;
PID_t MotorB_PID;

int A = 0;
int B = 0;

void Motor_PID_Init(void)
{
    // 电机1 PID参数初始化
    MotorA_PID.Kp = 0.3f;
	MotorA_PID.Ki = 0.0f;
	MotorA_PID.Kd = 0.0f;
	MotorA_PID.OutMax = 100.0f;
	MotorA_PID.OutMin = -100.0f;
    PID_Clear(&MotorA_PID); // 清空误差
    
    // 电机2 PID参数初始化（可独立配置）
    MotorB_PID.Kp = 0.3f;
	MotorB_PID.Ki = 0.0f;
	MotorB_PID.Kd = 0.0f;
	MotorB_PID.OutMax = 100.0f;
	MotorB_PID.OutMin = -100.0f;
    PID_Clear(&MotorB_PID);
}

void Motor_PID_Control(void)
{
    static int dutyA = 0;
    static int dutyB = 0;
    int speed;

    if (motor_dir >= 0)
    {
        speed = Encoder_Count;
    }
    else
    {
        speed = -Encoder_Count;
    }

    OLED_ShowNum(1, 1, speed, 3);

    Encoder_Count = 0;

    MotorA_PID.Actual = speed;
    MotorB_PID.Actual = speed;

//计算增量
    PID_Update(&MotorA_PID);
    PID_Update(&MotorB_PID);

//速度累加
    dutyA += (int)MotorA_PID.Out;
    dutyB += (int)MotorB_PID.Out;

//限制
    if (dutyA > Max_Duty) dutyA = Max_Duty;
    if (dutyA < 0)        dutyA = 0;

    if (dutyB > Max_Duty) dutyB = Max_Duty;
    if (dutyB < 0)        dutyB = 0;

    MotorA_Duty(dutyA);
    MotorB_Duty(dutyB);

    A = dutyA;
    B = dutyB;
}

void Motor_Target_Set(int spe)//电机目标值
{
	if(spe >= 0)
	{
		motor_dir = GPIO_PIN_SET;
		MotorA_PID.Target = spe;
		MotorB_PID.Target = spe;	
	}else{
		motor_dir = GPIO_PIN_RESET;
		MotorA_PID.Target = -spe;
		MotorB_PID.Target = -spe;
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

