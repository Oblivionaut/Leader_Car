#include "stm32f1xx_hal.h" 
#include "motor.h"
#include "PID.h"
#include "usart.h"

pid_t MotorA;
pid_t MotorB;
uint8_t head[2] = {0x03, 0xFC};
uint8_t tail[2] = {0xFC, 0x03};

static uint16_t speed_tick = 0;     // 速度统计节拍
static int32_t encA_sum = 0;
static int32_t encB_sum = 0;

void datavision_send()
{
	
	
	uint8_t A1 = (uint8_t)MotorA.target;
	uint8_t A2 = (uint8_t)MotorA.now;
	uint8_t B1 = (uint8_t)MotorB.target;
	uint8_t B2 = (uint8_t)MotorB.now;
	uint8_t buf[4];
	
	buf[0] = A1;
	buf[1] = A2;
	buf[2] = B1;
	buf[3] = B2;

	HAL_UART_Transmit(&huart3, head, 2, 10);
	HAL_UART_Transmit(&huart3, buf, 4, 10);
	HAL_UART_Transmit(&huart3, tail, 2, 10);
}

void PID_Init(pid_t *pid, uint32_t mode, float p, float i, float d)
{
	pid -> pid_mode = mode;
	pid -> p = p;
	pid -> i = i;
	pid -> d = d;
}

void Motor_Target_Set(int speA, int speB)
{
	if(speA >= 0)
	{
		motorA_dir = 1;
		MotorA.target = speA;
	}
	else
	{
		motorA_dir = 0;
		MotorA.target = -speA;
	}
	
	if(speB >= 0)
	{
		motorB_dir = 1;
		MotorB.target = speB;
	}
	else
	{
		motorB_dir = 0;
		MotorB.target = -speB;
	}
}

void PID_Control()
{
    //累加编码器
    encA_sum += Encoder_CountA;
    encB_sum += Encoder_CountB;

    Encoder_CountA = 0;
    Encoder_CountB = 0;

    speed_tick++;

    //每 5 次更新一次速度 
    if(speed_tick >= 5)
    {
        if(motorA_dir)
            MotorA.now = encA_sum * SCALE_FACTOR;
        else
            MotorA.now = -encA_sum * SCALE_FACTOR;

        if(motorB_dir)
            MotorB.now = encB_sum * SCALE_FACTOR;
        else
            MotorB.now = -encB_sum * SCALE_FACTOR;

        encA_sum = 0;
        encB_sum = 0;
        speed_tick = 0;
    }

    pid_cal(&MotorA);
    pid_cal(&MotorB);

    MotorA_Duty(MotorA.out);
    MotorB_Duty(MotorB.out);
}

void pid_cal(pid_t *pid)
{
	// 计算当前偏差
	pid->error[0] = pid->target - pid->now;

	// 计算输出
	if(pid->pid_mode == DELTA_PID)  // 增量式
	{
		pid->pout = pid->p * (pid->error[0] - pid->error[1]);
		pid->iout = pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		pid->out += pid->pout + pid->iout + pid->dout;
	}
	else if(pid->pid_mode == POSITION_PID)  // 位置式
	{
		pid->pout = pid->p * pid->error[0];
		pid->iout += pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - pid->error[1]);
		pid->out = pid->pout + pid->iout + pid->dout;
	}

	// 记录前两次偏差
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];

	// 输出限幅
	if(pid->out>=MAX_DUTY)	
		pid->out=MAX_DUTY;
	if(pid->out<=0)	
		pid->out=0;
	
}


