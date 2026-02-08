#ifndef __PID_h_
#define __PID_h_

enum
{
  POSITION_PID = 0,  // 位置式
  DELTA_PID,         // 增量式
};

typedef struct
{
	float target;	
	float now;
	float error[3];		
	float p,i,d;
	float pout, dout, iout;
	float out;   
	
	uint32_t pid_mode;

}pid_t;
void PID_Init(pid_t *pid, uint32_t mode, float p, float i, float d);
void pid_cal(pid_t *pid);
void PID_Control(void);
void Motor_Target_Set(int speA, int speB);
void datavision_send(void);

extern pid_t MotorA;
extern pid_t MotorB;
#endif
