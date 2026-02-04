#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
	float Target;
	float Actual;
	float Out;
	
	float Err0;
	float Err1;
	float ErrInt;
	
	float ErrIntThreshold;
	
	float Kp;
	float Ki;
	float Kd;
	
	float OutMax;
	float OutMin;
} PID_t;

typedef struct {
    int targetA, actualA, outA;
    float KpA, KiA, KdA;
    int targetB, actualB, outB;
    float KpB, KiB, KdB;
} PID_Debug_t;

typedef struct
{
    int16_t targetA;
    int16_t actualA;
    int16_t outA;
    int16_t targetB;
    int16_t actualB;
    int16_t outB;
} Waveform_Data_t;

extern Waveform_Data_t wave_data;

void PID_Clear(PID_t *p);
void PID_Update(PID_t *p);
void Motor_PID_Init(void);
void Motor_Target_Set(int speA, int speB);
void Motor_PID_Control(void);
void datavision_send(void);

extern int A,B;

extern PID_t MotorA_PID;
extern PID_t MotorB_PID;
extern PID_Debug_t pid_debug;
#endif
