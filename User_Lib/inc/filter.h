#ifndef _filter_h
#define _filter_h
#include "math.h"

typedef struct
{
	float Q_angle; 
	float Q_bias;  
	float R;       
	float P[2][2]; 
	float dt;      
	float K1, K2;  

	
	float Angle;     
	float Gyro_bias; 

}KF_t;

float Mahony_Filter(float gyro, float acc);
float Kalman_Filter(KF_t *kf, float obsValue, float ut);

extern KF_t KF_Yaw, KF_Roll, KF_Pitch;

#endif
