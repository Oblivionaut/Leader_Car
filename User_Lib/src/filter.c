#include "filter.h"


#define alpha  0.95238

KF_t KF_Yaw = {
	0.001,            // Q_angle
	0.003,            // Q_bias
	0.5,              // R
	{{1, 0}, {0, 1}}, // P[2][2]
	0.05              // dt
};

KF_t KF_Roll = {
	0.001,            // Q_angle
	0.003,            // Q_bias
	0.5,              // R
	{{1, 0}, {0, 1}}, // P[2][2]
	0.05              // dt
};

KF_t KF_Pitch = {
	0.001,            // Q_angle
	0.003,            // Q_bias
	0.5,              // R
	{{1, 0}, {0, 1}}, // P[2][2]
	0.05              // dt
};



float Mahony_Filter(float gyro, float acc)
{
	return (alpha * gyro + (1 - alpha) * acc);
}



float Kalman_Filter(KF_t *kf, float obsValue, float ut)		
{

	kf->Angle = kf->Angle + (ut - kf->Gyro_bias) * kf->dt;
	kf->Gyro_bias = kf->Gyro_bias;

	kf->P[0][0] = kf->P[0][0] - (kf->P[0][1] + kf->P[1][0]) * kf->dt + kf->P[1][1] * kf->dt * kf->dt + kf->Q_angle;
	kf->P[0][1] = kf->P[0][1] - kf->P[1][1] * kf->dt;
	kf->P[1][0] = kf->P[1][0] - kf->P[1][1] * kf->dt;
	kf->P[1][1] = kf->P[1][1] + kf->Q_bias;

	kf->K1 = kf->P[0][0] / (kf->P[0][0] + kf->R);
	kf->K2 = kf->P[1][0] / (kf->P[0][0] + kf->R);

	kf->Angle = kf->Angle + kf->K1 * (obsValue - kf->Angle);
	kf->Gyro_bias = kf->Gyro_bias + kf->K2 * (obsValue - kf->Angle);
	
	kf->P[0][0] = (1 - kf->K1) * kf->P[0][0];
	kf->P[0][1] = (1 - kf->K1) * kf->P[0][1];
	kf->P[1][0] = kf->P[1][0] - kf->P[0][0] * kf->K2;
	kf->P[1][1] = kf->P[1][1] - kf->P[0][1] * kf->K2;
	
	return kf->Angle;
}

