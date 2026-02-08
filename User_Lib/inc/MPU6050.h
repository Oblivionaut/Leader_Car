#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"  

// MPU6050设备地址
#define MPU6050_ADDR        0xD0  

// MPU6050寄存器地址
#define PWR_MGMT_1          0x6B
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define INT_ENABLE          0x38
#define ACCEL_XOUT_H        0x3B
#define ACCEL_YOUT_H        0x3D
#define ACCEL_ZOUT_H        0x3F
#define GYRO_XOUT_H         0x43
#define GYRO_YOUT_H         0x45
#define GYRO_ZOUT_H         0x47

extern I2C_HandleTypeDef hi2c1;  

void MPU6050_Write(uint8_t addr, uint8_t dat);
uint8_t MPU6050_Read(uint8_t addr);
void MPU6050_Init(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

#endif
