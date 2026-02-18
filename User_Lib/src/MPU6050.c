#include "MPU6050.h"
#include <stdlib.h>
float roll_gyro, pitch_gyro, yaw_gyro;
float roll_acc, pitch_acc, yaw_acc;
float roll_Kalman, pitch_Kalman, yaw_Kalman;

/**
 * @brief  向MPU6050写入一个字节
 * @param  addr: 寄存器地址
 * @param  dat: 要写入的数据
 * @retval 无
 */
void MPU6050_Write(uint8_t addr, uint8_t dat)
{
    uint8_t buf[2];
    buf[0] = addr;
    buf[1] = dat;
    
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buf, 2, HAL_MAX_DELAY);
}

/**
 * @brief  从MPU6050读取一个字节
 * @param  addr: 寄存器地址
 * @retval 读取的数据
 */
uint8_t MPU6050_Read(uint8_t addr)
{
    uint8_t dat;
    
    // 发送寄存器地址
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &addr, 1, HAL_MAX_DELAY);
    
    // 读取数据
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, &dat, 1, HAL_MAX_DELAY);
    
    return dat;
}

/**
 * @brief  初始化MPU6050
 * @param  无
 * @retval 无
 */
void MPU6050_Init(void)
{
    // 延时等待MPU6050上电稳定
    HAL_Delay(100);
    
    // 时钟源设置为PLL with Y axis gyroscope reference
    MPU6050_Write(PWR_MGMT_1, 0x02);
    
    // 采样率分频设置为200Hz
    MPU6050_Write(SMPLRT_DIV, 0x27);
    
    // 配置DLPF
    MPU6050_Write(CONFIG, 0x00);
    
    // 陀螺仪量程设置为2000
    MPU6050_Write(GYRO_CONFIG, 0x18);
    
    // 加速度计量程设置为2g
    MPU6050_Write(ACCEL_CONFIG, 0x00);
    
    // 使能INT中断就绪
    MPU6050_Write(INT_ENABLE, 0x01);
}

/**
 * @brief  获取MPU6050的加速度和陀螺仪数据
 * @param  AccX: 加速度X轴数据指针
 * @param  AccY: 加速度Y轴数据指针
 * @param  AccZ: 加速度Z轴数据指针
 * @param  GyroX: 陀螺仪X轴数据指针
 * @param  GyroY: 陀螺仪Y轴数据指针
 * @param  GyroZ: 陀螺仪Z轴数据指针
 * @retval 无
 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t DataH, DataL;
    
    // 读取加速度X轴
    DataH = MPU6050_Read(ACCEL_XOUT_H);
    DataL = MPU6050_Read(ACCEL_XOUT_H + 1);
    *AccX = (DataH << 8) | DataL;
    
    // 读取加速度Y轴
    DataH = MPU6050_Read(ACCEL_YOUT_H);
    DataL = MPU6050_Read(ACCEL_YOUT_H + 1);
    *AccY = (DataH << 8) | DataL;
    
    // 读取加速度Z轴
    DataH = MPU6050_Read(ACCEL_ZOUT_H);
    DataL = MPU6050_Read(ACCEL_ZOUT_H + 1);
    *AccZ = (DataH << 8) | DataL;
    
    // 读取陀螺仪X轴
    DataH = MPU6050_Read(GYRO_XOUT_H);
    DataL = MPU6050_Read(GYRO_XOUT_H + 1);
    *GyroX = (DataH << 8) | DataL;
    
    // 读取陀螺仪Y轴
    DataH = MPU6050_Read(GYRO_YOUT_H);
    DataL = MPU6050_Read(GYRO_YOUT_H + 1);
    *GyroY = (DataH << 8) | DataL;
    
    // 读取陀螺仪Z轴
    DataH = MPU6050_Read(GYRO_ZOUT_H);
    DataL = MPU6050_Read(GYRO_ZOUT_H + 1);
    *GyroZ = (DataH << 8) | DataL;
	
	int16_t gyro_z_offset = 15;  // 零漂偏移量（根据实际校准调整）
    *GyroZ -= gyro_z_offset;
    if(abs(*GyroZ) < 20)  // 小阈值设为0，避免微小波动
        *GyroZ = 0;
}

