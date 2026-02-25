#include "MPU6050.h"
#include <stdlib.h>
#include "stm32f1xx_hal_i2c.h" 
float roll_gyro, pitch_gyro, yaw_gyro;
float roll_acc, pitch_acc, yaw_acc;
float roll_Kalman, pitch_Kalman, yaw_Kalman;
float yaw_start = 0;
   
uint8_t yaw_initialized = 0;  

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
    
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buf, 2, 10);
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
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &addr, 1, 10);
    
    // 读取数据
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, &dat, 1, 10);
    
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
    uint8_t buf[14];
    uint8_t addr = ACCEL_XOUT_H;
    HAL_StatusTypeDef ret;
	
	if(__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY))
{
    I2C_Bus_Recovery();
}

    // 每次读取前检查I2C状态，若出错则复位
    if(__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_AF))
    {
        __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_AF);
        HAL_I2C_DeInit(&hi2c1);
        HAL_Delay(10);
        MX_I2C1_Init();  // 重新初始化I2C
    }

    // 发送起始地址
    ret = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &addr, 1, 100);
    if(ret != HAL_OK)
    {
        *AccX = *AccY = *AccZ = 0;  // 失败时置0，避免数据定格
        *GyroX = *GyroY = *GyroZ = 0;
        return;
    }

    // 批量读取数据
    ret = HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, buf, 14, 100);
    if(ret != HAL_OK)
    {
        *AccX = *AccY = *AccZ = 0;
        *GyroX = *GyroY = *GyroZ = 0;
        return;
    }

    // 解析数据
    *AccX = (buf[0] << 8) | buf[1];
    *AccY = (buf[2] << 8) | buf[3];
    *AccZ = (buf[4] << 8) | buf[5];
    *GyroX = (buf[8] << 8) | buf[9];
    *GyroY = (buf[10] << 8) | buf[11];
    *GyroZ = (buf[12] << 8) | buf[13];

    // 零漂校准
    int16_t gyro_z_offset = 15;
    *GyroZ -= gyro_z_offset;
    if(abs(*GyroZ) < 20)
        *GyroZ = 0;
}

void I2C_Bus_Recovery(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_I2C_DeInit(&hi2c1);

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = GPIO_PIN_6;   // SCL
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;   // SDA
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);

    // 发送 9 个时钟脉冲释放从机
    for(int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    // 发送 STOP 条件
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);

    MX_I2C1_Init();
}

