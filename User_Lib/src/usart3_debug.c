#include "usart3_debug.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>

extern PID_t MotorA_PID;
extern PID_t MotorB_PID;

extern UART_HandleTypeDef huart3;
extern PID_Debug_t pid_debug;

uint8_t rx_byte;

// 初始化 UART 接收中断
void UART3_Debug_Init(void)
{
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

// 重定向 printf
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&ch, 1);
    return ch;
}

// 串口接收回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3)
    {
        // 通过单字符调节 PID 或目标速度
        switch(rx_byte)
        {
            case 'q': MotorA_PID.Kp += 0.01f; break;
            case 'a': MotorA_PID.Kp -= 0.01f; break;
            case 'w': MotorA_PID.Ki += 0.001f; break;
            case 's': MotorA_PID.Ki -= 0.001f; break;
            case 'e': MotorA_PID.Kd += 0.001f; break;
            case 'd': MotorA_PID.Kd -= 0.001f; break;

            case 'r': MotorA_PID.Target += 10; break;
            case 'f': MotorA_PID.Target -= 10; break;

            // 可自行增加 MotorB 控制
            case 't': MotorB_PID.Kp += 0.01f; break;
            case 'g': MotorB_PID.Kp -= 0.01f; break;
            case 'y': MotorB_PID.Ki += 0.001f; break;
            case 'h': MotorB_PID.Ki -= 0.001f; break;
            case 'u': MotorB_PID.Kd += 0.001f; break;
            case 'j': MotorB_PID.Kd -= 0.001f; break;
            case 'i': MotorB_PID.Target += 10; break;
            case 'k': MotorB_PID.Target -= 10; break;
        }

        // 再次开启中断接收
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}




