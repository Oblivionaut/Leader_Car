#ifndef __USART3_DEBUG_H
#define __USART3_DEBUG_H

#include "main.h"
#include "PID.h"

void UART3_Debug_Init(void);
extern UART_HandleTypeDef huart3;
extern PID_Debug_t pid_debug;
void Send_PID_Waveform(void);
void datavision_send(void);
#endif
