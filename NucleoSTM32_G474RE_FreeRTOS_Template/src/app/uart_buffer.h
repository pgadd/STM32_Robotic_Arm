#ifndef UART_BUFFER
#define UART_BUFFER

#include "main_app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32g4xx_hal.h"

extern UART_HandleTypeDef lpuart1;
void UART_Init(void);

#endif