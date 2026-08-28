#ifndef MOTOR
#define MOTOR
 
#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdio.h>

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
void Motor_Init(void);

#endif