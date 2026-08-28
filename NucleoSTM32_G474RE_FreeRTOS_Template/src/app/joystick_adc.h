#ifndef JOYSTICK_ADC
#define JOYSTICK_ADC

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdio.h>

ADC_HandleTypeDef hadc1;
void ADC_Init(void);

#endif