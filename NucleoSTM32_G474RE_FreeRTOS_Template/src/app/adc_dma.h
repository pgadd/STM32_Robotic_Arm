#ifndef ADC_DMA
#define ADC_DMA

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdio.h>

DMA_HandleTypeDef hdma_adc1;
void DMA_Init(ADC_HandleTypeDef* hadc);

#endif