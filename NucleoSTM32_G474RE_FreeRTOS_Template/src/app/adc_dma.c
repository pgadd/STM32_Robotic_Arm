#include "adc_dma.h"

void DMA_Init(ADC_HandleTypeDef* hadc) {
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    if(hadc->Instance == ADC1) {
        __HAL_RCC_ADC12_CLK_ENABLE();

        hdma_adc1.Instance = DMA1_Channel1;
        hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
        hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE; // Don't increment peripheral address
        hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;     // DO increment memory address
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // 16-bit
        hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;    // 16-bit
        hdma_adc1.Init.Mode = DMA_CIRCULAR;
        hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
        
        HAL_DMA_Init(&hdma_adc1);

        // Link the initialized DMA handle to the ADC handle
        __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);
    }
}