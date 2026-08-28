#include "joystick_adc.h"

void ADC_Init(void){
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B; // 0-4095 range
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;  // Essential for multiple channels
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc1.Init.ContinuousConvMode = ENABLE;     // Keep reading in the background
    hadc1.Init.NbrOfConversion = 4;             // We have 4 joystick axes
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    HAL_ADC_Init(&hadc1);


    ADC_ChannelConfTypeDef sConfig = {0};

    // Joy 1 X (Length) -> PA0
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Joy 1 Y (Width) -> PA1
    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Joy 2 Y (Height) -> PC1
    sConfig.Channel = ADC_CHANNEL_7;
    sConfig.Rank = ADC_REGULAR_RANK_3;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Joy 2 X (Claw) -> PC0
    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = ADC_REGULAR_RANK_4;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);


    GPIO_InitTypeDef Adc_pins = {0};

    Adc_pins.Pin = GPIO_PIN_0 | GPIO_PIN_1; // PA0 (Joy1 X), PA1 (Joy1 Y)
    Adc_pins.Mode = GPIO_MODE_ANALOG;
    Adc_pins.Pull = GPIO_NOPULL; 
    HAL_GPIO_Init(GPIOA, &Adc_pins);

    Adc_pins.Pin = GPIO_PIN_0 | GPIO_PIN_1; // PC0 (Joy2 X), PC1 (Joy2 Y)
    HAL_GPIO_Init(GPIOC, &Adc_pins);

}