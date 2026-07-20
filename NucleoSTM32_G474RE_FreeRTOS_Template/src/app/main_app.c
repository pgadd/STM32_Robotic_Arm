#include "main_app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32g4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern uint32_t SystemCoreClock;

typedef struct {
    uint16_t j1_x;
    uint16_t j1_y;
    uint16_t j2_x;
    uint16_t j2_y;
} JoystickData_t;

QueueHandle_t joystickQueue;

void Hardware_Override_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Force A0 (PA0) and A1 (PA1) as Analog (Joystick 1)
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Force A5 (PC0) and A4 (PC1) as Analog (Joystick 2)
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Force D14 (PB9) as PWM for TIM17_CH1 (Servo 4)
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM17; 
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// --- THE SLEDGEHAMMER HELPER FUNCTION ---
uint16_t Read_ADC_Channel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5; // Give capacitor time to charge
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    
    // Force the HAL state machine to unlock before changing channels
    hadc1.State = HAL_ADC_STATE_READY; 
    
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    
    return value;
}

void TaskJoystickRead(void *pvParameters) {
    JoystickData_t joyData = {2048, 2048, 2048, 2048}; 
    
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc1); 
    
    for (;;) {
        // Read each channel using the unlocked helper function
        joyData.j1_x = Read_ADC_Channel(ADC_CHANNEL_1); // A0
        joyData.j1_y = Read_ADC_Channel(ADC_CHANNEL_2); // A1
        joyData.j2_x = Read_ADC_Channel(ADC_CHANNEL_6); // A5
        joyData.j2_y = Read_ADC_Channel(ADC_CHANNEL_7); // A4

        xQueueSend(joystickQueue, &joyData, 0);
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

void TaskServoControl(void *pvParameters) {
    uint32_t prescaler = (SystemCoreClock / 1000000) - 1; 
    
    htim3.Instance->PSC = prescaler;
    htim3.Instance->ARR = 20000 - 1; 
    htim16.Instance->PSC = prescaler;
    htim16.Instance->ARR = 20000 - 1; 
    htim17.Instance->PSC = prescaler;
    htim17.Instance->ARR = 20000 - 1; 

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // D12
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // D11
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); // D15
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1); // D14
    
    JoystickData_t data;
    
    uint32_t pulse_1x = 1500; 
    uint32_t pulse_1y = 1500;
    uint32_t pulse_2x = 1500; 
    uint32_t pulse_2y = 1500;

    for (;;) {
        if (xQueueReceive(joystickQueue, &data, portMAX_DELAY) == pdPASS) {
            
            if (data.j1_x > 2200) pulse_1x += 15; else if (data.j1_x < 1800) pulse_1x -= 15;
            if (data.j1_y > 2200) pulse_1y += 15; else if (data.j1_y < 1800) pulse_1y -= 15;
            if (data.j2_x > 2200) pulse_2x += 15; else if (data.j2_x < 1800) pulse_2x -= 15;
            if (data.j2_y > 2200) pulse_2y += 15; else if (data.j2_y < 1800) pulse_2y -= 15;

            if (pulse_1x > 2000) pulse_1x = 2000; else if (pulse_1x < 1000) pulse_1x = 1000;
            if (pulse_1y > 2000) pulse_1y = 2000; else if (pulse_1y < 1000) pulse_1y = 1000;
            if (pulse_2x > 2000) pulse_2x = 2000; else if (pulse_2x < 1000) pulse_2x = 1000;
            if (pulse_2y > 2000) pulse_2y = 2000; else if (pulse_2y < 1000) pulse_2y = 1000;
            
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_1x);  
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_1y);  
            __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pulse_2x); 
            __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, pulse_2y); 
        }
    }
}

void App_Main(void) {
    Hardware_Override_Init();

    joystickQueue = xQueueCreate(5, sizeof(JoystickData_t));

    if (joystickQueue != NULL) {
        xTaskCreate(TaskJoystickRead, "JoyRead", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
        xTaskCreate(TaskServoControl, "ServoCtrl", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
    }
}