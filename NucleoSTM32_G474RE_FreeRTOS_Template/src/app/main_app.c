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

// Hardware DMA buffer - automatically updated by the ADC in the background
volatile uint16_t joystick_dma[4]; 

// FreeRTOS Queue Handle
QueueHandle_t joystickQueue;

typedef struct {
    uint16_t j1_x;
    uint16_t j1_y;
    uint16_t j2_x;
    uint16_t j2_y;
} JoystickData_t;

void Route_TIM16_To_D15(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Redirect TIM16_CH1 from hidden PA12 over to Arduino header D15 (PB8)
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM16; 
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// --- PRODUCER TASK: Reads background DMA and pushes to Queue ---
void TaskJoystickRead(void *pvParameters) {
    JoystickData_t joyData;

    for (;;) {
        // Grab the latest hardware conversions from the DMA stream
        joyData.j1_x = joystick_dma[0]; // A0
        joyData.j1_y = joystick_dma[1]; // A1
        joyData.j2_x = joystick_dma[2]; // A5
        joyData.j2_y = joystick_dma[3]; // A4

        // Send the packaged struct to the servo controller (non-blocking)
        xQueueSend(joystickQueue, &joyData, 0);

        // Run at exactly 50Hz (20ms) to match the physical servo refresh period
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

// --- CONSUMER TASK: Waits for Queue data and drives Motors ---
void TaskServoControl(void *pvParameters) {
    // Dynamically calculate prescaler to force exactly 50Hz PWM
    uint32_t prescaler = (SystemCoreClock / 1000000) - 1; 
    
    htim3.Instance->PSC = prescaler;
    htim3.Instance->ARR = 20000 - 1; 
    htim16.Instance->PSC = prescaler;
    htim16.Instance->ARR = 20000 - 1; 
    htim17.Instance->PSC = prescaler;
    htim17.Instance->ARR = 20000 - 1; 

    // Start PWM outputs
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // D12
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // D9
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); // D15 (Routed to PB8)
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1); // D11
    
    JoystickData_t data;
    
    uint32_t pulse_1x = 1500; 
    uint32_t pulse_1y = 1500;
    uint32_t pulse_2x = 1500; 
    uint32_t pulse_2y = 1500;

    for (;;) {
        // Sleep cleanly until new joystick data arrives in the queue
        if (xQueueReceive(joystickQueue, &data, portMAX_DELAY) == pdPASS) {
            
            // Adjust pulse widths based on joystick thresholds
            if (data.j1_x > 2200) pulse_1x += 15; else if (data.j1_x < 1800) pulse_1x -= 15;
            if (data.j1_y > 2200) pulse_1y += 15; else if (data.j1_y < 1800) pulse_1y -= 15;
            if (data.j2_x > 2200) pulse_2x += 15; else if (data.j2_x < 1800) pulse_2x -= 15;
            if (data.j2_y > 2200) pulse_2y += 15; else if (data.j2_y < 1800) pulse_2y -= 15;

            // Clamp pulses to physical servo limits (1000us to 2000us)
            if (pulse_1x > 2000) pulse_1x = 2000; else if (pulse_1x < 1000) pulse_1x = 1000;
            if (pulse_1y > 2000) pulse_1y = 2000; else if (pulse_1y < 1000) pulse_1y = 1000;
            if (pulse_2x > 2000) pulse_2x = 2000; else if (pulse_2x < 1000) pulse_2x = 1000;
            if (pulse_2y > 2000) pulse_2y = 2000; else if (pulse_2y < 1000) pulse_2y = 1000;
            
            // Update hardware compare registers instantly
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_1x);  
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_1y);  
            __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pulse_2x); 
            __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, pulse_2y); 
        }
    }
}

void App_Main(void) {
    // 1. Route hidden TIM16_CH1 to Arduino header D15
    Route_TIM16_To_D15();

    // 2. Start the hardware DMA stream
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)joystick_dma, 4);

    // 3. Create the inter-task communication queue
    joystickQueue = xQueueCreate(5, sizeof(JoystickData_t));

    // 4. Launch the RTOS tasks
    if (joystickQueue != NULL) {
        xTaskCreate(TaskJoystickRead, "JoyRead", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
        xTaskCreate(TaskServoControl, "ServoCtrl", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
    }
}