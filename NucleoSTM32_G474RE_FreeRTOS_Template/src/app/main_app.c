#include <math.h>
#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32g4xx_hal.h"
#include "main.h"

#include "main_app.h"
#include "motor_pwm.h"
#include "joystick_adc.h"
#include "adc_dma.h"
#include "uart_buffer.h"

#define SHOULDER_ARM 90.0f
#define ELBOW_ARM 95.0f

typedef struct {
    int32_t x; //length
    int32_t y; //Width
    int32_t z; //Height
} TargetPosition;

typedef struct {
    float theta1; // Base
    float theta2; // Shoulder
    float theta3; // Elbow
} TargetAngles;

QueueHandle_t TargetPositionQueue;
QueueHandle_t TargetAnglesQueue;

uint16_t adc_buffer[4];
void Input_Task(void *argument);
void Input_Task(void *argument) {
    TargetPosition target = {0};
    char msg[50];
    //HAL_UART_Transmit(&lpuart1, "3\r\n", 3, 100);

    while(1) {
        //HAL_UART_Transmit(&lpuart1, "4\r\n", 3, 100);

        uint16_t raw_x_len = adc_buffer[0]; // Joy 1 X
        uint16_t raw_y_wid = adc_buffer[1]; // Joy 1 Y
        uint16_t raw_z_hgt = adc_buffer[2]; // Joy 2 Y
        uint16_t raw_claw  = adc_buffer[3]; // Joy 2 X

        if (raw_x_len > 2500 && raw_y_wid > 2000 && raw_z_hgt > 2000) {
            target.x += 1;
        } else if (raw_x_len < 1100 && raw_y_wid < 1550) {
            target.x -= 1;
        } 

        if (raw_y_wid < 1500 && raw_x_len > 2800) {
            target.y += 1;
        } else if (raw_y_wid > 2250 && raw_z_hgt > 2000 && raw_x_len < 1100) {
            target.y -= 1;
        } 

        if (raw_z_hgt < 700 && raw_claw > 2900) {
            target.z += 1;
        } else if (raw_z_hgt > 2450 && raw_claw < 1100) {
            target.z -= 1;
        } 

        
        //HAL_UART_Transmit(&lpuart1, "5\r\n", 3, 100);

        snprintf(msg, sizeof(msg), "J1X: %d, J1Y: %d, J2Y: %d, J2X: %d\r\n", raw_x_len, raw_y_wid, raw_z_hgt, raw_claw);

        HAL_UART_Transmit(&lpuart1, msg, strlen(msg), 500);

        xQueueOverwrite(TargetPositionQueue, &target);

        vTaskDelay(500);

    }
}

void Kinematics_Task(void *argument);
void Kinematics_Task(void *argument) {
    
}



void App_Main(void) {
    //HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    Motor_Init();
    ADC_Init();
    DMA_Init(&hadc1);
    UART_Init();
    TargetPositionQueue = xQueueCreate(1, sizeof(TargetPosition));
    TargetAnglesQueue = xQueueCreate(1, sizeof(TargetAngles));

    HAL_UART_Transmit(&lpuart1, "1\r\n", 3, 100);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 4);

    HAL_UART_Transmit(&lpuart1, "2\r\n", 3, 100);

    BaseType_t out = xTaskCreate(Input_Task, "Input", 512, NULL, 1, NULL);
    if (out == pdPASS) {
        HAL_UART_Transmit(&lpuart1, (uint8_t*)"task successful\r\n", strlen("task successful"), 500);
    } else {
        HAL_UART_Transmit(&lpuart1, (uint8_t*)"task failed\r\n", strlen("task failed"), 500);
    }

    vTaskStartScheduler();

}
