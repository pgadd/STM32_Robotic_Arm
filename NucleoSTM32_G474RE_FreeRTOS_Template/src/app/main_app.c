#include <math.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

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
    int32_t C; //Claw
} TargetPosition;

typedef struct {
    float theta1; // Base
    float theta2; // Shoulder
    float theta3; // Elbow
    float theta4; // Claw
} TargetAngles;


TargetAngles target_angles = {0};
TargetPosition target = {0};

QueueHandle_t TargetPositionQueue;
QueueHandle_t TargetAnglesQueue;

uint16_t adc_buffer[4];
void Input_Task(void *argument);
void Input_Task(void *argument) {
    char msg[50];
    //HAL_UART_Transmit(&lpuart1, "3\r\n", 3, 100);
    target.x = 0;
    target.y = (int)ELBOW_ARM;
    target.z = (int)SHOULDER_ARM;
    target.C = 1500;

    while(1) {
        //HAL_UART_Transmit(&lpuart1, "4\r\n", 3, 100);

        

        uint16_t raw_x_len = adc_buffer[0]; // Joy 1 X
        uint16_t raw_y_wid = adc_buffer[1]; // Joy 1 Y
        uint16_t raw_z_hgt = adc_buffer[2]; // Joy 2 Y
        uint16_t raw_claw  = adc_buffer[3]; // Joy 2 X

        TargetPosition backup = target; //Backup state


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

        if(raw_claw < 1050 && raw_z_hgt > 1605){
            target.C -= 20;
        } else if (raw_claw > 2900 && raw_z_hgt < 1621) {
            target.C += 20;
        }

        float radius = sqrt((target.x * target.x) + (target.y + target.y));
        float hypotenuse = sqrt ((radius * radius) + (target.z * target.z)); 
        float max_distance = SHOULDER_ARM + ELBOW_ARM;

        if (hypotenuse > max_distance || radius > max_distance) {
            target = backup;
        }
        
        //HAL_UART_Transmit(&lpuart1, "5\r\n", 3, 100);

        snprintf(msg, sizeof(msg), "J1X: %d, J1Y: %d, J2Y: %d, J2X: %d\r\n", raw_x_len, raw_y_wid, raw_z_hgt, raw_claw);

        HAL_UART_Transmit(&lpuart1, msg, strlen(msg), 500);

        xQueueOverwrite(TargetPositionQueue, &target);

        vTaskDelay(20);

    }
}

void Kinematics_Task(void *argument);
void Kinematics_Task(void *argument) {
    TargetPosition local = {0};
    while (1) {
        xQueueReceive(TargetPositionQueue, &local, portMAX_DELAY);

        target_angles.theta1 = atan2(local.y, local.x) * (180.0f / M_PI);

        float r = sqrt((local.x * local.x) + (local.y * local.y));

        float d = sqrt((r * r) + (local.z * local.z));

        target_angles.theta2 = (acosf(((SHOULDER_ARM * SHOULDER_ARM) + (d * d) - (ELBOW_ARM * ELBOW_ARM))/(2 * SHOULDER_ARM * d)) *(180.0f / M_PI)) + (atan2f(local.z, r) * (180.0f / M_PI));

        target_angles.theta3 = acosf(((SHOULDER_ARM * SHOULDER_ARM) + (ELBOW_ARM * ELBOW_ARM) - (d * d))/ (2 * SHOULDER_ARM * ELBOW_ARM)) * (180.0f / M_PI);


        target_angles.theta4 = local.C;

        xQueueOverwrite(TargetAnglesQueue, &target_angles);
 
    }

}


void Servo_Task(void* pvParameter);
void Servo_Task(void* pvParameter) {
    TargetAngles local = {90.0f, 90.0f, 90.0f, 1500};

    int current_base = 1500;
    int current_shoulder = 1500;
    int current_elbow = 1500;
    int current_claw = 1500;

    while(1) {
        xQueueReceive(TargetAnglesQueue, &local, 10);
        
        int target_base = (local.theta1 * 2000.0f/180.0f) + 500;
        int target_shoulder = (local.theta2 * 2000.0f/180.0f) + 500;
        int target_elbow = (local.theta3 * 2000.0f/180.0f) + 500;
        int target_claw = local.theta4;

        if (current_base + 12 <= target_base){
            current_base += 10;
        } else if (current_base > target_base + 12) {
            current_base -=10;
        }

        if (current_shoulder + 12 <= target_shoulder){
            current_shoulder += 10;
        } else if (current_shoulder > target_shoulder + 12) {
            current_shoulder -=10;
        }

        if (current_elbow + 12 <= target_elbow){
            current_elbow += 10;
        } else if (current_elbow > target_elbow + 12) {
            current_elbow -=10;
        }

        if (current_claw + 12 < target_claw){
            current_claw += 10;
        } else if (current_claw > target_claw + 12) {
            current_claw -= 10;
        }

        TIM4 -> CCR3 = current_base;
        TIM3 -> CCR1 = current_shoulder;
        TIM8 -> CCR2 = current_elbow;
        TIM3 -> CCR2 = current_claw;

        vTaskDelay(20);
    }

}


void App_Main(void) {
    Motor_Init();
    ADC_Init();
    DMA_Init(&hadc1);
    UART_Init();
    TargetPositionQueue = xQueueCreate(1, sizeof(TargetPosition));
    TargetAnglesQueue = xQueueCreate(1, sizeof(TargetAngles));

    //HAL_UART_Transmit(&lpuart1, "1\r\n", 3, 100);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 4);

    //HAL_UART_Transmit(&lpuart1, "2\r\n", 3, 100);

    xTaskCreate(Input_Task, "Input", 256, NULL, 1, NULL);
    xTaskCreate(Kinematics_Task, "Calculate", 256, NULL, 2, NULL);
    xTaskCreate(Servo_Task, "Motors", 128, NULL, 3, NULL);

    //Test case for task creation.
    //BaseType_t out = 
    // if (out == pdPASS) {
    //     HAL_UART_Transmit(&lpuart1, (uint8_t*)"task successful\r\n", strlen("task successful"), 500);
    // } else {
    //     HAL_UART_Transmit(&lpuart1, (uint8_t*)"task failed\r\n", strlen("task failed"), 500);
    // }

    vTaskStartScheduler();

}
