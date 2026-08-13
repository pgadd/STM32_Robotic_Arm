#include "main_app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32g4xx_hal.h"
#include "main.h"

#include "robot_config.h"
#include "kinematics.h"
#include "signal_utils.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern uint32_t SystemCoreClock;

/* Filled continuously by ADC1 + DMA1 in circular mode (see MX_ADC1_Init /
 * MX_DMA_Init in main.c) -- no CPU polling required to keep it fresh.
 * [0]=J1 X (PA0), [1]=J1 Y (PA1), [2]=J2 X (PC0), [3]=J2 Y (PC1) */
volatile uint16_t joystick_dma[4];

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

static void ConfigServoTimer(TIM_HandleTypeDef *htim) {
    // 1MHz timer tick regardless of SYSCLK, 20000 ticks -> 20ms period (50Hz)
    uint32_t prescaler = (SystemCoreClock / 1000000) - 1;
    htim->Instance->PSC = prescaler;
    htim->Instance->ARR = (SERVO_PWM_PERIOD_US) - 1;
}

/* Reads a joystick channel, applies EMA smoothing then a center deadband.
 * Returns the conditioned value still in raw ADC-count units (0..4095). */
static float ReadConditionedAxis(EmaFilter *filt, volatile uint16_t *dma_slot) {
    float raw = EmaFilter_Update(filt, (float)(*dma_slot), ADC_FILTER_ALPHA);
    return ApplyDeadband(raw, ADC_CENTER, ADC_DEADBAND);
}

/* theta1/theta2: shoulder + elbow, solved via 2-link planar IK from
 * joystick 1's (x, y) deflection. TIM3 CH1 = shoulder (D12), CH2 = elbow (D9). */
void TaskArmPlanarIK(void *pvParameters) {
    (void)pvParameters;
    ConfigServoTimer(&htim3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    EmaFilter filt_x, filt_y;
    EmaFilter_Reset(&filt_x);
    EmaFilter_Reset(&filt_y);

    /* Start centered in the workspace rather than at 0 degrees, so the arm
     * doesn't snap on the first tick after boot. */
    float shoulder_deg = (SHOULDER_ANGLE_MIN_DEG + SHOULDER_ANGLE_MAX_DEG) * 0.5f;
    float elbow_deg = (ELBOW_ANGLE_MIN_DEG + ELBOW_ANGLE_MAX_DEG) * 0.5f;

    for (;;) {
        float adc_x = ReadConditionedAxis(&filt_x, &joystick_dma[0]);
        float adc_y = ReadConditionedAxis(&filt_y, &joystick_dma[1]);

        float target_x_mm = MapRange(adc_x, 0.0f, ADC_COUNTS_MAX, WORKSPACE_X_MIN_MM, WORKSPACE_X_MAX_MM);
        float target_y_mm = MapRange(adc_y, 0.0f, ADC_COUNTS_MAX, WORKSPACE_Y_MIN_MM, WORKSPACE_Y_MAX_MM);

        ArmIK_Result ik;
        if (ArmIK_Solve(target_x_mm, target_y_mm, LINK1_LENGTH_MM, LINK2_LENGTH_MM, &ik)) {
            float target_shoulder = ClampF(ik.shoulder_deg, SHOULDER_ANGLE_MIN_DEG, SHOULDER_ANGLE_MAX_DEG);
            float target_elbow = ClampF(ik.elbow_deg, ELBOW_ANGLE_MIN_DEG, ELBOW_ANGLE_MAX_DEG);

            // Step toward the IK solution instead of jumping straight to it.
            shoulder_deg = RateLimitStep(shoulder_deg, target_shoulder, MAX_STEP_DEG_PER_TICK);
            elbow_deg = RateLimitStep(elbow_deg, target_elbow, MAX_STEP_DEG_PER_TICK);

            uint32_t pulse_shoulder = AngleDegToPulseUs(shoulder_deg, SHOULDER_ANGLE_MIN_DEG, SHOULDER_ANGLE_MAX_DEG,
                                                          SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
            uint32_t pulse_elbow = AngleDegToPulseUs(elbow_deg, ELBOW_ANGLE_MIN_DEG, ELBOW_ANGLE_MAX_DEG,
                                                       SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);

            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_shoulder);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_elbow);
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));
    }
}

/* theta0/theta3: base rotation + wrist/gripper, driven directly from
 * joystick 2 -- no IK, just a straight ADC-to-angle map. X axis rotates the
 * whole arm's plane (TIM16 CH1 / D15), Y axis drives the wrist or gripper
 * (TIM17 CH1 / D11). */
void TaskBaseAndWrist(void *pvParameters) {
    (void)pvParameters;
    ConfigServoTimer(&htim16);
    ConfigServoTimer(&htim17);
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

    EmaFilter filt_base, filt_wrist;
    EmaFilter_Reset(&filt_base);
    EmaFilter_Reset(&filt_wrist);

    float base_deg = (BASE_ANGLE_MIN_DEG + BASE_ANGLE_MAX_DEG) * 0.5f;
    float wrist_deg = (WRIST_ANGLE_MIN_DEG + WRIST_ANGLE_MAX_DEG) * 0.5f;

    for (;;) {
        float adc_base = ReadConditionedAxis(&filt_base, &joystick_dma[2]);
        float adc_wrist = ReadConditionedAxis(&filt_wrist, &joystick_dma[3]);

        float target_base = MapRange(adc_base, 0.0f, ADC_COUNTS_MAX, BASE_ANGLE_MIN_DEG, BASE_ANGLE_MAX_DEG);
        float target_wrist = MapRange(adc_wrist, 0.0f, ADC_COUNTS_MAX, WRIST_ANGLE_MIN_DEG, WRIST_ANGLE_MAX_DEG);

        base_deg = RateLimitStep(base_deg, target_base, MAX_STEP_DEG_PER_TICK);
        wrist_deg = RateLimitStep(wrist_deg, target_wrist, MAX_STEP_DEG_PER_TICK);

        uint32_t pulse_base = AngleDegToPulseUs(base_deg, BASE_ANGLE_MIN_DEG, BASE_ANGLE_MAX_DEG,
                                                  SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
        uint32_t pulse_wrist = AngleDegToPulseUs(wrist_deg, WRIST_ANGLE_MIN_DEG, WRIST_ANGLE_MAX_DEG,
                                                   SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);

        __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pulse_base);
        __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, pulse_wrist);

        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));
    }
}

void App_Main(void) {
    // Redirect TIM16_CH1 from hidden PA12 over to Arduino header D15 (PB8)
    Route_TIM16_To_D15();

    // Start background DMA using the flawless initialization generated by CubeMX in main.c
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)joystick_dma, 4);

    if (xTaskCreate(TaskArmPlanarIK, "ArmIK", 384, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS) {
        Error_Handler();
    }
    if (xTaskCreate(TaskBaseAndWrist, "BaseWrist", 256, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS) {
        Error_Handler();
    }
}
