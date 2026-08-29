#include "motor_pwm.h"

void Motor_Init(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 15;            
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 19999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim3);

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 15;            
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 19999;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim4);

    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 15;            
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 19999;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim8);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = 1500;

    HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1); // D12 | PA6 Shoulder
    HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2); // D11 | PA7 Claw
    HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3); // D15 | PB8 Base
    HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2); // D9 | PC7 Elbow

    GPIO_InitTypeDef Motor_pin = {0};
    
    Motor_pin.Mode = GPIO_MODE_AF_PP;
    Motor_pin.Pull = GPIO_NOPULL;
    Motor_pin.Speed = GPIO_SPEED_FREQ_LOW;
    
    Motor_pin.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    Motor_pin.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &Motor_pin);

    Motor_pin.Pin = GPIO_PIN_8;
    Motor_pin.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &Motor_pin);

    Motor_pin.Pin = GPIO_PIN_7;
    Motor_pin.Alternate = GPIO_AF4_TIM8;
    HAL_GPIO_Init(GPIOC, &Motor_pin);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

}
