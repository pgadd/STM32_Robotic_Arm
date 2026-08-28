#include "uart_buffer.h"

UART_HandleTypeDef lpuart1 = {0};

void UART_Init(void) {
    __HAL_RCC_LPUART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef Uart = {0};

    Uart.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    Uart.Mode = GPIO_MODE_AF_PP;
    Uart.Pull = GPIO_NOPULL;
    Uart.Speed = GPIO_SPEED_FREQ_LOW;
    Uart.Alternate = GPIO_AF12_LPUART1;

    HAL_GPIO_Init(GPIOA, &Uart);

    lpuart1.Instance = LPUART1;
    lpuart1.Init.BaudRate = 115200;
    lpuart1.Init.WordLength = UART_WORDLENGTH_8B;
    lpuart1.Init.StopBits = UART_STOPBITS_1;
    lpuart1.Init.Parity = UART_PARITY_NONE;
    lpuart1.Init.Mode = UART_MODE_TX_RX;

    HAL_UART_Init(&lpuart1);

}