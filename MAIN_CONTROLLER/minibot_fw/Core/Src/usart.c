/* ===========================================================================
 *  usart.c -- USART1 on PA9/PA10, the link to the Raspberry Pi 5.
 *
 *  Called by:  main() -> MX_USART1_UART_Init()
 *  Fires:      USART1_IRQHandler (stm32f4xx_it.c)
 *                -> HAL_UART_IRQHandler
 *                  -> HAL_UART_RxCpltCallback (main.c) -> MB_UartRxByteISR()
 *                  -> HAL_UART_ErrorCallback (main.c)  -> MB_UartErrorISR()
 *
 *  115200 8N1, no flow control. One byte every 87 us.
 * ========================================================================= */

#include "usart.h"

UART_HandleTypeDef huart1;

void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}
