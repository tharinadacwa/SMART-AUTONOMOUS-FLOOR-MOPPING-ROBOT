/* ===========================================================================
 *  stm32f4xx_hal_msp.c -- MCU Support Package.
 *
 *  The HAL calls these from inside HAL_TIM_Base_Init() / HAL_UART_Init().
 *  This is where peripheral clocks, alternate-function pins, and the NVIC get
 *  configured.
 *
 *  ############ THE INTERRUPT PRIORITIES ARE NOT ARBITRARY ############
 *
 *      TIM7   preempt priority 0   (HIGHEST)
 *      USART1 preempt priority 1
 *      SysTick priority 15         (LOWEST, set in stm32f4xx_hal_conf.h)
 *
 *  TIM7 must never be preempted or delayed. A late step pulse is timing jitter,
 *  which is audible whine at best and lost steps at worst -- and a lost step on
 *  an encoderless robot is a silent lie in your odometry that nothing downstream
 *  can detect.
 *
 *  The TIM7 ISR is only ~2 us long. USART1 at 115200 receives one byte every
 *  87 us. So even if a byte arrives at the worst possible moment, the UART is
 *  serviced ~2 us later -- 40x inside its deadline. Zero risk of overrun.
 *
 *  DO NOT SWAP THESE. The temptation is to give the UART top priority "so we
 *  never miss a byte". That trades a problem you do not have (UART overrun, at
 *  40x margin) for one you cannot detect (step jitter).
 * ========================================================================= */

#include "main.h"

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /* 4 bits of preemption priority, 0 bits of sub-priority. */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7) {
        __HAL_RCC_TIM7_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);      /* HIGHEST */
        HAL_NVIC_EnableIRQ(TIM7_IRQn);
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7) {
        __HAL_RCC_TIM7_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM7_IRQn);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef g = {0};

    if (huart->Instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA9 = USART1_TX, PA10 = USART1_RX. Alternate function AF7. */
        g.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
        g.Mode      = GPIO_MODE_AF_PP;
        g.Pull      = GPIO_PULLUP;
        g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        g.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &g);

        HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);    /* below TIM7 */
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }
}
