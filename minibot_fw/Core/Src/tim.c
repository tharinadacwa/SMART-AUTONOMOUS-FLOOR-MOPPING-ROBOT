/* ===========================================================================
 *  tim.c -- TIM7, the 40 kHz step-engine tick.
 *
 *  Called by:  main() -> MX_TIM7_Init(). Started by MB_Init() via
 *              HAL_TIM_Base_Start_IT().
 *  Fires:      TIM7_IRQHandler (stm32f4xx_it.c)
 *                -> HAL_TIM_IRQHandler
 *                  -> HAL_TIM_PeriodElapsedCallback (main.c)
 *                    -> MB_StepISR() (minibot_motion.c)
 *
 *  CLOCK ARITHMETIC
 *      TIM7 is on APB1. APB1 prescaler is /4, and the STM32 clock tree doubles
 *      the timer clock whenever the APB prescaler is not 1. So:
 *          SYSCLK 168 MHz -> PCLK1 42 MHz -> TIM7 clock 84 MHz
 *          84 MHz / 84 = 1 MHz        (PSC = 84-1)
 *           1 MHz / 25 = 40 kHz       (ARR = 25-1)
 *
 *  If you change the clock config, THESE TWO NUMBERS MUST CHANGE TOO, or your
 *  step rates are silently wrong by the same ratio -- and so is your odometry.
 * ========================================================================= */

#include "tim.h"

TIM_HandleTypeDef htim7;

void MX_TIM7_Init(void)
{
    TIM_MasterConfigTypeDef master = {0};

    htim7.Instance               = TIM7;
    htim7.Init.Prescaler         = 84 - 1;      /* 84 MHz -> 1 MHz  */
    htim7.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim7.Init.Period            = 25 - 1;      /*  1 MHz -> 40 kHz */
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
        Error_Handler();
    }

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &master) != HAL_OK) {
        Error_Handler();
    }
}
