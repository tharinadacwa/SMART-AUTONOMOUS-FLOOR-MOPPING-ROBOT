/* ===========================================================================
 *  stm32f4xx_it.c -- interrupt vectors.
 *
 *  The fault handlers KILL THE MOTORS before hanging. A robot whose firmware
 *  has hard-faulted must not still be driving across the room. This costs three
 *  register writes and it is the difference between a debugging session and a
 *  hole in your wall.
 * ========================================================================= */

#include "main.h"
#include "stm32f4xx_it.h"
#include "tim.h"
#include "usart.h"
#include "minibot_motion.h"

/* ---- core exceptions ---------------------------------------------------- */

static inline void kill_motors(void)
{
    DRV_EN_GPIO_Port->BSRR = DRV_EN_Pin;                        /* disable   */
    STEP_L_GPIO_Port->BSRR = (uint32_t)STEP_L_Pin << 16;        /* step low  */
    STEP_R_GPIO_Port->BSRR = (uint32_t)STEP_R_Pin << 16;
}

void NMI_Handler(void)        { kill_motors(); while (1) { } }
void HardFault_Handler(void)  { kill_motors(); while (1) { } }
void MemManage_Handler(void)  { kill_motors(); while (1) { } }
void BusFault_Handler(void)   { kill_motors(); while (1) { } }
void UsageFault_Handler(void) { kill_motors(); while (1) { } }

void SVC_Handler(void)        { }
void DebugMon_Handler(void)   { }
void PendSV_Handler(void)     { }

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* ---- peripherals -------------------------------------------------------- */

/* 40 kHz. -> HAL_TIM_PeriodElapsedCallback -> MB_StepISR() */
void TIM7_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim7);
}

/* -> HAL_UART_RxCpltCallback -> MB_UartRxByteISR()
 * -> HAL_UART_ErrorCallback -> MB_UartErrorISR()  */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}
