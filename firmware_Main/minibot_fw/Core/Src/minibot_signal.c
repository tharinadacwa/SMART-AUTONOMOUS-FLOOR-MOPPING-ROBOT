/* ===========================================================================
 *  minibot_signal.c -- pump drive on PA12: 3 s HIGH (pump ON), 5 s LOW (OFF).
 *
 *  PA12 -> 220 ohm -> IRLZ44N gate (low-side switch). HIGH = MOSFET on = pump
 *  runs; LOW = MOSFET off = pump stopped. One full cycle is 8 s (3 on, 5 off).
 *  Timing comes from the 1 kHz SysTick (HAL_GetTick, milliseconds); 1 ms
 *  resolution is far finer than these intervals, so the edges are exact.
 * ========================================================================= */

#include "minibot_signal.h"
#include "main.h"

#define SIG_HIGH_MS   3000u   /* pump ON  time */
#define SIG_LOW_MS    5000u   /* pump OFF time */

static uint32_t s_last_edge_ms;
static uint8_t  s_level;             /* 0 = LOW, 1 = HIGH */

void Sig_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();    /* already on from MX_GPIO_Init; harmless */

    /* Start LOW before switching the pin to an output. */
    HAL_GPIO_WritePin(SIG_GPIO_Port, SIG_Pin, GPIO_PIN_RESET);

    GPIO_InitTypeDef g = {0};
    g.Pin   = SIG_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;   /* slow line: no fast edges needed */
    HAL_GPIO_Init(SIG_GPIO_Port, &g);

    s_level        = 0;
    s_last_edge_ms = HAL_GetTick();
}

void Sig_Task(void)
{
    uint32_t now = HAL_GetTick();

    /* Dwell depends on the current level: 3 s while HIGH, 5 s while LOW. */
    uint32_t dwell = s_level ? SIG_HIGH_MS : SIG_LOW_MS;

    /* Unsigned subtraction handles the 49-day tick wrap correctly. */
    if ((now - s_last_edge_ms) >= dwell) {
        s_last_edge_ms = now;
        s_level ^= 1u;
        HAL_GPIO_WritePin(SIG_GPIO_Port, SIG_Pin,
                          s_level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}
