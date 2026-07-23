/* ===========================================================================
 *  gpio.c -- PE3..PE7 -> the two DRV8825 drivers.
 *
 *  Called by: main() -> MX_GPIO_Init(), once, before anything else touches a pin.
 *  Depends on: main.h (pin map).
 *
 *  GPIO_SPEED_FREQ_VERY_HIGH is not cosmetic here. We are generating pulses with
 *  ~2 us edges into a DRV8825 that wants a clean 1.9 us minimum. On a slow slew
 *  rate down a long jumper wire, a square edge becomes a ramp and the driver can
 *  miss it entirely -- which presents as "the motor sometimes just doesn't move".
 * ========================================================================= */

#include "gpio.h"

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* SAFE STARTUP: drivers DISABLED (nENABLE is active LOW -> drive it HIGH),
     * STEP low, DIR low. The motors must not be energised before the firmware
     * knows what it is doing. */
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, STEP_L_Pin | DIR_L_Pin | STEP_R_Pin | DIR_R_Pin,
                      GPIO_PIN_RESET);

    g.Pin   = DRV_EN_Pin | STEP_L_Pin | DIR_L_Pin | STEP_R_Pin | DIR_R_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOE, &g);
}
