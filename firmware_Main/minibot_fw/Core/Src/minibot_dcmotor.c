/* ===========================================================================
 *  minibot_dcmotor.c -- 4x brushed DC motors via 2x L298N H-bridges.
 *
 *  See minibot_dcmotor.h for the design summary and main.h for the pin map.
 *
 *  ONE L298N CHANNEL, DIRECTION TRUTH TABLE (with EN tied HIGH):
 *      IN_a  IN_b   result
 *        1    0     forward   (labelled CW here)
 *        0    1     reverse   (labelled CCW here)
 *        0    0     coast     (motor free-wheels)   <- our "stop"
 *        1    1     brake     (we never drive this)
 *
 *  We only ever output 1/0, 0/1 or 0/0.
 * ========================================================================= */

#include "minibot_dcmotor.h"
#include "main.h"

/* One row per H-bridge channel: the two IN pins that steer it.
 * "a" high + "b" low is the direction we label CW. */
typedef struct {
    GPIO_TypeDef *port_a;  uint16_t pin_a;   /* first  IN pin */
    GPIO_TypeDef *port_b;  uint16_t pin_b;   /* second IN pin */
} DCM_Channel;

static const DCM_Channel s_ch[DCM_MOTOR_COUNT] = {
    /* DCM_MOTOR_1 */ { M1_IN1_GPIO_Port, M1_IN1_Pin, M1_IN2_GPIO_Port, M1_IN2_Pin },
    /* DCM_MOTOR_2 */ { M2_IN3_GPIO_Port, M2_IN3_Pin, M2_IN4_GPIO_Port, M2_IN4_Pin },
    /* DCM_MOTOR_3 */ { M3_IN1_GPIO_Port, M3_IN1_Pin, M3_IN2_GPIO_Port, M3_IN2_Pin },
    /* DCM_MOTOR_4 */ { M4_IN3_GPIO_Port, M4_IN3_Pin, M4_IN4_GPIO_Port, M4_IN4_Pin },
};

/* Atomic single-pin set/clear via BSRR (no read-modify-write, IRQ-safe). */
static inline void pin_write(GPIO_TypeDef *port, uint16_t pin, int high)
{
    port->BSRR = high ? (uint32_t)pin : ((uint32_t)pin << 16);
}

void DCM_Set(DCM_Motor m, DCM_Dir d)
{
    if (m >= DCM_MOTOR_COUNT) {
        return;
    }
    const DCM_Channel *c = &s_ch[m];

    int a, b;
    switch (d) {
        case DCM_CW:  a = 1; b = 0; break;
        case DCM_CCW: a = 0; b = 1; break;
        default:      a = 0; b = 0; break;   /* DCM_STOP -> coast */
    }
    pin_write(c->port_a, c->pin_a, a);
    pin_write(c->port_b, c->pin_b, b);
}

void DCM_StopAll(void)
{
    for (int i = 0; i < DCM_MOTOR_COUNT; ++i) {
        DCM_Set((DCM_Motor)i, DCM_STOP);
    }
}

void DCM_Run(void)
{
    /* Spec: on EACH driver, one motor turns CW and the other CCW. */
    DCM_Set(DCM_MOTOR_1, DCM_CW);    /* driver 1, channel A */
    DCM_Set(DCM_MOTOR_2, DCM_CCW);   /* driver 1, channel B */
    DCM_Set(DCM_MOTOR_3, DCM_CW);    /* driver 2, channel A */
    DCM_Set(DCM_MOTOR_4, DCM_CCW);   /* driver 2, channel B */
}

void DCM_Init(void)
{
    GPIO_InitTypeDef g = {0};

    /* GPIOA and GPIOE are already clocked by MX_GPIO_Init(); GPIOD is not.
     * Re-enabling an already-enabled clock is a harmless no-op, so we enable
     * all three here to stay independent of call order. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* SAFE STARTUP: pre-load every IN pin LOW (0/0 = coast) in the output data
     * register BEFORE switching the pins to outputs, so the first level driven
     * is LOW and no motor twitches during bring-up. Mirrors gpio.c. */
    pin_write(M1_IN1_GPIO_Port, M1_IN1_Pin, 0);
    pin_write(M1_IN2_GPIO_Port, M1_IN2_Pin, 0);
    pin_write(M2_IN3_GPIO_Port, M2_IN3_Pin, 0);
    pin_write(M2_IN4_GPIO_Port, M2_IN4_Pin, 0);
    pin_write(M3_IN1_GPIO_Port, M3_IN1_Pin, 0);
    pin_write(M3_IN2_GPIO_Port, M3_IN2_Pin, 0);
    pin_write(M4_IN3_GPIO_Port, M4_IN3_Pin, 0);
    pin_write(M4_IN4_GPIO_Port, M4_IN4_Pin, 0);

    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;    /* DC direction lines: no fast edges needed */

    /* GPIOD: PD0, PD1, PD2, PD3  (driver 1) */
    g.Pin = M1_IN1_Pin | M1_IN2_Pin | M2_IN3_Pin | M2_IN4_Pin;
    HAL_GPIO_Init(GPIOD, &g);

    /* GPIOA: PA2, PA3  (driver 2, motor 3) */
    g.Pin = M3_IN1_Pin | M3_IN2_Pin;
    HAL_GPIO_Init(GPIOA, &g);

    /* GPIOE: PE14, PE15  (driver 2, motor 4) */
    g.Pin = M4_IN3_Pin | M4_IN4_Pin;
    HAL_GPIO_Init(GPIOE, &g);
}
