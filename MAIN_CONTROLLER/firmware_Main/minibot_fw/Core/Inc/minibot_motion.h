/* ===========================================================================
 *  minibot_motion.h -- the step engine.
 *
 *  All tuning constants come from minibot_config.h, which is GENERATED from
 *  robot.yaml by tools/generate_config.py. Do not hand-edit them.
 *
 *  HOW IT WORKS, IN THREE LAYERS
 *
 *  1. DDS PULSE ENGINE (runs in the TIM7 ISR at MB_ISR_HZ = 40 kHz)
 *     Each motor owns a 32-bit phase accumulator:
 *
 *         inc  = step_rate_hz * (2^32 / MB_ISR_HZ)
 *         acc += inc                                  every tick
 *         if (acc wrapped past 2^32)  ->  emit ONE step pulse
 *
 *     Direct digital synthesis. Produces any rate from 1 Hz to the ceiling with
 *     sub-tick average accuracy, and needs zero timer reconfiguration to change
 *     speed. The step COUNTER is incremented in the same breath, which is why
 *     the counts we report are exact -- they are not an estimate, they are a
 *     tally of pulses we actually emitted.
 *
 *     PULSE SHAPE: STEP is driven LOW at the top of the ISR and HIGH at the
 *     bottom. That gives ~23 us high and >= 25 us low. The DRV8825 needs 1.9 us
 *     of each. The low-time guarantee rests on a proof: if inc < 0.5 (i.e. step
 *     rate < MB_ISR_HZ/2) then after a wrap the accumulator holds a value < inc
 *     < 0.5, so the NEXT tick cannot possibly wrap. Two pulses can never land on
 *     consecutive ticks. generate_config.py enforces this at build time.
 *
 *  2. JERK-LIMITED PROFILE (runs at MB_RAMP_HZ = 1 kHz, inside the same ISR)
 *     A plain trapezoidal ramp steps the acceleration discontinuously from 0 to
 *     a_max. For a stepper that is a torque step -- exactly the impulse that
 *     makes it skip. So we limit d(accel)/dt as well:
 *
 *         a += clamp(a_desired - a, -jerk*dt, +jerk*dt)
 *         v += a * dt
 *
 *     That is an S-curve. Your coverage robot reverses direction at the end of
 *     EVERY lane -- ~19 times per room -- so this is not a refinement, it is
 *     what keeps the odometry honest across a 15-minute run.
 *
 *  3. SIGNED VELOCITY
 *     v is signed and the profile passes THROUGH zero on a reversal, so the DIR
 *     pin only ever flips while the motor is essentially stopped. Flipping DIR
 *     at speed is the other classic way to lose steps.
 * ========================================================================= */

#ifndef MINIBOT_MOTION_H
#define MINIBOT_MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "minibot_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Call once from main(), after the peripherals are up, before while(1). */
void MB_Init(void);

/* Call from while(1). Non-blocking: parses commands, runs the watchdog,
 * emits the feedback frame. */
void MB_Task(void);

/* From HAL_TIM_PeriodElapsedCallback, TIM7. 40 kHz. Keep it short. */
void MB_StepISR(void);

/* From HAL_UART_RxCpltCallback, USART1. */
void MB_UartRxByteISR(void);

/* From HAL_UART_ErrorCallback, USART1. */
void MB_UartErrorISR(void);

/* Cut motor power immediately. Safe to call from a fault handler. */
void MB_EmergencyStop(void);

/* Diagnostics */
int32_t  MB_GetStepsLeft(void);
int32_t  MB_GetStepsRight(void);
uint16_t MB_GetFlags(void);

#ifdef __cplusplus
}
#endif

#endif /* MINIBOT_MOTION_H */
