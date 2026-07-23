/* ===========================================================================
 *  minibot_dcmotor.h -- 4x brushed DC motors on 2x L298N H-bridges.
 *
 *  This module is fully ADDITIVE. It does not touch the stepper step-engine,
 *  the UART protocol, TIM7, or any pin used by the original firmware. It owns
 *  only eight GPIO lines that were previously unused (see the pin map in
 *  main.h): PD0 PD1 PD2 PD3 (driver 1) and PA2 PA3 PE14 PE15 (driver 2).
 *
 *  ENA and ENB on each driver are tied together and taken to +5V, so both
 *  channels are permanently enabled. That means there is no PWM speed control
 *  here -- the motors run at whatever speed the 12V rail delivers, and the
 *  firmware only chooses direction (or stop) through the IN pins.
 *
 *  "CW" / "CCW" are just labels; the physical spin direction depends on how
 *  the two motor leads land on OUTx. If a motor turns the wrong way, swap its
 *  two output wires, or swap DCM_CW/DCM_CCW for that motor.
 * ========================================================================= */

#ifndef MINIBOT_DCMOTOR_H
#define MINIBOT_DCMOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Logical motor IDs. 1&2 live on driver 1, 3&4 on driver 2. */
typedef enum {
    DCM_MOTOR_1 = 0,   /* Driver 1, channel A (OUT1/OUT2) */
    DCM_MOTOR_2,       /* Driver 1, channel B (OUT3/OUT4) */
    DCM_MOTOR_3,       /* Driver 2, channel A (OUT1/OUT2) */
    DCM_MOTOR_4,       /* Driver 2, channel B (OUT3/OUT4) */
    DCM_MOTOR_COUNT
} DCM_Motor;

typedef enum {
    DCM_STOP = 0,      /* both IN low  -> motor coasts (free-wheels) */
    DCM_CW,            /* clockwise         (IN_a = 1, IN_b = 0)      */
    DCM_CCW            /* counter-clockwise (IN_a = 0, IN_b = 1)      */
} DCM_Dir;

/* Configure the 8 IN pins as outputs, all LOW (every motor stopped).
 * Call once from main(), after MX_GPIO_Init(). */
void DCM_Init(void);

/* Drive one motor in a direction (or stop it). */
void DCM_Set(DCM_Motor m, DCM_Dir d);

/* Apply the required run pattern: on EACH driver one motor turns CW and the
 * other CCW  ->  M1 CW, M2 CCW, M3 CW, M4 CCW. */
void DCM_Run(void);

/* Stop every motor (all IN pins LOW -> coast). Safe from a fault handler. */
void DCM_StopAll(void);

#ifdef __cplusplus
}
#endif

#endif /* MINIBOT_DCMOTOR_H */
