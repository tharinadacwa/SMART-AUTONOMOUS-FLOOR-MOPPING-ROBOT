/* main.h -- minibot STM32F407VET6 stepper controller. */
#ifndef __MAIN_H
#define __MAIN_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Error_Handler(void);

/* ---------------------------------------------------------------------------
 *  PIN MAP  (exactly as wired)
 *
 *    PE3  nENABLE   -> BOTH DRV8825 nENABLE pins, tied together. ACTIVE LOW.
 *    PE4  STEP      -> LEFT  DRV8825 STEP
 *    PE5  DIR       -> LEFT  DRV8825 DIR
 *    PE6  STEP      -> RIGHT DRV8825 STEP
 *    PE7  DIR       -> RIGHT DRV8825 DIR
 *    PA9  USART1_TX -> Raspberry Pi 5 RX
 *    PA10 USART1_RX -> Raspberry Pi 5 TX
 * ------------------------------------------------------------------------- */
#define DRV_EN_Pin          GPIO_PIN_3
#define DRV_EN_GPIO_Port    GPIOE
#define STEP_L_Pin          GPIO_PIN_4
#define STEP_L_GPIO_Port    GPIOE
#define DIR_L_Pin           GPIO_PIN_5
#define DIR_L_GPIO_Port     GPIOE
#define STEP_R_Pin          GPIO_PIN_6
#define STEP_R_GPIO_Port    GPIOE
#define DIR_R_Pin           GPIO_PIN_7
#define DIR_R_GPIO_Port     GPIOE

/* ---------------------------------------------------------------------------
 *  L298N DC-MOTOR PIN MAP  (added -- 4 motors on 2 drivers, 8 IN lines)
 *
 *  Only these 8 pins are free on this board, and all 8 are used as L298N IN
 *  lines. There is therefore NO spare pin for the enable inputs, so on each
 *  driver the (tied-together) ENA+ENB node must go to +5V -- see the wiring
 *  guide. Direction and stop are done entirely with the IN pins.
 *
 *    DRIVER 1  (ENA+ENB tied together -> +5V):
 *      PD0  M1_IN1  -> DRIVER1 IN1  \__ Motor 1  (OUT1/OUT2)
 *      PD1  M1_IN2  -> DRIVER1 IN2  /
 *      PD2  M2_IN3  -> DRIVER1 IN3  \__ Motor 2  (OUT3/OUT4)
 *      PD3  M2_IN4  -> DRIVER1 IN4  /
 *    DRIVER 2  (ENA+ENB tied together -> +5V):
 *      PA2  M3_IN1  -> DRIVER2 IN1  \__ Motor 3  (OUT1/OUT2)
 *      PA3  M3_IN2  -> DRIVER2 IN2  /
 *      PE14 M4_IN3  -> DRIVER2 IN3  \__ Motor 4  (OUT3/OUT4)
 *      PE15 M4_IN4  -> DRIVER2 IN4  /
 * ------------------------------------------------------------------------- */
#define M1_IN1_Pin          GPIO_PIN_0
#define M1_IN1_GPIO_Port    GPIOD
#define M1_IN2_Pin          GPIO_PIN_1
#define M1_IN2_GPIO_Port    GPIOD
#define M2_IN3_Pin          GPIO_PIN_2
#define M2_IN3_GPIO_Port    GPIOD
#define M2_IN4_Pin          GPIO_PIN_3
#define M2_IN4_GPIO_Port    GPIOD
#define M3_IN1_Pin          GPIO_PIN_2
#define M3_IN1_GPIO_Port    GPIOA
#define M3_IN2_Pin          GPIO_PIN_3
#define M3_IN2_GPIO_Port    GPIOA
#define M4_IN3_Pin          GPIO_PIN_14
#define M4_IN3_GPIO_Port    GPIOE
#define M4_IN4_Pin          GPIO_PIN_15
#define M4_IN4_GPIO_Port    GPIOE

/* ---------------------------------------------------------------------------
 *  HEARTBEAT SIGNAL (added)
 *    PA12  SIG  -> IRLZ44N gate (via 220R): HIGH 3 s (pump ON), LOW 5 s (OFF).
 *                  Free pin (USB is not used in this project).
 * ------------------------------------------------------------------------- */
#define SIG_Pin             GPIO_PIN_12
#define SIG_GPIO_Port       GPIOA

#ifdef __cplusplus
}
#endif
#endif /* __MAIN_H */
