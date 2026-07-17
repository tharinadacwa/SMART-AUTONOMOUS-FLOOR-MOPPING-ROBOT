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

#ifdef __cplusplus
}
#endif
#endif /* __MAIN_H */
