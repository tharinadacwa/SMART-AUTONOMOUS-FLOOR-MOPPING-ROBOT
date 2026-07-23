/* ===========================================================================
 *  main.c -- minibot stepper controller, STM32F407VET6.
 *
 *  BOOT ORDER (and why)
 *      HAL_Init()            SysTick, flash prefetch/cache, NVIC grouping
 *      SystemClock_Config()  HSE 8 MHz -> PLL -> 168 MHz
 *      MX_GPIO_Init()        FIRST among peripherals: it puts nENABLE HIGH,
 *                            i.e. motors DEAD, before anything else can twitch.
 *      MX_USART1_UART_Init() so we can report faults if TIM7 init fails
 *      MX_TIM7_Init()        the 40 kHz step engine
 *      MB_Init()             arms UART RX, starts TIM7, motors still disabled
 *      while(1) MB_Task()    parse, watchdog, feedback
 * ========================================================================= */

#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "minibot_motion.h"

static void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();            /* motors disabled here, before anything else */
    MX_USART1_UART_Init();
    MX_TIM7_Init();

    MB_Init();

    while (1) {
        MB_Task();
    }
}

/* ===========================================================================
 *  Clock: HSE 8 MHz -> SYSCLK 168 MHz
 *
 *      VCO_in  = HSE / PLLM  =  8 MHz /   8 =   1 MHz
 *      VCO_out = VCO_in * N  =  1 MHz * 336 = 336 MHz
 *      SYSCLK  = VCO_out / P =    336 /   2 = 168 MHz
 *      48 MHz  = VCO_out / Q =    336 /   7 =  48 MHz  (unused; must be legal)
 *
 *      AHB  /1 -> HCLK  168 MHz
 *      APB1 /4 -> PCLK1  42 MHz -> TIM7 clock  84 MHz  (x2 rule)
 *      APB2 /2 -> PCLK2  84 MHz
 *
 *  ###################### 25 MHz CRYSTAL? READ THIS ######################
 *  Many STM32F407VET6 "black boards" ship a 25 MHz crystal, not 8 MHz. If yours
 *  does and you do not change this, then your BAUD RATE and your STEP RATE are
 *  BOTH wrong by the same 25/8 ratio. The symptom is "garbage on the serial port
 *  AND the motors run at the wrong speed" -- two confusing symptoms at once, and
 *  people lose an evening to it.
 *
 *  The fix is two lines:
 *      stm32f4xx_hal_conf.h :  #define HSE_VALUE ((uint32_t)25000000U)
 *      here                 :  osc.PLL.PLLM = 25;
 *  Leave PLLN/PLLP/PLLQ alone. PLLM exists solely to bring the PLL input down to
 *  1 MHz (8/8 = 1, 25/25 = 1), so everything downstream still lands on 168 MHz.
 *  ####################################################################### */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    /* Scale-1 regulator voltage is required for 168 MHz. */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 8;          /* <-- 25 if you have a 25 MHz crystal */
    osc.PLL.PLLN       = 336;
    osc.PLL.PLLP       = RCC_PLLP_DIV2;
    osc.PLL.PLLQ       = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;

    /* 5 wait states: mandatory at 168 MHz on a 3.3 V part. */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* ===========================================================================
 *  HAL callbacks -> the motion core. This is the whole bridge.
 * ========================================================================= */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7) {
        MB_StepISR();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        MB_UartRxByteISR();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        MB_UartErrorISR();
    }
}

/* ===========================================================================
 *  Error handler. KILL THE MOTORS, then hang.
 * ========================================================================= */
void Error_Handler(void)
{
    __disable_irq();
    DRV_EN_GPIO_Port->BSRR = DRV_EN_Pin;                 /* nENABLE HIGH = off */
    STEP_L_GPIO_Port->BSRR = (uint32_t)STEP_L_Pin << 16;
    STEP_R_GPIO_Port->BSRR = (uint32_t)STEP_R_Pin << 16;
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
    Error_Handler();
}
#endif
