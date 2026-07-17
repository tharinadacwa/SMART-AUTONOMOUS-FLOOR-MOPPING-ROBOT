/* ===========================================================================
 *  minibot_motion.c -- see minibot_motion.h for the design rationale.
 * ========================================================================= */

#include "minibot_motion.h"
#include "minibot_protocol.h"
#include "main.h"

#include <math.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef  htim7;

#define L 0
#define R 1

/* 2^32 / MB_ISR_HZ -- accumulator increment per Hz of step rate. */
#define MB_ACC_PER_HZ   (4294967296.0f / (float)MB_ISR_HZ)
#define MB_RAMP_DIV     (MB_ISR_HZ / MB_RAMP_HZ)
#define MB_DT           (1.0f / (float)MB_RAMP_HZ)

/* How aggressively the profile chases the target. The desired acceleration is
 * (target - current) / MB_TAU. Smaller = snappier but more likely to saturate
 * jerk; 0.06 s is a good compromise at 1 kHz. */
#define MB_TAU          0.06f

/* ---------------- shared state ------------------------------------------ */
/* Written by MB_Task (main loop), read by the ISR. int32 writes are atomic on
 * Cortex-M4, so no critical section is needed for these. */
static volatile int32_t  s_target_sps[2] = {0, 0};
static volatile uint8_t  s_estop         = 0;
static volatile uint8_t  s_reset_counts  = 0;

/* Owned exclusively by the ISR. */
static float    s_vel[2]      = {0.0f, 0.0f};   /* signed steps/s            */
static float    s_acc[2]      = {0.0f, 0.0f};   /* signed steps/s^2          */
static uint32_t s_phase[2]    = {0u, 0u};       /* DDS accumulator           */
static uint32_t s_inc[2]      = {0u, 0u};       /* DDS increment             */
static int8_t   s_dir[2]      = {1, 1};         /* +1/-1, matches the DIR pin */
static volatile int32_t s_steps[2] = {0, 0};    /* THE ODOMETRY              */
static uint16_t s_ramp_div    = 0;

/* Owned by MB_Task. */
static uint8_t  s_enabled     = 0;
static uint32_t s_last_cmd_ms = 0;
static uint32_t s_last_fb_ms  = 0;
static uint16_t s_flags       = 0;
static uint16_t s_seq         = 0;

static uint8_t  s_rx_byte;
static char     s_tx_buf[MB_FRAME_MAX];

/* ISR -> main-loop command queue. Single slot; the Pi sends at ~50 Hz and the
 * main loop drains far faster than that, so it can never back up. */
static volatile MB_Cmd  s_cmd;
static volatile uint8_t s_cmd_ready = 0;

/* ---------------- small helpers ----------------------------------------- */

static inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static void mb_tx(const char *s, uint16_t n)
{
    /* Non-blocking. If the previous frame is still in flight we skip this one.
     * Feedback frames are idempotent snapshots -- dropping one is harmless, and
     * the seq counter lets the Pi see that it happened. */
    if (huart1.gState == HAL_UART_STATE_READY && n > 0 && n <= sizeof(s_tx_buf)) {
        memcpy(s_tx_buf, s, n);
        HAL_UART_Transmit_IT(&huart1, (uint8_t *)s_tx_buf, n);
    }
}

/* ========================================================================= */
/*  Layer 2: jerk-limited S-curve profile.  Runs at MB_RAMP_HZ.              */
/* ========================================================================= */

static void mb_profile_update(void)
{
    const float A = MB_MAX_ACCEL_SPS2;
    const float J = MB_MAX_JERK_SPS3;
    const float dA = J * MB_DT;            /* max change in accel per tick */

    for (int i = 0; i < 2; ++i) {

        float target = s_estop ? 0.0f : (float)s_target_sps[i];

        if (s_estop) {
            /* E-STOP BYPASSES THE PROFILE ENTIRELY. There is no scenario where
             * "brake smoothly" beats "stop now" -- losing steps during an
             * emergency stop is an acceptable trade for stopping. */
            s_vel[i] = 0.0f;
            s_acc[i] = 0.0f;
        } else {
            float verr  = target - s_vel[i];

            /* Desired acceleration, saturated at a_max. */
            float a_des = clampf(verr / MB_TAU, -A, A);

            /* JERK LIMIT: acceleration itself may only change so fast. This is
             * what rounds the corners of the trapezoid into an S. */
            s_acc[i] += clampf(a_des - s_acc[i], -dA, dA);
            s_acc[i]  = clampf(s_acc[i], -A, A);

            s_vel[i] += s_acc[i] * MB_DT;

            /* Snap on arrival, so we do not dither around the target forever. */
            if (fabsf(target - s_vel[i]) < 1.0f) {
                s_vel[i] = target;
                s_acc[i] = 0.0f;
            }
        }

        /* Hard clamp -- belt and braces. The DDS low-time proof depends on this
         * never being exceeded, so we do not trust the profile alone. */
        s_vel[i] = clampf(s_vel[i], -(float)MB_MAX_STEP_RATE,
                                     (float)MB_MAX_STEP_RATE);

        /* Direction. Because s_vel is SIGNED and the profile passes through
         * zero, the DIR pin only flips while the motor is essentially stopped. */
        if (s_vel[i] >  0.5f) s_dir[i] = +1;
        else if (s_vel[i] < -0.5f) s_dir[i] = -1;
        /* else: hold the previous direction; we are not stepping anyway. */

        int level = (s_dir[i] > 0) ? 1 : 0;
        if (i == L) {
#if MB_LEFT_DIR_INVERT
            level = !level;
#endif
            DIR_L_GPIO_Port->BSRR = level ? DIR_L_Pin
                                          : ((uint32_t)DIR_L_Pin << 16);
        } else {
#if MB_RIGHT_DIR_INVERT
            level = !level;
#endif
            DIR_R_GPIO_Port->BSRR = level ? DIR_R_Pin
                                          : ((uint32_t)DIR_R_Pin << 16);
        }

        float f = fabsf(s_vel[i]);
        s_inc[i] = (f < 1.0f) ? 0u : (uint32_t)(f * MB_ACC_PER_HZ);
    }

    if (s_reset_counts) {
        s_steps[L] = 0;
        s_steps[R] = 0;
        s_reset_counts = 0;
    }
}

/* ========================================================================= */
/*  Layer 1: DDS pulse engine.  Runs at MB_ISR_HZ (40 kHz).                  */
/* ========================================================================= */

void MB_StepISR(void)
{
    /* Terminate the pulse started on the previous tick. See the header for why
     * this ordering is what guarantees the DRV8825 its minimum low time. */
    STEP_L_GPIO_Port->BSRR = (uint32_t)STEP_L_Pin << 16;
    STEP_R_GPIO_Port->BSRR = (uint32_t)STEP_R_Pin << 16;

    if (++s_ramp_div >= MB_RAMP_DIV) {
        s_ramp_div = 0;
        mb_profile_update();
    }

    uint32_t prev;

    prev = s_phase[L];
    s_phase[L] += s_inc[L];
    if (s_inc[L] && (s_phase[L] < prev)) {          /* 32-bit wrap = one step */
        STEP_L_GPIO_Port->BSRR = STEP_L_Pin;
        s_steps[L] += s_dir[L];
    }

    prev = s_phase[R];
    s_phase[R] += s_inc[R];
    if (s_inc[R] && (s_phase[R] < prev)) {
        STEP_R_GPIO_Port->BSRR = STEP_R_Pin;
        s_steps[R] += s_dir[R];
    }
}

/* ========================================================================= */
/*  UART                                                                     */
/* ========================================================================= */

void MB_UartRxByteISR(void)
{
    MB_Cmd cmd;
    bool crc_err = false;

    if (MB_ProtoRxByte((char)s_rx_byte, &cmd, &crc_err)) {
        if (!s_cmd_ready) {
            s_cmd = cmd;
            s_cmd_ready = 1;
        }
        /* If the main loop has not drained the previous command yet we drop
         * this one. At 50 Hz command rate against a loop running in the tens of
         * kHz, this cannot happen in practice -- but dropping a stale velocity
         * command is strictly safer than queueing it. */
    }
    if (crc_err) {
        s_flags |= MB_FLAG_CRC_ERR;
    }

    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

void MB_UartErrorISR(void)
{
    /* An unhandled ORE (overrun) silently kills RX forever. Clear every error
     * flag and re-arm. */
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    s_flags |= MB_FLAG_OVERRUN;
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

/* ========================================================================= */
/*  Command handling                                                         */
/* ========================================================================= */

static int32_t mb_clamp_sps(int32_t v)
{
    if (v >  MB_MAX_STEP_RATE) { s_flags |= MB_FLAG_CLAMPED; return  MB_MAX_STEP_RATE; }
    if (v < -MB_MAX_STEP_RATE) { s_flags |= MB_FLAG_CLAMPED; return -MB_MAX_STEP_RATE; }
    return v;
}

static void mb_enable(uint8_t on)
{
    if (on) {
        DRV_EN_GPIO_Port->BSRR = (uint32_t)DRV_EN_Pin << 16;   /* active LOW */
        s_enabled = 1;
        s_flags |= MB_FLAG_ENABLED;
    } else {
        DRV_EN_GPIO_Port->BSRR = DRV_EN_Pin;
        s_enabled = 0;
        s_flags &= (uint16_t)~MB_FLAG_ENABLED;
    }
}

static void mb_handle(const MB_Cmd *c)
{
    switch (c->type) {

    case MB_CMD_VELOCITY:
        s_target_sps[L] = mb_clamp_sps(c->left_sps);
        s_target_sps[R] = mb_clamp_sps(c->right_sps);
        s_estop = 0;
        s_flags &= (uint16_t)~(MB_FLAG_ESTOP | MB_FLAG_CMD_TIMEOUT);
        s_last_cmd_ms = HAL_GetTick();
        if (!s_enabled) mb_enable(1);
        break;

    case MB_CMD_ENABLE:
        if (c->enable) {
            mb_enable(1);
            s_estop = 0;
            s_flags &= (uint16_t)~MB_FLAG_ESTOP;
        } else {
            s_target_sps[L] = 0;
            s_target_sps[R] = 0;
            s_estop = 1;
            mb_enable(0);
        }
        s_last_cmd_ms = HAL_GetTick();
        break;

    case MB_CMD_ESTOP:
        MB_EmergencyStop();
        s_last_cmd_ms = HAL_GetTick();
        break;

    case MB_CMD_RESET:
        s_reset_counts = 1;
        break;

    case MB_CMD_PING: {
        uint16_t n = MB_ProtoBuildSimple(s_tx_buf, sizeof(s_tx_buf), 'K');
        mb_tx(s_tx_buf, n);
        break;
    }

    default:
        break;
    }
}

void MB_EmergencyStop(void)
{
    s_target_sps[L] = 0;
    s_target_sps[R] = 0;
    s_estop = 1;
    s_flags |= MB_FLAG_ESTOP;
    /* Kill the pulses immediately; the profile will zero the velocity on its
     * next tick (25 us away), and mb_profile_update short-circuits on estop. */
    s_inc[L] = 0;
    s_inc[R] = 0;
    STEP_L_GPIO_Port->BSRR = (uint32_t)STEP_L_Pin << 16;
    STEP_R_GPIO_Port->BSRR = (uint32_t)STEP_R_Pin << 16;
}

/* ========================================================================= */
/*  Public                                                                   */
/* ========================================================================= */

void MB_Init(void)
{
    mb_enable(0);                                  /* motors dead at boot */
    STEP_L_GPIO_Port->BSRR = (uint32_t)STEP_L_Pin << 16;
    STEP_R_GPIO_Port->BSRR = (uint32_t)STEP_R_Pin << 16;

    s_last_cmd_ms = HAL_GetTick();
    s_last_fb_ms  = s_last_cmd_ms;
    s_flags       = 0;
    s_seq         = 0;

    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
    HAL_TIM_Base_Start_IT(&htim7);
}

void MB_Task(void)
{
    uint32_t now = HAL_GetTick();

    if (s_cmd_ready) {
        MB_Cmd local;
        __disable_irq();
        local = *(MB_Cmd *)&s_cmd;
        s_cmd_ready = 0;
        __enable_irq();
        mb_handle(&local);
    }

    uint32_t since = now - s_last_cmd_ms;

    /* COMMS WATCHDOG. If ROS dies, the network drops, or the cable falls out,
     * a robot that keeps executing its last velocity command is a robot that
     * drives into a wall. Ramp to a stop -- but stay ENERGISED, so we keep
     * holding torque and do not roll away on a slope. */
    if (since > MB_CMD_TIMEOUT_MS) {
        s_target_sps[L] = 0;
        s_target_sps[R] = 0;
        s_flags |= MB_FLAG_CMD_TIMEOUT;
    }

    /* After a long idle, drop the coils. A stationary stepper at full current
     * is a 10 W heater doing no work. */
    if (s_enabled && since > MB_IDLE_DISABLE_MS) {
        mb_enable(0);
    }

    if ((now - s_last_fb_ms) >= (1000u / MB_FEEDBACK_HZ)) {
        s_last_fb_ms = now;

        __disable_irq();
        int32_t sl = s_steps[L];
        int32_t sr = s_steps[R];
        int32_t vl = (int32_t)s_vel[L];
        int32_t vr = (int32_t)s_vel[R];
        __enable_irq();

        char buf[MB_FRAME_MAX];
        uint16_t n = MB_ProtoBuildFeedback(buf, sizeof(buf), sl, sr, vl, vr,
                                           s_flags, s_seq++);
        mb_tx(buf, n);
    }
}

int32_t  MB_GetStepsLeft(void)  { return s_steps[L]; }
int32_t  MB_GetStepsRight(void) { return s_steps[R]; }
uint16_t MB_GetFlags(void)      { return s_flags; }
