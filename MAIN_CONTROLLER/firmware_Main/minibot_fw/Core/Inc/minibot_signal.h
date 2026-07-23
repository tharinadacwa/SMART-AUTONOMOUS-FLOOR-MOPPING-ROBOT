/* ===========================================================================
 *  minibot_signal.h -- timed pump drive on PA12 (low-side MOSFET).
 *
 *  Drives PA12 HIGH for 3 s (pump ON) then LOW for 5 s (pump OFF), forever.
 *  Fully ADDITIVE: it owns only PA12 and touches nothing else.
 *
 *  NON-BLOCKING BY DESIGN. The toggle is scheduled off HAL_GetTick() from the
 *  main loop, so it never stalls MB_Task() (the stepper comms watchdog and
 *  feedback must keep running). No blocking delay is used.
 * ========================================================================= */

#ifndef MINIBOT_SIGNAL_H
#define MINIBOT_SIGNAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Configure PA12 as an output, driven LOW. Call once from main(),
 * after MX_GPIO_Init(). */
void Sig_Init(void);

/* Call every iteration of the main while(1) loop. Non-blocking: it flips PA12
 * on the 3 s / 5 s schedule. */
void Sig_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* MINIBOT_SIGNAL_H */
