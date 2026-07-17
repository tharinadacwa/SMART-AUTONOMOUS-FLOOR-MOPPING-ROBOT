/* ===========================================================================
 *  minibot_protocol.h -- framed, CRC-protected serial link to the Raspberry Pi.
 *
 *  WIRE FORMAT (ASCII, so you can debug it with a terminal)
 *
 *      $<PAYLOAD>*<CRC8><LF>
 *
 *      $        start of frame. Resynchronises the parser after any garbage.
 *      PAYLOAD  comma-separated fields, first field = command letter
 *      *        end of payload
 *      CRC8     two uppercase hex digits, CRC-8/ATM (poly 0x07, init 0x00)
 *               computed over PAYLOAD only (between $ and *)
 *      LF       \n
 *
 *  Pi -> STM32
 *      $V,<left_sps>,<right_sps>*XX     signed steps/second target
 *      $E,<0|1>*XX                      driver enable / disable
 *      $S*XX                            EMERGENCY STOP (bypasses the ramp)
 *      $R*XX                            reset step counters to zero
 *      $P*XX                            ping -> replies $K*XX
 *
 *  STM32 -> Pi   (at MB_FEEDBACK_HZ)
 *      $F,<l_steps>,<r_steps>,<l_sps>,<r_sps>,<flags>,<seq>*XX
 *
 *          l_steps/r_steps  int32, signed, cumulative. THIS IS YOUR ODOMETRY.
 *          l_sps/r_sps      int32, the ACTUAL current step rate (post-profile)
 *          flags            bitfield, see MB_FLAG_* below
 *          seq              uint16 frame counter, wraps. Lets the Pi detect
 *                           dropped frames rather than silently interpolating
 *                           across them.
 *
 *  WHY CRC AND NOT A BARE CHECKSUM
 *      The motor wires next to this cable are switching amps at 20 kHz. A
 *      single flipped bit in a velocity command is a robot that drives into a
 *      wall. CRC-8 catches all single-bit, all double-bit, and all burst errors
 *      up to 8 bits. A sum or XOR does not.
 * ========================================================================= */

#ifndef MINIBOT_PROTOCOL_H
#define MINIBOT_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define MB_FRAME_MAX        64

/* feedback flags */
#define MB_FLAG_ENABLED     (1u << 0)   /* drivers energised                  */
#define MB_FLAG_ESTOP       (1u << 1)   /* emergency stop latched             */
#define MB_FLAG_CMD_TIMEOUT (1u << 2)   /* no command from the Pi -> stopped  */
#define MB_FLAG_CRC_ERR     (1u << 3)   /* at least one bad CRC since boot    */
#define MB_FLAG_OVERRUN     (1u << 4)   /* UART overrun seen since boot       */
#define MB_FLAG_CLAMPED     (1u << 5)   /* last command exceeded MB_MAX_STEP_RATE */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MB_CMD_NONE = 0,
    MB_CMD_VELOCITY,      /* V */
    MB_CMD_ENABLE,        /* E */
    MB_CMD_ESTOP,         /* S */
    MB_CMD_RESET,         /* R */
    MB_CMD_PING           /* P */
} MB_CmdType;

typedef struct {
    MB_CmdType type;
    int32_t    left_sps;
    int32_t    right_sps;
    uint8_t    enable;
} MB_Cmd;

/* CRC-8/ATM: poly 0x07, init 0x00, no reflection, no final xor. */
uint8_t MB_Crc8(const char *data, uint16_t len);

/* Feed one received byte. Returns true and fills *out when a VALID frame
 * completes. Silently discards frames with a bad CRC (and sets *crc_err). */
bool MB_ProtoRxByte(char c, MB_Cmd *out, bool *crc_err);

/* Build a feedback frame into buf (NUL-terminated). Returns its length. */
uint16_t MB_ProtoBuildFeedback(char *buf, uint16_t cap,
                               int32_t l_steps, int32_t r_steps,
                               int32_t l_sps, int32_t r_sps,
                               uint16_t flags, uint16_t seq);

/* Build a bare "$<letter>*CRC\n" frame (used for the K ping reply). */
uint16_t MB_ProtoBuildSimple(char *buf, uint16_t cap, char letter);

#ifdef __cplusplus
}
#endif

#endif /* MINIBOT_PROTOCOL_H */
