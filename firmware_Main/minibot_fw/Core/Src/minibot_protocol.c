/* minibot_protocol.c -- see minibot_protocol.h */

#include "minibot_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* CRC-8/ATM, bitwise. At 115200 baud a frame is <= 64 bytes, so this runs
 * ~500 times a second. Bitwise is plenty; a table would be premature. */
uint8_t MB_Crc8(const char *data, uint16_t len)
{
    uint8_t crc = 0x00u;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= (uint8_t)data[i];
        for (uint8_t b = 0; b < 8; ++b) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u)
                                : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static int hexval(char c)
{
    if (c >= (char)0x30 && c <= (char)0x39) return c - 0x30;        /* 0-9 */
    if (c >= (char)0x41 && c <= (char)0x46) return c - 0x41 + 10;   /* A-F */
    if (c >= (char)0x61 && c <= (char)0x66) return c - 0x61 + 10;   /* a-f */
    return -1;
}

/* Receiver state machine. Deliberately tiny and allocation-free -- it runs in
 * the USART1 ISR at priority 1, underneath the 40 kHz step engine. */
typedef enum { S_IDLE, S_PAYLOAD, S_CRC1, S_CRC2 } RxState;

static RxState  s_state = S_IDLE;
static char     s_payload[MB_FRAME_MAX];
static uint16_t s_len;
static uint8_t  s_crc_hi;

static bool parse_payload(const char *p, MB_Cmd *out)
{
    out->type      = MB_CMD_NONE;
    out->left_sps  = 0;
    out->right_sps = 0;
    out->enable    = 0;

    switch (p[0]) {
    case 0x56: {                                   /* V */
        long l = 0, r = 0;
        if (sscanf(p + 1, ",%ld,%ld", &l, &r) != 2) return false;
        out->type      = MB_CMD_VELOCITY;
        out->left_sps  = (int32_t)l;
        out->right_sps = (int32_t)r;
        return true;
    }
    case 0x45: {                                   /* E */
        int e = 0;
        if (sscanf(p + 1, ",%d", &e) != 1) return false;
        out->type   = MB_CMD_ENABLE;
        out->enable = (uint8_t)(e ? 1 : 0);
        return true;
    }
    case 0x53:  out->type = MB_CMD_ESTOP; return true;   /* S */
    case 0x52:  out->type = MB_CMD_RESET; return true;   /* R */
    case 0x50:  out->type = MB_CMD_PING;  return true;   /* P */
    default:    return false;
    }
}

bool MB_ProtoRxByte(char c, MB_Cmd *out, bool *crc_err)
{
    /* $ ALWAYS restarts the frame, from any state. This is what lets the
     * parser resynchronise after line noise instead of wedging forever. */
    if (c == 0x24) {                               /* $ */
        s_state = S_PAYLOAD;
        s_len   = 0;
        return false;
    }

    switch (s_state) {

    case S_IDLE:
        return false;                              /* junk before a $ */

    case S_PAYLOAD:
        if (c == 0x2A) {                           /* * */
            s_state = S_CRC1;
        } else if (c == 0x0A || c == 0x0D) {
            s_state = S_IDLE;                      /* frame ended with no CRC */
        } else if (s_len < (MB_FRAME_MAX - 1)) {
            s_payload[s_len++] = c;
        } else {
            s_state = S_IDLE;                      /* overlong -> drop */
        }
        return false;

    case S_CRC1: {
        int v = hexval(c);
        if (v < 0) { s_state = S_IDLE; return false; }
        s_crc_hi = (uint8_t)v;
        s_state  = S_CRC2;
        return false;
    }

    case S_CRC2: {
        int v = hexval(c);
        s_state = S_IDLE;
        if (v < 0) return false;

        uint8_t want = (uint8_t)((s_crc_hi << 4) | (uint8_t)v);
        s_payload[s_len] = 0x00;

        if (MB_Crc8(s_payload, s_len) != want) {
            if (crc_err) *crc_err = true;          /* corrupted -> DROP IT */
            return false;
        }
        return parse_payload(s_payload, out);
    }

    default:
        s_state = S_IDLE;
        return false;
    }
}

uint16_t MB_ProtoBuildFeedback(char *buf, uint16_t cap,
                               int32_t l_steps, int32_t r_steps,
                               int32_t l_sps, int32_t r_sps,
                               uint16_t flags, uint16_t seq)
{
    char payload[MB_FRAME_MAX];
    int n = snprintf(payload, sizeof(payload), "F,%ld,%ld,%ld,%ld,%u,%u",
                     (long)l_steps, (long)r_steps,
                     (long)l_sps, (long)r_sps,
                     (unsigned)flags, (unsigned)seq);
    if (n <= 0 || (uint16_t)n >= sizeof(payload)) return 0;

    uint8_t crc = MB_Crc8(payload, (uint16_t)n);
    int m = snprintf(buf, cap, "$%s*%02X\n", payload, crc);
    return (m > 0 && (uint16_t)m < cap) ? (uint16_t)m : 0;
}

uint16_t MB_ProtoBuildSimple(char *buf, uint16_t cap, char letter)
{
    char payload[2];
    payload[0] = letter;
    payload[1] = 0x00;

    uint8_t crc = MB_Crc8(payload, 1);
    int m = snprintf(buf, cap, "$%c*%02X\n", letter, crc);
    return (m > 0 && (uint16_t)m < cap) ? (uint16_t)m : 0;
}
