/* hil_proto.h
 * Message format between the PC simulator and the STM32 in HIL mode.
 *
 * Every message ("frame") looks like this:
 *
 *   A5 5A | type | len | seq_lo seq_hi | payload (len bytes) | crc_lo crc_hi
 *
 *   A5 5A    start marker: lets the receiver find the beginning of a message
 *   type     what kind of message (SENSOR or CMD, below)
 *   len      how many payload bytes follow
 *   seq      message number, low byte first; the board echoes it in its reply
 *   payload  the actual values (floats sent as their raw 4 bytes)
 *   crc      check number over type..payload, to detect damaged messages
 *
 * Multi-byte values are sent low byte first ("little-endian"). Both the STM32
 * and a normal PC store numbers this way, so floats can be copied directly.
 */

#ifndef HIL_PROTO_H          /* "include guard": if this header was already */
#define HIL_PROTO_H          /* included, skip it, so nothing is defined twice */

#include <stddef.h>          /* size_t: the standard type for byte counts */
#include <stdint.h>          /* uint8_t (1 byte, 0..255), uint16_t (2 bytes), uint32_t */

/* ---- Fixed values both sides must agree on ---- */
#define HIL_SYNC0        0xA5u   /* first start-marker byte  (the u = unsigned) */
#define HIL_SYNC1        0x5Au   /* second start-marker byte */

#define HIL_TYPE_SENSOR  0x01u   /* PC -> board: "here are your sensor readings" */
#define HIL_TYPE_CMD     0x02u   /* board -> PC: "here is my motor command" */

#define HIL_MAX_PAYLOAD  16u     /* biggest payload we'll ever accept, in bytes */
/* Biggest possible whole frame: 2 start + 4 header (type,len,seq x2) + payload + 2 CRC */
#define HIL_MAX_FRAME    (2u + 4u + HIL_MAX_PAYLOAD + 2u)

#define HIL_SENSOR_LEN   12u     /* SENSOR payload: 3 floats x 4 bytes */
#define HIL_CMD_LEN      5u      /* CMD payload: 1 float (4 bytes) + 1 status byte */

#define HIL_STATUS_OK    0u      /* board is running normally */
#define HIL_STATUS_FAULT 1u      /* board has detected a fault (used in step 6) */

/* SENSOR message: what the PC tells the board each step */
typedef struct {
    uint16_t seq;            /* message number, 0..65535 (wraps around) */
    float angle_sp_deg;      /* angle the controller should hold, degrees */
    float angle_deg;         /* "measured" angle from the simulated drone, degrees */
    float rate_dps;          /* "measured" roll rate, degrees per second */
} hil_sensor_t;

/* CMD message: what the board answers */
typedef struct {
    uint16_t seq;            /* copy of the SENSOR message number this answers */
    float cmd;               /* motor command, -1..1 */
    uint8_t status;          /* HIL_STATUS_OK or HIL_STATUS_FAULT */
} hil_cmd_t;

/* ---- The receiver's memory between bytes ----
 * Because bytes arrive one at a time, the parser must remember where it is
 * in the current message. Everything it needs to remember lives here. */
typedef struct {
    uint8_t state;                       /* which part of the frame we're reading (see .c) */
    uint8_t body[4u + HIL_MAX_PAYLOAD];  /* saved bytes: type, len, seq_lo, seq_hi, payload */
    uint8_t idx;                         /* how many body bytes saved so far */
    uint8_t need;                        /* how many body bytes this frame has in total */
    uint8_t crc_lo;                      /* first CRC byte, kept until the second arrives */
    uint32_t crc_errors;                 /* count of damaged frames thrown away (for diagnostics) */
} hil_parser_t;

/* ---- Functions ---- */

/* Compute the 16-bit check number over `len` bytes starting at `data`. */
uint16_t hil_crc16(const uint8_t *data, size_t len);

/* Turn a SENSOR / CMD struct into frame bytes in `out`.
 * Returns how many bytes were written (the frame length). */
size_t hil_encode_sensor(const hil_sensor_t *s, uint8_t out[HIL_MAX_FRAME]);
size_t hil_encode_cmd(const hil_cmd_t *c, uint8_t out[HIL_MAX_FRAME]);

/* Reset the receiver to "waiting for a start marker". Call once before use. */
void hil_parser_init(hil_parser_t *p);

/* Give the receiver one byte that just arrived.
 * Returns the message type (1 or 2) at the moment a complete, undamaged
 * frame finishes; returns 0 the rest of the time. */
uint8_t hil_parser_feed(hil_parser_t *p, uint8_t byte);

/* After hil_parser_feed returned a type, pull the values out of the frame.
 * Return 0 on success, -1 if the frame isn't the expected kind/size. */
int hil_decode_sensor(const hil_parser_t *p, hil_sensor_t *s);
int hil_decode_cmd(const hil_parser_t *p, hil_cmd_t *c);

#endif /* HIL_PROTO_H */