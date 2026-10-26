/* hil_proto.c
 * Building and reading HIL frames. Pure C: no UART, no STM32 code, so it can be
 * unit-tested on the PC and used unchanged in the firmware. */
#include "hil_proto.h"
#include <string.h>         /* memcpy, memeset */

/* The parser's possible states, in the order a frame arrives.
 * (An enum just gives names to the numbers 0, 1, 2, 3, 4.) */
enum {
    ST_SYNC0,   /* waiting for the first start byte  (A5) */
    ST_SYNC1,   /* got A5, waiting for the second    (5A) */
    ST_BODY,    /* reading type, len, seq, payload */
    ST_CRC0,    /* waiting for the CRC's low byte */
    ST_CRC1     /* waiting for the CRC's high byte */
};

/* How it works, roughly: for each byte, mix it into the top of the 16-bit
 * value, then shift the value left one bit at a time, 8 times. Each time a 1
 * falls off the top, XOR in the mixing constant. That scrambling is what makes
 * every single-bit change give a different result.
 * ------------------------------------------------------------------------- */
 uint16_t hil_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;                          /* standard starting value */

    for (size_t i = 0; i < len; i++) {              /* for every byte... */
        crc ^= (uint16_t)((uint16_t)data[i] << 8);   /* mix the byte into the top 8 bits */

        for (int b = 0; b < 8; b++) {                /* ...process its 8 bits */
            if (crc & 0x8000u) {                     /* top bit is 1: */
                crc = (uint16_t)((crc << 1) ^ 0x1021u);  /* shift left and mix in the constant */
            } else {                                 /* top bit is 0: */
                crc = (uint16_t)(crc << 1);          /* just shift left */
            }
        }
    }
    return crc;
}

/* ---------------------------------------------------------------------------
 * Float <-> 4 bytes
 * memcpy copies the float's 4 raw bytes exactly. We don't write
 * *(float *)ptr = v because the STM32 can fault on reading a float from an
 * address that isn't a multiple of 4, and C's rules forbid reading memory
 * through the "wrong" type. memcpy avoids both problems.
 * ------------------------------------------------------------------------- */
static void put_f32(uint8_t *dst, float v)
{
    memcpy(dst, &v, 4);          /* float -> 4 bytes */
}

static float get_f32(const uint8_t *src)
{
    float v;
    memcpy(&v, src, 4);          /* 4 bytes -> float */
    return v;
}

/* ---------------------------------------------------------------------------
 * frame(): wrap a payload in the envelope (start marker, header, CRC).
 * Shared by both encode functions so the layout is only written once.
 * `static` = only usable inside this file.
 * ------------------------------------------------------------------------- */

 static size_t frame(uint8_t type, uint16_t seq, const uint8_t *payload,
                    uint8_t len, uint8_t out[HIL_MAX_FRAME])
{
    out[0] = HIL_SYNC0;                          /* start marker, byte 1 */
    out[1] = HIL_SYNC1;                          /* start marker, byte 2 */
    out[2] = type;                               /* what kind of message */
    out[3] = len;                                /* how many payload bytes follow */
    out[4] = (uint8_t)(seq & 0xFFu);             /* message number, low byte  (e.g. 513 -> 0x01) */
    out[5] = (uint8_t)(seq >> 8);                /* message number, high byte (e.g. 513 -> 0x02) */
    memcpy(&out[6], payload, len);               /* the contents */

    /* Check number over everything from `type` (out[2]) to the end of the payload:
       4 header bytes + len payload bytes. The start marker isn't included; it's
       always the same, so checking it would add nothing. */
    uint16_t crc = hil_crc16(&out[2], 4u + len);
    out[6 + len] = (uint8_t)(crc & 0xFFu);       /* CRC low byte */
    out[7 + len] = (uint8_t)(crc >> 8);          /* CRC high byte */

    return 8u + len;                             /* total frame size: 2 + 4 + len + 2 */
}

/* Build a SENSOR frame: put the 3 floats into a 12-byte payload, then wrap it. */
size_t hil_encode_sensor(const hil_sensor_t *s, uint8_t out[HIL_MAX_FRAME])
{
    uint8_t p[HIL_SENSOR_LEN];
    put_f32(&p[0], s->angle_sp_deg);             /* payload bytes 0-3 */
    put_f32(&p[4], s->angle_deg);                /* payload bytes 4-7 */
    put_f32(&p[8], s->rate_dps);                 /* payload bytes 8-11 */
    return frame(HIL_TYPE_SENSOR, s->seq, p, HIL_SENSOR_LEN, out);
}

/* Build a CMD frame: 1 float + 1 status byte = 5-byte payload. */
size_t hil_encode_cmd(const hil_cmd_t *c, uint8_t out[HIL_MAX_FRAME])
{
    uint8_t p[HIL_CMD_LEN];
    put_f32(&p[0], c->cmd);                      /* payload bytes 0-3 */
    p[4] = c->status;                            /* payload byte 4 */
    return frame(HIL_TYPE_CMD, c->seq, p, HIL_CMD_LEN, out);
}

/* ---------------------------------------------------------------------------
 * Receiver
 * ------------------------------------------------------------------------- */

void hil_parser_init(hil_parser_t *p)
{
    memset(p, 0, sizeof *p);     /* set every field to 0 (including crc_errors) */
    p->state = ST_SYNC0;         /* start by looking for a start marker */
}

/* Called once per received byte. See the state table in the guide. */
uint8_t hil_parser_feed(hil_parser_t *p, uint8_t byte)
{
    switch (p->state) {

    case ST_SYNC0:                               /* looking for A5 */
        if (byte == HIL_SYNC0) {
            p->state = ST_SYNC1;                 /* found it: now expect 5A */
        }
        /* anything else: junk, ignore it and keep looking */
        break;

    case ST_SYNC1:                               /* got A5, expecting 5A */
        if (byte == HIL_SYNC1) {
            p->idx = 0;                          /* start saving body bytes from the beginning */
            p->need = 4;                         /* first we need the 4 header bytes */
            p->state = ST_BODY;
        } else if (byte != HIL_SYNC0) {
            p->state = ST_SYNC0;                 /* false alarm: go back to looking */
        }
        /* if the byte was A5 again: stay here, since this A5 may be the real start */
        break;

    case ST_BODY:                                /* saving type, len, seq, payload */
        p->body[p->idx++] = byte;                /* store the byte, then advance the count */

        if (p->idx == 4) {                       /* header just completed: */
            if (p->body[1] > HIL_MAX_PAYLOAD) {  /* body[1] is len; is it impossible? */
                p->state = ST_SYNC0;             /* then this wasn't a real frame: start over */
                break;
            }
            p->need = (uint8_t)(4u + p->body[1]); /* total body = header + payload */
        }

        if (p->idx == p->need) {                 /* got the whole body */
            p->state = ST_CRC0;                  /* next come the 2 check bytes */
        }
        break;

    case ST_CRC0:                                /* first check byte (low) */
        p->crc_lo = byte;                        /* remember it until the high byte arrives */
        p->state = ST_CRC1;
        break;

    case ST_CRC1: {                              /* second check byte (high): decide */
        p->state = ST_SYNC0;                     /* whatever happens, next we look for a new frame */

        /* Rebuild the 16-bit check number the sender attached */
        uint16_t rx = (uint16_t)(p->crc_lo | ((uint16_t)byte << 8));

        /* Compute our own over the bytes we actually received, and compare */
        if (rx == hil_crc16(p->body, p->need)) {
            return p->body[0];                   /* match: frame is good; report its type */
        }
        p->crc_errors++;                         /* mismatch: damaged, so count it and discard */
        break;
    }

    default:                                     /* should never happen: recover safely */
        p->state = ST_SYNC0;
        break;
    }

    return 0;                                    /* no complete frame on this byte */
}

/* Read the 2-byte message number back out of the saved header. */
static uint16_t body_seq(const hil_parser_t *p)
{
    return (uint16_t)(p->body[2] | ((uint16_t)p->body[3] << 8));  /* low | (high << 8) */
}

/* Pull a SENSOR message's values out of the frame the parser just finished.
 * body[0] = type, body[1] = len, body[2..3] = seq, body[4..] = payload. */
int hil_decode_sensor(const hil_parser_t *p, hil_sensor_t *s)
{
    if (p->body[0] != HIL_TYPE_SENSOR || p->body[1] != HIL_SENSOR_LEN) {
        return -1;                               /* wrong kind or size: refuse */
    }
    s->seq          = body_seq(p);
    s->angle_sp_deg = get_f32(&p->body[4]);      /* payload bytes 0-3 */
    s->angle_deg    = get_f32(&p->body[8]);      /* payload bytes 4-7 */
    s->rate_dps     = get_f32(&p->body[12]);     /* payload bytes 8-11 */
    return 0;
}

/* Same for a CMD message. */
int hil_decode_cmd(const hil_parser_t *p, hil_cmd_t *c)
{
    if (p->body[0] != HIL_TYPE_CMD || p->body[1] != HIL_CMD_LEN) {
        return -1;
    }
    c->seq    = body_seq(p);
    c->cmd    = get_f32(&p->body[4]);            /* payload bytes 0-3 */
    c->status = p->body[8];                      /* payload byte 4 */
    return 0;
}