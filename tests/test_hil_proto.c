/* test_hil_proto.c
 * Host tests for the HIL message format. Run with `make test`. */
#include <stdio.h>
#include <string.h>
#include "hil_proto.h"

static int fails = 0;

/* Print PASS/FAIL for one check and count failures. */
static void check(const char *name, int ok)
{
    printf("%s: %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) fails++;
}

/* Feed every byte of a buffer into the parser, as if it arrived over the cable.
 * Returns the type of the last complete frame seen, or 0 if none completed. */
static uint8_t feed_all(hil_parser_t *p, const uint8_t *b, size_t n)
{
    uint8_t got = 0;
    for (size_t i = 0; i < n; i++) {
        uint8_t t = hil_parser_feed(p, b[i]);
        if (t) got = t;
    }
    return got;
}

int main(void)
{
    uint8_t buf[HIL_MAX_FRAME];      /* room for one encoded frame */
    hil_parser_t p;                  /* the receiver */
    size_t n;                        /* frame length returned by the encoders */

    /* 1. The CRC recipe matches the published standard.
     *    If this fails, every other CRC check is meaningless. */
    check("crc check value 0x29B1",
          hil_crc16((const uint8_t *)"123456789", 9) == 0x29B1);

    /* 2. SENSOR round trip: values -> bytes -> parser -> values must be identical.
     *    Proves the encoder and decoder agree on where every byte goes. */
    hil_sensor_t s = { .seq = 513, .angle_sp_deg = 10.0f,
                       .angle_deg = -3.25f, .rate_dps = 42.5f };
    hil_sensor_t s2;
    n = hil_encode_sensor(&s, buf);
    hil_parser_init(&p);
    check("sensor frame length 20", n == 20);                     /* 2+4+12+2 */
    check("sensor frame parsed", feed_all(&p, buf, n) == HIL_TYPE_SENSOR);
    check("sensor decode ok", hil_decode_sensor(&p, &s2) == 0);
    check("sensor values exact", s2.seq == 513 && s2.angle_sp_deg == 10.0f &&
                                 s2.angle_deg == -3.25f && s2.rate_dps == 42.5f);

    /* 3. CMD round trip, using the largest message number (65535) to check
     *    both sequence bytes are handled. */
    hil_cmd_t c = { .seq = 65535, .cmd = -0.55f, .status = HIL_STATUS_FAULT };
    hil_cmd_t c2;
    n = hil_encode_cmd(&c, buf);
    hil_parser_init(&p);
    check("cmd frame parsed", feed_all(&p, buf, n) == HIL_TYPE_CMD);
    check("cmd values exact", hil_decode_cmd(&p, &c2) == 0 && c2.seq == 65535 &&
                              c2.cmd == -0.55f && c2.status == HIL_STATUS_FAULT);

    /* 4. Damage: flip one bit in the middle of the payload.
     *    The CRC must catch it, and the frame must be thrown away and counted. */
    n = hil_encode_sensor(&s, buf);
    buf[9] ^= 0x01;                                 /* XOR with 1 flips the lowest bit */
    hil_parser_init(&p);
    check("corrupted frame rejected", feed_all(&p, buf, n) == 0);
    check("crc error counted", p.crc_errors == 1);

    /* 5. Junk before a real frame, including fake start bytes (A5).
     *    The parser must skip the junk and still find the real frame. */
    uint8_t stream[64];
    const uint8_t junk[] = { 0x00, 0xA5, 0x13, 0xFF, 0xA5 };
    memcpy(stream, junk, sizeof junk);                     /* junk first... */
    n = hil_encode_sensor(&s, stream + sizeof junk);       /* ...then a real frame */
    hil_parser_init(&p);
    check("resyncs after garbage", feed_all(&p, stream, sizeof junk + n) == HIL_TYPE_SENSOR);

    /* 6. Two frames back to back: after finishing one, the parser must be
     *    ready for the next immediately. */
    size_t n1 = hil_encode_sensor(&s, stream);             /* frame 1 at the start */
    size_t n2 = hil_encode_cmd(&c, stream + n1);           /* frame 2 right after it */
    hil_parser_init(&p);
    check("first of two frames", feed_all(&p, stream, n1) == HIL_TYPE_SENSOR);
    check("second of two frames", feed_all(&p, stream + n1, n2) == HIL_TYPE_CMD);

    printf(fails ? "\nFAILURES: %d\n" : "\nALL PASSED (%d failures)\n", fails);
    return fails ? 1 : 0;
}