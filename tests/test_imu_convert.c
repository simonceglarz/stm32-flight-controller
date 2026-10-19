/*
 * test_imu_convert.c — Host tests for core/imu_convert.c.
 *
 *   gcc -Wall -Wextra -std=c11 -Icore core/imu_convert.c tests/test_imu_convert.c -o tests/build/test_imu_convert -lm
 *   ./tests/build/test_imu_convert
 */

#include <stdio.h>
#include <math.h>
#include "imu_convert.h"
#include "lsm6dsox_regs.h"

static int failures = 0;

static void check(const char *name, float got, float expected, float tol)
{
    if (fabsf(got - expected) > tol) {
        printf("FAIL %-34s got %f, expected %f\n", name, got, expected);
        failures++;
    } else {
        printf("pass %s\n", name);
    }
}

int main(void)
{
    /* 1. Read command sets the top bit (p.19): 0x0F -> 0x8F, 0x22 -> 0xA2. */
    check("read cmd WHO_AM_I", imu_read_cmd(LSM6DSOX_WHO_AM_I), 0x8F, 0);
    check("read cmd OUTX_L_G", imu_read_cmd(LSM6DSOX_OUTX_L_G), 0xA2, 0);

    /* 2. Byte order: low=0x34, high=0x12 -> 0x1234 = 4660 (catches swapped bytes). */
    check("little-endian assembly", imu_bytes_to_i16(0x34, 0x12), 4660, 0);

    /* 3. Sign: two's complement edge cases. */
    check("0xFFFF is -1",      imu_bytes_to_i16(0xFF, 0xFF), -1, 0);
    check("0x8000 is -32768",  imu_bytes_to_i16(0x00, 0x80), -32768, 0);
    check("0x7FFF is 32767",   imu_bytes_to_i16(0xFF, 0x7F), 32767, 0);

    /* 4. Full conversion with a known buffer.
          Gyro X = +1000 counts -> 1000 x 0.0175 = 17.5 dps
          Gyro Y = -1000 counts -> -17.5 dps
          Gyro Z = 0
          Accel Z = 8197 counts -> 8197 x 0.000122 = 1.000 g (board lying flat)
          1000 = 0x03E8, -1000 = 0xFC18, 8197 = 0x2005 */
    const uint8_t raw[12] = {
        0xE8, 0x03,   /* gyro  X  +1000 */
        0x18, 0xFC,   /* gyro  Y  -1000 */
        0x00, 0x00,   /* gyro  Z      0 */
        0x00, 0x00,   /* accel X      0 */
        0x00, 0x00,   /* accel Y      0 */
        0x05, 0x20,   /* accel Z  +8197 */
    };
    imu_sample_t s;
    imu_convert(raw, &s);
    check("gyro X scale",        s.gyro_dps[0], 17.5f,  1e-4f);
    check("gyro Y negative",     s.gyro_dps[1], -17.5f, 1e-4f);
    check("gyro Z zero",         s.gyro_dps[2], 0.0f,   1e-6f);
    check("accel X zero",        s.accel_g[0],  0.0f,   1e-6f);
    check("accel Z flat = 1 g",  s.accel_g[2],  1.0f,   1e-3f);

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}