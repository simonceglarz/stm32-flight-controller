/*
 * imu_convert.c — Implementation of imu_convert.h.
 */

#include "imu_convert.h"
#include "lsm6dsox_regs.h"

uint8_t imu_read_cmd(uint8_t reg_addr)
{
    /* The sensor reads the first bit of a transaction as RW (1 = read),
       followed by the 7-bit address. Setting the top bit marks it as a read:
       e.g. WHO_AM_I 0x0F -> 0x8F. */
    return (uint8_t)(reg_addr | LSM6DSOX_SPI_READ);
}

int16_t imu_bytes_to_i16(uint8_t low, uint8_t high)
{
    /* The sensor stores each value low byte first ("little-endian").
       Shift the high byte up 8 bits and OR in the low byte to rebuild the
       16-bit pattern. Casting to int16_t reinterprets it as two's complement,
       so 0xFFFF becomes -1 and 0x8000 becomes -32768.
       (C leaves this narrowing cast implementation-defined; GCC, including
       the ARM compiler, defines it as wrap-around, which is what we need.) */
    uint16_t bits = (uint16_t)(((uint16_t)high << 8) | low);
    return (int16_t)bits;
}

void imu_convert(const uint8_t raw[12], imu_sample_t *out)
{
    /* Burst layout from OUTX_L_G (0x22): gyro X,Y,Z at bytes 0-5,
       accel X,Y,Z at bytes 6-11; each axis is [low, high]. */
    for (int axis = 0; axis < 3; axis++) {
        int16_t g = imu_bytes_to_i16(raw[2 * axis],     raw[2 * axis + 1]);
        int16_t a = imu_bytes_to_i16(raw[6 + 2 * axis], raw[6 + 2 * axis + 1]);

        /* Counts x (units per count) = physical value. Scale factors are
           float constants (f suffix) so no slow double math on the F4. */
        out->gyro_dps[axis] = (float)g * LSM6DSOX_GYRO_DPS_PER_LSB;
        out->accel_g[axis]  = (float)a * LSM6DSOX_ACC_G_PER_LSB;
    }
}