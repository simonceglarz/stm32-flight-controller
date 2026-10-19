/*
 * imu_convert.h — Turn raw LSM6DSOX bytes into physical units.
 * No hardware access: the SPI driver reads the bytes, this code interprets them.
 * Keeping the two separate lets this logic be unit-tested on a PC.
 */

#ifndef IMU_CONVERT_H
#define IMU_CONVERT_H

#include <stdint.h>             /* fixed-width types: uint8_t, int16_t */

typedef struct {
    float gyro_dps[3];          /* angular rate X, Y, Z in degrees per second */
    float accel_g[3];           /* acceleration X, Y, Z in g (1 g = gravity)  */
} imu_sample_t;

/* Build the first byte of an SPI read: register address with the read bit set. */
uint8_t imu_read_cmd(uint8_t reg_addr);

/* Combine a low and high byte (sensor order: low first) into a signed 16-bit value. */
int16_t imu_bytes_to_i16(uint8_t low, uint8_t high);

/* Convert the 12-byte burst from OUTX_L_G into gyro (dps) and accel (g). */
void imu_convert(const uint8_t raw[12], imu_sample_t *out);

#endif /* IMU_CONVERT_H */