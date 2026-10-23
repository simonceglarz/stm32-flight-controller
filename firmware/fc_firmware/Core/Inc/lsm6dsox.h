/*
 * lsm6dsox.h — Public interface of the LSM6DSOX IMU driver.
 * Other files include this to use the driver; they never see the bus details.
 */

#ifndef LSM6DSOX_H              /* include guard: skip if already included */
#define LSM6DSOX_H

#include <stdint.h>             /* uint8_t (unsigned 8-bit integer = one byte) */

/*
 * Bring the sensor up: wait for power-up, check its identity, reset it,
 * write the configuration and read every register back to confirm it stuck.
 * Must be called from a FreeRTOS task (it uses osDelay).
 * Returns 0 on success, or a negative code saying which step failed:
 *   -1 no response on the bus      -2 wrong WHO_AM_I value
 *   -3 reset command failed        -4 reset never finished
 *   -5..-10 a configuration register didn't hold its value
 */
int lsm6dsox_init(void);

/*
 * Read the 12 output bytes in one transaction:
 *   raw[0..5]  = gyro  X, Y, Z   (each: low byte, high byte)
 *   raw[6..11] = accel X, Y, Z   (each: low byte, high byte)
 * Returns 0 on success, -1 on a bus error. Convert with imu_convert().
 */
int lsm6dsox_read_raw(uint8_t raw[12]);

#endif /* LSM6DSOX_H */
