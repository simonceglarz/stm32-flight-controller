/*
 * lsm6dsox.c — LSM6DSOX IMU driver.
 *
 * Two layers:
 *   1. Bus layer   (bus_read, bus_write): the ONLY code that knows whether we're
 *      on I2C or SPI. Two versions exist below; IMU_BUS_I2C picks one at compile time.
 *   2. Device layer (lsm6dsox_init, lsm6dsox_read_raw): sensor logic, written only
 *      in terms of bus_read/bus_write, so it is identical for both buses.
 *
 * Register addresses and values: core/lsm6dsox_regs.h
 * Datasheet reasoning:           docs/lsm6dsox-notes.md
 */

#include "lsm6dsox.h"        /* our own interface (function prototypes)               */
#include "lsm6dsox_regs.h"   /* register addresses + config values (in core/)         */
#include "imu_convert.h"     /* imu_read_cmd(): builds the SPI read byte (in core/)   */
#include "main.h"            /* HAL functions + pin names CubeMX generated (IMU_CS_*) */
#include "cmsis_os.h"        /* osDelay(): FreeRTOS-aware delay                       */

/* 1 = use I2C (4 wires, tonight). 0 = use SPI (final wiring).
   #if below removes the unused version before compiling, so only one exists. */
#define IMU_BUS_I2C 0

/* Longest we'll wait for any single bus transaction before giving up.
   A healthy transfer takes well under 1 ms; 10 ms means "something is wrong". */
#define IMU_TIMEOUT_MS 10u


/* =====================================================================
 *                      BUS LAYER — I2C VERSION
 * ===================================================================== */
#if IMU_BUS_I2C

#include "i2c.h"             /* hi2c1: the I2C1 "handle" CubeMX created in i2c.c.
                                A handle is a struct holding that peripheral's
                                settings and state; every HAL I2C call needs it. */

/*
 * The sensor's 7-bit I2C address is 0x6A (DO/SA0 pin low, Adafruit's default;
 * it would be 0x6B if that pin were pulled high).
 * On the wire, the address is followed by 1 bit meaning read(1)/write(0).
 * ST's HAL wants the address already shifted into the top 7 bits, so << 1.
 */
#define IMU_I2C_ADDR (0x6Au << 1)

/*
 * Read `len` bytes starting at register `reg` into `buf`.
 * `static` = private to this file; nothing outside can call it.
 */
static int bus_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    /*
     * HAL_I2C_Mem_Read performs a complete "register read" transaction:
     *   START, [address + write], [reg], repeated START, [address + read],
     *   then reads `len` bytes, then STOP.
     * Arguments, in order:
     *   &hi2c1               which I2C peripheral to use
     *   IMU_I2C_ADDR         which device on the bus
     *   reg                  which register to start reading from
     *   I2C_MEMADD_SIZE_8BIT register addresses on this sensor are 1 byte long
     *   buf, len             where to put the data, and how many bytes
     *   IMU_TIMEOUT_MS       give up after this many milliseconds
     * It returns HAL_OK on success (or HAL_ERROR / HAL_BUSY / HAL_TIMEOUT).
     * We convert that to our own convention: 0 = success, -1 = failure.
     */
    if (HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                         buf, len, IMU_TIMEOUT_MS) == HAL_OK) {
        return 0;
    }
    return -1;
}

/* Write one byte `val` into register `reg`. */
static int bus_write(uint8_t reg, uint8_t val)
{
    /* Same structure as above, but writing: START, [address + write], [reg], [val], STOP.
       HAL wants a pointer to the data, hence &val; length is 1 byte. */
    if (HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                          &val, 1, IMU_TIMEOUT_MS) == HAL_OK) {
        return 0;
    }
    return -1;
}


/* =====================================================================
 *                 BUS LAYER — SPI VERSION (untested yet)
 * ===================================================================== */
#else

#include "spi.h"             /* hspi2: the SPI2 handle CubeMX created in spi.c */

/*
 * Chip select (CS) is a normal output pin (PB12) that we drive by hand.
 * IMU_CS_GPIO_Port / IMU_CS_Pin come from main.h: CubeMX generated them from
 * the "IMU_CS" user label you gave PB12.
 * HAL_GPIO_WritePin(port, pin, level): RESET = drive low, SET = drive high.
 * CS is active-low: low means "IMU, listen to me".
 */
#define CS_LOW()  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET)

static int bus_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    /* First byte of an SPI read = register address with the top bit set
       (datasheet p.19: first bit is RW, 1 = read). e.g. 0x0F -> 0x8F. */
    uint8_t cmd = imu_read_cmd(reg);
    int ok;

    CS_LOW();                                            /* start the transaction    */

    /* Send the 1-byte command. HAL_SPI_Transmit(handle, data, length, timeout). */
    ok = (HAL_SPI_Transmit(&hspi2, &cmd, 1, IMU_TIMEOUT_MS) == HAL_OK);

    /* Receive `len` bytes. SPI only moves data while the master clocks it, so
       HAL_SPI_Receive sends dummy bytes to generate the clock while it
       captures what the IMU sends back on MISO. */
    ok = ok && (HAL_SPI_Receive(&hspi2, buf, len, IMU_TIMEOUT_MS) == HAL_OK);

    CS_HIGH();                                           /* end the transaction      */
    return ok ? 0 : -1;
}

static int bus_write(uint8_t reg, uint8_t val)
{
    /* Write = 2 bytes in one transaction: [address with top bit 0][value]. */
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), val };     /* & 0x7F clears the read bit */
    int ok;

    CS_LOW();
    ok = (HAL_SPI_Transmit(&hspi2, tx, 2, IMU_TIMEOUT_MS) == HAL_OK);
    CS_HIGH();
    return ok ? 0 : -1;
}

#endif /* IMU_BUS_I2C */


/* =====================================================================
 *                           DEVICE LAYER
 *          (identical for both buses — only calls bus_read/bus_write)
 * ===================================================================== */

/*
 * Write a register, then read it back and compare.
 * Catches wiring faults and wrong addresses that a plain write wouldn't reveal,
 * because a write gives no proof the value actually landed.
 */
static int write_verify(uint8_t reg, uint8_t val)
{
    uint8_t check = 0;
    if (bus_write(reg, val) != 0)      return -1;   /* the write itself failed   */
    if (bus_read(reg, &check, 1) != 0) return -1;   /* the read-back failed      */
    return (check == val) ? 0 : -1;                 /* value didn't stick        */
}

int lsm6dsox_init(void)
{
    uint8_t id = 0;

    /* The sensor needs 35 ms after power-up before it responds (datasheet Table 3).
       osDelay(40) sleeps this task for 40 ms of FreeRTOS ticks (1 tick = 1 ms),
       letting other tasks run meanwhile. */
    osDelay(40);

    /* ---- 1. Identity check ----
       WHO_AM_I is a read-only register fixed at 0x6C. Reading it first proves
       the wiring, the bus settings and the address all work, before we change anything. */
    if (bus_read(LSM6DSOX_WHO_AM_I, &id, 1) != 0) return -1;   /* nobody answered     */
    if (id != LSM6DSOX_WHO_AM_I_VALUE)            return -2;   /* wrong device/value  */

    /* ---- 2. Software reset ----
       Puts every register back to its default, so the result doesn't depend on
       what a previous run (or a crash) left behind. */
    if (bus_write(LSM6DSOX_CTRL3_C, LSM6DSOX_CTRL3_SW_RESET) != 0) return -3;

    /* The reset bit clears itself when the reset is done. Check up to 10 times,
       1 ms apart. c3 starts with the bit SET so a failed read counts as "not done yet". */
    for (int i = 0; i < 10; i++) {
        uint8_t c3 = LSM6DSOX_CTRL3_SW_RESET;
        osDelay(1);
        if (bus_read(LSM6DSOX_CTRL3_C, &c3, 1) == 0 &&
            (c3 & LSM6DSOX_CTRL3_SW_RESET) == 0) {
            break;                                   /* bit cleared: reset finished */
        }
        if (i == 9) return -4;                       /* still set after 10 ms       */
    }

    /* ---- 3. Configuration (each write is read back) ---- */

    /* BDU: the chip won't update an output value between our reads of its low
       and high bytes (prevents mixing halves of two different samples).
       IF_INC: register address auto-increments, so one burst reads all 12 bytes. */
    if (write_verify(LSM6DSOX_CTRL3_C, LSM6DSOX_CTRL3_BDU_IFINC) != 0) return -5;

#if !IMU_BUS_I2C
    /* SPI only: turn off the sensor's I2C block so SPI traffic can't be
       misread as I2C. Never do this while talking over I2C: it would cut
       our own connection mid-setup. */
    if (write_verify(LSM6DSOX_CTRL4_C, LSM6DSOX_CTRL4_I2C_DIS) != 0) return -6;
#endif

    /* Accelerometer: 1.66 kHz output rate, +/-4 g range. */
    if (write_verify(LSM6DSOX_CTRL1_XL, LSM6DSOX_CTRL1_XL_CFG) != 0) return -7;

    /* Gyroscope: 1.66 kHz output rate, +/-500 deg/s range. */
    if (write_verify(LSM6DSOX_CTRL2_G, LSM6DSOX_CTRL2_G_CFG) != 0)   return -8;

    /* Data-ready signal as a 75 us pulse per sample instead of a level that
       stays high until read (a missed read would otherwise stall the loop). */
    if (write_verify(LSM6DSOX_COUNTER_BDR_REG1, LSM6DSOX_DRDY_PULSED) != 0) return -9;

    /* Route the gyro's data-ready signal to the INT1 pin (used once wired, with SPI). */
    if (write_verify(LSM6DSOX_INT1_CTRL, LSM6DSOX_INT1_DRDY_G) != 0) return -10;

    return 0;                                        /* everything verified */
}

int lsm6dsox_read_raw(uint8_t raw[12])
{
    /* Start at OUTX_L_G (0x22) and read 12 bytes. Because IF_INC is on, the
       sensor steps through 0x22..0x2D by itself: gyro X,Y,Z then accel X,Y,Z.
       One transaction means all six values come from the same sample. */
    return bus_read(LSM6DSOX_OUTX_L_G, raw, LSM6DSOX_BURST_LEN);
}
