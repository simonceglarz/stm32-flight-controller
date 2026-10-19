/*
 * lsm6dsox_regs.h — LSM6DSOX register addresses and the configuration values
 * this project writes. Source: ST datasheet DS12814 Rev 4 (page numbers below).
 * Derivations and reasoning: docs/lsm6dsox-notes.md
 */

#ifndef LSM6DSOX_REGS_H
#define LSM6DSOX_REGS_H

/* ---- SPI protocol (p.19) ----
   First bit of every transaction is RW: 1 = read. OR this into the address. */
#define LSM6DSOX_SPI_READ        0x80u

/* ---- Register addresses (register map, p.43-45) ---- */
#define LSM6DSOX_COUNTER_BDR_REG1 0x0Bu  /* data-ready pulsed/latched (p.51)       */
#define LSM6DSOX_INT1_CTRL        0x0Du  /* what drives the INT1 pin (p.52)        */
#define LSM6DSOX_WHO_AM_I         0x0Fu  /* fixed identity value (p.53)            */
#define LSM6DSOX_CTRL1_XL         0x10u  /* accel rate + range (p.54)              */
#define LSM6DSOX_CTRL2_G          0x11u  /* gyro rate + range (p.55)               */
#define LSM6DSOX_CTRL3_C          0x12u  /* reset, BDU, auto-increment (p.56)      */
#define LSM6DSOX_CTRL4_C          0x13u  /* I2C disable (p.57)                     */
#define LSM6DSOX_STATUS_REG       0x1Eu  /* new-data flags (p.67)                  */
#define LSM6DSOX_OUTX_L_G         0x22u  /* first of 12 output bytes (p.68)        */

/* ---- Expected / configuration values (bit math in docs/lsm6dsox-notes.md §5) ---- */
#define LSM6DSOX_WHO_AM_I_VALUE   0x6Cu  /* p.53: "fixed at 6Ch"                    */
#define LSM6DSOX_CTRL3_SW_RESET   0x01u  /* bit 0: software reset (self-clearing)   */
#define LSM6DSOX_CTRL3_BDU_IFINC  0x44u  /* bit 6 BDU + bit 2 IF_INC                */
#define LSM6DSOX_CTRL4_I2C_DIS    0x04u  /* bit 2: I2C_disable                      */
#define LSM6DSOX_CTRL1_XL_CFG     0x88u  /* ODR 1.66 kHz, +/-4 g                    */
#define LSM6DSOX_CTRL2_G_CFG      0x84u  /* ODR 1.66 kHz, +/-500 dps                */
#define LSM6DSOX_DRDY_PULSED      0x80u  /* bit 7: pulsed data-ready (decision D1)  */
#define LSM6DSOX_INT1_DRDY_G      0x02u  /* bit 1: gyro data-ready on INT1          */

/* ---- Burst read layout ----
   One read starting at OUTX_L_G returns 12 bytes:
   gyro X, Y, Z, then accel X, Y, Z — each as low byte, then high byte. */
#define LSM6DSOX_BURST_LEN        12u

/* ---- Scale factors for the chosen ranges (Table 2, p.10) ---- */
#define LSM6DSOX_GYRO_DPS_PER_LSB (17.50e-3f)   /* +/-500 dps: 17.50 mdps per count */
#define LSM6DSOX_ACC_G_PER_LSB    (0.122e-3f)   /* +/-4 g:     0.122 mg per count   */

#endif /* LSM6DSOX_REGS_H */