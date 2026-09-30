/* LSM6DSV16X driver. Register values come from ST's lsm6dsv16x_reg.h. Both
   sensors run at 480 Hz in high performance mode and the ECU reads the newest
   sample each control period. */
#include "imu.h"
#include "hal.h"
#include <stdint.h>

#define REG_WHO_AM_I 0x0F
#define REG_CTRL1    0x10 // accel rate and mode
#define REG_CTRL2    0x11 // gyro rate and mode
#define REG_CTRL3    0x12
#define REG_CTRL6    0x15 // gyro range
#define REG_CTRL8    0x17 // accel range
#define REG_OUTX_L_G 0x22 // gyro x, y, z then accel x, y, z, 12 bytes

#define WHO_AM_I_VALUE 0x70
#define READ_BIT       0x80
#define ODR_480_HZ     0x08
#define CTRL3_RESET    0x01
#define CTRL3_BDU_INC  0x44 // no half-updated samples, auto-increment addresses
#define GYRO_500_DPS   0x02
#define ACCEL_8_G      0x02

#define GYRO_RAD_PER_LSB  (0.0175f * 0.017453293f) // 17.5 mdps per bit at 500 dps
#define ACCEL_MS2_PER_LSB (0.000244f * 9.80665f)   // 0.244 mg per bit at 8 g

static void write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg, value }, rx[2];
    hal_imu_transfer(tx, rx, 2);
}

static void read_regs(uint8_t reg, uint8_t *out, int count)
{
    uint8_t tx[13] = { (uint8_t)(reg | READ_BIT) }, rx[13];
    hal_imu_transfer(tx, rx, (size_t)count + 1);
    for (int i = 0; i < count; i++)
        out[i] = rx[i + 1];
}

static uint8_t read_reg(uint8_t reg)
{
    uint8_t value;
    read_regs(reg, &value, 1);
    return value;
}

int imu_init(void)
{
    if (read_reg(REG_WHO_AM_I) != WHO_AM_I_VALUE) return 0;

    write_reg(REG_CTRL3, CTRL3_RESET);
    for (int tries = 0; tries < 1000 && (read_reg(REG_CTRL3) & CTRL3_RESET); tries++) { }
    write_reg(REG_CTRL3, CTRL3_BDU_INC);
    write_reg(REG_CTRL6, GYRO_500_DPS);
    write_reg(REG_CTRL8, ACCEL_8_G);
    write_reg(REG_CTRL1, ODR_480_HZ);
    write_reg(REG_CTRL2, ODR_480_HZ);
    return read_reg(REG_CTRL2) == ODR_480_HZ;
}

int imu_read(ImuSample *out)
{
    uint8_t raw[12];
    read_regs(REG_OUTX_L_G, raw, 12);
    for (int axis = 0; axis < 3; axis++) {
        int16_t gyro     = (int16_t)(raw[2 * axis] | (raw[2 * axis + 1] << 8));
        int16_t accel    = (int16_t)(raw[6 + 2 * axis] | (raw[6 + 2 * axis + 1] << 8));
        out->gyro[axis]  = gyro * GYRO_RAD_PER_LSB;
        out->accel[axis] = accel * ACCEL_MS2_PER_LSB;
    }
    // a sensor that reset itself still answers WHO_AM_I, but is powered down again
    return read_reg(REG_WHO_AM_I) == WHO_AM_I_VALUE && read_reg(REG_CTRL2) == ODR_480_HZ;
}
