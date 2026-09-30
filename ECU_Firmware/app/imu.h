/* LSM6DSV16X accelerometer and gyroscope on SPI. */
#ifndef IMU_H
#define IMU_H

typedef struct {
    float gyro[3];  // rad/s about x, y, z
    float accel[3]; // m/s^2 along x, y, z
} ImuSample;

int imu_init(void);           // returns 1 if the sensor answered and took its settings
int imu_read(ImuSample *out); // returns 0 if the sensor stopped answering

#endif
