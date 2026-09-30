/* The ECU application: reads the inputs, decides whether torque is allowed,
   runs derating and torque vectoring, and commands the motor controllers. */
#ifndef APP_H
#define APP_H

#include "derate.h"
#include "driver_inputs.h"
#include <stdint.h>

enum { ECU_STARTUP, ECU_STANDBY, ECU_DRIVE };

// Reasons torque is held at zero. SOFT ones return torque as soon as they clear.
enum {
    INHIBIT_STARTUP        = 1u << 0,
    INHIBIT_MOTOR_OFFLINE  = 1u << 1,
    INHIBIT_MOTOR_FAULT    = 1u << 2,
    INHIBIT_NO_POWER       = 1u << 3, // main input missing, so the pedal sensors are unpowered
    INHIBIT_PEDAL_SENSOR   = 1u << 4,
    INHIBIT_PEDAL_DISAGREE = 1u << 5,
    INHIBIT_BRAKE_THROTTLE = 1u << 6,
    INHIBIT_HIL_LOST       = 1u << 7,
    INHIBIT_IMU            = 1u << 8,
    INHIBIT_CAN            = 1u << 9,
};
#define INHIBIT_SOFT (INHIBIT_PEDAL_DISAGREE | INHIBIT_BRAKE_THROTTLE)

typedef struct {
    uint8_t state;    // ECU_*
    int hil;          // 1 while the simulator supplies the inputs
    uint16_t inhibit; // INHIBIT_* bits
    uint8_t online;   // bit per motor controller, FL first
    float speed_ms;
    float steering_rad;
    float request_nm;
    float yaw_rate;
    float target_yaw_rate;
    float torque_nm[4]; // what was sent this period
    Derate derate;
    float vin_v;
    float ecu_temp_c;
    float gyro_bias;
    int imu_ok;
    DriverInputs pedals;
} EcuStatus;

void app_init(void);
void app_poll(void); // as often as the main loop spins
void app_tick(void); // every millisecond

const EcuStatus *app_status(void);
void app_disarm(void);

#endif
