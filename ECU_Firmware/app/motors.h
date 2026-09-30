/* The four motor controllers as the ECU sees them over CAN: their telemetry,
   whether they are healthy, and the commands that put them in torque mode and
   set their current. */
#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

typedef struct {
    uint32_t heard_ms;   // last heartbeat
    int heard;           // 1 once any heartbeat has arrived
    uint8_t state, mode; // MC_STATE_*, MC_MODE_*
    uint16_t faults;     // the controller's own fault bits
    float iq_a;          // measured q-axis current
    float speed_rads;    // motor shaft, as the controller reports it
    float bus_v;
    float fet_temp_c;
    uint32_t asked_ms; // last state request
} Motor;

void motors_init(void);

// Returns 1 if the frame came from a motor controller.
int motors_handle_frame(uint32_t id, const uint8_t *data, uint8_t len, uint32_t now);

const Motor *motor_get(int wheel);
int motor_online(int wheel, uint32_t now);
int motor_healthy(int wheel, uint32_t now); // online, no faults, not booting or calibrating
float motor_wheel_speed(int wheel);         // road wheel rad/s, forward positive

// Sends one control period of commands. With enable 0 the controllers go idle.
void motors_command(const float torque_nm[4], int enable, uint32_t now);

// 1 if a controller was asked for torque mode but has not got there in time.
int motors_refusing(uint32_t now);

void motors_clear_faults(void);
void motors_estop(void);

#endif
