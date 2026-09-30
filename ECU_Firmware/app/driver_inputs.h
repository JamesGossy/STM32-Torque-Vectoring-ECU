/* Pedals and steering: turns sensor voltages into throttle, brake, steering
   angle and a torque request, with the Formula Student plausibility checks. */
#ifndef DRIVER_INPUTS_H
#define DRIVER_INPUTS_H

#include <stdint.h>

typedef struct {
    float apps1_v, apps2_v, bpps_v, steering_v; // at the connector, before the board divider
} PedalVolts;

enum {
    PEDAL_SENSOR_FAULT   = 1u << 0, // a wire is open or shorted
    PEDAL_DISAGREE       = 1u << 1, // the two throttle sensors disagree for too long
    PEDAL_BRAKE_THROTTLE = 1u << 2, // throttle pressed with the brake on
};

typedef struct {
    float throttle;     // 0..1
    float brake;        // 0..1
    float steering_rad; // positive left
    float request_nm;   // total torque request, negative is regen
    uint8_t problems;   // PEDAL_* bits, any of them means zero torque
} DriverInputs;

void driver_inputs_reset(void);
void driver_inputs_update(const PedalVolts *volts, uint32_t dt_ms, DriverInputs *out);

#endif
