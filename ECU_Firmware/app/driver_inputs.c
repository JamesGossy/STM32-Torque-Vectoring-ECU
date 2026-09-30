/* Pedal and steering sensor handling. The throttle request is only trusted when
   both throttle sensors agree, and pressing the brake cuts drive torque until
   the throttle is released, as the Formula Student rules ask. */
#include "driver_inputs.h"
#include "config.h"
#include "util.h"

static uint32_t disagree_ms;
static int brake_cut;

void driver_inputs_reset(void)
{
    disagree_ms = 0;
    brake_cut   = 0;
}

static int in_range(float volts)
{
    return volts >= SENSOR_MIN_V && volts <= SENSOR_MAX_V;
}

// Position 0..1 between the zero and full voltages, with a small deadband at each end.
static float travel(float volts, float zero_v, float full_v)
{
    float position = (volts - zero_v) / (full_v - zero_v);
    position       = (position - PEDAL_DEADBAND) / (1.0f - 2.0f * PEDAL_DEADBAND);
    return clampf(position, 0.0f, 1.0f);
}

void driver_inputs_update(const PedalVolts *volts, uint32_t dt_ms, DriverInputs *out)
{
    out->problems = 0;

    // 1. scale each sensor
    float apps1       = travel(volts->apps1_v, APPS1_ZERO_V, APPS1_FULL_V);
    float apps2       = travel(volts->apps2_v, APPS2_ZERO_V, APPS2_FULL_V);
    out->throttle     = 0.5f * (apps1 + apps2);
    out->brake        = travel(volts->bpps_v, BPPS_ZERO_V, BPPS_FULL_V);
    float steer       = (volts->steering_v - STEER_CENTRE_V) / (STEER_LEFT_V - STEER_CENTRE_V);
    out->steering_rad = clampf(steer, -1.0f, 1.0f) * STEER_LOCK_RAD;

    // 2. wiring faults
    if (!in_range(volts->apps1_v) || !in_range(volts->apps2_v) || !in_range(volts->bpps_v)
        || !in_range(volts->steering_v)) {
        out->problems |= PEDAL_SENSOR_FAULT;
    }

    // 3. throttle sensors must agree within 10 % for no more than 100 ms
    if (apps1 - apps2 > APPS_DISAGREE || apps2 - apps1 > APPS_DISAGREE) {
        disagree_ms += dt_ms;
    } else {
        disagree_ms = 0;
    }
    if (disagree_ms > APPS_DISAGREE_MS) out->problems |= PEDAL_DISAGREE;

    // 4. brake with throttle cuts torque until the throttle is nearly released
    int braking = out->brake > BRAKE_ON;
    if (braking && out->throttle > BRAKE_APPS_CUT) brake_cut = 1;
    if (out->throttle < BRAKE_APPS_RESET) brake_cut = 0;
    if (brake_cut) out->problems |= PEDAL_BRAKE_THROTTLE;

    // 5. brake pedal asks for regen, otherwise the throttle asks for drive
    if (braking) {
        out->request_nm = -out->brake * REGEN_REQUEST_MAX_NM;
    } else {
        out->request_nm = out->throttle * DRIVE_REQUEST_MAX_NM;
    }
    if (out->problems) out->request_nm = 0.0f;
}
