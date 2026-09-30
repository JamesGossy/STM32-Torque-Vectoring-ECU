/* Derating from temperature, motor speed and bus voltage. Heat limits both
   drive and regen. Speed and a sagging bus limit drive only. A rising bus limits
   regen only, because regen is what pushes it up. Regen also fades out as the car
   stops, because at standstill it would push the car backwards. */
#include "derate.h"
#include "config.h"
#include "util.h"
#include <math.h>

float derate_ramp(float value, float start, float end)
{
    if (!isfinite(value)) return 0.0f; // a broken reading must not give full torque
    return clampf(1.0f - (value - start) / (end - start), 0.0f, 1.0f);
}

// Lower *factor to the new value and remember why.
static void limit(float *factor, float value, uint8_t reason, uint8_t *reasons)
{
    if (value < 1.0f) *reasons |= reason;
    if (value < *factor) *factor = value;
}

Derate derate_compute(const DerateInputs *in)
{
    float heat = 1.0f, speed = 1.0f, low_bus = 1.0f, high_bus = 1.0f;
    uint8_t reasons = 0;

    limit(&heat, derate_ramp(in->ecu_temp_c, DERATE_ECU_START_C, DERATE_ECU_END_C), DERATE_ECU_TEMP,
        &reasons);

    for (int wheel = 0; wheel < 4; wheel++) {
        if (!in->used[wheel]) continue;
        float fet  = derate_ramp(in->fet_temp_c[wheel], DERATE_FET_START_C, DERATE_FET_END_C);
        float spin = derate_ramp(fabsf(in->motor_speed_rads[wheel]),
            DERATE_SPEED_START * MOTOR_MAX_SPEED_RADS, MOTOR_MAX_SPEED_RADS);
        float sag  = derate_ramp(in->bus_v[wheel], DERATE_LOW_V_START, DERATE_LOW_V_END);
        float rise = derate_ramp(in->bus_v[wheel], DERATE_HIGH_V_START, DERATE_HIGH_V_END);
        limit(&heat, fet, DERATE_FET_TEMP, &reasons);
        limit(&speed, spin, DERATE_SPEED, &reasons);
        limit(&low_bus, sag, DERATE_LOW_BUS, &reasons);
        limit(&high_bus, rise, DERATE_HIGH_BUS, &reasons);
    }

    float stopping = 1.0f, fade = in->car_speed_ms / REGEN_FULL_SPEED_MS;
    limit(&stopping, isfinite(fade) ? clampf(fade, 0.0f, 1.0f) : 0.0f, DERATE_SLOW, &reasons);

    Derate out;
    out.drive   = fminf(heat, fminf(speed, low_bus));
    out.regen   = fminf(heat, fminf(high_bus, stopping));
    out.reasons = reasons;
    return out;
}
