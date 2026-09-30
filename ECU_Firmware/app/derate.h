/* Derating: lowers the torque limits as the motor controllers, the ECU or the
   bus supply get close to their limits. One factor covers the whole car, so
   derating never adds a yaw moment of its own. */
#ifndef DERATE_H
#define DERATE_H

#include <stdint.h>

typedef struct {
    int used[4];               // only online controllers count
    float fet_temp_c[4];       // controller power stage temperatures
    float motor_speed_rads[4]; // as the controllers measure it
    float bus_v[4];            // as the controllers measure it
    float ecu_temp_c;
    float car_speed_ms;
} DerateInputs;

enum {
    DERATE_FET_TEMP = 1u << 0,
    DERATE_ECU_TEMP = 1u << 1,
    DERATE_SPEED    = 1u << 2,
    DERATE_LOW_BUS  = 1u << 3,
    DERATE_HIGH_BUS = 1u << 4,
    DERATE_SLOW     = 1u << 5, // regen fading out near standstill
};

typedef struct {
    float drive;     // 0..1 of the full drive torque
    float regen;     // 0..1 of the full regen torque
    uint8_t reasons; // DERATE_* bits that are active
} Derate;

// 1 up to start, 0 from end, a straight line between. Works for falling limits too.
float derate_ramp(float value, float start, float end);

Derate derate_compute(const DerateInputs *in);

#endif
