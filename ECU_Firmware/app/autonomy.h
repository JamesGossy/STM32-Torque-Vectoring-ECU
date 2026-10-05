/* Driverless control on the ECU. From one scan of cones and the car's speed it
   works out where to steer and how much torque to ask for. Runs every 10 ms. */
#ifndef AUTONOMY_H
#define AUTONOMY_H

#include "local_map.h"

typedef struct {
    float steering;        // in steering units, wheel angle / STEERING_TO_WHEEL, positive left
    float request_nm;      // total torque for all four motors, negative is regen
    float target_speed_ms; // for the status line
} AutoCommand;

void autonomy_reset(void);
AutoCommand autonomy_step(const ConeScan *scan, float speed_ms);

#endif
