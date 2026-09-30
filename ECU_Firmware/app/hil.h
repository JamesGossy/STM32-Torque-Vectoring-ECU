/* Inputs sent by the HIL simulator over CAN. They stand in for the signals a
   bench cannot produce: steering, the driver's torque request, yaw rate and
   road wheel speeds. */
#ifndef HIL_H
#define HIL_H

#include <stdint.h>

typedef struct {
    int on;    // the simulator asked for HIL mode
    int drive; // the simulated driver wants torque
    float steering_rad;
    float request_nm;
    float yaw_rate;
    float wheel_speed[4]; // road wheel rad/s
    uint32_t updated_ms;
    int updated; // 1 once a complete set has arrived
} HilInputs;

void hil_init(void);

// Returns 1 if the frame was a HIL frame.
int hil_handle_frame(uint32_t id, const uint8_t *data, uint8_t len, uint32_t now);

const HilInputs *hil_inputs(void);
int hil_fresh(uint32_t now);

#endif
