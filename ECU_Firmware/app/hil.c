/* HIL frame decoding. Frames are collected into a pending set and only become
   the live inputs when the CONTROL frame arrives, so the control step never
   mixes values from two simulator ticks. */
#include "hil.h"
#include "can_protocol.h"
#include "config.h"
#include <string.h>

static HilInputs live, pending;

void hil_init(void)
{
    memset(&live, 0, sizeof live);
    memset(&pending, 0, sizeof pending);
}

int hil_handle_frame(uint32_t id, const uint8_t *data, uint8_t len, uint32_t now)
{
    if (CAN_NODE(id) != ECU_NODE || len < 8) return 0;

    switch (CAN_MSG(id)) {
    case HIL_MSG_DRIVER:
        pending.steering_rad = can_get_f32(data);
        pending.request_nm   = can_get_f32(data + 4);
        return 1;
    case HIL_MSG_WHEELS_F:
        pending.wheel_speed[0] = can_get_f32(data);
        pending.wheel_speed[1] = can_get_f32(data + 4);
        return 1;
    case HIL_MSG_WHEELS_R:
        pending.wheel_speed[2] = can_get_f32(data);
        pending.wheel_speed[3] = can_get_f32(data + 4);
        return 1;
    case HIL_MSG_CONTROL:
        pending.on         = (data[0] & HIL_FLAG_ON) != 0;
        pending.drive      = (data[0] & HIL_FLAG_DRIVE) != 0;
        pending.yaw_rate   = can_get_f32(data + 4);
        pending.updated_ms = now;
        pending.updated    = 1;
        live               = pending;
        return 1;
    }
    return 0;
}

const HilInputs *hil_inputs(void)
{
    return &live;
}

int hil_fresh(uint32_t now)
{
    return live.updated && now - live.updated_ms <= HIL_TIMEOUT_MS;
}
