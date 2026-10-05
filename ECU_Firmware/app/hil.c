/* HIL frame decoding. Frames are collected into a pending set and only become
   the live inputs when the CONTROL frame arrives, so the control step never
   mixes values from two simulator ticks. */
#include "hil.h"
#include "can_protocol.h"
#include "config.h"
#include <string.h>

static HilInputs live, pending;

// Cone frames of the scan being received, so a scan is only used when all of it arrived.
static Cone incoming[AUTO_MAX_CONES];
static unsigned incoming_mask;
static int incoming_count, incoming_scan = -1;

void hil_init(void)
{
    memset(&live, 0, sizeof live);
    memset(&pending, 0, sizeof pending);
    incoming_mask  = 0;
    incoming_count = 0;
    incoming_scan  = -1;
}

static void take_cone(const uint8_t *data)
{
    int index = data[5], count = data[6], scan = data[7];
    if (count > AUTO_MAX_CONES || index >= (count ? count : 1)) return;
    if (scan != incoming_scan) { // the first frame of a new scan
        incoming_scan = scan;
        incoming_mask = 0;
    }
    incoming_count = count;
    if (count == 0) return;
    incoming[index] = (Cone) { can_get_u16(data) * 0.01f, can_get_i16(data + 2) * 0.001f,
        data[4] == CONE_YELLOW ? CONE_YELLOW : CONE_BLUE };
    incoming_mask |= 1u << index;
}

// An empty scan is a single frame with count 0, which is told apart by the scan number.
static int scan_complete(int scan)
{
    if (scan != incoming_scan) return 0;
    return incoming_mask == (1u << incoming_count) - 1u;
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
    case HIL_MSG_CONE:
        take_cone(data);
        return 1;
    case HIL_MSG_CONTROL:
        pending.on       = (data[0] & HIL_FLAG_ON) != 0;
        pending.drive    = (data[0] & HIL_FLAG_DRIVE) != 0;
        pending.autonomy = (data[0] & HIL_FLAG_AUTONOMY) != 0;
        if (scan_complete(data[1])) {
            pending.scan.count = incoming_count;
            memcpy(pending.scan.cone, incoming, sizeof incoming);
            pending.cones_ms   = now;
            pending.cones_seen = 1;
            incoming_scan      = -1; // each scan is used once
        }
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

int hil_cones_fresh(uint32_t now)
{
    return live.cones_seen && now - live.cones_ms <= CONES_TIMEOUT_MS;
}
