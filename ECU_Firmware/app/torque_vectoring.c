/* Proportional yaw-rate torque vectoring, the same law the Formula Student sim
   runs in software, so HIL laps can be compared with pure simulation laps. */
#include "torque_vectoring.h"
#include "config.h"
#include "util.h"
#include <math.h>

float tv_target_yaw_rate(float speed_ms, float steering_rad)
{
    float wheel_angle = steering_rad * STEERING_TO_WHEEL;
    return speed_ms * tanf(wheel_angle) / WHEELBASE_M;
}

void torque_vectoring(const TvInputs *in, const TorqueLimits *limits, float torque_nm[4])
{
    for (int wheel = 0; wheel < 4; wheel++)
        torque_nm[wheel] = 0.0f;
    if (!isfinite(in->request_nm) || !isfinite(in->speed_ms) || !isfinite(in->steering_rad)
        || !isfinite(in->yaw_rate)) {
        return;
    }

    // 1. share the request equally between the four motors
    float base = clampf(in->request_nm / 4.0f, -limits->regen_nm, limits->drive_nm);

    // 2. more torque on the right turns the car left
    float correction = 0.0f;
    if (in->speed_ms > TV_MIN_SPEED_MS) {
        float target = tv_target_yaw_rate(in->speed_ms, in->steering_rad);
        correction   = in->gain * TV_KP_NM_PER_RADS * (target - in->yaw_rate);
    }

    // 3. keep both sides inside the limits so the total torque is unchanged
    float room = fminf(limits->drive_nm - base, base + limits->regen_nm);
    float most = fminf(TV_MAX_CORRECTION_NM, room);
    correction = clampf(correction, -most, most);

    torque_nm[WHEEL_FL] = base - correction;
    torque_nm[WHEEL_RL] = base - correction;
    torque_nm[WHEEL_FR] = base + correction;
    torque_nm[WHEEL_RR] = base + correction;
}
