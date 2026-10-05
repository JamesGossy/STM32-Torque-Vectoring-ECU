/* Pure pursuit steering and a proportional speed controller, both working on the
   centre line from the planner. The car sits at the origin facing along x. */
#include "autonomy.h"
#include "config.h"
#include "planner.h"
#include "util.h"
#include <math.h>
#include <stdbool.h>

static float previous_steering;

void autonomy_reset(void)
{
    previous_steering = 0.0f;
}

/* ---- steering ---- */

// Steering that drives a circle through the target point.
static float pure_pursuit(MapPoint target)
{
    float distance_squared = target.x * target.x + target.y * target.y;
    if (distance_squared < 0.01f) return 0.0f;
    float curvature = 2.0f * target.y / distance_squared;
    return atanf(WHEELBASE_M * curvature) / STEERING_TO_WHEEL;
}

static MapPoint closest_on_segment(MapPoint start, MapPoint end)
{
    float dx             = end.x - start.x;
    float dy             = end.y - start.y;
    float length_squared = dx * dx + dy * dy;
    if (length_squared < 1e-6f) return start;
    float fraction = clampf(-(start.x * dx + start.y * dy) / length_squared, 0.0f, 1.0f);
    return (MapPoint) { start.x + fraction * dx, start.y + fraction * dy };
}

// The point AUTO_LOOKAHEAD_M along the path from where the car is closest to it.
static bool lookahead_point(const LocalMap *path, MapPoint *target)
{
    if (path->count < 2) return false;
    int segments    = path->count - 1;
    int first       = -1;
    float closest_m = INFINITY;
    MapPoint start  = { 0.0f, 0.0f };
    for (int i = 0; i < segments; i++) {
        MapPoint point = closest_on_segment(path->points[i], path->points[i + 1]);
        float distance = hypotf(point.x, point.y);
        if (distance < closest_m) {
            closest_m = distance;
            first     = i;
            start     = point;
        }
    }

    *target           = start;
    float remaining_m = AUTO_LOOKAHEAD_M;
    for (int segment = first; segment < segments; segment++) {
        MapPoint end = path->points[segment + 1];
        float length = hypotf(end.x - start.x, end.y - start.y);
        *target      = end;
        if (length >= remaining_m && length > 1e-6f) {
            float fraction = remaining_m / length;
            target->x      = start.x + fraction * (end.x - start.x);
            target->y      = start.y + fraction * (end.y - start.y);
            break;
        }
        remaining_m -= length;
        start = end;
    }
    return hypotf(target->x, target->y) > AUTO_MIN_TARGET_M;
}

// The steering motor cannot jump, so the change per step is limited.
static float limit_steering(float wanted)
{
    float angle  = clampf(wanted, -AUTO_MAX_STEERING, AUTO_MAX_STEERING);
    float change = AUTO_MAX_STEERING_RATE * CONTROL_PERIOD_MS * 0.001f;
    return clampf(angle, previous_steering - change, previous_steering + change);
}

/* ---- the step ---- */

AutoCommand autonomy_step(const ConeScan *scan, float speed_ms)
{
    AutoCommand command = { previous_steering, 0.0f, 0.0f };
    if (!isfinite(speed_ms)) return command;

    LocalMap path;
    planner_step(scan, &path);

    float wanted = previous_steering; // keep the wheel where it is if there is no path
    MapPoint target;
    if (lookahead_point(&path, &target)) {
        wanted = pure_pursuit(target);
        // slow down for tighter steering
        float wheel_angle       = wanted * STEERING_TO_WHEEL;
        command.target_speed_ms = AUTO_CRUISE_SPEED_MS / (1.0f + fabsf(wheel_angle));
    }

    previous_steering  = limit_steering(wanted);
    command.steering   = previous_steering;
    command.request_nm = clampf(AUTO_SPEED_KP_NM * (command.target_speed_ms - speed_ms),
        -REGEN_REQUEST_MAX_NM, DRIVE_REQUEST_MAX_NM);
    return command;
}
