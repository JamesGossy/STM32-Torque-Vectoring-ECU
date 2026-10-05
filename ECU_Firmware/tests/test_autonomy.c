/* The driverless planner and controller: centre line, steering direction, speed. */
#include "test.h"
#include "autonomy.h"
#include "config.h"
#include "planner.h"

static void add_cone(ConeScan *scan, float x, float y, int colour)
{
    Cone *cone        = &scan->cone[scan->count++];
    cone->range_m     = hypotf(x, y);
    cone->bearing_rad = atan2f(y, x);
    cone->colour      = (uint8_t)colour;
}

// Gates 3 m apart, 3 m wide, the centre line bending by `bend` m of sideways shift per gate.
static ConeScan corridor(float bend)
{
    ConeScan scan = { .count = 0 };
    for (int gate = 1; gate <= 4; gate++) {
        float x      = 3.0f * (float)gate;
        float centre = bend * (float)(gate * gate) * 0.25f;
        add_cone(&scan, x, centre + 1.5f, CONE_BLUE);
        add_cone(&scan, x, centre - 1.5f, CONE_YELLOW);
    }
    return scan;
}

static void planner_puts_the_line_between_the_cones(void)
{
    ConeScan scan = corridor(0.0f);
    LocalMap path;
    planner_step(&scan, &path);
    CHECK(path.count == 5); // the car, then four gates
    for (int i = 1; i < path.count; i++)
        CHECK_NEAR(path.points[i].y, 0.0f, 1e-2);
    CHECK_NEAR(path.points[1].x, 3.0f, 1e-2);
}

static void planner_follows_one_edge_when_the_other_is_missing(void)
{
    ConeScan scan = { .count = 0 };
    for (int gate = 1; gate <= 3; gate++)
        add_cone(&scan, 3.0f * (float)gate, 1.5f, CONE_BLUE);
    LocalMap path;
    planner_step(&scan, &path);
    CHECK(path.count >= 2);
    // PLAN_EDGE_OFFSET_M to the right of the left edge
    CHECK_NEAR(path.points[1].y, 1.5f - PLAN_EDGE_OFFSET_M, 0.3);
}

static void planner_copes_with_no_cones(void)
{
    ConeScan scan = { .count = 0 };
    LocalMap path;
    planner_step(&scan, &path);
    CHECK(path.count < 2);
}

static void straight_corridor_steers_straight_and_speeds_up(void)
{
    autonomy_reset();
    ConeScan scan      = corridor(0.0f);
    AutoCommand result = autonomy_step(&scan, 0.0f);
    CHECK_NEAR(result.steering, 0.0f, 1e-3);
    CHECK_NEAR(result.target_speed_ms, AUTO_CRUISE_SPEED_MS, 1e-3);
    CHECK_NEAR(result.request_nm, DRIVE_REQUEST_MAX_NM, 1e-3); // far below target, so clamped
}

static void left_bend_steers_left_and_right_bend_steers_right(void)
{
    ConeScan left = corridor(0.5f), right = corridor(-0.5f);
    float steer_left = 0.0f, steer_right = 0.0f;
    autonomy_reset();
    for (int i = 0; i < 20; i++)
        steer_left = autonomy_step(&left, 5.0f).steering;
    autonomy_reset();
    for (int i = 0; i < 20; i++)
        steer_right = autonomy_step(&right, 5.0f).steering;
    CHECK(steer_left > 0.05f);
    CHECK(steer_right < -0.05f);
    CHECK_NEAR(steer_left, -steer_right, 1e-3);
}

static void tighter_steering_lowers_the_target_speed(void)
{
    ConeScan straight = corridor(0.0f), bend = corridor(0.8f);
    autonomy_reset();
    float fast = autonomy_step(&straight, 5.0f).target_speed_ms;
    autonomy_reset();
    float slow = 0.0f;
    for (int i = 0; i < 20; i++)
        slow = autonomy_step(&bend, 5.0f).target_speed_ms;
    CHECK(slow < fast);
}

static void steering_cannot_jump(void)
{
    ConeScan bend = corridor(1.0f);
    autonomy_reset();
    AutoCommand first = autonomy_step(&bend, 5.0f);
    CHECK(fabsf(first.steering) <= AUTO_MAX_STEERING_RATE * CONTROL_PERIOD_MS * 0.001f + 1e-5f);
}

static void above_the_target_speed_asks_for_regen(void)
{
    ConeScan scan = corridor(0.0f);
    autonomy_reset();
    CHECK(autonomy_step(&scan, 9.0f).request_nm < 0.0f);
}

static void no_cones_holds_the_wheel_and_stops(void)
{
    ConeScan bend = corridor(0.5f), empty = { .count = 0 };
    autonomy_reset();
    float held = 0.0f;
    for (int i = 0; i < 20; i++)
        held = autonomy_step(&bend, 5.0f).steering;
    AutoCommand blind = autonomy_step(&empty, 5.0f);
    CHECK_NEAR(blind.steering, held, 1e-5);
    CHECK_NEAR(blind.target_speed_ms, 0.0f, 1e-6);
    CHECK(blind.request_nm < 0.0f);
}

static void bad_speed_gives_no_torque(void)
{
    ConeScan scan = corridor(0.0f);
    autonomy_reset();
    AutoCommand result = autonomy_step(&scan, NAN);
    CHECK_NEAR(result.request_nm, 0.0f, 1e-6);
}

int main(void)
{
    test_case cases[] = {
        T(planner_puts_the_line_between_the_cones),
        T(planner_follows_one_edge_when_the_other_is_missing),
        T(planner_copes_with_no_cones),
        T(straight_corridor_steers_straight_and_speeds_up),
        T(left_bend_steers_left_and_right_bend_steers_right),
        T(tighter_steering_lowers_the_target_speed),
        T(steering_cannot_jump),
        T(above_the_target_speed_asks_for_regen),
        T(no_cones_holds_the_wheel_and_stops),
        T(bad_speed_gives_no_torque),
    };
    return run_tests(cases, sizeof cases / sizeof cases[0]);
}
