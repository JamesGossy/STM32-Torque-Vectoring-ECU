/* Torque vectoring and derating: signs, totals, limits and bad inputs. */
#include "test.h"
#include "config.h"
#include "derate.h"
#include "torque_vectoring.h"

static const TorqueLimits FULL = { MOTOR_MAX_TORQUE_NM, MOTOR_MAX_TORQUE_NM };

static float total(const float t[4])
{
    return t[0] + t[1] + t[2] + t[3];
}

static TvInputs straight(float request, float speed)
{
    TvInputs in = { request, speed, 0.0f, 0.0f };
    return in;
}

/* ---- torque vectoring ---- */

static void equal_split_when_going_straight(void)
{
    float t[4];
    TvInputs in = straight(40.0f, 10.0f);
    torque_vectoring(&in, &FULL, t);
    for (int i = 0; i < 4; i++)
        CHECK_NEAR(t[i], 10.0f, 1e-5);
}

static void left_turn_moves_torque_right(void)
{
    float t[4];
    TvInputs in     = straight(40.0f, 10.0f);
    in.steering_rad = 1.0f; // car not yet rotating, so it needs help turning left
    torque_vectoring(&in, &FULL, t);
    CHECK(t[WHEEL_FR] > t[WHEEL_FL]);
    CHECK_NEAR(t[WHEEL_FR], t[WHEEL_RR], 1e-6);
    CHECK_NEAR(total(t), 40.0f, 1e-4);
    CHECK_NEAR(t[WHEEL_FR] - t[WHEEL_FL], 2.0f * TV_MAX_CORRECTION_NM, 1e-4);
}

static void over_rotation_moves_torque_left(void)
{
    float t[4];
    TvInputs in = straight(40.0f, 10.0f);
    in.yaw_rate = 0.3f; // rotating left with the wheel straight
    torque_vectoring(&in, &FULL, t);
    CHECK(t[WHEEL_FL] > t[WHEEL_FR]);
    CHECK_NEAR(total(t), 40.0f, 1e-4);
}

static void vectoring_works_under_regen(void)
{
    float t[4];
    TvInputs in     = straight(-40.0f, 10.0f);
    in.steering_rad = 0.5f;
    torque_vectoring(&in, &FULL, t);
    CHECK(t[WHEEL_FR] > t[WHEEL_FL]);
    CHECK_NEAR(total(t), -40.0f, 1e-4);
}

static void correction_keeps_motors_inside_limits(void)
{
    float t[4];
    TvInputs in     = straight(1000.0f, 10.0f);
    in.steering_rad = 1.0f;
    torque_vectoring(&in, &FULL, t);
    for (int i = 0; i < 4; i++)
        CHECK_NEAR(t[i], MOTOR_MAX_TORQUE_NM, 1e-5);

    TorqueLimits derated = { 10.0f, 5.0f };
    in.request_nm        = 30.0f;
    torque_vectoring(&in, &derated, t);
    for (int i = 0; i < 4; i++)
        CHECK(t[i] <= 10.0f + 1e-5f && t[i] >= -5.0f - 1e-5f);
    in.request_nm = -1000.0f;
    torque_vectoring(&in, &derated, t);
    for (int i = 0; i < 4; i++)
        CHECK_NEAR(t[i], -5.0f, 1e-5);
}

static void no_vectoring_when_stopped(void)
{
    float t[4];
    TvInputs in     = straight(20.0f, 0.0f);
    in.steering_rad = 1.0f;
    torque_vectoring(&in, &FULL, t);
    CHECK_NEAR(t[WHEEL_FL], t[WHEEL_FR], 1e-6);
}

static void bad_inputs_give_zero(void)
{
    float t[4];
    TvInputs in = straight(NAN, 10.0f);
    torque_vectoring(&in, &FULL, t);
    CHECK_NEAR(total(t), 0.0f, 1e-9);
    in          = straight(20.0f, 10.0f);
    in.yaw_rate = INFINITY;
    torque_vectoring(&in, &FULL, t);
    CHECK_NEAR(total(t), 0.0f, 1e-9);
}

static void target_yaw_matches_bicycle_model(void)
{
    float expected = 10.0f * tanf(0.5f * STEERING_TO_WHEEL) / WHEELBASE_M;
    CHECK_NEAR(tv_target_yaw_rate(10.0f, 0.5f), expected, 1e-6);
    CHECK(tv_target_yaw_rate(10.0f, -0.5f) < 0.0f);
}

/* ---- derating ---- */

static DerateInputs cool_car(void)
{
    DerateInputs in;
    for (int i = 0; i < 4; i++) {
        in.used[i]             = 1;
        in.fet_temp_c[i]       = 30.0f;
        in.motor_speed_rads[i] = 100.0f;
        in.bus_v[i]            = 24.0f;
    }
    in.ecu_temp_c   = 30.0f;
    in.car_speed_ms = 10.0f;
    return in;
}

static void ramp_shape(void)
{
    CHECK_NEAR(derate_ramp(50.0f, 80.0f, 90.0f), 1.0f, 1e-6);
    CHECK_NEAR(derate_ramp(85.0f, 80.0f, 90.0f), 0.5f, 1e-6);
    CHECK_NEAR(derate_ramp(95.0f, 80.0f, 90.0f), 0.0f, 1e-6);
    CHECK_NEAR(derate_ramp(12.0f, 14.0f, 10.0f), 0.5f, 1e-6); // falling limit
    CHECK_NEAR(derate_ramp(NAN, 80.0f, 90.0f), 0.0f, 1e-6);
}

static void cool_car_has_full_torque(void)
{
    DerateInputs in = cool_car();
    Derate d        = derate_compute(&in);
    CHECK_NEAR(d.drive, 1.0f, 1e-6);
    CHECK_NEAR(d.regen, 1.0f, 1e-6);
    CHECK(d.reasons == 0);
}

static void hot_controller_limits_both_directions(void)
{
    DerateInputs in         = cool_car();
    in.fet_temp_c[WHEEL_RR] = 0.5f * (DERATE_FET_START_C + DERATE_FET_END_C);
    Derate d                = derate_compute(&in);
    CHECK_NEAR(d.drive, 0.5f, 1e-5);
    CHECK_NEAR(d.regen, 0.5f, 1e-5);
    CHECK(d.reasons == DERATE_FET_TEMP);
}

static void speed_and_low_bus_limit_drive_only(void)
{
    DerateInputs in               = cool_car();
    in.motor_speed_rads[WHEEL_FL] = -MOTOR_MAX_SPEED_RADS; // reverse counts too
    Derate d                      = derate_compute(&in);
    CHECK_NEAR(d.drive, 0.0f, 1e-6);
    CHECK_NEAR(d.regen, 1.0f, 1e-6);

    in                 = cool_car();
    in.bus_v[WHEEL_FR] = DERATE_LOW_V_END;
    d                  = derate_compute(&in);
    CHECK_NEAR(d.drive, 0.0f, 1e-6);
    CHECK_NEAR(d.regen, 1.0f, 1e-6);
    CHECK(d.reasons == DERATE_LOW_BUS);
}

static void high_bus_limits_regen_only(void)
{
    DerateInputs in    = cool_car();
    in.bus_v[WHEEL_RL] = DERATE_HIGH_V_END;
    Derate d           = derate_compute(&in);
    CHECK_NEAR(d.drive, 1.0f, 1e-6);
    CHECK_NEAR(d.regen, 0.0f, 1e-6);
    CHECK(d.reasons == DERATE_HIGH_BUS);
}

static void offline_controllers_are_ignored(void)
{
    DerateInputs in    = cool_car();
    in.used[WHEEL_FL]  = 0;
    in.bus_v[WHEEL_FL] = 0.0f; // never heard from
    Derate d           = derate_compute(&in);
    CHECK_NEAR(d.drive, 1.0f, 1e-6);
}

static void regen_fades_near_standstill(void)
{
    DerateInputs in = cool_car();
    in.car_speed_ms = 0.5f * REGEN_FULL_SPEED_MS;
    Derate d        = derate_compute(&in);
    CHECK_NEAR(d.regen, 0.5f, 1e-5);
    CHECK_NEAR(d.drive, 1.0f, 1e-6);
    in.car_speed_ms = 0.0f;
    d               = derate_compute(&in);
    CHECK_NEAR(d.regen, 0.0f, 1e-6);
    CHECK(d.reasons == DERATE_SLOW);
}

static void broken_ecu_sensor_stops_torque(void)
{
    DerateInputs in = cool_car();
    in.ecu_temp_c   = 999.0f;
    Derate d        = derate_compute(&in);
    CHECK_NEAR(d.drive, 0.0f, 1e-6);
    CHECK(d.reasons & DERATE_ECU_TEMP);
}

int main(void)
{
    const test_case cases[] = {
        T(equal_split_when_going_straight),
        T(left_turn_moves_torque_right),
        T(over_rotation_moves_torque_left),
        T(vectoring_works_under_regen),
        T(correction_keeps_motors_inside_limits),
        T(no_vectoring_when_stopped),
        T(bad_inputs_give_zero),
        T(target_yaw_matches_bicycle_model),
        T(ramp_shape),
        T(cool_car_has_full_torque),
        T(hot_controller_limits_both_directions),
        T(speed_and_low_bus_limit_drive_only),
        T(high_bus_limits_regen_only),
        T(offline_controllers_are_ignored),
        T(regen_fades_near_standstill),
        T(broken_ecu_sensor_stops_torque),
    };
    return run_tests(cases, sizeof cases / sizeof cases[0]);
}
