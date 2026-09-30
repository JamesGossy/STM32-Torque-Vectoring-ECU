/* Pedal and steering scaling, the plausibility rules, and the GPS reader. */
#include "test.h"
#include "config.h"
#include "driver_inputs.h"
#include "gps.h"

// Connector volts for a throttle position, the way a healthy pedal would read.
static PedalVolts pedals(float throttle, float brake, float steer_fraction)
{
    PedalVolts v;
    v.apps1_v    = APPS1_ZERO_V + throttle * (APPS1_FULL_V - APPS1_ZERO_V);
    v.apps2_v    = APPS2_ZERO_V + throttle * (APPS2_FULL_V - APPS2_ZERO_V);
    v.bpps_v     = BPPS_ZERO_V + brake * (BPPS_FULL_V - BPPS_ZERO_V);
    v.steering_v = STEER_CENTRE_V + steer_fraction * (STEER_LEFT_V - STEER_CENTRE_V);
    return v;
}

/* ---- pedals ---- */

static void released_pedals_ask_for_nothing(void)
{
    DriverInputs out;
    PedalVolts v = pedals(0, 0, 0);
    driver_inputs_reset();
    driver_inputs_update(&v, 10, &out);
    CHECK(out.problems == 0);
    CHECK_NEAR(out.request_nm, 0.0f, 1e-6);
    CHECK_NEAR(out.steering_rad, 0.0f, 1e-6);
}

static void full_throttle_asks_for_full_drive(void)
{
    DriverInputs out;
    PedalVolts v = pedals(1, 0, 0);
    driver_inputs_reset();
    driver_inputs_update(&v, 10, &out);
    CHECK_NEAR(out.throttle, 1.0f, 1e-5);
    CHECK_NEAR(out.request_nm, DRIVE_REQUEST_MAX_NM, 1e-3);
    v = pedals(0.5f, 0, 0);
    driver_inputs_update(&v, 10, &out);
    CHECK_NEAR(out.throttle, 0.5f, 1e-5);
}

static void brake_pedal_asks_for_regen(void)
{
    DriverInputs out;
    PedalVolts v = pedals(0, 1, 0);
    driver_inputs_reset();
    driver_inputs_update(&v, 10, &out);
    CHECK_NEAR(out.request_nm, -REGEN_REQUEST_MAX_NM, 1e-3);
}

static void steering_scales_to_lock(void)
{
    DriverInputs out;
    PedalVolts v = pedals(0, 0, 1);
    driver_inputs_reset();
    driver_inputs_update(&v, 10, &out);
    CHECK_NEAR(out.steering_rad, STEER_LOCK_RAD, 1e-5);
    v = pedals(0, 0, -0.5f);
    driver_inputs_update(&v, 10, &out);
    CHECK_NEAR(out.steering_rad, -0.5f * STEER_LOCK_RAD, 1e-5);
}

static void disagreement_cuts_after_100_ms(void)
{
    DriverInputs out;
    PedalVolts v = pedals(0.5f, 0, 0);
    v.apps2_v    = APPS2_ZERO_V + 0.3f * (APPS2_FULL_V - APPS2_ZERO_V); // 20 % apart
    driver_inputs_reset();
    for (int ms = 10; ms <= 100; ms += 10) {
        driver_inputs_update(&v, 10, &out);
        CHECK(out.problems == 0);
    }
    driver_inputs_update(&v, 10, &out);
    CHECK(out.problems & PEDAL_DISAGREE);
    CHECK_NEAR(out.request_nm, 0.0f, 1e-6);

    v = pedals(0.5f, 0, 0);
    driver_inputs_update(&v, 10, &out);
    CHECK(out.problems == 0);
}

static void brake_with_throttle_cuts_until_released(void)
{
    DriverInputs out;
    PedalVolts v = pedals(0.5f, 0.5f, 0);
    driver_inputs_reset();
    driver_inputs_update(&v, 10, &out);
    CHECK(out.problems & PEDAL_BRAKE_THROTTLE);

    v = pedals(0.5f, 0, 0); // brake off but throttle still pressed
    driver_inputs_update(&v, 10, &out);
    CHECK(out.problems & PEDAL_BRAKE_THROTTLE);

    v = pedals(0.0f, 0, 0);
    driver_inputs_update(&v, 10, &out);
    v = pedals(0.5f, 0, 0);
    driver_inputs_update(&v, 10, &out);
    CHECK(out.problems == 0);
    CHECK(out.request_nm > 0.0f);
}

static void broken_wire_is_a_fault(void)
{
    DriverInputs out;
    PedalVolts v = pedals(0, 0, 0);
    v.apps1_v    = 0.0f;
    driver_inputs_reset();
    driver_inputs_update(&v, 10, &out);
    CHECK(out.problems & PEDAL_SENSOR_FAULT);
    v            = pedals(0, 0, 0);
    v.steering_v = 5.0f;
    driver_inputs_update(&v, 10, &out);
    CHECK(out.problems & PEDAL_SENSOR_FAULT);
}

/* ---- GPS ---- */

static void feed(const char *text)
{
    while (*text)
        gps_feed(*text++);
}

static void reads_rmc_sentence(void)
{
    gps_init();
    feed("$GNRMC,012345.00,A,3351.1234,S,15112.5678,E,10.0,90.5,300926,,,A*");
    // work out the checksum here so the test data stays readable
    const char *body = "GNRMC,012345.00,A,3351.1234,S,15112.5678,E,10.0,90.5,300926,,,A";
    unsigned sum     = 0;
    for (const char *p = body; *p; p++)
        sum ^= (unsigned char)*p;
    char tail[8];
    snprintf(tail, sizeof tail, "%02X\r\n", sum);
    feed(tail);

    const GpsFix *fix = gps_fix();
    CHECK(fix->sentences == 1);
    CHECK(fix->valid);
    CHECK_NEAR(fix->lat_deg, -(33.0 + 51.1234 / 60.0), 1e-6);
    CHECK_NEAR(fix->lon_deg, 151.0 + 12.5678 / 60.0, 1e-6);
    CHECK_NEAR(fix->speed_ms, 5.14444, 1e-3);
    CHECK_NEAR(fix->course_deg, 90.5, 1e-4);
}

static void ignores_bad_checksum_and_other_sentences(void)
{
    gps_init();
    feed("$GNRMC,012345.00,A,3351.1234,S,15112.5678,E,10.0,90.5,300926,,,A*00\r\n");
    feed("$GNGGA,012345.00,3351.1234,S,15112.5678,E,1,08,1.0,10.0,M,0.0,M,,*00\r\n");
    feed("$GPRMC,,V,,,,,,,,,,N*53\r\n"); // real no-fix sentence
    const GpsFix *fix = gps_fix();
    CHECK(fix->sentences == 1);
    CHECK(!fix->valid);
}

int main(void)
{
    const test_case cases[] = {
        T(released_pedals_ask_for_nothing),
        T(full_throttle_asks_for_full_drive),
        T(brake_pedal_asks_for_regen),
        T(steering_scales_to_lock),
        T(disagreement_cuts_after_100_ms),
        T(brake_with_throttle_cuts_until_released),
        T(broken_wire_is_a_fault),
        T(reads_rmc_sentence),
        T(ignores_bad_checksum_and_other_sentences),
    };
    return run_tests(cases, sizeof cases / sizeof cases[0]);
}
