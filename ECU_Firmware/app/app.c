/* ECU main logic. A 100 Hz control step picks the input source (own sensors or
   the HIL simulator), checks that torque is allowed, then runs derating and
   torque vectoring and sends the result to the motor controllers. */
#include "app.h"
#include "can_protocol.h"
#include "config.h"
#include "console.h"
#include "gps.h"
#include "hal.h"
#include "autonomy.h"
#include "hil.h"
#include "imu.h"
#include "motors.h"
#include "panel.h"
#include "torque_vectoring.h"
#include <math.h>
#include <string.h>

static EcuStatus status;
static uint32_t started_ms, hil_lost_ms, brake_held_ms, tick_count;
static int hil_lost, hil_drive_before, imu_found, brake_must_lift;
static float gyro_sum;
static uint32_t gyro_samples;
static uint8_t status_count;

void app_init(void)
{
    memset(&status, 0, sizeof status);
    motors_init();
    panel_init();
    hil_init();
    autonomy_reset();
    gps_init();
    driver_inputs_reset();
    console_init();
    status.state = ECU_STARTUP;
    started_ms   = hal_millis();
    hil_lost = hil_drive_before = 0;
    brake_held_ms = tick_count = 0;
    brake_must_lift            = 1; // a brake already held at power-up does not count
    gyro_sum                   = 0.0f;
    gyro_samples               = 0;
    imu_found                  = imu_init();
}

const EcuStatus *app_status(void)
{
    return &status;
}

// Every way out of drive comes through here, so a new arming gesture is always needed.
static void go_standby(void)
{
    if (status.state == ECU_DRIVE) status.state = ECU_STANDBY;
    brake_held_ms   = 0;
    brake_must_lift = 1;
}

// A hard stop while driving is latched, so a flickering fault cannot re-arm the car.
static void latch_fault(uint16_t cause)
{
    status.state    = ECU_FAULT;
    status.fault    = cause;
    brake_held_ms   = 0;
    brake_must_lift = 1;
}

void app_disarm(void)
{
    go_standby();
}

void app_clear_fault(void)
{
    if (status.state != ECU_FAULT) return;
    status.state = ECU_STANDBY;
    status.fault = 0;
}

/* ---- inputs ---- */

// Board NTC to degC. An open or shorted sensor reads very hot, so derating stops torque.
static float ntc_temp_c(float volts)
{
    if (volts < 0.02f || volts > ADC_VREF_V - 0.02f) return 999.0f;
    float ohms = NTC_R_TOP_OHM * volts / (ADC_VREF_V - volts);
    return 1.0f / (1.0f / 298.15f + logf(ohms / NTC_R25_OHM) / NTC_BETA) - 273.15f;
}

static void read_board(void)
{
    PedalVolts volts;
    volts.apps1_v    = hal_adc_volts(ADC_APPS1) / SENSOR_DIVIDER;
    volts.apps2_v    = hal_adc_volts(ADC_APPS2) / SENSOR_DIVIDER;
    volts.bpps_v     = hal_adc_volts(ADC_BPPS) / SENSOR_DIVIDER;
    volts.steering_v = hal_adc_volts(ADC_STEERING) / SENSOR_DIVIDER;
    driver_inputs_update(&volts, CONTROL_PERIOD_MS, &status.pedals);

    status.vin_v      = hal_adc_volts(ADC_VIN) * VIN_DIVIDER;
    status.ecu_temp_c = ntc_temp_c(hal_adc_volts(ADC_TEMP));
}

// Returns the measured yaw rate, or NAN if the IMU is not answering.
static float read_yaw_rate(void)
{
    if (!imu_found) imu_found = imu_init(); // the sensor may still be booting
    ImuSample sample;
    status.imu_ok = imu_found && imu_read(&sample);
    if (!status.imu_ok) {
        imu_found = 0;
        return NAN;
    }
    float yaw = sample.gyro[2] * IMU_YAW_SIGN;
    if (status.state == ECU_STARTUP) {
        gyro_sum += yaw;
        gyro_samples++;
        status.gyro_bias = gyro_sum / gyro_samples;
    }
    return yaw - status.gyro_bias;
}

// Moves between the ECU's own sensors and the simulator's inputs.
static void choose_source(uint32_t now)
{
    const HilInputs *hil = hil_inputs();
    int fresh            = hil_fresh(now);

    if (status.hil && (!fresh || !hil->on)) {
        status.hil = 0;
        if (!fresh && status.state == ECU_DRIVE) latch_fault(INHIBIT_HIL_LOST);
        go_standby();
        if (!fresh) {
            hil_lost    = 1;
            hil_lost_ms = now;
        }
    }
    if (HIL_ALLOWED && !status.hil && fresh && hil->on && status.state != ECU_DRIVE) {
        status.hil       = 1;
        hil_drive_before = hil->drive; // arming needs a fresh off to on edge
    }
    if (hil_lost && now - hil_lost_ms > 1000) hil_lost = 0;
}

static void gather_inputs(float own_yaw)
{
    const HilInputs *hil = hil_inputs();
    float wheel_sum      = 0.0f;

    status.autonomy        = 0;
    status.target_speed_ms = 0.0f;
    if (status.hil) {
        status.steering_rad = hil->steering_rad;
        status.request_nm   = hil->request_nm;
        status.yaw_rate     = hil->yaw_rate;
        for (int wheel = 0; wheel < 4; wheel++)
            wheel_sum += hil->wheel_speed[wheel];
        if (hil->autonomy) {
            // The ECU drives: the simulator's own steering and torque request are ignored.
            AutoCommand command    = autonomy_step(&hil->scan, 0.25f * wheel_sum * WHEEL_RADIUS_M);
            status.autonomy        = 1;
            status.steering_rad    = command.steering;
            status.request_nm      = command.request_nm;
            status.target_speed_ms = command.target_speed_ms;
        }
    } else {
        status.steering_rad = status.pedals.steering_rad;
        status.request_nm   = status.pedals.request_nm;
        status.yaw_rate     = own_yaw;
        for (int wheel = 0; wheel < 4; wheel++)
            wheel_sum += motor_wheel_speed(wheel);
    }
    status.speed_ms        = 0.25f * wheel_sum * WHEEL_RADIUS_M;
    status.target_yaw_rate = tv_target_yaw_rate(status.speed_ms, status.steering_rad);
}

/* ---- safety ---- */

static uint16_t find_inhibits(uint32_t now)
{
    uint16_t inhibit = 0;
    status.online    = 0;

    for (int wheel = 0; wheel < 4; wheel++) {
        if (motor_online(wheel, now)) status.online |= (uint8_t)(1u << wheel);
        if (!motor_online(wheel, now)) {
            inhibit |= INHIBIT_MOTOR_OFFLINE;
        } else if (!motor_healthy(wheel, now)) {
            inhibit |= INHIBIT_MOTOR_FAULT;
        }
    }
    if (motors_refusing(now)) inhibit |= INHIBIT_MOTOR_FAULT;
    if (status.state == ECU_STARTUP) inhibit |= INHIBIT_STARTUP;
    if (!hal_can_ok()) inhibit |= INHIBIT_CAN;
    if (hil_lost) inhibit |= INHIBIT_HIL_LOST;
    if (status.hil && hil_inputs()->autonomy && !hil_cones_fresh(now))
        inhibit |= INHIBIT_CONES_LOST;

    // the ECU's own sensors only matter when the simulator is not standing in for them
    if (!status.hil) {
        uint8_t pedal = status.pedals.problems;
        if (status.vin_v < VIN_MIN_V) inhibit |= INHIBIT_NO_POWER;
        if (pedal & PEDAL_SENSOR_FAULT) inhibit |= INHIBIT_PEDAL_SENSOR;
        if (pedal & PEDAL_DISAGREE) inhibit |= INHIBIT_PEDAL_DISAGREE;
        if (pedal & PEDAL_BRAKE_THROTTLE) inhibit |= INHIBIT_BRAKE_THROTTLE;
        if (!status.imu_ok) inhibit |= INHIBIT_IMU;
    }
    return inhibit;
}

// Pedals: press the brake, throttle released, and hold it in standby. HIL: the sim's request.
static int arm_requested(void)
{
    if (status.hil) {
        int drive        = hil_inputs()->drive;
        int edge         = drive && !hil_drive_before;
        hil_drive_before = drive;
        return edge;
    }
    int braking = status.pedals.brake > BRAKE_ON;
    if (!braking) brake_must_lift = 0;
    int holding   = braking && !brake_must_lift && status.pedals.throttle < BRAKE_APPS_RESET;
    brake_held_ms = holding && status.state == ECU_STANDBY ? brake_held_ms + CONTROL_PERIOD_MS : 0;
    return brake_held_ms >= ARM_BRAKE_HOLD_MS;
}

static void update_state(uint32_t now)
{
    int armed_request = arm_requested();
    int hard_stop     = (status.inhibit & ~INHIBIT_SOFT & ~INHIBIT_STARTUP) != 0;

    switch (status.state) {
    case ECU_STARTUP:
        if (now - started_ms >= GYRO_CAL_MS) status.state = ECU_STANDBY;
        break;
    case ECU_STANDBY:
        if (armed_request && !hard_stop) {
            status.state  = ECU_DRIVE;
            brake_held_ms = 0;
        }
        break;
    case ECU_DRIVE:
        if (hard_stop) latch_fault(status.inhibit & ~INHIBIT_SOFT);
        break;
    case ECU_FAULT:
        break;
    }
    if (status.hil && !hil_inputs()->drive) go_standby();
}

/* ---- torque ---- */

static void compute_torque(uint32_t now)
{
    DerateInputs in;
    for (int wheel = 0; wheel < 4; wheel++) {
        const Motor *m             = motor_get(wheel);
        in.used[wheel]             = motor_online(wheel, now);
        in.fet_temp_c[wheel]       = m->fet_temp_c;
        in.motor_speed_rads[wheel] = m->speed_rads;
        in.bus_v[wheel]            = m->bus_v;
    }
    in.ecu_temp_c   = status.ecu_temp_c;
    in.car_speed_ms = status.speed_ms;
    status.derate   = derate_compute(&in);
    status.panel    = *panel_dials();

    TorqueLimits limits;
    limits.drive_nm = MOTOR_MAX_TORQUE_NM * status.derate.drive * status.panel.power;
    limits.regen_nm = MOTOR_MAX_TORQUE_NM * status.derate.regen * status.panel.regen;

    TvInputs tv;
    tv.request_nm   = status.request_nm;
    tv.speed_ms     = status.speed_ms;
    tv.steering_rad = status.steering_rad;
    tv.yaw_rate     = status.yaw_rate;
    tv.gain         = status.panel.tv;
    torque_vectoring(&tv, &limits, status.torque_nm);

    if (status.state != ECU_DRIVE || status.inhibit) {
        for (int wheel = 0; wheel < 4; wheel++)
            status.torque_nm[wheel] = 0.0f;
    }
}

/* ---- CAN out ---- */

static uint8_t percent(float fraction)
{
    return (uint8_t)lroundf(fraction * 100.0f);
}

static void send_status(void)
{
    uint8_t data[8];
    data[0] = status.state;
    data[1] = (uint8_t)((status.hil ? ECU_FLAG_HIL : 0) | (status.online << 4));
    data[2] = (uint8_t)status.inhibit;
    data[3] = (uint8_t)(status.inhibit >> 8);
    data[4] = percent(status.derate.drive);
    data[5] = percent(status.derate.regen);
    data[6] = status.derate.reasons;
    data[7] = status_count++;
    hal_can_send(CAN_ID(ECU_NODE, ECU_MSG_STATUS), data, 8);

    can_put_f32(data, status.target_yaw_rate);
    can_put_f32(data + 4, status.yaw_rate);
    hal_can_send(CAN_ID(ECU_NODE, ECU_MSG_YAW), data, 8);

    if (status.autonomy) {
        can_put_f32(data, status.steering_rad);
        can_put_f32(data + 4, status.request_nm);
        hal_can_send(CAN_ID(ECU_NODE, ECU_MSG_COMMAND), data, 8);
    }
}

/* ---- loops ---- */

static void control_step(void)
{
    uint32_t now = hal_millis();

    // 1. read everything
    read_board();
    float own_yaw = read_yaw_rate();
    choose_source(now);
    gather_inputs(own_yaw);

    // 2. decide whether torque is allowed
    status.inhibit = find_inhibits(now);
    update_state(now);
    if (status.state != ECU_STARTUP) status.inhibit &= (uint16_t)~INHIBIT_STARTUP;
    if (status.state == ECU_FAULT) status.inhibit |= status.fault;

    // 3. derate, vector and send
    compute_torque(now);
    motors_command(status.torque_nm, status.state == ECU_DRIVE, now);
    send_status();
}

void app_poll(void)
{
    uint32_t id, now = hal_millis();
    uint8_t data[8], len;
    while (hal_can_recv(&id, data, &len)) {
        if (!motors_handle_frame(id, data, len, now) && !panel_handle_frame(id, data, len))
            hil_handle_frame(id, data, len, now);
    }

    uint8_t bytes[32];
    size_t count;
    while ((count = hal_gps_read(bytes, sizeof bytes)) > 0) {
        for (size_t i = 0; i < count; i++)
            gps_feed((char)bytes[i]);
    }

    console_poll();
}

void app_tick(void)
{
    tick_count++;
    if (tick_count % CONTROL_PERIOD_MS == 0) control_step();

    uint32_t blink = status.state == ECU_FAULT ? 50u
        : status.state == ECU_DRIVE            ? 100u
        : status.state == ECU_STANDBY          ? 500u
                                               : 250u;
    hal_led_set((hal_millis() / blink) % 2);
    console_tick();
    hal_wdg_kick();
}
