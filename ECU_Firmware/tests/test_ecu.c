/* The whole ECU application on the host board, against four small fake motor
   controllers that follow the real controller's CAN protocol. */
#include "test.h"
#include "app.h"
#include "can_protocol.h"
#include "config.h"
#include "hal.h"
#include "local_map.h"
#include "sim.h"
#include <string.h>

typedef struct {
    int present; // answers on the bus
    int refuse;  // stays idle, like an uncalibrated controller
    uint8_t state;
    uint16_t faults;
    float iq, speed, bus_v, fet_c;
    uint32_t setpoint_ms;
} FakeMotor;

static FakeMotor fake[4];
static int hil_on, hil_drive;
static float hil_steer, hil_request, hil_yaw, hil_wheel;
static int use_hil;
static int hil_auto, hil_cones_off; // driverless: send cone frames, or stop sending them
static ConeScan hil_cones;
static uint8_t hil_count;
static float command_steer, command_request;
static int command_frames;

/* ---- fake motor controllers ---- */

static void fake_reset(void)
{
    memset(fake, 0, sizeof fake);
    for (int i = 0; i < 4; i++) {
        fake[i].present = 1;
        fake[i].state   = MC_STATE_IDLE;
        fake[i].bus_v   = 24.0f;
        fake[i].fet_c   = 30.0f;
    }
}

static int fake_of_node(uint32_t node)
{
    for (int i = 0; i < 4; i++) {
        if (MOTOR_NODE[i] == node) return i;
    }
    return -1;
}

// Everything the ECU sent since the last call.
static void fake_listen(void)
{
    uint32_t id;
    uint8_t data[8], len;
    while (sim_can_pop(&id, data, &len)) {
        if (id == CAN_ESTOP_ID) {
            for (int i = 0; i < 4; i++)
                fake[i].state = MC_STATE_IDLE;
            continue;
        }
        if (id == CAN_ID(ECU_NODE, ECU_MSG_COMMAND)) {
            command_steer   = can_get_f32(data);
            command_request = can_get_f32(data + 4);
            command_frames++;
            continue;
        }
        int i = fake_of_node(CAN_NODE(id));
        if (i < 0 || !fake[i].present) continue;
        if (CAN_MSG(id) == MC_CMD_SET_STATE && !fake[i].refuse && fake[i].state != MC_STATE_FAULT) {
            fake[i].state       = data[0] == MC_RUN_TORQUE ? MC_STATE_RUN : MC_STATE_IDLE;
            fake[i].setpoint_ms = sim_millis();
        }
        if (CAN_MSG(id) == MC_CMD_SET_IQ) {
            fake[i].iq          = fake[i].state == MC_STATE_RUN ? can_get_f32(data) : 0.0f;
            fake[i].setpoint_ms = sim_millis();
        }
        if (CAN_MSG(id) == MC_CMD_CLEAR_FAULTS && fake[i].state == MC_STATE_FAULT) {
            fake[i].state  = MC_STATE_IDLE;
            fake[i].faults = 0;
        }
    }
    for (int i = 0; i < 4; i++) {
        if (fake[i].state == MC_STATE_RUN && sim_millis() - fake[i].setpoint_ms > 250) {
            fake[i].state = MC_STATE_IDLE;
        }
    }
}

static void fake_talk(void)
{
    for (int i = 0; i < 4; i++) {
        if (!fake[i].present) continue;
        uint8_t data[8] = { fake[i].state, MC_MODE_TORQUE, (uint8_t)fake[i].faults,
            (uint8_t)(fake[i].faults >> 8), 0, 0, 0, 0 };
        sim_can_inject(CAN_ID(MOTOR_NODE[i], MC_MSG_HEARTBEAT), data, 8);
        can_put_f32(data, fake[i].iq);
        can_put_f32(data + 4, fake[i].speed);
        sim_can_inject(CAN_ID(MOTOR_NODE[i], MC_MSG_IQ_SPEED), data, 8);
        int16_t fet = (int16_t)(fake[i].fet_c * 10.0f), amb = 250;
        can_put_f32(data, fake[i].bus_v);
        memcpy(data + 4, &fet, 2);
        memcpy(data + 6, &amb, 2);
        sim_can_inject(CAN_ID(MOTOR_NODE[i], MC_MSG_BUS_TEMP), data, 8);
    }
}

/* ---- simulator inputs ---- */

static void hil_talk(void)
{
    uint8_t data[8];
    can_put_f32(data, hil_steer);
    can_put_f32(data + 4, hil_request);
    sim_can_inject(CAN_ID(ECU_NODE, HIL_MSG_DRIVER), data, 8);
    can_put_f32(data, hil_wheel);
    can_put_f32(data + 4, hil_wheel);
    sim_can_inject(CAN_ID(ECU_NODE, HIL_MSG_WHEELS_F), data, 8);
    sim_can_inject(CAN_ID(ECU_NODE, HIL_MSG_WHEELS_R), data, 8);
    if (hil_auto) {
        int send = hil_cones.count ? hil_cones.count : 1; // an empty scan is one frame
        for (int i = 0; i < send; i++) {
            if (hil_cones_off) continue;
            memset(data, 0, 8);
            if (hil_cones.count) {
                can_put_u16(data, (uint16_t)lroundf(hil_cones.cone[i].range_m * 100.0f));
                can_put_i16(data + 2, (int16_t)lroundf(hil_cones.cone[i].bearing_rad * 1000.0f));
                data[4] = hil_cones.cone[i].colour;
            }
            data[5] = (uint8_t)i;
            data[6] = (uint8_t)hil_cones.count;
            data[7] = hil_count;
            sim_can_inject(CAN_ID(ECU_NODE, HIL_MSG_CONE), data, 8);
        }
    }
    memset(data, 0, 8);
    data[0] = (uint8_t)((hil_on ? HIL_FLAG_ON : 0) | (hil_drive ? HIL_FLAG_DRIVE : 0)
        | (hil_auto ? HIL_FLAG_AUTONOMY : 0));
    data[1] = hil_count++;
    can_put_f32(data + 4, hil_yaw);
    sim_can_inject(CAN_ID(ECU_NODE, HIL_MSG_CONTROL), data, 8);
}

// Runs the bus in 10 ms steps, like the simulator does.
static void run(int ms)
{
    for (int t = 0; t < ms; t += 10) {
        fake_talk();
        if (use_hil) hil_talk();
        sim_run_ms(10);
        fake_listen();
    }
}

static void boot(void)
{
    sim_reset();
    fake_reset();
    use_hil = hil_on = hil_drive = hil_auto = hil_cones_off = 0;
    hil_cones.count                                         = 0;
    command_frames                                          = 0;
    hil_steer = hil_request = hil_yaw = hil_wheel = 0.0f;
    run(GYRO_CAL_MS + 20);
}

static float motor_torque(int wheel)
{
    return fake[wheel].iq * MOTOR_NM_PER_AMP * MOTOR_DIRECTION[wheel];
}

static void start_hil_drive(float request)
{
    use_hil = hil_on = 1;
    hil_request      = request;
    hil_wheel        = 5.0f / WHEEL_RADIUS_M;
    run(50);
    hil_drive = 1;
    run(50);
}

/* ---- tests ---- */

static void boots_into_standby_with_no_torque(void)
{
    boot();
    const EcuStatus *s = app_status();
    CHECK(s->state == ECU_STANDBY);
    CHECK(s->online == 0x0F);
    CHECK(s->imu_ok);
    for (int i = 0; i < 4; i++)
        CHECK(fake[i].state == MC_STATE_IDLE);
}

static void hil_drive_sends_scaled_currents(void)
{
    boot();
    start_hil_drive(40.0f);
    const EcuStatus *s = app_status();
    CHECK(s->hil);
    CHECK(s->state == ECU_DRIVE);
    CHECK(s->inhibit == 0);
    for (int i = 0; i < 4; i++) {
        CHECK(fake[i].state == MC_STATE_RUN);
        CHECK_NEAR(motor_torque(i), 10.0f, 1e-3);
    }
    CHECK(fake[WHEEL_FL].iq < 0.0f); // left motors spin the other way
    CHECK_NEAR(s->speed_ms, 5.0f, 1e-4);
}

static void hil_steering_vectors_torque(void)
{
    boot();
    hil_steer = 0.5f;
    start_hil_drive(40.0f);
    CHECK(motor_torque(WHEEL_FR) > motor_torque(WHEEL_FL) + 1.0f);
    float total = 0.0f;
    for (int i = 0; i < 4; i++)
        total += motor_torque(i);
    CHECK_NEAR(total, 40.0f, 1e-3);
}

static void hil_needs_a_fresh_drive_request(void)
{
    boot();
    use_hil = hil_on = hil_drive = 1; // drive already on when HIL starts
    hil_request                  = 40.0f;
    run(100);
    CHECK(app_status()->hil);
    CHECK(app_status()->state == ECU_STANDBY);
}

static void lost_simulator_stops_torque(void)
{
    boot();
    start_hil_drive(40.0f);
    use_hil = 0;
    run(HIL_TIMEOUT_MS + 20);
    const EcuStatus *s = app_status();
    CHECK(!s->hil);
    CHECK(s->state == ECU_FAULT);
    CHECK(s->inhibit & INHIBIT_HIL_LOST);
    run(400);
    for (int i = 0; i < 4; i++)
        CHECK(fake[i].state == MC_STATE_IDLE);
}

static void hot_controller_derates_all_wheels(void)
{
    boot();
    start_hil_drive(1000.0f);
    fake[WHEEL_RL].fet_c = 0.5f * (DERATE_FET_START_C + DERATE_FET_END_C);
    run(30);
    const EcuStatus *s = app_status();
    CHECK(s->derate.reasons & DERATE_FET_TEMP);
    for (int i = 0; i < 4; i++)
        CHECK_NEAR(motor_torque(i), 0.5f * MOTOR_MAX_TORQUE_NM, 0.1);
}

static void faulted_controller_latches_a_fault(void)
{
    boot();
    start_hil_drive(40.0f);
    fake[WHEEL_FR].state  = MC_STATE_FAULT;
    fake[WHEEL_FR].faults = 1u << 3;
    run(30);
    CHECK(app_status()->state == ECU_FAULT);
    CHECK(app_status()->inhibit & INHIBIT_MOTOR_FAULT);

    fake[WHEEL_FR].state  = MC_STATE_IDLE; // the cause goes away, the latch stays
    fake[WHEEL_FR].faults = 0;
    run(100);
    CHECK(app_status()->state == ECU_FAULT);
    CHECK(app_status()->fault & INHIBIT_MOTOR_FAULT);
    for (int i = 0; i < 4; i++)
        CHECK(fake[i].state == MC_STATE_IDLE);

    sim_serial_inject("clear\n");
    run(30);
    CHECK(app_status()->state == ECU_STANDBY);
    CHECK(app_status()->fault == 0);
}

static void missing_controller_blocks_arming(void)
{
    boot();
    fake[WHEEL_RR].present = 0;
    run(MOTOR_TIMEOUT_MS + 20);
    start_hil_drive(40.0f);
    CHECK(app_status()->state == ECU_STANDBY);
    CHECK(app_status()->inhibit & INHIBIT_MOTOR_OFFLINE);
    CHECK_NEAR(motor_torque(WHEEL_FL), 0.0f, 1e-6);
}

static void refusing_controller_latches_a_fault(void)
{
    boot();
    fake[WHEEL_FL].refuse = 1;
    start_hil_drive(40.0f);
    CHECK(app_status()->state == ECU_DRIVE);
    for (int i = 0; i < 4; i++)
        CHECK_NEAR(motor_torque(i), 0.0f, 1e-6); // no one-sided drive
    run(MOTOR_START_MS + 50);
    CHECK(app_status()->state == ECU_FAULT);
}

static void pedals_arm_after_brake_hold(void)
{
    boot();
    sim_set_pedals(APPS1_ZERO_V, APPS2_ZERO_V, BPPS_FULL_V, STEER_CENTRE_V);
    run(ARM_BRAKE_HOLD_MS - 50);
    CHECK(app_status()->state == ECU_STANDBY);
    run(100);
    CHECK(app_status()->state == ECU_DRIVE);

    sim_set_pedals(APPS1_ZERO_V + 2.0f, APPS2_ZERO_V - 2.0f, BPPS_ZERO_V, STEER_CENTRE_V);
    run(30);
    CHECK_NEAR(app_status()->request_nm, 0.5f * DRIVE_REQUEST_MAX_NM, 3.0);
    for (int i = 0; i < 4; i++)
        CHECK(motor_torque(i) > 10.0f);
}

static void pedal_disagreement_cuts_torque_but_stays_armed(void)
{
    boot();
    sim_set_pedals(APPS1_ZERO_V, APPS2_ZERO_V, BPPS_FULL_V, STEER_CENTRE_V);
    run(ARM_BRAKE_HOLD_MS + 50);
    sim_set_pedals(APPS1_ZERO_V + 2.0f, APPS2_ZERO_V - 1.0f, BPPS_ZERO_V, STEER_CENTRE_V);
    run(APPS_DISAGREE_MS + 30);
    const EcuStatus *s = app_status();
    CHECK(s->state == ECU_DRIVE);
    CHECK(s->inhibit & INHIBIT_PEDAL_DISAGREE);
    for (int i = 0; i < 4; i++)
        CHECK_NEAR(motor_torque(i), 0.0f, 1e-6);
}

static void brake_at_standstill_never_drives_backwards(void)
{
    boot();
    sim_set_pedals(APPS1_ZERO_V, APPS2_ZERO_V, BPPS_FULL_V, STEER_CENTRE_V);
    run(ARM_BRAKE_HOLD_MS + 50);
    CHECK(app_status()->state == ECU_DRIVE);
    CHECK(app_status()->request_nm < 0.0f);
    for (int i = 0; i < 4; i++)
        CHECK_NEAR(motor_torque(i), 0.0f, 1e-6);
}

static void off_stays_off_while_the_brake_is_held(void)
{
    boot();
    sim_set_pedals(APPS1_ZERO_V, APPS2_ZERO_V, BPPS_FULL_V, STEER_CENTRE_V);
    run(ARM_BRAKE_HOLD_MS + 50);
    CHECK(app_status()->state == ECU_DRIVE);
    sim_serial_inject("off\n");
    run(3 * ARM_BRAKE_HOLD_MS);
    CHECK(app_status()->state == ECU_STANDBY);

    sim_set_pedals(APPS1_ZERO_V, APPS2_ZERO_V, BPPS_ZERO_V, STEER_CENTRE_V); // lift, press again
    run(50);
    sim_set_pedals(APPS1_ZERO_V, APPS2_ZERO_V, BPPS_FULL_V, STEER_CENTRE_V);
    run(ARM_BRAKE_HOLD_MS + 50);
    CHECK(app_status()->state == ECU_DRIVE);
}

static void disarm_zeroes_current_at_once(void)
{
    boot();
    start_hil_drive(40.0f);
    hil_drive = 0;
    run(20);
    CHECK(app_status()->state == ECU_STANDBY);
    for (int i = 0; i < 4; i++) {
        CHECK_NEAR(fake[i].iq, 0.0f, 1e-6);
        CHECK(fake[i].state == MC_STATE_IDLE);
    }
}

static void send_panel(uint8_t tv, uint8_t power, uint8_t regen)
{
    uint8_t data[8] = { tv, power, regen, 0, 0, 0, 0, 0 };
    sim_can_inject(CAN_ID(PANEL_NODE, PANEL_MSG_DIALS), data, 8);
}

static void panel_power_dial_limits_torque(void)
{
    boot();
    start_hil_drive(1000.0f);
    send_panel(100, 50, 100);
    run(30);
    for (int i = 0; i < 4; i++)
        CHECK_NEAR(motor_torque(i), 0.5f * MOTOR_MAX_TORQUE_NM, 0.1);
    send_panel(100, 0, 100);
    run(30);
    for (int i = 0; i < 4; i++)
        CHECK_NEAR(motor_torque(i), 0.0f, 1e-6);
}

static void panel_is_optional_and_kept_when_it_goes_quiet(void)
{
    boot();
    start_hil_drive(1000.0f);
    CHECK_NEAR(app_status()->panel.power, 1.0f, 1e-6); // no panel, no change
    send_panel(100, 30, 100);
    run(2000); // far longer than any frame period
    CHECK_NEAR(app_status()->panel.power, 0.3f, 1e-6);
}

static void usb_only_power_blocks_pedal_mode(void)
{
    boot();
    sim_set_adc_volts(ADC_VIN, 0.0f);
    run(20);
    CHECK(app_status()->inhibit & INHIBIT_NO_POWER);
    start_hil_drive(40.0f); // HIL does not need the external sensors
    CHECK(app_status()->state == ECU_DRIVE);
}

static void own_gyro_is_used_without_hil(void)
{
    sim_reset();
    fake_reset();
    use_hil = 0;
    sim_set_gyro_z(0.02f); // bias while standing still
    run(GYRO_CAL_MS + 20);
    CHECK_NEAR(app_status()->gyro_bias, 0.02f, 1e-3);
    sim_set_gyro_z(0.52f);
    run(20);
    CHECK_NEAR(app_status()->yaw_rate, 0.5f, 1e-3);

    sim_set_imu_present(0);
    run(20);
    CHECK(app_status()->inhibit & INHIBIT_IMU);
}

static void console_reports_status(void)
{
    boot();
    char text[4096];
    sim_serial_take(text, sizeof text);
    sim_serial_inject("status\n");
    run(10);
    sim_serial_take(text, sizeof text);
    CHECK(strstr(text, "STANDBY") != NULL);
    CHECK(strstr(text, "inhibit=none") != NULL);
    sim_serial_inject("motors\n");
    run(10);
    sim_serial_take(text, sizeof text);
    CHECK(strstr(text, "RR node 4 online") != NULL);
}

static void status_frame_goes_out(void)
{
    sim_reset();
    sim_run_ms(10);
    uint32_t id;
    uint8_t data[8], len;
    int seen = 0;
    while (sim_can_pop(&id, data, &len)) {
        if (id == CAN_ID(ECU_NODE, ECU_MSG_STATUS)) seen = data[0] == ECU_STARTUP;
    }
    CHECK(seen);
}

/* ---- driverless ---- */

static void corridor(ConeScan *scan, float bend)
{
    scan->count = 0;
    for (int gate = 1; gate <= 4; gate++) {
        float x = 3.0f * (float)gate, centre = bend * (float)(gate * gate) * 0.25f;
        for (int side = 0; side < 2; side++) {
            float y           = centre + (side ? -1.5f : 1.5f);
            Cone *cone        = &scan->cone[scan->count++];
            cone->range_m     = hypotf(x, y);
            cone->bearing_rad = atan2f(y, x);
            cone->colour      = side ? CONE_YELLOW : CONE_BLUE;
        }
    }
}

static void start_autonomy(float bend)
{
    corridor(&hil_cones, bend);
    hil_auto = 1;
    start_hil_drive(0.0f);
}

static void autonomy_drives_from_cones(void)
{
    boot();
    start_autonomy(0.0f);
    const EcuStatus *s = app_status();
    CHECK(s->autonomy);
    CHECK(s->state == ECU_DRIVE);
    CHECK_NEAR(s->steering_rad, 0.0f, 0.02);
    CHECK(s->request_nm > 0.0f); // 5 m/s, aiming for 6
    CHECK_NEAR(s->target_speed_ms, AUTO_CRUISE_SPEED_MS, 1e-3);
    float total = 0.0f;
    for (int i = 0; i < 4; i++)
        total += motor_torque(i);
    CHECK_NEAR(total, s->request_nm, 0.5);
}

static void autonomy_ignores_the_simulators_own_steering_and_torque(void)
{
    boot();
    hil_steer = 2.0f; // what a software autopilot would have asked for
    start_autonomy(0.0f);
    CHECK_NEAR(app_status()->steering_rad, 0.0f, 0.02);
    CHECK(app_status()->request_nm < 100.0f);
}

static void autonomy_steers_and_reports_the_command(void)
{
    boot();
    start_autonomy(0.5f);
    run(300);
    CHECK(app_status()->steering_rad > 0.05f);
    command_frames = 0;
    run(20);
    CHECK(command_frames >= 1);
    CHECK_NEAR(command_steer, app_status()->steering_rad, 1e-5);
    CHECK_NEAR(command_request, app_status()->request_nm, 1e-4);
}

static void autonomy_without_cones_slows_the_car(void)
{
    boot();
    start_autonomy(0.0f);
    hil_cones.count = 0; // frames still arrive, they just show nothing
    run(50);
    CHECK(app_status()->state == ECU_DRIVE);
    CHECK_NEAR(app_status()->target_speed_ms, 0.0f, 1e-6);
    CHECK(app_status()->request_nm < 0.0f);
}

static void stale_cones_latch_a_fault(void)
{
    boot();
    start_autonomy(0.0f);
    CHECK(app_status()->state == ECU_DRIVE);
    hil_cones_off = 1;
    run(100);
    CHECK(app_status()->state == ECU_FAULT);
    CHECK(app_status()->fault & INHIBIT_CONES_LOST);
    for (int i = 0; i < 4; i++)
        CHECK_NEAR(motor_torque(i), 0.0f, 1e-3);
    hil_cones_off = 0; // cones back, but the fault stays until cleared
    run(100);
    CHECK(app_status()->state == ECU_FAULT);
}

static void autonomy_will_not_arm_without_cones(void)
{
    boot();
    corridor(&hil_cones, 0.0f);
    hil_auto = hil_cones_off = 1;
    start_hil_drive(0.0f);
    CHECK(app_status()->state == ECU_STANDBY);
    CHECK(app_status()->inhibit & INHIBIT_CONES_LOST);
}

static void a_scan_with_a_lost_frame_is_not_used(void)
{
    boot();
    start_autonomy(0.0f);
    // claim 8 cones but only send 7 of them
    uint8_t data[8] = { 0 };
    for (int i = 0; i < 7; i++) {
        can_put_u16(data, 500);
        data[5] = (uint8_t)i;
        data[6] = 8;
        data[7] = hil_count;
        sim_can_inject(CAN_ID(ECU_NODE, HIL_MSG_CONE), data, 8);
    }
    memset(data, 0, 8);
    data[0] = HIL_FLAG_ON | HIL_FLAG_DRIVE | HIL_FLAG_AUTONOMY;
    data[1] = hil_count++;
    sim_can_inject(CAN_ID(ECU_NODE, HIL_MSG_CONTROL), data, 8);
    sim_run_ms(10);
    CHECK(app_status()->state == ECU_DRIVE); // one bad tick is not a fault
    CHECK_NEAR(app_status()->target_speed_ms, AUTO_CRUISE_SPEED_MS, 1e-3); // the old scan is kept
}

static void normal_hil_is_unchanged_by_the_cone_frames(void)
{
    boot();
    corridor(&hil_cones, 0.0f);
    start_hil_drive(40.0f); // cones are not sent, autonomy flag is off
    CHECK(!app_status()->autonomy);
    CHECK_NEAR(app_status()->request_nm, 40.0f, 1e-4);
    CHECK(command_frames == 0);
}

int main(void)
{
    const test_case cases[] = {
        T(boots_into_standby_with_no_torque),
        T(hil_drive_sends_scaled_currents),
        T(hil_steering_vectors_torque),
        T(hil_needs_a_fresh_drive_request),
        T(lost_simulator_stops_torque),
        T(hot_controller_derates_all_wheels),
        T(faulted_controller_latches_a_fault),
        T(missing_controller_blocks_arming),
        T(refusing_controller_latches_a_fault),
        T(pedals_arm_after_brake_hold),
        T(pedal_disagreement_cuts_torque_but_stays_armed),
        T(brake_at_standstill_never_drives_backwards),
        T(off_stays_off_while_the_brake_is_held),
        T(disarm_zeroes_current_at_once),
        T(usb_only_power_blocks_pedal_mode),
        T(own_gyro_is_used_without_hil),
        T(console_reports_status),
        T(status_frame_goes_out),
        T(panel_power_dial_limits_torque),
        T(panel_is_optional_and_kept_when_it_goes_quiet),
        T(autonomy_drives_from_cones),
        T(autonomy_ignores_the_simulators_own_steering_and_torque),
        T(autonomy_steers_and_reports_the_command),
        T(autonomy_without_cones_slows_the_car),
        T(stale_cones_latch_a_fault),
        T(autonomy_will_not_arm_without_cones),
        T(a_scan_with_a_lost_frame_is_not_used),
        T(normal_hil_is_unchanged_by_the_cone_frames),
    };
    return run_tests(cases, sizeof cases / sizeof cases[0]);
}
