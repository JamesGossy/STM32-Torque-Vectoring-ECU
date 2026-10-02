/* The whole ECU application on the host board, against four small fake motor
   controllers that follow the real controller's CAN protocol. */
#include "test.h"
#include "app.h"
#include "can_protocol.h"
#include "config.h"
#include "hal.h"
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
    memset(data, 0, 8);
    data[0] = (uint8_t)((hil_on ? HIL_FLAG_ON : 0) | (hil_drive ? HIL_FLAG_DRIVE : 0));
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
    use_hil = hil_on = hil_drive = 0;
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
    };
    return run_tests(cases, sizeof cases / sizeof cases[0]);
}
