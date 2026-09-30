/* Client side of the motor controller CAN protocol. Controllers drop to idle if
   setpoints stop for 250 ms, so the ECU resends every control period and never
   relies on a final zero command getting through. */
#include "motors.h"
#include "can_protocol.h"
#include "config.h"
#include "hal.h"
#include <string.h>

static Motor motors[4];
static int was_enabled;
static uint32_t enabled_ms;

void motors_init(void)
{
    memset(motors, 0, sizeof motors);
    was_enabled = 0;
}

static int wheel_of_node(uint32_t node)
{
    for (int wheel = 0; wheel < 4; wheel++) {
        if (MOTOR_NODE[wheel] == node) return wheel;
    }
    return -1;
}

int motors_handle_frame(uint32_t id, const uint8_t *data, uint8_t len, uint32_t now)
{
    int wheel = wheel_of_node(CAN_NODE(id));
    if (wheel < 0 || len < 8) return 0;
    Motor *m = &motors[wheel];

    switch (CAN_MSG(id)) {
    case MC_MSG_HEARTBEAT:
        m->state    = data[0];
        m->mode     = data[1];
        m->faults   = can_get_u16(data + 2);
        m->heard_ms = now;
        m->heard    = 1;
        return 1;
    case MC_MSG_IQ_SPEED:
        m->iq_a       = can_get_f32(data);
        m->speed_rads = can_get_f32(data + 4);
        return 1;
    case MC_MSG_BUS_TEMP:
        m->bus_v      = can_get_f32(data);
        m->fet_temp_c = can_get_i16(data + 4) / 10.0f;
        return 1;
    }
    return 0;
}

const Motor *motor_get(int wheel)
{
    return &motors[wheel];
}

int motor_online(int wheel, uint32_t now)
{
    return motors[wheel].heard && now - motors[wheel].heard_ms <= MOTOR_TIMEOUT_MS;
}

int motor_healthy(int wheel, uint32_t now)
{
    const Motor *m = &motors[wheel];
    int usable     = m->state == MC_STATE_IDLE || m->state == MC_STATE_RUN;
    return motor_online(wheel, now) && usable && m->faults == 0;
}

float motor_wheel_speed(int wheel)
{
    return motors[wheel].speed_rads * MOTOR_DIRECTION[wheel] / GEAR_RATIO;
}

static void send_state(int wheel, uint8_t run, uint32_t now)
{
    hal_can_send(CAN_ID(MOTOR_NODE[wheel], MC_CMD_SET_STATE), &run, 1);
    motors[wheel].asked_ms = now;
}

static void send_current(int wheel, float amps)
{
    uint8_t data[4];
    can_put_f32(data, amps);
    hal_can_send(CAN_ID(MOTOR_NODE[wheel], MC_CMD_SET_IQ), data, 4);
}

static int is_running(const Motor *m)
{
    return m->state == MC_STATE_RUN && m->mode == MC_MODE_TORQUE;
}

void motors_command(const float torque_nm[4], int enable, uint32_t now)
{
    int just_disabled = was_enabled && !enable;
    if (enable && !was_enabled) enabled_ms = now;
    was_enabled = enable;

    // no torque until all four run, or the running side alone would yaw the car
    int all_running = 1;
    for (int wheel = 0; wheel < 4; wheel++)
        all_running &= is_running(&motors[wheel]);

    for (int wheel = 0; wheel < 4; wheel++) {
        Motor *m    = &motors[wheel];
        int may_ask = now - m->asked_ms >= MOTOR_RETRY_MS;

        if (!enable) {
            send_current(wheel, 0.0f); // in case the idle request is lost
            if (just_disabled || (m->state == MC_STATE_RUN && may_ask)) {
                send_state(wheel, MC_RUN_IDLE, now);
            }
            continue;
        }
        if (!is_running(m) && may_ask) send_state(wheel, MC_RUN_TORQUE, now);
        float amps = torque_nm[wheel] * MOTOR_DIRECTION[wheel] / MOTOR_NM_PER_AMP;
        send_current(wheel, all_running ? amps : 0.0f);
    }
}

int motors_refusing(uint32_t now)
{
    if (!was_enabled || now - enabled_ms < MOTOR_START_MS) return 0;
    for (int wheel = 0; wheel < 4; wheel++) {
        if (!is_running(&motors[wheel])) return 1;
    }
    return 0;
}

void motors_clear_faults(void)
{
    for (int wheel = 0; wheel < 4; wheel++) {
        hal_can_send(CAN_ID(MOTOR_NODE[wheel], MC_CMD_CLEAR_FAULTS), NULL, 0);
    }
}

void motors_estop(void)
{
    hal_can_send(CAN_ESTOP_ID, NULL, 0);
}
