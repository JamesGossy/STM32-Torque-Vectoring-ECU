/* The real motor controller firmware (its SIL build, node 1) against the ECU
   firmware in ecu_sim, with simple stand-ins for nodes 2 to 4. Checks arming,
   current following, speed derating on a free motor and both ways of stopping.
   Usage: motor_interop <ecu_sim port> */
#include "../../app/can_protocol.h"
#include "../../sim/tcp.h"
#include "config.h" // this and the next two come from the motor controller repo
#include "foc.h"
#include "sim.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REQUEST_NM 20.0f

static int sock, failures;
static uint8_t stand_in_state[5]
    = { 0, MC_STATE_IDLE, MC_STATE_IDLE, MC_STATE_IDLE, MC_STATE_IDLE };
static uint8_t ecu_state;
static uint16_t ecu_inhibit;
static int drive;

static void check(int ok, const char *what)
{
    printf("%s %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) failures++;
}

static void send_record(uint8_t kind, uint32_t id, const uint8_t *data, uint8_t len)
{
    LinkRecord r;
    memset(&r, 0, sizeof r);
    r.kind = kind;
    r.id   = id;
    r.len  = len;
    if (data) memcpy(r.data, data, len);
    tcp_write(sock, &r, sizeof r);
}

static void send_stand_ins(void)
{
    for (uint32_t node = 2; node <= 4; node++) {
        uint8_t data[8] = { stand_in_state[node], MC_MODE_TORQUE, 0, 0, 0, 0, 0, 0 };
        send_record(LINK_FRAME, CAN_ID(node, MC_MSG_HEARTBEAT), data, 8);
        memset(data, 0, 8);
        send_record(LINK_FRAME, CAN_ID(node, MC_MSG_IQ_SPEED), data, 8);
        int16_t fet = 300, board = 250;
        can_put_f32(data, 24.0f);
        memcpy(data + 4, &fet, 2);
        memcpy(data + 6, &board, 2);
        send_record(LINK_FRAME, CAN_ID(node, MC_MSG_BUS_TEMP), data, 8);
    }
}

static void send_hil_inputs(void)
{
    uint8_t data[8] = { 0 };
    can_put_f32(data + 4, REQUEST_NM);
    send_record(LINK_FRAME, CAN_ID(ECU_NODE, HIL_MSG_DRIVER), data, 8);
    memset(data, 0, 8);
    send_record(LINK_FRAME, CAN_ID(ECU_NODE, HIL_MSG_WHEELS_F), data, 8);
    send_record(LINK_FRAME, CAN_ID(ECU_NODE, HIL_MSG_WHEELS_R), data, 8);
    data[0] = HIL_FLAG_ON | (drive ? HIL_FLAG_DRIVE : 0);
    send_record(LINK_FRAME, CAN_ID(ECU_NODE, HIL_MSG_CONTROL), data, 8);
}

// One 10 ms bus period: controller telemetry and HIL inputs out, ECU commands back.
static void bus_step(void)
{
    uint32_t id;
    uint8_t data[8], len;
    while (sim_can_pop(&id, data, &len))
        send_record(LINK_FRAME, id, data, len);
    send_stand_ins();
    send_hil_inputs();
    send_record(LINK_STEP, 10, NULL, 0);

    LinkRecord r;
    while (tcp_read(sock, &r, sizeof r) == 0 && r.kind != LINK_STEP_DONE) {
        uint32_t node = CAN_NODE(r.id), msg = CAN_MSG(r.id);
        if (node == ECU_NODE && msg == ECU_MSG_STATUS) {
            ecu_state   = r.data[0];
            ecu_inhibit = can_get_u16(r.data + 2);
        } else if (node == 1 || r.id == CAN_ESTOP_ID) {
            sim_can_inject(r.id, r.data, r.len);
        } else if (node <= 4 && msg == MC_CMD_SET_STATE) {
            stand_in_state[node] = r.data[0] == MC_RUN_TORQUE ? MC_STATE_RUN : MC_STATE_IDLE;
        }
    }
    sim_run_ms(10);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: motor_interop <ecu_sim port>\n");
        return 2;
    }

    // 1. boot and calibrate the motor controller, as on the bench
    sim_config_t config;
    sim_default_config(&config);
    sim_init(&config);
    sim_run_ms(300);
    sim_serial_inject("calibrate\n");
    sim_run_ms(200);
    for (int i = 0; i < 100 && foc.state == ST_CAL; i++)
        sim_run_ms(100);
    check(foc.state == ST_IDLE && foc.faults == 0, "motor controller calibrated");

    sock = tcp_connect(atoi(argv[1]));
    LinkRecord hello;
    if (sock < 0 || tcp_read(sock, &hello, sizeof hello) != 0) {
        printf("FAIL cannot reach ecu_sim\n");
        return 1;
    }

    // 2. wait for standby, ask for drive, and let the free motor spin up
    float worst_speed = 0.0f, settled_iq = 0.0f;
    for (int t = 0; t < 800; t++) {
        bus_step();
        if (ecu_state == 1 && ecu_inhibit == 0) drive = 1;
        if (t == 130) settled_iq = foc.iq;
        worst_speed = fmaxf(worst_speed, fabsf(foc.omega_m));
    }
    float expected_amps = REQUEST_NM / 4.0f / MOTOR_NM_PER_AMP * MOTOR_DIRECTION[0];
    check(ecu_state == 2, "ECU armed");
    check(foc.state == ST_RUN && foc.mode == MODE_TORQUE, "controller running in torque mode");
    check(fabsf(settled_iq - expected_amps) < 0.2f, "controller follows the commanded current");
    check(worst_speed < 1.05f * SPEED_MAX, "speed derating holds the free motor near its limit");

    // 3. drive off, then the ECU disappears
    drive = 0;
    for (int t = 0; t < 30; t++)
        bus_step();
    check(ecu_state == 1 && foc.state == ST_IDLE, "drive off puts the controller in idle");

    drive = 1;
    for (int t = 0; t < 30; t++)
        bus_step();
    tcp_close(sock);
    for (int t = 0; t < 40; t++)
        sim_run_ms(10);
    check(foc.state == ST_IDLE, "controller stops by itself when the ECU goes away");

    printf("motor speed peak %.0f rad/s (limit %.0f), current at 1.3 s %.2f A (expected %.2f)\n",
        (double)worst_speed, (double)SPEED_MAX, (double)settled_iq, (double)expected_amps);
    return failures ? 1 : 0;
}
