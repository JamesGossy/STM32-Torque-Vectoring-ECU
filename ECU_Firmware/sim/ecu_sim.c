/* ecu_sim: the ECU firmware built for the PC, joined to the Formula Student
   simulator over TCP instead of a real CAN bus. It runs in lock-step: time only
   moves when the simulator sends a STEP, so every run is repeatable.

     ecu_sim [--port 5700] [--once]    --once exits after the first client leaves */
#include "can_protocol.h"
#include "sim.h"
#include "tcp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int send_record(int sock, uint8_t kind, uint32_t id, const uint8_t *data, uint8_t len)
{
    LinkRecord record;
    memset(&record, 0, sizeof record);
    record.kind = kind;
    record.len  = len;
    record.id   = id;
    if (data) memcpy(record.data, data, len > 8 ? 8 : len);
    return tcp_write(sock, &record, sizeof record);
}

// Runs one client session. Returns when the simulator disconnects.
static void serve(int sock)
{
    uint8_t realtime = 0;
    sim_reset();
    if (send_record(sock, LINK_HELLO, 0, &realtime, 1) < 0) return;

    LinkRecord record;
    while (tcp_read(sock, &record, sizeof record) == 0) {
        if (record.kind == LINK_FRAME) {
            sim_can_inject(record.id, record.data, record.len);
        } else if (record.kind == LINK_STEP) {
            sim_run_ms(record.id);
            uint32_t id;
            uint8_t data[8], len;
            while (sim_can_pop(&id, data, &len)) {
                if (send_record(sock, LINK_FRAME, id, data, len) < 0) return;
            }
            if (send_record(sock, LINK_STEP_DONE, sim_millis(), NULL, 0) < 0) return;
        }
    }
}

int main(int argc, char **argv)
{
    int port = LINK_PORT, once = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--port") && i + 1 < argc) {
            port = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--once")) {
            once = 1;
        } else {
            printf("usage: ecu_sim [--port %d] [--once]\n", LINK_PORT);
            return 1;
        }
    }

    int listener = tcp_listen(port);
    if (listener < 0) {
        fprintf(stderr, "ecu_sim: cannot listen on port %d\n", port);
        return 1;
    }
    printf("ecu_sim: waiting for the simulator on tcp://127.0.0.1:%d\n", port);
    fflush(stdout);

    do {
        int sock = tcp_accept(listener);
        if (sock < 0) continue;
        printf("ecu_sim: simulator connected, ECU booted\n");
        fflush(stdout);
        serve(sock);
        tcp_close(sock);
        printf("ecu_sim: simulator left\n");
        fflush(stdout);
    } while (!once);

    tcp_close(listener);
    return 0;
}
