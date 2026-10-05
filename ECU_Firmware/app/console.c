/* Text console on the USB port for bench checks. Type "help" for the list:
   status, stream on|off, motors, gps, off, clear, estop. */
#include "console.h"
#include "app.h"
#include "can_protocol.h"
#include "config.h"
#include "gps.h"
#include "hal.h"
#include "motors.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define LINE_MAX 64

static char line[LINE_MAX];
static int length, streaming;
static uint32_t stream_ms;

static const char *STATE_NAMES[]   = { "STARTUP", "STANDBY", "DRIVE", "FAULT" };
static const char *INHIBIT_NAMES[] = { "startup", "motor-offline", "motor-fault", "no-main-power",
    "pedal-sensor", "pedal-disagree", "brake+throttle", "hil-lost", "imu", "can", "cones-lost" };

static void say(const char *format, ...)
{
    char text[256];
    va_list args;
    va_start(args, format);
    int n = vsnprintf(text, sizeof text - 2, format, args);
    va_end(args);
    if (n < 0) return;
    if (n > (int)sizeof text - 3) n = (int)sizeof text - 3;
    text[n++] = '\r';
    text[n++] = '\n';
    hal_serial_write(text, (size_t)n);
}

static void inhibit_text(uint16_t bits, char *out, size_t size)
{
    size_t used = 0;
    out[0]      = 0;
    for (unsigned bit = 0; bit < sizeof INHIBIT_NAMES / sizeof INHIBIT_NAMES[0]; bit++) {
        if (!(bits & (1u << bit)) || used >= size) continue;
        int n = snprintf(out + used, size - used, "%s%s", used ? "," : "", INHIBIT_NAMES[bit]);
        if (n > 0) used += (size_t)n;
    }
    if (!used) snprintf(out, size, "none");
}

static void print_status(void)
{
    const EcuStatus *s = app_status();
    char inhibits[128];
    inhibit_text(s->inhibit, inhibits, sizeof inhibits);
    say("%s%s v=%.2f steer=%.2f req=%.1f yaw=%.3f/%.3f T=%.1f,%.1f,%.1f,%.1f "
        "derate=%.0f%%/%.0f%% panel=%.0f/%.0f/%.0f%% vin=%.1f ecu=%.1fC inhibit=%s",
        STATE_NAMES[s->state], s->hil ? " HIL" : "", (double)s->speed_ms, (double)s->steering_rad,
        (double)s->request_nm, (double)s->yaw_rate, (double)s->target_yaw_rate,
        (double)s->torque_nm[0], (double)s->torque_nm[1], (double)s->torque_nm[2],
        (double)s->torque_nm[3], (double)(100.0f * s->derate.drive),
        (double)(100.0f * s->derate.regen), (double)(100.0f * s->panel.tv),
        (double)(100.0f * s->panel.power), (double)(100.0f * s->panel.regen), (double)s->vin_v,
        (double)s->ecu_temp_c, inhibits);
}

static void print_motors(void)
{
    static const char *NAMES[] = { "FL", "FR", "RL", "RR" };
    uint32_t now               = hal_millis();
    for (int wheel = 0; wheel < 4; wheel++) {
        const Motor *m = motor_get(wheel);
        say("%s node %d %s state=%d faults=0x%04x iq=%.2fA speed=%.0frad/s bus=%.1fV fet=%.1fC",
            NAMES[wheel], MOTOR_NODE[wheel], motor_online(wheel, now) ? "online " : "offline",
            m->state, m->faults, (double)m->iq_a, (double)m->speed_rads, (double)m->bus_v,
            (double)m->fet_temp_c);
    }
}

static void print_gps(void)
{
    const GpsFix *f = gps_fix();
    say("gps %s lat=%.6f lon=%.6f speed=%.2fm/s course=%.1f sentences=%lu",
        f->valid ? "fix" : "no-fix", f->lat_deg, f->lon_deg, (double)f->speed_ms,
        (double)f->course_deg, (unsigned long)f->sentences);
}

static void run(const char *command)
{
    if (!strcmp(command, "status")) {
        print_status();
    } else if (!strcmp(command, "stream on")) {
        streaming = 1;
    } else if (!strcmp(command, "stream off")) {
        streaming = 0;
    } else if (!strcmp(command, "motors")) {
        print_motors();
    } else if (!strcmp(command, "gps")) {
        print_gps();
    } else if (!strcmp(command, "off")) {
        app_disarm();
        say("torque off");
    } else if (!strcmp(command, "clear")) {
        motors_clear_faults();
        app_clear_fault();
        say("cleared the ECU fault and asked the controllers to clear theirs");
    } else if (!strcmp(command, "estop")) {
        app_disarm();
        motors_estop();
        say("e-stop sent");
    } else {
        say("commands: status | stream on|off | motors | gps | off | clear | estop");
    }
}

void console_init(void)
{
    length    = 0;
    streaming = 0;
    stream_ms = 0;
}

void console_poll(void)
{
    uint8_t bytes[32];
    size_t count;
    while ((count = hal_serial_read(bytes, sizeof bytes)) > 0) {
        for (size_t i = 0; i < count; i++) {
            char c = (char)bytes[i];
            if (c == '\r' || c == '\n') {
                line[length] = 0;
                if (length) run(line);
                length = 0;
            } else if (length < LINE_MAX - 1) {
                line[length++] = c;
            }
        }
    }
}

void console_tick(void)
{
    uint32_t now = hal_millis();
    if (streaming && now - stream_ms >= CONSOLE_STREAM_MS) {
        stream_ms = now;
        print_status();
    }
}
