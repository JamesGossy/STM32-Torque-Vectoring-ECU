/* In-memory ECU board. Time only moves when sim_run_ms() is called, so tests
   and the lock-step HIL link are exactly repeatable. The IMU is a small model
   of the LSM6DSV16X register map, so the real driver code is exercised. */
#include "sim.h"
#include "app.h"
#include "config.h"
#include "hal.h"
#include <math.h>
#include <string.h>

#define CAN_DEPTH  256
#define BYTE_DEPTH 8192

typedef struct {
    uint32_t id;
    uint8_t data[8], len;
} Frame;

typedef struct {
    Frame frames[CAN_DEPTH];
    unsigned head, tail;
} FrameQueue;

typedef struct {
    uint8_t bytes[BYTE_DEPTH];
    size_t head, tail;
} ByteQueue;

static uint32_t now_ms;
static float adc[ADC_COUNT];
static uint8_t imu_regs[128];
static int imu_present, can_ok;
static FrameQueue can_rx, can_tx;
static ByteQueue gps_rx, serial_rx, serial_tx;

/* ---- queues ---- */

static int frame_push(FrameQueue *q, uint32_t id, const uint8_t *data, uint8_t len)
{
    unsigned next = (q->head + 1) % CAN_DEPTH;
    if (next == q->tail) return 0;
    Frame *f = &q->frames[q->head];
    f->id    = id;
    f->len   = len > 8 ? 8 : len;
    memset(f->data, 0, 8);
    if (data) memcpy(f->data, data, f->len);
    q->head = next;
    return 1;
}

static int frame_pop(FrameQueue *q, uint32_t *id, uint8_t *data, uint8_t *len)
{
    if (q->tail == q->head) return 0;
    Frame *f = &q->frames[q->tail];
    *id      = f->id;
    *len     = f->len;
    memcpy(data, f->data, 8);
    q->tail = (q->tail + 1) % CAN_DEPTH;
    return 1;
}

static void bytes_push(ByteQueue *q, const void *data, size_t count)
{
    const uint8_t *p = data;
    for (size_t i = 0; i < count; i++) {
        size_t next = (q->head + 1) % BYTE_DEPTH;
        if (next == q->tail) return;
        q->bytes[q->head] = p[i];
        q->head           = next;
    }
}

static size_t bytes_pop(ByteQueue *q, uint8_t *out, size_t max)
{
    size_t count = 0;
    while (count < max && q->tail != q->head) {
        out[count++] = q->bytes[q->tail];
        q->tail      = (q->tail + 1) % BYTE_DEPTH;
    }
    return count;
}

/* ---- hal.h ---- */

void hal_init(void) { }
void hal_start(void) { }
uint32_t hal_millis(void)
{
    return now_ms;
}
float hal_adc_volts(int channel)
{
    return adc[channel];
}

void hal_imu_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    uint8_t reg = tx[0] & 0x7F;
    int reading = (tx[0] & 0x80) != 0;
    rx[0]       = 0xFF;
    for (size_t i = 1; i < len; i++, reg = (uint8_t)((reg + 1) & 0x7F)) {
        if (!imu_present) {
            rx[i] = 0xFF;
        } else if (reading) {
            rx[i] = imu_regs[reg];
        } else {
            imu_regs[reg] = tx[i] & (reg == 0x12 ? 0xFE : 0xFF); // software reset finishes at once
            rx[i]         = 0xFF;
        }
    }
}

int hal_can_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    return frame_push(&can_tx, id, data, len);
}

int hal_can_recv(uint32_t *id, uint8_t *data, uint8_t *len)
{
    return frame_pop(&can_rx, id, data, len);
}

int hal_can_ok(void)
{
    return can_ok;
}

size_t hal_gps_read(uint8_t *data, size_t max)
{
    return bytes_pop(&gps_rx, data, max);
}

int hal_serial_write(const void *data, size_t len)
{
    bytes_push(&serial_tx, data, len);
    return 1;
}

size_t hal_serial_read(uint8_t *data, size_t max)
{
    return bytes_pop(&serial_rx, data, max);
}

void hal_led_set(int on)
{
    (void)on;
}
void hal_wdg_kick(void) { }

/* ---- harness ---- */

void sim_set_adc_volts(int channel, float volts_at_pin)
{
    adc[channel] = volts_at_pin;
}

void sim_set_pedals(float apps1_v, float apps2_v, float bpps_v, float steering_v)
{
    adc[ADC_APPS1]    = apps1_v * SENSOR_DIVIDER;
    adc[ADC_APPS2]    = apps2_v * SENSOR_DIVIDER;
    adc[ADC_BPPS]     = bpps_v * SENSOR_DIVIDER;
    adc[ADC_STEERING] = steering_v * SENSOR_DIVIDER;
}

void sim_set_gyro_z(float rad_s)
{
    long counts = lroundf(rad_s / (0.0175f * 0.017453293f) / IMU_YAW_SIGN);
    if (counts > 32767) counts = 32767;
    if (counts < -32768) counts = -32768;
    uint16_t raw   = (uint16_t)(int16_t)counts;
    imu_regs[0x26] = (uint8_t)raw; // OUTZ_L_G
    imu_regs[0x27] = (uint8_t)(raw >> 8);
}

void sim_set_imu_present(int present)
{
    imu_present = present;
}
void sim_set_can_ok(int ok)
{
    can_ok = ok;
}

void sim_can_inject(uint32_t id, const uint8_t *data, uint8_t len)
{
    frame_push(&can_rx, id, data, len);
}

int sim_can_pop(uint32_t *id, uint8_t *data, uint8_t *len)
{
    return frame_pop(&can_tx, id, data, len);
}

void sim_gps_inject(const char *text)
{
    bytes_push(&gps_rx, text, strlen(text));
}
void sim_serial_inject(const char *text)
{
    bytes_push(&serial_rx, text, strlen(text));
}

size_t sim_serial_take(char *out, size_t max)
{
    size_t count = bytes_pop(&serial_tx, (uint8_t *)out, max ? max - 1 : 0);
    if (max) out[count] = 0;
    return count;
}

uint32_t sim_millis(void)
{
    return now_ms;
}

// A healthy, stationary board on a 24 V supply with the pedals released.
void sim_reset(void)
{
    now_ms = 0;
    memset(&can_rx, 0, sizeof can_rx);
    memset(&can_tx, 0, sizeof can_tx);
    memset(&gps_rx, 0, sizeof gps_rx);
    memset(&serial_rx, 0, sizeof serial_rx);
    memset(&serial_tx, 0, sizeof serial_tx);
    memset(imu_regs, 0, sizeof imu_regs);
    imu_regs[0x0F] = 0x70; // WHO_AM_I
    imu_present    = 1;
    can_ok         = 1;

    sim_set_pedals(APPS1_ZERO_V, APPS2_ZERO_V, BPPS_ZERO_V, STEER_CENTRE_V);
    adc[ADC_VIN]  = 24.0f / VIN_DIVIDER;
    adc[ADC_TEMP] = ADC_VREF_V * NTC_R25_OHM / (NTC_R_TOP_OHM + NTC_R25_OHM); // 25 C

    hal_init();
    app_init();
    hal_start();
}

void sim_run_ms(uint32_t ms)
{
    while (ms--) {
        app_poll();
        now_ms++;
        app_tick();
    }
}
