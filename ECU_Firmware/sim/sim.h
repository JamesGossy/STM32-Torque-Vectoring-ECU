/* Host build of the ECU board: implements hal.h in memory so the real app/ code
   runs on a PC. Used by the tests and by ecu_sim, which joins the Formula
   Student simulator's virtual CAN bus. */
#ifndef SIM_H
#define SIM_H

#include <stddef.h>
#include <stdint.h>

void sim_reset(void);         // fresh board at time 0, application started
void sim_run_ms(uint32_t ms); // main loop and 1 ms tick, once per millisecond
uint32_t sim_millis(void);

/* ---- sensors ---- */

void sim_set_adc_volts(int channel, float volts_at_pin);
void sim_set_pedals(
    float apps1_v, float apps2_v, float bpps_v, float steering_v); // connector volts
void sim_set_gyro_z(float rad_s);
void sim_set_imu_present(int present);
void sim_set_can_ok(int ok);

/* ---- links ---- */

void sim_can_inject(uint32_t id, const uint8_t *data, uint8_t len);
int sim_can_pop(uint32_t *id, uint8_t *data, uint8_t *len);
void sim_gps_inject(const char *text);
void sim_serial_inject(const char *text);
size_t sim_serial_take(char *out, size_t max);

#endif
