/* Simple yaw-rate torque vectoring. Shares the driver's torque request equally,
   then moves torque from one side to the other so the car rotates at the rate
   the steering asks for. */
#ifndef TORQUE_VECTORING_H
#define TORQUE_VECTORING_H

typedef struct {
    float request_nm; // total torque the driver wants, negative is regen
    float speed_ms;
    float steering_rad; // steering angle, positive left
    float yaw_rate;     // measured, rad/s, positive left
} TvInputs;

typedef struct {
    float drive_nm; // most drive torque one motor may give, >= 0
    float regen_nm; // most regen torque one motor may give, >= 0
} TorqueLimits;

float tv_target_yaw_rate(float speed_ms, float steering_rad);

// Wheel torques in FL, FR, RL, RR order, Nm at the motor shaft.
void torque_vectoring(const TvInputs *in, const TorqueLimits *limits, float torque_nm[4]);

#endif
