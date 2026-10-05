/* Every number the ECU firmware uses: the car, the sensors, the limits and the
   derating thresholds. Values marked "bench" suit the current bench setup with
   the STM32 motor controllers on a lab power supply. */
#ifndef CONFIG_H
#define CONFIG_H

/* ---- wheels ---- */

enum { WHEEL_FL, WHEEL_FR, WHEEL_RL, WHEEL_RR };

/* ---- timing ---- */

#define CONTROL_PERIOD_MS 10   // 100 Hz, same as the simulator
#define MOTOR_TIMEOUT_MS  50   // controllers send a heartbeat every 10 ms
#define MOTOR_START_MS    500  // time a controller gets to reach torque mode
#define MOTOR_RETRY_MS    100  // gap between repeated state requests
#define HIL_TIMEOUT_MS    50   // the simulator sends inputs every 10 ms
#define GYRO_CAL_MS       1000 // car must stay still while the gyro bias is measured
#define ARM_BRAKE_HOLD_MS 1000 // hold the brake this long to enable torque
#define APPS_DISAGREE_MS  100  // FS rules allow 100 ms of pedal sensor disagreement
#define CONSOLE_STREAM_MS 100

/* ---- vehicle ---- */

#define WHEELBASE_M          1.55f
#define WHEEL_RADIUS_M       0.254f
#define GEAR_RATIO           15.47f
#define STEERING_TO_WHEEL    0.23f   // road wheel angle per steering angle, the sim's ACK_NOMINAL
#define MOTOR_MAX_TORQUE_NM  29.4f   // at the motor shaft
#define MOTOR_MAX_SPEED_RADS 1047.2f // bench: 10k rpm, the controller's SPEED_MAX

/* ---- driver requests ---- */

#define DRIVE_REQUEST_MAX_NM (4.0f * MOTOR_MAX_TORQUE_NM) // full throttle
#define REGEN_REQUEST_MAX_NM (4.0f * MOTOR_MAX_TORQUE_NM) // full brake pedal

/* ---- torque vectoring ---- */

#define TV_KP_NM_PER_RADS    4.0f // extra torque per side for each rad/s of yaw error
#define TV_MAX_CORRECTION_NM 4.0f
#define TV_MIN_SPEED_MS      0.5f // below this the yaw target is meaningless

/* ---- derating ---- */

#define DERATE_FET_START_C  80.0f // controllers trip at 100 C
#define DERATE_FET_END_C    95.0f
#define DERATE_ECU_START_C  70.0f
#define DERATE_ECU_END_C    85.0f
#define DERATE_SPEED_START  0.70f // of max speed. A wide ramp keeps a free bench motor steady
#define DERATE_LOW_V_START  14.0f // bench: 24 V supply, controllers trip at 8 V
#define DERATE_LOW_V_END    10.0f
#define DERATE_HIGH_V_START 50.0f // bench: a lab supply cannot absorb regen, so the bus rises
#define DERATE_HIGH_V_END   56.0f // controllers trip at 60 V
#define REGEN_FULL_SPEED_MS 2.0f  // regen fades out below this, or it would drive backwards

/* ---- pedals and steering ---- */

// Sensor volts at the connector. The board divides them by 47k / (33k + 47k).
#define SENSOR_DIVIDER   (47.0f / 80.0f)
#define SENSOR_MIN_V     0.25f // below this the wire is open or shorted to ground
#define SENSOR_MAX_V     4.75f // above this it is shorted to the supply
#define APPS1_ZERO_V     0.5f
#define APPS1_FULL_V     4.5f
#define APPS2_ZERO_V     4.5f // second sensor runs the other way, so one fault cannot fool both
#define APPS2_FULL_V     0.5f
#define BPPS_ZERO_V      0.5f
#define BPPS_FULL_V      4.5f
#define STEER_CENTRE_V   2.5f
#define STEER_LEFT_V     4.5f // at full left lock
#define STEER_LOCK_RAD   2.4f // steering angle at full lock, the sim's g_MAX_STEER_RAD
#define PEDAL_DEADBAND   0.03f
#define APPS_DISAGREE    0.10f
#define BRAKE_ON         0.10f
#define BRAKE_APPS_CUT   0.25f // throttle above this with the brake on cuts torque
#define BRAKE_APPS_RESET 0.05f // until the throttle comes back below this

/* ---- board ---- */

#define ADC_VREF_V    3.3f
#define VIN_DIVIDER   21.0f // 200k over 10k
#define VIN_MIN_V     10.0f // below this the board runs from USB only
#define NTC_R_TOP_OHM 39000.0f
#define NTC_R25_OHM   10000.0f
#define NTC_BETA      3380.0f
#define IMU_YAW_SIGN  1.0f // flip if the board is mounted upside down

/* ---- driverless control ---- */

#define AUTO_CRUISE_SPEED_MS   6.0f  // speed on a straight, the sim's g_CRUISE_SPEED_MS
#define AUTO_LOOKAHEAD_M       3.0f  // how far ahead on the path the car aims
#define AUTO_SPEED_KP_NM       20.0f // total torque per m/s of speed error
#define AUTO_MAX_STEERING      2.4f  // steering units, the sim's g_MAX_STEER_RAD
#define AUTO_MAX_STEERING_RATE 8.0f  // steering units per second
#define AUTO_MIN_TARGET_M      0.2f  // a target closer than this is ignored
#define PLAN_MAX_GATE_M        10.0f // a blue and yellow cone further apart are not a gate
#define PLAN_EDGE_OFFSET_M     2.0f  // how far inside one edge to drive when only that edge is seen
#define PLAN_MIN_SEGMENT_M     0.2f
#define PLAN_MIN_POINT_GAP_M   0.3f
#define CONES_TIMEOUT_MS       50 // no cone frame for this long means perception is lost

/* ---- HIL ---- */

#define HIL_ALLOWED 1 // set to 0 for the car so nothing on the bus can replace the sensors

#endif
