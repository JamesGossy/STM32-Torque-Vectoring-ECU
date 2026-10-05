/* Every CAN message on the car's bus: the motor controller protocol, the ECU's
   own frames and the HIL frames the simulator sends. Classic CAN, 11-bit ids.
   The Formula Student sim keeps an identical copy in shared/can_protocol.h,
   so change both files together. */
#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>
#include <string.h>

#define CAN_BITRATE 1000000

/* ---- id layout ---- */

// id = (node << 5) | message, the scheme the motor controllers use.
#define CAN_ID(node, msg) ((((uint32_t)(node)) << 5) | (uint32_t)(msg))
#define CAN_NODE(id)      ((id) >> 5)
#define CAN_MSG(id)       ((id) & 0x1Fu)
#define CAN_ESTOP_ID      0x000 // every motor controller goes idle

/* ---- motor controllers ---- */

enum {
    MC_CMD_SET_STATE    = 0x01, // u8: 0 idle, 1 torque
    MC_CMD_SET_IQ       = 0x02, // f32 q-axis current, A
    MC_CMD_CLEAR_FAULTS = 0x04,
    MC_MSG_HEARTBEAT    = 0x10, // u8 state, u8 mode, u16 faults, u16 driver status x2
    MC_MSG_IQ_SPEED     = 0x11, // f32 iq A, f32 motor speed rad/s
    MC_MSG_BUS_TEMP     = 0x13, // f32 bus volts, i16 FET degC*10, i16 board degC*10
};

enum { MC_RUN_IDLE = 0, MC_RUN_TORQUE = 1 };
enum { MC_STATE_BOOT, MC_STATE_IDLE, MC_STATE_CAL, MC_STATE_RUN, MC_STATE_FAULT };
#define MC_MODE_TORQUE 1

/* ---- car wiring ---- */

// Wheel order is FL, FR, RL, RR everywhere.
static const uint8_t MOTOR_NODE[4]    = { 1, 2, 3, 4 };
static const float MOTOR_DIRECTION[4] = { -1.0f, 1.0f, -1.0f, 1.0f }; // left motors are mirrored

// Car motor torque per amp of controller current: 29.4 Nm peak over the 20 A limit.
#define MOTOR_NM_PER_AMP 1.47f

/* ---- ECU ---- */

// Node 16 is free in the controller id scheme, so ECU frames never clash.
#define ECU_NODE       16
#define ECU_MSG_STATUS 0x10 // u8 state, flags, u16 inhibit, u8 drive %, regen %, derate, count
#define ECU_MSG_YAW    0x11 // f32 target yaw rate, f32 measured yaw rate, rad/s
#define ECU_MSG_COMMAND                                                                            \
    0x12 // driverless only: f32 steering (steering units), f32 total torque request Nm

#define ECU_FLAG_HIL 0x01 // bits 4..7 are the online motors, FL first

/* ---- steering wheel panel ---- */

#define PANEL_NODE      17
#define PANEL_MSG_DIALS 0x01 // u8 torque vectoring %, drive power %, regen %, u8 count

/* ---- HIL, simulator to ECU ---- */

enum {
    HIL_MSG_DRIVER   = 0x01, // f32 steering rad, f32 total torque request Nm
    HIL_MSG_WHEELS_F = 0x02, // f32 FL, f32 FR wheel speed rad/s
    HIL_MSG_WHEELS_R = 0x03, // f32 RL, f32 RR wheel speed rad/s
    HIL_MSG_CONTROL  = 0x04, // u8 flags, u8 count, u16 spare, f32 yaw rate rad/s
    HIL_MSG_CONE = 0x05, // u16 range cm, i16 bearing mrad, u8 colour (0 blue, 1 yellow), u8 index,
                         // u8 cones in scan, u8 scan number (the CONTROL count of the same tick)
};

// CONTROL is sent last each tick, so it marks a complete set of inputs.
#define HIL_FLAG_ON       0x01 // use these inputs instead of the ECU's own sensors
#define HIL_FLAG_DRIVE    0x02 // the simulated driver wants torque
#define HIL_FLAG_AUTONOMY 0x04 // the ECU steers and sets the torque request from the CONE frames

#define HIL_MAX_CONES                                                                              \
    16 // nearest cones, one CAN frame each; an empty scan is one frame with count 0

/* ---- TCP link between the simulator and the bus ---- */

// The simulator connects to either ecu_sim (host build of this firmware) or
// can_bridge.py (a real bus through a USB-CAN adapter). Both speak this.
#define LINK_PORT 5700

enum {
    LINK_FRAME     = 1, // one CAN frame, either direction
    LINK_STEP      = 2, // simulator to bus: id holds the milliseconds to run
    LINK_STEP_DONE = 3, // bus to simulator: every frame for that step has been sent
    LINK_HELLO     = 4, // bus to simulator on connect: data[0] is 1 for a real-time bus
};

typedef struct {
    uint8_t kind;
    uint8_t len;
    uint8_t spare[2];
    uint32_t id; // little endian on the wire, as on every supported PC
    uint8_t data[8];
} LinkRecord;

/* ---- payload helpers ---- */

static inline float can_get_f32(const uint8_t *data)
{
    float value;
    memcpy(&value, data, 4);
    return value;
}

static inline void can_put_f32(uint8_t *data, float value)
{
    memcpy(data, &value, 4);
}

static inline int16_t can_get_i16(const uint8_t *data)
{
    int16_t value;
    memcpy(&value, data, 2);
    return value;
}

static inline void can_put_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static inline void can_put_i16(uint8_t *data, int16_t value)
{
    memcpy(data, &value, 2);
}

static inline uint16_t can_get_u16(const uint8_t *data)
{
    return (uint16_t)(data[0] | (data[1] << 8));
}

#endif
