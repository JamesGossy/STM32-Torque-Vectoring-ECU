# Code and comment style

## Comments
- No AI-sounding language. Plain, direct English only.
- No em dashes anywhere (not in comments, not in strings, not in docs).
- Default: no comments. Only add one when the WHY is non-obvious: a hidden constraint, a subtle invariant, a workaround for a known bug.
- Strongly prefer inline comments (`// ...` at end of line) over standalone lines.
- File top: a short block comment is allowed (2-5 lines max). Describe what the file does and any key design constraint. Not a list of functions.
- Function top: one short line is allowed if the function is non-obvious. No multi-line docstrings.
- Logic blocks: one short standalone comment above a new logical stage is allowed (e.g. `// feedforward from curvature`). Keep these to one line.
- Never describe WHAT the code does. Only WHY it does something non-obvious.

## Section dividers
- Split long files into named sections with: `/* ---- section name ---- */`
- Use these in headers too (e.g. `/* ---- steering ---- */`, `/* ---- torque vectoring ---- */`).
- In long functions, number the main steps: `// 1. project onto line`, `// 2. Stanley feedback`, etc.

## C formatting
- clang-format, WebKit-based. Run `make format` before committing.
- Allman braces on functions, attached on control flow.
- 4-space indent, 100-col line limit.
- Do not hand-align operands inside expressions — clang-format will normalise it.

---

# Working in this repo

Firmware for the torque vectoring and derating ECU (STM32G474CEU6) described in
`ECU_Hardware/`. It commands four STM32 motor controllers over CAN. The old
full-physics HIL simulation that used to live here was removed; HIL testing now uses
the Formula Student simulator repo, which talks to this ECU over CAN.

## Layout and the one rule that matters

- `ECU_Firmware/app/` is the whole application and must never touch hardware. It only
  includes `hal/hal.h`. The same code runs on the board (`hal/stm32g474/`) and on a PC
  (`sim/sim.c`), which is what makes the tests and HIL runs meaningful.
- Every number lives in `app/config.h`. Every CAN id and payload lives in
  `app/can_protocol.h`.
- `app/can_protocol.h` is copied byte for byte into the simulator repo as
  `shared/can_protocol.h`. Change both together.
- The motor controller protocol (ids, payloads, states) is defined by the
  STM32-Motor-Controller firmware (`controller_firmware/app/can_proto.h`). Match it, do
  not change it from this side.
- `hal/stm32g474/usb_cdc.c`, `usb_desc.c`, `usb_ll.h`, `syscalls.c`, the CMSIS headers
  and the startup file are copied from the motor controller repo and keep its style.
  They are excluded from `make format`.

## Checks after any change

```
make test         # host build, unit and whole-application tests
make firmware     # must build with no warnings
make format-check
```

With the motor controller repo next to this one, also run
`make test MC_DIR=../STM32-Motor-Controller/controller_firmware`. It builds the real
controller firmware for the PC and runs it against this ECU through `ecu_sim`.

After changing torque vectoring, derating, the state machine or the protocol, run the
simulator's lap matrix through this firmware: build `ecu_sim` (`make sim`), then in the
Formula-Student-Autonomous-Sim repo run `make test-hil`. It must keep 0 off-track ticks
and at least two laps on both tracks.

## Key facts

- Wheel order is FL, FR, RL, RR everywhere. Motor controller nodes are 1 to 4 in that
  order. The ECU uses node 16.
- Torque inside the ECU is car motor-shaft Nm. It becomes controller current with
  `MOTOR_NM_PER_AMP` (29.4 Nm over 20 A), and the left motors get the opposite sign.
- The control step runs every 10 ms from `app_tick()`. In HIL the simulator sends
  `CONTROL` last each tick, and `hil.c` only takes inputs as a full set when it arrives.
- Torque needs `ECU_DRIVE` and no inhibit bits. Soft inhibits (pedal disagreement,
  brake with throttle) zero torque while active. Every other inhibit while driving latches `ECU_FAULT`, cleared by the console `clear` command.
- Arming needs a fresh request: brake held for 1 s with the throttle released, or an
  off to on edge of the simulator's drive flag.
- Panel dials (`panel.c`, node 17) scale the yaw correction and the drive and regen limits.
  Default 100 %, last value held if the panel goes quiet.
- `HIL_ALLOWED` in `config.h` must be 0 on the car.
- `sim/sim.c` only moves time in `sim_run_ms()`, so host tests are exact and repeatable.
  The fake IMU is a register map, so the real `imu.c` driver runs in the tests.
- The STM32 drivers could not be tried on the board from here. Anything that touches
  registers needs a bench check after changes.
