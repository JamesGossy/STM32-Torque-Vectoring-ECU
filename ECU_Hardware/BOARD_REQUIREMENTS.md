# STM32 Torque Vectoring ECU — Board Requirements

Status: Design draft | Updated: 2026-09-13

This STM32 board runs torque vectoring and derating algorithms. Four brushless motor controllers and a BMS communicate with it over CAN. The board commands torque; it does not directly drive motors. This document records requirements, not verification of the existing schematic.

**Confirmed** items reflect agreed scope. **Proposed** items are recommendations. **TBD** items must be resolved during design.

## 1. Confirmed scope

| Block | Requirement |
| --- | --- |
| Main input | 12–60 V DC through a keyed XT60 connector. |
| Buck regulator | Convert the main input to 5 V. |
| USB-C | Supply board power for testing and provide a diagnostic/debug interface. |
| Power ORing | Combine buck and USB 5 V through ideal-diode circuitry, preventing backfeeding into either source. |
| Logic supply | Generate 3.3 V from combined 5 V using an LDO. |
| External sensors | Must not receive power from USB-C. |
| MCU | STM32 with sufficient computation, memory and peripherals, including CAN FD capability. Part TBD. |
| IMU | Onboard inertial sensing for the control algorithm. |
| GPS | GPS/GNSS data; receiver and antenna arrangement TBD. |
| Driver inputs | Steering angle, accelerator pedal position and brake pedal position. Sensors/interfaces selected during board design. |
| CAN | One CAN FD-capable bus with two connectors for daisy chaining. |
| Temperature | Ambient temperature sensing. |
| Indicators | Indicator LEDs; functions/count TBD. |
| Programming | SWD header for an external ST-LINK. |
| Protection/monitoring | TVS input protection and input rail voltage monitoring. |
| Environment | Approximately 30 °C ambient; full operating limits TBD. |

## 2. CAN interfaces

### Four motor controllers

Receive from each controller:

- Motor/wheel speed information, with units and motor-to-wheel conversion defined.
- Motor and inverter temperatures.
- Echoed torque and speed setpoints.
- D-axis and q-axis currents (`Id` and `Iq`).

Send individual torque commands over CAN. An echoed torque setpoint is command feedback, not an independent measurement of actual torque.

Proposed: receive readiness, status and faults where supported by the controller protocol.

### BMS

Receive all BMS information needed for derating, including state of charge and temperatures. Finalise the complete signal list during protocol design, including applicable pack voltage/current, charge/discharge limits and fault/status messages. No dedicated motor/battery temperature inputs are currently required on this board.

### Hardware and protocol

- Use an appropriate STM32 FDCAN peripheral and CAN FD-rated transceiver.
- Wire both connectors to the same CAN_H/CAN_L network; these are not independent channels.
- Proposed: selectable termination, enabled only when the ECU is at a physical bus end.
- Define connector pinout, reference ground, shielding, protection and any isolation requirement.
- Define arbitration/data rates, identifiers, payloads, units, scaling, update rates, timeouts and bus-load budget.
- Confirm that all attached devices support the intended network mode. ECU CAN FD capability alone does not establish whole-network FD compatibility.

## 3. Power architecture

```text
XT60 12–60 V -> protection/filtering -> 5 V buck -> ideal diode --+
                                             |                |
                                             +-> external     +-> board 5 V
                                                 sensor rail  |      |
USB-C 5 V ------------------------------------> ideal diode --+      +-> LDO -> 3.3 V
```

The external sensor supply branch shall be fed before ORing, from main-input power only. Provide any additional regulation required by the selected sensors. External sensors shall not be connected directly to the combined board rail.

- Prevent reverse current into USB and the buck output.
- Prevent phantom powering of external sensors through signal pins during USB-only operation; check powered/unpowered interfaces in both directions.
- Define main-only, USB-only and simultaneous-source operation, including source insertion/removal.
- Proposed: inhibit torque commands in USB-only test mode and identify unavailable external sensors in diagnostics.
- Calculate board and external-sensor current budgets before selecting regulators.
- Implement USB-C sink CC configuration and respect the source's advertised current capability. USB PD is not a confirmed requirement.
- Check LDO heat dissipation: approximately `(5 V - 3.3 V) × load current`, plus quiescent losses.

### Protection decisions

| Item | Requirement/status |
| --- | --- |
| TVS | Confirmed. Select standoff, clamping voltage and pulse capability against the actual supply envelope and protected component ratings. |
| Input monitoring | Confirmed. Use a scaled, filtered and protected ADC input; account for an unpowered MCU. Monitoring does not replace hardware protection. |
| OCP | Proposed: a board-input fuse or documented upstream fused feed sized for ECU wiring/input circuitry, plus buck current-limit/short-circuit protection. A separate electronic OCP IC is not automatically required. |
| Active OVP | TBD; not mandatory at this stage. Decide from supply fault/transient limits and component ratings. Do not assume a TVS can absorb sustained overvoltage indefinitely. |
| Reverse polarity | Dedicated protection omitted from current scope, assuming a correctly wired keyed XT60 harness. Keying prevents reversed mating but not reversed harness wiring; verify polarity during assembly. |
| External sensor faults | Proposed: current limiting/fault containment appropriate to the selected sensor loads. |

Clarify whether 60 V includes the maximum normal charging/regeneration voltage. Check operating and absolute maximum ratings against the input/transient envelope and TVS clamp voltage; do not select components solely from a nominal “60 V” rating.

## 4. Sensor requirements

### Steering and pedals — TBD during board design

For each sensor, determine:

- Supply voltage/current and connector pinout.
- Interface type: analogue, PWM, digital bus or other.
- Signal range, accuracy, bandwidth and sample rate.
- Channel count, including redundant accelerator channels if the chosen pedal uses them.
- Filtering, electrical protection and open/short-circuit detection.
- Calibration procedure and persistent storage.

Do not freeze connector pin counts or ADC allocation before selecting these interfaces.

### IMU, GPS and ambient temperature

- Select IMU axes, ranges, sample rates, interface and interrupts to suit the algorithm; document orientation and calibration.
- Define GPS update rate, interface, antenna placement/connector, any active-antenna supply and whether PPS timing is needed.
- Define invalid/stale sensor handling and required accuracy.
- Place the ambient sensor away from regulator heating where practical. Decide whether the measurement represents enclosure air or PCB temperature.

## 5. MCU, programming and diagnostics

- Finalise STM32 computation, memory and peripheral/pin allocation against algorithm timing and interfaces.
- Implement device-specific decoupling, analogue supply/reference, clock, reset and boot requirements.
- SWD header: SWDIO, SWCLK, NRST, ground and target-voltage reference; SWO optional.
- Define USB diagnostics and any firmware-update support. External ST-LINK uses the SWD header; onboard ST-LINK is not currently required. Native USB diagnostics alone do not provide breakpoint debugging.
- Proposed LEDs: power, firmware heartbeat, CAN activity and fault.
- Proposed test points: input, buck 5 V, USB 5 V, combined 5 V, 3.3 V, ground and reset.
- Provide persistent calibration/configuration storage. External memory or bulk logging is optional unless internal resources are insufficient.

## 6. Control and fault behaviour — proposed

- Define control-loop period, maximum latency and input freshness limits.
- Inhibit torque at startup until required inputs and controller/BMS status are valid.
- Define responses to implausible driver inputs, stale feedback, BMS/controller faults and CAN bus-off.
- Use watchdog and brownout/reset handling with defined recovery behaviour.
- Require receiving motor controllers to time out stale torque commands. Sending a final zero-torque command cannot be relied on after ECU power or communication loss.
- Define fault-specific torque responses, timeouts and recovery conditions; values TBD.
- Decide whether the vehicle architecture needs an independent hardware torque-inhibit/interlock interface.
- Report diagnostic faults and retain useful fault information where practical.

## 7. Thermal and mechanical requirements

The intended ambient condition is approximately 30 °C. Verify buck/LDO self-heating at maximum load and across the input range; component junction temperature can be substantially above ambient.

Board dimensions, mounting, enclosure, connector access, harness strain relief and environmental exposure limits remain TBD. Preserve practical SWD and test-point access.

## 8. Verification checklist

- Verify regulated rails across 12–60 V input and the agreed load range.
- Verify both-source, USB-only and main-only operation, source transitions and reverse-current blocking.
- Verify external sensors remain unpowered on USB-only supply, including signal-pin backfeeding checks.
- Measure regulator temperature rise at approximately 30 °C ambient under worst-case load.
- Check input-voltage measurement accuracy and protection ratings against the agreed input envelope.
- Verify SWD programming/debugging and USB diagnostics.
- Exercise the intended CAN mode with all four controllers and the BMS; check termination, timing and bus load.
- Validate selected driver sensors, calibration and fault detection.
- Validate IMU/GPS data, orientation, timing and temperature sensing.
- Verify startup inhibition, watchdog recovery, stale-data handling and receiving-controller command timeouts.

## 9. Remaining decisions

1. STM32 part, pin allocation and timing/resource budgets.
2. Pedal/steering sensor selection and interfaces.
3. Supply/transient envelope, fuse arrangement and need for active OVP.
4. Regulators, current budgets and external sensor supply implementation.
5. CAN protocol, rates, connectors, protection and isolation.
6. IMU/GPS parts and measurement requirements.
7. Control/fault timing, derating thresholds and hardware interlock decision.
8. USB functions, LEDs, mechanical dimensions and environmental limits.

## References

- [TI power protection overview](https://www.ti.com/product-category/power-management/power-protection-switches-controllers/overview.html): protection options; not a requirement to use a dedicated protection IC.
- [TI input protection and buck reference design](https://www.ti.com/lit/pdf/tiduee8): example of coordinated protection; component choices require evaluation for this board's input envelope.
- [ST STM32 hardware development guidance](https://www.st.com/resource/en/application_note/an6011-getting-started-with-stm32u3-mcu-hardware-development-stmicroelectronics.pdf): example MCU/debug support requirements; use the matching guide for the selected STM32.
