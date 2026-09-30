# GPS, IMU and ambient temperature interfaces

Implemented in `ECU_Hardware/schematics/` and connected through `stm32_ecu.kicad_sch` to U11, STM32G474CEUx. PCB routing and firmware implementation are separate work.

## STM32 pin allocation

Signal names use the MCU perspective.

| Signal | MCU pin | Package pad | Configuration | Sensor connection |
|---|---|---:|---|---|
| GPS_TX | PA9 | 31 | USART1_TX, AF7 | U1 RXD, pad 3 |
| GPS_RX | PA10 | 32 | USART1_RX, AF7 | U1 TXD, pad 2 |
| GPS_PPS | PA8 | 30 | TIM1_CH1 input capture, AF6; alternatively EXTI8 | U1 1PPS, pad 4 |
| IMU_SCK | PA5 | 13 | SPI1_SCK, AF5 | U3 SCL/SPC, pad 13 |
| IMU_MISO | PA6 | 14 | SPI1_MISO, AF5 | U3 SDO, pad 1 |
| IMU_MOSI | PA7 | 15 | SPI1_MOSI, AF5 | U3 SDA/SDI, pad 14 |
| IMU_CS_N | PA4 | 12 | GPIO output, initially high | U3 CS, pad 12 |
| IMU_INT | PC4 | 16 | EXTI4 input | U3 INT1, pad 4 |
| TEMP_SENSE | PA0 | 8 | ADC1_IN1, analog, no pulls | R5/TH1/C11 junction |

Existing CAN, USB and SWD pin assignments are preserved. Pedal, steering and VIN input interfaces remain reserved and unfinished.

## GPS

U1 remains the selected ATGM336H-5N31. It uses board 3.3 V with 10 µF bulk and 100 nF bypass capacitors. VBAT also uses board 3.3 V with a separate 100 nF bypass: there is no backup battery, so backup retention is lost when board power is removed.

J3 is a U.FL connector for an **external 3.3 V active GNSS antenna**. L4 is a 47 nH RF choke feeding the antenna from the module's VCC_RF output, following its active antenna reference circuit. Select an antenna with compatible supply/current and the module's specified 15–30 dB gain range. The module's antenna short-circuit limit is approximately 50 mA. Budget up to 100 mA module peak current plus antenna current when checking the complete 3.3 V regulator load.

ON/OFF and nRESET are deliberately left open as in the reference circuit; reserved and unused interfaces have no-connect markers. UART configuration and PPS capture still need firmware. Route the antenna feed as a short 50 Ω line over continuous ground, away from the buck switch node and fast digital traces. Place bypass capacitors at their respective module pins and check supply ripple against the 50 mVpp recommendation. The antenna is part of the GNSS subsystem; external pedal/steering supply policy is unchanged.

## IMU

U3 remains LSM6DSV16XTR, configured for four-wire SPI in connection mode 1. Both supply pins use 3.3 V with separate 100 nF bypass capacitors and 1 µF local bulk. A 10 kΩ CS pull-up keeps the device deselected while the MCU resets.

INT1 connects to PC4. INT2 and auxiliary pins 10/11 are unused. Pins 2/3 are grounded as specified when the sensor hub, analog hub and Qvar functions are disabled. Their project symbol electrical types are input to represent this mode; do not enable the sensor-hub output functions with this wiring. Other imported pin types were corrected from unspecified to their documented functions in both the project library and schematic cache.

Firmware must initialize the sensor, configure SPI, choose ranges and sample rates, route data-ready/FIFO events to INT1, and calibrate the installed axes. Place the IMU rigidly and document orientation on the PCB.

## Temperature

TH1 is Murata NCU18XH103F60RB: 10 kΩ at 25 °C, ±1%, B25/50 = 3380 K ±1%. R5 changes from 4.7 kΩ to 39 kΩ ±1% to reduce excitation current and self-heating. C11 is 100 nF to ground.

At 25 °C and a 3.3 V supply, the divider produces about 0.673 V, draws 67 µA, and dissipates 0.045 mW in the NTC. The filter corner is approximately 200 Hz at that temperature. Hotter temperatures produce lower ADC voltages.

Use a long ADC acquisition time, such as 247.5 ADC clock cycles, and average samples. Account for the actual ADC reference and divider supply. Calculate `Rntc = 39000 × Vout / (V3V3 − Vout)` and use Murata's resistance/temperature table; the beta equation is an approximation. Detect near-ground and near-supply readings as possible short/open faults. Place TH1 at a ventilated board edge away from heat sources. Its reading represents local board/enclosure temperature; layout determines how closely that follows ambient air.

## Libraries and verification

Resistors and capacitors use the installed PCM_JLCPCB symbols and footprints with their existing manufacturer/LCSC fields. The installed catalog lacks the selected NTC and 47 nH RF choke, so `libraries/PCM_JLCPCB-Project.kicad_sym` is an explicitly documented **project extension**, registered in `sym-lib-table`:

| Part | Manufacturer | LCSC | PCM footprint |
|---|---|---|---|
| NCU18XH103F60RB | Murata | C440252 | R_0603 |
| LQW15AN47NG00D | Murata | C22334 | L_0402 |

KiCad 10 netlist checks confirm all nine signals, supplies, grounds, pull-up, antenna bias and thermistor connectivity. Existing connections outside the replaced sensor circuits are preserved. References are unique, and the selected footprints exist with the connected pad numbers. The rendered hierarchy and five affected pages were inspected.

ERC findings reduced from 188 to 89. There are no ERC findings on the GPS, IMU or ambient temperature pages. The remaining findings concern existing circuitry, missing libraries/footprints elsewhere and unfinished reserved interfaces; this is not a whole-board manufacturing sign-off.

## Datasheet references

- [STM32G474 datasheet](https://www.st.com/resource/en/datasheet/stm32g474ce.pdf), pin definitions and alternate functions.
- [LSM6DSV16X datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf), section 7 and Table 23.
- [ATGM336H-5N manufacturer user manual](https://datasheet4u.com/pdf/1260662/ATGM336H-5N.pdf), sections 2.4–2.8; manufacturer document hosted by a distributor/archive.
- [Murata NCU18 specifications](https://www.murata.com.cn/products/productdata/8828036775966/SPEC-NCU18.pdf?1728271827000=).
- [Murata LQW15AN47NG00D](https://www.murata.com/en-us/products/productdetail?partno=LQW15AN47NG00D).
