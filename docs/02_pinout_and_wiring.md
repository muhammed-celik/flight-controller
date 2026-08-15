# Cmod A7 Flight Controller Pinout and Wiring

## 1. Sources and Electrical Domain

This assignment is derived from:

- `docs/datasheet/cmod_a7_sch.pdf`, especially schematic pages 1 through 4.
- `docs/datasheet/gy91_pinout.jpg`.
- The MPU-9250, AK8963, and BMP280 datasheets in `docs/datasheet/`.

The Cmod A7 exposed digital banks use 3.3 V. External digital signals must not
exceed the Artix-7 input limits. The presence of a 5 V power pin on a module does
not make its signal pins 5 V tolerant.

## 2. Assigned FPGA Pins

| Function | Board connection | FPGA package pin | Direction |
| --- | --- | --- | --- |
| 12 MHz input | Onboard GCLK | `L17` | Input |
| Reset | BTN0 | `A18` | Input |
| Status LED | LED1 | `A17` | Output |
| Fault/lock LED | LED2 | `C16` | Output |
| GY-91 I2C SCL | Pmod JA1 | `G17` | Open-drain bidirectional |
| GY-91 I2C SDA | Pmod JA2 | `G19` | Open-drain bidirectional |
| CRSF RX | J2 pin 2, PIO26 | `R3` | Input |
| CRSF TX | J2 pin 3, PIO27 | `T3` | Output |
| Motor 1 DShot | J2 pin 4, PIO28 | `R2` | Output/bidirectional |
| ESC 1 telemetry | J2 pin 5, PIO29 | `T1` | Input |
| Motor 2 DShot | J2 pin 6, PIO30 | `T2` | Output/bidirectional |
| ESC 2 telemetry | J2 pin 7, PIO31 | `U1` | Input |
| Motor 3 DShot | J2 pin 8, PIO32 | `W2` | Output/bidirectional |
| ESC 3 telemetry | J2 pin 9, PIO33 | `V2` | Input |
| Motor 4 DShot | J2 pin 10, PIO34 | `W3` | Output/bidirectional |
| ESC 4 telemetry | J2 pin 11, PIO35 | `V3` | Input |
| External hardware kill | J1 pin 1, PIO01 | `M3` | Input |
| Buzzer control | J1 pin 2, PIO02 | `L3` | Output |
| Battery voltage | J1 pin 15, AIN15 | `G3/G2` | XADC analog pair |
| Debug UART RX | Onboard USB UART | `J17` | Input |
| Debug UART TX | Onboard USB UART | `J18` | Output |

Digital external ports use `LVCMOS33`. DShot and UART outputs begin with a low
drive setting and `SLEW=SLOW`; drive and slew are increased only when measured
signal integrity requires it.

The assignment does not use the onboard QSPI, JTAG, SRAM, configuration, or
clock pins as general-purpose I/O.

## 3. GY-91 Wiring

The supplied module has this eight-pin header:

| GY-91 pin | Required connection |
| --- | --- |
| `5V` | Leave disconnected |
| `3.3V` | Pmod 3.3 V |
| `GND` | Pmod ground |
| `SCK/SCL` | JA1, FPGA `G17` |
| `MOSI/SDA` | JA2, FPGA `G19` |
| `MISO/SAO` | Strap low through approximately 10 kohm |
| `NCS` | Hard pull-up to 3.3 V |
| `CSB` | Hard pull-up to 3.3 V |

Expected addresses with SAO low are MPU `0x68`, BMP280 `0x76`, and AK8963
`0x0C`. The module photograph does not prove how both address straps are wired,
so RTL probes both MPU addresses and both BMP addresses.

`NCS`, `CSB`, and `SAO` use physical resistors. They must not depend on FPGA
startup values. BMP280 `CSB` must be high during power-up to select I2C mode.

The module exposes no MPU interrupt pin, so acquisition uses scheduled polling.

## 4. I2C Pull-Ups

The Cmod Pmod signal paths contain 200 ohm series resistors. They are acceptable
for a short 400 kHz I2C connection.

Before adding pull-ups, measure resistance from SDA and SCL to the module 3.3 V
pin with power removed. If the module has no suitable populated pull-ups, add one
2.2 to 4.7 kohm pull-up from each bus line to 3.3 V. Do not unintentionally
parallel multiple strong pull-up sets.

Start at 100 kHz and inspect bus rise time. Enable 400 kHz only after confirming
that the bus reaches a valid high level with adequate margin.

## 5. CRSF Receiver

The baseline radio is a RadioMaster Pocket ELRS 2.4 GHz transmitter with a
RadioMaster RP1 receiver.

Connections:

- Receiver TX to Cmod CRSF RX, J2 pin 2.
- Receiver RX to Cmod CRSF TX, J2 pin 3.
- Receiver ground to system ground.
- Receiver supply to a suitable regulated supply, normally 5 V according to
  the selected receiver's verified specification.

Confirm the receiver UART signal levels before connection. Add a 22 to 100 ohm
source-series resistor on FPGA TX if wiring length warrants it.

## 6. ESC Connections

J2 groups each DShot output beside an optional separate telemetry input. If the
ESCs support bidirectional DShot, telemetry uses the DShot pin after RTL
tri-states it and the separate telemetry pins remain available for expansion.

Each ESC connection requires a low-impedance common ground. A carrier or wiring
board should provide individual signal/ground pairs because the Cmod DIP header
does not provide four adjacent ground contacts.

No telemetry signal above 3.3 V may connect directly to the FPGA. Use a buffer,
level shifter, or divider when required. Source-series resistors of approximately
22 to 100 ohm may be fitted near the FPGA on long DShot wires.

## 7. Power Distribution

The flight battery must never connect directly to Cmod `VU`. The Cmod schematic
specifies a safe VU range of 4.5 to 5.5 V.

```text
flight battery
   |
   +-> ESC power distribution
   |
   +-> regulated 5 V BEC
          |
          +-> Cmod VU
          +-> ELRS receiver

Cmod 3.3 V -> GY-91 3.3 V
```

All grounds are common, but motor-current return paths must not flow through the
sensor or FPGA ground wiring.

## 8. Battery Measurement

AIN15 is reserved for battery voltage. An external divider must keep the header
input safely below its permitted range at the maximum charged and transient
battery voltage. The front end requires a divider, RC filter, series protection,
and suitable overvoltage protection. Final resistor values depend on whether the
aircraft uses a 3S, 4S, or 6S battery.

## 9. External Kill and Buzzer

The external kill input uses a physical pull-down so an open or disconnected
wire produces the killed state. RTL synchronizes and qualifies the signal. The
input grants permission to arm but cannot arm the aircraft by itself.

The buzzer output drives an external transistor or MOSFET. An FPGA pin must not
power a buzzer directly. An inductive buzzer requires appropriate flyback
protection.

## 10. Motor and Sensor Orientation

Initial motor convention, viewed from above with the aircraft nose forward:

```text
             FRONT

       M1             M2
   front-left     front-right

       M4             M3
    rear-left      rear-right
```

Initial rotation convention:

| Motor | Rotation |
| --- | --- |
| M1 | Clockwise |
| M2 | Counterclockwise |
| M3 | Clockwise |
| M4 | Counterclockwise |

Motor order, rotation signs, and sensor-axis mapping remain configurable until
verified with propellers removed.

Mount the GY-91 rigidly near the center of gravity, away from motor-current
wiring. Its final axis permutation is determined experimentally by rotating one
physical aircraft axis at a time and observing accelerometer and gyro response.
