# Flight Controller System Architecture

## 1. Purpose

This document defines the architecture of a quadcopter flight controller built
around a Digilent Cmod A7-35T (`xc7a35tcpg236-1`). The FPGA contains a
MicroBlaze V soft CPU and a custom SystemVerilog flight-controller IP.

The first complete system targets a manually controlled X quadcopter with:

- RadioMaster Pocket ELRS 2.4 GHz transmitter and RadioMaster RP1 receiver.
- CRSF commands at 420 kbaud.
- GY-91 sensor module containing MPU-9250, AK8963, and BMP280 devices.
- Four DShot600 ESC outputs.
- Acro/rate, stabilized angle, heading-hold, and altitude-hold modes.

The GY-91 cannot measure horizontal position. The hover mode therefore holds
attitude, heading, and barometric altitude but cannot prevent horizontal drift.
Position hold requires a future optical-flow, GPS, UWB, or similar sensor.

## 2. Design Principles

1. Hard real-time control and final motor safety reside in RTL.
2. CPU scheduling or failure cannot bypass the motor safety gate.
3. The rate-control path can operate while MicroBlaze V is held in reset.
4. CPU results cross into RTL through atomic, bounded, freshness-checked
   mailboxes.
5. All sensor and control values carry timestamps, sequence numbers, and
   validity information.
6. Configuration changes become active atomically at control-frame boundaries.
7. Fixed-point arithmetic saturates instead of wrapping.
8. Board-specific primitives and pin handling remain outside the portable
   flight-controller core.

## 3. Top-Level Organization

```text
GY-91 shared I2C
        |
        v
sensor scheduler -> calibration -> filtering -> sensor snapshots
                                            |            |
                                            |            +-> AXI4-Lite
                                            v                    |
                                  RTL 6-axis estimator           v
                                            |              MicroBlaze V
                                            |                    |
                                            +<-- estimator ------+
                                            |    mailbox
CRSF receiver -> command decoder ---------> control arbitration
                                                    |
                                                    v
                                          rate/angle control
                                                    |
                                                    v
                                      X mixer and desaturation
                                                    |
                                          arming/safety gate
                                                    |
                                                    v
                                           four DShot600 outputs
```

The initial system uses one 100 MHz clock domain. Lower update rates use clock
enables rather than generated logic clocks.

The common clock infrastructure publishes a 64-bit 100 MHz cycle timestamp, so
one timestamp count represents 10 ns. Deterministic one-cycle enables provide
the 1 kHz, 250 Hz, 100 Hz, 50 Hz, and 10 Hz scheduling boundaries. All counters
restart from a common phase after qualified reset release.

## 4. Hardware and Software Partition

### 4.1 SystemVerilog RTL

RTL owns the functions whose timing or safety must not depend on firmware:

| Function | Required behavior |
| --- | --- |
| Clock/reset control | Asynchronous assertion and synchronous reset release |
| Timebase | 64-bit timestamp and deterministic rate enables |
| I2C master | Open-drain operation, timeout, and stuck-bus recovery |
| Sensor scheduler | Priority polling with bounded transactions |
| MPU acquisition | 1 kHz inertial samples and freshness monitoring |
| AK8963/BMP280 acquisition | 100 Hz and 50 Hz timestamped samples |
| Calibration/filter datapath | Axis mapping, saturation, and fixed latency |
| Sensor snapshots | Atomic CPU reads with sequence and timestamp |
| RTL attitude estimator | 6-axis quaternion fallback |
| CRSF receiver | Frame validation, channel decoding, and link watchdog |
| Rate PID | Deterministic 1 kHz inner loop |
| Angle controller | Deterministic 250 Hz stabilized control |
| Mixer | X-quad mixing and priority desaturation |
| DShot600 | Four parallel, precisely timed motor outputs |
| Arming/failsafe | Final motor authorization and sticky faults |
| AXI4-Lite registers | Validated CPU access and atomic mailboxes |

### 4.2 MicroBlaze V Firmware

Firmware owns algorithms that are lower-rate or expected to evolve:

| Function | Required behavior |
| --- | --- |
| 9-axis attitude estimation | Quaternion estimator with disturbance rejection |
| Magnetometer calibration | Factory adjustment, hard-iron, and soft-iron correction |
| BMP280 compensation | Bosch signed 64-bit pressure compensation |
| Vertical estimation | Barometric altitude and inertial vertical velocity fusion |
| Altitude supervision | Altitude and climb-rate loops with bounded RTL output |
| Heading supervision | Magnetometer-corrected heading command |
| Configuration | Validate and atomically commit gains and calibration |
| Telemetry/logging | Debug UART and future CRSF return telemetry |
| CPU health | Periodic heartbeat and deadline status |

Firmware never writes raw motor commands or motor-enable state.

## 5. Sensor Architecture

The supplied GY-91 exposes no MPU interrupt pin. The baseline is therefore a
shared 400 kHz I2C bus with scheduled polling.

| Device | Address | Runtime rate |
| --- | --- | --- |
| MPU-9250 | Probe `0x68` and `0x69` | 1 kHz |
| AK8963 | `0x0C` through MPU bypass | 100 Hz |
| BMP280 | Probe `0x76` and `0x77` | 50 Hz |

Bring-up begins at 100 kHz. Operation moves to 400 kHz only after the physical
bus rise time is measured and found acceptable.

### 5.1 MPU-9250 Baseline

- Gyroscope full scale: +/-2000 degrees/s.
- Accelerometer full scale: +/-8 g.
- Gyroscope DLPF: 92 Hz.
- Accelerometer DLPF: 99 Hz.
- Sample rate: 1 kHz.
- Clock source: automatic PLL.
- AK8963 bypass: enabled after initialization.

The scheduler reads `INT_STATUS` and the complete 14-byte inertial sample. A
missing data-ready indication causes a bounded retry and increments a diagnostic
counter. It never stalls the scheduler indefinitely.

### 5.2 AK8963 Baseline

- Read factory sensitivity adjustment during initialization.
- Use 16-bit continuous measurement mode at 100 Hz.
- Read `ST1`, all six axis bytes, and `ST2` in one transaction.
- Reject samples with data-ready clear or magnetic overflow set.
- Perform hard-iron and soft-iron calibration in firmware.
- Reject magnetometer correction during implausible field magnitude or change.

### 5.3 BMP280 Baseline

- Temperature oversampling x1.
- Pressure oversampling x4.
- IIR coefficient 4.
- Read pressure and temperature in one six-byte transaction at 50 Hz.
- Perform Bosch 64-bit compensation in firmware.

## 6. Estimation

### 6.1 RTL Fallback

The fallback estimator is a fixed-point, 6-axis quaternion
complementary/Mahony implementation:

- Gyroscope propagation at 1 kHz.
- Accelerometer gravity correction only when acceleration magnitude is valid.
- Quaternion storage in `Q2.30`.
- Periodic normalization with bounded arithmetic.
- Roll and pitch extraction for stabilized fallback control.
- Health, age, sequence, and confidence outputs.

It bounds roll and pitch but permits yaw drift because it does not depend on the
magnetometer or CPU.

### 6.2 CPU Primary Estimator

MicroBlaze V initially runs a 9-axis Mahony estimator. It publishes quaternion,
body rates, heading, source-sensor sequence, timestamp, and health through a
shadow-plus-commit mailbox.

RTL selects CPU output only when:

- The commit sequence advances.
- The source sensor sequence and timestamp are valid.
- Result age is below the configured limit, initially 5 ms.
- Quaternion norm and all fields pass plausibility limits.
- CPU heartbeat and estimator health are valid.

Otherwise RTL selects its fallback estimator.

## 7. Flight Modes

| Mode | Control behavior | CPU dependency |
| --- | --- | --- |
| Disarmed | Continuous DShot stop command | None |
| Acro/rate | Sticks command body rates | None |
| Angle | Roll/pitch sticks command angles; yaw commands rate | Optional primary estimator |
| Heading hold | Centered yaw stick holds magnetic heading | Required for magnetic correction |
| Altitude hold | Centered throttle holds altitude; stick commands climb rate | Required |
| Hover | Level attitude plus heading and altitude hold | Required |

CPU loss in acro mode does not interrupt control. CPU or barometer loss exits
altitude hold. The RTL estimator remains available for bounded roll/pitch
stabilization during a fault response.

## 8. Control Pipeline

### 8.1 Rate Controller

The inner loop runs at 1 kHz and includes:

- Three-axis PID.
- Derivative on measurement.
- Configurable derivative filtering.
- Feedforward.
- Integrator clamping and back-calculation anti-windup.
- Integrator reset or decay while disarmed.
- Saturating, explicitly rounded fixed-point arithmetic.
- Frame-boundary activation of gain changes.

### 8.2 Angle and Altitude Controllers

The RTL angle controller runs at 250 Hz and produces bounded body-rate demands.
Heading corrections from firmware are range-, slew-, and freshness-checked.

Firmware estimates altitude and vertical speed at 50 to 100 Hz. It sends a
bounded collective-thrust correction to RTL. The pilot throttle stick commands
climb rate around a center deadband while altitude hold is active.

### 8.3 Mixer

The baseline is an X-quad mixer. Exact signs and motor order remain configurable
until validated against the assembled airframe.

Desaturation priority is:

1. Preserve roll and pitch authority.
2. Reduce yaw authority.
3. Shift collective throttle.
4. Scale attitude corrections if necessary.
5. Clamp only as the final operation.

## 9. Arming and Failsafe

Arming requires valid RC data, low throttle, a deliberate arm-switch sequence,
healthy inertial data, completed gyro calibration, a valid estimator for the
requested mode, no hard fault, and the external hardware-kill input in its safe
permission state.

Hard motor shutdown conditions include:

- Explicit pilot disarm.
- External kill assertion.
- Stale MPU data beyond its hard timeout.
- Clock/reset failure.
- Internal control or DShot watchdog failure.
- Invalid safety-FSM state.

Faults are sticky and cannot be cleared while armed. Link recovery never causes
automatic re-arming. The exact RC-loss descent profile must be validated on a
restrained vehicle; the GY-91 alone cannot guarantee a safe autonomous landing.

## 10. CPU Interface

The flight-controller IP exposes one flattened 32-bit AXI4-Lite slave and one
level interrupt to the MicroBlaze V subsystem.

| Offset | Region |
| --- | --- |
| `0x0000` | Identity, capabilities, and build information |
| `0x0100` | Timebase, global status, and interrupt control |
| `0x0200` | Sensor status and coherent snapshots |
| `0x0300` | Calibration shadow registers |
| `0x0400` | Filter configuration |
| `0x0500` | RTL and CPU estimator mailboxes |
| `0x0600` | Rate and angle controller settings |
| `0x0700` | Mixer and motor-output settings |
| `0x0800` | CRSF channels and link status |
| `0x0900` | Barometer and altitude mailbox |
| `0x0A00` | Arming and failsafe status |
| `0x0B00` | Debug counters and trace control |

Sensor values use coherent snapshots. Multiword writes use shadow registers and
an explicit commit. Active control settings update only on control boundaries.
Fault and IRQ status uses write-one-to-clear where clearing is permitted.

## 11. Target RTL Structure

```text
cores/
  common/
    axi_lite_slave/
    crc8/
    fixed_point/
    i2c_master/
    reset_sync/
    uart_rx/
  flight_controller/
    src/
      fc_top.sv
      fc_axi_regs.sv
      fc_timebase.sv
      sensor_scheduler.sv
      mpu9250_ctrl.sv
      ak8963_ctrl.sv
      bmp280_ctrl.sv
      sensor_calibration.sv
      sensor_filter.sv
      attitude_fallback.sv
      estimator_mux.sv
      crsf_decoder.sv
      command_mapper.sv
      rate_pid.sv
      angle_controller.sv
      motor_mixer.sv
      dshot_tx.sv
      arming_failsafe.sv
  cmod_a7_flight_top/
    src/
      top.sv
sw/
  microblaze_v/
```

The future `cmod_a7_flight_top` board wrapper owns Xilinx primitives, physical
I/O buffers, and Cmod pin mapping. The obsolete blinky and demonstration Cmod
top cores have been removed. The portable `flight_controller` core contains no
board-specific pin names or processor-specific signals.
