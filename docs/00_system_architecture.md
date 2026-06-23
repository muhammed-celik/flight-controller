# Flight Controller System Architecture

## 1. Selected Architecture

This project uses the following fixed design choices:

- **Sensor module:** GY-91, containing an MPU9250 (3-axis gyroscope,
  3-axis accelerometer, and AK8963 3-axis magnetometer) plus a BMP280
  barometer.
- **Real-time datapath:** implemented in synthesizable RTL.
- **Processor:** MicroBlaze or an RV32 soft CPU.
- **Processor interconnect:** AXI4-Lite slave interface on the RTL flight
  controller.
- **Attitude estimator:** selectable between an RTL complementary filter and a
  CPU software EKF. The choice is not yet frozen.

The CPU choice is intentionally abstracted behind AXI4-Lite. No RTL module may
depend on MicroBlaze-specific or RISC-V-specific signals.

## 2. Hardware/Software Partition

| Function | Owner | Reason |
|---|---|---|
| GY-91 bus transactions and sample timing | RTL | Deterministic acquisition and watchdog timing |
| Raw sample snapshot registers and interrupt | RTL | Coherent CPU reads over AXI4-Lite |
| Optional fixed-point calibration and PT1 filtering | RTL | Low, deterministic per-sample cost |
| Complementary attitude filter (option A) | RTL | Self-contained, deterministic, low resource cost |
| EKF prediction/update (option B) | CPU | Better bias modeling and multi-sensor fusion |
| Barometer compensation and altitude conversion | CPU | Uses factory coefficients and wider arithmetic |
| Magnetometer calibration | CPU | Requires vector/matrix operations |
| Rate PID, motor mixer, DSHOT/PWM, hard failsafe | RTL | Bounded latency even if software stalls |
| Configuration, tuning, telemetry and logging | CPU | Software is easier to change |

The inner angular-rate loop remains entirely in RTL and consumes filtered gyro
data directly. The slower outer attitude loop accepts either the RTL
complementary-filter result or a committed CPU EKF result. CPU jitter therefore
never enters the most time-critical stabilization path.

## 3. Data Flow

```text
GY-91 -> RTL acquisition -> calibration/PT1 -> rate PID -> mixer -> motors
                    |              |
                    |              +-> RTL complementary filter --+
                    |                                            |
                    +-> AXI snapshot + IRQ -> CPU EKF -> mailbox +-> estimator mux -> outer loop
```

In complementary-filter mode, the estimator never waits for the CPU. In EKF
mode, the CPU reads a coherent timestamped sensor frame and writes its result to
a double-buffered AXI mailbox. RTL accepts it only after an atomic commit.

## 4. AXI4-Lite Contract

AXI4-Lite is used for control and low-rate sample exchange; it is not treated
as an unframed stream. The register block shall provide:

- identification and version registers;
- configuration and status registers;
- a coherent GY-91 sample snapshot;
- sample sequence, timestamp, valid flags, and sticky error flags;
- interrupt status/enable/acknowledge registers;
- estimator-mode selection and status;
- a double-buffered CPU estimator mailbox with a commit register;
- PID, filter, mixer, motor-output, and failsafe configuration registers.

All multiword payloads use a snapshot or commit handshake so that the CPU can
never observe or publish a partially updated vector. Software must detect
skipped sensor frames from the monotonically increasing sequence number.

## 5. Nominal Rates

| Function | Nominal rate |
|---|---:|
| MPU9250 gyro/accelerometer acquisition | 1 kHz |
| RTL angular-rate control | 500 Hz or 1 kHz |
| RTL complementary-filter update | 500 Hz or 1 kHz |
| Optional CPU EKF prediction | 500 Hz or 1 kHz |
| AK8963 magnetometer update | 50-100 Hz |
| BMP280 pressure/temperature update | 25-50 Hz |
| Outer attitude control | 250-500 Hz |

These are initial targets, not hard-coded constants. Rates and watchdog
thresholds are configurable through AXI4-Lite.
