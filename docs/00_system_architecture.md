# Flight Controller System Architecture

## 1. Selected Architecture

This project uses the following fixed design choices:

- **Sensor module:** GY-91, containing an MPU9250 (3-axis gyroscope,
  3-axis accelerometer, and AK8963 3-axis magnetometer) plus a BMP280
  barometer.
- **Real-time datapath:** implemented in synthesizable RTL.
- **First implementation:** pure RTL flight controller. The vehicle must be
  able to initialize sensors, calibrate gyro bias, accept RC input, arm,
  stabilize, failsafe, and drive motors without any CPU firmware running.
- **Future processor:** MicroBlaze or an RV32 soft CPU, connected through an
  optional AXI4-Lite slave interface on the RTL flight controller.
- **Attitude estimator:** RTL complementary filter for first flight; optional
  CPU software EKF added later behind the same estimator contract.

The CPU choice is intentionally abstracted behind AXI4-Lite. No RTL module may
depend on MicroBlaze-specific or RISC-V-specific signals, and no first-flight
function may require a CPU register write before arming.

## 2. Hardware/Software Partition

| Function | Owner | Reason |
|---|---|---|
| GY-91 bus transactions and sample timing | RTL | Deterministic acquisition and watchdog timing |
| RTL-default configuration | RTL | Synthesized parameters allow flight with no CPU present |
| Raw sample snapshot registers and interrupt | RTL | Optional coherent CPU reads over AXI4-Lite |
| Fixed-point calibration and PT1 filtering | RTL | Low, deterministic per-sample cost |
| Complementary attitude filter | RTL | First-flight estimator; self-contained and deterministic |
| EKF prediction/update (future option) | CPU | Better bias modeling and multi-sensor fusion |
| Barometer compensation and altitude conversion | CPU/future RTL option | Not required for first-flight manual stabilization |
| Magnetometer calibration | CPU/future RTL option | Not required for first-flight manual stabilization |
| Rate PID, motor mixer, DSHOT/PWM, hard failsafe | RTL | Bounded latency even if software stalls |
| Configuration, tuning, telemetry and logging | CPU, later | Software is easier to change after RTL defaults are proven |

The first-flight control path remains entirely in RTL and consumes filtered gyro
and RTL complementary-filter attitude data directly. Later, the slower outer
attitude loop may accept a committed CPU EKF result. CPU jitter therefore never
enters the most time-critical stabilization path, and a missing CPU cannot block
basic flight.

## 3. Data Flow

```text
GY-91 -> RTL acquisition -> RTL boot calibration -> PT1 -> rate PID -> mixer -> motors
                    |                         |
                    |                         +-> RTL complementary filter -> outer loop
                    |
                    +-> optional AXI snapshot + IRQ -> CPU EKF -> mailbox -> estimator mux
```

In first-flight mode, the estimator never waits for the CPU. In later EKF mode,
the CPU reads a coherent timestamped sensor frame and writes its result to a
double-buffered AXI mailbox. RTL accepts it only after an atomic commit and can
fall back to the RTL estimator or inhibit attitude-dependent modes if the CPU
result is stale.

## 4. RTL-First Configuration Policy

Every module that affects first-flight behavior shall have safe synthesized
defaults or board-strap parameters:

- sensor addresses and operating points have default probe/config sequences;
- gyro boot calibration is performed by RTL before arming is allowed;
- filter coefficients, PID gains, mixer limits, output protocol, and failsafe
  thresholds have conservative RTL defaults;
- CPU/AXI writes may override defaults later, but absence of AXI writes must not
  leave a required register uninitialized;
- status, sticky faults, and snapshots remain readable when a CPU is added.

## 5. AXI4-Lite Contract

AXI4-Lite is optional for first flight. When present, it is used for control,
low-rate sample exchange, tuning, logging, and EKF handoff; it is not treated as
an unframed stream. The register block shall provide:

- identification and version registers;
- configuration and status registers with documented reset/default values;
- a coherent GY-91 sample snapshot;
- sample sequence, timestamp, valid flags, and sticky error flags;
- interrupt status/enable/acknowledge registers;
- estimator-mode selection and status;
- a double-buffered CPU estimator mailbox with a commit register;
- PID, filter, mixer, motor-output, and failsafe configuration registers whose
  reset values match the RTL-first defaults.

All multiword payloads use a snapshot or commit handshake so that the CPU can
never observe or publish a partially updated vector. Software must detect
skipped sensor frames from the monotonically increasing sequence number.

## 6. Nominal Rates

| Function | Nominal rate |
|---|---:|
| MPU9250 gyro/accelerometer acquisition | 1 kHz |
| RTL angular-rate control | 500 Hz or 1 kHz |
| RTL complementary-filter update | 500 Hz or 1 kHz |
| Optional CPU EKF prediction | 500 Hz or 1 kHz |
| AK8963 magnetometer update | 50-100 Hz |
| BMP280 pressure/temperature update | 25-50 Hz |
| Outer attitude control | 250-500 Hz |

These are initial targets. First-flight RTL uses synthesized defaults; later CPU
firmware may tune rates and watchdog thresholds through AXI4-Lite where the RTL
implementation exposes overrides.
