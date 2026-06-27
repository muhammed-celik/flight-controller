# Module 13: MicroBlaze V IP Integration and Register Map Guide

## 1. Goal

The design will use a MicroBlaze V soft CPU for configuration, tuning,
telemetry, logging, and optional higher-level estimation. The hard real-time
flight path remains in RTL:

```text
GY-91 -> calibration -> filter -> estimator -> PID -> mixer -> motor output
```

MicroBlaze V interacts with that path through AXI4-Lite registers, interrupts,
and coherent snapshot/mailbox handshakes. The CPU must not be placed in the
inner rate-control timing path. If firmware stalls, resets, or is absent, the
RTL defaults and failsafe logic must keep the vehicle in a bounded state.

## 2. Recommended System Topology

Use a Vivado block design for the processor subsystem and keep the flight
controller as a packaged RTL IP block.

```text
                 +----------------------+
                 | MicroBlaze V system  |
                 |                      |
clk/reset -----> | CPU                  |
JTAG/UART -----> | debug/console        |
BRAM/DDR  -----> | instruction/data mem |
timer/irq <----- | interrupt controller |
                 | AXI master           |
                 +----------+-----------+
                            |
                         AXI4-Lite
                            |
                 +----------v-----------+
                 | flight_controller_ip |
                 |                      |
sensor pins <--> | RTL subcores         |
RC input   ----> | register bank        |
motor pins <---- | irq output           |
                 +----------------------+
```

The MicroBlaze V subsystem should own software memory, debug, UART, timers, and
the AXI interconnect. The flight controller IP should own sensor pins, RC input,
motor outputs, real-time clocks/enables, safety gates, and all RTL state needed
for flight.

## 3. IP Core Boundary

Create one top-level packaged IP named `flight_controller_ip`. Internally it may
instantiate several subcores, but the CPU-facing boundary should stay small and
stable.

### 3.1 External Ports

| Port group | Direction | Purpose |
|---|---:|---|
| `aclk`, `aresetn` | input | AXI/control clock and active-low reset |
| sensor bus pins | bidir/output/input | GY-91 MPU9250, AK8963, BMP280 access |
| `sbus_in` | input | RC receiver input |
| `motor_out[3:0]` | output | DSHOT/PWM ESC signals |
| `irq` | output | Level interrupt to AXI interrupt controller |
| optional debug pins | output | Logic analyzer visibility during bring-up |

Keep AXI4-Lite at 32-bit data width. Use word-aligned addresses and reject or
ignore unsupported byte strobes unless the addressed register explicitly allows
partial writes.

### 3.2 Internal Subcores

| Subcore | Responsibility | CPU interaction |
|---|---|---|
| `fc_axi_regs` | AXI4-Lite slave, register decode, IRQ registers | Directly connected to AXI |
| `imu_acq` | Sensor initialization and sample acquisition | Snapshot/status/config |
| `sensor_cal` | Boot bias calibration and optional overrides | Status/control/coefficients |
| `digital_filter` | PT1 filters for gyro/accel | Coefficients/status |
| `estimator_rtl` | Complementary filter | Mode/config/status |
| `estimator_mailbox` | CPU EKF result handoff | Double-buffered commit |
| `pid_ctrl` | Cascaded angle/rate PID | Gains/limits/status |
| `motor_mixer` | Quad-X mixing and saturation handling | Limits/status |
| `motor_output` | DSHOT600/PWM generation | Mode/status/telemetry request |
| `sbus_decoder` | RC channel decode and link status | Snapshot/status |
| `failsafe` | Arming, watchdogs, motor gate | Control/status/config |

Only `fc_axi_regs` should contain AXI protocol logic. The other subcores should
use simple local configuration/status signals. This keeps the flight logic easy
to simulate without a processor bus.

## 4. Register Bank Rules

Use one register bank for the whole flight controller IP. Avoid giving every
subcore an independent AXI slave unless the design becomes too large to decode
cleanly. One slave is easier to document, easier to drive from firmware, and
uses fewer interconnect resources.

### 4.1 Access Types

| Type | Meaning |
|---|---|
| `RO` | Read-only hardware status |
| `RW` | Read/write configuration register |
| `W1C` | Write one to clear sticky status bit |
| `WO` | Write-only command pulse or commit |
| `RC` | Read clears event, use sparingly |

Prefer `W1C` over read-clear for interrupt and fault registers. Read-clear can
hide events during debugging.

### 4.2 Reset and Default Policy

Every writable register must have a documented reset value. Reset values must
match the RTL-first safe defaults from the module documents:

- sensor acquisition starts from a known initialization sequence;
- PID/filter/mixer/failsafe defaults are conservative;
- motors remain disarmed after reset;
- CPU overrides are disabled until explicitly enabled;
- sticky fault registers preserve useful failure information until cleared.

### 4.3 Multiword Data Rule

Never expose a changing multiword vector directly. Use one of these patterns:

| Pattern | Direction | Use case |
|---|---|---|
| snapshot latch | RTL to CPU | sensor frames, RC channel frames, status groups |
| shadow plus commit | CPU to RTL | EKF result, grouped PID gains, calibration overrides |
| active/pending banks | CPU to RTL | mode changes that must apply on a control boundary |

The CPU should see either the old complete value or the new complete value,
never a partially updated structure.

## 5. Address Map

Reserve a 64 KiB AXI aperture for `flight_controller_ip`. That is larger than
needed now but makes the map stable as features are added.

| Offset range | Block | Purpose |
|---:|---|---|
| `0x0000-0x00ff` | Global | ID, version, control, IRQ, build status |
| `0x0100-0x01ff` | IMU snapshot | MPU9250 raw/sample data and timing |
| `0x0200-0x02ff` | Calibration | Bias, scale, calibration control/status |
| `0x0300-0x03ff` | Filter | PT1 coefficients and filtered samples |
| `0x0400-0x04ff` | Estimator | RTL estimator config and CPU EKF mailbox |
| `0x0500-0x05ff` | PID | Gains, limits, integrator control, saturation |
| `0x0600-0x06ff` | Mixer/output | Mixer limits, motor values, DSHOT/PWM mode |
| `0x0700-0x07ff` | RC input | SBUS channels and link status |
| `0x0800-0x08ff` | Barometer | BMP280 raw data, trim data, status |
| `0x0900-0x09ff` | Magnetometer | AK8963 raw data, ASA data, status |
| `0x0a00-0x0aff` | Failsafe | Arming, watchdog thresholds, fault status |
| `0x0b00-0x0fff` | Reserved | Future expansion |
| `0x1000-0xffff` | Debug/log | Optional trace windows, counters, test hooks |

Reserved registers read as zero and ignore writes. Do not reuse a published
offset for a different meaning; add a new register and leave the old one
reserved or deprecated.

## 6. Global Registers

| Offset | Name | Access | Reset | Description |
|---:|---|---|---:|---|
| `0x0000` | `FC_ID` | RO | `0x46433031` | ASCII-like ID, `FC01` |
| `0x0004` | `FC_VERSION` | RO | project-defined | Major/minor/patch RTL version |
| `0x0008` | `FC_CAPS` | RO | bitfield | Implemented subcores and options |
| `0x000c` | `FC_CONTROL` | RW/WO | `0x00000000` | soft reset pulses, global enable bits |
| `0x0010` | `FC_STATUS` | RO | hardware | global ready/armed/failsafe state |
| `0x0014` | `IRQ_STATUS` | W1C | `0x00000000` | pending interrupt events |
| `0x0018` | `IRQ_ENABLE` | RW | `0x00000000` | interrupt mask |
| `0x001c` | `IRQ_SET` | WO | none | optional software test interrupt |
| `0x0020` | `TIME_LO` | RO | counter | free-running timestamp low word |
| `0x0024` | `TIME_HI` | RO | counter | free-running timestamp high word |
| `0x0028` | `BUILD_HASH` | RO | build-defined | short build/source identifier |

Suggested `IRQ_STATUS` bits:

| Bit | Event |
|---:|---|
| 0 | New IMU snapshot |
| 1 | New RC frame |
| 2 | Barometer sample ready |
| 3 | Magnetometer sample ready |
| 4 | PID/mixer saturation changed |
| 5 | Failsafe state changed |
| 6 | Sticky fault set |
| 7 | CPU mailbox accepted or rejected |

## 7. IMU Snapshot Block

The IMU block exports coherent frames. Hardware updates an internal live frame
at the sensor rate, then copies it into CPU-visible snapshot registers only at a
frame boundary.

| Offset | Name | Access | Description |
|---:|---|---|---|
| `0x0100` | `IMU_SNAPSHOT_CTRL` | RW/WO | bit 0 requests manual latch, bit 1 enables auto latch |
| `0x0104` | `IMU_SEQ` | RO | increments once per latched frame |
| `0x0108` | `IMU_TIME_LO` | RO | timestamp low word |
| `0x010c` | `IMU_TIME_HI` | RO | timestamp high word |
| `0x0110` | `ACCEL_XY_RAW` | RO | two signed 16-bit raw values |
| `0x0114` | `ACCEL_Z_TEMP_RAW` | RO | accel Z and temperature raw |
| `0x0118` | `GYRO_XY_RAW` | RO | two signed 16-bit raw values |
| `0x011c` | `GYRO_Z_RAW` | RO | signed 16-bit gyro Z |
| `0x0120` | `IMU_STATUS` | RO/W1C | valid, overrun, stale, bus error, ID mismatch |

Firmware read sequence:

```text
read IMU_SEQ as seq0
read all frame registers
read IMU_SEQ as seq1
accept frame only if seq0 == seq1 and valid is set
```

If auto latch is enabled and the CPU reads slower than the sensor rate, skipped
frames are detected by sequence gaps.

## 8. CPU EKF Mailbox

The EKF mailbox lets MicroBlaze V publish an attitude estimate without allowing
partial writes into the live control path.

| Offset | Name | Access | Description |
|---:|---|---|---|
| `0x0400` | `EST_MODE` | RW | 0 = RTL complementary, 1 = CPU EKF allowed |
| `0x0404` | `EST_STATUS` | RO/W1C | selected source, freshness, reject reason |
| `0x0410` | `CPU_EST_SEQ` | RW | firmware sequence number |
| `0x0414` | `CPU_EST_TIME_LO` | RW | estimate timestamp low word |
| `0x0418` | `CPU_EST_TIME_HI` | RW | estimate timestamp high word |
| `0x041c` | `CPU_QW` | RW | attitude quaternion W, Q1.30 or Q2.30 |
| `0x0420` | `CPU_QX` | RW | attitude quaternion X |
| `0x0424` | `CPU_QY` | RW | attitude quaternion Y |
| `0x0428` | `CPU_QZ` | RW | attitude quaternion Z |
| `0x042c` | `CPU_BIAS_GX` | RW | optional gyro bias correction, Q16.16 |
| `0x0430` | `CPU_BIAS_GY` | RW | optional gyro bias correction, Q16.16 |
| `0x0434` | `CPU_BIAS_GZ` | RW | optional gyro bias correction, Q16.16 |
| `0x0438` | `CPU_EST_COMMIT` | WO | write key to atomically publish pending estimate |

`CPU_EST_COMMIT` should accept a fixed key such as `0x45535431` (`EST1`). On
commit, RTL validates the pending payload:

- timestamp is newer than the last accepted estimate;
- sequence number is monotonic;
- quaternion norm is within bounds;
- age is below the configured estimator timeout;
- no reserved control bits are set.

If validation fails, RTL keeps the previous selected estimate or falls back to
the complementary filter and sets a reject reason.

## 9. Configuration Commit Groups

Some CPU writes should apply only at a safe boundary. Use grouped commit
registers for these areas:

| Group | Example registers | Apply boundary |
|---|---|---|
| filter config | `alpha_gyro`, `alpha_accel`, enable bits | next sensor sample |
| PID gains | outer/inner Kp, Ki, Kd, limits | next `ctrl_update` when disarmed or when allowed |
| mixer/output | idle speed, max throttle, output mode | next motor frame while disarmed preferred |
| failsafe thresholds | timeouts, battery limits, arming thresholds | immediately when disarmed; otherwise bounded update |

For high-risk settings, require an `override_enable` bit and reject writes while
armed unless a specific in-flight tuning bit is enabled.

## 10. Interrupt Strategy

Use one `irq` output from the flight controller IP into the AXI interrupt
controller. Inside the IP:

```text
irq = |(IRQ_STATUS & IRQ_ENABLE)
```

Interrupt handlers should be short. Firmware should read status, copy snapshot
data into software-owned buffers, clear `W1C` bits, and return. Telemetry and
logging should run outside the interrupt handler.

Recommended interrupt priorities:

| Priority | Event |
|---:|---|
| highest | failsafe state changed, sticky fault |
| high | new IMU snapshot if CPU EKF is enabled |
| medium | RC frame ready |
| low | barometer/magnetometer sample ready, telemetry events |

## 11. Clock, Reset, and CDC

The simplest first integration uses one 100 MHz clock for AXI and flight RTL.
If a subcore later needs a different clock, isolate the crossing at the subcore
boundary and keep AXI registers in the AXI clock domain.

Rules:

- synchronize external asynchronous inputs such as RC serial before decoding;
- use valid/ready, toggle, or small async FIFO crossings for clock-domain data;
- reset AXI-visible registers with `aresetn`;
- ensure motor outputs go inactive immediately on reset/failsafe;
- do not let a CPU soft reset bypass hard failsafe state.

## 12. Vivado Packaging Flow

1. Keep RTL sources in the repository under `cores/`.
2. Create `flight_controller_ip` as a top-level SystemVerilog wrapper.
3. Instantiate existing subcores inside the wrapper.
4. Instantiate `fc_axi_regs` and connect local config/status signals.
5. Package the wrapper as a Vivado IP with one AXI4-Lite slave interface.
6. In the block design, add MicroBlaze V, local memory, AXI interconnect, AXI
   interrupt controller, clock/reset, UART, and the packaged flight controller.
7. Assign a fixed base address, for example `0x4000_0000`.
8. Export the hardware platform for Vitis firmware.
9. Generate a C header from the register map and keep it versioned with the RTL.

The repository can still use FuseSoC and cocotb for RTL simulation. Vivado IP
packaging is an integration artifact; the core logic should remain simulator
friendly.

## 13. Firmware Driver Shape

Create a small driver layer instead of scattering raw addresses through the
application.

```c
#define FC_BASE              0x40000000u
#define FC_REG32(off)        (*(volatile uint32_t *)(FC_BASE + (off)))

#define FC_ID                0x0000u
#define FC_VERSION           0x0004u
#define FC_IRQ_STATUS        0x0014u
#define FC_IRQ_ENABLE        0x0018u
#define FC_IMU_SEQ           0x0104u
#define FC_CPU_EST_COMMIT    0x0438u
#define FC_CPU_EST_COMMIT_KEY 0x45535431u
```

Driver functions should cover:

- probe ID/version/capability registers;
- enable selected interrupts;
- read coherent IMU and RC snapshots;
- write grouped configuration with commit;
- publish CPU EKF estimates;
- read and clear sticky faults;
- force disarm through the documented software command path.

## 14. Verification Plan

Before relying on MicroBlaze V firmware, verify the register bank with cocotb or
a simple AXI bus functional model:

| Test | Expected result |
|---|---|
| reset defaults | all registers match documented values |
| invalid address | read returns zero or DECERR policy, write has no effect |
| byte strobes | unsupported partial writes are rejected or ignored consistently |
| W1C faults | writing one clears only selected bits |
| IMU snapshot | CPU never observes torn multiword frames |
| EKF commit | partial writes do not affect live estimator |
| EKF reject | stale/bad quaternion is rejected and status explains why |
| IRQ mask | `irq` follows `IRQ_STATUS & IRQ_ENABLE` |
| armed protection | dangerous config writes are blocked while armed |

Then verify in hardware:

1. bring up MicroBlaze V with UART and memory test only;
2. read `FC_ID`, `FC_VERSION`, and `FC_STATUS`;
3. enable IMU interrupts and print sequence/timestamp gaps;
4. tune/read registers while disarmed;
5. verify failsafe and software disarm with motors disconnected;
6. test motor output with props removed;
7. enable CPU EKF mailbox only after RTL complementary mode is stable.

## 15. Implementation Order

1. Implement `fc_axi_regs` with global ID, version, status, IRQ, and a small
   scratch register.
2. Package `flight_controller_ip` and confirm MicroBlaze V can read/write it.
3. Add IMU snapshot registers and interrupt.
4. Add failsafe status and software disarm.
5. Add filter/PID/mixer configuration groups with reset defaults.
6. Add RC, barometer, and magnetometer snapshots.
7. Add CPU EKF mailbox and estimator mux.
8. Add generated C header and driver tests.

This order gives early proof that the CPU bus works while keeping the
safety-critical RTL control path independent.
