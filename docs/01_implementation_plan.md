# Flight Controller Implementation Plan

## 1. Execution Rules

Implementation is divided into small, independently testable phases. A phase is
complete only after its documented acceptance gate passes. Later phases may not
be used to hide failures in an earlier phase.

Every RTL phase runs this quality gate:

1. Slang directly parses, elaborates, and lints all repository RTL, with package
   files ordered before other compilation units.
2. FuseSoC resolves the selected simulation core and dependency source set.
3. FuseSoC builds the Verilator simulation model for Cocotb with project
   warnings treated as errors.
4. Focused Cocotb simulation tests run against that model.
5. Full Verilator+Cocotb regression.
6. Vivado synthesis check when FPGA-specific resources or interfaces change.
7. Dedicated functional documentation and requirements update.

Every new functional core must have a document under `docs/` before its phase
can be marked complete. The document must describe its purpose, external
interface, operating behavior, reset and timing semantics, configuration,
status and error handling, safety-relevant behavior, and verification coverage.
Register-mapped cores must also document every register and atomic-access rule.

## Phase Status

This table is the persistent cross-session implementation tracker. Update it
when a phase starts, becomes blocked, or passes its acceptance gate. Only one
phase may be `In progress` at a time.

| Phase | Scope | Status | Evidence |
| ---: | --- | --- | --- |
| 0 | Specifications | Complete | Architecture, pinout, verification, and implementation documents approved |
| 1 | Build and verification infrastructure | Complete | `make lint` clean; `make sim` passes 2/2 clock-wrapper tests; failure propagation verified |
| 2 | Clock, reset, and timebase | Complete | Slang clean; 10/10 Cocotb tests pass; all cores synthesize; clock constraints verified |
| 3 | AXI4-Lite register core | Complete | Slang clean; 17/17 full-regression tests pass; Vivado synthesis clean |
| 4 | I2C master | Complete | Slang clean; 30/30 full-regression tests pass; Vivado synthesis clean |
| 5 | GY-91 initialization | Complete | Slang clean; 40/40 full-regression tests pass; integrated synthesis clean |
| 6 | Sensor scheduler and snapshots | Not started | - |
| 7 | Calibration and filtering | Not started | - |
| 8 | CRSF receiver | Not started | - |
| 9 | DShot600 output | Not started | - |
| 10 | Arming and failsafe | Not started | - |
| 11 | Rate PID and mixer | Not started | - |
| 12 | RTL attitude fallback | Not started | - |
| 13 | Angle control and mode arbitration | Not started | - |
| 14 | MicroBlaze V integration | Not started | - |
| 15 | 9-axis and altitude firmware | Not started | - |
| 16 | Full-system simulation | Not started | - |
| 17 | FPGA and bench validation | Not started | - |
| 18 | Incremental flight test | Not started | - |

## 2. Phase 0: Specifications

Deliverables:

- System architecture and hardware/software partition.
- Pin assignment and electrical-interface specification.
- Verification strategy.
- Clock, reset, sample-rate, fixed-point, and safety conventions.
- AXI register map and atomic-access rules.
- Requirements-to-test traceability table.

Acceptance gate:

- Every planned module has defined inputs, outputs, reset behavior, timing, and
  fault behavior.
- No interface decision blocks RTL implementation.

## 3. Phase 1: Build and Verification Infrastructure

Current infrastructure baseline:

- Top-level `Makefile` provides `clean`, `lint`, `sim`, `synth`, `cores`, and
  `versions` targets plus hyphenated aliases.
- FuseSoC 2.4.6 and Cocotb 2.0.1 are installed user-wide with `pipx` using
  Python 3.13.
- Edalize 0.6.1 is pinned for FuseSoC 2.4.6 tool-backend compatibility.
- `make lint` invokes Slang directly and never invokes FuseSoC or Verilator.
- `make sim` is the only Make target that invokes Verilator and requires
  Cocotb.
- The obsolete blinky and demonstration `cmod_a7_top` cores are removed.
- `clk_gen` is the temporary default core until the new flight-controller
  integration core exists.

Deliverables:

- Repaired FuseSoC dependency graph.
- Direct repository-wide Slang syntax and lint target.
- Verilator+Cocotb simulation target.
- Shared Cocotb utilities and Python reference-model layout.
- Xilinx primitive simulation stubs or wrappers.
- Repeatable local and CI-compatible commands.

Acceptance gate:

- Repository RTL passes direct Slang syntax, elaboration, and lint.
- A small RTL core passes a separate Verilator+Cocotb simulation.
- Cocotb tests execute consistently through the project flow.
- Tool failures return nonzero status.
- Build artifacts remain outside source directories.

Completion evidence, 2026-08-15:

- `make lint` completed with zero Slang errors and zero warnings.
- `make sim` used FuseSoC-resolved sources and passed both `clk_gen` Cocotb
  tests under Verilator.
- The simulation verifies MMCM lock/reset behavior and the modeled 100 MHz
  output period.
- A deliberately missing Cocotb module returned a nonzero Make status,
  confirming failure propagation.
- Xilinx `MMCME2_BASE` and `BUFG` simulation stubs are isolated in the
  `clk_gen` simulation fileset and are not included in synthesis RTL.
- Shared verification and reference-model directories exist under `tests/`.
- Generated EDAM, Verilator, Cocotb, and JUnit artifacts remain under `build/`.

## 4. Phase 2: Clock, Reset, and Timebase

Implemented contracts:

- `reset_sync` asynchronously asserts active-low reset and releases it after
  three rising clock edges by default. Synchronizer registers carry Vivado
  `ASYNC_REG` and `SHREG_EXTRACT=NO` attributes.
- `fc_clock_reset` converts 12 MHz to 100 MHz, requires 16 stable output-clock
  cycles after MMCM lock, then releases reset through `reset_sync`.
- External reset or MMCM lock loss asynchronously removes `clock_locked` and
  asserts system reset without waiting for an output-clock edge.
- `fc_timebase` publishes a 64-bit 100 MHz cycle timestamp. One count equals
  10 ns and natural rollover occurs after approximately 5,849 years.
- `rate_enable` emits one-cycle enables. The production timebase provides exact
  1 kHz, 250 Hz, 100 Hz, 50 Hz, and 10 Hz enables from 100 MHz.
- All enable counters and the timestamp reset to a common phase. The first pulse
  occurs exactly one configured interval after reset release.

Deliverables:

- 12 MHz to 100 MHz Cmod clock wrapper.
- Asynchronous reset assertion and synchronous release.
- Reset qualification after MMCM lock.
- 64-bit timestamp.
- 1 kHz, 250 Hz, 100 Hz, 50 Hz, and 10 Hz enables.
- Safe output behavior during reset and loss of lock.

Tests:

- Reset assertion at arbitrary clock phases.
- MMCM lock acquisition and loss.
- Exact enable periods and phase relationships.
- Timestamp rollover.
- Safe startup outputs.

Acceptance gate:

- All timing tests pass.
- Vivado recognizes generated clocks and reset paths correctly.

Completion evidence, 2026-08-15:

- `make lint` completed with zero Slang errors and zero warnings.
- `make regress` passed 10 of 10 Cocotb tests across `clk_gen`, `reset_sync`,
  `timebase`, and `clock_reset`.
- Reset tests cover asynchronous assertion between clock edges, three-stage
  synchronous release, short asynchronous pulses, lock qualification, and
  relocking.
- Timebase tests cover exact enable intervals, common reset phase, 8-bit
  accelerated rollover, and timestamp reset behavior.
- `reset_sync`, `timebase`, and `clock_reset` synthesized successfully for
  `xc7a35tcpg236-1` with zero errors and zero critical warnings.
- The only synthesis warning was Vivado's benign parallel-synthesis criterion
  notice for these small designs.
- `make check-clock-reset` verified an 83.333 ns primary clock, inferred 10.000
  ns MMCM output clock, and the asynchronous `ext_reset` false-path exception.
- Test-only Xilinx primitive models reside in the `xilinx_7series_sim` FuseSoC
  core and cannot enter synthesis targets.

## 5. Phase 3: AXI4-Lite Register Core

Deliverables:

- Protocol-correct 32-bit AXI4-Lite slave.
- ID, version, scratch, time, status, and interrupt registers.
- Independent AW and W handling.
- Byte strobes and invalid-address `SLVERR` responses.
- Reusable coherent-snapshot and shadow-plus-commit logic.
- Flattened Vivado-compatible wrapper.

Tests:

- AW before W, W before AW, and simultaneous arrival.
- Arbitrary master and slave backpressure.
- Partial writes and invalid alignment.
- Reset during every transaction phase.
- Atomic multiword snapshots and commits.

Acceptance gate:

- Randomized Cocotb AXI tests pass.
- No AXI register directly asserts motor authorization.

Completion evidence, 2026-08-15:

- `make lint` completed with zero Slang errors and zero warnings.
- `make regress` passed 17 of 17 Cocotb tests, including all seven AXI register
  tests and the complete Phase 1-2 regression.
- Directed AXI tests cover independent write-channel ordering, read and write
  backpressure, byte strobes, invalid and misaligned accesses, coherent
  snapshots, atomic commits, sticky masked interrupts, and transaction resets.
- Randomized testing covers 100 writes with varied AW/W ordering, byte strobes,
  values, and response backpressure.
- `fc_axi_regs` synthesized successfully for `xc7a35tcpg236-1` with zero errors
  and zero critical warnings.
- Reviewed synthesis warnings are limited to intentionally ignored AXI
  `AWPROT`/`ARPROT` attributes and Vivado's benign parallel-synthesis criterion
  notice for the small standalone design.
- The register map exposes configuration data only through shadow and active
  outputs; it provides no motor-authorization register or output.

## 6. Phase 4: I2C Master

Deliverables:

- Open-drain SDA and SCL control.
- START, repeated START, STOP, read, write, ACK, and NACK handling.
- 100 kHz and 400 kHz timing.
- Clock-stretch and actual-line-state sampling.
- Transaction timeout and stuck-bus detection.
- Nine-clock bus recovery.

Tests:

- Behavioral I2C slaves with randomized response delays.
- NACK at every byte position.
- Stuck SDA/SCL and clock stretching.
- Repeated START and multi-byte transfers.
- Timeout and recovery from every state.

Acceptance gate:

- FPGA outputs never actively drive an I2C line high.
- All recovery paths terminate in a bounded time.

Completion evidence, 2026-08-15:

- `make lint` completed with zero Slang errors and zero warnings.
- `make regress` passed 30 of 30 Cocotb tests, including all 13 I2C bus-level
  tests and the complete Phase 1-3 regression.
- The I2C tests use resolved wired-AND lines and an edge-driven behavioral
  target; they cover writes, reads, repeated START, stream backpressure,
  standard/fast timing, randomized stretching, NACK positions, stuck lines,
  exact nine-clock recovery, bounded timeouts, and reset interruption.
- Production defaults meet 100 kHz standard-mode and 400 kHz fast-mode timing,
  including their respective 4.70 us and 1.30 us bus-free minima.
- `i2c_master` synthesized successfully for `xc7a35tcpg236-1` with zero errors
  and zero critical warnings. The only synthesis warning was Vivado's benign
  parallel-synthesis criterion notice for the small standalone design.
- SDA and SCL are represented only by resolved inputs and drive-low outputs;
  the core has no signal capable of actively driving either line high.
- All SCL-high waits, stream stalls, and recovery sequences have explicit
  finite bounds, as documented in `docs/05_i2c_master.md`.

## 7. Phase 5: GY-91 Initialization

Deliverables:

- MPU address probing and identity verification.
- MPU reset, startup waits, configuration, and readback.
- AK8963 bypass, identity, factory adjustment, and continuous-mode setup.
- BMP address probing, identity, reset, calibration-byte read, and setup.
- Per-device state, health, and error counters.

Tests:

- Both possible MPU and BMP addresses.
- Missing and incorrect devices.
- Delayed startup and failed readback.
- I2C error during every initialization state.
- Bounded retries and reinitialization.

Acceptance gate:

- Ready asserts only after required configuration is read back correctly.
- Initialization cannot hang indefinitely.

Completion evidence, 2026-08-15:

- `make lint` completed with zero Slang errors and zero warnings.
- `make regress` passed 40 of 40 Cocotb tests, including all ten integrated
  GY-91 subsystem tests and the complete Phase 1-4 regression.
- The subsystem regression connects the initializer and register adapter to the
  real I2C master and register-accurate MPU-9250, AK8963, and BMP280 models over
  resolved open-drain bus lines.
- Tests verify the exact 42-transaction nominal sequence, alternate addresses,
  reset delays, configuration values and masks, factory-data packing, delayed
  BMP status, dependency handling, and explicit reinitialization.
- A transient address NACK is injected independently at every nominal
  transaction position; every operation retries identically and initialization
  still succeeds. Exhausted retries and semantic failures are checked for all
  three device paths.
- `gy91_init_subsystem`, including the I2C master and register adapter,
  synthesized successfully for `xc7a35tcpg236-1` with zero errors and zero
  critical warnings.
- Reviewed synthesis warnings are limited to the intentionally constant
  standard-mode selection, removal of an unreachable adapter state bit, and
  Vivado's benign small-design parallel-synthesis notice.
- All waits, polls, I2C operations, and retries have finite limits; `ready`
  remains low until every required identity and masked readback passes.
- The full behavior and diagnostics are documented in
  `docs/06_gy91_initialization.md`.

## 8. Phase 6: Sensor Scheduler and Snapshots

Deliverables:

- 1 kHz MPU, 100 Hz AK8963, and 50 Hz BMP280 scheduling.
- Priority arbitration favoring inertial acquisition.
- Timestamped, sequenced samples.
- Data-ready checking and bounded MPU retry.
- Coherent CPU snapshots and a small sensor FIFO.
- Freshness, missed-sample, and bus-utilization counters.

Tests:

- Long-run sample-rate accuracy.
- Bus contention and retry behavior.
- Duplicate, missed, stale, and overflowing samples.
- Pressure/temperature atomicity.
- Snapshot and FIFO coherency under CPU backpressure.

Acceptance gate:

- MPU deadlines are met under maximum expected normal bus load.
- Torn sensor snapshots are impossible.

## 9. Phase 7: Calibration and Filtering

Deliverables:

- Signed sensor-axis permutation.
- Stationary detection and startup gyro bias estimation.
- Offset, scale, and matrix correction datapaths.
- Configurable gyro and accelerometer filters.
- Explicit fixed-point widths, rounding, and saturation.
- Atomic coefficient activation.

Tests:

- Bit-exact comparison with Python reference models.
- All axis permutations and signs.
- Saturation and rounding boundaries.
- Stationary and moving startup records.
- Invalid coefficients and commit timing.

Acceptance gate:

- RTL and golden model outputs are bit-identical.
- No arithmetic result wraps silently.

## 10. Phase 8: CRSF Receiver

Deliverables:

- 420 kbaud UART receive path with input synchronization.
- CRSF synchronization, length checking, and CRC-8.
- Channel and link-statistics decoding.
- Command normalization, deadbands, and switch qualification.
- Frame-age and link watchdogs.

Tests:

- Recorded and generated CRSF streams.
- Noise, truncation, bad CRC, bad length, and back-to-back frames.
- Link loss, recovery, and out-of-range commands.

Acceptance gate:

- Invalid frames never modify active commands.
- Link recovery never causes automatic arming.

## 11. Phase 9: DShot600 Output

Deliverables:

- Four parallel DShot encoders.
- Checksum generation and exact waveform timing.
- Continuous stop commands while disarmed.
- Configurable motor ordering.
- Optional DShot300 bring-up mode.
- Bidirectional I/O capability reserved for future telemetry.

Tests:

- All command values and checksums.
- Exact pulse widths and frame intervals.
- Four-channel concurrency.
- Reset and lock loss during every transmit state.
- Arm/disarm transitions.

Acceptance gate:

- Pre-arm outputs can only be inactive or valid stop commands.
- Logic-analyzer measurements agree with simulation.

## 12. Phase 10: Arming and Failsafe

Deliverables:

- Explicit arming/failsafe state machine.
- RC, IMU, estimator, CPU, clock, and internal watchdogs.
- External hardware-kill input.
- Sticky fault state and final motor-authorization gate.
- Deliberate arming sequence and prohibited automatic re-arm.

Tests:

- Every fault injected from every armed state.
- Simultaneous and rapidly changing faults.
- CPU, sensor, RC, clock, and external-kill failures.
- Illegal-state recovery and reset behavior.

Acceptance gate:

- Assertions demonstrate that motor authorization requires every mandatory
  safety condition.
- Firmware cannot clear a hard fault while armed.

## 13. Phase 11: Rate PID and Mixer

Deliverables:

- Three-axis 1 kHz rate PID.
- Derivative on measurement, filtering, feedforward, and anti-windup.
- Atomic gain updates and disarmed integrator behavior.
- X-quad mixer with configurable order and signs.
- Yaw-first desaturation, collective shifting, scaling, and final clamping.

Tests:

- Step, impulse, saturation, and windup recovery.
- Gain commits during operation.
- Mixer signs, ordering, and all desaturation combinations.
- Bit-exact Python reference comparison.

Acceptance gate:

- Complete rate-mode path operates without CPU participation.
- Numeric bounds are documented and tested.

## 14. Phase 12: RTL Attitude Fallback

Deliverables:

- Fixed-point 6-axis quaternion complementary/Mahony estimator.
- Conditional accelerometer correction.
- Quaternion normalization and `Q2.30` output.
- Roll/pitch extraction and estimator confidence.

Tests:

- Static orientations and constant-rate rotations.
- Acceleration disturbances and gyro bias.
- Quaternion norm and long-duration drift behavior.
- Recorded-data comparison against a floating-point model.

Acceptance gate:

- Roll and pitch remain bounded through CPU-estimator loss.
- Implausible acceleration is rejected from gravity correction.

## 15. Phase 13: Angle Control and Mode Arbitration

Deliverables:

- 250 Hz angle controller.
- Acro, stabilized, heading, altitude, and hover mode arbitration.
- Bounded rate/angle demands and slew limits.
- RTL/CPU estimator selection with freshness fallback.
- Bumpless transitions.

Tests:

- Every mode transition and estimator transition.
- Heading wraparound and extreme stick commands.
- Stale, corrupt, or discontinuous CPU estimator results.

Acceptance gate:

- CPU-estimator failure cannot interrupt the inner rate loop.
- Mode transitions stay within configured motor-command step limits.

## 16. Phase 14: MicroBlaze V Integration

Deliverables:

- Vivado block design with MicroBlaze V, BRAM, UART, interrupt controller, and
  AXI interconnect.
- Packaged flight-controller AXI IP.
- Firmware register driver, interrupt handling, estimator commit, and heartbeat.
- Native firmware unit tests where practical.

Tests:

- AXI access through MicroBlaze V.
- Interrupt delivery and acknowledgement.
- CPU stall, reset, heartbeat timeout, and partial mailbox writes.

Acceptance gate:

- Rate mode remains operational with MicroBlaze V held in reset.
- CPU behavior cannot violate RTL safety invariants.

## 17. Phase 15: 9-Axis and Altitude Firmware

Deliverables:

- 9-axis Mahony estimator with magnetic and acceleration rejection.
- Magnetometer calibration.
- Bosch BMP280 signed 64-bit compensation.
- Altitude and vertical-speed estimation.
- Heading, altitude, and climb-rate supervisory loops.
- Bounded mailbox outputs to RTL.

Tests:

- Native tests against double-precision models.
- Recorded sensor datasets.
- Magnetic interference, pressure drift, and acceleration disturbance.
- Missed deadlines and invalid output publication.

Acceptance gate:

- RTL rejects stale and implausible firmware results.
- Entry and exit from altitude hold are bumpless.

## 18. Phase 16: Full-System Simulation

Deliverables:

- Integrated Verilator+Cocotb environment.
- GY-91, CRSF, ESC, and simplified quadcopter plant models.
- Automated flight and fault scenarios.
- End-to-end latency and deadline reports.

Required scenarios:

- Arm, idle, takeoff, stabilize, and disarm.
- Acro, angle, heading, altitude, and hover commands.
- RC loss, CPU failure, I2C faults, sensor freeze, and excessive noise.
- Motor saturation, reset, and clock failure.

Acceptance gate:

- Every safety requirement has an automated passing test.
- Sensor-to-DShot latency meets its budget.
- Regression runs deterministically.

## 19. Phase 17: FPGA and Bench Validation

Deliverables:

- Vivado implementation and timing reports.
- CDC and unconstrained-path review.
- Hardware I2C scan and sensor initialization.
- CRSF capture and DShot logic-analyzer measurements.
- Tests with ESCs and then motors, both without propellers.
- Restrained controller tests.

Acceptance gate:

- 100 MHz timing closes without unconstrained paths.
- External voltages and bus rise times are measured.
- Hardware safety behavior agrees with simulation.

## 20. Phase 18: Incremental Flight Test

Flight testing proceeds in this order:

1. Confirm motor order and direction without propellers.
2. Perform restrained low-throttle tests.
3. Perform a short rate-mode hop.
4. Tune and validate the rate loop.
5. Test angle mode.
6. Test heading hold.
7. Test altitude hold.
8. Test combined hover mode.

Logs and safety behavior must be reviewed before moving to the next step.
