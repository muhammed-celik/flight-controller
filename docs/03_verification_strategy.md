# Flight Controller Verification Strategy

## 1. Required Tools

The required RTL verification stack is:

- Slang for SystemVerilog syntax, parsing, elaboration, and lint.
- Verilator as the RTL simulation backend.
- Cocotb for directed, randomized, and integration testbenches.
- FuseSoC for source manifests, core dependencies, and build targets.
- Vivado for FPGA synthesis, implementation, timing, CDC review, IP packaging,
  and MicroBlaze V integration.

Icarus Verilog is not a required or authoritative simulation backend.

The repository-level commands are:

```text
make lint    direct Slang invocation over repository RTL
make sim     FuseSoC Verilator backend with Cocotb tests
make synth   FuseSoC Vivado backend
make clean   remove generated build artifacts
```

FuseSoC and Verilator must not be used by the lint target. Cocotb must be
present whenever the Verilator simulation target is run.

## 2. RTL Quality Gate

Every RTL change must pass, in order:

```text
Slang syntax/elaboration
        |
        v
Slang lint
        |
        v
warning-clean Verilator+Cocotb focused simulation
        |
        v
full Cocotb regression
        |
        v
Vivado checks when FPGA-specific behavior changed
```

Warnings selected by the project policy are treated as errors. Suppressions
must be local, justified, and documented. A suppression may not hide width,
signedness, latch, incomplete-case, multiple-driver, or clock/reset problems.

## 3. Source Organization

Portable RTL and board-specific RTL are tested separately:

- Portable cores contain no Xilinx primitive dependencies.
- The Cmod wrapper contains MMCM, buffer, I/O, and XADC primitives.
- Simulation wrappers provide deterministic models for board primitives.
- Synthesis builds use the actual Xilinx primitives.

Slang directly receives all Verilog and SystemVerilog sources under `cores/`,
with package files ordered first. FuseSoC resolves source metadata separately
for simulation and synthesis. Verilator is selected only for Cocotb simulation.

## 4. Test Levels

### 4.1 Unit Tests

Each module has focused Cocotb tests for:

- Reset from every externally reachable state.
- Nominal operation.
- Minimum and maximum legal values.
- Invalid input and protocol behavior.
- Timeouts and backpressure.
- Numeric saturation and rounding.
- State and output stability when idle.

### 4.2 Subsystem Tests

Subsystem regressions cover:

- I2C master plus GY-91 controllers and scheduler.
- CRSF UART plus parser, command mapping, and link watchdog.
- Sensor calibration, filtering, estimation, and control.
- PID, mixer, safety gate, and four DShot channels.
- AXI registers, snapshots, mailboxes, and interrupts.

### 4.3 Full-System Tests

The complete portable flight-controller core runs under Verilator with Cocotb
models for sensors, radio, ESCs, CPU mailbox activity, and a simplified vehicle
plant. Board primitives are not required for the portable full-system test.

## 5. Behavioral Models

Python/Cocotb models provide:

- MPU-9250 register behavior and inertial sample generation.
- AK8963 factory data, continuous samples, overflow, and stale behavior.
- BMP280 calibration data, raw conversions, and status behavior.
- I2C ACK/NACK, clock stretch, stuck lines, and transaction logging.
- CRSF frames, CRC corruption, noise, and link loss.
- DShot waveform decoding and checksum verification.
- AXI4-Lite master behavior with randomized channel ordering/backpressure.
- Simplified rigid-body and motor response for closed-loop tests.

Sensor models must support fault injection at every transaction byte and state.

## 6. Numeric Reference Models

Python reference models are authoritative for fixed-point behavior. They specify:

- Input and output Q formats.
- Intermediate widths and signedness.
- Rounding rules.
- Saturation limits.
- Filter coefficient generation.
- PID and anti-windup order of operations.
- Quaternion normalization.
- Mixer and desaturation priority.

Tests compare RTL outputs bit-for-bit against these models. Floating-point
models additionally measure algorithmic error but do not replace bit-exact
checks.

## 7. Safety Properties

Assertions and directed fault tests must demonstrate:

1. Motors cannot receive non-stop commands unless the safety FSM is armed.
2. Reset, loss of clock lock, or external kill removes motor authorization.
3. CPU writes cannot directly enable motors or bypass a hard fault.
4. Stale MPU data removes motor authorization within its specified timeout.
5. Invalid CPU estimator or altitude results are never selected.
6. A partial multiword AXI write cannot activate configuration.
7. Link recovery cannot automatically re-arm the vehicle.
8. DShot outputs remain safe through reset from every transmitter state.
9. Arithmetic saturates rather than wrapping.
10. Illegal state-machine encodings transition to a safe state.

Where practical, safety-state and motor-gate properties should also be checked
with formal tools in addition to Slang/Verilator/Cocotb. Formal verification is
supplementary and does not replace the required simulation stack.

## 8. Protocol Coverage

### AXI4-Lite

- Independent AW and W arrival in every order.
- Backpressure on all response channels.
- Byte strobes, alignment, and invalid addresses.
- Reset during every channel phase.
- Snapshot coherency and commit atomicity.

### I2C

- START, repeated START, STOP, ACK, and NACK.
- 100 kHz and 400 kHz timing.
- Clock stretch, stuck lines, timeout, and recovery.
- NACK and timeout at every byte position.
- Verification that high level is always produced by releasing the line.

### CRSF

- Valid channel and link-statistics frames.
- Bad sync, length, type, CRC, and truncation.
- Noise, inter-byte gaps, and back-to-back frames.
- Link timeout and recovery.

### DShot

- Every command value and checksum.
- Pulse widths, frame spacing, and all four channels.
- Stop behavior, reset, and loss of clock lock.

## 9. Coverage and Traceability

Each requirement receives a stable identifier and links to one or more tests.
The regression reports at least:

- Test pass/fail status.
- Requirement coverage.
- State and transition coverage where available from Verilator.
- Fault-injection scenario coverage.
- Maximum observed transaction and control latency.
- Numeric maximum error against reference models.

A line-coverage percentage alone is not an acceptance criterion. Safety states,
fault transitions, arithmetic boundaries, and protocol error paths require
explicit tests.

## 10. Vivado Checks

Vivado is required after changes involving clocks, resets, Xilinx primitives,
physical I/O, AXI IP packaging, or the integrated top level. Required reports:

- Synthesis warnings and utilization.
- Setup and hold timing.
- Unconstrained paths.
- Clock interaction.
- CDC analysis.
- I/O standards and package-pin conflicts.
- DRC results.

The target is a warning-reviewed, timing-clean 100 MHz implementation. A build
with unexplained critical warnings or unconstrained timing paths does not pass.

## 11. Hardware Verification Sequence

1. Validate power rails and all external signal voltages.
2. Check I2C pull-ups and rise time at 100 kHz, then 400 kHz.
3. Verify sensor identity, configuration readback, and sample rates.
4. Capture CRSF frames and timeout behavior.
5. Capture DShot waveforms with no ESC connected.
6. Connect ESCs with motors disconnected or propellers removed.
7. Verify motor order and direction with propellers removed.
8. Inject RC, sensor, CPU, kill, reset, and clock faults.
9. Perform restrained closed-loop tests.
10. Begin incremental flight testing only after all preceding gates pass.

## 12. Completion Definition

A feature is complete only when:

- Its interface and behavior are documented.
- Slang syntax/elaboration and lint pass.
- The Verilator model compiles warning-clean as part of the Cocotb simulation
  target.
- Focused and full Verilator+Cocotb regressions pass.
- Fixed-point behavior matches its reference model where applicable.
- Fault and reset behavior are tested.
- Safety requirements remain satisfied.
- FPGA-specific changes pass the required Vivado checks.
