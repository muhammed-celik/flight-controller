# GY-91 Initialization

## Purpose And Boundaries

`gy91_init` performs bounded boot-time discovery, reset, calibration capture,
configuration, and readback for the MPU-9250, its bypassed AK8963, and the
BMP280 on a GY-91 module. It emits register requests but does not implement I2C
waveforms. `i2c_register_master` converts each request into one atomic register
write or combined register-address/read transaction. `gy91_init_subsystem`
connects both blocks to `i2c_master` and exposes sampled SCL/SDA inputs and
open-drain drive-low outputs. The subsystem wrapper uses production defaults;
only `gy91_init_tb` uses accelerated verification timing.

The adapter latches a per-request mode selection with the other request fields.
The initializer ties standard mode; after initialization releases ownership
and the runtime path is reset/idle, the sensor scheduler takes ownership and
uses fast mode for every acquisition. See `07_sensor_scheduler.md` for the
handoff and runtime reset contract.

Acquisition, periodic scheduling, sample compensation, and CPU snapshots are
outside this block.

## Interface And Status Contract

All signals are synchronous to `clk`. `rst_n` is active-low and synchronous.
The standalone initializer request fields remain stable while `req_valid` is
waiting for `req_ready`. Read bytes transfer on `rx_valid && rx_ready` and carry
a zero-based `rx_index`. `rsp_done` completes a request; `rsp_error` and
`rsp_error_code` describe an I2C failure. The register adapter additionally
publishes the underlying I2C error-byte index for clients that require it.

| Signal | Contract |
| --- | --- |
| `reinitialize`, `reinitialize_ready` | A high request is accepted from either completed state when `reinitialize_ready` is high. It starts a fresh power-up delay. |
| `initializing` | High from reset/start through terminal success or failure. |
| `ready` | High only when MPU, AK, and BMP configuration and readback all succeeded. Never high during initialization. |
| `failed` | High after any terminal run that did not make all three devices ready. |
| `*_present` | Identity was observed at the selected address. Presence does not imply configuration success. |
| `*_ready` | Device calibration/configuration and required readback completed. |
| `*_failed` | That device path failed. MPU failure also marks AK failed because bypass access is unavailable. |
| `mpu_address`, `bmp_address` | Selected unshifted 7-bit address; zero until a valid identity is found. |
| `ak_asa[23:0]` | Three factory sensitivity bytes, byte 0 in bits 7:0. |
| `bmp_calibration[191:0]` | Twenty-four calibration bytes, register `0x88` in bits 7:0 through `0x9f` in bits 191:184. |
| `*_error_count` | Saturating 16-bit count of I2C attempts and semantic validation failures attributed to each device. Preserved by reinitialize. |
| `last_i2c_error` | Most recent I2C error code; cleared by reinitialize. |
| `last_failed_device` | 0 none, 1 MPU, 2 AK, 3 BMP. |
| `last_failed_step` | Numeric initialization step that most recently exhausted retries or failed validation. |
| `init_sequence` | Saturating 16-bit count of completed runs, successful or failed. |
| `scl_i`, `sda_i` | Resolved external bus levels on the subsystem wrapper. |
| `scl_drive_low`, `sda_drive_low` | Open-drain controls; one pulls low and zero releases. They never drive high. |

The subsystem's `monitor_cmd_*` outputs are non-functional observation points
for verification. `monitor_cmd_accepted` pulses with address, write count, read
count, and fast-mode selection when the real I2C master accepts a command.

## MPU-9250 Sequence

The initializer probes `0x68`, then `0x69`, by reading `WHO_AM_I` (`0x75`) and
requiring `0x71`. Each I2C failure is retried before moving to the alternate
address. It then performs:

1. Write `PWR_MGMT_1` `0x6b = 0x80`; wait `MPU_RESET_CYCLES`.
2. Read `WHO_AM_I` and require `0x71` after reset.
3. Write `PWR_MGMT_1 = 0x01` and `PWR_MGMT_2` `0x6c = 0x00`; wait `MPU_WAKE_CYCLES`.
4. Write `SMPLRT_DIV` `0x19 = 0x00`, `CONFIG` `0x1a = 0x02`, `GYRO_CONFIG` `0x1b = 0x18`, `ACCEL_CONFIG` `0x1c = 0x10`, and `ACCEL_CONFIG_2` `0x1d = 0x02`.
5. Write `USER_CTRL` `0x6a = 0x00` and `INT_PIN_CFG` `0x37 = 0x02` to disable the internal I2C master and enable bypass.
6. Read back identity, power, and all seven configuration registers.

Readback masks are respectively `ff, 7f, 3f, ff, 7f, fb, f8, 0f, 30, 02`
for registers `75, 6b, 6c, 19, 1a, 1b, 1c, 1d, 6a, 37`. This permits
documented reserved/self-changing bits while requiring every configured field.

## AK8963 Sequence

AK access depends on successful MPU bypass configuration. At fixed address
`0x0c`, the sequence is:

1. Write `CNTL2` `0x0b = 0x01`; wait `AK_MODE_CYCLES`.
2. Read `WIA` `0x00` and require `0x48`.
3. Write `CNTL1` `0x0a = 0x00`; wait, then write fuse-ROM mode `0x0f`; wait.
4. Read three ASA bytes from `0x10`.
5. Write power-down `0x00`; wait, then continuous-measurement-2, 16-bit mode `0x16`; wait.
6. Read `CNTL1` and require `(value & 0x1f) == 0x16`.

ASA is invalid if all three bytes are `0x00` or all are `0xff`. The bytes are
packed in ascending register order, so ASAX is `ak_asa[7:0]`, ASAY is
`[15:8]`, and ASAZ is `[23:16]`.

## BMP280 Sequence

The initializer probes `0x76`, then `0x77`, by reading `id` (`0xd0`) and
requiring `0x58`. It then performs:

1. Write software reset `reset` `0xe0 = 0xb6`.
2. Wait `BMP_POLL_INTERVAL_CYCLES`, then read `status` `0xf3` until bits `0x09` are both clear. Polling is bounded by `BMP_POLL_LIMIT`.
3. Re-read `id` and require `0x58` after reset.
4. Read exactly 24 bytes from `0x88` through `0x9f` in one combined transfer.
5. Write sleep configuration `ctrl_meas` `0xf4 = 0x2c`.
6. Write `config` `0xf5 = 0x08`; read it back and require `(value & 0xfd) == 0x08`.
7. Write normal mode `ctrl_meas = 0x2f`; read back exactly `0x2f`.

Calibration is packed little-endian by bus order. Thus `dig_T1` is
`bmp_calibration[15:0]` and `dig_P1` is `[63:48]`. The complete vector must not
be all zero or all one, and both unsigned divisor coefficients `dig_T1` and
`dig_P1` must be nonzero.

With temperature x1, pressure x4, and no standby (`config = 0x08`), normal mode
internally converts at approximately 83 Hz. Phase 6 deliberately schedules
pressure/temperature reads at 50 Hz; the slower scheduler consumes recent
completed conversions and leaves bus budget for the higher-rate inertial path.

## Retries, Dependencies, And Reset

An I2C error increments the attributed device counter and retries the identical
transaction after `RETRY_DELAY_CYCLES`. `MAX_TRANSACTION_RETRIES = 2` means an
initial attempt plus two retries. Probe exhaustion moves to the alternate MPU
or BMP address. Semantic failures are not retried and increment the same
counter once.

MPU failure prevents AK access, marks both paths failed, and continues with BMP.
AK failure continues with BMP. BMP failure terminates the run. A run is globally
ready only if all three paths are ready, even when a later independent device
succeeded.

Explicit reinitialize clears readiness, presence, addresses, calibration,
last-error metadata, and per-run state, but preserves error counters and
increments `init_sequence` only when the new run completes. Reset aborts delays
or active I2C traffic, releases both lines on the next clock, clears all status
and counters, and automatically starts initialization from the power-up delay.

## Runtime Handoff

The sensor scheduler owns periodic MPU/AK/BMP arbitration, 1 kHz/100 Hz/50 Hz scheduling,
data-ready policy, sample timestamps and sequence numbers, coherent snapshots,
FIFO behavior, freshness and missed-sample accounting, and runtime I2C fault
policy. This phase exports calibration bytes but does not compensate samples.

## Verification Coverage

The Cocotb subsystem regression uses a wired-AND resolver and edge-driven,
register-accurate MPU-9250, AK8963, and BMP280 models. It checks the exact
nominal write/read sequence and masks, both MPU/BMP addresses, bypass gating,
software-reset effects, fuse-ROM ASA access and packing, delayed BMP status,
24-byte calibration packing, readiness timing, and standard-mode-only commands.
It covers missing and wrong identities at both probes, every AK validity and
readback failure, BMP poll exhaustion/post-reset identity/calibration/config
failures, one transient NACK at every nominal transaction position with
same-command retry, per-device retry exhaustion and continuation, failed-run
repair and reinitialize persistence, and reset during both delay and active bus
traffic. Every wait is bounded and all register and data validation occurs on
actual resolved bus edges.
