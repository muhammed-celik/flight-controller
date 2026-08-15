# Sensor Scheduler

## Boundaries And Ownership

`sensor_scheduler` is the runtime GY-91 acquisition policy block. It consumes
one-cycle 1 kHz, 100 Hz, 50 Hz, and 10 Hz enables plus a modular 64-bit cycle
timestamp. It emits register transactions, validates complete sensor bursts,
formats records, maintains latest values and diagnostics, and feeds a sample
FIFO and coherent snapshot bank. It does not generate rate enables, initialize
devices, compensate raw values, or implement I2C waveforms.

`i2c_register_master` adapts a scheduler request to one atomic combined I2C
transaction. Its per-request `req_fast_mode` is latched with the other request
fields and handed to `i2c_master`; every runtime request selects fast mode.
Initialization owns the bus until all devices are ready and uses standard mode.
The integration contract is a break-before-make handoff: deassert runtime
`init_ready`, reset/idle the runtime adapter and master, switch ownership, then
assert `init_ready` with stable discovered MPU/BMP addresses.

All scheduler inputs and outputs are synchronous to `clk`. Reset is active-low
and synchronous. Read bytes are accepted on `rx_valid && rx_ready`; `rsp_done`
ends the transaction and `rsp_error` qualifies `rsp_error_code`.

## Scheduling And Bandwidth

The release mapping is MPU at 1 kHz, AK at 100 Hz, and BMP at 50 Hz. A release
sets one pending bit and captures its timestamp and monotonically wrapping
32-bit release sequence. There is no sample backlog. A release for a device
already active or pending is coalesced and increments both that device's missed
release and missed sample counters. Arbitration is strict priority MPU, AK,
then BMP whenever the scheduler returns idle. Thus an MPU arriving during a
lower-priority transaction runs next, while an active transfer is not preempted.

The MPU data-ready bit receives one special retry. A clear first status byte
waits 5,000 production clock cycles, then retries the same release at highest
priority before pending AK/BMP work. A second clear status completes as an
invalid/missed sample. Transport errors are not retried by this block.

Production deadlines measured from release through response completion are
1 ms MPU, 10 ms AK, and 20 ms BMP. At 400 kHz, combined transactions use about
18 byte-times per MPU burst, 11 per AK burst, and 9 per BMP burst, including
addresses and register. The budget is
`18*9*1000 + 11*9*100 + 9*9*50 = 175,950` SCL clocks/s, or 44.0% before
START/STOP and bus-free overhead. Measured master-busy utilization is expected
to be about 45-46%, leaving bounded priority/retry margin.
`bus_busy_cycles` measures actual master busy time; `bus_window_busy_cycles`
captures the preceding 10 Hz window.

## Runtime Bursts And Validity

| Device | Address/register/count | Bus order and acceptance |
| --- | --- | --- |
| MPU-9250 | discovered `mpu_address`, `INT_STATUS` `0x3a`, 15 bytes | Byte 0 bit 0 must be set. Bytes 1..14 are seven signed 16-bit big-endian accel, temperature, and gyro words retained in bus order. A clear bit gets one delayed retry. |
| AK8963 | fixed `0x0c`, `ST1` `0x02`, 8 bytes | ST1, six little-endian axis bytes, then ST2. Accept when DRDY=1, HOFL=0, and BITM=1. DOR is retained as a flag and counted as overrun/missed, but does not reject otherwise valid data. |
| BMP280 | discovered `bmp_address`, `press_msb` `0xf7`, 6 bytes | One atomic pressure/temperature burst. Each 20-bit value is MSB first with its low nibble in bits 7:4 of the third byte. Either `0x80000` sentinel rejects the burst. |

Every successful response must contain each zero-based byte index exactly once
and no out-of-range or duplicate index. A short, duplicate, or malformed burst
is invalid and cannot partially update latest state or the FIFO. BMP bytes are
buffered before either value is published, preventing half-old/half-new output.

## Record Format

All records are 256 bits. Unlisted bits are zero.

| Bits | Meaning |
| --- | --- |
| `1:0` | Type: 0 MPU, 1 AK, 2 BMP. |
| `2` | MPU retry result flag. |
| `3` | Data ready: MPU INT_STATUS.DRDY or AK ST1.DRDY. |
| `4` | AK ST1.DOR. |
| `5` | AK ST2.HOFL. |
| `6` | AK ST2.BITM. |
| `47:16` | Per-device accepted-sample sequence. |
| `111:48` | Completion timestamp. |
| `223:112` | Payload. MPU uses 14 bus-order bytes; AK uses six bus-order axis bytes; BMP uses six bytes with each low nibble cleared after extracting the 20-bit values. |
| `255:224` | Device release sequence associated with the attempt. |

Accepted-sample sequences increment only on a valid commit and wrap modulo 32
bits. Release sequences increment on every rate release, including coalesced
releases. Timestamp subtraction is modular 64-bit arithmetic, so freshness and
latency work across wrap for practical intervals below half the timestamp range.

## Freshness And Faults

`*_valid` means at least one sample committed since reset/readiness loss.
Freshness uses completion age and inclusive production thresholds: MPU 2 ms,
AK 30 ms, and BMP 60 ms. MPU is hard stale after more than 5 ms. Before the
first MPU commit, hard-stale age starts when runtime becomes ready, covering a
missing initial sample. Hard stale sets sticky `runtime_fault`.

Three consecutive MPU transport or semantic failures also set
`runtime_fault`. A valid MPU clears the internal consecutive count but not the
sticky fault. AK/BMP consecutive counts do not independently assert the global
fault. Reset or `init_ready` loss clears it.

## Diagnostics

All exposed counts are saturating 32-bit values unless noted otherwise.

| Counter/status | Definition |
| --- | --- |
| `*_release_count` | Rate-enable releases observed. |
| `*_accepted_sample_count` | Complete valid records committed. |
| `*_missed_release_count` | Releases coalesced while the device was active/pending. |
| `*_missed_sample_count` | Coalesced releases plus invalid/failed attempts; AK DOR additionally records the overwritten measurement. |
| `*_i2c_error_count` | Transport failures attributed per device. |
| `*_invalid_sample_count` | Responses rejected by byte-count or sensor validity rules. |
| `mpu_duplicate_poll_count`, `ak_duplicate_poll_count` | Complete bursts with data-ready clear. |
| `ak_overrun_count` | AK bursts with ST1.DOR set. |
| `mpu_retry_count` | First clear-DRDY polls entering delayed retry. |
| `*_max_latency_cycles` | Maximum final-attempt latency, saturated to 32 bits. |
| `deadline_miss_count` | Final attempts exceeding their device deadline. |
| `bus_transaction_count`, `bus_error_count` | Adapter-accepted requests and failed responses. |
| `bus_busy_cycles` | Saturating 64-bit clocks with the I2C master busy. |
| `bus_window_busy_cycles` | Busy clocks accumulated between 10 Hz enables. |
| `last_i2c_error` | Most recent failed transport error code. |

## FIFO And Snapshots

The production FIFO depth is 16 records and must be a power of two of at least
2. Valid commits always advance latest records even when full. Overflow uses
drop-newest: resident order/head remains unchanged, the sticky bit sets, and
the overflow count increments. Pop on empty increments underflow. Simultaneous
push/pop on a nonempty FIFO accepts both and holds level; it permits push when
full if the head is popped in the same cycle. Head is zero when empty and stable
until an accepted pop.

`snapshot_capture` copies `{bmp_latest_record, ak_latest_record,
mpu_latest_record}`, live status, and increments snapshot sequence. Status bits
0..2 are valid, 3..5 fresh, 6 MPU hard stale, 7 runtime fault, 8 FIFO empty, 9
FIFO full, and `31:16` FIFO level. Outputs remain stable while live records
advance or software drains the FIFO. Capture coincident with commit returns the
prior complete records because sequential updates consume prior-cycle values.

## Reset And Readiness

Reset and `init_ready=0` abort scheduling and clear active/pending requests,
latest valid state, sequences, FIFO/snapshot contents, diagnostics, stale state,
and runtime fault. Enables while not ready are ignored. Integration must reset
the adapter and I2C master at the same boundary so an old completion cannot
commit after readiness returns. Open-drain outputs then release both lines.

## Deferred Integration

AK ASA/BMP calibration application and physical-unit compensation remain
downstream work. AXI allocation, interrupt/watermark policy, DMA, and software
ownership of FIFO/snapshot controls are also deferred. They must preserve the
record map, drop-newest behavior, and coherent capture contract.

## Verification Coverage

The Cocotb regression instantiates the real scheduler, register adapter, and
I2C master on resolved open-drain lines. Its edge-driven model validates actual
address/register bytes, repeated START, read address, byte count, final master
NACK, and fast mode. Ten tests cover priority/packing; a 101-period scaled load
run; MPU retry; AK flags and BMP atomicity/sentinels; NACK, timeout, and sticky
fault; coalescing; FIFO corners; snapshots; freshness, timestamp wrap, and
readiness loss; and reset/recovery. Simulation scales timing uniformly by ten,
preserving production utilization and release-period ratios.
