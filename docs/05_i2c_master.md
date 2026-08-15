# I2C Master

## Purpose

`i2c_master` is a single-controller, 7-bit-address I2C transaction engine. It
supports standard-mode and fast-mode timing, writes, reads, combined
write/repeated-START/read transfers, target clock stretching, bounded stream
stalls, and nine-clock bus recovery. The block exposes low-drive controls
rather than bidirectional pins so the board-level I/O implementation remains
technology-specific.

## Interface Contract

All interfaces are synchronous to `clk`; reset is active-low and synchronous.

| Port | Direction | Contract |
| --- | --- | --- |
| `clk` | input | Controller clock. Production defaults assume 100 MHz. |
| `rst_n` | input | Synchronous active-low reset. |
| `cmd_valid` | input | Presents a command. Hold command fields stable until accepted. |
| `cmd_ready` | output | High only while idle. A command transfers when `cmd_valid && cmd_ready`. |
| `cmd_address[6:0]` | input | Unshifted 7-bit target address. |
| `cmd_write_count[7:0]` | input | Number of write payload bytes. |
| `cmd_read_count[7:0]` | input | Number of read payload bytes. |
| `cmd_fast_mode` | input | Selects fast timing when one, standard timing when zero. |
| `tx_valid` | input | Presents the next write payload byte. |
| `tx_ready` | output | Requests one payload byte. Transfer occurs on `tx_valid && tx_ready`; hold `tx_data` stable until then. |
| `tx_data[7:0]` | input | Write payload, in transaction order. |
| `rx_valid` | output | Presents one received byte. Remains asserted, with stable `rx_data`, until accepted. |
| `rx_ready` | input | Accepts a byte on `rx_valid && rx_ready`. SCL is held low while backpressured. |
| `rx_data[7:0]` | output | Read payload, in transaction order. |
| `busy` | output | High from command acceptance through STOP/recovery completion. |
| `done` | output | One-`clk` completion pulse, including rejected commands and errors. |
| `error` | output | Result associated with the most recent `done`; cleared when a new command is accepted. |
| `error_code[3:0]` | output | Completion status described below. |
| `error_byte_index[7:0]` | output | Zero-based write payload index for data NACK; zero for all other errors. |
| `scl_i`, `sda_i` | input | Resolved bus levels after board-level open-drain combination. |
| `scl_drive_low`, `sda_drive_low` | output | Assert to pull the corresponding line low; deassert to release it. These are never active-high drives. |

A command with both counts zero is accepted and completes immediately with
`INVALID_COMMAND`; it produces no bus activity. TX bytes may be supplied only
when requested. RX backpressure is legal and holds SCL low, but both kinds of
stream stall remain subject to the transaction timeout.

## Bus Sequences

A write emits `START`, `{address, 0}`, each write byte, and `STOP`. A read emits
`START`, `{address, 1}`, the requested bytes, and `STOP`. For reads the master
ACKs every byte except the final byte, which it NACKs before STOP.

A command with both counts nonzero emits one atomic combined sequence:
`START`, `{address, 0}`, write bytes, repeated `START`, `{address, 1}`, read
bytes, and `STOP`. There is no intervening STOP or bus-free interval.

## Timing And Synchronization

Production defaults are 130/120 controller clocks low/high and 130 clocks bus
free in fast mode. Standard mode uses 500/500 clocks low/high and 470 clocks
bus free. At 100 MHz these are 1.30/1.20/1.30 us and 5.00/5.00/4.70 us,
respectively. Parameters are nonzero and elaboration fails if any timing or
timeout parameter is zero.

High-phase timing begins only after synchronized `scl_i` is observed high.
Consequently a target may hold SCL low after the master releases it; this adds
to the bus low time without shortening the programmed high time. SCL and SDA
each pass through a two-flop synchronizer before protocol decisions. This adds
latency to observed edges and must be included in system timing budgets.

At integration, implement wired-AND resolution independently for each line:

```systemverilog
assign i2c_scl = scl_drive_low ? 1'b0 : 1'bz;
assign i2c_sda = sda_drive_low ? 1'b0 : 1'bz;
assign scl_i = i2c_scl;
assign sda_i = i2c_sda;
```

External pull-ups are required. Do not convert the low-drive outputs into
push-pull outputs or drive either bus line high.

## Completion And Errors

| Code | Name | Meaning |
| --- | --- | --- |
| 0 | `NONE` | Successful transaction. |
| 1 | `INVALID_COMMAND` | Both byte counts were zero. |
| 2 | `ADDRESS_NACK` | Target NACKed either write or read address byte. |
| 3 | `DATA_NACK` | Target NACKed a write payload byte. |
| 4 | `STRETCH_TIMEOUT` | Resolved SCL failed to rise within the bounded wait. |
| 5 | `TRANSACTION_TIMEOUT` | Total transaction budget expired, including TX/RX stream stalls. |
| 6 | `RECOVERY_FAILED` | SDA remained low after recovery and its generated STOP. |

Address NACK and data NACK terminate with STOP. `error_byte_index` identifies
the zero-based write payload byte for `DATA_NACK`; an address NACK during the
read leg of a combined transfer still reports index zero. Stretch timeout is
bounded by `STRETCH_TIMEOUT_CYCLES`. The total command, including software
delays on TX and RX streams, is bounded by `TRANSACTION_TIMEOUT_CYCLES`.

If the bus is not free before START, the controller generates exactly nine
complete SCL recovery pulses, then generates STOP. If SDA releases, the
original command continues after a bus-free interval. If it remains low,
completion reports `RECOVERY_FAILED`. A transaction timeout also invokes the
nine-clock recovery/STOP sequence, then reports `TRANSACTION_TIMEOUT`; it does
not resume the command. A stuck-low SCL during normal operation or recovery
reports a bounded stretch timeout.

## Reset And Safety

Reset immediately returns the state machine and handshakes to idle on the next
`clk` edge, clears completion status and counters, and releases both bus lines.
No STOP is promised for a transaction interrupted by reset. System software
must treat the interrupted transfer as aborted; a subsequent command performs
normal bus-free qualification and recovery if a target still holds the bus.

The block does not implement multi-controller arbitration, 10-bit addressing,
general call policy, target mode, SMBus PEC, high-speed mode, or electrical
glitch filtering beyond input synchronization.

## Verification

The Cocotb regression resolves both lines as wired-AND buses and uses only
resolved bus edges to model a target. It checks idle/open-drain behavior,
invalid commands, timing-mode ratio and phase lengths, multi-byte write, read
backpressure and final NACK, combined repeated-START transfers, deterministic
random response delay/stretch, address and per-byte data NACKs, successful and
failed nine-clock recovery, stuck-SCL timeout, TX/RX stream transaction
timeouts, and reset during active transfer and recovery.
