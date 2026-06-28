# SPI Master Core Design Notes

This note describes the recommended SPI master structure for the flight
controller project. The goal is to keep sensor controllers simple by separating
raw SPI byte shifting from sensor/register transaction logic.

## Recommended Module Split

Use two SPI-facing modules:

```text
imu_controller.sv
  -> spi_master_controller.sv
    -> spi_master_driver.sv
      -> SCLK / MOSI / MISO / CS
```

`spi_master_driver.sv` is the low-level bus engine. It generates `SCLK`,
asserts/deasserts chip select, shifts bytes on `MOSI`, samples bytes from
`MISO`, handles CPOL/CPHA, and counts bytes. It does not know about registers,
read commands, dummy bytes, MPU9250, BMP280, or auto-increment.

`spi_master_controller.sv` is the transaction helper. It accepts register-like
requests from `imu_controller`, builds the required SPI byte stream, provides
dummy bytes during reads, ignores address-phase receive bytes, and returns only
payload bytes to the controller.

## Low-Level Driver Interface

Suggested driver interface:

```systemverilog
module spi_master_driver #(
  parameter int CS_CNT = 2
)(
  input  logic i_clk,
  input  logic i_rstn,

  input  logic i_start,
  output logic o_busy,
  output logic o_done,

  input  logic [$clog2(CS_CNT)-1:0] i_cs_sel,
  input  logic [15:0] i_clk_div,
  input  logic [1:0] i_mode,
  input  logic [7:0] i_nbytes,

  input  logic i_tx_valid,
  output logic o_tx_ready,
  input  logic [7:0] i_tx_data,

  output logic o_rx_valid,
  input  logic i_rx_ready,
  output logic [7:0] o_rx_data,

  output logic [CS_CNT-1:0] o_cs,
  output logic o_sclk,
  output logic o_mosi,
  input  logic i_miso
);
```

### Driver Control Ports

`i_start` is a one-cycle pulse that starts a complete SPI frame. When accepted,
the driver asserts the selected CS line and shifts `i_nbytes` bytes.

`o_busy` is high from the accepted `i_start` until the frame is complete and CS
has been released.

`o_done` is a one-cycle pulse after the final byte has been shifted and the
driver is returning to idle.

`i_cs_sel` selects which active-low chip-select output to assert.

`i_clk_div` is the number of system clock cycles per SPI half-period. With a
100 MHz system clock, `i_clk_div = 50` gives 1 MHz SPI and `i_clk_div = 5`
gives 10 MHz SPI.

`i_mode[1]` is CPOL and `i_mode[0]` is CPHA.

`i_nbytes` is the total number of bytes shifted while CS remains active. For a
one-byte register read this is normally 2: one address/command byte and one
dummy byte to clock in the response.

## Valid/Ready Handshake Timing

The TX and RX byte interfaces should use a normal synchronous valid/ready
contract:

```text
transfer happens on rising i_clk when valid == 1 and ready == 1
```

Both sides sample the handshake on the same rising edge of `i_clk`.

## TX Handshake

TX direction:

```text
controller -> driver
```

The driver asserts `o_tx_ready` when it can accept a new transmit byte. In a
simple non-buffered design, assert `o_tx_ready` when all of these are true:

```text
driver is inside an active frame
driver is not currently holding an unshifted TX byte
there are still bytes left to transmit
the next byte must be loaded before shifting can continue
```

The controller asserts `i_tx_valid` when `i_tx_data` contains a real byte for
the driver. It must hold `i_tx_valid` and `i_tx_data` stable until a handshake
occurs:

```text
i_tx_valid && o_tx_ready
```

After that rising edge, the controller may deassert `i_tx_valid`, change
`i_tx_data`, or present the next byte. If it already has the next byte ready,
it may keep `i_tx_valid` high and change `i_tx_data` on the cycle after the
accepted transfer.

### TX Timing Example

```text
clk edge       0    1    2    3    4
o_tx_ready    0    1    1    0    0
i_tx_valid    0    1    1    0    0
i_tx_data     --   A0   00   --   --

accepted            A0   00
```

At edge 1, byte `A0` is accepted. At edge 2, byte `00` is accepted.

If the driver is not ready, the controller waits:

```text
clk edge       0    1    2    3
o_tx_ready    0    0    1    0
i_tx_valid    1    1    1    0
i_tx_data     A0   A0   A0   --

accepted                 A0
```

The controller keeps the same byte stable until the ready cycle.

## RX Handshake

RX direction:

```text
driver -> controller
```

The driver asserts `o_rx_valid` when `o_rx_data` contains a completed received
byte. It must hold `o_rx_valid` and `o_rx_data` stable until the controller
accepts the byte:

```text
o_rx_valid && i_rx_ready
```

The controller asserts `i_rx_ready` when it can accept a received byte. If the
controller can always consume RX bytes immediately, it may tie `i_rx_ready` high.

This is the same contract as TX, only with the data direction reversed.

### RX Timing Example

```text
clk edge       0    1    2    3
o_rx_valid    0    1    1    0
i_rx_ready    0    0    1    1
o_rx_data     --   71   71   --

accepted                 71
```

The driver finishes receiving byte `71` at edge 1, but the controller is not
ready yet. The driver holds the byte stable until edge 2, where it is accepted.

## Backpressure Choice

There are two reasonable driver implementation choices.

### Simple Driver

The simple driver stalls SPI clocking whenever it needs a TX byte or has an RX
byte that has not been accepted.

This is easiest to implement and verify:

```text
if waiting for TX byte: hold SCLK idle, keep CS asserted
if waiting for RX acceptance: hold SCLK idle, keep CS asserted
```

This is acceptable for low-rate sensor configuration and startup traffic. It is
also fine for deterministic polling if the controller always keeps up.

### Buffered Driver

The buffered driver contains small TX/RX FIFOs. `o_tx_ready` means the TX FIFO
has space, and `o_rx_valid` means the RX FIFO is not empty. SCLK can continue
as long as the TX FIFO has data and the RX FIFO has space.

This is better for high-throughput streaming, but it is more logic than needed
for the first clean version.

For this project, start with the simple driver. Add FIFOs only if later timing
or throughput measurements show that they are needed.

## High-Level Controller Interface

Suggested register controller interface:

```systemverilog
module spi_master_controller #(
  parameter int CS_CNT = 2
)(
  input  logic i_clk,
  input  logic i_rstn,

  input  logic i_req_valid,
  output logic o_req_ready,

  input  logic [$clog2(CS_CNT)-1:0] i_req_device,
  input  logic i_req_write,
  input  logic [7:0] i_req_addr,
  input  logic [7:0] i_req_len,
  input  logic i_req_auto_inc,

  input  logic i_wdata_valid,
  output logic o_wdata_ready,
  input  logic [7:0] i_wdata,

  output logic o_rdata_valid,
  input  logic i_rdata_ready,
  output logic [7:0] o_rdata,

  output logic o_busy,
  output logic o_done,
  output logic o_error,

  output logic [CS_CNT-1:0] o_cs,
  output logic o_sclk,
  output logic o_mosi,
  input  logic i_miso
);
```

`i_req_valid` and `o_req_ready` form the command handshake. A request is
accepted on a rising clock edge where both are high.

`i_req_device` selects the target chip.

`i_req_write` selects direction: `0` for read and `1` for write.

`i_req_addr` is the register address before device-specific read/write bit
encoding.

`i_req_len` is the number of payload bytes, excluding the address/command byte.

`i_req_auto_inc` says whether this request assumes sequential register access.
Do not bake auto-increment into the low-level driver.

`i_wdata_valid`, `o_wdata_ready`, and `i_wdata` stream write payload bytes from
the sensor controller into the SPI register controller.

`o_rdata_valid`, `i_rdata_ready`, and `o_rdata` stream read payload bytes back
to the sensor controller. The register controller should hide protocol bytes
such as dummy reads during the address phase.

## Register Controller Behavior

For a one-byte read:

```text
request: read addr 0x75, len 1
driver nbytes: 2
TX byte 0: address with read bit applied
TX byte 1: dummy byte
RX byte 0: ignored
RX byte 1: returned on o_rdata
```

For a five-byte write:

```text
request: write addr 0x19, len 5
driver nbytes: 6
TX byte 0: address with write bit applied
TX byte 1..5: write payload bytes
RX bytes: ignored
```

For a burst read:

```text
request: read start addr, len N, auto_inc 1
driver nbytes: N + 1
TX byte 0: start address with read bit applied
TX byte 1..N: dummy bytes
RX byte 0: ignored
RX byte 1..N: returned as payload
```

For a device that does not support auto-increment, the register controller
should issue multiple single-register transactions or use a device-specific raw
frame mode if the device supports repeated address phases under one CS.

## Naming Guidance

Avoid using `o_valid` by itself because it is ambiguous. Prefer names that say
what is valid:

```text
o_rx_valid      received byte is valid
o_rdata_valid   register payload byte is valid
o_done          whole transaction is complete
o_busy          transaction is active
```

This keeps the IMU controller from confusing byte-level events with
transaction-level events.

## Practical Implementation Order

1. Implement `spi_master_driver.sv` with one-byte TX/RX valid-ready handshakes.
2. Verify mode 0 or mode 3 single-byte shifting with a small testbench.
3. Add `i_nbytes`, CS hold, and `o_done`.
4. Implement `spi_master_controller.sv` for single register read/write.
5. Add burst read/write support.
6. Update `imu_controller.sv` to request register operations instead of driving
   raw SPI bytes directly.

