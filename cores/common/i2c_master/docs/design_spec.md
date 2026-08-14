# I2C Master Controller — Design Specification

## 1. Overview

A feature-rich, parameterizable I2C Master controller supporting Standard, Fast, and Fast-Plus modes, 7/10-bit addressing, multi-master arbitration, clock stretching, repeated START, FIFO buffering, and SMBus-compatible timeout detection. Designed for integration via a register-based CPU/bus interface.

---

## 2. Feature Summary

| Feature | Description |
|---------|-------------|
| Speed Modes | Standard (100kHz), Fast (400kHz), Fast-Plus (1MHz) |
| Addressing | 7-bit and 10-bit slave addressing |
| Multi-Master | Arbitration detection and graceful loss handling |
| Clock Stretching | Detects slave-held SCL low, waits indefinitely or with timeout |
| Repeated START | Issue Sr without intervening STOP for combined transfers |
| General Call | Support for broadcast address 0x00 |
| TX/RX FIFOs | Configurable depth, independent TX and RX |
| DMA Interface | Handshake signals for TX/RX DMA channels |
| Interrupts | Transfer done, NACK, arbitration lost, bus error, FIFO events |
| Bus Timeout | Configurable SCL/SDA stuck-low detection (SMBus 25/35ms) |
| Bus Recovery | Automatic 9-clock SCL toggle to release stuck SDA |
| Byte/Burst Transfers | Single-byte or multi-byte with auto-increment |
| ACK Polling | Hardware retry on NACK for EEPROM-style devices |
| Spike Filter | Glitch suppression on SCL/SDA inputs |

---

## 3. Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `P_FIFO_DEPTH` | int | 16 | TX and RX FIFO depth (power of 2) |
| `P_CLK_FREQ_HZ` | int | 100_000_000 | System clock frequency |
| `P_DEFAULT_SCL_FREQ` | int | 400_000 | Default I2C clock frequency |
| `P_SPIKE_FILTER_NS` | int | 50 | Glitch filter width in nanoseconds |
| `P_TIMEOUT_EN` | bit | 1 | Enable bus timeout detection |

---

## 4. Port Interface

### 4.1 System Ports

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `clk` | in | 1 | System clock |
| `rst_n` | in | 1 | Active-low async reset |

### 4.2 Register/Bus Interface

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `reg_addr` | in | 4 | Register address |
| `reg_wdata` | in | 32 | Write data |
| `reg_wen` | in | 1 | Write enable |
| `reg_ren` | in | 1 | Read enable |
| `reg_rdata` | out | 32 | Read data |
| `reg_ready` | out | 1 | Access acknowledge |

### 4.3 I2C Pad Interface (Open-Drain)

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `scl_i` | in | 1 | SCL input (from pad) |
| `scl_o` | out | 1 | SCL output drive (active-low, 0=drive low) |
| `scl_oe` | out | 1 | SCL output enable (1=driving) |
| `sda_i` | in | 1 | SDA input (from pad) |
| `sda_o` | out | 1 | SDA output drive (active-low, 0=drive low) |
| `sda_oe` | out | 1 | SDA output enable (1=driving) |

### 4.4 Interrupt and DMA

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `irq` | out | 1 | Combined interrupt output |
| `dma_tx_req` | out | 1 | TX DMA request (FIFO below watermark) |
| `dma_tx_ack` | in | 1 | TX DMA acknowledge |
| `dma_rx_req` | out | 1 | RX DMA request (FIFO above watermark) |
| `dma_rx_ack` | in | 1 | RX DMA acknowledge |

---

## 5. Register Map

| Addr | Name | R/W | Bits | Description |
|------|------|-----|------|-------------|
| 0x0 | `CTRL` | RW | [0] I2C_EN — module enable | |
| | | | [1] START — issue START/repeated-START | |
| | | | [2] STOP — issue STOP after current byte | |
| | | | [3] RD_WR_N — 0:write, 1:read | |
| | | | [4] ACK_GEN — 0:send ACK, 1:send NACK (master-rx) | |
| | | | [5] ADDR_10BIT — 1:10-bit addressing mode | |
| | | | [6] GENERAL_CALL — 1:use general call address | |
| | | | [7] ACK_POLL_EN — 1:auto-retry on NACK | |
| | | | [8] BUS_RECOVER — 1:initiate bus recovery sequence | |
| | | | [10:9] SPEED — 0:Standard, 1:Fast, 2:Fast-Plus | |
| 0x1 | `CLKDIV` | RW | [15:0] DIV_HIGH — SCL high period in sys clocks | |
| | | | [31:16] DIV_LOW — SCL low period in sys clocks | |
| 0x2 | `SLAVE_ADDR` | RW | [9:0] ADDR — target slave address (7 or 10 bit) | |
| 0x3 | `TXDATA` | W | [7:0] — Write pushes byte to TX FIFO | |
| 0x4 | `RXDATA` | R | [7:0] — Read pops byte from RX FIFO | |
| 0x5 | `XFER_LEN` | RW | [15:0] — Number of bytes to transfer (0=single) | |
| 0x6 | `STATUS` | R | [0] TX_FULL | |
| | | | [1] TX_EMPTY | |
| | | | [2] RX_FULL | |
| | | | [3] RX_EMPTY | |
| | | | [4] BUSY — transfer in progress | |
| | | | [5] ARB_LOST — arbitration was lost | |
| | | | [6] NACK_RCVD — slave sent NACK | |
| | | | [7] BUS_BUSY — SDA/SCL indicate bus in use | |
| | | | [8] BUS_ERROR — invalid START/STOP detected | |
| | | | [11:9] TX_LEVEL — TX FIFO occupancy | |
| | | | [14:12] RX_LEVEL — RX FIFO occupancy | |
| 0x7 | `IRQ_EN` | RW | [0] XFER_DONE_IE | |
| | | | [1] NACK_IE | |
| | | | [2] ARB_LOST_IE | |
| | | | [3] BUS_ERROR_IE | |
| | | | [4] RX_FULL_IE | |
| | | | [5] TX_EMPTY_IE | |
| | | | [6] TIMEOUT_IE | |
| | | | [7] RX_OVERRUN_IE | |
| | | | [8] TX_UNDERRUN_IE | |
| 0x8 | `IRQ_STATUS` | R/W1C | Same bit positions as IRQ_EN | |
| 0x9 | `TIMEOUT_CFG` | RW | [15:0] SCL_TIMEOUT — sys clocks before SCL stuck-low (0=disabled) | |
| | | | [31:16] SDA_TIMEOUT — sys clocks before SDA stuck-low (0=disabled) | |
| 0xA | `ACK_POLL_CFG` | RW | [7:0] MAX_RETRIES — max NACK retries before giving up | |
| | | | [23:8] RETRY_INTERVAL — sys clocks between retries | |
| 0xB | `DMA_CTRL` | RW | [0] DMA_TX_EN | |
| | | | [1] DMA_RX_EN | |
| | | | [5:2] TX_WATERMARK | |
| | | | [9:6] RX_WATERMARK | |
| 0xC | `SETUP_HOLD` | RW | [7:0] SDA_SETUP — SDA setup time in sys clocks | |
| | | | [15:8] SDA_HOLD — SDA hold time in sys clocks | |
| | | | [23:16] START_HOLD — START condition hold in sys clocks | |
| | | | [31:24] STOP_SETUP — STOP condition setup in sys clocks | |

---

## 6. Internal Architecture

```
                ┌────────────────────────────────────────────────────┐
 reg_bus ──────►│  Register File & Command Sequencer                  │
                │                                                    │
                │   ┌─────────┐       ┌──────────────┐              │
                │   │ TX FIFO │──────►│  Bit-Level   │◄────► scl_o/oe
                │   └─────────┘       │  Engine      │              │
                │                     │              │◄────► sda_o/oe
                │   ┌─────────┐       │              │              │
                │   │ RX FIFO │◄──────│              │◄──── scl_i
                │   └─────────┘       └──────────────┘◄──── sda_i
                │                           │                        │
                │   ┌───────────────┐      │                        │
                │   │ Spike Filter  │◄─────┘ (on scl_i, sda_i)     │
                │   └───────────────┘                                │
                │                                                    │
                │   ┌───────────────┐                                │
                │   │ Arbitration   │ (monitors sda_i vs sda_o)     │
                │   │ Detector      │                                │
                │   └───────────────┘                                │
                │                                                    │
                │   ┌───────────────┐                                │
                │   │ Clock Stretch │ (monitors scl_i after release) │
                │   │ & Timeout     │                                │
                │   └───────────────┘                                │
                │                                                    │
                │   ┌───────────────┐                                │
                │   │ Bus Recovery  │ (9 SCL pulses + STOP)          │
                │   └───────────────┘                                │
                │                                                    │
                │   ┌─────────────┐                                  │
                │   │ IRQ & DMA   │─────────────────────► irq, dma_*│
                │   │ Logic       │                                  │
                │   └─────────────┘                                  │
                └────────────────────────────────────────────────────┘
```

### Sub-blocks:
1. **Register File & Command Sequencer** — Decode bus accesses, orchestrate byte-level operations
2. **TX FIFO** — Synchronous FIFO for transmit data bytes
3. **RX FIFO** — Synchronous FIFO for received data bytes
4. **Bit-Level Engine** — Drives SCL/SDA for START, STOP, data bits, ACK/NACK
5. **Spike Filter** — Configurable glitch rejection on SCL/SDA inputs
6. **Arbitration Detector** — Compares driven SDA vs sensed SDA, detects loss
7. **Clock Stretch & Timeout** — Waits for slave to release SCL, counts timeout
8. **Bus Recovery** — Generates 9 clock pulses + STOP to recover stuck bus
9. **IRQ & DMA Logic** — Event aggregation and threshold-based DMA requests

---

## 7. FSM — Byte-Level Controller

| State | Description |
|-------|-------------|
| `IDLE` | Bus free. Waiting for START command |
| `START` | Drive SDA low while SCL high (START condition) |
| `ADDR_PHASE` | Shift out 7-bit address + R/W bit (or first byte of 10-bit) |
| `ADDR10_2ND` | Shift out second byte of 10-bit address |
| `WAIT_ACK` | Release SDA, clock in ACK/NACK from slave |
| `TX_DATA` | Shift out 8 data bits from TX FIFO |
| `RX_DATA` | Shift in 8 data bits, store to RX FIFO |
| `SEND_ACK` | Drive ACK (or NACK if last byte) to slave |
| `REPEATED_START` | Drive repeated START (SDA high→low while SCL high) |
| `STOP` | Drive SDA low→high while SCL high (STOP condition) |
| `ARB_LOST` | Arbitration lost — release bus, set flag |
| `BUS_RECOVER` | Toggle SCL 9 times, then issue STOP |
| `ACK_POLL_WAIT` | Wait retry interval before re-attempting address phase |

### Transitions:
- `IDLE → START`: START command written, bus free
- `START → ADDR_PHASE`: START hold time expired
- `ADDR_PHASE → WAIT_ACK`: 8 bits (addr[6:0] + R/W) shifted out
- `WAIT_ACK → TX_DATA`: ACK received, RD_WR_N=0, TX FIFO not empty
- `WAIT_ACK → RX_DATA`: ACK received, RD_WR_N=1
- `WAIT_ACK → ACK_POLL_WAIT`: NACK received, ACK_POLL_EN=1, retries remaining
- `WAIT_ACK → STOP`: NACK received, ACK_POLL_EN=0 (or retries exhausted)
- `TX_DATA → WAIT_ACK`: 8 bits shifted out
- `RX_DATA → SEND_ACK`: 8 bits shifted in
- `SEND_ACK → RX_DATA`: More bytes to receive
- `SEND_ACK → REPEATED_START`: Transfer direction change requested
- `SEND_ACK → STOP`: Last byte received (NACK sent to slave)
- `TX_DATA → REPEATED_START`: Combined R/W transfer
- `REPEATED_START → ADDR_PHASE`: After repeated START hold
- `STOP → IDLE`: STOP condition complete
- **Any active state → ARB_LOST**: SDA mismatch detected during SCL high
- `ARB_LOST → IDLE`: Bus released
- `IDLE → BUS_RECOVER`: BUS_RECOVER command, bus stuck
- `BUS_RECOVER → IDLE`: 9 clocks + STOP issued

---

## 8. FSM — Bit-Level Engine

| State | Description |
|-------|-------------|
| `BIT_IDLE` | Waiting for byte controller command |
| `SCL_LOW` | SCL driven low, setup SDA data |
| `SDA_SETUP` | SDA stable, wait setup time |
| `SCL_RISE` | Release SCL, wait for it to actually rise (clock stretch detect) |
| `SCL_HIGH` | SCL high, sample SDA (for reads), hold for high period |
| `SCL_FALL` | Drive SCL low, begin next bit or signal done |

### Key Behaviors:
- After releasing SCL (`scl_oe=0`), monitor `scl_i` — if still low, slave is clock-stretching → stay in `SCL_RISE`
- During `SCL_HIGH` in TX mode: compare `sda_i` with `sda_o` for arbitration
- Timeout counter increments during `SCL_RISE` if SCL doesn't rise

---

## 9. Operational Notes

### Clock Generation
- SCL low period = `DIV_LOW + 1` system clocks
- SCL high period = `DIV_HIGH + 1` system clocks
- Asymmetric high/low allows meeting I2C spec minimums independently
- Default values calculated from `P_CLK_FREQ_HZ` and `P_DEFAULT_SCL_FREQ`

### Addressing Modes
- **7-bit**: Sends `{addr[6:0], R/W}` in one byte
- **10-bit**: Sends `{5'b11110, addr[9:8], W}` then `{addr[7:0]}`, optionally repeated START + `{5'b11110, addr[9:8], R}` for reads
- **General Call**: Sends address byte `0x00` followed by command byte

### Arbitration
- During every SCL high phase when transmitting, compare `sda_o` (what we drive) vs `sda_i` (actual bus state)
- If we drive high but read low → another master is driving → we lose arbitration
- On loss: immediately cease driving, set ARB_LOST flag, return to IDLE
- SW must retry the entire transaction

### Clock Stretching
- After releasing SCL, poll `scl_i` before proceeding
- If `scl_i` remains low beyond `SCL_TIMEOUT` clocks → set TIMEOUT flag and interrupt
- The engine remains in stretch state until either SCL rises or SW aborts

### Bus Recovery (Stuck SDA)
- If SDA is stuck low (slave holding bus), master can recover by:
  1. Toggling SCL 9 times (slave will release SDA after seeing clocks)
  2. If SDA releases, issue STOP condition
  3. If SDA still stuck after 9 clocks → set BUS_ERROR, require external intervention

### ACK Polling (EEPROM Write Completion)
- After sending address+write and receiving NACK, automatically retry
- Wait `RETRY_INTERVAL` system clocks between attempts
- Give up after `MAX_RETRIES`, set NACK flag
- Useful for EEPROMs that NACK during internal write cycle

### Spike Filtering
- Input signals `scl_i` and `sda_i` pass through a configurable spike filter
- Pulses shorter than `P_SPIKE_FILTER_NS` are suppressed
- Filter width in system clocks = ceil(`P_SPIKE_FILTER_NS` * `P_CLK_FREQ_HZ` / 1e9)

### Error Handling
| Error | Detection | Response |
|-------|-----------|----------|
| NACK | SDA high during ACK clock | Flag set, STOP (or retry if ACK_POLL) |
| Arbitration Lost | SDA mismatch during SCL high | Release bus, flag set |
| Bus Error | START/STOP at illegal time | Flag set, return to IDLE |
| SCL Timeout | SCL stuck low beyond threshold | Flag set, interrupt |
| SDA Timeout | SDA stuck low beyond threshold | Flag set, suggest bus recovery |
| TX Underrun | TX FIFO empty mid-transfer | Clock stretch (hold SCL low), flag set |
| RX Overrun | RX FIFO full, new byte arrived | Byte discarded, flag set |

---

## 10. Timing Diagram — Write Transaction

```
        ____                                                          ____
SDA         \___[A6][A5][A4][A3][A2][A1][A0][W]___[ACK]___[D7]...[D0]___[ACK]___/
        ________   _   _   _   _   _   _   _   _       _   _       _   _   ________
SCL             \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_____/ \_/ \_...\_/ \_/ \_/
             S   1   2   3   4   5   6   7   8   9      1   2       8   9    P
           START         Address Byte          ACK       Data Byte      ACK  STOP
```

## 11. Timing Diagram — Repeated START (Write then Read)

```
     S [ADDR+W] ACK [REG_ADDR] ACK Sr [ADDR+R] ACK [DATA] NACK P
     │                              │                      │
     └── First START                └── Repeated START     └── Master NACKs last byte
```

---

## 12. I2C Timing Parameters (Reference)

| Parameter | Standard | Fast | Fast-Plus | Register Field |
|-----------|----------|------|-----------|----------------|
| f_SCL max | 100 kHz | 400 kHz | 1 MHz | CLKDIV |
| t_LOW min | 4.7 µs | 1.3 µs | 0.5 µs | DIV_LOW |
| t_HIGH min | 4.0 µs | 0.6 µs | 0.26 µs | DIV_HIGH |
| t_SU;DAT min | 250 ns | 100 ns | 50 ns | SDA_SETUP |
| t_HD;DAT min | 0 ns | 0 ns | 0 ns | SDA_HOLD |
| t_HD;STA min | 4.0 µs | 0.6 µs | 0.26 µs | START_HOLD |
| t_SU;STO min | 4.0 µs | 0.6 µs | 0.26 µs | STOP_SETUP |
| t_BUF min | 4.7 µs | 1.3 µs | 0.5 µs | (derived from DIV_LOW) |
| t_SP max | 50 ns | 50 ns | 50 ns | P_SPIKE_FILTER_NS |

---
