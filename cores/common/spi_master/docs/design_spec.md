# SPI Master Controller — Design Specification

## 1. Overview

A feature-rich, parameterizable SPI Master controller supporting all four SPI modes, multi-slave select, configurable data widths, FIFO buffering, and Dual/Quad SPI extensions. Designed for integration via a simple register-based CPU/bus interface or a streaming command interface.

---

## 2. Feature Summary

| Feature | Description |
|---------|-------------|
| All 4 SPI Modes | CPOL/CPHA = {0,0}, {0,1}, {1,0}, {1,1} |
| Configurable Data Width | 4, 8, 16, 24, 32-bit frame sizes (runtime selectable) |
| Clock Divider | Programmable SCK frequency via even-integer divider |
| Multi-Slave CS | Up to N chip-select lines, active-low, independently controlled |
| Bit Order | MSB-first or LSB-first (runtime selectable) |
| TX/RX FIFOs | Configurable depth, independent TX and RX |
| Continuous Transfer | Multi-frame transfers without CS deassertion |
| Inter-Frame Gap | Programmable idle cycles between frames |
| Dual/Quad SPI | 2-bit and 4-bit data modes for high-throughput reads |
| DMA Interface | Handshake signals for TX/RX DMA channels |
| Interrupts | TX empty, RX full, transfer done, RX overrun, TX underrun |
| Loopback Mode | Internal MOSI→MISO loopback for self-test |
| CS-to-SCK Delay | Programmable setup/hold time for CS assertion |
| Manual CS Control | Software-driven CS for non-standard protocols |

---

## 3. Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `P_NUM_CS` | int | 4 | Number of chip-select outputs |
| `P_FIFO_DEPTH` | int | 16 | TX and RX FIFO depth (power of 2) |
| `P_DATA_WIDTH_MAX` | int | 32 | Maximum frame data width |
| `P_CLK_FREQ_HZ` | int | 100_000_000 | System clock frequency |
| `P_QUAD_EN` | bit | 1 | Enable Dual/Quad SPI data paths |

---

## 4. Port Interface

### 4.1 System Ports

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `clk` | in | 1 | System clock |
| `rst_n` | in | 1 | Active-low async reset |

### 4.2 Register/Bus Interface (Simple Valid-Ready)

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `reg_addr` | in | 4 | Register address |
| `reg_wdata` | in | 32 | Write data |
| `reg_wen` | in | 1 | Write enable |
| `reg_ren` | in | 1 | Read enable |
| `reg_rdata` | out | 32 | Read data |
| `reg_ready` | out | 1 | Access acknowledge |

### 4.3 SPI Pad Interface

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `spi_sck` | out | 1 | SPI serial clock |
| `spi_cs_n` | out | P_NUM_CS | Chip selects (active-low) |
| `spi_mosi` | out | 1 | Master-Out Slave-In (standard mode) |
| `spi_miso` | in | 1 | Master-In Slave-Out (standard mode) |
| `spi_io_o` | out | 4 | Quad data output (Quad mode) |
| `spi_io_i` | in | 4 | Quad data input (Quad mode) |
| `spi_io_oe` | out | 4 | Quad data output-enable (active-high) |

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
| 0x0 | `CTRL` | RW | [0] SPI_EN — module enable | |
| | | | [1] CPOL — clock polarity | |
| | | | [2] CPHA — clock phase | |
| | | | [3] LSB_FIRST — bit order | |
| | | | [4] LOOPBACK — loopback enable | |
| | | | [6:5] DATA_MODE — 0:Standard, 1:Dual, 2:Quad | |
| | | | [7] MANUAL_CS — manual CS control enable | |
| | | | [12:8] FRAME_SIZE — bits per frame minus 1 (0–31) | |
| | | | [15:13] CS_SEL — active chip-select index | |
| 0x1 | `CLKDIV` | RW | [15:0] DIV — SCK = clk / (2*(DIV+1)) | |
| 0x2 | `CSTIME` | RW | [7:0] CS_SETUP — clk cycles CS assert to first SCK edge | |
| | | | [15:8] CS_HOLD — clk cycles last SCK edge to CS deassert | |
| | | | [23:16] INTER_FRAME — clk cycles between frames | |
| 0x3 | `TXDATA` | W | [31:0] — Write pushes to TX FIFO | |
| 0x4 | `RXDATA` | R | [31:0] — Read pops from RX FIFO | |
| 0x5 | `STATUS` | R | [0] TX_FULL | |
| | | | [1] TX_EMPTY | |
| | | | [2] RX_FULL | |
| | | | [3] RX_EMPTY | |
| | | | [4] BUSY — transfer in progress | |
| | | | [7:5] TX_LEVEL — TX FIFO occupancy | |
| | | | [10:8] RX_LEVEL — RX FIFO occupancy | |
| 0x6 | `IRQ_EN` | RW | [0] TX_EMPTY_IE | |
| | | | [1] RX_FULL_IE | |
| | | | [2] XFER_DONE_IE | |
| | | | [3] RX_OVERRUN_IE | |
| | | | [4] TX_UNDERRUN_IE | |
| 0x7 | `IRQ_STATUS` | R/W1C | Same bit positions as IRQ_EN | |
| 0x8 | `XFER_LEN` | RW | [15:0] — Number of frames in a burst (0=single) | |
| 0x9 | `CS_OVERRIDE` | RW | [P_NUM_CS-1:0] — Direct CS pin values when MANUAL_CS=1 | |
| 0xA | `DMA_CTRL` | RW | [0] DMA_TX_EN | |
| | | | [1] DMA_RX_EN | |
| | | | [5:2] TX_WATERMARK — TX FIFO threshold for DMA req | |
| | | | [9:6] RX_WATERMARK — RX FIFO threshold for DMA req | |

---

## 6. Internal Architecture

```
                  ┌─────────────────────────────────────────────┐
  reg_bus ──────►│  Register File & Control Logic               │
                  │                                             │
                  │   ┌─────────┐        ┌─────────┐           │
                  │   │ TX FIFO │───────►│  Shift  │──► spi_mosi/io_o
                  │   └─────────┘        │  Engine │
                  │                      │         │◄── spi_miso/io_i
                  │   ┌─────────┐        │         │
                  │   │ RX FIFO │◄───────│         │──► spi_sck
                  │   └─────────┘        └─────────┘
                  │                           │
                  │   ┌──────────────┐       │
                  │   │ CS & Timing  │◄──────┘──────────► spi_cs_n
                  │   │ Controller   │
                  │   └──────────────┘
                  │                                             │
                  │   ┌─────────────┐                          │
                  │   │ IRQ & DMA   │──────────────────► irq, dma_*
                  │   │ Logic       │                          │
                  │   └─────────────┘                          │
                  └─────────────────────────────────────────────┘
```

### Sub-blocks:
1. **Register File** — Decode bus accesses, hold config, provide FIFO push/pop
2. **TX FIFO** — Synchronous FIFO, parameterized depth
3. **RX FIFO** — Synchronous FIFO, parameterized depth
4. **Shift Engine** — Serial/parallel shift register, handles Standard/Dual/Quad
5. **CS & Timing Controller** — Manages CS assertion, setup/hold/inter-frame timing
6. **Clock Generator** — Divides system clock, applies CPOL
7. **IRQ & DMA Logic** — Monitors FIFO levels and events, generates outputs

---

## 7. FSM — Shift Engine

| State | Description |
|-------|-------------|
| `IDLE` | No transfer. Waiting for TX FIFO non-empty and SPI_EN |
| `CS_ASSERT` | Assert CS, count CS_SETUP cycles |
| `SHIFT` | Clock out/in bits (1, 2, or 4 per SCK edge depending on mode) |
| `FRAME_GAP` | Optional inter-frame wait between frames in a burst |
| `CS_DEASSERT` | Count CS_HOLD cycles, then release CS |
| `DONE` | Signal transfer complete, return to IDLE |

### Transitions:
- `IDLE → CS_ASSERT`: TX FIFO not empty, SPI_EN=1
- `CS_ASSERT → SHIFT`: CS_SETUP count expired
- `SHIFT → SHIFT`: More bits remaining in frame
- `SHIFT → FRAME_GAP`: Frame complete, more frames in burst (XFER_LEN > 0)
- `SHIFT → CS_DEASSERT`: Last frame complete
- `FRAME_GAP → SHIFT`: Inter-frame count expired, next frame begins
- `CS_DEASSERT → DONE`: CS_HOLD count expired
- `DONE → IDLE`: Always (single-cycle)

---

## 8. Operational Notes

### Clock Generation
- SCK frequency = `clk / (2 * (CLKDIV + 1))`
- Minimum divider of 2 (DIV=0 → clk/2)
- CPOL determines idle state of SCK
- CPHA determines sample edge (leading vs trailing)

### Dual/Quad Mode
- In Dual mode: 2 bits per clock on io[1:0], halving transfer time
- In Quad mode: 4 bits per clock on io[3:0], quartering transfer time
- `spi_io_oe` controls direction per pin (needed for bidirectional Quad reads)
- Typically: command phase in Standard, data phase in Dual/Quad

### Continuous Transfer
- When `XFER_LEN > 0`, the engine shifts that many frames back-to-back without de-asserting CS
- TX underrun during continuous transfer: SCK pauses, TX_UNDERRUN flag set

### Error Handling
- **TX Underrun**: Transfer stalls, flag set, recoverable
- **RX Overrun**: New data discarded, flag set, RX FIFO contents preserved
- Both flags generate interrupt if enabled, cleared by W1C

---

## 9. Timing Diagram (Standard Mode, CPOL=0, CPHA=0)

```
         ___                                           ___
CS_N        \_________ ... __________________________ /
              |<-setup->|                    |<-hold->|
         ________   _   _   _   _   _   _   _   ________
SCK              \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/
                  0   1   2   3   4   5   6   7
         --------X===X===X===X===X===X===X===X===--------
MOSI             |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |
         --------X===X===X===X===X===X===X===X===--------
MISO       sample^   ^   ^   ^   ^   ^   ^   ^
```

---
