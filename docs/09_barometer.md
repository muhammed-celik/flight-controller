# Module 09: Barometer Interface (BMP390 via I²C)

## 1. Why This Module is Needed

A barometer provides absolute atmospheric pressure measurements that can be converted to altitude estimates. This is essential for:

- **Altitude hold mode**: Maintaining a constant height above ground without GPS requires a pressure-based altitude reference. GPS vertical accuracy (±5–10 m) is insufficient for stable hover.
- **Vertical velocity estimation**: Differentiating barometric altitude over time provides vertical velocity (climb/descent rate), which is fused with accelerometer data for robust vertical state estimation.
- **Ground-level reference**: On startup, the current pressure is stored as P₀ (sea-level equivalent), and all subsequent altitude readings are relative to the takeoff point.
- **Complementary to accelerometer**: The accelerometer provides short-term vertical dynamics but drifts over time; the barometer provides long-term stable altitude reference but is noisy at short timescales. Sensor fusion combines both.

Without a barometer, the drone cannot autonomously hold altitude and will drift vertically, making stable hover and landing impossible without constant pilot input.

---

## 2. Sensor/Approach Options

| Parameter | BMP280 | BMP388 | BMP390 | MS5611 | LPS22HH |
|-----------|--------|--------|--------|--------|---------|
| **Manufacturer** | Bosch | Bosch | Bosch | TE Connectivity | STMicroelectronics |
| **Pressure Resolution** | 0.16 Pa (1.3 cm) | 0.08 Pa (0.66 cm) | 0.03 Pa (0.25 cm) | 0.012 mbar (10 cm) | 0.00024 hPa (2 cm) |
| **Noise (RMS)** | 1.3 Pa | 0.08 Pa | 0.03 hPa | 0.024 mbar | 0.65 Pa |
| **Altitude Noise** | ±11 cm | ±6.6 cm | ±25 cm | ±10 cm | ±5.4 cm |
| **Max ODR** | 157 Hz | 200 Hz | 200 Hz | 100 Hz (typical) | 200 Hz |
| **Interface** | I²C / SPI | I²C / SPI | I²C / SPI | I²C / SPI | I²C / SPI |
| **Supply Voltage** | 1.71–3.6 V | 1.65–3.6 V | 1.65–3.6 V | 1.8–3.6 V | 1.7–3.6 V |
| **Temperature Sensor** | Yes (integrated) | Yes (integrated) | Yes (integrated) | Yes (integrated) | Yes (integrated) |
| **FIFO** | No | 512 bytes | 512 bytes | No | 128 bytes |
| **Package Size** | 2.0×2.5 mm | 2.0×2.0 mm | 2.0×2.0 mm | 5.0×3.0 mm | 2.0×2.0 mm |
| **Cost** | ~$1.50 | ~$3.00 | ~$3.50 | ~$5.00 | ~$2.50 |

**Advantages/Disadvantages:**

| Sensor | Advantages | Disadvantages |
|--------|-----------|---------------|
| BMP280 | Cheap, widely available, simple | Lowest resolution of Bosch series, no FIFO |
| BMP388 | Good resolution, FIFO buffer | Being phased out in favor of BMP390 |
| **BMP390** | **Best noise performance (0.03 hPa), 200 Hz ODR, FIFO, well-documented** | **Slightly more expensive than BMP280** |
| MS5611 | Excellent resolution, proven in many flight controllers | Large package, slower ODR, higher cost |
| LPS22HH | Low noise, small package | Less community support, fewer open-source drivers |

**Chosen: BMP390** — Offers 0.03 hPa RMS noise (±0.25 m altitude resolution), 200 Hz output data rate, integrated 512-byte FIFO, I²C/SPI interface, and excellent documentation. Widely supported in open-source flight controller ecosystems.

---

## 3. Theoretical Foundations

### 3.1 I²C Protocol Basics

I²C (Inter-Integrated Circuit) is a synchronous, multi-master, multi-slave serial bus:

- **Two wires**: SCL (clock), SDA (data) — both open-drain with pull-up resistors
- **Standard Mode**: 100 kHz clock
- **Fast Mode**: 400 kHz clock (used here)
- **7-bit addressing**: supports up to 128 devices on one bus

**Transaction format:**

```
START → [7-bit Address | R/W] → ACK → [Data Byte] → ACK → ... → STOP
```

- **START condition**: SDA falls while SCL is high
- **STOP condition**: SDA rises while SCL is high
- **ACK**: Receiver pulls SDA low during 9th clock cycle
- **NACK**: Receiver leaves SDA high during 9th clock cycle (signals end of read)

**BMP390 I²C Address**: `0x77` (SDO pin tied high) or `0x76` (SDO pin tied low)

**SCL Generation from 100 MHz system clock:**
- Target: 400 kHz SCL → period = 2.5 µs
- Clock divider: 100 MHz ÷ 400 kHz = 250 counts per SCL period
- 125 counts high, 125 counts low
- Each bit takes 250 clock cycles; 9 bits per byte = 2250 cycles per byte

### 3.2 Barometric Altitude Formula

The International Standard Atmosphere (ISA) model relates pressure to altitude:

```
altitude = 44330 × (1 - (P / P₀)^(1/5.255))
```

Where:
- `altitude` = height above reference in meters
- `P` = measured pressure in Pascals
- `P₀` = reference pressure at sea level (101325 Pa, or measured at takeoff)
- `5.255` = g×M / (R×L) where g=9.80665, M=0.0289644, R=8.31447, L=0.0065

**Note**: This formula involves exponentiation (fractional power), which is computationally expensive in fixed-point hardware. Therefore, altitude calculation is performed on the CPU (soft-core or external MCU), and the RTL module only handles raw I²C data acquisition.

### 3.3 BMP390 Temperature and Pressure Compensation

The BMP390 outputs raw ADC values (20-bit for both pressure and temperature). These must be compensated using factory-stored trimming parameters (NVM coefficients):

**Temperature compensation:**
```
partial_data1 = raw_temp - par_t1
partial_data2 = partial_data1 × par_t2
comp_temp = partial_data2 + (partial_data1² × par_t3)
```

**Pressure compensation:**
```
(Uses comp_temp as input along with par_p1 through par_p11)
comp_press = f(raw_press, comp_temp, par_p1..par_p11)
```

These computations involve 64-bit intermediate values and floating-point operations. They are performed on the CPU, not in RTL. The FPGA module provides raw 24-bit pressure and temperature data only.

### 3.4 BMP390 Register Map (Relevant Subset)

| Register | Address | Description |
|----------|---------|-------------|
| CHIP_ID | 0x00 | Should read 0x60 for BMP390 |
| STATUS | 0x03 | Bit 5: drdy_press, Bit 6: drdy_temp |
| DATA_0 | 0x04 | Pressure [7:0] (LSB) |
| DATA_1 | 0x05 | Pressure [15:8] |
| DATA_2 | 0x06 | Pressure [23:16] (MSB) |
| DATA_3 | 0x07 | Temperature [7:0] (LSB) |
| DATA_4 | 0x08 | Temperature [15:8] |
| DATA_5 | 0x09 | Temperature [23:16] (MSB) |
| PWR_CTRL | 0x1B | Enable pressure/temp measurement, set mode |
| OSR | 0x1C | Oversampling settings |
| ODR | 0x1D | Output data rate |

### 3.5 Read Sequence

1. Write to register 0x03 (STATUS) address to set read pointer
2. Read STATUS byte, check bit 5 (drdy_press) is set
3. If DRDY = 1: burst read 6 bytes starting at 0x04 (pressure[2:0] + temperature[2:0])
4. If DRDY = 0: skip this cycle, try again next tick

### 3.6 I²C Timing Requirements (Fast Mode, 400 kHz)

| Parameter | Symbol | Min | Max |
|-----------|--------|-----|-----|
| SCL clock frequency | f_SCL | 0 | 400 kHz |
| START hold time | t_HD;STA | 0.6 µs | — |
| SCL low period | t_LOW | 1.3 µs | — |
| SCL high period | t_HIGH | 0.6 µs | — |
| Data setup time | t_SU;DAT | 100 ns | — |
| Data hold time | t_HD;DAT | 0 | 0.9 µs |
| STOP setup time | t_SU;STO | 0.6 µs | — |

At 100 MHz clock (10 ns period), 125 cycles for SCL half-period = 1.25 µs, which satisfies both t_LOW (min 1.3 µs → use 130 cycles) and t_HIGH (min 0.6 µs → use 120 cycles).

---

## 4. Module Interface

### 4.1 Input Signals

| Signal | Width | Format | Source | Description |
|--------|-------|--------|--------|-------------|
| clk | 1 | — | System | 100 MHz system clock |
| rst_n | 1 | — | System | Active-low synchronous reset |
| tick_25hz | 1 | Pulse | Timer | Single-cycle pulse every 40 ms, triggers a new read |
| i2c_sda_i | 1 | — | I²C Bus | SDA input (read from bus) |
| i2c_bus_grant | 1 | — | I²C Arbiter | Bus granted to this module |
| cfg_osr | 4 | Unsigned | AXI Config | Oversampling ratio setting (0–5) |
| cfg_odr | 4 | Unsigned | AXI Config | Output data rate divider |

### 4.2 Output Signals

| Signal | Width | Format | Destination | Description |
|--------|-------|--------|-------------|-------------|
| i2c_scl | 1 | — | I²C Bus | SCL output (directly driven, open-drain externally) |
| i2c_sda_o | 1 | — | I²C Bus | SDA output data |
| i2c_sda_oe | 1 | — | I²C Bus | SDA output enable (1 = drive, 0 = tristate/release) |
| i2c_bus_req | 1 | — | I²C Arbiter | Request for bus access |
| raw_pressure | 24 | Unsigned | CPU/DMA | Raw pressure ADC reading [23:0] |
| raw_temperature | 24 | Unsigned | CPU/DMA | Raw temperature ADC reading [23:0] |
| baro_valid | 1 | Pulse | Downstream | Single-cycle pulse when new valid data is available |
| baro_error | 1 | Level | Status | I²C transaction error (NACK, timeout) |
| busy | 1 | Level | Status | Module is mid-transaction |

---

## 5. Algorithm (Pseudocode)

### 5.1 Top-Level State Machine

```
STATE: IDLE
  On tick_25hz:
    Assert i2c_bus_req
    → WAIT_GRANT

STATE: WAIT_GRANT
  On i2c_bus_grant:
    → CHECK_STATUS
  On timeout (>1 ms):
    Set baro_error
    Deassert i2c_bus_req
    → IDLE

STATE: CHECK_STATUS
  Perform I²C write: [START, 0x77<<1|W, 0x03, STOP]
  Perform I²C read:  [START, 0x77<<1|R, read 1 byte, NACK, STOP]
  If status_byte[5] == 1 (drdy_press):
    → READ_DATA
  Else:
    Deassert i2c_bus_req
    → IDLE (will retry next tick)

STATE: READ_DATA
  Perform I²C write: [START, 0x77<<1|W, 0x04, STOP]
  Perform I²C read:  [START, 0x77<<1|R, read 6 bytes with ACK, last byte NACK, STOP]
  Store bytes:
    raw_pressure[7:0]   = byte[0]
    raw_pressure[15:8]  = byte[1]
    raw_pressure[23:16] = byte[2]
    raw_temperature[7:0]   = byte[3]
    raw_temperature[15:8]  = byte[4]
    raw_temperature[23:16] = byte[5]
  Pulse baro_valid
  Deassert i2c_bus_req
  → IDLE

STATE: ERROR
  Deassert i2c_bus_req
  Set baro_error flag
  Wait for tick_25hz to retry
  → IDLE
```

### 5.2 I²C Master Byte-Level Engine

```
PROCEDURE send_start():
  SDA = 1, SCL = 1 (both released)
  Wait t_SU;STA (60 clock cycles)
  SDA = 0 (pull low while SCL high)
  Wait t_HD;STA (60 clock cycles)
  SCL = 0 (begin first clock)

PROCEDURE send_stop():
  SDA = 0
  SCL = 1 (release clock)
  Wait t_SU;STO (60 clock cycles)
  SDA = 1 (release data while clock high)

PROCEDURE send_byte(data[7:0]) → returns ACK/NACK:
  For bit_index = 7 downto 0:
    SDA = data[bit_index]
    Wait t_SU;DAT
    SCL = 1
    Wait t_HIGH (120 cycles)
    SCL = 0
    Wait t_LOW (130 cycles)
  // 9th clock: read ACK
  Release SDA (SDA_OE = 0)
  SCL = 1
  Wait t_HIGH
  Sample SDA → ack_bit
  SCL = 0
  Wait t_LOW
  Return (ack_bit == 0) // 0 = ACK, 1 = NACK

PROCEDURE read_byte(send_ack) → returns data[7:0]:
  Release SDA (SDA_OE = 0)
  For bit_index = 7 downto 0:
    SCL = 1
    Wait t_HIGH
    data[bit_index] = sample SDA
    SCL = 0
    Wait t_LOW
  // 9th clock: send ACK or NACK
  If send_ack:
    SDA = 0 (drive low = ACK)
  Else:
    SDA = 1 (release = NACK)
  SCL = 1
  Wait t_HIGH
  SCL = 0
  Wait t_LOW
  Return data
```

### 5.3 SCL Clock Divider

```
REGISTER: clk_divider (8-bit counter)

On every system clock rising edge:
  If clk_divider == 249:
    clk_divider = 0
    scl_tick = 1   // One full SCL period elapsed
  Else:
    clk_divider = clk_divider + 1
    scl_tick = 0

  // Generate SCL phases
  If clk_divider < 125:
    scl_phase = LOW
  Else:
    scl_phase = HIGH
```

### 5.4 Initialization Sequence (Power-On)

```
On reset de-assertion:
  Wait 2 ms (soft-start time for BMP390)
  
  // Verify chip ID
  Read register 0x00 → should be 0x60
  If chip_id != 0x60: set baro_error, → IDLE
  
  // Configure sensor
  Write register 0x1B (PWR_CTRL) = 0x33
    // Enable pressure (bit 0), enable temperature (bit 1), normal mode (bits 5:4 = 11)
  Write register 0x1C (OSR) = cfg_osr
    // Set oversampling for pressure and temperature
  Write register 0x1D (ODR) = cfg_odr
    // Set output data rate
  
  → IDLE (ready for periodic reads)
```

---

## 6. Resource Estimate

| Resource | Count | Notes |
|----------|-------|-------|
| LUTs | ~250 | State machine, clock divider, shift register, byte counter |
| Flip-Flops | ~200 | State register, data registers (6×8=48), counters, control |
| DSP48E1 | 0 | No arithmetic computation in RTL |
| BRAM (36Kb) | 0 | Data fits in registers |
| I/O Pins | 2 | SCL (output), SDA (bidirectional) — shared with magnetometer |
| Clock | 1 | 100 MHz system clock only |

**Breakdown:**
- I²C shift register + control: ~80 LUTs, ~60 FFs
- State machine (main + I²C engine): ~60 LUTs, ~30 FFs
- Clock divider (250-count): ~10 LUTs, ~8 FFs
- Data registers (6 bytes + valid): ~50 LUTs, ~50 FFs
- Bus arbitration logic: ~20 LUTs, ~20 FFs
- Timeout counter: ~30 LUTs, ~32 FFs
