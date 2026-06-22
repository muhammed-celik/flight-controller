# Module 10: Magnetometer Interface (QMC5883L via I²C)

## 1. Why This Module is Needed

A magnetometer (digital compass) measures the Earth's magnetic field vector and provides an absolute yaw (heading) reference. This is critical for:

- **Yaw reference**: Gyroscopes measure angular rate but accumulate drift over time. Without an absolute heading reference, the drone's yaw estimate will drift by several degrees per minute, making it impossible to maintain a fixed heading.
- **GPS navigation**: Waypoint following requires knowing which direction the drone is facing. GPS provides position but not orientation — the magnetometer fills this gap.
- **Return-to-home**: To fly back to the launch point, the drone must know its heading relative to the home position vector.
- **Prevents yaw drift**: The gyroscope-only yaw estimate accumulates 1–10°/min of drift. The magnetometer provides a drift-free (but noisy) heading reference that is fused with the gyroscope via a complementary or Kalman filter.
- **Hover stability**: In GPS-assisted hover, heading drift causes the drone to slowly rotate, which couples into lateral drift through the attitude controller.

Without a magnetometer, the drone can only fly in "rate mode" (no heading hold) and cannot perform autonomous navigation.

---

## 2. Sensor/Approach Options

| Parameter | QMC5883L | HMC5883L | LIS3MDL | BMM150 | MMC5603 |
|-----------|----------|----------|---------|--------|---------|
| **Manufacturer** | QST Corp | Honeywell | STMicroelectronics | Bosch | MEMSIC |
| **Measurement Range** | ±2 / ±8 Gauss | ±1.3 to ±8.1 Gauss | ±4 / ±8 / ±12 / ±16 Gauss | ±13 Gauss | ±30 Gauss |
| **Resolution** | 16-bit | 12-bit | 16-bit | 16-bit (0.3 µT) | 18-bit (0.0625 mG) |
| **Noise Density** | 2 mGauss RMS | 2 mGauss RMS | 3.2 µT RMS | 0.3 µT RMS | 0.4 mGauss RMS |
| **Max ODR** | 200 Hz | 160 Hz | 1000 Hz | 30 Hz | 1000 Hz |
| **Interface** | I²C | I²C / SPI | I²C / SPI | I²C / SPI | I²C |
| **Supply Voltage** | 2.16–3.6 V | 2.16–3.6 V | 1.9–3.6 V | 1.62–3.6 V | 1.62–3.6 V |
| **Package Size** | 3.0×3.0 mm | 3.0×3.0 mm | 2.0×2.0 mm | 1.56×1.56 mm | 0.8×0.8 mm |
| **Cost** | ~$0.50 | ~$2.00 (counterfeit issues) | ~$2.50 | ~$1.50 | ~$1.00 |
| **Availability** | Excellent | Discontinued/counterfeits | Good | Good | Good |

**Advantages/Disadvantages:**

| Sensor | Advantages | Disadvantages |
|--------|-----------|---------------|
| **QMC5883L** | **Very cheap, widely available, 200 Hz ODR, 16-bit, well-documented** | **No SPI option, limited range selection** |
| HMC5883L | Well-known, many libraries available | Discontinued by Honeywell, rampant counterfeits (often QMC5883L relabeled) |
| LIS3MDL | High ODR (1000 Hz), multiple range options, SPI+I²C | More expensive, higher noise |
| BMM150 | Very small package, low power | Low ODR (30 Hz max), limited community support |
| MMC5603 | Excellent resolution (18-bit), wide range, small | Newer part, less community support, degaussing required |

**Chosen: QMC5883L** — Offers ±8 Gauss range (sufficient for Earth's field of 0.25–0.65 Gauss), 200 Hz ODR, 16-bit resolution, I²C interface, extremely low cost (~$0.50), and excellent availability. Well-proven in hobbyist and commercial drone platforms.

---

## 3. Theoretical Foundations

### 3.1 Earth's Magnetic Field

The Earth's magnetic field has a magnitude of approximately 25–65 µT (0.25–0.65 Gauss) depending on geographic location:

- **Equator**: ~30 µT horizontal, ~0 µT vertical
- **Mid-latitudes**: ~20 µT horizontal, ~45 µT vertical (inclination ~60°)
- **Poles**: ~0 µT horizontal, ~60 µT vertical

The field vector can be decomposed into:
- **Horizontal component** (used for heading): 15–30 µT typically
- **Vertical component** (inclination/dip angle): varies by latitude

### 3.2 Heading Calculation

**Basic heading (2D, level):**
```
heading = atan2(mag_y, mag_x)
```

This only works when the sensor is perfectly level. For a tilting drone, tilt compensation is required.

**Tilt-compensated heading:**

Given roll (φ) and pitch (θ) from the attitude estimator:
```
mag_x_h = mag_x × cos(θ) + mag_z × sin(θ)
mag_y_h = mag_x × sin(φ) × sin(θ) + mag_y × cos(φ) - mag_z × sin(φ) × cos(θ)
heading = atan2(-mag_y_h, mag_x_h)
```

Where:
- `mag_x`, `mag_y`, `mag_z` = calibrated magnetometer readings (bias-subtracted)
- `φ` = roll angle (from attitude module)
- `θ` = pitch angle (from attitude module)
- `heading` = magnetic heading in radians (0 = magnetic north, positive clockwise)

**True heading:**
```
true_heading = magnetic_heading + declination_angle
```

The declination angle varies by geographic location (e.g., ~5°E in central Europe, ~14°W in US east coast). It is stored as a configuration parameter.

### 3.3 Magnetic Calibration

**Hard-iron distortion**: Constant magnetic fields from permanent magnets, magnetized components on the PCB. Manifests as a constant offset (bias) in each axis.

```
mag_calibrated[axis] = mag_raw[axis] - bias[axis]
```

Bias is determined by rotating the sensor in all orientations and computing the center of the sphere:
```
bias_x = (max_x + min_x) / 2
bias_y = (max_y + min_y) / 2
bias_z = (max_z + min_z) / 2
```

**Soft-iron distortion**: Ferromagnetic materials that distort the field direction. Transforms the measurement sphere into an ellipsoid. Corrected with a 3×3 transformation matrix:

```
[cal_x]   [S11 S12 S13] [raw_x - bias_x]
[cal_y] = [S21 S22 S23] [raw_y - bias_y]
[cal_z]   [S31 S32 S33] [raw_z - bias_z]
```

**Note**: All calibration and heading computation is performed on the CPU (requires trigonometry, matrix multiply, atan2). The RTL module only handles raw data acquisition.

### 3.4 QMC5883L Register Map

| Register | Address | Description |
|----------|---------|-------------|
| DATA_X_L | 0x00 | X-axis data, low byte |
| DATA_X_H | 0x01 | X-axis data, high byte |
| DATA_Y_L | 0x02 | Y-axis data, low byte |
| DATA_Y_H | 0x03 | Y-axis data, high byte |
| DATA_Z_L | 0x04 | Z-axis data, low byte |
| DATA_Z_H | 0x05 | Z-axis data, high byte |
| STATUS | 0x06 | Bit 0: DRDY (data ready), Bit 1: OVL (overflow) |
| CONTROL_1 | 0x09 | Mode, ODR, Range, OSR settings |
| CONTROL_2 | 0x0A | Soft reset, pointer roll-over enable |
| SET_RESET | 0x0B | Set/Reset period (recommended: 0x01) |
| CHIP_ID | 0x0D | Should read 0xFF for QMC5883L |

**CONTROL_1 register (0x09) bit fields:**
- Bits [1:0]: Mode — 00=Standby, 01=Continuous
- Bits [3:2]: ODR — 00=10Hz, 01=50Hz, 10=100Hz, 11=200Hz
- Bits [5:4]: Range — 00=2G, 01=8G
- Bits [7:6]: OSR — 00=512, 01=256, 10=128, 11=64

### 3.5 I²C Communication Details

- **QMC5883L I²C Address**: `0x0D` (7-bit, fixed — no address pin)
- **Bus speed**: 400 kHz (Fast Mode)
- **Data format**: 16-bit signed, little-endian (LSB first)
- **Read sequence**: Set register pointer to 0x00, burst read 6 bytes (X_L, X_H, Y_L, Y_H, Z_L, Z_H)

### 3.6 I²C Bus Sharing with Barometer

Both the BMP390 (address 0x77) and QMC5883L (address 0x0D) share the same physical I²C bus. A bus arbiter multiplexes access:

- Barometer reads at 25 Hz (40 ms period)
- Magnetometer reads at 50 Hz (20 ms period)
- Maximum bus occupancy per read: ~300 µs (12 bytes × 9 bits × 2.5 µs/bit + overhead)
- Total bus utilization: 25×300 + 50×300 = 22.5 ms out of 1000 ms = 2.25% (well within capacity)

**Arbitration scheme**: Round-robin with priority — barometer has lower priority since its data changes slowly; magnetometer reads are interleaved. If both request simultaneously, magnetometer goes first (shorter transaction).

### 3.7 Magnetic Field Units and Conversions

For QMC5883L at ±8 Gauss range:
- Sensitivity: 3000 LSB/Gauss
- Full scale: ±8 Gauss = ±24000 LSB (approximately)
- Earth's field at 0.5 Gauss → ~1500 LSB reading

For QMC5883L at ±2 Gauss range:
- Sensitivity: 12000 LSB/Gauss
- Full scale: ±2 Gauss = ±24000 LSB
- Earth's field at 0.5 Gauss → ~6000 LSB reading (better resolution)

---

## 4. Module Interface

### 4.1 Input Signals

| Signal | Width | Format | Source | Description |
|--------|-------|--------|--------|-------------|
| clk | 1 | — | System | 100 MHz system clock |
| rst_n | 1 | — | System | Active-low synchronous reset |
| tick_50hz | 1 | Pulse | Timer | Single-cycle pulse every 20 ms, triggers new read |
| i2c_sda_i | 1 | — | I²C Bus | SDA input (read from shared bus) |
| i2c_bus_grant | 1 | — | I²C Arbiter | Bus access granted to this module |
| cfg_mode | 2 | Unsigned | AXI Config | Operating mode (standby/continuous) |
| cfg_odr | 2 | Unsigned | AXI Config | Output data rate (10/50/100/200 Hz) |
| cfg_range | 1 | Unsigned | AXI Config | Full-scale range (0=±2G, 1=±8G) |
| cfg_osr | 2 | Unsigned | AXI Config | Oversampling ratio (64/128/256/512) |

### 4.2 Output Signals

| Signal | Width | Format | Destination | Description |
|--------|-------|--------|-------------|-------------|
| i2c_scl | 1 | — | I²C Bus | SCL output (shared with barometer) |
| i2c_sda_o | 1 | — | I²C Bus | SDA output data |
| i2c_sda_oe | 1 | — | I²C Bus | SDA output enable (1 = drive low, 0 = release) |
| i2c_bus_req | 1 | — | I²C Arbiter | Request for bus access |
| mag_x | 16 | Signed | CPU/DMA | Raw X-axis magnetic field reading |
| mag_y | 16 | Signed | CPU/DMA | Raw Y-axis magnetic field reading |
| mag_z | 16 | Signed | CPU/DMA | Raw Z-axis magnetic field reading |
| mag_valid | 1 | Pulse | Downstream | Single-cycle pulse when new valid data available |
| mag_overflow | 1 | Level | Status | Sensor measurement overflow (field too strong) |
| mag_error | 1 | Level | Status | I²C transaction error (NACK, timeout) |
| busy | 1 | Level | Status | Module is mid-transaction |

---

## 5. Algorithm (Pseudocode)

### 5.1 Top-Level State Machine

```
STATE: IDLE
  On tick_50hz:
    Assert i2c_bus_req
    → WAIT_GRANT

STATE: WAIT_GRANT
  On i2c_bus_grant:
    → CHECK_STATUS
  On timeout (>1 ms):
    Set mag_error
    Deassert i2c_bus_req
    → IDLE

STATE: CHECK_STATUS
  // Write register address pointer to STATUS register
  I²C write: [START, 0x0D<<1|W, 0x06, STOP]
  // Read STATUS byte
  I²C read:  [START, 0x0D<<1|R, read 1 byte, NACK, STOP]
  If status_byte[0] == 1 (DRDY):
    If status_byte[1] == 1 (OVL):
      Set mag_overflow flag
    → READ_DATA
  Else:
    Deassert i2c_bus_req
    → IDLE (retry next tick)

STATE: READ_DATA
  // Set register address pointer to DATA_X_L (0x00)
  I²C write: [START, 0x0D<<1|W, 0x00, STOP]
  // Burst read 6 bytes
  I²C read:  [START, 0x0D<<1|R, read 6 bytes (ACK each), last byte NACK, STOP]
  
  Store bytes (little-endian):
    mag_x = {byte[1], byte[0]}   // X_H:X_L → signed 16-bit
    mag_y = {byte[3], byte[2]}   // Y_H:Y_L → signed 16-bit
    mag_z = {byte[5], byte[4]}   // Z_H:Z_L → signed 16-bit
  
  Pulse mag_valid
  Deassert i2c_bus_req
  → IDLE

STATE: ERROR
  Deassert i2c_bus_req
  Set mag_error flag
  Wait for tick_50hz to retry
  → IDLE
```

### 5.2 Initialization Sequence (Power-On)

```
On reset de-assertion:
  Wait 1 ms (power-on time for QMC5883L)
  
  // Soft reset
  I²C write: [START, 0x0D<<1|W, 0x0A, 0x80, STOP]
  Wait 1 ms
  
  // Verify chip ID (optional — register 0x0D should read 0xFF)
  // Note: QMC5883L chip ID register is unreliable on some clones
  
  // Configure Set/Reset period
  I²C write: [START, 0x0D<<1|W, 0x0B, 0x01, STOP]
  
  // Configure CONTROL_1: continuous mode, 200Hz ODR, ±8G range, OSR=512
  // Bits: [7:6]=00 (OSR 512), [5:4]=01 (8G), [3:2]=11 (200Hz), [1:0]=01 (continuous)
  control1_value = (cfg_osr << 6) | (cfg_range << 4) | (cfg_odr << 2) | cfg_mode
  I²C write: [START, 0x0D<<1|W, 0x09, control1_value, STOP]
  
  // Enable pointer roll-over
  I²C write: [START, 0x0D<<1|W, 0x0A, 0x40, STOP]
  
  → IDLE (ready for periodic reads)
```

### 5.3 I²C Transaction Sequencing

```
// This module reuses the same I²C byte-level engine as the barometer module.
// The engine provides: send_start(), send_stop(), send_byte(), read_byte()
// See Module 09 for byte-level I²C implementation details.

PROCEDURE write_register(reg_addr, data):
  send_start()
  send_byte(0x0D << 1 | 0)   // Write address: 0x1A
  send_byte(reg_addr)
  send_byte(data)
  send_stop()

PROCEDURE read_registers(start_addr, count) → returns byte_array:
  // Set pointer
  send_start()
  send_byte(0x0D << 1 | 0)   // Write address
  send_byte(start_addr)
  send_stop()
  
  // Read data
  send_start()
  send_byte(0x0D << 1 | 1)   // Read address: 0x1B
  For i = 0 to count-1:
    If i == count-1:
      byte_array[i] = read_byte(NACK)  // Last byte gets NACK
    Else:
      byte_array[i] = read_byte(ACK)
  send_stop()
  Return byte_array
```

### 5.4 Bus Arbitration Interaction

```
// The I²C bus arbiter grants access to one client at a time.
// This module's interaction with the arbiter:

On needing bus access:
  Assert i2c_bus_req = 1
  Wait for i2c_bus_grant = 1
  Perform I²C transactions (SCL/SDA driven only when granted)
  When done: deassert i2c_bus_req = 0

// If bus is busy (barometer using it):
//   i2c_bus_grant stays low until barometer releases
//   Timeout counter prevents indefinite waiting
```

---

## 6. Resource Estimate

| Resource | Count | Notes |
|----------|-------|-------|
| LUTs | ~200 | State machine, shift register, byte counter (simpler than baro — fewer states) |
| Flip-Flops | ~180 | State register, data registers (6×8=48 bits for raw + 3×16=48 for output), counters |
| DSP48E1 | 0 | No arithmetic computation in RTL (heading math on CPU) |
| BRAM (36Kb) | 0 | All data fits in registers |
| I/O Pins | 0 | Shares SCL and SDA pins with barometer module (bus mux in arbiter) |
| Clock | 1 | 100 MHz system clock only |

**Breakdown:**
- I²C shift register + control: ~70 LUTs, ~50 FFs (shared engine design with baro)
- State machine (main): ~40 LUTs, ~20 FFs
- Data registers (6 raw bytes + 3×16 output): ~30 LUTs, ~60 FFs
- Bus arbitration interface: ~20 LUTs, ~20 FFs
- Timeout counter: ~25 LUTs, ~20 FFs
- Configuration registers: ~15 LUTs, ~10 FFs

**Note on shared I²C engine**: In the final implementation, a single I²C master engine may be shared between barometer and magnetometer modules, controlled by the bus arbiter. This would reduce total resource usage by ~70 LUTs and ~50 FFs compared to duplicating the engine.
