# Module 01: IMU Sensor Acquisition via SPI

## 1. Why This Module is Needed

A quadrotor has no inherent aerodynamic stability. Unlike a fixed-wing aircraft
with dihedral or sweep that passively returns to level, a multirotor is an
inverted pendulum in three axes simultaneously. Any perturbation — a wind gust,
motor RPM mismatch, center-of-gravity offset — causes the vehicle to tip and
accelerate into a crash within 200–500 ms if uncorrected.

The Inertial Measurement Unit (IMU) provides the raw physical measurements that
make active stabilization possible:

- **Gyroscope (3-axis):** Measures angular velocity (°/s). This is the PRIMARY
  feedback for the rate control loop. Without it, no flight is possible.
- **Accelerometer (3-axis):** Measures specific force (g). In hover, this equals
  gravity and provides an absolute "which way is down" reference. Prevents the
  gyro-only attitude estimate from drifting over time.
- **Temperature sensor:** Embedded in the IMU die. Used for bias compensation
  (sensor bias changes with temperature).

The SPI master is the hardware interface that reads these measurements from the
IMU chip at high speed. It must operate deterministically within the control loop
timing budget.

---

## 2. IMU Sensor Options

### 2.1 Comparison Table

| Parameter            | MPU-6500        | ICM-20689       | ICM-42688-P     | BMI270          | LSM6DSO         |
|----------------------|-----------------|-----------------|-----------------|-----------------|-----------------|
| Manufacturer         | InvenSense      | InvenSense      | TDK InvenSense  | Bosch           | ST Micro        |
| Gyro noise density   | 0.015 °/s/√Hz   | 0.008 °/s/√Hz   | 0.0028 °/s/√Hz  | 0.014 °/s/√Hz   | 0.004 °/s/√Hz   |
| Accel noise density  | 300 µg/√Hz      | 230 µg/√Hz      | 70 µg/√Hz       | 160 µg/√Hz      | 80 µg/√Hz       |
| Gyro range options   | ±250–2000°/s    | ±250–2000°/s    | ±250–2000°/s    | ±125–2000°/s    | ±125–2000°/s    |
| Accel range options  | ±2–16 g         | ±2–16 g         | ±2–16 g         | ±2–16 g         | ±2–16 g         |
| ADC resolution       | 16-bit          | 16-bit          | 16-bit          | 16-bit          | 16-bit          |
| Max output data rate | 8 kHz           | 32 kHz          | 32 kHz          | 6.4 kHz         | 6.66 kHz        |
| SPI max clock (read) | 1 MHz           | 8 MHz           | 24 MHz          | 10 MHz          | 10 MHz          |
| Built-in anti-alias  | No              | No              | Yes (configurable)| Yes            | Yes             |
| Temp sensitivity     | High            | Medium          | Low             | Medium          | Low             |
| Price (2024)         | ~$3             | ~$4             | ~$5             | ~$4             | ~$4             |
| Availability         | EOL risk        | Good            | Excellent       | Excellent       | Good            |
| Used in              | Older FCs       | BetaFlight FCs  | Latest FCs      | DJI products    | ArduPilot       |

### 2.2 Detailed Analysis

**MPU-6500:**
- Advantages: Cheap, widely documented, large community
- Disadvantages: High noise (5× worse than ICM-42688-P), slow SPI (1 MHz max
  for reads — takes 120 µs per burst, consuming 6% of control period), no
  hardware anti-alias filter, end-of-life risk
- Impact on design: Requires aggressive digital filtering (fc < 30 Hz) which
  adds 5+ ms group delay, destroying phase margin at high bandwidth

**ICM-20689:**
- Advantages: Good noise, fast SPI (8 MHz), proven in Betaflight
- Disadvantages: No hardware AAF, slightly worse noise than ICM-42688-P
- Impact on design: Solid choice, but ICM-42688-P is strictly better

**ICM-42688-P (RECOMMENDED):**
- Advantages:
  - Lowest gyro noise (0.0028 °/s/√Hz) — 5× better than MPU-6500
  - Fastest SPI (24 MHz) — burst read in 5 µs
  - Built-in configurable anti-alias filter (hardware LPF before ADC)
  - Low temperature sensitivity
  - 32 kHz ODR — far exceeds our 1 kHz requirement
  - FIFO buffer — prevents data loss if read timing jitters
- Disadvantages: Slightly more expensive ($5 vs $3)
- Impact on design: Allows higher control bandwidth (fc = 80 Hz instead of 30 Hz),
  reducing phase margin loss. Fastest SPI frees timing budget for computation.

**BMI270:**
- Advantages: Used in DJI products (proven in production), good availability
- Disadvantages: Lower max ODR (6.4 kHz), slightly higher noise than ICM-42688-P
- Impact on design: Adequate but not optimal

**LSM6DSO:**
- Advantages: Low noise, good temperature stability, SPI+I²C
- Disadvantages: Lower max ODR, ST's register interface is more complex
- Impact on design: Good alternative if TDK parts are unavailable

### 2.3 Sensor Selection Rationale

**Chosen: ICM-42688-P**

The noise floor directly determines the minimum useful filter cutoff:
```
Noise at output = noise_density × √(bandwidth)
ICM-42688-P: 0.0028 × √(80) = 0.025 °/s RMS  → fc=80 Hz works
MPU-6500:    0.015  × √(80) = 0.134 °/s RMS  → too noisy, must use fc=30 Hz
```

Lower filter cutoff = more group delay = less phase margin = less stable drone.
The ICM-42688-P allows 2.7× higher control bandwidth than MPU-6500.

---

## 3. SPI Protocol Theory

### 3.1 Physical Interface

SPI is a synchronous, full-duplex serial protocol with 4 signals:

| Signal | Direction    | Description                                     |
|--------|--------------|-------------------------------------------------|
| SCLK   | Master → Slave | Clock signal generated by FPGA                |
| MOSI   | Master → Slave | Master Out, Slave In (commands/addresses)     |
| MISO   | Slave → Master | Master In, Slave Out (sensor data)            |
| CS#    | Master → Slave | Chip Select (active low, selects the device)  |

Data is exchanged simultaneously on MOSI and MISO — one bit per SCLK edge.
The master (FPGA) always generates the clock; the slave (IMU) responds.

### 3.2 SPI Modes (Clock Polarity and Phase)

| Mode | CPOL | CPHA | SCLK Idle | Data Sampled On | Data Shifted On |
|------|------|------|-----------|-----------------|-----------------|
| 0    | 0    | 0    | Low       | Rising edge     | Falling edge    |
| 1    | 0    | 1    | Low       | Falling edge    | Rising edge     |
| 2    | 1    | 0    | High      | Falling edge    | Rising edge     |
| 3    | 1    | 1    | High      | Rising edge     | Falling edge    |

**ICM-42688-P uses Mode 3 (CPOL=1, CPHA=1):**
- SCLK idles HIGH
- Data is shifted out on the falling edge of SCLK
- Data is sampled (captured) on the rising edge of SCLK
- MSB transmitted first

### 3.3 ICM-42688-P SPI Timing Requirements

| Parameter              | Min   | Typical | Max   | Unit |
|------------------------|-------|---------|-------|------|
| SCLK frequency (read)  |       |         | 24    | MHz  |
| SCLK frequency (write) |       |         | 1     | MHz  |
| CS# setup before SCLK  | 5     |         |       | ns   |
| CS# hold after SCLK    | 5     |         |       | ns   |
| MISO output delay       |       |         | 35    | ns   |
| CS# high between xfers | 15    |         |       | ns   |

**FPGA SPI clock derivation:**
- System clock: 100 MHz (10 ns period)
- For 8 MHz SPI: divide by 12.5 → use divide-by-13 (7.69 MHz) or divide-by-12 (8.33 MHz)
- Simpler: divide-by-12 gives SCLK = 8.33 MHz, period = 120 ns
- Well within 24 MHz max, with comfortable timing margins

### 3.4 Read Protocol

To read a register from ICM-42688-P:

```
1. Assert CS# low (wait ≥ 5 ns)
2. Transmit address byte: bit[7]=1 (read), bit[6:0]=register_address
3. Simultaneously receive: dummy byte (ignore)
4. For each additional byte: transmit 0x00, receive data byte
5. Deassert CS# high (wait ≥ 15 ns before next transaction)
```

**Burst read (auto-increment):** After the address byte, the sensor
automatically increments the register address for each subsequent byte.
This allows reading 14 consecutive registers in one CS# assertion.

### 3.5 IMU Register Map (Key Registers)

| Register  | Address | Content                              |
|-----------|---------|--------------------------------------|
| ACCEL_XOUT_H | 0x1F | Accel X [15:8]                      |
| ACCEL_XOUT_L | 0x20 | Accel X [7:0]                       |
| ACCEL_YOUT_H | 0x21 | Accel Y [15:8]                      |
| ACCEL_YOUT_L | 0x22 | Accel Y [7:0]                       |
| ACCEL_ZOUT_H | 0x23 | Accel Z [15:8]                      |
| ACCEL_ZOUT_L | 0x24 | Accel Z [7:0]                       |
| GYRO_XOUT_H  | 0x25 | Gyro X [15:8]                       |
| GYRO_XOUT_L  | 0x26 | Gyro X [7:0]                        |
| GYRO_YOUT_H  | 0x27 | Gyro Y [15:8]                       |
| GYRO_YOUT_L  | 0x28 | Gyro Y [7:0]                        |
| GYRO_ZOUT_H  | 0x29 | Gyro Z [15:8]                       |
| GYRO_ZOUT_L  | 0x2A | Gyro Z [7:0]                        |
| TEMP_OUT_H   | 0x1D | Temperature [15:8]                  |
| TEMP_OUT_L   | 0x1E | Temperature [7:0]                   |

Burst read starting at 0x1F captures all 14 bytes (accel XYZ + gyro XYZ + temp)
in a single SPI transaction.

---

## 4. Module Interface

### 4.1 Inputs

| Signal        | Width | Format          | Source         | Description                         |
|---------------|-------|-----------------|----------------|-------------------------------------|
| clk           | 1     | Clock           | System         | 100 MHz system clock                |
| rst_n         | 1     | Active-low      | System         | Synchronous reset                   |
| tick_1khz     | 1     | Pulse           | Tick Generator | Triggers IMU read every 1 ms        |
| spi_miso      | 1     | Serial          | ICM-42688-P    | SPI data from sensor                |

### 4.2 Outputs

| Signal        | Width | Format          | Dest           | Description                         |
|---------------|-------|-----------------|----------------|-------------------------------------|
| spi_sclk      | 1     | Clock           | ICM-42688-P    | SPI clock (8 MHz)                   |
| spi_mosi      | 1     | Serial          | ICM-42688-P    | SPI data to sensor                  |
| spi_cs_n      | 1     | Active-low      | ICM-42688-P    | SPI chip select                     |
| accel_x       | 32    | Q16.16 signed   | Calibration    | Accel X in g units                  |
| accel_y       | 32    | Q16.16 signed   | Calibration    | Accel Y in g units                  |
| accel_z       | 32    | Q16.16 signed   | Calibration    | Accel Z in g units                  |
| gyro_x        | 32    | Q16.16 signed   | Calibration    | Gyro X in °/s                       |
| gyro_y        | 32    | Q16.16 signed   | Calibration    | Gyro Y in °/s                       |
| gyro_z        | 32    | Q16.16 signed   | Calibration    | Gyro Z in °/s                       |
| temp_raw      | 32    | Q16.16 signed   | CPU (optional) | Die temperature in °C               |
| imu_valid     | 1     | Pulse           | Calibration    | New data available (1 clk pulse)    |

### 4.3 Configuration (from CPU via AXI)

| Register      | Width | Default    | Description                              |
|---------------|-------|------------|------------------------------------------|
| gyro_range    | 2     | 2'b11      | 00=±250, 01=±500, 10=±1000, 11=±2000°/s |
| accel_range   | 2     | 2'b11      | 00=±2g, 01=±4g, 10=±8g, 11=±16g         |
| odr_div       | 8     | 8'd0       | ODR divider (0 = 1kHz at default config) |

---

## 5. Theoretical Foundations

### 5.1 Gyroscope Physics

A MEMS gyroscope exploits the Coriolis effect: a vibrating mass experiences a
force perpendicular to both its vibration direction and any applied rotation.

```
F_coriolis = -2m · (ω × v)
```

Where:
- m = proof mass
- ω = angular velocity vector (what we're measuring)
- v = vibration velocity of the proof mass

The Coriolis force displaces the mass proportionally to ω, and this displacement
is detected capacitively. The sensor outputs a voltage proportional to angular rate.

**Sensitivity (at ±2000°/s, 16-bit):**
```
LSB = full_range / 2^(bits-1) = 2000 / 32768 = 0.061 °/s per LSB
```

### 5.2 Accelerometer Physics

A MEMS accelerometer measures specific force: the difference between true
acceleration and gravitational acceleration.

```
a_measured = a_true - g
```

In static conditions (hovering): a_true = 0, so a_measured = -g = [0, 0, -9.81] m/s²
(or [0, 0, -1] in g units, with z pointing up).

In the sensor body frame with z pointing DOWN (NED convention):
a_measured = [0, 0, +1g] when level.

**Sensitivity (at ±16g, 16-bit):**
```
LSB = 16 / 32768 = 0.000488 g per LSB
```

### 5.3 Raw-to-Physical Conversion

The sensor outputs 16-bit signed integers. We must convert to Q16.16
fixed-point physical units:

**Gyro (±2000°/s range):**
```
physical_rate = raw_16bit × (2000.0 / 32768)  [°/s]

In Q16.16: multiply raw by scale factor
GYRO_SCALE = round((2000/32768) × 65536) = round(3.99999 × 65536) = 0x0003FFFF
           ≈ 0x00040000 (simpler: exactly 4.0 in Q16.16)

Actually: 2000/32768 = 125/2048 ≈ 0.061035
In Q16.16: 0.061035 × 65536 = 4000 = 0x00000FA0

More precisely: raw × 4000 gives result in Q16.16 °/s
(since raw is integer, this is: result = raw × 0x00000FA0, already Q16.16)
```

**Accel (±16g range):**
```
physical_accel = raw_16bit × (16.0 / 32768)  [g]

In Q16.16:
ACCEL_SCALE = round((16/32768) × 65536) = round(0.000488 × 65536) = 32 = 0x00000020

result = raw × 32 (simple left-shift by 5)
```

**Temperature:**
```
temp_°C = (raw / 132.48) + 25
In Q16.16: temp = raw × TEMP_SCALE + TEMP_OFFSET
TEMP_SCALE = round((1/132.48) × 65536) = 495 = 0x000001EF
TEMP_OFFSET = 25 × 65536 = 0x00190000
```

---

## 6. Algorithm (Pseudocode)

### 6.1 SPI Master Engine

```
PARAMETERS:
    CLK_DIV = 12          // 100 MHz / 12 = 8.33 MHz SPI clock
    BURST_LEN = 15        // 1 address byte + 14 data bytes
    START_REG = 0x1F      // ACCEL_XOUT_H (first of 14 consecutive registers)

STATE MACHINE:
    IDLE:
        spi_cs_n = 1
        spi_sclk = 1      // idle high (Mode 3)
        ON tick_1khz → go to CS_SETUP

    CS_SETUP:
        spi_cs_n = 0       // assert chip select
        wait 1 clock cycle (10 ns > 5 ns minimum)
        → go to TRANSFER, byte_count = 0, bit_count = 7

    TRANSFER:
        // Generate SCLK at CLK_DIV rate
        // On falling SCLK: shift out MOSI bit (MSB first)
        // On rising SCLK: sample MISO bit

        IF byte_count == 0:
            tx_byte = 0x80 | START_REG    // read command
        ELSE:
            tx_byte = 0x00                 // dummy for read

        // After 8 bits:
        rx_buffer[byte_count] = received_byte
        byte_count = byte_count + 1

        IF byte_count == BURST_LEN:
            → go to CS_HOLD
        ELSE:
            bit_count = 7, continue TRANSFER

    CS_HOLD:
        wait 1 clock cycle
        spi_cs_n = 1       // deassert chip select
        → go to CONVERT

    CONVERT:
        // Reassemble and scale (see section 6.2)
        → go to IDLE, assert imu_valid for 1 cycle
```

### 6.2 Data Reassembly and Conversion

```
ON entering CONVERT state:
    // Reassemble 16-bit signed from big-endian bytes
    // rx_buffer[0] is dummy (response to address byte)
    accel_x_raw = sign_extend_16(rx_buffer[1] << 8 | rx_buffer[2])
    accel_y_raw = sign_extend_16(rx_buffer[3] << 8 | rx_buffer[4])
    accel_z_raw = sign_extend_16(rx_buffer[5] << 8 | rx_buffer[6])
    gyro_x_raw  = sign_extend_16(rx_buffer[7] << 8 | rx_buffer[8])
    gyro_y_raw  = sign_extend_16(rx_buffer[9] << 8 | rx_buffer[10])
    gyro_z_raw  = sign_extend_16(rx_buffer[11] << 8 | rx_buffer[12])
    temp_raw_i  = sign_extend_16(rx_buffer[13] << 8 | rx_buffer[14])

    // Convert to Q16.16 physical units
    // Gyro: raw × 0x00000FA0 → Q16.16 °/s
    // (16-bit × 16-bit = 32-bit, already in Q16.16 since raw is integer)
    gyro_x = gyro_x_raw × GYRO_SCALE
    gyro_y = gyro_y_raw × GYRO_SCALE
    gyro_z = gyro_z_raw × GYRO_SCALE

    // Accel: raw × 32 = raw << 5 → Q16.16 g
    accel_x = accel_x_raw << 5
    accel_y = accel_y_raw << 5
    accel_z = accel_z_raw << 5

    // Temperature: raw × 495 + 25.0
    temp_raw = temp_raw_i × 0x000001EF + 0x00190000

    // Assert output valid
    imu_valid = 1 (single-cycle pulse)

FUNCTION sign_extend_16(val_16) → 32-bit signed:
    IF val_16[15] == 1:
        RETURN 0xFFFF0000 | val_16
    ELSE:
        RETURN val_16
```

### 6.3 IMU Initialization Sequence (at power-up)

```
// Performed once at startup (can be slow, uses 1 MHz SPI write clock)
INIT SEQUENCE:
    1. Wait 100 ms after power-on (sensor boot time)
    2. Write DEVICE_CONFIG (0x11) = 0x01      // soft reset
    3. Wait 1 ms
    4. Write GYRO_CONFIG0 (0x4F) = 0x06       // ±2000°/s, ODR=1kHz
    5. Write ACCEL_CONFIG0 (0x50) = 0x06      // ±16g, ODR=1kHz
    6. Write GYRO_CONFIG1 (0x51) = 0x01       // enable gyro LN mode
    7. Write PWR_MGMT0 (0x4E) = 0x0F         // gyro + accel ON, LN mode
    8. Wait 50 ms (sensor stabilization)
    9. Ready for burst reads
```

---

## 7. Timing Analysis

**Burst read at 8.33 MHz SPI:**
```
Bits per transaction = 15 bytes × 8 bits = 120 bits
Time = 120 / 8.33 MHz = 14.4 µs
Plus CS setup/hold: ~0.1 µs
Total: ~14.5 µs per IMU read
```

**Percentage of 2 ms control period:** 14.5 / 2000 = 0.73%

**If using 24 MHz SPI (maximum):**
```
Time = 120 / 24 MHz = 5.0 µs (0.25% of period)
```

We use 8.33 MHz as a conservative choice — still well within budget, and
provides comfortable timing margins for MISO setup/hold.

---

## 8. Error Handling

| Condition              | Detection Method              | Response                        |
|------------------------|-------------------------------|---------------------------------|
| IMU not responding     | All-zeros or all-ones on MISO | Assert imu_timeout flag         |
| Data corruption        | Unexpected WHO_AM_I value     | Re-initialize IMU               |
| Read overrun           | tick_1khz while still reading | Skip (rely on previous data)    |
| Temperature out of range| temp > 85°C or < -40°C       | Flag to CPU for monitoring      |

---

## 9. Resource Estimate

| Resource       | Count  | Notes                                    |
|----------------|--------|------------------------------------------|
| Flip-flops     | ~150   | Shift registers, state machine, counters |
| LUTs           | ~200   | Clock divider, mux, scaling              |
| DSP48E1        | 1      | Gyro scaling multiply (shared/muxed)     |
| Block RAM      | 0      | 15-byte buffer fits in FFs               |
| I/O pins       | 4      | SCLK, MOSI, MISO, CS#                   |
