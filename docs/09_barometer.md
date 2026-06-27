# Module 09: GY-91 BMP280 Barometer Interface

## 1. Selected Sensor and Partition

The selected barometer is the **BMP280 already present on the GY-91 module**.
No separate BMP390 is used.

RTL schedules raw BMP280 transactions, validates identity, and retains coherent
ADC values plus factory coefficients. The barometer is not required for
first-flight manual rate/attitude control. When a CPU is added, software
performs Bosch compensation, pressure-to-altitude conversion, and any
vertical-state filtering or EKF update. A future RTL altitude-hold extension may
also consume the raw/coefficient data.

## 2. Interface and Identification

The BMP280 shares the GY-91 I²C bus with the MPU9250 subsystem. On the selected
eight-pin breakout, `CSB` is held high for I²C and becomes the dedicated BMP280
chip select if optional SPI mode is implemented. Its 7-bit address is normally
`0x76` or `0x77`, depending on the `SA0/SDO` strap, and must be probed. Register
`id` at `0xD0` should return `0x58`.

| Register | Address | Purpose |
|---|---:|---|
| `id` | `0xD0` | Chip identification (`0x58`) |
| `reset` | `0xE0` | Soft reset (`0xB6`) |
| `status` | `0xF3` | Measuring/NVM-copy status |
| `ctrl_meas` | `0xF4` | Temperature/pressure oversampling and mode |
| `config` | `0xF5` | Standby time, IIR filter, SPI mode |
| `press_msb..xlsb` | `0xF7..0xF9` | 20-bit raw pressure |
| `temp_msb..xlsb` | `0xFA..0xFC` | 20-bit raw temperature |
| calibration block | `0x88..0xA1` | `dig_T1..T3`, `dig_P1..P9` |

The 20-bit raw values are reconstructed as:

```text
adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4)
adc_T = (temp_msb  << 12) | (temp_lsb  << 4) | (temp_xlsb  >> 4)
```

## 3. Operating Point

Start with pressure oversampling x8 or x16, temperature oversampling x2, the
internal IIR filter enabled, and a 25-50 Hz output rate. The final settings must
be tuned from propeller-on vibration and pressure-noise logs.

Pressure is not sampled at the 1 kHz IMU rate. The shared RTL bus scheduler
gives gyro/accelerometer reads priority and inserts barometer transactions in
the remaining slots.

## 4. RTL Responsibilities

Suggested logical client: `bmp280_client`, using the shared `gy91_i2c_master`.

| Output | Width | Description |
|---|---:|---|
| `baro_pressure_raw` | 20 | Uncompensated pressure ADC value |
| `baro_temperature_raw` | 20 | Uncompensated temperature ADC value |
| `baro_valid` | 1 | Pulse when a coherent pair is captured |
| `baro_seq` | 32 | Monotonic sample number |
| `baro_timestamp` | 64 | Acquisition timestamp |
| `baro_error` | 1 | Sticky NACK/timeout/identity fault |

At initialization RTL reads and retains all calibration bytes. They are exposed
to optional AXI with the correct signedness: `dig_T1`, `dig_P1` are unsigned;
the remaining coefficients are signed 16-bit values.

## 5. Initialization and Acquisition

```text
1. Probe 0x76 and 0x77; require id == 0x58.
2. Issue reset 0xB6 and wait until status.im_update clears.
3. Read the complete calibration block once.
4. Reject obviously invalid coefficients, especially dig_P1 == 0.
5. Configure ctrl_meas and config, then read them back.
6. At each scheduled barometer slot, burst-read 0xF7..0xFC.
7. Reconstruct adc_P and adc_T, timestamp them, increment baro_seq, and pulse
   baro_valid.
```

## 6. Future CPU Compensation and Altitude

When enabled later, CPU software implements the BMP280 datasheet compensation
algorithm exactly, including its wide intermediate values. The result is
compensated temperature and pressure in Pa. Do not substitute BMP390
coefficients or formulas; the two devices have different calibration models and
register maps.

An approximate altitude relative to reference pressure `p0` is

```text
h = 44330 * (1 - (p / p0)^0.190294957)
```

`p0` should be captured or configured near takeoff. Prop wash, enclosure
pressure, weather, and temperature cause errors, so pressure is fused rather
than treated as ground truth. Neither attitude-estimator choice requires
pressure; an EKF vertical-state extension or separate CPU altitude filter may
use it.

## 7. AXI4-Lite and Error Handling

When AXI support is included, the CPU-visible sensor snapshot includes raw
pressure/temperature, timestamp, sequence, valid/stale status, and calibration
coefficients. RTL reset defaults select oversampling, IIR, output rate, and
enable state; optional AXI fields may override them later.

| Failure | Response |
|---|---|
| Both addresses NACK | Mark barometer absent; inhibit altitude-hold mode |
| Wrong chip ID | Set sticky identity fault; do not apply BMP280 formulas |
| Stale sample | RTL reports age; future CPU skips pressure update |
| Bus timeout | Recover shared bus and retry at next barometer slot |
| Invalid coefficient block | Disable compensation and report configuration fault |

A barometer failure need not disarm manual rate/attitude flight, but it must
prevent or terminate altitude-hold behavior.
