# Module 01: GY-91 Sensor Acquisition Interface

> The filename is retained for compatibility. The selected baseline interface
> is shared I²C; SPI is an optional board-specific optimization.

## 1. Selected Sensor

The selected sensor module is the **GY-91**:

- **MPU9250:** 16-bit 3-axis accelerometer and gyroscope;
- **AK8963:** 16-bit 3-axis magnetometer inside the MPU9250 package;
- **BMP280:** pressure and temperature sensor on the same breakout.

Together these provide ten measured quantities: acceleration (3), angular rate
(3), magnetic field (3), and pressure (1), plus sensor temperatures. The GY-91
variant used by this project is the eight-pin board shown in the
[Electropeak GY-91 guide](https://electropeak.com/learn/interfacing-gy-91-9-axis-mpu9250-bmp280-module-with-arduino/).
Firmware and RTL must still verify each device independently at startup.

### 1.1 Exact breakout pinout

| Board pin | I²C function | SPI function | Project use |
|---|---|---|---|
| `VIN` | 5 V module supply | 5 V module supply | Do not use when powering through `3V3` |
| `GND` | Ground | Ground | Common FPGA/module ground |
| `3V3` | 3.3 V module supply | 3.3 V module supply | Preferred FPGA-compatible supply input |
| `SCL` | Shared I²C clock | Shared SPI clock | RTL serial clock |
| `SDA` | Shared I²C data | Shared MOSI | Bidirectional SDA in baseline mode |
| `SA0/SDO` | Address selection | Shared MISO | Strap for I²C; input to FPGA in SPI mode |
| `NCS` | Hold high | MPU9250 chip select | Hold high in I²C baseline |
| `CSB` | Hold high | BMP280 chip select | Hold high in I²C baseline |

This variant does **not expose the MPU9250 `INT` pin**, so the baseline RTL
cannot use a hardware data-ready interrupt. It uses deterministic scheduled
polling and checks sensor status/data freshness instead. Never power `VIN` and
`3V3` simultaneously. Before connecting to a particular FPGA board, measure the
module's I/O-high voltage and confirm that any on-board pull-ups go to 3.3 V.

## 2. Physical Bus Choice

### 2.1 Baseline: shared I²C

Use one 400 kHz I²C master in RTL for the complete GY-91 module. The referenced
board and example use this shared-bus connection, and it gives direct access to
all three sensors with the least RTL complexity.
Expected 7-bit addresses are:

| Device | Address | Identification |
|---|---:|---|
| MPU9250 | `0x68` or `0x69` | `WHO_AM_I` (`0x75`) normally returns `0x71` |
| AK8963 | `0x0C` | `WIA` (`0x00`) returns `0x48` |
| BMP280 | `0x76` or `0x77` | `id` (`0xD0`) returns `0x58` |

`SA0/SDO` selects the address strap; the resulting MPU9250 and BMP280 addresses
must be confirmed with an address scan during bring-up. Address values remain
configurable/discovered at startup so clone or assembly differences fail
cleanly rather than being mistaken for valid sensors.

### 2.2 Optional SPI mode

This exact board exposes separate `NCS` and `CSB` pins, so the MPU9250 and
BMP280 can share SCLK/MOSI/MISO and be selected independently in SPI mode. SPI
substantially reduces gyro/accelerometer bus occupancy. The AK8963 is still not
a host-SPI device: RTL must configure the MPU9250 auxiliary I²C master and read
magnetometer bytes through `EXT_SENS_DATA` registers. That extra controller
complexity is why shared I²C remains the first implementation. The AXI contract
and downstream modules do not change if SPI is added later.

## 3. MPU9250 Acquisition

### 3.1 Relevant registers

| Register | Address | Purpose |
|---|---:|---|
| `SMPLRT_DIV` | `0x19` | Gyro sample-rate divider |
| `CONFIG` | `0x1A` | Gyro DLPF configuration |
| `GYRO_CONFIG` | `0x1B` | Gyroscope full scale |
| `ACCEL_CONFIG` | `0x1C` | Accelerometer full scale |
| `ACCEL_CONFIG2` | `0x1D` | Accelerometer DLPF |
| `INT_PIN_CFG` | `0x37` | Interrupt/bypass configuration |
| `INT_ENABLE` | `0x38` | Data-ready interrupt enable |
| `ACCEL_XOUT_H` | `0x3B` | Start of 14-byte sensor burst |
| `PWR_MGMT_1` | `0x6B` | Reset, sleep, and clock source |
| `USER_CTRL` | `0x6A` | I²C-master/SPI control |
| `WHO_AM_I` | `0x75` | Device identity |

A 14-byte burst from `0x3B` returns accelerometer XYZ, temperature, and
gyroscope XYZ in big-endian 16-bit words.

### 3.2 Initial operating point

- sample gyro and accelerometer at 1 kHz;
- gyroscope full scale: ±2000 degrees/s (16.4 LSB per degree/s);
- accelerometer full scale: ±16 g (2048 LSB/g);
- enable the MPU9250 digital low-pass filters;
- use a deterministic 1 kHz RTL scheduler because this breakout does not expose
  the MPU9250 data-ready interrupt.

The exact DLPF bandwidth is a tuning parameter. MPU9250 noise and the airframe's
vibration spectrum must be measured before assuming an 80 Hz control bandwidth.

### 3.3 Raw conversion

```text
gyro_deg_s = gyro_raw / 16.4
accel_g    = accel_raw / 2048
temp_C     = temp_raw / 333.87 + 21
```

RTL may expose raw signed samples to the CPU and separately provide calibrated
Q16.16 values to the RTL controller. Keeping raw values in the AXI snapshot is
important for estimator tuning, calibration, and logging.

## 4. RTL Module Interface

Suggested top-level block: `gy91_sensor_if`.

### 4.1 External signals

| Signal | Direction | Description |
|---|---|---|
| `i2c_scl_o` | output | Open-drain SCL drive-low control |
| `i2c_sda_o` | output | Open-drain SDA drive-low control |
| `i2c_sda_i` | input | Sampled SDA |
| `mpu_ncs_o` | output | Held high in I²C mode; MPU9250 select in optional SPI mode |
| `bmp_csb_o` | output | Held high in I²C mode; BMP280 select in optional SPI mode |

The top-level I/O buffers must implement open-drain behavior; RTL never drives
SCL or SDA high.

### 4.2 Internal outputs

| Signal | Format | Consumer |
|---|---|---|
| `accel_raw[2:0]` | 3 x signed 16-bit | AXI snapshot/calibration |
| `gyro_raw[2:0]` | 3 x signed 16-bit | AXI snapshot/calibration |
| `imu_temp_raw` | signed 16-bit | AXI snapshot |
| `imu_valid` | one-cycle pulse | filter, watchdog, snapshot logic |
| `sample_seq` | unsigned 32-bit | CPU frame-loss detection |
| `sample_timestamp` | unsigned 64-bit | Estimator time step and logging |
| `sensor_error` | bit field | CPU and failsafe |

Magnetometer and barometer payloads are specified in Modules 10 and 09. All
three clients share one I²C byte engine and an RTL transaction scheduler.

## 5. AXI4-Lite Snapshot

When `imu_valid` occurs, RTL copies the entire IMU frame into shadow registers,
increments `sample_seq`, records a free-running timestamp, and raises an
interrupt status bit. The CPU reads the shadow frame and acknowledges the
interrupt. The next acquisition may update the live datapath without tearing
the CPU-visible frame.

At minimum the snapshot contains raw accel, gyro, IMU temperature, latest mag,
latest pressure/temperature, per-sensor validity/age, sequence, and timestamp.

## 6. Initialization Sequence

```text
1. Wait for GY-91 power stabilization.
2. Probe both MPU9250 addresses and verify WHO_AM_I.
3. Reset MPU9250, select a stable clock source, and wake it.
4. Configure gyro/accelerometer ranges, DLPFs, and 1 kHz sample divider.
5. Configure and start the deterministic RTL polling scheduler.
6. Initialize AK8963 and read its sensitivity-adjustment values.
7. Initialize BMP280 and read all factory compensation coefficients.
8. Take stationary samples; CPU calculates boot-time bias.
9. Mark the sensor subsystem ready only when required identities and data
   freshness checks pass.
```

Configuration writes must be read back where the device permits it.

## 7. Timing and Errors

A 14-byte MPU9250 read over 400 kHz I²C occupies roughly 0.4 ms including
addressing and protocol overhead, leaving room in a 1 ms period for scheduled
magnetometer and barometer transactions. Do not poll every device at 1 kHz:
magnetometer and pressure updates are much slower.

| Error | RTL response |
|---|---|
| Address NACK or wrong identity | Keep sensor not-ready; set sticky error |
| I²C bus held low | Time out, release bus, attempt bus recovery |
| New trigger while busy | Count overrun; preserve last coherent sample |
| IMU sample stale | Assert watchdog fault and inhibit arming |
| Mag/baro sample stale | Mark measurement invalid for estimator/CPU |

## 8. Resource Direction

Use one shared I²C byte engine, a transaction scheduler, and small register
buffers. Attitude math belongs to Module 04; pressure compensation and EKF
matrix arithmetic, when enabled, belong to the CPU.
