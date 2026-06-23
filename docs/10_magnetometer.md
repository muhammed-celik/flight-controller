# Module 10: GY-91 AK8963 Magnetometer Interface

## 1. Selected Sensor and Partition

The selected magnetometer is the **AK8963 contained inside the MPU9250 on the
GY-91 module**. No separate QMC5883L is used.

RTL acquires coherent raw magnetic samples and status. CPU software normally
performs hard-iron/soft-iron calibration, frame rotation, and validation. EKF
mode uses the result directly; RTL complementary mode may initially omit
magnetic yaw correction or consume CPU-calibrated heading later.

## 2. Access Path and Identification

In the shared-I²C baseline used by the selected eight-pin breakout, configure
MPU9250 `INT_PIN_CFG.BYPASS_EN` so the host can address the AK8963 directly at
7-bit address `0x0C`. Its `WIA` register at `0x00` should return `0x48`.

If a verified board implementation later uses MPU9250 SPI, the AK8963 must be
read through the MPU9250 auxiliary I²C master and its bytes mirrored into
`EXT_SENS_DATA` registers. Downstream AXI and EKF interfaces remain unchanged.

## 3. Relevant Registers

| Register | Address | Purpose |
|---|---:|---|
| `WIA` | `0x00` | Device identity (`0x48`) |
| `ST1` | `0x02` | Data-ready flag |
| `HXL..HZH` | `0x03..0x08` | XYZ data, little-endian |
| `ST2` | `0x09` | Overflow status; must complete each read |
| `CNTL1` | `0x0A` | Mode and output resolution |
| `CNTL2` | `0x0B` | Soft reset |
| `ASAX..ASAZ` | `0x10..0x12` | Factory sensitivity adjustment |

A valid measurement transaction reads eight bytes from `ST1` through `ST2`.
The axis words are little-endian, unlike the MPU9250 accel/gyro burst. Reading
`ST2` is required to release the sensor data latch.

## 4. Initialization

```text
1. Enable MPU9250 I²C bypass and verify AK8963 WIA == 0x48.
2. Write power-down mode and wait at least 100 microseconds.
3. Enter fuse-ROM access mode.
4. Read ASAX, ASAY, and ASAZ.
5. Return to power-down and wait again.
6. Select 16-bit continuous-measurement mode 2 (100 Hz).
7. Schedule reads at 50-100 Hz.
```

For each axis, CPU software applies the factory adjustment approximately as

```text
adjustment = ((ASA - 128) / 256) + 1
field_uT = raw * adjustment * 0.15
```

Retain the raw word and ASA byte in the AXI-visible data for reproducible
calibration and logging.

## 5. RTL Responsibilities

Suggested logical client: `ak8963_client`, using the shared GY-91 I²C engine.

| Output | Width | Description |
|---|---:|---|
| `mag_raw[2:0]` | 3 x signed 16 | Little-endian XYZ words |
| `mag_valid` | 1 | Pulse only for DRDY=1 and HOFL=0 |
| `mag_overflow` | 1 | `ST2.HOFL` from latest transaction |
| `mag_seq` | 32 | Monotonic valid sample number |
| `mag_timestamp` | 64 | Acquisition timestamp |
| `mag_error` | 1 | Sticky NACK/timeout/identity fault |

The scheduler must not reread stale data as if it were new. If `ST1.DRDY` is
clear, retain the previous frame and do not increment `mag_seq`.

## 6. Calibration and Estimator Use

Hard-iron calibration subtracts an offset vector. Soft-iron calibration applies
a 3 x 3 matrix:

```text
m_cal = S * (m_raw - b)
```

Calibration is computed and applied on the CPU. The CPU also rotates the vector
from sensor axes into the vehicle body frame. Axis signs and permutation must be
measured for the actual GY-91 mounting orientation; they must not be inferred
from a generic breakout-board photo.

The CPU EKF, or any later RTL heading-correction extension, uses a magnetic
observation only when:

- the sample is fresh and not overflowed;
- calibrated magnitude lies within configured local limits;
- the innovation passes a statistical gate; and
- the vehicle is not in a known high-current interference condition.

Rejecting a bad magnetic update is safer than pulling yaw toward a corrupted
heading. Gyroscope prediction and accel-based roll/pitch updates continue when
mag data is unavailable.

## 7. Bus Scheduling and Fault Behavior

Gyro/accelerometer acquisition at 1 kHz has first priority, AK8963 runs at
50-100 Hz, and BMP280 at 25-50 Hz. One shared transaction scheduler prevents
electrical contention and records data age independently for each sensor.

| Failure | Response |
|---|---|
| AK8963 absent or wrong WIA | Disable heading-hold; continue gyro/accel control |
| Overflow bit set | Discard sample and count overflow |
| Stale sample | Skip magnetic estimator update |
| Implausible field/innovation | CPU rejects update and raises diagnostic flag |
| Shared bus timeout | RTL performs bus recovery and retries later |

Magnetometer loss alone does not require immediate disarm, but autonomous modes
that require absolute heading must be inhibited.
