# IMU Programming Guide — ISM330DHCX + MMC5983MA

Host-side programming sequence for the SparkFun 9DoF IMU breakout
(ISM330DHCX accel/gyro + MMC5983MA magnetometer), written for an FPGA
implementation. This document is interface/protocol only — no RTL.

Sources: ST `ism330dhcx_reg.c/.h` (STMems driver), ISM330DHCX datasheet,
MCC5983MA datasheet Rev A, SparkFun MMC5983MA Arduino library.
Downloaded copies live in `~/Projects/imu_drivers/`.

---

## 0. Quick facts

| Item | ISM330DHCX | MMC5983MA |
|---|---|---|
| Function | 3-axis accel + 3-axis gyro | 3-axis magnetometer |
| Bus here | SPI (I2C also possible) | I2C only |
| 7-bit I2C address | `0x6A` (SA0=0) / `0x6B` (SA0=1, default) | `0x30` (fixed) |
| ID register | `WHO_AM_I` 0x0F = `0x6B` | `PROD_ID` 0x2F = `0x30` |
| Output width | 16-bit signed per axis | 18-bit unsigned per axis |
| Data order | low byte first (little-endian) | see §5.2 |
| Data-ready | `STATUS_REG` 0x1E (XLDA/GDA/TDA) | `STATUS` 0x08 (MEAS_M_DONE) |

**Bus conventions**

- SPI read: set bit 7 of the address byte (`addr | 0x80`); write uses bit 7 = 0.
- SPI/I2C multi-byte access needs `CTRL3_C.IF_INC = 1` (auto-increment).
- Set `CTRL3_C.BDU = 1` so a burst read cannot return a torn high/low byte pair.
- ISM sensitivity:
  - Accel: ±2 g → 0.061 mg/LSB, ±4 → 0.122, ±8 → 0.244, ±16 → 0.488.
  - Gyro: ±125 dps → 4.375 mdps/LSB, ±250 → 8.75, ±500 → 17.5,
    ±1000 → 35, ±2000 → 70, ±4000 → 140.
  - Temperature: 256 LSB/°C, `T[°C] = raw/256 + 25`.
- MMC scale: 16384 counts/Gauss at ±8 G; unsigned zero at `2^17 = 131072`.

---

## 1. Overall flow

```
        ┌─────────────┐     ┌──────────────┐     ┌──────────────────┐
        │  STAGE 1    │     │   STAGE 2    │     │     STAGE 3      │
        │ INITIALIZE  │ ──► │  CALIBRATE   │ ──► │ CONTINUOUS READ  │
        └─────────────┘     └──────────────┘     └──────────────────┘
   reset, ID check,       gyro bias, accel      poll data-ready,
   configure ODR/FS/      offset/gain, mag      burst-read, scale,
   filters, interrupts    hard/soft iron        (filter), publish
```

Keep the vehicle **stationary** during Stage 2. Only enter Stage 3 after
calibration successes; otherwise assert a fault.

---

## 2. Common low-level primitives

```
register_write(dev, addr, byte)
register_read (dev, addr) -> byte
burst_read   (dev, addr, len) -> byte[len]      // IF_INC must be 1 (ISM)
```

- ISM burst read: send `addr | 0x80`, then clock out `len` bytes.
- MMC burst read: I2C repeated-START, address, register, repeated-START,
  read `len` bytes; acknowledge all but the last.

---

## 3. STAGE 1 — Initialization

### 3.1 ISM330DHCX init sequence

| # | Action | Register | Value | Notes |
|---|---|---|---|---|
| 1 | Power-up wait | — | — | wait ≥ 10 ms |
| 2 | Verify ID | `WHO_AM_I` 0x0F | must read `0x6B` | fault if mismatch |
| 3 | Software reset | `CTRL3_C` 0x12 | `0x01` | set `SW_RESET` |
| 4 | Wait reset | `CTRL3_C` 0x12 | poll bit0 == 0 | self-clearing |
| 5 | Enable device config + axes | `CTRL9_XL` 0x18 | `0xE2` | `DEVICE_CONF \| DEN_X \| DEN_Y \| DEN_Z` |
| 6 | Global config | `CTRL3_C` 0x12 | `0x44` | `BDU \| IF_INC` |
| 7 | Accel config | `CTRL1_XL` 0x10 | see §3.2 | ODR + FS + LPF2 |
| 8 | Gyro config | `CTRL2_G` 0x11 | see §3.2 | ODR + FS |
| 9 | Accel filter (optional) | `CTRL6_C` 0x15 | e.g. `0x00` | `ftype` = LPF1 bandwidth |
| 10 | Data-ready route (optional) | `INT1_CTRL` 0x0D | `0x03` | `DRDY_XL \| DRDY_G` → INT1 |

> `CTRL9_XL`: registers default to DEN=1 on reset, but writing `0xE2` sets
> `device_conf=1` and guarantees all axes enabled. If you prefer
> read-modify-write, set only bit 1.

### 3.2 ISM field encodings (worked example)

`CTRL1_XL` = `[odr_xl:7-4][fs_xl:3-2][lpf2_xl_en:1]`
`CTRL2_G`  = `[odr_g:7-4][fs_g:3-0]`

Example: **accel 1666 Hz / ±8 g / LPF2 on**, **gyro 1666 Hz / ±2000 dps**

```
CTRL1_XL = (8 << 4) | (3 << 2) | (1 << 1) = 0x8E
CTRL2_G  = (8 << 4) | (0xC)               = 0x8C
```

ODR codes: `12.5=1, 26=2, 52=3, 104=4, 208=5, 416=6, 833=7, 1666=8, 3332=9, 6667=10`
Accel FS: `±2g=0, ±16g=1, ±4g=2, ±8g=3`
Gyro FS: `±250=0x0, ±125=0x2, ±500=0x4, ±1000=0x8, ±2000=0xC, ±4000=0x1`

### 3.3 MMC5983MA init sequence

> `INT_CTRL_0` (0x09), `INT_CTRL_2` (0x0B), `INT_CTRL_3` (0x0C) are
> **write-only**; `INT_CTRL_1` (0x0A) reads reserved bits as 1. Keep a
> shadow copy and always write a full byte (no read-modify-write).

| # | Action | Register | Value | Notes |
|---|---|---|---|---|
| 1 | Software reset | `INT_CTRL_1` 0x0A | `0x80` | `SW_RST`, clears all regs |
| 2 | Wait | — | — | ≥ 10 ms (use 15 ms) |
| 3 | Set bandwidth | `INT_CTRL_1` 0x0A | `0x03` | `BW1:BW0` = 800 Hz |
| 4 | Auto SET/RESET (optional) | `INT_CTRL_0` 0x09 | `0x20` | `AUTO_SR_EN` |
| 5 | Continuous mode | `INT_CTRL_2` 0x0B | `0x0D` | `CM_FREQ=100Hz, CMM_EN` |
| 5b | + periodic SET (optional) | `INT_CTRL_2` 0x0B | `0x88 \| (N<<4)` | `EN_PRD_SET\|CMM_EN`, `N`=`PRD_SET` |

Bandwidth: `00=100Hz (8ms), 01=200Hz (4ms), 10=400Hz (2ms), 11=800Hz (0.5ms)`
CM_FREQ: `000=off, 001=1Hz, 010=10Hz, 011=20Hz, 100=50Hz, 101=100Hz, 110=200Hz, 111=1000Hz`
Periodic SET (`PRD_SET`): `0=1, 1=25, 2=75, 3=100, 4=250, 5=500, 6=1000, 7=2000`
measurements; requires both `AUTO_SR_EN` and `CMM_EN`.

For **one-shot** operation instead of continuous mode, leave `CMM_EN = 0`
and trigger each measurement (§5.2).

---

## 4. STAGE 2 — Calibration

### 4.1 Gyro bias

Vehicle stationary. Collect `N` raw samples per axis (N a power of two, e.g.
256 or 512) and average:

```
gyro_bias[axis] = sum(raw[axis]) / N
```

Subtract `gyro_bias` in Stage 3. Optionally write it into hardware via the
`X/Y/Z_OFS_USR` registers (0x73–0x75, signed 8-bit), enabled by
`CTRL7_G.USR_OFF_ON_OUT = 1`, with weight selected by `CTRL6_C.USR_OFF_W`
(1 mg/LSB or 16 mg/LSB).

### 4.2 Accelerometer offset and gain

- **Minimal (static):** with the board level, the Z axis should read +1 g;
  store `offset = measured - ideal` per axis.
- **Full (6-face tumble):** for each of the 6 orientations `(+X,-X,+Y,-Y,+Z,-Z)`,
  capture the average. Then:
  ```
  offset[a] = (avg_plus[a] + avg_minus[a]) / 2
  gain[a]   = (avg_plus[a] - avg_minus[a]) / 2        // should be ~1 g
  corrected = (raw - offset) / gain
  ```

### 4.3 Magnetometer — SET/RESET bridge-offset removal

The datasheet method (host-driven):

```
SET   measurement  ->  Output1 = +H + Offset
RESET measurement  ->  Output2 = -H + Offset
```

Procedure:

1. Write `INT_CTRL_0 = 0x08` (`SET`); the bit self-clears (~500 ns).
2. Wait ≥ 1 ms, wait for `MEAS_M_DONE`, read X/Y/Z (7-byte burst).
   (Optionally measure twice and use the second to avoid the first being stale.)
3. Write `INT_CTRL_0 = 0x10` (`RESET`).
4. Wait ≥ 1 ms, wait for `MEAS_M_DONE`, read X/Y/Z again.
5. Compute:
   ```
   Offset[axis] = (SET[axis] + RESET[axis]) / 2
   H[axis]      = (SET[axis] - RESET[axis]) / 2
   ```

Store `Offset` and subtract it in Stage 3.

**Hard/soft iron (optional, better heading):** the per-axis `Offset` above is
hard-iron. For soft-iron, collect samples over a full tumble, fit an
ellipsoid, and compute a 3×3 correction matrix — typically done on a host
tool and downloaded into BRAM.

### 4.4 Built-in self-test (optional functional gate)

ISM: `CTRL5_C.st_xl[1:0]` / `st_g[3:2]` = `DISABLE=0, POSITIVE=1,
NEGATIVE=2 (accel) / 3 (gyro)`. Run each mode at the ODR/FS specified in the
datasheet, compare the output shift against the min/max limits, then disable.
Reference: `st_mems_drivers/ism330dhcx_STdC/examples/ism330dhcx_self_test.c`.

MMC: `INT_CTRL_3.ST_ENP / ST_ENM` inject an extra coil current to check for
saturation.

---

## 5. STAGE 3 — Continuous read

### 5.1 ISM330DHCX burst read (polling)

```
1. status = read(STATUS_REG 0x1E)
2. if (status & 0x01)  // XLDA
       a[6] = burst_read(0x28, 6)      // OUTX_L_A .. OUTZ_H_A
       accel.x = signed16(a[1],a[0]); accel.y = signed16(a[3],a[2]);
       accel.z = signed16(a[5],a[4]);
3. if (status & 0x02)  // GDA
       g[6] = burst_read(0x22, 6)      // OUTX_L_G .. OUTZ_H_G
       gyro.x  = signed16(g[1],g[0]); ...
4. temperature = signed16(burst_read(0x20,2))
```

If `INT1_CTRL.DRDY_XL/DRDY_G` is routed to a pin, use the pin edge as the
trigger instead of polling. Apply axis remap → subtract calibration → scale
→ optional low-pass filter before publishing.

### 5.2 MMC5983MA read

**One-shot:** write `INT_CTRL_0.TM_M = 0x01`, poll `STATUS.MEAS_M_DONE`
(0x08 bit0), then burst-read 7 bytes from 0x00, then write `TM_M = 0`.

**Continuous:** with `CMM_EN` set, poll `MEAS_M_DONE` (it clears on the next
measurement start) and burst-read 7 bytes from 0x00.

18-bit assembly (bytes `b[0..6]` = regs 0x00..0x06):

```
X = (b[0] << 10) | (b[1] << 2) | (b[6] >> 6)
Y = (b[2] << 10) | (b[3] << 2) | ((b[6] >> 4) & 3)
Z = (b[4] << 10) | (b[5] << 2) | ((b[6] >> 2) & 3)
```

Convert to signed and Gauss:

```
H_signed[axis] = X - 131072 - Offset[axis]      // remove bridge offset
H_gauss[axis]  = H_signed[axis] / 16384.0       // ±8 G range
```

Timeout: `4 × measurement_time` (e.g. 32 ms at 100 Hz, 5 ms at 800 Hz).

---

## 6. Timing and error handling

| Event | Constraint |
|---|---|
| ISM power-up to first access | ≥ 10 ms |
| ISM SW_RESET | poll `SW_RESET` until 0 |
| MMC power-up / SW_RESET | ≥ 10 ms (use 15 ms) |
| MMC SET/RESET pulse | ~500 ns, self-clearing; wait ≥ 1 ms before measuring |
| MMC measurement | 8/4/2/0.5 ms for BW 100/200/400/800 Hz |
| MMC read timeout | 4 × measurement time |

Implement a watchdog on every "wait for ready" loop; on timeout raise a
`sensor_fault` and fail safe. For SPI, enable the ISM read CRC if available
and retry on mismatch — do **not** rely on the LPF to fix bit errors.

---

## 7. Register quick reference

### ISM330DHCX (used registers)

| Addr | Name | Purpose |
|---|---|---|
| 0x01 | `FUNC_CFG_ACCESS` | access embedded-function / sensor-hub banks |
| 0x0D/0x0E | `INT1_CTRL`/`INT2_CTRL` | route data-ready/FIFO to pins |
| 0x0F | `WHO_AM_I` | = 0x6B |
| 0x10 | `CTRL1_XL` | accel ODR/FS/LPF2 |
| 0x11 | `CTRL2_G` | gyro ODR/FS |
| 0x12 | `CTRL3_C` | SW reset, IF_INC, BDU, BOOT |
| 0x13 | `CTRL4_C` | gyro LPF1 select, DRDY mask |
| 0x14 | `CTRL5_C` | self-test, rounding |
| 0x15 | `CTRL6_C` | accel LPF1 BW, user-offset weight |
| 0x16 | `CTRL7_G` | gyro HP filter, user offset on output |
| 0x17 | `CTRL8_XL` | accel LPF2/HP filter |
| 0x18 | `CTRL9_XL` | `device_conf`, DEN per axis |
| 0x19 | `CTRL10_C` | timestamp enable |
| 0x1A | `ALL_INT_SRC` | interrupt source flags |
| 0x1E | `STATUS_REG` | XLDA/GDA/TDA |
| 0x20–0x21 | `OUT_TEMP` | temperature |
| 0x22–0x27 | `OUT_*_G` | gyro X/Y/Z |
| 0x28–0x2D | `OUT_*_A` | accel X/Y/Z |
| 0x3A/0x3B | `FIFO_STATUS` | FIFO level/status |
| 0x40–0x43 | `TIMESTAMP` | 32-bit timestamp |
| 0x5E/0x5F | `MD1_CFG`/`MD2_CFG` | route functional events |
| 0x73–0x75 | `X/Y/Z_OFS_USR` | hardware user offset |

### MMC5983MA (all registers)

| Addr | Name | Purpose |
|---|---|---|
| 0x00–0x06 | `X/Y/Z_OUT` | 18-bit field data |
| 0x07 | `T_OUT` | temperature, ~0.8 °C/LSB, 0 = −75 °C |
| 0x08 | `STATUS` | `MEAS_M_DONE`, `MEAS_T_DONE`, `OTP_READ_DONE` |
| 0x09 | `INT_CTRL_0` (W) | `TM_M`, `TM_T`, IRQ en, `SET`, `RESET`, `AUTO_SR_EN`, `OTP_READ` |
| 0x0A | `INT_CTRL_1` (W) | `SW_RST`, `BW1:BW0`, channel inhibit |
| 0x0B | `INT_CTRL_2` (W) | `CM_FREQ`, `CMM_EN`, `PRD_SET`, `EN_PRD_SET` |
| 0x0C | `INT_CTRL_3` (W) | self-test, SPI 3-wire |
| 0x2F | `PROD_ID` | = 0x30 |

---

## 8. State machine sketch (host view)

```
RESET_WAIT ──► ID_CHECK ──► ISM_CONFIG ──► MMC_CONFIG ──┐
      │              │                                      │
      │ mismatch     │ mismatch                             ▼
      └──────────────┴──────────────► FAULT          CALIB_GYRO
                                                          │
                                                          ▼
                                                     CALIB_ACCEL
                                                          │
                                                          ▼
                                                      CALIB_MAG
                                                          │
                                                          ▼
                    ┌──────────────────────────────► RUN_POLL
                    │                                    │ ready
                    │                                    ▼
                    │                                RUN_READ
                    │                                    │
                    └────────── publish ◄───────────────┘
```

---

## 9. Gotchas checklist

- [ ] SPI read flag: `addr | 0x80`; write flag clear.
- [ ] `IF_INC = 1` before any burst read; `BDU = 1` to avoid torn reads.
- [ ] ISM data is little-endian (low byte first).
- [ ] `CTRL9_XL.device_conf = 1` after reset (and enable DEN axes).
- [ ] MMC `INT_CTRL_0/2/3` write-only, `INT_CTRL_1` reserved bits read as 1 →
      keep shadow bytes, write full bytes.
- [ ] MMC SET/RESET bits self-clear; wait ≥ 1 ms before measuring.
- [ ] MMC 18-bit output is unsigned centered at 131072.
- [ ] Mag offset = `(SET + RESET)/2`, field = `(SET − RESET)/2`.
- [ ] Timeout every "wait for ready" loop; raise fault on failure.
- [ ] Bit errors → CRC/retry, not a low-pass filter.

---

## 10. References (downloaded)

- `~/Projects/imu_drivers/ism330dhcx-pid/ism330dhcx_reg.{c,h}` — ST register driver.
- `~/Projects/imu_drivers/st_mems_drivers/ism330dhcx_STdC/examples/` — polling + self-test.
- `~/Projects/imu_drivers/sparkfun_mmc5983ma/` — MMC driver, 7 examples, datasheet PDF.
- `~/Projects/imu_drivers/sparkfun_9dof/Documentation/` — ISM datasheet + board usage PDF.
- `~/Projects/fpga_flight_controller/rtl/imu/imu_regmap_pkg.sv` — SystemVerilog register map.
