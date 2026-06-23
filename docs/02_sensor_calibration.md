# Module 02: Sensor Calibration

## 1. Why This Module is Needed

Raw IMU sensor readings contain systematic errors that, if uncorrected, directly
degrade flight performance:

- **Bias (offset):** The sensor reports a non-zero value when the true input is
  zero. A gyro bias of 1°/s means the attitude estimate drifts at 1° per second
  — the drone would tilt 30° in 30 seconds if uncorrected.
- **Scale error:** The sensor's sensitivity is not exactly its nominal value.
  A 2% scale error on the gyro means a 100°/s rotation is reported as 102°/s,
  causing the PID to over-correct.
- **Cross-axis coupling:** Rotation about X leaks into the Y reading. Typically
  0.5–2% for MEMS sensors. Ignored in first implementation.

Without calibration, the selected MPU9250 can produce unusable
attitude estimates after a few seconds of flight. The calibration module applies
pre-computed corrections in real-time, every sample, with zero additional latency
beyond a single multiply-add pipeline.

**Gyro bias is the most critical error:** manufacturing variation and temperature
mean each MPU9250 has a different power-on bias, commonly large enough to cause
rapid attitude drift. This bias MUST be
measured and subtracted before flight.

---

## 2. Calibration Approaches

### 2.1 Comparison of Methods

| Method                   | When Applied | Accuracy  | Complexity | Suitable for FPGA? |
|--------------------------|-------------|-----------|------------|-------------------|
| Factory calibration      | One-time    | Moderate  | None (use datasheet) | N/A |
| Power-on averaging       | Every boot  | Good      | Low        | ✅ Yes            |
| Runtime adaptive (EKF)   | Continuous  | Best      | Very high  | ❌ CPU only       |
| Temperature LUT          | Continuous  | Good      | Medium     | ⚠️ Requires characterization |
| 6-position accel cal.    | One-time    | Excellent | Offline    | N/A (done on PC)  |

### 2.2 Chosen Approach: Power-On Bias + Pre-loaded Scale

**Bias estimation (every power-on):**
- Drone must be stationary and level for 2–5 seconds after power-on
- Average N gyro samples: bias = mean(gyro_raw[0:N-1])
- Average N accel samples: bias = mean(accel_raw[0:N-1]) - [0, 0, 1g]
- N = 2000 samples at 1 kHz = 2 seconds

**Scale factor (pre-loaded via CPU at boot):**
- Determined during manufacturing test or user calibration
- Stored in non-volatile memory (SPI flash or CPU RAM)
- Loaded into FPGA registers via AXI before arming

**Runtime adaptive calibration:**
- When CPU EKF mode is selected, it can continuously estimate gyroscope bias.
- Its bias estimate may be written back through the atomic AXI mailbox for
  monitoring or slow correction.
- The boot-time stationary estimate is required in both estimator modes and
  prevents arming with a grossly biased sensor.
- Deterministic per-sample offset/scale application remains in RTL.

---

## 3. Theoretical Foundations

### 3.1 Sensor Error Model

The complete sensor model for a 3-axis inertial sensor:

```
y = M · S · (x_true + b + n)
```

Where:
- y ∈ ℝ³ — sensor output (what we read)
- x_true ∈ ℝ³ — true physical quantity
- b ∈ ℝ³ — bias vector (constant + slow drift)
- n ∈ ℝ³ — random noise (zero-mean, white)
- S = diag(sx, sy, sz) — scale factor matrix
- M ∈ ℝ³ˣ³ — misalignment/cross-axis matrix

**Simplified (first implementation):**

Ignore cross-axis (M = I₃) and assume noise is handled by the filter stage:
```
y = S · (x_true + b)
```

Solving for the true value:
```
x_true = S⁻¹ · y - b = (1/s) · y - b
```

In practice, we apply: `x_calibrated = scale · (y - bias)`

Where scale ≈ 1/s (precomputed reciprocal of the sensor's actual sensitivity).

### 3.2 Bias Estimation Theory

**Gyro bias at rest:**

When the drone is stationary, the true angular rate is zero:
```
ω_true = 0  →  y_gyro = S · (0 + b_gyro + n) = S · b_gyro + S · n
```

Averaging N samples eliminates noise (E[n] = 0):
```
b̂_gyro = (1/N) · Σᵢ₌₀ᴺ⁻¹ y_gyro[i]
```

**Noise reduction by averaging:**
```
σ_bias_estimate = σ_sensor / √N
```

For the MPU9250, estimate the actual stationary sample standard deviation from
captured data rather than copying a different sensor's datasheet number. With
`N=2000`, the standard error of the mean is approximately `sample_sigma/sqrt(N)`.

**Accel bias at rest (level):**

When stationary and level, the true acceleration is gravity in the z-axis:
```
a_true = [0, 0, +1g]  (NED convention, z = down)
b̂_accel = (1/N) · Σᵢ₌₀ᴺ⁻¹ y_accel[i] - [0, 0, 1g]
```

Note: This only works if the drone is LEVEL. If tilted during calibration,
the bias estimate will be wrong and the drone will think "level" is at an angle.

### 3.3 Scale Factor

The nominal sensitivity of MPU9250 at ±2000°/s is:
```
S_nominal = 16.4 LSB/(°/s)  →  conversion factor = 1/16.4 = 0.061 °/s/LSB
```

Actual sensitivity varies ±1–3% between chips. The scale correction factor:
```
scale_correction = S_nominal / S_actual
```

This is determined by applying a known rotation (using a rate table) or
by the factory trim stored in the sensor's OTP memory. For initial flight,
scale = 1.0 (no correction) is acceptable — 2% error is tolerable for RC flight.

### 3.4 Temperature Effects

Gyro bias changes with temperature at approximately:
```
b(T) = b₀ + TCO × (T - T_ref)
```

The coefficient is device- and unit-dependent; characterize it from logged
MPU9250 temperature and stationary bias rather than assuming a value. Solutions:
1. Re-calibrate bias if temperature changes >10°C (monitored by CPU)
2. Apply linear temperature compensation (requires characterization)
3. For initial implementation: ignore (flights are short, temperature stable)

---

## 4. Module Interface

### 4.1 Inputs

| Signal        | Width | Format          | Source         | Description                         |
|---------------|-------|-----------------|----------------|-------------------------------------|
| clk           | 1     | Clock           | System         | 100 MHz system clock                |
| rst_n         | 1     | Active-low      | System         | Synchronous reset                   |
| accel_x/y/z   | 32    | Q16.16 signed   | IMU Acq.       | Raw accel (g units)                 |
| gyro_x/y/z    | 32    | Q16.16 signed   | IMU Acq.       | Raw gyro (°/s)                      |
| imu_valid      | 1     | Pulse           | IMU Acq.       | New IMU data available              |
| bias_gx/gy/gz  | 32    | Q16.16 signed   | CPU (AXI)      | Gyro bias offset per axis           |
| bias_ax/ay/az  | 32    | Q16.16 signed   | CPU (AXI)      | Accel bias offset per axis          |
| scale_gx/gy/gz | 32    | Q16.16 signed   | CPU (AXI)      | Gyro scale factor (default=1.0)     |
| scale_ax/ay/az | 32    | Q16.16 signed   | CPU (AXI)      | Accel scale factor (default=1.0)    |
| cal_enable     | 1     | Level           | CPU (AXI)      | 1=apply calibration, 0=passthrough  |

### 4.2 Outputs

| Signal          | Width | Format          | Dest           | Description                       |
|-----------------|-------|-----------------|----------------|-----------------------------------|
| cal_accel_x/y/z | 32    | Q16.16 signed   | IIR Filter     | Calibrated acceleration           |
| cal_gyro_x/y/z  | 32    | Q16.16 signed   | IIR Filter     | Calibrated angular rate           |
| cal_valid        | 1     | Pulse           | IIR Filter     | Calibrated data ready             |

### 4.3 CPU-Side Interface (for bias estimation)

The CPU performs the averaging algorithm at boot time:
1. CPU reads raw sensor data via AXI status registers (or IMU Acq outputs)
2. CPU accumulates 2000 samples in software
3. CPU divides by N to get bias
4. CPU writes bias values to the calibration registers
5. CPU sets cal_enable = 1

This hybrid approach puts the simple real-time operation (subtract + multiply)
in RTL and the complex but infrequent operation (averaging, division) on the CPU.

---

## 5. Algorithm (Pseudocode)

### 5.1 Real-Time Calibration (RTL, every sample)

```
CONSTANT ONE = 0x00010000    // 1.0 in Q16.16

ON imu_valid:
    IF cal_enable == 0:
        // Passthrough mode (during boot calibration)
        cal_accel_x = accel_x
        cal_accel_y = accel_y
        cal_accel_z = accel_z
        cal_gyro_x  = gyro_x
        cal_gyro_y  = gyro_y
        cal_gyro_z  = gyro_z
    ELSE:
        // Apply calibration: output = scale × (input - bias)
        cal_gyro_x  = q16_mul(scale_gx, gyro_x  - bias_gx)
        cal_gyro_y  = q16_mul(scale_gy, gyro_y  - bias_gy)
        cal_gyro_z  = q16_mul(scale_gz, gyro_z  - bias_gz)
        cal_accel_x = q16_mul(scale_ax, accel_x - bias_ax)
        cal_accel_y = q16_mul(scale_ay, accel_y - bias_ay)
        cal_accel_z = q16_mul(scale_az, accel_z - bias_az)

    ASSERT cal_valid = 1 (single-cycle pulse)

// Q16.16 multiply helper
FUNCTION q16_mul(a, b) → Q16.16:
    product_64 = (signed_64)a × (signed_64)b    // 32×32 → 64 bit
    RETURN product_64[47:16]                      // extract Q16.16 result
```

### 5.2 Boot-Time Bias Estimation (CPU software)

```
// Runs on MicroBlaze or RV32 CPU at power-on
FUNCTION estimate_bias():
    CONSTANT N = 2000
    sum_gx = sum_gy = sum_gz = 0
    sum_ax = sum_ay = sum_az = 0

    // Collect N samples (2 seconds at 1 kHz)
    FOR i = 0 TO N-1:
        WAIT for imu_valid pulse
        sum_gx += gyro_x_raw
        sum_gy += gyro_y_raw
        sum_gz += gyro_z_raw
        sum_ax += accel_x_raw
        sum_ay += accel_y_raw
        sum_az += accel_z_raw

    // Compute gyro bias (should be zero at rest)
    bias_gx = sum_gx / N
    bias_gy = sum_gy / N
    bias_gz = sum_gz / N

    // Compute accel bias (should be [0, 0, +1g] at rest, level)
    bias_ax = sum_ax / N - 0            // X should be 0
    bias_ay = sum_ay / N - 0            // Y should be 0
    bias_az = sum_az / N - ONE_G        // Z should be 1g (0x0009CE8A in Q16.16)

    // Write to hardware registers
    WRITE_AXI(REG_BIAS_GX, bias_gx)
    WRITE_AXI(REG_BIAS_GY, bias_gy)
    WRITE_AXI(REG_BIAS_GZ, bias_gz)
    WRITE_AXI(REG_BIAS_AX, bias_ax)
    WRITE_AXI(REG_BIAS_AY, bias_ay)
    WRITE_AXI(REG_BIAS_AZ, bias_az)

    // Set scale to 1.0 (no correction initially)
    WRITE_AXI(REG_SCALE_GX, ONE)
    WRITE_AXI(REG_SCALE_GY, ONE)
    WRITE_AXI(REG_SCALE_GZ, ONE)
    WRITE_AXI(REG_SCALE_AX, ONE)
    WRITE_AXI(REG_SCALE_AY, ONE)
    WRITE_AXI(REG_SCALE_AZ, ONE)

    // Enable calibration
    WRITE_AXI(REG_CAL_ENABLE, 1)
```

### 5.3 Variance Check (optional, detects if drone is moving during cal)

```
// If variance exceeds threshold, drone is not stationary — abort calibration
FUNCTION check_stationary(samples[N]):
    mean = sum(samples) / N
    variance = sum((samples[i] - mean)² for i in 0..N-1) / N

    IF variance > VARIANCE_THRESHOLD:
        RETURN ERROR_NOT_STATIONARY
    RETURN OK

// Thresholds:
// Gyro: variance should be < (0.5°/s)² = 0.25 in (°/s)²
// Accel: variance should be < (0.05g)² = 0.0025 in g²
```

---

## 6. Computational Cost

### Per-Sample (RTL):

| Operation        | Count | Resources                  |
|------------------|-------|----------------------------|
| Subtraction      | 6     | 6 × 32-bit adder (LUTs)   |
| Q16.16 multiply  | 6     | 2 DSP48E1 (time-muxed, 3 cycles each) |
| Total latency    |       | ~10 clock cycles (100 ns)  |

### Time-Multiplexing Strategy:

With 2 DSP slices, processing 6 axes sequentially:
- 3 multiplies per DSP × 1 clock per multiply = 3 clocks per DSP
- Total: 6 multiplies / 2 DSPs = 3 clock cycles for multiplies
- Plus 1 cycle for subtraction, 1 for output register
- **Total: ~5–10 cycles = 50–100 ns** (negligible compared to 2 ms period)

---

## 7. Calibration Quality Verification

After calibration, the CPU can verify the result:

| Check                          | Expected Value      | Tolerance | Action if Fail     |
|--------------------------------|---------------------|-----------|--------------------|
| gyro_bias magnitude            | < 5°/s              | ±10°/s    | Retry or flag error|
| accel_bias magnitude (X,Y)     | < 0.1g              | ±0.3g     | Drone not level    |
| accel magnitude at rest        | 1.0g ± 0.02g       | ±0.05g    | Sensor fault       |
| gyro variance during cal       | < 0.25 (°/s)²      | —         | Drone was moving   |

---

## 8. Resource Estimate

| Resource       | Count  | Notes                                    |
|----------------|--------|------------------------------------------|
| Flip-flops     | ~100   | Input/output registers, pipeline         |
| LUTs           | ~150   | Subtractors, muxes, passthrough logic    |
| DSP48E1        | 2      | Time-muxed for 6 multiplications         |
| Block RAM      | 0      | All coefficients in registers            |
| AXI registers  | 13     | 6 bias + 6 scale + 1 enable             |
