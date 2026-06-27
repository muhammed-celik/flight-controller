# Module 02: Sensor Calibration

## 1. Why This Module is Needed

Raw IMU sensor readings contain systematic errors that, if uncorrected, directly
degrade flight performance:

- **Bias (offset):** The sensor reports a non-zero value when the true input is
  zero. A gyro bias of 1 deg/s means the attitude estimate drifts at 1 degree
  per second.
- **Scale error:** The sensor sensitivity is not exactly nominal. A 2% scale
  error on the gyro means a 100 deg/s rotation is reported as 102 deg/s.
- **Cross-axis coupling:** Rotation about one axis leaks into another axis.
  This is ignored in the first RTL implementation.

Gyro bias is the critical first-flight calibration. Each MPU9250 has a different
power-on bias, and that bias changes with temperature. The first implementation
therefore performs stationary gyro-bias estimation in RTL after sensor
initialization and before arming is allowed.

The CPU is not required for calibration in the first build. Later CPU firmware
may inspect samples, override coefficients, run an EKF bias estimator, or store
factory/user calibration values, but the drone must be able to reach `cal_done`
with RTL alone.

---

## 2. Calibration Approaches

### 2.1 Comparison of Methods

| Method | When Applied | Accuracy | Complexity | First RTL build |
|---|---|---:|---:|---|
| Nominal datasheet scale | Always | Moderate | Very low | Yes |
| RTL power-on gyro averaging | Every boot | Good | Low | Yes |
| RTL stationary/level sanity checks | Every boot | Good | Low-medium | Yes |
| 6-position accel calibration | One-time/offline | Excellent | Medium | Later |
| Temperature LUT | Continuous | Good | Medium | Later |
| Runtime adaptive EKF bias | Continuous | Best | High | CPU later |

### 2.2 Chosen First-Flight Approach

**Bias estimation in RTL at every power-on:**

- Drone must be stationary for 2-5 seconds after power-on.
- Average `N` gyro samples: `bias_g = mean(gyro_raw[0:N-1])`.
- Use accel only as a stationary/level plausibility check for first flight.
- `N = 2000` samples at 1 kHz gives a 2 second calibration window.
- If variance or plausibility checks fail, keep `cal_done = 0` and inhibit
  arming.

**Scale factors in first RTL build:**

- Use nominal MPU9250 conversion constants from the configured full-scale
  ranges.
- Internal scale-correction coefficients default to `1.0`.
- AXI override registers may be added later, but their reset values must be
  flight-capable.

**Future CPU/EKF path:**

- CPU firmware may write refined bias/scale coefficients through AXI.
- A CPU EKF may continuously estimate gyro bias and publish slow corrections.
- RTL remains the per-sample application point for deterministic subtract/scale.

---

## 3. Theoretical Foundations

### 3.1 Sensor Error Model

The complete sensor model for a 3-axis inertial sensor is:

```text
y = M * S * (x_true + b + n)
```

Where:

- `y` is the sensor output.
- `x_true` is the true physical quantity.
- `b` is the bias vector.
- `n` is zero-mean sensor noise.
- `S` is the scale factor matrix.
- `M` is the misalignment/cross-axis matrix.

The first implementation ignores cross-axis terms and applies:

```text
x_calibrated = scale * (y - bias)
```

For first flight, `scale = 1.0` after the raw MPU9250 samples are converted into
Q16.16 physical units.

### 3.2 Gyro Bias Estimation

When the drone is stationary, true angular rate is zero:

```text
gyro_true = 0
bias_g = mean(gyro_measured)
```

Averaging reduces random noise:

```text
standard_error = sample_sigma / sqrt(N)
```

The actual stationary standard deviation should be measured from GY-91 logs
after the RTL path is working. The initial design only needs enough precision to
remove gross power-on bias before arming.

### 3.3 Accel Handling in First Flight

Boot-time accel bias estimation only works if the vehicle is level. If the drone
is tilted during calibration, the module would incorrectly learn that tilted
orientation as level. For this reason, first-flight RTL does not depend on a
computed accel bias for arming.

Instead, RTL uses accel during calibration for plausibility:

```text
abs(norm(accel_mean) - 1 g) < accel_norm_limit
```

Optional level checks may require `accel_x` and `accel_y` to be near zero, but
they should be treated as pre-arm safety checks rather than a full accel
calibration. A later 6-position user calibration can provide accel offsets and
scale factors.

### 3.4 Temperature Effects

Gyro bias changes with temperature. First flight ignores temperature correction
because flights are short and the boot-time estimate removes the largest error.
Later firmware may:

1. request recalibration when stationary and temperature changes significantly;
2. apply a characterized linear or LUT correction;
3. let the CPU EKF estimate slow gyro-bias drift.

---

## 4. Module Interface

### 4.1 Inputs

| Signal | Width | Format | Source | Description |
|---|---:|---|---|---|
| `clk` | 1 | Clock | System | 100 MHz system clock |
| `rst_n` | 1 | Active-low | System | Synchronous reset |
| `accel_x/y/z` | 32 | Q16.16 signed | IMU acquisition | Raw converted accel in g |
| `gyro_x/y/z` | 32 | Q16.16 signed | IMU acquisition | Raw converted gyro in deg/s or rad/s, project-wide choice |
| `imu_valid` | 1 | Pulse | IMU acquisition | New IMU data available |
| `sensor_ready` | 1 | Level | IMU acquisition | Required identities/configuration are valid |
| `arm_request` | 1 | Level | Failsafe/arming | Used only to block arming until calibration completes |
| `recal_request` | 1 | Pulse | RTL/optional AXI | Request a new stationary calibration |
| `cfg_override_en` | 1 | Level | Optional AXI | Use externally supplied coefficients |
| `cfg_bias_*` | 32 | Q16.16 signed | Optional AXI | CPU/user coefficient override |
| `cfg_scale_*` | 32 | Q16.16 signed | Optional AXI | CPU/user scale override, reset `1.0` |

### 4.2 Outputs

| Signal | Width | Format | Destination | Description |
|---|---:|---|---|---|
| `cal_accel_x/y/z` | 32 | Q16.16 signed | IIR filter | Calibrated acceleration |
| `cal_gyro_x/y/z` | 32 | Q16.16 signed | IIR filter/PID | Calibrated angular rate |
| `cal_valid` | 1 | Pulse | IIR filter | Calibrated data ready |
| `cal_done` | 1 | Level | Failsafe/arming | Gyro bias was measured and accepted |
| `cal_busy` | 1 | Level | Status | Boot/recalibration in progress |
| `cal_error` | bit field | Status | Reason calibration failed |
| `bias_gx/gy/gz_status` | 32 | Q16.16 signed | Optional AXI/status | Active gyro bias values |

The optional AXI interface exposes status and overrides, but reset/default
values must be sufficient for pure RTL operation.

---

## 5. Algorithm

### 5.1 RTL Boot Calibration FSM

```text
CONSTANT N = 2000
CONSTANT ONE = 0x00010000

STATE RESET:
    cal_done = 0
    cal_busy = 0
    wait until sensor_ready
    -> COLLECT

STATE COLLECT:
    cal_busy = 1
    sums = 0
    sums_sq_for_variance = 0
    count = 0

    on each imu_valid:
        accumulate gyro_x/y/z
        optionally accumulate accel mean and simple variance statistics
        count += 1
        if count == N:
            -> CHECK

STATE CHECK:
    compute gyro_bias = gyro_sum / N
    check gyro variance below threshold
    check accel mean magnitude approximately 1 g
    check bias magnitude below gross-fault threshold

    if checks pass:
        latch active gyro biases
        active accel biases = 0 for first flight
        active scales = 1.0 unless cfg_override_en
        cal_done = 1
        cal_busy = 0
        -> RUN
    else:
        cal_done = 0
        cal_error = reason
        -> WAIT_STATIONARY_OR_RETRY

STATE RUN:
    apply realtime calibration on every imu_valid
    if recal_request and not armed:
        cal_done = 0
        -> COLLECT
```

Division by `N = 2000` can be implemented as a small sequential divider, a
multiply-by-reciprocal approximation, or by choosing a power-of-two sample count
such as `N = 2048`. A power-of-two count is attractive for first RTL because the
mean is a shift, but the sample duration should remain roughly 2 seconds.

### 5.2 Real-Time Calibration

```text
ON imu_valid:
    IF cal_done == 0:
        cal_accel_x = accel_x
        cal_accel_y = accel_y
        cal_accel_z = accel_z
        cal_gyro_x  = gyro_x
        cal_gyro_y  = gyro_y
        cal_gyro_z  = gyro_z
    ELSE:
        cal_gyro_x  = q16_mul(scale_gx, gyro_x  - bias_gx)
        cal_gyro_y  = q16_mul(scale_gy, gyro_y  - bias_gy)
        cal_gyro_z  = q16_mul(scale_gz, gyro_z  - bias_gz)
        cal_accel_x = q16_mul(scale_ax, accel_x - bias_ax)
        cal_accel_y = q16_mul(scale_ay, accel_y - bias_ay)
        cal_accel_z = q16_mul(scale_az, accel_z - bias_az)

    ASSERT cal_valid for one cycle

FUNCTION q16_mul(a, b):
    product_64 = signed_64(a) * signed_64(b)
    return product_64[47:16]
```

### 5.3 Optional CPU Override Path

```text
if cfg_override_en:
    active_bias = cfg_bias
    active_scale = cfg_scale
else:
    active_bias = rtl_measured_bias
    active_scale = 1.0
```

CPU overrides must not clear `cal_done` unless they intentionally request
recalibration or publish invalid coefficients.

---

## 6. Calibration Quality Verification

RTL performs the checks needed to inhibit arming:

| Check | Expected value | First action if fail |
|---|---|---|
| Gyro bias magnitude | Small, board-characterized limit | Keep `cal_done = 0` |
| Gyro variance during cal | Below stationary threshold | Retry or report moving |
| Accel magnitude at rest | Approximately 1 g | Report not stationary/level |
| IMU freshness | Valid 1 kHz samples | Sensor fault, inhibit arming |

Later CPU firmware may perform deeper diagnostics, log histograms, estimate
temperature drift, and present user-facing calibration quality.

---

## 7. Computational Cost

### Per Sample

| Operation | Count | Resources |
|---|---:|---|
| Subtraction | 6 | 32-bit adders |
| Q16.16 multiply | 6 | Time-muxed DSP48E1s |
| Status/register update | small | FF/LUT |

### Boot Calibration

| Operation | Count | Resources |
|---|---:|---|
| Accumulators | 6 means, optional variance | 48-64 bit registers |
| Counter | 1 | 12 bits for 2048 samples |
| Mean calculation | 3 required gyro axes | Shift or sequential divide |
| Stationary checks | few comparators | LUTs |

The boot FSM is not in the realtime critical path. Per-sample subtract/scale
latency remains negligible compared with the 1-2 ms control period.

---

## 8. Resource Estimate

| Resource | Count | Notes |
|---|---:|---|
| Flip-flops | ~250-400 | Accumulators, counters, active coefficients, pipeline |
| LUTs | ~250 | FSM, subtractors, muxes, checks |
| DSP48E1 | 2 | Time-muxed for realtime multiply; optional variance reuse |
| Block RAM | 0 | No sample buffer required |
| Optional AXI registers | 13+ | Bias/scale overrides, control, status |
