# Module 04: Attitude Estimation (Complementary Filter + CORDIC)

## 1. Why This Module is Needed

The flight controller must know the drone's orientation (attitude) in space to
maintain level flight. Attitude is represented as three Euler angles:

- **Roll (φ):** Rotation about the forward axis (X). Tilts the drone left/right.
- **Pitch (θ):** Rotation about the lateral axis (Y). Tilts the drone forward/back.
- **Yaw (ψ):** Rotation about the vertical axis (Z). Points the drone's nose.

**Why not just use the gyro?**

Integrating angular rate gives angle: θ = ∫ω dt. But any tiny bias in ω
accumulates without bound:
```
Error after t seconds = bias × t
Example: 0.1°/s bias → 6° error after 1 minute → drone flies sideways
```

**Why not just use the accelerometer?**

The accelerometer gives "which way is down" (gravity direction). From gravity,
we can compute roll and pitch angles directly. But:
- Vibration corrupts the reading (especially during flight)
- Linear acceleration (moving the drone) is indistinguishable from tilt
- Response is slow (filtered at 20 Hz)

**Solution: Complementary Filter**

Fuse both sensors: trust the gyro for fast changes (high-frequency) and the
accelerometer for long-term reference (low-frequency). This gives:
- Fast response (from gyro) — no lag in control loop
- No drift (from accel) — stable over minutes/hours
- Simple to implement in hardware

---

## 2. Estimation Algorithm Options

### 2.1 Comparison Table

| Algorithm            | Accuracy | Complexity    | Latency      | FPGA Suitable? |
|---------------------|----------|---------------|--------------|----------------|
| Complementary filter | Good     | Very low      | 1 sample     | ✅ Ideal       |
| Mahony filter        | Good     | Low           | 1 sample     | ✅ Possible    |
| Madgwick (gradient)  | Better   | Medium        | 1 sample     | ⚠️ Needs normalize |
| Extended Kalman (EKF)| Best     | Very high     | Variable     | ❌ CPU only    |
| Unscented Kalman     | Best     | Extreme       | Variable     | ❌ CPU only    |

### 2.2 Chosen: Complementary Filter

**Why complementary filter:**
- Only 2 multiplies + 2 adds per axis per sample
- No matrix operations, no square roots, no divisions
- Deterministic single-cycle computation
- Accuracy is more than adequate for RC flight (±0.5° static)
- Same algorithm used in most hobby flight controllers

**Why not EKF:**
- Requires 6×6 or 7×7 matrix multiply (216 multiplies per step)
- Requires matrix inversion or Cholesky decomposition
- Variable execution time — incompatible with hard real-time pipeline
- Overkill: complementary filter achieves <1° error in practice

**Why not Madgwick:**
- Requires vector normalization (square root + division)
- Square root in fixed-point is expensive (~20 cycles iterative)
- Quaternion form adds complexity without benefit for small-angle RC flight

---

## 3. Theoretical Foundations

### 3.1 Attitude from Accelerometer (Static Reference)

When the drone is not accelerating linearly, the accelerometer measures only
gravity. The gravity vector in the body frame gives roll and pitch:

```
Given: a = [ax, ay, az] (accelerometer reading in g, body frame)

Roll from accel:
    φ_accel = atan2(ay, az)

Pitch from accel:
    θ_accel = atan2(-ax, √(ay² + az²))

Simplified (small angle, az ≈ 1g):
    φ_accel ≈ atan2(ay, az) ≈ ay / az  (in radians, for small angles)
    θ_accel ≈ atan2(-ax, az) ≈ -ax / az
```

Note: Yaw CANNOT be determined from accelerometer alone (gravity has no
horizontal component). Yaw requires a magnetometer or GPS.

### 3.2 Attitude from Gyroscope (Integration)

The gyro gives angular rate. Integrating over the sample period dt:

```
φ_gyro[n] = φ[n-1] + ωx × dt
θ_gyro[n] = θ[n-1] + ωy × dt
ψ_gyro[n] = ψ[n-1] + ωz × dt
```

Where dt = 1/500 = 0.002 s (at 500 Hz control rate) or 1/1000 = 0.001 s (at 1 kHz IMU rate).

This is accurate for short time periods but drifts over time due to bias.

### 3.3 Complementary Filter Fusion

The complementary filter is a weighted combination:

```
angle[n] = α_cf × (angle[n-1] + gyro_rate × dt) + (1 - α_cf) × accel_angle
         = α_cf × gyro_prediction + (1 - α_cf) × accel_measurement
```

Where α_cf (NOT the same as filter α from module 03) is the trust factor:
- α_cf close to 1.0: trust gyro more (faster response, but drifts)
- α_cf close to 0.0: trust accel more (no drift, but noisy/laggy)

**Typical value: α_cf = 0.98 (at 500 Hz) or α_cf = 0.996 (at 1 kHz)**

The equivalent time constant:
```
τ = dt / (1 - α_cf)

At 1 kHz, α_cf = 0.996: τ = 0.001 / 0.004 = 0.25 s
At 500 Hz, α_cf = 0.98:  τ = 0.002 / 0.02  = 0.1 s
```

This means the accel correction has a ~0.25 second time constant — fast enough
to correct gyro drift, slow enough to reject vibration.

### 3.4 Equivalent Form (Fewer Operations)

Rearranging the complementary filter:
```
angle[n] = angle[n-1] + gyro_rate × dt + (1 - α_cf) × (accel_angle - angle[n-1])
```

This form makes the structure clear:
1. Predict: add gyro increment to previous angle
2. Correct: nudge toward accelerometer reading by small amount (1-α_cf)

### 3.5 Why CORDIC is Needed

Computing `accel_angle = atan2(ay, az)` requires the arctangent function.
In fixed-point hardware without a floating-point unit, we use CORDIC
(COordinate Rotation DIgital Computer):

- Computes atan2(y, x) using only shifts and adds
- 16 iterations give 16-bit precision (~0.003° accuracy)
- Iterative: takes 16 clock cycles per computation
- No multiplier needed (shifts are free in hardware)

---

## 4. CORDIC Algorithm

### 4.1 Theory

CORDIC rotates a vector by successively smaller angles until the y-component
becomes zero. The accumulated rotation angles give the arctangent.

**Vectoring mode** (for atan2):
```
Goal: Rotate vector (x, y) until y = 0. The total rotation = atan2(y, x).

Iteration i (for i = 0, 1, ..., N-1):
    σᵢ = -sign(yᵢ)     // rotate to drive y toward zero
    xᵢ₊₁ = xᵢ - σᵢ × yᵢ × 2⁻ⁱ
    yᵢ₊₁ = yᵢ + σᵢ × xᵢ × 2⁻ⁱ
    zᵢ₊₁ = zᵢ - σᵢ × arctan(2⁻ⁱ)
```

After N iterations: z_N ≈ atan2(y₀, x₀)

### 4.2 Pre-computed Angle Table

| Iteration i | arctan(2⁻ⁱ) degrees | arctan(2⁻ⁱ) Q16.16 |
|-------------|---------------------|---------------------|
| 0           | 45.0000°            | 0x002D0000          |
| 1           | 26.5651°            | 0x001A9097          |
| 2           | 14.0362°            | 0x000E0939          |
| 3           | 7.1250°             | 0x00071FFE          |
| 4           | 3.5763°             | 0x000393A8          |
| 5           | 1.7899°             | 0x0001CA75          |
| 6           | 0.8952°             | 0x0000E544          |
| 7           | 0.4476°             | 0x000072A6          |
| 8           | 0.2238°             | 0x00003953          |
| 9           | 0.1119°             | 0x00001CAA          |
| 10          | 0.0560°             | 0x00000E55          |
| 11          | 0.0280°             | 0x0000072B          |
| 12          | 0.0140°             | 0x00000395          |
| 13          | 0.0070°             | 0x000001CB          |
| 14          | 0.0035°             | 0x000000E5          |
| 15          | 0.0018°             | 0x00000073          |

### 4.3 Quadrant Handling

CORDIC vectoring mode works when x > 0. For the full atan2 range:
```
IF x < 0 AND y ≥ 0: rotate by +180°, negate both (x,y), add π to result
IF x < 0 AND y < 0: rotate by -180°, negate both (x,y), subtract π from result
IF x ≥ 0: use directly
```

### 4.4 CORDIC Gain Compensation

CORDIC introduces a gain factor K:
```
K = Π(i=0 to N-1) √(1 + 2⁻²ⁱ) ≈ 1.6468 (for 16 iterations)
```

For atan2 in vectoring mode, we only need the angle z — the gain on x/y
doesn't matter. **No gain compensation needed for attitude estimation.**

---

## 5. Module Interface

### 5.1 Inputs

| Signal           | Width | Format         | Source      | Description                      |
|------------------|-------|----------------|-------------|----------------------------------|
| clk              | 1     | Clock          | System      | 100 MHz system clock             |
| rst_n            | 1     | Active-low     | System      | Synchronous reset                |
| filt_accel_x/y/z | 32    | Q16.16 signed  | IIR Filter  | Filtered accel (g)               |
| filt_gyro_x/y/z  | 32    | Q16.16 signed  | IIR Filter  | Filtered gyro (°/s)              |
| filt_valid        | 1     | Pulse          | IIR Filter  | New filtered data                |
| alpha_cf          | 32    | Q16.16 unsigned| CPU (AXI)   | Comp. filter coefficient (0.996) |
| dt                | 32    | Q16.16 unsigned| CPU (AXI)   | Sample period (0.001 or 0.002)   |

### 5.2 Outputs

| Signal      | Width | Format         | Dest         | Description                       |
|-------------|-------|----------------|--------------|-----------------------------------|
| roll        | 32    | Q16.16 signed  | PID (angle)  | Roll angle in degrees             |
| pitch       | 32    | Q16.16 signed  | PID (angle)  | Pitch angle in degrees            |
| yaw         | 32    | Q16.16 signed  | PID (angle)  | Yaw angle in degrees (gyro only)  |
| att_valid   | 1     | Pulse          | PID          | New attitude estimate ready       |

### 5.3 Configuration

| Register  | Width | Default        | Description                               |
|-----------|-------|----------------|-------------------------------------------|
| alpha_cf  | 32    | 0x0000FEB8     | 0.996 in Q16.16 (trust gyro 99.6%)       |
| dt        | 32    | 0x00000042     | 0.001 in Q16.16 (1 ms sample period)     |

---

## 6. Algorithm (Pseudocode)

### 6.1 Complementary Filter (Main Loop)

```
STATE: roll = 0, pitch = 0, yaw = 0   // in degrees, Q16.16

CONSTANT ONE_MINUS_ALPHA = 0x00010000 - alpha_cf  // (1 - 0.996) = 0.004 = 0x00000106

ON filt_valid:
    // Step 1: Gyro prediction (integrate rate)
    roll_pred  = roll  + q16_mul(filt_gyro_x, dt)    // degrees += °/s × s
    pitch_pred = pitch + q16_mul(filt_gyro_y, dt)
    yaw_pred   = yaw   + q16_mul(filt_gyro_z, dt)

    // Step 2: Accel reference angles (via CORDIC atan2)
    roll_accel  = cordic_atan2(filt_accel_y, filt_accel_z)   // degrees
    pitch_accel = cordic_atan2(-filt_accel_x, filt_accel_z)  // degrees

    // Step 3: Complementary fusion (roll and pitch only)
    roll  = q16_mul(alpha_cf, roll_pred)  + q16_mul(ONE_MINUS_ALPHA, roll_accel)
    pitch = q16_mul(alpha_cf, pitch_pred) + q16_mul(ONE_MINUS_ALPHA, pitch_accel)

    // Step 4: Yaw — gyro only (no accel reference for yaw)
    yaw = yaw_pred
    // Wrap yaw to [-180, +180] degrees
    IF yaw > 180.0: yaw = yaw - 360.0
    IF yaw < -180.0: yaw = yaw + 360.0

    ASSERT att_valid = 1 (single-cycle pulse)
```

### 6.2 CORDIC atan2 (Iterative, 16 cycles)

```
CONSTANT ATAN_TABLE[16] = {45.0, 26.565, 14.036, 7.125, 3.576, 1.790,
                            0.895, 0.448, 0.224, 0.112, 0.056, 0.028,
                            0.014, 0.007, 0.004, 0.002}  // in degrees, Q16.16

FUNCTION cordic_atan2(y_in, x_in) → angle in degrees (Q16.16):
    // Quadrant preprocessing
    negate_result = false
    IF x_in < 0:
        x = -x_in
        y = -y_in
        IF y_in >= 0:
            angle_offset = +180.0   // Q16.16: 0x00B40000
        ELSE:
            angle_offset = -180.0   // Q16.16: 0xFF4C0000
        negate_result = true
    ELSE:
        x = x_in
        y = y_in
        angle_offset = 0

    // Initialize
    z = 0   // accumulated angle

    // 16 iterations (vectoring mode: drive y to zero)
    FOR i = 0 TO 15:
        IF y < 0:
            // Rotate clockwise (positive direction)
            x_new = x - (y >> i)    // x - y×2⁻ⁱ (but y is negative, so x increases)
            y_new = y + (x >> i)    // y + x×2⁻ⁱ
            z_new = z - ATAN_TABLE[i]
        ELSE:
            // Rotate counter-clockwise (negative direction)
            x_new = x + (y >> i)    // x + y×2⁻ⁱ
            y_new = y - (x >> i)    // y - x×2⁻ⁱ
            z_new = z + ATAN_TABLE[i]
        x = x_new
        y = y_new
        z = z_new

    // Apply quadrant correction
    IF negate_result:
        RETURN angle_offset - z
    ELSE:
        RETURN z
```

### 6.3 Alternative Form (Fewer Multiplies)

```
// Equivalent to 6.1 but with 1 less multiply per axis:
// angle[n] = angle[n-1] + gyro×dt + (1-α) × (accel_angle - angle[n-1] - gyro×dt)

ON filt_valid:
    gyro_increment_x = q16_mul(filt_gyro_x, dt)
    gyro_increment_y = q16_mul(filt_gyro_y, dt)

    roll_accel  = cordic_atan2(filt_accel_y, filt_accel_z)
    pitch_accel = cordic_atan2(-filt_accel_x, filt_accel_z)

    roll_error  = roll_accel  - (roll  + gyro_increment_x)
    pitch_error = pitch_accel - (pitch + gyro_increment_y)

    roll  = roll  + gyro_increment_x + q16_mul(ONE_MINUS_ALPHA, roll_error)
    pitch = pitch + gyro_increment_y + q16_mul(ONE_MINUS_ALPHA, pitch_error)
    yaw   = yaw   + q16_mul(filt_gyro_z, dt)
```

---

## 7. Timing Budget

| Operation                | Clock Cycles | Notes                          |
|--------------------------|-------------|--------------------------------|
| Gyro × dt (3 axes)       | 3           | 3 DSP multiplies               |
| CORDIC atan2 (roll)      | 16          | 16 shift-add iterations        |
| CORDIC atan2 (pitch)     | 16          | Can pipeline with roll          |
| Complementary fusion     | 4           | 4 multiplies (2 per axis)      |
| Yaw wrap                 | 2           | Compare + add                  |
| **Total**                | **~41**     | **410 ns at 100 MHz**          |

At 500 Hz control rate (2 ms period): 410 ns = 0.02% of budget. Negligible.

If CORDIC instances are duplicated (2 parallel): total drops to ~25 cycles.

---

## 8. Accuracy Analysis

### 8.1 Static Accuracy

With α_cf = 0.996 and sensor noise:
```
Static angle noise ≈ (1 - α_cf) × accel_noise_angle
                    = 0.004 × arctan(0.003g / 1g)
                    = 0.004 × 0.17° = 0.0007° RMS
```

Essentially negligible. Static accuracy is limited by accel calibration.

### 8.2 Dynamic Accuracy

During acceleration (e.g., flying forward at 0.5g):
```
Accel error = arctan(0.5g / 1g) = 26.6° (accel thinks drone is tilted!)
Complementary filter rejects this because:
- Time constant τ = 0.25 s
- Maneuver lasts < τ → filter trusts gyro
- Steady-state error = 0 (accel eventually corrects)
```

Worst case: sustained 0.5g acceleration for > 1 second causes ~1° transient error.
Acceptable for RC flight.

### 8.3 Gyro Drift Rejection

With α_cf = 0.996 at 1 kHz, a 1°/s gyro bias is corrected by the accel:
```
Steady-state error = bias × τ = 1°/s × 0.25 s = 0.25°
```

This is the maximum attitude error from a 1°/s bias. Excellent.

---

## 9. Resource Estimate

| Resource       | Count  | Notes                                         |
|----------------|--------|-----------------------------------------------|
| Flip-flops     | ~250   | State (roll/pitch/yaw), CORDIC pipeline       |
| LUTs           | ~400   | CORDIC shift/add logic, comparators, muxes    |
| DSP48E1        | 2      | Gyro×dt, α multiplies (time-muxed)           |
| Block RAM      | 0      | ATAN_TABLE fits in LUT ROM (16 × 32-bit)      |
| CORDIC units   | 1–2    | 1 iterative (16 cyc) or 2 parallel (8 cyc)    |
