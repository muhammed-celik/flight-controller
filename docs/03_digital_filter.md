# Module 03: Digital Low-Pass Filter (IIR)

## 1. Why This Module is Needed

Raw IMU data contains high-frequency noise from multiple sources:

- **Sensor noise:** Thermal noise in the MEMS proof mass (white noise floor)
- **Motor vibration:** Propellers spinning at 5000–20000 RPM produce vibration
  at their rotation frequency and harmonics (83–333 Hz for typical quads)
- **Structural resonance:** Frame resonance modes (typically 100–500 Hz)
- **Electrical noise:** Power supply ripple, switching noise

If this noise reaches the PID controller, it creates high-frequency oscillations
in the motor commands ("motor buzz"), which:
1. Wastes battery (motors heat up fighting noise, not moving the drone)
2. Reduces motor life (high-frequency current spikes stress windings)
3. Can excite mechanical resonances, causing structural failure
4. Makes the drone sound terrible (high-pitched whine)

The digital filter attenuates noise above a cutoff frequency while passing the
useful control signal through with minimal phase distortion. The filter cutoff
is the MOST IMPORTANT tuning parameter for flight performance — it directly
determines the achievable control bandwidth and stability margin.

---

## 2. Filter Type Options

### 2.1 Comparison Table

| Filter Type       | Order | Group Delay    | Passband Ripple | Complexity | Suitable? |
|-------------------|-------|----------------|-----------------|------------|-----------|
| Moving average    | N/A   | N/2 samples    | Sinc nulls      | Very low   | ❌ Poor frequency selectivity |
| 1st-order IIR     | 1     | 1/(2πfc) sec   | None            | Minimal    | ✅ Primary choice |
| 2nd-order IIR (biquad) | 2 | Depends on Q  | None (Butterworth) | Low    | ✅ If sharper roll-off needed |
| FIR (windowed)    | N     | (N-1)/2 samples| Depends on window | High    | ⚠️ High latency for sharp cutoff |
| Notch filter      | 2     | Minimal at fc   | None            | Low        | ✅ For specific motor freq |
| PT1 + PT2 cascade | 3+    | Sum of stages   | None            | Medium     | ⚠️ Overkill initially |

### 2.2 Chosen Approach: First-Order IIR (PT1)

**Why first-order IIR:**
- Simplest implementation: 1 multiply + 1 subtract + 1 add per sample per axis
- Predictable, constant group delay
- Tunable cutoff via single coefficient (α)
- Used in Betaflight, KISS, and all major FC firmware
- Adequate as a first stage when combined with the MPU9250's configured DLPF;
  final cutoff/notch choices must be based on propeller-on vibration logs

**Why not FIR:**
- A 50-tap FIR at 1 kHz with fc=80 Hz has group delay = 25 ms (12.5× worse than PT1)
- Requires 50 multiplies per sample per axis = 300 multiplies total
- Would consume all 90 DSP slices on Artix-7 35T

**Why not higher-order IIR (initially):**
- 2nd-order Butterworth at same fc has ~2× the group delay
- More complex coefficient calculation
- Can be added later if motor vibration requires sharper attenuation

---

## 3. Theoretical Foundations

### 3.1 Continuous-Time Low-Pass Filter (Analog Prototype)

The simplest low-pass filter is a first-order RC circuit (PT1):

```
Transfer function: H(s) = ωc / (s + ωc)

Where: ωc = 2π × fc  (cutoff frequency in rad/s)
```

Properties:
- Unity gain at DC (0 Hz): |H(0)| = 1
- -3 dB at cutoff: |H(jωc)| = 1/√2 ≈ 0.707
- Roll-off: -20 dB/decade above fc
- Phase at cutoff: -45°
- Group delay: τ = 1/(2πfc) seconds

### 3.2 Discretization: Bilinear Transform (Tustin's Method)

To implement in digital hardware, we discretize using the bilinear transform:

```
s → (2/T) × (1 - z⁻¹) / (1 + z⁻¹)
```

Where T = 1/fs = sample period (1 ms at 1 kHz).

This maps the analog frequency axis to digital while preserving stability.

**Frequency warping:** The bilinear transform introduces warping:
```
ω_digital = (2/T) × tan(ω_analog × T/2)
```

For fc = 80 Hz at fs = 1000 Hz: warping is ~3%. We pre-warp:
```
ωc_warped = (2/T) × tan(π × fc / fs) = 2000 × tan(π × 80/1000)
          = 2000 × tan(0.2513) = 2000 × 0.2553 = 510.6 rad/s
```

### 3.3 Simplified Exponential Moving Average (EMA) Approximation

For fs >> fc (which is true: 1000 >> 80), the bilinear transform result
simplifies to the exponential moving average:

```
y[n] = α × x[n] + (1 - α) × y[n-1]
```

Where the smoothing coefficient α is:
```
α = dt / (RC + dt) = (2π × fc × dt) / (1 + 2π × fc × dt)
```

With dt = 1/fs = 0.001 s and fc = 80 Hz:
```
α = (2π × 80 × 0.001) / (1 + 2π × 80 × 0.001)
  = 0.5027 / 1.5027
  = 0.3345
```

**Equivalent form (fewer multiplies):**
```
y[n] = y[n-1] + α × (x[n] - y[n-1])
```

This uses only 1 multiply, 1 subtract, and 1 add.

### 3.4 Coefficient Calculation for Various Cutoffs

| fc (Hz) | α (Q16.16 hex) | α (decimal) | Group delay (ms) | -3dB freq (Hz) |
|----------|----------------|-------------|-------------------|-----------------|
| 20       | 0x00002066     | 0.1257      | 7.96              | 20              |
| 40       | 0x00003F37     | 0.2469      | 3.98              | 40              |
| 60       | 0x00005ADA     | 0.3541      | 2.65              | 60              |
| 80       | 0x000055B0     | 0.3345      | 1.99              | 80              |
| 100      | 0x00008A5B     | 0.5404      | 1.59              | 100             |
| 150      | 0x0000B853     | 0.7194      | 1.06              | 150             |
| 200      | 0x0000D067     | 0.8133      | 0.80              | 200             |

**Correction for α at 80 Hz:**
```
α = 2π × 80 × 0.001 / (1 + 2π × 80 × 0.001) = 0.5027 / 1.5027 = 0.3345
In Q16.16: round(0.3345 × 65536) = 21924 = 0x000055A4
```

### 3.5 Group Delay Impact on Stability

The filter adds phase lag to the control loop. For a first-order IIR:
```
Phase lag at frequency f = -arctan(f / fc)
Group delay ≈ 1 / (2π × fc)  (constant for first-order)
```

At the PID crossover frequency (typically fc_pid ≈ fc_filter / 3):
```
fc = 80 Hz → group delay = 1.99 ms
Phase loss at 30 Hz = -arctan(30/80) = -20.6°
```

Total loop phase margin consumed by filter: ~20°
With typical 60° available: 60° - 20° = 40° remaining (safe, >30° minimum)

If fc = 30 Hz (with noisier sensor): group delay = 5.3 ms
Phase loss at 30 Hz = -arctan(30/30) = -45° → only 15° margin (DANGEROUS)

**This is why sensor noise matters: it forces a lower fc, which destroys stability.**

### 3.6 Second-Order Biquad (Future Extension)

For sharper motor-frequency attenuation, a 2nd-order Butterworth:
```
y[n] = b0×x[n] + b1×x[n-1] + b2×x[n-2] - a1×y[n-1] - a2×y[n-2]
```

5 multiplies per sample per axis. Only needed if motor vibration is severe.

---

## 4. Module Interface

### 4.1 Inputs

| Signal          | Width | Format          | Source       | Description                      |
|-----------------|-------|-----------------|--------------|----------------------------------|
| clk             | 1     | Clock           | System       | 100 MHz system clock             |
| rst_n           | 1     | Active-low      | System       | Synchronous reset                |
| cal_accel_x/y/z | 32    | Q16.16 signed   | Calibration  | Calibrated acceleration          |
| cal_gyro_x/y/z  | 32    | Q16.16 signed   | Calibration  | Calibrated angular rate          |
| cal_valid        | 1     | Pulse           | Calibration  | New calibrated data available    |
| alpha_gyro       | 32    | Q16.16 unsigned | CPU (AXI)    | Gyro filter coefficient          |
| alpha_accel      | 32    | Q16.16 unsigned | CPU (AXI)    | Accel filter coefficient         |

### 4.2 Outputs

| Signal           | Width | Format          | Dest             | Description                    |
|------------------|-------|-----------------|------------------|--------------------------------|
| filt_accel_x/y/z | 32    | Q16.16 signed   | RTL estimator/AXI | Filtered acceleration for complementary filter or CPU |
| filt_gyro_x/y/z  | 32    | Q16.16 signed   | RTL PID/AXI      | Filtered angular rate          |
| filt_valid        | 1     | Pulse           | Next stage       | Filtered data ready            |

### 4.3 Configuration

| Register    | Width | Default        | Description                               |
|-------------|-------|----------------|-------------------------------------------|
| alpha_gyro  | 32    | 0x000055A4     | Gyro filter α (default fc=80 Hz)          |
| alpha_accel | 32    | 0x00002066     | Accel filter α (default fc=20 Hz)         |
| filter_en   | 1     | 1              | Enable filter (0=passthrough for debug)   |

Note: Gyro and accel have different cutoff frequencies because:
- Gyro feeds rate PID (needs higher bandwidth, 80 Hz)
- Accel feeds either the RTL complementary filter or CPU EKF; its cutoff must
  preserve estimator dynamics rather than assuming it is gravity-only

---

## 5. Algorithm (Pseudocode)

### 5.1 First-Order IIR Filter

```
// State: previous output for each axis (initialized to 0 on reset)
STATE: prev_gyro_x, prev_gyro_y, prev_gyro_z  = 0
STATE: prev_accel_x, prev_accel_y, prev_accel_z = 0

ON cal_valid:
    IF filter_en == 0:
        // Passthrough (debug mode)
        filt_gyro_x  = cal_gyro_x
        filt_gyro_y  = cal_gyro_y
        filt_gyro_z  = cal_gyro_z
        filt_accel_x = cal_accel_x
        filt_accel_y = cal_accel_y
        filt_accel_z = cal_accel_z
    ELSE:
        // IIR filter: y[n] = y[n-1] + α × (x[n] - y[n-1])
        filt_gyro_x  = prev_gyro_x  + q16_mul(alpha_gyro, cal_gyro_x  - prev_gyro_x)
        filt_gyro_y  = prev_gyro_y  + q16_mul(alpha_gyro, cal_gyro_y  - prev_gyro_y)
        filt_gyro_z  = prev_gyro_z  + q16_mul(alpha_gyro, cal_gyro_z  - prev_gyro_z)
        filt_accel_x = prev_accel_x + q16_mul(alpha_accel, cal_accel_x - prev_accel_x)
        filt_accel_y = prev_accel_y + q16_mul(alpha_accel, cal_accel_y - prev_accel_y)
        filt_accel_z = prev_accel_z + q16_mul(alpha_accel, cal_accel_z - prev_accel_z)

    // Update state
    prev_gyro_x  = filt_gyro_x
    prev_gyro_y  = filt_gyro_y
    prev_gyro_z  = filt_gyro_z
    prev_accel_x = filt_accel_x
    prev_accel_y = filt_accel_y
    prev_accel_z = filt_accel_z

    ASSERT filt_valid = 1 (single-cycle pulse)
```

### 5.2 Coefficient Pre-Computation (CPU, at boot or on parameter change)

```
// Called when user changes filter cutoff frequency
FUNCTION compute_alpha(fc_hz, fs_hz) → Q16.16:
    dt = 1.0 / fs_hz
    rc = 1.0 / (2π × fc_hz)
    alpha = dt / (rc + dt)
    RETURN float_to_q16(alpha)

// Example: compute_alpha(80, 1000) → 0x000055A4
```

### 5.3 Initialization (First Sample Handling)

```
// On first valid sample after reset, initialize filter state to input
// (avoids slow convergence from zero)
ON first cal_valid after reset:
    prev_gyro_x  = cal_gyro_x
    prev_gyro_y  = cal_gyro_y
    prev_gyro_z  = cal_gyro_z
    prev_accel_x = cal_accel_x
    prev_accel_y = cal_accel_y
    prev_accel_z = cal_accel_z
    // Output = input (no filtering on first sample)
```

---

## 6. Fixed-Point Considerations

### 6.1 Precision Analysis

The critical operation is: `α × (x - y_prev)`

Where:
- α = 0.3345 in Q16.16 = 0x000055A4 (16 fractional bits used)
- (x - y_prev) could be as large as ±2000°/s (full gyro range)
- In Q16.16: ±2000 × 65536 = ±131,072,000 (fits in 32 bits signed)

The multiply: 0x000055A4 × large_delta:
- Worst case: 0x000055A4 × 0x07D00000 (2000.0 in Q16.16)
- = 21924 × 131072000 = 2,873,655,296,000 (needs 42 bits)
- After >>16: = 43,832,320 (fits in 32 bits)

**No overflow possible** because α < 1.0, so `α × anything` < `anything`.

### 6.2 Quantization Noise

With Q16.16, the minimum representable change is 1/65536 ≈ 0.0000153.
For gyro (°/s): this is 0.0000153°/s — far below sensor noise floor.
For accel (g): this is 0.0000153g — also negligible.

No precision concerns with Q16.16 for this filter.

---

## 7. Frequency Response Verification

To verify the filter works correctly, the CPU can inject a known-frequency
signal and measure attenuation:

| Input freq (Hz) | Expected output (dB) | α = 0.3345 (fc=80 Hz) |
|-----------------|---------------------|------------------------|
| 0 (DC)          | 0 dB (unity)        | 0 dB                   |
| 40              | -0.97 dB            | -0.97 dB               |
| 80              | -3.01 dB            | -3.01 dB               |
| 160             | -6.99 dB            | -6.99 dB               |
| 500             | -16.0 dB            | -16.0 dB               |

---

## 8. Resource Estimate

| Resource       | Count  | Notes                                        |
|----------------|--------|----------------------------------------------|
| Flip-flops     | ~200   | 6 × 32-bit state registers + pipeline        |
| LUTs           | ~120   | Subtractors, adders, muxes                   |
| DSP48E1        | 2      | Time-muxed for 6 multiplications             |
| Block RAM      | 0      | States fit in registers                      |
| AXI registers  | 3      | alpha_gyro, alpha_accel, filter_en           |

**Total latency:** ~6–10 clock cycles (60–100 ns) — negligible.
