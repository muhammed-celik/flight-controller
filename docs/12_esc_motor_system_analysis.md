# Module 12: ESC, Motor Selection, and System Bandwidth Analysis

## 1. Why This Analysis is Needed

A flight controller is a closed-loop control system. Its performance is limited
by the **slowest element in the loop**, not the fastest. Designing a 1 kHz IMU
and 80 Hz filter is pointless if the motor can only respond at 30 Hz. Conversely,
a slow sensor bottlenecks the entire chain even if the actuator is fast.

This document identifies the true bottleneck (motor mechanical response),
justifies each component's specification, and guides ESC/motor selection to
match the FPGA flight controller's capabilities.

**The control loop bandwidth hierarchy (slowest to fastest):**
```
Motor+prop mechanical → PID output useful bandwidth → Filter cutoff →
IMU sample rate → FPGA computation → ESC protocol speed
```

Each element should be ~2–5× faster than the one to its left to avoid
being a limiting factor.

---

## 2. System Bandwidth Chain

### 2.1 Complete Signal Path Timing

| Stage | Latency/Period | Bandwidth | Limits What? |
|-------|---------------|-----------|--------------|
| GY-91 MPU9250 sampling | 1 ms (1 kHz) | 500 Hz Nyquist | Filter input rate |
| I²C IMU burst | ~0.4 ms | — | Acquisition latency and bus occupancy |
| Calibration | ~100 ns | — | Nothing (passthrough) |
| IIR filter | Measured/tuned | Configurable | Rate-loop input bandwidth |
| RTL complementary filter | Bounded RTL latency | 500 Hz target | Outer-loop attitude in RTL mode |
| Optional CPU EKF | Must finish before mailbox deadline | 500 Hz target | Outer-loop attitude in EKF mode |
| RTL PID computation | ~1 µs target | — | Negligible after RTL verification |
| Motor mixer | ~50 ns | — | Nothing (computation) |
| DSHOT600 frame TX | 26.7 µs | 37 kHz max rate | ESC command rate |
| ESC internal loop | 21–42 µs | 24–48 kHz | Commutation timing |
| Motor electrical (L/R) | 0.05–0.2 ms | 1–5 kHz | Current rise time |
| **Motor+prop mechanical** | **15–50 ms** | **3–10 Hz** | **RPM change rate** |
| Airframe response | 50–200 ms | 1–3 Hz | Attitude change rate |

### 2.2 The True Bottleneck

**Motor + propeller mechanical inertia** is the dominant limiting factor:

```
Mechanical time constant: τ_m = J / (Kt × Kv × V)

Where:
  J = moment of inertia of rotor + propeller (kg·m²)
  Kt = motor torque constant (N·m/A)
  Kv = motor velocity constant (rad/s/V)
  V = supply voltage

Typical 2306 motor + 5" prop:
  J ≈ 1.5 × 10⁻⁶ kg·m²
  Kt × Kv ≈ 0.001 N·m/A × 260 rad/s/V = 0.26
  V = 14.8V (4S LiPo)
  τ_m = 1.5e-6 / (0.26 × 14.8) ≈ 0.4 ms (electrical)

But propeller aerodynamic drag dominates:
  τ_mech ≈ 15–30 ms for 5" prop (measured, not theoretical)
  → Mechanical bandwidth ≈ 1/(2π × τ) ≈ 5–10 Hz for full throttle step
  → For small perturbations (PID corrections): ~30–50 Hz effective
```

The motor can track small corrections at ~40 Hz, but large throttle changes
(0→100%) take 20–30 ms to complete.

### 2.3 Why Each Specification is Justified

| Component | Spec | Justification |
|-----------|------|---------------|
| IMU at 1 kHz | 25× motor BW | Oversampling for filter settling, noise averaging, bias estimation |
| Gyro filter initial fc=40-80 Hz | Approximately 1-2× motor BW | Tune from MPU9250 and airframe logs |
| Accel filter fc=20 Hz | Gravity-only | Only measures tilt (quasi-static), vibration rejection |
| Control loop 500 Hz | 12× motor BW | Sufficient phase margin, low latency for rate loop |
| DSHOT600 | 74× loop rate | Protocol simplicity, not speed, drove this choice |
| Baro at 25 Hz | 50× altitude BW | Altitude changes at 0.5–1 Hz, massive margin |
| Mag at 50 Hz | 50× heading BW | Heading changes at ~1 Hz, massive margin |

### 2.4 What Happens If Specs Are Mismatched

| Mismatch | Consequence |
|----------|-------------|
| IMU too slow (< 200 Hz) | Insufficient sampling margin for the intended rate loop |
| Filter fc too high (> motor BW) | Noise passes through, motors buzz, no performance gain |
| Filter fc too low (< 20 Hz) | Excessive phase lag, oscillations, instability |
| Control loop too slow (< 100 Hz) | Phase margin lost, can't control fast disturbances |
| ESC too slow (PWM 50 Hz) | Massive quantization of commands, sluggish response |
| Motor too slow (heavy prop) | No amount of fast sensors helps — system is sluggish |

---

## 3. ESC Selection

### 3.1 ESC Firmware Comparison

| Parameter | BLHeli_S | BLHeli_32 | AM32 | KISS | SimonK |
|-----------|----------|-----------|------|------|--------|
| MCU | EFM8 (8-bit) | STM32 (32-bit) | STM32 (32-bit) | Proprietary | ATmega (8-bit) |
| Max protocol | DSHOT600 | DSHOT1200 | DSHOT1200 | DSHOT600 | PWM only |
| Internal loop | 24 kHz | 48 kHz | 48 kHz | 48 kHz | 8 kHz |
| Commutation | Trapezoidal | Sinusoidal (opt.) | Sinusoidal (opt.) | Sinusoidal | Trapezoidal |
| Telemetry | Bidirectional DSHOT | Serial + bidir | Serial + bidir | Serial | None |
| Startup | Good | Excellent | Excellent | Excellent | Fair |
| Braking | Active | Active + regen | Active + regen | Active | Passive |
| Price range | $5–10 | $12–25 | $8–18 | $20–35 | $3–5 |
| Availability | Ubiquitous | Common | Growing | Niche | Obsolete |
| Open source | No (closed) | No (closed) | Yes | No | Yes |
| Configurability | BLHeliSuite | BLHeliSuite32 | AM32 config | KISS GUI | Limited |

### 3.2 ESC Protocol Comparison

| Protocol | Bit Rate | Resolution | Latency | Jitter | CRC | Bidirectional |
|----------|---------|------------|---------|--------|-----|---------------|
| Standard PWM | N/A | ~1000 steps (1-2ms) | 2 ms | ±1–5 µs | No | No |
| OneShot125 | N/A | ~1000 steps | 250 µs | ±0.5 µs | No | No |
| OneShot42 | N/A | ~1000 steps | 84 µs | ±0.5 µs | No | No |
| MultiShot | N/A | ~1000 steps | 25 µs | ±0.1 µs | No | No |
| DSHOT150 | 150 kbit/s | 2048 steps | 106.7 µs | 0 (digital) | Yes | Optional |
| DSHOT300 | 300 kbit/s | 2048 steps | 53.3 µs | 0 (digital) | Yes | Optional |
| **DSHOT600** | **600 kbit/s** | **2048 steps** | **26.7 µs** | **0** | **Yes** | **Optional** |
| DSHOT1200 | 1200 kbit/s | 2048 steps | 13.3 µs | 0 | Yes | Optional |

### 3.3 Why DSHOT600

- **Zero jitter:** Digital protocol — ESC sees exact throttle value every time
- **CRC protection:** 4-bit CRC detects bit errors (critical for reliability)
- **2048 resolution:** 11-bit throttle (vs ~1000 for PWM) — smoother control
- **26.7 µs latency:** 0.5% of our 2 ms control period — negligible
- **Universal support:** All modern ESCs (BLHeli_S/32, AM32, KISS) support it
- **Simple FPGA implementation:** Just timed pulse widths, no analog concerns

DSHOT1200 offers no benefit: our 500 Hz command rate means each frame is sent
every 2 ms. Whether the frame takes 26.7 µs (DSHOT600) or 13.3 µs (DSHOT1200)
is irrelevant — both are <<2 ms.

### 3.4 Recommended ESC: BLHeli_32 (or AM32)

**Reasons:**
- 48 kHz loop rate: smoothest possible commutation, lowest motor noise
- Sinusoidal drive option: reduces vibration (helps IMU)
- Bidirectional DSHOT: ESC reports RPM back to FC (enables RPM filtering)
- Active braking: faster RPM reduction when PID commands decrease thrust
- Configurable timing advance: optimize for specific motor
- 4S/6S support: flexible battery choice

**BLHeli_S is acceptable** for initial testing (cheaper, still supports DSHOT600),
but BLHeli_32 should be the production choice.

---

## 4. Motor Selection

### 4.1 Motor Size Classes

| Prop Size | Motor Size | Typical KV (4S) | Typical KV (6S) | Use Case |
|-----------|-----------|-----------------|-----------------|----------|
| 3" | 1404–1507 | 3000–4500 | 1900–2500 | Micro/cinewhoop |
| 5" | 2205–2306 | 1700–2700 | 1200–1900 | Standard quad |
| 7" | 2806–3115 | 1300–1700 | 900–1300 | Long-range |
| 10" | 3508–4010 | 400–700 | 300–500 | Heavy lift/cinema |

### 4.2 Motor Comparison (5" Quad, 4S LiPo)

| Motor | Size | KV | Max Thrust | Weight | τ_mech (est.) | BW (est.) | Best For |
|-------|------|-----|-----------|--------|---------------|-----------|----------|
| EMAX ECO II 2207 | 22×7 | 1700 | 850g | 28g | ~30 ms | ~5 Hz | Efficiency |
| T-Motor F40 Pro IV | 23×6 | 1950 | 1050g | 32g | ~22 ms | ~7 Hz | All-round |
| EMAX ECO II 2306 | 23×6 | 2400 | 1200g | 33g | ~18 ms | ~9 Hz | Freestyle |
| T-Motor Velox V2 | 22×6 | 2550 | 1350g | 30g | ~16 ms | ~10 Hz | Racing |
| iFlight XING2 | 22×7 | 2755 | 1500g | 29g | ~14 ms | ~11 Hz | Pure racing |

### 4.3 Motor Parameters That Affect Control

**KV Rating (RPM per Volt):**
```
Higher KV → lower inductance → faster current rise → faster torque response
Higher KV → spins faster → more prop drag → lower effective inertia ratio
Higher KV → less torque per amp → needs more current for same thrust

Trade-off: Higher KV = faster response BUT less efficient, more heat
```

**Stator Volume (affects torque):**
```
Torque ∝ stator_diameter² × stator_height
2306 (23mm × 6mm): Volume = π × 11.5² × 6 = 2490 mm³
2207 (22mm × 7mm): Volume = π × 11² × 7 = 2661 mm³

Larger stator = more torque = faster RPM changes for given prop
```

**Propeller Inertia (dominates mechanical response):**
```
J_prop ∝ mass × radius²
5" prop: J ≈ 1.2–2.0 × 10⁻⁶ kg·m²
3" prop: J ≈ 0.2–0.4 × 10⁻⁶ kg·m²
7" prop: J ≈ 4–8 × 10⁻⁶ kg·m²

Smaller prop = faster response (but less thrust)
```

### 4.4 Recommended Motor: 2306 2400KV (4S) or 2306 1750KV (6S)

**For initial development (2306 2400KV on 4S):**
- Good thrust-to-weight (5:1 minimum for agile flight)
- Moderate mechanical time constant (~18 ms)
- Well-matched to our 80 Hz filter cutoff
- Common, cheap, replaceable when you crash
- Sufficient bandwidth for the PID loop

---

## 5. Propeller Selection Impact

### 5.1 Prop Comparison (5" Class)

| Propeller | Pitch | Blades | Inertia (relative) | Thrust | Efficiency | Response |
|-----------|-------|--------|---------------------|--------|------------|----------|
| 5×3 | 3" | 2 | Low | Moderate | Best | Fastest |
| 5×4 | 4" | 2 | Medium | High | Good | Fast |
| 5×4.3 | 4.3" | 3 (tri) | High | Highest | Moderate | Slower |
| 5×3.5 | 3.5" | 3 (tri) | Medium-High | High | Moderate | Medium |

**Higher pitch:** More thrust per RPM but more drag → slower RPM changes.
**More blades:** More thrust at given RPM but more inertia → slower response.

**Recommendation:** Start with 2-blade 5040 or 5045 for best control response.
Switch to triblade for more thrust once control loop is tuned.

---

## 6. Matching Sensors to Actuator Bandwidth

### 6.1 Required vs Actual Sensor Specifications

| Sensor | Required Rate | Actual Rate | Required Accuracy | Actual | Verdict |
|--------|--------------|-------------|-------------------|--------|---------|
| MPU9250 gyro | >160 Hz (2× motor BW) | 1000 Hz target | Determine from logs | Not yet characterized | Validate on mounted airframe |
| MPU9250 accel | >4 Hz (2× tilt BW) | 1000 Hz target | Determine from estimator needs | Not yet characterized | Validate vibration rejection |
| BMP280 baro | >2 Hz (2× alt BW) | 25-50 Hz target | <1 m after filtering | Not yet characterized | Validate enclosure/prop-wash effects |
| AK8963 mag | >2 Hz (2× heading BW) | 50-100 Hz target | <2° after calibration | Not yet characterized | Validate motor-current interference |

### 6.2 Where You Could Save Resources (But Shouldn't)

| Optimization | Savings | Risk | Verdict |
|-------------|---------|------|---------|
| Reduce IMU to 500 Hz | Frees substantial shared-I²C time | Less rate-loop sampling margin | Profile before changing |
| Reduce baro to 10 Hz | ~60% I²C bus time | None for hover | Acceptable |
| Reduce mag to 10 Hz | ~80% I²C bus time | None for RC flight | Acceptable |
| Remove baro entirely | 1 I²C device less | No altitude hold | OK for manual-only |
| Remove mag entirely | 1 I²C device less | Yaw drift over minutes | OK for short flights |
| Use DSHOT300 instead | Longer frame time (53µs) | None | Acceptable |

### 6.3 Where Specs Are Critical (Cannot Reduce)

| Spec | Why It's Critical |
|------|-------------------|
| Gyro at ≥500 Hz | Maintains sampling margin for a 250-500 Hz rate loop |
| Measured vibration/noise budget | Directly determines filter and notch settings |
| Control loop ≥250 Hz | Below this, rate loop phase margin becomes dangerous |
| DSHOT (not PWM) | PWM jitter creates limit cycles in PID at high gains |
| ESC loop ≥24 kHz | Below this, commutation noise feeds back into vibration |

---

## 7. Complete System Budget

### 7.1 Timing Budget Per Control Period (2 ms = 500 Hz)

| Task | Initial budget | Owner |
|---|---:|---|
| MPU9250 I²C burst (one of two 1 kHz reads) | ~0.4 ms each | RTL bus engine |
| AK8963/BMP280 scheduled reads | Amortized in free bus slots | RTL scheduler |
| Calibration + PT1 + rate PID + mixer | <10 µs target | RTL datapath |
| DSHOT600 frame transmission | 26.7 µs | RTL output engine |
| Complementary filter + CORDIC | Profile after RTL implementation | RTL estimator |
| AXI snapshot + EKF + mailbox | <1 ms target when enabled | CPU software |

I²C bus occupancy and both estimator implementations must be measured on the
selected FPGA and exact GY-91 board. In EKF mode, execution also depends on CPU
configuration and compiler options. The selected estimator has a freshness
watchdog, while the RTL rate loop remains independent of either attitude path.

### 7.2 Latency Budget (Sensor Event to Motor Response)

| Stage | Cumulative Latency |
|-------|-------------------|
| MPU9250 sample becomes readable | 0 |
| Shared-I²C burst completes | ~0.4 ms target |
| RTL calibration/filter/rate PID/mixer | <0.01 ms target |
| DSHOT frame completes | +0.0267 ms |
| ESC processing and motor-current response | Device dependent |

The rate path does not wait for either attitude estimator. Complementary mode
stays within RTL; EKF mode additionally includes AXI transfer and software
execution. Both have separate validity/freshness monitoring. Final latency
claims require logic-analyzer and software profiling measurements.

---

## 8. Recommendations Summary

| Component | Recommendation | Why |
|-----------|---------------|-----|
| ESC | BLHeli_32 or AM32 | 48kHz loop, DSHOT600, telemetry, active braking |
| Motor | 2306 2400KV (4S) | Good thrust, moderate response, common |
| Prop | 5040 or 5045 (2-blade) | Fast response, good efficiency |
| Battery | 4S 1300–1550 mAh LiPo | Standard for 5" quad, 5+ min flight |
| Protocol | DSHOT600 | Sufficient for 500 Hz loop, universal support |
| Frame | 5" true-X or stretched-X | Standard geometry for mixing matrix |

### 8.1 First Flight Configuration

For initial testing and PID tuning:
```
Motor: EMAX ECO II 2306 2400KV (cheap, replaceable)
ESC: Any BLHeli_S 35A (cheapest DSHOT600 option)
Prop: Gemfan 5040 (low inertia, predictable)
Battery: 4S 1300mAh (light, keeps thrust-to-weight >4:1)
AUW target: <600g (with battery)
Thrust-to-weight: >5:1
```

### 8.2 Upgrade Path

Once basic flight is achieved:
```
1. Switch to BLHeli_32 ESCs (enable RPM telemetry)
2. Add RPM-based notch filter (removes motor-frequency vibration)
3. Try triblade props (more thrust, test if vibration is acceptable)
4. Tune PID gains with Blackbox logging (from MicroBlaze or RV32 CPU)
5. Enable altitude hold (barometer feedback)
6. Enable heading hold (magnetometer feedback)
```
