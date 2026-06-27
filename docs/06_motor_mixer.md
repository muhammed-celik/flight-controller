# Module 06: Motor Mixer and Output Scaling

## 1. Why This Module is Needed

A quadrotor has four motors but the flight controller computes commands in the body-frame axes: throttle (collective thrust), roll torque, pitch torque, and yaw torque. The motor mixer translates these abstract commands into individual motor speed values.

Without a mixer, there is no way to independently control attitude and altitude — the PID controller outputs torque demands, but individual ESCs need throttle percentages. The mixer implements the geometric relationship between motor positions/spin directions and the resulting forces/moments on the airframe.

For an **X-configuration** quadrotor (arms at 45° to the body axes), each motor contributes to multiple axes simultaneously, requiring a specific linear combination of all four control inputs.

---

## 2. Options/Approaches

| Approach | Pros | Cons | Selected |
|----------|------|------|----------|
| Fixed mixing matrix (hardcoded coefficients) | Minimal logic, zero latency, deterministic | Cannot change geometry without resynthesis | **Yes** |
| Configurable mixing matrix (register-based) | Supports any frame type (hex, Y6, etc.) | Extra registers, multipliers needed for arbitrary coefficients | No (overkill for quad) |
| Normalized mixing (÷4 per axis) | Mathematically clean | Requires division or shift; loses resolution | No |
| Unity mixing (±1 coefficients only) | Simple add/subtract, no multiply needed | Only works for symmetric X/+ quads | **Yes** (X-config has ±1 coefficients) |
| Software mixer on CPU | Maximum flexibility | Adds jitter, wastes cycles on trivial math | No |

### Desaturation Strategies

| Strategy | Description | Selected |
|----------|-------------|----------|
| Hard clamp (clip to 0/max) | Simple but causes attitude authority loss | Partial |
| Throttle reduction (lower all motors equally) | Preserves attitude control at expense of altitude | **Yes** |
| Priority-based (throttle > roll/pitch > yaw) | Yaw sacrificed first, then roll/pitch, throttle last | **Yes** |
| Airmode (allow negative throttle offset) | Maximum attitude authority even at zero throttle | Future enhancement |

---

## 3. Theoretical Foundations

### 3.1 X-Configuration Geometry

Motor layout (top view, nose = up):

```
    Front
  4       1
   \     /
    \   /
     CG
    /   \
   /     \
  3       2
    Rear
```

| Motor | Position | Spin Direction | Torque Contribution |
|-------|----------|---------------|---------------------|
| M1 | Front-Right | CW (clockwise) | +thrust, -roll, -pitch, +yaw |
| M2 | Rear-Right | CCW (counter-clockwise) | +thrust, -roll, +pitch, -yaw |
| M3 | Rear-Left | CW (clockwise) | +thrust, +roll, +pitch, +yaw |
| M4 | Front-Left | CCW (counter-clockwise) | +thrust, +roll, -pitch, -yaw |

### 3.2 Mixing Matrix

The mixing equations (all inputs and outputs in the same units):

```
M1 = throttle - roll_cmd - pitch_cmd + yaw_cmd
M2 = throttle - roll_cmd + pitch_cmd - yaw_cmd
M3 = throttle + roll_cmd + pitch_cmd + yaw_cmd
M4 = throttle + roll_cmd - pitch_cmd - yaw_cmd
```

In matrix form:

```
| M1 |   | +1  -1  -1  +1 |   | throttle |
| M2 | = | +1  -1  +1  -1 | × | roll_cmd |
| M3 |   | +1  +1  +1  +1 |   | pitch_cmd|
| M4 |   | +1  +1  -1  -1 |   | yaw_cmd  |
```

### 3.3 Output Saturation and Clamping

Motor values must be constrained to valid range:

```
For DSHOT:  M_clamped = clamp(M, 0, 2047)
For PWM:    M_clamped = clamp(M, 1000, 2000)  // in µs
```

### 3.4 Desaturation with Priority (Throttle Headroom)

When any motor exceeds max or goes below min:

```
// Step 1: Compute raw motor values
M_raw[i] = throttle + mix_coeff[i] × axis_cmds

// Step 2: Find overshoot/undershoot
max_motor = max(M_raw[1..4])
min_motor = min(M_raw[1..4])

// Step 3: Throttle adjustment (preserve attitude)
IF max_motor > MAX_THROTTLE:
    throttle_offset = MAX_THROTTLE - max_motor   // negative: lower all motors
IF min_motor < IDLE_SPEED:
    throttle_offset = IDLE_SPEED - min_motor     // positive: raise all motors

// Step 4: Apply offset and re-clamp
M_adjusted[i] = M_raw[i] + throttle_offset
M_final[i] = clamp(M_adjusted[i], IDLE_SPEED, MAX_THROTTLE)

// Step 5: If still saturated after throttle adjustment, scale yaw last
IF still_saturated:
    reduce yaw_cmd proportionally until within bounds
```

### 3.5 Idle Speed

When armed but at zero throttle, motors must spin at a minimum idle speed to:
- Maintain gyroscopic stability
- Ensure ESCs remain synchronized
- Provide immediate thrust response

```
IDLE_SPEED = 48 (DSHOT value, ~2.3% throttle)
```

### 3.6 Scaling from Q16.16 to Output Range

PID outputs are in Q16.16 (range approximately ±1.0 in normalized units). Throttle from RC is 0 to +1.0. These must be scaled to the output range:

```
For DSHOT (0-2047):
    motor_out = (normalized_value × 2047) >> 16    // Q16.16 × integer, take upper bits

For PWM (1000-2000 µs):
    motor_out = 1000 + (normalized_value × 1000) >> 16
```

---

## 4. Module Interface

### 4.1 Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `NUM_MOTORS` | 4 | Quadrotor |
| `OUTPUT_BITS` | 11 | 0-2047 for DSHOT |
| `CLK_FREQ` | 100 MHz | System clock |

### 4.2 Input Signals

| Signal | Width | Format | Source | Description |
|--------|-------|--------|--------|-------------|
| `clk` | 1 | — | System | 100 MHz system clock |
| `rst_n` | 1 | — | System | Active-low synchronous reset |
| `mix_start` | 1 | Pulse | PID Controller | Trigger: PID done, begin mixing |
| `throttle_cmd` | 32 | Q16.16 | RC Input | Collective thrust (0.0 to +1.0 normalized) |
| `roll_cmd` | 32 | Q16.16 | PID Controller | Roll torque demand (signed) |
| `pitch_cmd` | 32 | Q16.16 | PID Controller | Pitch torque demand (signed) |
| `yaw_cmd` | 32 | Q16.16 | PID Controller | Yaw torque demand (signed) |
| `armed` | 1 | — | Control Logic | Motor enable (0 = all outputs forced to 0) |
| `idle_speed` | 11 | Unsigned | RTL default/optional AXI | Minimum motor output when armed, default 48 |
| `max_throttle` | 11 | Unsigned | RTL default/optional AXI | Maximum motor output, default 2047 |
| `output_mode` | 1 | — | RTL default/optional AXI | 0 = DSHOT, 1 = PWM |

The first RTL build uses fixed X-quad mixing and synthesized defaults for idle
speed, maximum throttle, and output mode. Optional AXI overrides are for later
tuning and ESC compatibility; no CPU configuration is required for the default
DSHOT path.

### 4.3 Output Signals

| Signal | Width | Format | Destination | Description |
|--------|-------|--------|-------------|-------------|
| `motor_out[3:0]` | 4×11 | Unsigned | Motor Output | Individual motor commands (0-2047) |
| `mix_done` | 1 | Pulse | Timing Gen | Asserted when mixing complete |
| `motor_saturated` | 4 | — | Telemetry | Per-motor saturation indicator |
| `desaturated` | 1 | — | Telemetry | Throttle was adjusted for desaturation |

---

## 5. Algorithm

### 5.1 Main Mixing Procedure (Pseudocode)

```
ON mix_start pulse:
    IF NOT armed:
        motor_out[0..3] = 0
        motor_saturated = 0
        ASSERT mix_done
        RETURN

    // Step 1: Scale Q16.16 inputs to integer motor range (0-2047)
    thr_scaled = (throttle_cmd × max_throttle) >> 16     // 0 to max_throttle
    roll_scaled = (roll_cmd × max_throttle) >> 17        // ±max_throttle/2
    pitch_scaled = (pitch_cmd × max_throttle) >> 17      // ±max_throttle/2
    yaw_scaled = (yaw_cmd × max_throttle) >> 17          // ±max_throttle/2

    // Step 2: Apply mixing matrix (X-configuration)
    m1_raw = thr_scaled - roll_scaled - pitch_scaled + yaw_scaled
    m2_raw = thr_scaled - roll_scaled + pitch_scaled - yaw_scaled
    m3_raw = thr_scaled + roll_scaled + pitch_scaled + yaw_scaled
    m4_raw = thr_scaled + roll_scaled - pitch_scaled - yaw_scaled

    // Step 3: Find min/max for desaturation
    max_raw = max(m1_raw, m2_raw, m3_raw, m4_raw)
    min_raw = min(m1_raw, m2_raw, m3_raw, m4_raw)

    // Step 4: Desaturation — adjust throttle to keep within bounds
    offset = 0
    IF max_raw > max_throttle:
        offset = max_throttle - max_raw    // negative offset (lower all)
    IF (min_raw + offset) < idle_speed:
        offset = idle_speed - min_raw      // positive offset (raise all)

    // Step 5: Apply offset
    m1_adj = m1_raw + offset
    m2_adj = m2_raw + offset
    m3_adj = m3_raw + offset
    m4_adj = m4_raw + offset

    // Step 6: Final clamp (safety net)
    motor_out[0] = clamp(m1_adj, idle_speed, max_throttle)
    motor_out[1] = clamp(m2_adj, idle_speed, max_throttle)
    motor_out[2] = clamp(m3_adj, idle_speed, max_throttle)
    motor_out[3] = clamp(m4_adj, idle_speed, max_throttle)

    // Step 7: Set status flags
    FOR i IN 0..3:
        motor_saturated[i] = (motor_out[i] != m_adj[i])  // clamp was active
    desaturated = (offset != 0)

    ASSERT mix_done
```

### 5.2 Timing

```
Clock | Operation
------|----------
  1   | Scale throttle/roll/pitch/yaw to motor range (shift)
  2   | Apply mixing matrix (4 additions/subtractions in parallel)
  3   | Find min and max of 4 motor values
  4   | Compute desaturation offset
  5   | Apply offset to all 4 motors
  6   | Clamp all 4 motors, set flags, assert mix_done

Total: 6 clocks @ 100 MHz = 60 ns
```

---

## 6. Resource Estimate

| Resource | Count | Notes |
|----------|-------|-------|
| DSP48E1 | 0 | No multiplies needed — mix uses only add/sub; scaling uses shifts |
| LUTs | ~100 | 4× adder/subtractor (12-bit), comparators for min/max/clamp |
| Flip-Flops | ~50 | 4× motor output registers (11-bit), pipeline regs |
| BRAM | 0 | No storage needed |
| Fmax | >200 MHz | Pure combinational add/sub with single pipeline stage |
| Latency | 6 clocks | 60 ns from mix_start to mix_done |

### Artix-7 35T Utilization (XC7A35T)

| Resource | Available | Used | Utilization |
|----------|-----------|------|-------------|
| DSP48E1 | 90 | 0 | 0% |
| LUTs | 20,800 | 100 | 0.5% |
| Flip-Flops | 41,600 | 50 | 0.1% |
