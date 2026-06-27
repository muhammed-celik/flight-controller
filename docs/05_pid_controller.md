# Module 05: Cascaded PID Controller

## 1. Why This Module is Needed

A quadrotor is an inherently unstable plant — without active closed-loop control it will diverge from level flight within tens of milliseconds. A PID (Proportional-Integral-Derivative) controller provides the real-time feedback mechanism that computes corrective torque commands based on the error between the desired state (setpoint from the pilot) and the measured state (from the IMU/attitude estimator).

A **cascaded** architecture (outer angle loop → inner rate loop) is used rather than a single-loop controller because:

| Aspect | Single-Loop PID | Cascaded PID (Angle → Rate) |
|--------|----------------|----------------------------|
| Disturbance rejection | Slow — must propagate through full plant | Fast — inner loop rejects rate disturbances immediately |
| Tuning difficulty | High — one set of gains must handle both angle and rate dynamics | Lower — each loop tuned independently |
| Stability margin | Narrow | Wider — inner loop "linearizes" the plant for the outer loop |
| Setpoint response | Derivative kick on angle step | Derivative on measurement avoids kick; inner loop limits rate |
| Industry adoption | Rarely used in multirotors | Universal standard (Betaflight, PX4, ArduPilot) |

The outer loop runs at 500 Hz (matching the control loop) and outputs desired angular rates. The inner loop also runs at 500 Hz (could be 1 kHz with direct gyro input) and outputs torque commands.

---

## 2. Options/Approaches

| Approach | Pros | Cons | Selected |
|----------|------|------|----------|
| Parallel PID (all axes simultaneously) | Lowest latency, simple timing | Uses 6× hardware (12 multipliers) | No |
| Time-multiplexed PID (sequential axes) | Minimal DSP usage (4 DSP48E1 shared) | Adds pipeline latency (~12 clocks per instance) | **Yes** |
| Software PID on MicroBlaze/RV32 | Flexible, easy to modify gains | Scheduling jitter enters the control path | No for inner rate loop |
| PID with derivative filter (1st-order LPF on D-term) | Reduces noise amplification | Extra multiply per axis | Yes (combined) |
| PI-D (derivative on measurement only) | Avoids derivative kick on setpoint change | Slightly different behavior on setpoint ramps | **Yes** |

---

## 3. Theoretical Foundations

### 3.1 Continuous-Time PID

The ideal continuous PID controller:

```
u(t) = Kp · e(t) + Ki · ∫e(τ)dτ + Kd · de(t)/dt
```

Where:
- `e(t) = setpoint(t) - measurement(t)` — the error signal
- `Kp` — proportional gain (immediate response to error)
- `Ki` — integral gain (eliminates steady-state error)
- `Kd` — derivative gain (provides damping, predicts future error)

### 3.2 Discrete-Time PID (Backward Euler)

For a fixed sample period `dt = 1/500 = 0.002 s`:

```
P[n] = Kp × e[n]

I[n] = I[n-1] + Ki × e[n] × dt

D[n] = Kd × (measurement[n-1] - measurement[n]) / dt
```

Note: **Derivative is computed on measurement** (not error) to prevent derivative kick when the setpoint changes abruptly (e.g., pilot stick movement).

**Output:**
```
output[n] = P[n] + I[n] + D[n]
```

### 3.3 Anti-Windup via Integral Clamping

When the output saturates (hits ±MAX_OUTPUT), the integrator must stop accumulating to prevent windup:

```
if (output[n] ≥ +MAX_OUTPUT) AND (e[n] > 0):
    I[n] = I[n-1]          // freeze integrator (don't increase)
else if (output[n] ≤ -MAX_OUTPUT) AND (e[n] < 0):
    I[n] = I[n-1]          // freeze integrator (don't decrease)
else:
    I[n] = I[n-1] + Ki × e[n] × dt   // normal integration
```

### 3.4 Output Saturation

```
output_clamped[n] = clamp(output[n], -MAX_OUTPUT, +MAX_OUTPUT)
```

Where MAX_OUTPUT in Q16.16 corresponds to the maximum torque command the mixer can accept.

### 3.5 Cascaded Loop Structure

The inner angular-rate loop is the hard real-time RTL loop. The outer attitude
loop consumes a common attitude-error vector produced by the estimator adapter.
That adapter uses wrapped Euler error for the RTL complementary filter or
quaternion error for the CPU EKF.

```
                 Outer Loop (Angle)           Inner Loop (Rate)
Setpoint ──→ [+]──→ [PID_angle] ──→ rate_setpoint ──→ [+]──→ [PID_rate] ──→ torque_cmd
              -↑                                        -↑
              │                                         │
         attitude_angle                            gyro_rate
        (from estimator)                        (from IMU filter)
```

### 3.6 Q16.16 Fixed-Point Representation

All values use signed Q16.16 (32-bit):
- Range: -32768.0 to +32767.99998 (approximately)
- Resolution: 1/65536 ≈ 0.0000153
- Multiplication: `(A × B) >> 16` to maintain format (uses DSP48E1 in 32×32 mode, taking middle 32 bits)
- `dt` in Q16.16 = `0.002 × 65536 = 131` (0x00000083)

---

## 4. Module Interface

### 4.1 Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `NUM_AXES` | 3 | Roll, Pitch, Yaw |
| `NUM_LOOPS` | 2 | Outer (angle), Inner (rate) |
| `Q_FORMAT` | Q16.16 | Signed fixed-point |
| `CLK_FREQ` | 100 MHz | System clock |
| `CTRL_RATE` | 500 Hz | Control loop update rate |

### 4.2 Input Signals

| Signal | Width | Format | Source | Description |
|--------|-------|--------|--------|-------------|
| `clk` | 1 | — | System | 100 MHz system clock |
| `rst_n` | 1 | — | System | Active-low synchronous reset |
| `ctrl_update` | 1 | Pulse | Timing Gen | 500 Hz strobe: start new PID computation |
| `attitude_error[2:0]` | 3×32 | Q16.16 | Estimator adapter | Selected estimator's shortest-path attitude error |
| `attitude_valid` | 1 | — | Estimator mux | Selected estimate is healthy and fresh |
| `rate_setpoint_direct[2:0]` | 3×32 | Q16.16 | RC command mapper | Rate-mode fallback command |
| `rate_measurement[2:0]` | 3×32 | Q16.16 | IMU Filter | Filtered gyroscope angular rates (rad/s) |
| `gains_kp_outer[2:0]` | 3×32 | Q16.16 | RTL defaults/optional AXI | Proportional gain, outer loop |
| `gains_ki_outer[2:0]` | 3×32 | Q16.16 | RTL defaults/optional AXI | Integral gain, outer loop |
| `gains_kd_outer[2:0]` | 3×32 | Q16.16 | RTL defaults/optional AXI | Derivative gain, outer loop |
| `gains_kp_inner[2:0]` | 3×32 | Q16.16 | RTL defaults/optional AXI | Proportional gain, inner loop |
| `gains_ki_inner[2:0]` | 3×32 | Q16.16 | RTL defaults/optional AXI | Integral gain, inner loop |
| `gains_kd_inner[2:0]` | 3×32 | Q16.16 | RTL defaults/optional AXI | Derivative gain, inner loop |
| `output_limit` | 32 | Q16.16 | RTL default/optional AXI | Maximum output magnitude |
| `integral_limit` | 32 | Q16.16 | RTL default/optional AXI | Maximum integrator magnitude |
| `armed` | 1 | — | Control | When deasserted, all outputs = 0, integrals cleared |

### 4.3 Output Signals

| Signal | Width | Format | Destination | Description |
|--------|-------|--------|-------------|-------------|
| `torque_cmd[2:0]` | 3×32 | Q16.16 | Motor Mixer | Roll, pitch, yaw torque commands |
| `pid_done` | 1 | Pulse | Timing Gen | Asserted when all 6 PID instances complete |
| `saturated[2:0]` | 3 | — | Telemetry | Per-axis output saturation flag |

If `attitude_valid` is false, the outer loop is disabled and its integrators are
cleared. The inner rate loop remains available for a configured rate-mode
fallback. Hard failsafe logic may still disarm for independent critical faults.

First-flight builds use conservative synthesized gain and limit parameters.
Optional AXI registers may override them later for tuning, but reset values must
produce a bounded, usable controller without CPU initialization.

---

## 5. Algorithm

### 5.1 Top-Level Sequencing (Pseudocode)

```
ON ctrl_update pulse:
    if NOT armed:
        clear all integrators
        set all torque_cmd to 0
        assert pid_done
        return

    // --- Outer Loop (Selected attitude estimator → Rate Setpoint) ---
    if NOT attitude_valid:
        clear outer integrators
        rate_setpoint = rate_setpoint_direct
        skip outer loop
    ELSE:
      error_outer = attitude_error
      FOR axis IN {roll, pitch, yaw}:
        // Proportional
        P_outer = gains_kp_outer[axis] * error_outer  // Q16.16 multiply

        // Damping from measured body rate; avoids quaternion setpoint kick
        D_outer = -gains_kd_outer[axis] * rate_measurement[axis]

        // Integral with anti-windup
        tentative_I_outer = integral_outer[axis] + gains_ki_outer[axis] * error_outer * dt
        tentative_output = P_outer + tentative_I_outer + D_outer
        IF |tentative_output| < output_limit OR sign(error_outer) != sign(tentative_output):
            integral_outer[axis] = clamp(tentative_I_outer, -integral_limit, +integral_limit)
        // else: freeze integrator

        // Sum and clamp
        output_outer = P_outer + integral_outer[axis] + D_outer
        rate_setpoint[axis] = clamp(output_outer, -output_limit, +output_limit)

    // --- Inner Loop (Rate → Torque Command) ---
    FOR axis IN {roll, pitch, yaw}:
        error_inner = rate_setpoint[axis] - rate_measurement[axis]

        // Proportional
        P_inner = gains_kp_inner[axis] * error_inner

        // Integral with anti-windup
        tentative_I_inner = integral_inner[axis] + gains_ki_inner[axis] * error_inner * dt
        tentative_output = P_inner + tentative_I_inner + D_inner
        IF |tentative_output| < output_limit OR sign(error_inner) != sign(tentative_output):
            integral_inner[axis] = clamp(tentative_I_inner, -integral_limit, +integral_limit)

        // Derivative on measurement
        D_inner = gains_kd_inner[axis] * (prev_rate[axis] - rate_measurement[axis]) / dt
        prev_rate[axis] = rate_measurement[axis]

        // Sum and clamp
        output_inner = P_inner + integral_inner[axis] + D_inner
        torque_cmd[axis] = clamp(output_inner, -output_limit, +output_limit)
        saturated[axis] = (|output_inner| >= output_limit)

    ASSERT pid_done
```

### 5.2 Time-Multiplexed Execution Schedule

```
Clock  | Operation              | DSP Usage
-------|------------------------|----------
  1-2  | Roll outer: multiply Kp × error    | DSP A
  3-4  | Roll outer: multiply Ki × error    | DSP A
  5-6  | Roll outer: multiply Kd × Δmeas   | DSP A
  7    | Roll outer: accumulate P+I+D      | Adder
  8    | Roll outer: clamp output          | Comparator
 9-16  | Pitch outer: same sequence        | DSP A
17-24  | Yaw outer: same sequence          | DSP A
25-48  | Inner loops (roll, pitch, yaw)    | DSP A
 49    | Assert pid_done                   | —

Total: ~50 clocks @ 100 MHz = 500 ns (well within 2 ms budget)
```

---

## 6. Resource Estimate

| Resource | Count | Notes |
|----------|-------|-------|
| DSP48E1 | 4 | 2 for 32×32 multiply (Kp/Ki/Kd × error), 2 for accumulate; time-muxed across 6 instances |
| LUTs | ~300 | Mux logic, clamping comparators, control FSM |
| Flip-Flops | ~400 | 6 integrator states (32b each), 6 prev_measurement (32b each), pipeline regs |
| BRAM | 0 | All state fits in FFs (6×2×32 = 384 bits for integrators + 384 bits for prev) |
| Fmax | >150 MHz | Comfortable at 100 MHz system clock |
| Latency | ~50 clocks | 500 ns from ctrl_update to pid_done |
| Throughput | 500 Hz | Matches control loop rate |

### Artix-7 35T Utilization (XC7A35T)

| Resource | Available | Used | Utilization |
|----------|-----------|------|-------------|
| DSP48E1 | 90 | 4 | 4.4% |
| LUTs | 20,800 | 300 | 1.4% |
| Flip-Flops | 41,600 | 400 | 1.0% |
