# Module 11: Failsafe and Arming Logic

## 1. Why This Module is Needed

The failsafe module is the most safety-critical component in the flight controller. It prevents catastrophic failures that can result in:

- **Flyaway prevention**: If the RC transmitter loses connection, an uncontrolled drone will fly until the battery dies, potentially traveling kilometers and striking people, vehicles, or structures.
- **Crash mitigation on signal loss**: Without failsafe, a drone that loses RC signal will maintain its last throttle command indefinitely, climbing uncontrollably or maintaining flight with no way to land.
- **Protection of people and property**: A quadcopter spinning at 10,000+ RPM with carbon fiber propellers can cause serious injury. Motors must be immediately disabled when unsafe conditions are detected.
- **Regulatory compliance**: Aviation regulations (FAA Part 107, EU drone regulations) require failsafe mechanisms that bring the drone down safely on link loss.
- **IMU failure handling**: If the attitude estimation system fails (no gyroscope data), the PID controller will output garbage, causing violent oscillations. The motors must be cut immediately.
- **Accidental arming prevention**: Motors must never spin unintentionally. A strict set of pre-arm checks ensures the drone only arms when the pilot explicitly commands it with throttle at zero.

**Design philosophy**: The failsafe module is the ultimate authority over motor outputs. No other module can override a failsafe disarm. The system fails safe — any ambiguous condition results in motors off.

---

## 2. Sensor/Approach Options

| Approach | Description | Advantages | Disadvantages |
|----------|-------------|-----------|---------------|
| **Hardware watchdog (chosen)** | Dedicated RTL state machine with clock-cycle-accurate timers | Deterministic timing, cannot be bypassed by software bugs, zero latency | More complex RTL design, harder to update thresholds |
| Software-only failsafe | CPU checks timing in main loop | Easy to modify, flexible logic | Subject to software crashes, loop timing jitter, can be bypassed by bugs |
| External failsafe IC | Dedicated safety microcontroller monitors signals | Independent from main processor | Additional component cost, communication overhead, another failure point |
| RC receiver built-in failsafe | Receiver outputs pre-set values on signal loss | No FPGA logic needed | Only handles RC loss, not IMU failure or other conditions, receiver-dependent behavior |
| Dual-redundant with voting | Two independent systems must agree | Very high reliability | Doubles hardware cost, complex synchronization |

**Chosen approach: Hardware watchdog state machine in RTL**

Rationale:
- Runs in dedicated hardware, independent of any software — cannot crash or hang
- Sub-microsecond response time to fault conditions
- Deterministic timing guaranteed by synthesized logic
- All timeout thresholds configurable via AXI registers (updatable without resynthesis)
- Motor gating is the final output stage — physically impossible for any other logic to bypass

---

## 3. Theoretical Foundations

### 3.1 Failure Mode Analysis

| Failure Mode | Detection Method | Response | Severity |
|--------------|-----------------|----------|----------|
| RC signal loss | No valid RC frame for N ms | Coast → Descend → Disarm | Critical |
| IMU data timeout | No imu_valid pulse for 50 ms | Immediate disarm | Critical |
| Low battery | Battery voltage below threshold | Warning → Land → Disarm | High |
| Motor failure | Current anomaly / no RPM feedback | Immediate disarm | Critical |
| GPS loss | No GPS fix for N seconds | Return to manual mode | Medium |
| Sensor saturation | IMU output at maximum | Warning flag | Low |

### 3.2 Arming Requirements

ALL of the following conditions must be simultaneously true to allow arming:

1. **Throttle at minimum** (< 5% of full range)
   - Prevents arming with throttle up, which would cause immediate liftoff
   - Threshold: throttle_channel < 0x0CCC in Q16.16 (≈ 5% of 65535)

2. **Arm switch active** (auxiliary RC channel above threshold)
   - Dedicated two-position switch on transmitter
   - Threshold: arm_channel > 0x7FFF (midpoint = armed)
   - Requires deliberate pilot action

3. **Gyroscope calibration complete**
   - cal_done signal from IMU module must be asserted
   - Ensures gyro bias has been measured and subtracted
   - Prevents flying with uncalibrated sensors

4. **No active failsafe conditions**
   - rc_valid must be asserted (receiver is receiving frames)
   - imu_valid must have pulsed within last 50 ms
   - No error flags active

5. **Battery voltage above minimum** (if battery monitoring enabled)
   - Prevents arming with nearly-dead battery that would die mid-flight
   - Threshold configurable via AXI register

### 3.3 Disarm Conditions

ANY single condition triggers immediate disarm:

1. **Arm switch deactivated**: arm_channel drops below threshold
2. **RC failsafe timeout**: rc_valid absent for > 500 ms (configurable)
3. **IMU data timeout**: imu_valid absent for > 50 ms (configurable)
4. **Explicit disarm command**: Software writes to disarm register
5. **Battery critical**: Voltage below emergency threshold (if enabled)

### 3.4 Failsafe Descent Behavior

When armed and RC signal is lost, a graduated response is used:

```
Time since RC loss:     Action:
─────────────────────────────────────────────────
0 – 500 ms             COAST: Hold last known commands
                       (brief glitches are normal, don't overreact)

500 ms – 3 s           DESCEND: Reduce throttle gradually
                       throttle = last_throttle × decay_factor
                       decay_factor decreases by 1/256 per 10 ms
                       Roll/pitch/yaw commands → 0 (level flight)

> 3 s                  DISARM: Cut all motors immediately
                       (drone will fall — but controlled descent
                       has already reduced altitude significantly)
```

### 3.5 Watchdog Timer Design

At 100 MHz system clock:
- 1 ms = 100,000 clock cycles (17-bit counter)
- 50 ms = 5,000,000 clock cycles (23-bit counter)
- 500 ms = 50,000,000 clock cycles (26-bit counter)
- 3 s = 300,000,000 clock cycles (29-bit counter)

**RC watchdog**: 26-bit counter, resets on every valid RC frame (rc_valid pulse).
**IMU watchdog**: 23-bit counter, resets on every imu_valid pulse.

When counter reaches threshold → corresponding timeout flag asserts.

### 3.6 Motor Output Gating

The motor gating is the absolute final stage before PWM/DShot outputs:

```
motor_output[i] = armed AND motor_enable ? pid_output[i] : 0
```

This is implemented as a simple AND gate on each motor channel. When `armed = 0` or `motor_enable = 0`, all motor outputs are forced to zero regardless of what the PID controller produces. This provides a hardware guarantee that cannot be violated by software or upstream logic errors.

### 3.7 State Transition Diagram

```
                    ┌─────────┐
         Reset ────→│DISARMED │←────── Arm switch off
                    └────┬────┘        OR RC timeout > 3s
                         │              OR IMU timeout
                         │ All arm conditions met
                         ▼
                    ┌─────────┐
                    │  ARMED  │←────── RC recovered during coast/descend
                    └────┬────┘
                         │ RC loss detected
                         ▼
                    ┌─────────────────┐
                    │ FAILSAFE_COAST  │  (hold last commands, 0–500ms)
                    └────┬────────────┘
                         │ 500 ms elapsed, still no RC
                         ▼
                    ┌─────────────────────┐
                    │ FAILSAFE_DESCEND    │  (reduce throttle, 500ms–3s)
                    └────┬────────────────┘
                         │ 3 s elapsed, still no RC
                         ▼
                    ┌─────────┐
                    │DISARMED │  (motors off)
                    └─────────┘
```

### 3.8 Pre-Arm vs In-Flight Checks

| Check | Pre-Arm Threshold | In-Flight Threshold | Rationale |
|-------|-------------------|---------------------|-----------|
| RC signal | Must be valid | 500 ms grace period | Brief glitches normal in flight |
| IMU data | Must be valid | 50 ms grace period | Short processing gaps acceptable |
| Battery | > 11.4 V (3S) | > 10.5 V (3S) | Lower threshold in flight to avoid premature land |
| Throttle | Must be < 5% | N/A (any value OK) | Only checked at arming time |
| Gyro cal | Must be done | N/A (already done) | One-time check at startup |

---

## 4. Module Interface

### 4.1 Input Signals

| Signal | Width | Format | Source | Description |
|--------|-------|--------|--------|-------------|
| clk | 1 | — | System | 100 MHz system clock |
| rst_n | 1 | — | System | Active-low synchronous reset |
| rc_valid | 1 | Pulse | RC Module | Pulses each time a valid RC frame is received (~50 Hz) |
| imu_valid | 1 | Pulse | IMU Module | Pulses each time valid IMU data is produced (~1 kHz) |
| arm_channel | 16 | Unsigned | RC Module | Arm switch channel value (0=disarm, 65535=arm) |
| throttle_channel | 16 | Unsigned | RC Module | Throttle stick value (0=min, 65535=max) |
| cal_done | 1 | Level | IMU Module | Gyroscope calibration complete flag |
| battery_voltage | 12 | Unsigned | ADC Module | Battery voltage (scaled, optional) |
| battery_valid | 1 | Level | ADC Module | Battery monitoring is active and reading valid |
| sw_disarm | 1 | Pulse | CPU/AXI | Software-initiated disarm command |
| cfg_rc_timeout_coast | 26 | Unsigned | AXI Config | RC loss coast duration in clock cycles (default: 50M = 500ms) |
| cfg_rc_timeout_descend | 29 | Unsigned | AXI Config | RC loss descend duration in clock cycles (default: 300M = 3s) |
| cfg_imu_timeout | 23 | Unsigned | AXI Config | IMU timeout threshold in clock cycles (default: 5M = 50ms) |
| cfg_throttle_min | 16 | Unsigned | AXI Config | Maximum throttle for arming (default: 0x0CCC ≈ 5%) |
| cfg_arm_threshold | 16 | Unsigned | AXI Config | Minimum arm channel value to consider armed (default: 0x7FFF) |
| cfg_battery_min_arm | 12 | Unsigned | AXI Config | Minimum battery voltage for arming |
| cfg_battery_min_fly | 12 | Unsigned | AXI Config | Minimum battery voltage in flight |

### 4.2 Output Signals

| Signal | Width | Format | Destination | Description |
|--------|-------|--------|-------------|-------------|
| armed | 1 | Level | All Modules | Drone is armed — motors may spin |
| motor_enable | 1 | Level | Motor Mixer | Motors allowed to produce output (AND with armed) |
| failsafe_active | 1 | Level | Status/LED | Any failsafe condition is active |
| failsafe_state | 3 | Encoded | CPU/Status | Current state: 0=DISARMED, 1=ARMED, 2=COAST, 3=DESCEND, 4=ERROR |
| error_flags | 8 | Bitfield | CPU/Status | Bit 0: RC timeout, Bit 1: IMU timeout, Bit 2: battery low, Bit 3: cal incomplete, Bit 4–7: reserved |
| throttle_override | 16 | Unsigned | Motor Mixer | Overridden throttle during failsafe descent (replaces pilot throttle) |
| throttle_override_valid | 1 | Level | Motor Mixer | When high, use throttle_override instead of RC throttle |
| rc_watchdog_count | 26 | Unsigned | Debug/Status | Current RC watchdog counter value (for diagnostics) |
| imu_watchdog_count | 23 | Unsigned | Debug/Status | Current IMU watchdog counter value (for diagnostics) |

---

## 5. Algorithm (Pseudocode)

### 5.1 Main State Machine

```
REGISTER: state (3-bit) = DISARMED
REGISTER: rc_watchdog (26-bit counter)
REGISTER: imu_watchdog (23-bit counter)
REGISTER: descend_timer (29-bit counter)
REGISTER: descend_throttle (16-bit)
REGISTER: last_throttle (16-bit)

STATE: DISARMED (state = 0)
  armed = 0
  motor_enable = 0
  failsafe_active = 0
  throttle_override_valid = 0
  
  // Check arming conditions
  If ALL of:
    - arm_channel > cfg_arm_threshold
    - throttle_channel < cfg_throttle_min
    - cal_done == 1
    - rc_watchdog < cfg_rc_timeout_coast (RC is actively received)
    - imu_watchdog < cfg_imu_timeout (IMU is actively producing data)
    - (battery_valid == 0) OR (battery_voltage > cfg_battery_min_arm)
    - error_flags == 0
  Then:
    → ARMED

STATE: ARMED (state = 1)
  armed = 1
  motor_enable = 1
  failsafe_active = 0
  throttle_override_valid = 0
  last_throttle = throttle_channel  // Save for coast/descend
  
  // Check disarm conditions
  If arm_channel < cfg_arm_threshold:
    → DISARMED
  If sw_disarm:
    → DISARMED
  If imu_watchdog >= cfg_imu_timeout:
    Set error_flags[1]
    → DISARMED  // Immediate disarm on IMU loss
  If rc_watchdog >= cfg_rc_timeout_coast:
    → FAILSAFE_COAST
  If battery_valid AND battery_voltage < cfg_battery_min_fly:
    Set error_flags[2]
    → FAILSAFE_DESCEND  // Low battery → start descending

STATE: FAILSAFE_COAST (state = 2)
  armed = 1
  motor_enable = 1
  failsafe_active = 1
  throttle_override = last_throttle  // Hold last known throttle
  throttle_override_valid = 1
  
  // RC recovered?
  If rc_valid pulse received (rc_watchdog resets to 0):
    → ARMED
  
  // IMU still lost?
  If imu_watchdog >= cfg_imu_timeout:
    → DISARMED  // Can't fly without IMU
  
  // Coast timeout expired?
  If rc_watchdog >= cfg_rc_timeout_descend:
    descend_throttle = last_throttle
    descend_timer = 0
    → FAILSAFE_DESCEND

STATE: FAILSAFE_DESCEND (state = 3)
  armed = 1
  motor_enable = 1
  failsafe_active = 1
  throttle_override_valid = 1
  
  // Gradually reduce throttle
  Every 1 ms (every 100,000 clock cycles):
    If descend_throttle > 256:
      descend_throttle = descend_throttle - (descend_throttle >> 8)
      // Exponential decay: lose ~0.4% per ms → ~33% per second
    Else:
      descend_throttle = 0
  
  throttle_override = descend_throttle
  
  // RC recovered?
  If rc_valid pulse received:
    → ARMED
  
  // IMU lost?
  If imu_watchdog >= cfg_imu_timeout:
    → DISARMED
  
  // Total RC loss exceeded maximum?
  If rc_watchdog >= cfg_rc_timeout_descend:
    → DISARMED  // Final cutoff — motors off
  
  // Throttle decayed to zero?
  If descend_throttle == 0:
    → DISARMED
```

### 5.2 Watchdog Timers

```
// RC Watchdog — counts clock cycles since last valid RC frame
On every clock rising edge:
  If rc_valid == 1:
    rc_watchdog = 0
  Else if rc_watchdog < MAX_26BIT:
    rc_watchdog = rc_watchdog + 1
  // Saturates at max value (never wraps)

// IMU Watchdog — counts clock cycles since last valid IMU sample
On every clock rising edge:
  If imu_valid == 1:
    imu_watchdog = 0
  Else if imu_watchdog < MAX_23BIT:
    imu_watchdog = imu_watchdog + 1
```

### 5.3 Error Flag Generation

```
// Error flags are set by conditions, cleared by reset or software clear
On every clock rising edge:
  error_flags[0] = (rc_watchdog >= cfg_rc_timeout_coast)   // RC timeout
  error_flags[1] = (imu_watchdog >= cfg_imu_timeout)       // IMU timeout
  error_flags[2] = battery_valid AND (battery_voltage < cfg_battery_min_fly)  // Battery low
  error_flags[3] = NOT cal_done                             // Calibration incomplete
  // Bits 4-7 reserved for future use
```

### 5.4 Motor Output Gating Logic

```
// This is the absolute final gate before motor outputs
// Implemented as simple combinational logic

For each motor output channel (0 to 3):
  If armed == 1 AND motor_enable == 1:
    motor_out[i] = pid_motor_out[i]
  Else:
    motor_out[i] = 0

// Note: motor_out goes to the PWM/DShot generator
// When motor_out = 0, PWM outputs minimum (1000 µs) or DShot outputs 0
// This ensures motors are physically stopped
```

### 5.5 Arming Transition Debounce

```
// Prevent spurious arming from noisy signals
REGISTER: arm_hold_counter (16-bit)
CONSTANT: ARM_HOLD_THRESHOLD = 50000  // 500 µs at 100 MHz (debounce time)

// Arm switch must be held in "arm" position for ARM_HOLD_THRESHOLD
// consecutive clock cycles before arming is permitted

On every clock rising edge:
  If arm_channel > cfg_arm_threshold:
    If arm_hold_counter < ARM_HOLD_THRESHOLD:
      arm_hold_counter = arm_hold_counter + 1
  Else:
    arm_hold_counter = 0
  
  arm_switch_valid = (arm_hold_counter >= ARM_HOLD_THRESHOLD)
```

---

## 6. Resource Estimate

| Resource | Count | Notes |
|----------|-------|-------|
| LUTs | ~150 | State machine, comparators, watchdog logic, gating |
| Flip-Flops | ~100 | State register, watchdog counters (26+23+29=78 bits), error flags, config |
| DSP48E1 | 0 | No multiplications — all comparisons and counters |
| BRAM (36Kb) | 0 | No memory needed — all logic is combinational + registers |
| I/O Pins | 0 | All signals are internal (module-to-module connections) |
| Clock | 1 | 100 MHz system clock only |

**Breakdown:**
- State machine (5 states, transitions): ~30 LUTs, ~5 FFs
- RC watchdog counter (26-bit) + comparator: ~15 LUTs, ~26 FFs
- IMU watchdog counter (23-bit) + comparator: ~12 LUTs, ~23 FFs
- Descend timer + throttle decay: ~25 LUTs, ~20 FFs
- Arming condition logic (5 comparators): ~30 LUTs, ~5 FFs
- Error flag generation: ~10 LUTs, ~8 FFs
- Motor gating (4 channels): ~8 LUTs, ~0 FFs (combinational)
- Configuration register interface: ~10 LUTs, ~8 FFs
- Debounce counter: ~10 LUTs, ~16 FFs

**Critical path note**: The arming logic has multiple comparators that must resolve within one clock cycle. At 100 MHz (10 ns period), 16-bit comparisons easily fit within timing. No special pipelining is required for this module.
