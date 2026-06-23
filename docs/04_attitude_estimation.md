# Module 04: Attitude Estimation Alternatives

## 1. Supported Approaches

The design deliberately supports two attitude-estimation implementations:

| Approach | Location | Advantages | Limitations |
|---|---|---|---|
| Complementary filter | RTL | Deterministic, low latency, self-contained | Simpler error model; yaw needs an external reference |
| Extended Kalman filter | MicroBlaze/RV32 software | Bias estimation, gating, flexible multi-sensor fusion | More software and CPU cost; must meet mailbox deadline |

The **RTL complementary filter is a valid final implementation**, not merely a
temporary placeholder. The CPU EKF is an optional upgrade. Both leave the inner
angular-rate PID in RTL.

## 2. Common Estimator Contract

The controller consumes a selected estimator record:

| Field | Format | Description |
|---|---|---|
| roll, pitch, yaw | 3 x signed Q16.16 | Wrapped radians |
| quaternion | 4 x signed Q2.30 | Optional but preferred for EKF mode |
| source | enum | RTL complementary or CPU EKF |
| sequence | unsigned 32-bit | Monotonic estimator result number |
| timestamp | unsigned 64-bit | Source sensor timestamp |
| valid | 1 bit | Initialized, healthy, and fresh |
| health | bit field | Accel used, mag used, rejection/stale faults |

An estimator mux publishes only one source to the outer controller. Switching
sources while armed is disabled by default; if enabled later, it requires
agreement checks and a bumpless transfer.

## 3. Option A: RTL Complementary Filter

### 3.1 Equations

The accelerometer gives a long-term roll/pitch reference:

```text
roll_acc  = atan2(a_y, a_z)
pitch_acc = atan2(-a_x, sqrt(a_y^2 + a_z^2))
```

The gyro propagates the previous estimate, then the complementary filter applies
a low-frequency accelerometer correction:

```text
roll  = alpha * (roll_prev  + gyro_x * dt) + (1-alpha) * roll_acc
pitch = alpha * (pitch_prev + gyro_y * dt) + (1-alpha) * pitch_acc
yaw   = wrap(yaw_prev + gyro_z * dt)
```

For a first implementation, `alpha` is configurable over AXI4-Lite and computed
from a chosen correction time constant. `dt` is derived from the RTL sample
timestamp/tick rather than assumed implicitly.

These independent-axis equations are an approximation. They are acceptable for
initial moderate-angle flight; a quaternion complementary implementation is the
upgrade path for aggressive motion without moving estimation to software.

### 3.2 Acceleration gating

Accelerometer tilt is trustworthy only when measured acceleration is dominated
by gravity. RTL shall reduce or skip correction when

```text
abs(norm(accel) - 1 g) > accel_gate
```

The inexpensive implementation may compare squared magnitude against configured
bounds and avoid a square root. This prevents strong translational acceleration
from being interpreted immediately as tilt.

### 3.3 CORDIC

An iterative, time-multiplexed CORDIC block computes `atan2`; the same engine can
serve roll and pitch sequentially. Sixteen iterations are sufficient for the
Q16.16 angle interface. Quadrant correction and the `(0,0)` case must be handled
explicitly.

### 3.4 Yaw behavior

Gyro-only yaw drifts. Initial RTL mode may expose that limitation and support
rate mode without heading hold. Optional heading correction can use calibrated
AK8963 data, but tilt compensation, magnetic gating, and motor-current
interference handling must be implemented before claiming absolute heading.

### 3.5 RTL interface

Inputs are calibrated/filtered accel and gyro samples plus `imu_valid`, `dt`,
`alpha`, acceleration-gate thresholds, enable, and reset/reinitialize controls.
Outputs are the common estimator record and diagnostic flags. The filter becomes
valid only after stationary initialization and consecutive plausible samples.

## 4. Option B: CPU Extended Kalman Filter

A practical initial state is

```text
x = [q_w, q_x, q_y, q_z, b_gx, b_gy, b_gz]
```

The CPU predicts from gyro measurements and conditionally updates from gravity
and calibrated magnetic observations. A later vertical extension may consume
BMP280 pressure. `dt` always comes from the RTL timestamp.

```text
on sensor interrupt:
    read coherent AXI snapshot
    verify sequence, timestamp, validity, and sample age
    predict quaternion and covariance from gyro
    conditionally update from accelerometer
    conditionally update from fresh, calibrated AK8963 data
    normalize quaternion and run covariance/innovation checks
    write inactive estimator mailbox, metadata, and health
    commit mailbox atomically
```

The mailbox retains the last complete result until commit. RTL rejects stale,
non-monotonic, unhealthy, or badly normalized results. If CPU execution stalls,
the RTL rate loop continues and attitude-dependent modes are inhibited.

## 5. Selection Guidance

Start with the RTL complementary filter when the goal is the smallest
self-contained flight controller and straightforward bring-up. Select the CPU
EKF when logs show that bias estimation, magnetic gating, covariance-based
measurement rejection, or navigation-state fusion materially improves the
vehicle.

Keep both implementations behind the common estimator contract. This makes the
decision a configuration/build choice rather than a redesign of PID, mixer,
motor output, or failsafe modules.

## 6. Verification

Replay the same timestamped GY-91 logs through both estimators and compare static
level error, dynamic lag, yaw drift, recovery after acceleration, invalid-sample
handling, and CPU-stall behavior. Complementary mode needs CORDIC and fixed-point
golden-vector tests; EKF mode needs quaternion norm, covariance, innovation, and
mailbox deadline tests.
