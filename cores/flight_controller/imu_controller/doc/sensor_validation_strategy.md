# Sensor Measurement Validation Strategy

How to detect **out-of-range**, **physically impossible**, and **cross-sensor
inconsistent** measurements from the ISM330DHCX (accel/gyro) + MMC5983MA
(mag), written for an FPGA flight controller. Algorithm/interface level only —
no RTL.

> All thresholds below are **starting points**. Tune them against logged data;
> aggressive flight legitimately produces large accel and rate values, so the
> art is separating "legal but extreme" from "corrupt".

---

## 1. Principles

1. **Validate calibrated, body-frame data**, after axis remap and offset/gain
   correction. Only the saturation/transport checks run on raw values.
2. **Cheap checks first**: squared-magnitude range tests need no `sqrt`; do them
   every sample. Expensive model checks (EKF residuals) run at a lower rate.
3. **Never act on a single sample.** Use counters/hysteresis so one glitch does
   not trigger a fault (see §7).
4. **Prefer graceful degradation over hard failure.** A disturbed magnetometer
   should drop the mag, not kill the controller.
5. **Cross-sensor checks need a reference.** At startup, *learn* local gravity
   magnitude, mag field magnitude, and dip angle; validate later samples
   against those learned baselines.
6. **No CRC exists on either bus** (verified), so plausibility and redundancy
   are your integrity mechanism. See `imu_programming_guide.md` §6.

---

## 2. Pipeline placement

```
 raw burst ──► Tier 0 (transport/raw)
                 │
                 ▼
             axis remap + calibration (offsets/gain, hard/soft iron)
                 │
                 ▼
             Tier 1 (absolute range)          ── per sample, cheap
                 │
                 ▼
             Tier 2 (temporal consistency)    ── per sample, cheap
                 │
                 ▼
             Tier 3 (cross-sensor)            ── per sample / decimated
                 │
                 ▼
             Tier 4 (estimator innovation)    ── filter rate
                 │
                 ▼
         quality flag + fused state ──► control / failsafe
```

Each stage produces a per-sample `valid` and a per-sensor health state
(`OK / SUSPECT / FAIL`).

---

## 3. Tier 0 — transport and raw sanity

Run on the raw register bytes before calibration.

| Check | Condition | Meaning |
|---|---|---|
| ID/match | `WHO_AM_I == 0x6B`, `PROD_ID == 0x30` | comms / wrong device |
| Saturation | raw16 == `0x7FFF` or `0x8000` (accel/gyro) | ADC/sensor railed |
| All-zero / all-ones | 6 output bytes are `0x00` or `0xFF` | bus stuck / open |
| Frozen frame | identical burst N times (N ≈ 64–256) | device or bus frozen |
| Read-back | write→read mismatch on config regs | config corruption (ISM) |

Saturation is not always "invalid" — a railed gyro during a hard tumble is
real but means the value is clipped, so mark `CLIPPED` rather than discard.

---

## 4. Tier 1 — absolute physical range (per sample)

Use **squared magnitudes** to avoid `sqrt` in the hot path.

Let `a = (ax, ay, az)`, `w = (wx, wy, wz)`, `m = (mx, my, mz)` in physical units.

### Accelerometer
```
|a|^2 = ax^2 + ay^2 + az^2
HARD:   0.05g   <= |a| <= 20g          // beyond this = corrupt
STATIC: 0.70g   <= |a| <= 1.30g        // "quiet" window, used for gating
```
- `|a| > 20g` on a small multirotor is almost certainly corrupt.
- `|a| < 0.05g` sustained is free-fall (real!) — flag as event, not fault.
- Saturation: any axis at raw rail.

### Gyroscope
```
|w|^2 <= (FS_dps)^2                    // must be within configured FS
|w| <= W_MAX (e.g. 2000 dps)           // plausible body rate
```
- Saturation matters most here because integrating a clipped rate corrupts
  attitude. Track a `clipped` flag and freeze/hold attitude integration while
  clipped.

### Magnetometer (after hard/soft-iron correction)
```
|B| learnt baseline  B_ref
ACCEPT: (1 - tol_b) * B_ref <= |B| <= (1 + tol_b) * B_ref   // tol_b ~ 0.15
ABSOLUTE: 0.20 G <= |B| <= 0.70 G      // global Earth field envelope
```

### Temperature
```
-40 C <= T <= +85 C                    // sensor operating range
```

---

## 5. Tier 2 — temporal consistency (single sensor)

Per-axis, compare against previous sample(s).

### 5.1 Slew / gradient limit
```
| x[n] - x[n-1] | <= SL * dt
```
- Gyro: `SL` = max believable angular acceleration (e.g. 20 000 dps/s).
- Accel: `SL` = max believable jerk (e.g. 200 g/s).
- Mag: `SL` sized for the vehicle's fastest yaw.

### 5.2 Spike rejection (median)
Cheap in hardware: keep the last 3 samples, take the **median** (sort 3
comparators per axis). If the newest deviates from the median by more than
`SP`, replace it with the median and flag a spike.
```
if |x[n] - median(x[n], x[n-1], x[n-2])| > SP: spike++, x[n] = median
```

### 5.3 Stuck / constant detection
```
if accel AND gyro bursts are byte-identical for N consecutive reads:
    mark STUCK (comms returning a frozen buffer)
```
Do **not** flag a legitimately-zero gyro as stuck using this alone — require
*all* axes/registers frozen simultaneously.

### 5.4 Noise / variance floor
Over a window (e.g. 64 samples), the per-axis variance should be non-zero and
below a ceiling. Zero variance while moving = stuck; huge variance = noise or
loose wiring.

---

## 6. Tier 3 — cross-sensor consistency

This is the strongest detector because it uses physics, not just bounds.

### 6.1 Gravity magnitude and accel disturbance detect
```
e_g = | |a| - g_learned |              // g_learned ~ 1g
if e_g > 0.15g:  accel is being accelerated -> reduce/disable accel correction
else:            accel trustworthy for leveling
```
This is the classic complementary-filter gain scheduler.

### 6.2 Gyro vs accel rotational consistency
From the current attitude estimate `R`, predict gravity direction:
```
g_pred = R * [0,0,1]^T                 // unit vector, body frame
a_hat  = a / |a|
d_ga   = | a_hat - g_pred |            // 0..2 ; ~sin(angle)
if |a| is near g and d_ga > D_GA (~0.15 .. 0.25): accel disagrees with gyro
```
A sudden, sustained disagreement means the accel is disturbed (vibration,
linear acceleration) or the gyro is drifting/saturating.

### 6.3 Magnetometer vs accel — dip-angle consistency
The angle between the magnetic field and gravity is **constant at a location**
(the magnetic dip). Learn it at calibration:
```
dip_ref = angle(a_ref, m_ref)          // computed once, stored
cos_ref = (a_ref . m_ref)/(|a_ref||m_ref|)
```
Per sample, compare:
```
cos_now = (a . m)/(|a||m|)
if |cos_now - cos_ref| > D_DIP (~0.1):  mag disturbed or accel disturbed
```
Avoid `acos`: compare `cos` directly, or compare squared dot products against
precomputed `cos^2` bounds.

### 6.4 Magnetometer field magnitude
Covered in Tier 1; here additionally track the magnitude over a window. A
*slow* deviation indicates magnetic interference ramping with throttle/current
(e.g. motors). A *fast* deviation means a spike.

### 6.5 Yaw rate cross-check
Compare magnetic heading rate with gyro Z:
```
psi_meas = atan2(...)  derived from mag (or its rate)
compare d(psi_meas)/dt to gyro_z (body frame; account for tilt)
```
Use a loose threshold — mag heading is noisy; rely on longer averages.

### 6.6 Sensor-fusion residual
If you run a complementary/Mahony/Madgwick filter, its correction term *is* a
consistency signal:
```
res_acc = filtered_accel_error
res_mag = filtered_mag_error
```
Gate: when `res_acc` is large, drop the accel gain; when `res_mag` is large,
drop the mag gain. This avoids a separate detector.

---

## 7. Debounce, hysteresis, and fault states

Single-sample counters are not enough; use independent up/down counters.

```
range_fail_cnt: ++ on range failure, -- on pass, clamp [0, N1]
slew_fail_cnt:  ++ on slew/spike failure, -- on pass, clamp [0, N2]
cross_fail_cnt: ++ on cross-sensor failure, -- on pass, clamp [0, N3]

state = OK
if range_fail_cnt >= N1 or cross_fail_cnt >= N3: state = FAIL
else if any cnt > 0:                             state = SUSPECT
// hysteresis: require M consecutive clean samples before OK again
```

Recommended starting counters (at ~1 kHz sample rate):

| Level | Enter FAIL | Back to OK |
|---|---|---|
| Range | 10 consecutive | 50 clean |
| Slew/spike | 5 consecutive | 20 clean |
| Cross-sensor | 50 consecutive | 200 clean |
| Stuck | 100 identical | comms reset |

Rationale: a real disturbance (motor startup, hard landing) lasts 10s–100s
of ms, so the counters should be long enough to ignore a single glitch but
short enough to react within a control cycle or two.

---

## 8. Fault classification and response

| Fault | Classification | Response |
|---|---|---|
| Single spike | transient | replace with median, do not fuse raw |
| Accel disturbed (`e_g` large) | expected in maneuver | trust gyro, disable accel leveling |
| Accel FAIL (sustained range/cross) | sensor fault | leveling off; use gyro+mag |
| Gyro clipped | saturation | hold attitude integration, flag |
| Gyro FAIL | sensor fault | switch to accel+mag (no rate damping) |
| Mag magnitude/dip FAIL | magnetic disturbance | drop mag; gyro+accel heading drifts |
| Both accel & mag FAIL | degraded | gyro-only; yaw unobservable → failsafe |
| Comms ID/frozen FAIL | bus/device fault | comms reset; if persists → failsafe |

The controller should treat "which sensors are trustworthy" as a **fusion
weight** (0…1) rather than a binary, so degradation is smooth:
```
w_accel = clamp(k * (1 - e_g/0.15g)) * health[accel]
w_mag   = clamp(k * (1 - e_mag/tol)) * health[mag]
```

---

## 9. Fixed-point implementation notes (FPGA)

- Keep one unit system, e.g. accel in **mg/1000 (LSB)**, gyro in **0.01 dps**,
  mag in **0.01 G**. Convert thresholds once at build time.
- **Squared magnitudes** everywhere; compare `x^2+y^2+z^2` against squared
  bounds. Widths: accel LSB at ±8g ≈ 4096/g; three squares ≤ ~3·(32768)^2 ≈
  `2^31.6`, so use a 48-bit accumulator during the sum.
- For cos/dip checks avoid division: compare
  `(a·m)^2` vs `cos^2_bound · |a|^2 · |m|^2` using wide (64-bit) accumulate.
- `sqrt` is only needed to produce physical magnitudes for telemetry; the
  comparisons themselves stay in the squared domain.
- Median-of-3: two comparators per axis (min/max), middle = `a+b+c-min-max`.
- Run Tier 1/2 every sample; run Tier 3/4 at the fusion rate (e.g. 1/4 or 1/8
  of the fast sample rate) to save logic.
- All integer; no floating point in the hot path.

---

## 10. Threshold cheat-sheet (starting values)

| Quantity | Symbol | Start | Notes |
|---|---|---|---|
| Accel hard window | — | 0.05–20 g | beyond = corrupt |
| Accel quiet window | — | 0.70–1.30 g | "not accelerating" |
| Accel disturbance | `e_g` | 0.15 g | gain-schedule leveling |
| Gyro max rate | `W_MAX` | 2000 dps | tune to airframe |
| Gyro angular accel | `SL_g` | 20 000 dps/s | slew limit |
| Mag field envelope | — | 0.20–0.70 G | global Earth field |
| Mag field tolerance | `tol_b` | ±15 % | vs learned `B_ref` |
| Dip-angle tolerance | `D_DIP` | Δcos 0.10 | ~6° |
| Gyro/accel angle | `D_GA` | 0.15–0.25 | ~9–14° |
| Accel jerk limit | `SL_a` | 200 g/s | slew limit |
| Spike threshold | `SP` | 3–5 σ of noise | per axis |

Earth field reference: total intensity ≈ **22–67 µT (0.22–0.67 G)**, dip 0°
at the magnetic equator to ±90° at the poles.

---

## 11. Tuning procedure

1. Log raw + calibrated data plus all residual signals during:
   bench rest, hand-tumble, motor-spin (props off), hover, and aggressive
   maneuvers.
2. For each signal, measure the **noise σ** and the genuine operating range.
3. Set range limits just outside the observed legal range; set disturbance
   thresholds at ~3–5 σ of the clean signal.
4. Set counter lengths so that a real disturbance trips FAIL but a single
   dropout does not.
5. Replay the logs through the validator offline and confirm zero false
   positives in clean flight and zero false negatives during induced faults.

---

## 12. Pseudocode

```
// per calibrated body-frame sample
validate(a, w, m, dt):
    // Tier 1
    a2 = dot(a,a)
    if a2 > (20g)^2 or a2 < (0.05g)^2:   range_fail(accel)
    if dot(w,w) > FS_g^2:                clip_flag(gyro); range_fail(gyro)
    if |B(m)| < 0.20G or > 0.70G:        range_fail(mag)

    // Tier 2
    if |a - a_prev| > SL_a*dt:           slew_fail(accel)
    if |w - w_prev| > SL_g*dt:           slew_fail(gyro)
    a = median3(a); w = median3(w); m = median3(m)   // spike replace

    // Tier 3
    e_g   = abs(|a| - g_ref)
    accel_trust = (e_g < 0.15g)
    d_ga  = |a/|a| - R*[0,0,1]|
    if accel_trust and d_ga > D_GA:      cross_fail(accel)
    cos_now = dot(a,m)/(|a||m|)
    if abs(cos_now - cos_ref) > D_DIP:   cross_fail(mag)

    // counters + hysteresis -> health[accel|gyro|mag]
    // fusion weights -> estimator
```

---

## 13. Checklist

- [ ] Validate *calibrated* data; only saturation/raw checks use raw bytes.
- [ ] Use squared magnitudes; no `sqrt` in comparisons.
- [ ] Learn `g_ref`, `B_ref`, `dip_ref` at calibration and store them.
- [ ] Debounce every check with counters + hysteresis.
- [ ] Expose per-sensor health and feed it as a fusion weight, not a hard kill.
- [ ] Handle free-fall, clipping, and genuine high-g as events, not faults.
- [ ] Watch for mag interference correlated with throttle (motors/ESC).
- [ ] Log residuals for offline tuning; replay logs through the validator.
- [ ] Remember: no CRC on these buses — plausibility *is* the integrity layer.
