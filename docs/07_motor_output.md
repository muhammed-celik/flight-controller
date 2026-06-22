# Module 07: Motor Output (DSHOT600 + PWM)

## 1. Why This Module is Needed

The motor output module is the final interface between the digital flight controller and the physical ESCs (Electronic Speed Controllers) that drive the brushless motors. It converts the 11-bit motor command values (0–2047) into electrical waveforms that the ESCs can interpret.

**Why DSHOT600 over analog PWM:**

| Aspect | Traditional PWM | DSHOT600 |
|--------|----------------|----------|
| Resolution | ~1000 steps (1000–2000 µs) | 2048 steps (11-bit) |
| Update rate | Limited by pulse period (2.5 ms @ 400 Hz) | Frame time ~26.7 µs, allows >30 kHz |
| Jitter sensitivity | High — ESC interprets timing | None — digital protocol, bit values only |
| Error detection | None | 4-bit CRC per frame |
| Bidirectional telemetry | Not possible | Supported (motor RPM, temperature, voltage) |
| Wiring | Same (single signal wire) | Same (single signal wire) |
| ESC compatibility | Universal (all ESCs) | Modern ESCs (BLHeli_S, BLHeli_32, AM32) |

DSHOT600 is selected as primary protocol for its noise immunity, precision, and CRC protection — critical for a safety-critical flight controller. PWM fallback is included for compatibility with legacy ESCs.

---

## 2. Options/Approaches

| Approach | Pros | Cons | Selected |
|----------|------|------|----------|
| DSHOT600 only | Simplest, best performance | No legacy ESC support | No |
| DSHOT600 + PWM fallback | Best of both worlds | Slightly more logic | **Yes** |
| DSHOT300 (slower) | More timing margin | Lower update rate, no real advantage | No |
| DSHOT1200 (faster) | Faster updates | Tight timing at 100 MHz (83 clks/bit), less ESC support | No |
| OneShot125/OneShot42 | Faster than PWM | Still analog timing-based, no CRC | No |
| Multishot | Very fast | Extremely tight timing, rare ESC support | No |

### DSHOT Variant Comparison

| Protocol | Bit Rate | Bit Period (clks @100MHz) | T1H (clks) | T0H (clks) | Frame Time |
|----------|----------|--------------------------|-------------|-------------|------------|
| DSHOT150 | 150 kbit/s | 667 | 500 | 250 | 106.7 µs |
| DSHOT300 | 300 kbit/s | 333 | 250 | 125 | 53.3 µs |
| **DSHOT600** | **600 kbit/s** | **167** | **125** | **63** | **26.7 µs** |
| DSHOT1200 | 1200 kbit/s | 83 | 63 | 31 | 13.3 µs |

---

## 3. Theoretical Foundations

### 3.1 DSHOT600 Frame Structure

Each DSHOT frame is 16 bits transmitted MSB-first:

```
Bit Position:  [15] [14] [13] [12] [11] [10] [9] [8] [7] [6] [5] [4] [3] [2] [1] [0]
               |<--------- Throttle (11 bits) -------->| T  |<---- CRC (4 bits) ---->|
```

| Field | Bits | Range | Description |
|-------|------|-------|-------------|
| Throttle | [15:5] | 0–2047 | Motor command (0=disarmed, 48=idle, 2047=full) |
| Telemetry | [4] | 0–1 | Request ESC telemetry on next frame |
| CRC | [3:0] | 0–15 | Error detection checksum |

### 3.2 CRC Calculation

The CRC is computed as XOR of three 4-bit nibbles of the 12-bit payload (throttle + telemetry bit):

```
payload[11:0] = {throttle[10:0], telemetry_request}

CRC = payload[11:8] XOR payload[7:4] XOR payload[3:0]
```

Example: throttle = 1500 (0x5DC), telemetry = 0
```
payload = 0xBB8  (1500 << 1 | 0 = 3000 = 0xBB8)
CRC = 0xB XOR 0xB XOR 0x8 = 0x8
Frame = {0x5DC, 0, 0x8} = 0xBB88 (transmitted MSB-first)
```

### 3.3 Bit Encoding (Pulse Width Modulation of Each Bit)

Each bit occupies a fixed period. The duty cycle encodes the value:

```
Bit period = 1/600000 = 1.667 µs = 167 clock cycles @ 100 MHz

Logic "1": High for 74.85% of period → T1H = 125 clocks, T1L = 42 clocks
Logic "0": High for 37.43% of period → T0H = 63 clocks,  T0L = 104 clocks
```

Waveform for one bit:
```
         T_HIGH          T_LOW
    ┌──────────────┐
    │              │
────┘              └──────────── (next bit starts)
    |<--- bit period = 167 clks --->|
```

### 3.4 Frame Timing

```
Total frame = 16 bits × 167 clocks = 2672 clocks = 26.72 µs
Inter-frame gap ≥ 2 µs (≥ 200 clocks idle low)

Maximum update rate = 1 / (26.72 + 2) µs ≈ 34.8 kHz
At 500 Hz control loop: plenty of margin (2 ms between frames)
```

### 3.5 PWM Fallback Mode

Standard RC PWM for legacy ESCs:

```
Pulse width: 1000 µs (min/off) to 2000 µs (max throttle)
Period: 2500 µs (400 Hz update rate)
Resolution: 1 µs = 100 clocks @ 100 MHz → 1000 steps

Mapping from 11-bit DSHOT value to PWM:
    pulse_width_us = 1000 + (motor_value × 1000) / 2047
    pulse_width_clks = pulse_width_us × 100
```

### 3.6 Special DSHOT Commands (Throttle Values 0–47)

Values 0–47 are reserved as special commands (not throttle):

| Value | Command |
|-------|---------|
| 0 | Disarm (motor stop) |
| 1–5 | Beep patterns |
| 6 | ESC info request |
| 7–11 | Rotation direction |
| 12–47 | Reserved/extended telemetry |
| 48–2047 | Normal throttle range |

---

## 4. Module Interface

### 4.1 Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `NUM_MOTORS` | 4 | Number of output channels |
| `CLK_FREQ` | 100 MHz | System clock frequency |
| `DSHOT_RATE` | 600000 | DSHOT600 bit rate |
| `BIT_PERIOD` | 167 | Clocks per DSHOT bit |
| `T1H_CLKS` | 125 | High-time for logic 1 |
| `T0H_CLKS` | 63 | High-time for logic 0 |
| `PWM_PERIOD` | 250000 | PWM period in clocks (2.5 ms @ 100 MHz) |

### 4.2 Input Signals

| Signal | Width | Format | Source | Description |
|--------|-------|--------|--------|-------------|
| `clk` | 1 | — | System | 100 MHz system clock |
| `rst_n` | 1 | — | System | Active-low synchronous reset |
| `motor_value[3:0]` | 4×11 | Unsigned | Motor Mixer | Throttle values (0–2047) per motor |
| `telemetry_req[3:0]` | 4 | — | Control | Per-motor telemetry request bit |
| `send_frame` | 1 | Pulse | Timing Gen | Trigger: begin transmitting new frame to all 4 ESCs |
| `output_mode` | 1 | — | AXI Regs | 0 = DSHOT600, 1 = PWM |
| `armed` | 1 | — | Control | When deasserted, output value 0 (disarm command) |

### 4.3 Output Signals

| Signal | Width | Format | Destination | Description |
|--------|-------|--------|-------------|-------------|
| `dshot_out[3:0]` | 4 | Digital | ESC pins | DSHOT/PWM signal to each ESC |
| `frame_done` | 1 | Pulse | Timing Gen | All 4 frames transmitted |
| `busy` | 1 | — | Control | High while frame transmission in progress |

---

## 5. Algorithm

### 5.1 DSHOT Frame Transmission (Pseudocode)

```
ON send_frame pulse:
    IF busy:
        IGNORE (drop frame)
        RETURN

    SET busy = 1

    FOR each motor channel (0..3) IN PARALLEL:
        // Step 1: Build 16-bit frame
        IF NOT armed:
            throttle_val = 0    // disarm command
        ELSE:
            throttle_val = motor_value[ch]

        payload = {throttle_val[10:0], telemetry_req[ch]}   // 12 bits
        crc = payload[11:8] XOR payload[7:4] XOR payload[3:0]
        frame = {payload[11:0], crc[3:0]}                    // 16 bits

        // Step 2: Transmit 16 bits MSB-first
        FOR bit_idx = 15 DOWNTO 0:
            current_bit = frame[bit_idx]

            // Drive output high
            dshot_out[ch] = 1

            IF current_bit == 1:
                WAIT T1H_CLKS cycles (125 clocks)
            ELSE:
                WAIT T0H_CLKS cycles (63 clocks)

            // Drive output low for remainder of bit period
            dshot_out[ch] = 0
            WAIT remaining cycles until BIT_PERIOD total (167 clocks)

        // Step 3: Inter-frame gap (hold low)
        dshot_out[ch] = 0
        // Remains low until next send_frame

    WHEN all 4 channels complete bit 0:
        SET busy = 0
        ASSERT frame_done
```

### 5.2 PWM Mode (Pseudocode)

```
ON send_frame pulse (PWM mode):
    FOR each motor channel (0..3) IN PARALLEL:
        IF NOT armed:
            pulse_width = 100000   // 1000 µs × 100 clks = disarm
        ELSE:
            // Map 0-2047 to 1000-2000 µs (100000-200000 clocks)
            pulse_width = 100000 + (motor_value[ch] × 100000) / 2047

        // Generate pulse
        dshot_out[ch] = 1
        WAIT pulse_width clock cycles
        dshot_out[ch] = 0
        WAIT (PWM_PERIOD - pulse_width) clock cycles

    ASSERT frame_done
```

### 5.3 Bit-Level Timing Engine (Per Channel)

```
INTERNAL STATE per channel:
    bit_counter: 4 bits (15 downto 0)
    clk_counter: 8 bits (0 to 167)
    shift_reg: 16 bits (frame data)
    phase: IDLE | HIGH_PHASE | LOW_PHASE

ON each clock cycle:
    CASE phase:
        IDLE:
            output = 0
            IF start_trigger:
                load shift_reg with frame[15:0]
                bit_counter = 15
                clk_counter = 0
                phase = HIGH_PHASE
                output = 1

        HIGH_PHASE:
            output = 1
            clk_counter += 1
            threshold = (shift_reg[bit_counter] == 1) ? T1H_CLKS : T0H_CLKS
            IF clk_counter >= threshold:
                phase = LOW_PHASE
                output = 0

        LOW_PHASE:
            output = 0
            clk_counter += 1
            IF clk_counter >= BIT_PERIOD:
                clk_counter = 0
                IF bit_counter == 0:
                    phase = IDLE    // frame complete
                ELSE:
                    bit_counter -= 1
                    phase = HIGH_PHASE
                    output = 1
```

---

## 6. Resource Estimate

| Resource | Count | Notes |
|----------|-------|-------|
| DSP48E1 | 0 | No multiplications (CRC is XOR, timing is counters) |
| LUTs | ~200 | 4× bit-timing engines, CRC logic, mux for PWM/DSHOT mode |
| Flip-Flops | ~150 | 4× (16-bit shift reg + 8-bit counter + 4-bit bit_counter + state) |
| BRAM | 0 | No storage needed |
| Output Pins | 4 | One per motor/ESC |
| Fmax | >200 MHz | Simple counter logic with comparators |
| Latency | 2672 clocks | 26.72 µs per DSHOT frame (all 4 channels in parallel) |

### Per-Channel Breakdown

| Item | Bits | Count ×4 |
|------|------|----------|
| Shift register | 16 | 64 FF |
| Clock counter | 8 | 32 FF |
| Bit counter | 4 | 16 FF |
| State (phase) | 2 | 8 FF |
| CRC logic | — | ~20 LUT |
| Comparators | — | ~30 LUT |
| **Total per channel** | — | **30 FF + 50 LUT** |

### Artix-7 35T Utilization (XC7A35T)

| Resource | Available | Used | Utilization |
|----------|-----------|------|-------------|
| DSP48E1 | 90 | 0 | 0% |
| LUTs | 20,800 | 200 | 1.0% |
| Flip-Flops | 41,600 | 150 | 0.4% |
| I/O Pins | 210 | 4 | 1.9% |
