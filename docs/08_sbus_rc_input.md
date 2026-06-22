# Module 08: SBUS RC Input Decoder

## 1. Why This Module is Needed

The RC (Radio Control) input module receives pilot commands from a radio receiver and decodes them into usable channel values for the flight controller. Without RC input, the drone cannot be manually piloted — there is no way to command roll, pitch, yaw, throttle, or auxiliary functions (arming, flight mode selection).

**SBUS** (Serial Bus) is the de facto standard protocol for modern RC systems (FrSky, Futaba, TBS Crossfire, ELRS). It provides:
- 16 channels of 11-bit resolution in a single wire
- Low latency (7–14 ms frame rate)
- Built-in failsafe detection
- Digital protocol (immune to PWM timing jitter)

The alternative — individual PWM inputs — would require 8+ input pins, 8+ pulse-width measurement circuits, and has no failsafe indication. SBUS consolidates all this into a single UART-like serial stream.

---

## 2. Options/Approaches

| Approach | Pros | Cons | Selected |
|----------|------|------|----------|
| SBUS (inverted UART) | 16 channels, 1 wire, failsafe, industry standard | Inverted signal (needs HW or SW inversion) | **Yes** |
| Individual PWM inputs (1 per channel) | Simple, universal | 8+ pins, no failsafe, timing-sensitive | No |
| PPM (combined pulse train) | Single wire, 8 channels | Low resolution, long frame (22 ms), no failsafe | No |
| CRSF (Crossfire protocol) | Very low latency, bidirectional | More complex parsing, variable frame size | Future |
| IBUS (FlySky) | Simple UART, 14 channels | Limited ecosystem, no failsafe byte | No |
| SUMD (Graupner) | 16 channels, CRC protected | Rare outside Graupner ecosystem | No |

### Signal Inversion Options

| Method | Pros | Cons | Selected |
|--------|------|------|----------|
| External inverter IC (e.g., SN74LVC1G04) | Works with any FPGA | Extra component, board space | Backup |
| FPGA IOB polarity inversion | Zero components, configured in constraints | Not all FPGA I/O banks support this | **Yes** |
| Software inversion (invert in logic) | Universal | Trivial extra LUT | **Yes** (combined) |

---

## 3. Theoretical Foundations

### 3.1 SBUS Serial Parameters

SBUS uses an inverted UART protocol with non-standard parameters:

| Parameter | Value |
|-----------|-------|
| Baud rate | 100,000 bps |
| Data bits | 8 |
| Parity | Even |
| Stop bits | 2 |
| Logic | **Inverted** (idle = LOW, start bit = HIGH) |
| Byte time | 12 bits × 10 µs = 120 µs |
| Frame size | 25 bytes |
| Frame time | 25 × 120 µs = 3 ms (transmission time) |
| Frame interval | 14 ms (analog) or 7 ms (digital Rx) |

### 3.2 Baud Rate Timing

At 100 MHz system clock:

```
Clocks per bit = 100,000,000 / 100,000 = 1000 clocks
Sample point = 1000 / 2 = 500 clocks (mid-bit sampling)
```

### 3.3 Frame Structure

Each SBUS frame is 25 bytes:

```
Byte 0:       0x0F (start/header byte)
Bytes 1-22:   Channel data (16 channels × 11 bits = 176 bits, packed LSB-first)
Byte 23:      Flags byte
Byte 24:      0x00 (end/footer byte)
```

### 3.4 Channel Data Packing

The 16 channels (11 bits each) are packed contiguously across bytes 1–22 in LSB-first order:

```
Byte 1:   CH1[7:0]               (lower 8 bits of channel 1)
Byte 2:   CH2[4:0] | CH1[10:8]   (upper 3 of CH1, lower 5 of CH2)
Byte 3:   CH3[1:0] | CH2[10:5]   (upper 6 of CH2, lower 2 of CH3)
Byte 4:   CH3[9:2]               (middle 8 bits of CH3)
Byte 5:   CH4[6:0] | CH3[10]     (upper 1 of CH3, lower 7 of CH4)
Byte 6:   CH5[3:0] | CH4[10:7]   (upper 4 of CH4, lower 4 of CH5)
...       (pattern repeats every 8 channels = 11 bytes)
Byte 12:  CH9[7:0]               (same pattern restarts for CH9-CH16)
...
Byte 22:  CH16[10:3]             (upper 8 bits of channel 16)
```

General formula for extracting channel N (0-indexed):
```
bit_offset = N × 11
start_byte = 1 + (bit_offset / 8)
start_bit  = bit_offset % 8
channel[N] = data_bytes[start_byte .. start_byte+2] >> start_bit & 0x7FF
```

### 3.5 Channel Value Range

| Value | Meaning |
|-------|---------|
| 0 | Absolute minimum (below normal range) |
| 172 | Typical stick minimum |
| 992 | Center/neutral position |
| 1811 | Typical stick maximum |
| 2047 | Absolute maximum (above normal range) |

### 3.6 Flags Byte (Byte 23)

```
Bit 0: Channel 17 (digital, binary)
Bit 1: Channel 18 (digital, binary)
Bit 2: Frame lost (1 = receiver lost a frame)
Bit 3: Failsafe active (1 = receiver in failsafe mode — link lost)
Bits 4-7: Reserved (always 0)
```

### 3.7 Failsafe Behavior

When the radio link is lost:
- **Frame lost** (bit 2): Indicates occasional dropped frames — temporary condition
- **Failsafe active** (bit 3): Persistent link loss — receiver outputs pre-programmed values

Flight controller response to failsafe:
```
IF failsafe_active:
    Immediately disarm motors (or enter auto-land/return-to-home)
    Ignore all channel data (use safe defaults)

IF frame_lost AND (consecutive_lost_frames > THRESHOLD):
    Enter degraded mode / hold last known good values
```

### 3.8 Signal Inversion

Standard UART: idle = HIGH, start bit = LOW
SBUS (inverted): idle = LOW, start bit = HIGH

Inversion is applied at the input:
```
sbus_data_corrected = NOT sbus_rx_pin
```

After inversion, standard UART reception logic applies.

---

## 4. Module Interface

### 4.1 Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `CLK_FREQ` | 100 MHz | System clock |
| `BAUD_RATE` | 100000 | SBUS baud rate |
| `CLKS_PER_BIT` | 1000 | 100 MHz / 100 kbaud |
| `NUM_CHANNELS` | 16 | SBUS analog channels |
| `CHANNEL_BITS` | 11 | Bits per channel |
| `FRAME_BYTES` | 25 | Total SBUS frame size |
| `FRAME_TIMEOUT` | 50 ms | Max time between valid frames before timeout |

### 4.2 Input Signals

| Signal | Width | Format | Source | Description |
|--------|-------|--------|--------|-------------|
| `clk` | 1 | — | System | 100 MHz system clock |
| `rst_n` | 1 | — | System | Active-low synchronous reset |
| `sbus_rx` | 1 | — | Rx pin | SBUS serial input (inverted UART from receiver) |

### 4.3 Output Signals

| Signal | Width | Format | Destination | Description |
|--------|-------|--------|-------------|-------------|
| `channel[15:0]` | 16×11 | Unsigned | RC Mapping | 16 decoded channel values (0–2047) |
| `channel_valid` | 1 | Pulse | Control | Asserted for 1 clock when new frame decoded |
| `failsafe` | 1 | Level | Safety Logic | 1 = receiver reports failsafe (link lost) |
| `frame_lost` | 1 | Level | Telemetry | 1 = receiver reports dropped frame |
| `link_timeout` | 1 | Level | Safety Logic | 1 = no valid frame received within FRAME_TIMEOUT |
| `ch17_digital` | 1 | — | Aux | Digital channel 17 |
| `ch18_digital` | 1 | — | Aux | Digital channel 18 |

### 4.4 Channel Mapping (Application-Level)

| Channel | Function | Range | Notes |
|---------|----------|-------|-------|
| CH1 | Roll | 172–1811 | Center = 992 |
| CH2 | Pitch | 172–1811 | Center = 992 |
| CH3 | Throttle | 172–1811 | Low = 172 (no center) |
| CH4 | Yaw | 172–1811 | Center = 992 |
| CH5 | Arm switch | <992 / >992 | 2-position switch |
| CH6 | Flight mode | 3-position | Low/Mid/High |
| CH7–16 | Auxiliary | 172–1811 | User configurable |

---

## 5. Algorithm

### 5.1 UART Byte Reception (Pseudocode)

```
INTERNAL STATE:
    rx_state: IDLE | START_BIT | DATA_BITS | PARITY_BIT | STOP_BITS
    bit_counter: 0..7 (for 8 data bits)
    clk_counter: 0..999 (baud rate timing)
    shift_reg: 8 bits
    rx_inverted: inverted version of sbus_rx pin

ON each clock cycle:
    rx_inverted = NOT sbus_rx   // invert SBUS polarity

    CASE rx_state:
        IDLE:
            IF rx_inverted == 0 (start bit detected after inversion):
                clk_counter = 500    // wait half bit to sample at center
                rx_state = START_BIT

        START_BIT:
            clk_counter -= 1
            IF clk_counter == 0:
                IF rx_inverted == 0:   // confirm start bit is still low
                    bit_counter = 0
                    clk_counter = 1000
                    rx_state = DATA_BITS
                ELSE:
                    rx_state = IDLE    // false start, go back

        DATA_BITS:
            clk_counter -= 1
            IF clk_counter == 0:
                shift_reg[bit_counter] = rx_inverted   // LSB first
                bit_counter += 1
                clk_counter = 1000
                IF bit_counter == 8:
                    rx_state = PARITY_BIT

        PARITY_BIT:
            clk_counter -= 1
            IF clk_counter == 0:
                parity_received = rx_inverted
                expected_parity = XOR of all 8 data bits (even parity)
                parity_ok = (parity_received == expected_parity)
                clk_counter = 1000
                rx_state = STOP_BITS

        STOP_BITS:
            clk_counter -= 1
            IF clk_counter == 0:
                IF rx_inverted == 1:   // stop bit = HIGH after inversion
                    IF parity_ok:
                        OUTPUT byte_received = shift_reg
                        ASSERT byte_valid
                rx_state = IDLE
```

### 5.2 Frame Assembly (Pseudocode)

```
INTERNAL STATE:
    frame_buffer[0..24]: 25-byte buffer
    byte_index: 0..24
    frame_state: WAIT_HEADER | COLLECT_DATA | VALIDATE

ON byte_valid assertion:
    CASE frame_state:
        WAIT_HEADER:
            IF byte_received == 0x0F:
                frame_buffer[0] = 0x0F
                byte_index = 1
                frame_state = COLLECT_DATA

        COLLECT_DATA:
            frame_buffer[byte_index] = byte_received
            byte_index += 1
            IF byte_index == 25:
                frame_state = VALIDATE

        VALIDATE:
            IF frame_buffer[24] == 0x00:   // valid footer
                // Frame is valid — decode channels
                CALL decode_channels(frame_buffer[1..22])
                
                // Extract flags
                failsafe = frame_buffer[23][3]
                frame_lost = frame_buffer[23][2]
                ch17_digital = frame_buffer[23][0]
                ch18_digital = frame_buffer[23][1]
                
                ASSERT channel_valid
                RESET link_timeout_counter
            ELSE:
                // Invalid frame — discard and resync
                // (could be mid-frame sync loss)
            
            frame_state = WAIT_HEADER
```

### 5.3 Channel Decoding (Bit Unpacking)

```
PROCEDURE decode_channels(data[1..22]):
    // Unpack 176 bits (16 channels × 11 bits) from 22 bytes
    // Bits are packed LSB-first contiguously

    bit_stream[175:0] = concatenate data[1] through data[22] (LSB of byte 1 = bit 0)

    FOR ch = 0 TO 15:
        start_bit = ch × 11
        channel[ch] = bit_stream[start_bit + 10 : start_bit]   // 11 bits, unsigned

    // Equivalent shift-register approach (hardware-friendly):
    // Load all 22 bytes into a 176-bit shift register
    // Extract 11 bits at a time, shifting by 11 each iteration
```

### 5.4 Link Timeout Detection

```
INTERNAL STATE:
    timeout_counter: counts up each clock cycle

ON each clock cycle:
    IF channel_valid asserted:
        timeout_counter = 0
    ELSE:
        timeout_counter += 1
    
    IF timeout_counter >= FRAME_TIMEOUT × CLK_FREQ:   // 50ms × 100MHz = 5,000,000
        link_timeout = 1
    ELSE:
        link_timeout = 0
```

---

## 6. Resource Estimate

| Resource | Count | Notes |
|----------|-------|-------|
| DSP48E1 | 0 | No multiplications needed |
| LUTs | ~200 | UART state machine, bit counter, frame parser, channel extraction muxes |
| Flip-Flops | ~300 | 176-bit shift register (channel unpacking), 25-byte frame buffer (200 bits), UART regs, counters |
| BRAM | 0 | Frame buffer fits in distributed RAM/FFs (25 bytes = 200 bits) |
| Input Pins | 1 | Single SBUS serial input |
| Fmax | >200 MHz | Simple sequential logic with counters |
| Latency | ~3 ms | Frame transmission time (25 bytes × 120 µs/byte) |
| Update rate | 71–143 Hz | 7 ms (digital Rx) to 14 ms (analog Rx) frame interval |

### Register Breakdown

| Item | Bits | Notes |
|------|------|-------|
| UART shift register | 8 | Byte reception |
| UART counters | 10+3+2 = 15 | clk_counter, bit_counter, state |
| Frame buffer | 200 | 25 bytes × 8 bits |
| Channel outputs | 176 | 16 channels × 11 bits |
| Timeout counter | 23 | Counts to 5,000,000 |
| Flags/status | 5 | failsafe, frame_lost, link_timeout, ch17, ch18 |
| **Total** | **~427** | Conservative estimate |

### Artix-7 35T Utilization (XC7A35T)

| Resource | Available | Used | Utilization |
|----------|-----------|------|-------------|
| DSP48E1 | 90 | 0 | 0% |
| LUTs | 20,800 | 200 | 1.0% |
| Flip-Flops | 41,600 | 300 | 0.7% |
| I/O Pins | 210 | 1 | 0.5% |
