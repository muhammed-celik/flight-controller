# AXI4-Lite Global Register Map

## Interface Contract

`fc_axi_regs` exposes a flattened 32-bit AXI4-Lite slave with a 16-bit address
and a 64 KiB aperture. It permits one outstanding read and one outstanding
write. Write-address and write-data channels are accepted independently and may
arrive in either order.

All registers are 32-bit and require 4-byte alignment. Misaligned, unmapped,
read-only writes, and write-only reads return `SLVERR`. AXI responses and read
data remain stable until accepted by the master.

## Phase 3 Registers

| Offset | Name | Access | Reset | Description |
| ---: | --- | --- | ---: | --- |
| `0x0000` | `IP_ID` | RO | `0x46430001` | Flight-controller register-bank identity |
| `0x0004` | `VERSION` | RO | `0x00010000` | Major/minor hardware interface version |
| `0x0008` | `CAPABILITIES` | RO | `0x00000003` | Bit 0 snapshot, bit 1 shadow/commit |
| `0x000C` | `SCRATCH` | RW | `0` | Byte-strobe validation and software probe |
| `0x0010` | `STATUS` | RO | external | Current global status input |
| `0x0014` | `IRQ_STATUS` | RO | `0` | Sticky pending interrupt sources |
| `0x0018` | `IRQ_ENABLE` | RW | `0` | Interrupt output enable mask |
| `0x001C` | `IRQ_CLEAR` | WO | - | Write-one-to-clear pending sources |
| `0x0020` | `TIME_CAPTURE` | WO | - | Write bit 0 to capture `time_cycles` |
| `0x0024` | `TIME_LO` | RO | `0` | Captured timestamp bits 31:0 |
| `0x0028` | `TIME_HI` | RO | `0` | Captured timestamp bits 63:32 |
| `0x002C` | `TIME_SEQUENCE` | RO | `0` | Increments after every capture |
| `0x0030` | `CFG_SHADOW_LO` | RW | `0` | Pending configuration bits 31:0 |
| `0x0034` | `CFG_SHADOW_HI` | RW | `0` | Pending configuration bits 63:32 |
| `0x0038` | `CFG_COMMIT` | WO | - | Write bit 0 to atomically activate shadow data |
| `0x003C` | `CFG_ACTIVE_LO` | RO | `0` | Active configuration bits 31:0 |
| `0x0040` | `CFG_ACTIVE_HI` | RO | `0` | Active configuration bits 63:32 |
| `0x0044` | `CFG_SEQUENCE` | RO | `0` | Increments after every commit |

`IRQ_STATUS` applies clear-before-set semantics each cycle. An interrupt source
that remains asserted is therefore re-latched and cannot be cleared until the
source condition is removed. The external `irq` output is the reduction OR of
`IRQ_STATUS & IRQ_ENABLE`.

Configuration writes change only shadow storage. `CFG_COMMIT` copies the entire
64-bit shadow value to active storage on one clock edge and emits a one-cycle
`config_commit` pulse. A shadow write and commit must be separate AXI writes.
