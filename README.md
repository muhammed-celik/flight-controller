# HDL Build System — FuseSoC + Cocotb

Target FPGA: **Xilinx CMOD Artix A7-35T** (`xc7a35tcpg236-1`)

## Prerequisites

```bash
pip install fusesoc edalize cocotb
```

Also install one or more simulators: Icarus Verilog, Verilator, or a commercial tool.
For FPGA synthesis, Vivado must be in your `$PATH`.

## Project Structure

```
.
├── fusesoc.conf               # FuseSoC library/workspace config
├── README.md
└── cores/
    ├── clk_gen/               # Standalone clock generator core
    │   ├── clk_gen.core
    │   └── src/
    │       └── clk_gen.sv
    ├── blinky/                # Standalone blinky core
    │   ├── blinky.core
    │   ├── src/
    │   │   └── blinky.sv
    │   └── tb/
    │       └── test_blinky.py
    └── cmod_a7_top/           # Top-level core (depends on clk_gen + blinky)
        ├── cmod_a7_top.core
        ├── src/
        │   └── top.sv
        ├── tb/
        │   └── test_top.py
        └── constraints/
            └── cmod_a7.xdc
```

Each core is self-contained with its own `.core` file, sources, and testbench.
The `cmod_a7_top` core declares dependencies on `clk_gen` and `blinky`, which
FuseSoC resolves automatically when building.

## FuseSoC Quick Start

### Installation & Setup

```bash
# Install FuseSoC
pip install fusesoc

# Initialize a workspace (creates fusesoc.conf)
fusesoc init

# Or use the provided fusesoc.conf which points to this directory
```

### Core Discovery

```bash
# List all cores found in configured libraries
fusesoc core list

# Show details about a specific core
fusesoc core show blinky
```

### Running Targets

```bash
# Simulate blinky core with cocotb (uses Verilator by default)
fusesoc run --target sim blinky

# Simulate the top-level (pulls in clk_gen + blinky via dependencies)
fusesoc run --target sim cmod_a7_top

# Override the simulator tool
fusesoc run --target sim --tool icarus blinky

# Synthesize for CMOD A7-35T (requires Vivado)
fusesoc run --target synth cmod_a7_top

# Pass parameters to override defaults
fusesoc run --target sim blinky --CLK_FREQ=2400 --BLINK_HZ=2
```

### Build Output

FuseSoC places all build artifacts under `build/` by default:

```
build/
├── blinky_1.0.0/
│   └── sim-verilator/        # Simulation output + VCD traces
└── cmod_a7_top_1.0.0/
    ├── sim-verilator/        # Simulation output
    └── synth-vivado/         # Synthesis output (bitstream, reports)
```

## Library Management

FuseSoC resolves IP dependencies from **libraries** — directories containing `.core` files.

### Adding Libraries

```bash
# Add a remote library (cloned via git)
fusesoc library add fusesoc-cores https://github.com/fusesoc/fusesoc-cores

# Add a local library path
fusesoc library add my_common_ip /path/to/shared/ip

# List configured libraries
fusesoc library list
```

### fusesoc.conf

The `fusesoc.conf` file controls library paths and cache settings:

```ini
[main]
cachedir = build/cache

[library.local]
location = .
sync-type = local
auto-sync = false

[library.fusesoc-cores]
location = fusesoc_libraries/fusesoc-cores
sync-uri = https://github.com/fusesoc/fusesoc-cores
sync-type = git
auto-sync = true
```

### Dependencies Between Cores

Cores can depend on other cores. FuseSoC resolves them automatically:

```yaml
# uart.core
CAPI=2:
name: ::uart:1.0.0

filesets:
  rtl:
    files:
      - src/uart_tx.sv
      - src/uart_rx.sv
    file_type: systemVerilogSource
    depend:
      - ::fifo:>=1.0.0    # version constraint
      - ::clk_gen:1.0.0   # exact version
```

## Core File Reference (CAPI2)

A `.core` file is YAML describing an IP block:

```yaml
CAPI=2:
name: ::<core_name>:<version>
description: Short description

filesets:
  <fileset_name>:
    files:
      - path/to/file.sv
    file_type: systemVerilogSource | vhdlSource | xdc | user
    depend:
      - ::other_core:1.0.0

targets:
  <target_name>:
    filesets: [list, of, filesets]
    toplevel: module_name
    default_tool: icarus | verilator | vivado | quartus
    parameters: [PARAM1=value]
    tools:
      vivado:
        part: xc7a35tcpg236-1
      icarus:
        iverilog_options: [-g2012]

parameters:
  PARAM1:
    datatype: int | str | bool | real
    paramtype: vlogparam | vlogdefine | generic
```

## Cocotb Integration

FuseSoC natively supports cocotb via the simulator backends. Add your Python
test files as a fileset with `file_type: user` and FuseSoC will configure the
simulator to run them automatically:

```bash
# Run blinky cocotb tests (Verilator)
fusesoc run --target sim blinky

# Run top-level cocotb tests (resolves all dependencies)
fusesoc run --target sim cmod_a7_top

# Override simulator
fusesoc run --target sim --tool icarus blinky
```

## Clock Generator (clk_gen)

The `clk_gen` module wraps a Xilinx `MMCME2_BASE` primitive with a `BUFG`
output buffer. Default configuration: 12 MHz → 100 MHz.

```systemverilog
clk_gen #(
  .CLKIN_FREQ (12.0),   // Input MHz
  .VCO_MULT   (50.0),   // VCO = 12 × 50 = 600 MHz
  .OUT_DIV    (6.0),    // Out = 600 / 6 = 100 MHz
  .DIV_CLK    (1)
) u_clk_gen (
  .clk_in  (clk_12mhz),
  .rst     (reset),
  .clk_out (clk_100mhz),
  .locked  (mmcm_locked)
);
```

VCO must stay within 600–1200 MHz for the 7-series speed grade -1.

## Adding New IP

1. Create a new `<module>.core` file
2. Define filesets with source files and types
3. Declare dependencies with `depend:` if the module uses other cores
4. Add targets for sim, synth, lint, etc.
5. Run `fusesoc core list` to verify discovery
