# FPGA Quadcopter Flight Controller

Flight-controller development for the Digilent Cmod A7-35T
(`xc7a35tcpg236-1`). The design combines deterministic SystemVerilog RTL with a
MicroBlaze V supervisor and uses FuseSoC for core and build management.

The approved architecture and execution plan are in:

- `docs/00_system_architecture.md`
- `docs/01_implementation_plan.md`
- `docs/02_pinout_and_wiring.md`
- `docs/03_verification_strategy.md`

## Toolchain

Required tools:

- FuseSoC 2.4.6
- Edalize 0.6.1
- Slang
- Verilator
- Cocotb 2.0.1
- Vivado 2024.1 or a compatible release
- GNU Make

FuseSoC and Cocotb are installed user-wide with `pipx`, not in a project
virtual environment. Cocotb 2.0.1 requires Python 3.13 or earlier.

```bash
pipx install --python python3.13 fusesoc==2.4.6
pipx inject --include-apps fusesoc cocotb==2.0.1
pipx runpip fusesoc install edalize==0.6.1
```

Ensure `~/.local/bin` is on `PATH`.

## Build Commands

```bash
make help
make cores
make versions
make lint
make sim
make regress
make synth
make check-clock-reset
make clean
```

The equivalent hyphenated aliases `make-lint`, `make-sim`, `make-synth`, and
`make-clean` are also available.

Targets accept overrides:

```bash
make lint SLANG_FLAGS='--std 1800-2017 --lint-only --single-unit'
make sim CORE=<core-with-sim-target>
make synth CORE=<board-integration-core>
```

Tool responsibilities are deliberately separated:

- `make lint` invokes Slang directly over repository RTL. It does not invoke
  FuseSoC or Verilator.
- `make sim` uses Verilator only as the Cocotb simulation backend.
- `make synth` uses Vivado.

The temporary default core is `clk_gen`. The old blinky and demonstration
`cmod_a7_top` cores have been removed. Simulation and synthesis become available
for the full design when the new portable flight-controller and Cmod integration
cores are introduced by the implementation plan.

The current `clk_gen` core has a Verilator+Cocotb simulation target, so
`make sim` is operational as the default smoke test. `make regress` runs all
registered core simulations.

## Current Core Layout

```text
cores/
  common/
    axi/
    clk_gen/
    clock_reset/
    reset_sync/
    timebase/
    xilinx_7series_sim/
```

Planned cores are described in `docs/00_system_architecture.md` and are added in
small, independently verified phases.

## Verification Policy

Every RTL phase must pass:

1. FuseSoC dependency resolution.
2. Slang syntax, elaboration, and lint.
3. Focused Verilator+Cocotb simulation.
4. Full Verilator+Cocotb regression.
5. Vivado checks when FPGA-specific behavior changes.

See `docs/03_verification_strategy.md` for protocol, numeric, safety, and
hardware verification requirements.
