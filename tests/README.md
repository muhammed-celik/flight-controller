# Verification Support

Shared verification code is organized as follows:

- `tests/cocotb/`: reusable Cocotb drivers, monitors, scoreboards, and protocol
  helpers.
- `tests/reference/`: bit-accurate and floating-point Python reference models.

Core-specific tests remain beside their FuseSoC core under `tb/`. Shared code is
added only when at least two cores use it.
