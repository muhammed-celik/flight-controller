SHELL := /usr/bin/env bash

.DEFAULT_GOAL := help
.DELETE_ON_ERROR:

CORE              ?= clk_gen
SIM_TARGET        ?= sim
SYNTH_TARGET      ?= synth
BUILD_ROOT        ?= $(CURDIR)/build
SIM_BUILD_ROOT    ?= $(BUILD_ROOT)/sim/$(CORE)
FUSESOC_FLAGS     ?=
SLANG_FLAGS       ?= --std 1800-2017 --lint-only --single-unit --ignore-unknown-modules
COCOTB_MODULE     ?= test_$(CORE)

RTL_SOURCE_DIRS   := $(sort $(wildcard cores/*/src cores/*/*/src cores/*/*/*/src))
SLANG_ALL_SOURCES := $(foreach dir,$(RTL_SOURCE_DIRS),$(wildcard $(dir)/*.sv $(dir)/*.v))
SLANG_PACKAGES    := $(filter %_pkg.sv %_package.sv,$(SLANG_ALL_SOURCES))
SLANG_SOURCES     ?= $(SLANG_PACKAGES) $(filter-out $(SLANG_PACKAGES),$(SLANG_ALL_SOURCES))

FUSESOC           ?= fusesoc
SLANG             ?= slang
VERILATOR         ?= verilator
VIVADO            ?= vivado
COCOTB_CONFIG      ?= cocotb-config

.PHONY: help clean lint sim regress synth check-clock-reset cores versions \
        check-fusesoc check-slang check-verilator \
        check-cocotb check-vivado make-clean make-lint make-sim make-synth

help:
	@printf '%s\n' \
	  'Flight controller build targets:' \
	  '  make lint       Lint all repository RTL directly with Slang' \
	  '  make sim        Simulate CORE with Verilator and Cocotb' \
	  '  make regress    Run all registered Cocotb core regressions' \
	  '  make synth      Synthesize CORE with Vivado' \
	  '  make check-clock-reset  Validate synthesized clock/reset constraints' \
	  '  make clean      Remove generated build artifacts' \
	  '  make cores      List FuseSoC cores' \
	  '  make versions   Print selected tool versions' \
	  '' \
	  'Common overrides:' \
	  '  CORE=<core> BUILD_ROOT=<path> FUSESOC=<path>' \
	  '  SIM_TARGET=<target> COCOTB_MODULE=<python-module>' \
	  '  SYNTH_TARGET=<target>' \
	  '  SLANG=<path> SLANG_FLAGS=<flags> SLANG_SOURCES=<files>'

clean:
	@build_root="$(abspath $(BUILD_ROOT))"; \
	if [[ -z "$$build_root" || "$$build_root" == "/" || "$$build_root" == "$(CURDIR)" ]]; then \
	  printf 'Refusing to remove unsafe BUILD_ROOT: %s\n' "$$build_root" >&2; \
	  exit 2; \
	fi; \
	rm -rf -- "$$build_root" "$(CURDIR)/sim_build" "$(CURDIR)/obj_dir"; \
	rm -f -- "$(CURDIR)/results.xml"

lint: check-slang
	@test -n "$(strip $(SLANG_SOURCES))" || { \
	  printf 'No Verilog or SystemVerilog sources found under cores/\n' >&2; exit 2; \
	}
	$(SLANG) $(SLANG_FLAGS) $(SLANG_SOURCES)

sim: check-fusesoc check-verilator check-cocotb
	@rm -rf -- "$(SIM_BUILD_ROOT)"
	$(FUSESOC) $(FUSESOC_FLAGS) run \
	  --target=$(SIM_TARGET) \
	  --tool=verilator \
	  --setup \
	  --build-root="$(SIM_BUILD_ROOT)" \
	  $(CORE)
	$$($(COCOTB_CONFIG) --python-bin) scripts/run_cocotb.py \
	  --build-root "$(SIM_BUILD_ROOT)" \
	  --test-module "$(COCOTB_MODULE)"

regress:
	$(MAKE) sim CORE=clk_gen COCOTB_MODULE=test_clk_gen
	$(MAKE) sim CORE=reset_sync COCOTB_MODULE=test_reset_sync
	$(MAKE) sim CORE=timebase COCOTB_MODULE=test_timebase
	$(MAKE) sim CORE=clock_reset COCOTB_MODULE=test_clock_reset
	$(MAKE) sim CORE=fc_axi_regs COCOTB_MODULE=test_fc_axi_regs

synth: check-fusesoc check-vivado
	PATH="$(dir $(shell command -v $(VIVADO))):$$PATH" $(FUSESOC) $(FUSESOC_FLAGS) run \
	  --target=$(SYNTH_TARGET) \
	  --tool=vivado \
	  --build-root="$(BUILD_ROOT)" \
	  $(CORE)

check-clock-reset: check-vivado
	@test -f "$(BUILD_ROOT)/clock_reset_1.0.0/synth-vivado/clock_reset_1.0.0.xpr" || { \
	  printf 'Synthesize clock_reset before running this check\n' >&2; exit 2; \
	}
	PATH="$(dir $(shell command -v $(VIVADO))):$$PATH" $(VIVADO) \
	  -mode batch -notrace \
	  -source cores/common/clock_reset/tcl/check_synth.tcl \
	  -tclargs "$(BUILD_ROOT)/clock_reset_1.0.0/synth-vivado/clock_reset_1.0.0.xpr"

cores: check-fusesoc
	$(FUSESOC) $(FUSESOC_FLAGS) core list

versions: check-fusesoc check-slang check-verilator check-cocotb
	@$(FUSESOC) --version
	@$(SLANG) --version
	@$(VERILATOR) --version
	@$(COCOTB_CONFIG) --version
	@if command -v "$(VIVADO)" >/dev/null 2>&1; then $(VIVADO) -version; fi

check-fusesoc:
	@command -v "$(FUSESOC)" >/dev/null 2>&1 || { \
	  printf 'FuseSoC not found: %s\n' "$(FUSESOC)" >&2; exit 127; \
	}

check-slang:
	@command -v "$(SLANG)" >/dev/null 2>&1 || { \
	  printf 'Slang not found: %s\n' "$(SLANG)" >&2; exit 127; \
	}

check-verilator:
	@command -v "$(VERILATOR)" >/dev/null 2>&1 || { \
	  printf 'Verilator not found: %s\n' "$(VERILATOR)" >&2; exit 127; \
	}

check-cocotb:
	@command -v "$(COCOTB_CONFIG)" >/dev/null 2>&1 || { \
	  printf 'Cocotb not found: %s\n' "$(COCOTB_CONFIG)" >&2; exit 127; \
	}

check-vivado:
	@command -v "$(VIVADO)" >/dev/null 2>&1 || { \
	  printf 'Vivado not found: %s\n' "$(VIVADO)" >&2; exit 127; \
	}

# Compatibility aliases for callers that use hyphenated target names.
make-clean: clean
make-lint: lint
make-sim: sim
make-synth: synth
