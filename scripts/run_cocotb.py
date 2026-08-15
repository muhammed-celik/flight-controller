#!/usr/bin/env python3
"""Run a Cocotb test using HDL sources staged by FuseSoC."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from cocotb_tools.runner import get_results, get_runner
import yaml


VERILOG_FILE_TYPES = {
    "systemVerilogSource",
    "systemVerilogSource-2005",
    "systemVerilogSource-2009",
    "systemVerilogSource-2012",
    "systemVerilogSource-2017",
    "verilogSource",
    "verilogSource-95",
    "verilogSource-2001",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-root", type=Path, required=True)
    parser.add_argument("--test-module", required=True)
    return parser.parse_args()


def find_one(root: Path, pattern: str, description: str) -> Path:
    matches = sorted(root.rglob(pattern))
    if len(matches) != 1:
        listing = "\n".join(f"  {path}" for path in matches) or "  <none>"
        raise RuntimeError(
            f"expected one {description} under {root}, found {len(matches)}:\n"
            f"{listing}"
        )
    return matches[0]


def main() -> int:
    args = parse_args()
    build_root = args.build_root.resolve()
    edam_path = find_one(build_root, "*.eda.yml", "FuseSoC EDAM manifest")
    work_root = edam_path.parent

    with edam_path.open(encoding="utf-8") as stream:
        edam = yaml.safe_load(stream)

    sources: list[Path] = []
    includes: set[Path] = set()
    for entry in edam.get("files", []):
        if entry.get("file_type", "") not in VERILOG_FILE_TYPES:
            continue
        source = (work_root / entry["name"]).resolve()
        if entry.get("is_include_file", False):
            includes.add(source.parent)
        else:
            sources.append(source)
        if entry.get("include_path"):
            includes.add((work_root / entry["include_path"]).resolve())

    if not sources:
        raise RuntimeError(f"no Verilog sources found in {edam_path}")

    test_file = find_one(
        work_root, f"{args.test_module}.py", "Cocotb test module"
    )
    toplevel = edam.get("toplevel")
    if not isinstance(toplevel, str) or not toplevel:
        raise RuntimeError(f"missing HDL toplevel in {edam_path}")

    cocotb_build = work_root / "cocotb_build"
    runner = get_runner("verilator")
    runner.build(
        sources=sources,
        includes=sorted(includes),
        hdl_toplevel=toplevel,
        build_dir=cocotb_build,
        build_args=[
            "--timing",
            "-Wall",
            "-Wno-PINCONNECTEMPTY",
            "-Wno-DECLFILENAME",
            "-Wno-TIMESCALEMOD",
        ],
        always=True,
        timescale=("1ns", "1ps"),
    )
    results_xml = cocotb_build / "results.xml"
    runner.test(
        test_module=args.test_module,
        hdl_toplevel=toplevel,
        build_dir=cocotb_build,
        test_dir=test_file.parent,
        results_xml=str(results_xml),
    )
    test_count, failure_count = get_results(results_xml)
    if test_count == 0:
        raise RuntimeError("Cocotb did not execute any tests")
    if failure_count:
        print(
            f"run_cocotb: {failure_count} of {test_count} tests failed",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, yaml.YAMLError) as error:
        print(f"run_cocotb: {error}", file=sys.stderr)
        sys.exit(2)
