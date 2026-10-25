#!/usr/bin/env python3
"""Build and run the cocotb suite with Icarus Verilog.

    python3 test/run_tests.py                # forwarding and interlock-only builds
    python3 test/run_tests.py --forwarding 1 # one build
    python3 test/run_tests.py --top          # full system over UART (slow)
    TESTCASE=test_timing python3 test/run_tests.py --forwarding 1
"""
import argparse
import json
import os
import sys
from pathlib import Path

from cocotb_tools.runner import get_runner

ROOT = Path(__file__).resolve().parent.parent
SOURCES = [ROOT / "rtl" / "riscv_core.sv", ROOT / "rtl" / "memory.sv", ROOT / "test" / "tb_core.sv"]


def run(forwarding: int) -> None:
    build = ROOT / "build" / f"sim_fwd{forwarding}"
    runner = get_runner(os.environ.get("SIM", "icarus"))
    runner.build(
        sources=SOURCES,
        hdl_toplevel="tb_core",
        parameters={"FORWARDING": forwarding},
        build_dir=build,
        build_args=["-g2012"],
        timescale=("1ns", "1ps"),
        always=True,
    )
    runner.test(
        hdl_toplevel="tb_core",
        test_module="test_riscv",
        test_dir=ROOT / "test",
        build_dir=build,
        extra_env={"FORWARDING": str(forwarding), "RESULTS_DIR": str(ROOT / "build")},
        testcase=os.environ.get("TESTCASE") or None,
    )


def run_top() -> None:
    """riscv_top + UART, driven by the unmodified host script scripts/test_fpga.py."""
    rtl = ROOT / "rtl"
    build = ROOT / "build" / "sim_top"
    runner = get_runner(os.environ.get("SIM", "icarus"))
    runner.build(
        sources=[rtl / "riscv_core.sv", rtl / "memory.sv", rtl / "riscv_top.sv",
                 rtl / "uart.v", rtl / "uart_tx.v", rtl / "uart_rx.v"],
        hdl_toplevel="riscv_top",
        build_dir=build,
        build_args=["-g2012"],
        timescale=("1ns", "1ps"),
        always=True,
    )
    runner.test(hdl_toplevel="riscv_top", test_module="test_top",
                test_dir=ROOT / "test", build_dir=build)


def summarize() -> None:
    files = {f: ROOT / "build" / f"cpi_forwarding{f}.json" for f in (1, 0)}
    if not all(p.exists() for p in files.values()):
        return
    fwd, nofwd = (json.loads(files[f].read_text()) for f in (1, 0))
    print(f"\n{'benchmark':16s} {'CPI fwd':>8s} {'CPI interlock':>14s} {'speedup':>8s}")
    for name in fwd:
        a, b = fwd[name], nofwd[name]
        print(f"{name:16s} {a['cpi']:8.3f} {b['cpi']:14.3f} {b['cycles'] / a['cycles']:7.2f}x")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--forwarding", type=int, choices=(0, 1))
    ap.add_argument("--top", action="store_true", help="run only the full-system UART test")
    args = ap.parse_args()
    if args.top:
        run_top()
        sys.exit(0)
    for f in ([args.forwarding] if args.forwarding is not None else [1, 0]):
        run(f)
    summarize()
    sys.exit(0)
