#!/usr/bin/env python3
"""Build and run the cocotb suites with Icarus Verilog.

    python3 test/run_tests.py                 # core suite, BPRED=1 and BPRED=0
    python3 test/run_tests.py --bpred 1       # one build
    python3 test/run_tests.py --isa           # riscv-tests (needs `make firmware`)
    python3 test/run_tests.py --system        # boot ROM, UART, flash, firmware
    TESTCASE=test_timing python3 test/run_tests.py --bpred 1
"""
import argparse
import json
import os
import sys
from pathlib import Path

from cocotb_tools.runner import get_runner

ROOT = Path(__file__).resolve().parent.parent
RTL = [ROOT / "rtl" / f for f in ("riscv_core.sv", "icache.sv", "muldiv.sv", "csr.sv",
                                  "memory.sv", "spi_flash.sv", "periph.sv", "soc.sv")]
SOURCES = RTL + [ROOT / "test" / "tb_soc.sv"]
BOOTROM = ROOT / "firmware" / "bootrom" / "bootrom.hex"


def run(module, build_name, parameters, env=None):
    build = ROOT / "build" / build_name
    runner = get_runner(os.environ.get("SIM", "icarus"))
    runner.build(
        sources=SOURCES,
        hdl_toplevel="tb_soc",
        parameters=parameters,
        build_dir=build,
        build_args=["-g2012"],
        timescale=("1ns", "1ps"),
        always=True,
    )
    results = runner.test(
        hdl_toplevel="tb_soc",
        test_module=module,
        test_dir=ROOT / "test",
        build_dir=build,
        extra_env={"RESULTS_DIR": str(ROOT / "build"), **(env or {})},
        testcase=os.environ.get("TESTCASE") or None,
    )
    text = Path(results).read_text()
    if "<failure" in text or "<error" in text:
        sys.exit(f"FAILED: {module} ({build_name})")


def summarize():
    files = {b: ROOT / "build" / f"cpi_bpred{b}.json" for b in (1, 0)}
    if not all(p.exists() for p in files.values()):
        return
    btb, static = (json.loads(files[b].read_text()) for b in (1, 0))
    print(f"\n{'benchmark':16s} {'instr':>7s} {'CPI BTB':>8s} {'CPI static':>11s} {'speedup':>8s}")
    for name in btb:
        a, b = btb[name], static[name]
        print(f"{name:16s} {a['instret']:7d} {a['cpi']:8.3f} {b['cpi']:11.3f} "
              f"{b['cycles'] / a['cycles']:7.2f}x")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--bpred", type=int, choices=(0, 1))
    ap.add_argument("--isa", action="store_true", help="riscv-tests in simulation")
    ap.add_argument("--system", action="store_true", help="boot ROM / UART / flash / firmware")
    args = ap.parse_args()
    rom = f'"{BOOTROM}"'
    if args.isa:
        run("test_isa", "sim_isa", {"RESET_PC": "32'h00000000", "ROM_FILE": rom})
    elif args.system:
        run("test_system", "sim_system", {"ROM_FILE": rom})
    else:
        for b in ([args.bpred] if args.bpred is not None else [1, 0]):
            run("test_core", f"sim_bpred{b}",
                {"RESET_PC": "32'h00002000", "BPRED": b, "ROM_FILE": rom},
                {"BPRED": str(b)})
        summarize()
