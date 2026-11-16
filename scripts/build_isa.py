#!/usr/bin/env python3
"""Build the official RISC-V ISA tests (riscv-tests) for the SoC.

The tests and their bare-metal "p" environment are used unmodified; only the
link address changes (RAM starts at 0). A test reports its result by writing
the `tohost` word: 1 = pass, anything else = (failing test number << 1) | 1.

Writes one flat binary per test plus manifest.json with each `tohost` address.

Left out, with the reason:
  rv32ui/ma_data        needs misaligned loads/stores to succeed; this core traps
                        on them, which the ISA allows
  rv32mi/breakpoint     debug triggers (tselect/tdata) are not implemented
  rv32mi/pmpaddr        no PMP
  rv32mi/illegal        exercises supervisor mode and vectored interrupts
  rv32mi/instret_overflow  writes minstret; the counters are read-only here
  rv32mi/*-misaligned   expect misaligned accesses to succeed
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SUITES = ("rv32ui", "rv32um", "rv32mi")
SKIP = {
    "rv32ui": {"ma_data"},
    "rv32um": set(),
    "rv32mi": {"breakpoint", "pmpaddr", "illegal", "instret_overflow", "lh-misaligned",
               "lw-misaligned", "sh-misaligned", "sw-misaligned"},
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="riscv-tests checkout")
    ap.add_argument("--out", required=True)
    ap.add_argument("--cc", default="riscv-none-elf-gcc")
    ap.add_argument("--objcopy", default="riscv-none-elf-objcopy")
    args = ap.parse_args()

    src, out = Path(args.src), Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    if not (src / "env" / "p" / "riscv_test.h").exists():
        subprocess.run(["git", "-C", str(src), "submodule", "update", "--init", "--depth", "1",
                        "env"], check=True)

    nm = args.cc.replace("gcc", "nm")
    manifest = {}
    for suite in SUITES:
        for path in sorted((src / "isa" / suite).glob("*.S")):
            if path.stem in SKIP[suite]:
                continue
            name = f"{suite}-{path.stem}"
            elf, binary = out / f"{name}.elf", out / f"{name}.bin"
            subprocess.run([
                args.cc, "-march=rv32im_zicsr_zifencei", "-mabi=ilp32", "-static",
                "-mcmodel=medany", "-fvisibility=hidden", "-nostdlib", "-nostartfiles",
                f"-I{src / 'env' / 'p'}", f"-I{src / 'isa' / 'macros' / 'scalar'}",
                f"-T{ROOT / 'firmware' / 'isa' / 'link.ld'}", "-Wl,--no-warn-rwx-segments",
                str(path), "-o", str(elf)], check=True)
            subprocess.run([args.objcopy, "-O", "binary", str(elf), str(binary)], check=True)
            symbols = subprocess.run([nm, str(elf)], check=True, capture_output=True, text=True)
            tohost = next(int(line.split()[0], 16) for line in symbols.stdout.splitlines()
                          if line.split()[-1] == "tohost")
            manifest[name] = {"bin": binary.name, "tohost": tohost}

    (out / "manifest.json").write_text(json.dumps(manifest, indent=1))
    print(f"built {len(manifest)} ISA tests in {out}")


if __name__ == "__main__":
    sys.exit(main())
