#!/usr/bin/env python3
"""Check D1S interrupt vectors in a built ELF.

This catches the known-bad LTO failure mode where the startup file's weak
aliases win and critical vectors point at Default_Handler instead of the real
C handlers. It can be run manually or loaded as a PlatformIO post-build script.
"""

from __future__ import annotations

import argparse
import pathlib
import re
import struct
import subprocess
import sys
import tempfile
from typing import Callable


# Vector indices, not IRQ numbers. Cortex exceptions occupy entries 0..15.
# These are the handlers that made the old LTO build nonfunctional when they
# resolved to Default_Handler.
REQUIRED_VECTORS = (
    (15, "SysTick_Handler"),
    (16 + 20, "USB_LP_CAN1_RX0_IRQHandler"),
    (16 + 28, "TIM2_IRQHandler"),
    (16 + 38, "USART2_IRQHandler"),
    (16 + 52, "UART4_IRQHandler"),
)


NM_LINE = re.compile(r"^([0-9a-fA-F]{8,})\s+([A-Za-z])\s+(.+)$")


def run_command(command: list[str]) -> str:
    try:
        return subprocess.check_output(command, text=True, stderr=subprocess.STDOUT)
    except FileNotFoundError:
        raise RuntimeError(f"tool not found: {command[0]}") from None
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(exc.output.strip() or f"{command[0]} failed") from None


def load_symbols(elf: pathlib.Path, nm: str) -> dict[str, tuple[int, str]]:
    symbols: dict[str, tuple[int, str]] = {}
    for line in run_command([nm, "-n", str(elf)]).splitlines():
        match = NM_LINE.match(line.strip())
        if not match:
            continue
        address, kind, name = match.groups()
        symbols[name] = (int(address, 16), kind)
    return symbols


def load_isr_vector(elf: pathlib.Path, objcopy: str) -> list[int]:
    with tempfile.TemporaryDirectory() as tmpdir:
        raw = pathlib.Path(tmpdir) / "isr_vector.bin"
        run_command([
            objcopy,
            "-O",
            "binary",
            "--only-section=.isr_vector",
            str(elf),
            str(raw),
        ])
        data = raw.read_bytes()

    if not data:
        raise RuntimeError(".isr_vector section is empty or missing")
    if len(data) % 4:
        raise RuntimeError(f".isr_vector size is not word aligned: {len(data)} bytes")

    return list(struct.unpack(f"<{len(data) // 4}I", data))


def check_elf(
    elf: pathlib.Path,
    nm: str,
    objcopy: str,
    out: Callable[[str], None] = print,
) -> int:
    if not elf.exists():
        out(f"FAIL: ELF not found: {elf}")
        return 1

    try:
        symbols = load_symbols(elf, nm)
        vectors = load_isr_vector(elf, objcopy)
    except RuntimeError as exc:
        out(f"FAIL: {exc}")
        return 1

    default = symbols.get("Default_Handler")
    if default is None:
        out("FAIL: Default_Handler symbol not found")
        return 1

    default_addr = default[0]
    failures: list[str] = []

    out(f"D1S IRQ vector check: {elf}")
    out(f"  Default_Handler = 0x{default_addr:08x}")

    for vector_index, handler_name in REQUIRED_VECTORS:
        symbol = symbols.get(handler_name)
        if symbol is None:
            failures.append(f"{handler_name}: symbol missing")
            continue

        handler_addr, handler_kind = symbol
        if vector_index >= len(vectors):
            failures.append(f"{handler_name}: vector[{vector_index}] missing")
            continue

        vector_entry = vectors[vector_index]
        vector_addr = vector_entry & ~1
        expected_entry = handler_addr | 1
        status = "OK"

        if handler_addr == default_addr:
            status = "FAIL"
            failures.append(
                f"{handler_name}: symbol resolves to Default_Handler "
                f"(kind {handler_kind}, 0x{handler_addr:08x})"
            )
        elif vector_addr == default_addr:
            status = "FAIL"
            failures.append(
                f"{handler_name}: vector[{vector_index}] points to Default_Handler "
                f"(0x{vector_entry:08x})"
            )
        elif vector_entry != expected_entry:
            status = "FAIL"
            failures.append(
                f"{handler_name}: vector[{vector_index}] is 0x{vector_entry:08x}, "
                f"expected 0x{expected_entry:08x}"
            )

        out(
            f"  {status}: vector[{vector_index:02d}] {handler_name} "
            f"-> 0x{vector_entry:08x} (symbol {handler_kind} 0x{handler_addr:08x})"
        )

    if failures:
        out("FAIL: critical D1S vectors are not safe:")
        for failure in failures:
            out(f"  - {failure}")
        return 1

    out("OK: critical D1S vectors resolve to real handlers.")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "elf",
        nargs="?",
        default=".pio/build/himill_d1s/firmware.elf",
        help="ELF file to inspect",
    )
    parser.add_argument("--nm", default="arm-none-eabi-nm")
    parser.add_argument("--objcopy", default="arm-none-eabi-objcopy")
    args = parser.parse_args(argv)

    return check_elf(pathlib.Path(args.elf), args.nm, args.objcopy)


def register_platformio_post_action() -> bool:
    try:
        Import("env")  # type: ignore[name-defined]
    except NameError:
        return False

    def post_check(source, target, env):  # type: ignore[no-untyped-def]
        elf = pathlib.Path(str(target[0]))
        nm = env.subst("$NM") or "arm-none-eabi-nm"
        objcopy = env.subst("$OBJCOPY") or "arm-none-eabi-objcopy"
        if check_elf(elf, nm, objcopy) != 0:
            raise RuntimeError("D1S IRQ vector check failed")

    action = env.Action(post_check, "Checking D1S IRQ vectors $TARGET")
    env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", action)  # type: ignore[name-defined]
    return True


if not register_platformio_post_action():
    sys.exit(main(sys.argv[1:]))
