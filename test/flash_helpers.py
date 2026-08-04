# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
"""Flash and probe helpers for ESP-Prog-2 hardware tests."""

import os
import subprocess
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

DEFAULT_MAGIC_BAUD = 1200
DEFAULT_SETTLE_S = 2.0
BRIDGE_CHIP = 'esp32s3'
DEFAULT_OPENOCD_TIMEOUT_S = 30.0


def enter_download_mode(
    port: str,
    magic_baud: int = DEFAULT_MAGIC_BAUD,
    settle_s: float = DEFAULT_SETTLE_S,
) -> None:
    """Arm download mode at magic baud, then clear DTR on close to fire the reset.

    Matches the host sequence documented in the project README: open at the
    configured magic baud (DTR asserted by pyserial), then close (DTR cleared).
    """
    with serial.Serial(port, magic_baud):
        pass
    time.sleep(settle_s)


def wait_for_port(port: str, timeout_s: float = 30.0, poll_s: float = 0.5) -> None:
    """Wait until the serial device node exists again after USB re-enumeration."""
    deadline = time.monotonic() + timeout_s
    path = Path(port)
    while time.monotonic() < deadline:
        if path.exists():
            return
        time.sleep(poll_s)
    raise TimeoutError(f'Serial port {port} did not reappear within {timeout_s}s')


def usb_serial_from_port(port: str) -> str:
    """Return the USB iSerial of the composite device that owns ``port``.

    Udev symlinks such as ``/dev/serial_ports/prog2_jtag`` are resolved because
    ``comports()`` lists only the real device nodes. OpenOCD matches this
    string with ``adapter serial`` when several ``0x303a:0x1002`` bridges are
    present.
    """
    device = os.path.realpath(port)
    for info in list_ports.comports():
        if os.path.realpath(info.device) == device and info.serial_number:
            return info.serial_number
    raise FileNotFoundError(f'USB serial not found for {port}')


def _format_cmd_output(stdout: str, stderr: str) -> str:
    parts = []
    if stdout:
        parts.append(stdout.rstrip())
    if stderr:
        parts.append(stderr.rstrip())
    return '\n'.join(parts) if parts else '(no output)'


def run_esptool(*args: str) -> subprocess.CompletedProcess[str]:
    cmd = [sys.executable, '-m', 'esptool', *args]
    result = subprocess.run(
        cmd,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f'esptool failed (exit {result.returncode}): {" ".join(cmd)}\n'
            f'{_format_cmd_output(result.stdout, result.stderr)}'
        )
    print(f'$ {" ".join(cmd)}\n{_format_cmd_output(result.stdout, result.stderr)}')
    return result


def flash_bridge(port: str, image: Path) -> None:
    """Enter download mode and flash the bridge (ESP32-S3) merged image."""
    enter_download_mode(port)
    wait_for_port(port)
    run_esptool(
        '--chip',
        BRIDGE_CHIP,
        '-p',
        port,
        'write-flash',
        '0x0',
        str(image),
    )
    # Application reboot re-enumerates the CDC interface.
    wait_for_port(port)
    # Give TinyUSB / serial bridge a moment to come up before talking to the target.
    time.sleep(2.0)


def run_openocd(
    *args: str,
    timeout_s: float = DEFAULT_OPENOCD_TIMEOUT_S,
    pre_commands: list[str] | None = None,
    post_commands: list[str] | None = None,
) -> str:
    """Run openocd, capture diagnostics, and fail cleanly on timeout/non-zero exit.

    ``pre_commands`` are emitted before ``args`` (useful for adapter overrides).
    ``post_commands`` default to ``init`` + ``shutdown`` so the process exits.
    """
    cmd = ['openocd']
    for tcl in pre_commands or []:
        cmd.extend(['-c', tcl])
    cmd.extend(args)

    trailing = post_commands if post_commands is not None else ['init', 'shutdown']
    for tcl in trailing:
        cmd.extend(['-c', tcl])

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        output = _format_cmd_output(exc.stdout or '', exc.stderr or '')
        raise TimeoutError(
            f'openocd timed out after {timeout_s}s\nCommand: {" ".join(cmd)}\n{output}'
        ) from exc

    output = _format_cmd_output(result.stdout, result.stderr)
    if result.returncode != 0:
        raise RuntimeError(
            f'openocd failed (exit {result.returncode}): {" ".join(cmd)}\n{output}'
        )
    print(f'$ {" ".join(cmd)}\n{output}')
    return output
