# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
"""Hardware target test for the ESP-Prog-2 JTAG bridge (eub_jtag)."""

from flash_helpers import run_openocd, usb_serial_from_port


def probe_jtag_esp32(adapter_serial: str) -> str:
    """Probe an ESP32 over the ESP USB Bridge JTAG interface."""
    # Pin the adapter by USB serial: several Prog-2s share VID/PID 0x303a:0x1002
    # and OpenOCD otherwise opens the first match (often the UART bench).
    return run_openocd(
        '-c',
        f'adapter serial {adapter_serial}',
        '-f',
        'board/esp32-bridge.cfg',
    )


def test_jtag_flash_and_probe(eub_port: str) -> None:
    """OpenOCD-probe the attached ESP32 through a session-flashed JTAG bridge."""
    adapter_serial = usb_serial_from_port(eub_port)
    output = probe_jtag_esp32(adapter_serial)
    assert f'serial ({adapter_serial})' in output, (
        f'OpenOCD did not open USB serial {adapter_serial}:\n{output}'
    )
    # The OpenOCD banner contains "esp32" even when the target config is wrong.
    assert '[esp32.cpu0] Examination succeed' in output, (
        f'ESP32 was not examined via JTAG:\n{output}'
    )
