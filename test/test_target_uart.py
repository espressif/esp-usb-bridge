# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
"""Hardware target test for the ESP-Prog-2 UART serial bridge (eub_uart)."""

import random
import subprocess
from pathlib import Path

from flash_helpers import run_esptool, wait_for_port


TARGET_CHIP = 'esp32c3'
UF2_TEST_OFFSET = 0x100000
UF2_TEST_DATA_SIZE = 256
UF2_BLOCK_SIZE = 512
# First sector after the FAT16 README cluster (FIRST_ELSE_SECTOR in main/msc.c).
MSC_DATA_LBA = 35


def target_chip_id(port: str, chip: str = TARGET_CHIP) -> str:
    """Run esptool chip_id against the MCU on the Prog2 UART (ESP32-C3)."""
    result = run_esptool(
        '--chip',
        chip,
        '-p',
        port,
        'chip-id',
    )
    return result.stdout + result.stderr


def flash_uf2(image: Path, device: Path) -> None:
    """Write a UF2 image directly to the bridge mass-storage device."""
    wait_for_port(str(device))
    subprocess.run(
        [
            'dd',
            f'if={image}',
            f'of={device}',
            f'bs={UF2_BLOCK_SIZE}',
            f'seek={MSC_DATA_LBA}',
            'conv=fsync,notrunc',
        ],
        check=True,
    )


def test_uart_flash_and_chip_id(eub_port: str) -> None:
    """chip_id the ESP32-C3 through a session-flashed UART bridge."""
    output = target_chip_id(eub_port)
    assert 'ESP32-C3' in output, f'Unexpected chip_id output:\n{output}'


def test_uart_uf2_flash(eub_port: str, uf2_device: Path, tmp_path: Path) -> None:
    """Flash and verify a UF2 payload on the ESP32-C3 through the bridge MSC."""
    payload_bin = tmp_path / 'payload.bin'
    uf2_bin = tmp_path / 'payload.uf2'
    readback_bin = tmp_path / 'readback.bin'
    payload = random.randbytes(UF2_TEST_DATA_SIZE)

    payload_bin.write_bytes(payload)
    run_esptool(
        '--chip',
        TARGET_CHIP,
        'merge-bin',
        '--format',
        'uf2',
        '--output',
        str(uf2_bin),
        hex(UF2_TEST_OFFSET),
        str(payload_bin),
    )

    flash_uf2(uf2_bin, uf2_device)
    run_esptool(
        '--chip',
        TARGET_CHIP,
        '-p',
        eub_port,
        'read-flash',
        hex(UF2_TEST_OFFSET),
        hex(len(payload)),
        str(readback_bin),
    )
    assert readback_bin.read_bytes() == payload
