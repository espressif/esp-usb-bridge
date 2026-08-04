# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
"""Hardware target test for the ESP-Prog-2 SWD / CMSIS-DAP bridge (eub_swd)."""

from flash_helpers import run_openocd, usb_serial_from_port


def probe_swd_stm32f4(adapter_serial: str) -> str:
    """Probe an STM32F4 (Nucleo F411RE) over the CMSIS-DAP SWD interface.

    Examines the running target without halting or resuming it.
    """
    # Pin the adapter by USB serial: OpenOCD otherwise opens the first CMSIS-DAP probe it finds.
    return run_openocd(
        '-c',
        f'adapter serial {adapter_serial}',
        '-f',
        'interface/cmsis-dap.cfg',
        '-f',
        'target/stm32f4x.cfg',
        post_commands=[
            'adapter speed 5000',
            'init',
            'targets',
            'shutdown',
        ],
    )


def test_swd_flash_and_probe(eub_port: str) -> None:
    """OpenOCD-probe the Nucleo F411RE through a session-flashed SWD bridge without stopping it."""
    output = probe_swd_stm32f4(usb_serial_from_port(eub_port))
    assert '[stm32f4x.cpu] Examination succeed' in output, (
        f'STM32F4 was not examined via SWD:\n{output}'
    )
