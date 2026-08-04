# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
"""Pytest fixtures for ESP-Prog-2 target tests."""

from pathlib import Path

import pytest

from flash_helpers import flash_bridge as _flash_bridge

REPO_ROOT = Path(__file__).resolve().parents[1]
MERGED_BIN = REPO_ROOT / 'build' / 'bridge_merged.bin'


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        '--port',
        action='store',
        required=True,
        help='Serial port of the ESP-Prog-2 USB CDC interface',
    )
    parser.addoption(
        '--uf2-device',
        action='store',
        help='Block-device path of the ESP-Prog-2 mass-storage partition',
    )


@pytest.fixture(scope='session')
def eub_port(pytestconfig: pytest.Config) -> str:
    return pytestconfig.getoption('--port')


@pytest.fixture(scope='session')
def uf2_device(pytestconfig: pytest.Config) -> Path:
    device = pytestconfig.getoption('--uf2-device')
    if not device:
        pytest.fail('--uf2-device is required for UF2 target tests')
    return Path(device)


@pytest.fixture(scope='session')
def merged_bin() -> Path:
    if not MERGED_BIN.is_file():
        raise FileNotFoundError(f'Merged firmware not found at {MERGED_BIN}')
    return MERGED_BIN


@pytest.fixture(scope='session', autouse=True)
def flash_bridge(eub_port: str, merged_bin: Path) -> None:
    """Flash the bridge once per pytest session."""
    _flash_bridge(eub_port, merged_bin)
