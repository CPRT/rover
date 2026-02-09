"""
Shared helper utilities for u-blox configuration scripts.

Centralises common functions used by:
 - fr_basestation.py
 - fr_heading.py
 - fr_rover.py

Provides:
 - send_msg(serial_out, ubx)
 - factory_reset_msg()
 - config_navigation_rate(rate_ms)
 - read_messages(stream, ubxreader)
 - constants: SHOW_PRESET, ALL_LAYERS
"""

from serial import Serial
from pyubx2 import UBXMessage, SET
import argparse

# Control printing of preset payloads (default: off)
SHOW_PRESET = False

# Layer bitflags for UBX CFG operations
# 1 = RAM, 2 = BBR, 4 = Flash (can be OR'd)
LAYER_RAM = 1
LAYER_BBR = 2
LAYER_FLASH = 4
ALL_LAYERS = LAYER_RAM | LAYER_BBR | LAYER_FLASH


def send_msg(serial_out: Serial, ubx: UBXMessage) -> None:
    """
    Send config message to receiver and print a short log.
    """
    print("Sending configuration message to receiver...")
    print(ubx)
    serial_out.write(ubx.serialize())


def factory_reset_msg() -> UBXMessage:
    """Build and return a CFG-CFG factory reset UBXMessage (does not send)."""
    return UBXMessage(
        "CFG",
        "CFG-CFG",
        SET,
        clearMask=b"\x1f\x1f\x00\x00",  # clear everything
        loadMask=b"\x1f\x1f\x00\x00",  # reload everything
        devBBR=1,  # clear from battery-backed RAM
        devFlash=1,  # clear from flash memory
        devEEPROM=1,  # clear from EEPROM memory
    )


def config_navigation_rate(rate_ms: int = 1000) -> UBXMessage:
    """
    Configure navigation measurement rate (CFG_RATE_MEAS and CFG_RATE_NAV).
    rate_ms: measurement rate in milliseconds (eg. 200 for 5Hz).
    """
    layers = ALL_LAYERS
    transaction = 0
    cfg_data = [
        ("CFG_RATE_MEAS", int(rate_ms)),
        ("CFG_RATE_NAV", 1),
    ]
    ubx = UBXMessage.config_set(layers, transaction, cfg_data)

    if SHOW_PRESET:
        hz = 1000 / int(rate_ms) if rate_ms else 0
        print(
            f"Set Navigation Rate to {hz}Hz, CFG, CFG_VALSET, {ubx.payload.hex()}, 1\n"
        )

    return ubx


def add_common_args(
    parser: argparse.ArgumentParser,
    *,
    port_default: str = "/dev/ttyACM0",
    baud_initial_default: int = 38400,
    baud_after_reset_default: int | None = None,
    uart_baud_default: int = 115200,
    timeout_default: float = 5.0,
    nav_rate_default: int = 200,
) -> argparse.ArgumentParser:
    """Add shared CLI arguments to an existing ArgumentParser.

    Caller can override `port_default` to keep script-specific defaults.
    Returns the same parser for convenience.
    """
    if port_default is None:
        parser.add_argument("--port", "-p", default=None)
    else:
        parser.add_argument("--port", "-p", default=port_default)

    parser.add_argument(
        "--baud-initial",
        "-bi",
        type=int,
        default=baud_initial_default,
        help="Baud for initial connection (before reset)",
    )
    parser.add_argument(
        "--baud-after-reset",
        "-ba",
        type=int,
        default=(
            baud_after_reset_default
            if baud_after_reset_default is not None
            else baud_initial_default
        ),
        help="Baud for configuration after reset",
    )
    parser.add_argument(
        "--uart-baud",
        "-u",
        type=int,
        default=uart_baud_default,
        help="Target UART1/2 baud rate",
    )
    parser.add_argument(
        "--timeout",
        "-T",
        type=float,
        default=timeout_default,
        help="Serial timeout (s)",
    )
    parser.add_argument(
        "--nav-rate-ms",
        type=int,
        default=nav_rate_default,
        help="Navigation rate in ms (eg. 200 = 5Hz)",
    )
    parser.add_argument(
        "--show-preset", action="store_true", help="Print preset payloads"
    )
    parser.add_argument(
        "--factory-reset",
        action="store_true",
        help="Send factory reset before configuring",
    )

    return parser
