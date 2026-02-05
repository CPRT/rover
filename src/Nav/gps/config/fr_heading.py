"""
f9p_relposned_config.py

Custom configuration for u-blox ZED-F9P receiver.
Configures NAV-RELPOSNED output on USB with specific UART settings.

Created on 08 Oct 2025

:author: Custom Configuration
:license: BSD 3-Clause
"""

from serial import Serial
from pyubx2 import UBXMessage, SET
from time import sleep
import argparse

# use the helper module so we can change SHOW_PRESET at runtime
import helper_functions as helpers


def config_usb_relposned() -> UBXMessage:
    """
    Configure NAV-RELPOSNED to output on USB at 5Hz.
    """
    print("\nFormatting USB NAV-RELPOSNED MSGOUT CFG-VALSET message...")
    layers = helpers.ALL_LAYERS  # RAM + BBR + Flash for persistent storage
    transaction = 0
    cfg_data = [
        (
            "CFG_MSGOUT_UBX_NAV_RELPOSNED_USB",
            1,
        ),  # 1 = 1 time per epoch. epoch is how many times the NAV_RATE happens,
        # ie 5 would be once every 5 epochs, with a NAV_RATE of 200ms it would be triggering every 1 second.
    ]

    ubx = UBXMessage.config_set(layers, transaction, cfg_data)

    if helpers.SHOW_PRESET:
        print(
            "Set ZED-F9P NAV-RELPOSNED USB, "
            f"CFG, CFG_VALSET, {ubx.payload.hex()}, 1\n"
        )

    return ubx


def config_uart1_settings(baudrate: int) -> UBXMessage:
    """
    Configure UART1: All protocols input and output.
    """
    print(f"\nFormatting UART1 settings CFG-VALSET message...")
    layers = helpers.ALL_LAYERS  # RAM + BBR + Flash for persistent storage
    transaction = 0
    cfg_data = [
        ("CFG_UART1_BAUDRATE", baudrate),
        # Input protocols
        ("CFG_UART1INPROT_UBX", 1),  # Enable UBX input
        ("CFG_UART1INPROT_NMEA", 0),  # Enable NMEA input
        ("CFG_UART1INPROT_RTCM3X", 1),  # Enable RTCM3 input
        # Output protocols
        ("CFG_UART1OUTPROT_UBX", 1),  # Enable UBX output
        ("CFG_UART1OUTPROT_NMEA", 1),  # Enable NMEA output
        ("CFG_UART1OUTPROT_RTCM3X", 1),  # Enable RTCM3 output
    ]

    ubx = UBXMessage.config_set(layers, transaction, cfg_data)

    if helpers.SHOW_PRESET:
        print(
            f"Set UART1 to {baudrate} baud with all protocols, "
            f"CFG, CFG_VALSET, {ubx.payload.hex()}, 1\n"
        )

    return ubx


def config_uart2_settings(baudrate: int) -> UBXMessage:
    """
    Configure UART2: UBX input, RTCM3 output only.
    """
    print(f"\nFormatting UART2 settings CFG-VALSET message...")
    layers = helpers.ALL_LAYERS  # RAM + BBR + Flash for persistent storage
    transaction = 0
    cfg_data = [
        ("CFG_UART2_BAUDRATE", baudrate),
        # Input protocols
        ("CFG_UART2INPROT_UBX", 1),  # Enable UBX input
        ("CFG_UART2INPROT_NMEA", 0),  # Disable NMEA input
        ("CFG_UART2INPROT_RTCM3X", 0),  # Disable RTCM3 input
        # Output protocols
        ("CFG_UART2OUTPROT_UBX", 0),  # Disable UBX output
        ("CFG_UART2OUTPROT_NMEA", 0),  # Disable NMEA output
        ("CFG_UART2OUTPROT_RTCM3X", 1),  # Enable RTCM3 output
    ]

    ubx = UBXMessage.config_set(layers, transaction, cfg_data)

    if helpers.SHOW_PRESET:
        print(
            f"Set UART2 to {baudrate} baud with UBX input, RTCM3 output, "
            f"CFG, CFG_VALSET, {ubx.payload.hex()}, 1\n"
        )

    return ubx


def parse_args():
    p = argparse.ArgumentParser(
        description="Configure a ZED-F9P receiver for NAV-RELPOSNED + UART settings."
    )
    helpers.add_common_args(
        p,
        port_default="/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D30EFLJN-if00-port0",
        baud_initial_default=38400,
        baud_after_reset_default=115200,
        uart_baud_default=115200,
        timeout_default=5,
        nav_rate_default=200,
    )
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()

    PORT = args.port
    BAUD_INITIAL = args.baud_initial
    BAUD_AFTER_RESET = args.baud_after_reset
    UART_BAUD = args.uart_baud
    TIMEOUT = args.timeout
    NAV_RATE_MS = args.nav_rate_ms

    # allow helper module to show presets when requested
    helpers.SHOW_PRESET = args.show_preset

    print("\n" + "=" * 70)
    print("ZED-F9P NAV-RELPOSNED CONFIGURATION")
    print("=" * 70)

    # Step 1: Factory Reset (optional)
    if args.factory_reset:
        print("\n[STEP 1] Performing factory reset...")
        with Serial(PORT, BAUD_INITIAL, timeout=TIMEOUT) as stream:
            msg = helpers.factory_reset_msg()
            helpers.send_msg(stream, msg)
            print("Factory reset command sent. Waiting for device to reset...")
            sleep(3)  # Wait for reset to complete

    # Step 2: Configure receiver
    print(f"\n[STEP 2] Configuring receiver on {PORT} @ {BAUD_AFTER_RESET:,} baud...")
    with Serial(PORT, BAUD_AFTER_RESET, timeout=TIMEOUT) as stream:

        # Set navigation rate
        print("\n--- Setting Navigation Rate ---")
        msg = helpers.config_navigation_rate(NAV_RATE_MS)
        helpers.send_msg(stream, msg)
        sleep(0.5)

        # Configure UART1 settings
        print("\n--- Configuring UART1 ---")
        msg = config_uart1_settings(UART_BAUD)
        helpers.send_msg(stream, msg)
        sleep(0.5)

        # Configure UART2 settings
        print("\n--- Configuring UART2 ---")
        msg = config_uart2_settings(UART_BAUD)
        helpers.send_msg(stream, msg)
        sleep(0.5)

        # Configure USB NAV-RELPOSNED output
        print("\n--- Configuring USB NAV-RELPOSNED Output ---")
        msg = config_usb_relposned()
        helpers.send_msg(stream, msg)
        sleep(0.5)
