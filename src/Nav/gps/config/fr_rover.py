"""
f9p_custom_config.py

Custom configuration for u-blox ZED-F9P receiver.
Configures comprehensive message output on USB, NMEA on all ports except USB,
and RTCM3 output on UART1 and UART2.

Created on 06 Oct 2025

:author: Custom Configuration
:license: BSD 3-Clause
"""

from serial import Serial
from pyubx2 import UBXMessage, SET
from time import sleep
import argparse

# use centralized helpers
from . import helper_functions as helpers


def config_usb_ubx_messages() -> UBXMessage:
    """
    Configure all UBX messages to output on USB at 5Hz.
    """
    print("\nFormatting USB UBX MSGOUT CFG-VALSET message...")
    layers = helpers.ALL_LAYERS  # RAM + BBR + Flash for persistent storage
    transaction = 0
    cfg_data = []

    # NAV messages (essential navigation data)
    nav_messages = [
        "NAV_RELPOSNED",
    ]
    ubx_messages = nav_messages.copy()

    for msg in ubx_messages:
        cfg = f"CFG_MSGOUT_UBX_{msg}_USB"
        cfg_data.append((cfg, 1))  # 1 = output enabled (per-epoch semantics)

    ubx = UBXMessage.config_set(layers, transaction, cfg_data)

    if helpers.SHOW_PRESET:
        print(
            "Set ZED-F9P USB UBX Messages, "
            f"CFG, CFG_VALSET, {ubx.payload.hex()}, 1\n"
        )

    return ubx


def config_nmea_messages(port_type: str) -> UBXMessage:
    """
    Configure NMEA messages to output on specified port at 5Hz.
    """
    print(f"\nFormatting NMEA MSGOUT CFG-VALSET message for {port_type}...")
    layers = helpers.ALL_LAYERS  # RAM + BBR + Flash for persistent storage
    transaction = 0
    cfg_data = []

    for nmea_type in ("GGA", "GLL", "GSA", "GSV", "RMC", "VTG"):
        cfg = f"CFG_MSGOUT_NMEA_ID_{nmea_type}_{port_type}"
        cfg_data.append((cfg, 5))  # 5 = rate indicator consistent with other scripts

    ubx = UBXMessage.config_set(layers, transaction, cfg_data)

    if helpers.SHOW_PRESET:
        print(
            f"Set ZED-F9P NMEA Messages {port_type}, "
            f"CFG, CFG_VALSET, {ubx.payload.hex()}, 1\n"
        )

    return ubx


def config_rtcm_messages(port_type: str) -> UBXMessage:
    """
    Configure RTCM3 messages to output on specified port at 5Hz.
    """
    print(f"\nFormatting RTCM3 MSGOUT CFG-VALSET message for {port_type}...")
    layers = helpers.ALL_LAYERS  # RAM + BBR + Flash for persistent storage
    transaction = 0
    cfg_data = []

    for rtcm_type in ("1077", "1087", "1097", "1127", "1230", "4072_0", "4072_1"):
        cfg = f"CFG_MSGOUT_RTCM_3X_TYPE{rtcm_type}_{port_type}"
        cfg_data.append((cfg, 1))  # 1 = output enabled (per-epoch semantics)

    ubx = UBXMessage.config_set(layers, transaction, cfg_data)

    if helpers.SHOW_PRESET:
        print(
            f"Set ZED-F9P RTCM3 Messages {port_type}, "
            f"CFG, CFG_VALSET, {ubx.payload.hex()}, 1\n"
        )

    return ubx


def config_uart_settings(uart_num: int, baudrate: int) -> UBXMessage:
    """
    Configure UART baud rate and protocol settings.
    No input protocols, RTCM3 output only.
    """
    print(f"\nFormatting UART{uart_num} settings CFG-VALSET message...")
    layers = helpers.ALL_LAYERS  # RAM + BBR + Flash for persistent storage
    transaction = 0
    cfg_data = [
        (f"CFG_UART{uart_num}_BAUDRATE", baudrate),
        (f"CFG_UART{uart_num}INPROT_UBX", 0),  # Disable UBX input
        (f"CFG_UART{uart_num}INPROT_NMEA", 0),  # Disable NMEA input
        (f"CFG_UART{uart_num}INPROT_RTCM3X", 0),  # Disable RTCM3 input
        (f"CFG_UART{uart_num}OUTPROT_UBX", 0),  # Disable UBX output
        (f"CFG_UART{uart_num}OUTPROT_NMEA", 0),  # Disable NMEA output
        (f"CFG_UART{uart_num}OUTPROT_RTCM3X", 1),  # Enable RTCM3 output
    ]

    ubx = UBXMessage.config_set(layers, transaction, cfg_data)

    if helpers.SHOW_PRESET:
        print(
            f"Set UART{uart_num} to {baudrate} baud with RTCM3 output only, "
            f"CFG, CFG_VALSET, {ubx.payload.hex()}, 1\n"
        )

    return ubx


def parse_args():
    p = argparse.ArgumentParser(
        description="Configure a ZED-F9P receiver (rover profile)."
    )
    helpers.add_common_args(
        p,
        port_default="/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00",  # /dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00
        baud_initial_default=38400,
        baud_after_reset_default=38400,
        uart_baud_default=115200,
        timeout_default=5,
        nav_rate_default=200,
    )

    # Rover-specific args
    p.add_argument(
        "--min-time", type=int, default=600, help="Survey-in minimum duration (s)"
    )
    p.add_argument("--acc-limit", type=int, default=200, help="Accuracy limit in mm")

    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()

    PORT = args.port
    BAUD_INITIAL = args.baud_initial
    BAUD_AFTER_RESET = args.baud_after_reset
    UART_BAUD = args.uart_baud
    TIMEOUT = args.timeout
    NAV_RATE_MS = args.nav_rate_ms
    MIN_TIME = args.min_time
    ACC_LIMIT = args.acc_limit

    # let helper module show presets when requested
    helpers.SHOW_PRESET = args.show_preset

    print("\n" + "=" * 70)
    print("ZED-F9P CUSTOM CONFIGURATION (ROVER)")
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
        helpers.send_msg(stream, config_uart_settings(1, UART_BAUD))
        sleep(0.5)

        # Configure UART2 settings
        print("\n--- Configuring UART2 ---")
        helpers.send_msg(stream, config_uart_settings(2, UART_BAUD))
        sleep(0.5)

        # Configure USB UBX messages
        print("\n--- Configuring USB UBX Messages ---")
        helpers.send_msg(stream, config_usb_ubx_messages())
        sleep(0.5)

        # Configure NMEA on UART1
        print("\n--- Configuring NMEA on UART1 ---")
        helpers.send_msg(stream, config_nmea_messages("UART1"))
        sleep(0.5)

        # Configure NMEA on UART2
        print("\n--- Configuring NMEA on UART2 ---")
        helpers.send_msg(stream, config_nmea_messages("UART2"))
        sleep(0.5)

        # Configure RTCM3 on UART1
        print("\n--- Configuring RTCM3 on UART1 ---")
        helpers.send_msg(stream, config_rtcm_messages("UART1"))
        sleep(0.5)

        # Configure RTCM3 on UART2
        print("\n--- Configuring RTCM3 on UART2 ---")
        helpers.send_msg(stream, config_rtcm_messages("UART2"))
        sleep(0.5)

        # Set to Fixed/Survey-In Mode
        # print("\n--- Configuring Fixed/Survey-In Mode ---")
        # helpers.send_msg(stream, config_tmode(ACC_LIMIT, MIN_TIME))
        # sleep(0.5)

    print("\nConfiguration complete.")
