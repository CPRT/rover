"""
f9p_basestation.py

Example showing how to configure a u-blox ZED-F9P
receiver to operate in RTK Base Station mode (either
Survey-In or Fixed Timing Mode). This can be used to
complement PyGPSClient's NTRIP Caster functionality.

It also optionally formats a user-defined preset
configuration message string suitable for copying
and pasting into the PyGPSClient ubxpresets file.

Created on 26 Apr 2022

:author: semuadmin (Steve Smith)
:copyright: semuadmin © 2022
:license: BSD 3-Clause
"""

from serial import Serial

from pyubx2 import UBXMessage, SET, UBX_PROTOCOL, UBXReader

from time import sleep
from . import helper_functions as helpers
import argparse

TMODE_SVIN = 1
TMODE_FIXED = 2


def config_rtcm(port_type: str) -> UBXMessage:
    """
    Configure which RTCM3 messages to output.
    """

    print("\nFormatting RTCM MSGOUT CFG-VALSET message...")
    layers = helpers.ALL_LAYERS  # 1 = RAM, 2 = BBR, 4 = Flash (can be OR'd)
    transaction = 0
    cfg_data = []
    for rtcm_type in (
        "1005",
        "1077",
        "1087",
        "1097",
        "1127",
        "1230",
    ):
        cfg = f"CFG_MSGOUT_RTCM_3X_TYPE{rtcm_type}_{port_type}"
        cfg_data.append([cfg, 1])

    ubx = UBXMessage.config_set(layers, transaction, cfg_data)

    if helpers.SHOW_PRESET:
        print(
            "Set ZED-F9P RTCM3 MSGOUT Basestation, "
            f"CFG, CFG_VALSET, {ubx.payload.hex()}, 1\n"
        )

    return ubx


def config_svin(port_type: str, acc_limit: int, svin_min_dur: int) -> UBXMessage:
    """
    Configure Survey-In mode with specied accuracy limit.
    """
    print("\nFormatting SVIN TMODE CFG-VALSET message...")
    tmode = TMODE_SVIN
    layers = helpers.ALL_LAYERS
    transaction = 0
    acc_limit = int(
        acc_limit * 10
    )  # Scale mm -> 0.1 mm units (u-blox expects tenths of a millimetre for CFG_TMODE_SVIN_ACC_LIMIT)
    cfg_data = [
        ("CFG_TMODE_MODE", tmode),
        ("CFG_TMODE_SVIN_ACC_LIMIT", acc_limit),
        ("CFG_TMODE_SVIN_MIN_DUR", svin_min_dur),
        (f"CFG_MSGOUT_UBX_NAV_SVIN_{port_type}", 1),
    ]

    ubx = UBXMessage.config_set(layers, transaction, cfg_data)

    if helpers.SHOW_PRESET:
        print(
            "Set ZED-F9P to Survey-In Timing Mode Basestation, "
            f"CFG, CFG_VALSET, {ubx.payload.hex()}, 1\n"
        )

    return ubx


def parse_args():
    p = argparse.ArgumentParser(
        description="Configure a ZED-F9P receiver to operate as an RTK base station."
    )
    helpers.add_common_args(
        p,
        port_default="/dev/ttyUSB0",  # Check this, it will be different because of the ports on the base station module, Do not know how we are reading it yet
        baud_initial_default=115200,
        uart_baud_default=38400,
        timeout_default=5,
        nav_rate_default=200,
    )

    p.add_argument(
        "--port-types",
        nargs="+",
        default=["USB", "UART1"],
        help='Port types to configure, e.g. "USB UART1"',
    )
    p.add_argument("--tmode", choices=["svin", "fixed"], default="svin")
    p.add_argument("--acc-limit", type=int, default=200, help="Accuracy limit in mm")
    p.add_argument(
        "--svin-min-dur", type=int, default=600, help="Survey-in minimum duration (s)"
    )
    p.add_argument(
        "--set-uart-baud",
        type=int,
        default=None,
        help="If set, configure UART1/2 to this baud (e.g. 115200)",
    )

    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()

    PORT = args.port
    BAUD = args.baud_initial
    NAV_RATE_MS = args.nav_rate_ms
    TIMEOUT = args.timeout
    PORT_TYPES = args.port_types
    TMODE = TMODE_SVIN if args.tmode == "svin" else TMODE_FIXED
    ACC_LIMIT = args.acc_limit
    SVIN_MIN_DUR = args.svin_min_dur

    # if args.factory_reset:
    with Serial(PORT, BAUD, timeout=TIMEOUT) as stream:
        ubr = UBXReader(stream, protfilter=UBX_PROTOCOL)
        msg = helpers.factory_reset_msg()
        helpers.send_msg(stream, msg)
        sleep(1)

    # with Serial(PORT, BAUD, timeout=TIMEOUT) as stream:
    #     for port_type in PORT_TYPES:
    #         msg = config_rtcm(port_type)
    #         helpers.send_msg(stream, msg)
    #         sleep(1)
    #         msg = helpers.config_navigation_rate(NAV_RATE_MS)
    #         helpers.send_msg(stream, msg)
    #         sleep(1)
    #         if TMODE == TMODE_SVIN:
    #             msg = config_svin(port_type, ACC_LIMIT, SVIN_MIN_DUR)
    #             helpers.send_msg(stream, msg)
    #             sleep(1)
    #         else:
    #             # If fixed mode behavior needed later, insert msg creation here
    #             pass

    # # if args.set_uart_baud:
    # with Serial(PORT, BAUD, timeout=TIMEOUT) as stream:
    #     layers = helpers.ALL_LAYERS
    #     transaction = 0
    #     cfg_data = [
    #         ("CFG_UART1_BAUDRATE", 115200),
    #         ("CFG_UART2_BAUDRATE", 115200),
    #     ]
    #     msg = UBXMessage.config_set(layers, transaction, cfg_data)
    #     helpers.send_msg(stream, msg)
    #     sleep(1)
