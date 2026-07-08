#!/usr/bin/env python3
import argparse
import os
import random
import shlex
import signal
import sys

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GLib", "2.0")
from gi.repository import GLib, Gst


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run morseLEDSim + morseLED and print decoded text."
    )
    parser.add_argument("--plugin-path", default="/workspaces/rover/build/morseled")
    parser.add_argument("--message", default="SOS")
    parser.add_argument(
        "--sim-wpm", type=int, default=18, help="Simulator speed (PARIS)"
    )
    parser.add_argument(
        "--decode-wpm", type=int, default=18, help="Decoder WPM (PARIS)"
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--framerate", type=int, default=90)
    parser.add_argument("--dot-x", type=int, default=320)
    parser.add_argument("--dot-y", type=int, default=240)
    parser.add_argument("--dot-radius", type=int, default=24)
    parser.add_argument(
        "--random-dot",
        action="store_true",
        help="Randomize dot position and radius within frame-safe bounds",
    )
    parser.add_argument("--random-seed", type=int, default=None)
    parser.add_argument("--min-dot-radius", type=int, default=8)
    parser.add_argument("--max-dot-radius", type=int, default=36)
    parser.add_argument(
        "--lead-in",
        type=float,
        default=1.0,
        help="Seconds of black frames before Morse starts (improves threshold lock)",
    )
    parser.add_argument("--roi-x", type=int, default=280)
    parser.add_argument("--roi-y", type=int, default=200)
    parser.add_argument("--roi-width", type=int, default=80)
    parser.add_argument("--roi-height", type=int, default=80)
    parser.add_argument("--calibrate", action="store_true")
    parser.add_argument("--calibration-seconds", type=float, default=2.0)
    parser.add_argument("--draw-roi", action="store_true", default=True)
    parser.add_argument("--no-draw-roi", dest="draw_roi", action="store_false")
    parser.add_argument("--always-on", action="store_true")
    parser.add_argument("--loop", action="store_true", default=False)
    parser.add_argument(
        "--num-buffers", type=int, default=0, help="0 means run continuously"
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Use fakesink instead of autovideosink (headless)",
    )
    return parser.parse_args()


def build_pipeline(args: argparse.Namespace) -> Gst.Pipeline:
    sink_name = "fakesink" if args.no_display else "autovideosink"

    sim_parts = [
        "morseLEDSim",
        "name=sim",
        f"message={shlex.quote(args.message)}",
        f"wpm={args.sim_wpm}",
        f"width={args.width}",
        f"height={args.height}",
        f"framerate={args.framerate}/1",
        f"dot-x={args.dot_x}",
        f"dot-y={args.dot_y}",
        f"dot-radius={args.dot_radius}",
        f"lead-in={args.lead_in}",
        f"loop={'true' if args.loop else 'false'}",
        f"always-on={'true' if args.always_on else 'false'}",
    ]
    if args.num_buffers > 0:
        sim_parts.append(f"num-buffers={args.num_buffers}")

    dec_parts = ["morseLED", "name=decoder"]

    pipeline_str = (
        f"{' '.join(sim_parts)} ! "
        f"{' '.join(dec_parts)} ! "
        f"videoconvert name=conv ! "
        f"{sink_name} name=sink"
    )
    element = Gst.parse_launch(pipeline_str)
    if not isinstance(element, Gst.Pipeline):
        raise RuntimeError("Pipeline creation failed")
    pipeline = element

    decoder = pipeline.get_by_name("decoder")
    if decoder is None:
        raise RuntimeError("Decoder element not found in pipeline")

    decoder.set_property("start-detection", True)
    decoder.set_property("roi-x", args.roi_x)
    decoder.set_property("roi-y", args.roi_y)
    decoder.set_property("roi-width", args.roi_width)
    decoder.set_property("roi-height", args.roi_height)
    decoder.set_property("wpm", args.decode_wpm)
    decoder.set_property("calibrate", args.calibrate)
    decoder.set_property("calibration-seconds", args.calibration_seconds)
    if decoder.find_property("draw-roi") is not None:
        decoder.set_property("draw-roi", args.draw_roi)
    elif args.draw_roi:
        print(
            "warning: loaded morseLED plugin has no draw-roi property; "
            "clear GStreamer cache or verify GST_PLUGIN_PATH",
            file=sys.stderr,
        )

    return pipeline


def maybe_randomize_dot(args: argparse.Namespace) -> None:
    if not args.random_dot:
        return

    if args.random_seed is not None:
        random.seed(args.random_seed)

    min_r = max(1, min(args.min_dot_radius, args.max_dot_radius))
    max_r = max(min_r, max(args.min_dot_radius, args.max_dot_radius))
    frame_limit = max(1, min(args.width, args.height) // 3)
    max_r = min(max_r, frame_limit)
    min_r = min(min_r, max_r)

    radius = random.randint(min_r, max_r)
    min_x = radius
    max_x = max(radius, args.width - radius - 1)
    min_y = radius
    max_y = max(radius, args.height - radius - 1)

    args.dot_radius = radius
    args.dot_x = random.randint(min_x, max_x)
    args.dot_y = random.randint(min_y, max_y)


def main() -> int:
    args = parse_args()
    maybe_randomize_dot(args)

    if args.plugin_path:
        current = os.environ.get("GST_PLUGIN_PATH", "")
        os.environ["GST_PLUGIN_PATH"] = (
            f"{args.plugin_path}:{current}" if current else args.plugin_path
        )

    Gst.init(None)

    try:
        pipeline = build_pipeline(args)
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    decoder = pipeline.get_by_name("decoder")
    if decoder is None:
        print("Error: decoder element not found.", file=sys.stderr)
        return 1

    state = {"last_text": ""}

    def on_character_decoded(_element: Gst.Element, char_code: int) -> None:
        try:
            ch = chr(char_code)
        except ValueError:
            ch = "?"
        print(f"character-decoded: {ch!r}")

    def on_roi_locked(
        _element: Gst.Element, roi_x: int, roi_y: int, roi_w: int, roi_h: int
    ) -> None:
        print(f"roi-locked: x={roi_x}, y={roi_y}, w={roi_w}, h={roi_h}")

    def on_decoded_text_notify(element: Gst.Element, _pspec) -> None:
        text = element.get_property("decoded-text")
        if text != state["last_text"]:
            state["last_text"] = text
            print(f"decoded-text: {text}")

    decoder.connect("character-decoded", on_character_decoded)
    decoder.connect("roi-locked", on_roi_locked)
    decoder.connect("notify::decoded-text", on_decoded_text_notify)

    def status_tick() -> bool:
        led_on = decoder.get_property("led-on")
        decoded_text = decoder.get_property("decoded-text")
        print(f"status: led_on={led_on} text={decoded_text!r}")
        return True

    loop = GLib.MainLoop()

    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def on_bus_message(_bus: Gst.Bus, message: Gst.Message) -> None:
        msg_type = message.type
        if msg_type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"ERROR: {err}", file=sys.stderr)
            if debug:
                print(f"DEBUG: {debug}", file=sys.stderr)
            loop.quit()
        elif msg_type == Gst.MessageType.EOS:
            print("EOS reached.")
            loop.quit()

    bus.connect("message", on_bus_message)

    def stop_handler(*_args) -> None:
        loop.quit()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    print("Starting pipeline...")
    print(
        f"Dot: x={args.dot_x}, y={args.dot_y}, radius={args.dot_radius}"
        f"{' (randomized)' if args.random_dot else ''}"
    )
    print(
        f"ROI: x={args.roi_x}, y={args.roi_y}, w={args.roi_width}, h={args.roi_height}, "
        f"draw-roi={args.draw_roi}, loop={args.loop}"
    )
    if args.calibrate:
        print(f"Calibration enabled for {args.calibration_seconds:.2f}s")
    print("Press Ctrl+C to stop.")
    GLib.timeout_add(500, status_tick)

    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    finally:
        pipeline.set_state(Gst.State.NULL)
        bus.remove_signal_watch()
        print("Pipeline stopped.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
