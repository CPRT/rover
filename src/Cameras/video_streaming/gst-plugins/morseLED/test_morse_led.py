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
        description="Run morseLED (sim or file source) and print decoded text."
    )
    parser.add_argument("--plugin-path", default="/workspaces/rover/build/morseled")
    parser.add_argument(
        "--input",
        default=None,
        help="Video file path (e.g. tomas.ts). If set, skip morseLEDSim.",
    )
    parser.add_argument("--message", default="SOS")
    parser.add_argument(
        "--sim-wpm", type=int, default=18, help="Simulator speed (PARIS)"
    )
    parser.add_argument(
        "--decode-wpm", type=int, default=18, help="Decoder WPM (PARIS)"
    )
    parser.add_argument(
        "--gap-detect-ratio",
        type=float,
        default=None,
        help="Fraction of nominal letter/word gap used as detection threshold",
    )
    parser.add_argument(
        "--min-transition-units",
        type=float,
        default=None,
        help="Min dit units a candidate on/off state must persist",
    )
    parser.add_argument(
        "--on-margin",
        type=float,
        default=None,
        help="Red-dominance margin above baseline to treat LED as on",
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
    parser.add_argument(
        "--metric-plot",
        default=None,
        help="Write metric-vs-time plot (.ppm/.csv); .png also renders via matplotlib",
    )
    return parser.parse_args()


def configure_decoder(decoder: Gst.Element, args: argparse.Namespace) -> None:
    decoder.set_property("start-detection", True)
    decoder.set_property("roi-x", args.roi_x)
    decoder.set_property("roi-y", args.roi_y)
    decoder.set_property("roi-width", args.roi_width)
    decoder.set_property("roi-height", args.roi_height)
    decoder.set_property("wpm", args.decode_wpm)
    decoder.set_property("calibrate", args.calibrate)
    decoder.set_property("calibration-seconds", args.calibration_seconds)
    if args.gap_detect_ratio is not None:
        decoder.set_property("gap-detect-ratio", args.gap_detect_ratio)
    if args.min_transition_units is not None:
        decoder.set_property("min-transition-units", args.min_transition_units)
    if args.on_margin is not None:
        decoder.set_property("on-margin", args.on_margin)
    if args.metric_plot:
        decoder.set_property("metric-plot-path", args.metric_plot)
    if decoder.find_property("draw-roi") is not None:
        decoder.set_property("draw-roi", args.draw_roi)
    elif args.draw_roi:
        print(
            "warning: loaded morseLED plugin has no draw-roi property; "
            "clear GStreamer cache or verify GST_PLUGIN_PATH",
            file=sys.stderr,
        )


def build_pipeline(args: argparse.Namespace) -> Gst.Pipeline:
    sink_name = "fakesink" if args.no_display else "autovideosink"
    dec_parts = ["morseLED", "name=decoder"]

    if args.input:
        uri = args.input
        if not uri.startswith(("file://", "http://", "https://", "rtsp://")):
            uri = "file://" + os.path.abspath(uri)
        pipeline_str = (
            f"uridecodebin name=src uri={shlex.quote(uri)} ! "
            f"videoconvert name=preconv ! video/x-raw,format=RGB ! "
            f"{' '.join(dec_parts)} ! "
            f"videoconvert name=conv ! "
            f"{sink_name} name=sink"
        )
    else:
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

    configure_decoder(decoder, args)
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


def render_metric_png_from_csv(plot_path: str) -> None:
    """If plot_path ends with .png, render a nicer PNG from the sidecar CSV."""
    if not plot_path.lower().endswith(".png"):
        return
    csv_path = os.path.splitext(plot_path)[0] + ".csv"
    if not os.path.isfile(csv_path):
        print(f"warning: metric CSV not found at {csv_path}", file=sys.stderr)
        return
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print(
            "warning: matplotlib not available; leaving .ppm/.csv only",
            file=sys.stderr,
        )
        return

    times = []
    metrics = []
    thr_on = []
    thr_off = []
    led_on = []
    with open(csv_path, encoding="utf-8") as handle:
        next(handle, None)  # header
        for line in handle:
            parts = line.strip().split(",")
            if len(parts) < 5:
                continue
            times.append(float(parts[0]))
            metrics.append(float(parts[1]))
            thr_on.append(float(parts[2]))
            thr_off.append(float(parts[3]))
            led_on.append(int(parts[4]))

    if not times:
        return

    fig, ax = plt.subplots(figsize=(12, 4), dpi=120)
    t0 = times[0]
    t_rel = [t - t0 for t in times]
    ax.plot(t_rel, thr_on, color="#50a0ff", linewidth=1.0, label="threshold_on")
    ax.plot(t_rel, thr_off, color="#ffb43c", linewidth=1.0, label="threshold_off")
    ax.plot(t_rel, metrics, color="#50dc78", linewidth=1.2, label="metric")
    ax.fill_between(
        t_rel,
        0,
        1,
        where=[bool(v) for v in led_on],
        color="#c84444",
        alpha=0.2,
        label="led_on",
    )
    ax.set_ylim(-0.02, 1.02)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("ROI lit fraction")
    ax.set_title("morseLED metric vs time")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(plot_path)
    plt.close(fig)
    print(f"Wrote metric PNG: {plot_path}")


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
    if args.input:
        print(f"Input: {args.input}")
    else:
        print(
            f"Dot: x={args.dot_x}, y={args.dot_y}, radius={args.dot_radius}"
            f"{' (randomized)' if args.random_dot else ''}"
        )
    print(
        f"ROI: x={args.roi_x}, y={args.roi_y}, w={args.roi_width}, h={args.roi_height}, "
        f"draw-roi={args.draw_roi}, loop={args.loop}, decode-wpm={args.decode_wpm}"
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
        if args.metric_plot:
            render_metric_png_from_csv(args.metric_plot)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
