#!/usr/bin/env python3
"""
video_server.py - Video Streaming Server for TELBot Logitech C920r Webcam
Wraps uStreamer and applies optimal V4L2 camera hardware presets for mobile robot teleoperation.
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Optional

# Add project root directory to sys.path so config.py can be imported from any working directory
ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))
from config import Camera as CameraConfig

# Pre-tuned presets optimized for C920r 16:9 sensor & teleoperation bandwidth limits
PRESETS = {
    "720p15": {
        "width": 1280,
        "height": 720,
        "fps": 15,
        "format": "MJPEG",
        "encoder": "HW",
        "desc": "Medium HD (1280x720 @ 15 fps, ~9-12 Mbps) - Recommended default for 50 Mbps Wi-Fi",
    },
    "480p24": {
        "width": 864,
        "height": 480,
        "fps": 24,
        "format": "MJPEG",
        "encoder": "HW",
        "desc": "16:9 Smooth (864x480 @ 24 fps, ~6-8 Mbps) - Wide field-of-view, zero lag spikes",
    },
    "360p30": {
        "width": 640,
        "height": 360,
        "fps": 30,
        "format": "MJPEG",
        "encoder": "HW",
        "desc": "Ultra-Low Bandwidth (640x360 @ 30 fps, ~3-4 Mbps) - For long range / noisy Wi-Fi",
    },
    "720p30": {
        "width": 1280,
        "height": 720,
        "fps": 30,
        "format": "MJPEG",
        "encoder": "HW",
        "desc": "Full HD 30 (1280x720 @ 30 fps, ~18-25 Mbps) - Crisp motion, needs strong Wi-Fi",
    },
    "1080p30": {
        "width": 1920,
        "height": 1080,
        "fps": 30,
        "format": "MJPEG",
        "encoder": "HW",
        "desc": "Full 1080p (1920x1080 @ 30 fps, ~25-35 Mbps) - Maximum detail",
    },
    "480p30": {
        "width": 640,
        "height": 480,
        "fps": 30,
        "format": "MJPEG",
        "encoder": "HW",
        "desc": "Standard 4:3 (640x480 @ 30 fps, ~6-9 Mbps) - Classic square frame",
    },
}


def find_camera_device(preferred: str, fallback: str) -> Optional[str]:
    """
    Finds an existing USB camera device, preferring the stable USB-by-id symlink.
    Returns None if no valid capture device is detected.
    """
    if os.path.exists(preferred):
        return preferred
    if os.path.exists(fallback):
        return fallback

    # Scan /sys/class/video4linux/ for actual USB webcam devices (ignore internal RPi ISP codecs)
    v4l_dir = Path("/sys/class/video4linux")
    if v4l_dir.exists():
        for dev in sorted(v4l_dir.glob("video*")):
            name_file = dev / "name"
            if name_file.exists():
                name = name_file.read_text().strip().lower()
                # Skip internal RPi 5 ISP / decoder endpoints
                if any(k in name for k in ("pisp", "hevc", "codec", "unicam", "bcm2835")):
                    continue
                dev_path = f"/dev/{dev.name}"
                if os.path.exists(dev_path):
                    return dev_path

    return None


def apply_camera_controls(device: str, autofocus: bool, focus_val: int, dynamic_fps: bool) -> None:
    """Applies V4L2 hardware controls sequentially to avoid UVC firmware I/O race conditions."""
    if not shutil.which("v4l2-ctl"):
        print("[WARN] v4l2-ctl not found. Skipping hardware control tuning.", file=sys.stderr)
        return

    print(f"[*] Applying hardware tuning to {device}:")
    try:
        if autofocus:
            print("    - Autofocus: ENABLED")
            subprocess.run(["v4l2-ctl", "-d", device, "-c", "focus_automatic_continuous=1"], check=False)
        else:
            print(f"    - Autofocus: DISABLED (Locked to {focus_val} / Infinity)")
            subprocess.run(["v4l2-ctl", "-d", device, "-c", "focus_automatic_continuous=0"], check=False)
            subprocess.run(["v4l2-ctl", "-d", device, "-c", f"focus_absolute={focus_val}"], check=False)

        if dynamic_fps:
            print("    - Low-light dynamic framerate: ENABLED")
            subprocess.run(["v4l2-ctl", "-d", device, "-c", "exposure_dynamic_framerate=1"], check=False)
        else:
            print("    - Low-light dynamic framerate: DISABLED (Enforcing constant FPS)")
            subprocess.run(["v4l2-ctl", "-d", device, "-c", "exposure_dynamic_framerate=0"], check=False)
    except Exception as e:
        print(f"[WARN] Failed to apply some V4L2 controls: {e}", file=sys.stderr)


def print_presets() -> None:
    print("\nAvailable Video Quality Presets:")
    print("=" * 78)
    for name, p in PRESETS.items():
        default_tag = " [DEFAULT]" if name == CameraConfig.DEFAULT_PRESET else ""
        print(f"  {name:<9}{default_tag}")
        print(f"      Resolution: {p['width']}x{p['height']} @ {p['fps']} fps")
        print(f"      Profile:    {p['desc']}")
    print("=" * 78)
    print("Usage example:")
    print("  ./video_server.py --preset 480p24")
    print("  ./video_server.py --preset 360p30 --port 8002\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="TELBot Logitech C920r Video Streaming Server (uStreamer)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--preset",
        "-P",
        choices=list(PRESETS.keys()),
        default=CameraConfig.DEFAULT_PRESET,
        help=f"Video preset profile (default: {CameraConfig.DEFAULT_PRESET})",
    )
    parser.add_argument(
        "--list-presets",
        action="store_true",
        help="List all preset profiles with details and exit",
    )
    parser.add_argument(
        "--resolution",
        "-r",
        help="Override resolution formatted as WIDTHxHEIGHT (e.g. 1280x720)",
    )
    parser.add_argument(
        "--fps",
        "-f",
        type=int,
        help="Override target frames per second (e.g. 15, 24, 30)",
    )
    parser.add_argument(
        "--device",
        "-d",
        default=CameraConfig.DEVICE,
        help=f"V4L2 camera device path (default: {CameraConfig.DEVICE})",
    )
    parser.add_argument(
        "--port",
        "-p",
        type=int,
        default=CameraConfig.PORT,
        help=f"HTTP streaming port (default: {CameraConfig.PORT})",
    )
    parser.add_argument(
        "--host",
        default=CameraConfig.HOST,
        help=f"HTTP bind host (default: {CameraConfig.HOST})",
    )
    parser.add_argument(
        "--autofocus",
        action="store_true",
        help="Enable continuous autofocus (default: disabled to prevent hunting while moving)",
    )
    parser.add_argument(
        "--focus",
        type=int,
        default=0,
        help="Manual focus absolute value when autofocus is disabled (0=Infinity, 250=Macro; default: 0)",
    )
    parser.add_argument(
        "--dynamic-fps",
        action="store_true",
        help="Allow camera to drop framerate in dark lighting (default: disabled to guarantee constant FPS)",
    )
    parser.add_argument(
        "--no-tune",
        action="store_true",
        help="Skip applying V4L2 camera hardware tuning",
    )
    parser.add_argument(
        "--kill-existing",
        "-k",
        action="store_true",
        help="Kill any running ustreamer instances before starting",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.list_presets:
        print_presets()
        sys.exit(0)

    if not shutil.which("ustreamer"):
        print("[ERROR] 'ustreamer' is not installed.", file=sys.stderr)
        print("Please run: sudo apt install -y ustreamer", file=sys.stderr)
        sys.exit(1)

    if args.kill_existing:
        print("[*] Terminating existing ustreamer instances...")
        subprocess.run(["pkill", "-9", "ustreamer"], check=False)

    # Resolve preset configuration
    preset_data = PRESETS[args.preset]
    width = preset_data["width"]
    height = preset_data["height"]
    fps = preset_data["fps"]

    # Allow custom CLI overrides
    if args.resolution:
        try:
            w_str, h_str = args.resolution.lower().split("x")
            width, height = int(w_str), int(h_str)
        except ValueError:
            print(f"[ERROR] Invalid resolution format '{args.resolution}'. Use WIDTHxHEIGHT (e.g. 1280x720).", file=sys.stderr)
            sys.exit(1)

    if args.fps:
        fps = args.fps

    device = find_camera_device(args.device, CameraConfig.FALLBACK_DEVICE)
    if device is None or not os.path.exists(device):
        print(f"[ERROR] No USB camera detected! Checked '{args.device}' and '{CameraConfig.FALLBACK_DEVICE}'.", file=sys.stderr)
        print("Please make sure the webcam is plugged in and recognized by Linux (check `lsusb` or `v4l2-ctl --list-devices`).", file=sys.stderr)
        sys.exit(1)

    # 1. Apply hardware camera controls
    if not args.no_tune:
        apply_camera_controls(
            device=device,
            autofocus=args.autofocus,
            focus_val=args.focus,
            dynamic_fps=args.dynamic_fps,
        )

    # 2. Build ustreamer command
    ustreamer_bin = shutil.which("ustreamer")
    if not ustreamer_bin:
        print(f"[ERROR] Failed to find ustreamer. Check if it's inside PATH.")
        sys.exit(1)

    cmd = [
        ustreamer_bin,
        f"--device={device}",
        f"--format={preset_data['format']}",
        f"--encoder={preset_data['encoder']}",
        f"--resolution={width}x{height}",
        f"--desired-fps={fps}",
        f"--buffers={CameraConfig.BUFFERS}",
        f"--host={args.host}",
        f"--port={args.port}",
        f"--allow-origin={CameraConfig.ALLOW_ORIGIN}",
    ]

    if CameraConfig.TCP_NODELAY:
        cmd.append("--tcp-nodelay")

    print("\n" + "=" * 60)
    print("TELBot Video Streaming Server")
    print("=" * 60)
    print(f"  Preset:      {args.preset}")
    print(f"  Resolution:  {width}x{height} @ {fps} fps ({preset_data['desc']})")
    print(f"  Device:      {device}")
    print(f"  Stream URL:  http://{args.host}:{args.port}/stream")
    print(f"  Snapshot:    http://{args.host}:{args.port}/snapshot")
    print(f"  State JSON:  http://{args.host}:{args.port}/state")
    print("=" * 60)
    print("Press Ctrl+C to stop.\n")
    sys.stdout.flush()

    # 3. Replace current process with ustreamer
    os.execvp(ustreamer_bin, cmd)


if __name__ == "__main__":
    main()
