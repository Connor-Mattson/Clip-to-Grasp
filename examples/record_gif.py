"""
Record the end-to-end demo (main.py) as an animated GIF.

Runs main.py with a fixed query and captures frames from an external camera while the robot works.
Nothing opens on screen: the simulation runs in PyBullet's DIRECT mode, frames come from PyBullet's
CPU renderer, and the robot moves as fast as the simulation allows while the GIF plays back in real time.

Run from the repository root:
    python -m examples.record_gif "a banana" --out media/clip-to-grasp_banana.gif
"""
import argparse
import builtins
import os
import runpy
import sys
import time

import numpy as np
import pybullet as p
from PIL import Image, ImageDraw, ImageFont

SIM_HZ = 240
STEPS_PER_FRAME = 10  # 240 Hz simulation -> 24 fps
FRAME_MS = 40
SUPERSAMPLE = 2  # render at 2x and downscale to smooth edges


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("query", nargs="?", default="a banana", help="object description to give main.py")
    parser.add_argument("--out", default="media/clip-to-grasp_demo.gif", help="output GIF path")
    parser.add_argument("--camera", nargs=6, type=float, metavar=("TX", "TY", "TZ", "DIST", "YAW", "PITCH"),
                        default=[0.3, 0.25, 0.3, 2.0, 35, -25],
                        help="camera target (m), distance (m), yaw and pitch (degrees)")
    parser.add_argument("--size", nargs=2, type=int, metavar=("WIDTH", "HEIGHT"), default=[640, 480])
    parser.add_argument("--no-caption", action="store_true", help="don't overlay the query text")
    return parser.parse_args()


def load_font(size=22):
    for path in ("/System/Library/Fonts/Helvetica.ttc", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            pass
    return ImageFont.load_default(size=size)


def save_gif(frames, path):
    # One palette shared by all frames so colors don't flicker. FASTOCTREE keeps small saturated objects
    # like the yellow banana, which MEDIANCUT folds into brown.
    width, height = frames[0].size
    sample = frames[::max(1, len(frames) // 12)]
    strip = Image.new("RGB", (width, height * len(sample)))
    for i, frame in enumerate(sample):
        strip.paste(frame, (0, i * height))
    palette = strip.quantize(colors=255, method=Image.Quantize.FASTOCTREE)
    gif_frames = [frame.quantize(palette=palette, dither=Image.Dither.NONE) for frame in frames]

    durations = [FRAME_MS] * len(gif_frames)
    durations[0], durations[-1] = 800, 1500  # pause on the query, then on the result
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    gif_frames[0].save(path, save_all=True, append_images=gif_frames[1:], duration=durations, loop=0,
                       optimize=True)
    print(f"Saved {path}: {len(frames)} frames, {os.path.getsize(path) / 1e6:.2f} MB, "
          f"{sum(durations) / 1000:.1f}s")


def main():
    args = parse_args()
    width, height = args.size
    tx, ty, tz, dist, yaw, pitch = args.camera
    view = p.computeViewMatrixFromYawPitchRoll([tx, ty, tz], dist, yaw, pitch, 0, 2)
    proj = p.computeProjectionMatrixFOV(45, width / height, 0.05, 10)
    font = load_font()
    caption = None if args.no_caption else f'"{args.query}"'
    frames = []

    def render():
        rgba = p.getCameraImage(width * SUPERSAMPLE, height * SUPERSAMPLE, view, proj, shadow=1,
                                lightDirection=[1, 1, 2], renderer=p.ER_TINY_RENDERER)[2]
        rgb = np.reshape(rgba, (height * SUPERSAMPLE, width * SUPERSAMPLE, 4))[:, :, :3].astype(np.uint8)
        frame = Image.fromarray(rgb).resize((width, height), Image.LANCZOS)
        if caption:
            draw = ImageDraw.Draw(frame)
            left, top, right, bottom = draw.textbbox((0, 0), caption, font=font)
            draw.rounded_rectangle((12, 12, 36 + right - left, 30 + bottom - top), radius=8, fill=(255, 255, 255))
            draw.text((24 - left, 21 - top), caption, font=font, fill=(30, 30, 30))
        frames.append(frame)

    # Run main.py headless and at full speed: open DIRECT instead of a GUI window, skip its real-time
    # sleeps, answer its prompt with the query, and capture a frame every STEPS_PER_FRAME simulation
    # steps from the moment the query is entered
    connect, step_simulation = p.connect, p.stepSimulation
    p.connect = lambda mode, *a, **k: connect(p.DIRECT if mode == p.GUI else mode, *a, **k)
    time.sleep = lambda seconds: None
    recording = False
    steps = 0

    def step_and_capture(*a, **k):
        nonlocal steps
        result = step_simulation(*a, **k)
        if recording:
            steps += 1
            if steps % STEPS_PER_FRAME == 0:
                render()
        return result

    def answer_prompt(prompt=""):
        nonlocal recording
        print(prompt + args.query)
        recording = True
        render()
        return args.query

    p.stepSimulation = step_and_capture
    builtins.input = answer_prompt

    try:
        runpy.run_path("main.py", run_name="__main__")
    except SystemExit:
        sys.exit(f"main.py stopped without moving the robot for {args.query!r}; no GIF written")

    # Hold on the final scene for a second after the release
    for _ in range(SIM_HZ):
        p.stepSimulation()
    p.disconnect()
    save_gif(frames, args.out)


if __name__ == "__main__":
    main()
