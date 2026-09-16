"""Render recorded four-pad grids without starting Isaac Sim."""

import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

LABELS = ("Left hand / finger 1", "Left hand / finger 2", "Right hand / finger 1", "Right hand / finger 2")


def jet_rgb(values):
    """Fixed blue-cyan-yellow-red scale over [0, 1], like the ALOHA viewer."""
    values = np.clip(values, 0, 1)
    channels = [np.clip(1.5 - np.abs(4 * values - center), 0, 1) for center in (3, 2, 1)]
    return np.uint8(np.stack(channels, axis=-1) * 255)


def render_frame(grids, title):
    if grids.shape != (4, 12, 32):
        raise ValueError(f"Expected four 12x32 pads, got {grids.shape}")
    canvas = Image.new("RGB", (1450, 270), "#f4f6fa")
    draw = ImageDraw.Draw(canvas)
    font_path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    font = ImageFont.truetype(str(font_path), 15) if font_path.exists() else ImageFont.load_default()
    draw.text((20, 10), title, font=font, fill="#182333")
    for finger, grid in enumerate(grids):
        x, y = 30 + 355 * finger, 70
        draw.text((x, 38), f"{LABELS[finger]}  max={grid.max():.3f}", font=font, fill="#182333")
        # No transpose or interpolation: one coloured cell per taxel.
        canvas.paste(Image.fromarray(jet_rgb(grid)).resize((320, 120), Image.Resampling.NEAREST), (x, y))
        for col in range(33):
            draw.line((x + col * 10, y, x + col * 10, y + 120), fill="#456078")
        for row in range(13):
            draw.line((x, y + row * 10, x + 320, y + row * 10), fill="#456078")
        draw.text((x, 198), "12 rows (width) x 32 columns (length)", font=font, fill="#182333")
    bar = Image.fromarray(jet_rgb(np.linspace(0, 1, 256)[None, :])).resize((256, 15))
    canvas.paste(bar, (570, 237))
    draw.text((20, 233), "Same instant for all four pads | normalized response", font=font, fill="#182333")
    draw.text((546, 233), "0", font=font, fill="#182333")
    draw.text((837, 233), "1", font=font, fill="#182333")
    draw.text((890, 233), "Blue: zero | Red: strong response", font=font, fill="#182333")
    return canvas


def export_views(frames, phases, steps, times, output, animate=False):
    if not np.isfinite(frames).all() or frames.min() < 0 or frames.max() > 1:
        raise ValueError("Expected finite normalized tactile values in [0, 1]")
    candidates = np.flatnonzero(phases == "probe_close")
    if not len(candidates):
        candidates = np.flatnonzero(phases == "grasp_hold")
    if not len(candidates):
        candidates = np.arange(len(frames))
    contact = int(candidates[np.argmax(frames[candidates].sum(axis=(1, 2, 3)))])

    def render(index):
        return render_frame(frames[index], f"{phases[index]} | step {steps[index]} | sim time {times[index]:.3f} s")

    render(contact).save(output / "tactile_contact.png")
    selected = [0, contact, len(frames) - 1]
    stages = Image.new("RGB", (1450, 270 * len(selected)), "white")
    for row, index in enumerate(selected):
        stages.paste(render(index), (0, row * 270))
    stages.save(output / "tactile_stages.png")
    if animate:
        # 120 Hz source sampled every six frames -> 20 fps, nominal real time.
        indices = list(range(0, len(frames), 6))
        if indices[-1] != len(frames) - 1:
            indices.append(len(frames) - 1)
        images = [render(i) for i in indices]
        images[0].save(output / "tactile_replay.gif", save_all=True, append_images=images[1:],
                       duration=50, loop=0, optimize=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sequence", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--gif", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    with np.load(args.sequence, allow_pickle=False) as data:
        export_views(data["tactile"], data["phase"], data["step"], data["time_s"], args.output, args.gif)
    print(args.output)


if __name__ == "__main__":
    main()
