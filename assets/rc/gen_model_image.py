#!/usr/bin/env python3
"""
gen_model_image.py -- generate a TX16S model-select image for a
BlueBoat.

    pip install pillow
    python3 gen_model_image.py ZOE 31
    python3 gen_model_image.py YIP 32 --out ../somewhere/

Output: <name lowercase>.png, 720x800 portrait (the size used by the
fleet's model-select page layout), written to --out (default: current
directory). Copy to the handset SD card's /IMAGES/ folder.

Layout (matches the fleet standard set with yip.png, 2026-08-12):
white band with the boat name, then BLUEBOAT / ID <n> in yellow, and
a top-down catamaran silhouette. The ID must match the model's
Receiver No. (Model Match). Fleet numbering is hull-based: zoe 31,
yip 32, xai 33, ...
"""

import argparse, os, sys
from PIL import Image, ImageDraw, ImageFont

NAVY   = (13, 27, 51)
YELLOW = (240, 178, 20)
WHITE  = (255, 255, 255)
W, H = 720, 800

FONT_FALLBACKS = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/System/Library/Fonts/Supplemental/Arial Bold.ttf",   # macOS
    "/Library/Fonts/Arial Bold.ttf",
]

def load_font(size):
    for path in FONT_FALLBACKS:
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()

def centered(draw, text, font, y_center, fill):
    b = draw.textbbox((0, 0), text, font=font)
    draw.text(((W - (b[2] - b[0])) / 2 - b[0],
               y_center - (b[3] - b[1]) / 2 - b[1]), text,
              font=font, fill=fill)

def fit_font(draw, text, max_width, start_size):
    size = start_size
    while size > 40:
        f = load_font(size)
        b = draw.textbbox((0, 0), text, font=f)
        if b[2] - b[0] <= max_width:
            return f
        size -= 8
    return load_font(size)

def catamaran(d, cx, top, scale=1.0):
    """Top-down catamaran, bows up, centred on cx."""
    hw   = int(42 * scale)    # hull half-width
    hh   = int(150 * scale)   # hull body height
    bow  = int(60 * scale)    # bow triangle height
    gap  = int(130 * scale)   # gap between hull inner edges
    for side in (-1, 1):
        hx = cx + side * (gap // 2 + hw)
        d.rounded_rectangle([hx - hw, top + bow, hx + hw,
                             top + bow + hh], radius=int(20 * scale),
                            fill=YELLOW)
        d.polygon([(hx - hw, top + bow + int(10 * scale)),
                   (hx, top),
                   (hx + hw, top + bow + int(10 * scale))], fill=YELLOW)
    # crossdeck with electronics box
    y0 = top + bow + int(35 * scale)
    y1 = y0 + int(90 * scale)
    x0, x1 = cx - gap // 2 - int(6 * scale), cx + gap // 2 + int(6 * scale)
    d.rectangle([x0, y0, x1, y1], fill=YELLOW)
    bw = int(34 * scale)
    d.rectangle([cx - bw, y0 + int(14 * scale),
                 cx + bw, y1 - int(14 * scale)], fill=NAVY)
    for side in (-1, 1):
        sx = cx + side * int(72 * scale)
        d.rectangle([sx - int(12 * scale), y0 + int(22 * scale),
                     sx + int(12 * scale), y1 - int(22 * scale)],
                    fill=NAVY)

def make(name, fleet_id, out_dir):
    if len(name.lower()) > 9:
        sys.exit(f"'{name}': EdgeTX image filenames are limited to "
                 "9 characters before .png")

    img = Image.new("RGB", (W, H), NAVY)
    d = ImageDraw.Draw(img)

    # white name band
    d.rectangle([0, 0, W, 258], fill=WHITE)
    centered(d, name.upper(),
             fit_font(d, name.upper(), 640, 230), 126, NAVY)

    # BLUEBOAT / ID block
    centered(d, "BLUEBOAT", load_font(92), 332, YELLOW)
    centered(d, f"ID {fleet_id}", load_font(130), 462, YELLOW)

    # catamaran
    catamaran(d, W // 2, 565, scale=1.15)

    path = os.path.join(out_dir, name.lower() + ".png")
    img.save(path, optimize=True)
    print(f"{path}  ({W}x{H}, ID {fleet_id})")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("name", help="boat name, e.g. ZOE")
    ap.add_argument("id", type=int,
                    help="fleet number = Receiver No. (zoe 31, yip 32, xai 33, ...)")
    ap.add_argument("--out", default=".")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    make(args.name, args.id, args.out)
