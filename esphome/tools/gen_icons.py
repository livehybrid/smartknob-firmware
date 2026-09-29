#!/usr/bin/env python3
"""Generates the icon set used on the SmartKnob's display (screensaver
weather icon, page menu icons): simple white line-art on a transparent
background, drawn with Pillow primitives so the whole set is self-contained
and reproducible, with no external asset download and no licensing to track.

Needs Pillow: pip install Pillow
Run from the esphome/ folder: python3 tools/gen_icons.py
Each icon is drawn at 4x, using PIL's own antialiasing, and downsampled to
its final size for clean edges. Output goes to images/, referenced by the
`image:` block in packages/smartknob-base.yaml.
"""

import math
import os

from PIL import Image, ImageDraw

OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "images")
SCALE = 4  # supersampling factor; drawn at SIZE*SCALE, saved at SIZE
WHITE = (255, 255, 255, 255)


def canvas(size):
    img = Image.new("RGBA", (size * SCALE, size * SCALE), (0, 0, 0, 0))
    return img, ImageDraw.Draw(img)


def save(img, size, name):
    img = img.resize((size, size), Image.LANCZOS)
    path = os.path.join(OUT_DIR, f"{name}.png")
    img.save(path)
    print(f"wrote {path} ({size}x{size})")


def s(v, size):
    """Scale a 0..size design-space coordinate to the supersampled canvas."""
    return v * SCALE


def stroke_width(size):
    return max(2, size // 12) * SCALE


# ------------------------------------------------------------- weather --


def draw_sun(d, size, cx, cy, r):
    w = stroke_width(size)
    d.ellipse([s(cx - r, size), s(cy - r, size), s(cx + r, size), s(cy + r, size)], fill=WHITE)
    ray_r1, ray_r2 = r * 1.35, r * 1.85
    for i in range(8):
        a = math.radians(i * 45)
        x1, y1 = cx + ray_r1 * math.cos(a), cy + ray_r1 * math.sin(a)
        x2, y2 = cx + ray_r2 * math.cos(a), cy + ray_r2 * math.sin(a)
        d.line([s(x1, size), s(y1, size), s(x2, size), s(y2, size)], fill=WHITE, width=w)


def draw_moon(d, size, cx, cy, r):
    img_size = size * SCALE
    mask = Image.new("L", (img_size, img_size), 0)
    md = ImageDraw.Draw(mask)
    md.ellipse([s(cx - r, size), s(cy - r, size), s(cx + r, size), s(cy + r, size)], fill=255)
    md.ellipse(
        [s(cx - r + r * 0.7, size), s(cy - r * 0.6, size), s(cx + r + r * 0.7, size), s(cy + r * 1.4, size)],
        fill=0,
    )
    d.bitmap((0, 0), mask, fill=WHITE)


def draw_cloud(d, size, cx, cy, r, filled=True):
    """A rounded cloud silhouette, three overlapping circles plus a base."""
    parts = [(cx - r * 0.6, cy + r * 0.1, r * 0.65), (cx, cy - r * 0.25, r * 0.85), (cx + r * 0.65, cy + r * 0.1, r * 0.6)]
    for x, y, rr in parts:
        d.ellipse([s(x - rr, size), s(y - rr, size), s(x + rr, size), s(y + rr, size)], fill=WHITE if filled else None,
                  outline=WHITE, width=stroke_width(size))
    d.rounded_rectangle(
        [s(cx - r * 1.15, size), s(cy - r * 0.05, size), s(cx + r * 1.15, size), s(cy + r * 0.55, size)],
        radius=s(r * 0.3, size),
        fill=WHITE if filled else None,
        outline=WHITE,
        width=stroke_width(size),
    )


def draw_raindrop(d, size, cx, cy, length, width_):
    w = stroke_width(size)
    x1, y1 = cx, cy
    x2, y2 = cx - length * 0.35, cy + length
    d.line([s(x1, size), s(y1, size), s(x2, size), s(y2, size)], fill=WHITE, width=w)


def draw_bolt(d, size, cx, cy, h):
    pts = [
        (cx + h * 0.15, cy - h * 0.5),
        (cx - h * 0.35, cy + h * 0.1),
        (cx, cy + h * 0.1),
        (cx - h * 0.15, cy + h * 0.55),
        (cx + h * 0.4, cy - h * 0.1),
        (cx + h * 0.05, cy - h * 0.1),
    ]
    d.polygon([s(px, size) for pt in pts for px in pt], fill=WHITE)


def icon_sunny(size):
    img, d = canvas(size)
    draw_sun(d, size, size / 2, size / 2, size * 0.22)
    save(img, size, "wx_sunny")


def icon_clear_night(size):
    img, d = canvas(size)
    draw_moon(d, size, size / 2, size / 2, size * 0.3)
    save(img, size, "wx_clear_night")


def icon_cloudy(size):
    img, d = canvas(size)
    draw_cloud(d, size, size / 2, size / 2 + size * 0.03, size * 0.34)
    save(img, size, "wx_cloudy")


def icon_partlycloudy(size):
    img, d = canvas(size)
    draw_sun(d, size, size * 0.38, size * 0.36, size * 0.16)
    draw_cloud(d, size, size * 0.56, size * 0.6, size * 0.32)
    save(img, size, "wx_partlycloudy")


def icon_rainy(size):
    img, d = canvas(size)
    draw_cloud(d, size, size / 2, size * 0.38, size * 0.3)
    for dx in (-0.22, 0, 0.22):
        draw_raindrop(d, size, size / 2 + size * dx, size * 0.68, size * 0.22, size * 0.05)
    save(img, size, "wx_rainy")


def icon_snowy(size):
    img, d = canvas(size)
    draw_cloud(d, size, size / 2, size * 0.36, size * 0.3)
    w = stroke_width(size)
    for dx in (-0.22, 0, 0.22):
        cx, cy, r = size / 2 + size * dx, size * 0.74, size * 0.07
        for i in range(3):
            a = math.radians(i * 60)
            x1, y1 = cx - r * math.cos(a), cy - r * math.sin(a)
            x2, y2 = cx + r * math.cos(a), cy + r * math.sin(a)
            d.line([s(x1, size), s(y1, size), s(x2, size), s(y2, size)], fill=WHITE, width=w)
    save(img, size, "wx_snowy")


def icon_fog(size):
    img, d = canvas(size)
    draw_cloud(d, size, size / 2, size * 0.32, size * 0.26)
    w = stroke_width(size)
    for i, y in enumerate((0.6, 0.72, 0.84)):
        x0 = size * (0.22 + 0.06 * (i % 2))
        x1 = size * (0.78 - 0.06 * (i % 2))
        d.line([s(x0, size), s(size * y, size), s(x1, size), s(size * y, size)], fill=WHITE, width=w)
    save(img, size, "wx_fog")


def icon_lightning(size):
    img, d = canvas(size)
    draw_cloud(d, size, size / 2, size * 0.34, size * 0.28)
    draw_bolt(d, size, size / 2, size * 0.72, size * 0.4)
    save(img, size, "wx_lightning")


def icon_windy(size):
    img, d = canvas(size)
    w = stroke_width(size)
    rows = [(0.36, 0.62, 0.2), (0.52, 0.8, 0.3), (0.68, 0.7, 0.15)]
    for y, x1, x0 in rows:
        yy = size * y
        d.line([s(size * x0, size), s(yy, size), s(size * x1, size), s(yy, size)], fill=WHITE, width=w)
        # curl at the end
        cx, cy, r = size * x1, yy - size * 0.05, size * 0.06
        d.arc([s(cx - r, size), s(cy - r, size), s(cx + r, size), s(cy + r, size)], start=0, end=270, fill=WHITE, width=w)
    save(img, size, "wx_windy")


WEATHER_ICONS = (
    icon_sunny,
    icon_clear_night,
    icon_cloudy,
    icon_partlycloudy,
    icon_rainy,
    icon_snowy,
    icon_fog,
    icon_lightning,
    icon_windy,
)


# ------------------------------------------------------------- menu -----


def icon_bulb(size):
    img, d = canvas(size)
    w = stroke_width(size)
    cx, cy, r = size / 2, size * 0.42, size * 0.26
    d.ellipse([s(cx - r, size), s(cy - r, size), s(cx + r, size), s(cy + r, size)], outline=WHITE, width=w)
    d.line(
        [s(cx - r * 0.35, size), s(cy + r * 0.85, size), s(cx + r * 0.35, size), s(cy + r * 0.85, size)],
        fill=WHITE, width=w,
    )
    d.rounded_rectangle(
        [s(cx - r * 0.45, size), s(cy + r * 0.85, size), s(cx + r * 0.45, size), s(cy + r * 1.55, size)],
        radius=s(r * 0.15, size), outline=WHITE, width=w,
    )
    save(img, size, "menu_light")


def icon_note(size):
    img, d = canvas(size)
    w = stroke_width(size)
    hx, hy, hr = size * 0.34, size * 0.68, size * 0.12
    d.ellipse([s(hx - hr, size), s(hy - hr, size), s(hx + hr, size), s(hy + hr, size)], fill=WHITE)
    d.line([s(hx + hr * 0.85, size), s(hy, size), s(hx + hr * 0.85, size), s(size * 0.22, size)], fill=WHITE, width=w)
    d.line(
        [s(hx + hr * 0.85, size), s(size * 0.22, size), s(size * 0.72, size), s(size * 0.3, size)],
        fill=WHITE, width=w,
    )
    save(img, size, "menu_media")


def icon_thermometer(size):
    img, d = canvas(size)
    w = stroke_width(size)
    cx = size / 2
    top, bulb_y, bulb_r, stem_r = size * 0.2, size * 0.68, size * 0.14, size * 0.08
    d.rounded_rectangle(
        [s(cx - stem_r, size), s(top, size), s(cx + stem_r, size), s(bulb_y, size)],
        radius=s(stem_r, size), outline=WHITE, width=w,
    )
    d.ellipse([s(cx - bulb_r, size), s(bulb_y - bulb_r, size), s(cx + bulb_r, size), s(bulb_y + bulb_r, size)], fill=WHITE)
    d.line([s(cx, size), s(top + stem_r * 0.6, size), s(cx, size), s(bulb_y, size)], fill=WHITE, width=max(2, w // 2))
    save(img, size, "menu_climate")


def icon_blinds(size):
    img, d = canvas(size)
    w = stroke_width(size)
    x0, x1, y0, y1 = size * 0.22, size * 0.78, size * 0.22, size * 0.78
    d.rectangle([s(x0, size), s(y0, size), s(x1, size), s(y1, size)], outline=WHITE, width=w)
    for i in range(1, 4):
        yy = y0 + (y1 - y0) * i / 4
        d.line([s(x0, size), s(yy, size), s(x1, size), s(yy, size)], fill=WHITE, width=max(2, w // 2))
    save(img, size, "menu_cover")


def icon_power(size):
    img, d = canvas(size)
    w = stroke_width(size)
    cx, cy, r = size / 2, size * 0.52, size * 0.28
    d.arc([s(cx - r, size), s(cy - r, size), s(cx + r, size), s(cy + r, size)], start=-60, end=240, fill=WHITE, width=w)
    d.line([s(cx, size), s(size * 0.16, size), s(cx, size), s(cy, size)], fill=WHITE, width=w)
    save(img, size, "menu_switch")


def icon_gear(size):
    img, d = canvas(size)
    w = stroke_width(size)
    cx, cy, r_out, r_in, teeth = size / 2, size / 2, size * 0.32, size * 0.24, 8
    for i in range(teeth):
        a = math.radians(i * (360 / teeth))
        x1, y1 = cx + r_in * math.cos(a), cy + r_in * math.sin(a)
        x2, y2 = cx + r_out * math.cos(a), cy + r_out * math.sin(a)
        d.line([s(x1, size), s(y1, size), s(x2, size), s(y2, size)], fill=WHITE, width=w)
    d.ellipse([s(cx - r_in, size), s(cy - r_in, size), s(cx + r_in, size), s(cy + r_in, size)], outline=WHITE, width=w)
    hr = size * 0.09
    d.ellipse([s(cx - hr, size), s(cy - hr, size), s(cx + hr, size), s(cy + hr, size)], fill=WHITE)
    save(img, size, "menu_setup")


MENU_ICONS = (icon_bulb, icon_note, icon_thermometer, icon_blinds, icon_power, icon_gear)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    for fn in WEATHER_ICONS:
        fn(48)
    for fn in MENU_ICONS:
        fn(40)


if __name__ == "__main__":
    main()
