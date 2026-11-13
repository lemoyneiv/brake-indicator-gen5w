#!/usr/bin/env python3
"""Generate deterministic stock-style Gen5W launcher tiles (99x99) and APK icon (72x72).

Follows the exact color and bounding-box rules from HMG-API's tools/mkicon.sh
(measured from US Kia EV6 Launcher_WP3 260507) using pure Python (struct + zlib),
with 4x supersampled anti-aliasing and no external dependencies.

Glyph: automotive brake-lamp symbol (( ● )) — central illuminated lamp disc/ring
flanked by curved left/right brake-shoe arcs.
"""

import math
import os
import struct
import zlib


def parse_hex(hex_color):
    h = hex_color.lstrip("#")
    return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)


def write_png(path, width, height, rgba_rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    raw = bytearray()
    for row in rgba_rows:
        raw.append(0)  # filter type 0 (None)
        raw.extend(row)

    def chunk(tag, data):
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)
    idat = zlib.compress(bytes(raw), 9)
    png = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", idat)
        + chunk(b"IEND", b"")
    )
    with open(path, "wb") as f:
        f.write(png)


def glyph_hit(nx, ny):
    """Evaluate brake-lamp symbol in normalized coordinates nx, ny in [-1, 1].

    Design:
    - Outer ring (radius 0.36..0.50)
    - Inner glowing core dot (radius <= 0.21)
    - Left & right curved brake-shoe arcs (radius 0.62..0.76, |angle| <= 46 deg)
    """
    r = math.hypot(nx, ny)
    # Central core lamp
    if r <= 0.21:
        return True
    # Central lamp ring
    if 0.35 <= r <= 0.50:
        return True
    # Left and right curved brake-shoe arcs
    if 0.62 <= r <= 0.76 and abs(nx) > 0.10:
        angle = math.degrees(math.atan2(abs(ny), abs(nx)))
        if angle <= 46.0:
            return True
    return False


def build_mask(size, glyph_box):
    """Build anti-aliased alpha mask [0..1] of size x size with glyph centered in glyph_box."""
    samples = 4
    half = glyph_box / 2.0
    cx = (size - 1) / 2.0
    cy = (size - 1) / 2.0
    mask = [[0.0] * size for _ in range(size)]
    for y in range(size):
        for x in range(size):
            hits = 0
            for sy in range(samples):
                py = y + (sy + 0.5) / samples - 0.5
                ny = (py - cy) / half
                for sx in range(samples):
                    px = x + (sx + 0.5) / samples - 0.5
                    nx = (px - cx) / half
                    if abs(nx) <= 1.0 and abs(ny) <= 1.0 and glyph_hit(nx, ny):
                        hits += 1
            mask[y][x] = hits / float(samples * samples)
    return mask


def mask_bbox(mask):
    h = len(mask)
    w = len(mask[0])
    xs = []
    ys = []
    for y in range(h):
        for x in range(w):
            if mask[y][x] >= 0.5:
                xs.append(x)
                ys.append(y)
    return min(xs), min(ys), max(xs), max(ys)


def render_gradient_tile(mask, tl, tr, bl, br):
    h = len(mask)
    w = len(mask[0])
    x0, y0, x1, y1 = mask_bbox(mask)
    c_tl = parse_hex(tl)
    c_tr = parse_hex(tr)
    c_bl = parse_hex(bl)
    c_br = parse_hex(br)
    rows = []
    for y in range(h):
        ty = max(0.0, min(1.0, (y - y0) / float(max(1, y1 - y0))))
        row = bytearray()
        for x in range(w):
            a = mask[y][x]
            if a <= 0.0:
                row.extend((0, 0, 0, 0))
                continue
            tx = max(0.0, min(1.0, (x - x0) / float(max(1, x1 - x0))))
            rgb = []
            for ch in range(3):
                top = c_tl[ch] * (1.0 - tx) + c_tr[ch] * tx
                bot = c_bl[ch] * (1.0 - tx) + c_br[ch] * tx
                val = int(round(top * (1.0 - ty) + bot * ty))
                rgb.append(max(0, min(255, val)))
            row.extend((rgb[0], rgb[1], rgb[2], int(round(a * 255))))
        rows.append(bytes(row))
    return rows


def render_flat_tile(mask, hex_color, opacity):
    h = len(mask)
    w = len(mask[0])
    r, g, b = parse_hex(hex_color)
    rows = []
    for y in range(h):
        row = bytearray()
        for x in range(w):
            a = int(round(mask[y][x] * opacity * 255))
            if a <= 0:
                row.extend((0, 0, 0, 0))
            else:
                row.extend((r, g, b, max(0, min(255, a))))
        rows.append(bytes(row))
    return rows


def render_app_icon(size=72):
    """72x72 dark rounded-badge app icon with red brake-lamp glyph."""
    mask = build_mask(size, 48)
    samples = 4
    rows = []
    for y in range(size):
        row = bytearray()
        for x in range(size):
            # Rounded square badge coverage
            badge_hits = 0
            for sy in range(samples):
                py = y + (sy + 0.5) / samples - 0.5
                for sx in range(samples):
                    px = x + (sx + 0.5) / samples - 0.5
                    # Margin 4 px, corner radius 14 px
                    dx = max(0.0, abs(px - (size - 1) / 2.0) - ((size - 8) / 2.0 - 14.0))
                    dy = max(0.0, abs(py - (size - 1) / 2.0) - ((size - 8) / 2.0 - 14.0))
                    if math.hypot(dx, dy) <= 14.0:
                        badge_hits += 1
            ba = badge_hits / float(samples * samples)
            if ba <= 0.0:
                row.extend((0, 0, 0, 0))
                continue
            ga = mask[y][x]
            # Blend red glyph (#ff3333) over dark slate badge (#1d2026)
            r = int(round(29 * (1.0 - ga) + 255 * ga))
            g = int(round(32 * (1.0 - ga) + 51 * ga))
            b = int(round(38 * (1.0 - ga) + 51 * ga))
            row.extend((r, g, b, int(round(ba * 255))))
        rows.append(bytes(row))
    return rows


def main():
    mod_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    icons_dir = os.path.join(mod_dir, "icons")
    res_dir = os.path.join(mod_dir, "app", "res", "drawable-hdpi")

    # Stock 99x99 tile mask (64px wide brake symbol centered on 99x99 tile)
    tile_mask = build_mask(99, 64)

    # Stock colours from HMG-API tools/mkicon.sh lines 53-56
    write_png(
        os.path.join(icons_dir, "icon.png"),
        99,
        99,
        render_gradient_tile(tile_mask, "#23e4c5", "#32b6ba", "#758cd7", "#8763cc"),
    )
    write_png(
        os.path.join(icons_dir, "icon_night.png"),
        99,
        99,
        render_gradient_tile(tile_mask, "#2ab8aa", "#2d9aa3", "#4c61b4", "#4d3fae"),
    )
    write_png(
        os.path.join(icons_dir, "icon_disabled.png"),
        99,
        99,
        render_flat_tile(tile_mask, "#62797f", 0.2),
    )
    write_png(
        os.path.join(icons_dir, "icon_disabled_night.png"),
        99,
        99,
        render_flat_tile(tile_mask, "#62797f", 0.4),
    )
    write_png(os.path.join(res_dir, "ic_app.png"), 72, 72, render_app_icon(72))
    print("Generated icons in", icons_dir, "and", res_dir)


if __name__ == "__main__":
    main()
