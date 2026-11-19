#!/usr/bin/env python3
"""
Pure-stdlib Python generator for GyroFlow-style iOS 12 AppIcon PNGs (24-bit RGB, no alpha).
Generates App/Assets.xcassets/Contents.json and App/Assets.xcassets/AppIcon.appiconset/*.png
"""
import json
import math
import os
import struct
import zlib


def write_png_rgb(path: str, width: int, height: int, rgb_bytes: bytearray) -> None:
    def chunk(tag: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

    raw_rows = bytearray()
    stride = width * 3
    for y in range(height):
        raw_rows.append(0)  # Filter type 0 (None)
        raw_rows.extend(rgb_bytes[y * stride : (y + 1) * stride])

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)  # 8-bit, color_type=2 (RGB, no alpha)
    idat = zlib.compress(bytes(raw_rows), level=9)
    png_data = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", idat) + chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(png_data)


def clamp01(v: float) -> float:
    return 0.0 if v < 0.0 else (1.0 if v > 1.0 else v)


def smoothstep(e0: float, e1: float, x: float) -> float:
    t = clamp01((x - e0) / (e1 - e0))
    return t * t * (3.0 - 2.0 * t)


def ellipse_ring_distance(px: float, py: float, rx: float, ry: float, angle_rad: float) -> float:
    ca = math.cos(angle_rad)
    sa = math.sin(angle_rad)
    x = px * ca + py * sa
    y = -px * sa + py * ca
    # Normalized implicit ellipse distance approximation (accurate near boundary)
    nx = x / rx
    ny = y / ry
    f = math.sqrt(nx * nx + ny * ny) - 1.0
    gx = nx / rx
    gy = ny / ry
    grad = math.sqrt(gx * gx + gy * gy) + 1e-9
    return abs(f) / grad


def render_master_512() -> bytearray:
    size = 512
    buf = bytearray(size * size * 3)
    inv_half = 2.0 / size

    # Palette (Electric Cyan #00F2FE -> Neon Azure #0072FF on Cosmic Navy #0B132B -> #050814)
    ang1 = math.radians(32.0)
    ang2 = math.radians(-32.0)
    rx = 0.72
    ry = 0.34
    ring_half_thick = 0.044
    glow_width = 0.095

    for py_i in range(size):
        py = (py_i + 0.5) * inv_half - 1.0
        row_offset = py_i * size * 3
        for px_i in range(size):
            px = (px_i + 0.5) * inv_half - 1.0
            r_center = math.sqrt(px * px + py * py)

            # 1. Cosmic radial background gradient
            bg_t = clamp01(r_center / 1.35)
            r = 11.0 + (5.0 - 11.0) * bg_t
            g = 19.0 + (8.0 - 19.0) * bg_t
            b = 43.0 + (20.0 - 43.0) * bg_t

            # Subtle central aura
            aura = math.exp(-r_center * r_center * 3.2) * 0.22
            r += aura * 0.0
            g += aura * 120.0
            b += aura * 220.0

            # 2. Dual intersecting 3D gyro gimbal rings
            d1 = ellipse_ring_distance(px, py, rx, ry, ang1)
            d2 = ellipse_ring_distance(px, py, rx, ry, ang2)

            for d, phase_shift in ((d1, 0.0), (d2, 0.5)):
                # Cyan -> Azure gradient along ring
                grad_t = clamp01(0.5 + 0.5 * (px * 0.8 + py * 0.4 + phase_shift * 0.2))
                cr = 0.0
                cg = 242.0 + (114.0 - 242.0) * grad_t
                cb = 254.0 + (255.0 - 254.0) * grad_t

                # Soft outer glow
                glow = math.exp(-max(0.0, d - ring_half_thick) ** 2 / (2.0 * glow_width * glow_width * 0.18)) * 0.42
                alpha_ring = 1.0 - smoothstep(ring_half_thick - 0.008, ring_half_thick + 0.008, d)
                blend = clamp01(alpha_ring + glow * (1.0 - alpha_ring))
                r = r * (1.0 - blend) + cr * blend
                g = g * (1.0 - blend) + cg * blend
                b = b * (1.0 - blend) + cb * blend

            # 3. Crisp horizontal Horizon Lock core line
            if abs(px) <= 0.62:
                line_fade = 1.0 - smoothstep(0.45, 0.62, abs(px))
                d_line = abs(py)
                line_core = (1.0 - smoothstep(0.012, 0.022, d_line)) * line_fade
                line_glow = math.exp(-(d_line * d_line) / (2.0 * 0.032 * 0.032)) * 0.45 * line_fade
                lb = clamp01(line_core + line_glow * (1.0 - line_core))
                lr = 225.0 * line_core + 0.0 * (1.0 - line_core)
                lg = 252.0
                lbb = 255.0
                r = r * (1.0 - lb) + lr * lb
                g = g * (1.0 - lb) + lg * lb
                b = b * (1.0 - lb) + lbb * lb

            # 4. Glowing optical lens center dot
            dot_core = 1.0 - smoothstep(0.055, 0.068, r_center)
            dot_glow = math.exp(-(r_center * r_center) / (2.0 * 0.06 * 0.06)) * 0.65
            db = clamp01(dot_core + dot_glow * (1.0 - dot_core))
            r = r * (1.0 - db) + (245.0 * dot_core + 40.0 * (1.0 - dot_core)) * db
            g = g * (1.0 - db) + 252.0 * db
            b = b * (1.0 - db) + 255.0 * db

            idx = row_offset + px_i * 3
            buf[idx] = max(0, min(255, int(r + 0.5)))
            buf[idx + 1] = max(0, min(255, int(g + 0.5)))
            buf[idx + 2] = max(0, min(255, int(b + 0.5)))

    return buf


def box_downsample_rgb(src: bytearray, src_size: int, dst_size: int) -> bytearray:
    if dst_size == src_size:
        return bytearray(src)
    dst = bytearray(dst_size * dst_size * 3)
    scale = src_size / float(dst_size)
    for dy in range(dst_size):
        sy0 = dy * scale
        sy1 = (dy + 1) * scale
        iy0 = int(math.floor(sy0))
        iy1 = min(src_size - 1, int(math.floor(sy1)))
        for dx in range(dst_size):
            sx0 = dx * scale
            sx1 = (dx + 1) * scale
            ix0 = int(math.floor(sx0))
            ix1 = min(src_size - 1, int(math.floor(sx1)))

            acc_r = acc_g = acc_b = acc_w = 0.0
            for sy in range(iy0, iy1 + 1):
                wy = min(sy + 1.0, sy1) - max(float(sy), sy0)
                if wy <= 0.0:
                    continue
                row = sy * src_size * 3
                for sx in range(ix0, ix1 + 1):
                    wx = min(sx + 1.0, sx1) - max(float(sx), sx0)
                    if wx <= 0.0:
                        continue
                    w = wx * wy
                    p = row + sx * 3
                    acc_r += src[p] * w
                    acc_g += src[p + 1] * w
                    acc_b += src[p + 2] * w
                    acc_w += w
            inv_w = 1.0 / max(acc_w, 1e-9)
            d_idx = (dy * dst_size + dx) * 3
            dst[d_idx] = max(0, min(255, int(acc_r * inv_w + 0.5)))
            dst[d_idx + 1] = max(0, min(255, int(acc_g * inv_w + 0.5)))
            dst[d_idx + 2] = max(0, min(255, int(acc_b * inv_w + 0.5)))
    return dst


def bilinear_upsample_rgb(src: bytearray, src_size: int, dst_size: int) -> bytearray:
    dst = bytearray(dst_size * dst_size * 3)
    scale = (src_size - 1) / float(max(1, dst_size - 1))
    for dy in range(dst_size):
        sy = dy * scale
        y0 = int(math.floor(sy))
        y1 = min(src_size - 1, y0 + 1)
        fy = sy - y0
        for dx in range(dst_size):
            sx = dx * scale
            x0 = int(math.floor(sx))
            x1 = min(src_size - 1, x0 + 1)
            fx = sx - x0
            p00 = (y0 * src_size + x0) * 3
            p10 = (y0 * src_size + x1) * 3
            p01 = (y1 * src_size + x0) * 3
            p11 = (y1 * src_size + x1) * 3
            d_idx = (dy * dst_size + dx) * 3
            for c in range(3):
                v0 = src[p00 + c] * (1.0 - fx) + src[p10 + c] * fx
                v1 = src[p01 + c] * (1.0 - fx) + src[p11 + c] * fx
                dst[d_idx + c] = max(0, min(255, int(v0 * (1.0 - fy) + v1 * fy + 0.5)))
    return dst


def main() -> None:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(script_dir)
    xcassets_dir = os.path.join(project_dir, "App", "Assets.xcassets")
    appicon_dir = os.path.join(xcassets_dir, "AppIcon.appiconset")
    os.makedirs(appicon_dir, exist_ok=True)

    with open(os.path.join(xcassets_dir, "Contents.json"), "w", encoding="utf-8") as f:
        json.dump({"info": {"version": 1, "author": "xcode"}}, f, indent=2)

    master_512 = render_master_512()

    specs = [
        ("Icon-20@2x.png", 40, "iphone", "20x20", "2x"),
        ("Icon-20@3x.png", 60, "iphone", "20x20", "3x"),
        ("Icon-29@2x.png", 58, "iphone", "29x29", "2x"),
        ("Icon-29@3x.png", 87, "iphone", "29x29", "3x"),
        ("Icon-40@2x.png", 80, "iphone", "40x40", "2x"),
        ("Icon-40@3x.png", 120, "iphone", "40x40", "3x"),
        ("Icon-60@2x.png", 120, "iphone", "60x60", "2x"),
        ("Icon-60@3x.png", 180, "iphone", "60x60", "3x"),
        ("Icon-1024.png", 1024, "ios-marketing", "1024x1024", "1x"),
    ]

    images = []
    for filename, px_size, idiom, pt_size, scale in specs:
        out_path = os.path.join(appicon_dir, filename)
        if px_size <= 512:
            img_data = box_downsample_rgb(master_512, 512, px_size)
        else:
            img_data = bilinear_upsample_rgb(master_512, 512, px_size)
        write_png_rgb(out_path, px_size, px_size, img_data)
        images.append({
            "filename": filename,
            "idiom": idiom,
            "scale": scale,
            "size": pt_size,
        })

    contents = {
        "images": images,
        "info": {"version": 1, "author": "xcode"},
    }
    with open(os.path.join(appicon_dir, "Contents.json"), "w", encoding="utf-8") as f:
        json.dump(contents, f, indent=2)

    print(f"Generated {len(specs)} AppIcon PNGs in {appicon_dir}")


if __name__ == "__main__":
    main()
