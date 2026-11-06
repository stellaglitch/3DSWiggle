#!/usr/bin/env python3
"""Generate 3DSWiggle's icon (48x48), HOME-menu banner (256x128) and banner jingle (WAV).
Standard library only:  python3 tools/make_assets.py
"""
import math
import os
import struct
import zlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "assets")


def write_png(path, w, h, px):
    raw = b"".join(b"\x00" + bytes(c for p in px[y * w:(y + 1) * w] for c in p) for y in range(h))

    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)

    data = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
    data += chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(data)


class Canvas:
    def __init__(self, w, h, bg=(0, 0, 0, 0)):
        self.w, self.h = w, h
        self.px = [bg] * (w * h)

    def blend(self, x, y, c, a=1.0):
        if 0 <= x < self.w and 0 <= y < self.h:
            o = self.px[y * self.w + x]
            a *= c[3] / 255 if len(c) > 3 else 1
            na = a + o[3] / 255 * (1 - a)
            if na <= 0:
                return
            rgb = tuple(int((c[k] * a + o[k] * (o[3] / 255) * (1 - a)) / na + 0.5) for k in range(3))
            self.px[y * self.w + x] = rgb + (int(na * 255 + 0.5),)

    def rect(self, x0, y0, x1, y1, c, a=1.0):
        for y in range(y0, y1):
            for x in range(x0, x1):
                self.blend(x, y, c, a)

    def rrect(self, x0, y0, x1, y1, r, c, a=1.0):
        for y in range(y0, y1):
            for x in range(x0, x1):
                dx = max(x0 + r - x - 0.5, 0, x - (x1 - r) + 0.5)
                dy = max(y0 + r - y - 0.5, 0, y - (y1 - r) + 0.5)
                d = math.hypot(dx, dy)
                cov = min(1.0, max(0.0, r - d + 0.5))
                if cov > 0:
                    self.blend(x, y, c, a * cov)


def photo(cv, x, y, w, h, tint, a=1.0):
    """A little Polaroid-style photo: white frame, sky, sun and hills."""
    cv.rrect(x, y, x + w, y + h, 2, (250, 250, 245), a)
    ix0, iy0, ix1, iy1 = x + 3, y + 3, x + w - 3, y + h - 3 - max(2, h // 6)
    for yy in range(iy0, iy1):
        t = (yy - iy0) / max(1, iy1 - iy0 - 1)
        sky = tuple(int(tint[k] * (1 - t) + (255, 214, 160)[k] * t) for k in range(3))
        for xx in range(ix0, ix1):
            u = (xx - ix0) / max(1, ix1 - ix0 - 1)
            hill = iy1 - (iy1 - iy0) * (0.28 + 0.12 * math.sin(u * 5.0 + 0.6))
            sx, sy = ix0 + (ix1 - ix0) * 0.7, iy0 + (iy1 - iy0) * 0.32
            if math.hypot(xx - sx, yy - sy) < (iy1 - iy0) * 0.14:
                cv.blend(xx, yy, (255, 236, 120), a)
            elif yy > hill:
                cv.blend(xx, yy, (60, 150, 90), a)
            else:
                cv.blend(xx, yy, sky, a)


FONT = {  # 5x7
    "W": ["10001", "10001", "10001", "10101", "10101", "11011", "10001"],
    "I": ["11111", "00100", "00100", "00100", "00100", "00100", "11111"],
    "G": ["01111", "10000", "10000", "10011", "10001", "10001", "01111"],
    "L": ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
    "E": ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    "3": ["11110", "00001", "00001", "01110", "00001", "00001", "11110"],
    "D": ["11110", "10001", "10001", "10001", "10001", "10001", "11110"],
    "R": ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    "A": ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    "M": ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
    "C": ["01111", "10000", "10000", "10000", "10000", "10000", "01111"],
    "S": ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
    " ": ["00000"] * 7,
}


def text(cv, s, x, y, scale, c, shadow=None):
    for i, ch in enumerate(s):
        g = FONT[ch]
        for gy, row in enumerate(g):
            for gx, bit in enumerate(row):
                if bit == "1":
                    px, py = x + (i * 6 + gx) * scale, y + gy * scale
                    if shadow:
                        cv.rect(px + scale // 3 + 1, py + scale // 3 + 1, px + scale + scale // 3 + 1,
                                py + scale + scale // 3 + 1, shadow)
                    cv.rect(px, py, px + scale, py + scale, c)


def text_width(s, scale):
    return (len(s) * 6 - 1) * scale


def make_icon():
    cv = Canvas(48, 48)
    cv.rrect(0, 0, 48, 48, 9, (40, 34, 72))
    for y in range(48):  # soft vertical gradient
        for x in range(48):
            cv.blend(x, y, (120, 70, 180), 0.35 * y / 47 * (1 if cv.px[y * 48 + x][3] else 0))
    photo(cv, 6, 9, 26, 30, (90, 200, 255), 0.55)    # frame 1 (ghost)
    photo(cv, 15, 9, 26, 30, (255, 120, 190), 1.0)   # frame 2
    # wiggle arrows
    for i in range(5):
        cv.blend(17 + i, 43, (255, 230, 90))
        cv.blend(27 + i, 43, (255, 230, 90))
    for d in range(3):
        cv.blend(17 + d, 43 - d, (255, 230, 90)); cv.blend(17 + d, 43 + d, (255, 230, 90))
        cv.blend(31 - d, 43 - d, (255, 230, 90)); cv.blend(31 - d, 43 + d, (255, 230, 90))
    write_png(os.path.join(OUT, "icon.png"), 48, 48, cv.px)


def make_banner():
    W, H = 256, 128
    cv = Canvas(W, H)
    for y in range(H):
        for x in range(W):
            t = y / (H - 1)
            u = x / (W - 1)
            c = (int(48 + 70 * u), int(38 + 20 * t), int(96 + 90 * t))
            cv.px[y * W + x] = c + (255,)
    photo(cv, 10, 28, 54, 64, (90, 200, 255), 0.5)
    photo(cv, 22, 28, 54, 64, (255, 120, 190), 1.0)
    x0 = 88
    text(cv, "3DSWIGGLE", x0, 38, 3, (255, 240, 120), shadow=(20, 10, 40, 160))
    text(cv, "WIGGLEGRAM", x0, 70, 2, (235, 235, 255))
    text(cv, "CAMERA", x0, 88, 2, (235, 235, 255))
    write_png(os.path.join(OUT, "banner.png"), W, H, cv.px)


def make_jingle():
    """A short two-note 'wig-gle' chime, 16-bit mono 22.05 kHz (< 3 s for bannertool)."""
    rate = 22050
    notes = [(0.00, 0.22, 784.0), (0.16, 0.22, 1046.5), (0.34, 0.20, 784.0), (0.50, 0.55, 1318.5)]
    n = int(rate * 1.2)
    buf = [0.0] * n
    for start, dur, f in notes:
        s0 = int(start * rate)
        for i in range(int((dur + 0.35) * rate)):
            if s0 + i >= n:
                break
            t = i / rate
            env = min(1.0, t / 0.008) * math.exp(-t * 6.0)
            buf[s0 + i] += 0.28 * env * (math.sin(2 * math.pi * f * t) + 0.3 * math.sin(4 * math.pi * f * t))
    pcm = b"".join(struct.pack("<h", int(max(-1, min(1, v)) * 32000)) for v in buf)
    with open(os.path.join(OUT, "audio.wav"), "wb") as f:
        f.write(b"RIFF" + struct.pack("<I", 36 + len(pcm)) + b"WAVE")
        f.write(b"fmt " + struct.pack("<IHHIIHH", 16, 1, 1, rate, rate * 2, 2, 16))
        f.write(b"data" + struct.pack("<I", len(pcm)) + pcm)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    make_icon()
    make_banner()
    make_jingle()
    print("wrote", ", ".join(sorted(os.listdir(OUT))))
