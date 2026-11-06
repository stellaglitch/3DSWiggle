#!/usr/bin/env python3
"""Decode an 8-bit RGB/RGBA PNG (stdlib only), centre-crop/scale to 400x240 and
write raw little-endian RGB565 -- the same format the 3DS cameras deliver.
   python3 tests/png_to_565.py in.png out.565
"""
import struct
import sys
import zlib


def read_png(path):
    data = open(path, "rb").read()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    pos, idat, w = 8, b"", 0
    while pos < len(data):
        n, t = struct.unpack(">I4s", data[pos:pos + 8])
        body = data[pos + 8:pos + 8 + n]
        if t == b"IHDR":
            w, h, depth, ctype, _, _, interlace = struct.unpack(">IIBBBBB", body)
            assert depth == 8 and ctype in (2, 6) and interlace == 0, "unsupported PNG"
            bpp = 3 if ctype == 2 else 4
        elif t == b"IDAT":
            idat += body
        pos += 12 + n
    raw = zlib.decompress(idat)
    stride = w * bpp
    rows, prev = [], bytearray(stride)
    for y in range(h):
        f = raw[y * (stride + 1)]
        line = bytearray(raw[y * (stride + 1) + 1:(y + 1) * (stride + 1)])
        for i in range(stride):
            a = line[i - bpp] if i >= bpp else 0
            b = prev[i]
            c = prev[i - bpp] if i >= bpp else 0
            if f == 1: line[i] = (line[i] + a) & 255
            elif f == 2: line[i] = (line[i] + b) & 255
            elif f == 3: line[i] = (line[i] + (a + b) // 2) & 255
            elif f == 4:
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                line[i] = (line[i] + (a if pa <= pb and pa <= pc else b if pb <= pc else c)) & 255
        rows.append(bytes(line))
        prev = line
    return w, h, bpp, rows


def main():
    w, h, bpp, rows = read_png(sys.argv[1])
    W, H = 400, 240
    scale = max(W / w, H / h)                  # cover, then centre-crop
    out = bytearray()
    for y in range(H):
        sy = min(h - 1, int((y + 0.5 + (h * scale - H) / 2) / scale))
        row = rows[sy]
        for x in range(W):
            sx = min(w - 1, int((x + 0.5 + (w * scale - W) / 2) / scale))
            r, g, b = row[sx * bpp], row[sx * bpp + 1], row[sx * bpp + 2]
            out += struct.pack("<H", ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3))
    open(sys.argv[2], "wb").write(out)


if __name__ == "__main__":
    main()
