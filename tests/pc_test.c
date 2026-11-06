// PC test for the portable wiggle core.
// Builds a synthetic stereo pair with three depth layers, runs the full
// pipeline, writes GIFs, then decodes them with an independent LZW decoder
// and checks every frame matches what was encoded.
//
//   gcc -O2 -Isource tests/pc_test.c source/wiggle.c -o pc_test && ./pc_test out_dir
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wiggle.h"

#define W 400
#define H 240

static unsigned hash2(int x, int y)
{
    unsigned h = (unsigned)x * 374761393u + (unsigned)y * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}

static float vnoise(float x, float y)
{
    int ix = (int)floorf(x), iy = (int)floorf(y);
    float fx = x - ix, fy = y - iy;
    float a = (hash2(ix, iy) & 255) / 255.f, b = (hash2(ix + 1, iy) & 255) / 255.f;
    float c = (hash2(ix, iy + 1) & 255) / 255.f, d = (hash2(ix + 1, iy + 1) & 255) / 255.f;
    fx = fx * fx * (3 - 2 * fx); fy = fy * fy * (3 - 2 * fy);
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy;
}

// view 0 = camera 1, view 1 = camera 2 (objects shift left by their disparity).
static void render_view(uint16_t *out, float view)
{
    const int dispBg = 4, dispMid = 22, dispNear = 40;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            float r, g, b;
            // near: circle
            float nx = x + view * dispNear - 120, ny = y - 140;
            // mid: rectangle
            float mx = x + view * dispMid - 230, my = y - 60;
            if (nx * nx + ny * ny < 55 * 55) {
                float n = vnoise(nx / 6, ny / 6);
                r = 220 * n + 30; g = 80 * n + 40; b = 60;
            } else if (mx >= 0 && mx < 120 && my >= 0 && my < 110) {
                int chk = (((int)mx / 12) + ((int)my / 12)) & 1;
                float n = vnoise(mx / 4, my / 4);
                r = 40 + 60 * n; g = chk ? 200 : 120; b = 90 + 100 * n;
            } else {
                float bx = x + view * dispBg;
                float n = vnoise(bx / 10, y / 10) * 0.6f + vnoise(bx / 3, y / 3) * 0.4f;
                r = 90 + 120 * n; g = 110 + 90 * n; b = 160 + 80 * n * (y / 240.f);
            }
            // slight exposure difference between cameras
            float gain = 1.0f + 0.04f * view;
            int R = (int)fminf(255, r * gain), G = (int)fminf(255, g * gain), B = (int)fminf(255, b * gain);
            out[y * W + x] = (uint16_t)(((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3));
        }
}

// ----------------------------------------------------------------------------
// Minimal independent GIF decoder (enough for our files)
// ----------------------------------------------------------------------------

typedef struct { const uint8_t *p, *end; } Rd;
static int rd8(Rd *r) { return r->p < r->end ? *r->p++ : -1; }
static int rd16(Rd *r) { int a = rd8(r), b = rd8(r); return a | (b << 8); }

static int lzw_decode(Rd *r, uint8_t *out, int npix)
{
    int minSize = rd8(r);
    // gather sub-blocks
    size_t cap = 1 << 20, len = 0;
    uint8_t *data = malloc(cap);
    for (;;) {
        int n = rd8(r);
        if (n <= 0) break;
        if (len + n > cap) { cap *= 2; data = realloc(data, cap); }
        memcpy(data + len, r->p, n); r->p += n; len += n;
    }
    int clear = 1 << minSize, eoi = clear + 1;
    static uint16_t prefix[4096];
    static uint8_t suffix[4096], stack[4097];
    int size = minSize + 1, next = eoi + 1, old = -1, first = 0;
    size_t bitpos = 0;
    int o = 0;
    for (;;) {
        if (bitpos + size > len * 8) { free(data); return -1; }
        int code = 0;
        for (int i = 0; i < size; i++, bitpos++)
            code |= ((data[bitpos >> 3] >> (bitpos & 7)) & 1) << i;
        if (code == clear) { size = minSize + 1; next = eoi + 1; old = -1; continue; }
        if (code == eoi) break;
        if (old < 0) {
            if (code >= clear) { free(data); return -2; }
            out[o++] = (uint8_t)code; old = code; first = code; continue;
        }
        int in = code, sp = 0;
        if (code > next) { free(data); return -3; }
        if (code == next) { stack[sp++] = (uint8_t)first; code = old; }
        while (code >= clear) { stack[sp++] = suffix[code]; code = prefix[code]; }
        stack[sp++] = (uint8_t)code;
        first = code;
        while (sp) { if (o < npix) out[o++] = stack[--sp]; else sp--; }
        if (next < 4096) {
            prefix[next] = (uint16_t)old; suffix[next] = (uint8_t)first; next++;
            if (next >= (1 << size) && size < 12) size++;
        }
        old = in;
    }
    free(data);
    return o;
}

static int verify_gif(const char *path, int expW, int expH, uint8_t *const *frames,
                      const int *delays, int nframes)
{
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *buf = malloc(sz);
    fread(buf, 1, sz, f); fclose(f);
    Rd r = {buf, buf + sz};
    if (memcmp(buf, "GIF89a", 6)) { printf("  bad header\n"); return 0; }
    r.p += 6;
    int w = rd16(&r), h = rd16(&r), flags = rd8(&r);
    rd8(&r); rd8(&r);
    if (w != expW || h != expH) { printf("  size %dx%d != %dx%d\n", w, h, expW, expH); return 0; }
    if (flags & 0x80) r.p += 3 << ((flags & 7) + 1);
    int fi = 0, delay = 0, loop = 0, ok = 1;
    uint8_t *px = malloc(w * h);
    for (;;) {
        int b = rd8(&r);
        if (b == 0x3B || b < 0) break;
        if (b == 0x21) {
            int label = rd8(&r);
            if (label == 0xF9) { rd8(&r); rd8(&r); delay = rd16(&r); rd8(&r); rd8(&r); }
            else {
                for (;;) { int n = rd8(&r); if (n <= 0) break;
                    if (label == 0xFF && n == 11 && !memcmp(r.p, "NETSCAPE2.0", 11)) loop = 1;
                    r.p += n; }
            }
        } else if (b == 0x2C) {
            rd16(&r); rd16(&r); int fw = rd16(&r), fh = rd16(&r); rd8(&r);
            memset(px, 0xEE, w * h);
            int got = lzw_decode(&r, px, fw * fh);
            if (fi >= nframes) { printf("  too many frames\n"); ok = 0; break; }
            if (got != fw * fh || memcmp(px, frames[fi], fw * fh)) {
                printf("  frame %d mismatch (decoded %d px)\n", fi, got); ok = 0;
            }
            if (delay != delays[fi]) { printf("  frame %d delay %d != %d\n", fi, delay, delays[fi]); ok = 0; }
            fi++;
        } else { printf("  unexpected block 0x%02x\n", b); ok = 0; break; }
    }
    if (fi != nframes) { printf("  decoded %d frames, expected %d\n", fi, nframes); ok = 0; }
    if (!loop) { printf("  missing loop extension\n"); ok = 0; }
    printf("  %s: %ld bytes, %d frames, %dx%d -> %s\n", path, sz, fi, w, h, ok ? "OK" : "FAIL");
    free(px); free(buf);
    return ok;
}

// Re-run the encode pipeline exactly like wg_make_gif to get reference frames.
static int roundtrip_ms(const WgScene *s, int ax, int ay, int n, int mode, int ms, const char *path)
{
    if (!wg_make_gif(s, ax, ay, n, mode, ms, path, NULL, NULL)) { printf("  make_gif failed\n"); return 0; }
    int seq[WG_MAX_SEQ], delays[WG_MAX_SEQ];
    int count = wg_sequence(n, ms, seq, delays);
    WgRect r; wg_crop(s, ax, ay, &r);
    size_t np = (size_t)r.w * r.h;
    uint32_t *hist = calloc(32768, 4);
    uint8_t *rgb[WG_MAX_INBETWEEN + 2], *idx[WG_MAX_INBETWEEN + 2], *sf[WG_MAX_SEQ];
    for (int k = 0; k < n + 2; k++) {
        rgb[k] = malloc(np * 3);
        wg_render(s, ax, ay, k * 256 / (n + 1), mode, 1, rgb[k]);
        wg_hist_add(hist, rgb[k], (int)np);
    }
    WgPalette *p = malloc(sizeof(*p));
    wg_palette_from_hist(p, hist);
    for (int k = 0; k < n + 2; k++) { idx[k] = malloc(np); wg_index_frame(p, rgb[k], r.w, r.h, 1, idx[k]); }
    for (int j = 0; j < count; j++) sf[j] = idx[seq[j]];
    int ok = verify_gif(path, r.w, r.h, sf, delays, count);
    // The app's own gallery reader must reproduce every frame exactly.
    {
        WgGif g;
        int lok = wg_gif_load(path, &g, 400, 240, 32);
        int bad = !lok || g.nframes != count || g.w != r.w || g.h != r.h;
        for (int j = 0; !bad && j < count; j++) {
            if (g.delay_cs[j] != delays[j]) bad = 1;
            for (size_t i = 0; !bad && i < np; i++) {
                const uint8_t *c = p->pal + sf[j][i] * 3;
                uint16_t want = (uint16_t)(((c[0] >> 3) << 11) | ((c[1] >> 2) << 5) | (c[2] >> 3));
                if (g.frames[j][i] != want) bad = 1;
            }
        }
        printf("  gallery reader: %s\n", bad ? "MISMATCH" : "frames identical");
        if (bad) ok = 0;
        wg_gif_free(&g);
    }
    // Also try an LZW stress case: random pixels force many dictionary resets.
    for (int k = 0; k < n + 2; k++) { free(rgb[k]); free(idx[k]); }
    free(hist); free(p);
    return ok;
}

static int roundtrip(const WgScene *s, int ax, int ay, int n, int mode, const char *path)
{
    return roundtrip_ms(s, ax, ay, n, mode, 120, path);
}

int main(int argc, char **argv)
{
    const char *dir = argc > 1 ? argv[1] : ".";
    static uint16_t f1[W * H], f2[W * H];
    render_view(f1, 0);
    render_view(f2, 1);

    WgScene s;
    if (!wg_scene_init(&s, f1, f2, W, H)) { printf("init failed\n"); return 1; }
    wg_compute_disparity(&s);

    int fails = 0;
    // Disparity sanity: raw d should be about -40 (near), -22 (mid), -4 (bg).
    struct { const char *name; int x, y, expect; } probes[] = {
        {"near circle", 120, 140, -40}, {"mid rect", 290, 115, -22}, {"background", 40, 30, -4},
        {"background2", 360, 210, -4},
    };
    for (int i = 0; i < 4; i++) {
        float d = s.dx[probes[i].y * W + probes[i].x] / 256.f;
        int good = fabsf(d - probes[i].expect) <= 3;
        printf("disparity %-12s %6.1f (expect %d) %s\n", probes[i].name, d, probes[i].expect, good ? "ok" : "BAD");
        fails += !good;
    }

    // Focal point on the mid rectangle -> alignment ~ +22.
    int x2, y2;
    wg_match_point(&s, 290, 115, &x2, &y2);
    int ax = 290 - x2, ay = 115 - y2;
    printf("focal match: (290,115) -> (%d,%d), offset %d,%d %s\n", x2, y2, ax, ay,
           (ax == 22 && ay == 0) ? "ok" : "BAD");
    fails += !(ax == 22 && ay == 0);

    // Timing: every step time the app allows (40..500 ms, 10 ms steps), every count.
    {
        int bad = 0, checked = 0;
        for (int ms = 40; ms <= 500; ms += 10)
            for (int n = 0; n <= wg_max_inbetweens(ms); n++) {
                int seq[WG_MAX_SEQ], d[WG_MAX_SEQ];
                int c = wg_sequence(n, ms, seq, d);
                int fwd = 0, back = 0, minD = 999;
                for (int j = 0; j <= n; j++) fwd += d[j];
                for (int j = n + 1; j < c; j++) back += d[j];
                for (int j = 0; j < c; j++) if (d[j] < minD) minD = d[j];
                if (c != 2 * n + 2 || fwd * 10 != ms || back * 10 != ms || minD < 2) {
                    printf("timing BAD ms=%d n=%d frames=%d fwd=%d back=%d min=%d\n", ms, n, c, fwd, back, minD);
                    bad++;
                }
                checked++;
            }
        printf("timing: %d combinations checked, max in-betweens: 40ms=%d 120ms=%d 240ms=%d 500ms=%d %s\n",
               checked, wg_max_inbetweens(40), wg_max_inbetweens(120), wg_max_inbetweens(240),
               wg_max_inbetweens(500), bad ? "BAD" : "ok");
        fails += bad;
    }

    // Morph check: at the focal point the in-between equals frame 1.
    {
        WgRect r; wg_crop(&s, ax, ay, &r);
        uint8_t *a = malloc(r.w * r.h * 3), *b = malloc(r.w * r.h * 3);
        wg_render(&s, ax, ay, 0, WG_MODE_MORPH, 1, a);
        wg_render(&s, ax, ay, 128, WG_MODE_MORPH, 1, b);
        int px = 290 - r.x, py = 115 - r.y, diff = 0;
        for (int k = 0; k < 3; k++) diff += abs(a[(py * r.w + px) * 3 + k] - b[(py * r.w + px) * 3 + k]);
        printf("focal point stable across in-betweens: diff %d %s\n", diff, diff < 30 ? "ok" : "BAD");
        fails += diff >= 30;
        free(a); free(b);
    }

    // Quality vs ground truth: the true in-between view at t is the scene seen
    // from a camera t of the way across, shifted by t*alignment.
    {
        WgRect r; wg_crop(&s, ax, ay, &r);
        static uint16_t gt565[W * H];
        uint8_t *img = malloc(r.w * r.h * 3);
        const int ts[] = {64, 128, 192};
        for (int ti = 0; ti < 3; ti++) {
            int t = ts[ti];
            render_view(gt565, t / 256.f);
            double err[2] = {0, 0};
            for (int m = 0; m < 2; m++) {
                wg_render(&s, ax, ay, t, m ? WG_MODE_MORPH : WG_MODE_CROSSFADE, 1, img);
                double se = 0;
                long cnt = 0;
                for (int y = 0; y < r.h; y++)
                    for (int x = 8; x < r.w - 8; x++) {
                        int gx = (int)lroundf(r.x + x - t / 256.f * ax), gyy = r.y + y;
                        if (gx < 0 || gx >= W) continue;
                        uint16_t g = gt565[gyy * W + gx];
                        int gr = ((g >> 11) & 31) << 3, gg = ((g >> 5) & 63) << 2, gb = (g & 31) << 3;
                        const uint8_t *p = img + (y * r.w + x) * 3;
                        se += (p[0] - gr) * (p[0] - gr) + (p[1] - gg) * (p[1] - gg) + (p[2] - gb) * (p[2] - gb);
                        cnt += 3;
                    }
                err[m] = 10 * log10(255.0 * 255.0 / (se / cnt));
            }
            int good = err[1] > err[0] + 3;
            printf("in-between t=%.2f  PSNR crossfade %.1f dB, morph %.1f dB %s\n", t / 256.f, err[0], err[1],
                   good ? "ok" : "BAD");
            fails += !good;
            if (t == 128) {
                // save the middle frame (morph and crossfade side by side) for eyeballing
                int cw = r.w * 2 + 4;
                uint8_t *both = calloc(cw * r.h * 3, 1);
                for (int m = 0; m < 2; m++) {
                    wg_render(&s, ax, ay, t, m ? WG_MODE_MORPH : WG_MODE_CROSSFADE, 1, img);
                    for (int y = 0; y < r.h; y++)
                        memcpy(both + (y * cw + m * (r.w + 4)) * 3, img + y * r.w * 3, r.w * 3);
                }
                uint32_t *hist = calloc(32768, 4);
                wg_hist_add(hist, both, cw * r.h);
                WgPalette *p = malloc(sizeof(*p));
                wg_palette_from_hist(p, hist);
                uint8_t *idx = malloc(cw * r.h);
                wg_index_frame(p, both, cw, r.h, 0, idx);
                char pth[512];
                snprintf(pth, sizeof(pth), "%s/mid_crossfade_vs_morph.gif", dir);
                FILE *f = fopen(pth, "wb");
                int dl[1] = {100};
                uint8_t *fr[1] = {idx};
                wg_gif_write(f, cw, r.h, p, fr, dl, 1);
                fclose(f);
                free(both); free(hist); free(p); free(idx);
            }
        }
        free(img);
    }

    char path[512];
    const int ns[] = {0, 3, 5};
    for (int i = 0; i < 3; i++) {
        snprintf(path, sizeof(path), "%s/test_morph_n%d.gif", dir, ns[i]);
        fails += !roundtrip(&s, ax, ay, ns[i], WG_MODE_MORPH, path);
    }
    snprintf(path, sizeof(path), "%s/test_crossfade_n3.gif", dir);
    fails += !roundtrip(&s, ax, ay, 3, WG_MODE_CROSSFADE, path);
    // focal point on the near circle instead
    wg_match_point(&s, 120, 140, &x2, &y2);
    snprintf(path, sizeof(path), "%s/test_focal_near_n3.gif", dir);
    fails += !roundtrip(&s, 120 - x2, 140 - y2, 3, WG_MODE_MORPH, path);

    snprintf(path, sizeof(path), "%s/test_morph_n11_240ms.gif", dir);
    fails += !roundtrip_ms(&s, ax, ay, 11, WG_MODE_MORPH, 240, path);

    // LZW stress test with noise (forces dictionary clears).
    {
        WgPalette *p = calloc(1, sizeof(*p));
        for (int i = 0; i < 256; i++) p->pal[i * 3] = (uint8_t)i;
        uint8_t *fr[2]; int dl[2] = {12, 12};
        for (int k = 0; k < 2; k++) {
            fr[k] = malloc(W * H);
            for (int i = 0; i < W * H; i++) fr[k][i] = (uint8_t)(hash2(i, k) % (k ? 256 : 7));
        }
        snprintf(path, sizeof(path), "%s/test_lzw_noise.gif", dir);
        FILE *f = fopen(path, "wb");
        wg_gif_write(f, W, H, p, fr, dl, 2);
        fclose(f);
        fails += !verify_gif(path, W, H, fr, dl, 2);
        // tiny images
        for (int n = 1; n <= 300; n += 37) {
            uint8_t *tiny[1] = {fr[1]};
            f = fopen(path, "wb");
            wg_gif_write(f, n, 1, p, tiny, dl, 1);
            fclose(f);
            if (!verify_gif(path, n, 1, tiny, dl, 1)) fails++;
        }
        free(fr[0]); free(fr[1]); free(p);
    }

    wg_scene_free(&s);
    printf("\n%s (%d failures)\n", fails ? "FAILED" : "ALL PASSED", fails);
    return fails ? 1 : 0;
}
