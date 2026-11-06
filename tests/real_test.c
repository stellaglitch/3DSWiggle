// Run the full wiggle pipeline on a real stereo pair (raw 400x240 RGB565 files).
//   real_test frame1.565 frame2.565 focal_x focal_y out_dir
// Writes: real_wiggle.gif (morph, 3 in-betweens), real_mid.gif
// (middle frame: crossfade | morph side by side).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "wiggle.h"

#define W 400
#define H 240

static int load(const char *p, uint16_t *buf)
{
    FILE *f = fopen(p, "rb");
    if (!f) return 0;
    size_t n = fread(buf, 2, W * H, f);
    fclose(f);
    return n == W * H;
}

int main(int argc, char **argv)
{
    if (argc < 6) { fprintf(stderr, "usage: real_test f1.565 f2.565 fx fy outdir\n"); return 2; }
    static uint16_t f1[W * H], f2[W * H];
    if (!load(argv[1], f1) || !load(argv[2], f2)) { fprintf(stderr, "load failed\n"); return 1; }
    int fx = atoi(argv[3]), fy = atoi(argv[4]);
    const char *dir = argv[5];

    WgScene s;
    wg_scene_init(&s, f1, f2, W, H);
    clock_t t0 = clock();
    wg_compute_disparity(&s);
    printf("disparity: %.0f ms on PC, gy=%d nsign=%d gain=%d,%d,%d\n",
           (clock() - t0) * 1000.0 / CLOCKS_PER_SEC, s.gy, s.nsign, s.gain[0], s.gain[1], s.gain[2]);
    int x2, y2;
    wg_match_point(&s, fx, fy, &x2, &y2);
    int ax = fx - x2, ay = fy - y2;
    printf("focal (%d,%d) -> (%d,%d), offset %d,%d\n", fx, fy, x2, y2, ax, ay);

    char path[512];
    snprintf(path, sizeof(path), "%s/real_wiggle.gif", dir);
    printf("gif: %s\n", wg_make_gif(&s, ax, ay, 3, WG_MODE_MORPH, 120, path, NULL, NULL) ? "ok" : "FAILED");

    // Middle frame, crossfade | morph, plus a disparity visualisation row.
    WgRect r;
    wg_crop(&s, ax, ay, &r);
    int cw = r.w * 2 + 4, ch = r.h * 2 + 4;
    uint8_t *both = calloc((size_t)cw * ch * 3, 1), *img = malloc((size_t)r.w * r.h * 3);
    for (int m = 0; m < 2; m++) {
        wg_render(&s, ax, ay, 128, m ? WG_MODE_MORPH : WG_MODE_CROSSFADE, 1, img);
        for (int y = 0; y < r.h; y++)
            memcpy(both + ((size_t)y * cw + m * (r.w + 4)) * 3, img + (size_t)y * r.w * 3, r.w * 3);
    }
    // disparity maps (frame-1 and frame-2 reference), brighter = nearer
    int lo = 1 << 30, hi = -(1 << 30);
    for (int i = 0; i < W * H; i++) { int n = s.nsign * s.dx[i]; if (n < lo) lo = n; if (n > hi) hi = n; }
    for (int m = 0; m < 2; m++)
        for (int y = 0; y < r.h; y++)
            for (int x = 0; x < r.w; x++) {
                int n = s.nsign * (m ? s.dx2 : s.dx)[(r.y + y) * W + r.x + x];
                int v = hi > lo ? (n - lo) * 255 / (hi - lo) : 0;
                uint8_t *p = both + ((size_t)(y + r.h + 4) * cw + m * (r.w + 4) + x) * 3;
                p[0] = p[1] = p[2] = (uint8_t)v;
            }
    uint32_t *hist = calloc(32768, 4);
    wg_hist_add(hist, both, cw * ch);
    WgPalette *p = malloc(sizeof(*p));
    wg_palette_from_hist(p, hist);
    uint8_t *idx = malloc((size_t)cw * ch);
    wg_index_frame(p, both, cw, ch, 0, idx);
    snprintf(path, sizeof(path), "%s/real_mid.gif", dir);
    FILE *f = fopen(path, "wb");
    int dl[1] = {100};
    uint8_t *fr[1] = {idx};
    wg_gif_write(f, cw, ch, p, fr, dl, 1);
    fclose(f);
    printf("wrote %s\n", path);
    wg_scene_free(&s);
    return 0;
}
