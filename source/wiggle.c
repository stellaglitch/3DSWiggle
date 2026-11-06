// 3DSWiggle - portable wigglegram core. See wiggle.h.
#include "wiggle.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline int iabs(int v) { return v < 0 ? -v : v; }

// ---------------------------------------------------------------------------
// Scene
// ---------------------------------------------------------------------------

static void unpack565(const uint16_t *src, uint8_t *rgb, uint8_t *y, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        unsigned v = src[i];
        unsigned r = (v >> 11) & 31, g = (v >> 5) & 63, b = v & 31;
        unsigned r8 = (r << 3) | (r >> 2), g8 = (g << 2) | (g >> 4), b8 = (b << 3) | (b >> 2);
        rgb[i * 3 + 0] = (uint8_t)r8;
        rgb[i * 3 + 1] = (uint8_t)g8;
        rgb[i * 3 + 2] = (uint8_t)b8;
        y[i] = (uint8_t)((r8 * 77 + g8 * 150 + b8 * 29) >> 8);
    }
}

int wg_scene_init(WgScene *s, const uint16_t *f1, const uint16_t *f2, int w, int h)
{
    memset(s, 0, sizeof(*s));
    size_t n = (size_t)w * h;
    s->w = w;
    s->h = h;
    s->rgb1 = malloc(n * 3);
    s->rgb2 = malloc(n * 3);
    s->y1 = malloc(n);
    s->y2 = malloc(n);
    s->dx = calloc(n, sizeof(int16_t));
    s->dx2 = calloc(n, sizeof(int16_t));
    if (!s->rgb1 || !s->rgb2 || !s->y1 || !s->y2 || !s->dx || !s->dx2) {
        wg_scene_free(s);
        return 0;
    }
    unpack565(f1, s->rgb1, s->y1, n);
    unpack565(f2, s->rgb2, s->y2, n);
    s->nsign = 1;
    s->gain[0] = s->gain[1] = s->gain[2] = 256;
    return 1;
}

void wg_scene_free(WgScene *s)
{
    free(s->rgb1); free(s->rgb2); free(s->y1); free(s->y2); free(s->dx); free(s->dx2);
    memset(s, 0, sizeof(*s));
}

// ---------------------------------------------------------------------------
// Focal point matching (mean-removed SAD template search, full resolution)
// ---------------------------------------------------------------------------

int wg_match_point(const WgScene *s, int x1, int y1, int *x2, int *y2)
{
    const int R = 10;           // 21x21 patch
    const int SX = 100, SY = 16; // search window
    const int w = s->w, h = s->h;
    x1 = clampi(x1, R, w - 1 - R);
    y1 = clampi(y1, R, h - 1 - R);

    int sum1 = 0;
    for (int j = -R; j <= R; j++)
        for (int i = -R; i <= R; i++)
            sum1 += s->y1[(y1 + j) * w + x1 + i];
    const int area = (2 * R + 1) * (2 * R + 1);
    const int m1 = sum1 / area;

    int best = INT_MAX, bx = x1, by = y1;
    for (int dy = -SY; dy <= SY; dy++) {
        int cy = y1 + dy;
        if (cy - R < 0 || cy + R >= h) continue;
        for (int dx = -SX; dx <= SX; dx++) {
            int cx = x1 + dx;
            if (cx - R < 0 || cx + R >= w) continue;
            int sum2 = 0;
            for (int j = -R; j <= R; j++) {
                const uint8_t *row = s->y2 + (cy + j) * w + cx - R;
                for (int i = 0; i <= 2 * R; i++) sum2 += row[i];
            }
            int m2 = sum2 / area;
            int cost = 0;
            for (int j = -R; j <= R && cost < best; j++) {
                const uint8_t *r1 = s->y1 + (y1 + j) * w + x1 - R;
                const uint8_t *r2 = s->y2 + (cy + j) * w + cx - R;
                for (int i = 0; i <= 2 * R; i++) cost += iabs((r1[i] - m1) - (r2[i] - m2));
            }
            // Slight preference for small vertical offsets (cameras are side by side).
            cost += iabs(dy) * 4;
            if (cost < best) { best = cost; bx = cx; by = cy; }
        }
    }
    *x2 = bx;
    *y2 = by;
    return best;
}

// ---------------------------------------------------------------------------
// Depth (disparity)
//
//  1. Coarse block matching finds the disparity range and the global vertical
//     offset between the two cameras.
//  2. Dense semi-global matching (census cost, 4 paths) at half resolution
//     gives a disparity per pixel with sharp object edges.
//  3. A left/right consistency check rejects occluded and mismatched pixels;
//     those are filled from the background side, then median filtered.
// ---------------------------------------------------------------------------

#define DB  8    // coarse block size at half resolution
#define DSX 44   // coarse horizontal search at half res (+-88 px full res)
#define DSY 3    // coarse vertical search at half res
#define SGM_P1 12
#define SGM_P2 80

static int cmp_int(const void *a, const void *b)
{
    int x = *(const int *)a, y = *(const int *)b;
    return (x > y) - (x < y);
}

static void coarse_estimate(const uint8_t *l1, const uint8_t *l2, int hw, int hh,
                            int *dmin, int *dmax, int *gyHalf, int *median)
{
    const int gw = hw / DB, gh = hh / DB;
    int *vx = malloc(sizeof(int) * gw * gh), *vy = malloc(sizeof(int) * gw * gh);
    int nv = 0;
    static int costs[2 * DSX + 1];
    *dmin = -40; *dmax = 40; *gyHalf = 0; *median = 0;
    if (!vx || !vy) { free(vx); free(vy); return; }

    for (int by = 0; by < gh; by++)
        for (int bx = 0; bx < gw; bx++) {
            const int x0 = bx * DB, y0 = by * DB;
            int texture = 0, sum1 = 0;
            for (int j = 0; j < DB; j++)
                for (int i = 0; i < DB; i++) {
                    int p = (y0 + j) * hw + x0 + i;
                    sum1 += l1[p];
                    if (x0 + i + 1 < hw) texture += iabs(l1[p + 1] - l1[p]);
                    if (y0 + j + 1 < hh) texture += iabs(l1[p + hw] - l1[p]);
                }
            if (texture < DB * DB * 3) continue;
            const int m1 = sum1 / (DB * DB);
            int best = INT_MAX, second = INT_MAX, bdx = 0, bdy = 0;
            for (int dy = -DSY; dy <= DSY; dy++) {
                int cy = y0 + dy;
                if (cy < 0 || cy + DB > hh) continue;
                int rowBest = INT_MAX, rowDx = 0;
                for (int dx = -DSX; dx <= DSX; dx++) {
                    int cx = x0 + dx;
                    costs[dx + DSX] = INT_MAX;
                    if (cx < 0 || cx + DB > hw) continue;
                    int sum2 = 0;
                    for (int j = 0; j < DB; j++)
                        for (int i = 0; i < DB; i++) sum2 += l2[(cy + j) * hw + cx + i];
                    int m2 = sum2 / (DB * DB), c = 0;
                    for (int j = 0; j < DB; j++) {
                        const uint8_t *r1 = l1 + (y0 + j) * hw + x0;
                        const uint8_t *r2 = l2 + (cy + j) * hw + cx;
                        for (int i = 0; i < DB; i++) c += iabs((r1[i] - m1) - (r2[i] - m2));
                    }
                    c += iabs(dy) * 16;
                    costs[dx + DSX] = c;
                    if (c < rowBest) { rowBest = c; rowDx = dx; }
                }
                if (rowBest < best) {
                    int sec = INT_MAX;
                    for (int dx = -DSX; dx <= DSX; dx++)
                        if (iabs(dx - rowDx) > 2 && costs[dx + DSX] < sec) sec = costs[dx + DSX];
                    best = rowBest; second = sec; bdx = rowDx; bdy = dy;
                }
            }
            if (best == INT_MAX || (second != INT_MAX && second * 10 <= best * 11)) continue;
            vx[nv] = bdx; vy[nv] = bdy; nv++;
        }

    if (nv >= 6) {
        qsort(vx, nv, sizeof(int), cmp_int);
        qsort(vy, nv, sizeof(int), cmp_int);
        int lo = vx[nv * 2 / 100], hi = vx[(nv * 98) / 100];
        *median = vx[nv / 2];
        *gyHalf = vy[nv / 2];
        *dmin = clampi(lo - 8, -DSX - 4, DSX);
        *dmax = clampi(hi + 8, *dmin + 16, DSX + 4);
    }
    free(vx);
    free(vy);
}

static void census5(const uint8_t *img, int w, int h, uint32_t *out)
{
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            uint32_t bits = 0;
            if (y >= 2 && y < h - 2 && x >= 2 && x < w - 2) {
                const int c = img[y * w + x];
                for (int j = -2; j <= 2; j++)
                    for (int i = -2; i <= 2; i++)
                        if (i || j) bits = (bits << 1) | (img[(y + j) * w + x + i] < c);
            }
            out[y * w + x] = bits;
        }
}

static inline int popcount32(uint32_t v)
{
    v = v - ((v >> 1) & 0x55555555u);
    v = (v & 0x33333333u) + ((v >> 2) & 0x33333333u);
    return (int)((((v + (v >> 4)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24);
}

// Accumulate one SGM path into S. Pixels are visited in the given order:
// for horizontal paths each row is a scanline, for vertical paths each column.
static void sgm_scan(const uint8_t *C, uint16_t *S, const uint8_t *img, int w, int h, int nd,
                     int horizontal, int forward, uint16_t *Lbuf)
{
    // Lbuf: w*nd entries (one previous L per column for vertical paths, 1 for horizontal)
    const int outer = horizontal ? h : 1;
    for (int o = 0; o < outer; o++) {
        const int len = horizontal ? w : h;
        for (int s = 0; s < len; s++) {
            const int step = forward ? s : len - 1 - s;
            // For vertical scans we process a whole row at a time.
            const int xs = horizontal ? step : 0, xe = horizontal ? step + 1 : w;
            const int y = horizontal ? o : step;
            for (int x = xs; x < xe; x++) {
                uint16_t *L = horizontal ? Lbuf : Lbuf + x * nd;
                const uint8_t *c = C + ((size_t)y * w + x) * nd;
                uint16_t *acc = S + ((size_t)y * w + x) * nd;
                if (s == 0) {
                    for (int k = 0; k < nd; k++) { L[k] = c[k]; acc[k] += c[k]; }
                    continue;
                }
                const int px = horizontal ? (forward ? x - 1 : x + 1) : x;
                const int py = horizontal ? y : (forward ? y - 1 : y + 1);
                const int grad = iabs(img[y * w + x] - img[py * w + px]);
                const int p2 = grad > 10 ? SGM_P2 / 3 : SGM_P2;
                int minPrev = 65535;
                for (int k = 0; k < nd; k++) if (L[k] < minPrev) minPrev = L[k];
                int prevKm1 = 65535 - SGM_P1; // L[k-1] before overwrite
                for (int k = 0; k < nd; k++) {
                    int cur = L[k];
                    int m = cur;
                    if (prevKm1 + SGM_P1 < m) m = prevKm1 + SGM_P1;
                    if (k + 1 < nd && L[k + 1] + SGM_P1 < m) m = L[k + 1] + SGM_P1;
                    if (minPrev + p2 < m) m = minPrev + p2;
                    int v = c[k] + m - minPrev;
                    prevKm1 = cur;
                    L[k] = (uint16_t)v;
                    acc[k] += (uint16_t)v;
                }
            }
        }
    }
}

// Fill invalid disparities in each row from the background (less near) side.
static void fill_background(int *d, const uint8_t *valid, int w, int h, int nsign)
{
    for (int y = 0; y < h; y++) {
        int *row = d + y * w;
        const uint8_t *v = valid + y * w;
        int x = 0;
        while (x < w) {
            if (v[x]) { x++; continue; }
            int e = x;
            while (e < w && !v[e]) e++;
            int hasL = x > 0, hasR = e < w, fillv = 0;
            if (hasL && hasR) {
                int a = row[x - 1], b = row[e];
                fillv = (nsign * a < nsign * b) ? a : b;
            } else if (hasL) fillv = row[x - 1];
            else if (hasR) fillv = row[e];
            for (int i = x; i < e; i++) row[i] = fillv;
            x = e;
        }
    }
}

static void median3x3(int *g, int gw, int gh)
{
    int *tmp = malloc(sizeof(int) * gw * gh);
    if (!tmp) return;
    for (int y = 0; y < gh; y++)
        for (int x = 0; x < gw; x++) {
            int v[9], n = 0;
            for (int j = -1; j <= 1; j++)
                for (int i = -1; i <= 1; i++) {
                    int xx = clampi(x + i, 0, gw - 1), yy = clampi(y + j, 0, gh - 1);
                    v[n++] = g[yy * gw + xx];
                }
            // partial insertion sort is plenty for 9 values
            for (int i = 1; i < 9; i++)
                for (int k = i; k > 0 && v[k - 1] > v[k]; k--) { int t = v[k]; v[k] = v[k - 1]; v[k - 1] = t; }
            tmp[y * gw + x] = v[4];
        }
    memcpy(g, tmp, sizeof(int) * gw * gh);
    free(tmp);
}

void wg_compute_disparity(WgScene *s)
{
    const int w = s->w, h = s->h;
    const int hw = w / 2, hh = h / 2, np = hw * hh;
    uint8_t *l1 = malloc(np), *l2 = malloc(np), *l2s = malloc(np);
    uint32_t *c1 = malloc(np * 4), *c2 = malloc(np * 4);
    uint8_t *C = NULL, *T = NULL, *v1 = calloc(np, 1), *v2 = calloc(np, 1);
    uint16_t *S = NULL, *Lbuf = NULL;
    int *dL = malloc(np * sizeof(int)), *dR = malloc(np * sizeof(int));
    if (!l1 || !l2 || !l2s || !c1 || !c2 || !v1 || !v2 || !dL || !dR) goto done;

    // Per-channel gain so exposure/white-balance differences between the
    // cameras can be blended away in the in-between frames.
    {
        uint64_t m1[3] = {0, 0, 0}, m2[3] = {0, 0, 0};
        for (int i = 0; i < w * h; i++)
            for (int k = 0; k < 3; k++) { m1[k] += s->rgb1[i * 3 + k]; m2[k] += s->rgb2[i * 3 + k]; }
        for (int k = 0; k < 3; k++)
            s->gain[k] = m2[k] ? clampi((int)((m1[k] * 256 + m2[k] / 2) / m2[k]), 192, 340) : 256;
    }

    for (int y = 0; y < hh; y++)
        for (int x = 0; x < hw; x++) {
            int i = (2 * y) * w + 2 * x;
            l1[y * hw + x] = (s->y1[i] + s->y1[i + 1] + s->y1[i + w] + s->y1[i + w + 1] + 2) >> 2;
            l2[y * hw + x] = (s->y2[i] + s->y2[i + 1] + s->y2[i + w] + s->y2[i + w + 1] + 2) >> 2;
        }

    int dmin, dmax, gyH, med;
    coarse_estimate(l1, l2, hw, hh, &dmin, &dmax, &gyH, &med);
    const int nd = dmax - dmin + 1;
    s->gy = gyH * 2;
    s->nsign = med >= 0 ? 1 : -1;

    // Frame 2 with the global vertical offset removed (rectified rows).
    for (int y = 0; y < hh; y++)
        memcpy(l2s + y * hw, l2 + clampi(y + gyH, 0, hh - 1) * hw, hw);

    C = malloc((size_t)np * nd);
    T = calloc((size_t)np * nd, 1);
    S = calloc((size_t)np * nd, sizeof(uint16_t));
    Lbuf = malloc((size_t)hw * nd * sizeof(uint16_t));
    if (!C || !T || !S || !Lbuf) goto done;

    census5(l1, hw, hh, c1);
    census5(l2s, hw, hh, c2);
    for (int y = 0; y < hh; y++)
        for (int x = 0; x < hw; x++) {
            uint8_t *t = T + ((size_t)y * hw + x) * nd;
            const uint32_t a = c1[y * hw + x];
            for (int k = 0; k < nd; k++) {
                int xr = x + dmin + k;
                t[k] = (xr < 0 || xr >= hw) ? 24 : (uint8_t)popcount32(a ^ c2[y * hw + xr]);
            }
        }
    // 3x3 box aggregation (separable): T -> C (horizontal), C -> T (vertical)
    for (int y = 0; y < hh; y++)
        for (int x = 0; x < hw; x++) {
            const uint8_t *a = T + ((size_t)y * hw + clampi(x - 1, 0, hw - 1)) * nd;
            const uint8_t *b = T + ((size_t)y * hw + x) * nd;
            const uint8_t *c = T + ((size_t)y * hw + clampi(x + 1, 0, hw - 1)) * nd;
            uint8_t *o = C + ((size_t)y * hw + x) * nd;
            for (int k = 0; k < nd; k++) o[k] = (uint8_t)(a[k] + b[k] + c[k]);
        }
    for (int y = 0; y < hh; y++)
        for (int x = 0; x < hw; x++) {
            const uint8_t *a = C + ((size_t)clampi(y - 1, 0, hh - 1) * hw + x) * nd;
            const uint8_t *b = C + ((size_t)y * hw + x) * nd;
            const uint8_t *c = C + ((size_t)clampi(y + 1, 0, hh - 1) * hw + x) * nd;
            uint8_t *o = T + ((size_t)y * hw + x) * nd;
            for (int k = 0; k < nd; k++) o[k] = (uint8_t)(a[k] + b[k] + c[k]);
        }

    sgm_scan(T, S, l1, hw, hh, nd, 1, 1, Lbuf);
    sgm_scan(T, S, l1, hw, hh, nd, 1, 0, Lbuf);
    sgm_scan(T, S, l1, hw, hh, nd, 0, 1, Lbuf);
    sgm_scan(T, S, l1, hw, hh, nd, 0, 0, Lbuf);

    // Winner-takes-all, frame-1 reference, with sub-pixel parabola (1/256 half-px).
    for (int i = 0; i < np; i++) {
        const uint16_t *sv = S + (size_t)i * nd;
        int bk = 0;
        for (int k = 1; k < nd; k++) if (sv[k] < sv[bk]) bk = k;
        int sub = 0;
        if (bk > 0 && bk < nd - 1) {
            int den = sv[bk - 1] - 2 * sv[bk] + sv[bk + 1];
            if (den > 0) sub = clampi((sv[bk - 1] - sv[bk + 1]) * 128 / den, -128, 128);
        }
        dL[i] = (dmin + bk) * 256 + sub;
    }
    // Frame-2 reference: pixel xr matches frame-1 pixel xr - d.
    for (int y = 0; y < hh; y++)
        for (int xr = 0; xr < hw; xr++) {
            int best = INT_MAX, bd = 0;
            for (int k = 0; k < nd; k++) {
                int x = xr - (dmin + k);
                if (x < 0 || x >= hw) continue;
                int v = S[((size_t)y * hw + x) * nd + k];
                if (v < best) { best = v; bd = dmin + k; }
            }
            dR[y * hw + xr] = bd * 256;
        }
    // Left/right consistency.
    for (int y = 0; y < hh; y++)
        for (int x = 0; x < hw; x++) {
            int d = dL[y * hw + x];
            int xr = x + ((d + 128) >> 8);
            v1[y * hw + x] = xr >= 0 && xr < hw && iabs(dR[y * hw + xr] - d) <= 256;
            int e = dR[y * hw + x];
            int xl = x - ((e + 128) >> 8);
            v2[y * hw + x] = xl >= 0 && xl < hw && iabs(dL[y * hw + xl] - e) <= 256;
        }
    fill_background(dL, v1, hw, hh, s->nsign);
    fill_background(dR, v2, hw, hh, s->nsign);
    median3x3(dL, hw, hh);
    median3x3(dR, hw, hh);

    // Up to full resolution. Each full-res pixel picks, from the disparities of
    // its 3x3 half-res neighbourhood, the one that best matches the two photos.
    // This keeps object edges crisp instead of "fattening" foreground objects.
    for (int pass = 0; pass < 2; pass++) {
        const int *src = pass ? dR : dL;
        int16_t *dst = pass ? s->dx2 : s->dx;
        const uint8_t *A = pass ? s->rgb2 : s->rgb1, *B = pass ? s->rgb1 : s->rgb2;
        const int sgn = pass ? -1 : 1;   // frame 2 looks back into frame 1
        for (int y = 0; y < h; y++) {
            const int ya = pass ? clampi(y + s->gy, 0, h - 1) : y;     // row in A
            const int yb = pass ? y : clampi(y + s->gy, 0, h - 1);     // row in B
            const int hy = clampi(y >> 1, 0, hh - 1);
            for (int x = 0; x < w; x++) {
                const int hx = clampi(x >> 1, 0, hw - 1);
                const int centre = src[hy * hw + hx] * 2;
                int best = centre, bestCost = INT_MAX;
                for (int j = -1; j <= 1; j++)
                    for (int i = -1; i <= 1; i++) {
                        int d = src[clampi(hy + j, 0, hh - 1) * hw + clampi(hx + i, 0, hw - 1)] * 2;
                        if (d != centre && i == 0 && j == 0) continue;
                        int off = (d + 128) >> 8, cost = 0;
                        for (int k = -2; k <= 2; k++) {
                            int xa = clampi(x + k, 0, w - 1), xb = clampi(x + k + sgn * off, 0, w - 1);
                            const uint8_t *pa = A + (ya * w + xa) * 3, *pb = B + (yb * w + xb) * 3;
                            cost += iabs(pa[0] - pb[0]) + iabs(pa[1] - pb[1]) + iabs(pa[2] - pb[2]);
                        }
                        if (d == centre) cost -= 12; // mild preference for the SGM value
                        if (cost < bestCost) { bestCost = cost; best = d; }
                    }
                dst[y * w + x] = (int16_t)clampi(best, -32000, 32000);
            }
        }
    }
    s->has_disparity = 1;

done:
    free(l1); free(l2); free(l2s); free(c1); free(c2); free(C); free(T); free(S);
    free(Lbuf); free(v1); free(v2); free(dL); free(dR);
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

void wg_crop(const WgScene *s, int ax, int ay, WgRect *r)
{
    r->x = ax > 0 ? ax : 0;
    r->y = ay > 0 ? ay : 0;
    r->w = s->w - iabs(ax);
    r->h = s->h - iabs(ay);
    if (r->w < 1) r->w = 1;
    if (r->h < 1) r->h = 1;
}

// Sample an RGB888 image at 8.8 fixed-point coordinates (clamped to edges).
// Result channels are 8.8 fixed point.
static inline void sample(const uint8_t *img, int w, int h, int fx, int fy, int bil, int c[3])
{
    if (!bil) {
        int x = clampi((fx + 128) >> 8, 0, w - 1), y = clampi((fy + 128) >> 8, 0, h - 1);
        const uint8_t *p = img + (y * w + x) * 3;
        c[0] = p[0] << 8; c[1] = p[1] << 8; c[2] = p[2] << 8;
        return;
    }
    int x0 = fx >> 8, y0 = fy >> 8, tx = fx & 255, ty = fy & 255;
    int x1 = clampi(x0 + 1, 0, w - 1), y1 = clampi(y0 + 1, 0, h - 1);
    x0 = clampi(x0, 0, w - 1);
    y0 = clampi(y0, 0, h - 1);
    const uint8_t *p00 = img + (y0 * w + x0) * 3, *p01 = img + (y0 * w + x1) * 3;
    const uint8_t *p10 = img + (y1 * w + x0) * 3, *p11 = img + (y1 * w + x1) * 3;
    for (int k = 0; k < 3; k++) {
        int top = p00[k] * (256 - tx) + p01[k] * tx;
        int bot = p10[k] * (256 - tx) + p11[k] * tx;
        c[k] = (top * (256 - ty) + bot * ty) >> 8;
    }
}

#define WG_MAXW 1024

// Occlusion-aware view interpolation for one in-between frame.
//  a. Forward-splat both disparity maps to time t, keeping the nearest
//     surface per pixel (z-buffer), and fill the gaps from the background.
//  b. For each output pixel, look up where that surface is in each photo and
//     only use a photo if the surface is actually visible there.
static void render_morph(const WgScene *s, int ax, int ay, int t, int bil, const WgRect *r, uint8_t *out)
{
    const int w = s->w, h = s->h;
    const int A = ax * 256, Dy = (s->gy + ay) * 256, G = s->gy * 256;
    const int ns = s->nsign;
    static int wd[WG_MAXW], wn[WG_MAXW];
    int m1[3], m2[3];
    for (int k = 0; k < 3; k++) {
        // Exposure matching that is exact at the key frames:
        // frame 1 drifts towards frame 2's colour as t -> 1 and vice versa.
        int g = s->gain[k], inv = 65536 / g;
        m1[k] = (256 * (256 - t) + inv * t) >> 8;
        m2[k] = (g * (256 - t) + 256 * t) >> 8;
    }

    for (int y = 0; y < r->h; y++) {
        const int v = r->y + y;
        const int y1f = v * 256 - (t * Dy) / 256;          // frame-1 row (8.8)
        const int y1 = clampi((y1f + 128) >> 8, 0, h - 1);
        const int16_t *R1 = s->dx + y1 * w, *R2 = s->dx2 + y1 * w;

        for (int u = 0; u < w; u++) wn[u] = INT_MIN;
        for (int x = 0; x < w; x++) {
            int d = R1[x];
            int u = (x * 256 + (t * (d + A)) / 256 + 128) >> 8;
            int n = ns * d;
            if (u >= 0 && u < w && n > wn[u]) { wn[u] = n; wd[u] = d; }
        }
        for (int x = 0; x < w; x++) {
            int d = R2[x];
            int u = (x * 256 - ((256 - t) * d) / 256 + (t * A) / 256 + 128) >> 8;
            int n = ns * d;
            if (u >= 0 && u < w && n > wn[u]) { wn[u] = n; wd[u] = d; }
        }
        // Gaps (disoccluded areas): take the farther of the two neighbours.
        for (int u = 0; u < w;) {
            if (wn[u] != INT_MIN) { u++; continue; }
            int e = u;
            while (e < w && wn[e] == INT_MIN) e++;
            int f;
            if (u > 0 && e < w) f = (ns * wd[u - 1] < ns * wd[e]) ? wd[u - 1] : wd[e];
            else if (u > 0) f = wd[u - 1];
            else if (e < w) f = wd[e];
            else f = 0;
            for (int i = u; i < e; i++) { wd[i] = f; wn[i] = ns * f; }
            u = e;
        }

        uint8_t *o = out + (size_t)y * r->w * 3;
        for (int x = 0; x < r->w; x++, o += 3) {
            const int u = r->x + x;
            const int d = wd[u];
            const int x1 = u * 256 - (t * (d + A)) / 256;   // where it is in photo 1
            const int x2 = x1 + d;                          // and in photo 2
            const int i1 = (x1 + 128) >> 8, i2 = (x2 + 128) >> 8;
            int vis1 = i1 >= 0 && i1 < w && iabs(R1[i1] - d) <= 512;
            int vis2 = i2 >= 0 && i2 < w && iabs(R2[i2] - d) <= 512;
            int w1 = 256 - t, w2 = t;
            if (vis1 && !vis2) { w1 = 256; w2 = 0; }
            else if (vis2 && !vis1) { w1 = 0; w2 = 256; }
            int c1[3] = {0, 0, 0}, c2[3] = {0, 0, 0};
            if (w1) sample(s->rgb1, w, h, x1, y1f, bil, c1);
            if (w2) sample(s->rgb2, w, h, x2, y1f + G, bil, c2);
            for (int k = 0; k < 3; k++) {
                int a = (c1[k] >> 8) * m1[k], b = (c2[k] >> 8) * m2[k]; // 8.8
                int val = (a * w1 + b * w2 + 32768) >> 16;
                o[k] = (uint8_t)clampi(val, 0, 255);
            }
        }
    }
}

void wg_render(const WgScene *s, int ax, int ay, int t, int mode, int bilinear, uint8_t *out)
{
    WgRect r;
    wg_crop(s, ax, ay, &r);
    t = clampi(t, 0, 256);
    const int w = s->w, h = s->h;
    if (mode == WG_MODE_MORPH && s->has_disparity && t > 0 && t < 256 && w <= WG_MAXW) {
        render_morph(s, ax, ay, t, bilinear, &r, out);
        return;
    }
    // Key frames (exact) and plain crossfade.
    const int A = ax * 256, B = ay * 256;
    int c1[3] = {0, 0, 0}, c2[3] = {0, 0, 0};
    for (int y = 0; y < r.h; y++) {
        const int py = r.y + y;
        uint8_t *o = out + (size_t)y * r.w * 3;
        for (int x = 0; x < r.w; x++, o += 3) {
            const int px = r.x + x;
            if (t < 256) sample(s->rgb1, w, h, px * 256, py * 256, 0, c1);
            if (t > 0) sample(s->rgb2, w, h, px * 256 - A, py * 256 - B, 0, c2);
            for (int k = 0; k < 3; k++) {
                int v = (c1[k] * (256 - t) + c2[k] * t + 32768) >> 16;
                o[k] = (uint8_t)clampi(v, 0, 255);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

int wg_max_inbetweens(int step_ms)
{
    int T = (step_ms + 5) / 10;
    int n = T / 2 - 1; // every frame must be >= 2 cs (browsers clamp shorter delays)
    return clampi(n, 0, WG_MAX_INBETWEEN);
}

int wg_sequence(int inbetweens, int step_ms, int *seq, int *delay_cs)
{
    int n = clampi(inbetweens, 0, wg_max_inbetweens(step_ms));
    int T = (step_ms + 5) / 10;
    if (T < 2) T = 2;
    int steps[WG_MAX_INBETWEEN + 1];
    for (int i = 0; i <= n; i++) {
        int a = ((i + 1) * T * 2 + (n + 1)) / ((n + 1) * 2);
        int b = (i * T * 2 + (n + 1)) / ((n + 1) * 2);
        steps[i] = a - b < 2 ? 2 : a - b;
    }
    int count = 0;
    for (int k = 0; k <= n + 1; k++) seq[count++] = k;   // 0 .. n+1 (frame 1 -> frame 2)
    for (int k = n; k >= 1; k--) seq[count++] = k;       // n .. 1   (back towards frame 1)
    for (int j = 0; j < count; j++) {
        int step = j <= n ? j : (2 * n + 1 - j); // mirror the forward steps on the way back
        delay_cs[j] = steps[clampi(step, 0, n)];
    }
    return count;
}

// ---------------------------------------------------------------------------
// Palette (median cut on an RGB555 histogram)
// ---------------------------------------------------------------------------

void wg_hist_add(uint32_t *hist, const uint8_t *rgb, int npix)
{
    for (int i = 0; i < npix; i++, rgb += 3)
        hist[((rgb[0] >> 3) << 10) | ((rgb[1] >> 3) << 5) | (rgb[2] >> 3)]++;
}

typedef struct { uint16_t c; uint32_t n; } HEnt;
typedef struct { int lo, hi; } Box; // [lo, hi)
static int g_axis;
static int cmp_axis(const void *a, const void *b)
{
    int sh = (2 - g_axis) * 5;
    int va = (((const HEnt *)a)->c >> sh) & 31, vb = (((const HEnt *)b)->c >> sh) & 31;
    return va - vb;
}

static void box_stats(const HEnt *e, const Box *b, int *axis, int *range, uint64_t *count)
{
    int mn[3] = {31, 31, 31}, mx[3] = {0, 0, 0};
    uint64_t n = 0;
    for (int i = b->lo; i < b->hi; i++) {
        for (int k = 0; k < 3; k++) {
            int v = (e[i].c >> ((2 - k) * 5)) & 31;
            if (v < mn[k]) mn[k] = v;
            if (v > mx[k]) mx[k] = v;
        }
        n += e[i].n;
    }
    // Weight green a little higher (perceptual).
    int rr = (mx[0] - mn[0]) * 3, rg = (mx[1] - mn[1]) * 4, rb = (mx[2] - mn[2]) * 2;
    *axis = 0; *range = rr;
    if (rg > *range) { *axis = 1; *range = rg; }
    if (rb > *range) { *axis = 2; *range = rb; }
    *count = n;
}

void wg_palette_from_hist(WgPalette *p, const uint32_t *hist)
{
    memset(p, 0, sizeof(*p));
    HEnt *e = malloc(sizeof(HEnt) * 32768);
    if (!e) return;
    int ne = 0;
    for (int c = 0; c < 32768; c++)
        if (hist[c]) { e[ne].c = (uint16_t)c; e[ne].n = hist[c]; ne++; }
    if (ne == 0) { e[0].c = 0; e[0].n = 1; ne = 1; }

    Box boxes[256];
    int nb = 1;
    boxes[0].lo = 0; boxes[0].hi = ne;
    while (nb < 256) {
        int bi = -1, baxis = 0;
        double bscore = 0;
        for (int i = 0; i < nb; i++) {
            if (boxes[i].hi - boxes[i].lo < 2) continue;
            int axis, range; uint64_t cnt;
            box_stats(e, &boxes[i], &axis, &range, &cnt);
            double score = (double)range * (double)range * (double)cnt;
            if (range > 0 && score > bscore) { bscore = score; bi = i; baxis = axis; }
        }
        if (bi < 0) break;
        Box *b = &boxes[bi];
        g_axis = baxis;
        qsort(e + b->lo, b->hi - b->lo, sizeof(HEnt), cmp_axis);
        uint64_t total = 0, acc = 0;
        for (int i = b->lo; i < b->hi; i++) total += e[i].n;
        int split = b->lo + 1;
        for (int i = b->lo; i < b->hi - 1; i++) {
            acc += e[i].n;
            if (acc * 2 >= total) { split = i + 1; break; }
        }
        boxes[nb].lo = split; boxes[nb].hi = b->hi;
        b->hi = split;
        nb++;
    }

    for (int i = 0; i < nb; i++) {
        uint64_t sr = 0, sg = 0, sb = 0, n = 0;
        for (int j = boxes[i].lo; j < boxes[i].hi; j++) {
            int r = (e[j].c >> 10) & 31, g = (e[j].c >> 5) & 31, b = e[j].c & 31;
            sr += (uint64_t)((r << 3) | (r >> 2)) * e[j].n;
            sg += (uint64_t)((g << 3) | (g >> 2)) * e[j].n;
            sb += (uint64_t)((b << 3) | (b >> 2)) * e[j].n;
            n += e[j].n;
        }
        if (!n) n = 1;
        p->pal[i * 3 + 0] = (uint8_t)((sr + n / 2) / n);
        p->pal[i * 3 + 1] = (uint8_t)((sg + n / 2) / n);
        p->pal[i * 3 + 2] = (uint8_t)((sb + n / 2) / n);
    }
    p->ncolors = nb;
    free(e);

    // Nearest-colour lookup for every RGB555 value.
    for (int c = 0; c < 32768; c++) {
        int r = (c >> 10) & 31, g = (c >> 5) & 31, b = c & 31;
        r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
        int best = INT_MAX, bi = 0;
        for (int i = 0; i < nb; i++) {
            int dr = r - p->pal[i * 3], dg = g - p->pal[i * 3 + 1], db = b - p->pal[i * 3 + 2];
            int d = dr * dr * 3 + dg * dg * 4 + db * db * 2;
            if (d < best) { best = d; bi = i; if (!d) break; }
        }
        p->lut[c] = (uint8_t)bi;
    }
}

void wg_index_frame(const WgPalette *p, const uint8_t *rgb, int w, int h, int dither, uint8_t *out)
{
    // 4x4 Bayer; ordered dithering is stable from frame to frame (no shimmer).
    static const int bayer[16] = {0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5};
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++, rgb += 3) {
            int o = dither ? (bayer[(y & 3) * 4 + (x & 3)] - 8) / 2 : 0; // -4..3
            int r = clampi(rgb[0] + o, 0, 255), g = clampi(rgb[1] + o, 0, 255), b = clampi(rgb[2] + o, 0, 255);
            *out++ = p->lut[((r >> 3) << 10) | ((g >> 3) << 5) | (b >> 3)];
        }
}

// ---------------------------------------------------------------------------
// GIF writer (GIF89a, global palette, NETSCAPE loop, LZW)
// ---------------------------------------------------------------------------

typedef struct {
    FILE *f;
    uint8_t blk[255];
    int nblk;
    uint32_t acc;
    int nbits;
} BitWriter;

static void bw_byte(BitWriter *b, uint8_t v)
{
    b->blk[b->nblk++] = v;
    if (b->nblk == 255) {
        fputc(255, b->f);
        fwrite(b->blk, 1, 255, b->f);
        b->nblk = 0;
    }
}

static void bw_code(BitWriter *b, int code, int size)
{
    b->acc |= (uint32_t)code << b->nbits;
    b->nbits += size;
    while (b->nbits >= 8) {
        bw_byte(b, (uint8_t)(b->acc & 255));
        b->acc >>= 8;
        b->nbits -= 8;
    }
}

static void bw_flush(BitWriter *b)
{
    if (b->nbits > 0) bw_byte(b, (uint8_t)(b->acc & 255));
    b->acc = 0;
    b->nbits = 0;
    if (b->nblk) {
        fputc(b->nblk, b->f);
        fwrite(b->blk, 1, b->nblk, b->f);
        b->nblk = 0;
    }
}

#define HBITS 13
#define HSIZE (1 << HBITS)
static uint32_t g_hkey[HSIZE];
static uint16_t g_hval[HSIZE];

static void lzw_encode(FILE *f, const uint8_t *px, int npix)
{
    const int clearCode = 256, eoiCode = 257;
    BitWriter bw;
    memset(&bw, 0, sizeof(bw));
    bw.f = f;
    fputc(8, f); // minimum code size

    memset(g_hkey, 0, sizeof(g_hkey));
    int codeSize = 9, maxCode = eoiCode; // maxCode = last assigned code
    bw_code(&bw, clearCode, codeSize);
    if (npix <= 0) { bw_code(&bw, eoiCode, codeSize); bw_flush(&bw); fputc(0, f); return; }

    int cur = px[0];
    for (int i = 1; i < npix; i++) {
        const int c = px[i];
        const uint32_t key = ((uint32_t)cur << 8) | (uint32_t)c;
        uint32_t hidx = (key * 2654435761u) >> (32 - HBITS);
        while (g_hkey[hidx] && g_hkey[hidx] != key + 1) hidx = (hidx + 1) & (HSIZE - 1);
        if (g_hkey[hidx]) { cur = g_hval[hidx]; continue; }

        bw_code(&bw, cur, codeSize);
        maxCode++;
        g_hkey[hidx] = key + 1;
        g_hval[hidx] = (uint16_t)maxCode;
        if (maxCode >= (1 << codeSize) && codeSize < 12) codeSize++;
        if (maxCode == 4095) {
            bw_code(&bw, clearCode, codeSize);
            memset(g_hkey, 0, sizeof(g_hkey));
            codeSize = 9;
            maxCode = eoiCode;
        }
        cur = c;
    }
    bw_code(&bw, cur, codeSize);
    // The decoder adds one more table entry after reading the final code; it
    // may widen its code size before reading EOI, so mirror that here.
    if (maxCode > eoiCode && maxCode + 1 >= (1 << codeSize) && codeSize < 12) codeSize++;
    bw_code(&bw, eoiCode, codeSize);
    bw_flush(&bw);
    fputc(0, f); // block terminator
}

static void put16(FILE *f, int v) { fputc(v & 255, f); fputc((v >> 8) & 255, f); }

int wg_gif_write(FILE *f, int w, int h, const WgPalette *p,
                 uint8_t *const *frames, const int *delay_cs, int nframes)
{
    fwrite("GIF89a", 1, 6, f);
    put16(f, w);
    put16(f, h);
    fputc(0xF7, f); // global colour table, 8-bit colour resolution, 256 entries
    fputc(0, f);    // background index
    fputc(0, f);    // aspect
    fwrite(p->pal, 1, 256 * 3, f);

    // Loop forever.
    fputc(0x21, f); fputc(0xFF, f); fputc(11, f);
    fwrite("NETSCAPE2.0", 1, 11, f);
    fputc(3, f); fputc(1, f); put16(f, 0); fputc(0, f);

    for (int i = 0; i < nframes; i++) {
        fputc(0x21, f); fputc(0xF9, f); fputc(4, f);
        fputc(0x04, f);            // disposal: do not dispose, no transparency
        put16(f, delay_cs[i]);
        fputc(0, f); fputc(0, f);
        fputc(0x2C, f);
        put16(f, 0); put16(f, 0); put16(f, w); put16(f, h);
        fputc(0, f);               // no local palette, not interlaced
        lzw_encode(f, frames[i], w * h);
    }
    fputc(0x3B, f);
    return ferror(f) ? 0 : 1;
}

// ---------------------------------------------------------------------------
// One-shot GIF export
// ---------------------------------------------------------------------------

int wg_make_gif(const WgScene *s, int ax, int ay, int inbetweens, int mode,
                int step_ms, const char *path, WgProgressFn cb, void *user)
{
    int seq[WG_MAX_SEQ], delays[WG_MAX_SEQ];
    int n = clampi(inbetweens, 0, wg_max_inbetweens(step_ms));
    int count = wg_sequence(n, step_ms, seq, delays);
    const int unique = n + 2;

    WgRect r;
    wg_crop(s, ax, ay, &r);
    const size_t npix = (size_t)r.w * r.h;
    int ok = 0;
    uint8_t *rgb[WG_MAX_INBETWEEN + 2] = {0};
    uint8_t *idx[WG_MAX_INBETWEEN + 2] = {0};
    uint8_t *seqFrames[WG_MAX_SEQ];
    uint32_t *hist = calloc(32768, sizeof(uint32_t));
    WgPalette *pal = malloc(sizeof(WgPalette));
    FILE *f = NULL;
    if (!hist || !pal) goto out;

    for (int k = 0; k < unique; k++) {
        if (cb) cb("Rendering frames", k * 60 / unique, user);
        rgb[k] = malloc(npix * 3);
        if (!rgb[k]) goto out;
        int t = k * 256 / (n + 1);
        wg_render(s, ax, ay, t, mode, 1, rgb[k]);
        wg_hist_add(hist, rgb[k], (int)npix);
    }
    if (cb) cb("Building palette", 60, user);
    wg_palette_from_hist(pal, hist);
    for (int k = 0; k < unique; k++) {
        if (cb) cb("Quantizing", 70 + k * 15 / unique, user);
        idx[k] = malloc(npix);
        if (!idx[k]) goto out;
        wg_index_frame(pal, rgb[k], r.w, r.h, 1, idx[k]);
        free(rgb[k]);
        rgb[k] = NULL;
    }
    for (int j = 0; j < count; j++) seqFrames[j] = idx[seq[j]];

    if (cb) cb("Writing GIF", 85, user);
    f = fopen(path, "wb");
    if (!f) goto out;
    static char iobuf[64 * 1024];
    setvbuf(f, iobuf, _IOFBF, sizeof(iobuf));
    ok = wg_gif_write(f, r.w, r.h, pal, seqFrames, delays, count);
    if (fclose(f) != 0) ok = 0;
    f = NULL;
    if (cb) cb(ok ? "Done" : "Write failed", 100, user);

out:
    if (f) fclose(f);
    for (int k = 0; k < unique; k++) { free(rgb[k]); free(idx[k]); }
    free(hist);
    free(pal);
    return ok;
}

// ---------------------------------------------------------------------------
// GIF reader (gallery playback)
// ---------------------------------------------------------------------------

typedef struct { const uint8_t *p, *end; } GifRd;
static int gr8(GifRd *r) { return r->p < r->end ? *r->p++ : -1; }
static int gr16(GifRd *r) { int a = gr8(r), b = gr8(r); return (a < 0 || b < 0) ? -1 : a | (b << 8); }

static void gif_skip_blocks(GifRd *r)
{
    for (;;) {
        int n = gr8(r);
        if (n <= 0) return;
        if (r->end - r->p < n) { r->p = r->end; return; }
        r->p += n;
    }
}

// Decode one LZW image into idx (npix entries). Returns pixels written, <0 on error.
static int gif_lzw(GifRd *r, uint8_t *idx, int npix)
{
    static uint16_t prefix[4096];
    static uint8_t suffix[4096], stack[4097];
    const int minSize = gr8(r);
    if (minSize < 2 || minSize > 8) return -1;
    const int clear = 1 << minSize, eoi = clear + 1;
    int size = minSize + 1, next = eoi + 1, old = -1, first = 0, o = 0;
    uint32_t acc = 0;
    int nbits = 0, blockLeft = 0, done = 0;

    while (!done) {
        while (nbits < size) {
            if (blockLeft == 0) {
                blockLeft = gr8(r);
                if (blockLeft <= 0) { done = 1; break; }
            }
            int b = gr8(r);
            if (b < 0) { done = 1; break; }
            blockLeft--;
            acc |= (uint32_t)b << nbits;
            nbits += 8;
        }
        if (done) break;
        int code = (int)(acc & ((1u << size) - 1));
        acc >>= size;
        nbits -= size;

        if (code == clear) { size = minSize + 1; next = eoi + 1; old = -1; continue; }
        if (code == eoi) break;
        if (old < 0) {
            if (code >= clear) return -2;
            if (o < npix) idx[o++] = (uint8_t)code;
            old = first = code;
            continue;
        }
        int in = code, sp = 0;
        if (code > next) return -3;
        if (code == next) { stack[sp++] = (uint8_t)first; code = old; }
        while (code >= clear && sp < 4096) { stack[sp++] = suffix[code]; code = prefix[code]; }
        stack[sp++] = (uint8_t)code;
        first = code;
        while (sp) { uint8_t v = stack[--sp]; if (o < npix) idx[o++] = v; }
        if (next < 4096) {
            prefix[next] = (uint16_t)old;
            suffix[next] = (uint8_t)first;
            next++;
            if (next >= (1 << size) && size < 12) size++;
        }
        old = in;
    }
    // Skip whatever remains of the data sub-blocks.
    if (blockLeft > 0) r->p = (r->end - r->p < blockLeft) ? r->end : r->p + blockLeft;
    gif_skip_blocks(r);
    return o;
}

void wg_gif_free(WgGif *g)
{
    if (g->frames)
        for (int i = 0; i < g->nframes; i++) free(g->frames[i]);
    free(g->frames);
    free(g->delay_cs);
    memset(g, 0, sizeof(*g));
}

static inline uint16_t pal565(const uint8_t *p) { return (uint16_t)(((p[0] >> 3) << 11) | ((p[1] >> 2) << 5) | (p[2] >> 3)); }

int wg_gif_load(const char *path, WgGif *g, int maxW, int maxH, int maxFrames)
{
    memset(g, 0, sizeof(*g));
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz < 13 || sz > 32L * 1024 * 1024) { fclose(f); return 0; }
    uint8_t *buf = malloc(sz);
    if (!buf) { fclose(f); return 0; }
    size_t got = fread(buf, 1, sz, f);
    fclose(f);
    GifRd r = {buf, buf + got};

    int ok = 0;
    uint8_t gpal[768], lpal[768];
    uint16_t *canvas = NULL, *saved = NULL;
    uint8_t *idx = NULL;
    if (memcmp(buf, "GIF87a", 6) && memcmp(buf, "GIF89a", 6)) goto out;
    r.p += 6;
    const int w = gr16(&r), h = gr16(&r), flags = gr8(&r);
    const int bg = gr8(&r);
    gr8(&r);
    if (w <= 0 || h <= 0 || w > maxW || h > maxH) goto out;
    memset(gpal, 0, sizeof(gpal));
    if (flags & 0x80) {
        int n = 3 << ((flags & 7) + 1);
        if (r.end - r.p < n) goto out;
        memcpy(gpal, r.p, n);
        r.p += n;
    }
    g->w = w;
    g->h = h;
    g->frames = calloc(maxFrames, sizeof(uint16_t *));
    g->delay_cs = calloc(maxFrames, sizeof(int));
    canvas = malloc((size_t)w * h * 2);
    saved = malloc((size_t)w * h * 2);
    idx = malloc((size_t)w * h);
    if (!g->frames || !g->delay_cs || !canvas || !saved || !idx) goto out;
    {
        uint16_t bgc = (flags & 0x80) ? pal565(gpal + (bg & 255) * 3) : 0;
        for (int i = 0; i < w * h; i++) canvas[i] = bgc;
    }

    int delay = 10, disposal = 0, transp = -1;
    for (;;) {
        int b = gr8(&r);
        if (b < 0 || b == 0x3B) break;
        if (b == 0x21) {
            int label = gr8(&r);
            if (label == 0xF9) {
                int n = gr8(&r);
                if (n >= 4) {
                    int pf = gr8(&r);
                    delay = gr16(&r);
                    int ti = gr8(&r);
                    disposal = (pf >> 2) & 7;
                    transp = (pf & 1) ? ti : -1;
                    for (int i = 4; i < n; i++) gr8(&r);
                }
                gif_skip_blocks(&r);
            } else {
                gif_skip_blocks(&r);
            }
        } else if (b == 0x2C) {
            int fx = gr16(&r), fy = gr16(&r), fw = gr16(&r), fh = gr16(&r), lf = gr8(&r);
            if (fw <= 0 || fh <= 0 || fx < 0 || fy < 0) break;
            const uint8_t *pal = gpal;
            if (lf & 0x80) {
                int n = 3 << ((lf & 7) + 1);
                if (r.end - r.p < n) break;
                memset(lpal, 0, sizeof(lpal));
                memcpy(lpal, r.p, n);
                r.p += n;
                pal = lpal;
            }
            if ((size_t)fw * fh > (size_t)w * h) break;
            int npx = gif_lzw(&r, idx, fw * fh);
            if (npx < 0) break;
            if (disposal == 3) memcpy(saved, canvas, (size_t)w * h * 2);
            const int interlaced = lf & 0x40;
            for (int j = 0; j < fh; j++) {
                // interlaced row order: 0,8,16.. 4,12.. 2,6,.. 1,3,..
                int row = j;
                if (interlaced) {
                    int p1 = (fh + 7) / 8, p2 = (fh + 3) / 8, p3 = (fh + 1) / 4;
                    if (j < p1) row = j * 8;
                    else if (j < p1 + p2) row = (j - p1) * 8 + 4;
                    else if (j < p1 + p2 + p3) row = (j - p1 - p2) * 4 + 2;
                    else row = (j - p1 - p2 - p3) * 2 + 1;
                }
                int yy = fy + row;
                if (yy >= h) continue;
                for (int i = 0; i < fw; i++) {
                    int xx = fx + i, k = j * fw + i;
                    if (xx >= w || k >= npx) continue;
                    int c = idx[k];
                    if (c == transp) continue;
                    canvas[yy * w + xx] = pal565(pal + c * 3);
                }
            }
            if (g->nframes < maxFrames) {
                uint16_t *fr = malloc((size_t)w * h * 2);
                if (!fr) break;
                memcpy(fr, canvas, (size_t)w * h * 2);
                g->frames[g->nframes] = fr;
                g->delay_cs[g->nframes] = delay < 2 ? 10 : delay; // browsers treat <2 as 10
                g->nframes++;
            }
            if (disposal == 2) {
                for (int j = 0; j < fh; j++)
                    for (int i = 0; i < fw; i++)
                        if (fy + j < h && fx + i < w) canvas[(fy + j) * w + fx + i] = 0;
            } else if (disposal == 3) {
                memcpy(canvas, saved, (size_t)w * h * 2);
            }
            delay = 10; disposal = 0; transp = -1;
        } else {
            break; // unknown block
        }
    }
    ok = g->nframes > 0;

out:
    free(buf);
    free(canvas);
    free(saved);
    free(idx);
    if (!ok) wg_gif_free(g);
    return ok;
}
