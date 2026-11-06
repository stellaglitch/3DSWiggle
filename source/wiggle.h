// 3DSWiggle - portable wigglegram core (no 3DS dependencies).
//
// Pipeline:
//   1. wg_scene_init()          unpack the two RGB565 camera frames
//   2. wg_compute_disparity()   block-matching depth (per-pixel shift field)
//   3. wg_match_point()         find where a focal point in frame 1 is in frame 2
//   4. wg_render()              render an aligned key frame or interpolated in-between
//   5. wg_make_gif()            render all frames, quantize, write a looping GIF
//
// Coordinates: everything is expressed in frame-1 pixel space.  The alignment
// offset (ax, ay) is "focal point in frame 1 minus focal point in frame 2",
// i.e. frame 2 is shifted by (ax, ay) so the focal points coincide.
#ifndef WIGGLE_H
#define WIGGLE_H

#include <stdint.h>
#include <stdio.h>

// Upper bound on in-betweens; the real limit depends on the step time, since
// every GIF frame must last >= 20 ms (e.g. 120 ms -> 5, 240 ms -> 11).
#define WG_MAX_INBETWEEN 11
#define WG_MAX_SEQ       (2 * (WG_MAX_INBETWEEN + 1))

enum { WG_MODE_CROSSFADE = 0, WG_MODE_MORPH = 1 };

typedef struct {
    int w, h;
    uint8_t *rgb1, *rgb2;   // interleaved RGB888, w*h*3
    uint8_t *y1, *y2;       // luma, w*h
    // Horizontal disparity, 8.8 fixed point, same sign convention for both maps:
    //   dx  on frame-1 pixels:  I1(x, y)           ~ I2(x + dx, y + gy)
    //   dx2 on frame-2 pixels:  I2(x, y + gy)      ~ I1(x - dx2, y)   (row-indexed by y)
    int16_t *dx, *dx2;
    int gy;                 // global vertical offset frame1 -> frame2 (px)
    int nsign;              // nearness = nsign * disparity (bigger = closer to camera)
    int gain[3];            // per-channel mean(frame1)/mean(frame2), 8.8
    int has_disparity;
} WgScene;

typedef struct { int x, y, w, h; } WgRect;

typedef struct {
    uint8_t pal[256 * 3];
    int ncolors;
    uint8_t lut[32768];     // RGB555 -> palette index
} WgPalette;

typedef void (*WgProgressFn)(const char *msg, int percent, void *user);

int  wg_scene_init(WgScene *s, const uint16_t *f1, const uint16_t *f2, int w, int h);
void wg_scene_free(WgScene *s);

// Block-matching disparity between frame 1 and frame 2 (call once per capture).
void wg_compute_disparity(WgScene *s);

// Find the location in frame 2 matching (x1, y1) in frame 1. Returns match cost.
int  wg_match_point(const WgScene *s, int x1, int y1, int *x2, int *y2);

// Region of frame-1 space where both aligned frames have image data.
void wg_crop(const WgScene *s, int ax, int ay, WgRect *r);

// Render the frame at position t (0 = frame 1, 256 = frame 2) into out
// (RGB888, crop.w * crop.h * 3).  bilinear=0 is a fast preview path.
void wg_render(const WgScene *s, int ax, int ay, int t, int mode, int bilinear, uint8_t *out);

// Ping-pong frame order (0,1..n+1,n..1) and per-frame delays in centiseconds
// such that each key-frame-to-key-frame transition lasts step_ms.
int  wg_sequence(int inbetweens, int step_ms, int *seq, int *delay_cs);
int  wg_max_inbetweens(int step_ms);

// Palette + GIF helpers (exposed for testing).
void wg_hist_add(uint32_t *hist, const uint8_t *rgb, int npix);
void wg_palette_from_hist(WgPalette *p, const uint32_t *hist);
void wg_index_frame(const WgPalette *p, const uint8_t *rgb, int w, int h, int dither, uint8_t *out);
int  wg_gif_write(FILE *f, int w, int h, const WgPalette *p,
                  uint8_t *const *frames, const int *delay_cs, int nframes);

// GIF reader (for the gallery). Frames are fully composited RGB565 images, w*h each.
typedef struct {
    int w, h, nframes;
    uint16_t **frames;
    int *delay_cs;
} WgGif;
int  wg_gif_load(const char *path, WgGif *g, int maxW, int maxH, int maxFrames);
void wg_gif_free(WgGif *g);

// Everything at once. Returns 1 on success.
int  wg_make_gif(const WgScene *s, int ax, int ay, int inbetweens, int mode,
                 int step_ms, const char *path, WgProgressFn cb, void *user);

#endif
