// 3DSWiggle - wigglegram camera for Nintendo 3DS (libctru homebrew).
//
// Both outer cameras capture at the same instant (vsync-synchronised), you pick
// a focal point that stays still, and the app writes a looping GIF:
// frame 1 -> (in-betweens) -> frame 2 -> (in-betweens) -> frame 1 ...
// with 120 ms between the two real photos.
#include <3ds.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>

#include "wiggle.h"

#define W 400
#define H 240
#define FRAME_PIXELS (W * H)
#define FRAME_BYTES (FRAME_PIXELS * 2)
#define STEP_MS_DEFAULT 120 // default time between frame 1 and frame 2
#define STEP_MS_MIN 40
#define STEP_MS_MAX 500
#define STEP_MS_INC 10      // GIF delays are in 10 ms units
#define STATUS_ROW 20
#define SAVE_DIR "sdmc:/wigglegrams"

enum { ST_CAMERA, ST_EDIT, ST_GALLERY };
enum { LIVE_CAM1, LIVE_CAM2, LIVE_WIGGLE, LIVE_COUNT };
enum { VIEW_WIGGLE, VIEW_ONION, VIEW_FRAME1, VIEW_FRAME2, VIEW_COUNT };

static const char *liveNames[LIVE_COUNT] = {"Camera 1", "Camera 2", "Wiggle"};
static const char *viewNames[VIEW_COUNT] = {"Wiggle", "Onion skin", "Frame 1", "Frame 2"};

// ---------------------------------------------------------------------------
// Buffers
// ---------------------------------------------------------------------------

static u16 *recv1, *recv2;   // camera DMA targets
static u16 *disp1, *disp2;   // last complete frames
static u16 *cap1, *cap2;     // captured pair
static u16 *canvas;          // top-screen image (row-major 400x240)
static uint8_t *rgbTmp;      // render scratch (RGB888)

// ---------------------------------------------------------------------------
// Camera (both outer cameras, synchronised)
// ---------------------------------------------------------------------------

static Handle camEv[4];      // 0/1 = buffer error cam1/cam2, 2/3 = receive done
static u32 camBufSize;
static bool camRunning, camInterrupted;
static bool fresh1, fresh2;  // a new frame arrived since the flag was cleared

static void camStart(void)
{
    memset(camEv, 0, sizeof(camEv));
    CAMU_SetSize(SELECT_OUT1_OUT2, SIZE_CTR_TOP_LCD, CONTEXT_A);
    CAMU_SetOutputFormat(SELECT_OUT1_OUT2, OUTPUT_RGB_565, CONTEXT_A);
    CAMU_SetFrameRate(SELECT_OUT1_OUT2, FRAME_RATE_30);
    CAMU_SetNoiseFilter(SELECT_OUT1_OUT2, true);
    CAMU_SetAutoExposure(SELECT_OUT1_OUT2, true);
    CAMU_SetAutoWhiteBalance(SELECT_OUT1_OUT2, true);
    CAMU_SetTrimming(PORT_CAM1, false);
    CAMU_SetTrimming(PORT_CAM2, false);
    CAMU_GetMaxBytes(&camBufSize, W, H);
    CAMU_SetTransferBytes(PORT_BOTH, camBufSize, W, H);
    CAMU_Activate(SELECT_OUT1_OUT2);
    CAMU_GetBufferErrorInterruptEvent(&camEv[0], PORT_CAM1);
    CAMU_GetBufferErrorInterruptEvent(&camEv[1], PORT_CAM2);
    CAMU_ClearBuffer(PORT_BOTH);
    CAMU_SynchronizeVsyncTiming(SELECT_OUT1, SELECT_OUT2);
    CAMU_StartCapture(PORT_BOTH);
    camRunning = true;
    camInterrupted = false;
    fresh1 = fresh2 = false;
}

static void camStop(void)
{
    if (!camRunning) return;
    CAMU_StopCapture(PORT_BOTH);
    for (int i = 0; i < 4; i++)
        if (camEv[i]) { svcCloseHandle(camEv[i]); camEv[i] = 0; }
    CAMU_Activate(SELECT_NONE);
    camRunning = false;
}

// Service camera events without blocking.
static void camPump(void)
{
    if (!camRunning) return;
    for (int guard = 0; guard < 8; guard++) {
        if (camInterrupted) {
            CAMU_StartCapture(PORT_BOTH);
            camInterrupted = false;
        }
        if (!camEv[2]) CAMU_SetReceiving(&camEv[2], recv1, PORT_CAM1, FRAME_BYTES, (s16)camBufSize);
        if (!camEv[3]) CAMU_SetReceiving(&camEv[3], recv2, PORT_CAM2, FRAME_BYTES, (s16)camBufSize);

        s32 idx = 0;
        Result res = svcWaitSynchronizationN(&idx, camEv, 4, false, 0);
        if (res != 0) break; // timeout (nothing ready) or error

        switch (idx) {
        case 0: // buffer error on cam1: restart capture
            svcCloseHandle(camEv[2]); camEv[2] = 0; camInterrupted = true; break;
        case 1:
            svcCloseHandle(camEv[3]); camEv[3] = 0; camInterrupted = true; break;
        case 2:
            svcCloseHandle(camEv[2]); camEv[2] = 0;
            memcpy(disp1, recv1, FRAME_BYTES); fresh1 = true; break;
        case 3:
            svcCloseHandle(camEv[3]); camEv[3] = 0;
            memcpy(disp2, recv2, FRAME_BYTES); fresh2 = true; break;
        }
    }
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

static void blitTop(const u16 *img)
{
    // The top framebuffer is rotated: 240 px per column, origin bottom-left.
    u16 *fb = (u16 *)gfxGetFramebuffer(GFX_TOP, GFX_LEFT, NULL, NULL);
    for (int x = 0; x < W; x++) {
        u16 *col = fb + x * H;
        const u16 *src = img + x;
        for (int y = 0; y < H; y++) col[H - 1 - y] = src[y * W];
    }
}

static inline u16 rgb565(int r, int g, int b) { return (u16)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)); }

static void putPx(u16 *img, int x, int y, u16 c)
{
    if (x >= 0 && y >= 0 && x < W && y < H) img[y * W + x] = c;
}

static void drawCross(u16 *img, int cx, int cy)
{
    const u16 dark = rgb565(0, 0, 0), lite = rgb565(255, 230, 40);
    for (int pass = 0; pass < 2; pass++) {
        u16 c = pass ? lite : dark;
        int o = pass ? 0 : 1;
        for (int d = 3; d <= 9; d++) {
            putPx(img, cx + d, cy + o, c); putPx(img, cx - d, cy + o, c);
            putPx(img, cx + o, cy + d, c); putPx(img, cx + o, cy - d, c);
        }
    }
    putPx(img, cx, cy, lite);
}

// Render a frame of the scene into the canvas (centred crop, black border).
static void composeScene(const WgScene *sc, int ax, int ay, int t, int mode,
                         int fx, int fy, bool cross, WgRect *outCrop, int *outOx, int *outOy)
{
    WgRect r;
    wg_crop(sc, ax, ay, &r);
    wg_render(sc, ax, ay, t, mode, 0, rgbTmp);
    memset(canvas, 0, FRAME_BYTES);
    int ox = (W - r.w) / 2, oy = (H - r.h) / 2;
    for (int y = 0; y < r.h; y++) {
        const uint8_t *p = rgbTmp + (size_t)y * r.w * 3;
        u16 *dst = canvas + (y + oy) * W + ox;
        for (int x = 0; x < r.w; x++, p += 3) dst[x] = rgb565(p[0], p[1], p[2]);
    }
    if (cross) drawCross(canvas, fx - r.x + ox, fy - r.y + oy);
    if (outCrop) *outCrop = r;
    if (outOx) *outOx = ox;
    if (outOy) *outOy = oy;
}

static void present(void)
{
    gfxFlushBuffers();
    gfxSwapBuffers();
    gspWaitForVBlank();
}

// ---------------------------------------------------------------------------
// Bottom-screen text
// ---------------------------------------------------------------------------

static void line(int row, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void line(int row, const char *fmt, ...)
{
    char buf[48];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    printf("\x1b[%d;0H%-39.39s", row + 1, buf); // pad to overwrite old text
}

static void drawCameraHelp(int liveMode)
{
    line(0, " 3DSWIGGLE  - wigglegram camera");
    line(1, " -------------------------------------");
    line(3, " Hold the 3DS steady, frame your shot.");
    line(5, " A        take photo (both cameras)");
    line(6, " SELECT   preview: %s", liveNames[liveMode]);
    line(7, " Y        gallery (view / delete GIFs)");
    line(8, " START    quit");
    line(10, " Photos are saved as looping GIFs to");
    line(11, " %s/", SAVE_DIR);
}

// In-between frames are always crossfades between the two aligned photos.
#define BLEND_MODE WG_MODE_CROSSFADE

typedef struct {
    int ax, ay, fx, fy, n, view;
    int outW, outH;
    int ms;        // time between frame 1 and frame 2
} EditState;

// In-between count actually used (clamped to what the timing allows).
static int effN(const EditState *e)
{
    int maxN = wg_max_inbetweens(e->ms);
    return e->n < maxN ? e->n : maxN;
}

static char lastSaved[64];

static void drawEditHelp(const EditState *e)
{
    const int n = effN(e);
    line(0, " 3DSWIGGLE  - adjust & save");
    line(1, " -------------------------------------");
    line(2, " Touch    pick focal point (auto-align)");
    line(3, " D-Pad    fine-tune alignment");
    line(4, " Circle   timing: %d ms  (<- / ->)", e->ms);
    if (n > 0)
        line(5, " L / R    crossfade frames: %d (max %d)", n, wg_max_inbetweens(e->ms));
    else
        line(5, " L / R    crossfade frames: 0 (classic)");
    line(6, " ");
    line(7, " ");
    line(8, " SELECT   view: %s", viewNames[e->view]);
    line(9, " A        save GIF");
    line(10, " B        retake      START  quit");
    line(12, " Focal point  %3d,%3d", e->fx, e->fy);
    line(13, " Offset       %+4d,%+4d", e->ax, e->ay);
    line(14, " Output       %dx%d, %d frames", e->outW, e->outH, 2 * n + 2);
    if (n > 0) line(15, " Frame delay  ~%d ms each", e->ms / (n + 1));
    else       line(15, " Frame delay  %d ms each", e->ms);
    line(17, " %s", lastSaved[0] ? "Last saved:" : "");
    line(18, " %s", lastSaved);
    line(STATUS_ROW, " ");
}

static void progressCb(const char *msg, int pct, void *user)
{
    (void)user;
    line(STATUS_ROW, " %s... %d%%", msg, pct);
    gfxFlushBuffers();
    gspWaitForVBlank();
}

// ---------------------------------------------------------------------------
// Gallery
// ---------------------------------------------------------------------------

#define GAL_MAX 999
#define GAL_NAME 40
#define GAL_MAX_FRAMES 32

static char galNames[GAL_MAX][GAL_NAME];
static int galCount;

static int cmpNameDesc(const void *a, const void *b) { return strcmp((const char *)b, (const char *)a); }

// List the GIFs in the save folder, newest (highest number) first.
static void galScan(void)
{
    galCount = 0;
    DIR *d = opendir(SAVE_DIR);
    if (!d) return;
    struct dirent *e;
    while ((e = readdir(d)) && galCount < GAL_MAX) {
        size_t L = strlen(e->d_name);
        if (L > 4 && L < GAL_NAME && strcasecmp(e->d_name + L - 4, ".gif") == 0)
            memcpy(galNames[galCount++], e->d_name, L + 1);
    }
    closedir(d);
    qsort(galNames, galCount, GAL_NAME, cmpNameDesc);
}

static void drawGalleryHelp(int idx, const WgGif *g, bool loaded, bool confirmDel, const char *status)
{
    line(0, " 3DSWIGGLE  - gallery");
    line(1, " -------------------------------------");
    line(2, " < / >    browse  (D-Pad or L / R)");
    line(3, " X        delete this GIF");
    line(4, " B        back to camera");
    if (galCount == 0) {
        line(6, " No GIFs yet in %s/", SAVE_DIR + 5);
        for (int r = 7; r <= 12; r++) line(r, " ");
    } else {
        line(6, " %s", galNames[idx]);
        line(7, " %d of %d", idx + 1, galCount);
        if (loaded) {
            int total = 0;
            for (int i = 0; i < g->nframes; i++) total += g->delay_cs[i];
            line(8, " %dx%d, %d frames, %d ms loop", g->w, g->h, g->nframes, total * 10);
        } else {
            line(8, " (can't show this file)");
        }
        line(9, " ");
        if (confirmDel) {
            line(10, " Delete %s?", galNames[idx]);
            line(11, " This can't be undone.");
            line(12, " A = delete      B = cancel");
        } else {
            for (int r = 10; r <= 12; r++) line(r, " ");
        }
    }
    line(STATUS_ROW, " %s", status ? status : "");
}

// ---------------------------------------------------------------------------
// Saving
// ---------------------------------------------------------------------------

static bool saveGif(const WgScene *sc, const EditState *e)
{
    mkdir(SAVE_DIR, 0777);
    char path[64];
    struct stat st;
    int num = 1;
    for (; num < 10000; num++) {
        snprintf(path, sizeof(path), "%s/WIG_%04d.gif", SAVE_DIR, num);
        if (stat(path, &st) != 0) break;
    }
    if (num >= 10000) return false;
    bool ok = wg_make_gif(sc, e->ax, e->ay, effN(e), BLEND_MODE, e->ms, path, progressCb, NULL);
    if (ok) snprintf(lastSaved, sizeof(lastSaved), "%s", path + 5); // strip "sdmc:"
    return ok;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

static void setFocal(const WgScene *sc, EditState *e, int fx, int fy)
{
    int x2, y2;
    e->fx = fx < 0 ? 0 : (fx >= W ? W - 1 : fx);
    e->fy = fy < 0 ? 0 : (fy >= H ? H - 1 : fy);
    wg_match_point(sc, e->fx, e->fy, &x2, &y2);
    e->ax = e->fx - x2;
    e->ay = e->fy - y2;
}

int main(void)
{
    gfxInit(GSP_RGB565_OES, GSP_RGB565_OES, false);
    gfxSet3D(false);
    consoleInit(GFX_BOTTOM, NULL);
    osSetSpeedupEnable(true); // New 3DS: 804 MHz + L2 cache
    hidSetRepeatParameters(18, 4);

    recv1 = malloc(FRAME_BYTES); recv2 = malloc(FRAME_BYTES);
    disp1 = calloc(1, FRAME_BYTES); disp2 = calloc(1, FRAME_BYTES);
    cap1 = malloc(FRAME_BYTES); cap2 = malloc(FRAME_BYTES);
    canvas = malloc(FRAME_BYTES);
    rgbTmp = malloc(FRAME_PIXELS * 3);
    if (!recv1 || !recv2 || !disp1 || !disp2 || !cap1 || !cap2 || !canvas || !rgbTmp) {
        printf("Out of memory.\nPress START to exit.\n");
        while (aptMainLoop()) { hidScanInput(); if (hidKeysDown() & KEY_START) break; gspWaitForVBlank(); }
        gfxExit();
        return 0;
    }

    if (R_FAILED(camInit())) {
        printf("Could not start the cameras.\nPress START to exit.\n");
        while (aptMainLoop()) { hidScanInput(); if (hidKeysDown() & KEY_START) break; gspWaitForVBlank(); }
        gfxExit();
        return 0;
    }

    int state = ST_CAMERA;
    int liveMode = LIVE_WIGGLE;
    bool captureReq = false;
    WgScene scene;
    memset(&scene, 0, sizeof(scene));
    EditState ed = {0, 0, W / 2, H / 2, 3, VIEW_WIGGLE, W, H, STEP_MS_DEFAULT};

    int seq[WG_MAX_SEQ], delays[WG_MAX_SEQ], seqCount = 0, seqPos = 0;
    static u16 *frameCache[WG_MAX_INBETWEEN + 2];
    int cacheKey[4] = {0};
    bool cacheValid = false;
    for (int k = 0; k < WG_MAX_INBETWEEN + 2; k++) {
        frameCache[k] = malloc(FRAME_BYTES);
        if (!frameCache[k]) { printf("Out of memory.\n"); gfxExit(); return 0; }
    }
    u64 lastSwitch = osGetTime();
    bool dirty = true, helpDirty = true;

    // Gallery state
    WgGif gif;
    memset(&gif, 0, sizeof(gif));
    int galIdx = 0, galLoaded = -1, galFrame = 0;
    bool galOk = false, galConfirm = false;
    const char *galStatus = NULL;
    u64 galLast = 0;
    int touchX = -1, touchY = -1;

    camStart();
    consoleClear();

    while (aptMainLoop()) {
        hidScanInput();
        u32 down = hidKeysDown(), held = hidKeysHeld(), rep = hidKeysDownRepeat(), up = hidKeysUp();
        if (down & KEY_START) break;
        u64 now = osGetTime();

        if (state == ST_CAMERA) {
            camPump();
            if (helpDirty) { drawCameraHelp(liveMode); helpDirty = false; }
            if (down & KEY_SELECT) { liveMode = (liveMode + 1) % LIVE_COUNT; helpDirty = true; }
            if ((down & KEY_Y) && !captureReq) {
                camStop();
                galScan();
                galIdx = 0;
                galLoaded = -1;
                galConfirm = false;
                galStatus = NULL;
                state = ST_GALLERY;
                helpDirty = true;
                consoleClear();
                continue;
            }
            if ((down & KEY_A) && !captureReq) {
                captureReq = true;
                fresh1 = fresh2 = false; // wait for a pair that started after the press
            }

            if (captureReq && fresh1 && fresh2) {
                memcpy(cap1, disp1, FRAME_BYTES);
                memcpy(cap2, disp2, FRAME_BYTES);
                CAMU_PlayShutterSound(SHUTTER_SOUND_TYPE_NORMAL);
                camStop();
                captureReq = false;

                consoleClear();
                line(0, " 3DSWIGGLE");
                line(2, " Aligning...");
                gfxFlushBuffers(); gspWaitForVBlank();

                if (!wg_scene_init(&scene, cap1, cap2, W, H)) {
                    line(2, " Out of memory - retaking.");
                    camStart();
                    continue;
                }
                ed.view = VIEW_WIGGLE;
                setFocal(&scene, &ed, W / 2, H / 2);
                seqCount = wg_sequence(effN(&ed), ed.ms, seq, delays);
                seqPos = 0;
                lastSwitch = now;
                state = ST_EDIT;
                cacheValid = false; // new photos
                dirty = helpDirty = true;
                consoleClear();
                continue;
            }

            // Live preview
            const u16 *src = disp1;
            if (liveMode == LIVE_CAM2) src = disp2;
            else if (liveMode == LIVE_WIGGLE) src = ((now / (u64)ed.ms) & 1) ? disp2 : disp1;
            memcpy(canvas, src, FRAME_BYTES);
            if (captureReq) for (int x = 0; x < W; x++) { canvas[x] = 0xFFFF; canvas[(H - 1) * W + x] = 0xFFFF; }
            blitTop(canvas);
            present();
            continue;
        }

        // ------------------------------ ST_GALLERY -----------------------------
        if (state == ST_GALLERY) {
            if (galConfirm) {
                if (down & KEY_A) {
                    char path[96];
                    snprintf(path, sizeof(path), "%s/%s", SAVE_DIR, galNames[galIdx]);
                    wg_gif_free(&gif);
                    galLoaded = -1;
                    galStatus = remove(path) == 0 ? "Deleted." : "Delete FAILED.";
                    galScan();
                    if (galIdx >= galCount) galIdx = galCount - 1;
                    if (galIdx < 0) galIdx = 0;
                    galConfirm = false;
                    helpDirty = true;
                } else if (down & (KEY_B | KEY_X)) {
                    galConfirm = false;
                    galStatus = NULL;
                    helpDirty = true;
                }
            } else {
                if (down & KEY_B) {
                    wg_gif_free(&gif);
                    galLoaded = -1;
                    state = ST_CAMERA;
                    helpDirty = true;
                    consoleClear();
                    camStart();
                    continue;
                }
                if (galCount > 0) {
                    if (rep & (KEY_RIGHT | KEY_R)) { galIdx = (galIdx + 1) % galCount; galStatus = NULL; }
                    if (rep & (KEY_LEFT | KEY_L))  { galIdx = (galIdx + galCount - 1) % galCount; galStatus = NULL; }
                    if (down & KEY_X) { galConfirm = true; galStatus = NULL; helpDirty = true; }
                }
            }

            if (galCount > 0 && galLoaded != galIdx) {
                line(STATUS_ROW, " Loading...");
                gfxFlushBuffers();
                wg_gif_free(&gif);
                char path[96];
                snprintf(path, sizeof(path), "%s/%s", SAVE_DIR, galNames[galIdx]);
                galOk = wg_gif_load(path, &gif, W, H, GAL_MAX_FRAMES);
                galLoaded = galIdx;
                galFrame = 0;
                galLast = now;
                helpDirty = true;
            }
            if (helpDirty) {
                drawGalleryHelp(galIdx, &gif, galOk, galConfirm, galStatus);
                helpDirty = false;
            }

            memset(canvas, 0, FRAME_BYTES);
            if (galCount > 0 && galOk && gif.nframes > 0) {
                if (now - galLast >= (u64)gif.delay_cs[galFrame] * 10) {
                    galLast += (u64)gif.delay_cs[galFrame] * 10;
                    if (now - galLast > 500) galLast = now;
                    galFrame = (galFrame + 1) % gif.nframes;
                }
                const u16 *src = gif.frames[galFrame];
                const int ox = (W - gif.w) / 2, oy = (H - gif.h) / 2;
                for (int y = 0; y < gif.h; y++)
                    memcpy(canvas + (y + oy) * W + ox, src + y * gif.w, gif.w * 2);
            }
            blitTop(canvas);
            present();
            continue;
        }

        // ------------------------------- ST_EDIT -------------------------------
        if (down & KEY_B) {
            wg_scene_free(&scene);
            state = ST_CAMERA;
            helpDirty = true;
            consoleClear();
            camStart();
            continue;
        }
        bool seqChanged = false;
        // Circle Pad left/right: timing between the two photos.
        if ((rep & KEY_CPAD_LEFT) && ed.ms > STEP_MS_MIN)  { ed.ms -= STEP_MS_INC; seqChanged = true; }
        if ((rep & KEY_CPAD_RIGHT) && ed.ms < STEP_MS_MAX) { ed.ms += STEP_MS_INC; seqChanged = true; }
        const int maxN = wg_max_inbetweens(ed.ms);
        if (ed.n > maxN) ed.n = maxN;
        // L / R: number of crossfade frames (0 = classic two-frame wiggle).
        if ((down & KEY_L) && ed.n > 0) { ed.n--; seqChanged = true; }
        if ((down & KEY_R) && ed.n < maxN) { ed.n++; seqChanged = true; }
        if (down & KEY_SELECT) { ed.view = (ed.view + 1) % VIEW_COUNT; dirty = helpDirty = true; }
        // D-Pad only (KEY_LEFT etc. would also include the Circle Pad).
        if (rep & KEY_DLEFT)  { ed.ax--; dirty = helpDirty = true; }
        if (rep & KEY_DRIGHT) { ed.ax++; dirty = helpDirty = true; }
        if (rep & KEY_DUP)    { ed.ay--; dirty = helpDirty = true; }
        if (rep & KEY_DDOWN)  { ed.ay++; dirty = helpDirty = true; }
        if (ed.ax < -150) ed.ax = -150;
        if (ed.ax > 150) ed.ax = 150;
        if (ed.ay < -60) ed.ay = -60;
        if (ed.ay > 60) ed.ay = 60;
        if (seqChanged) {
            seqCount = wg_sequence(effN(&ed), ed.ms, seq, delays);
            seqPos = 0;
            lastSwitch = now;
            dirty = helpDirty = true;
        }

        // Touch: the bottom screen maps onto the top image (320 -> 400 wide).
        WgRect crop;
        wg_crop(&scene, ed.ax, ed.ay, &crop);
        const int ox = (W - crop.w) / 2, oy = (H - crop.h) / 2;
        if (held & KEY_TOUCH) {
            touchPosition tp;
            hidTouchRead(&tp);
            touchX = tp.px * W / 320;
            touchY = tp.py;
            dirty = true;
        }
        if ((up & KEY_TOUCH) && touchX >= 0) {
            line(STATUS_ROW, " Aligning...");
            gfxFlushBuffers();
            setFocal(&scene, &ed, touchX - ox + crop.x, touchY - oy + crop.y);
            touchX = touchY = -1;
            dirty = helpDirty = true;
        }

        if (ed.view == VIEW_WIGGLE && seqCount > 0) {
            if (now - lastSwitch >= (u64)delays[seqPos] * 10) {
                lastSwitch += (u64)delays[seqPos] * 10;
                if (now - lastSwitch > 500) lastSwitch = now;
                seqPos = (seqPos + 1) % seqCount;
                dirty = true;
            }
        }

        if (dirty) {
            WgRect r;
            int cox, coy;
            if (ed.view == VIEW_WIGGLE) {
                // Render every unique frame once per settings change, then just play them.
                const int n = effN(&ed);
                const int key[4] = {ed.ax, ed.ay, n, 0};
                if (!cacheValid || memcmp(key, cacheKey, sizeof(key))) {
                    line(STATUS_ROW, " Rendering preview...");
                    gfxFlushBuffers();
                    for (int k = 0; k < n + 2; k++) {
                        composeScene(&scene, ed.ax, ed.ay, k * 256 / (n + 1), BLEND_MODE, 0, 0, false, &r, &cox, &coy);
                        memcpy(frameCache[k], canvas, FRAME_BYTES);
                    }
                    memcpy(cacheKey, key, sizeof(key));
                    cacheValid = true;
                    helpDirty = true;
                }
                memcpy(canvas, frameCache[seq[seqPos]], FRAME_BYTES);
                wg_crop(&scene, ed.ax, ed.ay, &r);
            } else {
                int t = 0;
                switch (ed.view) {
                case VIEW_ONION:  t = 128; break;
                case VIEW_FRAME1: t = 0; break;
                default:          t = 256; break;
                }
                composeScene(&scene, ed.ax, ed.ay, t, BLEND_MODE, ed.fx, ed.fy, true, &r, &cox, &coy);
            }
            if (touchX >= 0) drawCross(canvas, touchX, touchY);
            ed.outW = r.w;
            ed.outH = r.h;
            dirty = false;
        }
        if (helpDirty) { drawEditHelp(&ed); helpDirty = false; }

        if (down & KEY_A) {
            bool ok = saveGif(&scene, &ed);
            drawEditHelp(&ed);
            line(STATUS_ROW, ok ? " Saved!" : " Save FAILED (SD card full?)");
            lastSwitch = osGetTime();
        }
        blitTop(canvas); // top screen is double-buffered: always fill the back buffer
        present();
    }

    if (state == ST_EDIT) wg_scene_free(&scene);
    wg_gif_free(&gif);
    camStop();
    camExit();
    free(recv1); free(recv2); free(disp1); free(disp2);
    free(cap1); free(cap2); free(canvas); free(rgbTmp);
    for (int k = 0; k < WG_MAX_INBETWEEN + 2; k++) free(frameCache[k]);
    gfxExit();
    return 0;
}
