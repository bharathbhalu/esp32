// Photo frame: built-in photos (photos.h) plus photos uploaded from a browser.
// Swipe left/right (or tap the right/left half) to change photo; swipe down from
// the top for the upload address. Uploads arrive already cropped to 368x448 JPEG
// and are stored in LittleFS.
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <JPEGDEC.h>
#include <Arduino_GFX_Library.h>
#include <vector>
#include <Preferences.h>
#include "photos.h"
#include "web_page.h"
#include "secrets.h"
#include "power.h"

static const int I2C_SDA = 15;
static const int I2C_SCL = 14;

// SH8601 AMOLED on QSPI. GFX has no SH8601 class; CO5300 shares its command set.
static const int LCD_CS = 12, LCD_SCK = 11;
static const int LCD_D0 = 4, LCD_D1 = 5, LCD_D2 = 6, LCD_D3 = 7;
static const int LCD_W = 368, LCD_H = 448;

// TCA9554 expander drives the LCD/touch reset lines (pins 0-2)
static const uint8_t TCA9554_ADDR = 0x20;
static const uint8_t FT3168_ADDR = 0x38;

static const int SWIPE_MIN = 40;          // finger travel below this is a tap
static const int TOP_ZONE = 60;           // swipe down starting here opens the info panel
static const size_t MIN_FREE = 120 * 1024; // refuse uploads below this much free space

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
Arduino_CO5300 *panel = new Arduino_CO5300(bus, GFX_NOT_DEFINED, 0, false, LCD_W, LCD_H);
// Compose each frame off-screen so photo changes don't tear
Arduino_Canvas *gfx = new Arduino_Canvas(LCD_W, LCD_H, panel);

static WebServer server(80);
static JPEGDEC jpeg;

static std::vector<String> uploads; // LittleFS paths, oldest first
static int current = 0;
static bool infoOpen = false;
static bool needsRedraw = true;

// Upload in progress
static File uploadFile;
static String uploadPath;
static String uploadError;

static int slideCount() { return PHOTO_COUNT + (int)uploads.size(); }

// Slideshow (settings persisted in NVS)
static const int SHOW_SECONDS[] = {0, 5, 10, 30, 60, 300}; // 0 = off
static const int SHOW_OPTIONS = sizeof(SHOW_SECONDS) / sizeof(SHOW_SECONDS[0]);
static const uint32_t DOUBLE_TAP_MS = 300;
static const uint8_t BRIGHTNESS = 200;
static int showInterval = 2; // index into SHOW_SECONDS: 10 s
static bool showShuffle = false;
static bool showBuiltin = false; // include the built-in photos, not just uploads
static bool showPaused = false;
static uint32_t lastAdvance = 0;
static uint32_t pauseIconUntil = 0;

// Transitions between photos
enum Transition { T_FADE, T_SLIDE, T_WIPE, T_DISSOLVE, T_BLACK, T_NONE, T_RANDOM, T_COUNT };
static const char *const TRANSITION_NAMES[T_COUNT] = {"Fade", "Slide", "Wipe", "Dissolve", "Black", "None", "Random"};
static const char *const SPEED_NAMES[3] = {"Fast", "Normal", "Slow"};
static const uint32_t SPEED_MS[3] = {300, 600, 1200};
static const uint32_t MANUAL_MAX_MS = 350; // swipes/taps never animate longer than this
static int transition = T_FADE;
static int speed = 1;

static void loadShowPrefs() {
    Preferences prefs;
    prefs.begin("show", true);
    showInterval = constrain((int)prefs.getUChar("interval", 2), 0, SHOW_OPTIONS - 1);
    showShuffle = prefs.getBool("shuffle", false);
    showBuiltin = prefs.getBool("builtin", false);
    transition = constrain((int)prefs.getUChar("transition", T_FADE), 0, T_COUNT - 1);
    speed = constrain((int)prefs.getUChar("speed", 1), 0, 2);
    prefs.end();
}

static void saveShowPrefs() {
    Preferences prefs;
    prefs.begin("show", false);
    prefs.putUChar("interval", showInterval);
    prefs.putBool("shuffle", showShuffle);
    prefs.putBool("builtin", showBuiltin);
    prefs.putUChar("transition", transition);
    prefs.putUChar("speed", speed);
    prefs.end();
}

// Slides the slideshow cycles through: uploads (plus built-ins if enabled).
// With no uploads yet it falls back to the built-in photos.
static std::vector<int> showSlides() {
    std::vector<int> list;
    if (showBuiltin || uploads.empty())
        for (int i = 0; i < PHOTO_COUNT; i++) list.push_back(i);
    for (size_t i = 0; i < uploads.size(); i++) list.push_back(PHOTO_COUNT + i);
    return list;
}

static int nextShowSlide() {
    std::vector<int> list = showSlides();
    if (list.size() < 2) return list.empty() ? current : list[0];
    if (showShuffle) {
        int pick;
        do pick = list[esp_random() % list.size()];
        while (pick == current);
        return pick;
    }
    for (int s : list)
        if (s > current) return s;
    return list[0]; // wrap around
}

static String showStatus() {
    if (!SHOW_SECONDS[showInterval]) return "Slideshow off";
    String s = "Slideshow " + String(SHOW_SECONDS[showInterval]) + "s";
    if (showPaused) return s + " - paused";
    s += showBuiltin ? " - all" : " - uploads";
    if (showShuffle) s += ", shuffle";
    return s;
}

static bool slideshowRunning() { return SHOW_SECONDS[showInterval] && !showPaused; }

// Bottom settings panel (swipe up from the bottom edge)
static bool settingsOpen = false;
static const int SETTINGS_Y = 162;
static const int BOTTOM_ZONE = 60; // swipe up starting this close to the bottom opens it

struct Rect {
    int x, y, w, h;
    bool contains(int px, int py) const { return px >= x && px < x + w && py >= y && py < y + h; }
};
static const Rect SET_PAUSE = {250, 176, 104, 32};
static Rect setEvery(int i) { return {12 + i * 58, 220, 54, 32}; }
static Rect setTransition(int i) { return {12 + (i % 4) * 88, 270 + (i / 4) * 38, 82, 32}; }
static Rect setSpeed(int i) { return {12 + i * 116, 360, 110, 32}; }
static const Rect SET_SHUFFLE = {12, 402, 170, 34};
static const Rect SET_BUILTIN = {186, 402, 170, 34};

// ---------- hardware ----------

static void tcaWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(TCA9554_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

// Pulse expander pins 0-2 low then high to reset the display and touch
static void resetPanel() {
    tcaWrite(0x03, 0xF8); // pins 0-2 as outputs
    tcaWrite(0x01, 0x00);
    delay(20);
    tcaWrite(0x01, 0x07);
    delay(300); // touch controller needs time to boot
}

// Returns 1 while a finger is down (with its coordinates), 0 when up, -1 on a read error
static int readTouch(int &x, int &y) {
    Wire.beginTransmission(FT3168_ADDR);
    Wire.write(0x02); // TD_STATUS, followed by P1 XH/XL/YH/YL
    if (Wire.endTransmission(false) != 0) return -1;
    if (Wire.requestFrom(FT3168_ADDR, (uint8_t)5) != 5) return -1;
    uint8_t buf[5];
    for (int i = 0; i < 5; i++) buf[i] = Wire.read();
    if ((buf[0] & 0x0F) == 0) return 0;
    x = ((buf[1] & 0x0F) << 8) | buf[2];
    y = ((buf[3] & 0x0F) << 8) | buf[4];
    return 1;
}

// ---------- storage ----------

static void scanUploads() {
    uploads.clear();
    File root = LittleFS.open("/");
    for (File f = root.openNextFile(); f; f = root.openNextFile()) {
        String name = f.name();
        if (name.endsWith(".jpg")) uploads.push_back("/" + name);
    }
    std::sort(uploads.begin(), uploads.end()); // names start with a timestamp
}

static bool isUpload(const String &path) {
    return std::find(uploads.begin(), uploads.end(), path) != uploads.end();
}

static size_t freeBytes() { return LittleFS.totalBytes() - LittleFS.usedBytes(); }

// ---------- drawing ----------

static int drawJpegBlock(JPEGDRAW *d) {
    gfx->draw16bitRGBBitmap(d->x, d->y, d->pPixels, d->iWidth, d->iHeight);
    return 1;
}

static bool drawUpload(const String &path) {
    File f = LittleFS.open(path, "r");
    if (!f) return false;
    size_t size = f.size();
    uint8_t *buf = (uint8_t *)ps_malloc(size);
    if (!buf) return false;
    f.read(buf, size);
    f.close();
    bool ok = jpeg.openRAM(buf, size, drawJpegBlock);
    if (ok) {
        int x = (LCD_W - jpeg.getWidth()) / 2, y = (LCD_H - jpeg.getHeight()) / 2;
        ok = jpeg.decode(max(x, 0), max(y, 0), 0);
        jpeg.close();
    }
    free(buf);
    return ok;
}

static void drawSlide(int index) {
    gfx->fillScreen(BLACK);
    if (index < PHOTO_COUNT) {
        const Photo &p = PHOTOS[index];
        gfx->draw16bitRGBBitmap((LCD_W - p.w) / 2, (LCD_H - p.h) / 2, (uint16_t *)p.pixels, p.w, p.h);
    } else if (!drawUpload(uploads[index - PHOTO_COUNT])) {
        gfx->setTextColor(WHITE);
        gfx->setTextSize(2);
        gfx->setCursor(80, 210);
        gfx->print("Can't show photo");
    }

    // Position dots (capped so many photos still fit)
    int n = slideCount(), shown = min(n, 15);
    int x0 = LCD_W / 2 - (shown - 1) * 9;
    for (int i = 0; i < shown; i++) {
        bool active = n <= 15 ? i == index : i == index * shown / n;
        gfx->fillCircle(x0 + i * 18, LCD_H - 14, 4, active ? WHITE : 0x4208);
        gfx->drawCircle(x0 + i * 18, LCD_H - 14, 5, BLACK);
    }
}

// ---------- battery time estimate ----------
// The AXP2101 gives a percentage but no current, so learn the rate from how the
// percentage moves: one sample every 30 s over a sliding 30 min window.

static const uint32_t BATT_SAMPLE_MS = 30000;
static const int BATT_SAMPLES = 60;
static const int BATT_MIN_CHANGE = 2;               // % change needed before estimating
static const uint32_t BATT_MIN_SPAN_MS = 5 * 60000; // and at least this long

static struct {
    int pct;
    uint32_t ms;
} battHistory[BATT_SAMPLES];
static int battCount = 0;
static bool battCharging = false;

static void sampleBattery() {
    static uint32_t last = 0;
    if (battCount && millis() - last < BATT_SAMPLE_MS) return;
    last = millis();
    BatteryStatus b = powerRead();
    if (!b.present || b.percent < 0) return;
    if (b.charging != battCharging) battCount = 0; // plugged/unplugged: start over
    battCharging = b.charging;
    if (battCount == BATT_SAMPLES) {
        memmove(battHistory, battHistory + 1, sizeof(battHistory[0]) * (BATT_SAMPLES - 1));
        battCount--;
    }
    battHistory[battCount++] = {b.percent, millis()};
}

// "~3h 50m left", "~45m to full", "Estimating..." or "" (no battery)
static String batteryEstimate(const BatteryStatus &b) {
    if (!b.present || b.percent < 0) return "";
    if (b.charging && b.percent >= 100) return "Full";
    if (battCount < 2 || b.charging != battCharging) return "Estimating...";
    int change = abs(battHistory[battCount - 1].pct - battHistory[0].pct);
    uint32_t span = battHistory[battCount - 1].ms - battHistory[0].ms;
    if (change < BATT_MIN_CHANGE || span < BATT_MIN_SPAN_MS) return "Estimating...";
    float msPerPct = (float)span / change;
    int left = b.charging ? 100 - b.percent : b.percent;
    uint32_t mins = (uint32_t)(left * msPerPct / 60000);
    char buf[24];
    if (mins >= 60) snprintf(buf, sizeof(buf), "~%luh %02lum", mins / 60, mins % 60);
    else snprintf(buf, sizeof(buf), "~%lum", mins);
    return String(buf) + (b.charging ? " to full" : " left");
}

// Battery row: icon + percentage, with a bolt and "Charging" while on charge
static void drawBattery(int x, int y) {
    BatteryStatus b = powerRead();
    gfx->setTextSize(2);
    if (!b.present) {
        gfx->setTextColor(0xA514);
        gfx->setCursor(x, y);
        gfx->print("No battery - USB power");
        return;
    }
    int pct = max(b.percent, 0);
    uint16_t fill = b.charging ? 0x07E0 : pct <= 15 ? 0xF800 : pct <= 35 ? 0xFD20 : 0x07E0;
    // Body, tip and fill level
    gfx->drawRoundRect(x, y - 2, 40, 20, 3, WHITE);
    gfx->fillRect(x + 40, y + 4, 3, 8, WHITE);
    gfx->fillRect(x + 3, y + 1, 34 * pct / 100, 14, fill);
    if (b.charging) {
        // Lightning bolt over the icon
        int cx = x + 20;
        gfx->fillTriangle(cx + 3, y - 1, cx - 6, y + 9, cx + 1, y + 9, BLACK);
        gfx->fillTriangle(cx - 1, y + 7, cx + 6, y + 7, cx - 3, y + 17, BLACK);
        gfx->fillTriangle(cx + 2, y + 1, cx - 4, y + 8, cx + 1, y + 8, YELLOW);
        gfx->fillTriangle(cx, y + 8, cx + 4, y + 8, cx - 2, y + 15, YELLOW);
    }
    gfx->setCursor(x + 54, y);
    gfx->setTextColor(WHITE);
    if (b.percent < 0) gfx->print("--");
    else gfx->printf("%d%%", b.percent);
    if (b.charging) {
        gfx->setTextColor(YELLOW);
        gfx->print("  Charging");
    }
    // Time estimate on its own line
    gfx->setCursor(x + 54, y + 24);
    gfx->setTextColor(0xA514);
    gfx->print(batteryEstimate(b));
}

static void drawInfo() {
    gfx->fillRoundRect(8, 8, LCD_W - 16, 276, 16, 0x18E3);
    gfx->setTextColor(0xA514);
    gfx->setTextSize(2);
    gfx->setCursor(24, 24);
    gfx->print("Upload photos at");

    bool online = WiFi.status() == WL_CONNECTED;
    String url = online ? "http://" + WiFi.localIP().toString() : "Wi-Fi not connected";
    gfx->setTextColor(online ? 0x05FF : 0xFD20);
    gfx->setTextSize(url.length() * 18 <= LCD_W - 48 ? 3 : 2);
    gfx->setCursor(24, 56);
    gfx->print(url);

    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);
    gfx->setCursor(24, 104);
    if (online) gfx->printf("Port 80  -  %s", WiFi.SSID().c_str());
    else gfx->print("Retrying...");
    gfx->setCursor(24, 132);
    gfx->printf("%d photos (%d uploaded)", slideCount(), (int)uploads.size());
    gfx->setCursor(24, 160);
    gfx->setTextColor(showPaused ? 0xFD20 : 0xA514);
    gfx->print(showStatus());
    drawBattery(24, 192);
    gfx->setTextColor(0x8410);
    gfx->setTextSize(1);
    gfx->setCursor(24, 256);
    gfx->print("Open in a browser on the same Wi-Fi. Swipe up to close.");
}

// Brief play/pause badge after a double-tap
static void drawPauseIcon() {
    int cx = LCD_W / 2, cy = LCD_H / 2;
    gfx->fillCircle(cx, cy, 44, 0x2104);
    if (showPaused) {
        gfx->fillRect(cx - 16, cy - 20, 11, 40, WHITE);
        gfx->fillRect(cx + 5, cy - 20, 11, 40, WHITE);
    } else {
        gfx->fillTriangle(cx - 12, cy - 22, cx - 12, cy + 22, cx + 22, cy, WHITE);
    }
}

static void chip(const Rect &r, const char *label, bool selected) {
    gfx->fillRoundRect(r.x, r.y, r.w, r.h, 8, selected ? 0x05FF : 0x4208);
    int size = strlen(label) * 12 <= r.w - 8 ? 2 : 1;
    gfx->setTextSize(size);
    gfx->setTextColor(selected ? BLACK : WHITE);
    gfx->setCursor(r.x + (r.w - (int)strlen(label) * 6 * size) / 2, r.y + (r.h - 8 * size) / 2);
    gfx->print(label);
}

static void drawSettings() {
    gfx->fillRoundRect(0, SETTINGS_Y, LCD_W, LCD_H - SETTINGS_Y + 16, 16, 0x18E3);
    gfx->fillRoundRect(LCD_W / 2 - 20, SETTINGS_Y + 6, 40, 4, 2, 0x8410);
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);
    gfx->setCursor(16, 184);
    gfx->print("Slideshow");
    chip(SET_PAUSE, showPaused ? "Resume" : "Pause", showPaused);

    auto label = [](int y, const char *text) {
        gfx->setTextSize(1);
        gfx->setTextColor(0x8410);
        gfx->setCursor(14, y);
        gfx->print(text);
    };
    label(208, "CHANGE EVERY");
    static const char *const EVERY[SHOW_OPTIONS] = {"Off", "5s", "10s", "30s", "1m", "5m"};
    for (int i = 0; i < SHOW_OPTIONS; i++) chip(setEvery(i), EVERY[i], i == showInterval);
    label(258, "TRANSITION");
    for (int i = 0; i < T_COUNT; i++) chip(setTransition(i), TRANSITION_NAMES[i], i == transition);
    label(348, "SPEED");
    for (int i = 0; i < 3; i++) chip(setSpeed(i), SPEED_NAMES[i], i == speed);
    chip(SET_SHUFFLE, showShuffle ? "Shuffle: On" : "Shuffle: Off", showShuffle);
    chip(SET_BUILTIN, showBuiltin ? "Built-in: On" : "Built-in: Off", showBuiltin);
}

// Composes the current screen into the canvas (no flush)
static void compose() {
    current = constrain(current, 0, slideCount() - 1);
    drawSlide(current);
    // Paused: red dot top-left
    if (SHOW_SECONDS[showInterval] && showPaused) {
        gfx->fillCircle(20, 20, 10, BLACK);
        gfx->fillCircle(20, 20, 8, RED);
    }
    if (infoOpen) drawInfo();
    if (settingsOpen) drawSettings();
    if (millis() < pauseIconUntil) drawPauseIcon();
}

static void render() {
    compose();
    gfx->flush();
}

// ---------- transitions ----------
// The old and new screens are kept in PSRAM and blended into the canvas frame by frame.

static uint16_t *oldFrame = nullptr, *newFrame = nullptr;
static const size_t FRAME_PIXELS = LCD_W * LCD_H;

// RGB565 blend, t = 0 (all a) .. 32 (all b)
static inline uint16_t blend565(uint16_t a, uint16_t b, uint32_t t) {
    uint32_t A = (a | ((uint32_t)a << 16)) & 0x07E0F81F;
    uint32_t B = (b | ((uint32_t)b << 16)) & 0x07E0F81F;
    uint32_t r = ((A * (32 - t) + B * t) >> 5) & 0x07E0F81F;
    return (uint16_t)(r | (r >> 16));
}

static float easeInOut(float t) { return t * t * (3 - 2 * t); }

// Moves to slide `index` with the chosen transition. dir: +1 = next (from the right), -1 = previous
static void goTo(int index, int dir, bool manual) {
    int kind = transition == T_RANDOM ? (int)(esp_random() % T_NONE) : transition;
    uint16_t *fb = gfx->getFramebuffer();
    if (kind == T_NONE || !oldFrame || !newFrame || !fb) {
        current = index;
        render();
        return;
    }
    memcpy(oldFrame, fb, FRAME_PIXELS * 2);
    current = index;
    compose();
    memcpy(newFrame, fb, FRAME_PIXELS * 2);

    uint32_t duration = SPEED_MS[speed];
    if (manual) duration = min(duration, MANUAL_MAX_MS);

    // Dissolve: reveal 16x16 blocks in a random order
    static const int BX = 16, BY = 16, BW = LCD_W / BX, BH = LCD_H / BY;
    static uint16_t order[BX * BY];
    int revealed = 0;
    if (kind == T_DISSOLVE) {
        for (int i = 0; i < BX * BY; i++) order[i] = i;
        for (int i = BX * BY - 1; i > 0; i--) std::swap(order[i], order[esp_random() % (i + 1)]);
        memcpy(fb, oldFrame, FRAME_PIXELS * 2);
    }

    uint32_t start = millis();
    for (;;) {
        float t = min(1.0f, (millis() - start) / (float)duration);
        float e = easeInOut(t);
        switch (kind) {
        case T_FADE: {
            uint32_t a = (uint32_t)(e * 32);
            for (size_t i = 0; i < FRAME_PIXELS; i++) fb[i] = blend565(oldFrame[i], newFrame[i], a);
            break;
        }
        case T_BLACK: {
            // Old fades to black, then new fades in from black
            bool second = e >= 0.5f;
            uint32_t a = (uint32_t)((second ? (1 - e) : e) * 2 * 32);
            const uint16_t *src = second ? newFrame : oldFrame;
            for (size_t i = 0; i < FRAME_PIXELS; i++) fb[i] = blend565(src[i], 0, min(a, (uint32_t)32));
            break;
        }
        case T_SLIDE: {
            int off = (int)((1 - e) * LCD_W); // how much of the old photo is still visible
            for (int y = 0; y < LCD_H; y++) {
                uint16_t *row = fb + y * LCD_W;
                const uint16_t *o = oldFrame + y * LCD_W, *n = newFrame + y * LCD_W;
                if (dir > 0) { // old exits left, new enters from the right
                    memcpy(row, o + (LCD_W - off), off * 2);
                    memcpy(row + off, n, (LCD_W - off) * 2);
                } else { // old exits right, new enters from the left
                    memcpy(row, n + off, (LCD_W - off) * 2);
                    memcpy(row + (LCD_W - off), o, off * 2);
                }
            }
            break;
        }
        case T_WIPE: {
            int edge = (int)(e * LCD_W);
            for (int y = 0; y < LCD_H; y++) {
                uint16_t *row = fb + y * LCD_W;
                const uint16_t *o = oldFrame + y * LCD_W, *n = newFrame + y * LCD_W;
                if (dir > 0) {
                    memcpy(row, n, edge * 2);
                    memcpy(row + edge, o + edge, (LCD_W - edge) * 2);
                } else {
                    memcpy(row, o, (LCD_W - edge) * 2);
                    memcpy(row + (LCD_W - edge), n + (LCD_W - edge), edge * 2);
                }
            }
            break;
        }
        case T_DISSOLVE: {
            int target = (int)(e * BX * BY);
            for (; revealed < target; revealed++) {
                int bx = order[revealed] % BX, by = order[revealed] / BX;
                for (int y = by * BH; y < (by + 1) * BH; y++)
                    memcpy(fb + y * LCD_W + bx * BW, newFrame + y * LCD_W + bx * BW, BW * 2);
            }
            break;
        }
        }
        gfx->flush();
        if (t >= 1) break;
    }
    // Land exactly on the new screen (covers rounding at the edges)
    memcpy(fb, newFrame, FRAME_PIXELS * 2);
    gfx->flush();
}

static void step(int dir, bool manual) {
    goTo((current + dir + slideCount()) % slideCount(), dir, manual);
    lastAdvance = millis();
}

// ---------- web server ----------

static void handleList() {
    String json = "{\"photos\":[";
    for (size_t i = 0; i < uploads.size(); i++) {
        if (i) json += ",";
        json += "\"" + uploads[i] + "\"";
    }
    json += "],\"free\":" + String(freeBytes()) + "}";
    server.send(200, "application/json", json);
}

static void handlePhoto() {
    String path = server.arg("name");
    if (!isUpload(path)) {
        server.send(404, "text/plain", "not found");
        return;
    }
    File f = LittleFS.open(path, "r");
    server.streamFile(f, "image/jpeg");
    f.close();
}

static void handleDelete() {
    String path = server.arg("name");
    if (!isUpload(path)) {
        server.send(404, "text/plain", "not found");
        return;
    }
    LittleFS.remove(path);
    scanUploads();
    needsRedraw = true;
    Serial.printf("deleted %s\n", path.c_str());
    server.send(200, "text/plain", "ok");
}

// Streams the multipart upload straight into a new LittleFS file
static void handleUploadChunk() {
    HTTPUpload &up = server.upload();
    if (up.status == UPLOAD_FILE_START) {
        uploadError = "";
        if (freeBytes() < MIN_FREE) {
            uploadError = "board storage full - delete some photos";
            return;
        }
        // Timestamp + random so names sort by upload order and never collide
        char name[32];
        snprintf(name, sizeof(name), "/%010lu_%04x.jpg", (unsigned long)millis(), (unsigned)(esp_random() & 0xFFFF));
        uploadPath = name;
        uploadFile = LittleFS.open(uploadPath, "w");
        if (!uploadFile) uploadError = "could not create file";
    } else if (up.status == UPLOAD_FILE_WRITE) {
        if (uploadFile && uploadFile.write(up.buf, up.currentSize) != up.currentSize) uploadError = "write failed";
    } else if (up.status == UPLOAD_FILE_END || up.status == UPLOAD_FILE_ABORTED) {
        if (uploadFile) uploadFile.close();
        if (up.status == UPLOAD_FILE_ABORTED) uploadError = "upload aborted";
        if (!uploadError.isEmpty() && !uploadPath.isEmpty()) LittleFS.remove(uploadPath);
    }
}

static void handleUploadDone() {
    if (!uploadError.isEmpty()) {
        server.send(500, "text/plain", uploadError);
        return;
    }
    scanUploads();
    // Jump to the new photo
    for (size_t i = 0; i < uploads.size(); i++)
        if (uploads[i] == uploadPath) current = PHOTO_COUNT + i;
    infoOpen = false;
    needsRedraw = true;
    Serial.printf("uploaded %s\n", uploadPath.c_str());
    server.send(200, "text/plain", "ok");
}

static void handleGetSettings() {
    String json = "{\"interval\":" + String(showInterval) + ",\"options\":[";
    for (int i = 0; i < SHOW_OPTIONS; i++) json += (i ? "," : "") + String(SHOW_SECONDS[i]);
    json += "],\"shuffle\":" + String(showShuffle ? "true" : "false");
    json += ",\"builtin\":" + String(showBuiltin ? "true" : "false");
    json += ",\"paused\":" + String(showPaused ? "true" : "false");
    json += ",\"transition\":" + String(transition) + ",\"transitions\":[";
    for (int i = 0; i < T_COUNT; i++) json += String(i ? "," : "") + "\"" + TRANSITION_NAMES[i] + "\"";
    json += "],\"speed\":" + String(speed) + ",\"speeds\":[";
    for (int i = 0; i < 3; i++) json += String(i ? "," : "") + "\"" + SPEED_NAMES[i] + "\"";
    json += "]}";
    server.send(200, "application/json", json);
}

static void handleSetSettings() {
    if (server.hasArg("interval")) showInterval = constrain(server.arg("interval").toInt(), 0, SHOW_OPTIONS - 1);
    if (server.hasArg("shuffle")) showShuffle = server.arg("shuffle") == "1";
    if (server.hasArg("builtin")) showBuiltin = server.arg("builtin") == "1";
    if (server.hasArg("paused")) showPaused = server.arg("paused") == "1";
    if (server.hasArg("transition")) transition = constrain(server.arg("transition").toInt(), 0, T_COUNT - 1);
    if (server.hasArg("speed")) speed = constrain(server.arg("speed").toInt(), 0, 2);
    saveShowPrefs();
    lastAdvance = millis();
    needsRedraw = true;
    handleGetSettings();
}

// Taps on the bottom settings panel; taps above it close the panel
static void tapSettings(int x, int y) {
    if (y < SETTINGS_Y) {
        settingsOpen = false;
        return;
    }
    if (SET_PAUSE.contains(x, y)) showPaused = !showPaused;
    for (int i = 0; i < SHOW_OPTIONS; i++)
        if (setEvery(i).contains(x, y)) showInterval = i;
    for (int i = 0; i < T_COUNT; i++)
        if (setTransition(i).contains(x, y)) transition = i;
    for (int i = 0; i < 3; i++)
        if (setSpeed(i).contains(x, y)) speed = i;
    if (SET_SHUFFLE.contains(x, y)) showShuffle = !showShuffle;
    if (SET_BUILTIN.contains(x, y)) showBuiltin = !showBuiltin;
    saveShowPrefs();
    lastAdvance = millis();
}

// ---------- Wi-Fi ----------

// Tries each network/password pair in turn (the same SSID may be listed with
// more than one password, which WiFiMulti can't do)
static bool connectWifi() {
    for (const auto &net : WIFI_NETWORKS) {
        Serial.printf("Wi-Fi: trying %s\n", net[0]);
        WiFi.begin(net[0], net[1]);
        uint32_t start = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) delay(100);
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("Wi-Fi %s, http://%s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
            return true;
        }
        Serial.printf("Wi-Fi: %s failed (status %d)\n", net[0], WiFi.status());
        WiFi.disconnect();
    }
    return false;
}

// ---------- main ----------

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);
    resetPanel();
    if (!gfx->begin()) {
        Serial.println("Display init failed");
        return;
    }
    panel->setBrightness(BRIGHTNESS);

    loadShowPrefs();
    oldFrame = (uint16_t *)ps_malloc(FRAME_PIXELS * 2);
    newFrame = (uint16_t *)ps_malloc(FRAME_PIXELS * 2);
    powerBegin();
    if (!LittleFS.begin(true, "/littlefs", 10, "spiffs")) Serial.println("LittleFS mount failed");
    scanUploads();
    render();

    WiFi.mode(WIFI_STA);
    // List what the board can see, to tell range/band problems from password problems
    int found = WiFi.scanNetworks();
    Serial.printf("Wi-Fi scan: %d network(s)\n", found);
    for (int i = 0; i < found; i++)
        Serial.printf("  %-32s ch%-2d %d dBm\n", WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i));
    WiFi.scanDelete();
    connectWifi();

    server.on("/", HTTP_GET, [] { server.send(200, "text/html", WEB_PAGE); });
    server.on("/list", HTTP_GET, handleList);
    server.on("/photo", HTTP_GET, handlePhoto);
    server.on("/delete", HTTP_POST, handleDelete);
    server.on("/upload", HTTP_POST, handleUploadDone, handleUploadChunk);
    server.on("/settings", HTTP_GET, handleGetSettings);
    server.on("/settings", HTTP_POST, handleSetSettings);
    server.begin();
    Serial.printf("%d built-in + %d uploaded photo(s), %u KB free\n", PHOTO_COUNT, (int)uploads.size(),
                  (unsigned)(freeBytes() / 1024));
}

void loop() {
    static bool down = false;
    static int startX, startY, lastX, lastY;
    static uint32_t lastWifiTry = 0;
    static bool tapPending = false;
    static uint32_t tapTime = 0;
    static int tapX = 0;

    server.handleClient();

    // Retry if Wi-Fi is down (auto-reconnect handles brief drops on its own)
    if (WiFi.status() != WL_CONNECTED && millis() - lastWifiTry > 60000) {
        lastWifiTry = millis();
        if (connectWifi() && infoOpen) needsRedraw = true;
    }

    int x, y;
    int touch = readTouch(x, y);
    if (touch == 1) {
        if (!down) {
            startX = x;
            startY = y;
        }
        lastX = x;
        lastY = y;
        down = true;
    } else if (touch == 0 && down) {
        down = false;
        int dx = lastX - startX, dy = lastY - startY;
        bool vertical = abs(dy) > abs(dx);
        bool isTap = abs(dx) < SWIPE_MIN && abs(dy) < SWIPE_MIN;
        if (settingsOpen) {
            if (isTap) tapSettings(startX, startY);
            else if (vertical && dy > 0) settingsOpen = false; // swipe down closes
        } else if (vertical && dy >= SWIPE_MIN && startY < TOP_ZONE && !infoOpen) {
            infoOpen = true;
        } else if (infoOpen) {
            infoOpen = false; // swipe up or any tap closes it
        } else if (vertical && dy <= -SWIPE_MIN && startY > LCD_H - BOTTOM_ZONE) {
            settingsOpen = true;
            tapPending = false;
        } else if (abs(dx) >= SWIPE_MIN && !vertical) {
            step(dx < 0 ? 1 : -1, true); // swipe left = next
        } else if (isTap) {
            // Hold single taps briefly so a second tap can make it a double-tap
            if (tapPending && millis() - tapTime < DOUBLE_TAP_MS) {
                tapPending = false;
                showPaused = !showPaused;
                pauseIconUntil = millis() + 800;
                lastAdvance = millis();
            } else {
                tapPending = true;
                tapTime = millis();
                tapX = startX;
            }
        }
        needsRedraw = true;
    }

    // Single tap (no second tap came): right half = next, left half = previous
    if (tapPending && millis() - tapTime >= DOUBLE_TAP_MS) {
        tapPending = false;
        step(tapX >= LCD_W / 2 ? 1 : -1, true);
    }

    // Auto advance
    int secs = SHOW_SECONDS[showInterval];
    if (slideshowRunning() && !infoOpen && !settingsOpen && !down && !tapPending &&
        millis() - lastAdvance >= (uint32_t)secs * 1000) {
        lastAdvance = millis();
        int next = nextShowSlide();
        if (next != current) goTo(next, 1, false);
    }

    if (pauseIconUntil && millis() >= pauseIconUntil) {
        pauseIconUntil = 0;
        needsRedraw = true;
    }

    sampleBattery();

    // Keep the info panel live (battery, charging, Wi-Fi) while it's open
    static uint32_t lastInfoRefresh = 0;
    if (infoOpen && millis() - lastInfoRefresh > 2000) {
        lastInfoRefresh = millis();
        needsRedraw = true;
    }

    if (needsRedraw) {
        needsRedraw = false;
        render();
    }
    delay(10);
}
