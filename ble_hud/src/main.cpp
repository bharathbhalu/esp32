// BLE navigation HUD for Android: the Chronos app forwards Google Maps
// turn-by-turn navigation over Bluetooth LE (via the MIT-licensed ChronosESP32
// library). The board shows a big arrow, the distance to the turn, the
// instruction text and ETA, and beeps before turns.
//
// Pairing: Chronos app > Watches > Pair New Devices > Search > "ESP32 HUD".
// Don't pair the board in Android's own Bluetooth settings.
#include <Arduino.h>
#include <Wire.h>
#include <ChronosESP32.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include "secrets.h"
#include "saved_songs.h"
#include <Arduino_GFX_Library.h>
#include "sound.h"

static const int I2C_SDA = 15;
static const int I2C_SCL = 14;

// SH8601 AMOLED on QSPI. GFX has no SH8601 class; CO5300 shares its command set.
static const int LCD_CS = 12, LCD_SCK = 11;
static const int LCD_D0 = 4, LCD_D1 = 5, LCD_D2 = 6, LCD_D3 = 7;
static const int LCD_W = 368, LCD_H = 448;

static const uint8_t TCA9554_ADDR = 0x20; // drives the LCD/touch reset lines (pins 0-2)
static const uint8_t FT3168_ADDR = 0x38;
static const int BOOT_BUTTON = 0;

static const uint8_t BRIGHT_DAY = 255, BRIGHT_NIGHT = 60;
static const float NEAR_TURN_M = 150;     // double beep when the next turn gets this close
static const bool LEFT_HAND_TRAFFIC = true; // India: roundabouts go clockwise

// Arrow types (drawn by us; matched from the instruction text)
enum Direction {
    D_NONE, D_STRAIGHT, D_LEFT, D_RIGHT, D_EASY_LEFT, D_EASY_RIGHT, D_SHARP_LEFT, D_SHARP_RIGHT,
    D_KEEP_LEFT, D_KEEP_RIGHT, D_EXIT_LEFT, D_EXIT_RIGHT, D_UTURN_LEFT, D_UTURN_RIGHT,
    D_ROUNDABOUT, D_END,
};

static const uint16_t ARROW = WHITE;
static const uint16_t DIM = 0x4208;
static const uint16_t ACCENT = 0x05FF;
static const uint16_t AMBER = 0xFD20;
static const uint16_t SOFT = 0xA514;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
Arduino_CO5300 *panel = new Arduino_CO5300(bus, GFX_NOT_DEFINED, 0, false, LCD_W, LCD_H);
Arduino_Canvas *gfx = new Arduino_Canvas(LCD_W, LCD_H, panel);

ChronosESP32 chronos("ESP32 HUD", CF_ESP32_466x466);

// What the screen shows (copied from Chronos in the main loop)
struct NavView {
    bool active = false;
    String step;       // distance to the next turn, e.g. "350 m"
    String directions; // e.g. "Turn right onto MG Road"
    String eta, duration, distance, speed;
    bool hasIcon = false;
    uint8_t icon[CS_ICON_DATA_SIZE];
};
static NavView view;

static volatile bool navChanged = false;
static volatile bool linkChanged = false;
static volatile bool musicChanged = false;
static bool musicPage = false; // swipe left: music, swipe right: back
static bool savedPage = false; // swipe left again from music: saved songs
static int savedScroll = 0;    // first row shown on the saved page
static String toast;           // short message at the bottom
static uint32_t toastUntil = 0;

static void showToast(const String &msg) {
    toast = msg;
    toastUntil = millis() + 2500;
}
static bool phoneMuted = false;
static bool songSinceConnect = false; // song info received since the last connect // tracked here: the phone doesn't report its mute state
static bool night = false;
static bool soundOn = true;
static bool screenOn = true;

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

// ---------- Chronos callbacks (BLE task: only set flags) ----------

static void onConnection(bool connected) {
    Serial.printf("phone %s\n", connected ? "connected" : "disconnected");
    linkChanged = true;
}

static volatile uint32_t lastPhoneData = 0;

// The library's connected flag goes false when a stale connection times out even
// if the phone has already reconnected on a new one, so count the open links
// instead (and treat recent data as connected)
static bool phoneConnected() {
    NimBLEServer *server = NimBLEDevice::getServer();
    if (server && server->getConnectedCount() > 0) return true;
    return lastPhoneData && millis() - lastPhoneData < 15000;
}

static void onConfig(Config config, uint32_t a, uint32_t b) {
    lastPhoneData = millis();
    if (config == CF_NAV_DATA || (config == CF_NAV_ICON && a == 2)) navChanged = true;
    if (config == CF_MUSIC) musicChanged = true;
}

// ---------- instruction text -> arrow ----------

static bool has(const String &s, const char *word) { return s.indexOf(word) >= 0; }

// Roundabout exit number from "... take the 2nd exit ..." (default 2: straight on)
static int exitNumber(const String &s) {
    static const char *const ORD[] = {"1st", "2nd", "3rd", "4th", "5th", "6th"};
    static const char *const WORD[] = {"first", "second", "third", "fourth", "fifth", "sixth"};
    for (int i = 0; i < 6; i++)
        if (has(s, ORD[i]) || has(s, WORD[i])) return i + 1;
    return 2;
}

static Direction directionFromText(String s) {
    s.toLowerCase();
    bool left = has(s, "left"), right = has(s, "right");
    if (has(s, "u-turn") || has(s, "u turn") || has(s, "make a u")) return right ? D_UTURN_RIGHT : left ? D_UTURN_LEFT : (LEFT_HAND_TRAFFIC ? D_UTURN_RIGHT : D_UTURN_LEFT);
    if (has(s, "roundabout") || has(s, "rotary") || has(s, "traffic circle")) return D_ROUNDABOUT;
    if (has(s, "arriv") || has(s, "destination")) return D_END;
    if (has(s, "sharp")) return left ? D_SHARP_LEFT : right ? D_SHARP_RIGHT : D_NONE;
    if (has(s, "slight") || has(s, "bear")) return left ? D_EASY_LEFT : right ? D_EASY_RIGHT : D_NONE;
    if (has(s, "keep") || has(s, "fork")) return left ? D_KEEP_LEFT : right ? D_KEEP_RIGHT : D_STRAIGHT;
    if (has(s, "exit") || has(s, "ramp")) return left ? D_EXIT_LEFT : right ? D_EXIT_RIGHT : D_STRAIGHT;
    if (left) return D_LEFT;
    if (right) return D_RIGHT;
    if (has(s, "head") || has(s, "continue") || has(s, "straight") || has(s, "follow")) return D_STRAIGHT;
    return D_NONE; // unknown: fall back to Google's own icon
}

// ---------- drawing primitives ----------

// Thick line with round caps
static void thickLine(float x0, float y0, float x1, float y1, int w, uint16_t c) {
    float dx = x1 - x0, dy = y1 - y0, len = sqrtf(dx * dx + dy * dy);
    if (len < 1) return;
    float nx = -dy / len * w / 2, ny = dx / len * w / 2;
    gfx->fillTriangle(x0 + nx, y0 + ny, x0 - nx, y0 - ny, x1 + nx, y1 + ny, c);
    gfx->fillTriangle(x0 - nx, y0 - ny, x1 - nx, y1 - ny, x1 + nx, y1 + ny, c);
    gfx->fillCircle(x0, y0, w / 2, c);
    gfx->fillCircle(x1, y1, w / 2, c);
}

// Arrow head at (x, y) pointing along angle (degrees, 0 = up, clockwise)
static void arrowHead(float x, float y, float angle, int size, uint16_t c) {
    float a = angle * DEG_TO_RAD;
    float fx = sinf(a), fy = -cosf(a); // forward
    float sx = cosf(a), sy = sinf(a);  // sideways
    gfx->fillTriangle(x + fx * size, y + fy * size, x - fx * size * 0.2f + sx * size, y - fy * size * 0.2f + sy * size,
                      x - fx * size * 0.2f - sx * size, y - fy * size * 0.2f - sy * size, c);
}

// Thick arc on compass angles (0 = up, clockwise), from a0 to a1 with a0 < a1
static void thickArc(int cx, int cy, int r, int w, float a0, float a1, uint16_t c) {
    gfx->fillArc(cx, cy, r + w / 2, r - w / 2, a0 - 90, a1 - 90, c);
}

static const int W = 28;                      // arrow stroke width
static const int CX = LCD_W / 2, BASE_Y = 250; // every arrow starts here (bottom)
static const int TURN_Y = 130;

// Stem from the bottom up to the turn point, then a leg at `angle` with an arrow head
static void turnArrow(float angle, float legLen = 85) {
    thickLine(CX, BASE_Y, CX, TURN_Y, W, ARROW);
    float a = angle * DEG_TO_RAD;
    float ex = CX + sinf(a) * legLen, ey = TURN_Y - cosf(a) * legLen;
    thickLine(CX, TURN_Y, ex, ey, W, ARROW);
    arrowHead(ex + sinf(a) * 10, ey - cosf(a) * 10, angle, 38, ARROW);
}

// Fork: both branches shown, the one to take bright
static void forkArrow(bool left, bool exitRamp) {
    float py = 160;
    thickLine(CX, BASE_Y, CX, py, W, ARROW);
    float otherAngle = exitRamp ? 0 : (left ? 28 : -28);
    float takeAngle = left ? -32 : 32;
    float oa = otherAngle * DEG_TO_RAD, ta = takeAngle * DEG_TO_RAD;
    thickLine(CX, py, CX + sinf(oa) * 110, py - cosf(oa) * 110, W - 6, DIM);
    float ex = CX + sinf(ta) * 95, ey = py - cosf(ta) * 95;
    thickLine(CX, py, ex, ey, W, ARROW);
    arrowHead(ex + sinf(ta) * 10, ey - cosf(ta) * 10, takeAngle, 38, ARROW);
}

static void uturnArrow(bool left) {
    int s = left ? -1 : 1, r = 46;
    int x0 = CX - s * r, x1 = CX + s * r;
    thickLine(x0, BASE_Y, x0, TURN_Y, W, ARROW);
    thickArc(CX, TURN_Y, r, W, 270, 360, ARROW);
    thickArc(CX, TURN_Y, r, W, 0, 90, ARROW);
    thickLine(x1, TURN_Y, x1, 185, W, ARROW);
    arrowHead(x1, 205, 180, 38, ARROW);
}

// Roundabout: ring, entry from the bottom, path around to exit `n`
static void roundaboutArrow(int n) {
    int rcx = CX, rcy = 140, r = 56;
    thickArc(rcx, rcy, r, W - 8, 0, 360, DIM);
    thickLine(CX, BASE_Y, CX, rcy + r, W, ARROW);
    // Exits are roughly 90 degrees apart around the ring from the entry (180):
    // left-hand traffic goes clockwise (1st exit = left), right-hand counterclockwise
    float sweep = constrain(n, 1, 4) * 90.0f;
    float from = 180, exitDeg;
    if (LEFT_HAND_TRAFFIC) {
        exitDeg = fmodf(from + sweep, 360);
        if (from + sweep <= 360) thickArc(rcx, rcy, r, W, from, from + sweep, ARROW);
        else thickArc(rcx, rcy, r, W, from, 360, ARROW), thickArc(rcx, rcy, r, W, 0, from + sweep - 360, ARROW);
    } else {
        exitDeg = fmodf(from - sweep + 360, 360);
        if (from - sweep >= 0) thickArc(rcx, rcy, r, W, from - sweep, from, ARROW);
        else thickArc(rcx, rcy, r, W, 0, from, ARROW), thickArc(rcx, rcy, r, W, from - sweep + 360, 360, ARROW);
    }
    float a = exitDeg * DEG_TO_RAD;
    float sx = rcx + sinf(a) * r, sy = rcy - cosf(a) * r;
    float ex = rcx + sinf(a) * (r + 52), ey = rcy - cosf(a) * (r + 52);
    thickLine(sx, sy, ex, ey, W, ARROW);
    arrowHead(ex + sinf(a) * 10, ey - cosf(a) * 10, exitDeg, 36, ARROW);
    // Exit number in the middle
    char num[2] = {(char)('0' + constrain(n, 1, 9)), 0};
    gfx->setTextSize(4);
    gfx->setTextColor(SOFT);
    gfx->setCursor(rcx - 11, rcy - 15);
    gfx->print(num);
}

static void flagIcon() {
    gfx->fillCircle(CX, 120, 64, 0x07E0);
    gfx->fillCircle(CX, 120, 26, BLACK);
    gfx->fillTriangle(CX - 53, 152, CX + 53, 152, CX, 250, 0x07E0);
}

// Google Maps' own 48x48 1-bit icon, scaled up
static void drawGoogleIcon(const uint8_t *icon) {
    const int scale = 4, x0 = CX - 24 * scale, y0 = 50;
    for (int y = 0; y < 48; y++)
        for (int x = 0; x < 48; x++) {
            int i = y * 48 + x;
            if ((icon[i / 8] >> (7 - i % 8)) & 1) gfx->fillRect(x0 + x * scale, y0 + y * scale, scale, scale, ARROW);
        }
}

static void drawArrow(Direction d, const NavView &v) {
    switch (d) {
    case D_STRAIGHT:    turnArrow(0, 70); break;
    case D_LEFT:        turnArrow(-90); break;
    case D_RIGHT:       turnArrow(90); break;
    case D_EASY_LEFT:   turnArrow(-45, 95); break;
    case D_EASY_RIGHT:  turnArrow(45, 95); break;
    case D_SHARP_LEFT:  turnArrow(-135, 75); break;
    case D_SHARP_RIGHT: turnArrow(135, 75); break;
    case D_KEEP_LEFT:   forkArrow(true, false); break;
    case D_KEEP_RIGHT:  forkArrow(false, false); break;
    case D_EXIT_LEFT:   forkArrow(true, true); break;
    case D_EXIT_RIGHT:  forkArrow(false, true); break;
    case D_UTURN_LEFT:  uturnArrow(true); break;
    case D_UTURN_RIGHT: uturnArrow(false); break;
    case D_ROUNDABOUT:  roundaboutArrow(exitNumber([&] { String s = v.directions; s.toLowerCase(); return s; }())); break;
    case D_END:         flagIcon(); break;
    default:
        if (v.hasIcon) drawGoogleIcon(v.icon);
        break;
    }
}

// ---------- text helpers ----------

// The built-in font is ASCII only. Map the UTF-8 that Google Maps and music apps
// use (narrow no-break space in "2:58 pm", no-break space, dashes, bullets,
// curly quotes) to ASCII and drop anything else rather than drawing garbage.
static String cleanText(const String &in) {
    String out;
    const uint8_t *p = (const uint8_t *)in.c_str();
    while (*p) {
        if (*p < 0x80) {
            out += (char)*p++;
            continue;
        }
        uint32_t cp = 0;
        int len = (*p & 0xE0) == 0xC0 ? 2 : (*p & 0xF0) == 0xE0 ? 3 : (*p & 0xF8) == 0xF0 ? 4 : 1;
        cp = len == 2 ? (*p & 0x1F) : len == 3 ? (*p & 0x0F) : (*p & 0x07);
        for (int i = 1; i < len && p[i]; i++) cp = (cp << 6) | (p[i] & 0x3F);
        p += len;
        if (cp == 0x00A0 || cp == 0x202F || cp == 0x2009 || cp == 0x2007) out += ' ';
        else if (cp == 0x2013 || cp == 0x2014 || cp == 0x2212) out += '-';
        else if (cp == 0x00B7 || cp == 0x2022) out += '-';
        else if (cp == 0x2018 || cp == 0x2019) out += '\'';
        else if (cp == 0x201C || cp == 0x201D) out += '"';
        else if (cp == 0x2026) out += "...";
        // anything else (emoji, non-Latin scripts) is skipped
    }
    out.trim();
    return out;
}

// Largest text size (up to maxSize) at which `s` fits the screen width with margins
static int fitSize(const String &s, int maxSize) {
    for (int size = maxSize; size > 1; size--)
        if ((int)s.length() * 6 * size <= LCD_W - 24) return size;
    return 1;
}

static void centeredText(int y, const String &s, int size, uint16_t c) {
    size = min(size, fitSize(s, size)); // never run off the screen edge
    gfx->setTextSize(size);
    gfx->setTextColor(c);
    gfx->setCursor((LCD_W - (int)s.length() * 6 * size) / 2, y);
    gfx->print(s);
}

// Word-wraps to at most `maxLines` lines of `maxChars`, centered
static void wrappedText(int y, String s, int size, int maxLines, uint16_t c) {
    int maxChars = (LCD_W - 24) / (6 * size);
    for (int line = 0; line < maxLines && s.length(); line++) {
        String part = s;
        if ((int)s.length() > maxChars) {
            int cut = s.lastIndexOf(' ', maxChars);
            if (cut <= 0) cut = maxChars;
            part = s.substring(0, cut);
            if (line == maxLines - 1) part = s.substring(0, maxChars - 3) + "...";
            s = s.substring(cut);
            s.trim();
        } else {
            s = "";
        }
        centeredText(y + line * (8 * size + 6), part, size, c);
    }
}

// ---------- screens ----------

// ---------- Wi-Fi ----------
// Tries each saved network in turn without blocking the loop, so navigation and
// music keep running while it connects; reconnects the same way if it drops.

static const uint32_t WIFI_TRY_MS = 12000;
static const int WIFI_COUNT = sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);

static void updateWifi() {
    static int tryIndex = -1;
    static uint32_t tryStart = 0;
    static bool wasConnected = false;
    bool connected = WiFi.status() == WL_CONNECTED;
    if (connected != wasConnected) {
        wasConnected = connected;
        if (connected) Serial.printf("Wi-Fi %s, %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
        else Serial.println("Wi-Fi lost");
    }
    if (connected) {
        tryIndex = -1;
        return;
    }
    if (tryIndex >= 0 && millis() - tryStart < WIFI_TRY_MS) return;
    tryIndex = (tryIndex + 1) % WIFI_COUNT;
    tryStart = millis();
    WiFi.disconnect();
    WiFi.begin(WIFI_NETWORKS[tryIndex][0], WIFI_NETWORKS[tryIndex][1]);
}

// Wi-Fi indicator: three arcs, lit when connected, plus the network name
static void drawWifi(int x, int y) {
    bool on = WiFi.status() == WL_CONNECTED;
    uint16_t c = on ? ACCENT : DIM;
    for (int r = 4; r <= 12; r += 4) gfx->fillArc(x, y, r, r - 2, 225, 315, c);
    gfx->fillCircle(x, y - 1, 2, c);
    gfx->setTextSize(1);
    gfx->setTextColor(c);
    gfx->setCursor(x + 14, y - 8);
    gfx->print(on ? WiFi.SSID() : String("No Wi-Fi"));
}

static void drawStatus(bool connected) {
    gfx->fillCircle(18, 18, 7, connected ? 0x07E0 : DIM);
    gfx->setTextSize(1);
    gfx->setTextColor(DIM);
    gfx->setCursor(32, 15);
    gfx->print(connected ? "Phone" : "Waiting");
    drawWifi(120, 24);
    gfx->setCursor(LCD_W - 60, 15);
    gfx->print(soundOn ? "Sound on" : "Muted");
}

static void drawWaiting() {
    centeredText(110, "BLE HUD", 4, ACCENT);
    centeredText(165, "Waiting for phone...", 2, WHITE);
    gfx->setTextSize(1);
    gfx->setTextColor(SOFT);
    const char *lines[] = {"Android: install the Chronos app (Play Store)",
                           "Chronos > Watches > Pair New Devices > Search",
                           "> select \"ESP32 HUD\"",
                           "Allow notification access, then navigate",
                           "with Google Maps.",
                           "",
                           "Double-tap: day / night   BOOT: screen off"};
    for (int i = 0; i < 7; i++) {
        gfx->setCursor((LCD_W - (int)strlen(lines[i]) * 6) / 2, 220 + i * 16);
        gfx->print(lines[i]);
    }
}

// Connected but not navigating: clock from the phone
static void drawIdle() {
    centeredText(140, chronos.getHourZ() + ":" + chronos.getTime("%M"), 9, WHITE);
    centeredText(230, chronos.getTime("%A, %d %B"), 2, SOFT);
    centeredText(300, "Start navigation in Google Maps", 1, DIM);
}

static void drawNav(const NavView &v) {
    Direction d = directionFromText(v.directions);
    drawArrow(d, v);
    // Distance to the turn, as large as fits
    if (v.step.length()) centeredText(266, v.step, 6, WHITE);
    // Instruction, up to two lines
    wrappedText(322, v.directions, 2, 2, SOFT);

    // Trip info on separate lines: arrival time, then time and distance left.
    // Google's ETA text already says "ETA" ("2:58 pm ETA"); don't add another.
    if (v.eta.length()) {
        String eta = v.eta;
        eta.replace(" ETA", "");
        eta.replace("ETA ", "");
        centeredText(378, "ETA " + eta, 2, ACCENT);
    }
    String left = v.duration;
    if (v.distance.length()) left += (left.length() ? "  -  " : "") + v.distance;
    if (left.length()) centeredText(404, left, 2, ACCENT);
}

// ---------- music page ----------

static const int HEART_X = LCD_W - 34, HEART_Y = 84;
static const int ROW_Y0 = 64, ROW_H = 46, ROWS = 7;

static void drawHeart(int x, int y, bool filled) {
    uint16_t c = filled ? 0xF8B2 : 0x8410;
    gfx->fillCircle(x - 7, y - 4, 8, c);
    gfx->fillCircle(x + 7, y - 4, 8, c);
    gfx->fillTriangle(x - 15, y - 1, x + 15, y - 1, x, y + 16, c);
    if (!filled) { // outline look: punch out the middle
        gfx->fillCircle(x - 7, y - 4, 5, BLACK);
        gfx->fillCircle(x + 7, y - 4, 5, BLACK);
        gfx->fillTriangle(x - 10, y - 1, x + 10, y - 1, x, y + 10, BLACK);
    }
}


static const int PREV_X = 82, PLAY_X = CX, NEXT_X = LCD_W - 82, CONTROL_Y = 318;
static const int SIDE_R = 40, PLAY_R = 54;
static const int VOL_Y = 392, VOL_W = 96, VOL_H = 42;
static int volButtonX(int i) { return 26 + i * 112; } // 0 = down, 1 = mute, 2 = up

static bool inCircle(int x, int y, int cx, int cy, int r) { return (x - cx) * (x - cx) + (y - cy) * (y - cy) <= r * r; }

static void skipIcon(int cx, int cy, bool forward, uint16_t c) {
    int s = forward ? 1 : -1;
    gfx->fillTriangle(cx - 14 * s, cy - 15, cx - 14 * s, cy + 15, cx + 6 * s, cy, c);
    gfx->fillRect(forward ? cx + 7 : cx - 11, cy - 15, 4, 30, c);
}

static void drawMusic(bool navigating) {
    MusicInfo &m = chronos.getMusicInfo();
    bool playing = m.state == 1;

    // Keep the next turn in view while navigating
    if (navigating) {
        String hint = view.step + "  " + view.directions;
        if (hint.length() > 40) hint = hint.substring(0, 37) + "...";
        centeredText(44, hint, 1, ACCENT);
    }

    if (m.title.length()) {
        String title = cleanText(m.title), artist = cleanText(m.artist), app = cleanText(m.appName);
        // Non-Latin titles (e.g. Telugu) can't be drawn by the built-in font
        if (title.isEmpty()) title = "Title not shown";
        centeredText(76, app.length() ? app : String("Music"), 2, ACCENT);
        drawHeart(HEART_X, HEART_Y, SavedSongs::contains(title, artist));
        wrappedText(118, title, 3, 2, WHITE);
        wrappedText(196, artist, 2, 1, SOFT);
        if (phoneMuted) centeredText(228, "MUTED", 2, 0xF800);
        else centeredText(232, playing ? "Playing" : "Paused", 1, playing ? 0x07E0 : DIM);
    } else if (!songSinceConnect) {
        // After a reconnect Chronos takes up to about a minute to resend the song
        centeredText(120, "Getting song info...", 2, SOFT);
        centeredText(160, "Arrives within a minute,", 1, DIM);
        centeredText(176, "or skip a track to refresh", 1, DIM);
    } else {
        centeredText(120, "Nothing playing", 3, SOFT);
        centeredText(170, "Start music on your phone", 2, DIM);
    }

    // Previous / play-pause / next
    gfx->fillCircle(PREV_X, CONTROL_Y, SIDE_R, 0x2104);
    skipIcon(PREV_X, CONTROL_Y, false, WHITE);
    gfx->fillCircle(PLAY_X, CONTROL_Y, PLAY_R, WHITE);
    if (playing) {
        gfx->fillRect(PLAY_X - 16, CONTROL_Y - 20, 11, 40, BLACK);
        gfx->fillRect(PLAY_X + 5, CONTROL_Y - 20, 11, 40, BLACK);
    } else {
        gfx->fillTriangle(PLAY_X - 12, CONTROL_Y - 22, PLAY_X - 12, CONTROL_Y + 22, PLAY_X + 22, CONTROL_Y, BLACK);
    }
    gfx->fillCircle(NEXT_X, CONTROL_Y, SIDE_R, 0x2104);
    skipIcon(NEXT_X, CONTROL_Y, true, WHITE);

    // Volume
    const char *const VOL[3] = {"Vol -", phoneMuted ? "Muted" : "Mute", "Vol +"};
    for (int i = 0; i < 3; i++) {
        bool mutedButton = i == 1 && phoneMuted;
        gfx->fillRoundRect(volButtonX(i), VOL_Y, VOL_W, VOL_H, 12, mutedButton ? 0xF800 : 0x2104);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->setCursor(volButtonX(i) + (VOL_W - (int)strlen(VOL[i]) * 12) / 2, VOL_Y + 13);
        gfx->print(VOL[i]);
    }
    centeredText(LCD_H - 10, "swipe right: back", 1, DIM);
}

// Taps on the music page; returns true if a control was hit
static bool tapMusic(int x, int y) {
    if (inCircle(x, y, HEART_X, HEART_Y, 26)) {
        MusicInfo &m = chronos.getMusicInfo();
        String title = cleanText(m.title), artist = cleanText(m.artist);
        if (title.isEmpty()) showToast("No song to save");
        else if (SavedSongs::add(title, artist)) showToast("Saved - swipe left to see");
        else showToast("Already saved");
        return true;
    }
    Control c;
    if (inCircle(x, y, PREV_X, CONTROL_Y, SIDE_R + 6)) c = MUSIC_PREVIOUS;
    else if (inCircle(x, y, PLAY_X, CONTROL_Y, PLAY_R + 6)) c = MUSIC_TOGGLE;
    else if (inCircle(x, y, NEXT_X, CONTROL_Y, SIDE_R + 6)) c = MUSIC_NEXT;
    else if (y >= VOL_Y && y < VOL_Y + VOL_H && x >= volButtonX(0) && x < volButtonX(0) + VOL_W) c = VOLUME_DOWN;
    else if (y >= VOL_Y && y < VOL_Y + VOL_H && x >= volButtonX(1) && x < volButtonX(1) + VOL_W) c = VOLUME_MUTE;
    else if (y >= VOL_Y && y < VOL_Y + VOL_H && x >= volButtonX(2) && x < volButtonX(2) + VOL_W) c = VOLUME_UP;
    else return false;
    if (c == VOLUME_MUTE && phoneMuted) {
        // The phone treats mute as mute-only: unmute with volume up, then step back
        // down so the volume ends where it was before muting
        chronos.musicControl(VOLUME_UP);
        delay(150);
        chronos.musicControl(VOLUME_DOWN);
        phoneMuted = false;
        return true;
    }
    chronos.musicControl(c);
    if (c == VOLUME_MUTE) phoneMuted = true;
    if (c == VOLUME_UP || c == VOLUME_DOWN) phoneMuted = false; // changing volume unmutes
    return true;
}

// ---------- saved songs page ----------

static void drawSaved() {
    centeredText(40, "Saved songs", 2, ACCENT);
    int n = SavedSongs::count();
    if (!n) {
        centeredText(170, "No saved songs yet", 2, SOFT);
        centeredText(200, "Tap the heart on the Music page", 1, DIM);
        return;
    }
    savedScroll = constrain(savedScroll, 0, max(0, n - ROWS));
    for (int r = 0; r < ROWS && savedScroll + r < n; r++) {
        const SavedSongs::Song &song = SavedSongs::get(savedScroll + r);
        int y = ROW_Y0 + r * ROW_H;
        gfx->fillRoundRect(10, y, LCD_W - 20, ROW_H - 6, 10, 0x2104);
        // Title and artist, cut to fit
        String t = song.title, a = song.artist;
        if (t.length() > 26) t = t.substring(0, 25) + "...";
        if (a.length() > 44) a = a.substring(0, 43) + "...";
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->setCursor(20, y + 5);
        gfx->print(t);
        gfx->setTextSize(1);
        gfx->setTextColor(SOFT);
        gfx->setCursor(20, y + 26);
        gfx->print(a);
        // Play triangle
        gfx->fillTriangle(LCD_W - 34, y + 10, LCD_W - 34, y + 30, LCD_W - 18, y + 20, 0x07E0);
    }
    if (n > ROWS) {
        char pos[16];
        snprintf(pos, sizeof(pos), "%d-%d of %d", savedScroll + 1, min(n, savedScroll + ROWS), n);
        centeredText(ROW_Y0 + ROWS * ROW_H + 2, pos, 1, DIM);
    }
    centeredText(LCD_H - 12, "tap: play  -  hold: delete  -  swipe right: back", 1, DIM);
}

// Row under (x, y) on the saved page, or -1
static int savedRowAt(int x, int y) {
    if (y < ROW_Y0 || x < 10 || x > LCD_W - 10) return -1;
    int r = (y - ROW_Y0) / ROW_H;
    int i = savedScroll + r;
    return r < ROWS && i < SavedSongs::count() ? i : -1;
}

static void render(bool connected) {
    gfx->fillScreen(BLACK);
    drawStatus(connected);
    if (savedPage && connected) drawSaved();
    else if (musicPage && connected) drawMusic(view.active);
    else if (view.active) drawNav(view);
    else if (connected) drawIdle();
    else drawWaiting();
    if (millis() < toastUntil) {
        int w = min(LCD_W - 20, (int)toast.length() * 12 + 28);
        gfx->fillRoundRect((LCD_W - w) / 2, 250, w, 40, 12, 0x3186);
        centeredText(262, toast, 2, WHITE);
    }
    gfx->flush();
}

// ---------- turn beeps ----------

// "350 m", "1.2 km", "500 ft", "0.3 mi" -> metres, or -1
static float parseMeters(const String &s) {
    const char *p = s.c_str();
    char *end;
    float v = strtof(p, &end);
    if (end == p) return -1;
    while (*end == ' ') end++;
    if (!strncasecmp(end, "km", 2)) return v * 1000;
    if (!strncasecmp(end, "mi", 2)) return v * 1609;
    if (!strncasecmp(end, "ft", 2)) return v * 0.3048f;
    return v;
}

enum Alert { AL_NONE, AL_NEW_INSTRUCTION, AL_TURN_NEAR };

static Alert alertFor(const NavView &prev, const NavView &cur) {
    if (!cur.active) return AL_NONE;
    if (cur.directions != prev.directions) return AL_NEW_INSTRUCTION;
    float before = parseMeters(prev.step), now = parseMeters(cur.step);
    if (before > NEAR_TURN_M && now >= 0 && now <= NEAR_TURN_M) return AL_TURN_NEAR;
    return AL_NONE;
}

static void beep(Alert a) {
    if (!soundOn) return;
    if (a == AL_NEW_INSTRUCTION) soundBeep(1200, 90);
    if (a == AL_TURN_NEAR) {
        soundBeep(1600, 90);
        delay(120);
        soundBeep(1600, 90);
    }
}

// Copies the latest navigation from Chronos
static void pullNavigation() {
    Navigation &n = chronos.getNavigation();
    NavView prev = view;
    view.active = n.active && n.isNavigation;
    view.step = cleanText(n.title);
    view.directions = cleanText(n.directions);
    view.eta = cleanText(n.eta);
    view.duration = cleanText(n.duration);
    view.distance = cleanText(n.distance);
    view.speed = cleanText(n.speed);
    view.hasIcon = n.hasIcon;
    if (n.hasIcon) memcpy(view.icon, n.icon, sizeof(view.icon));
    Serial.printf("nav %s: \"%s\" in %s (eta %s)\n", view.active ? "on" : "off", view.directions.c_str(),
                  view.step.c_str(), view.eta.c_str());
    Alert a = alertFor(prev, view);
    beep(a);
    if (a != AL_NONE) musicPage = savedPage = false; // don't let these pages hide a turn
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
    panel->setBrightness(BRIGHT_DAY);
    pinMode(BOOT_BUTTON, INPUT_PULLUP);
    if (soundBegin()) soundVolume(75);

    chronos.setConnectionCallback(onConnection);
    chronos.setConfigurationCallback(onConfig);
    chronos.begin();
    SavedSongs::begin();
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    Serial.printf("Chronos ready as \"ESP32 HUD\" (%s)\n", chronos.getAddress().c_str());
    render(false);
}

void loop() {
    static bool down = false, bootWasDown = false;
    static int startX, startY, lastX, lastY;
    static uint32_t pressStart = 0;
    static bool syncPending = false;
    static uint32_t connectedAt = 0;
    static uint32_t lastTap = 0, lastRender = 0;
    uint32_t now = millis();

    chronos.loop();
    updateWifi();

    // BOOT: screen off / on
    bool bootDown = digitalRead(BOOT_BUTTON) == LOW;
    if (bootDown && !bootWasDown) {
        screenOn = !screenOn;
        if (screenOn) panel->displayOn();
        panel->setBrightness(screenOn ? (night ? BRIGHT_NIGHT : BRIGHT_DAY) : 0);
        if (!screenOn) panel->displayOff();
        lastRender = 0;
    }
    bootWasDown = bootDown;

    // Touch (on release): swipe left = music, swipe right = back,
    // top-right = sound on/off, music controls, double-tap elsewhere = day/night
    int x, y;
    int touch = readTouch(x, y);
    if (touch == 1) {
        if (!down) {
            startX = x;
            startY = y;
            pressStart = now;
        }
        lastX = x;
        lastY = y;
    } else if (touch == 0 && down && screenOn) {
        int dx = lastX - startX, dy = lastY - startY;
        if (abs(dx) > 60 && abs(dx) > abs(dy)) {
            // Pages: main <-> music <-> saved songs
            if (!phoneConnected()) {
            } else if (dx < 0) {
                if (musicPage) savedPage = true;
                musicPage = true;
            } else {
                if (savedPage) savedPage = false;
                else musicPage = false;
            }
        } else if (savedPage && abs(dy) > 50 && abs(dy) > abs(dx)) {
            savedScroll += dy < 0 ? 3 : -3; // swipe up = further down the list
        } else if (abs(dx) < 25 && abs(dy) < 25) {
            if (startY < 40 && startX > LCD_W - 110) {
                soundOn = !soundOn;
            } else if (savedPage && phoneConnected() && savedRowAt(startX, startY) >= 0) {
                int i = savedRowAt(startX, startY);
                if (now - pressStart > 700) {
                    showToast("Deleted");
                    SavedSongs::remove(i);
                } else {
                    showToast("Playing on phone...");
                    render(phoneConnected());
                    String err = SavedSongs::play(SavedSongs::get(i));
                    showToast(err.isEmpty() ? "Sent to YouTube Music" : err);
                }
                lastTap = 0;
            } else if (musicPage && phoneConnected() && tapMusic(startX, startY)) {
                lastTap = 0;
            } else if (now - lastTap < 350) {
                night = !night;
                panel->setBrightness(night ? BRIGHT_NIGHT : BRIGHT_DAY);
                lastTap = 0;
            } else {
                lastTap = now;
            }
        }
        lastRender = 0;
    }
    if (touch >= 0) down = touch == 1;

    if (navChanged) {
        navChanged = false;
        pullNavigation();
        lastRender = 0;
    }
    if (musicChanged) {
        musicChanged = false;
        songSinceConnect = true;
        if (musicPage) lastRender = 0;
    }
    if (linkChanged) {
        linkChanged = false;
        if (!phoneConnected()) musicPage = savedPage = false;
        syncPending = phoneConnected();
        songSinceConnect = false; // the app resends the song only after a while
        connectedAt = now;
        lastRender = 0;
    }
    // After a (re)connect, ask the app to resend its state (current song etc.)
    // once it has subscribed; the app otherwise only sends music info on change
    if (syncPending && chronos.isSubscribed() && now - connectedAt > 1500) {
        syncPending = false;
        chronos.syncRequest();
        Serial.println("sync requested");
    }

    if (toastUntil && now > toastUntil) {
        toastUntil = 0;
        lastRender = 0;
    }

    // Redraw on change, and every second for the clock
    if (screenOn && (lastRender == 0 || now - lastRender > 1000)) {
        lastRender = now;
        render(phoneConnected());
    }
    delay(10);
}
