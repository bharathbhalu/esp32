// Pocket 4P remote (simulated camera): settings, profiles and hold-and-tilt gimbal control.
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Arduino_GFX_Library.h>
#include "camera.h"
#include "real_camera.h"
#include "frametap_relay.h"
#include "profiles.h"
#include "moves.h"
#include "imu.h"
#include "power.h"
#include "sound.h"

static const int I2C_SDA = 15;
static const int I2C_SCL = 14;

// SH8601 AMOLED on QSPI. GFX has no SH8601 class; CO5300 shares its command set.
static const int LCD_CS = 12, LCD_SCK = 11;
static const int LCD_D0 = 4, LCD_D1 = 5, LCD_D2 = 6, LCD_D3 = 7;
static const int LCD_W = 368, LCD_H = 448;

// TCA9554 expander drives the LCD/touch reset lines (pins 0-2)
static const uint8_t TCA9554_ADDR = 0x20;
static const uint8_t FT3168_ADDR = 0x38;

static const int BOOT_BUTTON = 0; // wakes the screen; active low
static const char *const LEVEL_NAMES[3] = {"Low", "Med", "High"};
static const uint8_t BRIGHTNESS_LEVELS[3] = {80, 170, 255};
static const int VOLUME_LEVELS[3] = {50, 65, 80};

// Tilt sensitivity: degrees ignored around the start position, and tilt for full speed
struct Sensitivity {
    const char *name;
    float deadzone, full;
};
static const Sensitivity SENSITIVITIES[] = {{"Low", 6, 45}, {"Med", 4, 30}, {"High", 2, 15}};
static const int SENS_COUNT = sizeof(SENSITIVITIES) / sizeof(SENSITIVITIES[0]);
// Flip these if the gimbal moves the wrong way when tilting
static const float YAW_SIGN = 1.0f;
static const float PITCH_SIGN = -1.0f;

// Finger travel below this is a tap; above it is a swipe
static const int SWIPE_MIN = 40;
// Up-then-down within this is contact bounce, not a new touch
static const uint32_t TOUCH_DEBOUNCE_MS = 40;

static const uint16_t BG = BLACK;
static const uint16_t PANEL = 0x18E3;
static const uint16_t GRAY = 0x4208;
static const uint16_t LIGHT_GRAY = 0xA514;
static const uint16_t ACCENT = 0x05FF; // cyan-blue
static const uint16_t REC_RED = 0xF800;
static const uint16_t OK_GREEN = 0x07E0;
static const uint16_t AMBER = 0xFD20;
static const uint16_t SLEEP_PURPLE = 0x897F;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
Arduino_CO5300 *panel = new Arduino_CO5300(bus, GFX_NOT_DEFINED, 0, false, LCD_W, LCD_H);
// Draw into a PSRAM framebuffer and flush whole frames, so redraws don't flicker
Arduino_Canvas *gfx = new Arduino_Canvas(LCD_W, LCD_H, panel);

static RealCamera realCamera;
static Camera &cam = realCamera;

// ---------- app state ----------

enum Tab { TAB_HOME, TAB_SETTINGS, TAB_PROFILES, TAB_GIMBAL, TAB_SETUP, TAB_COUNT };
static const char *const TAB_NAMES[TAB_COUNT] = {"HOME", "SET", "PROF", "GIMB", "SETUP"};

// HOME screen designs
enum Design { D_CLASSIC, D_TILES, D_DIAL, D_HUD, D_COUNT };
static const char *const DESIGN_NAMES[D_COUNT] = {"Classic", "Tiles", "Dial", "HUD"};

static Tab tab = TAB_HOME;
static int pickerSetting = -1; // setting whose option picker is open (HOME or SET)
static int profileMenu = -1;   // profile whose action popup is open
static bool sheetOpen = false; // swipe-up profiles sheet
static bool relaySheetOpen = false; // swipe-down FrameTap relay sheet
static int dialIndex = 0;      // Dial design: which setting is shown
static bool applying = false;  // a profile is being sent
static bool needsRedraw = true;

static String toast;
static uint32_t toastUntil = 0;

// Persisted preferences
static Transport method = Transport::Bluetooth;
static Design design = D_CLASSIC;
static bool tiltToggle = false; // false: tilt while holding the button; true: tap on / tap off
static int sensitivity = 1;     // index into SENSITIVITIES
static int moveSeconds = 1;     // index into MOVE_SECONDS
static bool videoWithMove = false;
static bool faceFrontFirst = false;

static const int MOVE_SECONDS[] = {3, 6, 10, 20};
static const int MOVE_SECONDS_COUNT = 4;

// Screen sleep after this long without a touch; 0 = never
static const int SLEEP_SECONDS[] = {5, 15, 30, 0};
static const int SLEEP_COUNT = 4;
static int sleepChoice = 0;
static bool asleep = false;
static uint32_t lastActivity = 0;
static const uint32_t DOUBLE_TAP_MS = 400; // two taps this close wake the screen
static bool swallowTouch = false;          // ignore the waking touch until the finger lifts

static bool soundOn = true;
static int volumeLevel = 1;     // index into VOLUME_LEVELS
static int brightnessLevel = 1; // index into BRIGHTNESS_LEVELS

// SETUP tab: category menu and its pages
enum SetupPage { SP_MENU, SP_CONNECTION, SP_DISPLAY, SP_SOUND, SP_GIMBAL, SP_ABOUT, SP_COUNT };
static const char *const SETUP_PAGE_NAMES[SP_COUNT] = {"Settings", "Connection", "Display", "Sound", "Gimbal", "About"};
static SetupPage setupPage = SP_MENU;

// Touch test (About): marks where touches land, BOOT exits
static bool touchTest = false;
static const int TOUCH_TEST_POINTS = 16;
static int touchPoints[TOUCH_TEST_POINTS][2];
static int touchPointCount = 0;

static bool imuOk = false;
static BatteryStatus boardBattery = {false, false, -1};
static uint32_t lastBatteryRead = 0;
static bool holding = false;
static float holdPitch, holdRoll;  // tilt when the hold started
static float tiltDP = 0, tiltDR = 0; // smoothed tilt relative to the hold position
static uint32_t lastStick = 0;

static int gimbalPage = 0; // 0 = control, 1 = moves (swipe left/right on GIMB)
static int slotMenu = -1;  // saved move whose action popup is open

// ---------- layout ----------

struct Rect {
    int x, y, w, h;
    bool contains(int px, int py) const { return px >= x && px < x + w && py >= y && py < y + h; }
};

static const int HEADER_H = 40;
static const int TABBAR_H = 60;
static const int CONTENT_Y = HEADER_H;
static const int ROW_H = 34;

static Rect tabRect(int i) {
    int w = LCD_W / TAB_COUNT;
    return {i * w, LCD_H - TABBAR_H, w, TABBAR_H};
}

static const Rect HOME_CONNECT = {64, 200, 240, 70};
static const Rect REC_BUTTON = {136, 282, 96, 96};

static const Rect PICKER_BACK = {14, 330, 340, 46};
static Rect pickerOption(int i) { return {14 + (i % 3) * 116, CONTENT_Y + 44 + (i / 3) * 54, 108, 46}; }

static Rect profileRow(int i) { return {14, CONTENT_Y + 6 + i * 48, 340, 44}; }
static const Rect PROFILE_SAVE = {14, 336, 340, 44};
static const Rect POPUP = {34, 110, 300, 200};
static const Rect POPUP_APPLY = {50, 170, 128, 54};
static const Rect POPUP_DELETE = {190, 170, 128, 54};
static const Rect POPUP_CANCEL = {50, 240, 268, 50};

// Profiles sheet: rises from the bottom edge
static int sheetHeight() { return max(130, 50 + Profiles::count() * 48 + 26); }
static int sheetTop() { return LCD_H - sheetHeight(); }
static Rect sheetRow(int i) { return {14, sheetTop() + 50 + i * 48, 340, 44}; }
// FrameTap relay sheet: pulls down from the top edge
static const int RELAY_SHEET_H = 300;
static const int RELAY_SHEET_Y = 0;
static const Rect RELAY_TOGGLE = {LCD_W - 120, RELAY_SHEET_Y + 16, 104, 36};
static const Rect RELAY_RESET = {16, RELAY_SHEET_Y + RELAY_SHEET_H - 62, LCD_W - 32, 40};

static Rect gimbalModeButton(int i) { return {10 + i * 88, CONTENT_Y + 8, 84, 44}; }
static const Rect GIMBAL_FIELD = {24, 100, 320, 122};

// GIMB moves page
static Rect movePreset(int i) { return {10 + (i % 4) * 88, 72 + (i / 4) * 46, 84, 40}; }
static Rect moveSecondsRect(int i) { return {110 + i * 62, 166, 56, 32}; }
static Rect moveSlot(int i) { return {14 + i * 86, 206, 82, 44}; }
static const Rect MOVE_RECORD = {14, 258, 340, 44};
static const Rect MOVE_VIDEO = {14, 308, 166, 34};
static const Rect MOVE_FRONT = {188, 308, 166, 34};
static const Rect MOVE_STATUS = {14, 348, 340, 34};
static Rect gimbalSens(int i) { return {150 + i * 70, 258, 66, 32}; }
static const Rect GIMBAL_RECENTER = {14, 300, 110, 36};
static const Rect GIMBAL_TILT_MODE = {14, 340, 110, 36};
static const Rect GIMBAL_HOLD = {134, 300, 220, 76};

// SETUP menu rows and sub-page controls
static Rect setupRow(int i) { return {8, 46 + i * 66, 352, 60}; } // i = 0..4 for pages 1..5
static const Rect SETUP_BACK = {8, 46, 96, 36};
static Rect choice4(int y, int i) { return {14 + i * 86, y, 82, 40}; }
static Rect choice3(int y, int i) { return {14 + i * 114, y, 108, 40}; }
static Rect choice2(int y, int i) { return {14 + i * 174, y, 166, 44}; }
static const Rect CONN_SLEEP = {14, 252, 340, 40};
static const Rect CONN_CONNECT = {14, 300, 340, 56};
static const Rect HOME_WAKE = {34, 200, 145, 70};
static const Rect HOME_SNAPSHOT = {189, 200, 145, 70};
static const Rect SOUND_TOGGLE = {14, 116, 340, 44};
static const Rect SOUND_TEST = {14, 260, 340, 44};
static const Rect ABOUT_TOUCH_TEST = {14, 290, 166, 56};
static const Rect ABOUT_POWER_OFF = {188, 290, 166, 56};
static bool powerOffConfirm = false; // power-off confirmation popup

// Side panel: swipe right from the left edge
static const int EDGE_SWIPE_ZONE = 30;
static const Rect SIDE_PANEL = {0, 0, 200, LCD_H};
static const Rect SIDE_SCREEN_OFF = {16, 70, 168, 60};
static const Rect SIDE_POWER_OFF = {16, 146, 168, 60};
static bool sidePanelOpen = false;

// Self-timer: countdown before the shutter / record start
static const int TIMER_SECONDS[] = {0, 3, 5, 10};
static const int TIMER_COUNT = 4;
static int selfTimer = 0;         // index into TIMER_SECONDS
static uint32_t countdownEnd = 0; // millis() when the shutter fires, 0 if no countdown
static Rect sideTimer(int i) { return {16 + i * 43, 384, 39, 28}; }

// Tiles design: 3 columns of setting tiles, then a REC pill
static Rect tileRect(int i) { return {8 + (i % 3) * 120, CONTENT_Y + 6 + (i / 3) * 66, 112, 60}; }
static const Rect TILES_REC = {104, 318, 160, 62};

// Dial design
static const Rect DIAL_REC = {144, 290, 80, 80};

// HUD design: 4 chips above and 4 below a central REC button
static Rect hudChip(int i) {
    static const int ys[4] = {52, 102, 292, 340};
    return {i % 2 ? 194 : 14, ys[i / 2], 160, 42};
}
static const Rect HUD_REC = {132, 147, 104, 104};

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

// Returns 1 while a finger is down (with its coordinates), 0 when up,
// -1 if the I2C read failed (caller should keep its previous state)
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

// Board tilt in degrees from the accelerometer
static bool readTilt(float &pitch, float &roll) {
    float ax, ay, az;
    if (!imuRead(ax, ay, az)) return false;
    pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    roll = atan2f(ay, az) * RAD_TO_DEG;
    return true;
}

static void loadPrefs() {
    Preferences prefs;
    prefs.begin("app", true);
    method = (Transport)prefs.getUChar("method", (uint8_t)Transport::Bluetooth);
    design = (Design)min((int)prefs.getUChar("design", D_CLASSIC), D_COUNT - 1);
    tiltToggle = prefs.getBool("tilttoggle", false);
    sensitivity = constrain((int)prefs.getUChar("sens", 1), 0, SENS_COUNT - 1);
    moveSeconds = constrain((int)prefs.getUChar("movesecs", 1), 0, MOVE_SECONDS_COUNT - 1);
    videoWithMove = prefs.getBool("movevideo", false);
    faceFrontFirst = prefs.getBool("movefront", false);
    Moves::setFaceFront(faceFrontFirst);
    sleepChoice = constrain((int)prefs.getUChar("sleep", 0), 0, SLEEP_COUNT - 1);
    soundOn = prefs.getBool("sound", true);
    selfTimer = constrain((int)prefs.getUChar("timer", 0), 0, TIMER_COUNT - 1);
    volumeLevel = constrain((int)prefs.getUChar("volume", 1), 0, 2);
    brightnessLevel = constrain((int)prefs.getUChar("bright", 1), 0, 2);
    prefs.end();
}

static void savePrefs() {
    Preferences prefs;
    prefs.begin("app", false);
    prefs.putUChar("method", (uint8_t)method);
    prefs.putUChar("design", (uint8_t)design);
    prefs.putBool("tilttoggle", tiltToggle);
    prefs.putUChar("sens", sensitivity);
    prefs.putUChar("movesecs", moveSeconds);
    prefs.putBool("movevideo", videoWithMove);
    prefs.putBool("movefront", faceFrontFirst);
    prefs.putUChar("sleep", sleepChoice);
    prefs.putBool("sound", soundOn);
    prefs.putUChar("timer", selfTimer);
    prefs.putUChar("volume", volumeLevel);
    prefs.putUChar("bright", brightnessLevel);
    prefs.end();
}

// ---------- drawing helpers ----------

static void text(int x, int y, const char *s, int size, uint16_t color) {
    gfx->setTextSize(size);
    gfx->setTextColor(color);
    gfx->setCursor(x, y);
    gfx->print(s);
}

static void textRight(int right, int y, const char *s, int size, uint16_t color) {
    text(right - (int)strlen(s) * 6 * size, y, s, size, color);
}

// Largest text size (up to maxSize) that fits the width
static int fitSize(const char *s, int w, int maxSize) {
    for (int size = maxSize; size > 1; size--)
        if ((int)strlen(s) * 6 * size <= w - 8) return size;
    return 1;
}

static void centered(const Rect &r, const char *s, int maxSize, uint16_t color) {
    int size = fitSize(s, r.w, maxSize);
    int tw = strlen(s) * 6 * size;
    text(r.x + (r.w - tw) / 2, r.y + (r.h - 8 * size) / 2, s, size, color);
}

static void button(const Rect &r, const char *s, uint16_t bg, uint16_t fg, int maxSize = 2) {
    gfx->fillRoundRect(r.x, r.y, r.w, r.h, 10, bg);
    centered(r, s, maxSize, fg);
}

static void showToast(const String &msg) {
    toast = msg;
    toastUntil = millis() + 2500;
    needsRedraw = true;
}

static bool connected() { return cam.link() == Link::Connected; }

static const char *methodName(Transport t) { return t == Transport::Bluetooth ? "Bluetooth" : "Wi-Fi"; }

// Settings shown for the camera's current mode (gimbal mode lives on GIMB)
static int visibleSettings(SettingId out[]) {
    int n = 0;
    for (int s = 0; s < S_COUNT; s++)
        if (s != S_GIMBAL_MODE && cam.available((SettingId)s)) out[n++] = (SettingId)s;
    return n;
}

// Current value with a short prefix where the bare value is ambiguous ("400" -> "ISO 400")
static String shortValue(SettingId s) {
    if (cam.pending(s)) return "...";
    String v = cam.label(s);
    if (s == S_ISO) return "ISO " + v;
    if (s == S_EV) return "EV " + v;
    return v;
}

// Shared REC / SHOOT button, drawn as a circle inside r
static void drawRecButton(const Rect &r) {
    int cx = r.x + r.w / 2, cy = r.y + r.h / 2, rad = min(r.w, r.h) / 2 - 2;
    if (cam.isPhotoMode()) {
        gfx->fillCircle(cx, cy, rad, WHITE);
        gfx->fillCircle(cx, cy, rad - 8, LIGHT_GRAY);
        centered(r, "SHOOT", 2, BLACK);
    } else if (cam.recording()) {
        gfx->fillCircle(cx, cy, rad, REC_RED);
        uint32_t s = cam.recordingSeconds();
        char t[8];
        snprintf(t, sizeof(t), "%02lu:%02lu", s / 60, s % 60);
        centered(r, t, 2, WHITE);
    } else {
        gfx->fillCircle(cx, cy, rad, WHITE);
        gfx->fillCircle(cx, cy, rad - 6, REC_RED);
        centered(r, "REC", 2, WHITE);
    }
    // Self-timer badge (not shown while recording: stopping is always instant)
    if (TIMER_SECONDS[selfTimer] && !cam.recording()) {
        char t[6];
        snprintf(t, sizeof(t), "%ds", TIMER_SECONDS[selfTimer]);
        int bx = cx + rad * 7 / 10, by = cy + rad * 7 / 10;
        gfx->fillCircle(bx, by, 13, AMBER);
        centered({bx - 13, by - 13, 26, 26}, t, 1, BLACK);
    }
}

// ---------- self-timer ----------

// REC / SHOOT pressed: start the countdown if a self-timer is set, else fire now
static void pressShutter() {
    if (TIMER_SECONDS[selfTimer] && !cam.recording()) countdownEnd = millis() + TIMER_SECONDS[selfTimer] * 1000;
    else cam.shutter();
}

static int countdownLeft() { return countdownEnd ? (int)((countdownEnd - millis() + 999) / 1000) : 0; }

// Full-screen countdown: big number inside a draining ring
static void drawCountdown() {
    gfx->fillScreen(BLACK);
    int total = TIMER_SECONDS[selfTimer] * 1000;
    float left = constrain((int32_t)(countdownEnd - millis()), 0, total) / (float)max(total, 1);
    int cx = LCD_W / 2, cy = LCD_H / 2 - 20;
    gfx->fillArc(cx, cy, 130, 118, 0, 360, PANEL);
    gfx->fillArc(cx, cy, 130, 118, 270, 270 + 360 * left, countdownLeft() <= 1 ? REC_RED : AMBER);
    char n[4];
    snprintf(n, sizeof(n), "%d", countdownLeft());
    centered({cx - 110, cy - 110, 220, 220}, n, 14, WHITE);
    centered({0, cy + 150, LCD_W, 20}, cam.isPhotoMode() ? "Photo in..." : "Recording in...", 2, LIGHT_GRAY);
    centered({0, cy + 180, LCD_W, 20}, "Tap to cancel", 1, GRAY);
}

// Beeps each second, fires at zero
static void updateCountdown() {
    static int lastBeep = 0;
    if (!countdownEnd) {
        lastBeep = 0;
        return;
    }
    if (!connected()) {
        countdownEnd = 0;
        return;
    }
    int left = countdownLeft();
    if ((int32_t)(millis() - countdownEnd) >= 0) {
        countdownEnd = 0;
        if (soundOn) soundBeep(2000, 220);
        cam.shutter();
        needsRedraw = true;
        return;
    }
    if (left != lastBeep) {
        lastBeep = left;
        if (soundOn) soundBeep(left == 1 ? 1500 : 1000, 90);
    }
}

// ---------- screens ----------

// Board battery: icon + percentage at the left of the header. Returns the x after it.
static int drawBoardBattery(int x) {
    const BatteryStatus &b = boardBattery;
    if (!b.present) {
        text(x, 16, "USB", 1, LIGHT_GRAY);
        return x + 24;
    }
    int pct = max(b.percent, 0);
    uint16_t fill = b.charging ? AMBER : pct <= 15 ? REC_RED : pct <= 35 ? AMBER : OK_GREEN;
    gfx->drawRoundRect(x, 13, 22, 12, 2, LIGHT_GRAY);
    gfx->fillRect(x + 22, 16, 2, 6, LIGHT_GRAY);
    gfx->fillRect(x + 2, 15, 18 * pct / 100, 8, fill);
    char label[8];
    if (b.percent < 0) strcpy(label, "--");
    else snprintf(label, sizeof(label), b.charging ? "%d+" : "%d%%", b.percent);
    text(x + 28, 16, label, 1, WHITE);
    return x + 28 + strlen(label) * 6;
}

static void drawHeader() {
    gfx->fillRect(0, 0, LCD_W, HEADER_H, PANEL);
    text(12, 12, "P4P", 2, WHITE);
    text(50, 16, "SIM", 1, AMBER);
    drawBoardBattery(78);

    Link l = cam.link();
    uint16_t dot = l == Link::Connected ? OK_GREEN : l == Link::Connecting ? AMBER : cam.asleep() ? SLEEP_PURPLE : GRAY;
    Transport t = l == Link::Disconnected ? method : cam.transport();
    const char *tName = t == Transport::Bluetooth ? "BLE" : "WiFi";
    int right = LCD_W - 12;
    if (connected()) {
        // Camera battery
        char batt[12];
        snprintf(batt, sizeof(batt), "CAM %d%%", cam.battery());
        textRight(right, 12, batt, 2, WHITE);
        right -= strlen(batt) * 12 + 12;
    }
    textRight(right, 12, tName, 2, LIGHT_GRAY);
    right -= strlen(tName) * 12 + 14;
    gfx->fillCircle(right, 19, 6, dot);
    // Grab handle hinting at the swipe-down relay sheet
    gfx->fillRoundRect(LCD_W / 2 - 20, HEADER_H - 6, 40, 3, 1, GRAY);
}

static void drawTabBar() {
    gfx->fillRect(0, LCD_H - TABBAR_H, LCD_W, TABBAR_H, PANEL);
    for (int i = 0; i < TAB_COUNT; i++) {
        Rect r = tabRect(i);
        if (i == tab) gfx->fillRect(r.x + 10, r.y, r.w - 20, 4, ACCENT);
        centered(r, TAB_NAMES[i], 2, i == tab ? ACCENT : LIGHT_GRAY);
    }
}

static void drawNotConnected() {
    if (cam.asleep()) {
        centered({0, 110, LCD_W, 40}, "Camera asleep", 3, SLEEP_PURPLE);
        if (const char *activity = cam.powerActivity()) {
            centered({0, 150, LCD_W, 30}, activity, 2, AMBER);
        } else {
            centered({0, 150, LCD_W, 30}, "Wake within 30 min of sleep", 2, GRAY);
            button(HOME_WAKE, "Wake", ACCENT, BLACK, 3);
            button(HOME_SNAPSHOT, "Snapshot", REC_RED, WHITE, 3);
        }
        return;
    }
    bool connecting = cam.link() == Link::Connecting;
    centered({0, 110, LCD_W, 40}, connecting ? "Connecting..." : "Not connected", 3,
             connecting ? AMBER : LIGHT_GRAY);
    char line[40];
    snprintf(line, sizeof(line), "Method: %s", methodName(method));
    centered({0, 150, LCD_W, 30}, line, 2, GRAY);
    if (!connecting) button(HOME_CONNECT, "Connect", ACCENT, BLACK, 3);
}

static void drawPicker() {
    SettingId s = (SettingId)pickerSetting;
    centered({0, CONTENT_Y + 6, LCD_W, 30}, SETTINGS[s].name, 2, WHITE);
    int n = cam.optionCount(s);
    for (int i = 0; i < n; i++) {
        bool selected = i == cam.value(s);
        button(pickerOption(i), cam.optionLabel(s, i), selected ? ACCENT : GRAY, selected ? BLACK : WHITE);
    }
    button(PICKER_BACK, "Back", PANEL, WHITE);
}

// --- HOME designs ---

// Ring gauge: grey track, coloured arc clockwise from the top, % in the middle, label below
static void drawRing(int cx, int cy, int r, int pct, uint16_t color, const char *label) {
    int thick = max(5, r / 5);
    gfx->fillArc(cx, cy, r, r - thick, 0, 360, GRAY);
    if (pct > 0) gfx->fillArc(cx, cy, r, r - thick, 270, 270 + 3.6f * min(pct, 100), color);
    char v[6];
    snprintf(v, sizeof(v), "%d%%", pct);
    centered({cx - r, cy - r, r * 2, r * 2}, v, r >= 34 ? 2 : 1, WHITE);
    centered({cx - r - 10, cy + r + 2, r * 2 + 20, 12}, label, 1, LIGHT_GRAY);
}

// Camera battery and card rings, side by side
static void drawCameraRings(int cy, int r, int leftX, int rightX) {
    int batt = cam.battery(), card = cam.storageUsed();
    uint16_t battColor = batt <= 15 ? REC_RED : batt <= 35 ? AMBER : OK_GREEN;
    uint16_t cardColor = card >= 90 ? REC_RED : card >= 75 ? AMBER : ACCENT;
    drawRing(leftX, cy, r, batt, battColor, "CAM BATT");
    drawRing(rightX, cy, r, card, cardColor, "CARD USED");
}

static void homeRow(int &y, const char *key, const char *value) {
    text(20, y, key, 2, LIGHT_GRAY);
    text(150, y, value, 2, WHITE);
    y += ROW_H;
}

static void drawHomeClassic() {
    int y = CONTENT_Y + 14;
    homeRow(y, "Mode", cam.label(S_MODE));
    if (cam.available(S_FORMAT)) homeRow(y, "Format", cam.label(S_FORMAT));
    char expo[40];
    if (cam.value(S_EXPOSURE) == 1)
        snprintf(expo, sizeof(expo), "ISO %s  %s", cam.label(S_ISO), cam.label(S_SHUTTER));
    else
        snprintf(expo, sizeof(expo), "Auto  EV %s", cam.label(S_EV));
    homeRow(y, "Exposure", expo);
    homeRow(y, "White bal.", cam.label(S_WB));
    if (cam.available(S_COLOR)) homeRow(y, "Color", cam.label(S_COLOR));
    homeRow(y, "Focus", cam.label(S_FOCUS));
    drawRecButton(REC_BUTTON);
}

static void drawHomeTiles() {
    SettingId list[S_COUNT];
    int n = min(visibleSettings(list), 12);
    for (int i = 0; i < n; i++) {
        Rect r = tileRect(i);
        gfx->fillRoundRect(r.x, r.y, r.w, r.h, 10, PANEL);
        centered({r.x, r.y + 4, r.w, 16}, SETTINGS[list[i]].name, 1, LIGHT_GRAY);
        String v = cam.pending(list[i]) ? "..." : cam.label(list[i]);
        centered({r.x, r.y + 20, r.w, r.h - 22}, v.c_str(), 3, cam.pending(list[i]) ? AMBER : WHITE);
    }
    // REC pill
    const Rect &p = TILES_REC;
    uint16_t bg = cam.isPhotoMode() ? LIGHT_GRAY : cam.recording() ? REC_RED : PANEL;
    gfx->fillRoundRect(p.x, p.y, p.w, p.h, p.h / 2, bg);
    if (!cam.isPhotoMode() && !cam.recording()) {
        gfx->drawRoundRect(p.x, p.y, p.w, p.h, p.h / 2, REC_RED);
        gfx->fillCircle(p.x + 34, p.y + p.h / 2, 12, REC_RED);
        text(p.x + 58, p.y + p.h / 2 - 12, "REC", 3, WHITE);
    } else if (cam.recording()) {
        uint32_t s = cam.recordingSeconds();
        char t[8];
        snprintf(t, sizeof(t), "%02lu:%02lu", s / 60, s % 60);
        centered(p, t, 3, WHITE);
    } else {
        centered(p, "SHOOT", 3, BLACK);
    }
}

static void drawHomeDial() {
    SettingId list[S_COUNT];
    int n = visibleSettings(list);
    dialIndex = constrain(dialIndex, 0, n - 1);
    SettingId s = list[dialIndex];

    // Position dots for the setting list
    int dotsX = LCD_W / 2 - (n - 1) * 7;
    for (int i = 0; i < n; i++) gfx->fillCircle(dotsX + i * 14, CONTENT_Y + 14, 3, i == dialIndex ? ACCENT : GRAY);

    centered({0, CONTENT_Y + 30, LCD_W, 36}, SETTINGS[s].name, 3, LIGHT_GRAY);

    int v = cam.value(s), count = cam.optionCount(s);
    if (v > 0) centered({0, 130, 90, 60}, cam.optionLabel(s, v - 1), 2, GRAY);
    if (v < count - 1) centered({278, 130, 90, 60}, cam.optionLabel(s, v + 1), 2, GRAY);
    gfx->drawRoundRect(90, 120, 188, 80, 14, ACCENT);
    centered({90, 120, 188, 80}, cam.pending(s) ? "..." : cam.label(s), 4, cam.pending(s) ? AMBER : WHITE);

    centered({0, 212, LCD_W, 14}, "< swipe to change value >", 1, GRAY);
    centered({0, 228, LCD_W, 14}, "swipe up / down for next setting", 1, GRAY);

    String summary = String(cam.label(S_MODE));
    if (cam.available(S_FORMAT)) summary += "  " + String(cam.label(S_FORMAT));
    if (cam.available(S_COLOR)) summary += "  " + String(cam.label(S_COLOR));
    centered({0, 252, LCD_W, 28}, summary.c_str(), 2, LIGHT_GRAY);

    drawRecButton(DIAL_REC);
}

// HUD chip order: what a shooter checks first
static int hudSettings(SettingId out[]) {
    static const SettingId ORDER[] = {S_MODE, S_FORMAT, S_ISO, S_SHUTTER, S_EV, S_WB, S_COLOR,
                                      S_FOCUS, S_EXPOSURE, S_ASPECT, S_TIMER, S_AUDIO, S_VOCAL};
    int n = 0;
    for (SettingId s : ORDER)
        if (n < 8 && cam.available(s)) out[n++] = s;
    return n;
}

static void drawHomeHud() {
    SettingId list[8];
    int n = hudSettings(list);
    for (int i = 0; i < n; i++) {
        Rect r = hudChip(i);
        gfx->drawRoundRect(r.x, r.y, r.w, r.h, r.h / 2, GRAY);
        centered(r, shortValue(list[i]).c_str(), 2, cam.pending(list[i]) ? AMBER : WHITE);
    }
    drawRecButton(HUD_REC);
    if (!cam.isPhotoMode()) {
        uint32_t s = cam.recordingSeconds();
        char t[8];
        snprintf(t, sizeof(t), "%02lu:%02lu", s / 60, s % 60);
        centered({0, 256, LCD_W, 26}, t, 2, cam.recording() ? REC_RED : GRAY);
    }
}

static void drawHome() {
    if (!connected()) {
        drawNotConnected();
        return;
    }
    if (pickerSetting >= 0) {
        drawPicker();
        return;
    }
    switch (design) {
    case D_TILES: drawHomeTiles(); break;
    case D_DIAL:  drawHomeDial(); break;
    case D_HUD:   drawHomeHud(); break;
    default:      drawHomeClassic(); break;
    }
}

// --- other tabs ---

static void drawSettings() {
    if (!connected()) {
        drawNotConnected();
        return;
    }
    if (pickerSetting >= 0) {
        drawPicker();
        return;
    }
    SettingId list[S_COUNT];
    int n = visibleSettings(list);
    for (int i = 0; i < n; i++) {
        int y = CONTENT_Y + 4 + i * ROW_H;
        text(16, y + 9, SETTINGS[list[i]].name, 2, LIGHT_GRAY);
        if (cam.pending(list[i])) textRight(LCD_W - 16, y + 9, "...", 2, AMBER);
        else textRight(LCD_W - 16, y + 9, cam.label(list[i]), 2, ACCENT);
        gfx->drawFastHLine(16, y + ROW_H - 1, LCD_W - 32, PANEL);
    }
}

static void drawProfiles() {
    for (int i = 0; i < Profiles::count(); i++) {
        const Profile &p = Profiles::get(i);
        Rect r = profileRow(i);
        gfx->fillRoundRect(r.x, r.y, r.w, r.h, 8, PANEL);
        text(r.x + 12, r.y + 6, p.name, 2, WHITE);
        text(r.x + 12, r.y + 28, Profiles::summary(p).c_str(), 1, LIGHT_GRAY);
    }
    if (applying) {
        char msg[32];
        snprintf(msg, sizeof(msg), "Applying... %d left", cam.queued());
        button(PROFILE_SAVE, msg, PANEL, AMBER);
    } else {
        button(PROFILE_SAVE, "+ Save current settings", GRAY, WHITE);
    }

    if (profileMenu >= 0) {
        const Profile &p = Profiles::get(profileMenu);
        gfx->fillRoundRect(POPUP.x, POPUP.y, POPUP.w, POPUP.h, 14, PANEL);
        gfx->drawRoundRect(POPUP.x, POPUP.y, POPUP.w, POPUP.h, 14, LIGHT_GRAY);
        centered({POPUP.x, POPUP.y + 14, POPUP.w, 30}, p.name, 2, WHITE);
        button(POPUP_APPLY, "Apply", ACCENT, BLACK);
        button(POPUP_DELETE, "Delete", REC_RED, WHITE);
        button(POPUP_CANCEL, "Cancel", GRAY, WHITE);
    }
}

static void drawMoves() {
    text(16, 48, "Moves", 2, WHITE);
    textRight(LCD_W - 16, 52, "swipe right: back", 1, GRAY);

    bool busy = Moves::playing() || Moves::recording();
    for (int i = 0; i < Moves::PRESET_COUNT; i++)
        button(movePreset(i), Moves::presetName(i), busy ? PANEL : GRAY, busy ? GRAY : WHITE);

    text(16, 174, "Time", 2, LIGHT_GRAY);
    for (int i = 0; i < MOVE_SECONDS_COUNT; i++) {
        char label[8];
        snprintf(label, sizeof(label), "%ds", MOVE_SECONDS[i]);
        button(moveSecondsRect(i), label, i == moveSeconds ? ACCENT : GRAY, i == moveSeconds ? BLACK : WHITE);
    }

    for (int i = 0; i < Moves::SLOT_COUNT; i++) {
        Rect r = moveSlot(i);
        char label[16];
        if (Moves::slotUsed(i)) snprintf(label, sizeof(label), "%d: %.0fs", i + 1, Moves::slotSeconds(i));
        else snprintf(label, sizeof(label), "%d: --", i + 1);
        bool used = Moves::slotUsed(i);
        button(r, label, used ? ACCENT : PANEL, used ? BLACK : GRAY);
    }

    if (Moves::recording()) {
        char label[32];
        snprintf(label, sizeof(label), "STOP  %.1f s", Moves::recordedSeconds());
        button(MOVE_RECORD, label, REC_RED, WHITE);
    } else {
        button(MOVE_RECORD, "Record a move (tilt)", Moves::playing() ? PANEL : GRAY,
               Moves::playing() ? GRAY : WHITE);
    }

    button(MOVE_VIDEO, videoWithMove ? "Video: ON" : "Video: OFF", videoWithMove ? ACCENT : PANEL,
           videoWithMove ? BLACK : LIGHT_GRAY);
    button(MOVE_FRONT, faceFrontFirst ? "Face front: ON" : "Face front: OFF", faceFrontFirst ? ACCENT : PANEL,
           faceFrontFirst ? BLACK : LIGHT_GRAY);

    if (Moves::playing()) {
        String s = "STOP   " + Moves::status();
        button(MOVE_STATUS, s.c_str(), AMBER, BLACK);
    } else if (Moves::recording()) {
        centered(MOVE_STATUS, "Tilt the board to steer", 2, OK_GREEN);
    } else {
        char pos[40];
        snprintf(pos, sizeof(pos), "Pan %+.0f  Tilt %+.0f", cam.gimbalYaw(), cam.gimbalPitch());
        centered(MOVE_STATUS, pos, 2, LIGHT_GRAY);
    }

    if (slotMenu >= 0) {
        char title[24];
        snprintf(title, sizeof(title), "Move %d  (%.1f s)", slotMenu + 1, Moves::slotSeconds(slotMenu));
        gfx->fillRoundRect(POPUP.x, POPUP.y, POPUP.w, POPUP.h, 14, PANEL);
        gfx->drawRoundRect(POPUP.x, POPUP.y, POPUP.w, POPUP.h, 14, LIGHT_GRAY);
        centered({POPUP.x, POPUP.y + 14, POPUP.w, 30}, title, 2, WHITE);
        button(POPUP_APPLY, "Play", ACCENT, BLACK);
        button(POPUP_DELETE, "Delete", REC_RED, WHITE);
        button(POPUP_CANCEL, "Cancel", GRAY, WHITE);
    }
}

static void drawGimbal() {
    if (!connected()) {
        drawNotConnected();
        return;
    }
    if (gimbalPage == 1) {
        drawMoves();
        return;
    }
    textRight(LCD_W - 16, 93, "swipe left: Moves", 1, GRAY);
    for (int i = 0; i < cam.optionCount(S_GIMBAL_MODE); i++) {
        bool selected = i == cam.value(S_GIMBAL_MODE);
        const char *label = cam.pending(S_GIMBAL_MODE) && selected ? "..." : cam.optionLabel(S_GIMBAL_MODE, i);
        button(gimbalModeButton(i), label, selected ? ACCENT : GRAY, selected ? BLACK : WHITE);
    }

    // Field to scale over the reachable range, so level/center is off-middle.
    // Crosshair and dot share the same mapping.
    const Rect &f = GIMBAL_FIELD;
    auto yawX = [&](float yaw) {
        return f.x + 12 + (int)((yaw - GIMBAL_YAW_MIN) / (GIMBAL_YAW_MAX - GIMBAL_YAW_MIN) * (f.w - 24));
    };
    auto pitchY = [&](float pitch) {
        return f.y + 10 + (int)((GIMBAL_PITCH_MAX - pitch) / (GIMBAL_PITCH_MAX - GIMBAL_PITCH_MIN) * (f.h - 20));
    };
    gfx->drawRoundRect(f.x, f.y, f.w, f.h, 10, GRAY);
    gfx->drawFastHLine(f.x + 6, pitchY(0), f.w - 12, GRAY);
    gfx->drawFastVLine(yawX(0), f.y + 6, f.h - 12, GRAY);
    gfx->fillCircle(yawX(cam.gimbalYaw()), pitchY(cam.gimbalPitch()), 9, holding ? OK_GREEN : ACCENT);

    char angles[40];
    snprintf(angles, sizeof(angles), "Pan %+4.0f   Tilt %+4.0f", cam.gimbalYaw(), cam.gimbalPitch());
    centered({0, 226, LCD_W, 26}, angles, 2, WHITE);

    text(14, 266, "Sensitivity", 2, LIGHT_GRAY);
    for (int i = 0; i < SENS_COUNT; i++)
        button(gimbalSens(i), SENSITIVITIES[i].name, i == sensitivity ? ACCENT : GRAY,
               i == sensitivity ? BLACK : WHITE);

    button(GIMBAL_RECENTER, "Center", GRAY, WHITE);
    button(GIMBAL_TILT_MODE, tiltToggle ? "Mode: Tap" : "Mode: Hold", PANEL, WHITE);
    const char *label = tiltToggle ? (holding ? "TILTING - TAP OFF" : "TAP TO TILT")
                                   : (holding ? "MOVING" : "HOLD + TILT");
    button(GIMBAL_HOLD, label, holding ? OK_GREEN : ACCENT, BLACK);
}

static void choiceButton(const Rect &r, const char *label, bool selected) {
    button(r, label, selected ? ACCENT : GRAY, selected ? BLACK : WHITE);
}

static String sleepLabel(int i) { return SLEEP_SECONDS[i] ? String(SLEEP_SECONDS[i]) + "s" : "Off"; }

static String batteryText() {
    const BatteryStatus &b = boardBattery;
    if (!b.present) return "USB power";
    String s = b.percent < 0 ? "--" : String(b.percent) + "%";
    return b.charging ? s + " charging" : s;
}

// One-line summary shown on each menu row
static String setupSummary(SetupPage p) {
    switch (p) {
    case SP_CONNECTION: {
        const char *state = cam.link() == Link::Connected    ? "on"
                            : cam.link() == Link::Connecting ? "..."
                            : cam.asleep()                   ? "asleep"
                                                             : "off";
        return String(methodName(method)) + " - " + state;
    }
    case SP_DISPLAY: return String(DESIGN_NAMES[design]) + " - " + sleepLabel(sleepChoice);
    case SP_SOUND:   return soundOn ? String("On - ") + LEVEL_NAMES[volumeLevel] : "Off";
    case SP_GIMBAL:  return String(tiltToggle ? "Tap" : "Hold") + " - " + SENSITIVITIES[sensitivity].name;
    case SP_ABOUT:   return batteryText();
    default:         return "";
    }
}

static void drawSetupMenu() {
    for (int p = SP_CONNECTION; p < SP_COUNT; p++) {
        Rect r = setupRow(p - 1);
        gfx->fillRoundRect(r.x, r.y, r.w, r.h, 10, PANEL);
        text(r.x + 14, r.y + 22, SETUP_PAGE_NAMES[p], 2, WHITE);
        String summary = setupSummary((SetupPage)p);
        textRight(r.x + r.w - 34, r.y + 22, summary.c_str(), 2, LIGHT_GRAY);
        text(r.x + r.w - 22, r.y + 20, ">", 3, GRAY);
    }
}

static void drawSetupConnection() {
    text(16, 92, "Method", 2, LIGHT_GRAY);
    bool ble = method == Transport::Bluetooth;
    choiceButton(choice2(116, 0), "Bluetooth", ble);
    choiceButton(choice2(116, 1), "Wi-Fi", !ble);
    text(16, 170, ble ? "Commands over Bluetooth only." : "Uses the camera's Wi-Fi link (faster).", 1, LIGHT_GRAY);
    text(16, 184, ble ? "May work alongside FrameTap (untested)." : "FrameTap must be disconnected.", 1, LIGHT_GRAY);

    if (connected()) button(CONN_SLEEP, "Put camera to sleep", PANEL, SLEEP_PURPLE);

    if (const char *activity = cam.powerActivity()) {
        button(CONN_CONNECT, activity, PANEL, AMBER);
    } else if (cam.asleep()) {
        button(CONN_CONNECT, "Wake camera", SLEEP_PURPLE, WHITE, 3);
    } else {
        switch (cam.link()) {
        case Link::Connected:    button(CONN_CONNECT, "Disconnect", REC_RED, WHITE, 3); break;
        case Link::Connecting:   button(CONN_CONNECT, "Connecting...", PANEL, AMBER, 3); break;
        case Link::Disconnected: button(CONN_CONNECT, "Connect", ACCENT, BLACK, 3); break;
        }
    }
}

static void drawSetupDisplay() {
    text(16, 92, "Home design", 2, LIGHT_GRAY);
    for (int i = 0; i < D_COUNT; i++) choiceButton(choice4(114, i), DESIGN_NAMES[i], i == design);
    text(16, 170, "Sleep after", 2, LIGHT_GRAY);
    for (int i = 0; i < SLEEP_COUNT; i++) choiceButton(choice4(192, i), sleepLabel(i).c_str(), i == sleepChoice);
    text(16, 248, "Brightness", 2, LIGHT_GRAY);
    for (int i = 0; i < 3; i++) choiceButton(choice3(270, i), LEVEL_NAMES[i], i == brightnessLevel);
    text(16, 330, "BOOT button: screen off / on. Double-tap also wakes.", 1, GRAY);
}

static void drawSetupSound() {
    text(16, 92, "Tap sound", 2, LIGHT_GRAY);
    choiceButton(SOUND_TOGGLE, soundOn ? "On" : "Off", soundOn);
    text(16, 176, "Volume", 2, LIGHT_GRAY);
    for (int i = 0; i < 3; i++) choiceButton(choice3(198, i), LEVEL_NAMES[i], i == volumeLevel);
    button(SOUND_TEST, "Play test click", PANEL, WHITE);
}

static void drawSetupGimbal() {
    text(16, 92, "Tilt control", 2, LIGHT_GRAY);
    choiceButton(choice2(116, 0), "Hold", !tiltToggle);
    choiceButton(choice2(116, 1), "Tap on/off", tiltToggle);
    text(16, 176, "Sensitivity", 2, LIGHT_GRAY);
    for (int i = 0; i < SENS_COUNT; i++) choiceButton(choice3(198, i), SENSITIVITIES[i].name, i == sensitivity);
    text(16, 256, "Also on the GIMB tab.", 1, GRAY);
}

static void drawSetupAbout() {
    int y = 96;
    auto line = [&](const char *key, const String &value) {
        text(16, y, key, 2, LIGHT_GRAY);
        text(150, y, value.c_str(), 2, WHITE);
        y += 34;
    };
    line("Battery", batteryText());
    line("Camera", "Simulated 4P");
    line("Link", setupSummary(SP_CONNECTION));
    line("Motion", imuOk ? "OK" : "Missing");
    line("Profiles", String(Profiles::count()) + " saved");
    button(ABOUT_TOUCH_TEST, "Touch test", GRAY, WHITE);
    button(ABOUT_POWER_OFF, "Power off", REC_RED, WHITE);
}

// Power-off confirmation (from About or the side panel), drawn over any screen
static void drawPowerConfirm() {
    bool usb = powerUsbPresent();
    gfx->fillRoundRect(POPUP.x, POPUP.y, POPUP.w, POPUP.h, 14, PANEL);
    gfx->drawRoundRect(POPUP.x, POPUP.y, POPUP.w, POPUP.h, 14, LIGHT_GRAY);
    centered({POPUP.x, POPUP.y + 10, POPUP.w, 26}, "Power off?", 2, WHITE);
    centered({POPUP.x, POPUP.y + 36, POPUP.w, 16},
             usb ? "On USB power - board will restart" : "Short press PWR to turn on", 1,
             usb ? AMBER : LIGHT_GRAY);
    button(POPUP_APPLY, "Off", REC_RED, WHITE);
    button(POPUP_CANCEL, "Cancel", GRAY, WHITE);
}

// Side panel, opened by swiping right from the left edge
static void drawSidePanel() {
    gfx->fillRect(0, 0, SIDE_PANEL.w, LCD_H, PANEL);
    gfx->drawFastVLine(SIDE_PANEL.w, 0, LCD_H, GRAY);
    text(16, 22, "Quick", 2, WHITE);
    button(SIDE_SCREEN_OFF, "Screen off", SLEEP_PURPLE, WHITE);
    button(SIDE_POWER_OFF, "Power off", REC_RED, WHITE);

    text(16, 232, "Pocket 4P", 2, WHITE);
    if (connected()) {
        drawCameraRings(300, 40, 52, SIDE_PANEL.w - 52);
    } else {
        text(16, 262, "Not connected", 1, LIGHT_GRAY);
        text(16, 276, "Battery and card show here", 1, GRAY);
        text(16, 288, "once the camera connects.", 1, GRAY);
    }

    text(16, 368, "Self-timer", 1, LIGHT_GRAY);
    for (int i = 0; i < TIMER_COUNT; i++) {
        char label[6];
        if (TIMER_SECONDS[i]) snprintf(label, sizeof(label), "%ds", TIMER_SECONDS[i]);
        else strcpy(label, "Off");
        bool sel = i == selfTimer;
        button(sideTimer(i), label, sel ? AMBER : GRAY, sel ? BLACK : WHITE, 1);
    }
    text(16, LCD_H - 24, "< swipe to close", 1, GRAY);
}

static void shutDown() {
    if (cam.link() != Link::Disconnected) cam.disconnect();
    gfx->fillScreen(BG);
    centered({0, 200, LCD_W, 40}, "Powering off...", 3, LIGHT_GRAY);
    gfx->flush();
    Serial.println("power off");
    delay(800);
    panel->setBrightness(0);
    panel->displayOff();
    powerOff();
    delay(2000); // still here: USB is keeping the board powered
    ESP.restart();
}

static void drawSetup() {
    if (setupPage == SP_MENU) {
        drawSetupMenu();
        return;
    }
    button(SETUP_BACK, "< Back", PANEL, WHITE);
    text(118, 56, SETUP_PAGE_NAMES[setupPage], 2, WHITE);
    switch (setupPage) {
    case SP_CONNECTION: drawSetupConnection(); break;
    case SP_DISPLAY:    drawSetupDisplay(); break;
    case SP_SOUND:      drawSetupSound(); break;
    case SP_GIMBAL:     drawSetupGimbal(); break;
    default:            drawSetupAbout(); break;
    }
}

// Full screen: crosshairs at every touch since the test started
static void drawTouchTest() {
    gfx->fillScreen(BG);
    gfx->drawRect(0, 0, LCD_W, LCD_H, GRAY);
    centered({0, 180, LCD_W, 30}, "Touch test", 3, LIGHT_GRAY);
    centered({0, 214, LCD_W, 20}, "Tap corners and edges - BOOT exits", 1, GRAY);
    for (int i = 0; i < touchPointCount; i++) {
        int x = touchPoints[i][0], y = touchPoints[i][1];
        bool last = i == touchPointCount - 1;
        uint16_t c = last ? OK_GREEN : ACCENT;
        gfx->drawFastHLine(x - 12, y, 25, c);
        gfx->drawFastVLine(x, y - 12, 25, c);
        if (last) {
            char pos[24];
            snprintf(pos, sizeof(pos), "x=%d y=%d", x, y);
            centered({0, 250, LCD_W, 24}, pos, 2, WHITE);
        }
    }
    gfx->flush();
}

static void drawSheet() {
    int y = sheetTop();
    gfx->fillRect(0, y, LCD_W, sheetHeight(), PANEL);
    gfx->drawFastHLine(0, y, LCD_W, GRAY);
    gfx->fillRoundRect(LCD_W / 2 - 20, y + 6, 40, 3, 1, LIGHT_GRAY);
    text(16, y + 14, "Profiles", 2, WHITE);
    textRight(LCD_W - 16, y + 18, applying ? "applying..." : "tap to apply", 1, applying ? AMBER : LIGHT_GRAY);
    for (int i = 0; i < Profiles::count(); i++) {
        const Profile &p = Profiles::get(i);
        Rect r = sheetRow(i);
        gfx->fillRoundRect(r.x, r.y, r.w, r.h, 8, GRAY);
        text(r.x + 12, r.y + 6, p.name, 2, WHITE);
        text(r.x + 12, r.y + 28, Profiles::summary(p).c_str(), 1, LIGHT_GRAY);
    }
}

static void drawRelaySheet() {
    int y = RELAY_SHEET_Y;
    gfx->fillRect(0, y, LCD_W, RELAY_SHEET_H, PANEL);
    gfx->drawFastHLine(0, y + RELAY_SHEET_H - 1, LCD_W, GRAY);
    gfx->fillRoundRect(LCD_W / 2 - 20, y + RELAY_SHEET_H - 12, 40, 3, 1, LIGHT_GRAY);
    gfx->fillRoundRect(RELAY_RESET.x, RELAY_RESET.y, RELAY_RESET.w, RELAY_RESET.h, 8, GRAY);
    centered(RELAY_RESET, "Reset connection", 2, WHITE);
    text(16, y + 24, "FrameTap relay", 2, WHITE);

    Relay::Status st = Relay::status();
    gfx->fillRoundRect(RELAY_TOGGLE.x, RELAY_TOGGLE.y, RELAY_TOGGLE.w, RELAY_TOGGLE.h, 8, st.enabled ? OK_GREEN : GRAY);
    centered(RELAY_TOGGLE, st.enabled ? "ON" : "OFF", 2, WHITE);

    const char *state;
    uint16_t stateColor = LIGHT_GRAY;
    if (!st.started) state = "Starts once the camera is connected";
    else if (!st.enabled) state = "Off";
    else if (st.tapWifi) state = "Shared with FrameTap", stateColor = OK_GREEN;
    else if (st.tapBle) state = "FrameTap pairing (BLE)", stateColor = AMBER;
    else if (st.advertising) state = "Waiting for FrameTap";
    else state = "Idle";

    int ry = y + 70;
    auto row = [&](const char *label, const String &value, uint16_t color) {
        text(16, ry, label, 1, GRAY);
        text(110, ry - 4, value.c_str(), 2, color);
        ry += 27; // six rows fit above the reset button
    };
    row("NAME", st.started ? st.name : String("-"), WHITE);
    row("STATUS", state, stateColor);
    if (st.claimed) row("BOARD", "sharing FrameTap's session", OK_GREEN);
    if (st.tapWifi) {
        row("PHONE", st.tapIp.toString(), WHITE);
        row("VIDEO", String(st.kbToTap) + " KB/s  " + String(st.videoPkts) + " pkt/s", WHITE);
        row("LOSS", String(st.videoMissing) + " pkt/s", st.videoMissing ? AMBER : OK_GREEN);
    } else {
        text(16, ry + 6, "In FrameTap, connect to the name above.", 1, LIGHT_GRAY);
        text(16, ry + 22, "Swipe up to close.", 1, LIGHT_GRAY);
    }
}

static void draw() {
    gfx->fillScreen(BG);
    drawHeader();
    switch (tab) {
    case TAB_HOME:     drawHome(); break;
    case TAB_SETTINGS: drawSettings(); break;
    case TAB_PROFILES: drawProfiles(); break;
    case TAB_GIMBAL:   drawGimbal(); break;
    default:           drawSetup(); break;
    }
    drawTabBar();
    if (sheetOpen) drawSheet();
    if (relaySheetOpen) drawRelaySheet();
    if (sidePanelOpen) drawSidePanel();
    if (powerOffConfirm) drawPowerConfirm();
    if (countdownEnd) drawCountdown();
    if (millis() < toastUntil) {
        Rect r = {14, 280, 340, 44};
        gfx->fillRoundRect(r.x, r.y, r.w, r.h, 10, 0x3186);
        centered(r, toast.c_str(), 2, WHITE);
    }
    gfx->flush();
}

// ---------- actions ----------

// Connects at start-up and retries while disconnected, unless the user disconnected
static bool userDisconnected = false;
static uint32_t lastConnectTry = 0;

static void connectCamera() {
    userDisconnected = false;
    lastConnectTry = millis();
    cam.connect(method);
}

static void applyProfile(int i) {
    if (!connected()) {
        showToast("Connect first");
        return;
    }
    Profiles::apply(cam, Profiles::get(i));
    applying = true;
}

static void startHold() {
    if (!imuOk) {
        showToast("Motion sensor not found");
        return;
    }
    if (!readTilt(holdPitch, holdRoll)) return;
    Moves::stop(cam); // steering takes over from a playing move
    tiltDP = tiltDR = 0;
    holding = true;
}

static void stopHold() {
    if (!holding) return;
    holding = false;
    cam.gimbalSpeed(0, 0);
    needsRedraw = true;
}

// Difference a - b the short way round, in -180..180. Held near flat, roll sits at
// the +-180 seam, so a plain subtraction jumps by ~360 on a small tilt.
static float angleDiff(float a, float b) {
    return fmodf(a - b + 540.0f, 360.0f) - 180.0f;
}

// Tilt beyond the dead zone -> speed in -1..1. Squared so small tilts give fine,
// slow movement instead of flickering between stop and crawl.
static float tiltToSpeed(float deg) {
    const Sensitivity &s = SENSITIVITIES[sensitivity];
    float mag = fabsf(deg) - s.deadzone;
    if (mag <= 0) return 0;
    float n = min(mag / (s.full - s.deadzone), 1.0f);
    return copysignf(n * n, deg);
}

static void updateHold() {
    if (!holding) return;
    // Sample every loop and low-pass filter to take out hand shake and sensor noise
    float pitch, roll;
    if (readTilt(pitch, roll)) {
        const float a = 0.25f;
        tiltDP += a * (angleDiff(pitch, holdPitch) - tiltDP);
        tiltDR += a * (angleDiff(roll, holdRoll) - tiltDR);
    }
    if (millis() - lastStick < 40) return; // ~25 commands/s
    lastStick = millis();
    cam.gimbalSpeed(YAW_SIGN * tiltToSpeed(tiltDR), PITCH_SIGN * tiltToSpeed(tiltDP));
}

static void finishRecording() {
    if (!Moves::recording()) return;
    stopHold();
    int slot = Moves::stopRecording();
    if (slot >= 0) showToast("Saved as move " + String(slot + 1));
    else showToast("Too short - not saved");
}

// Option picker taps (shared by HOME and SET)
static void tapPicker(int x, int y) {
    SettingId s = (SettingId)pickerSetting;
    for (int i = 0; i < cam.optionCount(s); i++) {
        if (pickerOption(i).contains(x, y)) {
            if (i != cam.value(s)) cam.set(s, i);
            pickerSetting = -1;
            return;
        }
    }
    if (PICKER_BACK.contains(x, y)) pickerSetting = -1;
}

// Shared "not connected" screen: only the Connect button is live
static bool tapNotConnected(int x, int y) {
    if (connected()) return false;
    if (cam.asleep()) {
        if (HOME_WAKE.contains(x, y)) cam.wake();
        if (HOME_SNAPSHOT.contains(x, y)) cam.snapshot();
    } else if (cam.link() == Link::Disconnected && HOME_CONNECT.contains(x, y)) {
        connectCamera();
    }
    return true;
}

static void tapHome(int x, int y) {
    if (tapNotConnected(x, y)) return;
    if (pickerSetting >= 0) {
        tapPicker(x, y);
        return;
    }
    switch (design) {
    case D_TILES: {
        SettingId list[S_COUNT];
        int n = min(visibleSettings(list), 12);
        for (int i = 0; i < n; i++)
            if (tileRect(i).contains(x, y)) pickerSetting = list[i];
        if (TILES_REC.contains(x, y)) pressShutter();
        break;
    }
    case D_DIAL:
        if (DIAL_REC.contains(x, y)) pressShutter();
        break;
    case D_HUD: {
        SettingId list[8];
        int n = hudSettings(list);
        for (int i = 0; i < n; i++)
            if (hudChip(i).contains(x, y)) pickerSetting = list[i];
        if (HUD_REC.contains(x, y)) pressShutter();
        break;
    }
    default:
        if (REC_BUTTON.contains(x, y)) pressShutter();
        break;
    }
}

static void tapSettings(int x, int y) {
    if (tapNotConnected(x, y)) return;
    if (pickerSetting >= 0) {
        tapPicker(x, y);
        return;
    }
    SettingId list[S_COUNT];
    int n = visibleSettings(list);
    int row = (y - CONTENT_Y - 4) / ROW_H;
    if (y >= CONTENT_Y + 4 && row < n) pickerSetting = list[row];
}

static void tapProfiles(int x, int y) {
    if (profileMenu >= 0) {
        int i = profileMenu;
        profileMenu = -1;
        if (POPUP_APPLY.contains(x, y)) {
            applyProfile(i);
        } else if (POPUP_DELETE.contains(x, y)) {
            Profiles::remove(i);
            showToast("Profile deleted");
        }
        return;
    }
    if (applying) return;
    for (int i = 0; i < Profiles::count(); i++) {
        if (profileRow(i).contains(x, y)) {
            profileMenu = i;
            return;
        }
    }
    if (PROFILE_SAVE.contains(x, y)) {
        if (!connected()) showToast("Connect first");
        else if (!Profiles::addFromCamera(cam)) showToast("Max 6 profiles - delete one");
        else showToast("Saved " + String(Profiles::get(Profiles::count() - 1).name));
    }
}

static void tapMoves(int x, int y) {
    if (slotMenu >= 0) {
        int i = slotMenu;
        slotMenu = -1;
        if (POPUP_APPLY.contains(x, y)) {
            stopHold();
            Moves::playSlot(cam, i, videoWithMove);
        } else if (POPUP_DELETE.contains(x, y)) {
            Moves::deleteSlot(i);
            showToast("Move deleted");
        }
        return;
    }
    if (Moves::playing()) {
        if (MOVE_STATUS.contains(x, y)) Moves::stop(cam);
        return;
    }
    if (MOVE_RECORD.contains(x, y)) {
        if (Moves::recording()) {
            finishRecording();
        } else if (!Moves::hasFreeSlot()) {
            showToast("All 4 slots full - delete one");
        } else {
            startHold(); // steer by tilt until Stop, whatever the Hold/Tap mode
            if (holding) Moves::startRecording();
        }
        return;
    }
    if (Moves::recording()) return;
    for (int i = 0; i < Moves::PRESET_COUNT; i++) {
        if (movePreset(i).contains(x, y)) {
            stopHold();
            Moves::playPreset(cam, i, MOVE_SECONDS[moveSeconds] * 1000, videoWithMove);
        }
    }
    for (int i = 0; i < MOVE_SECONDS_COUNT; i++) {
        if (moveSecondsRect(i).contains(x, y)) {
            moveSeconds = i;
            savePrefs();
        }
    }
    for (int i = 0; i < Moves::SLOT_COUNT; i++)
        if (moveSlot(i).contains(x, y) && Moves::slotUsed(i)) slotMenu = i;
    if (MOVE_VIDEO.contains(x, y)) {
        videoWithMove = !videoWithMove;
        savePrefs();
    }
    if (MOVE_FRONT.contains(x, y)) {
        faceFrontFirst = !faceFrontFirst;
        Moves::setFaceFront(faceFrontFirst);
        savePrefs();
    }
}

static void tapGimbal(int x, int y) {
    if (tapNotConnected(x, y)) return;
    if (gimbalPage == 1) {
        tapMoves(x, y);
        return;
    }
    for (int i = 0; i < cam.optionCount(S_GIMBAL_MODE); i++)
        if (gimbalModeButton(i).contains(x, y) && i != cam.value(S_GIMBAL_MODE)) cam.set(S_GIMBAL_MODE, i);
    if (GIMBAL_RECENTER.contains(x, y)) {
        stopHold(); // otherwise leftover tilt drives it straight off center again
        Moves::stop(cam);
        cam.gimbalRecenter();
    }
    for (int i = 0; i < SENS_COUNT; i++) {
        if (gimbalSens(i).contains(x, y)) {
            sensitivity = i;
            savePrefs();
        }
    }
    if (GIMBAL_TILT_MODE.contains(x, y)) {
        stopHold();
        tiltToggle = !tiltToggle;
        savePrefs();
    }
    // Tap mode: the button switches tilt control on and off
    if (tiltToggle && GIMBAL_HOLD.contains(x, y)) {
        if (holding) stopHold();
        else startHold();
    }
}

static void tapSetupConnection(int x, int y) {
    for (int i = 0; i < 2; i++) {
        if (!choice2(116, i).contains(x, y)) continue;
        Transport t = i == 0 ? Transport::Bluetooth : Transport::WiFi;
        if (t == method) return;
        method = t;
        savePrefs();
        // Reconnect over the new method
        if (cam.link() != Link::Disconnected) {
            cam.disconnect();
            connectCamera();
        }
    }
    if (CONN_SLEEP.contains(x, y) && connected()) cam.sleep();
    if (CONN_CONNECT.contains(x, y) && !cam.powerActivity()) {
        if (cam.asleep()) cam.wake();
        else if (cam.link() == Link::Connected) {
            cam.disconnect();
            userDisconnected = true;
        } else if (cam.link() == Link::Disconnected) connectCamera();
    }
}

static void tapSetupDisplay(int x, int y) {
    for (int i = 0; i < D_COUNT; i++)
        if (choice4(114, i).contains(x, y)) design = (Design)i;
    for (int i = 0; i < SLEEP_COUNT; i++)
        if (choice4(192, i).contains(x, y)) sleepChoice = i;
    for (int i = 0; i < 3; i++) {
        if (choice3(270, i).contains(x, y)) {
            brightnessLevel = i;
            panel->setBrightness(BRIGHTNESS_LEVELS[i]);
        }
    }
    savePrefs();
}

static void tapSetupSound(int x, int y) {
    if (SOUND_TOGGLE.contains(x, y)) soundOn = !soundOn;
    for (int i = 0; i < 3; i++) {
        if (choice3(198, i).contains(x, y)) {
            volumeLevel = i;
            soundVolume(VOLUME_LEVELS[i]);
        }
    }
    savePrefs();
    // Let the tap click play first, then the test click
    if (SOUND_TEST.contains(x, y) || choice3(198, volumeLevel).contains(x, y)) {
        delay(120);
        soundClick();
    }
}

static void tapSetupGimbal(int x, int y) {
    for (int i = 0; i < 2; i++) {
        if (choice2(116, i).contains(x, y)) {
            stopHold();
            tiltToggle = i == 1;
        }
    }
    for (int i = 0; i < SENS_COUNT; i++)
        if (choice3(198, i).contains(x, y)) sensitivity = i;
    savePrefs();
}

static void tapSetup(int x, int y) {
    if (setupPage == SP_MENU) {
        for (int p = SP_CONNECTION; p < SP_COUNT; p++)
            if (setupRow(p - 1).contains(x, y)) setupPage = (SetupPage)p;
        return;
    }
    if (SETUP_BACK.contains(x, y) && !powerOffConfirm) {
        setupPage = SP_MENU;
        return;
    }
    switch (setupPage) {
    case SP_CONNECTION: tapSetupConnection(x, y); break;
    case SP_DISPLAY:    tapSetupDisplay(x, y); break;
    case SP_SOUND:      tapSetupSound(x, y); break;
    case SP_GIMBAL:     tapSetupGimbal(x, y); break;
    default:
        if (ABOUT_TOUCH_TEST.contains(x, y)) {
            touchTest = true;
            touchPointCount = 0;
        } else if (ABOUT_POWER_OFF.contains(x, y)) {
            powerOffConfirm = true;
        }
        break;
    }
}

// ---------- screen sleep ----------

// Never sleep mid-action: steering, moves, video, profile apply or connecting
static bool busy() {
    return holding || Moves::playing() || Moves::recording() || cam.recording() || applying ||
           cam.link() == Link::Connecting || touchTest || cam.powerActivity() || countdownEnd;
}

static void goToSleep() {
    asleep = true;
    panel->setBrightness(0);
    panel->displayOff();
    soundEnable(false);
    Serial.println("screen sleep");
}

static void wake() {
    asleep = false;
    panel->displayOn();
    panel->setBrightness(BRIGHTNESS_LEVELS[brightnessLevel]);
    soundEnable(true);
    lastActivity = millis();
    needsRedraw = true;
    swallowTouch = true;
    Serial.println("screen wake");
}

// While asleep: BOOT, or two taps within DOUBLE_TAP_MS, wakes the screen.
// Returns true while still asleep (the caller skips UI work).
static bool updateSleep() {
    static bool bootWasDown = false;
    static bool sleepTouchDown = false;
    static uint32_t lastSleepTap = 0;
    bool bootDown = digitalRead(BOOT_BUTTON) == LOW;
    bool bootPressed = bootDown && !bootWasDown;
    bootWasDown = bootDown;

    if (asleep) {
        int x, y;
        int touch = readTouch(x, y);
        if (touch >= 0) {
            bool down = touch == 1;
            if (down && !sleepTouchDown) {
                if (millis() - lastSleepTap < DOUBLE_TAP_MS) {
                    lastSleepTap = 0;
                    wake();
                } else {
                    lastSleepTap = millis();
                }
            }
            sleepTouchDown = down;
        }
        if (bootPressed) wake();
        return asleep;
    }
    // BOOT toggles: pressed while awake turns the screen off right away. Only the
    // screen sleeps, so recording, steering or moves carry on.
    if (bootPressed) {
        goToSleep();
        return asleep;
    }
    if (busy()) lastActivity = millis();
    int secs = SLEEP_SECONDS[sleepChoice];
    if (secs && millis() - lastActivity > (uint32_t)secs * 1000) goToSleep();
    return asleep;
}

static void tapSheet(int x, int y) {
    if (y < sheetTop()) {
        sheetOpen = false; // tap above the sheet closes it
        return;
    }
    if (applying) return;
    for (int i = 0; i < Profiles::count(); i++) {
        if (sheetRow(i).contains(x, y)) {
            applyProfile(i);
            sheetOpen = false;
            return;
        }
    }
}

static void onTap(int x, int y) {
    if (countdownEnd) {
        countdownEnd = 0; // any tap cancels the self-timer
        showToast("Self-timer cancelled");
        return;
    }
    if (powerOffConfirm) {
        powerOffConfirm = false;
        if (POPUP_APPLY.contains(x, y)) shutDown();
        return;
    }
    if (sidePanelOpen) {
        for (int i = 0; i < TIMER_COUNT; i++) {
            if (sideTimer(i).contains(x, y)) {
                selfTimer = i; // choosing a timer keeps the panel open
                savePrefs();
                return;
            }
        }
        sidePanelOpen = false; // any other tap closes it; outside the panel means cancel
        if (SIDE_SCREEN_OFF.contains(x, y)) goToSleep();
        if (SIDE_POWER_OFF.contains(x, y)) powerOffConfirm = true;
        return;
    }
    if (sheetOpen) {
        tapSheet(x, y);
        return;
    }
    if (relaySheetOpen) {
        if (y >= RELAY_SHEET_Y + RELAY_SHEET_H) relaySheetOpen = false; // tap below the sheet closes it
        else if (RELAY_TOGGLE.contains(x, y)) Relay::setEnabled(!Relay::status().enabled);
        else if (RELAY_RESET.contains(x, y)) {
            // Drop FrameTap and the camera, then connect again from scratch
            bool relayOn = Relay::status().enabled;
            Relay::setEnabled(false);
            if (cam.link() != Link::Disconnected) cam.disconnect();
            Relay::setEnabled(relayOn);
            connectCamera();
            relaySheetOpen = false;
            showToast("Reconnecting...");
        }
        return;
    }
    for (int i = 0; i < TAB_COUNT; i++) {
        if (tabRect(i).contains(x, y)) {
            if (i == TAB_SETUP) setupPage = SP_MENU; // SETUP tab always opens on its menu
            powerOffConfirm = false;
            if (i != TAB_GIMBAL) {
                finishRecording();
                stopHold(); // tap-mode tilt shouldn't keep running off-screen
            }
            tab = (Tab)i;
            pickerSetting = -1;
            profileMenu = -1;
            slotMenu = -1;
            return;
        }
    }
    switch (tab) {
    case TAB_HOME:     tapHome(x, y); break;
    case TAB_SETTINGS: tapSettings(x, y); break;
    case TAB_PROFILES: tapProfiles(x, y); break;
    case TAB_GIMBAL:   tapGimbal(x, y); break;
    default:           tapSetup(x, y); break;
    }
}

// Dial design: horizontal swipe changes the value, vertical picks the setting
static void dialSwipe(int dx, int dy) {
    SettingId list[S_COUNT];
    int n = visibleSettings(list);
    if (abs(dx) > abs(dy)) {
        SettingId s = list[constrain(dialIndex, 0, n - 1)];
        int v = cam.value(s) + (dx < 0 ? 1 : -1); // swipe left = next value
        if (v >= 0 && v < cam.optionCount(s)) cam.set(s, v);
    } else {
        dialIndex = constrain(dialIndex + (dy < 0 ? 1 : -1), 0, n - 1); // swipe up = next setting
    }
}

static void onSwipe(int startX, int startY, int dx, int dy) {
    bool vertical = abs(dy) > abs(dx);
    if (sidePanelOpen) {
        if (!vertical && dx < 0) sidePanelOpen = false;
        return;
    }
    if (!sheetOpen && !vertical && dx > 0 && startX < EDGE_SWIPE_ZONE) {
        sidePanelOpen = true; // swiped in from the left edge
        pickerSetting = -1;
        profileMenu = -1;
        slotMenu = -1;
        return;
    }
    if (sheetOpen) {
        if (vertical && dy > 0) sheetOpen = false;
        return;
    }
    if (relaySheetOpen) {
        if (vertical && dy < 0) relaySheetOpen = false;
        return;
    }
    if (vertical && dy < 0 && startY > LCD_H - TABBAR_H - 20) {
        sheetOpen = true; // pushed up from the tab bar
        pickerSetting = -1;
        profileMenu = -1;
        return;
    }
    if (vertical && dy > 0 && startY < HEADER_H + 20) {
        relaySheetOpen = true; // pulled down from the header
        pickerSetting = -1;
        profileMenu = -1;
        slotMenu = -1;
        return;
    }
    // GIMB: swipe left for the Moves page, right to go back
    if (tab == TAB_GIMBAL && !vertical && !Moves::recording()) {
        int page = dx < 0 ? 1 : 0;
        if (page != gimbalPage) {
            if (!tiltToggle) stopHold();
            gimbalPage = page;
            slotMenu = -1;
        }
        return;
    }
    if (tab == TAB_HOME && design == D_DIAL && connected() && pickerSetting < 0) dialSwipe(dx, dy);
}

// Returns true if the press was consumed immediately (hold-to-tilt)
static bool onPressStart(int x, int y) {
    if (!tiltToggle && !sheetOpen && !relaySheetOpen && tab == TAB_GIMBAL && gimbalPage == 0 && connected() &&
        GIMBAL_HOLD.contains(x, y)) {
        startHold();
        needsRedraw = true;
        return true;
    }
    return false;
}

// ---------- main ----------

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Pocket 4P remote (simulated camera) ===");

    Wire.begin(I2C_SDA, I2C_SCL);
    resetPanel();
    imuOk = imuBegin();
    if (!gfx->begin()) {
        Serial.println("Display init failed");
        return;
    }
    loadPrefs();
    panel->setBrightness(BRIGHTNESS_LEVELS[brightnessLevel]);
    pinMode(BOOT_BUTTON, INPUT_PULLUP);
    lastActivity = millis();
    if (powerBegin()) boardBattery = powerRead();
    Serial.printf("Sound %s\n", soundBegin() ? "ok" : "failed");
    soundVolume(VOLUME_LEVELS[volumeLevel]);
    Serial.printf("Board battery: present=%d charging=%d percent=%d\n", boardBattery.present,
                  boardBattery.charging, boardBattery.percent);

    Profiles::begin();
    Moves::begin();
    Serial.printf("Method %s, design %s, %d profile(s), IMU %s\n", methodName(method),
                  DESIGN_NAMES[design], Profiles::count(), imuOk ? "ok" : "missing");
    draw();
    connectCamera();
}

void loop() {
    static bool touchDown = false, pressConsumed = false, releasePending = false;
    static int startX, startY, lastX, lastY;
    static uint32_t upSince = 0;
    static uint32_t lastDraw = 0;

    if (millis() - lastBatteryRead > 5000) {
        lastBatteryRead = millis();
        boardBattery = powerRead();
    }

    // Asleep: keep the camera ticking, ignore the touchscreen, wait for BOOT
    if (updateSleep()) {
        touchDown = false;
        releasePending = false;
        cam.update();
        cam.takeMessage();
        delay(30);
        return;
    }

    // Taps fire on release so swipes can be told apart; hold-to-tilt fires on press
    int x, y;
    int touch = readTouch(x, y);
    if (touch == 1) lastActivity = millis();
    if (swallowTouch) {
        if (touch == 0) swallowTouch = false;
        touch = -1;
    }

    // Touch test: mark each new touch; BOOT exits
    if (touchTest) {
        static bool testDown = false, bootWasDown = true;
        bool bootDown = digitalRead(BOOT_BUTTON) == LOW;
        if (bootDown && !bootWasDown) {
            touchTest = false;
            setupPage = SP_ABOUT;
            needsRedraw = true;
        }
        bootWasDown = bootDown;
        if (touch == 1 && !testDown) {
            if (soundOn) soundClick();
            if (touchPointCount == TOUCH_TEST_POINTS) {
                memmove(touchPoints, touchPoints[1], sizeof(touchPoints[0]) * (TOUCH_TEST_POINTS - 1));
                touchPointCount--;
            }
            touchPoints[touchPointCount][0] = x;
            touchPoints[touchPointCount][1] = y;
            touchPointCount++;
            Serial.printf("touch test x=%d y=%d\n", x, y);
        }
        if (touch >= 0) testDown = touch == 1;
        if (touchTest) drawTouchTest();
        touchDown = false;
        delay(20);
        return;
    }

    // A lift only counts once the finger has stayed up for TOUCH_DEBOUNCE_MS;
    // a touch-down before then is contact bounce and continues the same touch.
    if (touch >= 0) {
        bool down = touch == 1;
        if (down && !touchDown) {
            if (releasePending) {
                releasePending = false;
            } else {
                if (soundOn) soundClick();
                startX = lastX = x;
                startY = lastY = y;
                pressConsumed = onPressStart(x, y);
            }
        } else if (down) {
            lastX = x;
            lastY = y;
        } else if (touchDown) {
            releasePending = true;
            upSince = millis();
        }
        touchDown = down;
    }
    if (releasePending && millis() - upSince >= TOUCH_DEBOUNCE_MS) {
        releasePending = false;
        if (!tiltToggle && !Moves::recording()) stopHold();
        if (!pressConsumed) {
            int dx = lastX - startX, dy = lastY - startY;
            if (abs(dx) < SWIPE_MIN && abs(dy) < SWIPE_MIN) onTap(startX, startY);
            else onSwipe(startX, startY, dx, dy);
        }
        needsRedraw = true;
    }

    cam.update();
    if (const char *msg = cam.takeMessage()) showToast(msg);
    if (!userDisconnected && cam.link() == Link::Disconnected && !cam.asleep() && millis() - lastConnectTry > 10000)
        connectCamera();

    if (!connected()) {
        finishRecording();
        stopHold();
    }
    updateHold();
    updateCountdown();
    Moves::update(cam);
    if (Moves::recording() && Moves::recordingFull()) finishRecording();

    if (applying) {
        if (!connected()) {
            applying = false;
        } else if (cam.queued() == 0) {
            applying = false;
            showToast("Profile applied");
        }
    }

    // Periodic redraw picks up timers and simulated camera changes; faster while steering
    uint32_t interval = holding || Moves::playing() || countdownEnd ? 33 : 250;
    if (needsRedraw || millis() - lastDraw > interval) {
        draw();
        lastDraw = millis();
        needsRedraw = false;
    }
    delay(10);
}
