// Atomberg fan remote: lists fans from the Atomberg cloud API with
// power toggle, -/+ speed, and a tap-the-number speed picker.
//
// The UI (touch + drawing) runs in loop(); all HTTP runs in a separate
// network task so taps never wait on the cloud. Shared fan state is
// guarded by fanLock.
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Arduino_GFX_Library.h>
#include "secrets.h"

static const int I2C_SDA = 15;
static const int I2C_SCL = 14;

// SH8601 AMOLED on QSPI. GFX has no SH8601 class; CO5300 shares its command set.
static const int LCD_CS = 12, LCD_SCK = 11;
static const int LCD_D0 = 4, LCD_D1 = 5, LCD_D2 = 6, LCD_D3 = 7;
static const int LCD_W = 368, LCD_H = 448;

// TCA9554 expander drives the LCD/touch reset lines (pins 0-2)
static const uint8_t TCA9554_ADDR = 0x20;
static const uint8_t FT3168_ADDR = 0x38;

static const char *API_BASE = "https://api.developer.atomberg-iot.com";
// The API key has a daily request limit, so keep requests to a minimum
static const uint32_t POLL_MS = 10 * 60 * 1000;   // background state refresh
static const uint32_t CONFIRM_MS = 10000;         // one refresh this long after a command
static const uint32_t LIMIT_BACKOFF_MS = 60 * 60 * 1000; // pause polling after HTTP 429
static const uint32_t SETTLE_MS = 400;     // wait this long after the last tap before sending
static const uint32_t HOLD_LOCAL_MS = 15000; // ignore (lagging) cloud state this long after a change

static const uint16_t GRAY = 0x4208;
static const uint16_t DARK = 0x2945;
static const uint16_t LIGHT_GRAY = 0xA514;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
Arduino_CO5300 *gfx = new Arduino_CO5300(bus, GFX_NOT_DEFINED, 0, false, LCD_W, LCD_H);

enum RowStatus { ST_NONE, ST_SENDING, ST_FAILED, ST_OFFLINE, ST_LIMIT };

struct Fan {
    String id, name, room;
    bool power = false;
    int speed = 1;
    bool online = false;

    bool powerDirty = false;  // power/speed changed locally, not yet sent
    bool speedDirty = false;
    uint32_t lastInput = 0;   // last tap on this fan
    RowStatus status = ST_NONE;
    bool redraw = false;      // set by the network task, consumed by the UI
};

static const int MAX_FANS = 6;
static Fan fans[MAX_FANS];
static int fanCount = 0;
static SemaphoreHandle_t fanLock;
static volatile bool fullRedraw = false; // fan list changed; set by the network task

static int pickerFan = -1; // fan whose speed picker is open, or -1 (UI only)

// ---------- UI geometry ----------

struct Rect {
    int x, y, w, h;
    bool contains(int px, int py) const { return px >= x && px < x + w && py >= y && py < y + h; }
};

static const int HEADER_H = 44;
enum Control { CTL_POWER, CTL_MINUS, CTL_SPEED, CTL_PLUS };

static int rowHeight() { return (LCD_H - HEADER_H) / max(fanCount, 1); }

static Rect controlRect(int fan, Control c) {
    int y = HEADER_H + fan * rowHeight() + 34;
    int h = rowHeight() - 44;
    switch (c) {
    case CTL_POWER: return {10, y, 92, h};
    case CTL_MINUS: return {114, y, 66, h};
    case CTL_SPEED: return {190, y, 90, h};
    default:        return {290, y, 68, h};
    }
}

static const Rect PICKER_BOX = {24, 100, 320, 250};

static Rect pickerButton(int speed) {
    int i = speed - 1;
    return {40 + (i % 3) * 100, 170 + (i / 3) * 82, 88, 70};
}

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

// ---------- drawing (UI task only) ----------

static void drawCentered(const Rect &r, const char *text, int size, uint16_t color) {
    int tw = strlen(text) * 6 * size;
    int th = 8 * size;
    gfx->setTextSize(size);
    gfx->setTextColor(color);
    gfx->setCursor(r.x + (r.w - tw) / 2, r.y + (r.h - th) / 2);
    gfx->print(text);
}

static void drawHeader(const char *status) {
    gfx->fillRect(0, 0, LCD_W, HEADER_H, DARK);
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);
    gfx->setCursor(12, 14);
    gfx->print("Atomberg Fans");
    gfx->setTextColor(YELLOW);
    gfx->setCursor(LCD_W - 12 - strlen(status) * 12, 14);
    gfx->print(status);
}

// Caller holds fanLock
static void drawFan(int i) {
    Fan &f = fans[i];
    f.redraw = false;
    int y0 = HEADER_H + i * rowHeight();
    gfx->fillRect(0, y0, LCD_W, rowHeight(), BLACK);
    gfx->drawFastHLine(0, y0 + rowHeight() - 1, LCD_W, GRAY);

    gfx->setTextSize(2);
    gfx->setCursor(12, y0 + 10);
    gfx->setTextColor(WHITE);
    gfx->print(f.name);
    gfx->setTextColor(LIGHT_GRAY);
    gfx->print("  ");
    gfx->print(f.room);

    const char *tag = nullptr;
    uint16_t tagColor = RED;
    if (f.status == ST_SENDING || f.powerDirty || f.speedDirty) {
        tag = "...";
        tagColor = YELLOW;
    } else if (f.status == ST_FAILED) {
        tag = "failed";
    } else if (f.status == ST_LIMIT) {
        tag = "limit";
    } else if (f.status == ST_OFFLINE || !f.online) {
        tag = "offline";
    }
    if (tag) {
        gfx->setTextColor(tagColor);
        gfx->setCursor(LCD_W - 12 - strlen(tag) * 12, y0 + 10);
        gfx->print(tag);
    }

    Rect p = controlRect(i, CTL_POWER);
    gfx->fillRoundRect(p.x, p.y, p.w, p.h, 10, f.power ? GREEN : GRAY);
    drawCentered(p, f.power ? "ON" : "OFF", 3, f.power ? BLACK : WHITE);

    Rect m = controlRect(i, CTL_MINUS);
    gfx->fillRoundRect(m.x, m.y, m.w, m.h, 10, DARK);
    drawCentered(m, "-", 4, WHITE);

    Rect s = controlRect(i, CTL_SPEED);
    gfx->drawRoundRect(s.x, s.y, s.w, s.h, 10, WHITE);
    char num[4];
    snprintf(num, sizeof(num), "%d", f.speed);
    drawCentered(s, num, 4, f.power ? CYAN : GRAY);

    Rect pl = controlRect(i, CTL_PLUS);
    gfx->fillRoundRect(pl.x, pl.y, pl.w, pl.h, 10, DARK);
    drawCentered(pl, "+", 4, WHITE);
}

// Caller holds fanLock
static void drawAllFans() {
    gfx->fillRect(0, HEADER_H, LCD_W, LCD_H - HEADER_H, BLACK);
    if (fanCount == 0) {
        drawCentered({0, HEADER_H, LCD_W, 200}, "No fans found", 2, WHITE);
        return;
    }
    for (int i = 0; i < fanCount; i++) drawFan(i);
}

// Caller holds fanLock
static void drawPicker() {
    const Fan &f = fans[pickerFan];
    const Rect &b = PICKER_BOX;
    gfx->fillRoundRect(b.x, b.y, b.w, b.h, 16, DARK);
    gfx->drawRoundRect(b.x, b.y, b.w, b.h, 16, WHITE);
    drawCentered({b.x, b.y + 12, b.w, 40}, (f.name + " speed").c_str(), 2, WHITE);
    for (int s = 1; s <= 6; s++) {
        Rect r = pickerButton(s);
        bool selected = s == f.speed;
        gfx->fillRoundRect(r.x, r.y, r.w, r.h, 10, selected ? CYAN : GRAY);
        char num[2] = {(char)('0' + s), 0};
        drawCentered(r, num, 4, selected ? BLACK : WHITE);
    }
}

// ---------- Atomberg API (network task only, after setup) ----------

// One TLS connection, kept alive and reused across requests
static WiFiClientSecure tls;
static HTTPClient http;
static String accessToken;
static uint32_t limitedAt = 0; // millis() of the last HTTP 429, 0 if none

static bool rateLimited() { return limitedAt && millis() - limitedAt < LIMIT_BACKOFF_MS; }

static int httpCall(const char *method, const String &path, const String &bearer,
                    const String &body, String &resp) {
    http.begin(tls, String(API_BASE) + path);
    http.addHeader("x-api-key", ATOMBERG_API_KEY);
    http.addHeader("Authorization", "Bearer " + bearer);
    int code;
    if (strcmp(method, "POST") == 0) {
        http.addHeader("Content-Type", "application/json");
        code = http.POST(body);
    } else {
        code = http.GET();
    }
    resp = code > 0 ? http.getString() : "";
    http.end();
    if (code == 429) limitedAt = max(millis(), (uint32_t)1);
    return code;
}

// Returns the HTTP status of the token request
static int refreshAccessToken() {
    String resp;
    int code = httpCall("GET", "/v1/get_access_token", ATOMBERG_REFRESH_TOKEN, "", resp);
    if (code != 200) {
        Serial.printf("token refresh failed: %d %s\n", code, resp.c_str());
        return code;
    }
    JsonDocument doc;
    if (deserializeJson(doc, resp)) return -1;
    accessToken = doc["message"]["access_token"].as<String>();
    return accessToken.isEmpty() ? -1 : 200;
}

// Authenticated call; gets a new access token and retries once if it expired.
// Returns the HTTP status (or a negative error). Never retries a 429.
static int api(const char *method, const String &path, const String &body, JsonDocument &doc) {
    int code = -1;
    for (int attempt = 0; attempt < 2; attempt++) {
        if (accessToken.isEmpty()) {
            int tokenCode = refreshAccessToken();
            if (tokenCode != 200) return tokenCode;
        }
        String resp;
        code = httpCall(method, path, accessToken, body, resp);
        if (code == 401 || code == 403) {
            accessToken = "";
            continue;
        }
        if (code != 200) Serial.printf("%s %s -> %d %s\n", method, path.c_str(), code, resp.c_str());
        else if (deserializeJson(doc, resp)) return -1;
        return code;
    }
    return code;
}

// ---------- fan list cache (NVS), so boot works without the cloud ----------

static void saveFanList() {
    Preferences prefs;
    prefs.begin("fans", false);
    prefs.clear();
    prefs.putInt("count", fanCount);
    for (int i = 0; i < fanCount; i++) {
        prefs.putString(("id" + String(i)).c_str(), fans[i].id);
        prefs.putString(("name" + String(i)).c_str(), fans[i].name);
        prefs.putString(("room" + String(i)).c_str(), fans[i].room);
    }
    prefs.end();
}

static void loadFanList() {
    Preferences prefs;
    prefs.begin("fans", true);
    fanCount = min((int)prefs.getInt("count", 0), MAX_FANS);
    for (int i = 0; i < fanCount; i++) {
        fans[i].id = prefs.getString(("id" + String(i)).c_str(), "");
        fans[i].name = prefs.getString(("name" + String(i)).c_str(), "");
        fans[i].room = prefs.getString(("room" + String(i)).c_str(), "");
    }
    prefs.end();
    Serial.printf("%d fan(s) from cache\n", fanCount);
}

// Fetches the fan list from the cloud and caches it. Returns the HTTP status.
static int loadDevices() {
    JsonDocument doc;
    int code = api("GET", "/v1/get_list_of_devices", "", doc);
    if (code != 200) return code;
    JsonArray list = doc["message"]["devices_list"].as<JsonArray>();

    xSemaphoreTake(fanLock, portMAX_DELAY);
    int count = 0;
    for (JsonObject d : list) {
        if (count == MAX_FANS) break;
        Fan &f = fans[count++];
        f.id = d["device_id"].as<String>();
        f.name = d["name"].as<String>();
        f.room = d["room"].as<String>();
    }
    bool changed = count != fanCount;
    fanCount = count;
    saveFanList();
    xSemaphoreGive(fanLock);
    if (changed) fullRedraw = true;
    Serial.printf("%d fan(s) loaded\n", fanCount);
    return 200;
}

// Marks every row with the outcome of a failed whole-list request
static void markAll(RowStatus status) {
    xSemaphoreTake(fanLock, portMAX_DELAY);
    for (int i = 0; i < fanCount; i++) {
        fans[i].status = status;
        fans[i].redraw = true;
    }
    xSemaphoreGive(fanLock);
}

static bool refreshState() {
    JsonDocument doc;
    int code = api("GET", "/v1/get_device_state?device_id=all", "", doc);
    if (code == 429) markAll(ST_LIMIT);
    if (code != 200) return false;
    xSemaphoreTake(fanLock, portMAX_DELAY);
    for (JsonObject s : doc["message"]["device_state"].as<JsonArray>()) {
        String id = s["device_id"].as<String>();
        for (int i = 0; i < fanCount; i++) {
            Fan &f = fans[i];
            if (f.id != id) continue;
            bool online = s["is_online"] | false;
            if (online != f.online) f.redraw = true;
            f.online = online;
            // Cloud state lags behind commands; keep local values for a recent change
            if (f.powerDirty || f.speedDirty || millis() - f.lastInput < HOLD_LOCAL_MS) continue;
            bool power = s["power"] | false;
            int speed = s["last_recorded_speed"] | 1;
            if (power != f.power || speed != f.speed) f.redraw = true;
            f.power = power;
            f.speed = speed;
            if (f.status == ST_LIMIT || (f.status == ST_OFFLINE && online)) {
                f.status = ST_NONE;
                f.redraw = true;
            }
        }
    }
    xSemaphoreGive(fanLock);
    return true;
}

template <typename T>
static int sendCommand(const String &id, const char *key, T value) {
    JsonDocument req;
    req["device_id"] = id;
    req["command"][key] = value;
    String body;
    serializeJson(req, body);
    Serial.printf("send %s\n", body.c_str());
    JsonDocument doc;
    return api("POST", "/v1/send_command", body, doc);
}

static RowStatus statusFor(int code) {
    if (code == 200) return ST_NONE;
    if (code == 503) return ST_OFFLINE; // "Device is offline"
    if (code == 429) return ST_LIMIT;   // daily request limit
    return ST_FAILED;
}

static uint32_t confirmAt = 0; // millis() when a post-command refresh is due, 0 if none

// Sends any settled local changes for one fan
static void flushFan(int i) {
    xSemaphoreTake(fanLock, portMAX_DELAY);
    Fan &f = fans[i];
    bool sendPower = f.powerDirty, sendSpeed = f.speedDirty;
    if ((!sendPower && !sendSpeed) || millis() - f.lastInput < SETTLE_MS) {
        xSemaphoreGive(fanLock);
        return;
    }
    String id = f.id;
    bool power = f.power;
    int speed = f.speed;
    f.powerDirty = f.speedDirty = false;
    f.status = ST_SENDING;
    f.redraw = true;
    xSemaphoreGive(fanLock);

    int code = 200;
    if (sendPower) code = sendCommand(id, "power", power);
    if (code == 200 && sendSpeed) code = sendCommand(id, "speed", speed);

    xSemaphoreTake(fanLock, portMAX_DELAY);
    f.status = statusFor(code);
    if (code == 200) f.online = true;
    f.lastInput = millis(); // hold the local value while the cloud catches up
    f.redraw = true;
    xSemaphoreGive(fanLock);
    if (code == 200) confirmAt = max(millis() + CONFIRM_MS, (uint32_t)1);
}

static void networkTask(void *arg) {
    bool listLoaded = arg != nullptr;
    uint32_t lastPoll = millis();
    for (;;) {
        for (int i = 0; i < fanCount; i++) flushFan(i);

        bool pollDue = millis() - lastPoll > POLL_MS;
        bool confirmDue = confirmAt && (int32_t)(millis() - confirmAt) >= 0;
        if ((pollDue || confirmDue) && !rateLimited()) {
            lastPoll = millis();
            confirmAt = 0;
            if (!listLoaded) listLoaded = loadDevices() == 200;
            if (listLoaded) refreshState();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ---------- touch actions (UI task) ----------

// Records a local change; the network task sends it once taps settle
static void markChanged(int i, bool powerChanged, bool speedChanged) {
    Fan &f = fans[i];
    f.powerDirty |= powerChanged;
    f.speedDirty |= speedChanged;
    f.lastInput = millis();
}

static void handleTap(int x, int y) {
    xSemaphoreTake(fanLock, portMAX_DELAY);
    if (pickerFan >= 0) {
        Fan &f = fans[pickerFan];
        for (int s = 1; s <= 6; s++) {
            if (pickerButton(s).contains(x, y) && s != f.speed) {
                f.speed = s;
                markChanged(pickerFan, false, true);
            }
        }
        // Any tap closes the picker; outside the box means cancel
        pickerFan = -1;
        drawAllFans();
        xSemaphoreGive(fanLock);
        return;
    }

    for (int i = 0; i < fanCount; i++) {
        Fan &f = fans[i];
        if (controlRect(i, CTL_POWER).contains(x, y)) {
            f.power = !f.power;
            markChanged(i, true, false);
        } else if (controlRect(i, CTL_MINUS).contains(x, y)) {
            if (f.speed > 1) {
                f.speed--;
                markChanged(i, false, true);
            }
        } else if (controlRect(i, CTL_PLUS).contains(x, y)) {
            if (f.speed < 6) {
                f.speed++;
                markChanged(i, false, true);
            }
        } else if (controlRect(i, CTL_SPEED).contains(x, y)) {
            pickerFan = i;
            drawPicker();
            break;
        } else {
            continue;
        }
        drawFan(i);
        break;
    }
    xSemaphoreGive(fanLock);
}

// ---------- main ----------

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Atomberg remote ===");
    fanLock = xSemaphoreCreateMutex();

    Wire.begin(I2C_SDA, I2C_SCL);
    resetPanel();
    if (!gfx->begin()) {
        Serial.println("Display init failed");
        return;
    }
    gfx->setBrightness(200);
    gfx->fillScreen(BLACK);
    drawHeader("Wi-Fi...");

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) delay(200);
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Wi-Fi connect failed");
        drawHeader("no Wi-Fi");
        drawCentered({0, HEADER_H, LCD_W, 200}, "Check secrets.h", 2, WHITE);
        return;
    }
    Serial.printf("Wi-Fi OK, IP %s\n", WiFi.localIP().toString().c_str());

    tls.setInsecure(); // no CA pinning; fine for a home gadget
    http.setReuse(true);
    http.setTimeout(8000);

    drawHeader("loading");
    loadFanList();
    int code = loadDevices();
    bool listLoaded = code == 200;
    if (listLoaded) refreshState();
    else markAll(code == 429 ? ST_LIMIT : ST_FAILED);
    drawHeader(code == 429 ? "limit" : code == 200 ? "" : "error");
    xSemaphoreTake(fanLock, portMAX_DELAY);
    drawAllFans();
    xSemaphoreGive(fanLock);

    // HTTP on core 0 (with the Wi-Fi stack); UI stays on core 1
    xTaskCreatePinnedToCore(networkTask, "net", 16384, (void *)listLoaded, 1, nullptr, 0);
}

void loop() {
    static bool wasDown = false;
    int x, y;
    int touch = readTouch(x, y);
    if (touch >= 0) {
        bool down = touch == 1;
        if (down && !wasDown) handleTap(x, y);
        wasDown = down;
    }

    // Repaint rows the network task changed (not while the picker covers them)
    if (pickerFan < 0) {
        xSemaphoreTake(fanLock, portMAX_DELAY);
        if (fullRedraw) {
            fullRedraw = false;
            drawHeader("");
            drawAllFans();
        }
        for (int i = 0; i < fanCount; i++)
            if (fans[i].redraw) drawFan(i);
        xSemaphoreGive(fanLock);
    }
    delay(20);
}
