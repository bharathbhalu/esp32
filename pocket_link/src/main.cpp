// Phase 1 test: connect to a DJI Osmo Pocket 4P over Bluetooth, pair (approve
// "OSMO" on the camera), and read the camera's Wi-Fi name and password.
// Every step is shown on screen and in the serial log.
//
// Connection sequence follows OpenCine's DjiAppleConnectionRun.swift and the
// OpenPocketCine handbook (Apache License 2.0).
#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include <Arduino_GFX_Library.h>
#include "duml.h"
#include "datalink.h"

static const int I2C_SDA = 15;
static const int I2C_SCL = 14;

// SH8601 AMOLED on QSPI. GFX has no SH8601 class; CO5300 shares its command set.
static const int LCD_CS = 12, LCD_SCK = 11;
static const int LCD_D0 = 4, LCD_D1 = 5, LCD_D2 = 6, LCD_D3 = 7;
static const int LCD_W = 368, LCD_H = 448;
static const uint8_t TCA9554_ADDR = 0x20;

static const uint16_t DJI_COMPANY_ID = 0x08AA;
static const uint32_t WRITE_GAP_MS = 250; // fff5 writes drop if not paced

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
Arduino_CO5300 *panel = new Arduino_CO5300(bus, GFX_NOT_DEFINED, 0, false, LCD_W, LCD_H);
Arduino_Canvas *gfx = new Arduino_Canvas(LCD_W, LCD_H, panel);

// ---------- screen log ----------

static Datalink camLink;
static String camSsid, camPass;

// Second-remote test: status from the camera's Bluetooth pushes, REC/STOP buttons
static const uint8_t FT3168_ADDR = 0x38;
static bool paired = false;
static int btBattery = -1;
static bool btCharging = false;
static uint16_t cmdSeq = 0x9000;
struct Btn {
    int x, y, w, h;
    bool hit(int px, int py) const { return px >= x && px < x + w && py >= y && py < y + h; }
};
static const Btn REC_BTN = {16, 372, 160, 62}, STOP_BTN = {192, 372, 160, 62};
static String status = "Starting...";
static uint16_t statusColor = WHITE;
static std::vector<String> logLines;

static void draw() {
    gfx->fillScreen(BLACK);
    gfx->setTextSize(2);
    gfx->setTextColor(0x05FF);
    gfx->setCursor(12, 12);
    gfx->print("Pocket 4P link test");
    gfx->setTextColor(statusColor);
    gfx->setTextSize(status.length() * 12 <= LCD_W - 24 ? 2 : 1);
    gfx->setCursor(12, 44);
    gfx->print(status);
    gfx->setTextSize(1);
    gfx->setTextColor(0xA514);
    int y = 80;
    if (paired) {
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->setCursor(12, 78);
        if (btBattery < 0) gfx->print("Battery: --");
        else gfx->printf("Battery: %d%%%s", btBattery, btCharging ? " charging" : "");
        gfx->fillRoundRect(REC_BTN.x, REC_BTN.y, REC_BTN.w, REC_BTN.h, 14, 0xF800);
        gfx->fillRoundRect(STOP_BTN.x, STOP_BTN.y, STOP_BTN.w, STOP_BTN.h, 14, 0x4208);
        gfx->setTextSize(3);
        gfx->setCursor(REC_BTN.x + 52, REC_BTN.y + 20);
        gfx->print("REC");
        gfx->setCursor(STOP_BTN.x + 44, STOP_BTN.y + 20);
        gfx->print("STOP");
        gfx->setTextSize(1);
        gfx->setTextColor(0xA514);
        y = 106;
    }
    if (camLink.isOpen()) {
        const CameraState &st = camLink.state();
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        char line[48];
        snprintf(line, sizeof(line), "Battery: %s%s", st.batteryPercent < 0 ? "--" : String(st.batteryPercent).c_str(),
                 st.batteryPercent < 0 ? "" : (st.charging ? "% charging" : "%"));
        gfx->setCursor(12, 78);
        gfx->print(line);
        if (st.storageTotalMb > 0)
            snprintf(line, sizeof(line), "Card: %d of %d GB free", st.storageFreeMb / 1024, st.storageTotalMb / 1024);
        else
            snprintf(line, sizeof(line), "Card: --");
        gfx->setCursor(12, 102);
        gfx->print(line);
        snprintf(line, sizeof(line), "%s  mode %d", st.recording ? "RECORDING" : "Not recording", st.shootingMode);
        gfx->setTextColor(st.recording ? 0xF800 : WHITE);
        gfx->setCursor(12, 126);
        gfx->print(line);
        gfx->setTextSize(1);
        gfx->setTextColor(0xA514);
        y = 156;
    }
    for (const String &l : logLines) {
        if (paired && y > REC_BTN.y - 14) break;
        gfx->setCursor(8, y);
        gfx->print(l.substring(0, 59));
        y += 12;
    }
    gfx->flush();
}

static void log(const String &line) {
    Serial.println(line);
    logLines.push_back(line);
    if (logLines.size() > 30) logLines.erase(logLines.begin());
    draw();
}

static void setStatus(const String &s, uint16_t color = WHITE) {
    status = s;
    statusColor = color;
    Serial.println("== " + s);
    draw();
}

// ---------- hardware ----------

static void tcaWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(TCA9554_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static void resetPanel() {
    tcaWrite(0x03, 0xF8);
    tcaWrite(0x01, 0x00);
    delay(20);
    tcaWrite(0x01, 0x07);
    delay(300);
}

// ---------- BLE link ----------

static NimBLEClient *client = nullptr;
static NimBLERemoteCharacteristic *notifyChar = nullptr, *writeChar = nullptr;

// Notifications arrive on the BLE task; hand the bytes to the loop
static portMUX_TYPE rxLock = portMUX_INITIALIZER_UNLOCKED;
static std::vector<uint8_t> rxBytes;
static Duml::Assembler assembler;
static std::vector<Duml::Frame> inbox;
static uint32_t lastWrite = 0;

static void onNotify(NimBLERemoteCharacteristic *, uint8_t *data, size_t len, bool) {
    portENTER_CRITICAL(&rxLock);
    rxBytes.insert(rxBytes.end(), data, data + len);
    portEXIT_CRITICAL(&rxLock);
}

static void send(const Duml::Frame &f) {
    // Pace writes: the camera drops them if they come too fast
    uint32_t since = millis() - lastWrite;
    if (since < WRITE_GAP_MS) delay(WRITE_GAP_MS - since);
    std::vector<uint8_t> bytes = Duml::encode(f);
    bool ok = writeChar && writeChar->writeValue(bytes.data(), bytes.size(), false);
    lastWrite = millis();
    bool routine = f.cmdSet == 0x00 && f.cmdId == 0x2B; // keepalive
    if (!routine || !ok)
        log(String(ok ? "> " : "> FAILED ") + String(f.cmdSet, HEX) + "/" + String(f.cmdId, HEX) + " " +
            Duml::hex(f.payload, 12));
}

// Moves received bytes into frames; answers the camera's pairing approval request
static void pump() {
    std::vector<uint8_t> bytes;
    portENTER_CRITICAL(&rxLock);
    bytes.swap(rxBytes);
    portEXIT_CRITICAL(&rxLock);
    if (bytes.empty()) return;
    for (Duml::Frame &f : assembler.append(bytes.data(), bytes.size())) {
        // Skip the camera's routine once-a-second status pushes; mask the Wi-Fi password
        bool routine = (f.cmdSet == 0x0D && f.cmdId == 0x02) || (f.cmdSet == 0x04 && f.cmdId == 0x38) ||
                       (f.cmdSet == 0x00 && (f.cmdId == 0xF1 || f.cmdId == 0x81 || f.cmdId == 0x2B ||
                                             f.cmdId == 0xA6 || f.cmdId == 0x74));
        bool secret = f.cmdSet == 0x07 && f.cmdId == 0x0E;
        if (!routine)
            log("< " + String(f.cmdSet, HEX) + "/" + String(f.cmdId, HEX) + " fl " + String(f.flags, HEX) + " " +
                (secret ? String("(password hidden)") : Duml::hex(f.payload, 14)));
        if (f.cmdSet == 0x07 && f.cmdId == 0x46 && f.flags != Duml::FLAG_RESPONSE) {
            log("camera approved pairing - acknowledging");
            send(Duml::pairApprovalAck(f.seq));
        }
        if (f.cmdSet == 0x0D && f.cmdId == 0x02 && f.payload.size() >= 21) {
            int pct = f.payload[20];
            bool chg = f.payload.size() >= 34 && f.payload[32] == 1;
            if (pct <= 100 && (pct != btBattery || chg != btCharging)) {
                btBattery = pct;
                btCharging = chg;
                draw();
            }
        }
        inbox.push_back(f);
    }
}

// Returns 1 while a finger is down (with its coordinates), 0 when up, -1 on a read error
static int readTouch(int &x, int &y) {
    Wire.beginTransmission(FT3168_ADDR);
    Wire.write(0x02);
    if (Wire.endTransmission(false) != 0) return -1;
    if (Wire.requestFrom(FT3168_ADDR, (uint8_t)5) != 5) return -1;
    uint8_t buf[5];
    for (int i = 0; i < 5; i++) buf[i] = Wire.read();
    if ((buf[0] & 0x0F) == 0) return 0;
    x = ((buf[1] & 0x0F) << 8) | buf[2];
    y = ((buf[3] & 0x0F) << 8) | buf[4];
    return 1;
}

// Waits for a frame with set/cmd (any of up to two), or times out
static bool waitFor(uint8_t set, uint8_t cmd, uint32_t timeoutMs, Duml::Frame &out, uint8_t set2 = 0xFF,
                    uint8_t cmd2 = 0xFF) {
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        pump();
        for (size_t i = 0; i < inbox.size(); i++) {
            const Duml::Frame &f = inbox[i];
            if ((f.cmdSet == set && f.cmdId == cmd) || (f.cmdSet == set2 && f.cmdId == cmd2)) {
                out = f;
                inbox.erase(inbox.begin() + i);
                return true;
            }
        }
        if (client && !client->isConnected()) return false;
        delay(20);
    }
    return false;
}

static const NimBLEAdvertisedDevice *findCamera() {
    setStatus("Scanning for the camera...");
    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setActiveScan(true);
    NimBLEScanResults results = scan->getResults(6000, false);
    const NimBLEAdvertisedDevice *best = nullptr;
    for (int i = 0; i < results.getCount(); i++) {
        const NimBLEAdvertisedDevice *d = results.getDevice(i);
        std::string m = d->getManufacturerData();
        bool dji = m.size() >= 2 && ((uint8_t)m[0] | ((uint8_t)m[1] << 8)) == DJI_COMPANY_ID;
        String name = d->getName().c_str();
        if (!dji && name.indexOf("Osmo") < 0 && name.indexOf("Pocket") < 0) continue;
        std::vector<uint8_t> mb(m.begin(), m.end());
        log("found " + String(name.length() ? name : "(no name)") + " " + String(d->getRSSI()) + " dBm");
        log("  " + String(d->getAddress().toString().c_str()) + " mfr " + Duml::hex(mb, 16));
        if (!best || d->getRSSI() > best->getRSSI()) best = d;
    }
    return best;
}

static bool connectCamera(const NimBLEAdvertisedDevice *cam) {
    setStatus("Connecting...");
    client = NimBLEDevice::createClient();
    if (!client->connect(cam)) {
        log("connect failed");
        return false;
    }
    log("connected, MTU " + String(client->getMTU()));
    NimBLERemoteService *svc = client->getService("fff0");
    if (!svc) {
        log("service fff0 not found");
        return false;
    }
    // Enable notifications on every notifying characteristic (fff4 is the main one)
    for (NimBLERemoteCharacteristic *c : svc->getCharacteristics(true)) {
        String uuid = c->getUUID().toString().c_str();
        log("  char " + uuid + (c->canNotify() ? " notify" : "") + (c->canWriteNoResponse() ? " write" : ""));
        if (c->canNotify()) c->subscribe(true, onNotify);
        if (uuid.indexOf("fff4") >= 0) notifyChar = c;
        if (uuid.indexOf("fff5") >= 0) writeChar = c;
    }
    if (!notifyChar || !writeChar) {
        log("fff4 / fff5 missing");
        return false;
    }
    // Arm pairing: write 01 00 to fff4
    uint8_t arm[2] = {0x01, 0x00};
    bool armed = notifyChar->writeValue(arm, 2, true);
    log(String("armed: ") + (armed ? "ok" : "failed"));
    return true;
}

static bool pair() {
    setStatus("Pairing...");
    send(Duml::sessionWake());
    send(Duml::setPairingPin("osmo"));
    Duml::Frame reply;
    if (!waitFor(0x07, 0x45, 15000, reply, 0x07, 0x46)) {
        log("no pairing reply");
        return false;
    }
    if (reply.cmdId == 0x45 && reply.payload.size() >= 2) {
        uint8_t code = reply.payload[1];
        if (code == 1) {
            log("already paired");
            return true;
        }
        if (code == 2) {
            setStatus("Approve OSMO on the camera!", 0xFD20);
            Duml::Frame approval;
            if (!waitFor(0x07, 0x46, 90000, approval)) {
                log("approval timed out");
                return false;
            }
            return true; // pump() already acknowledged it
        }
        log("pairing reply code " + String(code));
        if (code == 10) { // seen while FrameTap holds the session: try commands anyway
            setStatus("Camera busy (code 10) - testing anyway", 0xFD20);
            return true;
        }
        return code == 0;
    }
    return true; // approval request arrived directly
}

static void readWifi() {
    setStatus("Reading camera Wi-Fi...");
    Duml::Frame reply;
    send(Duml::wakeAccessPoint());
    if (!waitFor(0x53, 0x10, 2000, reply)) log("no reply to AP wake (continuing)");
    delay(600);
    String ssid, pass;
    send(Duml::getWifiSsid());
    if (waitFor(0x07, 0x07, 4000, reply)) ssid = Duml::unpackStatusString(reply.payload);
    send(Duml::getWifiPassword());
    if (waitFor(0x07, 0x0E, 4000, reply)) pass = Duml::unpackStatusString(reply.payload);
    camSsid = ssid;
    camPass = pass;
    log("camera Wi-Fi: \"" + ssid + "\"");
    log("password: " + String(pass.length()) + " chars");
    Serial.printf("WIFI SSID=%s PASS=%s\n", ssid.c_str(), pass.c_str()); // serial only
    if (ssid.length()) setStatus("Paired! Wi-Fi: " + ssid, 0x07E0);
    else setStatus("Paired, but no Wi-Fi details", 0xFD20);
}

// Station mode: the board hosts a 2.4 GHz AP and tells the camera (over BLE) to
// join it, so both sit on the board's network and control runs over Wi-Fi.
static const char *AP_SSID = "PocketRemote";
static const char *AP_PASS = "osmo12345678";

static IPAddress waitForCameraJoin(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        pump();
        if (WiFi.softAPgetStationNum() > 0) {
            // ESP32 SoftAP DHCP hands the first client 192.168.4.2
            return IPAddress(192, 168, 4, 2);
        }
        delay(200);
    }
    return IPAddress((uint32_t)0);
}

static bool stationLink() {
    setStatus("Starting board Wi-Fi...");
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(AP_SSID, AP_PASS)) {
        log("softAP failed");
        return false;
    }
    log("board AP " + String(AP_SSID) + " at " + WiFi.softAPIP().toString());

    Duml::Frame reply;
    setStatus("Camera -> station mode...");
    send(Duml::stationMode(true, cmdSeq++));
    if (waitFor(0x07, 0x48, 4000, reply)) log("STA switch reply " + Duml::hex(reply.payload, 4));
    else log("no STA reply (continuing)");
    delay(1500);

    setStatus("Camera joining board Wi-Fi...");
    send(Duml::joinWifi(AP_SSID, AP_PASS, cmdSeq++));
    if (waitFor(0x07, 0x47, 12000, reply)) log("join reply " + Duml::hex(reply.payload, 4));
    else log("no join reply (continuing)");

    IPAddress camIp = waitForCameraJoin(20000);
    if (camIp == IPAddress((uint32_t)0)) {
        log("camera did not join the board AP");
        return false;
    }
    log("camera joined, IP " + camIp.toString());
    setStatus("Opening control link...");
    if (!camLink.open(camIp, log)) {
        log("control link failed");
        return false;
    }
    setStatus("Controlling Pocket 4P!", 0x07E0);
    return true;
}

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);
    resetPanel();
    gfx->begin();
    panel->setBrightness(200);
    draw();

    NimBLEDevice::init("ESP32 Remote");
    NimBLEDevice::setMTU(517);

    const NimBLEAdvertisedDevice *cam = findCamera();
    if (!cam) {
        setStatus("No DJI camera found", 0xF800);
        log("Is the 4P on and nearby?");
        return;
    }
    if (!connectCamera(cam)) {
        setStatus("Connection failed", 0xF800);
        return;
    }
    if (!pair()) {
        setStatus("Pairing failed", 0xF800);
        return;
    }
    paired = true;
    readWifi(); // harmless; also confirms BLE command path
    if (!stationLink()) {
        setStatus("Station link failed", 0xF800);
        return;
    }
    log("Tap REC / STOP - control over Wi-Fi now");
}

void loop() {
    // Keep the session alive and show anything the camera sends
    static uint32_t lastKeepalive = 0;
    pump();
    camLink.poll();
    static bool down = false;
    int tx, ty;
    int touch = readTouch(tx, ty);
    if (touch == 1 && !down && camLink.isOpen()) {
        // Record commands go over the Wi-Fi datalink (BLE can't control the camera)
        if (REC_BTN.hit(tx, ty)) {
            log("REC (wifi)");
            camLink.send(Duml::recordStart(0));
        } else if (STOP_BTN.hit(tx, ty)) {
            log("STOP (wifi)");
            camLink.send(Duml::recordStop(0));
        }
    }
    if (touch >= 0) down = touch == 1;
    static uint32_t lastDraw = 0;
    if (camLink.isOpen() && millis() - lastDraw > 500) {
        lastDraw = millis();
        draw();
    }
    if (client && client->isConnected() && millis() - lastKeepalive > 2000) {
        lastKeepalive = millis();
        send(Duml::sessionKeepalive());
    }
    if (client && !client->isConnected()) {
        static bool reported = false;
        if (!reported) {
            reported = true;
            setStatus("Camera disconnected", 0xF800);
        }
    }
    delay(20);
}
