// Bluetooth relay between FrameTap and a DJI Osmo Pocket 4P.
//
//   FrameTap <--BLE--> board (looks like the 4P) <--BLE--> Pocket 4P
//
// The board connects to the camera as a central, then advertises with the
// camera's name, manufacturer data and fff0 service so FrameTap connects to the
// board instead. Every write and notification is passed through unchanged and
// logged as decoded DUML frames. FrameTap still joins the camera's Wi-Fi
// directly for live view. Nothing is injected yet.
#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include <Arduino_GFX_Library.h>
#include <deque>
#include "duml.h"

static const int I2C_SDA = 15;
static const int I2C_SCL = 14;
static const int LCD_CS = 12, LCD_SCK = 11;
static const int LCD_D0 = 4, LCD_D1 = 5, LCD_D2 = 6, LCD_D3 = 7;
static const int LCD_W = 368, LCD_H = 448;
static const uint8_t TCA9554_ADDR = 0x20;

static const uint16_t DJI_COMPANY_ID = 0x08AA;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
Arduino_CO5300 *panel = new Arduino_CO5300(bus, GFX_NOT_DEFINED, 0, false, LCD_W, LCD_H);
Arduino_Canvas *gfx = new Arduino_Canvas(LCD_W, LCD_H, panel);

// ---------- screen ----------

static String camStatus = "Camera: searching", tapStatus = "FrameTap: waiting";
static uint32_t framesToCam = 0, framesToTap = 0;
static std::vector<String> logLines;
static bool needsDraw = true;

static void draw() {
    gfx->fillScreen(BLACK);
    gfx->setTextSize(2);
    gfx->setTextColor(0x05FF);
    gfx->setCursor(12, 10);
    gfx->print("Pocket 4P relay");
    gfx->setTextColor(WHITE);
    gfx->setCursor(12, 38);
    gfx->print(camStatus);
    gfx->setCursor(12, 62);
    gfx->print(tapStatus);
    gfx->setTextSize(1);
    gfx->setTextColor(0x8410);
    gfx->setCursor(12, 88);
    gfx->printf("FrameTap->camera %lu   camera->FrameTap %lu", framesToCam, framesToTap);
    gfx->setTextColor(0xA514);
    int y = 106;
    for (const String &l : logLines) {
        gfx->setCursor(6, y);
        gfx->print(l.substring(0, 60));
        y += 12;
    }
    gfx->flush();
}

static void log(const String &line) {
    Serial.println(line);
    logLines.push_back(line);
    if (logLines.size() > 27) logLines.erase(logLines.begin());
    needsDraw = true;
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

// ---------- relay plumbing ----------
// BLE callbacks run on the NimBLE task; they only queue bytes. The loop does
// the actual writes/notifies so neither side blocks the other.

enum Dir : uint8_t { TO_CAMERA, TO_FRAMETAP };
struct Packet {
    Dir dir;
    uint16_t charId; // 0xfff3 / 0xfff4 / 0xfff5
    bool withResponse;
    std::vector<uint8_t> data;
};
static portMUX_TYPE qLock = portMUX_INITIALIZER_UNLOCKED;
static std::deque<Packet> queue;

static void enqueue(Dir dir, uint16_t charId, const uint8_t *data, size_t len, bool withResponse) {
    Packet p{dir, charId, withResponse, std::vector<uint8_t>(data, data + len)};
    portENTER_CRITICAL(&qLock);
    if (queue.size() < 200) queue.push_back(p);
    portEXIT_CRITICAL(&qLock);
}

static uint16_t shortId(const NimBLEUUID &uuid) {
    String s = uuid.toString().c_str(); // "0xfff4" for 16-bit UUIDs
    return (uint16_t)strtol(s.c_str() + (s.startsWith("0x") ? 2 : 0), nullptr, 16);
}

// Camera side (we are the central)
static NimBLEClient *camClient = nullptr;
static NimBLERemoteCharacteristic *camChars[3] = {}; // fff3, fff4, fff5
static std::string camName, camMfr;

static void onCameraNotify(NimBLERemoteCharacteristic *c, uint8_t *data, size_t len, bool) {
    enqueue(TO_FRAMETAP, shortId(c->getUUID()), data, len, false);
}

// FrameTap side (we are the peripheral)
static NimBLEServer *server = nullptr;
static NimBLECharacteristic *tapChars[3] = {};
static volatile bool tapConnected = false;

class TapServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *, NimBLEConnInfo &info) override {
        tapConnected = true;
        tapStatus = "FrameTap: connected";
        needsDraw = true;
        Serial.printf("FrameTap connected from %s\n", info.getAddress().toString().c_str());
    }
    void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int reason) override {
        tapConnected = false;
        tapStatus = "FrameTap: disconnected";
        needsDraw = true;
        Serial.printf("FrameTap disconnected (reason %d)\n", reason);
        NimBLEDevice::startAdvertising();
    }
    void onMTUChange(uint16_t mtu, NimBLEConnInfo &) override { Serial.printf("FrameTap MTU %u\n", mtu); }
};

class TapCharCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override {
        NimBLEAttValue v = c->getValue();
        uint16_t id = shortId(c->getUUID());
        enqueue(TO_CAMERA, id, v.data(), v.size(), id == 0xfff4); // fff4 arm write expects a response
    }
    void onSubscribe(NimBLECharacteristic *c, NimBLEConnInfo &, uint16_t subValue) override {
        Serial.printf("FrameTap subscribed to %s: %u\n", c->getUUID().toString().c_str(), subValue);
    }
};


// ---------- logging decoded frames ----------

static Duml::Assembler asmToCam[3], asmToTap[3];

static bool routine(const Duml::Frame &f) {
    return (f.cmdSet == 0x0D && f.cmdId == 0x02) || (f.cmdSet == 0x04 && f.cmdId == 0x38) ||
           (f.cmdSet == 0x00 && (f.cmdId == 0xF1 || f.cmdId == 0x2B || f.cmdId == 0x81 || f.cmdId == 0xA6 ||
                                 f.cmdId == 0x74));
}

static void logFrames(const Packet &p) {
    int idx = p.charId - 0xfff3;
    if (idx < 0 || idx > 2) return;
    Duml::Assembler &a = p.dir == TO_CAMERA ? asmToCam[idx] : asmToTap[idx];
    std::vector<Duml::Frame> frames = a.append(p.data.data(), p.data.size());
    if (frames.empty() && p.data.size() <= 4) {
        log(String(p.dir == TO_CAMERA ? "T>C " : "C>T ") + String(p.charId, HEX) + " raw " + Duml::hex(p.data, 8));
        return;
    }
    for (const Duml::Frame &f : frames) {
        bool secret = f.cmdSet == 0x07 && f.cmdId == 0x0E && f.flags == Duml::FLAG_RESPONSE;
        String line = String(p.dir == TO_CAMERA ? "T>C " : "C>T ") + String(f.cmdSet, HEX) + "/" +
                      String(f.cmdId, HEX) + " fl" + String(f.flags, HEX) + " " +
                      (secret ? String("(hidden)") : Duml::hex(f.payload, 14));
        if (routine(f)) Serial.println(line);
        else log(line);
    }
}

// ---------- setup steps ----------

static NimBLEAddress camAddress;

// Finds the camera and remembers its name, manufacturer data and address
static void scanForCamera() {
    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setActiveScan(true);
    while (true) {
        camStatus = "Camera: searching";
        draw();
        NimBLEScanResults results = scan->getResults(5000, false);
        for (int i = 0; i < results.getCount(); i++) {
            const NimBLEAdvertisedDevice *d = results.getDevice(i);
            std::string m = d->getManufacturerData();
            if (m.size() >= 2 && ((uint8_t)m[0] | ((uint8_t)m[1] << 8)) == DJI_COMPANY_ID) {
                camName = d->getName();
                camMfr = m;
                camAddress = d->getAddress();
                log("camera " + String(camName.c_str()) + " " + String(camAddress.toString().c_str()) + " type" +
                    String(camAddress.getBase()->type));
                scan->stop();
                scan->clearResults();
                return;
            }
        }
    }
}

static bool connectCamera() {
    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setActiveScan(true);
    while (true) {
        camStatus = "Camera: searching";
        needsDraw = true;
        draw();
        (void)scan;
        camClient = NimBLEDevice::createClient();
        if (!camClient->connect(camAddress)) {
            log("camera connect failed, retrying");
            NimBLEDevice::deleteClient(camClient);
            delay(1000);
            continue;
        }
        NimBLERemoteService *svc = camClient->getService("fff0");
        if (!svc) {
            log("camera has no fff0 service");
            camClient->disconnect();
            continue;
        }
        for (NimBLERemoteCharacteristic *c : svc->getCharacteristics(true)) {
            int idx = shortId(c->getUUID()) - 0xfff3;
            if (idx < 0 || idx > 2) continue;
            camChars[idx] = c;
            if (c->canNotify()) c->subscribe(true, onCameraNotify);
        }
        // List any other services, in case FrameTap needs them too
        for (NimBLERemoteService *s : camClient->getServices(true))
            Serial.printf("camera service %s\n", s->getUUID().toString().c_str());
        camStatus = "Camera: connected";
        needsDraw = true;
        log("camera link up, MTU " + String(camClient->getMTU()));
        return true;
    }
}

// Build and start the GATT server (fff0/fff3/fff4/fff5) while the stack is idle
static void buildServer() {
    server = NimBLEDevice::createServer();
    server->setCallbacks(new TapServerCallbacks());
    NimBLEService *svc = server->createService("FFF0");
    static TapCharCallbacks charCallbacks;
    tapChars[0] = svc->createCharacteristic("FFF3", NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    tapChars[1] = svc->createCharacteristic(
        "FFF4", NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    tapChars[2] = svc->createCharacteristic(
        "FFF5", NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    for (NimBLECharacteristic *c : tapChars) c->setCallbacks(&charCallbacks);
    server->start();
}

// Start advertising as the camera (name + manufacturer data + fff0)
static void startAdvertising() {
    // A legacy advert holds 31 bytes: flags + manufacturer data go in the advert,
    // name + service in the scan response
    NimBLEAdvertisementData adv;
    adv.setFlags(0x06);
    adv.setManufacturerData(camMfr);
    NimBLEAdvertisementData scanResp;
    scanResp.setName(camName);
    scanResp.setCompleteServices16({NimBLEUUID((uint16_t)0xFFF0)});
    NimBLEAdvertising *a = NimBLEDevice::getAdvertising();
    bool advOk = a->setAdvertisementData(adv);
    bool respOk = a->setScanResponseData(scanResp);
    bool started = false;
    for (int attempt = 0; attempt < 5 && !started; attempt++) {
        started = a->start();
        if (!started) delay(300);
    }
    log(String("advert ") + (advOk ? "ok" : "TOO BIG") + ", scan resp " + (respOk ? "ok" : "TOO BIG") +
        ", start " + (started ? "ok" : "FAILED"));
    tapStatus = "FrameTap: turn it on now";
    needsDraw = true;
    log("advertising as " + String(camName.c_str()));
}

// ---------- main ----------

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);
    resetPanel();
    gfx->begin();
    panel->setBrightness(200);
    draw();

    NimBLEDevice::init("OsmoPocket4P");
    NimBLEDevice::setMTU(517);

    // Scan for the camera (gives the correct address + type), then connect FIRST
    // with our own address: one radio can't wear the camera's address and also
    // connect to it.
    buildServer(); // GATT ready while idle (can't be added once busy)
    scanForCamera();
    connectCamera();

    // Now take the camera's address for advertising only; the existing central
    // link keeps the address it was made with, so FrameTap sees "the camera".
    uint8_t own[6];
    memcpy(own, camAddress.getVal(), 6);
    int rc = ble_hs_id_set_rnd(own);
    if (rc == 0) {
        NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM);
        log("address cloned for advertising");
    } else {
        log("clone rc " + String(rc) + " - using own address");
    }
    startAdvertising();
}

void loop() {
    // Forward queued packets in order
    for (int n = 0; n < 20; n++) {
        Packet p;
        portENTER_CRITICAL(&qLock);
        bool has = !queue.empty();
        if (has) {
            p = queue.front();
            queue.pop_front();
        }
        portEXIT_CRITICAL(&qLock);
        if (!has) break;
        int idx = p.charId - 0xfff3;
        if (idx < 0 || idx > 2) continue;
        if (p.dir == TO_CAMERA) {
            if (camChars[idx] && camClient && camClient->isConnected())
                camChars[idx]->writeValue(p.data.data(), p.data.size(), p.withResponse);
            framesToCam++;
        } else {
            if (tapConnected && tapChars[idx]) tapChars[idx]->notify(p.data.data(), p.data.size());
            framesToTap++;
        }
        logFrames(p);
    }

    if (camClient && !camClient->isConnected() && camStatus != "Camera: LOST") {
        camStatus = "Camera: LOST";
        needsDraw = true;
        log("camera disconnected");
    }

    static uint32_t lastDraw = 0;
    if (needsDraw && millis() - lastDraw > 200) {
        lastDraw = millis();
        needsDraw = false;
        draw();
    }
    delay(5);
}
