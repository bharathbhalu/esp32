// Proxy: the board poses as "another Pocket 4P" so FrameTap controls the real
// camera through it.
//
//   FrameTap <--BLE--> board (own address + name) <--BLE--> Pocket 4P
//   FrameTap --Wi-Fi--> board AP 192.168.2.1 <--Wi-Fi-- Pocket 4P (station)
//
// At start the board pairs with the camera and has it join the board's AP.
// FrameTap's BLE traffic is then passed through, except Wi-Fi requests: SSID,
// password and channel are answered with the board's AP, and join/station
// commands are acked here so the camera stays put. FrameTap connects to the
// board at 192.168.2.1 (where the camera's own hotspot would be) and its UDP
// 9004 / TCP 7001 traffic is relayed unchanged to the camera. Camera-setting
// commands FrameTap sends are logged.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_coexist.h>
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
static const char *AP_SSID = "PocketRemote";
static const char *AP_PASS = "osmo12345678";
// false: only copy the camera's advert, don't connect to it (tests board <-> FrameTap alone)
static const bool CAMERA_LINK = true;
// Experiment: turn BLE off once FrameTap is streaming over Wi-Fi
static const bool BLE_OFF_WHILE_STREAMING = false;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
Arduino_CO5300 *panel = new Arduino_CO5300(bus, GFX_NOT_DEFINED, 0, false, LCD_W, LCD_H);
Arduino_Canvas *gfx = new Arduino_Canvas(LCD_W, LCD_H, panel);

// ---------- screen ----------

static String camStatus = "Camera: searching", tapStatus = "FrameTap: waiting", netStatus = "Wi-Fi: off";
static uint32_t framesToCam = 0, framesToTap = 0;
static std::vector<String> logLines;
static bool needsDraw = true;

static void draw() {
    gfx->fillScreen(BLACK);
    gfx->setTextSize(2);
    gfx->setTextColor(0x05FF);
    gfx->setCursor(12, 10);
    gfx->print("Pocket 4P proxy");
    gfx->setTextColor(WHITE);
    gfx->setCursor(12, 38);
    gfx->print(camStatus);
    gfx->setCursor(12, 62);
    gfx->print(tapStatus);
    gfx->setCursor(12, 86);
    gfx->print(netStatus);
    gfx->setTextSize(1);
    gfx->setTextColor(0x8410);
    gfx->setCursor(12, 110);
    gfx->printf("FrameTap->camera %lu   camera->FrameTap %lu", framesToCam, framesToTap);
    gfx->setTextColor(0xA514);
    int y = 126;
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
    if (logLines.size() > 25) logLines.erase(logLines.begin());
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

static volatile bool tapConnected = false;
static volatile bool bringupActive = false; // true while the board runs its own session
static std::vector<uint8_t> *bringupSink = nullptr;

static void onCameraNotify(NimBLERemoteCharacteristic *c, uint8_t *data, size_t len, bool) {
    if (bringupActive && bringupSink && shortId(c->getUUID()) == 0xfff4) {
        portENTER_CRITICAL(&qLock);
        bringupSink->insert(bringupSink->end(), data, data + len);
        portEXIT_CRITICAL(&qLock);
        return;
    }
    if (!tapConnected) return; // nobody to forward to (FrameTap leaves BLE once on Wi-Fi)
    enqueue(TO_FRAMETAP, shortId(c->getUUID()), data, len, false);
}

// FrameTap side (we are the peripheral)
static NimBLEServer *server = nullptr;
static NimBLECharacteristic *tapChars[3] = {};

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
        // pollNetwork() restarts advertising if FrameTap's Wi-Fi link isn't running
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

static void logFrame(const char *tag, const Duml::Frame &f) {
    bool secret = f.cmdSet == 0x07 && f.cmdId == 0x0E && f.flags == Duml::FLAG_RESPONSE;
    String line = String(tag) + String(f.cmdSet, HEX) + "/" + String(f.cmdId, HEX) + " fl" + String(f.flags, HEX) +
                  " " + (secret ? String("(hidden)") : Duml::hex(f.payload, 14));
    if (routine(f)) Serial.println(line);
    else log(line);
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

// ---------- Wi-Fi interception ----------

// Our answer to a FrameTap request: same seq, sender/receiver swapped
static Duml::Frame replyTo(const Duml::Frame &req, std::vector<uint8_t> payload) {
    Duml::Frame r;
    r.sender = req.receiver;
    r.receiver = req.sender;
    r.seq = req.seq;
    r.flags = req.flags | 0x80; // the camera answers FrameTap's 0x20 requests with 0xA0
    r.cmdSet = req.cmdSet;
    r.cmdId = req.cmdId;
    r.payload = payload;
    return r;
}

static std::vector<uint8_t> statusString(const char *s) {
    std::vector<uint8_t> p{0x00};
    std::vector<uint8_t> str = Duml::packString(s);
    p.insert(p.end(), str.begin(), str.end());
    return p;
}

static void notifyTap(const Duml::Frame &f) {
    std::vector<uint8_t> b = Duml::encode(f);
    if (tapConnected && tapChars[1]) tapChars[1]->notify(b.data(), b.size());
    framesToTap++;
}

// True if we answered the frame ourselves (it must not reach the camera)
static bool intercept(const Duml::Frame &f) {
    if (f.cmdSet != 0x07 || f.flags == Duml::FLAG_RESPONSE) return false;
    switch (f.cmdId) {
    case 0x45: // pairing: say "already paired" (code 1) so no approval is needed
        if (CAMERA_LINK) return false;
        notifyTap(replyTo(f, {0x00, 0x01}));
        log("  -> answered pairing: already paired");
        return true;
    case 0x07: // SSID read
        notifyTap(replyTo(f, statusString(AP_SSID)));
        log("  -> answered SSID with " + String(AP_SSID));
        return true;
    case 0x0E: // password read
        notifyTap(replyTo(f, statusString(AP_PASS)));
        log("  -> answered password with ours");
        return true;
    case 0x2C: // hotspot channel: the camera says 149 (5 GHz); ours is 2.4 GHz
        notifyTap(replyTo(f, {0x00, (uint8_t)WiFi.channel()}));
        log("  -> answered channel " + String(WiFi.channel()));
        return true;
    case 0x47: // join a network
    case 0x48: // AP/STA switch
        notifyTap(replyTo(f, {0x00}));
        log("  -> acked here, NOT sent to camera");
        return true;
    default:
        return false;
    }
}

// FrameTap -> camera: reassemble, answer Wi-Fi requests, forward the rest
static void forwardToCamera(const Packet &p, int idx) {
    std::vector<Duml::Frame> frames = asmToCam[idx].append(p.data.data(), p.data.size());
    bool camUp = camChars[idx] && camClient && camClient->isConnected();
    if (frames.empty()) {
        // Non-DUML writes (e.g. the 01 00 arm on fff4) go through raw
        if (p.data.size() <= 4) {
            log("T>C " + String(p.charId, HEX) + " raw " + Duml::hex(p.data, 8));
            asmToCam[idx].reset();
            if (camUp) camChars[idx]->writeValue(p.data.data(), p.data.size(), p.withResponse);
            framesToCam++;
        }
        return; // otherwise a partial frame: wait for the rest
    }
    for (const Duml::Frame &f : frames) {
        logFrame("T>C ", f);
        if (intercept(f)) continue;
        std::vector<uint8_t> b = Duml::encode(f);
        if (camUp) camChars[idx]->writeValue(b.data(), b.size(), p.withResponse);
        framesToCam++;
    }
}

// ---------- Wi-Fi relay ----------
// FrameTap talks to "the camera" at 192.168.2.1 (the board). Its packets are
// passed unchanged to the real camera, which joined the board's AP as a
// station, and the camera's replies go back. FrameTap and the camera run one
// datalink session between them, so nothing needs re-sequencing.
#include <lwip/sockets.h>

static const uint16_t DATALINK_PORT = 9004;
static volatile uint32_t camIpAddr = 0; // camera's station IP (network order), 0 until it joins
static volatile bool expectCamera = false; // the next station to get an IP is the camera
static WiFiServer tcpServer(7001);
static WiFiClient tapTcp, camTcp;
static volatile uint32_t udpToCam = 0, udpToTap = 0;
static volatile uint32_t bytesToCam = 0, bytesToTap = 0;
static volatile uint32_t sendFails = 0, maxPacket = 0;
// Video arriving from the camera: sequence gaps mean loss on the camera -> board leg
static volatile uint32_t videoPkts = 0, videoMissing = 0, videoOutOfOrder = 0;
static volatile uint32_t lastTapPacket = 0; // millis of FrameTap's last datalink packet

static void onWifiEvent(arduino_event_id_t event, arduino_event_info_t info) {
    if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED) {
        const uint8_t *m = info.wifi_ap_staconnected.mac;
        Serial.printf("Wi-Fi joined %02X:%02X:%02X:%02X:%02X:%02X\n", m[0], m[1], m[2], m[3], m[4], m[5]);
    } else if (event == ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED) {
        IPAddress ip(info.wifi_ap_staipassigned.ip.addr);
        if (expectCamera && camIpAddr == 0) {
            camIpAddr = info.wifi_ap_staipassigned.ip.addr;
            expectCamera = false;
            Serial.printf("camera got %s\n", ip.toString().c_str());
        } else {
            Serial.printf("FrameTap phone got %s\n", ip.toString().c_str());
        }
    } else if (event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) {
        Serial.println("station left");
    }
}

// Logs FrameTap's camera-setting commands (set 0x02), the ones we want to learn
static void logSettings(const uint8_t *raw, size_t n) {
    for (size_t i = 0; i + 13 <= n; i++) {
        if (raw[i] != 0x55) continue;
        size_t total = raw[i + 1] | ((raw[i + 2] & 0x03) << 8);
        if ((raw[i + 2] >> 2) != 1 || total < 13 || i + total > n) continue;
        if (Duml::crc8(raw + i, 3) != raw[i + 3]) continue;
        uint8_t set = raw[i + 9], cmd = raw[i + 10], flags = raw[i + 8];
        if (set == 0x02 || (set == 0x04 && cmd == 0x14)) {
            std::vector<uint8_t> payload(raw + i + 11, raw + i + total - 2);
            Serial.printf("SET %02X/%02X fl%02X %s\n", set, cmd, flags, Duml::hex(payload, 24).c_str());
        }
        i += total - 1;
    }
}

// Two tasks share one socket: the receiver only empties lwIP's small UDP queue
// (6 packets by default) into a large PSRAM ring, the sender forwards from it.
// Receiving and sending in one loop dropped whole bursts of live view.
static const int RING_SLOTS = 512;
struct Slot {
    uint16_t len;
    bool toCamera;
    uint8_t data[1600];
};
static Slot *ring = nullptr;
static QueueHandle_t ready = nullptr; // indexes of filled slots, in arrival order
static int relaySock = -1;
static sockaddr_in tapAddr{}; // FrameTap's address, learned from its first packet
static volatile uint32_t ringDrops = 0;

static void udpReceiveTask(void *) {
    uint16_t next = 0;
    while (true) {
        Slot &slot = ring[next];
        sockaddr_in from{};
        socklen_t fromLen = sizeof(from);
        int n = recvfrom(relaySock, slot.data, sizeof(slot.data), 0, (sockaddr *)&from, &fromLen);
        if (n <= 0 || camIpAddr == 0) continue;
        slot.len = n;
        slot.toCamera = from.sin_addr.s_addr != camIpAddr;
        if (slot.toCamera) {
            if (tapAddr.sin_addr.s_addr != from.sin_addr.s_addr || tapAddr.sin_port != from.sin_port) {
                tapAddr = from;
                Serial.printf("FrameTap datalink from %s:%u\n", IPAddress(from.sin_addr.s_addr).toString().c_str(),
                              ntohs(from.sin_port));
            }
            lastTapPacket = millis();
        } else if (n >= 8 && slot.data[6] == 0x02) {
            // Video: seq (bytes 4-5) steps by 8; a smaller or negative step is a resend
            static int32_t lastSeq = -1;
            uint16_t seq = slot.data[4] | (slot.data[5] << 8);
            if (lastSeq >= 0) {
                int16_t d = (int16_t)(seq - (uint16_t)lastSeq);
                if (d > 8 && d < 4000) videoMissing += d / 8 - 1;
                else if (d <= 0) videoOutOfOrder++;
            }
            if (lastSeq < 0 || (int16_t)(seq - (uint16_t)lastSeq) > 0) lastSeq = seq;
            videoPkts++;
        }
        if (xQueueSend(ready, &next, 0) != pdTRUE) {
            ringDrops++;
            continue; // ring full: reuse this slot
        }
        next = (next + 1) % RING_SLOTS;
    }
}

// Header capture for working out the session's numbering (video skipped):
// the first 60 packets in full, then up to 4 per direction+type every 5 s
static void sampleHeader(const Slot &slot) {
    if (slot.len < 8 || slot.data[6] == 0x02) return;
    static uint32_t total = 0, windowStart = 0;
    static uint8_t perType[2][8] = {};
    uint8_t type = slot.data[6] & 7;
    if (millis() - windowStart > 5000) {
        windowStart = millis();
        memset(perType, 0, sizeof(perType));
    }
    if (total >= 60 && perType[slot.toCamera][type] >= 4) return;
    total++;
    perType[slot.toCamera][type]++;
    char h[100];
    int n = slot.len < 32 ? slot.len : 32;
    for (int k = 0; k < n; k++) snprintf(h + k * 3, 4, "%02X ", slot.data[k]);
    Serial.printf("HDR %s t%u seq%5u len%4u  %s\n", slot.toCamera ? "T>C" : "C>T", type,
                  slot.data[4] | (slot.data[5] << 8), slot.len, h);
}

static void udpSendTask(void *) {
    uint16_t idx;
    sockaddr_in cam{};
    cam.sin_family = AF_INET;
    cam.sin_port = htons(DATALINK_PORT);
    while (true) {
        if (xQueueReceive(ready, &idx, portMAX_DELAY) != pdTRUE) continue;
        Slot &slot = ring[idx];
        const sockaddr_in *to = &tapAddr;
        if (slot.toCamera) {
            cam.sin_addr.s_addr = camIpAddr;
            to = &cam;
        } else if (tapAddr.sin_port == 0) {
            continue;
        }
        if (sendto(relaySock, slot.data, slot.len, 0, (const sockaddr *)to, sizeof(*to)) < 0) sendFails++;
        sampleHeader(slot);
        if (slot.toCamera) {
            udpToCam++;
            bytesToCam += slot.len;
            logSettings(slot.data, slot.len);
        } else {
            udpToTap++;
            bytesToTap += slot.len;
            if (slot.len > maxPacket) maxPacket = slot.len;
        }
    }
}

static void startRelay() {
    ring = (Slot *)ps_malloc(sizeof(Slot) * RING_SLOTS);
    // Two spare slots: the one being sent and the one being received into
    ready = xQueueCreate(RING_SLOTS - 2, sizeof(uint16_t));
    relaySock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(DATALINK_PORT);
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(relaySock, (sockaddr *)&local, sizeof(local));
    int rcvBuf = 64 * 1024;
    setsockopt(relaySock, SOL_SOCKET, SO_RCVBUF, &rcvBuf, sizeof(rcvBuf));
    // Receiver above the sender so the lwIP queue is always drained first
    xTaskCreatePinnedToCore(udpReceiveTask, "udpRx", 4096, nullptr, 12, nullptr, 1);
    xTaskCreatePinnedToCore(udpSendTask, "udpTx", 4096, nullptr, 11, nullptr, 1);
}

static void startNetwork() {
    WiFi.onEvent(onWifiEvent);
    WiFi.mode(WIFI_AP);
    // The camera's own hotspot puts it at 192.168.2.1, and that's where FrameTap connects
    IPAddress apIp(192, 168, 2, 1);
    WiFi.softAPConfig(apIp, apIp, IPAddress(255, 255, 255, 0));
    if (!WiFi.softAP(AP_SSID, AP_PASS)) {
        log("softAP failed");
        return;
    }
    // One radio for both: give Wi-Fi (live view) the larger share of airtime
    esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
    tcpServer.begin();
    startRelay();
    netStatus = "Wi-Fi: " + String(AP_SSID) + " up";
    log("AP " + String(AP_SSID) + " at " + WiFi.softAPIP().toString());
}

// TCP 7001: FrameTap's connection is mirrored to the camera's
static void pollNetwork() {
    WiFiClient c = tcpServer.accept();
    if (c && camIpAddr) {
        tapTcp = c;
        camTcp.stop();
        bool ok = camTcp.connect(IPAddress(camIpAddr), 7001, 2000);
        log("TCP 7001 from FrameTap -> camera " + String(ok ? "ok" : "FAILED"));
    }
    uint8_t buf[512];
    if (tapTcp && tapTcp.connected() && camTcp.connected()) {
        int n;
        while (tapTcp.available() && (n = tapTcp.read(buf, sizeof(buf))) > 0) camTcp.write(buf, n);
        while (camTcp.available() && (n = camTcp.read(buf, sizeof(buf))) > 0) tapTcp.write(buf, n);
    }
    // Quiet BLE while FrameTap is on Wi-Fi: no advertising, slow camera connection
    bool wifiLink = lastTapPacket && millis() - lastTapPacket < 3000;
    static bool quiet = false;
    // Experiment: once streaming is steady, switch Bluetooth off entirely so Wi-Fi
    // has the radio to itself. (Needs a reset to bring BLE back.)
    static bool bleOff = false;
    static uint32_t quietSince = 0;
    if (BLE_OFF_WHILE_STREAMING && quiet && !bleOff && millis() - quietSince > 5000) {
        bleOff = true;
        for (NimBLERemoteCharacteristic *&ch : camChars) ch = nullptr;
        for (NimBLECharacteristic *&ch : tapChars) ch = nullptr;
        camClient = nullptr;
        NimBLEDevice::deinit(true);
        camStatus = "Camera: Wi-Fi only (BLE off)";
        log("BLE switched OFF - Wi-Fi has the radio");
    }
    if (bleOff) {
        // skip all BLE handling below
    } else if (wifiLink && !tapConnected && !quiet) {
        quietSince = millis();
        quiet = true;
        NimBLEDevice::stopAdvertising();
        // 500 ms interval, 6 s supervision timeout (units: 1.25 ms / 10 ms)
        if (camClient && camClient->isConnected()) camClient->updateConnParams(400, 400, 0, 600);
        log("BLE quiet: advertising off, camera link slowed");
    } else if (quiet && (!wifiLink || tapConnected)) {
        quiet = false;
        if (camClient && camClient->isConnected()) camClient->updateConnParams(24, 40, 0, 400);
        if (!tapConnected) NimBLEDevice::startAdvertising();
        log("BLE normal again");
    } else if (!wifiLink && !tapConnected && !NimBLEDevice::getAdvertising()->isAdvertising()) {
        NimBLEDevice::startAdvertising();
    }

    static uint32_t lastCount = 0;
    if (millis() - lastCount > 1000) {
        uint32_t ms = millis() - lastCount;
        lastCount = millis();
        uint32_t toCam = bytesToCam, toTap = bytesToTap;
        bytesToCam = bytesToTap = 0;
        uint32_t fails = sendFails, biggest = maxPacket;
        uint32_t vp = videoPkts, vm = videoMissing, vo = videoOutOfOrder;
        sendFails = maxPacket = videoPkts = videoMissing = videoOutOfOrder = 0;
        uint32_t rd = ringDrops;
        ringDrops = 0;
        if (vp || vm)
            Serial.printf("VIDEO in %lu pkts, missing %lu, resent %lu, ring drops %lu\n", vp, vm, vo, rd);
        if (toCam || toTap)
            Serial.printf("RATE to cam %lu KB/s, to FrameTap %lu KB/s, send fails %lu, biggest %lu B\n",
                          toCam * 1000 / ms / 1024, toTap * 1000 / ms / 1024, fails, biggest);
        char line[48];
        snprintf(line, sizeof(line), "Wi-Fi relay: to cam %lu  to FrameTap %lu", udpToCam, udpToTap);
        if (String(line) != netStatus) {
            netStatus = line;
            needsDraw = true;
        }
    }
}

// ---------- camera bring-up (board's own session, before FrameTap) ----------

static std::vector<uint8_t> bringupBytes;
static Duml::Assembler bringupAsm;
static uint16_t cmdSeq = 0x9000;
static uint32_t lastWrite = 0;

static void sendOwn(const Duml::Frame &f) {
    uint32_t since = millis() - lastWrite;
    if (since < 250) delay(250 - since); // fff5 writes drop if not paced
    std::vector<uint8_t> b = Duml::encode(f);
    if (camChars[2]) camChars[2]->writeValue(b.data(), b.size(), false);
    lastWrite = millis();
}

static bool waitOwn(uint8_t set, uint8_t cmd, uint32_t timeoutMs, Duml::Frame &out) {
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        std::vector<uint8_t> bytes;
        portENTER_CRITICAL(&qLock);
        bytes.swap(bringupBytes);
        portEXIT_CRITICAL(&qLock);
        for (Duml::Frame &f : bringupAsm.append(bytes.data(), bytes.size())) {
            if (f.cmdSet == 0x07 && f.cmdId == 0x46 && f.flags != Duml::FLAG_RESPONSE) {
                log("camera approved pairing");
                sendOwn(Duml::pairApprovalAck(f.seq));
            }
            if (f.cmdSet == set && f.cmdId == cmd) {
                out = f;
                return true;
            }
        }
        delay(20);
    }
    return false;
}

// Pairs the board, then has the camera join the board's AP as a station
static bool cameraToStation() {
    bringupSink = &bringupBytes;
    bringupActive = true;
    uint8_t arm[2] = {0x01, 0x00};
    if (camChars[1]) camChars[1]->writeValue(arm, 2, true);
    Duml::Frame reply;
    sendOwn(Duml::sessionWake());
    sendOwn(Duml::setPairingPin("osmo"));
    if (!waitOwn(0x07, 0x45, 15000, reply)) log("no pairing reply (continuing)");
    else if (reply.payload.size() >= 2 && reply.payload[1] == 2) {
        camStatus = "Approve on the camera!";
        draw();
        waitOwn(0x07, 0x46, 90000, reply);
    }
    camStatus = "Camera: joining board Wi-Fi";
    draw();
    expectCamera = true;
    // The camera sometimes refuses a join (reply 01 FF); a fresh attempt usually works
    for (int attempt = 1; attempt <= 3 && camIpAddr == 0; attempt++) {
        sendOwn(Duml::stationMode(true, cmdSeq++));
        if (waitOwn(0x07, 0x48, 4000, reply)) log("STA reply " + Duml::hex(reply.payload, 4));
        delay(1500);
        sendOwn(Duml::joinWifi(AP_SSID, AP_PASS, cmdSeq++));
        if (waitOwn(0x07, 0x47, 12000, reply)) log("join reply " + Duml::hex(reply.payload, 4));
        uint32_t start = millis();
        while (camIpAddr == 0 && millis() - start < 15000) delay(100);
        if (camIpAddr == 0) log("join attempt " + String(attempt) + " failed");
    }
    bringupActive = false;
    if (camIpAddr == 0) {
        log("camera did not join the board AP");
        camStatus = "Camera: Wi-Fi join FAILED";
        return false;
    }
    log("camera on Wi-Fi at " + IPAddress(camIpAddr).toString());
    camStatus = "Camera: BLE + Wi-Fi up";
    needsDraw = true;
    return true;
}

// ---------- setup steps ----------

static NimBLEAddress camAddress;
static std::string tapName; // what FrameTap sees: the camera's name + "-R"

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
            // FrameTap's phone also advertises DJI data ("OsmoFrameTap-..."): match the name too
            if (m.size() >= 2 && ((uint8_t)m[0] | ((uint8_t)m[1] << 8)) == DJI_COMPANY_ID &&
                d->getName().rfind("OsmoPocket", 0) == 0) {
                camName = d->getName();
                camMfr = m;
                camAddress = d->getAddress();
                log("camera " + String(camName.c_str()) + " " + String(camAddress.toString().c_str()) + " type" +
                    String(camAddress.getBase()->type));
                log("mfr " + Duml::hex(std::vector<uint8_t>(m.begin(), m.end()), 32));
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
        // false: a refresh would delete the characteristics stored above
        for (NimBLERemoteService *s : camClient->getServices(false))
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
    scanResp.setName(tapName);
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
    log("advertising as " + String(tapName.c_str()));
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

    // The board keeps its own address for both roles, so nothing collides on air
    buildServer(); // GATT ready while idle (can't be added once busy)
    if (CAMERA_LINK) {
        scanForCamera();
        connectCamera();
        tapName = camName.substr(0, 20) + "-R";
        startNetwork();
        cameraToStation();
    } else {
        // Our own camera: the 4P's advert layout (AA 08 | model | MAC tail | status)
        // with the board's address in place of the camera's
        const uint8_t *mac = NimBLEDevice::getAddress().getVal(); // little-endian
        uint8_t mfr[] = {0xAA, 0x08, 0x00, 0x00, 0x00, 0xF5, 0x00, 0x04, mac[2], mac[1], mac[0],
                         0x20, 0xDA, 0x00, 0x00, 0x40};
        camMfr.assign((const char *)mfr, sizeof(mfr));
        tapName = "OsmoPocket4P-PRXY";
        camStatus = "Camera: not linked (own advert)";
        log("mfr " + Duml::hex(std::vector<uint8_t>(mfr, mfr + sizeof(mfr)), 32));
    }
    if (!CAMERA_LINK) startNetwork();
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
            forwardToCamera(p, idx);
        } else {
            if (tapConnected && tapChars[idx]) tapChars[idx]->notify(p.data.data(), p.data.size());
            framesToTap++;
            logFrames(p);
        }
    }
    pollNetwork();

    if (camClient && !camClient->isConnected() && camStatus != "Camera: LOST") {
        camStatus = "Camera: LOST";
        for (NimBLERemoteCharacteristic *&ch : camChars) ch = nullptr;
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
