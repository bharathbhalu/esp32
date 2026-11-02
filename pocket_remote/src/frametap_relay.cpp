// Ported from esp32/pocket_proxy, see frametap_relay.h.
#include "frametap_relay.h"
#include "duml.h"
#include <WiFi.h>
#include <esp_coexist.h>
#include <lwip/sockets.h>
#include <deque>

namespace Relay {

static const uint16_t DATALINK_PORT = 9004;

static bool started = false, enabled = true;
static String name;
static std::string mfr;
static const char *ssid = "", *pass = "";
static uint32_t camIp = 0; // network order

// ---------- BLE: FrameTap <-> camera ----------
// Callbacks run on the NimBLE task and only queue bytes; update() does the writes.

enum Dir : uint8_t { TO_CAMERA, TO_FRAMETAP };
struct Packet {
    Dir dir;
    uint16_t charId; // 0xfff3 / 0xfff4 / 0xfff5
    bool withResponse;
    std::vector<uint8_t> data;
};
static portMUX_TYPE qLock = portMUX_INITIALIZER_UNLOCKED;
static std::deque<Packet> queue;

static NimBLEClient *camClient = nullptr;
static NimBLERemoteCharacteristic *camChars[3] = {};
static NimBLEServer *server = nullptr;
static NimBLECharacteristic *tapChars[3] = {};
static volatile bool tapConnected = false;
static volatile uint32_t lastClaim = 0; // millis of FrameTap's last pairing/BLE/Wi-Fi activity
static Duml::Assembler fromTap[3], toTap[3];

// Pairing and Wi-Fi traffic (set 0x07) both ways, for diagnosing FrameTap's connect
static void logWifiFrames(const char *dir, const Duml::Frame &f) {
    if (f.cmdSet != 0x07) return;
    bool secret = f.cmdId == 0x0E;
    Serial.printf("relay %s 07/%02X fl%02X %s\n", dir, f.cmdId, f.flags,
                  secret ? "(hidden)" : Duml::hex(f.payload, 12).c_str());
}

static void enqueue(Dir dir, uint16_t charId, const uint8_t *data, size_t len, bool withResponse) {
    portENTER_CRITICAL(&qLock);
    if (queue.size() < 200) queue.push_back(Packet{dir, charId, withResponse, std::vector<uint8_t>(data, data + len)});
    portEXIT_CRITICAL(&qLock);
}

static uint16_t shortId(const NimBLEUUID &uuid) {
    String s = uuid.toString().c_str(); // "0xfff4" for 16-bit UUIDs
    return (uint16_t)strtol(s.c_str() + (s.startsWith("0x") ? 2 : 0), nullptr, 16);
}

class TapServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *, NimBLEConnInfo &info) override {
        tapConnected = true;
        Serial.printf("relay: FrameTap BLE connected %s\n", info.getAddress().toString().c_str());
    }
    void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int reason) override {
        tapConnected = false;
        Serial.printf("relay: FrameTap BLE disconnected (%d)\n", reason);
        // update() restarts advertising if FrameTap isn't on Wi-Fi
    }
};

class TapCharCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override {
        NimBLEAttValue v = c->getValue();
        uint16_t id = shortId(c->getUUID());
        enqueue(TO_CAMERA, id, v.data(), v.size(), id == 0xfff4); // fff4 arm write expects a response
    }
};

void prepare() {
    if (server) return; // bring-up runs again on every reconnect
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

bool onCameraNotify(uint16_t charId, const uint8_t *data, size_t len) {
    if (!started) return false;
    if (tapConnected && enabled) enqueue(TO_FRAMETAP, charId, data, len, false);
    return true; // nobody else reads the camera's BLE once the relay runs
}

// Our answer to a FrameTap request: same seq, sender/receiver swapped,
// flags as the camera does it (FrameTap's 0x20 requests get 0xA0)
static Duml::Frame replyTo(const Duml::Frame &req, std::vector<uint8_t> payload) {
    Duml::Frame r;
    r.sender = req.receiver;
    r.receiver = req.sender;
    r.seq = req.seq;
    r.flags = req.flags | 0x80;
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

// Outgoing BLE, only touched from update(). Sends fail with rc=6 when NimBLE's
// buffers are full; the packet then stays queued and is retried after a pause
// (retrying every loop starved Wi-Fi of airtime and stuttered live view).
// Relayed packets go stale after 2 s; our own replies to FrameTap never do.
struct Outgoing {
    bool toTap;
    int idx;
    bool withResponse;
    bool own;
    uint32_t queuedAt;
    std::vector<uint8_t> data;
};
static std::deque<Outgoing> outgoing;
static uint32_t retryAfter = 0;

static void send(bool toTap, int idx, const uint8_t *data, size_t len, bool withResponse = false, bool own = false) {
    if (outgoing.size() < 300)
        outgoing.push_back(Outgoing{toTap, idx, withResponse, own, millis(), std::vector<uint8_t>(data, data + len)});
}

// FrameTap left BLE: what it sent hasn't reached the camera and no longer should
static void dropFromTap() {
    for (auto it = outgoing.begin(); it != outgoing.end();) it = it->toTap ? it + 1 : outgoing.erase(it);
}

static void flushOutgoing() {
    if (millis() < retryAfter) return;
    for (int n = 0; n < 8 && !outgoing.empty(); n++) {
        Outgoing &o = outgoing.front();
        if (!o.own && millis() - o.queuedAt > 2000) {
            outgoing.pop_front();
            continue;
        }
        bool ok;
        if (o.toTap) {
            if (!tapConnected || !tapChars[o.idx]) ok = true; // nobody to send to: drop
            else ok = tapChars[o.idx]->notify(o.data.data(), o.data.size());
        } else {
            if (!camChars[o.idx] || !camClient || !camClient->isConnected()) ok = true;
            else ok = camChars[o.idx]->writeValue(o.data.data(), o.data.size(), o.withResponse);
        }
        if (!ok) {
            retryAfter = millis() + 50; // buffers full
            return;
        }
        outgoing.pop_front();
    }
}

static void notifyTap(const Duml::Frame &f) {
    std::vector<uint8_t> b = Duml::encode(f);
    send(true, 1, b.data(), b.size(), false, true);
}

// True if we answered the frame ourselves (it must not reach the camera)
static bool intercept(const Duml::Frame &f) {
    if (f.cmdSet != 0x07 || f.flags == Duml::FLAG_RESPONSE) return false;
    switch (f.cmdId) {
    // Pairing: the camera refuses (code 6) while the board's own session is open,
    // so answer "already paired" (code 1) ourselves and let FrameTap carry on
    case 0x45:
        lastClaim = millis();
        notifyTap(replyTo(f, {0x00, 0x01}));
        Serial.println("relay: FrameTap pairing - answered, board hands over");
        return true;
    case 0x07: notifyTap(replyTo(f, statusString(ssid))); return true;                 // SSID
    case 0x0E: notifyTap(replyTo(f, statusString(pass))); return true;                 // password
    case 0x2C: notifyTap(replyTo(f, {0x00, (uint8_t)WiFi.channel()})); return true;  // channel (camera says 149)
    case 0x47:                                                                         // join a network
    case 0x48: notifyTap(replyTo(f, {0x00})); return true;                            // AP/STA switch
    default:   return false;
    }
}

static void forwardToCamera(const Packet &p, int idx) {
    std::vector<Duml::Frame> frames = fromTap[idx].append(p.data.data(), p.data.size());
    if (frames.empty()) {
        // Non-DUML writes (the 01 00 arm on fff4) go through raw; otherwise wait for the rest
        if (p.data.size() <= 4) {
            fromTap[idx].reset();
            send(false, idx, p.data.data(), p.data.size(), p.withResponse);
        }
        return;
    }
    for (const Duml::Frame &f : frames) {
        logWifiFrames("T>C", f);
        if (intercept(f)) continue;
        std::vector<uint8_t> b = Duml::encode(f);
        send(false, idx, b.data(), b.size(), p.withResponse);
    }
}

// ---------- Wi-Fi: UDP 9004 ----------
// Two tasks share one socket: the receiver only empties lwIP's small UDP queue
// (6 packets by default) into a PSRAM ring, the sender forwards from it.
// Receiving and sending in one loop dropped whole bursts of live view.

static const int RING_SLOTS = 512;
struct Slot {
    uint16_t len;
    bool toCamera;
    uint8_t data[1600];
};
static Slot *ring = nullptr;
static QueueHandle_t ready = nullptr;
static int sock = -1;
static sockaddr_in tapAddr{}; // learned from FrameTap's first packet
static volatile uint32_t lastTapPacket = 0;
static volatile uint32_t bytesToCam = 0, bytesToTap = 0, videoPkts = 0, videoMissing = 0;
static volatile uint32_t tapCmds = 0; // FrameTap command packets (type 5) per second

// Header capture for working out where sequence numbers live (temporary):
// the receiver copies a few packets of each direction/type, update() prints them
struct Sample {
    uint32_t ms;
    bool toCamera;
    uint16_t len;
    uint8_t head[40];
};
static const int SAMPLE_SLOTS = 400;
static Sample *samples = nullptr;
static volatile uint16_t samplesWritten = 0;
static uint16_t samplesPrinted = 0;
static uint8_t sampleCount[2][8] = {};

static void sampleHeader(const Slot &slot) {
    if (!samples || slot.len < 8 || samplesWritten >= SAMPLE_SLOTS) return;
    uint8_t type = slot.data[6] & 7;
    uint8_t &count = sampleCount[slot.toCamera][type];
    if (count >= (type == 0x02 ? 10 : 30)) return;
    count++;
    Sample &s = samples[samplesWritten];
    s.ms = millis();
    s.toCamera = slot.toCamera;
    s.len = slot.len;
    memcpy(s.head, slot.data, min<size_t>(slot.len, sizeof(s.head)));
    samplesWritten = samplesWritten + 1;
}

static void printSamples() {
    for (int n = 0; n < 8 && samplesPrinted < samplesWritten; n++, samplesPrinted++) {
        const Sample &s = samples[samplesPrinted];
        char hex[3 * sizeof(s.head) + 1];
        int len = min<int>(s.len, sizeof(s.head));
        for (int i = 0; i < len; i++) sprintf(hex + 3 * i, "%02X ", s.head[i]);
        hex[3 * len] = 0;
        Serial.printf("SAMPLE %lu %s t%d len%u %s\n", s.ms, s.toCamera ? "T>C" : "C>T", s.head[6], s.len, hex);
    }
}

static void receiveTask(void *) {
    uint16_t next = 0;
    while (true) {
        Slot &slot = ring[next];
        sockaddr_in from{};
        socklen_t fromLen = sizeof(from);
        int n = recvfrom(sock, slot.data, sizeof(slot.data), 0, (sockaddr *)&from, &fromLen);
        if (n <= 0 || !enabled) continue;
        slot.len = n;
        slot.toCamera = from.sin_addr.s_addr != camIp;
        if (slot.toCamera) {
            tapAddr = from;
            lastTapPacket = millis();
        } else if (n >= 8 && slot.data[6] == 0x02) {
            // Video: seq (bytes 4-5) steps by 8
            static int32_t lastSeq = -1;
            uint16_t seq = slot.data[4] | (slot.data[5] << 8);
            int16_t d = (int16_t)(seq - (uint16_t)lastSeq);
            if (lastSeq >= 0 && d > 8 && d < 4000) videoMissing += d / 8 - 1;
            if (lastSeq < 0 || d > 0) lastSeq = seq;
            videoPkts++;
        }
        if (slot.toCamera && n >= 8 && slot.data[6] == 0x05) tapCmds++;
        sampleHeader(slot);
        if (xQueueSend(ready, &next, 0) != pdTRUE) continue; // ring full: reuse this slot
        next = (next + 1) % RING_SLOTS;
    }
}

// ---------- Command sequence sharing ----------
// The camera numbers each packet type separately. Only the command stream
// (type 5: seq in steps of 8, plus a per-command counter) is shared with the
// board. Camera number = FrameTap's number + 8 per board command injected
// before it, counter likewise + 1. Fields that echo command numbers back are
// shifted down again. Until the board injects, nothing is touched at all.
// Video and the camera's own numbering pass unchanged. Runs only in sendTask.
//
// Fields: t5 seq at 4-5 (in the xor) and 10-11, counter at 16; t4 short form
// pair at 24-27; t1 pair at 24-27; t3 ack at 8-9.

static const int INJ_SLOTS = 64;
static uint16_t injTapAfter[INJ_SLOTS]; // FrameTap's last command number when the board's went in
static uint16_t injCam[INJ_SLOTS];      // the board command's camera number
static int injCount = 0, injNext = 0;
static uint32_t injEvicted = 0;         // older injections, counted in full

static uint16_t tapSession = 0, tapLastSeq = 0, tapLastAck = 0, camLastSeq = 0;
static uint8_t camLastCounter = 0;
static bool tapSeen = false;
static volatile uint32_t xlatUp = 0, xlatDown = 0, injected = 0;

struct Injection {
    uint16_t len;
    uint8_t duml[256];
};
static QueueHandle_t injectQ = nullptr;

// Copies of the camera's status/reply datagrams to FrameTap, read in update()
struct Observed {
    uint16_t len;
    uint8_t data[1100];
};
static QueueHandle_t observeQ = nullptr;
static void (*observer)(const uint8_t *, size_t) = nullptr;

void setObserver(void (*fn)(const uint8_t *, size_t)) { observer = fn; }

static uint16_t rd16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static void wr16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }

static void resetShare() { injCount = injNext = 0, injEvicted = 0, tapSeen = false; }

// Numbers are 16-bit and FrameTap uses ~560 a second, so an injection compared
// against numbers half the range away (~58 s later) would flip sides. Once
// both streams are well past one it only ever counts in full.
static void foldOld() {
    while (injCount > 0) {
        int oldest = (injNext - injCount + INJ_SLOTS) % INJ_SLOTS;
        if ((int16_t)(tapLastSeq - injTapAfter[oldest]) < 0x2000 || (int16_t)(camLastSeq - injCam[oldest]) < 0x2000)
            break;
        injCount--;
        injEvicted++;
    }
}

// Board commands that precede FrameTap's command number v
static uint32_t boardsBeforeTap(uint16_t v) {
    uint32_t n = injEvicted;
    for (int i = 0; i < injCount; i++)
        if ((int16_t)((v & ~7) - injTapAfter[i]) > 0) n++;
    return n;
}

// Board commands at or before camera number v
static uint32_t boardsUpToCam(uint16_t v) {
    uint32_t n = injEvicted;
    for (int i = 0; i < injCount; i++)
        if ((int16_t)((v & ~7) - injCam[i]) >= 0) n++;
    return n;
}

static uint16_t tapToCam(uint16_t v) { return v + 8 * boardsBeforeTap(v); }
static uint16_t camToTap(uint16_t v) { return v - 8 * boardsUpToCam(v); }

static void translate(Slot &slot) {
    uint8_t *d = slot.data;
    if (slot.len < 8) return;
    uint8_t type = d[6];
    if (slot.toCamera) {
        if (type == 0x00 && slot.len >= 10) {
            // Handshake. FrameTap repeats it inside a session (e.g. on a mode
            // change); only a new session or base number restarts the sharing.
            static uint16_t hsSession = 0, hsBase = 0;
            uint16_t session = rd16(d + 2), base = rd16(d + 8);
            bool fresh = session != hsSession || base != hsBase;
            Serial.printf("relay: FrameTap handshake session %04X base %04X - %s\n", session, base,
                          fresh ? "new session, sharing reset" : "same session, offset kept");
            hsSession = session;
            hsBase = base;
            if (fresh) resetShare();
        }
        else if (type == 0x05 && slot.len >= 18) {
            uint16_t seq = rd16(d + 4);
            if (!tapSeen || (int16_t)((seq & ~7) - tapLastSeq) > 0) tapLastSeq = seq & ~7;
            tapSeen = true;
            tapSession = rd16(d + 2);
            tapLastAck = rd16(d + 8);
            foldOld();
            uint32_t n = injCount || injEvicted ? boardsBeforeTap(seq) : 0;
            if (n) {
                uint16_t cam = seq + 8 * n;
                wr16(d + 4, cam);
                if (rd16(d + 10) == seq) wr16(d + 10, cam);
                d[16] += n;
                d[7] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6];
                xlatUp++;
            }
            uint16_t cam = rd16(d + 4) & ~7;
            if ((int16_t)(cam - camLastSeq) >= 0) {
                camLastSeq = cam;
                camLastCounter = d[16];
            }
        } else if (type == 0x04 && slot.len == 34 && (injCount || injEvicted)) {
            wr16(d + 24, tapToCam(rd16(d + 24)));
            wr16(d + 26, tapToCam(rd16(d + 26)));
        }
    } else if (injCount || injEvicted) {
        // Reply to a board command: DUML (at byte 20) seq in the board's range
        if (type == 0x03 && slot.len > 30 && d[20] == 0x55 && d[27] >= 0xB0)
            Serial.printf("relay: camera answered board command %02X/%02X: %02X %02X %02X\n", d[29], d[30], d[31],
                          d[32], d[33]);
        if (type == 0x03 && slot.len >= 10) {
            wr16(d + 8, camToTap(rd16(d + 8)));
            xlatDown++;
        } else if (type == 0x01 && slot.len >= 28) {
            wr16(d + 24, camToTap(rd16(d + 24)));
            wr16(d + 26, camToTap(rd16(d + 26)));
            xlatDown++;
        }
    }
}

// A board command in FrameTap's session: next camera number and counter
static void sendInjection(const Injection &inj, const sockaddr_in &cam) {
    if (!tapSeen) return;
    uint16_t seq = camLastSeq + 8;
    uint8_t counter = camLastCounter + 1;
    uint8_t pkt[8 + 12 + sizeof(inj.duml)];
    size_t len = 8 + 12 + inj.len;
    wr16(pkt, 0x8000 | len);
    wr16(pkt + 2, tapSession);
    wr16(pkt + 4, seq);
    pkt[6] = 0x05;
    pkt[7] = pkt[0] ^ pkt[1] ^ pkt[2] ^ pkt[3] ^ pkt[4] ^ pkt[5] ^ pkt[6];
    wr16(pkt + 8, tapLastAck);
    wr16(pkt + 10, seq);
    memset(pkt + 12, 0, 4);
    pkt[16] = counter;
    pkt[17] = 0x01;
    pkt[18] = pkt[19] = 0;
    memcpy(pkt + 20, inj.duml, inj.len);
    sendto(sock, pkt, len, 0, (const sockaddr *)&cam, sizeof(cam));

    if (injCount == INJ_SLOTS) injEvicted++;
    else injCount++;
    injTapAfter[injNext] = tapLastSeq;
    injCam[injNext] = seq;
    injNext = (injNext + 1) % INJ_SLOTS;
    camLastSeq = seq;
    camLastCounter = counter;
    injected++;
    Serial.printf("relay: board command injected, cam seq %04X counter %u (FrameTap last %04X)\n", seq, counter,
                  tapLastSeq);
}

static void sendTask(void *) {
    uint16_t idx;
    sockaddr_in cam{};
    cam.sin_family = AF_INET;
    cam.sin_port = htons(DATALINK_PORT);
    cam.sin_addr.s_addr = camIp;
    Injection inj;
    while (true) {
        if (xQueueReceive(injectQ, &inj, 0) == pdTRUE) sendInjection(inj, cam);
        if (xQueueReceive(ready, &idx, pdMS_TO_TICKS(20)) != pdTRUE) continue;
        Slot &slot = ring[idx];
        if (!slot.toCamera && tapAddr.sin_port == 0) continue;
        translate(slot);
        if (!slot.toCamera && observer && observeQ && (slot.data[6] == 0x01 || slot.data[6] == 0x03) &&
            slot.len <= sizeof(Observed::data)) {
            static Observed o;
            o.len = slot.len;
            memcpy(o.data, slot.data, slot.len);
            xQueueSend(observeQ, &o, 0); // full: the board misses one status update
        }
        const sockaddr_in *to = slot.toCamera ? &cam : &tapAddr;
        sendto(sock, slot.data, slot.len, 0, (const sockaddr *)to, sizeof(*to));
        if (slot.toCamera) bytesToCam += slot.len;
        else bytesToTap += slot.len;
    }
}

bool inject(Duml::Frame f) {
    if (!started || !injectQ || !lastTapPacket || millis() - lastTapPacket > 3000) return false;
    static uint16_t dumlSeq = 0xB000; // own range, so replies to the board can be told apart
    f.seq = dumlSeq++;
    std::vector<uint8_t> b = Duml::encode(f);
    Injection inj;
    if (b.size() > sizeof(inj.duml)) return false;
    inj.len = b.size();
    memcpy(inj.duml, b.data(), b.size());
    return xQueueSend(injectQ, &inj, 0) == pdTRUE;
}

// TCP 7001: FrameTap's connection is mirrored to the camera's
static WiFiServer tcpServer(7001);
static WiFiClient tapTcp, camTcp;

static void pollTcp() {
    WiFiClient c = tcpServer.accept();
    if (c) {
        tapTcp = c;
        camTcp.stop();
        camTcp.connect(IPAddress(camIp), 7001, 2000);
    }
    uint8_t buf[512];
    int n;
    if (tapTcp && tapTcp.connected() && camTcp.connected()) {
        while (tapTcp.available() && (n = tapTcp.read(buf, sizeof(buf))) > 0) camTcp.write(buf, n);
        while (camTcp.available() && (n = camTcp.read(buf, sizeof(buf))) > 0) tapTcp.write(buf, n);
    }
}

// ---------- start / loop ----------

static void startAdvertising() {
    // flags + manufacturer data + fff0 in the advert, name in the scan response
    NimBLEAdvertisementData adv;
    adv.setFlags(0x06);
    adv.setManufacturerData(mfr);
    adv.setCompleteServices16({NimBLEUUID((uint16_t)0xFFF0)});
    NimBLEAdvertisementData scanResp;
    scanResp.setName(name.c_str());
    NimBLEAdvertising *a = NimBLEDevice::getAdvertising();
    a->setAdvertisementData(adv);
    a->setScanResponseData(scanResp);
    a->start();
}

void start(NimBLEClient *camera, NimBLERemoteCharacteristic *const chars[3], const std::string &camName,
           const std::string &camMfr, IPAddress cameraIp, const char *apSsid, const char *apPass) {
    camClient = camera;
    for (int i = 0; i < 3; i++) camChars[i] = chars[i];
    // "OsmoPocket4P-612D" -> "4P-612D-esp32-Relay"
    String n = camName.c_str();
    if (n.startsWith("OsmoPocket")) n = n.substring(10);
    name = n + "-esp32-Relay";
    mfr = camMfr;
    ssid = apSsid;
    pass = apPass;
    camIp = (uint32_t)cameraIp;
    if (started) { // reconnect: sockets and tasks already run
        if (enabled) startAdvertising();
        return;
    }

    // One radio for both: give Wi-Fi (live view) the larger share of airtime
    esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
    ring = (Slot *)ps_malloc(sizeof(Slot) * RING_SLOTS);
    samples = (Sample *)ps_malloc(sizeof(Sample) * SAMPLE_SLOTS);
    ready = xQueueCreate(RING_SLOTS - 2, sizeof(uint16_t)); // spare: the slot being sent + received into
    injectQ = xQueueCreate(4, sizeof(Injection));
    observeQ = xQueueCreate(12, sizeof(Observed));
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(DATALINK_PORT);
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(sock, (sockaddr *)&local, sizeof(local));
    int rcvBuf = 64 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvBuf, sizeof(rcvBuf));
    // Receiver above the sender so the lwIP queue is always drained first
    xTaskCreatePinnedToCore(receiveTask, "relayRx", 4096, nullptr, 12, nullptr, 1);
    xTaskCreatePinnedToCore(sendTask, "relayTx", 4096, nullptr, 11, nullptr, 1);
    tcpServer.begin();

    started = true;
    if (enabled) startAdvertising();
    Serial.printf("relay: advertising as %s\n", name.c_str());
}

static bool tapOnWifi() { return lastTapPacket && millis() - lastTapPacket < 3000; }

static uint32_t lastKbToTap = 0, lastKbToCam = 0, lastVideoPkts = 0, lastVideoMissing = 0;

void update() {
    if (!started) return;

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
        if (idx < 0 || idx > 2 || !enabled) continue;
        if (p.dir == TO_CAMERA) forwardToCamera(p, idx);
        else {
            for (const Duml::Frame &f : toTap[idx].append(p.data.data(), p.data.size())) logWifiFrames("C>T", f);
            send(true, idx, p.data.data(), p.data.size());
        }
    }
    static bool wasTapConnected = false;
    if (wasTapConnected && !tapConnected) dropFromTap();
    wasTapConnected = tapConnected;
    flushOutgoing();
    if (enabled) pollTcp();
    if (observeQ && observer) {
        static Observed o;
        for (int n = 0; n < 12 && xQueueReceive(observeQ, &o, 0) == pdTRUE; n++) observer(o.data, o.len);
    }
    printSamples();

    // Quiet BLE while FrameTap is on Wi-Fi: no advertising, slow camera connection
    static bool quiet = false;
    bool wifi = tapOnWifi();
    if (enabled && lastClaim && (wifi || tapConnected)) lastClaim = millis();
    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    if (wifi && !tapConnected && !quiet) {
        quiet = true;
        adv->stop();
        // 500 ms interval, 6 s supervision timeout (units: 1.25 ms / 10 ms)
        if (camClient && camClient->isConnected()) camClient->updateConnParams(400, 400, 0, 600);
    } else if (quiet && (!wifi || tapConnected)) {
        quiet = false;
        if (camClient && camClient->isConnected()) camClient->updateConnParams(24, 40, 0, 400);
    } else if (enabled && !wifi && !tapConnected && !adv->isAdvertising()) {
        startAdvertising();
    }

    static uint32_t lastStats = 0;
    if (millis() - lastStats >= 1000) {
        uint32_t ms = millis() - lastStats;
        lastStats = millis();
        lastKbToTap = bytesToTap * 1000 / ms / 1024;
        lastKbToCam = bytesToCam * 1000 / ms / 1024;
        lastVideoPkts = videoPkts;
        lastVideoMissing = videoMissing;
        bytesToTap = bytesToCam = videoPkts = videoMissing = 0;
        if (tapOnWifi())
            Serial.printf("relay: FrameTap Wi-Fi  to cam %lu KB/s, to FrameTap %lu KB/s, video %lu pkt/s, "
                          "cmds %lu, ble queue %u, injected %lu, xlat up %lu down %lu\n",
                          lastKbToCam, lastKbToTap, lastVideoPkts, tapCmds, (unsigned)outgoing.size(), injected,
                          xlatUp, xlatDown);
        xlatUp = xlatDown = tapCmds = 0;
    }
}

void setEnabled(bool on) {
    enabled = on;
    if (!started) return;
    if (!on) {
        NimBLEDevice::getAdvertising()->stop();
        if (server && tapConnected) server->disconnect(server->getPeerInfo(0).getConnHandle());
        outgoing.clear();
        tapTcp.stop();
        camTcp.stop();
        tapAddr = sockaddr_in{};
        lastTapPacket = 0;
        lastClaim = 0;
    }
}

bool claimed() { return started && enabled && lastClaim && millis() - lastClaim < 10000; }

Status status() {
    Status s;
    s.started = started;
    s.enabled = enabled;
    s.name = name;
    if (!started) return s;
    s.advertising = NimBLEDevice::getAdvertising()->isAdvertising();
    s.tapBle = tapConnected;
    s.tapWifi = tapOnWifi();
    s.claimed = claimed();
    s.tapIp = IPAddress(tapAddr.sin_addr.s_addr);
    s.kbToTap = lastKbToTap;
    s.kbToCam = lastKbToCam;
    s.videoPkts = lastVideoPkts;
    s.videoMissing = lastVideoMissing;
    return s;
}

} // namespace Relay
