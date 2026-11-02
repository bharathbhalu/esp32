// Ported from OpenCine (Apache License 2.0); see datalink.h.
#include "datalink.h"

namespace {

const uint16_t CAMERA_PORT = 9004;
const uint8_t PKT_HANDSHAKE = 0x00, PKT_TELEMETRY = 0x01, PKT_VIDEO = 0x02, PKT_ACKED_DATA = 0x03,
              PKT_ACK = 0x04, PKT_COMMAND = 0x05;

uint16_t u16(const uint8_t *p) { return p[0] | (p[1] << 8); }
uint32_t u32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }

// 8-byte transport header: [u16le 0x8000|total][u16le session][u16le seq][pktType][xor of the 7]
std::vector<uint8_t> transportHeader(uint8_t pktType, size_t payloadLen, uint16_t session, uint16_t seq) {
    uint16_t w0 = 0x8000 | ((8 + payloadLen) & 0x3FFF);
    std::vector<uint8_t> b = {(uint8_t)(w0 & 0xFF), (uint8_t)(w0 >> 8), (uint8_t)(session & 0xFF),
                              (uint8_t)(session >> 8), (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8), pktType};
    uint8_t x = 0;
    for (uint8_t v : b) x ^= v;
    b.push_back(x);
    return b;
}

// 12-byte routing header for command packets
std::vector<uint8_t> routingHeader(uint16_t seq, uint8_t counter) {
    uint16_t ack = seq - 8;
    return {(uint8_t)(ack & 0xFF), (uint8_t)(ack >> 8), (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8), 0, 0, 0, 0,
            counter, 0x01, 0x00, 0x00};
}

// 40-byte handshake payload (window 100, MTU 1472); first two bytes = baseSeq
std::vector<uint8_t> handshakePayload(uint16_t baseSeq) {
    std::vector<uint8_t> p = {0x00, 0x00, 0x64, 0x00, 0x64, 0x00, 0xC0, 0x05, 0x14, 0x00, 0x00, 0x64, 0x00, 0x00,
                              0x01, 0x90, 0x01, 0xC0, 0x05, 0x14, 0x00, 0x00, 0x64, 0x00, 0x14, 0x00, 0x64, 0x00,
                              0xC0, 0x05, 0x14, 0x00, 0x00, 0x64, 0x00, 0x01, 0x01, 0x04, 0x01, 0x02};
    p[0] = baseSeq & 0xFF;
    p[1] = baseSeq >> 8;
    return p;
}

// Every CRC-valid DUML frame inside a datagram (frames sit after the wrapper headers)
std::vector<Duml::Frame> scanFrames(const uint8_t *raw, size_t n) {
    std::vector<Duml::Frame> out;
    for (size_t i = 0; i + 13 <= n; i++) {
        if (raw[i] != 0x55) continue;
        size_t total = raw[i + 1] | ((raw[i + 2] & 0x03) << 8);
        if ((raw[i + 2] >> 2) != 1 || total < 13 || i + total > n) continue;
        if (Duml::crc8(raw + i, 3) != raw[i + 3]) continue;
        if (Duml::crc16(raw + i, total - 2) != u16(raw + i + total - 2)) continue;
        Duml::Frame f;
        f.sender = raw[i + 4];
        f.receiver = raw[i + 5];
        f.seq = u16(raw + i + 6);
        f.flags = raw[i + 8];
        f.cmdSet = raw[i + 9];
        f.cmdId = raw[i + 10];
        f.payload.assign(raw + i + 11, raw + i + total - 2);
        out.push_back(f);
    }
    return out;
}

uint8_t rx(uint8_t type, uint8_t id) { return (id << 5) | type; }

Duml::Frame frame(uint8_t receiver, uint8_t flags, uint8_t set, uint8_t cmd, std::vector<uint8_t> payload) {
    Duml::Frame f;
    f.receiver = receiver;
    f.flags = flags;
    f.cmdSet = set;
    f.cmdId = cmd;
    f.payload = payload;
    return f;
}

// 0x00/0x81 device info -> DM368 (type 8, id 2), flags 0x80
Duml::Frame appDeviceInfo() {
    std::vector<uint8_t> b(62, 0);
    b[1] = 0x41;
    b[2] = 0x50;
    b[3] = 0x50;
    b[41] = 0x02;
    b[50] = 0x02;
    b[51] = 0x08;
    return frame(rx(0x08, 2), 0x80, 0x00, 0x81, b);
}

// 0x00/0x88 app presence -> DM368 (type 8, id 1); re-sent ~1 Hz
Duml::Frame appPresence() {
    return frame(rx(0x08, 1), Duml::FLAG_REQUEST, 0x00, 0x88,
                 {0x17, 0x00, 0x46, 0x23, 0x7C, 0x41, 0x50, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02});
}

Duml::Frame gimbalInit() { return frame(rx(0x03, 0), Duml::FLAG_REQUEST, 0x03, 0xDA, {0x05, 0xFF, 0xFF, 0xFF, 0xFF}); }

// 0x00/0x99 subscribe: [02 02 00 00][subId u32][00 00 00][innerLen u16][nameLen u16][name][00 00 00 00]
Duml::Frame subscribeKey(const char *name, uint32_t subId) {
    size_t n = strlen(name);
    std::vector<uint8_t> p = {0x02, 0x02, 0x00, 0x00, (uint8_t)subId, (uint8_t)(subId >> 8), (uint8_t)(subId >> 16),
                              (uint8_t)(subId >> 24), 0, 0, 0, (uint8_t)((n + 6) & 0xFF), (uint8_t)((n + 6) >> 8),
                              (uint8_t)(n & 0xFF), (uint8_t)(n >> 8)};
    p.insert(p.end(), name, name + n);
    p.insert(p.end(), {0, 0, 0, 0});
    return frame(rx(0x08, 1), Duml::FLAG_REQUEST, 0x00, 0x99, p);
}

const char *const SUBSCRIPTION_KEYS[] = {
    "camcap_mode_profile", "camcap_video_format", "camcap_fov", "camcap_iso", "camcap_shutter",
    "camcap_photo_storage_format", "camcap_color_mode", "cam_storage", "cam_status", "timecode_info",
    "cam_expo_param", "cam_video_param_v2", "cam_record_time", "cam_image_effect", "cam_lens_state",
    "cam_fov", "cam_audio_status_v2",
};

} // namespace

bool Datalink::open(const IPAddress &camera, void (*log)(const String &)) {
    camera_ = camera;
    log_ = log;
    sessionId_ = random(0x1000, 0xFFFE);
    baseSeq_ = random(0x1000, 0xF000) & 0xFFF8; // 8-aligned, fresh per connect
    udpSeq_ = 0;
    dumlSeq_ = 0xA000;
    cmdCounter_ = 0;
    sawAcked_ = sawExtra_ = false;

    // TCP 7001 poke: a raw SetPairingPIN frame; keep the socket open all session
    if (poke_.connect(camera_, 7001, 2000)) {
        std::vector<uint8_t> pin = Duml::encode(Duml::setPairingPin("osmo"));
        poke_.write(pin.data(), pin.size());
        delay(400);
        log_("TCP 7001 poke ok");
    } else {
        log_("TCP 7001 poke failed (trying UDP anyway)");
    }

    udp_.begin(0); // ephemeral local port; the camera's 9004 is the remote
    // Handshake: repeat until the camera answers with a 0x00 packet and the
    // initial 34-byte telemetry window (which seeds our command sequence)
    bool acked = false;
    int window = -1;
    for (int attempt = 0; attempt < 12 && !(acked && window >= 0); attempt++) {
        std::vector<uint8_t> payload = handshakePayload(baseSeq_);
        std::vector<uint8_t> pkt = transportHeader(PKT_HANDSHAKE, payload.size(), sessionId_, udpSeq_);
        pkt.insert(pkt.end(), payload.begin(), payload.end());
        udp_.beginPacket(camera_, CAMERA_PORT);
        udp_.write(pkt.data(), pkt.size());
        udp_.endPacket();
        udpSeq_ += 8;
        uint32_t start = millis();
        while (millis() - start < 350) {
            int len = udp_.parsePacket();
            if (len <= 0) {
                delay(5);
                continue;
            }
            uint8_t buf[1600];
            len = udp_.read(buf, sizeof(buf));
            if (len >= 8 && buf[6] == PKT_HANDSHAKE) acked = true;
            if (len == 34 && buf[6] == PKT_TELEMETRY && window < 0) window = (u16(buf + 8) + 8) & 0xFFFF;
            if (acked && window >= 0) break;
        }
    }
    if (!acked || window < 0) {
        log_(String("handshake failed (ack ") + acked + ", window " + (window >= 0) + ")");
        udp_.stop();
        poke_.stop();
        return false;
    }
    udpSeq_ = window;
    open_ = true;
    log_("UDP handshake ok, session " + String(sessionId_, HEX));
    sendAck();
    registerApp();
    subscribe();
    lastRegister_ = millis();
    return true;
}

void Datalink::close() {
    open_ = false;
    udp_.stop();
    poke_.stop();
}

void Datalink::sendRaw(uint8_t pktType, const std::vector<uint8_t> &payload) {
    std::vector<uint8_t> pkt = transportHeader(pktType, payload.size(), sessionId_, udpSeq_);
    pkt.insert(pkt.end(), payload.begin(), payload.end());
    udp_.beginPacket(camera_, CAMERA_PORT);
    udp_.write(pkt.data(), pkt.size());
    udp_.endPacket();
    udpSeq_ += 8;
}

void Datalink::send(Duml::Frame f) {
    if (!open_) { // session handed to FrameTap: its session carries ours too
        if (forward_) forward_(f);
        return;
    }
    f.seq = dumlSeq_++;
    cmdCounter_++;
    std::vector<uint8_t> routing = routingHeader(udpSeq_, cmdCounter_);
    std::vector<uint8_t> duml = Duml::encode(f);
    routing.insert(routing.end(), duml.begin(), duml.end());
    sendRaw(PKT_COMMAND, routing);
}

// pktType 0x04 window ACK (seq 0): three duplicated-u16 groups + 2 zero bytes
void Datalink::sendAck() {
    auto grp = [](std::vector<uint8_t> &p, uint16_t v) {
        p.insert(p.end(), {(uint8_t)(v & 0xFF), (uint8_t)(v >> 8), (uint8_t)(v & 0xFF), (uint8_t)(v >> 8), 0, 0, 0, 0});
    };
    std::vector<uint8_t> p;
    grp(p, videoCursor_);
    grp(p, sawAcked_ ? ackedCursor_ : baseSeq_);
    grp(p, sawExtra_ ? extraCursor_ : baseSeq_);
    p.push_back(0);
    p.push_back(0);
    std::vector<uint8_t> pkt = transportHeader(PKT_ACK, p.size(), sessionId_, 0);
    pkt.insert(pkt.end(), p.begin(), p.end());
    udp_.beginPacket(camera_, CAMERA_PORT);
    udp_.write(pkt.data(), pkt.size());
    udp_.endPacket();
}

void Datalink::registerApp() {
    send(appDeviceInfo());
    sendAck();
    send(appPresence());
    sendAck();
    send(gimbalInit());
    sendAck();
}

void Datalink::subscribe() {
    uint32_t subId = 0x69DF;
    for (const char *key : SUBSCRIPTION_KEYS) send(subscribeKey(key, subId++));
    sendAck();
}

static bool isForward(uint16_t candidate, uint16_t current) {
    uint16_t d = candidate - current;
    return d != 0 && d < 0x8000;
}

void Datalink::handleDatagram(const uint8_t *data, size_t len) {
    if (len < 8) return;
    uint8_t type = data[6];
    uint16_t seq = u16(data + 4);
    if (type == PKT_VIDEO) videoCursor_ = seq;
    if (type == PKT_ACKED_DATA && (!sawAcked_ || isForward(seq, ackedCursor_))) {
        ackedCursor_ = seq;
        sawAcked_ = true;
    }
    if (type == PKT_TELEMETRY && len >= 34) {
        uint16_t acked = u16(data + 18);
        if (!sawAcked_ || isForward(acked, ackedCursor_)) {
            ackedCursor_ = acked;
            sawAcked_ = true;
        }
        extraCursor_ = u16(data + 26);
        sawExtra_ = true;
    }
    if (type == PKT_VIDEO) return; // no live view here
    for (const Duml::Frame &f : scanFrames(data, len)) {
        // Log command replies (camera's ACK to our SETs): cmdSet 0x02/0x04, response flag
        if (f.cmdSet == 0x02 && (f.flags == 0xC0 || f.flags == 0x80))
            Serial.printf("  reply %02X/%02X fl%02X %s\n", f.cmdSet, f.cmdId, f.flags,
                          f.payload.empty() ? "-" : String(f.payload[0], 16).c_str());
        apply(f);
    }
}

void Datalink::observe(const uint8_t *data, size_t len) {
    if (len < 8 || data[6] == PKT_VIDEO) return;
    // Pushes only: replies to FrameTap's own requests carry a status byte
    // first, so the same decoder reads them wrong (battery flickered to 0)
    for (const Duml::Frame &f : scanFrames(data, len))
        if (!(f.flags & 0x80)) apply(f);
}

// Status decoding from OpenCine's CameraStatus+DJIWire.swift
void Datalink::apply(const Duml::Frame &f) {
    const std::vector<uint8_t> &p = f.payload;
    if (f.cmdSet == 0x02 && f.cmdId == 0x80 && p.size() >= 13) {
        uint32_t flags = u32(p.data());
        state_.recording = flags & 0x80;
        uint32_t total = u32(p.data() + 5), freeMb = u32(p.data() + 9);
        if (total >= 1 && total <= 50000000) state_.storageTotalMb = total;
        if (freeMb <= 50000000) state_.storageFreeMb = freeMb;
        if (p.size() >= 31) state_.recordSeconds = u16(p.data() + 29);
        if (p.size() >= 58) state_.shootingMode = p[57];
        state_.lastStatusMs = millis();
    } else if (f.cmdSet == 0x0D && f.cmdId == 0x02 && p.size() >= 21) {
        if (p[20] <= 100) state_.batteryPercent = p[20];
        if (p.size() >= 34) state_.charging = p[32] == 1;
        state_.lastStatusMs = millis();
    }
}

void Datalink::poll() {
    if (!open_) return;
    // Drain everything that arrived
    for (int i = 0; i < 20; i++) {
        int len = udp_.parsePacket();
        if (len <= 0) break;
        static uint8_t buf[1600];
        len = udp_.read(buf, sizeof(buf));
        handleDatagram(buf, len);
    }
    uint32_t now = millis();
    if (now - lastAck_ >= 25) { // 40 Hz window ACK pump
        lastAck_ = now;
        sendAck();
    }
    if (now - lastRegister_ >= 1000) { // keep the session (and video) alive
        lastRegister_ = now;
        send(appDeviceInfo());
        send(appPresence());
    }
}
