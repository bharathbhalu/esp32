// Ported from OpenCine (Apache License 2.0); see duml.h.
#include "duml.h"

namespace Duml {

// Reflected CRC-8, init 0x77, poly 0x8C
uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t c = 0x77;
    for (size_t i = 0; i < len; i++) {
        c ^= data[i];
        for (int b = 0; b < 8; b++) c = (c & 1) ? (c >> 1) ^ 0x8C : c >> 1;
    }
    return c;
}

// Reflected CRC-16, init 0x3692, poly 0x8408
uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t c = 0x3692;
    for (size_t i = 0; i < len; i++) {
        c ^= data[i];
        for (int b = 0; b < 8; b++) c = (c & 1) ? (c >> 1) ^ 0x8408 : c >> 1;
    }
    return c;
}

std::vector<uint8_t> encode(const Frame &f) {
    size_t total = 13 + f.payload.size();
    std::vector<uint8_t> b;
    b.reserve(total);
    b.push_back(0x55);
    b.push_back(total & 0xFF);
    b.push_back((1 << 2) | ((total >> 8) & 0x03)); // version 1 + high length bits
    b.push_back(crc8(b.data(), 3));
    b.push_back(f.sender);
    b.push_back(f.receiver);
    b.push_back(f.seq & 0xFF);
    b.push_back(f.seq >> 8);
    b.push_back(f.flags);
    b.push_back(f.cmdSet);
    b.push_back(f.cmdId);
    b.insert(b.end(), f.payload.begin(), f.payload.end());
    uint16_t c = crc16(b.data(), b.size());
    b.push_back(c & 0xFF);
    b.push_back(c >> 8);
    return b;
}

std::vector<uint8_t> packString(const String &s) {
    std::vector<uint8_t> out;
    out.push_back((uint8_t)s.length());
    for (size_t i = 0; i < s.length(); i++) out.push_back((uint8_t)s[i]);
    return out;
}

String unpackStatusString(const std::vector<uint8_t> &p) {
    if (p.size() < 2) return "";
    size_t end = min(p.size(), (size_t)(2 + p[1]));
    String s;
    for (size_t i = 2; i < end; i++) s += (char)p[i];
    return s;
}

String hex(const std::vector<uint8_t> &b, size_t limit) {
    if (b.empty()) return "-";
    String s;
    char h[4];
    for (size_t i = 0; i < b.size() && i < limit; i++) {
        snprintf(h, sizeof(h), i ? " %02X" : "%02X", b[i]);
        s += h;
    }
    if (b.size() > limit) s += "...";
    return s;
}

std::vector<Frame> Assembler::append(const uint8_t *data, size_t len) {
    std::vector<Frame> out;
    if (len > 4096) {
        pending.clear();
        return out;
    }
    if (pending.size() + len > 4096) pending.clear();
    pending.insert(pending.end(), data, data + len);
    while (pending.size() >= 4) {
        if (pending[0] != 0x55 || (pending[2] >> 2) != 1 || crc8(pending.data(), 3) != pending[3]) {
            pending.erase(pending.begin());
            continue;
        }
        size_t length = pending[1] | ((pending[2] & 3) << 8);
        if (length < 13) {
            pending.erase(pending.begin());
            continue;
        }
        if (pending.size() < length) break;
        uint16_t checksum = pending[length - 2] | (pending[length - 1] << 8);
        if (crc16(pending.data(), length - 2) != checksum) {
            pending.erase(pending.begin());
            continue;
        }
        Frame f;
        f.sender = pending[4];
        f.receiver = pending[5];
        f.seq = pending[6] | (pending[7] << 8);
        f.flags = pending[8];
        f.cmdSet = pending[9];
        f.cmdId = pending[10];
        f.payload.assign(pending.begin() + 11, pending.begin() + length - 2);
        out.push_back(f);
        pending.erase(pending.begin(), pending.begin() + length);
    }
    return out;
}

static Frame make(uint8_t receiver, uint16_t seq, uint8_t set, uint8_t cmd, std::vector<uint8_t> payload,
                  uint8_t flags = FLAG_REQUEST) {
    Frame f;
    f.receiver = receiver;
    f.seq = seq;
    f.flags = flags;
    f.cmdSet = set;
    f.cmdId = cmd;
    f.payload = payload;
    return f;
}

Frame sessionWake() { return make(RX_SESSION, 0x802B, 0x00, 0x2B, {0x04, 0x00}); }
Frame sessionKeepalive() { return make(RX_SESSION, 0x802B, 0x00, 0x2B, {0x01, 0x01}); }

Frame setPairingPin(const char *pin) {
    std::vector<uint8_t> p = packString(IDENTIFIER);
    std::vector<uint8_t> pinBytes = packString(pin);
    p.insert(p.end(), pinBytes.begin(), pinBytes.end());
    return make(RX_WIFI, 0x8092, 0x07, 0x45, p);
}

Frame pairApprovalAck(uint16_t seq) { return make(RX_WIFI, seq, 0x07, 0x46, {0x00}, FLAG_RESPONSE); }
Frame wakeAccessPoint() { return make(RX_1C, 0x8053, 0x53, 0x10, {0, 0, 0, 0}); }
Frame getWifiSsid() { return make(RX_WIFI, 0x8007, 0x07, 0x07, {}); }
Frame getWifiPassword() { return make(RX_WIFI, 0x800E, 0x07, 0x0E, {}); }
Frame recordStart(uint16_t seq) { return make(RX_CAMERA, seq, 0x02, 0x02, {0x01}); }
Frame recordStop(uint16_t seq) { return make(RX_CAMERA, seq, 0x02, 0x02, {0x00}); }
Frame stationMode(bool enabled, uint16_t seq) { return make(RX_WIFI, seq, 0x07, 0x48, {(uint8_t)(enabled ? 1 : 0)}); }
Frame joinWifi(const String &ssid, const String &pass, uint16_t seq) {
    std::vector<uint8_t> p = packString(ssid);
    std::vector<uint8_t> pw = packString(pass);
    p.insert(p.end(), pw.begin(), pw.end());
    return make(RX_WIFI, seq, 0x07, 0x47, p);
}
Frame wifiWorkMode(uint16_t seq) { return make(RX_WIFI, seq, 0x07, 0x39, {0x00}); }

// Control commands: receiver = camera, seq filled in by Datalink::send
static Frame cam(uint8_t cmd, std::vector<uint8_t> payload) { return make(RX_CAMERA, 0, 0x02, cmd, payload); }
Frame shootPhoto() { return cam(0x01, {0x01}); }
Frame setMode(uint8_t raw) { return cam(0xE1, {raw}); }
Frame setExpoMode(uint8_t v) { return cam(0x1E, {v, 0x00}); }
Frame setIsoIndex(uint8_t idx) { return cam(0x2A, {idx}); }
Frame setShutterDenom(int denom) {
    uint16_t e = (uint16_t)denom | 0x8000;
    return cam(0x28, {0x01, (uint8_t)(e & 0xFF), (uint8_t)(e >> 8), 0x00, 0x00, 0x00, 0x40});
}
Frame setEvByte(uint8_t raw) { return cam(0x2E, {raw}); }
Frame setWhiteBalance(uint8_t mode, int kelvin, int tint) {
    int k = kelvin / 100;
    return cam(0x2C, {mode, (uint8_t)(k & 0xFF), (uint8_t)(k >> 8), (uint8_t)(tint & 0xFF), (uint8_t)(tint >> 8)});
}
Frame setColor(uint8_t raw) { return cam(0x42, {raw}); }
Frame setFocusMode(uint8_t v) { return cam(0x24, {v}); }
// 0x04/0x14 native timed target (OpenCine gimbalTimedTarget). Exact 8-byte layout:
// [yaw*10 i16-LE][00 00][nativePitch*10 i16-LE][0x05 abs mode][duration in 0.1s].
// Uses the NOTIFY flag (0x00), receiver = gimbal (type 4, id 0). duration 0.1-25.5 s.
Frame gimbalTarget(float yawDeg, float pitchDeg, uint32_t durationMs, float nativeDeg) {
    // Final safety clamp to the gimbal's reachable range (pan -48..225, tilt -44..70)
    yawDeg = constrain(yawDeg, -48.0f, 225.0f);
    pitchDeg = constrain(pitchDeg, -44.0f, 70.0f);
    // Display tilt -> native pitch: the gimbal reads ~180 deg at level, falling 1 deg
    // per 1 deg of look-up tilt. So level (display 0) is native 180, not 0, which is
    // why a pitch-0 pan was looking straight down. Wrap into (-180, 180].
    // Only an estimate: the reference moves (e.g. after a camera reset), so callers
    // pass one derived from the live attitude when they have it
    if (isnan(nativeDeg)) nativeDeg = 180.0f - pitchDeg;
    while (nativeDeg > 180.0f) nativeDeg -= 360.0f;
    while (nativeDeg <= -180.0f) nativeDeg += 360.0f;
    int16_t yaw = (int16_t)lroundf(yawDeg * 10);
    int16_t pitch = (int16_t)lroundf(nativeDeg * 10);
    uint8_t t = (uint8_t)constrain((long)(durationMs / 100), 1, 255); // tenths of a second
    Frame f = make((uint8_t)((0 << 5) | 0x04), 0, 0x04, 0x14, {}, FLAG_NOTIFY);
    f.payload = {(uint8_t)(yaw & 0xFF), (uint8_t)(yaw >> 8), 0x00, 0x00,
                 (uint8_t)(pitch & 0xFF), (uint8_t)(pitch >> 8), 0x05, t};
    return f;
}
// Relative zero target over 100 ms: cancels the current timed move.
Frame gimbalStick(int axis0, int axis1) {
    uint16_t a0 = constrain(axis0, 474, 1574), a1 = constrain(axis1, 474, 1574);
    Frame f = make((uint8_t)((0 << 5) | 0x04), 0, 0x04, 0x01, {}, FLAG_NOTIFY);
    f.payload = {(uint8_t)(a0 & 0xFF), (uint8_t)(a0 >> 8), 0x00, 0x00, (uint8_t)(a1 & 0xFF), (uint8_t)(a1 >> 8),
                 0x00, 0x80, 0x22, 0x00};
    return f;
}

Frame gimbalTiltLock(bool locked) {
    return make((uint8_t)((0 << 5) | 0x04), 0, 0x04, 0x50, {0x00, 0x04, 0x01, (uint8_t)(locked ? 1 : 0)});
}
Frame gimbalFollowSpeed(uint8_t speed) {
    return make((uint8_t)((0 << 5) | 0x04), 0, 0x04, 0x50, {0x00, 0x05, 0x01, speed});
}
Frame gimbalFlipCmd() { return make((uint8_t)((0 << 5) | 0x04), 0, 0x04, 0x4C, {0xFE, 0x09}); } // OpenCine gimbalFlip

Frame gimbalRecenterCmd() { return make((uint8_t)((0 << 5) | 0x04), 0, 0x04, 0x4C, {0xFE, 0x08}); }
Frame gimbalTimedStop() {
    Frame f = make((uint8_t)((0 << 5) | 0x04), 0, 0x04, 0x14, {0, 0, 0, 0, 0, 0, 0x04, 1}, FLAG_NOTIFY);
    return f;
}

Frame setVideoFormat(uint8_t resByte, uint8_t fpsIdx, uint8_t t0, uint8_t t1, uint8_t t2) {
    return cam(0x18, {resByte, fpsIdx, t0, t1, t2});
}
Frame param8E(uint8_t pidLo, uint8_t pidHi, uint8_t value) {
    return cam(0x8E, {0x01, 0x01, pidLo, pidHi, 0x01, value});
}
Frame gimbalModeSet(uint8_t a, uint8_t b) {
    return make((uint8_t)((0 << 5) | 0x04), 0, 0x04, 0x4C, {a, b});
}
Frame setTimer(const uint8_t p[6]) { return cam(0x4A, {p[0], p[1], p[2], p[3], p[4], p[5]}); }

} // namespace Duml
