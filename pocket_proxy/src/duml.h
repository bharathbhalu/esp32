// DUML framing for DJI Osmo cameras.
// Ported from OpenCine's Sources/OpenCineDJI/DumlFrame.swift and
// DumlNotificationAssembler.swift (https://github.com/erik-sutton95/OpenCine),
// Apache License 2.0. Byte layout, CRCs and command builders follow that code.
//
// Frame (total = 13 + payload):
//   55 | len_lo | (ver<<2)|len_hi | crc8(bytes 0..2) |
//   sender | receiver | seq:u16le | flags | cmdSet | cmdId | payload | crc16:u16le
#pragma once
#include <Arduino.h>
#include <vector>

namespace Duml {

static const uint8_t SENDER_APP = 0x02;  // id 0, type 2
static const uint8_t RX_CAMERA = 0x01;   // id 0, type 1
static const uint8_t RX_WIFI = 0x07;     // id 0, type 7
static const uint8_t RX_SESSION = 0xF0;  // session / heartbeat
static const uint8_t RX_1C = 0x1C;       // wake (0x53/0x10)

static const uint8_t FLAG_REQUEST = 0x40;
static const uint8_t FLAG_RESPONSE = 0xC0;
static const uint8_t FLAG_NOTIFY = 0x00;

struct Frame {
    uint8_t sender = SENDER_APP, receiver = 0;
    uint16_t seq = 0;
    uint8_t flags = FLAG_REQUEST, cmdSet = 0, cmdId = 0;
    std::vector<uint8_t> payload;
};

uint8_t crc8(const uint8_t *data, size_t len);
uint16_t crc16(const uint8_t *data, size_t len);
std::vector<uint8_t> encode(const Frame &f);
std::vector<uint8_t> packString(const String &s); // [len][utf8]
String unpackStatusString(const std::vector<uint8_t> &p); // [status][len][utf8...]
String hex(const std::vector<uint8_t> &b, size_t limit = 24);

// Reassembles frames from one ordered byte stream (BLE notifications don't
// line up with frame boundaries)
class Assembler {
public:
    std::vector<Frame> append(const uint8_t *data, size_t len);
    void reset() { pending.clear(); }

private:
    std::vector<uint8_t> pending;
};

// ---- commands (Commands.swift) ----
// App identity the camera remembers its pairing approval against
static const char *IDENTIFIER = "e5b2c0d1a47f43e8b1c6f2a9d3e70b51";

Frame sessionWake();                          // 0x00/0x2B 04 00
Frame sessionKeepalive();                     // 0x00/0x2B 01 01
Frame setPairingPin(const char *pin);         // 0x07/0x45
Frame pairApprovalAck(uint16_t seq);          // 0x07/0x46 response
Frame wakeAccessPoint();                      // 0x53/0x10
Frame getWifiSsid();                          // 0x07/0x07
Frame getWifiPassword();                      // 0x07/0x0E
Frame recordStart(uint16_t seq);              // 0x02/0x02 01 -> camera
Frame recordStop(uint16_t seq);               // 0x02/0x02 00 -> camera
Frame stationMode(bool enabled, uint16_t seq); // 0x07/0x48 -> wifi: switch AP/STA
Frame joinWifi(const String &ssid, const String &pass, uint16_t seq); // 0x07/0x47
Frame wifiWorkMode(uint16_t seq);             // 0x07/0x39 -> read AP/STA

} // namespace Duml
