// Wi-Fi control link to a DJI Osmo camera: TCP 7001 "poke", then DUML over UDP
// 9004 with DJI's transport/routing headers, handshake, window ACKs (40 Hz) and
// app registration (1 Hz).
// Ported from OpenCine's DumlTransport.swift, Commands.swift and the datalink
// driver in DjiAppleDatalink.swift (https://github.com/erik-sutton95/OpenCine),
// Apache License 2.0.
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <vector>
#include "duml.h"

struct CameraState {
    int batteryPercent = -1;
    bool charging = false;
    bool recording = false;
    uint32_t recordSeconds = 0;
    int storageTotalMb = -1, storageFreeMb = -1;
    int shootingMode = -1; // raw byte from 02/80
    // Current settings from the camera's subscription pushes (-1 = not reported yet)
    int expoMode = -1;     // 0x01 auto, 0x04 manual
    int isoRaw = -1;       // ISO index byte (0 in auto)
    int shutterDenom = -1; // configured 1/x in manual
    int evByte = -1;       // 0x10 = 0 EV, 1/3 stop steps
    int wbMode = -1;       // 0x00 auto, 0x06 custom
    int wbKelvin = -1;
    int colorRaw = -1;     // color profile byte
    // Gimbal attitude from the 50-byte 0x04/0x05 push (OpenCine/OpenPocketCine layout)
    float gimbalYaw = 0, gimbalTilt = 0, gimbalNative = 0; // degrees: joint pan, display tilt (up +), native pitch
    uint32_t gimbalMs = 0;
    int gimbalFamily = -1; // attitude byte 6 top bits: 0 Direction Lock, 1 FPV, 2 Follow family
    uint32_t lastStatusMs = 0;
};

class Datalink {
public:
    // Joins nothing itself: call once the board is on the camera's Wi-Fi
    bool open(const IPAddress &camera, void (*log)(const String &));
    void close();
    bool isOpen() const { return open_; }

    void send(Duml::Frame f);  // stamps seq/counter and wraps it
    void poll();               // receive, ACK pump, keepalive; call often

    // While closed, send() hands commands to this instead (FrameTap's shared session)
    void setForwarder(bool (*fn)(Duml::Frame)) { forward_ = fn; }
    // Status from camera datagrams that were meant for someone else
    void observe(const uint8_t *data, size_t len);
    const CameraState &state() const { return state_; }

private:
    void sendRaw(uint8_t pktType, const std::vector<uint8_t> &payload);
    void sendAck();
    void handleDatagram(const uint8_t *data, size_t len);
    void apply(const Duml::Frame &f);
    void registerApp();
    void subscribe();

    WiFiUDP udp_;
    WiFiClient poke_; // TCP 7001, kept open for the session
    IPAddress camera_;
    bool open_ = false;
    void (*log_)(const String &) = nullptr;
    bool (*forward_)(Duml::Frame) = nullptr;

    uint16_t sessionId_ = 0, baseSeq_ = 0, udpSeq_ = 0, dumlSeq_ = 0xA000;
    uint8_t cmdCounter_ = 0;
    uint16_t videoCursor_ = 0, ackedCursor_ = 0, extraCursor_ = 0;
    bool sawAcked_ = false, sawExtra_ = false;
    uint32_t lastAck_ = 0, lastRegister_ = 0;
    CameraState state_;
};
