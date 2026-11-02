// FrameTap relay: lets the FrameTap app control the camera through the board
// while the board's own remote keeps working.
//
//   FrameTap <--BLE--> board ("<cam>-esp32-Relay", own address) <--BLE--> Pocket 4P
//   FrameTap --Wi-Fi--> board AP 192.168.2.1 <--Wi-Fi-- Pocket 4P (station)
//
// FrameTap's BLE traffic goes through to the camera, except Wi-Fi requests:
// SSID, password and channel are answered with the board's AP, and join /
// station-mode commands are acked here so the camera stays on the board.
// FrameTap then connects to 192.168.2.1 (where the camera's own hotspot would
// be) and its UDP 9004 / TCP 7001 traffic is relayed unchanged to the camera.
#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

namespace Relay {

// Builds the GATT server; call right after NimBLEDevice::init (the stack must be idle)
void prepare();

// Starts advertising and the Wi-Fi relay once the camera has joined the AP.
// chars: the camera's fff3, fff4, fff5 (any may be null).
void start(NimBLEClient *camera, NimBLERemoteCharacteristic *const chars[3], const std::string &camName,
           const std::string &camMfr, IPAddress cameraIp, const char *apSsid, const char *apPass);

// Camera BLE notification; returns true if the relay took it
bool onCameraNotify(uint16_t charId, const uint8_t *data, size_t len);

// Call from the loop: BLE forwarding, TCP mirroring, stats
void update();

void setEnabled(bool on);

// True while FrameTap holds the camera (pairing, or on Wi-Fi, within the last 10 s).
// The camera serves one control session, so the board's own must close meanwhile.
bool claimed();

struct Status {
    bool started = false, enabled = false;
    bool advertising = false, tapBle = false, tapWifi = false, claimed = false;
    String name;
    IPAddress tapIp;
    uint32_t kbToTap = 0, kbToCam = 0;      // last second
    uint32_t videoPkts = 0, videoMissing = 0; // last second
};
Status status();

} // namespace Relay
