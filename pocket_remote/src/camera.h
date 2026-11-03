// Camera interface the UI talks to. SimCamera implements it today; a real
// Pocket 4P implementation (BLE / Wi-Fi + DUML) will replace it later.
//
// Opcodes are from the OpenPocketCine handbook (Pocket 4 Pro captures):
// https://opencapture.org/openpocketcine/docs/
#pragma once
#include <Arduino.h>

// Order matters: profiles are applied in this order (mode and exposure first,
// since they decide which other settings exist).
enum SettingId : uint8_t {
    S_MODE, S_EXPOSURE, S_FORMAT, S_ISO, S_SHUTTER, S_EV, S_WB, S_COLOR,
    S_FOCUS, S_AUDIO, S_VOCAL, S_ASPECT, S_TIMER, S_GIMBAL_MODE, S_COUNT
};

struct SettingInfo {
    const char *name;
    const char *opcode; // DUML cmdset/cmdid (parameter id after ':')
};

inline constexpr SettingInfo SETTINGS[S_COUNT] = {
    {"Mode", "02/E1"},       {"Exposure", "02/1E"},  {"Format", "02/18"},
    {"ISO", "02/2A"},        {"Shutter", "02/28"},   {"EV", "02/2E"},
    {"White bal.", "02/2C"}, {"Color", "02/42"},     {"Focus", "02/24"},
    {"Audio", "02/8E:0020"}, {"Vocal boost", "02/8E:004C"},
    {"Aspect", "02/12"},     {"Timer", "02/4A"},     {"Gimbal mode", "04/4C"},
};

// Reachable gimbal range (degrees), from the handbook's Motion Control notes:
// pan -48..+225, tilt -44 (down) .. +70 (up)
inline constexpr float GIMBAL_YAW_MIN = -48, GIMBAL_YAW_MAX = 225;
inline constexpr float GIMBAL_PITCH_MIN = -44, GIMBAL_PITCH_MAX = 70;

enum class Transport : uint8_t { Bluetooth, WiFi };
enum class Link : uint8_t { Disconnected, Connecting, Connected };

class Camera {
public:
    virtual ~Camera() {}

    virtual void connect(Transport t) = 0;
    virtual void disconnect() = 0;
    virtual Link link() const = 0;
    virtual Transport transport() const = 0;

    // Settings: values are indexes into the camera-reported option list,
    // which can change with mode (e.g. Format in Video vs Slow Motion)
    virtual bool available(SettingId s) const = 0;
    virtual int optionCount(SettingId s) const = 0;
    virtual const char *optionLabel(SettingId s, int i) const = 0;
    virtual int value(SettingId s) const = 0;
    virtual void set(SettingId s, int index) = 0;                 // queued until acked
    virtual void setByLabel(SettingId s, const char *label) = 0;  // resolved when sent
    virtual bool pending(SettingId s) const = 0;
    virtual int queued() const = 0;

    virtual void shutter() = 0; // record start/stop in video modes, take photo otherwise
    virtual bool recording() const = 0;
    virtual uint32_t recordingSeconds() const = 0;
    virtual int battery() const = 0;
    virtual int storageUsed() const = 0; // % of the camera's card in use

    // Gimbal speed in -1..1 per axis; 0,0 stops (and cancels a timed move)
    virtual void gimbalSpeed(float yaw, float pitch) = 0;
    // Timed move to an absolute angle (real camera: 04/14 target, 0.1 s resolution)
    virtual void gimbalMoveTo(float yaw, float pitch, uint32_t durationMs) = 0;
    virtual void gimbalRecenter() = 0;
    virtual float gimbalYaw() const = 0;
    virtual float gimbalPitch() const = 0;

    // Camera power, per DJI's Osmo BLE protocol (documented for Action 4 / 5 Pro;
    // unverified on Pocket 4P). Works from sleep only, not full power-off, and only
    // within 30 min of sleeping, for a remote that has connected before.
    virtual bool asleep() const = 0;
    virtual void sleep() = 0;    // power mode 00/1A = 3
    virtual void wake() = 0;     // 2 s "WKP"+MAC advert, then reconnect
    virtual void snapshot() = 0; // wake advert + SNAPSHOT key (00/11, 0x03); camera re-sleeps after
    virtual const char *powerActivity() = 0; // "Waking...", "Snapshot: ..." or nullptr

    virtual void update() = 0;
    // Returns the latest message for the user (errors, confirmations) once, or nullptr
    virtual const char *takeMessage() = 0;

    const char *label(SettingId s) const { return optionLabel(s, value(s)); }

    bool isPhotoMode() const {
        const char *m = label(S_MODE);
        return strcmp(m, "Photo") == 0 || strcmp(m, "Live Photo") == 0 || strcmp(m, "Panorama") == 0;
    }
};
