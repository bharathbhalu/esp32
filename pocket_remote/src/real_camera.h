// Real Pocket 4P behind the Camera interface. Brings the link up in a background
// task (BLE pair -> board's own 2.4 GHz AP -> camera joins as station -> Wi-Fi
// control link), reports live status, and sends real commands over Wi-Fi.
// BLE cannot control the 4P (verified); only Wi-Fi datalink does.
#pragma once
#include "camera.h"
#include "datalink.h"
#include <vector>

class RealCamera : public Camera {
public:
    void connect(Transport t) override;
    void disconnect() override;
    Link link() const override { return link_; }
    Transport transport() const override { return Transport::WiFi; }

    bool available(SettingId s) const override;
    int optionCount(SettingId s) const override;
    const char *optionLabel(SettingId s, int i) const override;
    int value(SettingId s) const override { return values_[s]; }
    void set(SettingId s, int index) override;
    void setByLabel(SettingId s, const char *label) override;
    bool pending(SettingId s) const override;
    int queued() const override { return (int)queue_.size(); }

    void shutter() override;
    bool recording() const override { return link_ == Link::Connected && datalink_.state().recording; }
    uint32_t recordingSeconds() const override;
    int battery() const override;
    int storageUsed() const override;

    void gimbalSpeed(float yaw, float pitch) override;
    void gimbalMoveTo(float yaw, float pitch, uint32_t durationMs) override;
    void gimbalRecenter() override;
    float gimbalYaw() const override { return yaw_; }
    float gimbalPitch() const override { return pitch_; }

    bool asleep() const override { return false; }
    void sleep() override;
    void wake() override {}
    void snapshot() override {}
    const char *powerActivity() override { return nullptr; }

    void update() override;
    const char *takeMessage() override;

    // Called from the background bring-up task
    Datalink &datalink() { return datalink_; }
    void setLink(Link l) { link_ = l; }
    void message(const char *fmt, ...);

private:
    struct Options {
        const char *const *items;
        int count;
    };
    Options options(SettingId s) const;
    void sendSetting(SettingId s, int index);

    Datalink datalink_;
    volatile Link link_ = Link::Disconnected;
    bool bringupRunning_ = false;

    int values_[S_COUNT] = {};
    std::vector<SettingId> queue_; // settings waiting to be sent
    uint32_t changedAt_[S_COUNT] = {}; // when the board itself last changed each setting
    void syncFromCamera();

    float yaw_ = 0, pitch_ = 0;
    uint32_t lastGimbal_ = 0;

    char message_[64] = {};
    bool hasMessage_ = false;
};
