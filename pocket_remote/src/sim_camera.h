// Simulated Pocket 4P: keeps camera state, reports Pocket 4 Pro option lists,
// acks commands after a transport-dependent delay (occasionally failing), and
// logs the DUML command a real camera would receive.
#pragma once
#include "camera.h"

class SimCamera : public Camera {
public:
    SimCamera();

    void connect(Transport t) override;
    void disconnect() override;
    Link link() const override { return link_; }
    Transport transport() const override { return transport_; }

    bool available(SettingId s) const override;
    int optionCount(SettingId s) const override;
    const char *optionLabel(SettingId s, int i) const override;
    int value(SettingId s) const override { return values_[s]; }
    void set(SettingId s, int index) override;
    void setByLabel(SettingId s, const char *label) override;
    bool pending(SettingId s) const override;
    int queued() const override { return queueLen_; }

    void shutter() override;
    bool recording() const override { return recording_; }
    uint32_t recordingSeconds() const override;
    int battery() const override;
    int storageUsed() const override { return min(100, (int)storageUsed_); }

    void gimbalSpeed(float yaw, float pitch) override;
    void gimbalMoveTo(float yaw, float pitch, uint32_t durationMs) override;
    void gimbalRecenter() override;
    float gimbalYaw() const override { return yaw_; }
    float gimbalPitch() const override { return pitch_; }

    bool asleep() const override { return asleep_; }
    void sleep() override;
    void wake() override;
    void snapshot() override;
    const char *powerActivity() override;

    void update() override;
    const char *takeMessage() override;

private:
    enum Kind : uint8_t { CMD_SET, CMD_SHUTTER, CMD_RECENTER, CMD_SLEEP };
    enum PowerOp : uint8_t { P_NONE, P_WAKING, P_SNAP_WAKING, P_SNAP_RECORDING };

    bool canWake();
    struct Cmd {
        Kind kind;
        SettingId setting;
        int index;      // -1 when resolving by label
        char label[16];
    };
    struct Options {
        const char *const *items;
        int count;
    };

    Options options(SettingId s) const;
    void push(const Cmd &c);
    void process(const Cmd &c);
    uint32_t latency() const;
    void message(const char *fmt, ...);
    const char *tag() const;

    static const int QUEUE_MAX = 32;
    Cmd queue_[QUEUE_MAX];
    int queueHead_ = 0, queueLen_ = 0;
    uint32_t headDue_ = 0;

    int values_[S_COUNT] = {};
    Link link_ = Link::Disconnected;
    Transport transport_ = Transport::Bluetooth;
    uint32_t connectDue_ = 0;

    bool recording_ = false;
    uint32_t recordStart_ = 0;

    float yawSpeed_ = 0, pitchSpeed_ = 0, yaw_ = 0, pitch_ = 0;
    uint32_t lastGimbalTick_ = 0, lastSpeedLog_ = 0, lastMoveLog_ = 0;

    // Timed move in progress
    bool moveActive_ = false;
    float moveFromYaw_ = 0, moveFromPitch_ = 0, moveToYaw_ = 0, moveToPitch_ = 0;
    uint32_t moveStart_ = 0, moveDuration_ = 0;

    char message_[64] = {};
    bool hasMessage_ = false;

    // Card: fills while recording and with each photo
    float storageUsed_ = 42;

    // Power state
    bool asleep_ = false, everConnected_ = false;
    uint32_t asleepSince_ = 0;
    PowerOp powerOp_ = P_NONE;
    uint32_t powerDue_ = 0;
    char activity_[40] = {};
};
