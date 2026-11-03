#include "sim_camera.h"
#include <stdarg.h>

// Option lists mirror the Pocket 4 Pro as described in the OpenPocketCine
// handbook. A real camera reports Format pairs itself (camcap_video_format).
static const char *const MODES[] = {"Video", "Slow Motion", "Photo", "Live Photo", "Timelapse", "Hyperlapse", "Night", "Panorama"};
static const char *const EXPOSURES[] = {"Auto", "Manual"};
static const char *const VIDEO_FORMATS[] = {
    "4K 24", "4K 25", "4K 30", "4K 48", "4K 50", "4K 60",
    "1080p 24", "1080p 25", "1080p 30", "1080p 48", "1080p 50", "1080p 60"};
static const char *const SLOWMO_FORMATS[] = {
    "4K 100", "4K 120", "4K 200", "4K 240", "1080p 120", "1080p 240"};
static const char *const ISOS[] = {"100", "200", "400", "800", "1600", "3200", "6400"};
static const char *const SHUTTERS[] = {
    "1/50", "1/60", "1/100", "1/120", "1/250", "1/500", "1/1000", "1/2000"};
static const char *const EVS[] = {
    "-2.0", "-1.5", "-1.0", "-0.5", "0", "+0.5", "+1.0", "+1.5", "+2.0"};
static const char *const WBS[] = {
    "Auto", "2700K", "3200K", "4000K", "5000K", "5600K", "6500K", "8000K"};
static const char *const COLORS[] = {"Normal", "D-Log", "D-Log2"};
static const char *const FOCUSES[] = {"AF-C", "AF-S", "MF"};
static const char *const AUDIOS[] = {"Stereo", "Mono"};
static const char *const ON_OFF[] = {"Off", "On"};
static const char *const ASPECTS[] = {"4:3", "16:9", "1:1"};
static const char *const TIMERS[] = {"Off", "0.5s", "3s", "5s", "10s"};
static const char *const GIMBAL_MODES[] = {"Follow", "Tilt Lock", "FPV", "Dir Lock"};

#define OPTS(a) Options{a, (int)(sizeof(a) / sizeof(a[0]))}

static const float FAILURE_RATE = 0.03f; // fraction of commands that get no ack

static const uint32_t WAKE_ADVERT_MS = 2000;              // DJI: broadcast the wake packet for 2 s
static const uint32_t WAKE_WINDOW_MS = 30UL * 60 * 1000;  // DJI: only within 30 min of sleeping
static const uint32_t SNAPSHOT_CLIP_MS = 5000;            // simulated snapshot clip length

SimCamera::SimCamera() {
    values_[S_FORMAT] = 2; // 4K 30
    values_[S_EV] = 4;     // 0
    values_[S_ISO] = 2;    // 400
}

const char *SimCamera::tag() const {
    return transport_ == Transport::Bluetooth ? "[BLE]" : "[WiFi 192.168.2.1:9004]";
}

void SimCamera::message(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(message_, sizeof(message_), fmt, args);
    va_end(args);
    hasMessage_ = true;
}

const char *SimCamera::takeMessage() {
    if (!hasMessage_) return nullptr;
    hasMessage_ = false;
    return message_;
}

uint32_t SimCamera::latency() const {
    return transport_ == Transport::Bluetooth ? 90 + random(60) : 30 + random(30);
}

// ---------- connection ----------

void SimCamera::connect(Transport t) {
    if (link_ != Link::Disconnected) return;
    if (asleep_) {
        message("Camera asleep - use Wake");
        return;
    }
    transport_ = t;
    link_ = Link::Connecting;
    if (t == Transport::Bluetooth) {
        Serial.println("[BLE] scan -> GATT fff0 (notify fff4, write fff5) -> pair 07/45");
        connectDue_ = millis() + 1500;
    } else {
        Serial.println("[BLE] scan -> pair 07/45 -> SSID 07/07 -> password 07/0E");
        Serial.println("[WiFi] join camera AP -> TCP :7001 poke -> UDP 192.168.2.1:9004 handshake");
        connectDue_ = millis() + 3000;
    }
}

void SimCamera::disconnect() {
    if (link_ == Link::Disconnected) return;
    if (yawSpeed_ != 0 || pitchSpeed_ != 0) gimbalSpeed(0, 0);
    Serial.printf("%s disconnect (%d queued command(s) dropped)\n", tag(), queueLen_);
    queueLen_ = 0;
    headDue_ = 0;
    link_ = Link::Disconnected;
}

// ---------- settings ----------

SimCamera::Options SimCamera::options(SettingId s) const {
    switch (s) {
    case S_MODE:        return OPTS(MODES);
    case S_EXPOSURE:    return OPTS(EXPOSURES);
    case S_FORMAT:
        if (values_[S_MODE] == 0) return OPTS(VIDEO_FORMATS);
        if (values_[S_MODE] == 1) return OPTS(SLOWMO_FORMATS);
        return Options{nullptr, 0};
    case S_ISO:         return OPTS(ISOS);
    case S_SHUTTER:     return OPTS(SHUTTERS);
    case S_EV:          return OPTS(EVS);
    case S_WB:          return OPTS(WBS);
    case S_COLOR:       return OPTS(COLORS);
    case S_FOCUS:       return OPTS(FOCUSES);
    case S_AUDIO:       return OPTS(AUDIOS);
    case S_VOCAL:       return OPTS(ON_OFF);
    case S_ASPECT:      return OPTS(ASPECTS);
    case S_TIMER:       return OPTS(TIMERS);
    case S_GIMBAL_MODE: return OPTS(GIMBAL_MODES);
    default:            return Options{nullptr, 0};
    }
}

bool SimCamera::available(SettingId s) const {
    bool video = values_[S_MODE] <= 1;
    bool manual = values_[S_EXPOSURE] == 1;
    switch (s) {
    case S_FORMAT:
    case S_COLOR:
    case S_AUDIO:
    case S_VOCAL:   return video;
    case S_ISO:
    case S_SHUTTER: return manual;
    case S_EV:      return !manual;
    case S_ASPECT:
    case S_TIMER:   return !video;
    default:        return true;
    }
}

int SimCamera::optionCount(SettingId s) const { return options(s).count; }

const char *SimCamera::optionLabel(SettingId s, int i) const {
    Options o = options(s);
    return (i >= 0 && i < o.count) ? o.items[i] : "-";
}

void SimCamera::push(const Cmd &c) {
    if (link_ != Link::Connected) {
        message("Not connected");
        return;
    }
    if (queueLen_ == QUEUE_MAX) {
        message("Camera busy, try again");
        return;
    }
    queue_[(queueHead_ + queueLen_++) % QUEUE_MAX] = c;
}

void SimCamera::set(SettingId s, int index) {
    Cmd c = {CMD_SET, s, index, ""};
    push(c);
}

void SimCamera::setByLabel(SettingId s, const char *label) {
    Cmd c = {CMD_SET, s, -1, ""};
    strlcpy(c.label, label, sizeof(c.label));
    push(c);
}

bool SimCamera::pending(SettingId s) const {
    for (int i = 0; i < queueLen_; i++) {
        const Cmd &c = queue_[(queueHead_ + i) % QUEUE_MAX];
        if (c.kind == CMD_SET && c.setting == s) return true;
    }
    return false;
}

void SimCamera::process(const Cmd &c) {
    if (random(1000) < FAILURE_RATE * 1000) {
        Serial.printf("%s no ack\n", tag());
        message("%s: no reply, try again",
                c.kind == CMD_SET ? SETTINGS[c.setting].name : "Camera");
        return;
    }

    switch (c.kind) {
    case CMD_SET: {
        const SettingInfo &info = SETTINGS[c.setting];
        int index = c.index;
        if (index < 0) {
            Options o = options(c.setting);
            for (int i = 0; i < o.count; i++)
                if (strcmp(o.items[i], c.label) == 0) index = i;
            if (index < 0) {
                message("%s %s not available", info.name, c.label);
                return;
            }
        }
        if (index >= optionCount(c.setting)) return;
        values_[c.setting] = index;
        Serial.printf("%s DUML %s %s = %s  (ack)\n", tag(), info.opcode, info.name,
                      optionLabel(c.setting, index));
        // A mode change reshapes the format list; the camera falls back to its first entry
        if (c.setting == S_MODE && values_[S_FORMAT] >= optionCount(S_FORMAT)) values_[S_FORMAT] = 0;
        break;
    }
    case CMD_SHUTTER:
        if (isPhotoMode()) {
            Serial.printf("%s DUML 02/01 shutter  (ack)\n", tag());
            message("Photo taken");
            storageUsed_ += 0.1f;
        } else {
            recording_ = !recording_;
            recordStart_ = millis();
            Serial.printf("%s DUML 02/02 record %s  (ack)\n", tag(), recording_ ? "start" : "stop");
        }
        break;
    case CMD_RECENTER:
        yaw_ = pitch_ = 0;
        moveActive_ = false;
        Serial.printf("%s DUML 04/14 recenter  (ack)\n", tag());
        break;
    case CMD_SLEEP:
        Serial.printf("%s DUML 00/1A power_mode=3 (sleep)  (ack)\n", tag());
        recording_ = false;
        queueLen_ = 0;
        headDue_ = 0;
        link_ = Link::Disconnected;
        asleep_ = true;
        asleepSince_ = millis();
        message("Camera asleep");
        break;
    }
}

// ---------- power: sleep, wake, snapshot ----------

void SimCamera::sleep() {
    Cmd c = {CMD_SLEEP, S_MODE, 0, ""};
    push(c);
}

bool SimCamera::canWake() {
    if (!asleep_ || powerOp_ != P_NONE) return false;
    if (!everConnected_) {
        message("Connect to the camera once first");
        return false;
    }
    if (millis() - asleepSince_ > WAKE_WINDOW_MS) {
        message("Asleep >30 min - press camera button");
        return false;
    }
    return true;
}

void SimCamera::wake() {
    if (!canWake()) return;
    Serial.println("[BLE] advertise 'WKP' + camera MAC (reversed) for 2 s");
    powerOp_ = P_WAKING;
    powerDue_ = millis() + WAKE_ADVERT_MS;
}

void SimCamera::snapshot() {
    if (!canWake()) return;
    Serial.println("[BLE] advertise 'WKP' + camera MAC (reversed) for 2 s  (snapshot)");
    powerOp_ = P_SNAP_WAKING;
    powerDue_ = millis() + WAKE_ADVERT_MS;
}

const char *SimCamera::powerActivity() {
    switch (powerOp_) {
    case P_WAKING:      return "Waking camera...";
    case P_SNAP_WAKING: return "Snapshot: waking...";
    case P_SNAP_RECORDING:
        snprintf(activity_, sizeof(activity_), "Snapshot: recording %lus", (millis() - recordStart_) / 1000);
        return activity_;
    default:            return nullptr;
    }
}

// ---------- recording, gimbal, battery ----------

void SimCamera::shutter() {
    Cmd c = {CMD_SHUTTER, S_MODE, 0, ""};
    push(c);
}

uint32_t SimCamera::recordingSeconds() const {
    return recording_ ? (millis() - recordStart_) / 1000 : 0;
}

int SimCamera::battery() const {
    return max(5, 87 - (int)(millis() / 120000)); // drains 1% every 2 minutes
}

void SimCamera::gimbalSpeed(float yaw, float pitch) {
    bool wasMoving = yawSpeed_ != 0 || pitchSpeed_ != 0;
    yawSpeed_ = yaw;
    pitchSpeed_ = pitch;
    bool moving = yaw != 0 || pitch != 0;
    if (moving || !wasMoving) moveActive_ = false; // the stick (or an explicit stop) overrides a timed move
    // Real stick command opcode is not confirmed yet in the handbook
    if (moving && millis() - lastSpeedLog_ > 500) {
        lastSpeedLog_ = millis();
        Serial.printf("%s DUML 04/?? stick yaw=%+.2f pitch=%+.2f\n", tag(), yaw, pitch);
    } else if (!moving && wasMoving) {
        Serial.printf("%s DUML 04/?? stick stop\n", tag());
    }
}

void SimCamera::gimbalMoveTo(float yaw, float pitch, uint32_t durationMs) {
    if (link_ != Link::Connected) return;
    yawSpeed_ = pitchSpeed_ = 0;
    moveFromYaw_ = yaw_;
    moveFromPitch_ = pitch_;
    moveToYaw_ = constrain(yaw, GIMBAL_YAW_MIN, GIMBAL_YAW_MAX);
    moveToPitch_ = constrain(pitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
    moveStart_ = millis();
    moveDuration_ = max(durationMs, (uint32_t)100);
    moveActive_ = true;
    // Replays stream a target every 100 ms; only log the slower ones and a 1 s sample
    if (durationMs >= 500 || millis() - lastMoveLog_ > 1000) {
        lastMoveLog_ = millis();
        Serial.printf("%s DUML 04/14 target pan=%+.1f tilt=%+.1f in %.1fs\n", tag(), moveToYaw_,
                      moveToPitch_, moveDuration_ / 1000.0f);
    }
}

void SimCamera::gimbalRecenter() {
    Cmd c = {CMD_RECENTER, S_MODE, 0, ""};
    push(c);
}

void SimCamera::update() {
    uint32_t now = millis();

    // Power operations
    if (powerOp_ != P_NONE && (int32_t)(now - powerDue_) >= 0) {
        switch (powerOp_) {
        case P_WAKING:
            asleep_ = false;
            powerOp_ = P_NONE;
            Serial.println("camera awake");
            connect(transport_);
            break;
        case P_SNAP_WAKING:
            Serial.println("[BLE] key report 00/11 SNAPSHOT (0x03)");
            if (isPhotoMode()) {
                message("Snapshot photo taken");
                powerOp_ = P_NONE;
                asleepSince_ = now;
            } else {
                recording_ = true;
                recordStart_ = now;
                powerOp_ = P_SNAP_RECORDING;
                powerDue_ = now + SNAPSHOT_CLIP_MS;
            }
            break;
        case P_SNAP_RECORDING:
            recording_ = false;
            powerOp_ = P_NONE;
            asleepSince_ = now; // back to sleep after the clip
            Serial.println("snapshot clip saved, camera back to sleep");
            message("Snapshot clip saved");
            break;
        default:
            break;
        }
    }

    if (link_ == Link::Connecting && (int32_t)(now - connectDue_) >= 0) {
        link_ = Link::Connected;
        everConnected_ = true;
        Serial.printf("%s connected (simulated)\n", tag());
        message("Connected via %s", transport_ == Transport::Bluetooth ? "Bluetooth" : "Wi-Fi");
    }

    if (link_ == Link::Connected && queueLen_ > 0) {
        if (headDue_ == 0) headDue_ = now + latency();
        if ((int32_t)(now - headDue_) >= 0) {
            Cmd c = queue_[queueHead_];
            queueHead_ = (queueHead_ + 1) % QUEUE_MAX;
            queueLen_--;
            headDue_ = 0;
            process(c);
        }
    }

    // Recording fills the card (~1% every 15 s in the simulation)
    static uint32_t lastStorageTick = now;
    if (recording_) storageUsed_ = min(100.0f, storageUsed_ + (now - lastStorageTick) / 15000.0f);
    lastStorageTick = now;

    // Gimbal motion: a timed move interpolates; otherwise integrate the stick
    // (full stick = 90 deg/s pan, 60 deg/s tilt)
    float dt = (now - lastGimbalTick_) / 1000.0f;
    lastGimbalTick_ = now;
    if (link_ == Link::Connected && moveActive_) {
        float f = min((now - moveStart_) / (float)moveDuration_, 1.0f);
        yaw_ = moveFromYaw_ + (moveToYaw_ - moveFromYaw_) * f;
        pitch_ = moveFromPitch_ + (moveToPitch_ - moveFromPitch_) * f;
        if (f >= 1) moveActive_ = false;
    } else if (link_ == Link::Connected && dt < 0.5f) {
        yaw_ = constrain(yaw_ + yawSpeed_ * 90 * dt, GIMBAL_YAW_MIN, GIMBAL_YAW_MAX);
        pitch_ = constrain(pitch_ + pitchSpeed_ * 60 * dt, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
    }
}
