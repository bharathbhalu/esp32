#include "moves.h"
#include <Preferences.h>

namespace Moves {

static const uint32_t PREP_MS = 1500; // travel to the start point
static const uint32_t HOLD_MS = 1000; // settle at the start before the take

struct Preset {
    const char *name;
    int count;
    float points[3][2]; // pan, tilt
};

// "Center" starts from wherever the gimbal is, so it has no prep leg
static const Preset PRESETS[PRESET_COUNT] = {
    {"Pan L>R", 2, {{-45, 0}, {45, 0}}},
    {"Pan R>L", 2, {{45, 0}, {-45, 0}}},
    {"Reveal", 2, {{0, -40}, {0, 0}}},
    {"Look up", 2, {{0, 0}, {0, 60}}},
    {"Diagonal", 2, {{-40, -30}, {40, 30}}},
    {"Move dn", 2, {{0, 0}, {0, -40}}},
    {"Center", 1, {{0, 0}}},
};

struct Waypoint {
    float yaw, pitch;
    uint32_t t; // ms from the start of the take
};

enum Phase { IDLE, FRONT, PREP, HOLD, RUN };

static Waypoint path[MAX_SAMPLES];
static int pathLen = 0;
static Phase phase = IDLE;
static uint32_t phaseStart = 0;
static int legIndex = 0;
static bool startedVideo = false, wantVideo = false;
static char playName[16];

// Preset pans are relative to the front, not fixed joint angles: the joint's
// front can sit anywhere (near 180 after a calibration), and a fixed -45..45 then
// swung the gimbal round the back. With faceFront the camera recenters first
// (its own 04/4C) and the front is read from where it settles.
static bool faceFront = false;
static const uint32_t FRONT_MS = 1500;
static int pendingPreset = -1;
static uint32_t pendingDuration = 0;
static bool pendingVideo = false;

// Recording buffer, stored as tenths of a degree
static int16_t recBuf[MAX_SAMPLES][2];
static int recLen = 0;
static bool isRecording = false;
static uint32_t lastSample = 0;

static uint16_t slotLens[SLOT_COUNT];

static String slotKey(int slot) { return "s" + String(slot); }

void begin() {
    Preferences prefs;
    prefs.begin("moves", true);
    for (int i = 0; i < SLOT_COUNT; i++) slotLens[i] = prefs.getBytesLength(slotKey(i).c_str()) / 4;
    prefs.end();
}

const char *presetName(int i) { return PRESETS[i].name; }

bool slotUsed(int slot) { return slotLens[slot] > 0; }

float slotSeconds(int slot) { return slotLens[slot] * SAMPLE_MS / 1000.0f; }

bool hasFreeSlot() {
    for (int i = 0; i < SLOT_COUNT; i++)
        if (!slotUsed(i)) return true;
    return false;
}

void deleteSlot(int slot) {
    Preferences prefs;
    prefs.begin("moves", false);
    prefs.remove(slotKey(slot).c_str());
    prefs.end();
    slotLens[slot] = 0;
}

// ---------- recording ----------

void startRecording() {
    recLen = 0;
    isRecording = true;
    lastSample = 0;
}

bool recording() { return isRecording; }

float recordedSeconds() { return recLen * SAMPLE_MS / 1000.0f; }

bool recordingFull() { return recLen >= MAX_SAMPLES; }

int stopRecording() {
    isRecording = false;
    if (recLen < 5) return -1; // under half a second isn't a move
    for (int slot = 0; slot < SLOT_COUNT; slot++) {
        if (slotUsed(slot)) continue;
        Preferences prefs;
        prefs.begin("moves", false);
        prefs.putBytes(slotKey(slot).c_str(), recBuf, recLen * 4);
        prefs.end();
        slotLens[slot] = recLen;
        Serial.printf("move saved to slot %d (%.1f s)\n", slot + 1, recordedSeconds());
        return slot;
    }
    return -1;
}

// ---------- playback ----------

// Ramp: instead of one constant-speed timed move per leg, each leg becomes a few
// short ones whose positions follow an ease curve, so the gimbal starts and
// stops gently. 0 Off, 1 Medium (smoothstep), 2 Soft (smootherstep).
static int ramp = 0;
static const int RAMP_STEPS = 6;

void setRamp(int r) { ramp = constrain(r, 0, 2); }

static float ease(float x) {
    if (ramp == 1) return x * x * (3 - 2 * x);
    return x * x * x * (x * (6 * x - 15) + 10);
}

static void applyRamp() {
    if (ramp == 0 || pathLen < 2 || (pathLen - 1) * RAMP_STEPS + 1 > MAX_SAMPLES) return;
    static Waypoint eased[MAX_SAMPLES];
    int n = 0;
    for (int leg = 0; leg < pathLen - 1; leg++) {
        const Waypoint &a = path[leg], &b = path[leg + 1];
        for (int k = 0; k < RAMP_STEPS; k++) {
            float f = ease((float)k / RAMP_STEPS);
            eased[n++] = {a.yaw + (b.yaw - a.yaw) * f, a.pitch + (b.pitch - a.pitch) * f,
                          a.t + (b.t - a.t) * k / RAMP_STEPS};
        }
    }
    eased[n++] = path[pathLen - 1];
    memcpy(path, eased, n * sizeof(Waypoint));
    pathLen = n;
}

static void start(Camera &cam, const char *name, bool withVideo) {
    applyRamp();
    strlcpy(playName, name, sizeof(playName));
    wantVideo = withVideo;
    startedVideo = false;
    legIndex = 0;
    phaseStart = millis();
    if (pathLen == 1) {
        // Single target (Center): just the timed move
        phase = RUN;
        return;
    }
    cam.gimbalMoveTo(path[0].yaw, path[0].pitch, PREP_MS);
    phase = PREP;
}

void setFaceFront(bool on) { faceFront = on; }

// Timelapse pan: front - 45 to front + 45 over minutes. One camera move lasts at
// most 25.5 s, so the sweep is a chain of 20 s legs; no ramp (it would be lost
// in a timelapse anyway)
void playTimelapsePan(Camera &cam, uint32_t totalMs, bool withVideo) {
    const uint32_t LEG_MS = 20000;
    int legs = constrain((int)((totalMs + LEG_MS - 1) / LEG_MS), 1, MAX_SAMPLES - 1);
    float front = cam.gimbalYaw();
    float from = constrain(front - 45, GIMBAL_YAW_MIN, GIMBAL_YAW_MAX);
    float to = constrain(front + 45, GIMBAL_YAW_MIN, GIMBAL_YAW_MAX);
    float tilt = cam.gimbalPitch();
    pathLen = legs + 1;
    for (int i = 0; i <= legs; i++) path[i] = {from + (to - from) * i / legs, tilt, totalMs * (uint32_t)i / legs};
    int saved = ramp;
    ramp = 0;
    start(cam, "TL pan", withVideo);
    ramp = saved;
}

// Preset waypoints are pan offsets from front (tilt stays absolute)
static void runPreset(Camera &cam, int preset, uint32_t durationMs, bool withVideo, float front) {
    const Preset &p = PRESETS[preset];
    pathLen = p.count;
    for (int i = 0; i < p.count; i++)
        path[i] = {constrain(front + p.points[i][0], GIMBAL_YAW_MIN, GIMBAL_YAW_MAX), p.points[i][1],
                   (uint32_t)(durationMs * i / (p.count - 1))};
    start(cam, p.name, withVideo);
}

void playPreset(Camera &cam, int preset, uint32_t durationMs, bool withVideo) {
    const Preset &p = PRESETS[preset];
    if (p.count == 1) {
        // Center: the camera's own recenter knows where its front is
        cam.gimbalRecenter();
        return;
    }
    if (faceFront) {
        strlcpy(playName, p.name, sizeof(playName));
        pendingPreset = preset;
        pendingDuration = durationMs;
        pendingVideo = withVideo;
        cam.gimbalRecenter();
        phaseStart = millis();
        phase = FRONT;
        return;
    }
    runPreset(cam, preset, durationMs, withVideo, cam.gimbalYaw()); // around where it points now
}

void playSlot(Camera &cam, int slot, bool withVideo) {
    Preferences prefs;
    prefs.begin("moves", true);
    pathLen = prefs.getBytes(slotKey(slot).c_str(), recBuf, sizeof(recBuf)) / 4;
    prefs.end();
    for (int i = 0; i < pathLen; i++)
        path[i] = {recBuf[i][0] / 10.0f, recBuf[i][1] / 10.0f, i * SAMPLE_MS};
    char name[16];
    snprintf(name, sizeof(name), "Move %d", slot + 1);
    if (pathLen >= 2) start(cam, name, withVideo);
}

bool playing() { return phase != IDLE; }

String status() {
    if (phase == FRONT) return String(playName) + "  facing front...";
    if (phase == PREP) return String(playName) + "  to start...";
    if (phase == HOLD) return String(playName) + "  get ready";
    if (phase != RUN) return "";
    char buf[48];
    float total = path[pathLen - 1].t / 1000.0f;
    float t = min((millis() - phaseStart) / 1000.0f, total);
    snprintf(buf, sizeof(buf), "%s  %.1f / %.1f s", playName, t, total);
    return buf;
}

static void finish(Camera &cam) {
    if (startedVideo && cam.recording()) cam.shutter();
    startedVideo = false;
    phase = IDLE;
}

void stop(Camera &cam) {
    if (phase == IDLE) return;
    cam.gimbalSpeed(0, 0); // cancels the timed move in progress
    finish(cam);
}

void update(Camera &cam) {
    uint32_t now = millis();

    if (isRecording && cam.link() == Link::Connected && now - lastSample >= SAMPLE_MS && recLen < MAX_SAMPLES) {
        lastSample = now;
        recBuf[recLen][0] = (int16_t)(cam.gimbalYaw() * 10);
        recBuf[recLen][1] = (int16_t)(cam.gimbalPitch() * 10);
        recLen++;
    }

    if (phase == IDLE) return;
    if (cam.link() != Link::Connected) {
        phase = IDLE;
        return;
    }
    uint32_t elapsed = now - phaseStart;
    switch (phase) {
    case FRONT:
        if (elapsed >= FRONT_MS) runPreset(cam, pendingPreset, pendingDuration, pendingVideo, cam.gimbalYaw());
        break;
    case PREP:
        if (elapsed >= PREP_MS) {
            phase = HOLD;
            phaseStart = now;
        }
        break;
    case HOLD:
        if (elapsed >= HOLD_MS) {
            phase = RUN;
            phaseStart = now;
        }
        break;
    case RUN:
        if (legIndex == 0 && wantVideo && !cam.recording() && !cam.isPhotoMode()) {
            cam.shutter();
            startedVideo = true;
        }
        // Send each leg as it starts: target = next waypoint, timed to arrive on schedule
        while (legIndex < pathLen - 1 && elapsed >= path[legIndex].t) {
            const Waypoint &to = path[legIndex + 1];
            cam.gimbalMoveTo(to.yaw, to.pitch, to.t - path[legIndex].t);
            legIndex++;
        }
        if (legIndex >= pathLen - 1 && elapsed >= path[pathLen - 1].t) finish(cam);
        break;
    default:
        break;
    }
}

} // namespace Moves
