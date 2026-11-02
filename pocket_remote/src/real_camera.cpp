#include "real_camera.h"
#include "frametap_relay.h"
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <stdarg.h>

// Option lists mirror SimCamera / the UI (Pocket 4 Pro). The real camera reports
// valid combinations too, but fixed lists keep the UI identical for this cut.
static const char *const MODES[] = {"Video", "Slow Motion", "Photo", "Live Photo"};
static const char *const EXPOSURES[] = {"Auto", "Manual"};
static const char *const VIDEO_FORMATS[] = {"4K 24", "4K 25", "4K 30", "4K 48", "4K 50", "4K 60",
                                            "1080p 24", "1080p 25", "1080p 30", "1080p 48", "1080p 50", "1080p 60"};
static const char *const SLOWMO_FORMATS[] = {"4K 100", "4K 120", "4K 200", "4K 240", "1080p 120", "1080p 240"};
static const char *const ISOS[] = {"100", "200", "400", "800", "1600", "3200", "6400"};
static const char *const SHUTTERS[] = {"1/50", "1/60", "1/100", "1/120", "1/250", "1/500", "1/1000", "1/2000"};
static const char *const EVS[] = {"-2.0", "-1.5", "-1.0", "-0.5", "0", "+0.5", "+1.0", "+1.5", "+2.0"};
static const char *const WBS[] = {"Auto", "2700K", "3200K", "4000K", "5000K", "5600K", "6500K", "8000K"};
static const char *const COLORS[] = {"Normal", "D-Log", "D-Log2"};
static const char *const FOCUSES[] = {"AF-C", "AF-S", "MF"};
static const char *const AUDIOS[] = {"Stereo", "Mono"};
static const char *const ON_OFF[] = {"Off", "On"};
static const char *const ASPECTS[] = {"4:3", "16:9", "1:1"};
static const char *const TIMERS[] = {"Off", "0.5s", "3s", "5s", "10s"};
static const char *const GIMBAL_MODES[] = {"Follow", "Tilt Lock", "FPV", "Dir Lock"};
#define OPTS(a) Options{a, (int)(sizeof(a) / sizeof(a[0]))}

// Value encodings (from OpenCine CameraControl enums)
static const uint8_t MODE_RAW[] = {0x01, 0x00, 0x17, 0x4D};   // Video, SlowMo, Photo, LivePhoto
static const uint8_t ISO_RAW[] = {0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
static const int SHUTTER_DENOM[] = {50, 60, 100, 120, 250, 500, 1000, 2000};
static const int WB_KELVIN[] = {0, 2700, 3200, 4000, 5000, 5600, 6500, 8000};
static const uint8_t COLOR_RAW[] = {0x3F, 0x17, 0x41};        // Normal, D-Log, D-Log2

static const char *AP_SSID = "PocketRemote";
static const char *AP_PASS = "osmo12345678";
static const uint32_t WRITE_GAP_MS = 250;

// ---------- shared BLE state (bring-up task + loop) ----------

static RealCamera *self = nullptr;
static NimBLEClient *bleClient = nullptr;
static NimBLERemoteCharacteristic *bleNotify = nullptr, *bleWrite = nullptr;
static NimBLERemoteCharacteristic *bleChars[3] = {}; // fff3, fff4, fff5 (for the FrameTap relay)
static volatile bool relayOwnsBle = false; // after bring-up the camera's BLE belongs to the relay
static const IPAddress CAMERA_IP(192, 168, 2, 2); // the AP's first DHCP client
static bool handedOff = false; // own session closed while FrameTap controls the camera
static portMUX_TYPE rxLock = portMUX_INITIALIZER_UNLOCKED;
static std::vector<uint8_t> rxBytes;
static Duml::Assembler bleAsm;
static std::vector<Duml::Frame> bleInbox;
static uint32_t lastBleWrite = 0;

static void onBleNotify(NimBLERemoteCharacteristic *c, uint8_t *d, size_t n, bool) {
    if (relayOwnsBle) {
        String u = c->getUUID().toString().c_str(); // "0xfff4"
        Relay::onCameraNotify((uint16_t)strtol(u.c_str() + 2, nullptr, 16), d, n);
        return;
    }
    portENTER_CRITICAL(&rxLock);
    rxBytes.insert(rxBytes.end(), d, d + n);
    portEXIT_CRITICAL(&rxLock);
}

static void bleSend(const Duml::Frame &f) {
    uint32_t since = millis() - lastBleWrite;
    if (since < WRITE_GAP_MS) delay(WRITE_GAP_MS - since);
    std::vector<uint8_t> b = Duml::encode(f);
    if (bleWrite) bleWrite->writeValue(b.data(), b.size(), false);
    lastBleWrite = millis();
}

static void blePump() {
    std::vector<uint8_t> bytes;
    portENTER_CRITICAL(&rxLock);
    bytes.swap(rxBytes);
    portEXIT_CRITICAL(&rxLock);
    for (Duml::Frame &f : bleAsm.append(bytes.data(), bytes.size())) {
        if (f.cmdSet == 0x07 && f.cmdId == 0x46 && f.flags != Duml::FLAG_RESPONSE)
            bleSend(Duml::pairApprovalAck(f.seq));
        bleInbox.push_back(f);
    }
}

static bool bleWait(uint8_t set, uint8_t cmd, uint32_t ms, Duml::Frame &out, uint8_t set2 = 0xFF, uint8_t cmd2 = 0xFF) {
    uint32_t start = millis();
    while (millis() - start < ms) {
        blePump();
        for (size_t i = 0; i < bleInbox.size(); i++) {
            Duml::Frame &f = bleInbox[i];
            if ((f.cmdSet == set && f.cmdId == cmd) || (f.cmdSet == set2 && f.cmdId == cmd2)) {
                out = f;
                bleInbox.erase(bleInbox.begin() + i);
                return true;
            }
        }
        if (bleClient && !bleClient->isConnected()) return false;
        delay(20);
    }
    return false;
}

// ---------- bring-up task ----------

// A failed attempt drops BLE too: a camera still connected stops advertising,
// so the next (automatic) attempt couldn't find it
static void failBringup(const char *msg) {
    self->message("%s", msg);
    if (bleClient && bleClient->isConnected()) bleClient->disconnect();
    self->setLink(Link::Disconnected);
    vTaskDelete(nullptr);
}

static void bringupTask(void *) {
    self->message("Finding camera...");
    NimBLEDevice::init("ESP32 Remote");
    NimBLEDevice::setMTU(517);
    Relay::prepare(); // GATT server must be built while the stack is idle
    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setActiveScan(true);

    const NimBLEAdvertisedDevice *cam = nullptr;
    NimBLEScanResults res = scan->getResults(6000, false);
    for (int i = 0; i < res.getCount(); i++) {
        const NimBLEAdvertisedDevice *d = res.getDevice(i);
        std::string m = d->getManufacturerData();
        // FrameTap's phone also advertises DJI data ("OsmoFrameTap-..."): match the name too
        if (m.size() >= 2 && ((uint8_t)m[0] | ((uint8_t)m[1] << 8)) == 0x08AA &&
            d->getName().rfind("OsmoPocket", 0) == 0)
            cam = d;
    }
    if (!cam) {
        self->message("No camera found");
        self->setLink(Link::Disconnected);
        vTaskDelete(nullptr);
        return;
    }

    std::string camName = cam->getName(), camMfr = cam->getManufacturerData();
    bleClient = NimBLEDevice::createClient();
    if (!bleClient->connect(cam)) {
        failBringup("BLE connect failed");
        return;
    }
    NimBLERemoteService *svc = bleClient->getService("fff0");
    for (NimBLERemoteCharacteristic *c : svc->getCharacteristics(true)) {
        String u = c->getUUID().toString().c_str();
        if (c->canNotify()) c->subscribe(true, onBleNotify);
        if (u.indexOf("fff4") >= 0) bleNotify = c;
        if (u.indexOf("fff5") >= 0) bleWrite = c;
        if (u.indexOf("fff3") >= 0) bleChars[0] = c;
    }
    uint8_t arm[2] = {0x01, 0x00};
    if (bleNotify) bleNotify->writeValue(arm, 2, true);

    self->message("Pairing...");
    bleSend(Duml::sessionWake());
    bleSend(Duml::setPairingPin("osmo"));
    Duml::Frame reply;
    if (bleWait(0x07, 0x45, 15000, reply, 0x07, 0x46)) {
        if (reply.cmdId == 0x45 && reply.payload.size() >= 2 && reply.payload[1] == 2) {
            self->message("Approve OSMO on the camera");
            bleWait(0x07, 0x46, 90000, reply);
        }
    }

    // Board hosts its own 2.4 GHz AP; camera joins it as a station
    self->message("Starting board Wi-Fi...");
    WiFi.mode(WIFI_AP);
    // 192.168.2.1 is where the camera's own hotspot sits, and where FrameTap connects
    IPAddress apIp(192, 168, 2, 1);
    WiFi.softAPConfig(apIp, apIp, IPAddress(255, 255, 255, 0));
    WiFi.softAP(AP_SSID, AP_PASS);
    uint16_t seq = 0x9000;
    bleSend(Duml::stationMode(true, seq++));
    bleWait(0x07, 0x48, 4000, reply);
    delay(1500);
    self->message("Camera joining...");
    bleSend(Duml::joinWifi(AP_SSID, AP_PASS, seq++));
    bleWait(0x07, 0x47, 12000, reply);

    uint32_t start = millis();
    while (WiFi.softAPgetStationNum() == 0 && millis() - start < 20000) {
        blePump();
        delay(200);
    }
    if (WiFi.softAPgetStationNum() == 0) {
        failBringup("Camera did not join");
        return;
    }
    self->message("Opening control link...");
    IPAddress camIp = CAMERA_IP;
    if (!self->datalink().open(camIp, [](const String &s) { Serial.println(s); })) {
        failBringup("Control link failed");
        return;
    }
    self->message("Connected to Pocket 4P");
    self->setLink(Link::Connected);
    bleChars[1] = bleNotify;
    bleChars[2] = bleWrite;
    relayOwnsBle = true;
    Relay::start(bleClient, bleChars, camName, camMfr, camIp, AP_SSID, AP_PASS);
    // While FrameTap holds the session the board's commands ride in it, and the
    // board reads the camera's status from the packets going to FrameTap
    static Datalink *link = &self->datalink();
    link->setForwarder(Relay::inject);
    Relay::setObserver([](const uint8_t *d, size_t n) { link->observe(d, n); });
    vTaskDelete(nullptr);
}

// ---------- RealCamera ----------

void RealCamera::connect(Transport) {
    if (link_ != Link::Disconnected) return;
    self = this;
    link_ = Link::Connecting;
    xTaskCreatePinnedToCore(bringupTask, "bringup", 16384, nullptr, 1, nullptr, 0);
}

void RealCamera::disconnect() {
    if (link_ == Link::Disconnected) return;
    datalink_.close();
    if (bleClient && bleClient->isConnected()) bleClient->disconnect();
    WiFi.softAPdisconnect(true);
    queue_.clear();
    link_ = Link::Disconnected;
    message("Disconnected");
}

RealCamera::Options RealCamera::options(SettingId s) const {
    switch (s) {
    case S_MODE:        return OPTS(MODES);
    case S_EXPOSURE:    return OPTS(EXPOSURES);
    case S_FORMAT:      return values_[S_MODE] == 0 ? OPTS(VIDEO_FORMATS)
                               : values_[S_MODE] == 1 ? OPTS(SLOWMO_FORMATS) : Options{nullptr, 0};
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

bool RealCamera::available(SettingId s) const {
    bool video = values_[S_MODE] <= 1, manual = values_[S_EXPOSURE] == 1;
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

int RealCamera::optionCount(SettingId s) const { return options(s).count; }

const char *RealCamera::optionLabel(SettingId s, int i) const {
    Options o = options(s);
    return (i >= 0 && i < o.count) ? o.items[i] : "-";
}

// Turns a setting choice into a real DUML command over Wi-Fi. Returns false for
// settings not yet wired (they still update locally so the UI and profiles work).
void RealCamera::sendSetting(SettingId s, int index) {
    Serial.printf("send %s = %s\n", SETTINGS[s].name, optionLabel(s, index));
    switch (s) {
    case S_MODE:     datalink_.send(Duml::setMode(MODE_RAW[index])); break;
    case S_EXPOSURE: datalink_.send(Duml::setExpoMode(index == 1 ? 0x04 : 0x01)); break;
    case S_ISO:      datalink_.send(Duml::setIsoIndex(ISO_RAW[index])); break;
    case S_SHUTTER:  datalink_.send(Duml::setShutterDenom(SHUTTER_DENOM[index])); break;
    case S_EV: {
        float ev = atof(EVS[index]);
        datalink_.send(Duml::setEvByte((uint8_t)(0x10 + lroundf(ev * 3))));
        break;
    }
    case S_WB:
        if (index == 0) datalink_.send(Duml::setWhiteBalance(0x00, 0, 0));
        else datalink_.send(Duml::setWhiteBalance(0x06, WB_KELVIN[index], 0));
        break;
    case S_COLOR: datalink_.send(Duml::setColor(COLOR_RAW[index])); break;
    case S_FOCUS:
        if (index == 0) datalink_.send(Duml::setFocusMode(0x02));      // AF-C
        else if (index == 1) datalink_.send(Duml::setFocusMode(0x01)); // AF-S
        break;                                                         // MF not wired
    case S_FORMAT: {
        bool slowmo = values_[S_MODE] == 1;
        const char *label = optionLabel(s, index);
        uint8_t res = strncmp(label, "4K", 2) == 0 ? 0x10 : 0x0A; // 16:9
        int fps = atoi(strchr(label, ' ') + 1);
        uint8_t idx = fps == 24 ? 1 : fps == 25 ? 2 : fps == 30 ? 3 : fps == 48 ? 4 : fps == 50 ? 5
                      : fps == 60 ? 6 : fps == 120 ? 7 : fps == 240 ? 8 : fps == 100 ? 0x0A
                      : fps == 200 ? 0x13 : 3;
        // Slow-motion trailer: 240 -> 00 08 00, 100/120/200 -> 00 04 00; video -> 00 00 00
        uint8_t t1 = !slowmo ? 0 : (fps == 240 ? 0x08 : 0x04);
        datalink_.send(Duml::setVideoFormat(res, idx, 0, t1, 0));
        break;
    }
    case S_AUDIO:  datalink_.send(Duml::param8E(0x20, 0x00, index == 1 ? 0x01 : 0x02)); break; // Mono/Stereo
    case S_VOCAL:  datalink_.send(Duml::param8E(0x4C, 0x00, index == 1 ? 0x01 : 0x00)); break; // On/Off
    case S_GIMBAL_MODE: {
        // Follow/Tilt Lock = 02 08, FPV = 01 08, Dir Lock = 00 08 (best effort)
        static const uint8_t A[] = {0x02, 0x02, 0x01, 0x00};
        datalink_.send(Duml::gimbalModeSet(A[index], 0x08));
        break;
    }
    case S_TIMER: {
        // 6-byte countdown: 00 01 <sec> 00 00 00; Off = all zero
        static const uint8_t SECS[] = {0, 0, 3, 5, 10};
        uint8_t p[6] = {0, (uint8_t)(index ? 1 : 0), SECS[index], 0, 0, 0};
        datalink_.send(Duml::setTimer(p));
        break;
    }
    default: // S_ASPECT: opcode uncertain, left unsent
        Serial.printf("setting %s not wired\n", SETTINGS[s].name);
        break;
    }
}

void RealCamera::set(SettingId s, int index) {
    if (index < 0 || index >= optionCount(s)) return;
    values_[s] = index;
    if (s == S_MODE && values_[S_FORMAT] >= optionCount(S_FORMAT)) values_[S_FORMAT] = 0;
    queue_.push_back(s);
}

void RealCamera::setByLabel(SettingId s, const char *label) {
    Options o = options(s);
    for (int i = 0; i < o.count; i++)
        if (strcmp(o.items[i], label) == 0) {
            set(s, i);
            return;
        }
}

bool RealCamera::pending(SettingId s) const {
    for (SettingId q : queue_)
        if (q == s) return true;
    return false;
}

void RealCamera::shutter() {
    if (link_ != Link::Connected) return;
    if (isPhotoMode()) datalink_.send(Duml::shootPhoto());
    else if (datalink_.state().recording) datalink_.send(Duml::recordStop(0));
    else datalink_.send(Duml::recordStart(0));
}

uint32_t RealCamera::recordingSeconds() const {
    return link_ == Link::Connected ? datalink_.state().recordSeconds : 0;
}

int RealCamera::battery() const {
    return link_ == Link::Connected ? datalink_.state().batteryPercent : -1;
}

int RealCamera::storageUsed() const {
    if (link_ != Link::Connected) return -1;
    const CameraState &st = datalink_.state();
    if (st.storageTotalMb <= 0) return -1;
    return 100 - (int)(100L * st.storageFreeMb / st.storageTotalMb);
}

void RealCamera::gimbalSpeed(float yaw, float pitch) {
    // Integrate the stick into a target and stream absolute targets (throttled).
    uint32_t now = millis();
    float dt = lastGimbal_ ? (now - lastGimbal_) / 1000.0f : 0;
    lastGimbal_ = now;
    if (dt > 0 && dt < 0.5f) {
        yaw_ = constrain(yaw_ + yaw * 90 * dt, GIMBAL_YAW_MIN, GIMBAL_YAW_MAX);
        pitch_ = constrain(pitch_ + pitch * 60 * dt, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
    }
    if (link_ != Link::Connected) return;
    static uint32_t lastSend = 0;
    bool moving = yaw != 0 || pitch != 0;
    static bool wasMoving = false;
    if (moving && now - lastSend > 150) {
        lastSend = now;
        datalink_.send(Duml::gimbalTarget(yaw_, pitch_, 200)); // reach the target in ~0.2 s
    } else if (!moving && wasMoving) {
        datalink_.send(Duml::gimbalTimedStop());
    }
    wasMoving = moving;
}

void RealCamera::gimbalMoveTo(float yaw, float pitch, uint32_t durationMs) {
    yaw_ = constrain(yaw, GIMBAL_YAW_MIN, GIMBAL_YAW_MAX);
    pitch_ = constrain(pitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
    if (link_ == Link::Connected) datalink_.send(Duml::gimbalTarget(yaw_, pitch_, durationMs));
}

void RealCamera::gimbalRecenter() {
    yaw_ = pitch_ = 0;
    // Official recenter button (camera handles it internally): a clean reset
    if (link_ == Link::Connected) datalink_.send(Duml::gimbalRecenterCmd());
}

void RealCamera::sleep() { message("Sleep not supported on this link"); }

void RealCamera::update() {
    if (link_ == Link::Connected) {
        if (!relayOwnsBle) blePump(); // keep BLE serviced (approval acks, drain)
        Relay::update();
        // One control session at a time: give it to FrameTap, take it back when it leaves
        bool claimed = Relay::claimed();
        if (claimed && !handedOff) {
            handedOff = true;
            datalink_.close();
            message("FrameTap joined - shared control");
        } else if (!claimed && handedOff) {
            handedOff = false;
            message("Board taking control back...");
            if (datalink_.open(CAMERA_IP, [](const String &s) { Serial.println(s); })) message("Board in control");
            else message("Reconnect failed - retrying");
        }
        if (!handedOff) {
            datalink_.poll();
            // Is the board's own session still getting the camera's status pushes?
            static bool ownAlive = true;
            bool alive = millis() - datalink_.state().lastStatusMs < 3000;
            if (alive != ownAlive) {
                ownAlive = alive;
                Serial.printf("own session: %s\n", alive ? "ALIVE" : "SILENT (camera stopped talking to the board)");
            }
        }
        // Send one queued setting at a time, paced so each lands before the next
        static uint32_t lastSend = 0;
        if (!queue_.empty() && millis() - lastSend > 300) {
            lastSend = millis();
            SettingId s = queue_.front();
            queue_.erase(queue_.begin());
            sendSetting(s, values_[s]);
        }
    }
}

void RealCamera::message(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(message_, sizeof(message_), fmt, args);
    va_end(args);
    hasMessage_ = true;
    Serial.println(message_);
}

const char *RealCamera::takeMessage() {
    if (!hasMessage_) return nullptr;
    hasMessage_ = false;
    return message_;
}
