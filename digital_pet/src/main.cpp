// Bingo, a digital pet blob. Four needs (food, joy, energy, clean) drift down over
// time, including while the board is off (tracked with the PCF85063 clock), and
// are restored with the action buttons or by petting Bingo. Bingo also reacts to
// the accelerometer: slides when tilted, jumps on a flick, gets dizzy when shaken,
// floats when dropped and splats on landing.
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Arduino_GFX_Library.h>
#include "imu.h"

static const int I2C_SDA = 15;
static const int I2C_SCL = 14;

// SH8601 AMOLED on QSPI. GFX has no SH8601 class; CO5300 shares its command set.
static const int LCD_CS = 12, LCD_SCK = 11;
static const int LCD_D0 = 4, LCD_D1 = 5, LCD_D2 = 6, LCD_D3 = 7;
static const int LCD_W = 368, LCD_H = 448;

static const uint8_t TCA9554_ADDR = 0x20; // drives the LCD/touch reset lines (pins 0-2)
static const uint8_t FT3168_ADDR = 0x38;
static const uint8_t PCF85063_ADDR = 0x51;

static const uint8_t BRIGHTNESS = 200;
static const uint8_t SLEEP_BRIGHTNESS = 40;
static const char *PET_NAME = "Bingo";

// Need decay per hour while awake; energy recovers while asleep
static const float HUNGER_PER_HOUR = 8, JOY_PER_HOUR = 6, ENERGY_PER_HOUR = 5, CLEAN_PER_HOUR = 4;
static const float ENERGY_SLEEP_PER_HOUR = 40;
static const uint32_t MAX_OFFLINE_SECONDS = 24 * 3600; // cap catch-up after a long time off

// Physics (pixels, seconds)
static const float SLIDE_ACCEL = 1500;   // per g of sideways tilt
static const float GRAVITY = 1700;       // pulls Bingo back to the ground after a jump
static const float WALL = 84;            // how far Bingo can slide from center
static const float SHAKE_DIZZY = 3.0f;   // shake energy that makes Bingo dizzy
static const float FREEFALL_G = 0.35f;   // below this the board is falling

// Colors
// Purple slime, shaded from light (top) to deep (bottom edge)
static const uint16_t SLIME = 0xB41C;       // lavender (180,130,230)
static const uint16_t SLIME_LIGHT = 0xD55E; // (210,170,245)
static const uint16_t SLIME_SHADE = 0x7A35; // (120,70,170)
static const uint16_t SLIME_DARK = 0x514F;  // (80,40,120) rim and features
static const uint16_t EYE = 0x28A7;         // deep purple-black (40,20,60)
static const uint16_t BLUSH = 0xFB2F;
static const uint16_t DIRT = 0x7A84;
static const uint16_t PANEL = 0x18E3;
static const uint16_t GRAY = 0x4208;
static const uint16_t LIGHT_GRAY = 0xA514;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
Arduino_CO5300 *panel = new Arduino_CO5300(bus, GFX_NOT_DEFINED, 0, false, LCD_W, LCD_H);
// Compose each frame off-screen for smooth animation
Arduino_Canvas *gfx = new Arduino_Canvas(LCD_W, LCD_H, panel);

// ---------- pet state ----------

struct Needs {
    float food, joy, energy, clean; // 0..100
};
static Needs needs = {80, 80, 80, 80};
static bool sleeping = false;
static uint32_t bornAt = 0;     // RTC seconds
static uint32_t lastUpdate = 0; // RTC seconds of the last need update

enum Mood { M_HAPPY, M_OK, M_HUNGRY, M_SLEEPY, M_SAD, M_DIRTY, M_ASLEEP };

// Short animations triggered by actions
enum Anim { A_NONE, A_EAT, A_PLAY, A_BATH, A_PET, A_REFUSE };
static Anim anim = A_NONE;
static uint32_t animStart = 0;
static const uint32_t ANIM_MS[] = {0, 1600, 1700, 2200, 1200, 1000};

// Motion reactions (timestamps until which they show)
static uint32_t dizzyUntil = 0, annoyedUntil = 0, startledUntil = 0, oofUntil = 0;
static uint32_t heyUntil = 0, giggleUntil = 0; // touch reactions
static int dizzyCount = 0;
static uint32_t lastDizzy = 0;

// Physics state: x offset from center, height above ground, squash after impacts
static float posX = 0, velX = 0, height = 0, velY = 0, squash = 0;
static bool floating = false; // board in free fall: Bingo is weightless
static float tiltX = 0;       // smoothed sideways gravity, +1 = tilted right

// Floating particles: hearts, bubbles, crumbs, zzz, stars
enum PType { P_HEART, P_BUBBLE, P_CRUMB, P_ZZZ, P_STAR };
struct Particle {
    PType type;
    float x, y, vx, vy;
    uint32_t born, life;
};
static const int MAX_PARTICLES = 32;
static Particle particles[MAX_PARTICLES];
static int particleCount = 0;

// Eyes: where the pupils look, and blinking
static float lookX = 0, lookY = 0, lookTargetX = 0, lookTargetY = 0;
static uint32_t nextLook = 0, nextBlink = 0, blinkUntil = 0;

// ---------- layout ----------

struct Rect {
    int x, y, w, h;
    bool contains(int px, int py) const { return px >= x && px < x + w && py >= y && py < y + h; }
};
static const char *const ACTIONS[4] = {"Feed", "Play", "Bath", "Sleep"};
static Rect actionRect(int i) { return {8 + i * 90, 384, 84, 54}; }
static const int PET_CX = LCD_W / 2;
static const int GROUND_Y = 334;

// Where Bingo is on screen right now (for petting)
static Rect petRect() { return {PET_CX + (int)posX - 100, GROUND_Y - (int)height - 170, 200, 175}; }

// ---------- hardware ----------

static void tcaWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(TCA9554_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

// Pulse expander pins 0-2 low then high to reset the display and touch
static void resetPanel() {
    tcaWrite(0x03, 0xF8); // pins 0-2 as outputs
    tcaWrite(0x01, 0x00);
    delay(20);
    tcaWrite(0x01, 0x07);
    delay(300); // touch controller needs time to boot
}

// Returns 1 while a finger is down (with its coordinates), 0 when up, -1 on a read error
static int readTouch(int &x, int &y) {
    Wire.beginTransmission(FT3168_ADDR);
    Wire.write(0x02); // TD_STATUS, followed by P1 XH/XL/YH/YL
    if (Wire.endTransmission(false) != 0) return -1;
    if (Wire.requestFrom(FT3168_ADDR, (uint8_t)5) != 5) return -1;
    uint8_t buf[5];
    for (int i = 0; i < 5; i++) buf[i] = Wire.read();
    if ((buf[0] & 0x0F) == 0) return 0;
    x = ((buf[1] & 0x0F) << 8) | buf[2];
    y = ((buf[3] & 0x0F) << 8) | buf[4];
    return 1;
}

static uint8_t bcd(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }

// Seconds since 2000-01-01 from the PCF85063, or 0 if it can't be read.
// The clock doesn't need the real date: only differences are used.
static uint32_t rtcSeconds() {
    Wire.beginTransmission(PCF85063_ADDR);
    Wire.write(0x04);
    if (Wire.endTransmission(false) != 0) return 0;
    if (Wire.requestFrom(PCF85063_ADDR, (uint8_t)7) != 7) return 0;
    uint8_t r[7];
    for (int i = 0; i < 7; i++) r[i] = Wire.read();
    int sec = bcd(r[0] & 0x7F), min = bcd(r[1] & 0x7F), hour = bcd(r[2] & 0x3F);
    int day = bcd(r[3] & 0x3F), month = bcd(r[5] & 0x1F), year = bcd(r[6]);
    static const int DAYS_BEFORE[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    uint32_t days = year * 365 + (year + 3) / 4 + DAYS_BEFORE[constrain(month, 1, 12) - 1] + day - 1;
    if (month > 2 && year % 4 == 0) days++;
    return ((days * 24 + hour) * 60 + min) * 60 + sec;
}

// ---------- persistence ----------

static void savePet() {
    Preferences prefs;
    prefs.begin("pet", false);
    prefs.putBytes("needs", &needs, sizeof(needs));
    prefs.putBool("sleeping", sleeping);
    prefs.putUInt("born", bornAt);
    prefs.putUInt("updated", lastUpdate);
    prefs.end();
}

static void clampNeeds() {
    needs.food = constrain(needs.food, 0.0f, 100.0f);
    needs.joy = constrain(needs.joy, 0.0f, 100.0f);
    needs.energy = constrain(needs.energy, 0.0f, 100.0f);
    needs.clean = constrain(needs.clean, 0.0f, 100.0f);
}

// Applies `seconds` of time passing to the needs
static void ageNeeds(uint32_t seconds) {
    float h = seconds / 3600.0f;
    needs.food -= HUNGER_PER_HOUR * h;
    needs.clean -= CLEAN_PER_HOUR * h;
    if (sleeping) {
        needs.energy += ENERGY_SLEEP_PER_HOUR * h;
        needs.joy -= JOY_PER_HOUR * 0.3f * h; // sleeping is calm
    } else {
        needs.energy -= ENERGY_PER_HOUR * h;
        needs.joy -= JOY_PER_HOUR * h;
    }
    clampNeeds();
    if (sleeping && needs.energy >= 100) sleeping = false; // wakes up rested
}

static void loadPet() {
    Preferences prefs;
    prefs.begin("pet", true);
    bool exists = prefs.getBytes("needs", &needs, sizeof(needs)) == sizeof(needs);
    sleeping = prefs.getBool("sleeping", false);
    bornAt = prefs.getUInt("born", 0);
    lastUpdate = prefs.getUInt("updated", 0);
    prefs.end();

    uint32_t now = rtcSeconds();
    if (!exists) {
        needs = {80, 80, 80, 80};
        bornAt = lastUpdate = now;
        Serial.println("Bingo is born");
        return;
    }
    // Catch up on time spent switched off (ignore a clock that went backwards)
    if (now > lastUpdate) {
        uint32_t away = min(now - lastUpdate, MAX_OFFLINE_SECONDS);
        ageNeeds(away);
        Serial.printf("Bingo was alone for %lu min\n", away / 60);
    }
    lastUpdate = now;
}

// ---------- mood ----------

static Mood mood() {
    if (sleeping) return M_ASLEEP;
    if (needs.food < 30) return M_HUNGRY;
    if (needs.energy < 25) return M_SLEEPY;
    if (needs.clean < 30) return M_DIRTY;
    if (needs.joy < 30) return M_SAD;
    float avg = (needs.food + needs.joy + needs.energy + needs.clean) / 4;
    return avg > 65 ? M_HAPPY : M_OK;
}

static bool isDizzy(uint32_t now) { return now < dizzyUntil; }
static bool isStartled(uint32_t now) { return now < startledUntil || floating; }
static bool isAnnoyed(uint32_t now) { return now < annoyedUntil; }

// ---------- particles ----------

static void spawn(PType type, float x, float y, float vx, float vy, uint32_t life) {
    if (particleCount == MAX_PARTICLES) return;
    particles[particleCount++] = {type, x, y, vx, vy, millis(), life};
}

static void updateParticles(float dt) {
    uint32_t now = millis();
    for (int i = 0; i < particleCount;) {
        Particle &p = particles[i];
        if (now - p.born > p.life) {
            particles[i] = particles[--particleCount];
            continue;
        }
        p.x += p.vx * dt;
        p.y += p.vy * dt;
        if (p.type == P_CRUMB) p.vy += 300 * dt; // crumbs fall
        i++;
    }
}

static void drawHeart(int x, int y, int s, uint16_t c) {
    gfx->fillCircle(x - s / 2, y, s / 2 + 1, c);
    gfx->fillCircle(x + s / 2, y, s / 2 + 1, c);
    gfx->fillTriangle(x - s - 1, y + 1, x + s + 1, y + 1, x, y + s + 3, c);
}

static void drawStar(int x, int y, uint16_t c) {
    gfx->drawFastHLine(x - 6, y, 13, c);
    gfx->drawFastVLine(x, y - 6, 13, c);
    gfx->drawLine(x - 4, y - 4, x + 4, y + 4, c);
    gfx->drawLine(x - 4, y + 4, x + 4, y - 4, c);
}

static void drawParticles() {
    for (int i = 0; i < particleCount; i++) {
        const Particle &p = particles[i];
        switch (p.type) {
        case P_HEART:  drawHeart(p.x, p.y, 7, BLUSH); break;
        case P_BUBBLE: gfx->drawCircle(p.x, p.y, 7, 0xBF7F); gfx->drawPixel(p.x - 3, p.y - 3, WHITE); break;
        case P_CRUMB:  gfx->fillCircle(p.x, p.y, 3, 0xFD20); break;
        case P_STAR:   drawStar(p.x, p.y, YELLOW); break;
        case P_ZZZ:
            gfx->setTextColor(LIGHT_GRAY);
            gfx->setTextSize(2);
            gfx->setCursor(p.x, p.y);
            gfx->print("z");
            break;
        }
    }
}

// ---------- drawing ----------

// Need bar in the purple header: lavender fill, turning pink then red when low
static void drawNeedBar(int i, const char *label, float value) {
    int x = 14 + i * 88, y = 12;
    gfx->setTextSize(1);
    gfx->setTextColor(SLIME_LIGHT);
    gfx->setCursor(x, y);
    gfx->print(label);
    uint16_t c = value > 60 ? SLIME_LIGHT : value > 30 ? BLUSH : 0xF800;
    gfx->fillRoundRect(x, y + 12, 78, 10, 4, SLIME_DARK);
    gfx->fillRoundRect(x, y + 12, max(4, (int)(78 * value / 100)), 10, 4, c);
}

static void drawHeader() {
    gfx->fillRoundRect(4, 2, LCD_W - 8, 42, 12, 0x3008); // deep purple panel (50,0,64)
    drawNeedBar(0, "FOOD", needs.food);
    drawNeedBar(1, "JOY", needs.joy);
    drawNeedBar(2, "ENERGY", needs.energy);
    drawNeedBar(3, "CLEAN", needs.clean);
    gfx->setTextSize(3);
    gfx->setTextColor(SLIME_LIGHT);
    gfx->setCursor(PET_CX - strlen(PET_NAME) * 9, 54);
    gfx->print(PET_NAME);
}

static void drawActions() {
    for (int i = 0; i < 4; i++) {
        Rect r = actionRect(i);
        bool active = i == 3 && sleeping;
        gfx->fillRoundRect(r.x, r.y, r.w, r.h, 14, active ? 0x897F : PANEL);
        const char *label = i == 3 && sleeping ? "Wake" : ACTIONS[i];
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE);
        gfx->setCursor(r.x + (r.w - strlen(label) * 12) / 2, r.y + 19);
        gfx->print(label);
    }
}

// Swirly dizzy eye (white swirl on the dark eye)
static void drawSpiral(int x, int y, uint32_t now) {
    float spin = now / 120.0f;
    for (int r = 4; r <= 13; r += 4) {
        float a0 = spin * 57.3f + r * 40;
        gfx->fillArc(x, y, r + 2, r, a0, a0 + 250, WHITE);
    }
}

// Big glossy kawaii eyes: dark ovals with two shine dots that follow the gaze
static void drawEyes(int cx, int cy, Mood m, uint32_t now) {
    bool blink = now < blinkUntil;
    for (int side = -1; side <= 1; side += 2) {
        int x = cx + side * 30, y = cy - 6;
        if (isDizzy(now)) {
            gfx->fillEllipse(x, y, 16, 19, EYE);
            drawSpiral(x, y, now);
        } else if (isStartled(now)) {
            gfx->fillEllipse(x, y, 17, 21, WHITE);
            gfx->fillCircle(x + lookX * 4, y + lookY * 4, 6, EYE);
        } else if (blink || m == M_ASLEEP) {
            gfx->fillArc(x, y - 4, 14, 11, 20, 160, SLIME_DARK); // closed, gently curved
        } else if (anim == A_PLAY || (m == M_HAPPY && anim == A_PET)) {
            gfx->fillArc(x, y + 8, 14, 11, 200, 340, SLIME_DARK); // joyful ^ ^
        } else {
            int h = m == M_SLEEPY ? 11 : 20;
            int ox = lookX * 3, oy = lookY * 3;
            gfx->fillEllipse(x + ox, y + oy, 15, h, EYE);
            // Two shine dots: big top-left, small bottom-right
            gfx->fillCircle(x + ox - 5, y + oy - h / 2 + 2, 5, WHITE);
            gfx->fillCircle(x + ox + 6, y + oy + h / 3, 2, WHITE);
            if (m == M_SLEEPY) gfx->fillRect(x - 17, y - 22, 34, 14, SLIME_LIGHT); // heavy eyelids
        }
        if (isAnnoyed(now)) {
            // Angry brows slanting down toward the middle
            for (int k = 0; k < 3; k++)
                gfx->drawLine(x - 14 * side, y - 28 + k, x + 8 * side, y - 20 + k, SLIME_DARK);
        }
    }
}

// Small kawaii mouth
static void drawMouth(int cx, int cy, Mood m, uint32_t now) {
    int y = cy + 20;
    if (anim == A_EAT) {
        int open = 3 + (int)(5 * fabsf(sinf((now - animStart) / 90.0f)));
        gfx->fillEllipse(cx, y + 2, 8, open, EYE); // chomping
        return;
    }
    if (isStartled(now)) {
        gfx->fillEllipse(cx, y + 3, 6, 8, EYE);
        return;
    }
    if (isDizzy(now)) {
        for (int i = -9; i < 9; i++) gfx->fillCircle(cx + i, y + 2 + (int)(2.5f * sinf(i / 2.0f + now / 80.0f)), 1, SLIME_DARK);
        return;
    }
    if (isAnnoyed(now) || m == M_SAD || m == M_DIRTY) {
        gfx->fillArc(cx, y + 9, 9, 7, 200, 340, SLIME_DARK); // little frown
        return;
    }
    switch (m) {
    case M_HAPPY:
        // "w" cat mouth
        gfx->fillArc(cx - 5, y - 2, 6, 4, 20, 160, SLIME_DARK);
        gfx->fillArc(cx + 5, y - 2, 6, 4, 20, 160, SLIME_DARK);
        break;
    case M_OK:     gfx->fillArc(cx, y - 4, 8, 6, 30, 150, SLIME_DARK); break;
    case M_HUNGRY: gfx->fillEllipse(cx, y + 2, 5, 7, EYE); break;
    case M_SLEEPY: gfx->fillEllipse(cx, y + 2, 4, 2, EYE); break;
    default:       gfx->fillCircle(cx, y + 2, 2, SLIME_DARK); break;
    }
}

static void drawBubbleText(int x, int y, const char *text, uint16_t color) {
    gfx->setTextSize(2);
    gfx->setTextColor(color);
    gfx->setCursor(constrain(x, 4, LCD_W - 4 - (int)strlen(text) * 12), y);
    gfx->print(text);
}

// Teardrop slime: a dome body plus a tapered tip on top, built from overlapping
// shapes in three shades so it reads as glossy jelly
static void drawDrop(int cx, int bodyY, int rx, int ry, int tipLean, int inset, uint16_t color) {
    rx -= inset;
    ry -= inset;
    gfx->fillEllipse(cx, bodyY, rx, ry, color);
    gfx->fillEllipse(cx, bodyY + ry - 16, rx - 4, 18, color); // flatter bottom
    // Tip: a cone of circles from the top of the dome up to a rounded point
    int baseY = bodyY - ry + 26, tipY = bodyY - ry - 34 + inset;
    for (int i = 0; i <= 12; i++) {
        float t = i / 12.0f;
        int r = (int)(36 - 30 * t) - inset / 2;
        if (r < 2) r = 2;
        gfx->fillCircle(cx + (int)(tipLean * t * t), baseY + (int)((tipY - baseY) * t), r, color);
    }
}

static void drawPet(uint32_t now) {
    Mood m = mood();
    float t = now / 1000.0f;

    // Breathing, plus squash from landings and wall hits
    float breath = sinf(t * (m == M_ASLEEP ? 1.6f : 3.0f));
    float sx = 1 + 0.04f * breath + squash, sy = 1 - 0.04f * breath - squash;
    uint32_t a = now - animStart;
    float wobble = 0;
    if (anim == A_PET) wobble = sinf(a / 60.0f) * 0.08f * (1 - a / (float)ANIM_MS[A_PET]);
    if (anim == A_REFUSE) wobble = sinf(a / 50.0f) * 0.06f;
    if (isDizzy(now)) wobble += sinf(t * 9) * 0.06f;
    if (height > 0) { // stretched while airborne
        sx -= 0.06f;
        sy += 0.06f;
    }
    sx += wobble;
    sy -= wobble;

    int rx = 92 * sx, ry = 70 * sy;
    int cx = PET_CX + (int)posX;
    int lean = (int)(tiltX * 14);
    // The tip sways with tilt and wobble, lagging like jelly
    int tipLean = (int)(-tiltX * 18 + wobble * 120 + sinf(t * 2.2f) * 3);

    // Shadow: smaller the higher Bingo is
    float shadow = max(0.35f, 1 - height / 220.0f);
    gfx->fillEllipse(cx, GROUND_Y, 82 * shadow, 10 * shadow, 0x1863);

    int bodyY = GROUND_Y - ry - (int)height;
    int bx = cx + lean / 2;
    // Glossy layers: deep rim, main colour shifted up, light top
    drawDrop(bx, bodyY, rx, ry, tipLean, 0, SLIME_DARK);
    drawDrop(bx, bodyY - 2, rx, ry, tipLean, 4, SLIME_SHADE);
    drawDrop(bx, bodyY - 6, rx, ry, tipLean, 10, SLIME);
    gfx->fillEllipse(bx - 6, bodyY - ry / 3, rx - 34, ry / 2, SLIME_LIGHT);
    gfx->fillEllipse(bx - 2, bodyY - ry / 3 + 8, rx - 30, ry / 2 - 4, SLIME);

    // Shine: curved white streak top-left plus a dot
    gfx->fillArc(bx - rx / 3, bodyY - ry / 3, rx / 3, rx / 3 - 6, 200, 250, WHITE);
    gfx->fillCircle(bx - rx / 2 + 2, bodyY - ry / 6, 4, WHITE);
    gfx->fillCircle(bx + tipLean / 2 - 6, bodyY - ry - 8, 3, WHITE); // glint on the tip

    if (needs.clean < 50) {
        int spots = needs.clean < 25 ? 5 : 3;
        static const int8_t SPOTS[5][2] = {{-45, 22}, {40, 30}, {14, -30}, {-18, 44}, {58, 4}};
        for (int i = 0; i < spots; i++)
            gfx->fillCircle(bx + SPOTS[i][0] * sx, bodyY + SPOTS[i][1] * sy, 6, DIRT);
    }

    int faceX = cx + lean, faceY = bodyY + 12;
    drawEyes(faceX, faceY, m, now);
    if (!isDizzy(now) && !isStartled(now) && m != M_SAD && m != M_DIRTY) {
        // Rosy cheeks with tiny blush lines
        uint16_t cheek = m == M_HAPPY || anim == A_PET ? BLUSH : 0xC9F9;
        for (int side = -1; side <= 1; side += 2) {
            int x = faceX + side * 56, y = faceY + 14;
            gfx->fillEllipse(x, y, 13, 7, cheek);
            if (m == M_HAPPY || anim == A_PET)
                for (int k = -1; k <= 1; k++) gfx->drawLine(x + k * 5 - 2, y + 3, x + k * 5 + 2, y - 3, 0xF8F0);
        }
    }
    drawMouth(faceX, faceY, m, now);

    // Speech
    int textY = bodyY - ry - 64;
    if (floating) drawBubbleText(cx + 40, textY, "wheee!", WHITE);
    else if (now < oofUntil) drawBubbleText(cx + 40, textY, "oof!", WHITE);
    else if (now < heyUntil) drawBubbleText(cx + 40, textY, "hey!", 0xFD20);
    else if (now < giggleUntil) drawBubbleText(cx + 30, textY, "hehehe!", BLUSH);
    else if (isAnnoyed(now)) drawBubbleText(cx + 30, textY, "stop it!", 0xFD20);
    else if (isDizzy(now)) drawBubbleText(cx + 40, textY, "@_@", LIGHT_GRAY);
    else if (isStartled(now)) drawBubbleText(cx + 40, textY, "!!", YELLOW);
    else if (m == M_HUNGRY) drawBubbleText(cx + 30, textY, "hungry...", LIGHT_GRAY);
    else if (m == M_DIRTY) drawBubbleText(cx + 30, textY, "itchy...", LIGHT_GRAY);
    else if (m == M_SAD) drawBubbleText(cx + 30, textY, "lonely...", LIGHT_GRAY);
}

static void draw(uint32_t now) {
    gfx->fillScreen(BLACK);
    drawHeader();
    drawPet(now);
    drawParticles();
    drawActions();
    gfx->flush();
}

// ---------- actions ----------

static void startAnim(Anim a) {
    anim = a;
    animStart = millis();
}

static bool onGround() { return height <= 0 && velY <= 0; }

static void jump(float strength) {
    if (!onGround() || sleeping) return;
    velY = strength;
}

static void doAction(int i) {
    if (sleeping && i != 3) return; // asleep: only Wake works
    float px = PET_CX + posX, py = GROUND_Y - 100;
    switch (i) {
    case 0: // Feed
        if (needs.food > 95) {
            startAnim(A_REFUSE);
            break;
        }
        needs.food += 25;
        needs.joy += 3;
        startAnim(A_EAT);
        for (int k = 0; k < 6; k++) spawn(P_CRUMB, px + random(-20, 20), py + 50, random(-60, 60), -random(60, 140), 900);
        break;
    case 1: // Play: a few happy hops
        if (needs.energy < 10) {
            startAnim(A_REFUSE); // too tired
            break;
        }
        needs.joy += 20;
        needs.energy -= 10;
        needs.food -= 4;
        startAnim(A_PLAY);
        jump(520);
        for (int k = 0; k < 3; k++) spawn(P_HEART, px + random(-70, 70), py - 40, random(-20, 20), -60, 1400);
        break;
    case 2: // Bath
        needs.clean = 100;
        needs.joy += 5;
        startAnim(A_BATH);
        for (int k = 0; k < 14; k++)
            spawn(P_BUBBLE, px + random(-100, 100), py + random(-40, 80), random(-15, 15), -random(30, 90), 2000);
        break;
    case 3: // Sleep / wake
        sleeping = !sleeping;
        panel->setBrightness(sleeping ? SLEEP_BRIGHTNESS : BRIGHTNESS);
        break;
    }
    clampNeeds();
    savePet();
}

static void petBingo() {
    if (sleeping) return;
    needs.joy += 5;
    clampNeeds();
    startAnim(A_PET);
    spawn(P_HEART, PET_CX + posX + random(-40, 40), GROUND_Y - 180, random(-25, 25), -70, 1200);
}

// ---------- touch ----------
// Tap Bingo = poke (eyes: "hey!", body: giggle + heart), stroke = petting, fast
// rubbing = tickle, press and hold = squish then boing on release, hold then move =
// pick up and drag, let go while moving = fling.

static const uint32_t HOLD_MS = 350;
static const int MOVE_MIN = 12;

enum Grab { G_NONE, G_TOUCH, G_STROKE, G_SQUISH, G_DRAG };
static Grab grab = G_NONE;
static uint32_t grabStart = 0, lastStroke = 0;
static int grabX0, grabY0, fingerX, fingerY;
static float grabPosX, grabHeight, fingerVX = 0, fingerVY = 0, rubDistance = 0;

static bool onEyes(int x, int y) {
    int faceX = PET_CX + (int)posX, eyeY = GROUND_Y - (int)height - 70;
    return abs(y - eyeY) < 28 && abs(abs(x - faceX) - 30) < 24;
}

static void touchStart(int x, int y, uint32_t now) {
    if (sleeping || !petRect().contains(x, y)) return;
    grab = G_TOUCH;
    grabStart = now;
    grabX0 = fingerX = x;
    grabY0 = fingerY = y;
    grabPosX = posX;
    grabHeight = height;
    fingerVX = fingerVY = 0;
    rubDistance = 0;
}

static void touchMove(int x, int y, float dt, uint32_t now) {
    if (grab == G_NONE) return;
    int dx = x - fingerX, dy = y - fingerY;
    if (dt > 0) {
        fingerVX += (dx / dt - fingerVX) * 0.5f;
        fingerVY += (dy / dt - fingerVY) * 0.5f;
    }
    fingerX = x;
    fingerY = y;
    bool moved = abs(x - grabX0) > MOVE_MIN || abs(y - grabY0) > MOVE_MIN;

    switch (grab) {
    case G_TOUCH:
        if (moved) grab = G_STROKE;
        else if (now - grabStart > HOLD_MS) grab = G_SQUISH;
        break;
    case G_SQUISH:
        squash = max(squash, 0.28f); // pressed down under the finger
        if (moved) grab = G_DRAG;    // held, then moved: picked up
        break;
    case G_STROKE:
        // Stroking pets; rubbing back and forth quickly tickles
        rubDistance = rubDistance * powf(0.3f, dt) + abs(dx) + abs(dy);
        if (rubDistance > 500 && now > giggleUntil) {
            giggleUntil = now + 1800;
            needs.joy += 4;
            clampNeeds();
            startAnim(A_PLAY);
        } else if (now - lastStroke > 450) {
            lastStroke = now;
            petBingo();
        }
        break;
    case G_DRAG:
        // Bingo follows the finger; lifting up raises it off the ground
        posX = constrain(grabPosX + (x - grabX0), -WALL, WALL);
        height = constrain(grabHeight + (grabY0 - y), 0.0f, 150.0f);
        velX = velY = 0;
        break;
    default:
        break;
    }
}

static void touchEnd(uint32_t now) {
    switch (grab) {
    case G_TOUCH: // quick tap = poke
        if (onEyes(grabX0, grabY0)) {
            heyUntil = now + 1200;
            blinkUntil = now + 300;
        } else {
            squash = max(squash, 0.15f);
            petBingo();
        }
        break;
    case G_SQUISH: // let go after pressing: springs up
        jump(480);
        break;
    case G_DRAG: // let go while moving: fling
        velX = constrain(fingerVX, -900.0f, 900.0f);
        velY = constrain(-fingerVY, -400.0f, 700.0f);
        if (fabsf(velX) > 400 || velY > 400) oofUntil = 0, heyUntil = 0;
        break;
    default:
        break;
    }
    grab = G_NONE;
}

static bool dragging() { return grab == G_DRAG; }

// ---------- motion ----------

static void wakeStartled(uint32_t now) {
    if (!sleeping) return;
    sleeping = false;
    panel->setBrightness(BRIGHTNESS);
    startledUntil = now + 1500;
    annoyedUntil = now + 3000; // grumpy at being shaken awake
    savePet();
}

// Reads the accelerometer and turns it into reactions and physics forces
static void updateMotion(float dt, uint32_t now) {
    static float shakeEnergy = 0;
    static uint32_t freefallSince = 0, lastJolt = 0;
    static bool upsideDown = false;
    float ax, ay, az;
    if (!imuRead(ax, ay, az)) return;
    float g = sqrtf(ax * ax + ay * ay + az * az);
    float jolt = fabsf(g - 1);

    // Sideways gravity in screen terms (+ = tilted right), from the axis mapping
    // found for the Pocket remote's tilt control
    tiltX += (constrain(-ay, -1.0f, 1.0f) - tiltX) * 0.3f;

    // Free fall: the board is dropped or tossed
    if (g < FREEFALL_G) {
        if (!freefallSince) freefallSince = now;
        if (now - freefallSince > 60 && !floating) {
            floating = true;
            velY = max(velY, 220.0f); // drifts up, weightless
        }
    } else {
        freefallSince = 0;
        if (floating) {
            // Caught or landed: Bingo comes crashing down
            floating = false;
            velY = -700;
            dizzyUntil = now + 2000;
        }
    }

    // Upside down (screen facing the floor), then turned back
    if (az > 0.6f) upsideDown = true;
    else if (upsideDown && az < -0.3f) {
        upsideDown = false;
        startledUntil = now + 2000;
        wakeStartled(now);
    }

    // Sharp jolts: one flick = jump, repeated = shaking -> dizzy -> annoyed
    shakeEnergy = max(0.0f, shakeEnergy - 2.5f * dt);
    if (jolt > 0.55f && !floating) {
        bool firstJolt = shakeEnergy < 0.4f && now - lastJolt > 400;
        shakeEnergy += jolt;
        lastJolt = now;
        if (sleeping && shakeEnergy > 1.2f) wakeStartled(now);
        else if (firstJolt) jump(380 + min(jolt, 1.5f) * 250);
    }
    if (shakeEnergy > SHAKE_DIZZY && !isDizzy(now)) {
        dizzyCount = now - lastDizzy < 10000 ? dizzyCount + 1 : 1;
        lastDizzy = now;
        dizzyUntil = now + 3000;
        shakeEnergy = 0;
        for (int k = 0; k < 4; k++)
            spawn(P_STAR, PET_CX + posX + random(-60, 60), GROUND_Y - 190, random(-40, 40), -random(10, 40), 1500);
        if (dizzyCount >= 2) {
            annoyedUntil = now + 4000;
            needs.joy -= 5;
            clampNeeds();
        }
    }
}

static bool dragging();

static void updatePhysics(float dt, uint32_t now) {
    if (dragging()) { // the finger holds Bingo; just let squash spring back
        squash *= powf(0.002f, dt);
        return;
    }
    // Sliding: tilt pushes Bingo sideways; more grip on the ground than in the air
    velX += tiltX * SLIDE_ACCEL * dt;
    velX *= powf(onGround() ? 0.08f : 0.5f, dt);
    posX += velX * dt;
    if (fabsf(posX) > WALL) {
        bool hard = fabsf(velX) > 220;
        posX = copysignf(WALL, posX);
        if (hard) {
            squash = max(squash, min(0.3f, fabsf(velX) / 1500));
            oofUntil = now + 700;
        }
        velX = -velX * 0.35f; // bounce off the edge
    }

    // Jumping / falling
    if (!floating) velY -= GRAVITY * dt;
    else velY *= powf(0.3f, dt); // weightless drift slows down
    height += velY * dt;
    if (height <= 0) {
        if (velY < -300) {
            // Landing splat, bigger the faster it fell
            squash = max(squash, min(0.4f, -velY / 1800));
            if (velY < -600) oofUntil = now + 800;
        }
        height = 0;
        velY = 0;
        // Play keeps hopping while its animation runs
        if (anim == A_PLAY) jump(520);
    }
    height = min(height, 150.0f);
    squash *= powf(0.002f, dt); // springs back
}

// ---------- main ----------

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);
    resetPanel();
    if (!gfx->begin()) {
        Serial.println("Display init failed");
        return;
    }
    if (!imuBegin()) Serial.println("motion sensor missing");
    loadPet();
    panel->setBrightness(sleeping ? SLEEP_BRIGHTNESS : BRIGHTNESS);
    savePet();
    Serial.printf("food %.0f joy %.0f energy %.0f clean %.0f%s\n", needs.food, needs.joy, needs.energy,
                  needs.clean, sleeping ? " (asleep)" : "");
}

void loop() {
    static uint32_t lastFrame = millis(), lastTick = millis(), lastSave = millis();
    static bool down = false;
    static int startX, startY;
    uint32_t now = millis();
    float dt = min((now - lastFrame) / 1000.0f, 0.05f);
    lastFrame = now;

    // Needs drift every few seconds while running
    if (now - lastTick >= 5000) {
        bool wasSleeping = sleeping;
        ageNeeds((now - lastTick) / 1000);
        lastTick = now;
        lastUpdate = rtcSeconds();
        if (wasSleeping && !sleeping) panel->setBrightness(BRIGHTNESS); // woke up rested
    }
    if (now - lastSave >= 60000) {
        lastSave = now;
        savePet();
    }

    // Touch: buttons, or poke / stroke / tickle / squish / drag Bingo
    int x, y;
    int touch = readTouch(x, y);
    if (touch == 1) {
        if (!down) {
            startX = x;
            startY = y;
            touchStart(x, y, now);
        }
        touchMove(x, y, dt, now);
        down = true;
    } else if (touch == 0 && down) {
        down = false;
        touchEnd(now);
        for (int i = 0; i < 4; i++)
            if (actionRect(i).contains(startX, startY)) doAction(i);
    }

    updateMotion(dt, now);
    updatePhysics(dt, now);

    // Eyes wander and blink; when tilted they look toward the low side
    if (now > nextLook) {
        lookTargetX = random(-100, 101) / 100.0f;
        lookTargetY = random(-60, 61) / 100.0f;
        nextLook = now + random(800, 2500);
    }
    float targetX = fabsf(tiltX) > 0.15f ? constrain(tiltX * 2.5f, -1.0f, 1.0f) : lookTargetX;
    lookX += (targetX - lookX) * 0.15f;
    lookY += (lookTargetY - lookY) * 0.15f;
    if (now > nextBlink) {
        blinkUntil = now + 140;
        nextBlink = now + random(2500, 5500);
    }

    // Floating zzz while asleep
    static uint32_t lastZ = 0;
    if (sleeping && now - lastZ > 900) {
        lastZ = now;
        spawn(P_ZZZ, PET_CX + posX + 60, GROUND_Y - 130, 18, -30, 2200);
    }

    if (anim != A_NONE && now - animStart > ANIM_MS[anim]) anim = A_NONE;

    updateParticles(dt);
    draw(now);
}
