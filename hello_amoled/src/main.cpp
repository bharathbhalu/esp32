// Touch demo: tap the big button to cycle its color.
#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>

static const int I2C_SDA = 15;
static const int I2C_SCL = 14;

// SH8601 AMOLED on QSPI. GFX has no SH8601 class; CO5300 shares its command set.
static const int LCD_CS = 12, LCD_SCK = 11;
static const int LCD_D0 = 4, LCD_D1 = 5, LCD_D2 = 6, LCD_D3 = 7;
static const int LCD_W = 368, LCD_H = 448;

// TCA9554 expander drives the LCD/touch reset lines (pins 0-2)
static const uint8_t TCA9554_ADDR = 0x20;
static const uint8_t FT3168_ADDR = 0x38;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
Arduino_CO5300 *gfx = new Arduino_CO5300(bus, GFX_NOT_DEFINED, 0, false, LCD_W, LCD_H);

struct NamedColor {
    uint16_t value;
    const char *name;
};
static const NamedColor COLORS[] = {
    {RED, "RED"}, {GREEN, "GREEN"}, {BLUE, "BLUE"},
    {YELLOW, "YELLOW"}, {MAGENTA, "MAGENTA"}, {CYAN, "CYAN"},
};
static const int NUM_COLORS = sizeof(COLORS) / sizeof(COLORS[0]);

// Button rectangle
static const int BTN_X = 34, BTN_Y = 120, BTN_W = 300, BTN_H = 220;

static int colorIndex = 0;
static uint32_t tapCount = 0;

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

static void scanI2C() {
    Serial.println("I2C scan:");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) Serial.printf("  0x%02X\n", addr);
    }
}

// Returns true while a finger is down, with its coordinates
static bool readTouch(int &x, int &y) {
    Wire.beginTransmission(FT3168_ADDR);
    Wire.write(0x02); // TD_STATUS, followed by P1 XH/XL/YH/YL
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(FT3168_ADDR, (uint8_t)5) != 5) return false;
    uint8_t buf[5];
    for (int i = 0; i < 5; i++) buf[i] = Wire.read();
    if ((buf[0] & 0x0F) == 0) return false;
    x = ((buf[1] & 0x0F) << 8) | buf[2];
    y = ((buf[3] & 0x0F) << 8) | buf[4];
    return true;
}

static void drawButton(bool pressed) {
    const NamedColor &c = COLORS[colorIndex];
    gfx->fillRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, 24, c.value);
    gfx->drawRoundRect(BTN_X - 4, BTN_Y - 4, BTN_W + 8, BTN_H + 8, 28, pressed ? WHITE : BLACK);

    // Dark text on light colors, white on dark ones
    bool light = c.value == YELLOW || c.value == CYAN || c.value == GREEN;
    gfx->setTextColor(light ? BLACK : WHITE);
    gfx->setTextSize(4);
    int textW = strlen(c.name) * 24;
    gfx->setCursor(BTN_X + (BTN_W - textW) / 2, BTN_Y + BTN_H / 2 - 16);
    gfx->print(c.name);
}

static void drawStatus() {
    gfx->fillRect(0, 380, LCD_W, 50, BLACK);
    gfx->setTextColor(WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(20, 395);
    gfx->printf("Taps: %lu", tapCount);
}

void setup() {
    Serial.begin(115200);
    // Wait (up to 10 s) for a serial monitor so the startup report isn't lost
    while (!Serial && millis() < 10000) delay(10);
    delay(500);
    Serial.println("\n=== Color toggle demo ===");

    Wire.begin(I2C_SDA, I2C_SCL);
    resetPanel();
    scanI2C();

    if (!gfx->begin()) {
        Serial.println("Display init failed");
        return;
    }
    gfx->setBrightness(200);
    gfx->fillScreen(BLACK);

    gfx->setTextColor(WHITE);
    gfx->setTextSize(3);
    gfx->setCursor(40, 50);
    gfx->println("Tap to toggle");

    drawButton(false);
    drawStatus();
    Serial.println("Ready - tap the button");
}

void loop() {
    static bool wasDown = false;
    int x, y;
    bool down = readTouch(x, y);

    // Act on the press edge only, so holding a finger doesn't repeat
    if (down && !wasDown) {
        Serial.printf("touch at %d,%d\n", x, y);
        if (x >= BTN_X && x < BTN_X + BTN_W && y >= BTN_Y && y < BTN_Y + BTN_H) {
            colorIndex = (colorIndex + 1) % NUM_COLORS;
            tapCount++;
            drawButton(true);
            drawStatus();
            Serial.printf("color -> %s\n", COLORS[colorIndex].name);
        }
    } else if (!down && wasDown) {
        drawButton(false);
    }
    wasDown = down;
    delay(20);
}
