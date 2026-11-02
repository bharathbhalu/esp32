#include "sound.h"
#include <ESP_I2S.h>
#include "es8311.h"

// Pins from Waveshare's ESP32-S3-Touch-AMOLED-1.8 ES8311 example
static const int I2S_MCK = 16, I2S_BCK = 9, I2S_WS = 45, I2S_DO = 8, I2S_DI = 10;
static const int AMP_ENABLE = 46;

static const int SAMPLE_RATE = 16000;
static const int DMA_FRAMES = 240; // Arduino ESP_I2S default dma_frame_num

static I2SClass i2s;
static es8311_handle_t codec = nullptr;
static bool ready = false;

// One click = 10 ms 1.8 kHz blip with a 1 ms fade-in, padded with silence to
// exactly two DMA buffers so it never straddles a buffer boundary mid-sound.
static const int CLICK_FRAMES = DMA_FRAMES * 2;
static const int TONE_FRAMES = SAMPLE_RATE * 10 / 1000;
static const int FADE_IN_FRAMES = SAMPLE_RATE / 1000;
static int16_t click[CLICK_FRAMES * 2]; // stereo 16-bit

bool soundBegin() {
    pinMode(AMP_ENABLE, OUTPUT);
    digitalWrite(AMP_ENABLE, LOW);

    i2s.setPins(I2S_BCK, I2S_WS, I2S_DO, I2S_DI, I2S_MCK);
    if (!i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
        Serial.println("I2S init failed");
        return false;
    }

    codec = es8311_create(0, ES8311_ADDRESS_0);
    const es8311_clock_config_t clk = {
        .mclk_inverted = false,
        .sclk_inverted = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency = SAMPLE_RATE * 256,
        .sample_frequency = SAMPLE_RATE,
    };
    if (!codec || es8311_init(codec, &clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16) != ESP_OK) {
        Serial.println("ES8311 init failed");
        return false;
    }

    memset(click, 0, sizeof(click));
    for (int i = 0; i < TONE_FRAMES; i++) {
        float attack = min(1.0f, (float)i / FADE_IN_FRAMES);
        float decay = 1.0f - (float)i / TONE_FRAMES;
        int16_t v = (int16_t)(5000 * attack * decay * decay * sinf(2 * PI * 1800 * i / SAMPLE_RATE));
        click[i * 2] = click[i * 2 + 1] = v;
    }

    digitalWrite(AMP_ENABLE, HIGH);
    ready = true;
    return true;
}

void soundVolume(int volume) {
    if (codec) es8311_voice_volume_set(codec, volume, NULL);
}

void soundClick() {
    // Keep clicks apart so they never stack into an overlapping sound
    static uint32_t last = 0;
    if (!ready || millis() - last < 80) return;
    last = millis();
    i2s.write((const uint8_t *)click, sizeof(click));
}

void soundBeep(int hz, int ms) {
    if (!ready) return;
    // Rounded to whole DMA buffers, like the click, with soft 3 ms edges
    static const int MAX_FRAMES = SAMPLE_RATE * 300 / 1000;
    static int16_t tone[((MAX_FRAMES + DMA_FRAMES - 1) / DMA_FRAMES * DMA_FRAMES) * 2];
    int frames = min(SAMPLE_RATE * ms / 1000, MAX_FRAMES);
    int padded = (frames + DMA_FRAMES - 1) / DMA_FRAMES * DMA_FRAMES;
    int edge = SAMPLE_RATE * 3 / 1000;
    for (int i = 0; i < padded; i++) {
        float env = i < frames ? min(1.0f, min(i, frames - i) / (float)edge) : 0;
        int16_t v = (int16_t)(4500 * env * sinf(2 * PI * hz * i / SAMPLE_RATE));
        tone[i * 2] = tone[i * 2 + 1] = v;
    }
    i2s.write((const uint8_t *)tone, padded * 4);
}

void soundEnable(bool on) {
    if (ready) digitalWrite(AMP_ENABLE, on ? HIGH : LOW);
}
