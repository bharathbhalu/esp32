// Gimbal moves: one-tap presets, and record/replay of a steered path.
// Playback follows OpenPocketCine's Motion Control approach: move to the start,
// hold briefly, then send timed absolute targets (04/14) leg by leg.
#pragma once
#include "camera.h"

namespace Moves {

static const int PRESET_COUNT = 7;
static const int SLOT_COUNT = 4;
static const uint32_t SAMPLE_MS = 100;   // recording rate: 10 Hz
static const int MAX_SAMPLES = 300;      // 30 s per recording

void begin();

const char *presetName(int i);

// Saved recordings
bool slotUsed(int slot);
float slotSeconds(int slot);
void deleteSlot(int slot);
bool hasFreeSlot();

// Recording: samples the camera's gimbal angles while you steer
void startRecording();
bool recording();
float recordedSeconds();
bool recordingFull();
int stopRecording(); // saves to the first free slot; returns it, or -1 if too short

// Playback
void playPreset(Camera &cam, int preset, uint32_t durationMs, bool withVideo);
void playSlot(Camera &cam, int slot, bool withVideo);
bool playing();
String status(); // e.g. "Pan L>R  2.4 / 6.0 s"
void stop(Camera &cam);

// Call every loop: drives playback and recording
void update(Camera &cam);

} // namespace Moves
