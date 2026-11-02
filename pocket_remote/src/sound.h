// UI sounds through the onboard ES8311 codec and speaker amplifier.
#pragma once
#include <Arduino.h>

bool soundBegin();         // call after Wire.begin()
void soundClick();         // short tap click
void soundEnable(bool on); // amplifier on/off (off while the screen sleeps)
void soundVolume(int volume); // codec volume, 0-100
void soundBeep(int hz, int ms); // short tone, up to 300 ms (countdowns, alerts)
