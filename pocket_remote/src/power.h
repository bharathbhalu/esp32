// Board battery from the AXP2101 power chip (I2C 0x34): fuel-gauge percentage
// and charge state.
#pragma once
#include <Arduino.h>

struct BatteryStatus {
    bool present;
    bool charging;
    int percent; // 0..100, -1 if unknown
};

bool powerBegin();
BatteryStatus powerRead();
bool powerUsbPresent();
void powerOff(); // full shutdown via the AXP2101; PWR short press turns it back on
