#include "power.h"
#include <Wire.h>

static const uint8_t AXP2101_ADDR = 0x34;

static bool readReg(uint8_t reg, uint8_t &val) {
    Wire.beginTransmission(AXP2101_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(AXP2101_ADDR, (uint8_t)1) != 1) return false;
    val = Wire.read();
    return true;
}

static bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(AXP2101_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool powerBegin() {
    uint8_t ctrl;
    if (!readReg(0x18, ctrl)) {
        Serial.println("AXP2101 not found");
        return false;
    }
    writeReg(0x18, ctrl | 0x08); // fuel gauge on (only touches the gauge enable bit)
    return true;
}

bool powerUsbPresent() {
    uint8_t status0;
    return readReg(0x00, status0) && (status0 & 0x20); // VBUS good
}

void powerOff() {
    uint8_t ctrl;
    if (!readReg(0x10, ctrl)) return;
    writeReg(0x10, ctrl | 0x01); // COMMON_CONFIG bit0: software power-off
}

BatteryStatus powerRead() {
    BatteryStatus s = {false, false, -1};
    uint8_t status0, status1, pct;
    if (!readReg(0x00, status0) || !readReg(0x01, status1)) return s;
    s.present = status0 & 0x08;                 // BATEXST
    s.charging = ((status1 >> 5) & 0x03) == 0x01; // 01 = charging
    if (s.present && readReg(0xA4, pct) && pct <= 100) s.percent = pct;
    return s;
}
