#include "imu.h"
#include <Wire.h>

static const uint8_t QMI8658_ADDR = 0x6B;

static bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(QMI8658_ADDR, len) != len) return false;
    for (int i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

bool imuBegin() {
    uint8_t id = 0;
    if (!readRegs(0x00, &id, 1) || id != 0x05) { // WHO_AM_I
        Serial.printf("QMI8658 not found (id 0x%02X)\n", id);
        return false;
    }
    writeReg(0x02, 0x40); // CTRL1: register address auto-increment
    writeReg(0x03, 0x05); // CTRL2: accel +-2 g, ~250 Hz
    writeReg(0x08, 0x01); // CTRL7: accelerometer on
    delay(20);
    return true;
}

bool imuRead(float &ax, float &ay, float &az) {
    uint8_t b[6];
    if (!readRegs(0x35, b, 6)) return false; // AX_L .. AZ_H
    ax = (int16_t)(b[0] | (b[1] << 8)) / 16384.0f;
    ay = (int16_t)(b[2] | (b[3] << 8)) / 16384.0f;
    az = (int16_t)(b[4] | (b[5] << 8)) / 16384.0f;
    return true;
}
