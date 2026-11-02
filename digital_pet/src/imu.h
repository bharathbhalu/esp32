// Minimal QMI8658 accelerometer reader (onboard IMU, I2C 0x6B).
#pragma once
#include <Arduino.h>

bool imuBegin();
// Acceleration in g; false if the read failed
bool imuRead(float &ax, float &ay, float &az);
