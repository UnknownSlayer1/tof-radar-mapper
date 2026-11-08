// main.cpp
// 360 Degree ToF Radar Mapper: ESP32 firmware, Arduino framework.
// Thin glue only. All mapping math lives in the pure C++ headers under
// src/ and is covered by the host tests in tests/.
//
// Wiring (see README.md for the full table):
//   VL53L0X  VIN -> 3V3, GND -> GND, SDA -> GPIO 21, SCL -> GPIO 22
//   ULN2003  IN1..IN4 -> GPIO 16, 17, 18, 19; board 5V from USB 5V rail
//   28BYJ-48 plugs into the ULN2003 board, 5V stepper
//
// Flow on boot:
//   1. Home the platform: user lines up the mark on the disc with the
//      mark on the base, presses the BOOT button, firmware calls home().
//   2. Sweep 360 deg, taking a ToF reading every 32 motor steps
//      (2.81 deg, 128 samples per revolution).
//   3. Print the ASCII map and the JSON grid over Serial at 115200.
//   4. Repeat, so the map refreshes every sweep.
#include <Arduino.h>
#include <Wire.h>

#include "map_serializer.h"
#include "occupancy_grid.h"
#include "polar.h"
#include "scan_assembler.h"
#include "stepper_controller.h"
#include "vl53l0x_driver.h"

namespace {

// ULN2003 inputs, IN1..IN4 order.
constexpr int kCoilPins[4] = {16, 17, 18, 19};
constexpr int kHomeButtonPin = 0;  // ESP32 BOOT button, active low
constexpr int kStepsPerSample = 32;    // 2.8125 deg per ToF reading
constexpr int kStepDelayMs = 10;       // ~41 s per full revolution
constexpr int kGridCells = 81;         // 81 x 50 mm = 4.05 m square map
constexpr float kCellSizeMm = 50.0f;

radar::OccupancyGrid g_grid(kCellSizeMm, kGridCells);
radar::StepperController g_stepper(kStepsPerSample);
radar::ScanAssembler g_scan(128);

// Wire wrapper so the pure C++ driver can talk I2C on hardware.
class WireTransport : public radar::I2cTransport {
public:
    bool writeRegister(uint8_t reg, uint8_t value) override {
        Wire.beginTransmission(radar::Vl53l0x::kDefaultAddress);
        Wire.write(reg);
        Wire.write(value);
        return Wire.endTransmission() == 0;
    }
    bool writeRegister16(uint8_t reg, uint16_t value) override {
        Wire.beginTransmission(radar::Vl53l0x::kDefaultAddress);
        Wire.write(reg);
        Wire.write(static_cast<uint8_t>(value >> 8));
        Wire.write(static_cast<uint8_t>(value & 0xFF));
        return Wire.endTransmission() == 0;
    }
    bool readRegister(uint8_t reg, uint8_t& value) override {
        Wire.beginTransmission(radar::Vl53l0x::kDefaultAddress);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) return false;
        if (Wire.requestFrom(radar::Vl53l0x::kDefaultAddress, 1) != 1) return false;
        value = Wire.read();
        return true;
    }
    bool readRegister16(uint8_t reg, uint16_t& value) override {
        Wire.beginTransmission(radar::Vl53l0x::kDefaultAddress);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) return false;
        if (Wire.requestFrom(radar::Vl53l0x::kDefaultAddress, 2) != 2) return false;
        value = static_cast<uint16_t>(Wire.read() << 8) | Wire.read();
        return true;
    }
};

WireTransport g_transport;
radar::Vl53l0x g_sensor(g_transport);

void driveCoils(int pattern_index) {
    const int pattern = radar::halfStepPattern(pattern_index);
    for (int i = 0; i < 4; ++i) {
        digitalWrite(kCoilPins[i], (pattern >> (3 - i)) & 1);
    }
}

void waitForHoming() {
    Serial.println("Line up the disc mark with the base mark, then press BOOT.");
    while (digitalRead(kHomeButtonPin) == HIGH) {
        delay(50);
    }
    g_stepper.home();
    Serial.println("Homed. Starting sweep.");
}

}  // namespace

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);  // SDA, SCL
    for (int pin : kCoilPins) pinMode(pin, OUTPUT);
    pinMode(kHomeButtonPin, INPUT_PULLUP);

    if (!g_sensor.begin()) {
        Serial.println("ERROR: VL53L0X not found at 0x29. Check wiring.");
        while (true) delay(1000);
    }
    Serial.println("VL53L0X found. 360 deg ToF radar ready.");
    waitForHoming();
}

void loop() {
    g_stepper.startSweep();
    g_scan.reset();

    while (!g_stepper.sweepComplete()) {
        // Drive the coils on every single step: the motor only moves
        // when the coil pattern actually changes.
        for (int i = 0; i < kStepsPerSample; ++i) {
            driveCoils(g_stepper.stepForward());
            delay(kStepDelayMs);
        }
        g_stepper.markSampleTaken();

        const float angle = g_stepper.angleDeg();
        g_sensor.startMeasurement();
        delay(35);  // VL53L0X timing budget, default mode
        uint16_t range_mm = 0;
        bool valid = false;
        if (g_sensor.isDataReady() && g_sensor.readRangeMm(range_mm, valid)) {
            g_scan.addSample(angle, static_cast<float>(range_mm), valid);
        } else {
            g_scan.addSample(angle, 0.0f, false);
        }
    }

    g_scan.integrateInto(g_grid);

    Serial.println("--- SCAN COMPLETE ---");
    Serial.println(radar::gridToAscii(g_grid).c_str());
    Serial.println("--- JSON GRID ---");
    Serial.println(radar::gridToJson(g_grid).c_str());
    Serial.println("--- JSON SCAN ---");
    Serial.println(radar::scanToJson(g_scan).c_str());

    delay(2000);  // pause, then sweep again for a live updating map
}
