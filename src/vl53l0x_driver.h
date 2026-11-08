// vl53l0x_driver.h
// Minimal VL53L0X register-level driver interface.
// Pure C++ except for the injected I2C transport, so the host tests can
// drive it with a mock transport and main.cpp can inject a Wire wrapper.
//
// IMPORTANT: this is a thin driver for the single-shot ranging mode
// used by the radar. It is NOT a vendored copy of the ST API or the
// Adafruit/Pololu libraries, and it does not claim to be. The register
// map below comes from the VL53L0X datasheet (DocID 029171) and the
// public ST API source. A production build should use the Pololu
// VL53L0X Arduino library instead; this driver exists so the project
// is self contained and the register protocol is visible in the code.
//
// I2C facts:
//   * Default 7 bit address 0x29. No address pins on the GY-VL53L0XV2
//     module, so only one sensor can share a bus without an I2C
//     multiplexer or an XSHUT line per sensor.
//   * The sensor needs a ~1.2 ms boot after power up before the first
//     register access, and ranging takes ~30 ms in default timing
//     budget, ~20 ms in high speed mode. The sweep loop budgets for it.
//   * Range status 0x14: bit 0..2 encode the error; 0x0B means
//     "no target", which the core maps to an invalid sample, not a
//     zero range. That distinction is what keeps phantom walls out
//     of the map.
#pragma once

#include <cstdint>

namespace radar {

// Transport the driver talks through. main.cpp implements this on top
// of Arduino Wire; tests implement it with a register map in RAM.
class I2cTransport {
public:
    virtual ~I2cTransport() = default;
    virtual bool writeRegister(uint8_t reg, uint8_t value) = 0;
    virtual bool writeRegister16(uint8_t reg, uint16_t value) = 0;
    virtual bool readRegister(uint8_t reg, uint8_t& value) = 0;
    virtual bool readRegister16(uint8_t reg, uint16_t& value) = 0;
};

class Vl53l0x {
public:
    static constexpr uint8_t kDefaultAddress = 0x29;

    // Registers used by the single-shot flow.
    static constexpr uint8_t kRegIdentificationModelId = 0xC0;  // reads 0xEE
    static constexpr uint8_t kRegSysrangeStart = 0x00;
    static constexpr uint8_t kRegResultInterruptStatus = 0x13;
    static constexpr uint8_t kRegResultRangeStatus = 0x14;
    static constexpr uint8_t kRegSystemInterruptClear = 0x0B;
    static constexpr uint8_t kRegFinalRangeResult = 0x1E;  // 16 bit, mm

    static constexpr uint8_t kExpectedModelId = 0xEE;
    static constexpr uint8_t kRangeStatusNoTarget = 0x0B;

    explicit Vl53l0x(I2cTransport& transport, uint8_t address = kDefaultAddress)
        : transport_(transport), address_(address) {}

    uint8_t address() const { return address_; }

    // Verify the sensor is present and answering. Must be called once
    // after power up; returns false if the model ID register does not
    // read 0xEE.
    bool begin() {
        uint8_t model_id = 0;
        if (!transport_.readRegister(kRegIdentificationModelId, model_id)) return false;
        if (model_id != kExpectedModelId) return false;
        // Enable single-shot ranging, interrupt on new sample ready.
        // Full ST init sequence (SPAD calibration, tuning) is omitted
        // here: the module ships pre-calibrated, and for this project
        // the Pololu library init is the recommended path on hardware.
        return true;
    }

    // Start one ranging measurement. Call isDataReady() then readRangeMm().
    bool startMeasurement() {
        return transport_.writeRegister(kRegSysrangeStart, 0x01);
    }

    bool isDataReady() {
        uint8_t status = 0;
        if (!transport_.readRegister(kRegResultInterruptStatus, status)) return false;
        return (status & 0x07) != 0;
    }

    // Read the last completed measurement. Returns false on I2C error.
    // Sets valid=false when the sensor reports no target (range status
    // 0x0B), so the caller never confuses "nothing there" with
    // "something at 0 mm".
    bool readRangeMm(uint16_t& range_mm, bool& valid) {
        uint8_t status = 0;
        uint16_t raw = 0;
        if (!transport_.readRegister(kRegResultRangeStatus, status)) return false;
        if (!transport_.readRegister16(kRegFinalRangeResult, raw)) return false;
        transport_.writeRegister(kRegSystemInterruptClear, 0x01);
        valid = status != kRangeStatusNoTarget;
        range_mm = raw;
        return true;
    }

private:
    I2cTransport& transport_;
    uint8_t address_;
};

}  // namespace radar
