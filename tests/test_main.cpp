// test_main.cpp
// Host tests for the 360 Degree ToF Radar Mapper core.
// Compiles the pure C++ headers under g++ with no Arduino dependency.
//
// Run: ./run_tests.sh
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "map_serializer.h"
#include "occupancy_grid.h"
#include "polar.h"
#include "scan_assembler.h"
#include "stepper_controller.h"
#include "vl53l0x_driver.h"

namespace {

int g_passed = 0;
int g_failed = 0;

void check(bool ok, const char* name) {
    if (ok) {
        ++g_passed;
        std::printf("  PASS: %s\n", name);
    } else {
        ++g_failed;
        std::printf("  FAIL: %s\n", name);
    }
}

bool near(float a, float b, float eps = 0.01f) {
    return std::fabs(a - b) <= eps;
}

// ---- polar to Cartesian ---------------------------------------------
void testPolar() {
    std::printf("polarToCartesian\n");
    radar::CartesianPoint p;
    p = radar::polarToCartesian(0.0f, 1000.0f);
    check(near(p.x_mm, 1000.0f) && near(p.y_mm, 0.0f), "0 deg -> +X");
    p = radar::polarToCartesian(90.0f, 1000.0f);
    check(near(p.x_mm, 0.0f) && near(p.y_mm, 1000.0f), "90 deg -> +Y");
    p = radar::polarToCartesian(180.0f, 1000.0f);
    check(near(p.x_mm, -1000.0f) && near(p.y_mm, 0.0f), "180 deg -> -X");
    p = radar::polarToCartesian(270.0f, 1000.0f);
    check(near(p.x_mm, 0.0f) && near(p.y_mm, -1000.0f), "270 deg -> -Y");
    p = radar::polarToCartesian(45.0f, 1414.2136f);
    check(near(p.x_mm, 1000.0f, 0.5f) && near(p.y_mm, 1000.0f, 0.5f),
          "45 deg diagonal");
    check(near(radar::normalizeAngle(370.0f), 10.0f), "normalize 370 -> 10");
    check(near(radar::normalizeAngle(-30.0f), 330.0f), "normalize -30 -> 330");
}

// ---- occupancy grid --------------------------------------------------
void testGrid() {
    std::printf("occupancyGrid\n");
    // 21 cells x 50 mm: 1050 mm square, robot at centre cell 10.
    radar::OccupancyGrid grid(50.0f, 21);
    grid.integrateReading(radar::PolarSample{0.0f, 400.0f, true});

    int cx = 0, cy = 0;
    // Endpoint at (400, 0) -> cell (18, 10) is occupied.
    check(grid.worldToCell(400.0f, 0.0f, cx, cy) && grid.isOccupied(cx, cy),
          "endpoint cell marked occupied");
    // Cell halfway along the ray (200, 0) -> cell (14, 10) is free.
    check(grid.worldToCell(200.0f, 0.0f, cx, cy) && grid.isFree(cx, cy),
          "ray cells marked free");
    // Cell just behind the obstacle (500, 0) was never observed.
    check(grid.worldToCell(500.0f, 0.0f, cx, cy) && grid.isUnknown(cx, cy),
          "cell behind obstacle stays unknown");
    // Robot cell itself is free (the ray starts there).
    check(grid.isFree(grid.centreCell(), grid.centreCell()),
          "robot cell is free");
    // Repeated hits accumulate log odds; repeated frees clear a cell.
    for (int i = 0; i < 10; ++i) grid.markHit(18, 10);
    check(grid.isOccupied(18, 10), "repeated hits stay occupied");
    for (int i = 0; i < 40; ++i) grid.markFree(18, 10);
    check(grid.isFree(18, 10), "many free passes clear a cell");
    // Out of range miss clears the ray to max range.
    radar::OccupancyGrid grid2(50.0f, 21);
    grid2.integrateMiss(0.0f, 1000.0f);
    check(grid2.worldToCell(500.0f, 0.0f, cx, cy) && grid2.isFree(cx, cy),
          "miss clears ray to max range");
}

// ---- scan assembly ---------------------------------------------------
void testScanAssembly() {
    std::printf("scanAssembler\n");
    radar::ScanAssembler scan(128);
    // Simulated room: a flat wall 1000 mm away on the +X side, so
    // readings near angle 0 report ~1000 mm; every other direction
    // sees nothing (out of range).
    for (int i = 0; i < 128; ++i) {
        const float angle = static_cast<float>(i) * 360.0f / 128.0f;
        if (angle < 10.0f || angle > 350.0f) {
            const float range = 1000.0f / std::cos(radar::degToRad(angle));
            scan.addSample(angle, range, true);
        } else {
            scan.addSample(angle, 0.0f, false);
        }
    }
    check(scan.isComplete(), "128 samples complete a full scan");
    check(scan.filledCount() == 128, "all slots filled");

    const radar::ScanPoint& wall = scan.points()[0];
    check(wall.hit && near(wall.range_mm, 1000.0f, 1.0f),
          "wall at 0 deg reads 1000 mm");
    check(near(wall.cartesian.x_mm, 1000.0f, 1.0f) &&
              near(wall.cartesian.y_mm, 0.0f, 1.0f),
          "wall point converts to (1000, 0)");
    check(!scan.points()[64].hit, "opposite direction reports no target");

    // The wall lands in the grid at the right place.
    radar::OccupancyGrid grid(50.0f, 41);  // 2050 mm square
    scan.integrateInto(grid);
    int cx = 0, cy = 0;
    check(grid.worldToCell(1000.0f, 0.0f, cx, cy) && grid.isOccupied(cx, cy),
          "wall cell occupied in grid after integration");
    check(grid.worldToCell(500.0f, 0.0f, cx, cy) && grid.isFree(cx, cy),
          "space in front of wall is free");
}

// ---- out of range handling -------------------------------------------
void testOutOfRange() {
    std::printf("outOfRange\n");
    radar::ScanAssembler scan(128);
    check(!scan.addSample(0.0f, 5000.0f, true), "5000 mm reading dropped");
    check(!scan.addSample(90.0f, 10.0f, true), "10 mm reading dropped");
    check(!scan.addSample(180.0f, 1000.0f, false), "invalid flag dropped");
    check(!scan.points()[0].hit, "dropped sample stored as miss");
    check(scan.points()[0].range_mm == radar::ScanAssembler::kMaxRangeMm,
          "miss stored at max range");

    radar::OccupancyGrid grid(50.0f, 21);
    scan.integrateInto(grid);
    int cx = 0, cy = 0;
    check(grid.worldToCell(500.0f, 0.0f, cx, cy) && grid.isFree(cx, cy),
          "out of range reading does not create a phantom wall");
    check(grid.worldToCell(500.0f, 0.0f, cx, cy) && !grid.isOccupied(cx, cy),
          "no occupied cell from a dropped reading");
}

// ---- stepper ----------------------------------------------------------
void testStepper() {
    std::printf("stepperController\n");
    radar::StepperController stepper(32);
    stepper.home();
    check(near(stepper.angleDeg(), 0.0f), "homed angle is 0");
    for (int i = 0; i < 4096; ++i) stepper.stepForward();
    check(near(stepper.angleDeg(), 0.0f), "4096 steps wraps back to 0");
    check(stepper.positionSteps() == 4096, "step count keeps counting past wrap");
    stepper.stepForward();
    check(near(stepper.angleDeg(), radar::StepperController::kDegPerStep),
          "4097 steps is one step past 360");
    for (int i = 0; i < 4095; ++i) stepper.stepForward();
    check(near(stepper.angleDeg(), 0.0f), "8192 steps is exactly two revolutions");

    radar::StepperController sweep(32);
    sweep.home();
    sweep.startSweep();
    int samples = 0;
    while (!sweep.sweepComplete()) {
        if (sweep.stepToNextSample()) ++samples;
    }
    check(samples == 128, "128 samples per revolution at 32 steps/sample");
    check(near(sweep.sampleSpacingDeg(), 2.8125f), "sample spacing 2.8125 deg");
    check(sweep.samplesPerRevolution() == 128, "samplesPerRevolution is 128");

    // Half step coil patterns are all distinct and non-zero.
    bool distinct = true;
    for (int i = 0; i < 8 && distinct; ++i) {
        for (int j = i + 1; j < 8; ++j) {
            if (radar::halfStepPattern(i) == radar::halfStepPattern(j)) distinct = false;
        }
        if (radar::halfStepPattern(i) == 0) distinct = false;
    }
    check(distinct, "8 half step coil patterns are distinct and non-zero");
}

// ---- ASCII map --------------------------------------------------------
void testAsciiMap() {
    std::printf("asciiMap\n");
    radar::OccupancyGrid grid(50.0f, 21);
    grid.integrateReading(radar::PolarSample{0.0f, 500.0f, true});
    const std::string ascii = radar::gridToAscii(grid);
    // Split into rows; rows are top first, so row 10 is y = 0.
    std::vector<std::string> rows;
    std::size_t start = 0;
    while (true) {
        const std::size_t nl = ascii.find('\n', start);
        if (nl == std::string::npos) break;
        rows.push_back(ascii.substr(start, nl - start));
        start = nl + 1;
    }
    check(rows.size() == 21, "ASCII map has 21 rows");
    check(rows[10].size() == 21, "ASCII rows have 21 columns");
    // Robot at centre (row 10, col 10); obstacle at (500, 0) -> col 20.
    check(rows[10][10] == 'R', "robot marked R at centre");
    check(rows[10][20] == '#', "obstacle rendered # at the right cell");
    check(rows[10][15] == '.', "free space rendered . along the ray");
    check(rows[0][0] == ' ', "unobserved corner stays blank");

    // JSON serializer produces well formed output with the right shape.
    const std::string json = radar::gridToJson(grid);
    check(json.find("\"cells\":21") != std::string::npos, "JSON carries cell count");
    check(json.find("\"cell_mm\":50") != std::string::npos, "JSON carries cell size");
    check(json.back() == '\n', "JSON ends with newline");
}

// ---- VL53L0X driver with a mock transport ----------------------------
class MockTransport : public radar::I2cTransport {
public:
    uint8_t regs[256] = {};
    bool fail = false;

    bool writeRegister(uint8_t reg, uint8_t value) override {
        if (fail) return false;
        regs[reg] = value;
        return true;
    }
    bool writeRegister16(uint8_t reg, uint16_t value) override {
        if (fail) return false;
        regs[reg] = static_cast<uint8_t>(value >> 8);
        regs[reg + 1] = static_cast<uint8_t>(value & 0xFF);
        return true;
    }
    bool readRegister(uint8_t reg, uint8_t& value) override {
        if (fail) return false;
        value = regs[reg];
        return true;
    }
    bool readRegister16(uint8_t reg, uint16_t& value) override {
        if (fail) return false;
        value = static_cast<uint16_t>(regs[reg] << 8) | regs[reg + 1];
        return true;
    }
};

void testDriver() {
    std::printf("vl53l0xDriver\n");
    MockTransport transport;
    transport.regs[radar::Vl53l0x::kRegIdentificationModelId] =
        radar::Vl53l0x::kExpectedModelId;
    radar::Vl53l0x sensor(transport);
    check(sensor.begin(), "begin succeeds with model ID 0xEE");
    check(sensor.address() == 0x29, "default I2C address is 0x29");

    transport.regs[radar::Vl53l0x::kRegResultInterruptStatus] = 0x07;
    transport.regs[radar::Vl53l0x::kRegResultRangeStatus] = 0x00;
    transport.regs[radar::Vl53l0x::kRegFinalRangeResult] = 0x03;
    transport.regs[radar::Vl53l0x::kRegFinalRangeResult + 1] = 0xE8;  // 1000 mm
    check(sensor.startMeasurement(), "startMeasurement writes SYSRANGE_START");
    check(sensor.isDataReady(), "data ready flag detected");
    uint16_t range = 0;
    bool valid = false;
    check(sensor.readRangeMm(range, valid) && valid && range == 1000,
          "valid reading returns 1000 mm");

    // No target status maps to valid=false, never to a 0 mm wall.
    transport.regs[radar::Vl53l0x::kRegResultRangeStatus] =
        radar::Vl53l0x::kRangeStatusNoTarget;
    check(sensor.readRangeMm(range, valid) && !valid,
          "no target status reports invalid, not 0 mm");

    MockTransport failing;
    failing.fail = true;
    radar::Vl53l0x dead(failing);
    check(!dead.begin(), "begin fails when I2C is dead");
}

}  // namespace

int main() {
    testPolar();
    testGrid();
    testScanAssembly();
    testOutOfRange();
    testStepper();
    testAsciiMap();
    testDriver();
    std::printf("\n%d passed, %d failed\n", g_passed, g_failed);
    return g_failed == 0 ? 0 : 1;
}
