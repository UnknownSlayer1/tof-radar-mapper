// polar.h
// Polar to Cartesian conversion for the 360 degree ToF radar mapper.
// Pure C++, no Arduino dependencies, so the host test harness can use it.
//
// Coordinate frame: the sensor sits at the origin. Angle 0 points along
// +X (east), angle grows counterclockwise, so 90 deg is +Y (north).
// Ranges are in millimetres, matching the VL53L0X native unit.
#pragma once

#include <cmath>

namespace radar {

struct PolarSample {
    float angle_deg = 0.0f;  // 0..360, 0 = +X, counterclockwise
    float range_mm = 0.0f;   // distance reported by the ToF sensor
    bool valid = true;       // false when the sensor reports no target
};

struct CartesianPoint {
    float x_mm = 0.0f;
    float y_mm = 0.0f;
};

inline float degToRad(float deg) {
    return deg * static_cast<float>(M_PI) / 180.0f;
}

// Normalize any angle into [0, 360).
inline float normalizeAngle(float deg) {
    float a = std::fmod(deg, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

inline CartesianPoint polarToCartesian(const PolarSample& s) {
    const float r = degToRad(s.angle_deg);
    return CartesianPoint{ s.range_mm * std::cos(r),
                           s.range_mm * std::sin(r) };
}

inline CartesianPoint polarToCartesian(float angle_deg, float range_mm) {
    return polarToCartesian(PolarSample{ angle_deg, range_mm, true });
}

}  // namespace radar
