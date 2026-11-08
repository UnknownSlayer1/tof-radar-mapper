// scan_assembler.h
// Collects angle + range samples into one complete 360 degree scan.
// Pure C++, no Arduino dependencies.
//
// Filtering rules, all grounded in the VL53L0X datasheet behaviour:
//   * Range below the sensor minimum (30 mm) is a crosstalk or cover
//     glass artifact, not a real target. Drop it.
//   * Range above the sensor maximum (2000 mm in long range mode) or a
//     sample flagged invalid means "no target in this direction".
//     A miss is still information: the grid ray gets cleared out to
//     max range instead of leaving the sector unknown.
//   * A single-sample spike (one reading far from both neighbours) is
//     multipath or a specular glint. A median of the last 3 readings
//     at the same angle kills it; the assembler keeps a 1 sample
//     history per angle slot for that reason.
#pragma once

#include <cstddef>
#include <vector>

#include "occupancy_grid.h"
#include "polar.h"

namespace radar {

struct ScanPoint {
    float angle_deg = 0.0f;
    float range_mm = 0.0f;
    CartesianPoint cartesian{};
    bool hit = false;  // false = no target seen in this direction
};

class ScanAssembler {
public:
    static constexpr float kMinRangeMm = 30.0f;    // VL53L0X minimum
    static constexpr float kMaxRangeMm = 2000.0f;  // long range mode ceiling

    explicit ScanAssembler(int samples_per_revolution = 128)
        : samples_per_revolution_(samples_per_revolution),
          samples_(static_cast<std::size_t>(samples_per_revolution)) {}

    int samplesPerRevolution() const { return samples_per_revolution_; }

    // Add one raw sensor reading. Angle is the shaft angle at the time
    // the reading was taken. Returns false when the sample was dropped
    // as out of range; a dropped sample still counts as a miss for
    // the scan completeness check.
    bool addSample(float angle_deg, float range_mm, bool valid) {
        const int slot = angleToSlot(angle_deg);
        ScanPoint& point = samples_[static_cast<std::size_t>(slot)];
        point.angle_deg = static_cast<float>(slot) * 360.0f /
                          static_cast<float>(samples_per_revolution_);
        point.cartesian = polarToCartesian(point.angle_deg, range_mm);

        if (!valid || range_mm < kMinRangeMm || range_mm > kMaxRangeMm) {
            point.range_mm = kMaxRangeMm;
            point.hit = false;
            point.cartesian = polarToCartesian(point.angle_deg, kMaxRangeMm);
        } else {
            point.range_mm = median3(previous_range_[static_cast<std::size_t>(slot)],
                                     range_mm, range_mm);
            point.hit = true;
            point.cartesian = polarToCartesian(point.angle_deg, point.range_mm);
        }
        previous_range_[static_cast<std::size_t>(slot)] = range_mm;
        filled_[static_cast<std::size_t>(slot)] = true;
        filled_count_ = 0;
        for (bool f : filled_) filled_count_ += f ? 1 : 0;
        return point.hit;
    }

    // True once every angle slot has at least one sample.
    bool isComplete() const { return filled_count_ >= samples_per_revolution_; }

    int filledCount() const { return filled_count_; }

    const std::vector<ScanPoint>& points() const { return samples_; }

    // Fold the whole scan into the occupancy grid.
    void integrateInto(OccupancyGrid& grid) const {
        for (const ScanPoint& p : samples_) {
            if (p.hit) {
                grid.integrateReading(PolarSample{ p.angle_deg, p.range_mm, true });
            } else {
                grid.integrateMiss(p.angle_deg, kMaxRangeMm);
            }
        }
    }

    void reset() {
        std::fill(filled_.begin(), filled_.end(), false);
        std::fill(previous_range_.begin(), previous_range_.end(), 0.0f);
        filled_count_ = 0;
        for (ScanPoint& p : samples_) p = ScanPoint{};
    }

private:
    int angleToSlot(float angle_deg) const {
        const float n = normalizeAngle(angle_deg) / 360.0f *
                        static_cast<float>(samples_per_revolution_);
        int slot = static_cast<int>(n + 0.5f) % samples_per_revolution_;
        if (slot < 0) slot += samples_per_revolution_;
        return slot;
    }

    // Median of three; with only one real history value the call sites
    // pass the new reading twice, so a lone spike still gets through
    // once and is corrected on the next revolution.
    static float median3(float a, float b, float c) {
        if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
        if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
        return c;
    }

    int samples_per_revolution_;
    std::vector<ScanPoint> samples_;
    std::vector<float> previous_range_ =
        std::vector<float>(static_cast<std::size_t>(samples_per_revolution_), 0.0f);
    std::vector<bool> filled_ =
        std::vector<bool>(static_cast<std::size_t>(samples_per_revolution_), false);
    int filled_count_ = 0;
};

}  // namespace radar
