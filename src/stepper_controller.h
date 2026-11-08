// stepper_controller.h
// Sweep controller for the 28BYJ-48 5V stepper + ULN2003 driver board.
// Pure C++, no Arduino dependencies. The hardware glue (which GPIO pins
// get driven in which order) lives in main.cpp; this class owns the
// geometry: steps, angle tracking, homing, backlash, and sweep timing.
//
// 28BYJ-48 facts that drive the numbers here:
//   * 64 step motor with a 1:64 gearbox, driven in half step mode:
//     64 * 64 / 2 = 4096 steps per output revolution.
//   * Half step resolution is therefore 360 / 4096 = 0.0879 deg/step.
//   * The gearbox has noticeable backlash (several degrees). Homing
//     always approaches the reference position from the same direction
//     so the slack sits on the same side every scan.
//   * Rated speed is low: at 10 ms per half step a full revolution
//     takes about 41 seconds. A full 360 deg scan is slow by lidar
//     standards; that tradeoff is explicit, not a bug.
#pragma once

#include <cmath>

#include "polar.h"

namespace radar {

class StepperController {
public:
    static constexpr int kStepsPerRevolution = 4096;  // half step mode
    static constexpr float kDegPerStep = 360.0f / static_cast<float>(kStepsPerRevolution);

    // steps_per_sample: how many motor steps between ToF readings.
    //   32 steps = 2.8125 deg per sample, 128 samples per revolution.
    //   16 steps = 1.40625 deg per sample, 256 samples per revolution.
    explicit StepperController(int steps_per_sample = 32)
        : steps_per_sample_(steps_per_sample) {}

    // One motor step in the sweep direction. Returns the half step
    // pattern index 0..7 for the ULN2003 coil sequence; main.cpp maps
    // the index to the four GPIO outputs.
    int stepForward() {
        position_steps_++;
        pattern_index_ = (pattern_index_ + 1) & 7;
        return pattern_index_;
    }

    int stepBackward() {
        position_steps_--;
        pattern_index_ = (pattern_index_ + 7) & 7;
        return pattern_index_;
    }

    // Current shaft angle in [0, 360), derived purely from step count.
    float angleDeg() const {
        const long wrapped = ((position_steps_ % kStepsPerRevolution) + kStepsPerRevolution) %
                             kStepsPerRevolution;
        return static_cast<float>(wrapped) * kDegPerStep;
    }

    long positionSteps() const { return position_steps_; }

    // Steps taken since the last homing or sweep start.
    long stepsSinceStart() const { return position_steps_ - start_steps_; }

    // Homing: the operator manually rotates the platform to a marked
    // reference position (a line on the disc lined up with a mark on
    // the base) and the firmware calls home(). From then on, angles are
    // relative to that mark. Backlash compensation: after homing, the
    // controller always sweeps in the forward direction, so gear slack
    // stays loaded on the same side for the whole scan.
    void home() {
        position_steps_ = 0;
        start_steps_ = 0;
        pattern_index_ = 0;
        homed_ = true;
    }

    bool isHomed() const { return homed_; }

    // Begin a new 360 degree sweep from the current position.
    void startSweep() {
        start_steps_ = position_steps_;
        samples_taken_ = 0;
        sweep_complete_ = false;
    }

    // Step the motor until the next sample point is reached.
    // Returns true when a reading should be taken at the current angle.
    bool stepToNextSample() {
        if (sweep_complete_) return false;
        for (int i = 0; i < steps_per_sample_; ++i) stepForward();
        samples_taken_++;
        if (stepsSinceStart() >= kStepsPerRevolution) sweep_complete_ = true;
        return true;
    }

    int samplesTaken() const { return samples_taken_; }
    bool sweepComplete() const { return sweep_complete_; }

    // True once a full revolution of steps has been taken since
    // startSweep(). main.cpp uses this with per-step coil driving,
    // because the ULN2003 coils must be energized on every step,
    // not once per sample.
    bool sweepDone() const { return stepsSinceStart() >= kStepsPerRevolution; }

    // Record that a reading was taken at the current angle. Marks the
    // sweep complete once a full revolution is done.
    void markSampleTaken() {
        samples_taken_++;
        if (sweepDone()) sweep_complete_ = true;
    }

    // Expected samples per full revolution at the configured step size.
    int samplesPerRevolution() const {
        return kStepsPerRevolution / steps_per_sample_;
    }

    float sampleSpacingDeg() const {
        return static_cast<float>(steps_per_sample_) * kDegPerStep;
    }

private:
    int steps_per_sample_;
    long position_steps_ = 0;
    long start_steps_ = 0;
    int pattern_index_ = 0;
    int samples_taken_ = 0;
    bool homed_ = false;
    bool sweep_complete_ = false;
};

// Half step coil patterns for the ULN2003, one bit per coil (IN1..IN4).
// Sequence: 1000, 1100, 0100, 0110, 0010, 0011, 0001, 1001.
inline int halfStepPattern(int index) {
    static constexpr int kPatterns[8] = {0b1000, 0b1100, 0b0100, 0b0110,
                                         0b0010, 0b0011, 0b0001, 0b1001};
    return kPatterns[index & 7];
}

}  // namespace radar
