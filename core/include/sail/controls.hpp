#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sail {

// Helm input.
//
// `tiller` runs from -1 to 1, and +1 is the tiller pushed fully to port. As on a real boat that
// swings the blade's trailing edge to starboard, so the rudder pushes the stern to port and the
// boat turns to starboard: a negative yaw moment and yaw rate about body z (up). Values outside
// the range, or not finite, are clamped to it or read as 0.
struct Controls {
    double tiller = 0.0;
};

// Tiller in [-1, 1] from a Controls value.
inline double clamped_tiller(const Controls& controls) {
    if (!std::isfinite(controls.tiller)) return 0.0;
    return std::clamp(controls.tiller, -1.0, 1.0);
}

// Rudder angle, rad, right-handed about the rudder stock axis (body +z, up), so a positive angle
// is the tiller to port and the trailing edge to starboard. The commanded angle is
// tiller * max_angle.
//
// The actual angle follows the command at no more than `rate`, by step(dt) with the time since
// the previous step, and lands on the command exactly once within reach.
class RudderActuator {
public:
    // max_angle in rad and rate in rad/s. Throws std::invalid_argument unless both are positive
    // and finite.
    RudderActuator(double max_angle, double rate) : max_angle_(max_angle), rate_(rate) {
        if (!(max_angle > 0.0) || !(rate > 0.0) || !std::isfinite(max_angle) ||
            !std::isfinite(rate)) {
            throw std::invalid_argument("RudderActuator: max angle and rate must be positive");
        }
    }

    void set_command(const Controls& controls) { command_ = clamped_tiller(controls) * max_angle_; }

    // Non-positive or non-finite dt does nothing.
    void step(double dt) {
        if (!(dt > 0.0) || !std::isfinite(dt)) return;
        const double reach = rate_ * dt;
        angle_ += std::clamp(command_ - angle_, -reach, reach);
    }

    // Puts the rudder at `angle` (clamped to +-max_angle) and commands it to stay there.
    void reset(double angle) {
        angle_ = std::isfinite(angle) ? std::clamp(angle, -max_angle_, max_angle_) : 0.0;
        command_ = angle_;
    }

    double angle() const { return angle_; }
    double command() const { return command_; }
    double max_angle() const { return max_angle_; }
    double rate() const { return rate_; }

private:
    double max_angle_;
    double rate_;
    double angle_ = 0.0;
    double command_ = 0.0;
};

}  // namespace sail
