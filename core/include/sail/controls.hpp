#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sail {

// Helm and sail inputs.
//
// `tiller` runs from -1 to 1, and +1 is the tiller pushed fully to port. As on a real boat that
// swings the blade's trailing edge to starboard, so the rudder pushes the stern to port and the
// boat turns to starboard: a negative yaw moment and yaw rate about body z (up). Values outside
// the range, or not finite, are clamped to it or read as 0.
//
// Sail controls, read by SheetController. Each is clamped to its range; a value that is not
// finite reads as the member's default.
//
// - `mainsheet`, `jib_sheet_port`, `jib_sheet_starboard`: 0 eased fully to 1 trimmed fully, a
//   linear fraction of the sheet's length between its longest and shortest. A jib sheet acts only
//   while the jib is on its side: the port sheet with the jib to port, on starboard tack.
// - `traveler`: -1 the car at the windward end of the track, 0 centred, 1 the leeward end. It is
//   relative to the side the main is on, so it holds its meaning through a tack.
// - `backstay`: 0 slack to 1 fully on.
// - `jib_car`: 0 the forward end of the jib track to 1 the aft end, both sides together.
// - `reef`: 0 shaken out, 1 reefed; below 0.5 reads as 0, otherwise 1.
struct Controls {
    double tiller = 0.0;
    double mainsheet = 0.0;
    double traveler = 0.0;
    double backstay = 0.0;
    double jib_sheet_port = 0.0;
    double jib_sheet_starboard = 0.0;
    double jib_car = 0.5;
    double reef = 0.0;
};

// Tiller in [-1, 1] from a Controls value.
inline double clamped_tiller(const Controls& controls) {
    if (!std::isfinite(controls.tiller)) return 0.0;
    return std::clamp(controls.tiller, -1.0, 1.0);
}

// Every member clamped to its range, a non-finite one replaced by its default, and the reef
// rounded to 0 or 1.
inline Controls clamped(const Controls& controls) {
    const Controls defaults;
    const auto clamp = [](double v, double fallback, double lo, double hi) {
        return std::isfinite(v) ? std::clamp(v, lo, hi) : fallback;
    };
    Controls out;
    out.tiller = clamped_tiller(controls);
    out.mainsheet = clamp(controls.mainsheet, defaults.mainsheet, 0.0, 1.0);
    out.traveler = clamp(controls.traveler, defaults.traveler, -1.0, 1.0);
    out.backstay = clamp(controls.backstay, defaults.backstay, 0.0, 1.0);
    out.jib_sheet_port = clamp(controls.jib_sheet_port, defaults.jib_sheet_port, 0.0, 1.0);
    out.jib_sheet_starboard =
        clamp(controls.jib_sheet_starboard, defaults.jib_sheet_starboard, 0.0, 1.0);
    out.jib_car = clamp(controls.jib_car, defaults.jib_car, 0.0, 1.0);
    out.reef = clamp(controls.reef, defaults.reef, 0.0, 1.0) < 0.5 ? 0.0 : 1.0;
    return out;
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
