#pragma once

#include "sail/rigid_body.hpp"
#include "sail/wind.hpp"

namespace sail::harness {

// rad in (-pi, pi]: the true wind's origin from the heading (body x projected on the
// horizontal), positive with the wind over the starboard side, as ApparentWind::awa.
double true_wind_angle(const State& state, const WindField& wind, double t);

// Below this horizontal speed, m/s, the course is taken to be the heading.
inline constexpr double kMinCourseSpeed = 0.1;

// rad in (-pi, pi]: the true wind's origin from the course, the horizontal velocity of the CG
// through the (still) water, signed as true_wind_angle(). It is the heading's angle plus the
// leeway (positive to port), the angle ORC's and Python-VPP's polars tabulate. The heading's
// angle below kMinCourseSpeed.
double course_true_wind_angle(const State& state, const WindField& wind, double t);

// Gains of the tiller PID, in tiller units (-1..1) per radian of true wind angle error.
struct AutopilotParams {
    // Per rad of error.
    double kp = 1.6;
    // Per rad*s of integrated error.
    double ki = 0.25;
    // Per rad/s of yaw rate (world z); damps the heading, acting on the measurement so a step in
    // the target does not kick the tiller.
    double kd = 1.6;
    // The integral term's magnitude limit, tiller units.
    double integral_limit = 0.6;
};

// Holds a true wind angle with the tiller. Error e = target - twa, wrapped to (-pi, pi]; the PID
// output u = kp e + I - kd r, with r the yaw rate. The signed TWA rises as the boat turns to port
// on either tack, so u > 0 asks for a turn to port, which is the tiller to starboard:
// tiller = -u, clamped to [-1, 1].
//
// Anti-windup: I is held within +-integral_limit, and does not integrate while the tiller is
// saturated in the direction the error would push it further.
class Autopilot {
public:
    explicit Autopilot(const AutopilotParams& params = {});

    void set_target(double twa) { target_ = twa; }
    double target() const { return target_; }

    // Sets the integral term, tiller units, the steady weather helm to start from: the tiller a
    // previous run settled on is -integral when the error and yaw rate are zero.
    void reset(double integral = 0.0);

    // The tiller for the next `dt` seconds, at measured `twa` (rad) and world-z yaw rate (rad/s).
    double update(double dt, double twa, double yaw_rate);

    double integral() const { return integral_; }
    double last_error() const { return error_; }
    const AutopilotParams& params() const { return params_; }

private:
    AutopilotParams params_;
    double target_ = 0.0;
    double integral_ = 0.0;
    double error_ = 0.0;
};

}  // namespace sail::harness
