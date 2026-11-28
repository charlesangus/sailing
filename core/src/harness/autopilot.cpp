#include "sail/harness/autopilot.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace sail::harness {

namespace {

double angle_from_origin(double direction, const State& state, const WindField& wind, double t) {
    const Vec3 to = wind.sample(state.position, t);
    if (std::hypot(to.x(), to.y()) < 1e-12) return 0.0;
    const double origin = std::atan2(-to.y(), -to.x());
    const double twa = std::remainder(direction - origin, 2.0 * std::numbers::pi);
    return twa <= -std::numbers::pi ? std::numbers::pi : twa;
}

}  // namespace

double true_wind_angle(const State& state, const WindField& wind, double t) {
    const Mat3 r = state.rotation();
    return angle_from_origin(std::atan2(r(1, 0), r(0, 0)), state, wind, t);
}

double course_true_wind_angle(const State& state, const WindField& wind, double t) {
    const Vec3 v = state.world_linear_velocity();
    if (std::hypot(v.x(), v.y()) < kMinCourseSpeed) return true_wind_angle(state, wind, t);
    return angle_from_origin(std::atan2(v.y(), v.x()), state, wind, t);
}

Autopilot::Autopilot(const AutopilotParams& params) : params_(params) {}

void Autopilot::reset(double integral) {
    integral_ = std::clamp(integral, -params_.integral_limit, params_.integral_limit);
    error_ = 0.0;
}

double Autopilot::update(double dt, double twa, double yaw_rate) {
    error_ = std::remainder(target_ - twa, 2.0 * std::numbers::pi);
    const double unclamped = params_.kp * error_ + integral_ - params_.kd * yaw_rate;
    const bool saturated_high = unclamped >= 1.0 && error_ > 0.0;
    const bool saturated_low = unclamped <= -1.0 && error_ < 0.0;
    if (dt > 0.0 && std::isfinite(dt) && !saturated_high && !saturated_low) {
        integral_ = std::clamp(integral_ + params_.ki * error_ * dt, -params_.integral_limit,
                               params_.integral_limit);
    }
    const double u = params_.kp * error_ + integral_ - params_.kd * yaw_rate;
    return -std::clamp(u, -1.0, 1.0);
}

}  // namespace sail::harness
