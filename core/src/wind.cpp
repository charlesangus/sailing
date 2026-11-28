#include "sail/wind.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace sail {

namespace {

// Below this speed (m/s) the direction of a horizontal vector is undefined.
constexpr double kMinSpeed = 1e-12;

// Angle from the bow to where the air comes from, positive to starboard, given the air's
// forward and port-going velocity components. Adding 0.0 turns a -0.0 into +0.0 so astern is +pi, not -pi.
double origin_angle(double toward_forward, double toward_port) {
    if (std::hypot(toward_forward, toward_port) < kMinSpeed) return 0.0;
    return std::atan2(toward_port + 0.0, -toward_forward);
}

}  // namespace

UniformWind::UniformWind(double speed_mps, double from_direction_deg) {
    const double from = from_direction_deg * std::numbers::pi / 180.0;
    velocity_ = Vec3(-speed_mps * std::sin(from), -speed_mps * std::cos(from), 0.0);
}

LogProfileWind::LogProfileWind(double speed_at_reference_mps, double from_direction_deg,
                               double reference_height, double roughness)
    : reference_velocity_(UniformWind(speed_at_reference_mps, from_direction_deg).sample(
          Vec3::Zero(), 0.0)),
      roughness_(roughness) {
    if (!(roughness > 0.0) || !(reference_height > 10.0 * roughness) ||
        !std::isfinite(reference_height)) {
        throw std::invalid_argument(
            "LogProfileWind: roughness must be positive and reference_height above 10 "
            "roughness lengths");
    }
    inverse_log_reference_ = 1.0 / std::log(reference_height / roughness);
}

Vec3 LogProfileWind::sample(const Vec3& world_point, double) const {
    const double z = std::max(world_point.z(), 10.0 * roughness_);
    return reference_velocity_ * (std::log(z / roughness_) * inverse_log_reference_);
}

ApparentWind apparent_wind(const State& state, const Vec3& r, const WindField& field, double t) {
    const Mat3 rotation = state.rotation();
    const Vec3 world_point = state.position + rotation * r;
    const Vec3 point_velocity_body = state.linear_velocity + state.angular_velocity.cross(r);
    const Vec3 wind_world = field.sample(world_point, t);

    ApparentWind out;
    out.world = wind_world - rotation * point_velocity_body;
    out.body = rotation.transpose() * out.world;
    out.aws = out.world.norm();

    const double heading = std::atan2(rotation(1, 0), rotation(0, 0));
    const double c = std::cos(heading);
    const double s = std::sin(heading);
    const double forward = c * out.world.x() + s * out.world.y();
    const double port = -s * out.world.x() + c * out.world.y();
    out.awa = origin_angle(forward, port);

    out.awa_sail = origin_angle(out.body.x(), out.body.y());
    out.aws_sail = std::hypot(out.body.x(), out.body.y());
    out.tack = out.awa < 0.0 ? Tack::Port : Tack::Starboard;
    return out;
}

}  // namespace sail
