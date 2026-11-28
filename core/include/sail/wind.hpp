#pragma once

#include "sail/rigid_body.hpp"

namespace sail {

// Air velocity, world frame, m/s, at a world point and time t seconds: where the wind blows TO.
class WindField {
public:
    virtual ~WindField() = default;
    virtual Vec3 sample(const Vec3& world_point, double t) const = 0;
};

// Steady, horizontal, spatially uniform wind. The direction is meteorological: the compass
// bearing the wind comes FROM, degrees true, 0 = from north, 90 = from east, clockwise.
class UniformWind final : public WindField {
public:
    UniformWind(double speed_mps, double from_direction_deg);
    Vec3 sample(const Vec3&, double) const override { return velocity_; }

private:
    Vec3 velocity_;
};

enum class Tack {
    // Apparent wind over the starboard side (AWA >= 0).
    Starboard,
    // Apparent wind over the port side (AWA < 0).
    Port,
};

// Apparent wind at one point of the boat: the air velocity relative to the point.
struct ApparentWind {
    // wind - (v_CG + omega x r), world frame and body frame, m/s. Where the air moves to.
    Vec3 world = Vec3::Zero();
    Vec3 body = Vec3::Zero();
    // |world|, m/s.
    double aws = 0.0;
    // rad in (-pi, pi]: angle of the wind's origin from the bow, in the horizontal heading frame
    // (yaw only, no heel or trim) using the horizontal components. 0 = head to wind, positive =
    // wind over the starboard side, +-pi = from astern. 0 when the horizontal speed is zero.
    double awa = 0.0;
    // Same convention in the plane normal to the mast (body z): the body x and y components.
    // tan(awa_sail) = tan(awa) * cos(heel) for horizontal wind and no trim.
    double awa_sail = 0.0;
    // Speed of the body x and y components, m/s.
    double aws_sail = 0.0;
    Tack tack = Tack::Starboard;
};

// `r` is the body-frame point relative to the CG (hull-frame point minus the CG).
ApparentWind apparent_wind(const State& state, const Vec3& r, const WindField& field, double t);

}  // namespace sail
