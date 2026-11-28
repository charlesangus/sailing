#include <doctest/doctest.h>

#include <cmath>
#include <numbers>

#include "sail/wind.hpp"

namespace {

using namespace sail;

constexpr double kKnot = 0.514444444444;
constexpr double kPi = std::numbers::pi;

Quat yaw_roll(double heading_from_north, double heel) {
    const double yaw = kPi / 2.0 - heading_from_north;
    return Quat(Eigen::AngleAxisd(yaw, Vec3::UnitZ())) * Quat(Eigen::AngleAxisd(heel, Vec3::UnitX()));
}

State moving(double heading, double speed, double heel = 0.0) {
    State s;
    s.orientation = yaw_roll(heading, heel);
    s.linear_velocity = Vec3(speed, 0.0, 0.0);
    return s;
}

}  // namespace

TEST_CASE("UniformWind blows toward the opposite of its from-direction") {
    const Vec3 from_north = UniformWind(10.0, 0.0).sample(Vec3::Zero(), 0.0);
    CHECK(from_north.x() == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(from_north.y() == doctest::Approx(-10.0));
    const Vec3 from_east = UniformWind(10.0, 90.0).sample(Vec3::Zero(), 3.0);
    CHECK(from_east.x() == doctest::Approx(-10.0));
    CHECK(from_east.y() == doctest::Approx(0.0).epsilon(1e-12));
}

TEST_CASE("beam wind on a boat heading north is over the starboard side") {
    const UniformWind wind(10.0 * kKnot, 90.0);
    const State s = moving(0.0, 5.0 * kKnot);
    const ApparentWind a = apparent_wind(s, Vec3::Zero(), wind, 0.0);
    CHECK(a.aws == doctest::Approx(std::sqrt(125.0) * kKnot).epsilon(1e-9));
    CHECK(a.awa == doctest::Approx(std::atan2(10.0, 5.0)).epsilon(1e-9));
    CHECK(a.awa_sail == doctest::Approx(a.awa).epsilon(1e-9));
    CHECK(a.tack == Tack::Starboard);
    CHECK(a.body.x() == doctest::Approx(-5.0 * kKnot));
    CHECK(a.body.y() == doctest::Approx(10.0 * kKnot));
}

TEST_CASE("wind from the west puts the boat on port tack with negative AWA") {
    const UniformWind wind(10.0 * kKnot, 270.0);
    const ApparentWind a = apparent_wind(moving(0.0, 5.0 * kKnot), Vec3::Zero(), wind, 0.0);
    CHECK(a.awa == doctest::Approx(-std::atan2(10.0, 5.0)).epsilon(1e-9));
    CHECK(a.tack == Tack::Port);
}

TEST_CASE("heel reduces the effective sail-plane angle by cos(heel)") {
    const UniformWind wind(10.0 * kKnot, 90.0);
    const double heel = 20.0 * kPi / 180.0;
    State s = moving(0.0, 5.0 * kKnot, heel);
    const ApparentWind a = apparent_wind(s, Vec3::Zero(), wind, 0.0);
    CHECK(heel_angle(s.rotation()) == doctest::Approx(heel).epsilon(1e-12));
    CHECK(a.awa == doctest::Approx(std::atan2(10.0, 5.0)).epsilon(1e-9));
    CHECK(std::abs(a.awa_sail - std::atan(std::tan(a.awa) * std::cos(heel))) < 1e-6);
    CHECK(a.awa_sail < a.awa);
    CHECK(a.aws_sail < a.aws);
}

TEST_CASE("heel with horizontal wind follows tan(AWA_eff) = tan(AWA) cos(heel)") {
    const UniformWind wind(8.0, 60.0);
    const double heel = 20.0 * kPi / 180.0;
    State s = moving(0.0, 0.0, heel);
    const ApparentWind a = apparent_wind(s, Vec3::Zero(), wind, 0.0);
    CHECK(std::abs(std::tan(a.awa_sail) - std::tan(a.awa) * std::cos(heel)) < 1e-6);
}

TEST_CASE("angular velocity adds omega x r at a masthead point") {
    const UniformWind calm(0.0, 0.0);
    State roll;
    roll.angular_velocity = Vec3(0.2, 0.0, 0.0);
    const ApparentWind a = apparent_wind(roll, Vec3(0.0, 0.0, 10.0), calm, 0.0);
    CHECK(a.body.x() == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(a.body.y() == doctest::Approx(2.0));
    CHECK(a.body.z() == doctest::Approx(0.0).epsilon(1e-12));

    State yaw;
    yaw.angular_velocity = Vec3(0.0, 0.0, 1.0);
    const ApparentWind b = apparent_wind(yaw, Vec3(3.0, 0.0, 10.0), calm, 0.0);
    CHECK(b.body.y() == doctest::Approx(-3.0));
    CHECK(b.world.y() == doctest::Approx(-3.0));
}

TEST_CASE("the point velocity rotates the CG offset into the world") {
    State s = moving(0.0, 0.0);
    s.angular_velocity = Vec3(0.0, 0.0, 1.0);
    const ApparentWind a = apparent_wind(s, Vec3(0.0, 2.0, 0.0), UniformWind(0.0, 0.0), 0.0);
    CHECK(a.body.x() == doctest::Approx(2.0));
}

TEST_CASE("zero wind and zero motion give zero apparent wind and AWA 0") {
    const ApparentWind a = apparent_wind(State{}, Vec3(1.0, 2.0, 3.0), UniformWind(0.0, 123.0), 0.0);
    CHECK(a.aws == 0.0);
    CHECK(a.awa == 0.0);
    CHECK(a.awa_sail == 0.0);
    CHECK(a.aws_sail == 0.0);
    CHECK(a.world.norm() == 0.0);
}

TEST_CASE("wind from astern gives AWA of pi") {
    const ApparentWind a = apparent_wind(moving(0.0, 0.0), Vec3::Zero(), UniformWind(6.0, 180.0), 0.0);
    CHECK(a.awa == doctest::Approx(kPi));
    CHECK(a.awa_sail == doctest::Approx(kPi));
    CHECK(a.aws == doctest::Approx(6.0));
}

TEST_CASE("AWA is measured from the boat heading") {
    const ApparentWind a = apparent_wind(moving(kPi / 2.0, 0.0), Vec3::Zero(), UniformWind(5.0, 90.0), 0.0);
    CHECK(a.awa == doctest::Approx(0.0).epsilon(1e-9));
}
