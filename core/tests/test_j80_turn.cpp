#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numbers>

#include <nlohmann/json.hpp>

#include "sail/boat_sim.hpp"
#include "sail/foil.hpp"

namespace {

using sail::TurnRequest;
using sail::TurnResult;

const std::filesystem::path kDataDir(SAIL_TEST_DATA_DIR);
const std::filesystem::path kBoatDir = kDataDir / "boats" / "j80";

constexpr double kKnot = 0.514444;
constexpr double kDeg = std::numbers::pi / 180.0;

std::shared_ptr<const sail::FoilSection> naca0012() {
    static const auto section = std::make_shared<const sail::FoilSection>(
        sail::FoilSection::load(kDataDir / "foils" / "naca0012.csv"));
    return section;
}

double loa() {
    std::ifstream in(kBoatDir / "particulars.json");
    REQUIRE(in);
    return nlohmann::json::parse(in).at("loa").at("value").get<double>();
}

TurnResult turn(double speed_kn, double tiller, double duration) {
    const auto boat = sail::BoatSim::load(kBoatDir, naca0012());
    TurnRequest request;
    request.speed = speed_kn * kKnot;
    request.tiller = tiller;
    request.duration = duration;
    return sail::run_turn(*boat, request);
}

}  // namespace

TEST_CASE("j80_turn full tiller at 5 kn turns in 1.5 boat lengths") {
    const TurnResult r = turn(5.0, 1.0, 40.0);
    REQUIRE(r.tow_converged);
    REQUIRE(r.finite);
    REQUIRE(r.steady);
    REQUIRE(r.steady->circle);
    const double diameter = 2.0 * r.steady->circle->radius;
    MESSAGE("steady diameter " << diameter << " m = " << diameter / loa() << " LOA, window "
                               << r.steady->t_begin << "-" << r.steady->t_end << " s");
    CHECK(diameter == doctest::Approx(1.5 * loa()).epsilon(0.10));
}

TEST_CASE("j80_turn positive tiller turns to starboard") {
    const TurnResult r = turn(5.0, 1.0, 20.0);
    REQUIRE(r.finite);
    REQUIRE(r.steady);
    CHECK(r.steady->yaw_rate < 0.0);
    CHECK(r.heading_change < 0.0);
}

TEST_CASE("j80_turn stays finite at zero speed") {
    const TurnResult r = turn(0.0, 1.0, 20.0);
    REQUIRE(r.tow_converged);
    CHECK(r.finite);
    CHECK_FALSE(r.first_nonfinite_time);
}

TEST_CASE("j80_turn in sternway the rudder lift reverses and its drag opposes the motion") {
    const TurnResult ahead = turn(1.0, 1.0, 20.0);
    const TurnResult astern = turn(-1.0, 1.0, 20.0);
    REQUIRE(ahead.finite);
    REQUIRE(astern.tow_converged);
    REQUIRE(astern.finite);
    CHECK_FALSE(astern.first_nonfinite_time);
    MESSAGE("heading change in 20 s at 1 kn: ahead " << ahead.heading_change / kDeg
                                                     << " deg, astern "
                                                     << astern.heading_change / kDeg << " deg");
    CHECK(ahead.heading_change < 0.0);
    CHECK(astern.heading_change > 0.0);

    for (const double kn : {1.0, -1.0}) {
        CAPTURE(kn);
        const auto boat = sail::BoatSim::load(kBoatDir, naca0012());
        TurnRequest request;
        request.speed = kn * kKnot;
        request.tiller = 1.0;
        request.duration = 1.0;
        REQUIRE(sail::run_turn(*boat, request).finite);
        const sail::RudderForceModule& rudder = boat->appendages().rudder();
        const double sign = kn > 0.0 ? 1.0 : -1.0;
        CHECK(rudder.angle() > 0.0);
        CHECK(sign * rudder.last_result().lift.y() > 0.0);
        CHECK(sign * rudder.last_result().drag.x() < 0.0);
    }
}

TEST_CASE("j80_turn repeats on a reused BoatSim and its Simulation can be stepped on") {
    const auto boat = sail::BoatSim::load(kBoatDir, naca0012());
    TurnRequest request;
    request.speed = 5.0 * kKnot;
    request.tiller = 1.0;
    request.duration = 5.0;
    const TurnResult first = sail::run_turn(*boat, request);
    const TurnResult second = sail::run_turn(*boat, request);
    REQUIRE(first.finite);
    REQUIRE(second.finite);
    CHECK(second.thrust == first.thrust);
    CHECK(second.heading_change == first.heading_change);
    CHECK(second.samples.back().x == first.samples.back().x);
    CHECK(second.max_rudder_hinge_moment == first.max_rudder_hinge_moment);
    CHECK(second.max_rudder_hinge_moment_time < second.samples.back().t);

    sail::Simulation& sim = boat->simulation();
    const double speed = second.samples.back().speed;
    for (int i = 0; i < 200; ++i) sim.step();
    const sail::Vec3 v = sim.state().world_linear_velocity();
    CHECK(std::isfinite(v.norm()));
    CHECK(std::abs(std::hypot(v.x(), v.y()) - speed) < 0.5);
}
