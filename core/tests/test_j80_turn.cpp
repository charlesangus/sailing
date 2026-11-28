#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>

#include <nlohmann/json.hpp>

#include "sail/boat_sim.hpp"
#include "sail/foil.hpp"

namespace {

using sail::TurnRequest;
using sail::TurnResult;

const std::filesystem::path kDataDir(SAIL_TEST_DATA_DIR);
const std::filesystem::path kBoatDir = kDataDir / "boats" / "j80";

constexpr double kKnot = 0.514444;

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

TEST_CASE("j80_turn stays finite at zero and sternway speed") {
    for (const double kn : {-1.0, 0.0}) {
        CAPTURE(kn);
        const TurnResult r = turn(kn, 1.0, 20.0);
        REQUIRE(r.tow_converged);
        CHECK(r.finite);
        CHECK_FALSE(r.first_nonfinite_time);
    }
}
