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

using sail::BoatSim;
using sail::TowRequest;
using sail::TowResult;

const std::filesystem::path kDataDir(SAIL_TEST_DATA_DIR);
const std::filesystem::path kBoatDir = kDataDir / "boats" / "j80";

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kKnot = 0.514444;

std::shared_ptr<const sail::FoilSection> naca0012() {
    static const auto section = std::make_shared<const sail::FoilSection>(
        sail::FoilSection::load(kDataDir / "foils" / "naca0012.csv"));
    return section;
}

BoatSim& j80() {
    static const std::unique_ptr<BoatSim> boat = BoatSim::load(kBoatDir, naca0012());
    return *boat;
}

nlohmann::json tow_reference() {
    std::ifstream in(kBoatDir / "hull_resistance_reference.json");
    REQUIRE(in);
    return nlohmann::json::parse(in).at("tow");
}

TowResult tow(double speed_kn, double heel_deg = 0.0, double leeway_deg = 0.0) {
    TowRequest request;
    request.speed = speed_kn * kKnot;
    request.heel = heel_deg * kDeg;
    request.leeway = leeway_deg * kDeg;
    return sail::solve_tow(j80(), request);
}

}  // namespace

TEST_CASE("j80_tow upright tow balance converges and holds the boat steady") {
    for (const double kn : {1.0, 3.0, 5.0, 7.0}) {
        CAPTURE(kn);
        const TowResult r = tow(kn);
        REQUIRE(r.converged);
        CHECK(r.tow_force_world.x() == doctest::Approx(r.total.resistance).epsilon(1e-9));
        CHECK(r.total.resistance > 0.0);
        CHECK(std::abs(r.total.side_force) < 1e-6 * r.total.resistance + 1e-3);
        CHECK(std::abs(r.trim - r.still.trim) < 1.0 * kDeg);
        CHECK(std::abs(r.state.position.z() - r.still.pose.position.z()) < 0.01);
        const double parts =
            r.canoe.resistance + r.keel.resistance + r.rudder.resistance + r.damping.resistance;
        CHECK(parts == doctest::Approx(r.total.resistance).epsilon(1e-3));
    }
}

TEST_CASE("j80_tow side force opposes leeway and grows with it") {
    const TowResult port = tow(5.0, 10.0, 3.0);
    const TowResult more = tow(5.0, 10.0, 5.0);
    const TowResult starboard = tow(5.0, 10.0, -3.0);
    REQUIRE(port.converged);
    REQUIRE(more.converged);
    REQUIRE(starboard.converged);
    CHECK(port.total.side_force < 0.0);
    CHECK(starboard.total.side_force > 0.0);
    CHECK(more.total.side_force < port.total.side_force);
    CHECK(port.keel.side_force < 0.0);
    CHECK(port.total.resistance > tow(5.0, 10.0).total.resistance);
}

TEST_CASE("j80_tow canoe-body upright resistance matches Python-VPP at 3, 5 and 6 kn") {
    const nlohmann::json ref = tow_reference();
    REQUIRE(ref.at("points").size() == 3);
    for (const auto& p : ref.at("points")) {
        const double kn = p.at("speed_kn").get<double>();
        CAPTURE(kn);
        CHECK(p.at("speed_mps").get<double>() == doctest::Approx(kn * kKnot).epsilon(1e-12));
        const TowResult r = tow(kn);
        REQUIRE(r.converged);
        const double ours = r.canoe.resistance;
        const double theirs = p.at("canoe").at("total_N").get<double>();
        MESSAGE(kn << " kn canoe: ours " << ours << " N, Python-VPP " << theirs << " N, gap "
                   << 100.0 * (ours / theirs - 1.0) << " %");
        CHECK(ours == doctest::Approx(theirs).epsilon(0.05));
    }
}

// Python-VPP's appendages carry the ORC strip friction on twice the planform area with a form
// factor, and the keel adds a residuary term from its rrk curve; ours are the section's profile
// drag on the planform area alone, with no wave-making term. That leaves ours 23 to 30 percent
// below its appendage viscous drag, and the keel residuary (-8 N at 5 kn, +25 N at 6 kn) has no
// counterpart, so the totals agree to 10 percent rather than 5.
TEST_CASE("j80_tow total upright resistance against Python-VPP with keel and rudder") {
    const nlohmann::json ref = tow_reference();
    for (const auto& p : ref.at("points")) {
        const double kn = p.at("speed_kn").get<double>();
        CAPTURE(kn);
        const TowResult r = tow(kn);
        REQUIRE(r.converged);
        const double ours = r.canoe.resistance + r.keel.resistance + r.rudder.resistance;
        const double theirs = p.at("total_N").get<double>();
        MESSAGE(kn << " kn canoe + foils: ours " << ours << " N (keel " << r.keel.resistance
                   << ", rudder " << r.rudder.resistance << "), Python-VPP " << theirs
                   << " N (keel " << p.at("keel").at("total_N").get<double>() << ", rudder "
                   << p.at("rudder").at("total_N").get<double>() << "), gap "
                   << 100.0 * (ours / theirs - 1.0) << " %; tow force with heave damping "
                   << r.total.resistance << " N");
        CHECK(ours == doctest::Approx(theirs).epsilon(0.10));
    }
}
