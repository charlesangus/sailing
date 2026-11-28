#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numbers>
#include <optional>

#include <nlohmann/json.hpp>

#include "sail/damping.hpp"
#include "sail/equilibrium.hpp"
#include "sail/hydrostatics.hpp"
#include "sail/mass_model.hpp"
#include "sail/gravity.hpp"
#include "sail/simulation.hpp"

namespace {

using sail::MassConfiguration;
using sail::MassModel;
using sail::State;
using sail::Vec3;

const std::filesystem::path kBoatDir = std::filesystem::path(SAIL_TEST_DATA_DIR) / "boats" / "j80";

constexpr double kG = sail::kGravity;
constexpr double kDeg = std::numbers::pi / 180.0;

double particular(const char* key) {
    std::ifstream in(kBoatDir / "particulars.json");
    REQUIRE(in);
    const nlohmann::json json = nlohmann::json::parse(in);
    return json.at(key).at("value").get<double>();
}

struct Boat {
    MassModel model;
    sail::BoatHydrostatics hydro;
    sail::HydrostaticParams params;

    explicit Boat(MassConfiguration configuration)
        : model(MassModel::load(kBoatDir, configuration)),
          hydro(sail::load_boat_hydrostatics(kBoatDir)),
          params(hydro.params(model, kG)) {}
};

// Heel as the angle of the body's up axis from vertical about the world x axis, degrees.
double heel_deg(const State& state) {
    const Vec3 up = state.orientation * Vec3(0.0, 0.0, 1.0);
    return std::atan2(-up.y(), up.z()) / kDeg;
}

}  // namespace

TEST_CASE("j80_hydro: floats at the DWL with level trim and certificate draft in measurement trim") {
    const Boat boat(MassConfiguration::measurement_trim);
    const double mass = boat.model.total_mass();
    const sail::Equilibrium eq = sail::solve_equilibrium(boat.hydro.mesh, boat.params, mass);
    REQUIRE(eq.converged);

    // The hull-frame origin is on the measurement-trim flotation plane, so at rest it sits on
    // the water.
    const Vec3 origin = eq.pose.position - eq.pose.orientation * boat.params.reference_point;
    CHECK(std::abs(origin.z()) < 0.005);
    CHECK(std::abs(eq.trim) < 0.5 * kDeg);

    double deepest = 0.0;
    for (const Vec3& v : boat.hydro.mesh.vertices) {
        const Vec3 world = eq.pose.orientation * (v - boat.params.reference_point) + eq.pose.position;
        deepest = std::min(deepest, world.z());
    }
    const double draft = -deepest;
    const double target_draft = particular("draft");
    MESSAGE("rest: hull origin at z " << origin.z() << " m, trim " << eq.trim / kDeg
                                      << " deg, draft " << draft << " m (certificate "
                                      << target_draft << ")");
    CHECK(draft == doctest::Approx(target_draft).epsilon(0.02));
}

TEST_CASE("j80_hydro: righting moment at 1 degree, measurement trim, level") {
    const Boat boat(MassConfiguration::measurement_trim);
    const sail::GzPoint p = sail::solve_gz_point(boat.hydro.mesh, boat.params,
                                                 boat.model.total_mass(), 1.0 * kDeg, false);
    REQUIRE(p.eq.converged);
    const double target = particular("righting_moment_1deg");
    const double excess = p.eq.righting_moment / target - 1.0;
    MESSAGE("RM at 1 deg: " << p.eq.righting_moment << " N*m vs certificate " << target << " ("
                            << 100.0 * excess << "% above)");
    CHECK(excess > 0.0);
    CHECK(excess < 0.20);
}

TEST_CASE("j80_hydro: angle of vanishing stability in ORC stability trim, level") {
    const double lps = particular("limit_of_positive_stability");

    const Boat orc(MassConfiguration::orc_stability);
    const double mass = orc.model.total_mass();
    const auto curve = sail::gz_curve(orc.hydro.mesh, orc.params, mass, 5.0 * kDeg, false);
    const std::optional<double> avs =
        sail::angle_of_vanishing_stability(orc.hydro.mesh, orc.params, mass, curve, false);
    REQUIRE(avs.has_value());
    MESSAGE("AVS orc level: " << *avs / kDeg << " deg (certificate " << lps << ")");
    CHECK(std::abs(*avs / kDeg - lps) < 5.0);
    CHECK(sail::gz_sign_changes(curve) == 1);

    const sail::GzPoint& inverted = curve.back();
    REQUIRE(inverted.heel == doctest::Approx(std::numbers::pi));
    REQUIRE(inverted.eq.converged);
    CHECK(inverted.eq.hydrostatics.volume * orc.params.density == doctest::Approx(mass).epsilon(1e-6));
    CHECK(std::abs(inverted.gz) < 1e-3);
}

TEST_CASE("j80_hydro: knockdown from 100 degrees recovers within 20 s") {
    const Boat boat(MassConfiguration::measurement_trim);
    const sail::BodyProperties body = boat.model.body_properties();

    sail::EquilibriumRequest request;
    request.heel = 100.0 * kDeg;
    request.free_trim = false;
    const sail::Equilibrium start =
        sail::solve_equilibrium(boat.hydro.mesh, boat.params, body.mass, request);
    REQUIRE(start.converged);

    // Heavy roll damping stands in for the keel and rig drag of a real knockdown recovery, which
    // the flat-water hydrostatics force alone does not provide.
    constexpr double kRollMultiplier = 8.0;
    auto damping = std::make_unique<sail::DampingModule>(sail::load_damping(boat.model));
    damping->set_multiplier(sail::DampingDof::roll, kRollMultiplier);

    sail::Simulation sim(body, start.pose);
    sim.add_module(std::make_unique<sail::GravityModule>(body.mass));
    sim.add_module(std::make_unique<sail::HydrostaticsModule>(boat.hydro.mesh, boat.params));
    sim.add_module(std::move(damping));

    constexpr int kSteps = 20 * 200;
    std::optional<double> first_within;
    for (int i = 0; i < kSteps; ++i) {
        sim.step();
        if (!first_within && std::abs(heel_deg(sim.state())) < 10.0) first_within = sim.time();
    }
    const double final_heel = heel_deg(sim.state());
    MESSAGE("knockdown (roll damping x" << kRollMultiplier << "): first within 10 deg at "
                                        << (first_within ? *first_within : -1.0)
                                        << " s, heel at 20 s " << final_heel << " deg");
    CHECK(first_within.has_value());
    CHECK(std::abs(final_heel) < 10.0);
}

TEST_CASE("j80_hydro: moving the crew to port mid-run heels a running simulation to port") {
    Boat boat(MassConfiguration::sailing);
    const sail::BodyProperties body = boat.model.body_properties();
    const sail::Equilibrium start =
        sail::solve_equilibrium(boat.hydro.mesh, boat.params, body.mass);
    REQUIRE(start.converged);

    sail::GravityModule gravity(body.mass);
    sail::HydrostaticsModule hydrostatics(boat.hydro.mesh, boat.params);
    sail::DampingModule damping(sail::load_damping(boat.model));
    damping.set_multipliers(4.0);
    sail::Simulation sim(body, start.pose);
    sim.add_module(gravity);
    sim.add_module(hydrostatics);
    sim.add_module(damping);

    for (int i = 0; i < 2 * 200; ++i) sim.step();
    CHECK(std::abs(heel_deg(sim.state())) < 0.1);

    // The crew sits out on the port deck edge.
    const Vec3 old_cg = boat.model.cg();
    const Vec3 crew = boat.model.crew_position();
    boat.model.set_crew_position(1.0, crew.z());
    const Vec3 shift = boat.model.cg() - old_cg;
    REQUIRE(shift.y() > 0.0);
    const Vec3 hull_origin_before =
        sim.state().position - sim.state().orientation * old_cg;
    sim.set_body(boat.model.body_properties(), shift);
    hydrostatics.set_reference_point(boat.model.cg());
    const Vec3 hull_origin_after =
        sim.state().position - sim.state().orientation * boat.model.cg();
    CHECK((hull_origin_after - hull_origin_before).norm() < 1e-12);

    for (int i = 0; i < 30 * 200; ++i) sim.step();
    const double heel = heel_deg(sim.state());

    // Static heel with the new CG: where the righting moment about it vanishes, by bisection.
    sail::HydrostaticParams params = boat.params;
    params.reference_point = boat.model.cg();
    const auto rm = [&](double h) {
        return sail::solve_gz_point(boat.hydro.mesh, params, body.mass, h * kDeg, true)
            .eq.righting_moment;
    };
    double lo = -30.0;
    double hi = 0.0;
    REQUIRE(rm(lo) < 0.0);
    REQUIRE(rm(hi) > 0.0);
    for (int k = 0; k < 30; ++k) {
        const double mid = 0.5 * (lo + hi);
        (rm(mid) < 0.0 ? lo : hi) = mid;
    }
    const double expected = 0.5 * (lo + hi);
    MESSAGE("crew to port: simulated heel " << heel << " deg, static heel " << expected << " deg");
    CHECK(heel < -1.0);
    CHECK(heel == doctest::Approx(expected).epsilon(0.02));
}
