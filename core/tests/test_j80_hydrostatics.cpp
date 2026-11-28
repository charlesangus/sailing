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
#include "sail/simulation.hpp"

namespace {

using sail::MassConfiguration;
using sail::MassModel;
using sail::State;
using sail::Vec3;
using sail::Wrench;

const std::filesystem::path kBoatDir = std::filesystem::path(SAIL_TEST_DATA_DIR) / "boats" / "j80";

constexpr double kG = 9.81;
constexpr double kDeg = std::numbers::pi / 180.0;

double particular(const char* key) {
    std::ifstream in(kBoatDir / "particulars.json");
    REQUIRE(in);
    const nlohmann::json json = nlohmann::json::parse(in);
    return json.at(key).at("value").get<double>();
}

class Gravity final : public sail::ForceModule {
public:
    explicit Gravity(double mass) : mass_(mass) {}
    void accumulate(const State& state, double, Wrench& wrench) override {
        wrench.force += state.orientation.conjugate() * Vec3(0.0, 0.0, -mass_ * kG);
    }

private:
    double mass_;
};

struct Boat {
    MassModel model;
    sail::BoatHydrostatics hydro;
    sail::HydrostaticParams params;

    explicit Boat(MassConfiguration configuration)
        : model(MassModel::load(kBoatDir, configuration)),
          hydro(sail::load_boat_hydrostatics(kBoatDir)),
          params(hydro.params(model, kG)) {}
};

std::optional<double> avs_deg(const Boat& boat, bool free_trim) {
    const double mass = boat.model.total_mass();
    const auto curve = sail::gz_curve(boat.hydro.mesh, boat.params, mass, 5.0 * kDeg, free_trim);
    const auto avs = sail::angle_of_vanishing_stability(boat.hydro.mesh, boat.params, mass, curve,
                                                        free_trim);
    if (!avs) return std::nullopt;
    return *avs / kDeg;
}

}  // namespace

TEST_CASE("j80_hydro: rest displacement, trim and draft in measurement trim") {
    const Boat boat(MassConfiguration::measurement_trim);
    const double mass = boat.model.total_mass();
    const sail::Equilibrium eq = sail::solve_equilibrium(boat.hydro.mesh, boat.params, mass);
    REQUIRE(eq.converged);

    const double displaced = eq.hydrostatics.volume * boat.params.density;
    const double target_mass = particular("displacement");
    CHECK(displaced == doctest::Approx(target_mass).epsilon(0.01));

    CHECK(std::abs(eq.trim) < 0.5 * kDeg);

    double deepest = 0.0;
    for (const Vec3& v : boat.hydro.mesh.vertices) {
        const Vec3 world = eq.pose.orientation * (v - boat.params.reference_point) + eq.pose.position;
        deepest = std::min(deepest, world.z());
    }
    const double draft = -deepest;
    const double target_draft = particular("draft");
    MESSAGE("rest: displaced " << displaced << " kg (certificate " << target_mass << "), trim "
                               << eq.trim / kDeg << " deg, draft " << draft << " m (certificate "
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
    const std::optional<double> avs = avs_deg(orc, false);
    REQUIRE(avs.has_value());
    MESSAGE("AVS orc level: " << *avs << " deg (certificate " << lps << ")");
    CHECK(std::abs(*avs - lps) < 5.0);

    const Boat measurement(MassConfiguration::measurement_trim);
    const std::optional<double> level = avs_deg(measurement, false);
    const std::optional<double> free = avs_deg(measurement, true);
    MESSAGE("AVS measurement level: " << (level ? *level : -1.0)
                                      << " deg, measurement free trim: " << (free ? *free : -1.0)
                                      << " deg (-1 means none)");
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
    sim.add_module(std::make_unique<Gravity>(body.mass));
    sim.add_module(std::make_unique<sail::HydrostaticsModule>(boat.hydro.mesh, boat.params));
    sim.add_module(std::move(damping));

    // Heel as the angle of the body's up axis from vertical about the world x axis.
    const auto heel_deg = [&] {
        const Vec3 up = sim.state().orientation * Vec3(0.0, 0.0, 1.0);
        return std::atan2(-up.y(), up.z()) / kDeg;
    };

    constexpr int kSteps = 20 * 200;
    std::optional<double> first_within;
    for (int i = 0; i < kSteps; ++i) {
        sim.step();
        if (!first_within && std::abs(heel_deg()) < 10.0) first_within = sim.time();
    }
    const double final_heel = heel_deg();
    MESSAGE("knockdown (roll damping x" << kRollMultiplier << "): first within 10 deg at "
                                        << (first_within ? *first_within : -1.0)
                                        << " s, heel at 20 s " << final_heel << " deg");
    CHECK(first_within.has_value());
    CHECK(std::abs(final_heel) < 10.0);
}
